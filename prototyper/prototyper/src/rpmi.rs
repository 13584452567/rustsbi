//! Prototyper-side RPMI transport over the shared-memory mailbox.
//!
//! The `rpmi` crate carries only the RPMI *definitions* (message protocol and
//! per-service-group constants). This module supplies the concrete runtime
//! that moves messages between the application processor (AP) and the
//! platform management processor (PuC), mirroring OpenSBI
//! `lib/utils/mailbox/{rpmi_mailbox.c, fdt_mailbox_rpmi_shmem.c}`.
//!
//! - [`SmqQueue`]: a shared-memory ring queue with the four A2P/P2A request
//!   and acknowledgement rings, keeping the data cache coherent with
//!   `cbo.flush` / `cbo.inval`;
//! - [`RpmiMailbox`]: a mailbox controller offering the normal (request +
//!   response) and posted (fire-and-forget) request patterns;
//! - [`CppcProbeReq`], [`CppcReadReq`] and [`CppcWriteReq`]: the CPPC service
//!   request payloads consumed by the SBI CPPC extension.
//!
//! The boardtest-facing service-group constants (which the `rpmi` crate keeps
//! private) and the SpacemiT-specific `RTC` / `PWRKEY` groups are re-exported
//! here so the MPXY channel set stays complete on the SpacemiT K3.

use core::sync::atomic::{AtomicU16, Ordering};

use log::warn;
use rpmi::message::{MessageHeader, MessageType, Status};

/// Size of the RPMI message header in bytes (`rpmi_msgprot.h`
/// `RPMI_MSG_HDR_SIZE`).
pub const RPMI_MSG_HDR_SIZE: usize = 8;
/// Token mask (16-bit).
pub const RPMI_MSG_TOKEN_MASK: u16 = 0xffff;
/// Default transfer timeout in milliseconds (OpenSBI `RPMI_DEF_TX_TIMEOUT` /
/// `RPMI_DEF_RX_TIMEOUT`).
pub const RPMI_DEF_TX_TIMEOUT: u32 = 500;
pub const RPMI_DEF_RX_TIMEOUT: u32 = 500;

/// mtime tick rate on the SpacemiT K3 (DTB `timebase-frequency` is 24 MHz).
const MTIME_FREQ_HZ: u64 = 24_000_000;

/// Read the hart's mtime tick count via the TIME CSR.
///
/// The TIME CSR is readable in M-mode (the SBI ecall path) and, unlike
/// `mcycle` (`mcountinhibit.CY`), cannot be inhibited.
#[inline]
fn mtime_ticks() -> u64 {
    riscv::register::time::read64()
}

/// RPMI service-group identifiers used by the SpacemiT K3 MPXY channels
/// (`rpmi_msgprot.h` `enum rpmi_servicegroup_id`).
pub mod servicegroup {
    /// Base service group.
    pub const BASE: u16 = 0x0001;
    /// System MSI service group.
    pub const SYSTEM_MSI: u16 = 0x0002;
    /// System reset service group.
    pub const SYSTEM_RESET: u16 = 0x0003;
    /// System suspend service group.
    pub const SYSTEM_SUSPEND: u16 = 0x0004;
    /// Hart State Management service group.
    pub const HSM: u16 = 0x0005;
    /// CPPC service group.
    pub const CPPC: u16 = 0x0006;
    /// Voltage service group.
    pub const VOLTAGE: u16 = 0x0007;
    /// Clock service group.
    pub const CLOCK: u16 = 0x0008;
    /// Device power domain service group.
    pub const DOMAIN: u16 = 0x0009;
    /// RTC service group (SpacemiT vendor group).
    pub const RTC: u16 = 0x000e;
    /// Power key service group (SpacemiT vendor group).
    pub const PWRKEY: u16 = 0x000f;
}

/// CPPC service request: probe a register (`rpmi_cppc_probe_req`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CppcProbeReq {
    /// Hart identifier.
    pub hart_id: u32,
    /// CPPC register identifier.
    pub reg_id: u32,
}

/// CPPC service request: read a register (`rpmi_cppc_read_reg_req`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CppcReadReq {
    /// Hart identifier.
    pub hart_id: u32,
    /// CPPC register identifier.
    pub reg_id: u32,
}

/// CPPC service request: write a register (`rpmi_cppc_write_reg_req`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CppcWriteReq {
    /// Hart identifier.
    pub hart_id: u32,
    /// CPPC register identifier.
    pub reg_id: u32,
    /// Lower 32 bits of the register value.
    pub data_lo: u32,
    /// Upper 32 bits of the register value.
    pub data_hi: u32,
}

/// Little-endian volatile 32-bit accessor.
#[repr(transparent)]
pub struct Le32(u32);

impl Le32 {
    /// Read the value in little-endian order.
    #[inline]
    pub fn read(&self) -> u32 {
        // Safety: `self` aliases a shared-memory word; the volatile read
        // cannot be cached or reordered by the compiler.
        unsafe { u32::from_le(core::ptr::addr_of!(self.0).read_volatile()) }
    }

    /// Write the value in little-endian order.
    #[inline]
    pub fn write(&self, value: u32) {
        // Safety: `self` aliases a shared-memory word; the volatile write
        // cannot be elided or reordered by the compiler.
        unsafe {
            core::ptr::addr_of!(self.0)
                .cast_mut()
                .write_volatile(value.to_le())
        }
    }
}

/// A shared-memory ring queue for RPMI messages.
pub struct SmqQueue {
    /// Head (read) index in shared memory.
    head: *const Le32,
    /// Tail (write) index in shared memory.
    tail: *const Le32,
    /// Slot buffer base.
    buffer: *mut u8,
    /// Size of one slot in bytes.
    slot_size: usize,
    /// Number of slots in the ring.
    num_slots: usize,
}

// Safety: the queue aliases shared memory accessed by both the AP and the
// PuC; all accesses are volatile and guarded by the queue indices published
// with release/acquire fences, so sharing the queue between harts (and the
// dispatcher static) is sound.
unsafe impl Send for SmqQueue {}
unsafe impl Sync for SmqQueue {}

impl SmqQueue {
    /// Create a new queue view.
    ///
    /// # Safety
    ///
    /// `head`, `tail` and `buffer` must point into shared memory that is
    /// accessible to both the AP and the PuC, and `slot_size * num_slots`
    /// bytes must be readable/writable at `buffer`.
    pub const unsafe fn new(
        head: *const Le32,
        tail: *const Le32,
        buffer: *mut u8,
        slot_size: usize,
        num_slots: usize,
    ) -> Self {
        Self {
            head,
            tail,
            buffer,
            slot_size,
            num_slots,
        }
    }

    /// Returns whether the queue is full (`(tail + 1) % n == head`).
    #[inline]
    fn is_full(&self) -> bool {
        let head = unsafe { &*self.head }.read() as usize;
        let tail = unsafe { &*self.tail }.read() as usize;
        (tail + 1) % self.num_slots == head
    }

    /// Returns whether the queue is empty (`head == tail`).
    #[inline]
    fn is_empty(&self) -> bool {
        let head = unsafe { &*self.head }.read() as usize;
        let tail = unsafe { &*self.tail }.read() as usize;
        head == tail
    }

    /// SpacemiT K3 L1 data-cache line size.
    const CACHE_LINE_SIZE: usize = 64;

    /// Clean and invalidate a data-cache range (`cbo.flush`), making writes
    /// visible to the remote management processor (PuC). Mirrors OpenSBI
    /// `csi_dcache_clean_invalid_range` / `__DCACHE_CIPA`.
    unsafe fn dcache_clean_invalid_range(addr: usize, size: usize) {
        core::arch::asm!("fence rw, rw");
        let start = addr & !(Self::CACHE_LINE_SIZE - 1);
        let end = addr + size;
        let mut op = start;
        while op < end {
            core::arch::asm!("cbo.flush 0({})", in(reg) op);
            op += Self::CACHE_LINE_SIZE;
        }
        core::arch::asm!("fence rw, rw");
    }

    /// Invalidate a data-cache range (`cbo.inval`) so the local hart reads
    /// fresh data written by the remote PuC. Mirrors OpenSBI
    /// `csi_dcache_invalid_range` / `__DCACHE_IPA`.
    unsafe fn dcache_invalid_range(addr: usize, size: usize) {
        core::arch::asm!("fence rw, rw");
        let start = addr & !(Self::CACHE_LINE_SIZE - 1);
        let end = addr + size;
        let mut op = start;
        while op < end {
            core::arch::asm!("cbo.inval 0({})", in(reg) op);
            op += Self::CACHE_LINE_SIZE;
        }
        core::arch::asm!("fence rw, rw");
    }

    /// Enqueue a message into the ring.
    ///
    /// Writes the header and payload into the tail slot, publishes the
    /// little-endian tail index, and rings the optional doorbell register.
    /// Returns `Err(())` when the queue is full.
    ///
    /// # Safety
    ///
    /// `data.len()` must not exceed `slot_size - 8`.
    pub unsafe fn send(
        &self,
        header: &MessageHeader,
        data: &[u8],
        doorbell: Option<&Le32>,
    ) -> Result<(), ()> {
        // Invalidate the PuC-written head (and our own tail) so the freed
        // slots are visible before checking whether the queue is full.
        // Mirrors OpenSBI `__smq_tx`'s `__DCACHE_IPA(headptr/tailptr)`.
        Self::dcache_invalid_range(self.head as usize, core::mem::size_of::<Le32>());
        Self::dcache_invalid_range(self.tail as usize, core::mem::size_of::<Le32>());
        if self.is_full() {
            warn!(
                "SMQ send FULL: token={} sg={} svc={} head={} tail={}",
                header.token(),
                header.service_group_id(),
                header.service_id(),
                unsafe { &*self.head }.read(),
                unsafe { &*self.tail }.read()
            );
            return Err(());
        }
        if data.len() > self.slot_size - RPMI_MSG_HDR_SIZE {
            warn!(
                "SMQ send TOO-BIG: token={} len={} slot={}",
                header.token(),
                data.len(),
                self.slot_size
            );
            return Err(());
        }

        let tail = unsafe { &*self.tail }.read() as usize;
        let slot = unsafe { self.buffer.add(tail * self.slot_size) };

        // Write the header little-endian. The RPMI logical words pack into
        // the same byte layout the shared-memory transport specifies.
        let words = header.words();
        unsafe {
            (slot as *mut u32).write_volatile(words[0].to_le());
            (slot.add(4) as *mut u32).write_volatile(words[1].to_le());
        }
        // Copy payload.
        if !data.is_empty() {
            unsafe {
                core::ptr::copy_nonoverlapping(
                    data.as_ptr(),
                    slot.add(RPMI_MSG_HDR_SIZE),
                    data.len(),
                );
            }
        }

        // Make the message data visible to the PuC before publishing the tail.
        Self::dcache_clean_invalid_range(slot as usize, self.slot_size);

        // Publish the tail index.
        unsafe { &*self.tail }.write(((tail + 1) % self.num_slots) as u32);
        // Flush the tail index so the PuC sees the new value.
        Self::dcache_clean_invalid_range(self.tail as usize, core::mem::size_of::<Le32>());

        // Ring the doorbell if present.
        if let Some(db) = doorbell {
            db.write(1);
        }
        Ok(())
    }

    /// Dequeue a message from the ring, matching `token`.
    ///
    /// Scans the queue for the slot carrying `token`, moves it to the head
    /// slot, copies the payload into `out`, and advances the head index.
    /// Returns the number of payload bytes copied, or `Err(())` when no
    /// matching message is present.
    pub unsafe fn receive(&self, token: u16, out: &mut [u8]) -> Result<usize, ()> {
        // Invalidate the PuC-written tail index and slots so we read fresh data.
        Self::dcache_invalid_range(self.tail as usize, core::mem::size_of::<Le32>());
        if self.is_empty() {
            return Err(());
        }
        let head = unsafe { &*self.head }.read() as usize;
        let tail = unsafe { &*self.tail }.read() as usize;

        // Locate the slot with the matching token.
        let mut pos = head;
        loop {
            let slot = unsafe { self.buffer.add(pos * self.slot_size) };
            Self::dcache_invalid_range(slot as usize, self.slot_size);
            let slot_token = unsafe { (slot.add(6) as *const u16).read_volatile() };
            if u16::from_le(slot_token) == token {
                break;
            }
            pos = (pos + 1) % self.num_slots;
            if pos == tail {
                return Err(());
            }
        }

        // Move the matched message to the head slot if it is not already
        // the first message.
        if pos != head {
            let head_slot = unsafe { self.buffer.add(head * self.slot_size) };
            let pos_slot = unsafe { self.buffer.add(pos * self.slot_size) };
            unsafe {
                for i in 0..self.slot_size {
                    let a = head_slot.add(i).read();
                    let b = pos_slot.add(i).read();
                    head_slot.add(i).write(b);
                    pos_slot.add(i).write(a);
                }
            }
        }

        // Read header and payload from the head slot.
        let slot = unsafe { self.buffer.add(head * self.slot_size) };
        let datalen = unsafe { u16::from_le((slot.add(4) as *const u16).read_volatile()) } as usize;
        let n = datalen.min(out.len());
        if n > 0 {
            unsafe {
                core::ptr::copy_nonoverlapping(slot.add(RPMI_MSG_HDR_SIZE), out.as_mut_ptr(), n);
            }
        }

        // Publish the advanced head index.
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { &*self.head }.write(((head + 1) % self.num_slots) as u32);
        // Flush the head index so the PuC sees the freed slot (mirrors
        // OpenSBI `__DCACHE_CIPA(headptr)` in `__smq_rx`).
        Self::dcache_clean_invalid_range(self.head as usize, core::mem::size_of::<Le32>());
        Ok(n)
    }

    /// Dequeue a notification message from the ring, matching the message
    /// identifier (service group ID + service ID + message type) instead of
    /// a token.
    ///
    /// Notifications carry no token, so they are matched by their message
    /// identifier (mirrors OpenSBI `__smq_rx` with `no_rx_token`). A
    /// `service_id` of `0xff` matches any service within the group. The
    /// payload is copied into `out` and the head index is advanced. Returns
    /// the number of payload bytes copied, or `Err(())` when no matching
    /// message is present.
    pub unsafe fn receive_by_message_id(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        msg_type: u8,
        out: &mut [u8],
    ) -> Result<usize, ()> {
        Self::dcache_invalid_range(self.tail as usize, core::mem::size_of::<Le32>());
        if self.is_empty() {
            return Err(());
        }
        let head = unsafe { &*self.head }.read() as usize;
        let tail = unsafe { &*self.tail }.read() as usize;

        // Locate the slot whose message identifier matches.
        let mut pos = head;
        loop {
            let slot = unsafe { self.buffer.add(pos * self.slot_size) };
            Self::dcache_invalid_range(slot as usize, self.slot_size);
            let sgid = unsafe { u16::from_le((slot as *const u16).read_volatile()) };
            let sid = unsafe { (slot.add(2) as *const u8).read_volatile() };
            let flags = unsafe { (slot.add(3) as *const u8).read_volatile() };
            let sid_match = service_id == 0xff || sid == service_id;
            if sgid == servicegroup_id && sid_match && (flags & 0x7) == msg_type {
                break;
            }
            pos = (pos + 1) % self.num_slots;
            if pos == tail {
                return Err(());
            }
        }

        // Move the matched message to the head slot if it is not already
        // the first message.
        if pos != head {
            let head_slot = unsafe { self.buffer.add(head * self.slot_size) };
            let pos_slot = unsafe { self.buffer.add(pos * self.slot_size) };
            unsafe {
                for i in 0..self.slot_size {
                    let a = head_slot.add(i).read();
                    let b = pos_slot.add(i).read();
                    head_slot.add(i).write(b);
                    pos_slot.add(i).write(a);
                }
            }
        }

        // Read header and payload from the head slot.
        let slot = unsafe { self.buffer.add(head * self.slot_size) };
        let datalen = unsafe { u16::from_le((slot.add(4) as *const u16).read_volatile()) } as usize;
        let n = datalen.min(out.len());
        if n > 0 {
            unsafe {
                core::ptr::copy_nonoverlapping(slot.add(RPMI_MSG_HDR_SIZE), out.as_mut_ptr(), n);
            }
        }

        // Publish the advanced head index.
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { &*self.head }.write(((head + 1) % self.num_slots) as u32);
        // Flush the head index so the PuC sees the freed slot (mirrors
        // OpenSBI `__DCACHE_CIPA(headptr)` in `__smq_rx`).
        Self::dcache_clean_invalid_range(self.head as usize, core::mem::size_of::<Le32>());
        Ok(n)
    }
}

/// Queue identifiers (`rpmi_msgprot.h` `enum rpmi_queue_idx`).
pub mod queue_idx {
    /// AP to PuC request.
    pub const A2P_REQ: usize = 0;
    /// PuC to AP acknowledgement.
    pub const P2A_ACK: usize = 1;
    /// PuC to AP request.
    pub const P2A_REQ: usize = 2;
    /// AP to PuC acknowledgement.
    pub const A2P_ACK: usize = 3;
    /// Number of queues.
    pub const MAX_COUNT: usize = 4;
}

/// RPMI shared-memory mailbox controller.
///
/// Owns the four ring queues and an optional doorbell register used to
/// notify the platform management processor. All operations take `&self`
/// so a single mailbox can be shared between several extensions.
pub struct RpmiMailbox {
    /// Size of one queue slot in bytes.
    slot_size: usize,
    /// The four RPMI queues in `queue_idx` order.
    queues: [SmqQueue; queue_idx::MAX_COUNT],
    /// Optional doorbell register (AP to PuC).
    doorbell: Option<&'static Le32>,
    /// Next token to use for requests.
    next_token: AtomicU16,
}

impl RpmiMailbox {
    /// Create a new mailbox controller.
    ///
    /// # Safety
    ///
    /// Every queue must alias shared memory accessible to both the AP and
    /// the PuC (see [`SmqQueue::new`]).
    pub unsafe fn new(
        slot_size: usize,
        queues: [SmqQueue; queue_idx::MAX_COUNT],
        doorbell: Option<&'static Le32>,
    ) -> Self {
        Self {
            slot_size,
            queues,
            doorbell,
            next_token: AtomicU16::new(1),
        }
    }

    /// Allocate the next message token.
    fn alloc_token(&self) -> u16 {
        let mut token = self.next_token.load(Ordering::Relaxed);
        loop {
            let next = token.wrapping_add(1) & RPMI_MSG_TOKEN_MASK;
            if next == 0 {
                continue;
            }
            match self.next_token.compare_exchange_weak(
                token,
                next,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => return next,
                Err(current) => token = current,
            }
        }
    }

    /// Perform a normal RPMI request expecting a response.
    ///
    /// Sends a `NormalRequest` message on the A2P_REQ queue and waits for
    /// the matching acknowledgement on the P2A_ACK queue. The response
    /// payload is copied into `resp` and the first word (the status code)
    /// is returned as a [`Status`].
    ///
    /// Returns `Err(())` on transport failure (queue full, timeout).
    pub fn normal_request_with_status(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        req: &[u8],
        resp: &mut [u8],
    ) -> Result<(Status, usize), ()> {
        if resp.len() < 4 {
            return Err(());
        }
        let token = self.alloc_token();
        let header = MessageHeader::from_fields(
            servicegroup_id,
            service_id,
            MessageType::NormalRequest.bits(),
            req.len() as u16,
            token,
        );
        // Safety: the queue aliases shared memory established at `new`.
        unsafe {
            self.queues[queue_idx::A2P_REQ]
                .send(&header, req, self.doorbell)
                .map_err(|_| ())?;
        }
        // Wait for the acknowledgement carrying our token. OpenSBI treats
        // `RPMI_DEF_RX_TIMEOUT` as milliseconds; counting bare spin
        // iterations only spans a few microseconds, which starves PuC
        // services that answer through slow buses (the RTC and voltage
        // reads go over I2C). Poll against a real-time mtime deadline
        // instead (mtime ticks at 24 MHz on the K3).
        let deadline = mtime_ticks() + RPMI_DEF_RX_TIMEOUT as u64 * MTIME_FREQ_HZ / 1000;
        let mut rx = [0u8; 256];
        loop {
            // Safety: as above.
            let n = unsafe { self.queues[queue_idx::P2A_ACK].receive(token, &mut rx) };
            if let Ok(n) = n {
                let status = Status::try_from(u32::from_le_bytes([rx[0], rx[1], rx[2], rx[3]]))
                    .unwrap_or(Status::Failed);
                // `rx` is the full acknowledgement payload
                // [status(4)][response data]. Copy it wholesale so the caller
                // sees the same layout OpenSBI's rpmi_normal_request_with_status
                // produces (status at offset 0, data from offset 4).
                let copy = n.min(resp.len());
                if copy > 0 {
                    resp[..copy].copy_from_slice(&rx[..copy]);
                }
                return Ok((status, copy));
            }
            if mtime_ticks() >= deadline {
                break;
            }
            // No message yet: yield and retry.
            core::hint::spin_loop();
        }
        warn!(
            "RPMI send TIMEOUT: sg={} svc={} token={} (no ack in {} ms)",
            servicegroup_id, service_id, token, RPMI_DEF_RX_TIMEOUT
        );
        Err(())
    }

    /// Perform a posted RPMI request without a response.
    ///
    /// Sends a `PostedRequest` message on the A2P_REQ queue and returns
    /// immediately.
    pub fn posted_request(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        req: &[u8],
    ) -> Result<(), ()> {
        let token = self.alloc_token();
        let header = MessageHeader::from_fields(
            servicegroup_id,
            service_id,
            MessageType::PostedRequest.bits(),
            req.len() as u16,
            token,
        );
        // Safety: the queue aliases shared memory established at `new`.
        unsafe {
            self.queues[queue_idx::A2P_REQ]
                .send(&header, req, self.doorbell)
                .map_err(|_| ())
        }
    }

    /// Receive an asynchronous notification from the platform management
    /// processor.
    ///
    /// Notifications arrive on the P2A_REQ queue and carry no token; they
    /// are matched by their message identifier (service group ID + service
    /// ID + notification type). The payload is copied into `out`. Returns
    /// the number of payload bytes copied, or `Err(())` when no matching
    /// notification is pending.
    pub fn receive_notification(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        out: &mut [u8],
    ) -> Result<usize, ()> {
        // Safety: the queue aliases shared memory established at `new`.
        unsafe {
            self.queues[queue_idx::P2A_REQ].receive_by_message_id(
                servicegroup_id,
                service_id,
                MessageType::Notification.bits(),
                out,
            )
        }
    }

    /// Returns the slot size of this mailbox.
    pub const fn slot_size(&self) -> usize {
        self.slot_size
    }

    /// Debug helper: the doorbell register address (or 0).
    pub fn doorbell_addr(&self) -> usize {
        self.doorbell.map(|db| db as *const _ as usize).unwrap_or(0)
    }
}
