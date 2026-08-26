//! RPMI Shared Memory Queue (SMQ) transport.
//!
//! The platform management processor (PuC) and the application processor
//! (AP) exchange RPMI messages through four shared-memory ring queues:
//! A2P_REQ (AP→PuC request), P2A_ACK (PuC→AP acknowledgement), P2A_REQ
//! (PuC→AP request) and A2P_ACK (AP→PuC acknowledgement).
//!
//! Each queue is a ring of fixed-size slots. The head index is written by
//! the queue reader and the tail index by the queue writer; all indices and
//! message fields in shared memory are little-endian. The implementation
//! mirrors OpenSBI `lib/utils/mailbox/fdt_mailbox_rpmi_shmem.c`
//! (`__smq_tx` / `__smq_rx`).

use crate::message::MessageHeader;

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

    /// Enqueue a message into the ring.
    ///
    /// Writes the header and payload into the tail slot, publishes the
    /// little-endian tail index, and rings the optional doorbell register.
    /// Returns `Err(())` when the queue is full.
    ///
    /// # Safety
    ///
    /// `data.len()` must not exceed `slot_size - 8`.
    pub unsafe fn send(&self, header: &MessageHeader, data: &[u8], doorbell: Option<&Le32>) -> Result<(), ()> {
        if self.is_full() {
            return Err(());
        }
        if data.len() > self.slot_size - crate::message::RPMI_MSG_HDR_SIZE {
            return Err(());
        }

        let tail = unsafe { &*self.tail }.read() as usize;
        let slot = unsafe { self.buffer.add(tail * self.slot_size) };

        // Write header fields little-endian.
        unsafe {
            (slot as *mut u16).write_volatile(header.servicegroup_id.to_le());
            (slot.add(2) as *mut u8).write_volatile(header.service_id);
            (slot.add(3) as *mut u8).write_volatile(header.flags);
            (slot.add(4) as *mut u16).write_volatile(header.datalen.to_le());
            (slot.add(6) as *mut u16).write_volatile(header.token.to_le());
        }
        // Copy payload.
        if !data.is_empty() {
            unsafe {
                core::ptr::copy_nonoverlapping(
                    data.as_ptr(),
                    slot.add(crate::message::RPMI_MSG_HDR_SIZE),
                    data.len(),
                );
            }
        }

        // Make queue changes visible before publishing the tail index.
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { &*self.tail }.write(((tail + 1) % self.num_slots) as u32);

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
        if self.is_empty() {
            return Err(());
        }
        let head = unsafe { &*self.head }.read() as usize;
        let tail = unsafe { &*self.tail }.read() as usize;

        // Locate the slot with the matching token.
        let mut pos = head;
        loop {
            let slot = unsafe { self.buffer.add(pos * self.slot_size) };
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
                core::ptr::copy_nonoverlapping(
                    slot.add(crate::message::RPMI_MSG_HDR_SIZE),
                    out.as_mut_ptr(),
                    n,
                );
            }
        }

        // Publish the advanced head index.
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { &*self.head }.write(((head + 1) % self.num_slots) as u32);
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
        if self.is_empty() {
            return Err(());
        }
        let head = unsafe { &*self.head }.read() as usize;
        let tail = unsafe { &*self.tail }.read() as usize;

        // Locate the slot whose message identifier matches.
        let mut pos = head;
        loop {
            let slot = unsafe { self.buffer.add(pos * self.slot_size) };
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
                core::ptr::copy_nonoverlapping(
                    slot.add(crate::message::RPMI_MSG_HDR_SIZE),
                    out.as_mut_ptr(),
                    n,
                );
            }
        }

        // Publish the advanced head index.
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { &*self.head }.write(((head + 1) % self.num_slots) as u32);
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::boxed::Box;

    use super::*;
    use crate::message::{MessageType, servicegroup};

    /// Build a queue over a leaked heap buffer so tests run on the host.
    fn test_queue() -> (SmqQueue, &'static mut [Le32]) {
        let buffer = Box::leak(Box::new([0u8; 4 * 64]));
        let indices = Box::leak(Box::new([Le32(0), Le32(0)]));
        let queue = unsafe {
            SmqQueue::new(
                indices.as_ptr(),
                indices.as_ptr().add(1),
                buffer.as_mut_ptr(),
                64,
                4,
            )
        };
        (queue, indices)
    }

    #[test]
    fn test_roundtrip() {
        let (queue, indices) = test_queue();
        let header = MessageHeader::new(servicegroup::HSM, 0x01, MessageType::NormalRequest, 4, 0x1234);
        let data = [1u8, 2, 3, 4];

        unsafe {
            queue.send(&header, &data, None).unwrap();
        }
        // The sender writes tail; emulate the PuC having consumed nothing.
        assert_eq!(indices[1].read(), 1);

        let mut out = [0u8; 16];
        let n = unsafe { queue.receive(0x1234, &mut out).unwrap() };
        assert_eq!(n, 4);
        assert_eq!(&out[..4], &data);
        assert_eq!(indices[0].read(), 1);
    }

    #[test]
    fn test_token_mismatch() {
        let (queue, _) = test_queue();
        let header = MessageHeader::new(servicegroup::BASE, 0x05, MessageType::NormalRequest, 0, 7);
        unsafe {
            queue.send(&header, &[], None).unwrap();
            assert!(queue.receive(8, &mut [0u8; 4]).is_err());
        }
    }

    #[test]
    fn test_full_queue() {
        let (queue, _) = test_queue();
        for i in 0..3 {
            let header = MessageHeader::new(servicegroup::BASE, 0, MessageType::NormalRequest, 0, i);
            unsafe {
                queue.send(&header, &[], None).unwrap();
            }
        }
        let header = MessageHeader::new(servicegroup::BASE, 0, MessageType::NormalRequest, 0, 99);
        unsafe {
            assert!(queue.send(&header, &[], None).is_err());
        }
    }
}
