use core::sync::atomic::{AtomicUsize, Ordering};

use rustsbi::SbiRet;
use sbi_spec::binary::SharedPtr;
use spin::Mutex;

use rpmi::RpmiMailbox;
use rpmi::message::{Error as RpmiError, servicegroup};

/// RPMI channel attribute IDs (rpmi_msgprot.h
/// `enum rpmi_channel_attribute_id`).
mod channel_attr {
    pub const PROTOCOL_VERSION: u32 = 0;
    pub const MAX_DATA_LEN: u32 = 1;
    pub const TX_TIMEOUT: u32 = 2;
    pub const RX_TIMEOUT: u32 = 3;
    pub const SERVICEGROUP_ID: u32 = 4;
    pub const SERVICEGROUP_VERSION: u32 = 5;
    pub const IMPL_ID: u32 = 6;
    pub const IMPL_VERSION: u32 = 7;
}

/// RPMI protocol version reported for every channel.
const RPMI_PROTOCOL_VERSION: u32 = 0x0001_0000; // v1.0
/// RPMI implementation ID reported for every channel.
const RPMI_IMPL_ID: u32 = 0x0;
/// RPMI implementation version reported for every channel.
const RPMI_IMPL_VERSION: u32 = 0x1;
/// Maximum message data length per channel.
const RPMI_MAX_DATA_LEN: u32 = 56;

/// The RPMI service groups exposed as MPXY channels. The channel ID is the
/// RPMI service group ID.
const CHANNELS: &[u16] = &[
    servicegroup::BASE,
    servicegroup::SYSTEM_RESET,
    servicegroup::SYSTEM_SUSPEND,
    servicegroup::HSM,
    servicegroup::CPPC,
    servicegroup::VOLTAGE,
    servicegroup::CLOCK,
    servicegroup::DOMAIN,
];

/// Convert an RPMI error to an SBI return value (mirrors OpenSBI
/// `rpmi_xlate_error`).
fn rpmi_error_to_sbi(err: RpmiError) -> SbiRet {
    match err {
        RpmiError::Success => SbiRet::success(0),
        RpmiError::Failed => SbiRet::failed(),
        RpmiError::NotSupported => SbiRet::not_supported(),
        RpmiError::InvalidParam => SbiRet::invalid_param(),
        RpmiError::Denied => SbiRet::denied(),
        RpmiError::InvalidAddr => SbiRet::invalid_address(),
        RpmiError::Already => SbiRet::already_available(),
        RpmiError::Extension => SbiRet::failed(),
        RpmiError::HwFault => SbiRet::io(),
        RpmiError::Busy => SbiRet::failed(),
        RpmiError::InvalidState => SbiRet::invalid_state(),
        RpmiError::BadRange => SbiRet::bad_range(),
        RpmiError::Timeout => SbiRet::timeout(),
        RpmiError::Io => SbiRet::io(),
        RpmiError::NoData => SbiRet::failed(),
    }
}

/// Implementation of SBI Message Proxy (MPXY) extension.
///
/// The Message Proxy extension forwards supervisor-mode messages to a
/// message protocol (RPMI) through message channels. Each MPXY channel maps
/// to one RPMI service group; the channel ID equals the service group ID
/// and the MPXY `message_id` equals the RPMI `service_id` (as in OpenSBI
/// `lib/utils/mpxy/fdt_mpxy_rpmi_mbox.c`).
///
/// The concrete shared-memory mailbox (queue pointers, doorbell) is
/// platform-specific and is injected via [`SbiMpxy::set_mailbox`]; until a
/// platform provides one, requests are reported as not supported.
pub(crate) struct SbiMpxy {
    mailbox: Mutex<Option<&'static RpmiMailbox>>,
    shmem: AtomicUsize,
}

impl SbiMpxy {
    /// Create a new MPXY extension without a mailbox backend.
    pub(crate) const fn new() -> Self {
        Self {
            mailbox: Mutex::new(None),
            shmem: AtomicUsize::new(0),
        }
    }

    /// Inject the platform mailbox backend.
    pub(crate) fn set_mailbox(&self, mailbox: &'static RpmiMailbox) {
        *self.mailbox.lock() = Some(mailbox);
    }

    /// Returns the injected mailbox backend, if any.
    pub(crate) fn mailbox(&self) -> Option<&'static RpmiMailbox> {
        *self.mailbox.lock()
    }

    /// Returns whether the given channel ID is exposed.
    fn is_channel(&self, channel_id: u32) -> bool {
        CHANNELS.iter().any(|&id| id as u32 == channel_id)
    }

    /// Read `count` channel attributes into `out` (little-endian u32s).
    fn read_channel_attrs(&self, channel_id: u32, base: u32, count: u32, out: &mut [u8]) -> bool {
        for i in 0..count {
            let attr = base + i;
            let value = match attr {
                channel_attr::PROTOCOL_VERSION => RPMI_PROTOCOL_VERSION,
                channel_attr::MAX_DATA_LEN => RPMI_MAX_DATA_LEN,
                channel_attr::TX_TIMEOUT => rpmi::mailbox::RPMI_DEF_TX_TIMEOUT,
                channel_attr::RX_TIMEOUT => rpmi::mailbox::RPMI_DEF_RX_TIMEOUT,
                channel_attr::SERVICEGROUP_ID => channel_id,
                channel_attr::SERVICEGROUP_VERSION => 0x0001_0000, // v1.0
                channel_attr::IMPL_ID => RPMI_IMPL_ID,
                channel_attr::IMPL_VERSION => RPMI_IMPL_VERSION,
                _ => return false,
            };
            let off = (i * 4) as usize;
            if off + 4 > out.len() {
                return false;
            }
            out[off..off + 4].copy_from_slice(&value.to_le_bytes());
        }
        true
    }
}

impl rustsbi::Mpxy for SbiMpxy {
    fn get_shmem_size(&self) -> usize {
        // Shared memory for request/response data plus the channel-ID
        // array header; 4 KiB aligned.
        4096
    }

    fn set_shmem(&self, shmem: SharedPtr<u8>, flags: usize) -> SbiRet {
        // Only OVERWRITE mode is supported; flags[1:0] must be 0b00.
        if flags & 0b11 != 0 {
            return SbiRet::invalid_param();
        }
        let all_ones = shmem.phys_addr_lo() == usize::MAX && shmem.phys_addr_hi() == usize::MAX;
        if all_ones {
            // Disable shared memory.
            self.shmem.store(0, Ordering::Release);
            return SbiRet::success(0);
        }
        if shmem.phys_addr_lo() & 0xfff != 0 {
            return SbiRet::invalid_param();
        }
        self.shmem.store(shmem.phys_addr_lo(), Ordering::Release);
        SbiRet::success(0)
    }

    fn get_channel_ids(&self, start_index: u32) -> SbiRet {
        let shmem = self.shmem.load(Ordering::Acquire);
        if shmem == 0 {
            return SbiRet::no_shmem();
        }
        let start = start_index as usize;
        if start >= CHANNELS.len() {
            return SbiRet::invalid_param();
        }
        // Layout: REMAINING at 0x0, RETURNED at 0x4, IDs from 0x8.
        let remaining = (CHANNELS.len() - start) as u32;
        let returned = remaining.min(1);
        // Safety: the shared memory is S-mode owned and writable.
        let base = shmem as *mut u8;
        unsafe {
            base.add(0).cast::<u32>().write_volatile(remaining);
            base.add(4).cast::<u32>().write_volatile(returned);
            base.add(8)
                .cast::<u32>()
                .write_volatile(CHANNELS[start] as u32);
        }
        SbiRet::success(0)
    }

    fn read_attributes(
        &self,
        channel_id: u32,
        base_attribute_id: u32,
        attribute_count: u32,
        output: SharedPtr<u8>,
    ) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        if attribute_count == 0 {
            return SbiRet::invalid_param();
        }
        let out = unsafe {
            core::slice::from_raw_parts_mut(
                output.phys_addr_lo() as *mut u8,
                (attribute_count as usize) * 4,
            )
        };
        if !self.read_channel_attrs(channel_id, base_attribute_id, attribute_count, out) {
            return SbiRet::bad_range();
        }
        SbiRet::success(0)
    }

    fn write_attributes(
        &self,
        channel_id: u32,
        _base_attribute_id: u32,
        _attribute_count: u32,
        _input: SharedPtr<u8>,
    ) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        // All RPMI channel attributes are read-only.
        SbiRet::denied()
    }

    fn send_message_with_response(
        &self,
        channel_id: u32,
        message_id: u32,
        message_data_len: usize,
    ) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        if message_data_len > RPMI_MAX_DATA_LEN as usize {
            return SbiRet::invalid_param();
        }
        let shmem = self.shmem.load(Ordering::Acquire);
        if shmem == 0 {
            return SbiRet::no_shmem();
        }
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            return SbiRet::not_supported();
        };
        // MPXY message_id == RPMI service_id.
        let service_id = message_id as u8;
        // Safety: the shared memory holds the request at offset 0x0.
        let req = unsafe { core::slice::from_raw_parts(shmem as *const u8, message_data_len) };
        let mut resp = [0u8; RPMI_MAX_DATA_LEN as usize];
        match mbox.normal_request_with_status(channel_id as u16, service_id, req, &mut resp) {
            Ok(RpmiError::Success) => {
                // Write the response back at offset 0x0; return its length.
                unsafe {
                    core::ptr::copy_nonoverlapping(resp.as_ptr(), shmem as *mut u8, resp.len());
                }
                SbiRet::success(resp.len())
            }
            Ok(err) => rpmi_error_to_sbi(err),
            Err(()) => SbiRet::timeout(),
        }
    }

    fn send_message_without_response(
        &self,
        channel_id: u32,
        message_id: u32,
        message_data_len: usize,
    ) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        if message_data_len > RPMI_MAX_DATA_LEN as usize {
            return SbiRet::invalid_param();
        }
        let shmem = self.shmem.load(Ordering::Acquire);
        if shmem == 0 {
            return SbiRet::no_shmem();
        }
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            return SbiRet::not_supported();
        };
        let service_id = message_id as u8;
        let req = unsafe { core::slice::from_raw_parts(shmem as *const u8, message_data_len) };
        match mbox.posted_request(channel_id as u16, service_id, req) {
            Ok(()) => SbiRet::success(0),
            Err(()) => SbiRet::timeout(),
        }
    }

    fn get_notification_events(&self, channel_id: u32) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        let shmem = self.shmem.load(Ordering::Acquire);
        if shmem == 0 {
            return SbiRet::no_shmem();
        }
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            return SbiRet::not_supported();
        };
        // Receive one pending notification for this channel (any service).
        let mut buf = [0u8; RPMI_MAX_DATA_LEN as usize];
        match mbox.receive_notification(channel_id as u16, 0xff, &mut buf) {
            Ok(n) => {
                // Events state data at offset 0x0, event payload at 0x10.
                let base = shmem as *mut u8;
                // Safety: the shared memory is S-mode owned and writable.
                unsafe {
                    base.add(0).cast::<u32>().write_volatile(0); // REMAINING
                    base.add(4).cast::<u32>().write_volatile(1); // RETURNED
                    base.add(8).cast::<u32>().write_volatile(0); // LOST
                    base.add(12).cast::<u32>().write_volatile(0); // RESERVED
                    core::ptr::copy_nonoverlapping(buf.as_ptr(), base.add(0x10), n);
                }
                SbiRet::success(n)
            }
            // No notification pending: report zero returned events.
            Err(()) => SbiRet::success(0),
        }
    }
}
