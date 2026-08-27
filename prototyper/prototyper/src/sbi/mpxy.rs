use core::sync::atomic::{AtomicUsize, Ordering};

use rustsbi::SbiRet;
use sbi_spec::binary::SharedPtr;
use spin::Mutex;

use rpmi::RpmiMailbox;
use rpmi::message::{Error as RpmiError, servicegroup};

/// SBI MPXY standard channel attribute IDs (sbi.h `enum sbi_mpxy_attribute_id`).
mod channel_attr {
    pub const MSG_PROT_ID: u32 = 0;
    pub const MSG_PROT_VERSION: u32 = 1;
    pub const MSG_MAX_LEN: u32 = 2;
    pub const MSG_SEND_TIMEOUT: u32 = 3;
    pub const MSG_COMPLETION_TIMEOUT: u32 = 4;
    pub const CAPABILITY: u32 = 5;
    pub const SSE_EVENT_ID: u32 = 6;
    pub const MSI_CONTROL: u32 = 7;
    pub const MSI_ADDR_LO: u32 = 8;
    pub const MSI_ADDR_HI: u32 = 9;
    pub const MSI_DATA: u32 = 10;
    pub const EVENTS_STATE_CONTROL: u32 = 11;
    /// RPMI message-protocol attributes begin here (SBI_MPXY_ATTR_MSGPROTO_ATTR_START).
    pub const RPMI_SERVICEGROUP_ID: u32 = 0x8000_0000;
    pub const RPMI_SERVICEGROUP_VERSION: u32 = 0x8000_0001;
    pub const RPMI_IMPL_ID: u32 = 0x8000_0002;
    pub const RPMI_IMPL_VERSION: u32 = 0x8000_0003;
}

/// RPMI protocol version reported for every channel.
const RPMI_PROTOCOL_VERSION: u32 = 0x0001_0000; // v1.0
/// RPMI implementation ID reported for every channel.
const RPMI_IMPL_ID: u32 = 0x0;
/// RPMI implementation version reported for every channel.
const RPMI_IMPL_VERSION: u32 = 0x1;
/// Maximum message data length per channel. The K3 mailbox uses a
/// slot size of 256 bytes, so the payload limit is `slot_size - 8`
/// (`RPMI_MSG_DATA_SIZE(256)`), matching OpenSBI.
const RPMI_MAX_DATA_LEN: u32 = 248;
/// RPMI message protocol ID reported as MSG_PROT_ID (sbi.h SBI_MPXY_MSGPROTO_RPMI_ID).
const RPMI_MSGPROTO_ID: u32 = 0x0;
/// Channel capability bitmask (sbi.h `SBI_MPXY_CHAN_CAP_*`): send-with/without
/// response plus notification events-state; MSI is not advertised so the
/// mailbox controller skips the MSI setup path.
const CHANNEL_CAPABILITY: u32 = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 2); // SEND_WITH_RESP|SEND_WITHOUT_RESP|GET_NOTIFICATIONS|EVENTS_STATE

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
    servicegroup::RTC,
    servicegroup::PWRKEY,
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
    // MPXY shared memory is per-hart: Linux's riscv-sbi-mpxy-mbox driver
    // allocates a separate shmem page for every CPU and calls SET_SHMEM on
    // each (mpxy_setup_shmem). OpenSBI tracks this per-hart
    // (hart_mpxy_state_get); a single shared field would end up pointing at
    // the last CPU's page while the current hart reads its own, so the S-mode
    // side sees zeros and reports "no MPXY channels available".
    shmem: [AtomicUsize; crate::cfg::NUM_HART_MAX],
}

impl SbiMpxy {
    /// Create a new MPXY extension without a mailbox backend.
    pub(crate) const fn new() -> Self {
        Self {
            mailbox: Mutex::new(None),
            shmem: [const { AtomicUsize::new(0) }; crate::cfg::NUM_HART_MAX],
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

    /// Returns the current hart's MPXY shared-memory address.
    #[inline]
    fn shmem_hart(&self) -> &AtomicUsize {
        &self.shmem[crate::riscv::current_hartid()]
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
                // Standard SBI MPXY channel attributes
                channel_attr::MSG_PROT_ID => RPMI_MSGPROTO_ID,
                channel_attr::MSG_PROT_VERSION => RPMI_PROTOCOL_VERSION,
                channel_attr::MSG_MAX_LEN => RPMI_MAX_DATA_LEN,
                channel_attr::MSG_SEND_TIMEOUT => rpmi::mailbox::RPMI_DEF_TX_TIMEOUT,
                channel_attr::MSG_COMPLETION_TIMEOUT => rpmi::mailbox::RPMI_DEF_RX_TIMEOUT,
                channel_attr::CAPABILITY => CHANNEL_CAPABILITY,
                channel_attr::SSE_EVENT_ID => 0,
                channel_attr::MSI_CONTROL => 0,
                channel_attr::MSI_ADDR_LO => 0,
                channel_attr::MSI_ADDR_HI => 0,
                channel_attr::MSI_DATA => 0,
                channel_attr::EVENTS_STATE_CONTROL => 0,
                // RPMI message-protocol attributes (channel ID == service group ID)
                channel_attr::RPMI_SERVICEGROUP_ID => channel_id,
                channel_attr::RPMI_SERVICEGROUP_VERSION => 0x0001_0000,
                channel_attr::RPMI_IMPL_ID => RPMI_IMPL_ID,
                channel_attr::RPMI_IMPL_VERSION => RPMI_IMPL_VERSION,
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
            self.shmem_hart().store(0, Ordering::Release);
            return SbiRet::success(0);
        }
        if shmem.phys_addr_lo() & 0xfff != 0 {
            return SbiRet::invalid_param();
        }
        self.shmem_hart().store(shmem.phys_addr_lo(), Ordering::Release);
        SbiRet::success(0)
    }

    fn get_channel_ids(&self, start_index: u32) -> SbiRet {
        let shmem = self.shmem_hart().load(Ordering::Acquire);
        if shmem == 0 {
            return SbiRet::no_shmem();
        }
        // Mirror OpenSBI sbi_mpxy.c sbi_mpxy_get_channel_ids():
        //  - start_index > count => invalid_param (start_index == count is
        //    valid and returns zero remaining/returned).
        //  - REMAINING (shmem[0]) is the number of channel IDs left AFTER the
        //    returned set; RETURNED (shmem[1]) is how many were written.
        //  - channel IDs follow at shmem[2..].
        let count = CHANNELS.len() as u32;
        if start_index > count {
            return SbiRet::invalid_param();
        }
        // Number of channel IDs that fit after the remaining/returned fields.
        let max_channelids = (self.get_shmem_size() / 4) - 2;
        let remaining_before = count - start_index;
        let returned = remaining_before.min(max_channelids as u32);
        let remaining_after = count - (start_index + returned);
        // Safety: the shared memory is S-mode owned and writable.
        let base = shmem as *mut u8;
        unsafe {
            base.add(0).cast::<u32>().write_volatile(remaining_after);
            base.add(4).cast::<u32>().write_volatile(returned);
            for i in 0..returned {
                base.add(8 + i as usize * 4)
                    .cast::<u32>()
                    .write_volatile(CHANNELS[(start_index + i) as usize] as u32);
            }
        }
        SbiRet::success(0)
    }

    fn read_attributes(
        &self,
        channel_id: u32,
        base_attribute_id: u32,
        attribute_count: u32,
        _output: SharedPtr<u8>,
    ) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        if attribute_count == 0 {
            return SbiRet::invalid_param();
        }
        // SBI MPXY READ_ATTRIBUTES writes the attribute values into the
        // shared memory established by SET_SHMEM (see OpenSBI
        // sbi_mpxy_read_attributes, which targets `hart_shmem_base(ms)`); the
        // `output` register argument is not used by the SBI ABI.
        let shmem = self.shmem_hart().load(Ordering::Acquire);
        if shmem == 0 {
            return SbiRet::no_shmem();
        }
        // Safety: the shared memory is S-mode owned and writable.
        let out = unsafe {
            core::slice::from_raw_parts_mut(shmem as *mut u8, (attribute_count as usize) * 4)
        };
        if !self.read_channel_attrs(channel_id, base_attribute_id, attribute_count, out) {
            return SbiRet::bad_range();
        }
        if base_attribute_id == 0 && attribute_count >= 2 {
            let v0 = u32::from_le_bytes([out[0], out[1], out[2], out[3]]);
            let v1 = u32::from_le_bytes([out[4], out[5], out[6], out[7]]);
            info!(
                "MPXY read_attrs DIAG: chan={} shmem=0x{:x} attr0={} attr1(proto_ver)={:#x}",
                channel_id, shmem, v0, v1
            );
        }
        SbiRet::success(0)
    }

    fn write_attributes(
        &self,
        channel_id: u32,
        base_attribute_id: u32,
        attribute_count: u32,
        _input: SharedPtr<u8>,
    ) -> SbiRet {
        if !self.is_channel(channel_id) {
            return SbiRet::not_supported();
        }
        if attribute_count == 0 {
            return SbiRet::invalid_param();
        }
        // Allow toggling the events-state control and MSI control (RustSBI
        // manages the notification buffer internally; MSI is unused but the
        // driver may still write it). All other channel attributes are
        // read-only.
        if base_attribute_id == channel_attr::EVENTS_STATE_CONTROL
            || base_attribute_id == channel_attr::MSI_CONTROL
        {
            return SbiRet::success(0);
        }
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
        let shmem = self.shmem_hart().load(Ordering::Acquire);
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
            Ok((RpmiError::Success, len)) => {
                info!(
                    "MPXY send DIAG: chan={} svc={} len={} resp0={} resp1={}",
                    channel_id,
                    service_id,
                    len,
                    u32::from_le_bytes([resp[0], resp[1], resp[2], resp[3]]),
                    u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]])
                );
                // Write the full [status][data] response back at offset 0x0;
                // return the actual response length.
                unsafe {
                    core::ptr::copy_nonoverlapping(resp.as_ptr(), shmem as *mut u8, len);
                }
                SbiRet::success(len)
            }
            Ok((err, _)) => {
                warn!(
                    "MPXY send DIAG: chan={} svc={} ERR={:?}",
                    channel_id, service_id, err
                );
                rpmi_error_to_sbi(err)
            }
            Err(()) => {
                warn!(
                    "MPXY send DIAG: chan={} svc={} TIMEOUT (no ack)",
                    channel_id, service_id
                );
                SbiRet::timeout()
            }
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
        let shmem = self.shmem_hart().load(Ordering::Acquire);
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
        let shmem = self.shmem_hart().load(Ordering::Acquire);
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
