use rustsbi::SbiRet;
use spin::Mutex;

use rpmi::RpmiMailbox;
use rpmi::message::{
    CppcProbeReq, CppcReadReq, CppcWriteReq, Error as RpmiError, cppc_service, servicegroup,
};

use crate::riscv::current_hartid;

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

/// Implementation of SBI CPPC extension.
///
/// CPPC register accesses are forwarded to the RPMI CPPC service group
/// through the shared-memory mailbox. The mailbox backend is injected via
/// [`SbiCppc::set_mailbox`]; until a platform provides one, register probes
/// report a zero width and accesses are rejected as not supported.
pub(crate) struct SbiCppc {
    mailbox: Mutex<Option<&'static RpmiMailbox>>,
}

impl SbiCppc {
    /// Create a new CPPC extension without a mailbox backend.
    pub(crate) const fn new() -> Self {
        Self {
            mailbox: Mutex::new(None),
        }
    }

    /// Inject the platform mailbox backend.
    pub(crate) fn set_mailbox(&self, mailbox: &'static RpmiMailbox) {
        *self.mailbox.lock() = Some(mailbox);
    }
}

impl rustsbi::Cppc for SbiCppc {
    fn probe(&self, reg_id: u32) -> SbiRet {
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            // No backend: register not implemented (width 0).
            return SbiRet::success(0);
        };
        let req = CppcProbeReq {
            hart_id: current_hartid() as u32,
            reg_id,
        };
        let mut resp = [0u8; 8];
        match mbox.normal_request_with_status(
            servicegroup::CPPC,
            cppc_service::PROBE_REG,
            as_bytes(&req),
            &mut resp,
        ) {
            Ok((RpmiError::Success, _)) => {
                let reg_len = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
                SbiRet::success(reg_len as usize)
            }
            Ok((err, _)) => rpmi_error_to_sbi(err),
            Err(()) => SbiRet::timeout(),
        }
    }

    fn read(&self, reg_id: u32) -> SbiRet {
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            return SbiRet::not_supported();
        };
        let req = CppcReadReq {
            hart_id: current_hartid() as u32,
            reg_id,
        };
        let mut resp = [0u8; 12];
        match mbox.normal_request_with_status(
            servicegroup::CPPC,
            cppc_service::READ_REG,
            as_bytes(&req),
            &mut resp,
        ) {
            Ok((RpmiError::Success, _)) => {
                let lo = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
                SbiRet::success(lo as usize)
            }
            Ok((err, _)) => rpmi_error_to_sbi(err),
            Err(()) => SbiRet::timeout(),
        }
    }

    fn read_hi(&self, reg_id: u32) -> SbiRet {
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            return SbiRet::not_supported();
        };
        let req = CppcReadReq {
            hart_id: current_hartid() as u32,
            reg_id,
        };
        let mut resp = [0u8; 12];
        match mbox.normal_request_with_status(
            servicegroup::CPPC,
            cppc_service::READ_REG,
            as_bytes(&req),
            &mut resp,
        ) {
            Ok((RpmiError::Success, _)) => {
                let hi = u32::from_le_bytes([resp[8], resp[9], resp[10], resp[11]]);
                SbiRet::success(hi as usize)
            }
            Ok((err, _)) => rpmi_error_to_sbi(err),
            Err(()) => SbiRet::timeout(),
        }
    }

    fn write(&self, reg_id: u32, val: u64) -> SbiRet {
        let mailbox = self.mailbox.lock();
        let Some(mbox) = mailbox.as_ref().copied() else {
            return SbiRet::not_supported();
        };
        let req = CppcWriteReq {
            hart_id: current_hartid() as u32,
            reg_id,
            data_lo: val as u32,
            data_hi: (val >> 32) as u32,
        };
        let mut resp = [0u8; 4];
        match mbox.normal_request_with_status(
            servicegroup::CPPC,
            cppc_service::WRITE_REG,
            as_bytes(&req),
            &mut resp,
        ) {
            Ok((RpmiError::Success, _)) => SbiRet::success(0),
            Ok((err, _)) => rpmi_error_to_sbi(err),
            Err(()) => SbiRet::timeout(),
        }
    }
}

/// View a `#[repr(C)]` structure as its little-endian wire bytes.
fn as_bytes<T>(value: &T) -> &[u8] {
    // Safety: the structure is `#[repr(C)]` with only integer fields; the
    // wire format is the native (little-endian) byte order on RISC-V.
    unsafe {
        core::slice::from_raw_parts(value as *const T as *const u8, core::mem::size_of::<T>())
    }
}
