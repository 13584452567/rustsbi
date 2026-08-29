use alloc::boxed::Box;
use rustsbi::SbiRet;
use spin::Mutex;

use crate::platform::BoardInfo;
use crate::platform::reset::{P1PmicResetWrap, SifiveTestDeviceWrap};

/// RPMI System Reset service group IDs (rpmi_msgprot.h
/// `enum rpmi_sysrst_service_id` / `enum rpmi_sysrst_reset_type`).
mod rpmi_sysrst {
    pub const SERVICE_SYSTEM_RESET: u8 = 0x03;
    pub const TYPE_SHUTDOWN: u8 = 0x0;
    pub const TYPE_COLD_REBOOT: u8 = 0x1;
    pub const TYPE_WARM_REBOOT: u8 = 0x2;
}

/// K3 system reset device: forwards SBI reset requests to the RPMI System
/// Reset service group over the injected shared-memory mailbox (mirrors
/// OpenSBI `fdt_reset_rpmi.c`). The mailbox is injected after the reset
/// extension is constructed, so it is resolved lazily on each request.
pub(crate) struct RpmiResetWrap;

impl RpmiResetWrap {
    fn do_reset(&self, reset_type: u8) -> ! {
        if let Some(mpxy) = crate::sbi::mpxy() {
            if let Some(mailbox) = mpxy.mailbox() {
                let req = (reset_type as u32).to_le_bytes();
                let _ = mailbox.posted_request(
                    crate::rpmi::servicegroup::SYSTEM_RESET,
                    rpmi_sysrst::SERVICE_SYSTEM_RESET,
                    &req,
                );
            }
        }
        // If the mailbox is unavailable, hang as a last resort.
        loop {
            core::hint::spin_loop()
        }
    }
}

impl ResetDevice for RpmiResetWrap {
    fn fail(&self, _code: u16) -> ! {
        self.do_reset(rpmi_sysrst::TYPE_SHUTDOWN)
    }
    fn pass(&self) -> ! {
        self.do_reset(rpmi_sysrst::TYPE_SHUTDOWN)
    }
    fn reset(&self) -> ! {
        self.do_reset(rpmi_sysrst::TYPE_COLD_REBOOT)
    }
}

pub trait ResetDevice: Send {
    fn fail(&self, code: u16) -> !;
    fn pass(&self) -> !;
    fn reset(&self) -> !;
}

pub struct SbiReset {
    pub reset_dev: Mutex<Box<dyn ResetDevice>>,
}

impl SbiReset {
    pub fn new(reset_dev: Mutex<Box<dyn ResetDevice>>) -> Self {
        Self { reset_dev }
    }

    #[allow(unused)]
    pub fn fail(&self) -> ! {
        trace!("Test fail, invoke process exit procedure on Reset device");
        self.reset_dev.lock().fail(0);
    }
}

impl rustsbi::Reset for SbiReset {
    #[inline]
    fn system_reset(&self, reset_type: u32, reset_reason: u32) -> SbiRet {
        use rustsbi::spec::srst::{
            RESET_REASON_NO_REASON, RESET_REASON_SYSTEM_FAILURE, RESET_TYPE_COLD_REBOOT,
            RESET_TYPE_SHUTDOWN, RESET_TYPE_WARM_REBOOT,
        };
        match reset_type {
            RESET_TYPE_SHUTDOWN => match reset_reason {
                RESET_REASON_NO_REASON => self.reset_dev.lock().pass(),
                RESET_REASON_SYSTEM_FAILURE => self.reset_dev.lock().fail(u16::MAX),
                value => self.reset_dev.lock().fail(value as _),
            },
            RESET_TYPE_COLD_REBOOT | RESET_TYPE_WARM_REBOOT => self.reset_dev.lock().reset(),

            _ => SbiRet::invalid_param(),
        }
    }
}

#[allow(unused)]
pub fn fail() -> ! {
    match crate::sbi::reset() {
        Some(reset) => reset.fail(),
        None => {
            trace!("test fail, begin dead loop");
            loop {
                core::hint::spin_loop()
            }
        }
    }
}

/// Initializes the SBI reset extension from the discovered board info.
pub(crate) fn init(board: &BoardInfo) -> Option<SbiReset> {
    if let Some(base) = board.reset {
        Some(SbiReset::new(Mutex::new(Box::new(
            SifiveTestDeviceWrap::new(base),
        ))))
    } else if let Some((i2c_base, pmic_addr)) = board.pmic_reset {
        Some(SbiReset::new(Mutex::new(Box::new(P1PmicResetWrap::new(
            i2c_base, pmic_addr,
        )))))
    } else if board.rpmi_reset {
        // K3: system reset is an RPMI System Reset service-group request
        // delivered over the shared-memory mailbox (mirrors OpenSBI
        // `fdt_reset_rpmi.c`). The mailbox is injected later in boot, so the
        // request is resolved lazily at reset time.
        Some(SbiReset::new(Mutex::new(Box::new(RpmiResetWrap))))
    } else {
        None
    }
}
