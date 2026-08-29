use core::sync::atomic::Ordering;

use riscv::register::mstatus;
use rustsbi::{Hsm, SbiRet};
use sbi_spec::hsm::{hart_state::STOPPED, suspend_type::NON_RETENTIVE};

use crate::riscv::current_hartid;
use crate::riscv::spacemit_k3;

use super::hsm::remote_hsm;

const SUSPEND_TO_RAM: u32 = 0x0;

/// Implementation of SBI System Suspend Extension extension.
pub(crate) struct SbiSuspend;

impl rustsbi::Susp for SbiSuspend {
    fn system_suspend(&self, sleep_type: u32, resume_addr: usize, opaque: usize) -> SbiRet {
        if sleep_type != SUSPEND_TO_RAM {
            return SbiRet::invalid_param();
        }

        let prev_mode = mstatus::read().mpp();
        if prev_mode != mstatus::MPP::Supervisor && prev_mode != mstatus::MPP::User {
            return SbiRet::failed();
        }

        // Check if all harts except the current hart are stopped
        let hart_enable_map = if let Some(hart_enable_map) = crate::platform::cpu_enabled() {
            hart_enable_map
        } else {
            return SbiRet::failed();
        };
        for (hartid, hart_enable) in hart_enable_map.iter().enumerate() {
            if *hart_enable && hartid != current_hartid() {
                match remote_hsm(hartid) {
                    Some(remote) => {
                        if remote.get_status() != STOPPED {
                            return SbiRet::denied();
                        }
                    }
                    None => return SbiRet::failed(),
                }
            }
        }

        // TODO: The validity of `resume_addr` should be checked.
        // If it is invalid, `SBI_ERR_INVALID_ADDRESS` should be returned.

        // SpacemiT K3: run the vendor suspend sequence (IMSIC state
        // save/vote/wfi/restore) through the platform hooks, mirroring
        // OpenSBI `__rpmi_hsm_suspend_pre` / `__rpmi_hsm_suspend`
        // (k3_corepm.c L621-795; research doc §5.2 issue ②).
        if crate::platform::IS_K3_PLATFORM.load(Ordering::Acquire) {
            let hartid = current_hartid();
            if !spacemit_k3::suspend_pre(hartid) {
                // Interrupts pending: do not enter low power mode, and do
                // not send 'suspend' to the RCPU (k3_corepm.c L670-674).
                return SbiRet::failed();
            }
            let mut state = spacemit_k3::ImsicConfig::default();
            unsafe {
                spacemit_k3::suspend(hartid, sleep_type, &mut state);
            }
            return SbiRet::success(0);
        }

        match crate::sbi::hsm() {
            Some(hsm) => hsm.hart_suspend(NON_RETENTIVE, resume_addr, opaque),
            None => SbiRet::not_supported(),
        }
    }
}
