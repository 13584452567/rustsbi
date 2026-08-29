#![feature(alloc_error_handler)]
#![feature(fn_align)]
#![no_std]
#![no_main]

extern crate alloc;
#[macro_use]
extern crate log;
#[macro_use]
mod macros;

mod cfg;
mod devicetree;
mod fail;
mod firmware;
mod platform;
mod riscv;
mod rpmi;
mod sbi;

use crate::firmware::BootInfo;
use crate::riscv::current_hartid;
use crate::sbi::features::{
    check_privilege, detect_hart_features, hart_mhpm_mask, hart_privileged_version,
};
use crate::sbi::heap;
use crate::sbi::hsm::hart_hsm;
use crate::sbi::ipi;
use crate::sbi::trap_stack;
use ::riscv::register::mstatus::MPP;
use rustsbi_prototyper_macros::entry;

#[entry]
fn main(boot: BootInfo) {
    // A hart PHY-woken by SBI HSM `hart_start` enters through the warm entry
    // (`_start_warm_k3`) with a2 = 0, so the generic BootInfo decode would
    // mis-elect it as the boot hart: `is_work_hart(0)` fails to read the
    // DynamicInfo and wins the boot-hart race. Detect the pending HSM wake
    // (the jump target placed by the boot hart) and route such a hart to the
    // secondary flow regardless of the decoded `is_boot`.
    let woken = crate::sbi::hsm::local_hsm().has_pending();
    info!(
        "hart {} entered RustSBI main (is_boot={}, woken={})",
        current_hartid(),
        boot.is_boot_hart(),
        woken
    );
    if boot.is_boot_hart() && !woken {
        boot_hart(&boot);
    } else {
        secondary_hart(&boot);
    }
}

fn boot_hart(boot: &BootInfo) {
    heap::init();
    platform::init_board(boot.fdt_address());

    let mem = platform::memory_range();
    firmware::set_pmp(&mem);
    firmware::log_pmp_cfg(&mem);

    let hart_id = current_hartid();
    info!("{:<30}: {}", "Boot HART ID", hart_id);

    detect_hart_features();
    trap_stack::prepare_for_trap();
    log_hart_capabilities(hart_id);

    let mut next = boot.next_stage();
    check_privilege(next.next_mode);

    platform::refresh_cpu_features();
    // DEBUG[A/B-EXPERIMENT]: pass the ORIGINAL un-patched DTB to U-Boot
    // instead of the RustSBI-patched copy. If U-Boot banner appears, the
    // patch (reserved-memory + AIA NOP) breaks U-Boot's early FDT handling.
    next.opaque = match core::option_env!("RUSTSBI_PASSTHRU_DTB") {
        Some(_) => boot.fdt_address(),
        None => firmware::patch_device_tree(boot.fdt_address()),
    };
    info!(
        "Redirecting hart {} to {:#016x} in {:?} mode.",
        hart_id, next.start_addr, next.next_mode
    );
    hart_hsm().start(next);
    // Secondary harts are parked by SPL and never enter RustSBI to
    // self-initialize; clear their HSM cells so HSM `hart_start` finds every
    // hart in STOPPED and can wake it for smp bring-up.
    crate::sbi::hsm::init_secondary_hsm_cells(hart_id);

    enable_supervisor_services();
}

fn secondary_hart(boot: &BootInfo) {
    // A hart PHY-wakened by HSM `hart_start` enters RustSBI carrying a
    // START_PENDING HSM cell (its NextStage) placed by the boot hart.
    // `prepare_for_trap()` below re-initializes that cell to STOPPED, so
    // capture the pending jump first and restore it afterwards; otherwise a
    // just-woken hart would idle in WFI instead of transferring.
    let pending = crate::sbi::hsm::local_hsm().take_pending();
    info!(
        "hart {} secondary pending={}",
        current_hartid(),
        pending.is_some()
    );
    let woken = pending.is_some();

    detect_hart_features();
    trap_stack::prepare_for_trap();

    if let Some(next) = pending {
        crate::sbi::hsm::local_hsm().restore_pending(next);
    }

    // Enable this hart's core snoop BEFORE spinning on the boot-ready flag,
    // so a cold-booted secondary observes the boot hart's writes (READY,
    // delegate/trap setup) coherently. OpenSBI enables snoop per-hart via
    // cold_boot_allowed before any cross-hart wait.
    platform::secondary_hart_init();
    platform::wait_until_ready();
    info!(
        "hart {} secondary past prepare (woken={})",
        current_hartid(),
        woken
    );
    platform::secondary_hart_init();
    firmware::set_pmp(&platform::memory_range());

    if woken {
        // Woken by HSM `hart_start`: the jump target/opaque come from the
        // pending NextStage (supervisor mode), not from the a2 DynamicInfo,
        // which for a PHY-woken hart is garbage.
        check_privilege(MPP::Supervisor);
    } else {
        let next = boot.next_stage();
        check_privilege(next.next_mode);
    }

    // Re-publish the enabled-CPU table: the boot hart ran
    // `refresh_cpu_features()` right after its own privilege check, before
    // this secondary hart had performed `check_privilege`, so this hart's
    // `CPU_PRIVILEGED_ENABLED` flag was still false and the table was
    // overwritten with `false` for all secondary hart IDs. Without this
    // refresh, the SBI HSM `hart_start` returns `invalid_param` for every
    // secondary hart and Linux reports "CPU1..15: failed to start".
    platform::refresh_cpu_features();

    enable_supervisor_services();
}

fn enable_supervisor_services() {
    ipi::clear_all();
    platform::aia::per_hart_init();
    sbi::features::configure_delegation_and_trap();
}

fn log_hart_capabilities(hart_id: usize) {
    info!(
        "{:<30}: {:?}",
        "Boot HART Privileged Version:",
        hart_privileged_version(hart_id)
    );
    info!(
        "{:<30}: {:#08x}",
        "Boot HART MHPM Mask:",
        hart_mhpm_mask(hart_id)
    );
}
