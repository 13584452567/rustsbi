//! HSM extension; `remote_hsm` indexes the per-hart trap stack array
//! (`ROOT_STACK`), which stays a `static mut`.
#![allow(static_mut_refs)]

use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    sync::atomic::{AtomicUsize, Ordering},
};
use riscv::register::mstatus::MPP;
use rustsbi::{SbiRet, spec::hsm::hart_state};

use crate::riscv::current_hartid;
use crate::sbi::hart_context::NextStage;
use crate::sbi::trap_stack::ROOT_STACK;
use crate::sbi::trap_stack::hart_context_mut;

use super::{trap::boot::boot, trap_stack::hart_context};

/// Special state indicating a hart is in the process of starting.
const HART_STATE_START_PENDING_EXT: usize = usize::MAX;

type HsmState = AtomicUsize;

/// Cell for managing hart state and shared data between harts.
pub(crate) struct HsmCell<T> {
    status: HsmState,
    inner: UnsafeCell<Option<T>>,
}

impl<T> HsmCell<T> {
    /// Creates a new HsmCell with STOPPED state and no inner data.
    pub const fn new() -> Self {
        Self {
            status: HsmState::new(hart_state::STOPPED),
            inner: UnsafeCell::new(None),
        }
    }

    /// Gets a local view of this cell for the current hart.
    ///
    /// # Safety
    ///
    /// Caller must ensure this cell belongs to the current hart.
    #[inline]
    pub unsafe fn local(&self) -> LocalHsmCell<'_, T> {
        LocalHsmCell(self)
    }

    /// Gets a remote view of this cell for accessing from other harts.
    #[inline]
    pub fn remote(&self) -> RemoteHsmCell<'_, T> {
        RemoteHsmCell(self)
    }
}

/// View of HsmCell for operations on the current hart.
pub struct LocalHsmCell<'a, T>(&'a HsmCell<T>);

/// View of HsmCell for operations from other harts.
pub struct RemoteHsmCell<'a, T>(&'a HsmCell<T>);

// Mark HsmCell as safe to share between threads
unsafe impl<T: Send> Sync for HsmCell<T> {}
unsafe impl<T: Send> Send for HsmCell<T> {}

impl<T> LocalHsmCell<'_, T> {
    /// Attempts to transition hart from START_PENDING to STARTED state.
    ///
    /// Returns inner data if successful, otherwise returns current state.
    #[inline]
    pub fn start(&self) -> Result<T, usize> {
        loop {
            match self.0.status.compare_exchange(
                hart_state::START_PENDING,
                hart_state::STARTED,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ) {
                Ok(_) => break Ok(unsafe { (*self.0.inner.get()).take().unwrap() }),
                Err(HART_STATE_START_PENDING_EXT) => spin_loop(),
                Err(s) => break Err(s),
            }
        }
    }

    /// Transitions hart to STOPPED state.
    #[allow(unused)]
    #[inline]
    pub fn stop(&self) {
        self.0.status.store(hart_state::STOPPED, Ordering::Release)
    }

    /// Transitions hart to SUSPENDED state.
    #[allow(unused)]
    #[inline]
    pub fn suspend(&self) {
        self.0
            .status
            .store(hart_state::SUSPENDED, Ordering::Relaxed)
    }

    /// Transitions hart to STARTED state.
    #[allow(unused)]
    #[inline]
    pub fn resume(&self) {
        self.0.status.store(hart_state::STARTED, Ordering::Relaxed)
    }
    /// Takes a pending NextStage set by a remote `hart_start` and returns to a
    /// clean STOPPED state.
    ///
    /// <para>Used by the secondary-hart entry path: a hart PHY-wakened by HSM
    /// `hart_start` enters RustSBI carrying a START_PENDING cell, but the
    /// subsequent per-hart setup (`prepare_for_trap`) re-initializes the HSM
    /// cell to STOPPED. Capturing the pending jump here lets the entry path
    /// preserve the upcoming transfer instead of idling in WFI.</para>
    #[inline]
    pub fn take_pending(&self) -> Option<T> {
        match self.0.status.load(Ordering::Acquire) {
            hart_state::START_PENDING | HART_STATE_START_PENDING_EXT => {
                let pending = unsafe { (*self.0.inner.get()).take() };
                self.0.status.store(hart_state::STOPPED, Ordering::Release);
                pending
            }
            _ => None,
        }
    }

    /// Checks whether the HSM cell currently holds a pending NextStage
    /// (START_PENDING) — i.e. this hart was PHY-woken by an SBI HSM
    /// `hart_start` and has not yet transferred to its supervisor target.
    ///
    /// <para>Non-destructive: unlike [`take_pending`], it does not consume the
    /// pending jump. The boot/entry path uses it to route a woken secondary
    /// hart to the `secondary_hart()` flow instead of mis-electing it as the
    /// boot hart (a2 is garbage/zero in the warm entry, so the generic
    /// BootInfo decode would win the boot-hart race).</para>
    #[inline]
    pub fn has_pending(&self) -> bool {
        matches!(
            self.0.status.load(Ordering::Acquire),
            hart_state::START_PENDING | HART_STATE_START_PENDING_EXT
        )
    }

    /// Restores a previously-captured pending NextStage (START_PENDING + inner).
    #[inline]
    pub fn restore_pending(&self, t: T) {
        unsafe { *self.0.inner.get() = Some(t) };
        self.0.status.store(hart_state::START_PENDING, Ordering::Release);
    }
}

/// Forces every non-boot hart's HSM cell into the fresh STOPPED state so a
/// later SBI HSM `hart_start` can transition it out (STOPPED -> START_PENDING)
/// and PHY-wake the parked hart.
///
/// <para>On the K3 boot chain (SPL -> RustSBI -> U-Boot -> Linux) the SPL hands
/// execution only to the boot hart; secondary harts stay parked in the boot
/// ROM / SPL and never enter an initial idle loop, so their per-hart `HsmCell`
/// in `ROOT_STACK` is stale. The `.bss.stack` region is placed before the
/// `sbi_bss_start..sbi_bss_end` zeroing loop and is never cleared, so those
/// cells hold undefined RAM contents (raw_status != STOPPED). When Linux later
/// issues `sbi_hart_start(hartid)`, the `compare_exchange(STOPPED ->
/// START_PENDING)` fails and RustSBI reports "already_available", so
/// "CPU&lt;N&gt;: failed to start".</para>
///
/// <para>Mirrors OpenSBI, which pre-"starts" every hart so that HSM `hart_start`
/// finds each secondary in STOPPED state and can wake it.</para>
pub(crate) fn init_secondary_hsm_cells(boot_hart: usize) {
    for hart_id in 0..crate::cfg::NUM_HART_MAX {
        if hart_id == boot_hart {
            continue;
        }
        hart_context_mut(hart_id).init();
    }
}

impl<T: core::fmt::Debug> RemoteHsmCell<'_, T> {
    /// Attempts to start a stopped hart by providing startup data.
    ///
    /// Returns true if successful, false if hart was not in STOPPED state.
    #[inline]
    pub fn start(&self, t: T) -> bool {
        if self
            .0
            .status
            .compare_exchange(
                hart_state::STOPPED,
                HART_STATE_START_PENDING_EXT,
                Ordering::Acquire,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            unsafe { *self.0.inner.get() = Some(t) };
            self.0
                .status
                .store(hart_state::START_PENDING, Ordering::Release);
            true
        } else {
            false
        }
    }

    /// Attempts to resume a suspended hart by providing resume data.
    ///
    /// Returns true if successful, false if hart was not in SUSPENDED state.
    #[inline]
    pub fn resume(&self, t: T) -> bool {
        if self
            .0
            .status
            .compare_exchange(
                hart_state::SUSPENDED,
                HART_STATE_START_PENDING_EXT,
                Ordering::Acquire,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            unsafe { *self.0.inner.get() = Some(t) };
            self.0
                .status
                .store(hart_state::START_PENDING, Ordering::Release);
            true
        } else {
            false
        }
    }

    /// Gets the current state of the hart.
    #[allow(unused)]
    #[inline]
    pub fn get_status(&self) -> usize {
        match self.0.status.load(Ordering::Relaxed) {
            HART_STATE_START_PENDING_EXT => hart_state::START_PENDING,
            normal => normal,
        }
    }

    /// Reads the raw (unmapped) status word. `HART_STATE_START_PENDING_EXT`
    /// is `usize::MAX`; any other value outside the SBI hart-state enum
    /// (0=STOPPED, 1=STARTED, 2=START_PENDING, 3=STOP_PENDING, 4=SUSPENDED)
    /// indicates the cell was never initialized via `HartContext::init()`
    /// (e.g. a stale value in the `.bss.stack` region that the boot-time
    /// `sbi_bss_start..sbi_bss_end` zeroing loop does not cover).
    #[allow(unused)]
    #[inline]
    pub fn raw_status(&self) -> usize {
        self.0.status.load(Ordering::Relaxed)
    }

    /// Checks if hart can receive IPIs (must be STARTED or SUSPENDED).
    #[allow(unused)]
    #[inline]
    pub fn allow_ipi(&self) -> bool {
        matches!(
            self.0.status.load(Ordering::Relaxed),
            hart_state::STARTED | hart_state::SUSPENDED
        )
    }
}

/// Gets the local HSM cell for the current hart.
pub(crate) fn local_hsm() -> LocalHsmCell<'static, NextStage> {
    unsafe { hart_context(current_hartid()).hsm.local() }
}

/// Returns a remote-capable view of the current hart's HSM cell.
pub(crate) fn hart_hsm() -> RemoteHsmCell<'static, NextStage> {
    hart_context(current_hartid()).hsm.remote()
}

/// Gets a remote view of any hart's HSM cell.
#[allow(unused)]
pub(crate) fn remote_hsm(hart_id: usize) -> Option<RemoteHsmCell<'static, NextStage>> {
    unsafe {
        ROOT_STACK
            .get_mut(hart_id)
            .map(|x| x.hart_context().hsm.remote())
    }
}

/// Implementation of SBI HSM (Hart State Management) extension.
pub(crate) struct SbiHsm;

impl rustsbi::Hsm for SbiHsm {
    /// Starts execution on a stopped hart.
    fn hart_start(&self, hartid: usize, start_addr: usize, opaque: usize) -> SbiRet {
        let hart_enable = crate::platform::cpu_enabled().unwrap();
        let enabled = hart_enable.get(hartid).copied().unwrap_or(false);
        if !enabled {
            warn!(
                "HSM hart_start: hart {} rejected (not in enabled-CPU table)",
                hartid
            );
            return SbiRet::invalid_param();
        }

        match remote_hsm(hartid) {
            Some(remote) => {
                if remote.start(NextStage {
                    start_addr,
                    opaque,
                    next_mode: MPP::Supervisor,
                }) {
                    // Platform wakeup hook (issue #237 1-d): on the SpacemiT
                    // K3 a stopped hart may be PMU-powered-down, in which case
                    // the MSIP interrupt alone cannot rouse it -?assert the
                    // hart's PMU wakeup register first, then raise MSIP.
                    crate::platform::wakeup_hart(hartid);
                    // OpenSBI on the K3 wakes the hart through the PMU
                    // wakeup register only (fdt_hsm_rpmi.c rpmi_hsm_start ->
                    // spacemit_wakeup_core); no software interrupt is raised
                    // because the woken hart starts at its RVBADDR warm
                    // entry, not an interrupt handler.
                    if !crate::platform::is_k3() {
                        crate::sbi::ipi().unwrap().set_msip(hartid);
                    }
                    SbiRet::success(0)
                } else {
                    warn!(
                        "HSM hart_start: hart {} not in STOPPED state (raw_status=0x{:x}, already_available)",
                        hartid,
                        remote.raw_status()
                    );
                    SbiRet::already_available()
                }
            }
            None => {
                warn!("HSM hart_start: hart {} has no HSM cell", hartid);
                SbiRet::invalid_param()
            }
        }
    }

    /// Stops execution on the current hart.
    #[inline]
    fn hart_stop(&self) -> SbiRet {
        local_hsm().stop();
        if crate::platform::is_k3() {
            // OpenSBI K3 hart_stop runs __rpmi_shutdown_process: vote the
            // core/cluster down, disable caches/snoop and park in wfi; the
            // hart only re-enters through its RVBADDR warm entry.
            crate::riscv::spacemit_k3::shutdown_process(current_hartid());
        }
        unsafe {
            riscv::register::mie::clear_msoft();
        }
        riscv::asm::wfi();
        SbiRet::success(0)
    }

    /// Gets the current state of a hart.
    #[inline]
    fn hart_get_status(&self, hartid: usize) -> SbiRet {
        let hart_enable = crate::platform::cpu_enabled().unwrap();
        let enabled = hart_enable.get(hartid).copied().unwrap_or(false);
        if !enabled {
            return SbiRet::invalid_param();
        }

        match remote_hsm(hartid) {
            Some(remote) => SbiRet::success(remote.get_status()),
            None => SbiRet::invalid_param(),
        }
    }

    /// Suspends execution on the current hart.
    fn hart_suspend(&self, suspend_type: u32, resume_addr: usize, opaque: usize) -> SbiRet {
        use rustsbi::spec::hsm::suspend_type::{NON_RETENTIVE, RETENTIVE};

        if !matches!(suspend_type, NON_RETENTIVE | RETENTIVE) {
            return SbiRet::invalid_param();
        }

        crate::sbi::trap::handler::msoft_ipi_handler();
        crate::sbi::ipi().unwrap().clear_msip(current_hartid());
        unsafe {
            riscv::register::mie::set_msoft();
        }
        local_hsm().suspend();
        riscv::asm::wfi();
        crate::sbi::trap::handler::msoft_ipi_handler();

        match suspend_type {
            RETENTIVE => {
                local_hsm().resume();
                return SbiRet::success(0);
            }
            NON_RETENTIVE => return self.hart_resume(current_hartid(), resume_addr, opaque),
            _ => return SbiRet::invalid_param(),
        }
    }
}

impl SbiHsm {
    // non retentive resume
    fn hart_resume(&self, hartid: usize, resume_addr: usize, opaque: usize) -> SbiRet {
        match remote_hsm(hartid) {
            Some(remote) => {
                if remote.resume(NextStage {
                    start_addr: resume_addr,
                    opaque,
                    next_mode: MPP::Supervisor,
                }) {
                    // reset the hart local context to prevent the hart context from being polluted
                    hart_context_mut(hartid).reset();
                    // boot resume hart from resume addr
                    unsafe {
                        boot();
                    }
                } else {
                    SbiRet::failed()
                }
            }
            None => SbiRet::failed(),
        }
    }
}
