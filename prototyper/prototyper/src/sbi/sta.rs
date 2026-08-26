use core::sync::atomic::{AtomicUsize, Ordering};

use rustsbi::SbiRet;
use sbi_spec::binary::SharedPtr;

use crate::cfg::NUM_HART_MAX;
use crate::riscv::current_hartid;

/// Steal-time shared memory structure (SBI 2.0 §16).
///
/// <para>
/// 64 字节窃取时间结构：`steal`（u64，纳秒）、`preempted`（u32，自上次读取
/// 后该虚拟 hart 是否被抢占）、`pad`（u32），其余 48 字节保留且必须为零。
/// 小端序。
/// </para>
///
/// <para>
/// The 64-byte steal-time structure: `steal` (u64, nanoseconds), `preempted`
/// (u32, whether the virtual hart was preempted since the last read), `pad`
/// (u32), and 48 reserved bytes that must stay zero. Little-endian.
/// </para>
#[repr(C)]
struct StealTime {
    steal: u64,
    preempted: u32,
    _pad: u32,
    _reserved: [u8; 48],
}

/// Per-hart steal-time shared memory physical base; `0` means "not set".
///
/// <para>
/// 以物理地址形式保存每个 hart 的共享内存基址（`0` 表示未启用）。使用原子
/// 变量使多 hart 并发读取/写入时不需要额外锁。仅当固件真正以 hypervisor /
/// 多域方式运行、需要抢占虚拟 hart 时，才由 M 态写入该共享内存。
/// </para>
///
/// <para>
/// Stores the shared-memory physical base per hart (`0` = disabled). Atomics
/// make concurrent hart access lock-free. The buffer is only written by
/// M-mode when the firmware actually withholds a virtual hart (hypervisor /
/// multi-domain mode).
/// </para>
static STA_SHMEM: [AtomicUsize; NUM_HART_MAX] = [const { AtomicUsize::new(0) }; NUM_HART_MAX];

/// Steal-time Accounting (STA) extension implementation.
///
/// <para>
/// 实现 SBI 2.0 的 Steal-time Accounting（窃取时间统计）扩展。本固件作为
/// 直接引导 Linux / ESOS 的监督程序，从不抢占虚拟 hart，因此记账值恒为零；
/// 但基础设施是完整的：按 hart 维护共享内存状态、校验对齐与可写性、按规范
/// 清零共享内存，并提供 [`SbiSta::report_steal_time`] 记账钩子，供未来
/// hypervisor / 多域模式写入真实数值。
/// </para>
///
/// <para>
/// Implements the SBI 2.0 Steal-time Accounting extension. As a supervisor
/// for directly booted OSes this firmware never withholds virtual harts, so
/// the reported values stay zero; the infrastructure is complete: per-hart
/// shared-memory state, alignment/writability validation, spec-mandated
/// zeroing, and a [`SbiSta::report_steal_time`] accounting hook ready for a
/// future hypervisor / multi-domain mode.
/// </para>
pub(crate) struct SbiSta;

impl SbiSta {
    /// Reports steal-time / preemption information for the given virtual hart.
    ///
    /// <para>
    /// 若该 hart 已通过 `set_shmem` 启用记账，则将 `steal_ns`（纳秒）与
    /// `preempted` 写入共享内存结构。当前固件从不抢占虚拟 hart，调用方应传入
    /// 零值；未来作为 hypervisor 抢占 guest 时可传入真实数值。写入前以 volatile
    /// 保证不会被编译器缓存或重排。
    /// </para>
    ///
    /// <para>
    /// Writes `steal_ns` (nanoseconds) and `preempted` into the shared-memory
    /// structure if accounting is enabled for the hart. Callers pass zero on
    /// this firmware (no virtual hart is ever withheld); a future hypervisor
    /// mode passes real numbers. The write is volatile so it cannot be
    /// cached or reordered by the compiler.
    /// </para>
    #[allow(dead_code)]
    pub(crate) fn report_steal_time(&self, hartid: usize, steal_ns: u64, preempted: bool) {
        let base = STA_SHMEM[hartid].load(Ordering::Acquire);
        if base == 0 {
            return;
        }
        let value = StealTime {
            steal: steal_ns,
            preempted: preempted as u32,
            _pad: 0,
            _reserved: [0; 48],
        };
        // Safety: `base` was validated in `set_shmem` to be a writable,
        // 64-byte-aligned physical address not overlapping firmware memory.
        unsafe {
            core::ptr::write_volatile(base as *mut StealTime, value);
            core::sync::atomic::fence(Ordering::SeqCst);
        }
    }
}

impl rustsbi::Sta for SbiSta {
    fn set_shmem(&self, shmem: SharedPtr<[u8; 64]>, flags: usize) -> SbiRet {
        // <para>The `flags` parameter is reserved and MUST be zero.</para>
        if flags != 0 {
            return SbiRet::invalid_param();
        }

        let lo = shmem.phys_addr_lo();
        let hi = shmem.phys_addr_hi();

        // All-ones shared pointer disables steal-time reporting (spec §16.1).
        if hi == usize::MAX && lo == usize::MAX {
            STA_SHMEM[current_hartid()].store(0, Ordering::Release);
            return SbiRet::success(0);
        }

        // The physical address MUST be 64-byte aligned; the upper half of
        // the 128-bit physical address must be zero on RV64.
        if lo & 0x3f != 0 || hi != 0 {
            return SbiRet::invalid_param();
        }

        // The supervisor must be able to write the shared memory: reject
        // addresses that fall inside the SBI firmware image or the K3
        // PMP-denied windows (spec: `invalid_address` otherwise).
        if !crate::firmware::supervisor_writable(lo, 64) {
            return SbiRet::invalid_address();
        }

        // Zero the 64-byte structure before returning (spec §16.1).
        // Safety: `lo` was validated above (writable, aligned, outside
        // firmware memory); the write is volatile so it cannot be elided.
        unsafe {
            core::ptr::write_bytes(lo as *mut u8, 0, 64);
            core::sync::atomic::fence(Ordering::SeqCst);
        }

        STA_SHMEM[current_hartid()].store(lo, Ordering::Release);
        SbiRet::success(0)
    }
}
