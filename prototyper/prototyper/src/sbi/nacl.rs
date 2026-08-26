use rustsbi::SbiRet;
use sbi_spec::binary::SharedPtr;
use sbi_spec::nacl::shmem_size::NATIVE;

/// Nested Acceleration (NACL) extension implementation.
///
/// <para>
/// 实现 SBI 2.0 的 Nested Acceleration（嵌套加速）扩展协议基础设施。SBI 规范
/// 明确说明：当底层平台在硬件中实现了 RISC-V H 扩展时，M 态固件**不应**实现
/// 该扩展——共享内存协议存在的意义是让 L0 hypervisor 与 L1 hypervisor 协作，
/// 减少 L0 仿真 H 扩展（CSR 访问、HFENCE、HLV/HSV 等）时陷入的次数。
/// </para>
///
/// <para>
/// SpacemiT X100 / K3 在硬件中实现了 H 扩展（RVA23S64 强制要求），因此本固件
/// 不仿真 H 扩展，所有 NACL 特性均不可用，扩展通过 base `probe_extension`
/// 正确报告为"不可用"。这里仍完整实现协议处理（特性探测、共享内存校验、同步
/// 桩函数），使基础设施齐备——若未来本固件作为 L0 hypervisor 构建，仅需在
/// [`SbiNacl::probe_feature`] 中按需启用对应特性。
/// </para>
///
/// <para>
/// Implements the SBI 2.0 Nested Acceleration protocol infrastructure. Per
/// the SBI spec, M-mode firmware must NOT implement NACL when the platform
/// implements the RISC-V H-extension in hardware — the protocol exists so an
/// L0 hypervisor can collaborate with an L1 hypervisor to reduce traps taken
/// while emulating the H-extension (CSR accesses, HFENCE, HLV/HSV, ...).
/// </para>
///
/// <para>
/// The SpacemiT X100 / K3 implements H in hardware (required by RVA23S64), so
/// this firmware does not emulate the H-extension, no NACL feature is
/// available, and the extension correctly reports itself as unavailable via
/// the base `probe_extension`. The protocol handlers (feature probing, shared
/// memory validation, sync stubs) are fully implemented so the infrastructure
/// is ready: a future L0-hypervisor build only needs to enable features in
/// [`SbiNacl::probe_feature`].
/// </para>
pub(crate) struct SbiNacl;

impl rustsbi::Nacl for SbiNacl {
    fn probe_feature(&self, _feature_id: u32) -> SbiRet {
        // No nested-acceleration feature is available: the firmware does not
        // emulate the H-extension (hardware H present on the target platform),
        // so there is nothing to accelerate. Returns `SbiRet::success(0)` per
        // the mandatory probe contract (value 0 = feature unavailable).
        SbiRet::success(0)
    }

    fn set_shmem(&self, shmem: SharedPtr<[u8; NATIVE]>, flags: usize) -> SbiRet {
        // With no feature enabled the extension reports unavailable, so this
        // path is not reached through the dispatcher. The handler is kept
        // spec-complete so an L0-hypervisor build only enables features above.
        //
        // <para>The `flags` parameter is reserved and MUST be zero.</para>
        if flags != 0 {
            return SbiRet::invalid_param();
        }

        let lo = shmem.phys_addr_lo();
        let hi = shmem.phys_addr_hi();

        // All-ones shared pointer disables the acceleration features.
        if hi == usize::MAX && lo == usize::MAX {
            return SbiRet::success(0);
        }

        // The physical address MUST be 4096-byte aligned; the upper half of
        // the 128-bit physical address must be zero on RV64.
        if lo & 0xfff != 0 || hi != 0 {
            return SbiRet::invalid_param();
        }

        // The supervisor must be able to write the shared memory: reject
        // addresses inside the SBI firmware image or the K3 PMP-denied
        // windows (spec: `invalid_address` otherwise).
        if !crate::firmware::supervisor_writable(lo, NATIVE) {
            return SbiRet::invalid_address();
        }

        // Zero the shared memory before returning (spec §15.6).
        // Safety: `lo` was validated above (writable, 4096-byte aligned,
        // outside firmware memory); the write is volatile.
        unsafe {
            core::ptr::write_bytes(lo as *mut u8, 0, NATIVE);
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        }

        SbiRet::success(0)
    }

    fn sync_csr(&self, _csr_num: usize) -> SbiRet {
        // SYNC_CSR feature unavailable (no H-extension emulation).
        SbiRet::not_supported()
    }

    fn sync_hfence(&self, _entry_index: usize) -> SbiRet {
        // SYNC_HFENCE feature unavailable (no H-extension emulation).
        SbiRet::not_supported()
    }

    fn sync_sret(&self) -> SbiRet {
        // SYNC_SRET feature unavailable (no H-extension emulation).
        SbiRet::not_supported()
    }

    fn _rustsbi_probe(&self) -> usize {
        // No NACL feature is available → the base `probe_extension(NACL)`
        // reports the extension as unavailable (value 0).
        sbi_spec::base::UNAVAILABLE_EXTENSION
    }
}
