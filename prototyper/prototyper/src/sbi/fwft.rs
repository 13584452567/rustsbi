use core::arch::asm;

use rustsbi::SbiRet;
use sbi_spec::fwft::feature_type;

/// Misaligned load/store exception bits in `medeleg` (CAUSE_MISALIGNED_LOAD
/// = 4, CAUSE_MISALIGNED_STORE = 6), mirroring OpenSBI
/// `lib/sbi/sbi_fwft.c` `MIS_DELEG`.
const MIS_DELEG: usize = (1 << 4) | (1 << 6);

/// `menvcfg` CSR number (machine environment configuration).
const CSR_MENVCFG: u16 = 0x30A;

/// `menvcfg` bit fields (see `include/sbi/riscv_encoding.h` `ENVCFG_*`).
const ENVCFG_LPE: usize = 1 << 0; // Landing pad (Zicfilp)
const ENVCFG_DTE: usize = 1 << 1; // Double trap (Smdbltrp)
const ENVCFG_ADUE: usize = 1 << 5; // PTE A/D hardware updating (SVADU)
const ENVCFG_SSE: usize = 1 << 8; // Shadow stack (Zicfiss)
const ENVCFG_PMM_SHIFT: usize = 9; // Pointer masking tag length (Smnpm)
const ENVCFG_PMM: usize = 0b11 << ENVCFG_PMM_SHIFT;

/// Read a CSR.
///
/// `CSR` must be a compile-time constant: the RISC-V `csrr` encoding requires
/// the CSR field to be an immediate, not a register.
#[inline]
unsafe fn csr_read<const CSR: u16>() -> usize {
    let r: usize;
    unsafe {
        asm!("csrr {r}, {csr}", r = out(reg) r, csr = const CSR, options(nomem));
    }
    r
}

/// Write a CSR.
///
/// `CSR` must be a compile-time constant: the RISC-V `csrw` encoding requires
/// the CSR field to be an immediate, not a register.
#[inline]
unsafe fn csr_write<const CSR: u16>(val: usize) {
    unsafe {
        asm!("csrw {csr}, {val}", csr = const CSR, val = in(reg) val, options(nomem));
    }
}

/// Implementation of SBI Firmware Features (FWFT) extension.
///
/// Mirrors OpenSBI `lib/sbi/sbi_fwft.c`:
///
/// - `MISALIGNED_EXC_DELEG` is supported when the hart implements the
///   supervisor mode (`misa.S`); setting it toggles the misaligned load and
///   store bits of `medeleg`, so the S-mode trap handler (rather than the
///   M-mode emulator) receives misaligned accesses.
/// - `LANDING_PAD`, `SHADOW_STACK`, `DOUBLE_TRAP`, `PTE_AD_HW_UPDATING` and
///   `POINTER_MASKING_PMLEN` are backed by the corresponding bits of the
///   `menvcfg` CSR (Zicfilp / Zicfiss / Smdbltrp / SVADU / Smnpm). A feature
///   is reported as supported only if the bit can actually be written, i.e.
///   the underlying hardware extension is present (mirroring OpenSBI
///   `fwft_try_to_set_pmm`).
pub(crate) struct SbiFwft;

impl SbiFwft {
    /// Returns whether the hart implements supervisor mode.
    fn has_s_mode() -> bool {
        riscv::register::misa::read().has_extension('S')
    }

    /// Reads the current misaligned delegation state from `medeleg`.
    fn misaligned_delegated() -> bool {
        (riscv::register::medeleg::read().bits() & MIS_DELEG) != 0
    }

    /// Sets or clears the misaligned delegation bits in `medeleg`.
    fn set_misaligned_delegation(value: usize) -> bool {
        let current = riscv::register::medeleg::read().bits();
        let next = match value {
            0 => current & !MIS_DELEG,
            1 => current | MIS_DELEG,
            _ => return false,
        };
        // Safety: writing `medeleg` from M-mode is a plain CSR store; the
        // value is derived from the current register contents.
        unsafe {
            riscv::register::medeleg::write(riscv::register::medeleg::Medeleg::from_bits(next));
        }
        true
    }

    /// Reads the `menvcfg` CSR.
    fn menvcfg_read() -> usize {
        // Safety: `CSR_MENVCFG` is a valid M-mode CSR and reading it has no
        // side effects.
        unsafe { csr_read::<CSR_MENVCFG>() }
    }

    /// Writes the `menvcfg` CSR.
    fn menvcfg_write(value: usize) {
        // Safety: `CSR_MENVCFG` is a valid M-mode CSR; the value is derived
        // from the current register contents (or a probe of it).
        unsafe { csr_write::<CSR_MENVCFG>(value) }
    }

    /// Returns the `menvcfg` bit backing a FWFT feature, if any.
    fn menvcfg_bit(feature_id: usize) -> Option<usize> {
        match feature_id {
            feature_type::LANDING_PAD => Some(ENVCFG_LPE),
            feature_type::SHADOW_STACK => Some(ENVCFG_SSE),
            feature_type::DOUBLE_TRAP => Some(ENVCFG_DTE),
            feature_type::PTE_AD_HW_UPDATING => Some(ENVCFG_ADUE),
            _ => None,
        }
    }

    /// Sets or clears a `menvcfg` bit for a FWFT feature.
    ///
    /// Mirrors OpenSBI `fwft_menvcfg_set_bit` combined with the write-then-
    /// read-back check of `fwft_try_to_set_pmm`: the new value is written and
    /// read back; if the bit did not take effect, the underlying hardware
    /// extension is absent and the feature is not supported.
    fn set_menvcfg_bit(bit: usize, value: usize) -> SbiRet {
        if value > 1 {
            return SbiRet::invalid_param();
        }
        let current = Self::menvcfg_read();
        let next = if value == 1 {
            current | bit
        } else {
            current & !bit
        };
        Self::menvcfg_write(next);
        let read_back = Self::menvcfg_read();
        if (read_back & bit) != (next & bit) {
            return SbiRet::not_supported();
        }
        SbiRet::success(0)
    }

    /// Sets the pointer masking tag length (`PMM` field of `menvcfg`).
    ///
    /// Mirrors OpenSBI `fwft_try_to_set_pmm`: the new `PMM` value is written
    /// and read back; if it did not take effect, the `Smnpm` extension is
    /// absent and the feature is not supported.
    fn set_pmm(value: usize) -> SbiRet {
        if value > 3 {
            return SbiRet::invalid_param();
        }
        let current = Self::menvcfg_read();
        let next = (current & !ENVCFG_PMM) | (value << ENVCFG_PMM_SHIFT);
        Self::menvcfg_write(next);
        let read_back = Self::menvcfg_read();
        if (read_back & ENVCFG_PMM) != (next & ENVCFG_PMM) {
            return SbiRet::not_supported();
        }
        SbiRet::success(0)
    }

    /// Returns whether the hardware implements the given `menvcfg` bits, by
    /// writing them and reading them back (mirroring OpenSBI
    /// `fwft_try_to_set_pmm`). The original value is restored before
    /// returning.
    fn menvcfg_bits_supported(mask: usize) -> bool {
        let current = Self::menvcfg_read();
        Self::menvcfg_write(current | mask);
        let probed = Self::menvcfg_read();
        Self::menvcfg_write(current);
        (probed & mask) != 0
    }
}

impl rustsbi::Fwft for SbiFwft {
    fn set(&self, feature_id: u32, value: usize, flags: usize) -> SbiRet {
        // The LOCK flag is not supported: locked features can never be
        // modified again, which would prevent firmware reconfiguration.
        if flags != 0 {
            return SbiRet::invalid_param();
        }
        match feature_id as usize {
            feature_type::MISALIGNED_EXC_DELEG => {
                if !Self::has_s_mode() {
                    return SbiRet::not_supported();
                }
                if Self::set_misaligned_delegation(value) {
                    SbiRet::success(0)
                } else {
                    SbiRet::invalid_param()
                }
            }
            feature_type::POINTER_MASKING_PMLEN => Self::set_pmm(value),
            _ => match Self::menvcfg_bit(feature_id as usize) {
                Some(bit) => Self::set_menvcfg_bit(bit, value),
                None => SbiRet::not_supported(),
            },
        }
    }

    fn get(&self, feature_id: u32) -> SbiRet {
        match feature_id as usize {
            feature_type::MISALIGNED_EXC_DELEG => {
                if !Self::has_s_mode() {
                    return SbiRet::not_supported();
                }
                SbiRet::success(Self::misaligned_delegated() as usize)
            }
            feature_type::POINTER_MASKING_PMLEN => {
                if !Self::menvcfg_bits_supported(ENVCFG_PMM) {
                    return SbiRet::not_supported();
                }
                let pmm = (Self::menvcfg_read() & ENVCFG_PMM) >> ENVCFG_PMM_SHIFT;
                SbiRet::success(pmm)
            }
            _ => match Self::menvcfg_bit(feature_id as usize) {
                Some(bit) => {
                    if !Self::menvcfg_bits_supported(bit) {
                        return SbiRet::not_supported();
                    }
                    SbiRet::success(((Self::menvcfg_read() & bit) != 0) as usize)
                }
                None => SbiRet::not_supported(),
            },
        }
    }
}
