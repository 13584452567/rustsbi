use ::riscv::register::mstatus::MPP;
use riscv::register::misa;
use seq_macro::seq;
use serde_device_tree::buildin::NodeSeq;

use core::arch::asm;
use core::sync::atomic::Ordering;

use crate::fail;
use crate::platform::CPU_PRIVILEGED_ENABLED;
use crate::platform::aia::is_aia_active;
use crate::riscv::csr::*;
use crate::riscv::current_hartid;
use crate::sbi::early_trap::{TrapInfo, csr_read_allow, csr_write_allow};
use crate::sbi::trap_stack::{hart_context, hart_context_mut};

use super::early_trap::csr_swap;

use riscv::register::{medeleg, mtvec};

pub struct HartFeatures {
    extensions: [bool; Extension::COUNT],
    privileged_version: PrivilegedVersion,
    mhpm_mask: u32,
    mhpm_bits: u32,
}

impl HartFeatures {
    pub const fn privileged_version(&self) -> PrivilegedVersion {
        self.privileged_version
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PrivilegedVersion {
    Unknown = 0,
    Version1_10 = 1,
    Version1_11 = 2,
    Version1_12 = 3,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Extension {
    Sstc = 0,
    Hypervisor = 1,
    Smaia = 2,
    // Remember to increment `Extension::COUNT` while implementing new extensions.
}

impl Extension {
    pub const COUNT: usize = 3;

    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::Sstc => "sstc",
            Self::Hypervisor => "h",
            Self::Smaia => "smaia", // TODO verify with DTB standard
        }
    }

    #[inline]
    pub const fn index(&self) -> usize {
        *self as usize
    }

    pub fn iter() -> impl Iterator<Item = Self> {
        [Self::Sstc, Self::Hypervisor, Self::Smaia].into_iter()
    }
}

/// Probes if a specific extension is supported for the given hart.
#[inline]
pub fn hart_extension_probe(hart_id: usize, ext: Extension) -> bool {
    hart_context(hart_id).features.extensions[ext.index()]
}

/// Gets the privileged version for the given hart.
#[inline]
pub fn hart_privileged_version(hart_id: usize) -> PrivilegedVersion {
    hart_context(hart_id).features.privileged_version
}

/// Gets the MHPM mask for the given hart.
#[inline]
pub fn hart_mhpm_mask(hart_id: usize) -> u32 {
    hart_context(hart_id).features.mhpm_mask
}

/// Detects RISC-V extensions from the device tree for all harts.
#[cfg(not(feature = "nemu"))]
pub fn extension_detection(cpus: &NodeSeq) {
    use crate::devicetree::Cpu;

    for cpu_iter in cpus.iter() {
        let cpu_data = cpu_iter.deserialize::<Cpu>();
        let hart_id = cpu_data.reg.iter().next().unwrap().0.start;
        let mut extensions = [false; Extension::COUNT];

        for ext in Extension::iter() {
            let ext_index = ext.index();
            let ext_name = ext.as_str();

            let dt_supported = check_extension_in_device_tree(ext_name, &cpu_data);
            extensions[ext_index] = match ext {
                Extension::Hypervisor if hart_id == current_hartid() => {
                    misa::read().has_extension('H')
                }
                // SpacemiT K3: `riscv,isa-extensions` in the DTB declares
                // "ssaia" but omits "smaia", even though the hardware does
                // implement M-level AIA (OpenSBI spacemit_k3 accesses
                // CSR_MIREG/CSR_SIREG). Fall back to a CSR probe so the
                // IMSIC-backed AIA path is not wrongly rejected.
                Extension::Smaia => dt_supported || probe_smaia_csr(),
                _ => dt_supported,
            };
        }

        hart_context_mut(hart_id).features.extensions = extensions;
    }
}

fn check_extension_in_device_tree(ext: &str, cpu: &crate::devicetree::Cpu) -> bool {
    // Check isa-extensions first (preferred, list of strings)
    if let Some(isa_exts) = &cpu.isa_extensions {
        return isa_exts.iter().any(|e| e == ext);
    }
    cpu.isa
        .iter()
        .next()
        .and_then(|isa| isa.iter().next())
        .map(|isa| {
            isa.split('_')
                .any(|part| part == ext || (ext.len() == 1 && part.contains(ext)))
        })
        .unwrap_or(false)
}

/// Probes whether M-level AIA (Smaia) is implemented by trying to read the
/// `miselect` CSR (0x350). SpacemiT K3's DTB omits "smaia" from
/// `riscv,isa-extensions` even though the hardware implements it (OpenSBI
/// spacemit_k3 accesses CSR_MIREG/CSR_SIREG), so the DT check alone would
/// wrongly reject the IMSIC-backed AIA path on K3.
fn probe_smaia_csr() -> bool {
    let mut trap_info = TrapInfo::default();
    unsafe {
        // miselect (0x350) is an M-level AIA selector CSR. If it does not
        // exist, the expected-trap handler records mcause != usize::MAX.
        csr_read_allow::<0x350>(&mut trap_info);
        trap_info.mcause == usize::MAX
    }
}

fn privileged_version_detection() {
    let mut current_priv_ver = PrivilegedVersion::Unknown;
    {
        if has_csr!(CSR_MCOUNTEREN) {
            current_priv_ver = PrivilegedVersion::Version1_10;
            if has_csr!(CSR_MCOUNTINHIBIT) {
                current_priv_ver = PrivilegedVersion::Version1_11;
                if has_csr!(CSR_MENVCFG) {
                    current_priv_ver = PrivilegedVersion::Version1_12;
                }
            }
        }
    }
    hart_context_mut(current_hartid())
        .features
        .privileged_version = current_priv_ver;
}

fn mhpm_detection() {
    // The standard specifies that mcycle,minstret,mtime must be implemented
    let mut current_mhpm_mask: u32 = 0b111;
    let mut trap_info: TrapInfo = TrapInfo::default();

    fn check_mhpm_csr<const CSR_NUM: u16>(trap_info: *mut TrapInfo, mhpm_mask: &mut u32) {
        unsafe {
            let old_value = csr_read_allow::<CSR_NUM>(trap_info);
            if (*trap_info).mcause == usize::MAX {
                csr_write_allow::<CSR_NUM>(trap_info, 1);
                if (*trap_info).mcause == usize::MAX && csr_swap::<CSR_NUM>(old_value) == 1 {
                    (*mhpm_mask) |= 1 << (CSR_NUM - CSR_MCYCLE);
                }
            }
        }
    }

    macro_rules! m_check_mhpm_csr {
        ($csr_num:expr, $trap_info:expr, $value:expr) => {
            check_mhpm_csr::<$csr_num>($trap_info, $value)
        };
    }

    // CSR_MHPMCOUNTER3:   0xb03
    // CSR_MHPMCOUNTER31:  0xb1f
    seq!(csr_num in 0xb03..=0xb1f{
        m_check_mhpm_csr!(csr_num, &mut trap_info, &mut current_mhpm_mask);
    });

    hart_context_mut(current_hartid()).features.mhpm_mask = current_mhpm_mask;
    // TODO: at present, rustsbi prptotyper only supports 64bit.
    hart_context_mut(current_hartid()).features.mhpm_bits = 64;
}

/// Detects the current hart's ISA extensions and privileged version.
pub fn detect_hart_features() {
    privileged_version_detection();
    mhpm_detection();
}

#[cfg(feature = "nemu")]
pub fn init(cpus: &NodeSeq) {
    for hart_id in 0..cpus.len() {
        let mut hart_exts = [false; Extension::COUNT];
        hart_exts[Extension::Sstc.index()] = true;
        hart_context(hart_id).features = HartFeatures {
            extension: hart_exts,
            privileged_version: PrivilegedVersion::Version1_12,
        }
    }
}

/// Checks that this hart supports the requested privilege mode.
///
/// Warns and stops the hart if it does not.
pub fn check_privilege(mpp: MPP) {
    let hart_id = current_hartid();
    match mpp {
        MPP::Supervisor => {
            if !misa::read().has_extension('S') {
                warn!("Hart {} does not support Supervisor mode", hart_id);
                fail::stop();
            }
            CPU_PRIVILEGED_ENABLED[hart_id].store(true, Ordering::Release);
        }
        MPP::User => {
            if !misa::read().has_extension('U') {
                warn!("Hart {} does not support User mode", hart_id);
                fail::stop();
            }
            CPU_PRIVILEGED_ENABLED[hart_id].store(true, Ordering::Release);
        }
        _ => {}
    }
}

/// Returns whether the `mstateen0` CSR is implemented (trap-tolerant probe).
#[inline(always)]
fn has_mstateen0() -> bool {
    has_csr!(CSR_MSTATEEN0)
}

/// Configures per-hart delegation and trap CSRs for supervisor hand-off.
pub fn configure_delegation_and_trap() {
    unsafe {
        // Delegate all interrupts and exceptions to supervisor mode.
        // Mirror OpenSBI sbi_hart.c mstatus_init(): enable FPU (FS) and
        // vector (VS) context for the next (supervisor) stage.
        let mstatus_s = riscv::register::mstatus::read().bits();
        let mut mstatus_set = mstatus_s;
        if riscv::register::misa::read().has_extension('F')
            || riscv::register::misa::read().has_extension('D')
        {
            mstatus_set |= 0b11 << 13; // MSTATUS_FS = 0b11 (full/dirty)
        }
        if riscv::register::misa::read().has_extension('V') {
            mstatus_set |= 0b11 << 9; // MSTATUS_VS = 0b11 (full/dirty)
        }
        if mstatus_set != mstatus_s {
            asm!("csrw mstatus, {}", in(reg) mstatus_set);
        }

        asm!("csrw mideleg,    {}", in(reg) ((1usize << 1) | (1usize << 5) | (1usize << 9))); // SSIP|STIP|SEIP
        asm!("csrw medeleg,    {}", in(reg) !0);
        asm!("csrw mcounteren, {}", in(reg) !0);
        asm!("csrw scounteren, {}", in(reg) 7usize); // CY|TM|IR (OpenSBI mstatus_init)
        // Keep supervisor environment calls and illegal instructions in M-mode.
        medeleg::clear_supervisor_env_call();
        medeleg::clear_load_misaligned();
        medeleg::clear_store_misaligned();
        medeleg::clear_illegal_instruction();
        // Keep load/store access faults in M-mode on K3. The K3 REGISTER_
        // PRESERVATION window (0xd4282000-0xd4283000, PMP NONE) is accessed
        // by U-Boot's early DM probes (reset/clk/qspi/eth); M-mode must
        // service those faults through access_fault_handler, mirroring
        // OpenSBI (medeleg excludes load/store access fault bits). Without
        // this, the faults are delegated to S-mode U-Boot before its gd is
        // initialized and the board silently hangs.
        if crate::platform::IS_K3_PLATFORM.load(Ordering::Acquire) {
            medeleg::clear_load_fault();
            medeleg::clear_store_fault();
        }

        let hart_priv_version = hart_privileged_version(current_hartid());
        if hart_priv_version >= PrivilegedVersion::Version1_11 {
            asm!("csrw mcountinhibit, {}", in(reg) !0b111usize);
        }
        if hart_priv_version >= PrivilegedVersion::Version1_12 {
            // Configure environment features based on available extensions.
            // PBMTE (menvcfg bit 62, Svpbmt) is mandatory: K3's DTB declares
            // "svpbmt", so Linux marks dma-noncoherent device mappings (e.g.
            // AMBA/primecell ioremap) with the PTE PBMT field (bit 62). If
            // menvcfg.PBMTE is 0, the S-mode MMU rejects any PTE with a
            // non-zero PBMT field and raises a load page fault — this is the
            // "Unable to handle kernel paging request" Oops in
            // amba_read_periphid during of_platform_default_populate_init.
            // OpenSBI sets ENVCFG_PBMTE when SVPBMT is present (sbi_hart.c);
            // mirror that here unconditionally (K3 implements Svpbmt).
            if hart_extension_probe(current_hartid(), Extension::Sstc) {
                menvcfg::set_bits(
                    menvcfg::STCE
                        | menvcfg::PBMTE
                        | menvcfg::CBIE_INVALIDATE
                        | menvcfg::CBCFE
                        | menvcfg::CBZE,
                );
            } else {
                menvcfg::set_bits(
                    menvcfg::PBMTE | menvcfg::CBIE_INVALIDATE | menvcfg::CBCFE | menvcfg::CBZE,
                );
            }
            // Mirror OpenSBI sbi_hart.c mstateen setup: unconditionally grant
            // S/HS-mode access to state-enable CSRs (STATEN), context CSRs
            // and henvcfg once mstateen0 exists. Linux 6.18 probes hstateen0
            // (0x60c) early during sdtrig init; without mstateen0.STATEN set,
            // that S-mode access raises illegal instruction and panics the
            // kernel ("Oops - illegal instruction" in
            // sdtrig_percpu_csrs_check). AIA-related bits are added when the
            // IMSIC-backed AIA path is active (K3 implements Smaia).
            let mstateen_present = has_mstateen0();
            if mstateen_present {
                let mut stateen0 = mstateen::STATEN | mstateen::CONTEXT | mstateen::HSENVCFG;
                if is_aia_active() && hart_extension_probe(current_hartid(), Extension::Smaia) {
                    stateen0 |= mstateen::AIA | mstateen::IMSIC | mstateen::SVSLCT;
                }
                mstateen::set_stateen0(stateen0);
            } else {
                warn!("mstateen0: CSR probe failed, NOT configured");
            }
        }
        // Set up trap handling.
        let val = mtvec::Mtvec::new(
            fast_trap::trap_entry as *const () as _,
            mtvec::TrapMode::Direct,
        );
        mtvec::write(val);
    }
}
