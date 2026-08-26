//! Memory-domain PMP region API.
//!
//! Declares protected physical memory regions and programs them into PMP TOR
//! entries. The flag semantics mirror OpenSBI `sbi_domain_root_add_memrange()`:
//! `ENF_PERMISSIONS` denies R/W/X in every mode (locking the PMP entry so even
//! M-mode is blocked — used for the K3 RCPU runtime spaces), while `M_RWX`
//! leaves M-mode unrestricted and only denies S/U (used for the K3
//! REGISTER_PRESERVATION window that M-mode must emulate).

use riscv::register::{Permission, Range, pmpcfg0, pmpcfg2};
use riscv::register::{
    pmpaddr0, pmpaddr1, pmpaddr2, pmpaddr3, pmpaddr4, pmpaddr5, pmpaddr6, pmpaddr7, pmpaddr8,
    pmpaddr9, pmpaddr10, pmpaddr11, pmpaddr12, pmpaddr13, pmpaddr14, pmpaddr15,
};

/// Deny all R/W/X access in all modes; the PMP entry is locked so even M-mode
/// is blocked. Mirrors OpenSBI `SBI_DOMAIN_MEMREGION_ENF_PERMISSIONS`.
pub const ENF_PERMISSIONS: u32 = 1 << 0;

/// M-mode R/W/X, S/U denied; the PMP entry is not locked (M-mode access is
/// what allows M-mode to emulate S-mode accesses, e.g. the K3
/// REGISTER_PRESERVATION window). Mirrors OpenSBI `SBI_DOMAIN_MEMREGION_M_RWX`.
pub const M_RWX: u32 = 1 << 1;

/// A protected physical memory region with access flags.
pub struct DomainRegion {
    /// Physical base address.
    pub base: usize,
    /// Region size in bytes.
    pub size: usize,
    /// Access flags (see [`ENF_PERMISSIONS`], [`M_RWX`]).
    pub flags: u32,
}

impl DomainRegion {
    /// Creates a protected region.
    pub const fn new(base: usize, size: usize, flags: u32) -> Self {
        Self { base, size, flags }
    }

    /// End (exclusive) of the region.
    #[inline]
    pub const fn end(&self) -> usize {
        self.base + self.size
    }
}

/// Programs a run of protected windows inside `[region_start, region_end)`
/// into PMP TOR entries beginning at `first_slot`.
///
/// Emits, per region, one RWX gap entry followed by one window entry (NONE —
/// locked for `ENF_PERMISSIONS`, unlocked for `M_RWX`), then a final RWX tail
/// to `region_end`. Returns the index after the last written entry, or `None`
/// if the entries would exceed the 16-entry PMP table.
pub fn program_windows(
    first_slot: usize,
    region_start: usize,
    region_end: usize,
    windows: &[DomainRegion],
) -> Option<usize> {
    let entries = windows.len() * 2 + 1;
    if first_slot + entries > 16 {
        return None;
    }
    let mut slot = first_slot;
    let mut cursor = region_start;
    for w in windows {
        // Gap before the window: RWX.
        write_entry(slot, Permission::RWX, false, cursor);
        slot += 1;
        // Window itself: NONE (locked for ENF_PERMISSIONS).
        let locked = w.flags & ENF_PERMISSIONS != 0;
        write_entry(slot, Permission::NONE, locked, w.end());
        slot += 1;
        cursor = w.end();
    }
    // Tail after the last window: RWX.
    write_entry(slot, Permission::RWX, false, region_end);
    Some(slot + 1)
}

/// Writes a single PMP TOR entry at `slot` (0..=15).
///
/// # Panics
///
/// Panics if `slot >= 16`.
pub fn write_entry(slot: usize, perm: Permission, locked: bool, addr: usize) {
    assert!(slot < 16, "PMP slot {slot} out of range");
    let idx = slot & 7;
    // Safety: PMP CSR accesses are volatile register writes; `slot` is
    // bounds-checked above and the addresses are validated by callers.
    unsafe {
        match slot {
            0..=7 => pmpcfg0::set_pmp(idx, Range::TOR, perm, locked),
            8..=15 => pmpcfg2::set_pmp(idx, Range::TOR, perm, locked),
            _ => unreachable!(),
        }
    }
    let addr = addr >> 2;
    unsafe {
        match slot {
            0 => pmpaddr0::write(addr),
            1 => pmpaddr1::write(addr),
            2 => pmpaddr2::write(addr),
            3 => pmpaddr3::write(addr),
            4 => pmpaddr4::write(addr),
            5 => pmpaddr5::write(addr),
            6 => pmpaddr6::write(addr),
            7 => pmpaddr7::write(addr),
            8 => pmpaddr8::write(addr),
            9 => pmpaddr9::write(addr),
            10 => pmpaddr10::write(addr),
            11 => pmpaddr11::write(addr),
            12 => pmpaddr12::write(addr),
            13 => pmpaddr13::write(addr),
            14 => pmpaddr14::write(addr),
            15 => pmpaddr15::write(addr),
            _ => unreachable!(),
        }
    }
}
