use crate::riscv::current_hartid;
use crate::sbi::features::{Extension, hart_extension_probe};
use crate::sbi::hsm::local_hsm;
use crate::sbi::ipi;
use crate::sbi::trap_stack;
use core::arch::naked_asm;
use riscv::register::{mie, mstatus, satp, sstatus};

/// Boot Function.
/// After boot, this flow will never back again,
/// so we can store a0, a1 and mepc only.
#[unsafe(naked)]
pub unsafe extern "C" fn boot() -> ! {
    naked_asm!(
        ".align 2",
        // Reset hart local stack
        "call    {locate_stack}",
        "csrw    mscratch, sp",
        // Allocate stack space
        "addi   sp, sp, -3*8",
        // Call handler with context pointer
        "mv     a0, sp",
        "call   {boot_handler}",
        // Restore mepc
        "ld     t0, 0*8(sp)
        li      t1, 1
        csrw    0x5db, t1
        csrr    t1, 0x7c1
        ori     t1, t1, 1
        csrw    0x7c1, t1
        csrw    mepc, t0",
        // Restore registers
        "ld      a0, 1*8(sp)",
        "ld      a1, 2*8(sp)",
        // Restore stack pointer
        "add     sp, sp, 3*8",
        // Switch stacks back
        "csrrw  sp, mscratch, sp",
        // Return from machine mode
        "mret",
        locate_stack = sym trap_stack::locate,
        boot_handler = sym boot_handler,
    )
}

pub extern "C" fn boot_handler(ctx: &mut BootContext) {
    #[inline(always)]
    fn boot(ctx: &mut BootContext, start_addr: usize, opaque: usize) {
        unsafe {
            // stvec BASE is four-byte aligned; HSM entry points may only be two-byte aligned.
            if start_addr & 0x3 == 0 {
                core::arch::asm!(
                    "csrw stvec, {start_addr}",
                    start_addr = in(reg) start_addr,
                    options(nomem),
                );
            }
            core::arch::asm!("csrw sscratch, zero", "csrw sie, zero", options(nomem),);
            sstatus::clear_sie();
            satp::write(satp::Satp::from_bits(0));
        }
        // DEBUG: dump the first bytes at the jump target so we can verify
        // whether SPL actually loaded U-Boot there (garbage/0xff => the
        // load stage is the problem, not the SBI jump).
        let target = unsafe { core::slice::from_raw_parts(start_addr as *const u8, 32) };
        info!("Jump target @ 0x{:x}: {:02x?}", start_addr, target);
        ctx.a0 = current_hartid();
        ctx.a1 = opaque;
        ctx.mepc = start_addr;
    }

    match local_hsm().start() {
        Ok(next_stage) => {
            ipi::claim_ipi();
            unsafe {
                // Align with OpenSBI sbi_hart_switch_mode: it sets
                // MSTATUS_MPIE=0 before mret, so the next stage runs with
                // MIE=0 (M-mode interrupts disabled). RustSBI previously
                // called set_mpie() here, which left MIE=1 after mret and
                // allowed M-level interrupts (e.g. a pending MSoft IPI) to
                // fire during U-Boot's early boot. Clear the MPIE bit
                // (mstatus bit 7) instead.
                core::arch::asm!(
                    "csrc mstatus, {}",
                    in(reg) 1usize << 7,
                    options(nomem, preserves_flags),
                );
                mstatus::set_mpp(next_stage.next_mode);
                if !hart_extension_probe(current_hartid(), Extension::Sstc) {
                    mie::set_mtimer();
                }
            }
            boot(ctx, next_stage.start_addr, next_stage.opaque);
        }
        Err(rustsbi::spec::hsm::HART_STOP) => {
            ipi::claim_ipi();
            unsafe {
                mie::set_msoft();
            }
            riscv::asm::wfi();
        }
        _ => {
            unreachable!("Boot stage hsm should be start or stop.");
        }
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct BootContext {
    pub mepc: usize,
    pub a0: usize,
    pub a1: usize,
}
