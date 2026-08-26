//! Safe boot entry points over the global platform state.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ops::Range;
use core::sync::atomic::Ordering;

use super::{
    BOARD_INFO, BoardInfo, IS_K1_PLATFORM, IS_K3_PLATFORM, READY, board_info, print_board_info,
    publish_cpu_enabled,
};
use crate::devicetree::{Tree, parse_device_tree};
use crate::fail;
use crate::riscv::spacemit_k1;
use crate::riscv::spacemit_k3;
use crate::sbi;
use crate::sbi::SbiDispatcher;
use crate::sbi::hsm::SbiHsm;
use crate::sbi::rfence::SbiRFence;
use crate::sbi::suspend::SbiSuspend;

/// Initializes the board from the device tree and runs the SoC-specific
/// early initialization.
pub fn init_board(fdt_address: usize) {
    let dtb = parse_device_tree(fdt_address).unwrap_or_else(fail::device_tree_format);
    let dtb = dtb.share();

    let root: serde_device_tree::buildin::Node =
        serde_device_tree::from_raw_mut(&dtb).unwrap_or_else(fail::device_tree_deserialize_root);
    let tree: Tree = root.deserialize();

    let mut board = BoardInfo::new();
    // Get console device, init sbi console and logger.
    board.discover_console(&root);
    let console = sbi::console::init(&board);
    // Get other info that later platform initialization depends on.
    let cpu_list = board.discover_misc(&tree);
    publish_cpu_enabled(cpu_list);
    // Get clint and reset device, init sbi ipi, reset, hsm, rfence and susp extension.
    board.discover_devices(&root);
    let ipi = sbi::ipi::init(&board);
    let hsm = ipi.as_ref().map(|_| SbiHsm);
    let reset = sbi::reset::init(&board);
    let rfence = ipi.as_ref().map(|_| SbiRFence);
    let susp = hsm.as_ref().map(|_| SbiSuspend);
    // Initialize pmu extension
    let pmu = sbi::pmu::init(&root);
    // Initialize firmware features extension
    let fwft = Some(sbi::fwft::SbiFwft);
    // Initialize debug triggers extension
    let dbtr = Some(sbi::dbtr::SbiDbtr);
    // Initialize cppc extension
    let cppc = Some(sbi::cppc::SbiCppc::new());
    // Initialize supervisor software events extension
    let sse = Some(sbi::sse::SbiSse);
    // Initialize message proxy extension (RPMI mailbox backend injected by
    // the platform; see sbi::mpxy::SbiMpxy::set_mailbox)
    let mpxy = Some(sbi::mpxy::SbiMpxy::new());
    // Initialize steal-time accounting extension (per-hart shared memory;
    // reports zero steal time as this SBI never withholds virtual harts)
    let sta = Some(sbi::sta::SbiSta);
    // Initialize nested acceleration extension. The K3 implements the
    // H-extension in hardware, so per SBI spec no NACL feature is available
    // and the extension reports itself unavailable through `probe_extension`.
    let nacl = Some(sbi::nacl::SbiNacl);

    // Publish the SBI extension set before the K1 detect / READY release,
    // so that harts observing `READY` also observe the published dispatcher.
    sbi::SBI_DISPATCHER.call_once(|| {
        SbiDispatcher::new(
            console, ipi, hsm, reset, rfence, susp, pmu, fwft, dbtr, cppc, sse, mpxy, sta, nacl,
        )
    });

    // Inject the RPMI shared-memory mailbox (if the device tree carries a
    // `riscv,rpmi-shmem-mbox` node) into the MPXY extension. Runs after the
    // dispatcher is published so `sbi::mpxy()` is available.
    inject_rpmi_mailbox(&root);

    // Publish the board facts before the K1 detect / READY release, so that
    // harts observing `READY` (Acquire) also observe the published board.
    BOARD_INFO.call_once(move || board);

    // Record K3/K1 platform detection *before* releasing the ready flag, so
    // that secondary harts observing `READY` also observe the flags.
    // K3 is checked first: its model strings ("SpacemiT K3 ...") also match
    // the loose SpacemiT fallback used for K1 detection, so the K1 check
    // runs only when the board is not a K3. This mirrors OpenSBI's
    // platform_override match priority. Both check the root node's
    // `compatible` strings first (OpenSBI's spacemit_k3_mach[] /
    // spacemit_k1_match[] tables), falling back to the model string.
    let k3_platform = match root.get_prop("compatible") {
        Some(prop) => {
            let seq = prop.deserialize::<serde_device_tree::buildin::StrSeq>();
            spacemit_k3::is_k3_platform(&board_info().model, seq.iter())
        }
        None => spacemit_k3::is_k3_platform(&board_info().model, core::iter::empty::<&str>()),
    };
    IS_K3_PLATFORM.store(k3_platform, Ordering::Release);

    if !k3_platform {
        let k1_platform = match root.get_prop("compatible") {
            Some(prop) => {
                let seq = prop.deserialize::<serde_device_tree::buildin::StrSeq>();
                spacemit_k1::is_k1_platform(&board_info().model, seq.iter())
            }
            None => spacemit_k1::is_k1_platform(&board_info().model, core::iter::empty::<&str>()),
        };
        IS_K1_PLATFORM.store(k1_platform, Ordering::Release);
    }

    READY.store(true, Ordering::Release);

    print_board_info();

    // SpacemiT K3 / K1 early initialization. The platform flags were
    // already recorded before the ready flag was released, so they are
    // visible here (and to secondary harts once they observe ready()).
    if IS_K3_PLATFORM.load(Ordering::Acquire) {
        // Configure ML2SETUP for the boot hart
        spacemit_k3::cold_boot_allowed(crate::riscv::current_hartid());

        unsafe {
            // Use the SBI link address as the warmboot entry
            let warmboot_addr = crate::cfg::SBI_LINK_START_ADDRESS as u64;
            spacemit_k3::early_init(true, warmboot_addr);
        }
        info!("SpacemiT K3: early init done (RVBADDR + CCI-550 + A100 park)");
    } else if IS_K1_PLATFORM.load(Ordering::Acquire) {
        // Configure ML2SETUP for the boot hart
        spacemit_k1::cold_boot_allowed(crate::riscv::current_hartid());

        unsafe {
            // Use the SBI link address as the warmboot entry
            let warmboot_addr = crate::cfg::SBI_LINK_START_ADDRESS as u64;
            spacemit_k1::early_init(true, warmboot_addr);
        }
        info!("SpacemiT K1: early init done (MSETUP + CCI-550)");
    }
}

/// Runs the SoC-specific per-hart setup for secondary harts.
pub fn secondary_hart_init() {
    // SpacemiT K3: Configure ML2SETUP for this hart
    if IS_K3_PLATFORM.load(Ordering::Acquire) {
        spacemit_k3::cold_boot_allowed(crate::riscv::current_hartid());
    } else if IS_K1_PLATFORM.load(Ordering::Acquire) {
        // SpacemiT K1: Configure ML2SETUP for this hart
        spacemit_k1::cold_boot_allowed(crate::riscv::current_hartid());
    }
}

/// SpacemiT platform hart-wakeup hook, called from HSM `hart_start`.
///
/// On the K3 a stopped hart may be powered down at the PMU; the software IPI
/// (MSIP) alone cannot rouse it, so the hart's `PMU_CAP_CORE*_WAKEUP`
/// register is asserted first (mirrors OpenSBI `spacemit_wakeup_core`,
/// k3_corepm.c L1057-1113). Writing the wakeup register is benign for a hart
/// that is already running.
pub fn wakeup_hart(hartid: usize) {
    if IS_K3_PLATFORM.load(Ordering::Acquire) {
        spacemit_k3::wakeup_core(hartid);
    }
}

/// Spins until the boot hart has finished platform initialization.
pub fn wait_until_ready() {
    while !READY.load(Ordering::Acquire) {
        core::hint::spin_loop()
    }
}

/// RPMI shared-memory queue slot layout (OpenSBI
/// `include/sbi_utils/mailbox/rpmi_msgprot.h`).
const RPMI_QUEUE_HEAD_SLOT: usize = 0;
const RPMI_QUEUE_TAIL_SLOT: usize = 1;
const RPMI_QUEUE_HEADER_SLOTS: usize = 2;
const RPMI_QUEUE_IDX_A2P_REQ: usize = 0;
const RPMI_QUEUE_IDX_P2A_ACK: usize = 1;
const RPMI_QUEUE_IDX_P2A_REQ: usize = 2;
const RPMI_QUEUE_IDX_A2P_ACK: usize = 3;

/// Discovers the `riscv,rpmi-shmem-mbox` node in the device tree and
/// injects the resulting shared-memory mailbox into the MPXY extension.
///
/// Mirrors OpenSBI `fdt_mailbox_rpmi_shmem.c`: the node carries a
/// `riscv,slot-size` property and one `reg` region per queue (named by
/// `reg-names`), plus a doorbell `db-reg`. Each queue stores its head and
/// tail indices in the first two slots and the message ring after them.
fn inject_rpmi_mailbox(root: &serde_device_tree::buildin::Node) {
    let mut found = false;
    let mut find = |node: &serde_device_tree::buildin::Node,
                    _parent: Option<&serde_device_tree::buildin::Node>| {
        if found {
            return;
        }
        let Some(compatible) = node.get_prop("compatible") else {
            return;
        };
        let seq = compatible.deserialize::<serde_device_tree::buildin::StrSeq>();
        if !seq.iter().any(|s| s == "riscv,rpmi-shmem-mbox") {
            return;
        }
        found = true;

        // Slot size.
        let Some(slot_size) = crate::platform::prop_u32_cells(node, "riscv,slot-size")
            .and_then(|c| c.first().copied())
            .map(|v| v as usize)
        else {
            warn!("rpmi-shmem-mbox: missing riscv,slot-size; skipping");
            return;
        };
        if slot_size < 128 {
            warn!("rpmi-shmem-mbox: slot-size too small; skipping");
            return;
        }

        // reg regions in order (a2p-req, p2a-ack, p2a-req, a2p-ack, db-reg).
        let Some(reg) = node.get_prop("reg") else {
            warn!("rpmi-shmem-mbox: missing reg; skipping");
            return;
        };
        let reg = reg.deserialize::<serde_device_tree::buildin::Reg>();
        let ranges: Vec<_> = reg.iter().map(|r| r.0).collect();
        if ranges.len() < 5 {
            warn!("rpmi-shmem-mbox: expected 4 queues + db-reg; skipping");
            return;
        }

        // Safety: the queue regions are MMIO shared memory owned by the
        // platform; indices and slots are volatile little-endian words.
        let mut queues = core::array::from_fn(|_| unsafe {
            rpmi::SmqQueue::new(
                core::ptr::null(),
                core::ptr::null(),
                core::ptr::null_mut(),
                slot_size,
                0,
            )
        });
        for (i, &idx) in [
            RPMI_QUEUE_IDX_A2P_REQ,
            RPMI_QUEUE_IDX_P2A_ACK,
            RPMI_QUEUE_IDX_P2A_REQ,
            RPMI_QUEUE_IDX_A2P_ACK,
        ]
        .iter()
        .enumerate()
        {
            let base = ranges[idx].start as *mut u8;
            let size = ranges[idx].len();
            let num_slots = (size - RPMI_QUEUE_HEADER_SLOTS * slot_size) / slot_size;
            queues[i] = unsafe {
                rpmi::SmqQueue::new(
                    base.add(RPMI_QUEUE_HEAD_SLOT * slot_size).cast(),
                    base.add(RPMI_QUEUE_TAIL_SLOT * slot_size).cast(),
                    base.add(RPMI_QUEUE_HEADER_SLOTS * slot_size),
                    slot_size,
                    num_slots,
                )
            };
        }
        // Doorbell register (db-reg).
        let doorbell = ranges[4].start as *const rpmi::Le32;
        let mailbox = unsafe { rpmi::RpmiMailbox::new(slot_size, queues, Some(&*doorbell)) };
        // Leak the mailbox so the MPXY and CPPC extensions can share it.
        let mailbox: &'static rpmi::RpmiMailbox = Box::leak(Box::new(mailbox));

        if let Some(mpxy) = sbi::mpxy() {
            mpxy.set_mailbox(mailbox);
            info!("SpacemiT: RPMI shared-memory mailbox injected");
        }
        // The CPPC extension shares the same mailbox backend.
        if let Some(cppc) = sbi::cppc() {
            cppc.set_mailbox(mailbox);
        }
    };
    crate::devicetree::search_with_parent(root, &mut find);

    if !found {
        warn!(
            "riscv,rpmi-shmem-mbox: node not found; RPMI mailbox not injected, MPXY/CPPC stay not supported"
        );
    }
}

/// Returns the board's memory range (set during `init_board`).
pub fn memory_range() -> Range<usize> {
    board_info().memory_range.as_ref().unwrap().clone()
}
