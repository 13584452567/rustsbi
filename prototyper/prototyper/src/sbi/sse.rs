use core::sync::atomic::{AtomicBool, Ordering};

use rustsbi::SbiRet;
use sbi_spec::binary::SharedPtr;
use spin::Mutex;

/// Implementation of SBI Supervisor Software Events (SSE) extension.
///
/// This is a minimal, platform-event based implementation mirroring the state
/// machine of OpenSBI `lib/sbi/sbi_sse.c`:
///
/// ```text
/// UNUSED → REGISTERED → ENABLED → RUNNING → ENABLED
///              ↑            │
///              └── DISABLED ┘
/// ```
///
/// Prototyper runs single-hart, so the per-event state is kept in a global
/// table protected by a `spin::Mutex`, and the per-hart global event mask is a
/// single `AtomicBool`. Only local events (bit 31 clear) are supported; global
/// events are rejected as not supported.
pub(crate) struct SbiSse;

/// Platform-supported local SSE event IDs, mirroring the `supported_events[]`
/// table of OpenSBI `sbi_sse.c`.
///
/// RISC-V SBI spec local events: RAS = 1, PMU = 2, DBG = 3. Global events
/// carry the bit-31 flag and are not supported by prototyper.
const SUPPORTED_EVENTS: &[u32] = &[0x1, 0x2, 0x3];

/// Number of supported platform events.
const EVENT_COUNT: usize = SUPPORTED_EVENTS.len();

/// Per-event state machine states, mirroring `sbi_sse.c`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
enum EventState {
    /// Event is not registered (initial state).
    Unused = 0,
    /// Event has a registered handler but is not enabled.
    Registered = 1,
    /// Event is enabled and ready to be injected.
    Enabled = 2,
    /// Event is registered but temporarily disabled.
    Disabled = 3,
    /// Event handler is currently running on a hart (pending).
    Running = 4,
}

/// Per-event handler record, mirroring `struct sbi_sse_event` in `sbi_sse.c`.
#[derive(Clone, Copy)]
struct EventRecord {
    state: EventState,
    handler_pc: usize,
    handler_arg: usize,
    priority: u32,
}

impl EventRecord {
    /// A freshly reset event record.
    const UNUSED: Self = Self {
        state: EventState::Unused,
        handler_pc: 0,
        handler_arg: 0,
        priority: 0,
    };
}

/// Per-event state table. Prototyper is single-hart, so a single global table
/// (rather than a per-hart array) is sufficient.
static EVENTS: Mutex<[EventRecord; EVENT_COUNT]> = Mutex::new([EventRecord::UNUSED; EVENT_COUNT]);

/// Per-hart global event mask; `true` means software events are masked
/// (blocked) on the calling hart, mirroring `sbi_sse_hart_mask()`.
static HART_MASKED: AtomicBool = AtomicBool::new(false);

/// Returns the index of `event_id` in `SUPPORTED_EVENTS`, or `None` if the
/// event is not supported by this platform.
fn event_index(event_id: u32) -> Option<usize> {
    SUPPORTED_EVENTS.iter().position(|&id| id == event_id)
}

impl rustsbi::Sse for SbiSse {
    fn read_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        output: SharedPtr<u8>,
    ) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let events = EVENTS.lock();
        let event = &events[idx];
        // Shared memory follows little-endian byte ordering (see sbi_sse.c).
        let base = output.phys_addr_lo() as *mut u8;
        for i in 0..attr_count {
            let attr_id = base_attr_id + i;
            let value: u32 = match attr_id {
                // SBI_SSE_ATTR_STATE: current event state.
                0 => event.state as u32,
                // SBI_SSE_ATTR_PRIORITY: event priority.
                1 => event.priority,
                _ => return SbiRet::bad_range(),
            };
            unsafe {
                (base.add(i as usize * 4) as *mut u32).write_volatile(value.to_le());
            }
        }
        SbiRet::success(0)
    }

    fn write_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        input: SharedPtr<u8>,
    ) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let mut events = EVENTS.lock();
        let base = input.phys_addr_lo() as *const u8;
        for i in 0..attr_count {
            let attr_id = base_attr_id + i;
            let value = unsafe { (base.add(i as usize * 4) as *const u32).read_volatile() };
            match attr_id {
                // SBI_SSE_ATTR_STATE is read-only.
                0 => return SbiRet::denied(),
                // SBI_SSE_ATTR_PRIORITY is writable.
                1 => events[idx].priority = value,
                _ => return SbiRet::bad_range(),
            }
        }
        SbiRet::success(0)
    }

    fn register(&self, event_id: u32, handler_entry_pc: usize, handler_entry_arg: usize) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let mut events = EVENTS.lock();
        let event = &mut events[idx];
        if event.state != EventState::Unused {
            return SbiRet::invalid_state();
        }
        event.handler_pc = handler_entry_pc;
        event.handler_arg = handler_entry_arg;
        event.state = EventState::Registered;
        SbiRet::success(0)
    }

    fn unregister(&self, event_id: u32) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let mut events = EVENTS.lock();
        let event = &mut events[idx];
        match event.state {
            EventState::Registered | EventState::Disabled => {
                event.handler_pc = 0;
                event.handler_arg = 0;
                event.state = EventState::Unused;
                SbiRet::success(0)
            }
            _ => SbiRet::invalid_state(),
        }
    }

    fn enable(&self, event_id: u32) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let mut events = EVENTS.lock();
        let event = &mut events[idx];
        match event.state {
            EventState::Registered | EventState::Disabled => {
                event.state = EventState::Enabled;
                SbiRet::success(0)
            }
            _ => SbiRet::invalid_state(),
        }
    }

    fn disable(&self, event_id: u32) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let mut events = EVENTS.lock();
        let event = &mut events[idx];
        match event.state {
            EventState::Enabled | EventState::Running => {
                event.state = EventState::Disabled;
                SbiRet::success(0)
            }
            _ => SbiRet::invalid_state(),
        }
    }

    fn complete(&self) -> SbiRet {
        let mut events = EVENTS.lock();
        for event in events.iter_mut() {
            if event.state == EventState::Running {
                // Handler finished; restore the event to the enabled state.
                event.state = EventState::Enabled;
                return SbiRet::success(0);
            }
        }
        SbiRet::invalid_state()
    }

    fn inject(&self, event_id: u32, _hart_id: usize) -> SbiRet {
        let Some(idx) = event_index(event_id) else {
            return SbiRet::not_supported();
        };
        let mut events = EVENTS.lock();
        let event = &mut events[idx];
        if event.state != EventState::Enabled {
            return SbiRet::invalid_state();
        }
        // Mark the event as pending (running) on the target hart.
        event.state = EventState::Running;
        SbiRet::success(0)
    }

    fn hart_unmask(&self) -> SbiRet {
        HART_MASKED.store(false, Ordering::SeqCst);
        SbiRet::success(0)
    }

    fn hart_mask(&self) -> SbiRet {
        HART_MASKED.store(true, Ordering::SeqCst);
        SbiRet::success(0)
    }
}
