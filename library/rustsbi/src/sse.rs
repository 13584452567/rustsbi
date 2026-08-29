use sbi_spec::binary::{SbiRet, SharedPtr};

/// SBI Supervisor Software Events (SSE) support extension.
///
/// The Supervisor Software Events extension allows the supervisor-mode
/// software to register and manage software events, whose handlers run in
/// supervisor mode with interrupts disabled. This enables lightweight event
/// notification without relying on interrupts or polling.
///
/// See RISC-V SBI specification chapter 17 for details.
pub trait Sse {
    /// Read a range of event attribute values from a software event.
    fn read_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        output: SharedPtr<u8>,
    ) -> SbiRet;
    /// Write a range of event attribute values to a software event.
    fn write_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        input: SharedPtr<u8>,
    ) -> SbiRet;
    /// Register an event handler for the software event.
    fn register(&self, event_id: u32, handler_entry_pc: usize, handler_entry_arg: usize) -> SbiRet;
    /// Unregister the event handler for the given `event_id`.
    fn unregister(&self, event_id: u32) -> SbiRet;
    /// Enable a software event for the calling hart.
    fn enable(&self, event_id: u32) -> SbiRet;
    /// Disable a software event for the calling hart.
    fn disable(&self, event_id: u32) -> SbiRet;
    /// Complete the handling of the current software event on the calling hart.
    fn complete(&self) -> SbiRet;
    /// Inject a software event on the specified hart.
    fn inject(&self, event_id: u32, hart_id: usize) -> SbiRet;
    /// Unmask software events on the calling hart.
    fn hart_unmask(&self) -> SbiRet;
    /// Mask software events on the calling hart.
    fn hart_mask(&self) -> SbiRet;
    /// Function internal to macros. Do not use.
    #[doc(hidden)]
    #[inline]
    fn _rustsbi_probe(&self) -> usize {
        sbi_spec::base::UNAVAILABLE_EXTENSION.wrapping_add(1)
    }
}

impl<T: Sse> Sse for &T {
    #[inline]
    fn read_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        output: SharedPtr<u8>,
    ) -> SbiRet {
        T::read_attrs(self, event_id, base_attr_id, attr_count, output)
    }
    #[inline]
    fn write_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        input: SharedPtr<u8>,
    ) -> SbiRet {
        T::write_attrs(self, event_id, base_attr_id, attr_count, input)
    }
    #[inline]
    fn register(&self, event_id: u32, handler_entry_pc: usize, handler_entry_arg: usize) -> SbiRet {
        T::register(self, event_id, handler_entry_pc, handler_entry_arg)
    }
    #[inline]
    fn unregister(&self, event_id: u32) -> SbiRet {
        T::unregister(self, event_id)
    }
    #[inline]
    fn enable(&self, event_id: u32) -> SbiRet {
        T::enable(self, event_id)
    }
    #[inline]
    fn disable(&self, event_id: u32) -> SbiRet {
        T::disable(self, event_id)
    }
    #[inline]
    fn complete(&self) -> SbiRet {
        T::complete(self)
    }
    #[inline]
    fn inject(&self, event_id: u32, hart_id: usize) -> SbiRet {
        T::inject(self, event_id, hart_id)
    }
    #[inline]
    fn hart_unmask(&self) -> SbiRet {
        T::hart_unmask(self)
    }
    #[inline]
    fn hart_mask(&self) -> SbiRet {
        T::hart_mask(self)
    }
}

impl<T: Sse> Sse for Option<T> {
    #[inline]
    fn read_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        output: SharedPtr<u8>,
    ) -> SbiRet {
        self.as_ref().map_or(SbiRet::not_supported(), |inner| {
            T::read_attrs(inner, event_id, base_attr_id, attr_count, output)
        })
    }
    #[inline]
    fn write_attrs(
        &self,
        event_id: u32,
        base_attr_id: u32,
        attr_count: u32,
        input: SharedPtr<u8>,
    ) -> SbiRet {
        self.as_ref().map_or(SbiRet::not_supported(), |inner| {
            T::write_attrs(inner, event_id, base_attr_id, attr_count, input)
        })
    }
    #[inline]
    fn register(&self, event_id: u32, handler_entry_pc: usize, handler_entry_arg: usize) -> SbiRet {
        self.as_ref().map_or(SbiRet::not_supported(), |inner| {
            T::register(inner, event_id, handler_entry_pc, handler_entry_arg)
        })
    }
    #[inline]
    fn unregister(&self, event_id: u32) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::unregister(inner, event_id))
    }
    #[inline]
    fn enable(&self, event_id: u32) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::enable(inner, event_id))
    }
    #[inline]
    fn disable(&self, event_id: u32) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::disable(inner, event_id))
    }
    #[inline]
    fn complete(&self) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::complete(inner))
    }
    #[inline]
    fn inject(&self, event_id: u32, hart_id: usize) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::inject(inner, event_id, hart_id))
    }
    #[inline]
    fn hart_unmask(&self) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::hart_unmask(inner))
    }
    #[inline]
    fn hart_mask(&self) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::hart_mask(inner))
    }
    #[inline]
    fn _rustsbi_probe(&self) -> usize {
        match self {
            Some(_) => sbi_spec::base::UNAVAILABLE_EXTENSION.wrapping_add(1),
            None => sbi_spec::base::UNAVAILABLE_EXTENSION,
        }
    }
}
