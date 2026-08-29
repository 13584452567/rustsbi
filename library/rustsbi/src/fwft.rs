use sbi_spec::binary::SbiRet;

/// SBI Firmware Features (FWFT) support extension.
///
/// The Firmware Features extension allows the supervisor-mode software to
/// configure the behavior of the SBI implementation itself, such as whether
/// misaligned load/store exceptions are delegated to S-mode, whether landing
/// pad / shadow stack / double trap are enabled, whether the hardware updates
/// PTE A/D bits, and the pointer-masking tag length.
///
/// See RISC-V SBI specification chapter 18 for details.
pub trait Fwft {
    /// Set the configuration value of the firmware feature specified by
    /// `feature_id`.
    ///
    /// # Return value
    ///
    /// The possible return error codes returned in `SbiRet.error` are shown in the table below:
    ///
    /// | Error code                | Description
    /// | ------------------------- | -------------------------------------------------
    /// | `SbiRet::success()`       | The feature was set successfully.
    /// | `SbiRet::not_supported()` | The feature is valid but not supported by the platform.
    /// | `SbiRet::invalid_param()` | The provided `value` or `flags` parameter is invalid.
    /// | `SbiRet::denied()`        | The set operation was denied by the SBI implementation, or the feature is reserved or platform-specific and unimplemented.
    /// | `SbiRet::denied_locked()` | The feature is locked and can no longer be modified.
    /// | `SbiRet::failed()`        | The set operation failed for unspecified or unknown other reasons.
    fn set(&self, feature_id: u32, value: usize, flags: usize) -> SbiRet;
    /// Get the configuration value of the firmware feature specified by
    /// `feature_id`.
    ///
    /// # Return value
    ///
    /// Returns the feature configuration value in `SbiRet.value` on success.
    ///
    /// The possible return error codes returned in `SbiRet.error` are shown in the table below:
    ///
    /// | Error code                | Description
    /// | ------------------------- | -------------------------------------------------
    /// | `SbiRet::success()`       | The feature value was retrieved successfully.
    /// | `SbiRet::not_supported()` | The feature is valid but not supported by the platform.
    /// | `SbiRet::denied()`        | The feature is reserved or platform-specific and unimplemented.
    /// | `SbiRet::failed()`        | The get operation failed for unspecified or unknown other reasons.
    fn get(&self, feature_id: u32) -> SbiRet;
    /// Function internal to macros. Do not use.
    #[doc(hidden)]
    #[inline]
    fn _rustsbi_probe(&self) -> usize {
        sbi_spec::base::UNAVAILABLE_EXTENSION.wrapping_add(1)
    }
}

impl<T: Fwft> Fwft for &T {
    #[inline]
    fn set(&self, feature_id: u32, value: usize, flags: usize) -> SbiRet {
        T::set(self, feature_id, value, flags)
    }
    #[inline]
    fn get(&self, feature_id: u32) -> SbiRet {
        T::get(self, feature_id)
    }
}

impl<T: Fwft> Fwft for Option<T> {
    #[inline]
    fn set(&self, feature_id: u32, value: usize, flags: usize) -> SbiRet {
        self.as_ref().map_or(SbiRet::not_supported(), |inner| {
            T::set(inner, feature_id, value, flags)
        })
    }
    #[inline]
    fn get(&self, feature_id: u32) -> SbiRet {
        self.as_ref()
            .map_or(SbiRet::not_supported(), |inner| T::get(inner, feature_id))
    }
    #[inline]
    fn _rustsbi_probe(&self) -> usize {
        match self {
            Some(_) => sbi_spec::base::UNAVAILABLE_EXTENSION.wrapping_add(1),
            None => sbi_spec::base::UNAVAILABLE_EXTENSION,
        }
    }
}
