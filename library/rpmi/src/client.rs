//! RPMI service group clients built on the shared-memory mailbox.
//!
//! Provides the Base service group client used to probe other service
//! groups and query the platform, mirroring OpenSBI
//! `lib/utils/mailbox/fdt_mailbox_rpmi_shmem.c` (`smq_base_get_two_u32`,
//! `rpmi_get_platform_info`).

use crate::mailbox::RpmiMailbox;
use crate::message::{Error, base_service, servicegroup};

/// Base service group client.
pub struct BaseClient<'a> {
    mailbox: &'a mut RpmiMailbox,
}

impl<'a> BaseClient<'a> {
    /// Create a new Base service group client over a mailbox.
    pub fn new(mailbox: &'a mut RpmiMailbox) -> Self {
        Self { mailbox }
    }

    /// Send a Base service request that takes one u32 input and returns
    /// two u32 output words (mirrors `smq_base_get_two_u32`).
    fn get_two_u32(&mut self, service_id: u8, inarg: u32) -> Result<[u32; 2], Error> {
        // Response is [status(4)][data word1(4)][data word2(4)].
        let mut resp = [0u8; 12];
        let req = inarg.to_le_bytes();
        let (err, _len) = self
            .mailbox
            .normal_request_with_status(servicegroup::BASE, service_id, &req, &mut resp)
            .map_err(|_| Error::Timeout)?;
        if err != Error::Success {
            return Err(err);
        }
        Ok([
            u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]),
            u32::from_le_bytes([resp[8], resp[9], resp[10], resp[11]]),
        ])
    }

    /// Probe a service group and return its version, or `Err` when the
    /// group is not supported.
    pub fn probe_service_group(&mut self, group_id: u16) -> Result<u32, Error> {
        let out = self.get_two_u32(base_service::PROBE_SERVICE_GROUP, group_id as u32)?;
        Ok(out[1])
    }

    /// Get the RPMI specification version implemented by the platform.
    pub fn get_spec_version(&mut self) -> Result<u32, Error> {
        let out = self.get_two_u32(base_service::GET_SPEC_VERSION, 0)?;
        Ok(out[1])
    }

    /// Get the implementation version of the platform management
    /// processor firmware.
    pub fn get_implementation_version(&mut self) -> Result<u32, Error> {
        let out = self.get_two_u32(base_service::GET_IMPLEMENTATION_VERSION, 0)?;
        Ok(out[1])
    }

    /// Get the platform information string.
    ///
    /// `buf` receives the platform information; the returned slice is the
    /// portion actually filled.
    pub fn get_platform_info<'b>(&mut self, buf: &'b mut [u8]) -> Result<&'b [u8], Error> {
        let mut resp = [0u8; 256];
        let (err, _len) = self
            .mailbox
            .normal_request_with_status(
                servicegroup::BASE,
                base_service::GET_PLATFORM_INFO,
                &[],
                &mut resp,
            )
            .map_err(|_| Error::Timeout)?;
        if err != Error::Success {
            return Err(err);
        }
        let len = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]) as usize;
        let n = len.min(buf.len());
        buf[..n].copy_from_slice(&resp[8..8 + n]);
        Ok(&buf[..n])
    }

    /// Enable notification for the given event on the Base service group.
    ///
    /// Mirrors `rpmi_enable_notification_req` / `resp`.
    pub fn enable_notification(&mut self, event_id: u32) -> Result<(), Error> {
        let mut resp = [0u8; 4];
        let req = event_id.to_le_bytes();
        let (err, _len) = self
            .mailbox
            .normal_request_with_status(
                servicegroup::BASE,
                base_service::ENABLE_NOTIFICATION,
                &req,
                &mut resp,
            )
            .map_err(|_| Error::Timeout)?;
        if err != Error::Success {
            return Err(err);
        }
        Ok(())
    }
}
