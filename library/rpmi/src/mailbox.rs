//! RPMI mailbox abstraction over shared-memory queues.
//!
//! Provides a mailbox controller that owns the four RPMI queues and the
//! doorbell register, and offers the two typical request patterns defined
//! by OpenSBI `lib/utils/mailbox/rpmi_mailbox.c`:
//! `normal_request_with_status` (request + expected response) and
//! `posted_request` (fire-and-forget).

use core::sync::atomic::{AtomicU16, Ordering};

use log::{info, warn};

use crate::message::{Error, MessageHeader, MessageType, RPMI_MSG_TOKEN_MASK};
use crate::smq::{Le32, SmqQueue};

/// Queue identifiers (rpmi_msgprot.h `enum rpmi_queue_idx`).
pub mod queue_idx {
    pub const A2P_REQ: usize = 0;
    pub const P2A_ACK: usize = 1;
    pub const P2A_REQ: usize = 2;
    pub const A2P_ACK: usize = 3;
    pub const MAX_COUNT: usize = 4;
}

/// Default transfer timeout in milliseconds (OpenSBI `RPMI_DEF_TX_TIMEOUT`
/// / `RPMI_DEF_RX_TIMEOUT`).
pub const RPMI_DEF_TX_TIMEOUT: u32 = 500;
pub const RPMI_DEF_RX_TIMEOUT: u32 = 500;

/// RPMI shared-memory mailbox controller.
///
/// Owns the four ring queues and an optional doorbell register used to
/// notify the platform management processor. All operations take `&self`
/// so a single mailbox can be shared between several extensions.
pub struct RpmiMailbox {
    /// Size of one queue slot in bytes.
    slot_size: usize,
    /// The four RPMI queues in `queue_idx` order.
    queues: [SmqQueue; queue_idx::MAX_COUNT],
    /// Optional doorbell register (AP → PuC).
    doorbell: Option<&'static Le32>,
    /// Next token to use for requests.
    next_token: AtomicU16,
}

impl RpmiMailbox {
    /// Create a new mailbox controller.
    ///
    /// # Safety
    ///
    /// Every queue must alias shared memory accessible to both the AP and
    /// the PuC (see [`SmqQueue::new`]).
    pub unsafe fn new(
        slot_size: usize,
        queues: [SmqQueue; queue_idx::MAX_COUNT],
        doorbell: Option<&'static Le32>,
    ) -> Self {
        Self {
            slot_size,
            queues,
            doorbell,
            next_token: AtomicU16::new(1),
        }
    }

    /// Allocate the next message token.
    fn alloc_token(&self) -> u16 {
        let mut token = self.next_token.load(Ordering::Relaxed);
        loop {
            let next = token.wrapping_add(1) & RPMI_MSG_TOKEN_MASK;
            if next == 0 {
                continue;
            }
            match self.next_token.compare_exchange_weak(
                token,
                next,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => return next,
                Err(current) => token = current,
            }
        }
    }

    /// Perform a normal RPMI request expecting a response.
    ///
    /// Sends a `NormalRequest` message on the A2P_REQ queue and waits for
    /// the matching acknowledgement on the P2A_ACK queue. The response
    /// payload is copied into `resp` and the first word (the status code)
    /// is returned as an [`Error`].
    ///
    /// Returns `Err(())` on transport failure (queue full, timeout).
    pub fn normal_request_with_status(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        req: &[u8],
        resp: &mut [u8],
    ) -> Result<(Error, usize), ()> {
        if resp.len() < 4 {
            return Err(());
        }
        let token = self.alloc_token();
        let header = MessageHeader::new(
            servicegroup_id,
            service_id,
            MessageType::NormalRequest,
            req.len() as u16,
            token,
        );
        // Safety: the queue aliases shared memory established at `new`.
        unsafe {
            self.queues[queue_idx::A2P_REQ]
                .send(&header, req, self.doorbell)
                .map_err(|_| ())?;
        }
        info!(
            "RPMI send: sg={} svc={} token={} len={} db={:?}",
            servicegroup_id, service_id, token, req.len(), self.doorbell_addr()
        );

        // Wait for the acknowledgement carrying our token.
        let mut rx = [0u8; 256];
        for _ in 0..RPMI_DEF_RX_TIMEOUT {
            // Safety: as above.
            let n = unsafe { self.queues[queue_idx::P2A_ACK].receive(token, &mut rx) };
            if let Ok(n) = n {
                let status = i32::from_le_bytes([rx[0], rx[1], rx[2], rx[3]]);
                info!(
                    "RPMI recv: sg={} svc={} token={} n={} status={:#x}",
                    servicegroup_id, service_id, token, n, status as u32
                );
                // `rx` is the full acknowledgement payload
                // [status(4)][response data]. Copy it wholesale so the caller
                // sees the same layout OpenSBI's rpmi_normal_request_with_status
                // produces (status at offset 0, data from offset 4).
                let copy = n.min(resp.len());
                if copy > 0 {
                    resp[..copy].copy_from_slice(&rx[..copy]);
                }
                return Ok((Error::from_status(status), copy));
            }
            // No message yet: yield and retry.
            core::hint::spin_loop();
        }
        warn!(
            "RPMI send TIMEOUT: sg={} svc={} token={} (no ack in {} iters)",
            servicegroup_id, service_id, token, RPMI_DEF_RX_TIMEOUT
        );
        Err(())
    }

    /// Perform a posted RPMI request without a response.
    ///
    /// Sends a `PostedRequest` message on the A2P_REQ queue and returns
    /// immediately.
    pub fn posted_request(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        req: &[u8],
    ) -> Result<(), ()> {
        let token = self.alloc_token();
        let header = MessageHeader::new(
            servicegroup_id,
            service_id,
            MessageType::PostedRequest,
            req.len() as u16,
            token,
        );
        // Safety: the queue aliases shared memory established at `new`.
        let r = unsafe {
            self.queues[queue_idx::A2P_REQ]
                .send(&header, req, self.doorbell)
                .map_err(|_| ())
        };
        info!(
            "RPMI posted: sg={} svc={} token={} len={} ok={}",
            servicegroup_id, service_id, token, req.len(), r.is_ok()
        );
        r
    }

    /// Receive an asynchronous notification from the platform management
    /// processor.
    ///
    /// Notifications arrive on the P2A_REQ queue and carry no token; they
    /// are matched by their message identifier (service group ID + service
    /// ID + notification type). The payload is copied into `out`. Returns
    /// the number of payload bytes copied, or `Err(())` when no matching
    /// notification is pending.
    pub fn receive_notification(
        &self,
        servicegroup_id: u16,
        service_id: u8,
        out: &mut [u8],
    ) -> Result<usize, ()> {
        // Safety: the queue aliases shared memory established at `new`.
        unsafe {
            self.queues[queue_idx::P2A_REQ].receive_by_message_id(
                servicegroup_id,
                service_id,
                MessageType::Notification as u8,
                out,
            )
        }
    }

    /// Returns the slot size of this mailbox.
    pub const fn slot_size(&self) -> usize {
        self.slot_size
    }

    /// Debug helper: the doorbell register address (or 0).
    pub fn doorbell_addr(&self) -> usize {
        self.doorbell
            .map(|db| db as *const _ as usize)
            .unwrap_or(0)
    }
}
