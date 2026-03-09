//! UDS Client (Tester) task — sends UDS requests and processes responses.

use cu29::prelude::*;
use cu_automotive_payloads::isotp::IsotpPdu;

const MAX_PENDING: usize = 8;

#[derive(Clone, Debug)]
struct PendingReq {
    data: [u8; 256],
    len: u16,
    active: bool,
}

impl Default for PendingReq {
    fn default() -> Self {
        Self { data: [0u8; 256], len: 0, active: false }
    }
}

/// UDS Client task for sending diagnostic requests.
///
/// Maintains a queue of pending requests. Each cycle sends at most one
/// and checks for incoming responses.
///
/// # Config
/// - `p2_timeout_ms` (i64): Response timeout. Default: 1000.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct UdsClient {
    queue: [PendingReq; MAX_PENDING],
    head: usize,
    tail: usize,
    awaiting: bool,
    #[allow(dead_code)]
    p2_timeout_ms: u64,
    pub last_response: Option<IsotpPdu>,
}

impl Freezable for UdsClient {}

impl UdsClient {
    pub fn enqueue_raw(&mut self, data: &[u8]) -> bool {
        let next = (self.tail + 1) % MAX_PENDING;
        if next == self.head { return false; }
        let len = data.len().min(256);
        self.queue[self.tail].data[..len].copy_from_slice(&data[..len]);
        self.queue[self.tail].len = len as u16;
        self.queue[self.tail].active = true;
        self.tail = next;
        true
    }

    pub fn enqueue_session_control(&mut self, session: u8) -> bool {
        self.enqueue_raw(&[0x10, session])
    }

    pub fn enqueue_tester_present(&mut self, suppress: bool) -> bool {
        self.enqueue_raw(&[0x3E, if suppress { 0x80 } else { 0x00 }])
    }

    pub fn enqueue_read_did(&mut self, did: u16) -> bool {
        self.enqueue_raw(&[0x22, (did >> 8) as u8, (did & 0xFF) as u8])
    }

    fn dequeue(&mut self) -> Option<IsotpPdu> {
        if self.head == self.tail { return None; }
        let req = &self.queue[self.head];
        if !req.active { return None; }
        let mut pdu = IsotpPdu::default();
        let len = req.len as usize;
        pdu.data[..len].copy_from_slice(&req.data[..len]);
        pdu.len = len as u16;
        self.queue[self.head].active = false;
        self.head = (self.head + 1) % MAX_PENDING;
        Some(pdu)
    }
}

impl CuTask for UdsClient {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(IsotpPdu);
    type Output<'m> = output_msg!(IsotpPdu);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where Self: Sized {
        let p2 = match config {
            Some(cfg) => cfg.get::<i64>("p2_timeout_ms")?.unwrap_or(1000) as u64,
            None => 1000,
        };
        Ok(Self {
            queue: core::array::from_fn(|_| PendingReq::default()),
            head: 0, tail: 0,
            awaiting: false,
            p2_timeout_ms: p2,
            last_response: None,
        })
    }

    fn process<'i, 'o>(
        &mut self, ctx: &CuContext,
        input: &Self::Input<'i>, output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if let Some(pdu) = input.payload() {
            self.last_response = Some(pdu.clone());
            self.awaiting = false;
        }
        if !self.awaiting {
            if let Some(pdu) = self.dequeue() {
                output.set_payload(pdu);
                output.tov = Tov::Time(ctx.now());
                self.awaiting = true;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enqueue_dequeue() {
        let mut c = UdsClient::new(None, ()).unwrap();
        assert!(c.enqueue_session_control(0x03));
        assert!(c.enqueue_tester_present(false));
        let p1 = c.dequeue().unwrap();
        assert_eq!(p1.data[0], 0x10);
        let p2 = c.dequeue().unwrap();
        assert_eq!(p2.data[0], 0x3E);
    }

    #[test]
    fn queue_full() {
        let mut c = UdsClient::new(None, ()).unwrap();
        for _ in 0..MAX_PENDING - 1 { assert!(c.enqueue_tester_present(false)); }
        assert!(!c.enqueue_tester_present(false));
    }
}
