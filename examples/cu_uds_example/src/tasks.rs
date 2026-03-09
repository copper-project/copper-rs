//! Local tasks for the UDS diagnostic example.
//!
//! UdsTestSource — generates diagnostic requests as IsotpPdu payloads.
//! UdsResponseSink — receives and logs UDS server responses.

use cu29::prelude::*;
use cu_automotive_payloads::isotp::IsotpPdu;
use cu_automotive_payloads::uds::UdsResponse;

/// Cycles through a predefined set of UDS requests.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct UdsTestSource {
    cycle: u32,
}

impl Freezable for UdsTestSource {}

impl CuSrcTask for UdsTestSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(IsotpPdu);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { cycle: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let request_bytes: &[u8] = match self.cycle % 3 {
            // DiagnosticSessionControl (0x10) — ExtendedDiagnosticSession (0x03)
            0 => &[0x10, 0x03],
            // ReadDataByIdentifier (0x22) — DID 0xF190 (VIN)
            1 => &[0x22, 0xF1, 0x90],
            // ReadDataByIdentifier (0x22) — DID 0xF187 (Part Number)
            _ => &[0x22, 0xF1, 0x87],
        };

        let pdu = IsotpPdu::from_data(request_bytes);
        output.set_payload(pdu);
        output.tov = Tov::Time(ctx.now());
        self.cycle += 1;
        Ok(())
    }
}

/// Sinks UDS server responses and logs them.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct UdsResponseSink {
    rx_count: u64,
}

impl Freezable for UdsResponseSink {}

impl CuSinkTask for UdsResponseSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(IsotpPdu);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { rx_count: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(pdu) = input.payload() {
            let raw = &pdu.data[..pdu.len as usize];
            if let Some(resp) = UdsResponse::from_bytes(pdu.source_addr, raw) {
                if resp.is_negative {
                    debug!(
                        "[UDS RX #{}] Negative response: SID=0x{:02X} NRC=0x{:02X}",
                        self.rx_count, resp.service_id, resp.nrc as u8
                    );
                } else {
                    debug!(
                        "[UDS RX #{}] Positive response: SID=0x{:02X} len={}",
                        self.rx_count, resp.service_id, resp.data_len
                    );
                }
            } else {
                debug!("[UDS RX #{}] Raw ISO-TP PDU: {} bytes", self.rx_count, pdu.len);
            }
            self.rx_count += 1;
        }
        Ok(())
    }
}
