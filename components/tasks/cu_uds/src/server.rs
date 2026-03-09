//! UDS Server task — receives UDS requests, dispatches, produces responses.

use cu29::prelude::*;
use cu_automotive_payloads::{
    isotp::IsotpPdu,
    uds::{Nrc, UdsResponse, UdsSessionType, UDS_MAX_PAYLOAD_SIZE},
};

// SID constants
const SID_DIAG_SESSION_CONTROL: u8 = 0x10;
const SID_ECU_RESET: u8 = 0x11;
const SID_CLEAR_DTC: u8 = 0x14;
const SID_READ_DID: u8 = 0x22;
const SID_SECURITY_ACCESS: u8 = 0x27;
const SID_WRITE_DID: u8 = 0x2E;
const SID_ROUTINE_CONTROL: u8 = 0x31;
const SID_TESTER_PRESENT: u8 = 0x3E;

/// UDS Server task that processes diagnostic requests.
///
/// Receives reassembled ISO-TP PDUs containing UDS requests, parses and
/// dispatches them, manages session state, and produces UDS response PDUs
/// to be sent back through ISO-TP.
///
/// # Config
/// - `session_timeout_ms` (i64): S3 server timer. Default: 5000.
/// - `p2_server_ms` (i64): P2 timing parameter. Default: 50.
/// - `p2_star_server_ms` (i64): P2* timing. Default: 5000.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct UdsServer {
    session_type: UdsSessionType,
    s3_remaining_ms: u64,
    session_timeout_ms: u64,
    #[allow(dead_code)]
    p2_server_ms: u64,
    #[allow(dead_code)]
    p2_star_server_ms: u64,
    security_level: u8,
    security_seed: u32,
    /// DID table: (DID, data, data_len)
    did_table: [(u16, [u8; 32], u8); 16],
    did_count: u8,
}

impl Freezable for UdsServer {}

impl UdsServer {
    fn reset_session(&mut self) {
        self.session_type = UdsSessionType::Default;
        self.security_level = 0;
        self.s3_remaining_ms = self.session_timeout_ms;
    }

    /// Build a negative-response ISO-TP PDU.
    fn nrc_pdu(sid: u8, nrc: Nrc) -> IsotpPdu {
        let resp = UdsResponse::negative(sid, 0, nrc);
        Self::resp_to_pdu(&resp)
    }

    /// Serialize a UdsResponse into an ISO-TP PDU.
    fn resp_to_pdu(resp: &UdsResponse) -> IsotpPdu {
        let mut pdu = IsotpPdu::default();
        let mut buf = [0u8; UDS_MAX_PAYLOAD_SIZE + 4];
        let len = resp.to_bytes(&mut buf);
        let copy = len.min(pdu.data.len());
        pdu.data[..copy].copy_from_slice(&buf[..copy]);
        pdu.len = copy as u16;
        pdu
    }

    /// Dispatch an incoming raw PDU.
    fn handle(&mut self, pdu: &IsotpPdu) -> Option<IsotpPdu> {
        let len = pdu.len as usize;
        if len == 0 {
            return Some(Self::nrc_pdu(0, Nrc::GeneralReject));
        }
        let sid = pdu.data[0];
        let sub = if len > 1 { pdu.data[1] } else { 0 };
        let suppress = sub & 0x80 != 0;
        let sub_clean = sub & 0x7F;

        // Reset S3 timer on any request
        self.s3_remaining_ms = self.session_timeout_ms;

        match sid {
            SID_DIAG_SESSION_CONTROL => self.handle_session(sub_clean, suppress),
            SID_ECU_RESET => self.handle_ecu_reset(sub_clean, suppress),
            SID_CLEAR_DTC => {
                if suppress { None } else {
                    Some(Self::resp_to_pdu(&UdsResponse::positive(sid, 0, &[])))
                }
            }
            SID_READ_DID => self.handle_read_did(&pdu.data[1..len]),
            SID_SECURITY_ACCESS => self.handle_security(&pdu.data[1..len], suppress),
            SID_WRITE_DID => self.handle_write_did(&pdu.data[1..len], suppress),
            SID_ROUTINE_CONTROL => self.handle_routine(sub_clean, &pdu.data[1..len]),
            SID_TESTER_PRESENT => {
                if sub_clean != 0 {
                    return Some(Self::nrc_pdu(sid, Nrc::SubFunctionNotSupported));
                }
                if suppress { None } else {
                    Some(Self::resp_to_pdu(&UdsResponse::positive(sid, 0, &[0x00])))
                }
            }
            _ => Some(Self::nrc_pdu(sid, Nrc::ServiceNotSupported)),
        }
    }

    fn handle_session(&mut self, sub: u8, suppress: bool) -> Option<IsotpPdu> {
        let session = match sub {
            0x01 => UdsSessionType::Default,
            0x02 => UdsSessionType::Programming,
            0x03 => UdsSessionType::Extended,
            _ => return Some(Self::nrc_pdu(SID_DIAG_SESSION_CONTROL, Nrc::SubFunctionNotSupported)),
        };
        self.session_type = session;
        self.security_level = 0;
        if suppress { return None; }
        let p2 = (self.p2_server_ms as u16).to_be_bytes();
        let p2s = ((self.p2_star_server_ms / 10) as u16).to_be_bytes();
        let data = [sub, p2[0], p2[1], p2s[0], p2s[1]];
        Some(Self::resp_to_pdu(&UdsResponse::positive(SID_DIAG_SESSION_CONTROL, 0, &data)))
    }

    fn handle_ecu_reset(&mut self, sub: u8, suppress: bool) -> Option<IsotpPdu> {
        match sub {
            0x01 | 0x02 | 0x03 => {
                self.reset_session();
                if suppress { None } else {
                    Some(Self::resp_to_pdu(&UdsResponse::positive(SID_ECU_RESET, 0, &[sub])))
                }
            }
            _ => Some(Self::nrc_pdu(SID_ECU_RESET, Nrc::SubFunctionNotSupported)),
        }
    }

    fn handle_security(&mut self, data: &[u8], suppress: bool) -> Option<IsotpPdu> {
        if data.is_empty() {
            return Some(Self::nrc_pdu(SID_SECURITY_ACCESS, Nrc::IncorrectMessageLengthOrInvalidFormat));
        }
        let access_type = data[0];
        if access_type % 2 == 1 {
            // Request seed
            self.security_seed = 0xDEAD_BEEF ^ (access_type as u32 * 0x1234);
            let sb = self.security_seed.to_be_bytes();
            if suppress { return None; }
            let resp_data = [access_type, sb[0], sb[1], sb[2], sb[3]];
            Some(Self::resp_to_pdu(&UdsResponse::positive(SID_SECURITY_ACCESS, 0, &resp_data)))
        } else {
            // Send key
            if data.len() < 5 {
                return Some(Self::nrc_pdu(SID_SECURITY_ACCESS, Nrc::IncorrectMessageLengthOrInvalidFormat));
            }
            let key = u32::from_be_bytes([data[1], data[2], data[3], data[4]]);
            let expected = self.security_seed ^ 0xCAFE_BABE;
            if key == expected {
                self.security_level = access_type / 2;
                if suppress { return None; }
                Some(Self::resp_to_pdu(&UdsResponse::positive(SID_SECURITY_ACCESS, 0, &[access_type])))
            } else {
                Some(Self::nrc_pdu(SID_SECURITY_ACCESS, Nrc::InvalidKey))
            }
        }
    }

    fn handle_read_did(&mut self, data: &[u8]) -> Option<IsotpPdu> {
        if data.len() < 2 {
            return Some(Self::nrc_pdu(SID_READ_DID, Nrc::IncorrectMessageLengthOrInvalidFormat));
        }
        let did = u16::from_be_bytes([data[0], data[1]]);
        for i in 0..self.did_count as usize {
            let (stored_did, ref stored_data, stored_len) = self.did_table[i];
            if stored_did == did {
                let dlen = stored_len as usize;
                let mut resp_data = [0u8; 34];
                resp_data[0] = (did >> 8) as u8;
                resp_data[1] = (did & 0xFF) as u8;
                resp_data[2..2 + dlen].copy_from_slice(&stored_data[..dlen]);
                return Some(Self::resp_to_pdu(&UdsResponse::positive(SID_READ_DID, 0, &resp_data[..2 + dlen])));
            }
        }
        Some(Self::nrc_pdu(SID_READ_DID, Nrc::RequestOutOfRange))
    }

    fn handle_write_did(&mut self, data: &[u8], suppress: bool) -> Option<IsotpPdu> {
        if data.len() < 3 {
            return Some(Self::nrc_pdu(SID_WRITE_DID, Nrc::IncorrectMessageLengthOrInvalidFormat));
        }
        if self.session_type == UdsSessionType::Default {
            return Some(Self::nrc_pdu(SID_WRITE_DID, Nrc::ServiceNotSupportedInActiveSession));
        }
        let did = u16::from_be_bytes([data[0], data[1]]);
        let write_len = (data.len() - 2).min(32);
        let write_data = &data[2..2 + write_len];

        // Update or add to table
        for i in 0..self.did_count as usize {
            if self.did_table[i].0 == did {
                self.did_table[i].1[..write_len].copy_from_slice(write_data);
                self.did_table[i].2 = write_len as u8;
                if suppress { return None; }
                return Some(Self::resp_to_pdu(&UdsResponse::positive(SID_WRITE_DID, 0, &data[..2])));
            }
        }
        if (self.did_count as usize) < self.did_table.len() {
            let idx = self.did_count as usize;
            self.did_table[idx].0 = did;
            self.did_table[idx].1[..write_len].copy_from_slice(write_data);
            self.did_table[idx].2 = write_len as u8;
            self.did_count += 1;
            if suppress { return None; }
            Some(Self::resp_to_pdu(&UdsResponse::positive(SID_WRITE_DID, 0, &data[..2])))
        } else {
            Some(Self::nrc_pdu(SID_WRITE_DID, Nrc::GeneralProgrammingFailure))
        }
    }

    fn handle_routine(&mut self, sub: u8, data: &[u8]) -> Option<IsotpPdu> {
        if data.len() < 3 {
            return Some(Self::nrc_pdu(SID_ROUTINE_CONTROL, Nrc::IncorrectMessageLengthOrInvalidFormat));
        }
        let routine_id_hi = data[0];
        let routine_id_lo = data[1];
        match sub {
            0x01 | 0x02 => {
                let resp_data = [sub, routine_id_hi, routine_id_lo];
                Some(Self::resp_to_pdu(&UdsResponse::positive(SID_ROUTINE_CONTROL, 0, &resp_data)))
            }
            0x03 => {
                let resp_data = [sub, routine_id_hi, routine_id_lo, 0x00];
                Some(Self::resp_to_pdu(&UdsResponse::positive(SID_ROUTINE_CONTROL, 0, &resp_data)))
            }
            _ => Some(Self::nrc_pdu(SID_ROUTINE_CONTROL, Nrc::SubFunctionNotSupported)),
        }
    }
}

impl CuTask for UdsServer {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(IsotpPdu);
    type Output<'m> = output_msg!(IsotpPdu);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let (timeout, p2, p2s) = match config {
            Some(cfg) => {
                let t = cfg.get::<i64>("session_timeout_ms")?.unwrap_or(5000) as u64;
                let p = cfg.get::<i64>("p2_server_ms")?.unwrap_or(50) as u64;
                let ps = cfg.get::<i64>("p2_star_server_ms")?.unwrap_or(5000) as u64;
                (t, p, ps)
            }
            None => (5000, 50, 5000),
        };

        let mut did_table = [(0u16, [0u8; 32], 0u8); 16];
        // Pre-populate VIN
        did_table[0].0 = 0xF190;
        let vin = b"COPPERVIN00000001";
        did_table[0].1[..vin.len()].copy_from_slice(vin);
        did_table[0].2 = vin.len() as u8;
        // Pre-populate SW version
        did_table[1].0 = 0xF101;
        let sw = b"CU-UDS v0.1.0";
        did_table[1].1[..sw.len()].copy_from_slice(sw);
        did_table[1].2 = sw.len() as u8;

        Ok(Self {
            session_type: UdsSessionType::Default,
            s3_remaining_ms: timeout,
            session_timeout_ms: timeout,
            p2_server_ms: p2,
            p2_star_server_ms: p2s,
            security_level: 0,
            security_seed: 0,
            did_table,
            did_count: 2,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if self.session_type != UdsSessionType::Default && self.s3_remaining_ms == 0 {
            self.reset_session();
        }

        if let Some(pdu) = input.payload() {
            if let Some(resp_pdu) = self.handle(pdu) {
                output.set_payload(resp_pdu);
                output.tov = Tov::Time(ctx.now());
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_server() -> UdsServer {
        UdsServer::new(None, ()).unwrap()
    }

    fn make_pdu(bytes: &[u8]) -> IsotpPdu {
        IsotpPdu::from_data(bytes)
    }

    #[test]
    fn session_control_default() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x10, 0x01]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x50); // positive
        assert_eq!(resp.data[1], 0x01);
    }

    #[test]
    fn session_control_extended() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x10, 0x03]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x50);
        assert_eq!(server.session_type, UdsSessionType::Extended);
    }

    #[test]
    fn tester_present() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x3E, 0x00]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x7E);
    }

    #[test]
    fn tester_present_suppress() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x3E, 0x80]);
        let resp = server.handle(&pdu);
        assert!(resp.is_none());
    }

    #[test]
    fn read_did_vin() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x22, 0xF1, 0x90]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x62);
        assert_eq!(resp.data[1], 0xF1);
        assert_eq!(resp.data[2], 0x90);
        assert_eq!(&resp.data[3..3 + 17], b"COPPERVIN00000001");
    }

    #[test]
    fn read_did_not_found() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x22, 0xFF, 0xFF]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x7F);
        assert_eq!(resp.data[1], 0x22);
        assert_eq!(resp.data[2], Nrc::RequestOutOfRange as u8);
    }

    #[test]
    fn security_access_flow() {
        let mut server = make_server();
        // Request seed (level 1)
        let pdu = make_pdu(&[0x27, 0x01]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x67);
        let seed = u32::from_be_bytes([resp.data[2], resp.data[3], resp.data[4], resp.data[5]]);

        // Send key
        let key = seed ^ 0xCAFE_BABE;
        let kb = key.to_be_bytes();
        let pdu2 = make_pdu(&[0x27, 0x02, kb[0], kb[1], kb[2], kb[3]]);
        let resp2 = server.handle(&pdu2).unwrap();
        assert_eq!(resp2.data[0], 0x67);
        assert_eq!(server.security_level, 1);
    }

    #[test]
    fn unsupported_service() {
        let mut server = make_server();
        let pdu = make_pdu(&[0x85, 0x02]);
        let resp = server.handle(&pdu).unwrap();
        assert_eq!(resp.data[0], 0x7F);
        assert_eq!(resp.data[2], Nrc::ServiceNotSupported as u8);
    }
}
