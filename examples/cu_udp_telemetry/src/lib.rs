#![no_std]
//! Plain UDP example. Every datagram contains one complete native record.

use cu29_logstream::protocol::{
    CuTelemetryEvent, CuTelemetryPoll, CuTelemetryRecord, CuTelemetryRx, CuTelemetrySession,
    CuTelemetryTx,
};
use cu29_logstream::{CuStreamRx, CuStreamRxError, CuStreamTx, CuStreamTxError};
use cu29_runtime::continuity::SourceGapReason;
use cu29_traits::{CuError, CuResult};

const MAGIC: &[u8; 4] = b"CUT2";
const SESSION: u8 = 0;
const LIST: u8 = 1;
const KEYFRAME: u8 = 2;
const LOG: u8 = 3;
const HEADER: usize = 4 + 16 + 1 + 8;
const DATAGRAM: usize = 1200;
pub const MAX_PAYLOAD: usize = DATAGRAM - HEADER;

/// The customer's datagram header is independent of Copper's FEC protocol.
pub struct PlainUdpTx<T> {
    socket: T,
    session: [u8; DATAGRAM],
    session_len: usize,
    packet: [u8; DATAGRAM],
}

impl<T: CuStreamTx> PlainUdpTx<T> {
    pub fn new(socket: T) -> Self {
        Self {
            socket,
            session: [0; DATAGRAM],
            session_len: 0,
            packet: [0; DATAGRAM],
        }
    }
}

fn send(socket: &mut impl CuStreamTx, packet: &[u8]) -> CuResult<()> {
    match socket.try_send(packet) {
        Ok(()) | Err(CuStreamTxError::WouldBlock) => Ok(()),
        Err(CuStreamTxError::Failed(message)) => Err(CuError::from(message)),
    }
}

impl<T: CuStreamTx> CuTelemetryTx for PlainUdpTx<T> {
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        let (kind, id, bytes) = match record {
            CuTelemetryRecord::Session(session) => {
                self.session[..4].copy_from_slice(MAGIC);
                self.session[4..20].copy_from_slice(&session.session_id);
                self.session[20] = SESSION;
                let len = bincode::encode_into_slice(
                    (session.context, &session.schema),
                    &mut self.session[HEADER..],
                    bincode::config::standard(),
                )
                .map_err(|_| CuError::from("session exceeds one UDP datagram"))?;
                self.session_len = HEADER + len;
                self.packet[..20].copy_from_slice(&self.session[..20]);
                return send(&mut self.socket, &self.session[..self.session_len]);
            }
            CuTelemetryRecord::CopperList { id, capture } => (LIST, id, capture),
            CuTelemetryRecord::KeyFrame {
                copperlist_id,
                state,
            } => (KEYFRAME, copperlist_id, state),
            CuTelemetryRecord::StructuredLog { sequence, entry } => (LOG, sequence, entry),
        };
        if self.session_len == 0 {
            return Err("UDP record before session".into());
        }
        if bytes.len() > MAX_PAYLOAD {
            return Err("native record exceeds one UDP datagram".into());
        }
        if kind == KEYFRAME {
            // A late listener can discover the session at the next snapshot.
            send(&mut self.socket, &self.session[..self.session_len])?;
        }
        self.packet[20] = kind;
        self.packet[21..HEADER].copy_from_slice(&id.to_be_bytes());
        self.packet[HEADER..HEADER + bytes.len()].copy_from_slice(bytes);
        send(&mut self.socket, &self.packet[..HEADER + bytes.len()])
    }
}

/// Uses the first valid session. A higher cycle ID immediately declares a gap.
/// Duplicates and older packets in each record family are discarded.
pub struct PlainUdpRx<R> {
    socket: R,
    session: Option<[u8; 16]>,
    next_list: u64,
    last_keyframe: Option<u64>,
    last_log: Option<u64>,
    packet: [u8; DATAGRAM],
}

impl<R: CuStreamRx> PlainUdpRx<R> {
    pub fn new(socket: R) -> Self {
        Self {
            socket,
            session: None,
            next_list: 0,
            last_keyframe: None,
            last_log: None,
            packet: [0; DATAGRAM],
        }
    }
}

impl<R: CuStreamRx> CuTelemetryRx for PlainUdpRx<R> {
    fn try_recv(
        &mut self,
        mut deliver: impl FnMut(CuTelemetryEvent<'_>) -> CuResult<()>,
    ) -> CuResult<CuTelemetryPoll> {
        let packet = match self.socket.try_recv(&mut self.packet) {
            Ok(None) => return Ok(CuTelemetryPoll::Pending),
            Err(CuStreamRxError::BufferTooSmall { .. }) => return Ok(CuTelemetryPoll::Progress),
            Err(CuStreamRxError::Failed(message)) => return Err(CuError::from(message)),
            Ok(Some(len)) => self
                .packet
                .get(..len)
                .ok_or_else(|| CuError::from("invalid UDP receive count"))?,
        };
        let Some((header, payload)) = packet.split_at_checked(HEADER) else {
            return Ok(CuTelemetryPoll::Progress);
        };
        if &header[..4] != MAGIC {
            return Ok(CuTelemetryPoll::Progress);
        }
        let session_id: [u8; 16] = header[4..20].try_into().expect("fixed header");
        let kind = header[20];
        let id = u64::from_be_bytes(header[21..].try_into().expect("fixed header"));
        let record = match kind {
            SESSION if self.session.is_none() && id == 0 => {
                // Bound decoded allocations as well as the wire length.
                if let Ok(((context, schema), used)) = bincode::decode_from_slice(
                    payload,
                    bincode::config::standard().with_limit::<16384>(),
                ) && used == payload.len()
                {
                    let session = CuTelemetrySession {
                        session_id,
                        context,
                        schema,
                    };
                    deliver(CuTelemetryEvent::Record(CuTelemetryRecord::Session(
                        &session,
                    )))?;
                    self.session = Some(session_id);
                }
                return Ok(CuTelemetryPoll::Progress);
            }
            _ if self.session != Some(session_id) => return Ok(CuTelemetryPoll::Progress),
            LIST if id >= self.next_list => {
                if id > self.next_list {
                    deliver(CuTelemetryEvent::Gap {
                        first_id: self.next_list,
                        last_id: id - 1,
                        reason: SourceGapReason::ProtocolLoss,
                    })?;
                }
                self.next_list = id
                    .checked_add(1)
                    .ok_or_else(|| CuError::from("cycle ID exhausted"))?;
                CuTelemetryRecord::CopperList {
                    id,
                    capture: payload,
                }
            }
            KEYFRAME if self.last_keyframe.is_none_or(|last| id > last) => {
                self.last_keyframe = Some(id);
                CuTelemetryRecord::KeyFrame {
                    copperlist_id: id,
                    state: payload,
                }
            }
            LOG if self.last_log.is_none_or(|last| id > last) => {
                self.last_log = Some(id);
                CuTelemetryRecord::StructuredLog {
                    sequence: id,
                    entry: payload,
                }
            }
            _ => return Ok(CuTelemetryPoll::Progress),
        };
        deliver(CuTelemetryEvent::Record(record))?;
        Ok(CuTelemetryPoll::Progress)
    }
}
