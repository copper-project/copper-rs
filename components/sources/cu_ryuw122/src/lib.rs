#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod protocol;

use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;
#[cfg(feature = "std")]
use cu_linux_resources::LinuxSerialPort;
use cu_sensor_payloads::{PeerRangeObservation, RangePeerId};
use cu29::bincode::de::{Decode, Decoder};
use cu29::bincode::enc::{Encode, Encoder};
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::clock::{CuTime, Tov};
use cu29::prelude::*;
use cu29::resource::{Owned, ResourceBindingMap, ResourceBindings, ResourceManager};
use embedded_io::{ErrorKind, ErrorType, Read, Write};

use crate::protocol::{ModemEvent, parse_line};

const DEFAULT_READ_BUFFER_BYTES: usize = 512;
const DEFAULT_MAX_PENDING_OBSERVATIONS: usize = 32;
const DEFAULT_RESPONSE_TIMEOUT_MS: u64 = 250;
const DEFAULT_POLL_PAYLOAD: &str = "PING";
const DEFAULT_LINE_BUFFER_BYTES: usize = 256;
const MODEM_ADDRESS_MAX_BYTES: usize = 8;
const MODEM_PAYLOAD_MAX_BYTES: usize = 12;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    Serial,
}

pub struct Ryuw122ResourcesT<S> {
    pub serial: Owned<S>,
}

#[cfg(feature = "std")]
pub type Ryuw122Resources = Ryuw122ResourcesT<LinuxSerialPort>;

impl<'r, S: 'static + Send + Sync> ResourceBindings<'r> for Ryuw122ResourcesT<S> {
    type Binding = Binding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| {
            CuError::from("Ryuw122InitiatorSourceTask requires a `serial` resource mapping")
        })?;
        let path = mapping.get(Binding::Serial).ok_or_else(|| {
            CuError::from(
                "Ryuw122InitiatorSourceTask resources must include `serial: <bundle.resource>`",
            )
        })?;

        let serial = manager
            .take::<S>(path.typed())
            .map_err(|e| e.add_cause("Failed to fetch RYUW122 serial resource"))?;

        Ok(Self { serial })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct InFlightRequest {
    anchor_index: usize,
    sent_at_ns: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PendingObservation {
    observed_at: CuTime,
    observation: PeerRangeObservation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DecodedLine {
    Observation {
        anchor_id: RangePeerId,
        distance_cm: u32,
        rssi_dbm: Option<i16>,
    },
    Error,
    Ignore,
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct Ryuw122InitiatorSourceTask<S> {
    #[reflect(ignore)]
    serial: S,
    #[reflect(ignore)]
    read_buffer: Vec<u8>,
    #[reflect(ignore)]
    line_buffer: Vec<u8>,
    #[reflect(ignore)]
    pending_observations: VecDeque<PendingObservation>,
    max_pending_observations: usize,
    #[reflect(ignore)]
    anchor_ids: Vec<RangePeerId>,
    #[reflect(ignore)]
    anchor_commands: Vec<Vec<u8>>,
    poll_payload: String,
    response_timeout_ns: u64,
    next_anchor_index: usize,
    #[reflect(ignore)]
    in_flight: Option<InFlightRequest>,
}

#[cfg(feature = "std")]
pub type Ryuw122InitiatorSource = Ryuw122InitiatorSourceTask<LinuxSerialPort>;

impl<S: 'static> TypePath for Ryuw122InitiatorSourceTask<S> {
    fn type_path() -> &'static str {
        "cu_ryuw122::Ryuw122InitiatorSourceTask"
    }

    fn short_type_path() -> &'static str {
        "Ryuw122InitiatorSourceTask"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Ryuw122InitiatorSourceTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_ryuw122")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_ryuw122")
    }
}

impl<S> fmt::Debug for Ryuw122InitiatorSourceTask<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Ryuw122InitiatorSourceTask")
            .field("anchor_count", &self.anchor_ids.len())
            .field("poll_payload", &self.poll_payload)
            .field("response_timeout_ns", &self.response_timeout_ns)
            .field("max_pending_observations", &self.max_pending_observations)
            .field("next_anchor_index", &self.next_anchor_index)
            .field("in_flight", &self.in_flight)
            .finish()
    }
}

impl<S> Freezable for Ryuw122InitiatorSourceTask<S> {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.line_buffer, encoder)?;

        Encode::encode(&(self.pending_observations.len() as u64), encoder)?;
        for pending in &self.pending_observations {
            Encode::encode(&pending.observed_at, encoder)?;
            Encode::encode(&pending.observation, encoder)?;
        }

        Encode::encode(&(self.next_anchor_index as u64), encoder)?;
        let in_flight = self
            .in_flight
            .map(|request| (request.anchor_index as u64, request.sent_at_ns));
        Encode::encode(&in_flight, encoder)?;
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let line_buffer: Vec<u8> = Decode::decode(decoder)?;
        if line_buffer.len() > self.line_buffer.capacity() {
            return Err(DecodeError::ArrayLengthMismatch {
                required: self.line_buffer.capacity(),
                found: line_buffer.len(),
            });
        }
        self.line_buffer.clear();
        self.line_buffer.extend_from_slice(&line_buffer);

        let pending_len: u64 = Decode::decode(decoder)?;
        let pending_len = usize::try_from(pending_len).map_err(|_| {
            DecodeError::OtherString(
                "RYUW122 pending observation count overflows usize".to_string(),
            )
        })?;
        if pending_len > self.max_pending_observations {
            return Err(DecodeError::ArrayLengthMismatch {
                required: self.max_pending_observations,
                found: pending_len,
            });
        }
        self.pending_observations.clear();
        for _ in 0..pending_len {
            self.pending_observations.push_back(PendingObservation {
                observed_at: Decode::decode(decoder)?,
                observation: Decode::decode(decoder)?,
            });
        }

        let next_anchor_index: u64 = Decode::decode(decoder)?;
        self.next_anchor_index = usize::try_from(next_anchor_index).map_err(|_| {
            DecodeError::OtherString("RYUW122 next anchor index overflows usize".to_string())
        })?;
        if !self.anchor_ids.is_empty() && self.next_anchor_index >= self.anchor_ids.len() {
            return Err(DecodeError::OtherString(
                "RYUW122 keyframe next anchor index is out of range".to_string(),
            ));
        }

        let in_flight: Option<(u64, u64)> = Decode::decode(decoder)?;
        self.in_flight = in_flight
            .map(|(anchor_index, sent_at_ns)| {
                let anchor_index = usize::try_from(anchor_index).map_err(|_| {
                    DecodeError::OtherString(
                        "RYUW122 in-flight anchor index overflows usize".to_string(),
                    )
                })?;
                if anchor_index >= self.anchor_ids.len() {
                    return Err(DecodeError::OtherString(
                        "RYUW122 keyframe in-flight anchor index is out of range".to_string(),
                    ));
                }
                Ok(InFlightRequest {
                    anchor_index,
                    sent_at_ns,
                })
            })
            .transpose()?;
        Ok(())
    }
}

impl<S> CuSrcTask for Ryuw122InitiatorSourceTask<S>
where
    S: Read + Write + ErrorType + Send + Sync + 'static,
    <S as ErrorType>::Error: embedded_io::Error + fmt::Debug + 'static,
{
    type Resources<'r> = Ryuw122ResourcesT<S>;
    type Output<'m> = output_msg!(PeerRangeObservation);

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let anchor_ids = config_anchor_ids(config)?;
        let poll_payload = config_string(config, "poll_payload", DEFAULT_POLL_PAYLOAD)?;
        validate_poll_payload(&poll_payload)?;

        let read_buffer_bytes = config_u32(
            config,
            "read_buffer_bytes",
            DEFAULT_READ_BUFFER_BYTES as u32,
        )? as usize;
        let max_pending_observations = config_u32(
            config,
            "max_pending_observations",
            DEFAULT_MAX_PENDING_OBSERVATIONS as u32,
        )? as usize;
        let response_timeout_ms =
            config_u64(config, "response_timeout_ms", DEFAULT_RESPONSE_TIMEOUT_MS)?;

        Ok(Self {
            serial: resources.serial.0,
            read_buffer: alloc::vec![0_u8; read_buffer_bytes.max(64)],
            line_buffer: Vec::with_capacity(read_buffer_bytes.max(DEFAULT_LINE_BUFFER_BYTES)),
            pending_observations: VecDeque::with_capacity(max_pending_observations.max(1)),
            max_pending_observations: max_pending_observations.max(1),
            anchor_commands: build_anchor_commands(&anchor_ids, &poll_payload),
            anchor_ids,
            poll_payload,
            response_timeout_ns: response_timeout_ms.max(1).saturating_mul(1_000_000),
            next_anchor_index: 0,
            in_flight: None,
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.line_buffer.clear();
        self.pending_observations.clear();
        self.next_anchor_index = 0;
        self.in_flight = None;
        self.drive_request_cycle(ctx.now().as_nanos())
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let observed_at = ctx.now();
        self.read_and_decode(observed_at)?;
        self.drive_request_cycle(observed_at.as_nanos())?;

        if let Some(pending) = self.pending_observations.pop_front() {
            output.tov = Tov::Time(pending.observed_at);
            output.set_payload(pending.observation);
        }

        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.in_flight = None;
        self.pending_observations.clear();
        self.line_buffer.clear();
        Ok(())
    }
}

impl<S> Ryuw122InitiatorSourceTask<S>
where
    S: Read + Write + ErrorType + Send + Sync + 'static,
    <S as ErrorType>::Error: embedded_io::Error + fmt::Debug + 'static,
{
    fn drive_request_cycle(&mut self, now_ns: u64) -> CuResult<()> {
        if self.anchor_ids.is_empty() {
            return Ok(());
        }

        if let Some(in_flight) = self.in_flight {
            if now_ns.saturating_sub(in_flight.sent_at_ns) < self.response_timeout_ns {
                return Ok(());
            }
            self.next_anchor_index = (in_flight.anchor_index + 1) % self.anchor_ids.len();
            self.in_flight = None;
        }

        self.send_request(self.next_anchor_index, now_ns)
    }

    fn send_request(&mut self, anchor_index: usize, now_ns: u64) -> CuResult<()> {
        let serial = &mut self.serial;
        let command = &self.anchor_commands[anchor_index];
        write_all(serial, command)?;
        serial
            .flush()
            .map_err(|e| CuError::from(format!("RYUW122 flush failed: {e:?}")))?;
        self.in_flight = Some(InFlightRequest {
            anchor_index,
            sent_at_ns: now_ns,
        });
        Ok(())
    }

    fn read_and_decode(&mut self, observed_at: CuTime) -> CuResult<()> {
        loop {
            match self.serial.read(&mut self.read_buffer) {
                Ok(0) => break,
                Ok(n) => {
                    let (line_buffer, read_buffer) = (&mut self.line_buffer, &self.read_buffer);
                    append_read_bytes(line_buffer, &read_buffer[..n]);
                    self.decode_from_buffer(observed_at);

                    if n < self.read_buffer.len() {
                        break;
                    }
                }
                Err(e)
                    if matches!(
                        embedded_io::Error::kind(&e),
                        ErrorKind::TimedOut | ErrorKind::Interrupted
                    ) =>
                {
                    break;
                }
                Err(e) => return Err(CuError::from(format!("RYUW122 serial read failed: {e:?}"))),
            }
        }

        Ok(())
    }

    fn decode_from_buffer(&mut self, observed_at: CuTime) {
        while let Some(newline_idx) = self.line_buffer.iter().position(|byte| *byte == b'\n') {
            let decoded = {
                let line_bytes = &self.line_buffer[..=newline_idx];
                let line = match core::str::from_utf8(line_bytes) {
                    Ok(line) => line,
                    Err(_) => {
                        self.line_buffer.drain(..=newline_idx);
                        continue;
                    }
                };

                match parse_line(line) {
                    Ok(ModemEvent::RangeResponse(event)) => {
                        match RangePeerId::try_from(event.peer_id) {
                            Ok(anchor_id) => DecodedLine::Observation {
                                anchor_id,
                                distance_cm: event.distance_cm,
                                rssi_dbm: event.rssi_dbm,
                            },
                            Err(_) => DecodedLine::Ignore,
                        }
                    }
                    Ok(ModemEvent::Error) => DecodedLine::Error,
                    Ok(_) | Err(_) => DecodedLine::Ignore,
                }
            };

            self.line_buffer.drain(..=newline_idx);

            match decoded {
                DecodedLine::Observation {
                    anchor_id,
                    distance_cm,
                    rssi_dbm,
                } => self.handle_range_response(observed_at, anchor_id, distance_cm, rssi_dbm),
                DecodedLine::Error => {
                    if let Some(in_flight) = self.in_flight.take() {
                        self.next_anchor_index =
                            (in_flight.anchor_index + 1) % self.anchor_ids.len();
                    }
                }
                DecodedLine::Ignore => {}
            }
        }
    }

    fn handle_range_response(
        &mut self,
        observed_at: CuTime,
        anchor_id: RangePeerId,
        distance_cm: u32,
        rssi_dbm: Option<i16>,
    ) {
        self.push_pending_observation(PendingObservation {
            observed_at,
            observation: PeerRangeObservation::from_centimeters(anchor_id, distance_cm, rssi_dbm),
        });

        if let Some(in_flight) = self.in_flight
            && self.anchor_ids[in_flight.anchor_index] == anchor_id
        {
            self.next_anchor_index = (in_flight.anchor_index + 1) % self.anchor_ids.len();
            self.in_flight = None;
        }
    }

    fn push_pending_observation(&mut self, observation: PendingObservation) {
        if self.pending_observations.len() >= self.max_pending_observations {
            self.pending_observations.pop_front();
        }
        self.pending_observations.push_back(observation);
    }
}

fn config_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> CuResult<u32> {
    if let Some(cfg) = config {
        Ok(cfg.get::<u32>(key)?.unwrap_or(default))
    } else {
        Ok(default)
    }
}

fn config_u64(config: Option<&ComponentConfig>, key: &str, default: u64) -> CuResult<u64> {
    if let Some(cfg) = config {
        Ok(cfg.get::<u64>(key)?.unwrap_or(default))
    } else {
        Ok(default)
    }
}

fn config_string(config: Option<&ComponentConfig>, key: &str, default: &str) -> CuResult<String> {
    if let Some(cfg) = config {
        Ok(cfg
            .get::<String>(key)?
            .unwrap_or_else(|| default.to_string()))
    } else {
        Ok(default.to_string())
    }
}

fn config_anchor_ids(config: Option<&ComponentConfig>) -> CuResult<Vec<RangePeerId>> {
    let config = config.ok_or_else(|| {
        CuError::from("Ryuw122InitiatorSourceTask requires a config with non-empty `anchor_ids`")
    })?;

    let raw_anchor_ids: Vec<String> = config.get_value("anchor_ids")?.ok_or_else(|| {
        CuError::from("Ryuw122InitiatorSourceTask config must include `anchor_ids`")
    })?;

    if raw_anchor_ids.is_empty() {
        return Err(CuError::from(
            "Ryuw122InitiatorSourceTask config must include at least one anchor id",
        ));
    }

    raw_anchor_ids
        .into_iter()
        .map(|anchor_id| {
            validate_anchor_id(&anchor_id)?;
            RangePeerId::try_from(anchor_id.as_str()).map_err(|err| {
                CuError::from(format!("anchor id `{anchor_id}` is not valid: {err}"))
            })
        })
        .collect()
}

fn build_anchor_commands(anchor_ids: &[RangePeerId], payload: &str) -> Vec<Vec<u8>> {
    let payload_len = payload.len();
    anchor_ids
        .iter()
        .map(|anchor_id| {
            format!(
                "AT+ANCHOR_SEND={},{},{}\r\n",
                anchor_id.as_str(),
                payload_len,
                payload
            )
            .into_bytes()
        })
        .collect()
}

fn append_read_bytes(line_buffer: &mut Vec<u8>, bytes: &[u8]) {
    if line_buffer.len() + bytes.len() > line_buffer.capacity() {
        line_buffer.clear();
    }

    debug_assert!(bytes.len() <= line_buffer.capacity());
    line_buffer.extend_from_slice(bytes);
}

fn write_all<S>(serial: &mut S, bytes: &[u8]) -> CuResult<()>
where
    S: Write + ErrorType,
    <S as ErrorType>::Error: embedded_io::Error + fmt::Debug + 'static,
{
    let mut written = 0;
    while written < bytes.len() {
        let n = serial
            .write(&bytes[written..])
            .map_err(|e| CuError::from(format!("RYUW122 write failed: {e:?}")))?;
        if n == 0 {
            return Err(CuError::from("RYUW122 write failed: zero-byte write"));
        }
        written += n;
    }
    Ok(())
}

fn validate_anchor_id(anchor_id: &str) -> CuResult<()> {
    if anchor_id.is_empty() {
        return Err(CuError::from("RYUW122 anchor ids must not be empty"));
    }
    if !anchor_id.is_ascii() {
        return Err(CuError::from("RYUW122 anchor ids must be ASCII"));
    }
    if anchor_id.len() > MODEM_ADDRESS_MAX_BYTES {
        return Err(CuError::from(format!(
            "RYUW122 anchor id `{anchor_id}` exceeds {MODEM_ADDRESS_MAX_BYTES} bytes"
        )));
    }
    Ok(())
}

fn validate_poll_payload(payload: &str) -> CuResult<()> {
    if !payload.is_ascii() {
        return Err(CuError::from("RYUW122 poll payload must be ASCII"));
    }
    if payload.len() > MODEM_PAYLOAD_MAX_BYTES {
        return Err(CuError::from(format!(
            "RYUW122 poll payload exceeds {MODEM_PAYLOAD_MAX_BYTES} bytes"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::VecDeque;
    use cu29::clock::CuDuration;

    #[derive(Default)]
    struct FakeSerial {
        reads: VecDeque<Result<Vec<u8>, std::io::Error>>,
        writes: Vec<Vec<u8>>,
    }

    impl FakeSerial {
        fn with_reads(reads: impl IntoIterator<Item = Result<Vec<u8>, std::io::Error>>) -> Self {
            Self {
                reads: reads.into_iter().collect(),
                writes: Vec::new(),
            }
        }

        fn writes_as_strings(&self) -> Vec<String> {
            self.writes
                .iter()
                .map(|bytes| String::from_utf8(bytes.clone()).unwrap())
                .collect()
        }
    }

    impl ErrorType for FakeSerial {
        type Error = std::io::Error;
    }

    impl Read for FakeSerial {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let Some(next) = self.reads.pop_front() else {
                return Err(std::io::Error::from(std::io::ErrorKind::TimedOut));
            };
            match next {
                Ok(chunk) => {
                    let len = chunk.len().min(buf.len());
                    buf[..len].copy_from_slice(&chunk[..len]);
                    Ok(len)
                }
                Err(err) => Err(err),
            }
        }
    }

    impl Write for FakeSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.writes.push(buf.to_vec());
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    fn build_task(serial: FakeSerial) -> Ryuw122InitiatorSourceTask<FakeSerial> {
        Ryuw122InitiatorSourceTask {
            serial,
            read_buffer: alloc::vec![0_u8; 64],
            line_buffer: Vec::with_capacity(64),
            pending_observations: VecDeque::new(),
            max_pending_observations: 8,
            anchor_ids: alloc::vec![
                RangePeerId::new("ANCH0001").unwrap(),
                RangePeerId::new("ANCH0002").unwrap(),
            ],
            anchor_commands: alloc::vec![
                b"AT+ANCHOR_SEND=ANCH0001,4,PING\r\n".to_vec(),
                b"AT+ANCHOR_SEND=ANCH0002,4,PING\r\n".to_vec(),
            ],
            poll_payload: "PING".to_string(),
            response_timeout_ns: 50_000_000,
            next_anchor_index: 0,
            in_flight: None,
        }
    }

    #[test]
    fn start_sends_first_request_immediately() {
        let mut task = build_task(FakeSerial::default());
        let (ctx, _mock) = CuContext::new_mock_clock();

        task.start(&ctx).unwrap();

        assert_eq!(
            task.serial.writes_as_strings(),
            alloc::vec!["AT+ANCHOR_SEND=ANCH0001,4,PING\r\n".to_string()]
        );
    }

    #[test]
    fn advances_to_next_anchor_after_timeout() {
        let mut task = build_task(FakeSerial::default());
        let (ctx, mock) = CuContext::new_mock_clock();

        task.start(&ctx).unwrap();
        mock.increment(CuDuration::from_millis(60));

        let mut output = <Ryuw122InitiatorSourceTask<FakeSerial> as CuSrcTask>::Output::default();
        task.process(&ctx, &mut output).unwrap();

        assert_eq!(
            task.serial.writes_as_strings(),
            alloc::vec![
                "AT+ANCHOR_SEND=ANCH0001,4,PING\r\n".to_string(),
                "AT+ANCHOR_SEND=ANCH0002,4,PING\r\n".to_string()
            ]
        );
    }

    #[test]
    fn response_emits_observation_and_immediately_sends_next_request() {
        let response = b"+ANCHOR_RCV=ANCH0001,5,HELLO,40 cm,-71 dBm\r\n".to_vec();
        let mut task = build_task(FakeSerial::with_reads([Ok(response)]));
        let (ctx, _mock) = CuContext::new_mock_clock();

        task.start(&ctx).unwrap();
        let mut output = <Ryuw122InitiatorSourceTask<FakeSerial> as CuSrcTask>::Output::default();
        task.process(&ctx, &mut output).unwrap();

        let payload = output.payload().expect("observation payload");
        assert_eq!(payload.peer_id.as_str(), "ANCH0001");
        assert_eq!(
            payload.distance.get::<cu29::units::si::length::meter>(),
            0.4
        );
        assert_eq!(payload.rssi_dbm, Some(-71));
        assert_eq!(
            task.serial.writes_as_strings(),
            alloc::vec![
                "AT+ANCHOR_SEND=ANCH0001,4,PING\r\n".to_string(),
                "AT+ANCHOR_SEND=ANCH0002,4,PING\r\n".to_string()
            ]
        );
    }
}
