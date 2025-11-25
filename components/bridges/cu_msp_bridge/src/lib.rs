//! Copper bridge that multiplexes MSP traffic in both directions over a single serial port.
//! The type exposes a `requests` Tx channel that accepts [`MspRequestBatch`] messages and a
//! `responses` Rx channel that yields [`MspResponseBatch`] payloads decoded from the line.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

// std implementation
#[cfg(feature = "std")]
mod std_impl {
    pub use std::{format, mem, string::String, vec::Vec};

    pub const DEVICE_KEY: &str = "device";
    pub const BAUD_KEY: &str = "baudrate";
    pub const TIMEOUT_KEY: &str = "timeout_ms";
    pub const DEFAULT_DEVICE: &str = "/dev/ttyUSB0";
    pub const DEFAULT_BAUDRATE: u32 = 115_200;
    pub const DEFAULT_TIMEOUT_MS: u64 = 50;
}

// no-std implementation
#[cfg(not(feature = "std"))]
mod no_std_impl {
    pub use alloc::{format, vec::Vec};
    pub use core::mem;

    pub const SERIAL_INDEX_KEY: &str = "serial_port_index";
}

#[cfg(not(feature = "std"))]
use no_std_impl::*;
#[cfg(feature = "std")]
use std_impl::*;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::*;
use cu_msp_lib::structs::{MspRequest, MspResponse};
use cu_msp_lib::{MspPacket, MspParser};
use embedded_io::{Read, Write};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

const READ_BUFFER_SIZE: usize = 512;
const MAX_REQUESTS_PER_BATCH: usize = 8;
const MAX_RESPONSES_PER_BATCH: usize = 16;

pub trait SerialFactory<E>: Write<Error = E> + Read<Error = E> + Send + 'static {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized;
}

#[cfg(not(feature = "std"))]
impl<S, E> SerialFactory<E> for S
where
    S: Write<Error = E> + Read<Error = E> + Send + 'static,
{
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let slot = config
            .and_then(|cfg| cfg.get::<u32>(SERIAL_INDEX_KEY))
            .map(|v| v as usize)
            .unwrap_or(0);

        cu_embedded_registry::take(slot).ok_or_else(|| {
            CuError::from(format!(
                "MSP bridge missing serial for slot {slot} (max index {})",
                cu_embedded_registry::MAX_SERIAL_SLOTS - 1
            ))
        })
    }
}

#[cfg(feature = "std")]
impl SerialFactory<std::io::Error> for std_serial::StdSerial {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let device = config
            .and_then(|cfg| cfg.get::<String>(DEVICE_KEY))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| DEFAULT_DEVICE.to_string());

        let baudrate = config
            .and_then(|cfg| cfg.get::<u32>(BAUD_KEY))
            .unwrap_or(DEFAULT_BAUDRATE);

        let timeout_ms = config
            .and_then(|cfg| cfg.get::<u64>(TIMEOUT_KEY))
            .unwrap_or(DEFAULT_TIMEOUT_MS);

        std_serial::open(&device, baudrate, timeout_ms).map_err(|err| {
            CuError::from(format!(
                "MSP bridge failed to open serial `{device}` at {baudrate} baud: {err}"
            ))
        })
    }
}

/// Batch of MSP requests transported over the bridge.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MspRequestBatch(pub SmallVec<[MspRequest; MAX_REQUESTS_PER_BATCH]>);

impl MspRequestBatch {
    pub fn new() -> Self {
        Self(SmallVec::new())
    }

    pub fn push(&mut self, req: MspRequest) {
        let Self(ref mut vec) = self;
        vec.push(req);
    }

    pub fn iter(&self) -> impl Iterator<Item = &MspRequest> {
        let Self(ref vec) = self;
        vec.iter()
    }
}

impl Encode for MspRequestBatch {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl Decode<()> for MspRequestBatch {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let values = <Vec<MspRequest> as Decode<()>>::decode(decoder)?;
        Ok(Self(values.into()))
    }
}

/// Batch of MSP responses collected by the bridge.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MspResponseBatch(pub SmallVec<[MspResponse; MAX_RESPONSES_PER_BATCH]>);

impl MspResponseBatch {
    pub fn new() -> Self {
        Self(SmallVec::new())
    }

    pub fn push(&mut self, resp: MspResponse) {
        let Self(ref mut vec) = self;
        vec.push(resp);
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl Encode for MspResponseBatch {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl Decode<()> for MspResponseBatch {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let values = <Vec<MspResponse> as Decode<()>>::decode(decoder)?;
        Ok(Self(values.into()))
    }
}

tx_channels! {
    pub struct TxChannels : TxId {
        requests => MspRequestBatch,
    }
}

rx_channels! {
    pub struct RxChannels : RxId {
        responses => MspResponseBatch,
    }
}

/// Bridge that multiplexes MSP traffic on a single serial link.
pub struct CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E>,
{
    serial: S,
    parser: MspParser,
    read_buffer: [u8; READ_BUFFER_SIZE],
    pending_responses: MspResponseBatch,
    tx_buffer: SmallVec<[u8; 256]>,
}

impl<S, E> CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E>,
{
    fn from_serial(serial: S) -> Self {
        Self {
            serial,
            parser: MspParser::new(),
            read_buffer: [0; READ_BUFFER_SIZE],
            pending_responses: MspResponseBatch::new(),
            tx_buffer: SmallVec::new(),
        }
    }

    fn send_request(&mut self, request: &MspRequest) -> CuResult<()> {
        let packet: MspPacket = request.into();
        let size = packet.packet_size_bytes();
        self.tx_buffer.resize(size, 0);
        packet.serialize(&mut self.tx_buffer).map_err(|err| {
            CuError::new_with_cause("MSP bridge failed to serialize request", err)
        })?;
        self.serial
            .write_all(&self.tx_buffer)
            .map_err(|_| CuError::from("MSP bridge failed to write serial"))
    }

    fn poll_serial(&mut self) -> CuResult<()> {
        loop {
            match self.serial.read(&mut self.read_buffer) {
                Ok(0) => break,
                Ok(n) => {
                    for &byte in &self.read_buffer[..n] {
                        match self.parser.parse(byte) {
                            Ok(Some(packet)) => {
                                let response = MspResponse::from(packet);
                                self.pending_responses.push(response);
                                if self.pending_responses.0.len() >= MAX_RESPONSES_PER_BATCH {
                                    break;
                                }
                            }
                            Ok(None) => {}
                            Err(err) => {
                                error!("MSP bridge parser error: {}", err.to_string());
                            }
                        }
                    }
                }
                Err(_) => break,
            }
        }
        Ok(())
    }
}

#[cfg(not(feature = "std"))]
impl<S, E> CuMspBridge<S, E>
where
    S: SerialFactory<E>,
{
    /// Register a serial port for use with MSP bridge in no-std environments
    pub fn register_serial(slot: usize, serial_port: S) -> CuResult<()> {
        cu_embedded_registry::register(slot, serial_port)
    }
}

impl<S, E> Freezable for CuMspBridge<S, E> where S: Write<Error = E> + Read<Error = E> {}

impl<S, E> CuBridge for CuMspBridge<S, E>
where
    S: SerialFactory<E>,
{
    type Tx = TxChannels;
    type Rx = RxChannels;

    fn new(
        config: Option<&ComponentConfig>,
        tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let _ = tx_channels;
        let _ = rx_channels;
        Ok(Self::from_serial(S::try_new(config)?))
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.poll_serial()
    }

    fn send<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id() {
            TxId::Requests => {
                let request_msg: &CuMsg<MspRequestBatch> = msg.downcast_ref()?;
                if let Some(batch) = request_msg.payload() {
                    for request in batch.iter() {
                        self.send_request(request)?;
                    }
                }
            }
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id() {
            RxId::Responses => {
                let response_msg: &mut CuMsg<MspResponseBatch> = msg.downcast_mut()?;
                let mut batch = MspResponseBatch::new();
                mem::swap(&mut batch, &mut self.pending_responses);
                response_msg.set_payload(batch);
            }
        }
        Ok(())
    }
}

#[cfg(feature = "std")]
pub mod std_serial {
    use embedded_io_adapters::std::FromStd;
    #[cfg(feature = "std")]
    use std::boxed::Box;
    #[cfg(feature = "std")]
    use std::time::Duration;

    pub type StdSerial = FromStd<Box<dyn serialport::SerialPort>>;

    pub fn open(path: &str, baud: u32, timeout_ms: u64) -> std::io::Result<StdSerial> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(timeout_ms))
            .open()?;
        Ok(FromStd::new(port))
    }
}

/// Type alias for MSP bridge using standard I/O (for backward compatibility)
#[cfg(feature = "std")]
pub type CuMspBridgeStd = CuMspBridge<std_serial::StdSerial, std::io::Error>;
