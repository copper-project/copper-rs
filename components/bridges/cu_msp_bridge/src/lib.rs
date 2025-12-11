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
}

#[cfg(not(feature = "std"))]
use no_std_impl::*;
#[cfg(feature = "std")]
use std_impl::*;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::config::ResourceMapping;
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::*;
use cu29::resource::{Owned, ResourceBindings, ResourceBundle, ResourceManager};
use cu_msp_lib::structs::{MspRequest, MspResponse};
use cu_msp_lib::{MspPacket, MspParser};
use embedded_io::{ErrorType, Read, Write};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use spin::Mutex;

const READ_BUFFER_SIZE: usize = 512;
const MAX_REQUESTS_PER_BATCH: usize = 8;
const MAX_RESPONSES_PER_BATCH: usize = 16;

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

impl<S, E> Freezable for CuMspBridge<S, E> where S: Write<Error = E> + Read<Error = E> {}

pub struct MspResources<S> {
    pub serial: Owned<S>,
}

impl<'r, S, E> ResourceBindings<'r> for MspResources<S>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
{
    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceMapping>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| {
            CuError::from("MSP bridge requires a `serial` resource mapping in copperconfig")
        })?;
        let path = mapping.get("serial").ok_or_else(|| {
            CuError::from("MSP bridge resources must include `serial: <bundle.resource>`")
        })?;
        let serial = manager
            .take::<S>(path)
            .map_err(|e| e.add_cause("Failed to fetch MSP serial resource"))?;
        Ok(Self { serial })
    }
}

impl<S, E> CuBridge for CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
{
    type Resources<'r> = MspResources<S>;
    type Tx = TxChannels;
    type Rx = RxChannels;

    fn new(
        config: Option<&ComponentConfig>,
        tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let _ = tx_channels;
        let _ = rx_channels;
        let _ = config;
        Ok(Self::from_serial(resources.serial.0))
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

#[cfg(feature = "std")]
pub struct StdSerialBundle;

#[cfg(feature = "std")]
impl ResourceBundle for StdSerialBundle {
    fn build(
        bundle_id: &str,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let cfg =
            config.ok_or_else(|| CuError::from("MSP serial bundle requires configuration"))?;
        let device = cfg
            .get::<String>(DEVICE_KEY)
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| DEFAULT_DEVICE.to_string());
        let baudrate = cfg.get::<u32>(BAUD_KEY).unwrap_or(DEFAULT_BAUDRATE);
        let timeout_ms = cfg.get::<u64>(TIMEOUT_KEY).unwrap_or(DEFAULT_TIMEOUT_MS);

        let serial = std_serial::open(&device, baudrate, timeout_ms).map_err(|err| {
            CuError::from(format!(
                "MSP bridge failed to open serial `{device}` at {baudrate} baud: {err}"
            ))
        })?;
        manager.add_owned(format!("{bundle_id}.serial"), LockedSerial::new(serial))
    }
}

pub struct LockedSerial<T>(pub Mutex<T>);

impl<T> LockedSerial<T> {
    pub fn new(inner: T) -> Self {
        Self(Mutex::new(inner))
    }
}

impl<T: ErrorType> ErrorType for LockedSerial<T> {
    type Error = T::Error;
}

impl<T: Read> Read for LockedSerial<T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut guard = self.0.lock();
        guard.read(buf)
    }
}

impl<T: Write> Write for LockedSerial<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut guard = self.0.lock();
        guard.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        let mut guard = self.0.lock();
        guard.flush()
    }
}

/// Type alias for MSP bridge using standard I/O (for backward compatibility)
#[cfg(feature = "std")]
pub type CuMspBridgeStd = CuMspBridge<LockedSerial<std_serial::StdSerial>, std::io::Error>;
