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
    pub use alloc::vec::Vec;
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
use cu_msp_lib::structs::{MspRequest, MspResponse};
use cu_msp_lib::{MSP_MAX_PAYLOAD_LEN, MspPacket, MspPacketDirection, MspParser};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::*;
#[cfg(feature = "std")]
use cu29::resource::ResourceBundle;
use cu29::resource::{Owned, ResourceBindings, ResourceManager};
use embedded_io::{Read, Write};
use heapless::Vec as HeaplessVec;
use serde::{Deserialize, Serialize};

const READ_BUFFER_SIZE: usize = 512;
const MAX_REQUESTS_PER_BATCH: usize = 8;
const MAX_RESPONSES_PER_BATCH: usize = 16;
const TX_BUFFER_CAPACITY: usize = MSP_MAX_PAYLOAD_LEN + 12;

/// Batch of MSP messages transported over the bridge.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MspBatch<T, const N: usize>(pub HeaplessVec<T, N>);

impl<T, const N: usize> MspBatch<T, N> {
    pub fn new() -> Self {
        Self(HeaplessVec::new())
    }

    pub fn push(&mut self, value: T, overflow_msg: &'static str) -> CuResult<()> {
        self.0.push(value).map_err(|_| CuError::from(overflow_msg))
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.0.iter()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl<T, const N: usize> Encode for MspBatch<T, N>
where
    T: Encode,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl<T, const N: usize> Decode<()> for MspBatch<T, N>
where
    T: Decode<()>,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let values = <Vec<T> as Decode<()>>::decode(decoder)?;
        let count = values.len();
        if count > N {
            return Err(DecodeError::ArrayLengthMismatch {
                required: N,
                found: count,
            });
        }
        let mut batch = MspBatch::new();
        for value in values {
            batch
                .0
                .push(value)
                .map_err(|_| DecodeError::ArrayLengthMismatch {
                    required: N,
                    found: count,
                })?;
        }
        Ok(batch)
    }
}

/// Batch of MSP requests transported over the bridge.
pub type MspRequestBatch = MspBatch<MspRequest, MAX_REQUESTS_PER_BATCH>;

/// Batch of MSP responses collected by the bridge.
pub type MspResponseBatch = MspBatch<MspResponse, MAX_RESPONSES_PER_BATCH>;

tx_channels! {
    pub struct TxChannels : TxId {
        requests => MspRequestBatch,
    }
}

rx_channels! {
    pub struct RxChannels : RxId {
        responses => MspResponseBatch,
        incoming => MspRequestBatch,
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
    pending_requests: MspRequestBatch,
    tx_buffer: HeaplessVec<u8, TX_BUFFER_CAPACITY>,
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
            pending_requests: MspRequestBatch::new(),
            tx_buffer: HeaplessVec::new(),
        }
    }

    fn send_request(&mut self, request: &MspRequest) -> CuResult<()> {
        let packet: MspPacket = request.into();
        let size = packet.packet_size_bytes();
        self.tx_buffer
            .resize(size, 0)
            .map_err(|_| CuError::from("MSP bridge tx buffer too small"))?;
        packet
            .serialize(self.tx_buffer.as_mut_slice())
            .map_err(|err| {
                CuError::new_with_cause("MSP bridge failed to serialize request", err)
            })?;
        self.serial
            .write_all(self.tx_buffer.as_slice())
            .map_err(|_| CuError::from("MSP bridge failed to write serial"))
    }

    fn poll_serial(&mut self) -> CuResult<()> {
        loop {
            if self.pending_responses.len() >= MAX_RESPONSES_PER_BATCH
                || self.pending_requests.len() >= MAX_REQUESTS_PER_BATCH
            {
                break;
            }
            let Ok(n) = self.serial.read(&mut self.read_buffer) else {
                break;
            };
            if n == 0 {
                break;
            }
            for &byte in &self.read_buffer[..n] {
                if self.pending_responses.len() >= MAX_RESPONSES_PER_BATCH
                    || self.pending_requests.len() >= MAX_REQUESTS_PER_BATCH
                {
                    break;
                }
                if let Ok(Some(packet)) = self.parser.parse(byte) {
                    if packet.direction == MspPacketDirection::ToFlightController {
                        // This is an incoming request from the VTX
                        if let Some(request) = MspRequest::from_packet(&packet) {
                            self.pending_requests
                                .push(request, "MSP request batch overflow")?;
                        }
                    } else {
                        // This is a response from the VTX
                        let response = MspResponse::from(packet);
                        self.pending_responses
                            .push(response, "MSP response batch overflow")?;
                    }
                }
            }
        }
        Ok(())
    }
}

impl<S, E> Freezable for CuMspBridge<S, E> where S: Write<Error = E> + Read<Error = E> {}

pub struct MspResources<S> {
    pub serial: Owned<S>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    Serial,
}

impl<'r, S, E> ResourceBindings<'r> for MspResources<S>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
{
    type Binding = Binding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&cu29::resource::ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| {
            CuError::from("MSP bridge requires a `serial` resource mapping in copperconfig")
        })?;
        let path = mapping.get(Binding::Serial).ok_or_else(|| {
            CuError::from("MSP bridge resources must include `serial: <bundle.resource>`")
        })?;
        let serial = manager
            .take::<S>(path.typed())
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
        let bridge = Self::from_serial(resources.serial.0);
        Ok(bridge)
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
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        msg.tov = Tov::Time(clock.now());
        match channel.id() {
            RxId::Responses => {
                let response_msg: &mut CuMsg<MspResponseBatch> = msg.downcast_mut()?;
                let mut batch = MspResponseBatch::new();
                mem::swap(&mut batch, &mut self.pending_responses);
                response_msg.set_payload(batch);
            }
            RxId::Incoming => {
                let request_msg: &mut CuMsg<MspRequestBatch> = msg.downcast_mut()?;
                let mut batch = MspRequestBatch::new();
                mem::swap(&mut batch, &mut self.pending_requests);
                request_msg.set_payload(batch);
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
bundle_resources!(StdSerialBundle: Serial);

#[cfg(feature = "std")]
impl ResourceBundle for StdSerialBundle {
    fn build(
        bundle: cu29::resource::BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let cfg = config.ok_or_else(|| {
            CuError::from(format!(
                "MSP serial bundle `{}` requires configuration",
                bundle.bundle_id()
            ))
        })?;
        let device = cfg
            .get::<String>(DEVICE_KEY)?
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| DEFAULT_DEVICE.to_string());
        let baudrate = cfg.get::<u32>(BAUD_KEY)?.unwrap_or(DEFAULT_BAUDRATE);
        let timeout_ms = cfg.get::<u64>(TIMEOUT_KEY)?.unwrap_or(DEFAULT_TIMEOUT_MS);

        let serial = std_serial::open(&device, baudrate, timeout_ms).map_err(|err| {
            CuError::from(format!(
                "MSP bridge failed to open serial `{device}` at {baudrate} baud: {err}"
            ))
        })?;
        let key = bundle.key(StdSerialBundleId::Serial);
        manager.add_owned(key, cu_serial_util::LockedSerial::new(serial))
    }
}

/// Type alias for MSP bridge using standard I/O (for backward compatibility)
#[cfg(feature = "std")]
pub type CuMspBridgeStd =
    CuMspBridge<cu_serial_util::LockedSerial<std_serial::StdSerial>, std::io::Error>;
