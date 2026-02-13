//! Copper bridge that multiplexes MSP traffic in both directions over a single serial port.
//! The type exposes a `requests` Tx channel that accepts [`MspRequestBatch`] messages and a
//! `responses` Rx channel that yields [`MspResponseBatch`] payloads decoded from the line.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

// std implementation
#[cfg(feature = "std")]
mod std_impl {
    pub use std::{mem, vec::Vec};
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
use cu29::resource::{Owned, ResourceBindings, ResourceManager};
use embedded_io::{Read, Write};
use heapless::Vec as HeaplessVec;
use serde::{Deserialize, Serialize};

const READ_BUFFER_SIZE: usize = 512;
const MAX_REQUESTS_PER_BATCH: usize = 8;
const MAX_RESPONSES_PER_BATCH: usize = 16;
const TX_BUFFER_CAPACITY: usize = MSP_MAX_PAYLOAD_LEN + 12;

fn decode_bounded_vec<T, const N: usize, D>(
    decoder: &mut D,
) -> Result<HeaplessVec<T, N>, DecodeError>
where
    T: Decode<()>,
    D: Decoder<Context = ()>,
{
    let values = <Vec<T> as Decode<()>>::decode(decoder)?;
    let count = values.len();
    if count > N {
        return Err(DecodeError::ArrayLengthMismatch {
            required: N,
            found: count,
        });
    }
    let mut batch = HeaplessVec::new();
    for value in values {
        batch
            .push(value)
            .map_err(|_| DecodeError::ArrayLengthMismatch {
                required: N,
                found: count,
            })?;
    }
    Ok(batch)
}

/// Batch of MSP requests transported over the bridge.
#[derive(Debug, Clone, Default, Serialize, Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct MspRequestBatch(pub HeaplessVec<MspRequest, MAX_REQUESTS_PER_BATCH>);

impl MspRequestBatch {
    pub fn new() -> Self {
        Self(HeaplessVec::new())
    }

    pub fn push(&mut self, req: MspRequest) -> CuResult<()> {
        self.0
            .push(req)
            .map_err(|_| CuError::from("MSP request batch overflow"))
    }

    pub fn iter(&self) -> impl Iterator<Item = &MspRequest> {
        self.0.iter()
    }
}

impl Encode for MspRequestBatch {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.as_slice(), encoder)
    }
}

impl Decode<()> for MspRequestBatch {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        decode_bounded_vec::<MspRequest, MAX_REQUESTS_PER_BATCH, _>(decoder).map(MspRequestBatch)
    }
}

/// Batch of MSP responses collected by the bridge.
#[derive(Debug, Clone, Default, Serialize, Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct MspResponseBatch(pub HeaplessVec<MspResponse, MAX_RESPONSES_PER_BATCH>);

impl MspResponseBatch {
    pub fn new() -> Self {
        Self(HeaplessVec::new())
    }

    pub fn push(&mut self, resp: MspResponse) -> CuResult<()> {
        self.0
            .push(resp)
            .map_err(|_| CuError::from("MSP response batch overflow"))
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
        decode_bounded_vec::<MspResponse, MAX_RESPONSES_PER_BATCH, _>(decoder).map(MspResponseBatch)
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
        incoming => MspRequestBatch,
    }
}

/// Bridge that multiplexes MSP traffic on a single serial link.
#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
    E: 'static,
{
    #[reflect(ignore)]
    serial: S,
    #[reflect(ignore)]
    parser: MspParser,
    #[reflect(ignore)]
    read_buffer: [u8; READ_BUFFER_SIZE],
    #[reflect(ignore)]
    pending_responses: MspResponseBatch,
    #[reflect(ignore)]
    pending_requests: MspRequestBatch,
    #[reflect(ignore)]
    tx_buffer: HeaplessVec<u8, TX_BUFFER_CAPACITY>,
}

impl<S, E> CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
    E: 'static,
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
            if self.pending_responses.0.len() >= MAX_RESPONSES_PER_BATCH
                || self.pending_requests.0.len() >= MAX_REQUESTS_PER_BATCH
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
                if self.pending_responses.0.len() >= MAX_RESPONSES_PER_BATCH
                    || self.pending_requests.0.len() >= MAX_REQUESTS_PER_BATCH
                {
                    break;
                }
                if let Ok(Some(packet)) = self.parser.parse(byte) {
                    if packet.direction == MspPacketDirection::ToFlightController {
                        // This is an incoming request from the VTX
                        if let Some(request) = MspRequest::from_packet(&packet) {
                            self.pending_requests.push(request)?;
                        }
                    } else {
                        // This is a response from the VTX
                        let response = MspResponse::from(packet);
                        self.pending_responses.push(response)?;
                    }
                }
            }
        }
        Ok(())
    }
}

impl<S, E> Freezable for CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
    E: 'static,
{
}

impl<S, E> cu29::reflect::TypePath for CuMspBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
    E: 'static,
{
    fn type_path() -> &'static str {
        "cu_msp_bridge::CuMspBridge"
    }

    fn short_type_path() -> &'static str {
        "CuMspBridge"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuMspBridge")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_msp_bridge")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_msp_bridge")
    }
}

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
    E: 'static,
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

/// Type alias for MSP bridge using standard I/O (for backward compatibility)
#[cfg(feature = "std")]
pub type CuMspBridgeStd = CuMspBridge<cu_linux_resources::LinuxSerialPort, std::io::Error>;
