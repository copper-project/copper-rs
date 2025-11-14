#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

mod messages;

pub use spin::{Mutex, Once};

use crate::messages::{LinkStatisticsPayload, RcChannelsPayload};
#[cfg(not(feature = "std"))]
use alloc::format;
use crsf::{LinkStatistics, Packet, PacketAddress, PacketParser, RcChannels};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::debug;
use cu29::prelude::*;
use embedded_io::{Read, Write};
#[cfg(feature = "std")]
use std::string::String;

const READ_BUFFER_SIZE: usize = 1024;
const PARSER_BUFFER_SIZE: usize = 1024;

#[cfg(feature = "std")]
const SERIAL_PATH_KEY: &str = "serial_path";
#[cfg(feature = "std")]
const SERIAL_BAUD_KEY: &str = "baudrate";
#[cfg(feature = "std")]
const SERIAL_TIMEOUT_KEY: &str = "timeout_ms";
#[cfg(not(feature = "std"))]
const SERIAL_INDEX_KEY: &str = "serial_port_index";

#[cfg(not(feature = "std"))]
pub mod serial_registry {
    use super::*;
    use alloc::boxed::Box;
    use core::any::{Any, TypeId};

    type SerialEntry = Box<dyn Any + Send>;

    struct SerialSlot {
        type_id: TypeId,
        entry: SerialEntry,
    }

    pub(super) const MAX_SERIAL_SLOTS: usize = 4;

    struct SerialSlots {
        entries: [Option<SerialSlot>; MAX_SERIAL_SLOTS],
    }

    impl SerialSlots {
        const fn new() -> Self {
            Self {
                entries: [None, None, None, None],
            }
        }
    }

    static SERIAL_SLOTS: Mutex<SerialSlots> = Mutex::new(SerialSlots::new());

    pub fn register<S, E>(slot: usize, serial: S) -> CuResult<()>
    where
        S: Write<Error = E> + Read<Error = E> + Send + 'static,
    {
        if slot >= MAX_SERIAL_SLOTS {
            return Err(CuError::from(format!(
                "CRSF serial registry slot {slot} out of range (max {MAX_SERIAL_SLOTS})"
            )));
        }
        let mut slots = SERIAL_SLOTS.lock();
        slots.entries[slot] = Some(SerialSlot {
            type_id: TypeId::of::<S>(),
            entry: Box::new(serial),
        });
        Ok(())
    }

    pub fn take<S, E>(slot: usize) -> Option<S>
    where
        S: Write<Error = E> + Read<Error = E> + Send + 'static,
    {
        if slot >= MAX_SERIAL_SLOTS {
            return None;
        }
        let mut slots = SERIAL_SLOTS.lock();
        let record = slots.entries[slot].take()?;
        if record.type_id != TypeId::of::<S>() {
            slots.entries[slot] = Some(record);
            return None;
        }
        record.entry.downcast::<S>().map(|boxed| *boxed).ok()
    }
}

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

        serial_registry::take(slot).ok_or_else(|| {
            CuError::from(format!(
                "CRSF bridge missing serial for slot {slot} (max index {})",
                serial_registry::MAX_SERIAL_SLOTS - 1
            ))
        })
    }
}

#[cfg(feature = "std")]
impl SerialFactory<std::io::Error> for std_serial::StdSerial {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let cfg = config.ok_or_else(|| {
            CuError::from("CRSF bridge requires configuration with serial parameters")
        })?;

        let path = cfg
            .get::<String>(SERIAL_PATH_KEY)
            .ok_or_else(|| CuError::from("CRSF bridge config missing `serial_path` entry"))?;
        let baud = cfg.get::<u32>(SERIAL_BAUD_KEY).unwrap_or(420_000);
        let timeout_ms = cfg.get::<u32>(SERIAL_TIMEOUT_KEY).unwrap_or(100) as u64;

        std_serial::open(&path, baud, timeout_ms).map_err(|err| {
            CuError::from(format!(
                "Failed to open serial `{path}` at {baud} baud: {err}"
            ))
        })
    }
}

rx_channels! {
    lq_rx => LinkStatisticsPayload,
    rc_rx => RcChannelsPayload
    // TODO(gbin): add other types
}

tx_channels! {
    lq_tx => LinkStatisticsPayload,
    rc_tx => RcChannelsPayload
}

/// Crossfire bridge for Copper-rs.
pub struct CuCrsfBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E>,
{
    serial_port: S,
    parser: PacketParser<PARSER_BUFFER_SIZE>,
    last_lq: Option<LinkStatistics>,
    last_rc: Option<RcChannels>,
}

impl<S, E> CuCrsfBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E>,
{
    fn from_serial(serial_port: S) -> Self {
        Self {
            serial_port,
            parser: PacketParser::<PARSER_BUFFER_SIZE>::new(),
            last_lq: None,
            last_rc: None,
        }
    }

    // decode from the serial buffer and update to the last received values
    fn update(&mut self) -> CuResult<()> {
        let mut buf = [0; READ_BUFFER_SIZE];
        match self.serial_port.read(buf.as_mut_slice()) {
            Ok(n) => {
                if n > 0 {
                    self.parser.push_bytes(&buf[..n]);
                    while let Some(Ok((_, packet))) = self.parser.next_packet() {
                        match packet {
                            Packet::LinkStatistics(link_statistics) => {
                                debug!(
                                    "LinkStatistics: Download LQ:{}",
                                    link_statistics.downlink_link_quality
                                );
                                self.last_lq = Some(link_statistics);
                            }
                            Packet::RcChannels(channels) => {
                                self.last_rc = Some(channels);
                                for (i, value) in self.last_rc.iter().enumerate() {
                                    debug!("RC Channel {}: {}", i, value.as_ref());
                                }
                            }
                            _ => {
                                info!("CRSF: Received other packet");
                            }
                        }
                    }
                }
            }
            _ => {
                error!("CRSF: Serial port read error");
            }
        }
        Ok(())
    }
}

#[cfg(not(feature = "std"))]
impl<S, E> CuCrsfBridge<S, E>
where
    S: SerialFactory<E>,
{
    pub fn register_serial(slot: usize, serial_port: S) -> CuResult<()> {
        serial_registry::register(slot, serial_port)
    }
}

impl<S, E> Freezable for CuCrsfBridge<S, E> where S: Write<Error = E> + Read<Error = E> {}

impl<S, E> CuBridge for CuCrsfBridge<S, E>
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
            TxId::LqTx => {
                let lsi: &CuMsg<LinkStatisticsPayload> = msg.downcast_ref()?;
                if let Some(lq) = lsi.payload() {
                    debug!(
                        "CRSF: Sent LinkStatistics: Downlink LQ:{}",
                        lq.0.downlink_link_quality
                    );
                    let ls = Packet::LinkStatistics(lq.0.clone());
                    let raw_packet = ls.into_raw(PacketAddress::Transmitter);
                    self.serial_port
                        .write_all(raw_packet.data())
                        .map_err(|_| CuError::from("CRSF: Serial port write error"))?;
                }
            }
            TxId::RcTx => {
                let rccs: &CuMsg<RcChannelsPayload> = msg.downcast_ref()?;
                if let Some(rc) = rccs.payload() {
                    for (i, value) in rc.0.iter().enumerate() {
                        debug!("Sending RC Channel {}: {}", i, value);
                    }
                    let rc = Packet::RcChannels(rc.0.clone());
                    let raw_packet = rc.into_raw(PacketAddress::Transmitter);
                    self.serial_port
                        .write_all(raw_packet.data())
                        .map_err(|_| CuError::from("CRSF: Serial port write error"))?;
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
        self.update()?;
        match channel.id() {
            RxId::LqRx => {
                if let Some(lq) = self.last_lq.as_ref() {
                    let lqp = LinkStatisticsPayload::from(lq.clone());
                    let lq_msg: &mut CuMsg<LinkStatisticsPayload> = msg.downcast_mut()?;
                    lq_msg.set_payload(lqp);
                }
            }
            RxId::RcRx => {
                if let Some(rc) = self.last_rc.as_ref() {
                    let rc = RcChannelsPayload::from(rc.clone());
                    let rc_msg: &mut CuMsg<RcChannelsPayload> = msg.downcast_mut()?;
                    rc_msg.set_payload(rc);
                }
            }
        }
        Ok(())
    }
}

#[cfg(feature = "std")]
pub mod std_serial {
    use embedded_io_adapters::std::FromStd;
    use std::boxed::Box;
    use std::time::Duration;

    pub type StdSerial = FromStd<Box<dyn serialport::SerialPort>>;

    pub fn open(path: &str, baud: u32, timeout_ms: u64) -> std::io::Result<StdSerial> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(timeout_ms))
            .open()?;
        Ok(FromStd::new(port))
    }
}
