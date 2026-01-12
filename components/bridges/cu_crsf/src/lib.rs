#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

pub mod messages;

pub use spin::Mutex;

// std implementation
#[cfg(feature = "std")]
mod std_impl {
    pub const SERIAL_PATH_KEY: &str = "serial_path";
    pub const SERIAL_BAUD_KEY: &str = "baudrate";
    pub const SERIAL_TIMEOUT_KEY: &str = "timeout_ms";
}

#[cfg(feature = "std")]
use std_impl::*;

use crate::messages::{LinkStatisticsPayload, RcChannelsPayload};
use crsf::{LinkStatistics, Packet, PacketAddress, PacketParser, RcChannels};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::*;
#[cfg(feature = "std")]
use cu29::resource::ResourceBundle;
use cu29::resources;
use embedded_io::{ErrorType, Read, Write};

const READ_BUFFER_SIZE: usize = 1024;
const PARSER_BUFFER_SIZE: usize = 1024;

rx_channels! {
    lq_rx => LinkStatisticsPayload,
    rc_rx => RcChannelsPayload
    // TODO(gbin): add other types
}

tx_channels! {
    lq_tx => LinkStatisticsPayload,
    rc_tx => RcChannelsPayload
}

resources!(for<S, E> where S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static {
    serial => Owned<S>,
});

/// Crossfire bridge for Copper-rs.
pub struct CrsfBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E>,
{
    serial_port: S,
    parser: PacketParser<PARSER_BUFFER_SIZE>,
    last_lq: Option<LinkStatistics>,
    last_rc: Option<RcChannels>,
}

impl<S, E> CrsfBridge<S, E>
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

impl<S, E> Freezable for CrsfBridge<S, E> where S: Write<Error = E> + Read<Error = E> {}

impl<S, E> CuBridge for CrsfBridge<S, E>
where
    S: Write<Error = E> + Read<Error = E> + Send + Sync + 'static,
{
    type Tx = TxChannels;
    type Rx = RxChannels;
    type Resources<'r> = Resources<S, E>;

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
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        self.update()?;
        msg.tov = Tov::Time(clock.now());
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
                "CRSF serial bundle `{}` requires configuration",
                bundle.bundle_id()
            ))
        })?;
        let path = cfg.get::<String>(SERIAL_PATH_KEY).ok_or_else(|| {
            CuError::from(format!(
                "CRSF serial bundle `{}` missing `serial_path` entry",
                bundle.bundle_id()
            ))
        })?;
        let baud = cfg.get::<u32>(SERIAL_BAUD_KEY).unwrap_or(420_000);
        let timeout_ms = cfg.get::<u32>(SERIAL_TIMEOUT_KEY).unwrap_or(100) as u64;

        let serial = std_serial::open(&path, baud, timeout_ms).map_err(|err| {
            CuError::from(format!(
                "Failed to open serial `{path}` at {baud} baud: {err}"
            ))
        })?;
        let key = bundle.key(StdSerialBundleId::Serial);
        manager.add_owned(key, LockedSerial::new(serial))
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

/// Convenience alias for std targets.
#[cfg(feature = "std")]
pub type CrsfBridgeStd = CrsfBridge<LockedSerial<std_serial::StdSerial>, std::io::Error>;
