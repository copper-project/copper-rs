#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

#[cfg(all(feature = "embedded-io-06", feature = "embedded-io-07"))]
compile_error!("Enable only one of `embedded-io-06` or `embedded-io-07` for cu-crsf.");

#[cfg(not(any(feature = "embedded-io-06", feature = "embedded-io-07")))]
compile_error!("Enable one of `embedded-io-06` or `embedded-io-07` for cu-crsf.");

#[cfg(all(feature = "std", feature = "embedded-io-07"))]
compile_error!(
    "The `std` feature requires `embedded-io-06`. \
     Use `embedded-io-07` only for no_std embedded targets."
);

#[cfg(feature = "embedded-io-06")]
use embedded_io_06 as embedded_io;
#[cfg(feature = "embedded-io-07")]
use embedded_io_07 as embedded_io;

pub mod messages;

use crate::messages::{LinkStatisticsPayload, RcChannelsPayload};
use crsf::{LinkStatistics, Packet, PacketAddress, PacketParser, RcChannels};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::*;
use cu29::resources;
use embedded_io::{Read, Write};

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

/// Convenience alias for std targets.
#[cfg(feature = "std")]
pub type CrsfBridgeStd = CrsfBridge<cu_linux_resources::LinuxSerialPort, std::io::Error>;
