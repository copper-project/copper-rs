use velodyne_lidar::Packet;
use velodyne_lidar::{Config, Config16};

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle, Freezable};
use cu29::{output_msg, CuResult};
use cu29_soa_derive::soa;
use std::net::UdpSocket;
use std::ops::{Add, Sub};
use std::time::Duration;
use uom::si::f32::Length;
use uom::si::length::meter;
use velodyne_lidar::iter::try_packet_to_frame_xyz;
use velodyne_lidar::types::frame_xyz::FrameXyz;

// const MAX_UDP_PACKET_SIZE: usize = 65507;

pub struct Vlp16 {
    velo_config: Config,
    listen_addr: String,
    #[allow(dead_code)]
    test_mode: bool,
    socket: Option<UdpSocket>,
}

impl Freezable for Vlp16 {}

impl CuTaskLifecycle for Vlp16 {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config: &NodeInstanceConfig = config.expect("Vlp16 requires a config");
        let listen_addr: String = config
            .get("listen_addr")
            .unwrap_or("0.0.0.0:2368".to_string());
        let return_type: String = config.get("return_type").unwrap_or("last".to_string());
        let test_mode: String = config.get("test_mode").unwrap_or("false".to_string());
        let velo_config = match return_type.as_str() {
            "strongest" => Config16::new_vlp_16_strongest(),
            "last" => Config16::new_vlp_16_last(),
            "dual" => Config16::new_vlp_16_dual(),
            _ => {
                return Err("Invalid return_type for Vlp16.".into());
            }
        };
        Ok(Vlp16 {
            velo_config: velo_config.into(),
            listen_addr,
            test_mode: test_mode == "true",
            socket: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let socket = UdpSocket::bind(&self.listen_addr).unwrap();
        socket
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        self.socket = Some(socket);
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.socket = None;
        Ok(())
    }
}

#[derive(Default, PartialEq, Debug, Copy, Clone)]
struct LidarLength(Length);

impl Encode for LidarLength {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.value, encoder)
    }
}

impl Decode for LidarLength {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(LidarLength(Length::new::<meter>(value)))
    }
}

impl Add for LidarLength {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        LidarLength(self.0 + other.0)
    }
}

impl Sub for LidarLength {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        LidarLength(self.0 - other.0)
    }
}

#[soa]
#[derive(Default, PartialEq, Debug)]
pub struct XYZ {
    x: LidarLength,
    y: LidarLength,
    z: LidarLength,
}

impl XYZ {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x: LidarLength(Length::new::<meter>(x)),
            y: LidarLength(Length::new::<meter>(y)),
            z: LidarLength(Length::new::<meter>(z)),
        }
    }
}

impl CuSrcTask for Vlp16 {
    type Output = output_msg!(XYZSoa<10000>);

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let socket = self.socket.as_ref().unwrap();
        let mut packet = [0u8; 1206];
        let (read_size, _peer_addr) = socket.recv_from(&mut packet).unwrap();
        let packet = &packet[..read_size];

        let packet = Packet::from_slice(packet).unwrap();
        let packets = [Ok::<Packet, ()>(packet)].into_iter(); // 🤮
        let frame: FrameXyz = try_packet_to_frame_xyz(self.velo_config.clone(), packets)
            .unwrap()
            .next()
            .unwrap()
            .unwrap();
        let i = 0;
        let mut output = XYZSoa::<10000>::default();

        frame.firing_iter().for_each(|firing| {
            firing.point_iter().for_each(|point| {
                let point = point.as_single().unwrap();
                let x = point.measurement.xyz[0].as_meters() as f32;
                let y = point.measurement.xyz[0].as_meters() as f32;
                let z = point.measurement.xyz[0].as_meters() as f32;
                output.set(i, XYZ::new(x, y, z));
            });
        });
        new_msg.set_payload(output);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use pcap_file::pcap::PcapReader;
    use std::fs::File;

    use super::*;

    #[test]
    fn vlp16_end_2_end_test() {
        let clk = RobotClock::new();
        let cfg = NodeInstanceConfig::new();
        let mut drv = Vlp16::new(Some(&cfg)).unwrap();
        let file_in = File::open("test/VLP_16_Single.pcap").expect("Error opening file");
        let mut pcap_reader = PcapReader::new(file_in).unwrap();

        drv.start(&clk).unwrap();

        // Read test.pcap
        while let Some(pkt) = pcap_reader.next_packet() {
            let pkt = pkt.unwrap();
            let data = &pkt.data[0x2a..];
            // send udp packet to 2368
            let socket = UdpSocket::bind("0.0.0.0:2367").unwrap();
            socket.send_to(&data, "127.0.0.1:2368").unwrap();
            // process
            let mut msg = CuMsg::new(Some(XYZSoa::<10000>::default()));
            drv.process(&clk, &mut msg).unwrap();
            assert_eq!(0.009406593f32, msg.payload().unwrap().x[0].0.value);
            break;
        }
        drv.stop(&clk).unwrap();
    }
}
