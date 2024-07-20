use velodyne_lidar::{Config, Config16, DataPacket};
use velodyne_lidar::Packet;

use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::time::Duration;
use bincode::enc::Encoder;
use bincode::error::EncodeError;
use velodyne_lidar::iter::try_packet_to_frame_xyz;
use velodyne_lidar::types::frame_xyz::FrameXyz;
use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::CuResult;
use cu29::cutask::{CuTaskLifecycle, CuSrcTask, Freezable, CuMsg};

const MAX_UDP_PACKET_SIZE: usize = 65507;

pub struct Vlp16 {
    velo_config: Config,
    listen_addr: String,
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
        let listen_addr: String = config.get("listen_addr").unwrap_or("0.0.0.0:2368".to_string());
        let return_type: String = config.get("return_type").unwrap_or("last".to_string());
        let test_mode: String = config.get("test_mode").unwrap_or("false".to_string());
        let velo_config = match return_type.as_str() {
            "strongest" => Config16::new_vlp_16_strongest(),
            "last" => Config16::new_vlp_16_last(),
            "dual" => Config16::new_vlp_16_dual(),
            _ => { return Err("Invalid return_type for Vlp16.".into()); }
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
        socket.set_read_timeout(Some(Duration::from_millis(100))).unwrap();
        self.socket = Some(socket);
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.socket = None;
        Ok(())
    }
}

struct XYZ {
    x: uom::si::f32::Length,
    y: uom::si::f32::Length,
    z: uom::si::f32::Length,
}

impl CuSrcTask for Vlp16 {
    type Output = ();

    fn process(&mut self, clock: &RobotClock, new_msg: &mut CuMsg<Self::Output>) -> CuResult<()> {
        let socket = self.socket.as_ref().unwrap();
        let mut packet = [0u8; 1206];
        let (read_size, peer_addr) = socket.recv_from(&mut packet).unwrap();
        let packet = &packet[..read_size];
        let packet = Packet::from_slice(packet).unwrap();
        let packets = [Ok::<Packet, ()>(packet)].into_iter(); // 🤮
        let frame:FrameXyz =  try_packet_to_frame_xyz(self.velo_config.clone(), packets).unwrap().next().unwrap().unwrap();

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert!(true);
    }
}
