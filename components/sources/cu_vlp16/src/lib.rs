use velodyne_lidar::{Config, Config16};
use velodyne_lidar::{DataPacket, Packet};

use cu29::clock::RobotClock;
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuSrcTask, Freezable};
use cu29::{output_msg, CuResult};
use cu_sensor_payloads::{PointCloud, PointCloudSoa};
use std::net::UdpSocket;
use std::time::Duration;
use velodyne_lidar::iter::try_packet_to_frame_xyz;
use velodyne_lidar::types::frame_xyz::FrameXyz;

pub struct Vlp16 {
    velo_config: Config,
    listen_addr: String,
    #[allow(dead_code)]
    test_mode: bool,
    socket: Option<UdpSocket>,
}

impl Freezable for Vlp16 {}

impl<'cl> CuSrcTask<'cl> for Vlp16 {
    type Output = output_msg!('cl, PointCloudSoa<10000>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config: &ComponentConfig = config.expect("Vlp16 requires a config");
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

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let socket = self.socket.as_ref().unwrap();
        let mut packet = [0u8; size_of::<DataPacket>()];
        let (read_size, _peer_addr) = socket.recv_from(&mut packet).unwrap();
        let packet = &packet[..read_size];

        let packet = Packet::from_slice(packet).unwrap();
        let packets = [Ok::<Packet, ()>(packet)].into_iter(); // 🤮
        let frame: FrameXyz = try_packet_to_frame_xyz(self.velo_config.clone(), packets)
            .unwrap()
            .next()
            .unwrap()
            .unwrap();
        let mut output = PointCloudSoa::<10000>::default();

        frame.firing_iter().for_each(|firing| {
            firing.point_iter().for_each(|point| {
                let point = point.as_single().unwrap();
                let tov = point.toh; // FIXME: Needs to sync the clocks here.
                let x = point.measurement.xyz[0].as_meters() as f32;
                let y = point.measurement.xyz[1].as_meters() as f32;
                let z = point.measurement.xyz[2].as_meters() as f32;
                let intensity = point.measurement.intensity as f32 / 255.0f32;
                output.push(PointCloud::new(tov.into(), x, y, z, intensity, None));
            });
        });
        new_msg.set_payload(output);
        Ok(())
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.socket = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_udp_inject::PcapStreamer;

    #[test]
    fn vlp16_end_2_end_test() {
        let clk = RobotClock::new();
        let cfg = ComponentConfig::new();
        let mut drv = Vlp16::new(Some(&cfg)).unwrap();

        let mut streamer = PcapStreamer::new("test/VLP_16_Single.pcap", "127.0.0.1:2368");

        drv.start(&clk).unwrap();

        const PACKET_SIZE: usize = size_of::<DataPacket>();

        // Read test.pcap
        if streamer
            .send_next::<PACKET_SIZE>()
            .expect("Failed to send packet")
        {
            let mut msg = CuMsg::new(Some(PointCloudSoa::<10000>::default()));
            drv.process(&clk, &mut msg).unwrap();
            assert_eq!(-0.05115497, msg.payload().unwrap().x[0].value);
        }
        drv.stop(&clk).unwrap();
    }
}
