pub mod parser;

use crate::parser::RefTime;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use chrono::Utc;
use cu_sensor_payloads::{PointCloud, PointCloudSoa};
use cu29::prelude::*;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io::{ErrorKind, Read};
use std::net::SocketAddr;
use std::ops::{Deref, DerefMut};
const DEFAULT_ADDR: &str = "0.0.0.0:56001";

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct Tele15 {
    #[reflect(ignore)]
    socket: Socket,
    #[reflect(ignore)]
    reftime: RefTime,
}

impl Tele15 {
    /// This give the matching timestamps between the UTC time to the Robot time.
    fn sync(&mut self, robot_clock: &RobotClock) {
        self.reftime = (Utc::now(), robot_clock.now());
    }
}

impl Freezable for Tele15 {}

const MAX_POINTS: usize = 100;

#[derive(Default, Clone, Debug, serde::Serialize, serde::Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct LidarCuMsgPayload(pub PointCloudSoa<MAX_POINTS>);

impl Deref for LidarCuMsgPayload {
    type Target = PointCloudSoa<MAX_POINTS>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for LidarCuMsgPayload {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Encode for LidarCuMsgPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.0.encode(encoder)
    }
}

impl Decode<()> for LidarCuMsgPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self(PointCloudSoa::<MAX_POINTS>::decode(decoder)?))
    }
}

impl CuSrcTask for Tele15 {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(LidarCuMsgPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let addr: SocketAddr = if let Some(cfg) = config {
            let addr_str = cfg
                .get::<String>("socket_addr")?
                .unwrap_or(DEFAULT_ADDR.to_string());
            addr_str.as_str().parse().unwrap()
        } else {
            DEFAULT_ADDR.parse().unwrap()
        };

        let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).unwrap();
        socket.bind(&SockAddr::from(addr)).unwrap();
        socket.set_nonblocking(true).unwrap();

        // just a temporary value, it will be redone at start.
        let rt: RefTime = (Utc::now(), RobotClock::new().now());
        Ok(Tele15 {
            socket,
            reftime: rt,
        })
    }
    fn start(&mut self, robot_clock: &RobotClock) -> CuResult<()> {
        self.sync(robot_clock);
        Ok(())
    }
    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let payload = new_msg.payload_mut().insert(LidarCuMsgPayload::default());
        let mut buf = [0u8; 1500];
        let size = match self.socket.read(&mut buf) {
            Ok(size) => size,
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                // Handle no data available (non-blocking behavior)
                new_msg.clear_payload();
                return Ok(());
            }
            Err(e) => return Err(CuError::new_with_cause("IO Error on UDP socket", e)), // Handle other errors
        };
        let lidar_packet = parser::parse_frame(&buf[..size])
            .map_err(|e| CuError::new_with_cause("Failed to parse Livox UDP packet", e))?;

        // let is_dual = lidar_packet.header.is_dual_return(); TODO: add dual return support
        for pt in lidar_packet.points.iter() {
            payload.push(PointCloud::new_uom(
                lidar_packet.header.timestamp(),
                pt.x(),
                pt.y(),
                pt.z(),
                pt.reflectivity(),
                None,
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::LidarFrame;
    use chrono::DateTime;
    use cu_udp_inject::PcapStreamer;
    use cu29::cutask::CuMsg;

    #[test]
    fn test_tele15() {
        let clock = RobotClock::new();
        let mut streamer = PcapStreamer::new("tests/livox_tele15_small.pcap", "127.0.0.1:56001");
        let config = ComponentConfig::new();

        let mut tele15 = Tele15::new(Some(&config), ()).unwrap();

        let new_payload = LidarCuMsgPayload::default();
        let mut new_msg = CuMsg::<LidarCuMsgPayload>::new(Some(new_payload));

        // Picking a timestamp from the beginning of the pcap file to align the robot clock with the capture + 1s buffer in the past because ref times are negative.
        let datetime = DateTime::parse_from_rfc3339("2024-09-17T15:47:11.684855Z")
            .unwrap()
            .with_timezone(&Utc);

        tele15.reftime = (datetime, clock.now());
        const PACKET_SIZE: usize = size_of::<LidarFrame>();
        while streamer
            .send_next::<PACKET_SIZE>()
            .expect("Failed to send next packet")
        {
            let err = tele15.process(&clock, &mut new_msg);
            if let Err(e) = err {
                println!("Error: {e:?}");
                continue;
            }
            if let Some(payload) = new_msg.payload() {
                println!("Lidar Payload: {payload:?}");
            }
            break;
        }
    }
}
