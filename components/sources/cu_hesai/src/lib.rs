pub mod parser;

use crate::parser::{RefTime, generate_default_elevation_calibration};
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use chrono::Utc;
use cu_sensor_payloads::{PointCloud, PointCloudSoa};
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f32::{Angle, Length};
use cu29::units::si::length::meter;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io::ErrorKind;
use std::io::Read;
use std::net::SocketAddr;
use std::ops::{Deref, DerefMut};

/// By default, Hesai broadcasts on this address.
const DEFAULT_ADDR: &str = "0.0.0.0:2368";

/// Convert spherical coordinates to cartesian coordinates.
/// With the physics-style spherical coordinate convention / right-handed coordinate system.
fn spherical_to_cartesian(
    azimuth: Angle,
    elevation: Angle,
    distance: Length,
) -> (Length, Length, Length) {
    let distance_m = distance.get::<meter>();
    let azimuth_rad = azimuth.get::<radian>();
    let elevation_rad = elevation.get::<radian>();

    let x = Length::new::<meter>(distance_m * elevation_rad.cos() * azimuth_rad.cos());
    let y = Length::new::<meter>(distance_m * elevation_rad.cos() * azimuth_rad.sin());
    let z = Length::new::<meter>(distance_m * elevation_rad.sin());
    (x, y, z)
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct Xt32 {
    #[reflect(ignore)]
    socket: Socket,
    #[reflect(ignore)]
    reftime: RefTime,
    #[reflect(ignore)]
    channel_elevations: [Angle; 32],
}

impl Xt32 {
    /// This give the matching timestamps between the UTC time to the Robot time.
    fn sync(&mut self, robot_clock: &RobotClock) {
        self.reftime = (Utc::now(), robot_clock.now());
    }
}

impl Freezable for Xt32 {}

const MAX_POINTS: usize = 32 * 10;

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

// In each round of firing, the firing sequence is from Channel 1 to Channel 32.
// Assuming that the start time of Block 6 is t6, the laser firing time of Channel i is
// t6 + [1.512µs * (i-1) + 0.28µs ], i∈{1, 2, ..., 32}.
// in copper resolution as 1ns
// t6 + 1512ns * (i-1) + 280ns
fn channel_time(t6: CuTime, i: u64) -> CuTime {
    if i == 0 {
        CuDuration(t6.0 - 1512 + 280) // this is an underflow, so we just subtract the value
    } else {
        CuDuration(t6.0 + 1512 * (i - 1) + 280)
    }
}

impl CuSrcTask for Xt32 {
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
        Ok(Xt32 {
            socket,
            reftime: rt,
            channel_elevations: generate_default_elevation_calibration(), // TODO: make the config able to override that
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
        let lidar_packet = parser::parse_packet(&buf[..size])
            .map_err(|e| CuError::new_with_cause("Failed to parse Hesai UDP packet", e))?;
        // this is the reference point for the block timings
        let t6 = lidar_packet
            .block_ts(&self.reftime)
            .map_err(|e| CuError::new_with_cause("Failed to get block timings", e))?[5]; // 0 == channel 1, 5 == channel 6

        let mut min_tov = CuTime::MAX;
        let mut max_tov = CuTime::MIN;

        // let is_dual = lidar_packet.header.is_dual_return(); TODO: add dual return support
        for block in lidar_packet.blocks.iter() {
            let azimuth = block.azimuth();
            block.channels.iter().enumerate().for_each(|(i, c)| {
                let elevation = self.channel_elevations[i];
                let d = c.distance();
                let r = c.reflectivity();

                let (x, y, z) = spherical_to_cartesian(azimuth, elevation, d);
                let t = channel_time(t6, i as u64);
                // TODO: we can precompute that from the packet itself in one shot.
                if t < min_tov {
                    min_tov = t;
                } else if t > max_tov {
                    max_tov = t;
                }
                payload.push(PointCloud::new_units(t, x, y, z, r, None));
            });
        }

        let tov_range = CuTimeRange {
            start: min_tov,
            end: max_tov,
        };
        new_msg.tov = Tov::Range(tov_range); // take the oldest timestamp
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::Packet;
    use chrono::DateTime;
    use cu_udp_inject::PcapStreamer;
    use cu29::cutask::CuMsg;

    #[test]
    fn test_xt32() {
        let clock = RobotClock::new();
        let mut streamer = PcapStreamer::new("tests/hesai-xt32-small.pcap", "127.0.0.1:2368");
        let config = ComponentConfig::new();

        let mut xt32 = Xt32::new(Some(&config), ()).unwrap();

        let new_payload = LidarCuMsgPayload::default();
        let mut new_msg = CuMsg::<LidarCuMsgPayload>::new(Some(new_payload));

        // Picking a timestamp from the beginning of the pcap file to align the robot clock with the capture + 1s buffer in the past because ref times are negative.
        let datetime = DateTime::parse_from_rfc3339("2024-09-17T15:47:11.684855Z")
            .unwrap()
            .with_timezone(&Utc);

        xt32.reftime = (datetime, clock.now());

        // 1076 is the expected payload size for Hesai XT32
        const PACKET_SIZE: usize = size_of::<Packet>();
        while streamer
            .send_next::<PACKET_SIZE>()
            .expect("Failed to send next packet")
        {
            let err = xt32.process(&clock, &mut new_msg);
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
