pub mod parser;

use crate::parser::{generate_default_elevation_calibration, RefTime};
use chrono::Utc;
use cu29::prelude::*;
use cu_sensor_payloads::{PointCloud, PointCloudSoa};
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io::ErrorKind;
use std::io::Read;
use std::net::SocketAddr;
use uom::si::f32::{Angle, Length};

/// By default, Hesai broadcasts on this address.
const DEFAULT_ADDR: &str = "0.0.0.0:2368";

/// Convert spherical coordinates to cartesian coordinates.
/// With the physics-style spherical coordinate convention / right-handed coordinate system.
fn spherical_to_cartesian(
    azimuth: Angle,
    elevation: Angle,
    distance: Length,
) -> (Length, Length, Length) {
    let x = distance * elevation.cos() * azimuth.cos();
    let y = distance * elevation.cos() * azimuth.sin();
    let z = distance * elevation.sin();
    (x, y, z)
}

pub struct Xt32 {
    socket: Socket,
    reftime: RefTime,
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

pub type LidarCuMsgPayload = PointCloudSoa<MAX_POINTS>;

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

impl<'cl> CuSrcTask<'cl> for Xt32 {
    type Output = output_msg!('cl, LidarCuMsgPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let addr: SocketAddr = if let Some(cfg) = config {
            let addr_str = cfg.get("socket_addr").unwrap_or(DEFAULT_ADDR.to_string());
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
    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let payload = new_msg.payload_mut().insert(LidarCuMsgPayload::default());
        let mut buf = [0u8; 1500];
        match self.socket.read(&mut buf) {
            Ok(size) => {
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
                        payload.push(PointCloud::new_uom(t, x, y, z, r, None));
                    });
                }

                let tov_range = CuTimeRange {
                    start: min_tov,
                    end: max_tov,
                };
                new_msg.metadata.tov = Tov::Range(tov_range); // take the oldest timestamp
            }
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                // Handle no data available (non-blocking behavior)
                new_msg.clear_payload();
                return Ok(());
            }
            Err(e) => return Err(CuError::new_with_cause("IO Error on UDP socket", e)), // Handle other errors
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::Packet;
    use chrono::DateTime;
    use cu29::cutask::CuMsg;
    use cu_udp_inject::PcapStreamer;

    #[test]
    fn test_xt32() {
        let clock = RobotClock::new();
        let mut streamer = PcapStreamer::new("tests/hesai-xt32-small.pcap", "127.0.0.1:2368");
        let config = ComponentConfig::new();

        let mut xt32 = Xt32::new(Some(&config)).unwrap();

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
