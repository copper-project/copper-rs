use crate::parser::{generate_default_elevation_calibration, RefTime};
use chrono::Utc;
use cu29::config::ComponentConfig;
use cu29::cutask::CuMsg;
use cu29::cutask::{CuSrcTask, CuTaskLifecycle, Freezable};
use cu29::{output_msg, CuError, CuResult};
use cu29_clock::{CuDuration, CuTime, RobotClock};
use cu_sensor_payloads::{LidarPayload, LidarPayloadSoa};
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io::ErrorKind;
use std::io::Read;
use std::net::SocketAddr;

pub mod parser;

const DEFAULT_ADDR: &str = "0.0.0.0:2368";

use sysctl::Sysctl;
use uom::si::f32::{Angle, Length};

fn get_max_recv_buffer_size() -> Option<u32> {
    let ctl = sysctl::Ctl::new("net.core.rmem_max").ok()?;
    match ctl.value().ok()? {
        sysctl::CtlValue::Int(value) => Some(value as u32),
        sysctl::CtlValue::Uint(value) => Some(value),
        sysctl::CtlValue::String(value) => value.parse::<u32>().ok(),
        other => {
            println!("Unexpected sysctl value type: {:?}", other);
            None
        }
    }
}

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

struct Xt32 {
    socket: Socket,
    addr: SocketAddr,
    reftime: RefTime,
    channel_elevations: [Angle; 32],
}

impl Xt32 {
    /// This give the matching timestamps between the UTC time to the Robot time.
    fn sync(&mut self, robot_clock: &RobotClock) {
        self.reftime = (Utc::now(), robot_clock.now());
    }
}

impl CuTaskLifecycle for Xt32 {
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
            addr,
            reftime: rt,
            channel_elevations: generate_default_elevation_calibration(), // TODO: make the config able to override that
        })
    }
    fn start(&mut self, robot_clock: &RobotClock) -> CuResult<()> {
        self.sync(&robot_clock);
        Ok(())
    }
}

impl Freezable for Xt32 {}

const MAX_POINTS: usize = 32 * 10;

type LidarCuMsgPayload = LidarPayloadSoa<MAX_POINTS>;

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

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
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

                let is_dual = lidar_packet.header.is_dual_return();
                for block in lidar_packet.blocks.iter() {
                    let azimuth = block.azimuth();
                    block.channels.iter().enumerate().for_each(|(i, c)| {
                        let elevation = self.channel_elevations[i];
                        let d = c.distance();
                        let r = c.reflectivity();

                        let (x, y, z) = spherical_to_cartesian(azimuth, elevation, d);
                        let t = channel_time(t6, i as u64);
                        payload.push(LidarPayload::new_uom(t, x, y, z, r));
                    });
                }
                new_msg.metadata.tov = payload.tov[0].into(); // take the oldest timestamp
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
    use chrono::DateTime;
    use cu29::cutask::CuMsg;
    use pcap::{Capture, Packet};
    use std::net::UdpSocket;
    use std::path::Path;
    use std::thread::sleep;
    use std::time::{Duration, Instant};

    /// Small helper to stream a pcap file over UDP
    pub struct PcapStreamer {
        capture: Capture<pcap::Offline>,
        socket: UdpSocket,
        target_addr: String,
        last_packet_ts: Option<Duration>,
        start_instant: Instant,
    }

    impl PcapStreamer {
        pub fn new(file_path: impl AsRef<Path>, target_addr: impl Into<String>) -> Self {
            let capture = Capture::from_file(file_path).expect("Failed to open pcap file");
            let socket = UdpSocket::bind("0.0.0.0:0").expect("Failed to bind UDP socket");
            Self {
                capture,
                socket,
                target_addr: target_addr.into(),
                last_packet_ts: None,
                start_instant: Instant::now(),
            }
        }

        pub fn send_next(&mut self) -> bool {
            // Get the next packet and check for end of stream
            let packet = match self.capture.next_packet() {
                Ok(packet) => packet,
                Err(_) => return false, // End of the stream
            };

            // Assume 42-byte header (Ethernet + IP + UDP) and an optional 4-byte FCS
            let payload_offset = 42;
            let fcs_size = 4;

            // Check if there's an FCS and slice it off if present
            let data_len = if packet.data.len() > payload_offset + fcs_size {
                packet.data.len() - fcs_size
            } else {
                packet.data.len()
            };

            // Extract only the payload, excluding headers and trailing FCS if present
            let payload = &packet.data[payload_offset..data_len];

            // Extract the timestamp from the packet
            let ts = packet.header.ts;
            let packet_ts = Duration::new(ts.tv_sec as u64, ts.tv_usec as u32 * 1000);

            if let Some(last_ts) = self.last_packet_ts {
                // Sleep to match the delay between packets
                let elapsed = self.start_instant.elapsed();
                if packet_ts > last_ts {
                    let wait_time = packet_ts - last_ts;
                    if elapsed < wait_time {
                        sleep(wait_time - elapsed);
                    }
                }
            }
            self.last_packet_ts = Some(packet_ts);

            self.socket
                .send_to(payload, &self.target_addr)
                .expect("Failed to send packet");
            true
        }

        fn packet_timestamp(&self, packet: &Packet) -> Duration {
            let ts = packet.header.ts;
            Duration::new(ts.tv_sec as u64, ts.tv_usec as u32 * 1000)
        }
    }

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

        while streamer.send_next() {
            let err = xt32.process(&clock, &mut new_msg);
            if let Err(e) = err {
                println!("Error: {:?}", e);
                continue;
            }
            if let Some(payload) = new_msg.payload() {
                println!("Lidar Payload: {:?}", payload);
            }
            break;
        }
    }
}
