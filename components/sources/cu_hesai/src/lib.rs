use crate::parser::RefTime;
use chrono::Utc;
use cu29::config::ComponentConfig;
use cu29::cutask::CuMsg;
use cu29::cutask::{CuSrcTask, CuTaskLifecycle, Freezable};
use cu29::{output_msg, CuError, CuResult};
use cu29_clock::RobotClock;
use cu_sensor_payloads::LidarPayloadSoa;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io::ErrorKind;
use std::io::Read;
use std::net::SocketAddr;

pub mod parser;

const DEFAULT_ADDR: &str = "0.0.0.0:2368";

const SOCKET_BUFFER_SIZE: usize = 4 * 1024 * 1024;

struct Xt32 {
    socket: Socket,
    addr: SocketAddr,
    reftime: RefTime,
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
        socket
            .set_recv_buffer_size(SOCKET_BUFFER_SIZE)
            .map_err(|e| e.to_string())?;
        let registered_size = socket
            .recv_buffer_size()
            .expect("Could not get back buffer size");
        if registered_size < SOCKET_BUFFER_SIZE {
            return Err("Failed to set socket buffer size".into());
        }
        socket.bind(&SockAddr::from(addr)).unwrap();
        socket.set_nonblocking(true).unwrap();

        // just a temporary value, it will be redone at start.
        let rt: RefTime = (Utc::now(), RobotClock::new().now());
        Ok(Xt32 {
            socket,
            addr,
            reftime: rt,
        })
    }
    fn start(&mut self, robot_clock: &RobotClock) -> CuResult<()> {
        self.sync(&robot_clock);
        Ok(())
    }
}

impl Freezable for Xt32 {}

impl<'cl> CuSrcTask<'cl> for Xt32 {
    type Output = output_msg!('cl, LidarPayloadSoa<10>);

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let mut buf = [0u8; 1500];
        match self.socket.read(&mut buf) {
            Ok(size) => {
                let lidar_packet = parser::parse_packet(&buf).unwrap();
                for (bid, ts) in lidar_packet
                    .block_ts(&self.reftime)
                    .unwrap()
                    .iter()
                    .enumerate()
                {
                    println!("Block {} tov: {}", bid, ts);
                }
                for block in lidar_packet.blocks.iter() {
                    println!("Block id: {:?}", block);
                    block.channels.iter().for_each(|c| {
                        let d = c.distance();
                        println!("Distance: {:?}", d);
                    });
                }
            }
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                // Handle no data available (non-blocking behavior)
                println!("No data available right now.");
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
    use cu29::cutask::CuMsg;

    fn stream_test_udp() -> CuResult<()> {
        // open pcap
        let pcap_file_path = "path/to/your/file.pcap";
        let target_addr = "127.0.0.1:12345";

        // Create a UDP socket bound to any available local port
        let socket = UdpSocket::bind("0.0.0.0:0")?;

        // Open the pcap file for reading
        let mut cap = Capture::from_file(pcap_file_path)?;

        // Iterate over packets in the pcap file
        while let Ok(packet) = cap.next() {
            let data = packet.data;

            // Send the packet data to the target address
            socket.send_to(data, target_addr)?;

            // Optional: add a delay if needed to simulate timing more realistically
            // std::thread::sleep(Duration::from_millis(10));
        }

        Ok(())
    }

    #[test]
    fn test_xt32() {
        let mut xt32 = Xt32 {
            socket: Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).unwrap(),
            addr: DEFAULT_ADDR.parse().unwrap(),
            reftime: (Utc::now(), RobotClock::new().now()),
        };
        let clock = RobotClock::new();
        let new_payload = LidarPayloadSoa::<10>::default();
        let mut new_msg = CuMsg::<LidarPayloadSoa<10>>::new(Some(new_payload));
        xt32.process(&clock, &mut new_msg).unwrap();
    }
}
