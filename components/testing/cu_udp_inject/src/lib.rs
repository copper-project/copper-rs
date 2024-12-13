use cu29_traits::CuResult;
use pcap::Capture;
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

    /// Send the next packet in the pcap file over UDP
    /// PS is the expected payload size to send in each packet
    /// Returns false if the end of the pcap file is reached
    pub fn send_next<const PS: usize>(&mut self) -> CuResult<bool> {
        // Get the next packet and check for end of stream
        let packet = match self.capture.next_packet() {
            Ok(packet) => packet,
            Err(_) => return Ok(false), // End of the stream
        };

        // Assume 42-byte header (Ethernet + IP + UDP) and an optional 4-byte FCS
        let payload_offset = 42;
        if packet.data.len() < payload_offset + PS {
            return Err("Packet too short".into());
        }

        // Extract only the payload, excluding headers and trailing FCS if present
        let payload = &packet.data[payload_offset..payload_offset + PS];

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
        Ok(true)
    }
}
