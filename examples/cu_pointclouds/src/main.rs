use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_hesai::LidarCuMsgPayload;
use pcap::Capture;
use rerun::Position3D;
use std::net::UdpSocket;
use std::path::{Path, PathBuf};
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
}

const SLAB_SIZE: Option<usize> = Some(100 * 1024 * 1024);

#[copper_runtime(config = "ptclouds.ron")]
struct PtCloudsApplication {}

struct RerunPlyViz {
    rec: rerun::RecordingStream,
}

impl Freezable for RerunPlyViz {}

impl<'cl> CuSinkTask<'cl> for RerunPlyViz {
    type Input = input_msg!('cl, LidarCuMsgPayload);

    fn new(_config: Option<&ComponentConfig>) -> Result<Self, CuError>
    where
        Self: Sized,
    {
        Ok(Self {
            rec: rerun::RecordingStreamBuilder::new("Ply Visualizer")
                .spawn()
                .map_err(|e| CuError::new_with_cause("Failed to spawn rerun stream", e))?,
        })
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> _CuResult<()> {
        let points: Vec<Position3D> = input
            .payload()
            .unwrap()
            .iter()
            .map(|p| Position3D::new(p.x.0.value, p.y.0.value, p.z.0.value))
            .collect();

        self.rec
            .log("points", &rerun::Points3D::new(points))
            .map_err(|e| CuError::new_with_cause("Failed to log points", e))?;

        Ok(())
    }
}
fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("ptclouds.copper");
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, false, None)
        .expect("Failed to setup copper.");
    let mut application =
        PtCloudsApplication::new(copper_ctx.clock.clone(), copper_ctx.unified_logger.clone())
            .expect("Failed to create application.");
    application
        .start_all_tasks()
        .expect("Failed to start all tasks.");

    loop {
        let mut streamer = PcapStreamer::new(
            "../../components/sources/cu_hesai/tests/hesai-xt32-small.pcap",
            "127.0.0.1:2368",
        );

        while streamer.send_next() {
            application.run_one_iteration().unwrap();
        }
    }
}
