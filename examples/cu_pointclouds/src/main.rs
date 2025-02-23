use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_hesai::parser::Packet;
use cu_hesai::LidarCuMsgPayload;
use cu_sensor_payloads::Distance;
use cu_udp_inject::PcapStreamer;
use rerun::Position3D;

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

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let payload = input.payload();
        if payload.is_none() {
            // Depending on the race condition, we might get an empty payload.
            return Ok(());
        }
        let payload = payload.unwrap();
        let points: Vec<Position3D> = payload
            .iter()
            .map(|p| {
                let Distance(x) = p.x;
                let Distance(y) = p.y;
                let Distance(z) = p.z;
                Position3D::new(x.value, y.value, z.value)
            })
            .collect();

        self.rec
            .log("points", &rerun::Points3D::new(points))
            .map_err(|e| CuError::new_with_cause("Failed to log points", e))?;

        Ok(())
    }
}
fn main() {
    const PACKET_SIZE: usize = size_of::<Packet>();
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("ptclouds.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, false, None).expect("Failed to setup copper.");
    let mut application = PtCloudsApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application");
    application
        .start_all_tasks()
        .expect("Failed to start all tasks.");

    loop {
        let mut streamer = PcapStreamer::new(
            "../../components/sources/cu_hesai/tests/hesai-xt32-small.pcap",
            "127.0.0.1:2368",
        );

        while streamer
            .send_next::<PACKET_SIZE>()
            .expect("Failed to send packet")
        {
            application.run_one_iteration().unwrap();
        }
    }
}
