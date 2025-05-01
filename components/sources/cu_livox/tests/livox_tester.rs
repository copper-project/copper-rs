use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_livox::parser::LidarFrame;
use cu_livox::LidarCuMsgPayload;
use cu_udp_inject::PcapStreamer;

use std::thread;
use std::time::Duration;
struct LivoxTestSink {}

impl Freezable for LivoxTestSink {}

impl<'cl> CuSinkTask<'cl> for LivoxTestSink {
    type Input = input_msg!('cl, LidarCuMsgPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        match &new_msg.payload() {
            None => {
                debug!("Received Nothing.");
                Err(CuError::from("Received Nothing."))
            }
            Some(payload_value) => {
                let &x = &payload_value.x;
                let &y = &payload_value.y;
                let &z = &payload_value.z;
                let &i = &payload_value.i;
                let &r = &payload_value.return_order;
                let &t = &payload_value.tov;
                let middle_index = payload_value.len() / 2;
                let last_index = payload_value.len() - 1;
                debug!(
                    "Received: first time: {} pt({},{},{}) i({}) return: {})",
                    t[0].0, x[0].value, y[1].value, z[0].value, i[0].value, r[0]
                );
                debug!(
                    "Received: middle time: {} pt({},{},{}) i({}) return: {})",
                    t[middle_index].0,
                    x[middle_index].value,
                    y[middle_index].value,
                    z[middle_index].value,
                    i[middle_index].value,
                    r[middle_index]
                );
                debug!(
                    "Received: last time: {} pt({},{},{}) i({}) return: {})",
                    t[last_index].0,
                    x[last_index].value,
                    y[last_index].value,
                    z[last_index].value,
                    i[last_index].value,
                    r[last_index]
                );
                Ok(())
            }
        }
    }
}

#[copper_runtime(config = "tests/copperconfig.ron")]
struct LivoxTester {}

#[test]
fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("livox_tester.copper");
    let copper_ctx =
        basic_copper_setup(&logger_path, None, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);

    thread::spawn(|| {
        let mut streamer = PcapStreamer::new("tests/livox_tele15_small.pcap", "127.0.0.1:56001");
        //Expected 1362 for Tele15
        const PACKET_SIZE: usize = size_of::<LidarFrame>();
        while streamer
            .send_next::<PACKET_SIZE>()
            .expect("Failed to send next packet")
        {
            debug!("Sending pcap packet!");
            thread::sleep(Duration::from_micros(200));
        }
    });

    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = LivoxTester::new(clock.clone(), copper_ctx.unified_logger.clone(), None)
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    for _ in 0..20 {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    debug!("End of program: {}.", clock.now());
    thread::sleep(Duration::from_secs(1));
}
