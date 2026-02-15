use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

pub mod bridges {
    use cu_ros2_bridge::Ros2Bridge;
    use cu29::prelude::*;

    tx_channels! {
        pub struct DemoTxChannels : DemoTxId {
            outgoing => i8 = "/loopback",
        }
    }

    rx_channels! {
        pub struct DemoRxChannels : DemoRxId {
            incoming => i8 = "/loopback",
        }
    }

    pub type ExampleRos2Bridge = Ros2Bridge<DemoTxChannels, DemoRxChannels>;
}

pub mod tasks {
    use super::*;

    static RX_COUNT: AtomicUsize = AtomicUsize::new(0);

    pub fn reset_count() {
        RX_COUNT.store(0, Ordering::SeqCst);
    }

    pub fn received_count() -> usize {
        RX_COUNT.load(Ordering::SeqCst)
    }

    #[derive(Reflect)]
    pub struct ExampleSrc;

    impl Freezable for ExampleSrc {}

    impl CuSrcTask for ExampleSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i8);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            new_msg.set_payload(42);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct RxVerifier;

    impl Freezable for RxVerifier {}

    impl CuSinkTask for RxVerifier {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i8);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(payload) = input.payload() {
                if *payload != 42 {
                    return Err(CuError::from(format!(
                        "expected value 42 from ROS2 bridge, got {payload}"
                    )));
                }
                RX_COUNT.fetch_add(1, Ordering::SeqCst);
            }
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const ITERATIONS: usize = 50;

fn main() {
    if let Err(error) = drive() {
        eprintln!("cu-ros2-bridge-demo failed: {error}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    tasks::reset_count();

    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("ros2_bridge.copper");
    let copper_ctx = basic_copper_setup(&logger_path, None, true, None)?;

    let mut app = App::new(
        copper_ctx.clock.clone(),
        copper_ctx.unified_logger.clone(),
        None,
    )?;
    app.start_all_tasks()?;

    for _ in 0..ITERATIONS {
        app.run_one_iteration()?;
        std::thread::sleep(Duration::from_millis(20));
    }

    let received = tasks::received_count();
    if received == 0 {
        return Err(CuError::from(
            "ROS2 bridge test did not receive any looped-back Rx message",
        ));
    }

    println!("ros2 bridge loopback OK: received {received} messages");
    Ok(())
}
