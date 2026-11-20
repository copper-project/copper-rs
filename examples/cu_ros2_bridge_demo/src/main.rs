use cu29::prelude::*;
use std::sync::atomic::{AtomicUsize, Ordering};

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

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
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

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
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
    let app = App::builder().with_log_path(&logger_path, None)?.build()?;
    let mut app = app.start()?;

    let reply = app.wait_until_ready(CuDuration::from_secs(1), || tasks::received_count() > 0);
    app.stop()?;
    reply.map_err(|error| match error {
        CuWaitError::Timeout => {
            CuError::from("ROS2 bridge did not receive a loopback reply within 1 s")
        }
        other => other.into(),
    })?;

    let received = tasks::received_count();
    println!("ros2 bridge loopback OK: received {received} messages");
    Ok(())
}
