use cu29::prelude::*;
use std::sync::Mutex;

/// Order in which the tasks ran, recorded by each `process()`.
static RAN: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());

pub mod tasks {
    use super::RAN;
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct LeftSrc;

    impl Freezable for LeftSrc {}

    impl CuSrcTask for LeftSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            RAN.lock().unwrap().push("a_left");
            new_msg.set_payload(1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct RightSrc;

    impl Freezable for RightSrc {}

    impl CuSrcTask for RightSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            RAN.lock().unwrap().push("b_right");
            new_msg.set_payload(2);
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = None;

fn main() {
    let logger_path = std::env::temp_dir().join("cu_custom_planner.copper");
    let application = App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime");
    let mut running = application.start().expect("Failed to start application.");
    running
        .run_one_iteration()
        .expect("Failed to run application.");
    running.stop().expect("Failed to stop application.");

    let ran = RAN.lock().unwrap().clone();
    // The out-of-tree Alphabetical planner runs a_left before b_right; the
    // default Linearity order would start with b_right (listed first).
    assert_eq!(ran, ["a_left", "b_right"]);
    debug!("Executed in the out-of-tree planner's order.");
}
