use std::fs;
use std::path::{Path, PathBuf};

use cu29::bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::prelude::app::CuSimApplication;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use serde::{Deserialize, Serialize};

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode)]
pub struct MyMsg {
    pub value: u32,
}

// A real source (we won’t run it in sim when run_in_sim = true)
pub struct MySource {
    next: u32,
}
impl Freezable for MySource {}
impl CuSrcTask for MySource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(MyMsg);
    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }
    fn process(&mut self, _clock: &RobotClock, out: &mut Self::Output<'_>) -> CuResult<()> {
        // Real source just increments. In this demo, the sim will bypass this when run_in_sim = false.
        self.next += 1;
        out.set_payload(MyMsg { value: self.next });
        // To show that this task is actually fully stubbed in sim and is not compiled in nor run at all.
        panic!("This source should never be called from the sim");
        // Ok(())
    }
}

// A normal “regular” task (run_in_sim is ignored for regular tasks)
pub struct Doubler;
impl Freezable for Doubler {}
impl CuTask for Doubler {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MyMsg);
    type Output<'m> = output_msg!(MyMsg);

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(m) = input.payload() {
            output.set_payload(MyMsg { value: m.value * 2 });
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

// A real sink (we want the real code to run in simulation => run_in_sim = true in config)
pub struct MySink;
impl Freezable for MySink {}
impl CuSinkTask for MySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MyMsg);

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        // THIS CODE WOULD NOT RUN IF run_in_sim = false in config
        if let Some(m) = input.payload() {
            println!("[MySink] got value = {}", m.value);
        } else {
            println!("[MySink] got no payload");
        }
        Ok(())
    }
}

// Generate the runtime in SIM mode from the RON config file
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct App {}

// A tiny loop that runs a few iterations to show values evolving
fn main() -> CuResult<()> {
    // Counter owned by the sim to inject data into the source placeholder
    let mut sim_counter: u32 = 0;

    // The generated SimStep enum gives you the per-task callback points.
    let mut sim_callback = move |step: <App as CuSimApplication<
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
    >>::Step<'_>|
          -> SimOverride {
        match step {
            // New: just demonstrate you can see config if needed
            default::SimStep::Src(CuTaskCallbackState::New(_)) => SimOverride::ExecuteByRuntime,

            // For the source’s Process: we must feed the sim placeholder with a payload
            default::SimStep::Src(CuTaskCallbackState::Process((), out)) => {
                // Since the source has run_in_sim = false, runtime uses a CuSimSrcTask which requires us
                // to set the payload here (ExecutedBySim).
                sim_counter += 1;
                out.set_payload(MyMsg { value: sim_counter });
                SimOverride::ExecutedBySim
            }

            // Let Doubler and Sink run normally
            default::SimStep::Proc(CuTaskCallbackState::Process(_, _)) => {
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::Sink(CuTaskCallbackState::Process(_, _)) => {
                // Here the sink has run_in_sim = true, so we want the real code to compile AND run.
                SimOverride::ExecuteByRuntime
            }

            // Start/Stop/Pre/Post: just defer to runtime
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/run_in_sim.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    // here we set up a mock clock so the simulation can take control of it.
    let (robot_clock, _mock) = RobotClock::mock();
    let copper_ctx = cu29_helpers::basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");
    debug!(
        "Logger created at {}. This is a simulation.",
        path = logger_path
    );

    let mut copper_app = AppBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut sim_callback)
        .build()
        .expect("Failed to create runtime.");

    // Start and run a few iterations, then stop
    copper_app.start_all_tasks(&mut sim_callback)?;
    for _ in 0..5 {
        copper_app.run_one_iteration(&mut sim_callback)?;
    }
    copper_app.stop_all_tasks(&mut sim_callback)?;
    Ok(())
}
