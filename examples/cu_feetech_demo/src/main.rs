//! Example application for the `cu_feetech` bridge.
//!
//! Simple publisher that reads servo positions from a SO-100 / SO-101 Feetech
//! STS3215 servo arm and logs them.
//!
//! ```sh
//! cargo run -p cu-feetech-demo -- arm_publisher
//! ```

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct FeetechDemoApp {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    let mission = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("Usage: cu-feetech-demo <mission>");
        eprintln!("  arm_publisher     – read and log servo positions");
        eprintln!("  leader_follower   – leader arm (serial0) drives follower arm (serial1)");
        std::process::exit(1);
    });

    let logger_path = "logs/cu_feetech_demo.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let selected = match mission.as_str() {
        "arm_publisher" => FeetechDemoAppMission::ArmPublisher,
        "leader_follower" => FeetechDemoAppMission::LeaderFollower,
        other => {
            eprintln!(
                "Unknown mission: '{}'. Use 'arm_publisher' or 'leader_follower'.",
                other
            );
            std::process::exit(1);
        }
    };
    let mut runner = FeetechDemoAppRunner::builder()
        .with_log_path(logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create mission runner");
    if let Err(error) = runner.start(selected) {
        print_setup_help(&error);
        std::process::exit(1);
    }
    debug!("Starting Feetech mission {}", mission.as_str());
    if let Err(error) = runner.run_until(|| false).and_then(|_| runner.stop()) {
        debug!("Feetech mission ended: {}", error);
    }
}

fn print_setup_help(e: &CuError) {
    eprintln!("ERROR: {}\n", e);
    eprintln!("Check: USB adapter plugged in? Device path in copperconfig.ron correct?");
    eprintln!("       Permission? Try: sudo usermod -aG dialout $USER");
}

// ===========================================================================
// Tasks
// ===========================================================================

mod tasks {
    use cu_feetech::messages::JointPositions;
    use cu29::prelude::*;

    // -----------------------------------------------------------------------
    // PositionLogger — logs positions
    // -----------------------------------------------------------------------

    #[derive(Default, Reflect)]
    pub struct PositionLogger;

    impl Freezable for PositionLogger {}

    impl CuSinkTask for PositionLogger {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(JointPositions);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(positions) = input.payload() {
                debug!("Present positions: {}", positions);
            }
            Ok(())
        }
    }
}
