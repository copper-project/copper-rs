//! Example application for the `cu_feetech` bridge.
//!
//! Simple publisher that reads servo positions from a SO-100 / SO-101 Feetech
//! STS3215 servo arm and logs them.
//!
//! ```sh
//! cargo run -p cu-feetech-demo -- arm_publisher
//! ```

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron")]
struct FeetechDemoApp {}

use arm_publisher::FeetechDemoAppBuilder as ArmPublisherBuilder;
use leader_follower::FeetechDemoAppBuilder as LeaderFollowerBuilder;

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn run_mission<App>(app: &mut App) -> CuResult<()>
where
    App: CuApplication<memmap::MmapSectionStorage, UnifiedLoggerWrite>,
{
    app.run()
}

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

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");

    match mission.as_str() {
        "arm_publisher" => {
            debug!("Starting ARM_PUBLISHER mission – reading positions.");
            let mut app = ArmPublisherBuilder::new()
                .with_context(&copper_ctx)
                .build()
                .unwrap_or_else(|e| {
                    print_setup_help(&e);
                    std::process::exit(1);
                });
            if let Err(e) = run_mission(&mut app) {
                debug!("Arm publisher ended: {}", e);
            }
        }
        "leader_follower" => {
            debug!("Starting LEADER_FOLLOWER mission – leader drives follower.");
            let mut app = LeaderFollowerBuilder::new()
                .with_context(&copper_ctx)
                .build()
                .unwrap_or_else(|e| {
                    print_setup_help(&e);
                    std::process::exit(1);
                });
            if let Err(e) = run_mission(&mut app) {
                debug!("Leader-follower ended: {}", e);
            }
        }
        other => {
            eprintln!(
                "Unknown mission: '{}'. Use 'arm_publisher' or 'leader_follower'.",
                other
            );
            std::process::exit(1);
        }
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

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(positions) = input.payload() {
                debug!("Present positions: {}", positions);
            }
            Ok(())
        }
    }
}
