use cu29::prelude::*;

mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

use A::App as MissionAApp;
use B::App as MissionBApp;

fn main() {
    let clock = RobotClock::default();

    let _ = MissionAApp::builder().with_clock(clock.clone()).build();
    let _ = MissionBApp::builder().with_clock(clock).build();
}
