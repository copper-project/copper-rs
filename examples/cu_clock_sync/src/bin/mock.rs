use cu29::prelude::*;
#[copper_runtime(config = "mock.ron")]
struct App {}
fn main() -> CuResult<()> {
    let clock = RobotClock::new();
    let app = App::builder()
        .with_clock(clock.clone())
        .with_log_path(cu_clock_sync::log_path("mock")?, None)?
        .build()?;
    cu_clock_sync::run(app, &clock)
}
