#[cfg(target_os = "linux")]
mod hosted {
    use cu29::prelude::*;
    #[copper_runtime(config = "linux.ron")]
    struct App {}
    pub fn main() -> CuResult<()> {
        let clock = RobotClock::new();
        let app = App::builder()
            .with_clock(clock.clone())
            .with_log_path(cu_clock_sync::log_path("linux")?, None)?
            .build()?;
        cu_clock_sync::run(app, &clock)
    }
}
#[cfg(target_os = "linux")]
fn main() -> cu29::prelude::CuResult<()> {
    hosted::main()
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("This reference requires Linux");
}
