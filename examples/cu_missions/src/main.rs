use cu29::prelude::*;

mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

fn main() -> CuResult<()> {
    let clock = RobotClock::default();
    let mut missions = AppRunner::builder().with_clock(clock.clone()).build()?;
    missions.start(AppMission::A)?;

    let switch_at = clock.raw_now() + CuDuration::from_secs(1);
    if missions.run_until(|| clock.raw_now() >= switch_at)? == CuRunExit::Requested {
        missions.transition_to(AppMission::B)?;
        let stop_at = clock.raw_now() + CuDuration::from_secs(1);
        missions.run_until(|| clock.raw_now() >= stop_at)?;
    }
    missions.stop()
}

#[test]
fn runner_reuses_custom_logger_and_resource_setup() {
    use std::cell::Cell;
    use std::sync::{Arc, Mutex};

    let writer = Arc::new(Mutex::new(NoopLogger::new()));
    let activations = Cell::new(0);
    let mut runner = AppRunner::builder()
        .with_logger::<NoopSectionStorage, _>(writer.clone())
        .with_resources(|_| {
            activations.set(activations.get() + 1);
            Ok(ResourceManager::new(&[]))
        })
        .build()
        .unwrap();

    assert_eq!(activations.get(), 0);
    runner.start(AppMission::A).unwrap();
    runner.transition_to(AppMission::B).unwrap();
    runner.transition_to(AppMission::B).unwrap();
    runner.stop().unwrap();
    assert_eq!(activations.get(), 3);
    drop(runner);
    assert_eq!(Arc::strong_count(&writer), 1);
}
