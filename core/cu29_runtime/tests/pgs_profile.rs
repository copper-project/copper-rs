//! A recorded run provides every input needed by profile-guided scheduling.
#![cfg(feature = "std")]

use cu29::prelude::*;
use cu29_export::pgs::compute_profile;
use cu29_runtime::planner::{CuContract, ExplicitSchedule, ProposeRequest, propose};

const ITERATIONS: u64 = 50;

#[derive(Reflect)]
pub struct Source {
    next: u64,
}

impl Freezable for Source {}

impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u64);

    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        if self.next.is_multiple_of(2) {
            output.tov = Tov::Time(ctx.now());
            output.set_payload(self.next);
        } else {
            output.clear_payload();
        }
        self.next += 1;
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Work;

impl Freezable for Work {}

impl CuTask for Work {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);
    type Output<'m> = output_msg!(u64);

    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        match input.payload() {
            Some(value) => {
                std::thread::sleep(std::time::Duration::from_micros(100));
                output.set_payload(value * 2);
            }
            None => output.clear_payload(),
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Sink;

impl Freezable for Sink {}

impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);

    fn new(_: Option<&ComponentConfig>, _: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _: &CuContext, _: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "tests/pgs_profile_config.ron")]
struct App {}

#[test]
fn profile_reports_operations_chains_and_source_rates() {
    let dir = tempfile::tempdir().unwrap();
    let log_base = dir.path().join("profile.copper");
    let app = App::builder()
        .with_log_path(&log_base, Some(16 * 1024 * 1024))
        .unwrap()
        .build()
        .unwrap();
    let mut running = app.start().unwrap();
    for _ in 0..ITERATIONS {
        running.run_one_iteration().unwrap();
    }
    running.stop().unwrap();

    let config = cu29::config::read_configuration("tests/pgs_profile_config.ron").unwrap();
    let contract = CuContract::deserialize_ron(
        r#"(
        chains: [(id: "hot", source: "src", sink: "sink", deadline_ms: 50)],
        sources: [(task: "src", period_ms: 1)],
        cpus: [0],
        max_in_flight: 1,
    )"#,
    )
    .unwrap();
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(&log_base)
        .build()
        .unwrap();
    let UnifiedLogger::Read(logger) = logger else {
        panic!("expected a reader");
    };
    let reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::CopperList);
    let profile =
        compute_profile::<default::CuStampedDataSet>(reader, &config, None, &contract).unwrap();

    assert_eq!(profile.copperlists, ITERATIONS);
    assert_eq!(profile.mission, "default");
    assert!(profile.window_ns > 0);
    assert_eq!(profile.copperlist_span.samples, ITERATIONS);

    let work = &profile.operations["mission:default|task:work|phase:whole"];
    assert_eq!(
        (work.fired.samples, work.skipped.samples),
        (ITERATIONS / 2, ITERATIONS / 2)
    );
    assert!(work.fired.p50_ns >= 100_000, "{work:?}");
    assert!(work.fired.min_ns <= work.fired.p50_ns && work.fired.p50_ns <= work.fired.max_ns);
    assert!(work.fired.p99_ns > work.skipped.p99_ns, "{work:?}");
    assert!(work.firing_rate_hz > 0.0);
    let firing = work.firing.as_ref().unwrap();
    assert_eq!((firing.period, firing.phases.len()), (2, 1));
    let src = &profile.operations["mission:default|task:src|phase:whole"];
    assert_eq!(src.fired.samples, ITERATIONS / 2);
    let sink = &profile.operations["mission:default|task:sink|phase:whole"];
    assert_eq!(
        (sink.fired.samples, sink.skipped.samples),
        (ITERATIONS / 2, ITERATIONS / 2)
    );

    let hot = &profile.chains["hot"];
    assert_eq!(
        (hot.deadline_ms, hot.latency.samples, hot.misses),
        (50, ITERATIONS / 2, 0)
    );
    assert!(hot.latency.min_ns >= 100_000, "{hot:?}");

    let source = &profile.sources["src"];
    assert_eq!((source.period_ms, source.fired), (1, ITERATIONS / 2));
    assert!(source.expected > 0.0 && source.delivered_rate > 0.0);
    let firings = src.firing_rate_hz * profile.window_ns as f64 / 1e9;
    assert!(
        (firings - (ITERATIONS / 2 - 1) as f64).abs() < 1e-6,
        "{firings}"
    );

    let text = profile.serialize_ron().unwrap();
    assert_eq!(
        cu29_runtime::planner::CuProfile::deserialize_ron(&text).unwrap(),
        profile
    );

    let candidates = propose(&ProposeRequest {
        config: &config,
        mission: "default",
        contract: &contract,
        profile: &profile,
        candidates: 2,
    })
    .unwrap();
    assert!(!candidates.is_empty());
    for candidate in candidates {
        candidate.plan.validate(&config).unwrap();
        let mut prepared = config.clone();
        ExplicitSchedule::new(candidate.plan.clone())
            .apply(&mut prepared)
            .unwrap();
        let reparsed = CuConfig::deserialize_ron(&prepared.serialize_ron().unwrap()).unwrap();
        assert_eq!(
            cu29_runtime::planner::CuPlan::from_config(&reparsed).unwrap(),
            candidate.plan
        );
    }
}
