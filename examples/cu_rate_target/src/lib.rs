use cu29::prelude::*;
use std::fs;
use std::path::{Path, PathBuf};

const TARGET_RATE_HZ: u64 = 1_000;

fn target_period() -> CuDuration {
    CuDuration::from_nanos(1_000_000_000 / TARGET_RATE_HZ)
}

fn probe_max_sample() -> CuDuration {
    CuDuration::from_millis(50)
}

fn report_interval() -> CuDuration {
    CuDuration::from_secs(1)
}

fn format_timestamp(time: CuTime) -> String {
    let nanos = time.as_nanos();
    let total_seconds = nanos / 1_000_000_000;
    let hours = total_seconds / 3600;
    let minutes = (total_seconds / 60) % 60;
    let seconds = total_seconds % 60;
    let fractional_1e4 = (nanos % 1_000_000_000) / 100_000;
    format!("{hours:02}:{minutes:02}:{seconds:02}.{fractional_1e4:04}")
}

fn emit_probe_line(now: CuTime, _plain_line: &str, colored_line: &str) {
    const TS_COLOR: &str = "\x1b[34m";
    const LEVEL_COLOR: &str = "\x1b[90m";
    const RESET: &str = "\x1b[0m";

    #[cfg(not(debug_assertions))]
    info!("{}", _plain_line);

    println!(
        "{ts_color}{}{reset} {level_color}[Info]{reset} {}",
        format_timestamp(now),
        colored_line,
        ts_color = TS_COLOR,
        level_color = LEVEL_COLOR,
        reset = RESET,
    );
}

struct LoopCadenceProbe {
    last_dispatch_at: Option<CuTime>,
    expected_dispatch_at: Option<CuTime>,
    last_report_at: Option<CuTime>,
    last_cl_id: Option<u64>,
    dt_stats: CuDurationStatistics,
    phase_error_stats: CuDurationStatistics,
    missed_copperlists: u64,
    max_cl_gap: u64,
}

impl LoopCadenceProbe {
    fn new() -> Self {
        Self {
            last_dispatch_at: None,
            expected_dispatch_at: None,
            last_report_at: None,
            last_cl_id: None,
            dt_stats: CuDurationStatistics::new(probe_max_sample()),
            phase_error_stats: CuDurationStatistics::new(probe_max_sample()),
            missed_copperlists: 0,
            max_cl_gap: 1,
        }
    }

    fn observe(&mut self, now: CuTime, cl_id: u64) {
        let target_period = target_period();

        if let Some(last_dispatch_at) = self.last_dispatch_at {
            self.dt_stats.record(now - last_dispatch_at);
        }
        self.last_dispatch_at = Some(now);

        match self.expected_dispatch_at {
            Some(expected_dispatch_at) => {
                let phase_error = if now >= expected_dispatch_at {
                    now - expected_dispatch_at
                } else {
                    expected_dispatch_at - now
                };
                self.phase_error_stats.record(phase_error);
                self.expected_dispatch_at = Some(expected_dispatch_at + target_period);
            }
            None => {
                self.expected_dispatch_at = Some(now + target_period);
            }
        }

        if let Some(last_cl_id) = self.last_cl_id {
            let gap = cl_id.saturating_sub(last_cl_id);
            self.max_cl_gap = self.max_cl_gap.max(gap);
            if gap > 1 {
                self.missed_copperlists = self.missed_copperlists.saturating_add(gap - 1);
            }
        }
        self.last_cl_id = Some(cl_id);

        let Some(last_report_at) = self.last_report_at else {
            self.last_report_at = Some(now);
            return;
        };

        if now - last_report_at < report_interval() || self.dt_stats.is_empty() {
            return;
        }

        let dt_p50_us = self.dt_stats.percentile(0.5).as_micros();
        let dt_p95_us = self.dt_stats.percentile(0.95).as_micros();
        let dt_max_us = self.dt_stats.max().as_micros().max(dt_p95_us);
        let dt_jitter_p95_us = self.dt_stats.jitter_percentile(0.95).as_micros();
        let dt_jitter_max_us = self.dt_stats.jitter_max().as_micros().max(dt_jitter_p95_us);
        let phase_p50_us = self.phase_error_stats.percentile(0.5).as_micros();
        let phase_p99_us = self.phase_error_stats.percentile(0.99).as_micros();
        let phase_max_us = self.phase_error_stats.max().as_micros().max(phase_p99_us);

        let plain_line = format!(
            "[probe] dt p50 {}us p95 {}us max {}us | jitter p95 {}us max {}us | phase p50 {}us p99 {}us max {}us | missed {} max_gap {}",
            dt_p50_us,
            dt_p95_us,
            dt_max_us,
            dt_jitter_p95_us,
            dt_jitter_max_us,
            phase_p50_us,
            phase_p99_us,
            phase_max_us,
            self.missed_copperlists,
            self.max_cl_gap,
        );
        let colored_line = format!(
            "[\x1b[94mprobe\x1b[0m] \x1b[92mdt\x1b[0m \x1b[93mp50\x1b[0m \x1b[36m{}us\x1b[0m \x1b[93mp95\x1b[0m \x1b[36m{}us\x1b[0m \x1b[93mmax\x1b[0m \x1b[36m{}us\x1b[0m | \x1b[92mjitter\x1b[0m \x1b[93mp95\x1b[0m \x1b[36m{}us\x1b[0m \x1b[93mmax\x1b[0m \x1b[36m{}us\x1b[0m | \x1b[92mphase\x1b[0m \x1b[93mp50\x1b[0m \x1b[36m{}us\x1b[0m \x1b[93mp99\x1b[0m \x1b[36m{}us\x1b[0m \x1b[93mmax\x1b[0m \x1b[36m{}us\x1b[0m | \x1b[92mmissed\x1b[0m \x1b[36m{}\x1b[0m \x1b[92mmax_gap\x1b[0m \x1b[36m{}\x1b[0m",
            dt_p50_us,
            dt_p95_us,
            dt_max_us,
            dt_jitter_p95_us,
            dt_jitter_max_us,
            phase_p50_us,
            phase_p99_us,
            phase_max_us,
            self.missed_copperlists,
            self.max_cl_gap,
        );
        emit_probe_line(now, &plain_line, &colored_line);

        self.last_report_at = Some(now);
        self.dt_stats.reset();
        self.phase_error_stats.reset();
        self.missed_copperlists = 0;
        self.max_cl_gap = 1;
    }
}

impl Default for LoopCadenceProbe {
    fn default() -> Self {
        Self::new()
    }
}

pub mod tasks {
    use crate::LoopCadenceProbe;
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct ExampleSrc {
        #[reflect(ignore)]
        cadence_probe: LoopCadenceProbe,
    }

    impl Freezable for ExampleSrc {}

    impl CuSrcTask for ExampleSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                cadence_probe: LoopCadenceProbe::new(),
            })
        }

        fn process(&mut self, _ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            let now = _ctx.now();
            self.cadence_probe.observe(now, _ctx.cl_id());
            new_msg.set_payload(42);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ExampleTask;

    impl Freezable for ExampleTask {}

    impl CuTask for ExampleTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);
        type Output<'m> = output_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let payload = input.payload().unwrap();
            output.set_payload(payload + 1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ExampleSink;

    impl Freezable for ExampleSink {}

    impl CuSinkTask for ExampleSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(i32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(150 * 1024 * 1024);

fn ensure_log_dir(logger_path: &Path) {
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
}

pub fn run_example(log_filename: &str) {
    let logger_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join(log_filename);
    ensure_log_dir(&logger_path);

    info!(
        "Logger created at {}.",
        logger_path.to_string_lossy().into_owned()
    );
    info!("Creating application...");
    let mut application = App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    info!("Running... starting clock: {}.", application.clock().now());
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    application.run().expect("Failed to run application.");
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
    info!("End of program: {}.", application.clock().now());
}
