use clap::{Parser, ValueEnum};
use cu29::config::ConfigGraphs;
use cu29::prelude::memmap::{MmapUnifiedLogger, MmapUnifiedLoggerBuilder};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup_with_logger;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub mod tasks {
    use cu29::bincode::{Decode, Encode};
    use cu29::prelude::*;
    use serde::{Deserialize, Serialize};
    use std::hint::black_box;

    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct BenchPayload {
        pub marker: u64,
        pub bytes: Vec<u8>,
    }

    #[derive(Reflect)]
    pub struct BenchSource {
        payload_bytes: usize,
        spin_iters: u64,
        next_marker: u64,
    }

    impl Freezable for BenchSource {}

    impl CuSrcTask for BenchSource {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(BenchPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let payload_bytes = config
                .and_then(|cfg| cfg.get::<u64>("payload_bytes").ok().flatten())
                .unwrap_or(256 * 1024) as usize;
            let spin_iters = config
                .and_then(|cfg| cfg.get::<u64>("spin_iters").ok().flatten())
                .unwrap_or(150_000);

            Ok(Self {
                payload_bytes,
                spin_iters,
                next_marker: 0,
            })
        }

        fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            let mut acc = self.next_marker ^ 0x9E37_79B9_7F4A_7C15;
            for i in 0..self.spin_iters {
                acc = acc
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(i ^ self.next_marker);
            }
            black_box(acc);

            let payload = output
                .payload_mut()
                .get_or_insert_with(BenchPayload::default);
            if payload.bytes.len() != self.payload_bytes {
                payload.bytes.resize(self.payload_bytes, 0);
            }
            payload.marker = self.next_marker;
            if let Some(first) = payload.bytes.first_mut() {
                *first = (self.next_marker & 0xFF) as u8;
            }
            if let Some(last) = payload.bytes.last_mut() {
                *last = ((self.next_marker >> 8) & 0xFF) as u8;
            }

            self.next_marker = self.next_marker.wrapping_add(1);
            Ok(())
        }
    }

    #[derive(Reflect, Default)]
    pub struct BenchSink {
        last_seen: u64,
    }

    impl Freezable for BenchSink {}

    impl CuSinkTask for BenchSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(BenchPayload);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self::default())
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(payload) = input.payload() {
                self.last_seen = payload.marker;
                black_box(self.last_seen);
                if let Some(first) = payload.bytes.first() {
                    black_box(*first);
                }
            }
            Ok(())
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "cu-async-cl-io-bench")]
struct Args {
    #[arg(long, value_enum, default_value_t = LoggerBackend::Mmap)]
    backend: LoggerBackend,
    #[arg(long, default_value_t = 200)]
    warmup: u32,
    #[arg(long, default_value_t = 5000)]
    iterations: u32,
    #[arg(long, default_value_t = 256)]
    payload_kib: u64,
    #[arg(long, default_value_t = 150000)]
    spin_iters: u64,
    #[arg(long, default_value_t = 8)]
    section_mib: u64,
    #[arg(long, default_value_t = 256)]
    slab_mib: u64,
    #[arg(long, default_value_t = 1_000_000)]
    keyframe_interval: u32,
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    enable_task_logging: bool,
    #[arg(long, default_value = "target/bench-logs")]
    log_root: PathBuf,
}

#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
enum LoggerBackend {
    Mmap,
    #[cfg(target_os = "linux")]
    IoUring,
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

fn percentile(sorted: &[u64], num: u64, den: u64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let len = sorted.len();
    let rank = ((len - 1) as u128 * num as u128) / den as u128;
    sorted[rank as usize]
}

fn format_ns_as_us(ns: u64) -> String {
    format!("{:.2}", ns as f64 / 1_000.0)
}

fn create_benchmark_log_dir(log_root: &Path) -> CuResult<tempfile::TempDir> {
    fs::create_dir_all(log_root)
        .map_err(|e| CuError::new_with_cause("failed to create benchmark log root", e))?;
    tempfile::Builder::new()
        .prefix("cu_async_cl_io_bench_")
        .tempdir_in(log_root)
        .map_err(|e| CuError::new_with_cause("failed to create benchmark log directory", e))
}

fn measure_log_bytes(log_dir: &Path) -> CuResult<u64> {
    let mut total = 0u64;
    for entry in fs::read_dir(log_dir)
        .map_err(|e| CuError::new_with_cause("failed to read benchmark log directory", e))?
    {
        let entry =
            entry.map_err(|e| CuError::new_with_cause("failed to read benchmark log entry", e))?;
        let file_type = entry
            .file_type()
            .map_err(|e| CuError::new_with_cause("failed to stat benchmark log entry", e))?;
        if !file_type.is_file() {
            continue;
        }
        let path = entry.path();
        if path.extension().and_then(|ext| ext.to_str()) != Some("copper") {
            continue;
        }
        total = total.saturating_add(
            entry
                .metadata()
                .map_err(|e| CuError::new_with_cause("failed to stat benchmark log file", e))?
                .len(),
        );
    }
    Ok(total)
}

fn benchmark_config<S, L>(args: &Args) -> CuConfig
where
    S: SectionStorage + 'static,
    L: UnifiedLogWrite<S> + 'static,
    App: CuApplication<S, L>,
{
    let bundled_config = <App as CuApplication<S, L>>::get_original_config();
    let mut config = CuConfig::deserialize_ron(&bundled_config)
        .expect("failed to deserialize bundled benchmark config");

    let logging = config
        .logging
        .get_or_insert_with(cu29::config::LoggingConfig::default);
    logging.enable_task_logging = args.enable_task_logging;
    logging.section_size_mib = Some(args.section_mib);
    logging.slab_size_mib = Some(args.slab_mib);
    logging.keyframe_interval = Some(args.keyframe_interval);
    match &mut config.graphs {
        ConfigGraphs::Simple(graph) => {
            let src_id = graph
                .get_node_id_by_name("src")
                .expect("benchmark source node 'src' missing");
            let src = graph
                .get_node_mut(src_id)
                .expect("benchmark source node id missing");
            src.set_param("payload_bytes", args.payload_kib * 1024);
            src.set_param("spin_iters", args.spin_iters);
        }
        ConfigGraphs::Missions(_) => panic!("benchmark config unexpectedly uses missions"),
    }

    config
}

fn run_benchmark<S, L>(args: &Args, logger: L, log_dir: &Path) -> CuResult<()>
where
    S: SectionStorage + 'static,
    L: UnifiedLogWrite<S> + 'static,
    App: CuApplication<S, L>,
{
    let config = benchmark_config::<S, L>(args);
    let copperlist_count = config
        .logging
        .as_ref()
        .and_then(|logging| logging.copperlist_count)
        .unwrap_or(2);
    let ctx = basic_copper_setup_with_logger::<S, L>(logger, None)?;
    let mut app = <App as CuApplication<S, L>>::new(
        ctx.clock.clone(),
        ctx.unified_logger.clone(),
        Some(config),
    )?;
    app.start_all_tasks()?;

    for _ in 0..args.warmup {
        app.run_one_iteration()?;
    }

    let mut samples_ns = Vec::with_capacity(args.iterations as usize);
    let benchmark_start = Instant::now();
    for _ in 0..args.iterations {
        let iter_start = Instant::now();
        app.run_one_iteration()?;
        let elapsed = iter_start.elapsed();
        samples_ns.push(elapsed.as_nanos() as u64);
    }
    let active_elapsed = benchmark_start.elapsed();
    app.stop_all_tasks()?;
    drop(app);
    let stop_elapsed = benchmark_start.elapsed();
    drop(ctx);
    let total_elapsed = benchmark_start.elapsed();
    let actual_log_bytes = measure_log_bytes(log_dir)?;

    let mut jitter_ns = samples_ns
        .windows(2)
        .map(|window| window[0].abs_diff(window[1]))
        .collect::<Vec<_>>();
    samples_ns.sort_unstable();
    jitter_ns.sort_unstable();

    let samples = samples_ns.len() as f64;
    let mean_ns = if samples > 0.0 {
        samples_ns.iter().copied().sum::<u64>() as f64 / samples
    } else {
        0.0
    };
    let iter_per_sec = if active_elapsed.as_secs_f64() > 0.0 {
        samples / active_elapsed.as_secs_f64()
    } else {
        0.0
    };
    let estimated_payload_bytes = (args.payload_kib as u128) * 1024 * (args.iterations as u128);
    let estimated_payload_gib = estimated_payload_bytes as f64 / (1024.0 * 1024.0 * 1024.0);
    let estimated_payload_gib_s = if total_elapsed.as_secs_f64() > 0.0 {
        estimated_payload_gib / total_elapsed.as_secs_f64()
    } else {
        0.0
    };
    let actual_log_gib = actual_log_bytes as f64 / (1024.0 * 1024.0 * 1024.0);
    let actual_log_gib_s = if total_elapsed.as_secs_f64() > 0.0 {
        actual_log_gib / total_elapsed.as_secs_f64()
    } else {
        0.0
    };

    println!(
        "mode={} backend={:?} logging={} copperlist_count={} payload_kib={} spin_iters={} section_mib={} slab_mib={} keyframe_interval={} warmup={} iterations={} log_root={} estimated_payload_gib={:.2} actual_log_gib={:.2}",
        if cfg!(feature = "async-cl-io") {
            "async-cl-io"
        } else {
            "sync-cl-io"
        },
        args.backend,
        args.enable_task_logging,
        copperlist_count,
        args.payload_kib,
        args.spin_iters,
        args.section_mib,
        args.slab_mib,
        args.keyframe_interval,
        args.warmup,
        args.iterations,
        args.log_root.display(),
        estimated_payload_gib,
        actual_log_gib
    );
    println!(
        "active_hz={:.2} estimated_payload_gib_s={:.2} actual_log_gib_s={:.2} active_s={:.2} stop_s={:.2} total_s={:.2} mean_us={:.2} p50_us={} p90_us={} p99_us={} p999_us={} max_us={} delta_p50_us={} delta_p90_us={} delta_p99_us={} delta_p999_us={} delta_max_us={}",
        iter_per_sec,
        estimated_payload_gib_s,
        actual_log_gib_s,
        active_elapsed.as_secs_f64(),
        stop_elapsed.as_secs_f64(),
        total_elapsed.as_secs_f64(),
        mean_ns / 1_000.0,
        format_ns_as_us(percentile(&samples_ns, 50, 100)),
        format_ns_as_us(percentile(&samples_ns, 90, 100)),
        format_ns_as_us(percentile(&samples_ns, 99, 100)),
        format_ns_as_us(percentile(&samples_ns, 999, 1000)),
        format_ns_as_us(*samples_ns.last().unwrap_or(&0)),
        format_ns_as_us(percentile(&jitter_ns, 50, 100)),
        format_ns_as_us(percentile(&jitter_ns, 90, 100)),
        format_ns_as_us(percentile(&jitter_ns, 99, 100)),
        format_ns_as_us(percentile(&jitter_ns, 999, 1000)),
        format_ns_as_us(*jitter_ns.last().unwrap_or(&0))
    );

    Ok(())
}

fn main() -> CuResult<()> {
    let args = Args::parse();
    let tmp_dir = create_benchmark_log_dir(&args.log_root)?;
    let logger_path = tmp_dir.path().join("async_cl_io_bench.copper");
    let slab_size = (args.slab_mib as usize) * 1024 * 1024;

    match args.backend {
        LoggerBackend::Mmap => {
            let logger = match MmapUnifiedLoggerBuilder::new()
                .write(true)
                .create(true)
                .file_base_name(&logger_path)
                .preallocated_size(slab_size)
                .build()
                .map_err(|e| CuError::new_with_cause("Failed to create mmap logger", e))?
            {
                MmapUnifiedLogger::Write(logger) => logger,
                MmapUnifiedLogger::Read(_) => {
                    return Err(CuError::from(
                        "mmap logger builder did not create a write-capable logger",
                    ));
                }
            };
            run_benchmark::<cu29::prelude::memmap::MmapSectionStorage, UnifiedLoggerWrite>(
                &args,
                logger,
                tmp_dir.path(),
            )
        }
        #[cfg(target_os = "linux")]
        LoggerBackend::IoUring => {
            let logger = match IoUringUnifiedLoggerBuilder::new()
                .write(true)
                .create(true)
                .file_base_name(&logger_path)
                .preallocated_size(slab_size)
                .build()
                .map_err(|e| CuError::new_with_cause("Failed to create io_uring logger", e))?
            {
                IoUringUnifiedLogger::Write(logger) => logger,
            };
            run_benchmark::<IoUringSectionStorage, IoUringUnifiedLoggerWrite>(
                &args,
                logger,
                tmp_dir.path(),
            )
        }
    }
}
