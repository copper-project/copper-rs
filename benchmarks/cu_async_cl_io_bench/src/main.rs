use clap::Parser;
use cu29::prelude::*;
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
    #[arg(long, default_value_t = 256 * 1024)]
    payload_bytes: u64,
    #[arg(long, default_value_t = 150_000)]
    spin_iters: u64,
    #[arg(long, default_value_t = 0)]
    structured_records: u64,
    #[arg(long, default_value_t = 200)]
    warmup: u32,
    #[arg(long, default_value_t = 5000)]
    iterations: u32,
    #[arg(long, default_value_t = 8)]
    section_mib: u64,
    #[arg(long, default_value_t = 256)]
    slab_mib: u64,
    #[arg(long, default_value_t = 1_000_000)]
    keyframe_interval: u32,
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    enable_task_logging: bool,
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

fn main() -> CuResult<()> {
    let args = Args::parse();
    let tmp_dir = tempfile::TempDir::new().expect("Could not create temporary benchmark directory");
    let logger_path = tmp_dir.path().join("async_cl_io_bench.copper");

    let bundled_config = <App as CuApplication<
        cu29::prelude::memmap::MmapSectionStorage,
        UnifiedLoggerWrite,
    >>::get_original_config();
    let mut config = CuConfig::deserialize_ron(&bundled_config)
        .expect("failed to deserialize bundled benchmark config");

    let logging = config
        .logging
        .get_or_insert_with(cu29::config::LoggingConfig::default);
    logging.enable_task_logging = args.enable_task_logging;
    logging.section_size_mib = Some(args.section_mib);
    logging.slab_size_mib = Some(args.slab_mib);
    logging.keyframe_interval = Some(args.keyframe_interval);
    let copperlist_count = logging.copperlist_count.unwrap_or(2);
    let graph = config.get_graph_mut(None)?;
    let source = graph
        .get_node_mut(0)
        .ok_or_else(|| CuError::from("Missing benchmark source"))?;
    source.set_param("payload_bytes", args.payload_bytes);
    source.set_param("spin_iters", args.spin_iters);

    let app = App::builder()
        .with_log_path(&logger_path, Some((args.slab_mib as usize) * 1024 * 1024))?
        .with_config(config)
        .build()?;
    let mut app = app.start()?;

    let output_start = Instant::now();
    for _ in 0..args.warmup {
        app.run_one_iteration()?;
    }

    let mut samples_ns = Vec::with_capacity(args.iterations as usize);
    let benchmark_start = Instant::now();
    let structured_records = args.structured_records;
    let structured_worker = (structured_records > 0).then(|| {
        std::thread::spawn(move || {
            for record in 0..structured_records {
                info!("Benchmark background record {}", record);
                std::thread::yield_now();
            }
        })
    });
    for _ in 0..args.iterations {
        let iter_start = Instant::now();
        app.run_one_iteration()?;
        let elapsed = iter_start.elapsed();
        samples_ns.push(elapsed.as_nanos() as u64);
    }
    let total_elapsed = benchmark_start.elapsed();

    if let Some(worker) = structured_worker {
        worker
            .join()
            .map_err(|_| CuError::from("Structured-log benchmark worker panicked"))?;
    }
    let mut stopped = app.stop()?.into_inner();
    let runtime = stopped.copper_runtime_mut();
    let dropped_copperlists = runtime.copperlists_manager.dropped_copperlists_total();
    let dropped_keyframes = runtime.copperlists_manager.dropped_keyframes_total();
    stopped.log_shutdown_completed()?;
    drop(stopped);
    let completed_elapsed = output_start.elapsed();
    let mut reader = UnifiedLoggerRead::new(&logger_path)
        .map_err(|e| CuError::new_with_cause("Cannot open benchmark log", e))?;
    if !reader.raw_main_header().clean_close {
        return Err(CuError::from("Benchmark output was not cleanly closed"));
    }
    let mut persisted_copperlists = 0u64;
    let mut persisted_keyframes = 0u64;
    let mut persisted_bytes = 0u64;
    let mut copperlist_ids = Vec::new();
    let mut keyframe_ids = Vec::new();
    loop {
        let (header, bytes) = reader.raw_read_section()?;
        if header.entry_type == UnifiedLogType::LastEntry {
            break;
        }
        persisted_bytes += bytes.len() as u64;
        let mut remaining = bytes.as_slice();
        while !remaining.is_empty() {
            let used = match header.entry_type {
                UnifiedLogType::CopperList => {
                    let (culist, used) = bincode::decode_from_slice::<default::CuList, _>(
                        remaining,
                        bincode::config::standard(),
                    )
                    .map_err(|e| {
                        CuError::new_with_cause("Cannot decode benchmark CopperList", e)
                    })?;
                    copperlist_ids.push(culist.id);
                    persisted_copperlists += 1;
                    used
                }
                UnifiedLogType::FrozenTasks => {
                    let (keyframe, used) =
                        bincode::decode_from_slice::<cu29::curuntime::KeyFrame, _>(
                            remaining,
                            bincode::config::standard(),
                        )
                        .map_err(|e| {
                            CuError::new_with_cause("Cannot decode benchmark keyframe", e)
                        })?;
                    keyframe_ids.push(keyframe.culistid);
                    persisted_keyframes += 1;
                    used
                }
                _ => remaining.len(),
            };
            if used == 0 {
                return Err(CuError::from("Benchmark reader made no progress"));
            }
            remaining = &remaining[used..];
        }
    }
    if args.enable_task_logging
        && persisted_copperlists + dropped_copperlists
            != u64::from(args.warmup) + u64::from(args.iterations)
    {
        return Err(CuError::from(
            "Persisted CopperList count does not match admissions and drops",
        ));
    }
    if args.enable_task_logging
        && keyframe_ids
            .iter()
            .any(|id| copperlist_ids.binary_search(id).is_err())
    {
        return Err(CuError::from(
            "Persisted keyframe has no associated CopperList",
        ));
    }

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
    let iter_per_sec = if total_elapsed.as_secs_f64() > 0.0 {
        samples / total_elapsed.as_secs_f64()
    } else {
        0.0
    };

    println!(
        "mode={} logging={} copperlist_count={} payload_bytes={} spin_iters={} section_mib={} slab_mib={} keyframe_interval={} warmup={} iterations={}",
        if cfg!(feature = "async-cl-io") {
            "async-cl-io"
        } else {
            "sync-cl-io"
        },
        args.enable_task_logging,
        copperlist_count,
        args.payload_bytes,
        args.spin_iters,
        args.section_mib,
        args.slab_mib,
        args.keyframe_interval,
        args.warmup,
        args.iterations
    );
    println!(
        "throughput_hz={:.2} mean_us={:.2} p50_us={} p90_us={} p99_us={} p999_us={} max_us={} delta_p50_us={} delta_p90_us={} delta_p99_us={} delta_p999_us={} delta_max_us={}",
        iter_per_sec,
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

    println!(
        "persisted_copperlists={persisted_copperlists} persisted_keyframes={persisted_keyframes} persisted_bytes={persisted_bytes} dropped_copperlists={dropped_copperlists} dropped_keyframes={dropped_keyframes} structured_records={} completed_ms={:.3} persisted_hz={:.2}",
        args.structured_records,
        completed_elapsed.as_secs_f64() * 1000.0,
        persisted_copperlists as f64 / completed_elapsed.as_secs_f64()
    );
    Ok(())
}
