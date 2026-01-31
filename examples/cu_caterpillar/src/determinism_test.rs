//! Determinism contract unit test.
//!
//! Contract:
//!   1) record A == record B  (same inputs, same deterministic clock)
//!   2) record A == resim(A)  (deterministic replay)

use cu_rp_gpio::RPGpioPayload;
use cu29::bincode;
use cu29::prelude::*;
use cu29_export::copperlists_reader;
use cu29_helpers::basic_copper_setup;

use crate::tasks;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Serialize this test even if the harness runs test threads > 1.
static DET_LOCK: Mutex<()> = Mutex::new(());

/// Ensure unique output dirs even across multiple determinism tests in the same process.
static RUN_ID: AtomicUsize = AtomicUsize::new(0);

// Keep the log slab big enough to satisfy the config's copperlist section size.
const DET_LOG_SLAB_SIZE: Option<usize> = Some(256 * 1024 * 1024);

// Put the runtime in sim_mode so we can drive a fixed number of iterations deterministically.
// IMPORTANT: define this inside the module to avoid name collisions with the "real" app in main.rs.
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct CaterpillarDeterminismApp {}

fn out_root_dir() -> PathBuf {
    // Prefer a stable repo-local path so CI can upload artifacts on failure.
    // If you set CARGO_TARGET_DIR in CI, you can switch to that.
    let base = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target"));

    base.join("determinism_tests")
}

fn fresh_case_dir(case: &str) -> PathBuf {
    let rid = RUN_ID.fetch_add(1, Ordering::SeqCst);
    let dir = out_root_dir().join(format!("{}_pid{}_{}", case, std::process::id(), rid));
    if dir.exists() {
        let _ = fs::remove_dir_all(&dir);
    }
    fs::create_dir_all(&dir).expect("failed to create determinism output dir");
    dir
}

fn record_run(log_base: &Path, iterations: usize, dt_ticks: u64) -> CuResult<()> {
    if let Some(parent) = log_base.parent() {
        fs::create_dir_all(parent).ok();
    }

    let (clock, clock_mock) = RobotClock::mock();

    let slab_size = DET_LOG_SLAB_SIZE;

    let ctx = basic_copper_setup(
        log_base,
        slab_size,
        /*text_log=*/ false,
        Some(clock.clone()),
    )?;
    let mut sim_state = true;
    let clock_for_sim = clock.clone();
    let mut record_callback = move |step: default::SimStep| -> SimOverride {
        use default::SimStep::*;

        match step {
            Src(CuTaskCallbackState::Process(_, output)) => {
                sim_state = !sim_state;
                let now = clock_for_sim.now();
                output.set_payload(RPGpioPayload { on: sim_state });
                output.tov = Tov::Time(now);
                output.metadata.process_time.start = now.into();
                output.metadata.process_time.end = now.into();
                output.metadata.set_status(sim_state);
                SimOverride::ExecutedBySim
            }
            Src(_) => SimOverride::ExecutedBySim,
            Gpio0(CuTaskCallbackState::Process(input, output))
            | Gpio1(CuTaskCallbackState::Process(input, output))
            | Gpio2(CuTaskCallbackState::Process(input, output))
            | Gpio3(CuTaskCallbackState::Process(input, output))
            | Gpio4(CuTaskCallbackState::Process(input, output))
            | Gpio5(CuTaskCallbackState::Process(input, output))
            | Gpio6(CuTaskCallbackState::Process(input, output))
            | Gpio7(CuTaskCallbackState::Process(input, output)) => {
                let now = clock_for_sim.now();
                output.tov = input.tov;
                output.metadata.process_time.start = now.into();
                output.metadata.process_time.end = now.into();
                if let Some(payload) = input.payload() {
                    output.metadata.set_status(payload.on);
                }
                SimOverride::ExecutedBySim
            }
            Gpio0(_) | Gpio1(_) | Gpio2(_) | Gpio3(_) | Gpio4(_) | Gpio5(_) | Gpio6(_)
            | Gpio7(_) => SimOverride::ExecutedBySim,
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    let mut app = CaterpillarDeterminismAppBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut record_callback)
        .build()
        .expect("failed to build app");

    app.start_all_tasks(&mut record_callback)
        .expect("failed to start tasks");

    for i in 0..iterations {
        clock_mock.set_value(dt_ticks.saturating_mul(i as u64));
        app.run_one_iteration(&mut record_callback)
            .expect("run_one_iteration failed");
    }

    app.stop_all_tasks(&mut record_callback)
        .expect("failed to stop tasks");

    Ok(())
}

fn read_copperlist_stream_encoded(log_base: &Path) -> CuResult<Vec<Vec<u8>>> {
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .expect("failed to open log for read")
    else {
        panic!("expected read logger");
    };

    let mut io_reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let iter = copperlists_reader::<default::CuStampedDataSet>(&mut io_reader);

    let mut out = Vec::new();
    for cl in iter {
        let bytes = bincode::encode_to_vec(cl, bincode::config::standard())
            .expect("failed to bincode-encode copperlist");
        out.push(bytes);
    }
    Ok(out)
}

fn resim_one_copperlist(
    app: &mut CaterpillarDeterminismApp,
    robot_clock_mock: &mut RobotClockMock,
    copper_list: CopperList<default::CuStampedDataSet>,
) {
    use default::SimStep::*;

    let msgs = &copper_list.msgs;

    // Sync clock to the recorded source output.
    let CuDuration(ticks) = msgs.get_src_output().metadata.process_time.start.unwrap();
    robot_clock_mock.set_value(ticks);

    let mut cb = move |step: default::SimStep| -> SimOverride {
        match step {
            // Stub source: inject recorded src output.
            Src(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_src_output().clone();
                SimOverride::ExecutedBySim
            }
            Src(_) => SimOverride::ExecutedBySim,

            // Stub sinks: keep metadata deterministic without touching hardware.
            Gpio0(CuTaskCallbackState::Process(input, output))
            | Gpio1(CuTaskCallbackState::Process(input, output))
            | Gpio2(CuTaskCallbackState::Process(input, output))
            | Gpio3(CuTaskCallbackState::Process(input, output))
            | Gpio4(CuTaskCallbackState::Process(input, output))
            | Gpio5(CuTaskCallbackState::Process(input, output))
            | Gpio6(CuTaskCallbackState::Process(input, output))
            | Gpio7(CuTaskCallbackState::Process(input, output)) => {
                let now = robot_clock_mock.now();
                output.tov = input.tov;
                output.metadata.process_time.start = now.into();
                output.metadata.process_time.end = now.into();
                if let Some(payload) = input.payload() {
                    output.metadata.set_status(payload.on);
                }
                SimOverride::ExecutedBySim
            }
            Gpio0(_) | Gpio1(_) | Gpio2(_) | Gpio3(_) | Gpio4(_) | Gpio5(_) | Gpio6(_)
            | Gpio7(_) => SimOverride::ExecutedBySim,

            // Everything else: runtime executes.
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    app.run_one_iteration(&mut cb)
        .expect("resim run_one_iteration failed");
}

fn resim_run(input_log_base: &Path, output_log_base: &Path) -> CuResult<()> {
    if let Some(parent) = output_log_base.parent() {
        fs::create_dir_all(parent).ok();
    }

    let (clock, mut clock_mock) = RobotClock::mock();
    let slab_size = DET_LOG_SLAB_SIZE;
    let ctx = basic_copper_setup(
        output_log_base,
        slab_size,
        /*text_log=*/ false,
        Some(clock.clone()),
    )?;

    fn init_cb(_step: default::SimStep) -> SimOverride {
        SimOverride::ExecuteByRuntime
    }

    let mut app = CaterpillarDeterminismAppBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut init_cb)
        .build()
        .expect("failed to build resim app");

    app.start_all_tasks(&mut init_cb)
        .expect("failed to start tasks (resim)");

    // Read recorded copperlists and replay them.
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(input_log_base)
        .build()
        .expect("failed to open input log for resim")
    else {
        panic!("expected read logger for input");
    };

    let mut io_reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let iter = copperlists_reader::<default::CuStampedDataSet>(&mut io_reader);

    for cl in iter {
        resim_one_copperlist(&mut app, &mut clock_mock, cl);
    }

    app.stop_all_tasks(&mut init_cb)
        .expect("failed to stop tasks (resim)");

    Ok(())
}

fn assert_streams_equal(label_a: &str, a: &[Vec<u8>], label_b: &str, b: &[Vec<u8>]) {
    assert_eq!(
        a.len(),
        b.len(),
        "determinism failure: stream length differs ({}={}, {}={})",
        label_a,
        a.len(),
        label_b,
        b.len()
    );

    for (i, (item_a, item_b)) in a.iter().zip(b.iter()).enumerate() {
        if item_a != item_b {
            panic!(
                "determinism failure: mismatch at copperlist index {} ({} vs {})",
                i, label_a, label_b
            );
        }
    }
}

#[test]
fn determinism_record_and_resim() {
    // Ensure this test is serialized (important if other tests exist).
    let _guard = DET_LOCK.lock().unwrap();

    // Allow overriding iteration count from CI without code changes.
    let iterations: usize = std::env::var("COPPER_DETERMINISM_ITERS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| {
            // Keep it light for local debug builds.
            if cfg!(debug_assertions) { 256 } else { 1024 }
        });

    let dt_ticks: u64 = std::env::var("COPPER_DETERMINISM_DT_TICKS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);

    let case_dir = fresh_case_dir("cu_caterpillar");
    let a_base = case_dir.join("record_a.copper");
    let b_base = case_dir.join("record_b.copper");
    let r_base = case_dir.join("resim_a.copper");

    // 1) record A and B
    record_run(&a_base, iterations, dt_ticks).expect("record A failed");
    record_run(&b_base, iterations, dt_ticks).expect("record B failed");

    let a_stream = read_copperlist_stream_encoded(&a_base).expect("read A failed");
    let b_stream = read_copperlist_stream_encoded(&b_base).expect("read B failed");

    // 2) A == B
    assert_streams_equal("record_a", &a_stream, "record_b", &b_stream);

    // 3) resim(A)
    resim_run(&a_base, &r_base).expect("resim(A) failed");
    let r_stream = read_copperlist_stream_encoded(&r_base).expect("read resim failed");

    // 4) A == resim(A)
    assert_streams_equal("record_a", &a_stream, "resim_a", &r_stream);

    // If you want to keep artifacts even on success, comment this out.
    let _ = fs::remove_dir_all(case_dir);
}
