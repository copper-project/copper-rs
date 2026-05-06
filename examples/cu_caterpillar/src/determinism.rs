use crate::tasks;
use cu_rp_gpio::RPGpioPayload;
use cu29::bincode;
use cu29::prelude::*;
use cu29_export::{copperlists_reader, keyframes_reader};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

static DET_LOCK: Mutex<()> = Mutex::new(());
static RUN_ID: AtomicUsize = AtomicUsize::new(0);
const DET_LOG_SLAB_SIZE: Option<usize> = Some(256 * 1024 * 1024);

#[copper_runtime(config = "config/copperconfig_determinism.ron", sim_mode = true)]
struct CaterpillarDeterminismApp {}

fn out_root_dir() -> PathBuf {
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

    let mut app = CaterpillarDeterminismApp::builder()
        .with_clock(clock.clone())
        .with_log_path(log_base, slab_size)?
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

fn read_keyframe_stream_encoded(log_base: &Path) -> CuResult<Vec<Vec<u8>>> {
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .expect("failed to open log for read")
    else {
        panic!("expected read logger");
    };

    let mut io_reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::FrozenTasks);
    let iter = keyframes_reader(&mut io_reader);

    let mut out = Vec::new();
    for kf in iter {
        let bytes = bincode::encode_to_vec(kf, bincode::config::standard())
            .expect("failed to bincode-encode keyframe");
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
    let ticks = msgs.get_src_output().metadata.process_time.start.unwrap();
    robot_clock_mock.set_value(ticks.as_nanos());

    let mut cb = move |step: default::SimStep| -> SimOverride {
        match step {
            Src(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_src_output().clone();
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
    fn init_cb(_step: default::SimStep) -> SimOverride {
        SimOverride::ExecuteByRuntime
    }

    let mut app = CaterpillarDeterminismApp::builder()
        .with_clock(clock.clone())
        .with_log_path(output_log_base, slab_size)?
        .with_sim_callback(&mut init_cb)
        .build()
        .expect("failed to build resim app");

    app.start_all_tasks(&mut init_cb)
        .expect("failed to start tasks (resim)");

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

#[cfg_attr(all(test, feature = "determinism_ci"), test)]
#[safety_case("DET-TEST-001")]
pub fn determinism_record_and_resim() {
    let _guard = DET_LOCK.lock().unwrap();

    let iterations: usize = std::env::var("COPPER_DETERMINISM_ITERS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| if cfg!(debug_assertions) { 256 } else { 1024 });

    let dt_ticks: u64 = std::env::var("COPPER_DETERMINISM_DT_TICKS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);

    let case_dir = fresh_case_dir("cu_caterpillar");
    let a_base = case_dir.join("record_a.copper");
    let b_base = case_dir.join("record_b.copper");
    let r_base = case_dir.join("resim_a.copper");

    record_run(&a_base, iterations, dt_ticks).expect("record A failed");
    record_run(&b_base, iterations, dt_ticks).expect("record B failed");

    let a_stream = read_copperlist_stream_encoded(&a_base).expect("read A failed");
    let b_stream = read_copperlist_stream_encoded(&b_base).expect("read B failed");
    let a_keyframes = read_keyframe_stream_encoded(&a_base).expect("read A keyframes failed");
    let b_keyframes = read_keyframe_stream_encoded(&b_base).expect("read B keyframes failed");

    safety_check_eq!(
        "DET-TEST-001-C1",
        "DET-REQ-001",
        "record A copperlists equal record B copperlists",
        &a_stream,
        &b_stream,
    );
    safety_check!(
        "DET-TEST-001-C2",
        "DET-REQ-002",
        "record A emits at least one keyframe",
        !a_keyframes.is_empty(),
    );
    safety_check_eq!(
        "DET-TEST-001-C3",
        "DET-REQ-002",
        "record A keyframes equal record B keyframes",
        &a_keyframes,
        &b_keyframes,
    );

    resim_run(&a_base, &r_base).expect("resim(A) failed");
    let r_stream = read_copperlist_stream_encoded(&r_base).expect("read resim failed");
    let r_keyframes = read_keyframe_stream_encoded(&r_base).expect("read resim keyframes failed");

    safety_check_eq!(
        "DET-TEST-001-C4",
        "DET-REQ-003",
        "replay A copperlists equal record A copperlists",
        &a_stream,
        &r_stream,
    );
    safety_check_eq!(
        "DET-TEST-001-C5",
        "DET-REQ-004",
        "replay A keyframes equal record A keyframes",
        &a_keyframes,
        &r_keyframes,
    );

    let _ = fs::remove_dir_all(case_dir);
}

#[cfg(feature = "safety-ids")]
pub fn link_safety_ids() {
    let _ = determinism_record_and_resim as fn();
}
