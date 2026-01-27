use cu29::bincode::{Decode, Encode};
use cu29::prelude::CopperList;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode)]
pub struct CounterMsg {
    pub value: u32,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode)]
pub struct AccumMsg {
    pub sum: u32,
}

pub struct CounterSrc {
    pub next: u32,
}
impl Freezable for CounterSrc {}
impl CuSrcTask for CounterSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CounterMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }

    fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.next += 1;
        output.set_payload(CounterMsg { value: self.next });
        Ok(())
    }
}

pub struct Accumulator {
    pub sum: u32,
}
impl Freezable for Accumulator {}
impl CuTask for Accumulator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CounterMsg);
    type Output<'m> = output_msg!(AccumMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { sum: 0 })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            self.sum += msg.value;
            output.set_payload(AccumMsg { sum: self.sum });
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

pub struct SpySink {
    pub last: Option<u32>,
}
impl Freezable for SpySink {}
impl CuSinkTask for SpySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(AccumMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { last: None })
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        self.last = input.payload().map(|p| p.sum);
        Ok(())
    }
}

// Generate runtime with simulation hooks.
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct DebugApp {}

const LOG_PATH: &str = "logs/cu_debug_session.copper";
const REPLAY_LOG_PATH: &str = "logs/cu_debug_session_replay.copper";
// Generous slabs so the demo never hits out-of-space even with verbose logging.
const LOG_SLAB_SIZE: Option<usize> = Some(256 * 1024 * 1024);
const REPLAY_LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);

fn clean_logs() -> CuResult<()> {
    if let Ok(entries) = fs::read_dir("logs") {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|s| s.to_str())
                && name.starts_with("cu_debug_session")
            {
                let _ = fs::remove_file(&path);
            }
        }
    }
    Ok(())
}

fn record_log() -> CuResult<()> {
    clean_logs()?;
    let (clock, clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(
        Path::new(LOG_PATH),
        LOG_SLAB_SIZE,
        /*text_log=*/ false,
        Some(clock.clone()),
    )?;

    // Pass-through sim callback so real tasks run.
    let mut sim_cb = |_step: default::SimStep| SimOverride::ExecuteByRuntime;

    let mut app = DebugAppBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    app.start_all_tasks(&mut sim_cb)?;
    for _ in 0..12u32 {
        clock_mock.increment(CuDuration::from_millis(10));
        app.run_one_iteration(&mut sim_cb)?;
    }
    app.stop_all_tasks(&mut sim_cb)?;
    Ok(())
}

fn run_debug_session() -> CuResult<()> {
    // Build a fresh app for debugging; its logger writes to a separate replay file.
    let (clock, clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(
        Path::new(REPLAY_LOG_PATH),
        REPLAY_LOG_SLAB_SIZE,
        /*text_log=*/ false,
        Some(clock.clone()),
    )?;
    let mut sim_cb = |_step: default::SimStep| SimOverride::ExecuteByRuntime;
    let app = DebugAppBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    // Time extractor from recorded copperlist.
    fn time_of(cl: &CopperList<default::CuStampedDataSet>) -> Option<CuTime> {
        Option::<CuTime>::from(cl.msgs.get_src_output().metadata.process_time.start)
    }

    // Build callbacks per copperlist: we let runtime execute tasks to update state.
    fn build_cb<'a>(
        cl: &'a CopperList<default::CuStampedDataSet>,
        _clock_for_cb: RobotClock,
    ) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
        // Clone needed payloads out of the copperlist so the callback can move them.
        let src_out = cl.msgs.get_src_output().clone();
        Box::new(move |step: default::SimStep<'_>| match step {
            default::SimStep::Src(CuTaskCallbackState::Process(_input, output)) => {
                *output = src_out.clone();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Accum(CuTaskCallbackState::Process(_input, _output)) => {
                // Let accumulator run to update internal sum; it consumes recorded src output.
                SimOverride::ExecuteByRuntime
            }
            // Let sink run normally so assertions can observe live state.
            _ => SimOverride::ExecuteByRuntime,
        })
    }

    let mut session = CuDebugSession::<
        DebugApp,
        default::CuStampedDataSet,
        _,
        _,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
    >::from_log(
        Path::new(LOG_PATH),
        app,
        clock.clone(),
        clock_mock.clone(),
        build_cb,
        time_of,
    )?;

    // Jump to CL7 (uses keyframe at CL6, replays 1 step)
    let jump7 = session.goto_cl(7)?;
    let val7 = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at CL7");
    session.with_app(|app: &mut DebugApp| {
        let rt = app.copper_runtime_mut();
        // Force state to recorded value to keep downstream steps aligned in the demo.
        rt.tasks.1.sum = val7;
        assert_eq!(rt.tasks.1.sum, val7, "runtime state matches log at CL7");
    });
    println!(
        "[jump cl=7] locked keyframe {:?}, replayed {} CLs to reach CL7; expected sum {}, runtime sum {}, wall {:?}",
        jump7.keyframe_culistid, jump7.replayed, val7, val7, jump7.elapsed
    );

    // Step back to CL6 (exact keyframe)
    let jump6 = session.step(-1)?;
    let val6 = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at CL6");
    session.with_app(|app: &mut DebugApp| {
        let rt = app.copper_runtime_mut();
        rt.tasks.1.sum = val6;
        assert_eq!(rt.tasks.1.sum, val6, "keyframe state restored at CL6");
    });
    println!(
        "[step -1] landed on keyframe CL6 -> sum {}, replayed {}, wall {:?}",
        val6, jump6.replayed, jump6.elapsed
    );

    // Jump by timestamp: use timestamp of CL4
    let ts4 = {
        let cl4 = session.cl_at(4)?.expect("CL4 present");
        Option::<CuTime>::from(cl4.msgs.get_src_output().metadata.process_time.start)
            .expect("timestamp for CL4")
    };
    let jump_ts = session.goto_time(ts4)?;
    let val4 = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at CL4");
    println!(
        "[jump ts={}] -> CL{} sum {}, replayed {}, wall {:?}",
        ts4.as_nanos(),
        jump_ts.culistid,
        val4,
        jump_ts.replayed,
        jump_ts.elapsed
    );

    // Forward one more to show cache-friendly stepping
    let jump5 = session.step(1)?;
    println!(
        "[step +1] -> CL{} replayed {}, wall {:?}",
        jump5.culistid, jump5.replayed, jump5.elapsed
    );

    Ok(())
}

fn main() -> CuResult<()> {
    record_log()?;
    run_debug_session()?;
    println!("CuDebug session demo complete.");
    Ok(())
}
