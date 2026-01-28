use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::bincode::{Decode, Encode};
use cu29::prelude::CopperList;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::time::Instant;

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
impl Freezable for CounterSrc {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}
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
impl Freezable for Accumulator {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.sum, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.sum = Decode::decode(decoder)?;
        Ok(())
    }
}
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
impl Freezable for SpySink {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.last, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.last = Decode::decode(decoder)?;
        Ok(())
    }
}
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
    // Record enough iterations to span multiple log sections and exercise indexing.
    for _ in 0..512u32 {
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
        // `clock` is the handle the runtime reads; `clock_mock` lets the debugger
        // set deterministic timestamps for replay and time-based seeks.
        clock.clone(),
        clock_mock.clone(),
        build_cb,
        time_of,
    )?;

    // Jump to CL7 (uses keyframe at CL6, replays 1 step)
    let t0 = Instant::now();
    let jump7 = session.goto_cl(7)?;
    let wall7 = t0.elapsed();
    let val7 = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at CL7");
    session.with_app(|app: &mut DebugApp| {
        let rt = app.copper_runtime_mut();
        assert_eq!(rt.tasks.1.sum, val7, "runtime state matches log at CL7");
    });
    println!(
        "[jump cl=7] locked keyframe {:?}, replayed {} CLs to reach CL7; expected sum {}, runtime sum {}, wall {:?}",
        jump7.keyframe_culistid, jump7.replayed, val7, val7, wall7
    );

    // Step back to CL6 (exact keyframe)
    let t0 = Instant::now();
    let jump6 = session.step(-1)?;
    let wall6 = t0.elapsed();
    let val6 = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at CL6");
    session.with_app(|app: &mut DebugApp| {
        let rt = app.copper_runtime_mut();
        assert_eq!(rt.tasks.1.sum, val6, "keyframe state restored at CL6");
    });
    println!(
        "[step -1] landed on keyframe CL6 -> sum {}, replayed {}, wall {:?}",
        val6, jump6.replayed, wall6
    );

    // Jump by timestamp: use timestamp of CL4
    let ts4 = {
        let cl4 = session.cl_at(4)?.expect("CL4 present");
        Option::<CuTime>::from(cl4.msgs.get_src_output().metadata.process_time.start)
            .expect("timestamp for CL4")
    };
    let t0 = Instant::now();
    let jump_ts = session.goto_time(ts4)?;
    let wall_ts = t0.elapsed();
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
        wall_ts
    );

    // Forward one more to show cache-friendly stepping
    let t0 = Instant::now();
    let jump5 = session.step(1)?;
    let wall5 = t0.elapsed();
    println!(
        "[step +1] -> CL{} replayed {}, wall {:?}",
        jump5.culistid, jump5.replayed, wall5
    );

    // Jump near the end of the log to exercise multi-section indexing.
    let mut last_idx = 0usize;
    while session.cl_at(last_idx + 1)?.is_some() {
        last_idx += 1;
    }
    let last_id = session.cl_at(last_idx)?.expect("last CL present").id;

    let t0 = Instant::now();
    let jump_last = session.goto_cl(last_id)?;
    let wall_last = t0.elapsed();
    let val_last = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at last CL");
    session.with_app(|app: &mut DebugApp| {
        let rt = app.copper_runtime_mut();
        assert_eq!(
            rt.tasks.1.sum, val_last,
            "runtime state matches log at last CL"
        );
    });
    println!(
        "[jump end cl={}] keyframe {:?}, replayed {}, sum {}, wall {:?}",
        jump_last.culistid, jump_last.keyframe_culistid, jump_last.replayed, val_last, wall_last
    );

    let t0 = Instant::now();
    let back_one = session.step(-1)?;
    let wall_back = t0.elapsed();
    let val_back = session
        .current_cl()?
        .as_ref()
        .and_then(|cl| cl.msgs.get_accum_output().payload())
        .map(|p| p.sum)
        .expect("payload at CL before last");
    println!(
        "[step -1 from end] -> CL{} sum {}, replayed {}, wall {:?}",
        back_one.culistid, val_back, back_one.replayed, wall_back
    );

    Ok(())
}

fn main() -> CuResult<()> {
    record_log()?;
    run_debug_session()?;
    println!("CuDebug session demo complete.");
    Ok(())
}
