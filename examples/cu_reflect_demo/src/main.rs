use cu_pid::{GenericPIDTask, PIDControlOutputPayload};
use cu29::bincode::{Decode, Encode};
use cu29::prelude::CopperList;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

const LOG_PATH: &str = "logs/cu_reflect_demo.copper";
const REPLAY_LOG_PATH: &str = "logs/cu_reflect_demo_replay.copper";
const LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);

pub mod tasks {
    use super::*;

    #[derive(Default, Debug, Clone, Serialize, Deserialize, Encode, Decode, Reflect)]
    pub struct MeasurementPayload {
        pub value: f32,
    }

    impl From<&MeasurementPayload> for f32 {
        fn from(value: &MeasurementPayload) -> Self {
            value.value
        }
    }

    #[derive(Debug, Reflect)]
    pub struct MeasurementSource {
        pub next_measurement: f32,
    }

    impl Freezable for MeasurementSource {}

    impl CuSrcTask for MeasurementSource {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(MeasurementPayload);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                next_measurement: 0.75,
            })
        }

        fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
            self.next_measurement += 0.05;
            output.tov = Some(clock.now()).into();
            output.set_payload(MeasurementPayload {
                value: self.next_measurement,
            });
            Ok(())
        }
    }

    pub type PidTask = GenericPIDTask<MeasurementPayload>;

    #[derive(Default, Debug, Reflect)]
    pub struct OutputSink {
        pub last_output: Option<f32>,
    }

    impl Freezable for OutputSink {}

    impl CuSinkTask for OutputSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(PIDControlOutputPayload);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self::default())
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            self.last_output = input.payload().map(|payload| payload.output);
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct ReflectDemo {}

fn clean_logs() {
    if let Ok(entries) = fs::read_dir("logs") {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|s| s.to_str())
                && name.starts_with("cu_reflect_demo")
            {
                let _ = fs::remove_file(path);
            }
        }
    }
}

fn record_log() -> CuResult<()> {
    clean_logs();
    let (clock, clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(
        Path::new(LOG_PATH),
        LOG_SLAB_SIZE,
        /* text_log */ false,
        Some(clock.clone()),
    )?;

    let mut sim_cb = |_step: default::SimStep| SimOverride::ExecuteByRuntime;
    let mut app = ReflectDemoBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    app.start_all_tasks(&mut sim_cb)?;
    for _ in 0..8 {
        clock_mock.increment(CuDuration::from_millis(10));
        app.run_one_iteration(&mut sim_cb)?;
    }
    app.stop_all_tasks(&mut sim_cb)?;
    Ok(())
}

fn run_reflect_debug_demo() -> CuResult<()> {
    let (clock, clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(
        Path::new(REPLAY_LOG_PATH),
        LOG_SLAB_SIZE,
        /* text_log */ false,
        Some(clock.clone()),
    )?;

    let mut sim_cb = |_step: default::SimStep| SimOverride::ExecuteByRuntime;
    let app = ReflectDemoBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut sim_cb)
        .build()?;

    fn time_of(cl: &CopperList<default::CuStampedDataSet>) -> Option<CuTime> {
        Option::<CuTime>::from(cl.msgs.get_src_output().metadata.process_time.start)
    }

    fn build_cb<'a>(
        cl: &'a CopperList<default::CuStampedDataSet>,
        _clock_for_cb: RobotClock,
    ) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
        let src_out = cl.msgs.get_src_output().clone();
        Box::new(move |step: default::SimStep<'_>| match step {
            default::SimStep::Src(CuTaskCallbackState::Process(_input, output)) => {
                *output = src_out.clone();
                SimOverride::ExecutedBySim
            }
            _ => SimOverride::ExecuteByRuntime,
        })
    }

    let mut session = CuDebugSession::<
        ReflectDemo,
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

    session.goto_cl(4)?;

    println!(
        "Registered reflected task schemas:\n{}",
        session.dump_reflected_task_schemas()
    );

    println!(
        "Reflected `src` task:\n{}",
        session.dump_reflected_task("src")?
    );
    println!(
        "Reflected `pid` task:\n{}",
        session.dump_reflected_task("pid")?
    );
    println!(
        "Reflected `sink` task:\n{}",
        session.dump_reflected_task("sink")?
    );

    Ok(())
}

fn main() -> CuResult<()> {
    record_log()?;
    run_reflect_debug_demo()
}
