pub mod tasks;
use cu29::prelude::*;
use cu29::prelude::app::CuSimApplication;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_helpers::basic_copper_setup;
use default::SimStep::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Src};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use std::path::{Path, PathBuf};

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct CaterpillarReSim {}

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        Src(_) | Gpio0(_) | Gpio1(_) | Gpio2(_) | Gpio3(_) | Gpio4(_) | Gpio5(_) | Gpio6(_)
        | Gpio7(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}
fn run_one_copperlist(
    copper_app: &mut CaterpillarReSim,
    robot_clock: &mut RobotClockMock,
    copper_list: CopperList<default::CuStampedDataSet>,
    pending_kf_ts: Option<CuDuration>,
) {
    // Sync the copper clock to the recorded physics clock and replay every task's output
    // byte-for-byte from the captured copperlist so we don't re-stamp metadata.
    let msgs = copper_list.msgs;

    if let Some(CuDuration(ts)) = pending_kf_ts {
        // Align clock exactly to the recorded keyframe timestamp for this CL
        // Also force the runtime to stamp the keyframe with this timestamp so it matches byte-for-byte.
        copper_app
            .copper_runtime
            .keyframes_manager
            .set_forced_timestamp(CuDuration(ts));
        robot_clock.set_value(ts);
    } else {
        let CuDuration(process_time) = msgs.get_src_output().metadata.process_time.start.unwrap();
        robot_clock.set_value(process_time);
    }

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        use default::SimStep::*;
        use CuTaskCallbackState::*;
        match step {
            Src(Process(_, output)) => {
                *output = msgs.get_src_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct0(Process(_, output)) => {
                *output = msgs.get_ct_0_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct1(Process(_, output)) => {
                *output = msgs.get_ct_1_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct2(Process(_, output)) => {
                *output = msgs.get_ct_2_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct3(Process(_, output)) => {
                *output = msgs.get_ct_3_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct4(Process(_, output)) => {
                *output = msgs.get_ct_4_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct5(Process(_, output)) => {
                *output = msgs.get_ct_5_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct6(Process(_, output)) => {
                *output = msgs.get_ct_6_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct7(Process(_, output)) => {
                *output = msgs.get_ct_7_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio0(Process(_, output)) => {
                *output = msgs.get_gpio_0_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio1(Process(_, output)) => {
                *output = msgs.get_gpio_1_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio2(Process(_, output)) => {
                *output = msgs.get_gpio_2_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio3(Process(_, output)) => {
                *output = msgs.get_gpio_3_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio4(Process(_, output)) => {
                *output = msgs.get_gpio_4_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio5(Process(_, output)) => {
                *output = msgs.get_gpio_5_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio6(Process(_, output)) => {
                *output = msgs.get_gpio_6_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio7(Process(_, output)) => {
                *output = msgs.get_gpio_7_output().clone();
                SimOverride::ExecutedBySim
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("Failed to run application.");
}

fn main() {
    // Create the Copper App in simulation mode.
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/caterpillarresim.copper";
    let (robot_clock, mut robot_clock_mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");

    let mut copper_app = CaterpillarReSimBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("Failed to create runtime.");

    copper_app
        .start_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");

    // Restore tasks from the first keyframe so sim starts from the recorded state.
    let UnifiedLogger::Read(dl_kf) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("logs/caterpillar.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };
    let mut keyframes_ioreader = UnifiedLoggerIOReader::new(dl_kf, UnifiedLogType::FrozenTasks);
    let mut kf_iter = keyframes_reader(&mut keyframes_ioreader).peekable();

    if let Some(first_kf) = kf_iter.peek() {
        <CaterpillarReSim as CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite>>::restore_keyframe(
            &mut copper_app,
            first_kf,
        )
        .expect("Failed to restore keyframe state");
        let CuDuration(ts) = first_kf.timestamp;
        robot_clock_mock.set_value(ts);
    }

    // Read back the logs from a previous run, applying keyframes exactly at their culistid.
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("logs/caterpillar.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };

    let mut copperlists = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let cl_iter = copperlists_reader::<default::CuStampedDataSet>(&mut copperlists);
    for entry in cl_iter {
        // Apply next keyframe if it corresponds to this CL id
        let pending_kf_ts = if let Some(kf) = kf_iter.peek() {
            if kf.culistid == entry.id {
                let ts = kf.timestamp;
                <CaterpillarReSim as CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite>>::restore_keyframe(
                    &mut copper_app,
                    kf,
                )
                .expect("Failed to restore keyframe state");
                kf_iter.next();
                Some(ts)
            } else {
                None
            }
        } else {
            None
        };

        run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry, pending_kf_ts);
    }

    // let cl_iter = copperlists_reader::<default::CuStampedDataSet>(&mut copperlists_reader);
    // for entry in cl_iter {
    //     println!("{entry:#?}");
    //     run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry);
    // }
    // copper_app
    //     .stop_all_tasks(&mut default_callback)
    //     .expect("Failed to stop all tasks.");
}
