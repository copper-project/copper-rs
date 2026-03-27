use cu_distributed_resim_demo::ensure_parent_dir;
use cu29::distributed_replay::{DistributedReplayAssignment, DistributedReplayPlan};
use cu29::prelude::*;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

mod scout_replay {
    use cu_distributed_resim_demo::{
        compare_streams, read_copperlist_stream_encoded, read_keyframe_stream_encoded,
    };
    use cu29::distributed_replay::{DistributedReplayAssignment, DistributedReplayBuilder};
    use cu29::prelude::*;
    use std::path::Path;

    pub mod bridges {
        pub use cu_distributed_resim_demo::bridges::*;
    }

    pub mod messages {
        pub use cu_distributed_resim_demo::messages::*;
    }

    pub mod tasks {
        pub use cu_distributed_resim_demo::tasks::*;
    }

    #[copper_runtime(config = "multi_copper.ron", subsystem = "scout", sim_mode = true)]
    struct ScoutReplayApp {}

    pub fn register(builder: DistributedReplayBuilder) -> CuResult<DistributedReplayBuilder> {
        builder.register::<ScoutReplayApp>("scout")
    }

    pub fn compare(
        assignment: &DistributedReplayAssignment,
        replay_log_base: &Path,
    ) -> CuResult<()> {
        let label = format!(
            "instance {} subsystem '{}'",
            assignment.instance_id, assignment.subsystem_id
        );
        let original_copperlists =
            read_copperlist_stream_encoded::<default::CuStampedDataSet>(&assignment.log.base_path)?;
        let replayed_copperlists =
            read_copperlist_stream_encoded::<default::CuStampedDataSet>(replay_log_base)?;
        compare_streams(
            &format!("{label} copperlists"),
            &original_copperlists,
            &replayed_copperlists,
        )?;

        let original_keyframes = read_keyframe_stream_encoded(&assignment.log.base_path)?;
        let replayed_keyframes = read_keyframe_stream_encoded(replay_log_base)?;
        compare_streams(
            &format!("{label} keyframes"),
            &original_keyframes,
            &replayed_keyframes,
        )
    }
}

mod plan_replay {
    use cu_distributed_resim_demo::{
        compare_streams, read_copperlist_stream_encoded, read_keyframe_stream_encoded,
    };
    use cu29::distributed_replay::{DistributedReplayAssignment, DistributedReplayBuilder};
    use cu29::prelude::*;
    use std::path::Path;

    pub mod bridges {
        pub use cu_distributed_resim_demo::bridges::*;
    }

    pub mod messages {
        pub use cu_distributed_resim_demo::messages::*;
    }

    pub mod tasks {
        pub use cu_distributed_resim_demo::tasks::*;
    }

    #[copper_runtime(config = "multi_copper.ron", subsystem = "plan", sim_mode = true)]
    struct PlanReplayApp {}

    pub fn register(builder: DistributedReplayBuilder) -> CuResult<DistributedReplayBuilder> {
        builder.register::<PlanReplayApp>("plan")
    }

    pub fn compare(
        assignment: &DistributedReplayAssignment,
        replay_log_base: &Path,
    ) -> CuResult<()> {
        let label = format!(
            "instance {} subsystem '{}'",
            assignment.instance_id, assignment.subsystem_id
        );
        let original_copperlists =
            read_copperlist_stream_encoded::<default::CuStampedDataSet>(&assignment.log.base_path)?;
        let replayed_copperlists =
            read_copperlist_stream_encoded::<default::CuStampedDataSet>(replay_log_base)?;
        compare_streams(
            &format!("{label} copperlists"),
            &original_copperlists,
            &replayed_copperlists,
        )?;

        let original_keyframes = read_keyframe_stream_encoded(&assignment.log.base_path)?;
        let replayed_keyframes = read_keyframe_stream_encoded(replay_log_base)?;
        compare_streams(
            &format!("{label} keyframes"),
            &original_keyframes,
            &replayed_keyframes,
        )
    }
}

mod control_replay {
    use cu_distributed_resim_demo::{
        compare_streams, read_copperlist_stream_encoded, read_keyframe_stream_encoded,
    };
    use cu29::distributed_replay::{DistributedReplayAssignment, DistributedReplayBuilder};
    use cu29::prelude::*;
    use std::path::Path;

    pub mod bridges {
        pub use cu_distributed_resim_demo::bridges::*;
    }

    pub mod messages {
        pub use cu_distributed_resim_demo::messages::*;
    }

    pub mod tasks {
        pub use cu_distributed_resim_demo::tasks::*;
    }

    #[copper_runtime(config = "multi_copper.ron", subsystem = "control", sim_mode = true)]
    struct ControlReplayApp {}

    pub fn register(builder: DistributedReplayBuilder) -> CuResult<DistributedReplayBuilder> {
        builder.register::<ControlReplayApp>("control")
    }

    pub fn compare(
        assignment: &DistributedReplayAssignment,
        replay_log_base: &Path,
    ) -> CuResult<()> {
        let label = format!(
            "instance {} subsystem '{}'",
            assignment.instance_id, assignment.subsystem_id
        );
        let original_copperlists =
            read_copperlist_stream_encoded::<default::CuStampedDataSet>(&assignment.log.base_path)?;
        let replayed_copperlists =
            read_copperlist_stream_encoded::<default::CuStampedDataSet>(replay_log_base)?;
        compare_streams(
            &format!("{label} copperlists"),
            &original_copperlists,
            &replayed_copperlists,
        )?;

        let original_keyframes = read_keyframe_stream_encoded(&assignment.log.base_path)?;
        let replayed_keyframes = read_keyframe_stream_encoded(replay_log_base)?;
        compare_streams(
            &format!("{label} keyframes"),
            &original_keyframes,
            &replayed_keyframes,
        )
    }
}

struct ReplayOptions {
    logs_root: PathBuf,
    replay_root: PathBuf,
}

fn main() {
    if let Err(err) = drive() {
        eprintln!("distributed-resim failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let options = parse_replay_options()?;
    ensure_parent_dir(&options.replay_root.join("placeholder"))?;

    let multi_config_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("multi_copper.ron");
    let builder = DistributedReplayPlan::builder(&multi_config_path)?
        .discover_logs_under(&options.logs_root)?;
    let builder = scout_replay::register(builder)?;
    let builder = plan_replay::register(builder)?;
    let plan = control_replay::register(builder)?.build()?;

    let assignments = plan.assignments.clone();
    let mut engine = plan.start_recording_logs_under(&options.replay_root)?;
    engine.run_all()?;

    if engine.executed_nodes() != engine.total_nodes() {
        return Err(CuError::from(format!(
            "distributed replay stopped early: executed {} of {} nodes",
            engine.executed_nodes(),
            engine.total_nodes()
        )));
    }

    let replay_outputs: BTreeMap<_, _> = assignments
        .iter()
        .map(|assignment| {
            let replay_log_path = engine
                .output_log_path(assignment.instance_id, &assignment.subsystem_id)
                .ok_or_else(|| {
                    CuError::from(format!(
                        "missing replay log path for instance {} subsystem '{}'",
                        assignment.instance_id, assignment.subsystem_id
                    ))
                })?;
            Ok((
                (assignment.instance_id, assignment.subsystem_id.clone()),
                replay_log_path.to_path_buf(),
            ))
        })
        .collect::<CuResult<_>>()?;
    let total_nodes = engine.total_nodes();
    drop(engine);

    for assignment in &assignments {
        let replay_log_path = replay_outputs
            .get(&(assignment.instance_id, assignment.subsystem_id.clone()))
            .ok_or_else(|| {
                CuError::from(format!(
                    "missing replay output after shutdown for instance {} subsystem '{}'",
                    assignment.instance_id, assignment.subsystem_id
                ))
            })?;
        compare_assignment(assignment, replay_log_path)?;
    }

    println!(
        "distributed replay verified {} subsystem logs across {} causal nodes",
        assignments.len(),
        total_nodes
    );
    Ok(())
}

fn parse_replay_options() -> CuResult<ReplayOptions> {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut logs_root = manifest_dir.join("logs").join("record");
    let mut replay_root = manifest_dir.join("logs").join("replay");

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--logs-root" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --logs-root"))?;
                logs_root = PathBuf::from(value);
            }
            "--replay-root" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --replay-root"))?;
                replay_root = PathBuf::from(value);
            }
            other => {
                return Err(CuError::from(format!(
                    "unsupported argument '{other}', expected --logs-root or --replay-root"
                )));
            }
        }
    }

    Ok(ReplayOptions {
        logs_root,
        replay_root,
    })
}

fn compare_assignment(
    assignment: &DistributedReplayAssignment,
    replay_log_base: &Path,
) -> CuResult<()> {
    match assignment.subsystem_id.as_str() {
        "scout" => scout_replay::compare(assignment, replay_log_base),
        "plan" => plan_replay::compare(assignment, replay_log_base),
        "control" => control_replay::compare(assignment, replay_log_base),
        other => Err(CuError::from(format!(
            "unknown subsystem '{other}' in validated distributed replay plan"
        ))),
    }
}
