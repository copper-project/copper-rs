use bincode::Encode;
use cu29::prelude::*;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

pub const LOG_SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);
pub const DEFAULT_SCOUT_PERIOD_MS: u64 = 30;
pub const DEFAULT_PLAN_PERIOD_MS: u64 = 10;
pub const DEFAULT_CONTROL_PERIOD_MS: u64 = 10;

pub mod messages {
    use super::*;

    #[derive(Debug, Default, Clone, Encode, bincode::Decode, Serialize, Deserialize, Reflect)]
    pub struct Pulse {
        pub robot_id: u32,
        pub cycle: u64,
        pub energy: i64,
        pub feedback_signature: i64,
    }

    #[derive(Debug, Default, Clone, Encode, bincode::Decode, Serialize, Deserialize, Reflect)]
    pub struct Plan {
        pub robot_id: u32,
        pub pulse_cycle: u64,
        pub score: i64,
        pub planner_state: i64,
    }

    #[derive(Debug, Default, Clone, Encode, bincode::Decode, Serialize, Deserialize, Reflect)]
    pub struct Feedback {
        pub robot_id: u32,
        pub pulse_cycle: u64,
        pub command: i64,
        pub signature: i64,
    }
}

pub mod bridges {
    use super::messages;
    use cu_zenoh_bridge::ZenohBridge;
    use cu29::prelude::*;

    tx_channels! {
        pub struct DemoTxChannels : DemoTxId {
            pulse => messages::Pulse = "distributed_resim/pulse",
            plan => messages::Plan = "distributed_resim/plan",
            feedback => messages::Feedback = "distributed_resim/feedback",
        }
    }

    rx_channels! {
        pub struct DemoRxChannels : DemoRxId {
            pulse => messages::Pulse = "distributed_resim/pulse",
            plan => messages::Plan = "distributed_resim/plan",
            feedback => messages::Feedback = "distributed_resim/feedback",
        }
    }

    pub type DemoZenohBridge = ZenohBridge<DemoTxChannels, DemoRxChannels>;
}

pub mod tasks {
    use super::messages::{Feedback, Plan, Pulse};
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct ScoutTask {
        phase: i64,
        last_feedback_signature: i64,
        emitted: u64,
        accepted_feedback: u64,
    }

    impl Freezable for ScoutTask {}

    impl CuTask for ScoutTask {
        type Input<'m> = input_msg!(Feedback);
        type Output<'m> = output_msg!(Pulse);
        type Resources<'r> = ();

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let phase = config
                .and_then(|cfg| cfg.get::<i64>("seed").ok().flatten())
                .unwrap_or(17);
            Ok(Self {
                phase,
                last_feedback_signature: 0,
                emitted: 0,
                accepted_feedback: 0,
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if let Some(feedback) = input.payload()
                && feedback.robot_id == ctx.instance_id()
            {
                self.last_feedback_signature = feedback.signature;
                self.phase = self
                    .phase
                    .wrapping_mul(5)
                    .wrapping_add(feedback.command)
                    .wrapping_add(11);
                self.accepted_feedback += 1;
            }

            // Keep the producer slower than the consumers so both robot instances
            // can share the same transport route without building queue backlogs.
            if !ctx.cl_id().is_multiple_of(3) {
                output.clear_payload();
                return Ok(());
            }

            let energy = self
                .phase
                .wrapping_add(ctx.instance_id() as i64 * 101)
                .wrapping_add(self.emitted as i64 * 17)
                .wrapping_add(self.last_feedback_signature.rem_euclid(31));
            output.set_payload(Pulse {
                robot_id: ctx.instance_id(),
                cycle: self.emitted,
                energy,
                feedback_signature: self.last_feedback_signature,
            });
            output.tov = Tov::Time(ctx.now());

            self.phase = self
                .phase
                .wrapping_add(energy.rem_euclid(13))
                .wrapping_add(self.accepted_feedback as i64);
            self.emitted += 1;
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct PlannerTask {
        integrator: i64,
        accepted: u64,
        ignored: u64,
    }

    impl Freezable for PlannerTask {}

    impl CuTask for PlannerTask {
        type Input<'m> = input_msg!(Pulse);
        type Output<'m> = output_msg!(Plan);
        type Resources<'r> = ();

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                integrator: 0,
                accepted: 0,
                ignored: 0,
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let Some(pulse) = input.payload() else {
                output.clear_payload();
                return Ok(());
            };

            if pulse.robot_id != ctx.instance_id() {
                self.ignored += 1;
                output.clear_payload();
                return Ok(());
            }

            self.integrator = self
                .integrator
                .wrapping_add(pulse.energy.wrapping_mul(2))
                .wrapping_add(pulse.feedback_signature)
                .wrapping_add(pulse.cycle as i64);
            self.accepted += 1;

            output.set_payload(Plan {
                robot_id: pulse.robot_id,
                pulse_cycle: pulse.cycle,
                score: self
                    .integrator
                    .wrapping_add(self.accepted as i64 * 19)
                    .wrapping_sub(self.ignored as i64 * 7),
                planner_state: self.integrator,
            });
            output.tov = Tov::Time(ctx.now());
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct ControllerTask {
        checksum: i64,
        applied: u64,
    }

    impl Freezable for ControllerTask {}

    impl CuTask for ControllerTask {
        type Input<'m> = input_msg!(Plan);
        type Output<'m> = output_msg!(Feedback);
        type Resources<'r> = ();

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                checksum: 0,
                applied: 0,
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let Some(plan) = input.payload() else {
                output.clear_payload();
                return Ok(());
            };

            if plan.robot_id != ctx.instance_id() {
                output.clear_payload();
                return Ok(());
            }

            self.checksum = self
                .checksum
                .wrapping_mul(31)
                .wrapping_add(plan.score)
                .wrapping_add(plan.planner_state);
            self.applied += 1;

            output.set_payload(Feedback {
                robot_id: plan.robot_id,
                pulse_cycle: plan.pulse_cycle,
                command: self.checksum.rem_euclid(1024) + plan.pulse_cycle as i64,
                signature: self
                    .checksum
                    .wrapping_add(self.applied as i64 * 23)
                    .rem_euclid(4096),
            });
            output.tov = Tov::Time(ctx.now());
            Ok(())
        }
    }
}

pub struct DemoRunOptions {
    pub log_path: PathBuf,
    pub instance_id: u32,
    pub iterations: Option<usize>,
    pub period: Duration,
}

pub fn default_record_log_path(file_name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join("record")
        .join(file_name)
}

pub fn parse_run_options(
    default_log_file: &str,
    default_period_ms: u64,
) -> CuResult<DemoRunOptions> {
    let mut log_path = default_record_log_path(default_log_file);
    let mut instance_id = 0u32;
    let mut iterations = None;
    let mut period_ms = default_period_ms;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--log" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --log"))?;
                log_path = PathBuf::from(value);
            }
            "--instance-id" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --instance-id"))?;
                instance_id = value.parse::<u32>().map_err(|err| {
                    CuError::new_with_cause(&format!("invalid --instance-id '{value}'"), err)
                })?;
            }
            "--iterations" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --iterations"))?;
                iterations = Some(value.parse::<usize>().map_err(|err| {
                    CuError::new_with_cause(&format!("invalid --iterations '{value}'"), err)
                })?);
            }
            "--period-ms" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --period-ms"))?;
                period_ms = value.parse::<u64>().map_err(|err| {
                    CuError::new_with_cause(&format!("invalid --period-ms '{value}'"), err)
                })?;
            }
            other => {
                return Err(CuError::from(format!(
                    "unsupported argument '{other}', expected --log, --instance-id, --iterations or --period-ms"
                )));
            }
        }
    }

    ensure_parent_dir(&log_path)?;

    Ok(DemoRunOptions {
        log_path,
        instance_id,
        iterations,
        period: Duration::from_millis(period_ms),
    })
}

pub fn ensure_parent_dir(path: &Path) -> CuResult<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|err| {
            CuError::new_with_cause(
                &format!("failed to create directory '{}'", parent.display()),
                err,
            )
        })?;
    }
    Ok(())
}

pub fn sleep_period(period: Duration) {
    std::thread::sleep(period);
}

pub fn read_copperlist_stream_encoded<P>(log_base: &Path) -> CuResult<Vec<Vec<u8>>>
where
    P: CopperListTuple + Encode,
{
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| {
            CuError::new_with_cause(&format!("failed to open log '{}'", log_base.display()), err)
        })?
    else {
        return Err(CuError::from(format!(
            "log '{}' is not readable",
            log_base.display()
        )));
    };

    let mut io_reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::CopperList);
    let mut out = Vec::new();
    for entry in copperlists_reader::<P>(&mut io_reader) {
        let bytes = bincode::encode_to_vec(entry, bincode::config::standard()).map_err(|err| {
            CuError::new_with_cause("failed to encode copperlist for comparison", err)
        })?;
        out.push(bytes);
    }
    Ok(out)
}

pub fn read_keyframe_stream_encoded(log_base: &Path) -> CuResult<Vec<Vec<u8>>> {
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| {
            CuError::new_with_cause(&format!("failed to open log '{}'", log_base.display()), err)
        })?
    else {
        return Err(CuError::from(format!(
            "log '{}' is not readable",
            log_base.display()
        )));
    };

    let mut io_reader = UnifiedLoggerIOReader::new(reader, UnifiedLogType::FrozenTasks);
    let mut out = Vec::new();
    for keyframe in keyframes_reader(&mut io_reader) {
        let bytes =
            bincode::encode_to_vec(keyframe, bincode::config::standard()).map_err(|err| {
                CuError::new_with_cause("failed to encode keyframe for comparison", err)
            })?;
        out.push(bytes);
    }
    Ok(out)
}

pub fn compare_streams(label: &str, original: &[Vec<u8>], replayed: &[Vec<u8>]) -> CuResult<()> {
    if original.len() != replayed.len() {
        return Err(CuError::from(format!(
            "{label} length mismatch: original={} replayed={}",
            original.len(),
            replayed.len()
        )));
    }

    for (idx, (left, right)) in original.iter().zip(replayed.iter()).enumerate() {
        if left != right {
            return Err(CuError::from(format!("{label} diverged at entry {idx}")));
        }
    }

    Ok(())
}
