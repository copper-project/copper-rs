mod receiver;
#[cfg(feature = "tui")]
mod telemetry_screen;

use clap::{Parser, Subcommand, ValueEnum};
use cu_logstream_demo::{ITERATIONS, read_lists, tasks::JointAngles};
use cu29::bincode;
use cu29::continuity::{SourceGapReason, StreamContinuityRecord};
use cu29::prelude::*;
use std::{
    error::Error,
    net::SocketAddr,
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
#[command(about = "Stream joint encoders and reconstruct a robot arm over UDP")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Sender {
        #[arg(long, default_value = "127.0.0.1:7447")]
        remote: SocketAddr,
        #[arg(long)]
        log_base: PathBuf,
        #[arg(long, default_value_t = ITERATIONS, value_parser = clap::value_parser!(u64).range(1..))]
        iterations: u64,
        /// Keep the sender alive without new captures to exercise autonomous repetition.
        #[arg(long, default_value_t = 0)]
        idle_ms: u64,
    },
    Receiver(receiver::ReceiverOptions),
    /// Native telemetry screen; Space pauses only the reader, q closes recording.
    #[cfg(feature = "tui")]
    Telemetry(receiver::ReceiverOptions),
    Verify {
        #[arg(long)]
        sender: PathBuf,
        #[arg(long)]
        received: PathBuf,
        #[arg(long, value_enum)]
        expect: Expectation,
        #[arg(long, default_value_t = ITERATIONS)]
        iterations: u64,
        /// Require every demonstrated robot log when the scenario preserves log packets.
        #[arg(long)]
        require_robot_logs: bool,
    },
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Impairment {
    Clean,
    Loss,
    Outage,
    Bootstrap,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Expectation {
    Complete,
    Outage,
    Late,
    Prefix,
}

fn prepare_log(path: &Path) -> Result<()> {
    // Demo reruns replace all old slabs, including the tail of longer runs.
    cu29::replay::remove_log_family(path)?;
    Ok(())
}

fn sender(remote: SocketAddr, path: &Path, iterations: u64, idle_ms: u64) -> Result<()> {
    prepare_log(path)?;
    cu_logstream_demo::run_sender(remote, path, iterations, idle_ms)?;
    println!(
        "Sender finished {iterations} iterations: {}",
        path.display()
    );
    Ok(())
}

fn verify(
    sender: &Path,
    received: &Path,
    expect: Expectation,
    iterations: u64,
    require_robot_logs: bool,
) -> Result<()> {
    let onboard = read_lists(sender)?;
    let ground = read_lists(received)?;
    if onboard.len() as u64 != iterations || ground.is_empty() {
        return Err(
            "Sender archive count differs from requested run, or receiver archive is empty".into(),
        );
    }
    for (id, list) in onboard.iter().enumerate() {
        if list.id != id as u64
            || list.msgs.get_encoders_output().payload() != Some(&JointAngles::at_tick(list.id))
        {
            return Err(format!("Unexpected deterministic graph output at {id}").into());
        }
    }
    let continuity: Vec<_> = cu29_export::stream_continuity_reader(UnifiedLoggerIOReader::new(
        UnifiedLoggerRead::new(received)?,
        UnifiedLogType::StreamContinuity,
    ))
    .collect();
    let gaps: Vec<_> = continuity
        .iter()
        .filter_map(|entry| match entry {
            StreamContinuityRecord::Gap {
                first_id,
                last_id,
                reason,
            } => Some((*first_id, *last_id, *reason)),
            _ => None,
        })
        .collect();
    let recovery_points: Vec<_> = continuity
        .iter()
        .filter_map(|entry| match entry {
            StreamContinuityRecord::RecoveryPoint { copperlist_id, .. } => Some(*copperlist_id),
            _ => None,
        })
        .collect();
    let mut next = 0;
    for list in &ground {
        if list.id < next || list.id >= iterations {
            return Err("Receiver CopperList IDs are not strictly ordered within the run".into());
        }
        for absent in next..list.id {
            if !gaps
                .iter()
                .any(|&(first, last, _)| (first..=last).contains(&absent))
            {
                return Err(format!("Unreported missing CopperList {absent}").into());
            }
        }
        if gaps
            .iter()
            .any(|&(first, last, _)| (first..=last).contains(&list.id))
        {
            return Err("Archive marks a received record as missing".into());
        }
        let expected = &onboard[list.id as usize];
        if list.msgs.get_kinematics_output().payload().is_some() {
            return Err("Arm pose payload was transmitted".into());
        }
        if bincode::encode_to_vec(list.msgs.get_encoders_output(), bincode::config::standard())?
            != bincode::encode_to_vec(
                expected.msgs.get_encoders_output(),
                bincode::config::standard(),
            )?
        {
            return Err(
                format!("Captured encoder input or metadata mismatch at {}", list.id).into(),
            );
        }
        next = list.id + 1;
    }
    if !matches!(
        continuity.first(),
        Some(StreamContinuityRecord::Manifest { .. })
    ) || !matches!(continuity.last(), Some(StreamContinuityRecord::Finished { next_copperlist_id }) if *next_copperlist_id == next)
    {
        return Err("Missing archive provenance/finalization".into());
    }
    let mut twin = cu29_logstream::twin::LiveTwin::<cu_logstream_demo::twin::Twin>::new()?;
    let keyframes = cu_logstream_demo::read_keyframes(received)?;
    for capture in cu_logstream_demo::read_captures(received)? {
        let id = capture.copperlist.id;
        let keyframe = keyframes.iter().find(|k| k.culistid == id);
        let reconstructed = twin
            .reconstruct(capture, keyframe)?
            .ok_or("Capture has no verified replay boundary")?;
        if bincode::encode_to_vec(&reconstructed, bincode::config::standard())?
            != bincode::encode_to_vec(&onboard[id as usize], bincode::config::standard())?
        {
            return Err(format!("Reconstructed output or sender metadata mismatch at {id}").into());
        }
    }
    let onboard_logs = cu_logstream_demo::read_logs(sender)?;
    let received_logs = cu_logstream_demo::read_logs(received)?;
    for entry in &received_logs {
        if !onboard_logs.contains(entry) {
            return Err("Received structured log differs from the robot's original entry".into());
        }
    }
    if require_robot_logs {
        let robot_entries: Vec<_> = onboard_logs
            .iter()
            .filter(|entry| entry.origin.task_index == Some(0))
            .collect();
        if robot_entries.is_empty()
            || robot_entries
                .iter()
                .any(|entry| !received_logs.contains(entry))
        {
            return Err("Missing streamed robot info! entries".into());
        }
    }
    let valid = match expect {
        Expectation::Complete => ground.len() == onboard.len() && gaps.is_empty(),
        Expectation::Outage => {
            !gaps.is_empty()
                && gaps.iter().any(|&(first, _, _)| first > 0)
                && recovery_points.iter().any(|&id| id >= 160)
                && next == iterations
        }
        Expectation::Late => {
            gaps.iter()
                .any(|&(first, _, reason)| first == 0 && reason == SourceGapReason::LateJoin)
                && recovery_points.iter().any(|&id| id > 0)
                && next == iterations
        }
        Expectation::Prefix => gaps.is_empty() && next >= 65 && next < iterations,
    };
    if !valid {
        return Err(format!("Archive does not satisfy {expect:?}").into());
    }
    println!(
        "Verified {} original robot structured logs.",
        received_logs.len()
    );
    println!(
        "Verified {} received CopperLists against onboard payloads and timestamps; {} explicit gaps, {} verified recovery points ({expect:?}).",
        ground.len(),
        gaps.len(),
        recovery_points.len()
    );
    Ok(())
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Sender {
            remote,
            log_base,
            iterations,
            idle_ms,
        } => sender(remote, &log_base, iterations, idle_ms),
        Command::Receiver(options) => receiver::run(&options),
        #[cfg(feature = "tui")]
        Command::Telemetry(options) => telemetry_screen::run(options),
        Command::Verify {
            sender,
            received,
            expect,
            iterations,
            require_robot_logs,
        } => verify(&sender, &received, expect, iterations, require_robot_logs),
    }
}
