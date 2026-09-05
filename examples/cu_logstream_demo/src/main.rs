#[cfg(feature = "tui")]
mod dashboard;
mod receiver;

use clap::{Parser, Subcommand, ValueEnum};
use cu_logstream_demo::{ITERATIONS, read_lists, tasks::Sample};
use cu29::bincode;
use cu29::continuity::{SourceGapReason, StreamContinuityRecord};
use cu29::prelude::*;
use std::{
    error::Error,
    fs,
    net::SocketAddr,
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
#[command(about = "Stream a deterministic counter → accumulator graph over UDP")]
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
    Dashboard(receiver::ReceiverOptions),
    Verify {
        #[arg(long)]
        sender: PathBuf,
        #[arg(long)]
        received: PathBuf,
        #[arg(long, value_enum)]
        expect: Expectation,
        #[arg(long, default_value_t = ITERATIONS)]
        iterations: u64,
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
    let first = cu29::replay::first_slab_path(path)?;
    if first.exists() || path.exists() {
        return Err(format!(
            "Log already exists: {}. Choose a new log base.",
            path.display()
        )
        .into());
    }
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
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

fn verify(sender: &Path, received: &Path, expect: Expectation, iterations: u64) -> Result<()> {
    let onboard = read_lists(sender)?;
    let ground = read_lists(received)?;
    if onboard.len() as u64 != iterations || ground.is_empty() {
        return Err(
            "Sender archive count differs from requested run, or receiver archive is empty".into(),
        );
    }
    for (id, list) in onboard.iter().enumerate() {
        if list.id != id as u64
            || list.msgs.get_counter_output().payload() != Some(&Sample(list.id))
            || list.msgs.get_sum_output().payload() != Some(&Sample(list.id * (list.id + 1) / 2))
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
    let anchors: Vec<_> = continuity
        .iter()
        .filter_map(|entry| match entry {
            StreamContinuityRecord::Anchor { copperlist_id, .. } => Some(*copperlist_id),
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
        let encoding = bincode::config::standard();
        if bincode::encode_to_vec(list, encoding)?
            != bincode::encode_to_vec(&onboard[list.id as usize], encoding)?
        {
            return Err(format!(
                "Payload or sender metadata mismatch at CopperList {}",
                list.id
            )
            .into());
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
    let valid = match expect {
        Expectation::Complete => ground.len() == onboard.len() && gaps.is_empty(),
        Expectation::Outage => {
            !gaps.is_empty()
                && gaps.iter().any(|&(first, _, _)| first > 0)
                && anchors.iter().any(|&id| id >= 160)
                && next == iterations
        }
        Expectation::Late => {
            gaps.iter()
                .any(|&(first, _, reason)| first == 0 && reason == SourceGapReason::LateJoin)
                && anchors.iter().any(|&id| id > 0)
                && next == iterations
        }
        Expectation::Prefix => gaps.is_empty() && next >= 65 && next < iterations,
    };
    if !valid {
        return Err(format!("Archive does not satisfy {expect:?}").into());
    }
    println!(
        "Verified {} received CopperLists against onboard payloads and timestamps; {} explicit gaps, {} verified anchors ({expect:?}).",
        ground.len(),
        gaps.len(),
        anchors.len()
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
        Command::Receiver(options) => receiver::run(&options, None, &Default::default()),
        #[cfg(feature = "tui")]
        Command::Dashboard(options) => dashboard::run(options),
        Command::Verify {
            sender,
            received,
            expect,
            iterations,
        } => verify(&sender, &received, expect, iterations),
    }
}
