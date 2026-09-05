use clap::{Parser, Subcommand, ValueEnum};
use cu_logstream_demo::{DataSet, ITERATIONS, SLAB_BYTES, read_lists, tasks::Sample};
use cu29::bincode;
use cu29::continuity::{SourceGapReason, StreamContinuityRecord};
use cu29::prelude::*;
use cu29_logstream::{
    CuStreamRx, FecSymbolKind, FiniteObjectLimits, NativeArchive, RecordKind, SessionEvent,
    SessionRouter, SessionRouterLimits, WirePacket,
};
use cu29_logstream_udp::CuUdpLogStreamConfig;
use std::{
    error::Error,
    fs,
    net::SocketAddr,
    path::{Path, PathBuf},
    thread,
    time::{Duration, Instant},
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
    },
    Receiver {
        #[arg(long, default_value = "127.0.0.1:7447")]
        listen: SocketAddr,
        #[arg(long)]
        log_base: PathBuf,
        /// Write the bound endpoint after socket setup (used by the demo launcher).
        #[arg(long)]
        ready_file: Option<PathBuf>,
        #[arg(long, value_enum, default_value_t = Impairment::Clean)]
        impairment: Impairment,
        /// Stop this receiver after archiving this CopperList, for the restart scenario.
        #[arg(long)]
        stop_at: Option<u64>,
        /// Exit after this much silence; this does not prove the sender's tail is complete.
        #[arg(long, default_value_t = 1000, value_parser = clap::value_parser!(u64).range(1..))]
        idle_ms: u64,
    },
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

fn sender(remote: SocketAddr, path: &Path, iterations: u64) -> Result<()> {
    prepare_log(path)?;
    cu_logstream_demo::run_sender(remote, path, iterations)?;
    println!(
        "Sender finished {iterations} iterations: {}",
        path.display()
    );
    Ok(())
}

#[derive(Default)]
struct Status {
    latest: Option<u64>,
    anchor: Option<u64>,
    gaps: usize,
    dropped: usize,
}

fn receiver(
    listen: SocketAddr,
    path: &Path,
    ready_file: Option<&Path>,
    impairment: Impairment,
    stop_at: Option<u64>,
    idle_ms: u64,
) -> Result<()> {
    prepare_log(path)?;
    let mut socket = CuUdpLogStreamConfig::new(listen);
    socket.recv_buffer_bytes = Some(262144);
    let (_, mut rx) = socket.open()?;
    let address = rx.local_addr()?;
    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_sessions: 1,
        max_startup_packets: 64,
        max_recovery_records: 8,
        max_pending_events: 64,
        max_record_bytes: 4096,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(65536, 1128, 4),
    })?;
    let mut archive: Option<NativeArchive<DataSet>> = None;
    let mut status = Status::default();
    if let Some(ready) = ready_file {
        // Coordination metadata only; the native archive contains all task data.
        use std::io::Write;
        let mut file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(ready)?;
        writeln!(file, "{address}")?;
    }
    println!("Receiver listening at {address}: {}", path.display());
    let started = Instant::now();
    let mut last_packet = started;
    let mut last_status = started;
    let mut seen_packet = false;
    let mut in_outage = false;
    let mut packet = [0; 1200];
    loop {
        if let Some(len) = rx
            .try_recv(&mut packet)
            .map_err(|error| format!("UDP receive: {error:?}"))?
        {
            last_packet = Instant::now();
            seen_packet = true;
            // Demo-only receive-side impairment. Production sender/resource code is unchanged.
            let wire = WirePacket::decode(&packet[..len])?;
            let header = wire.header;
            if header.record_kind == RecordKind::CopperList
                && header.symbol_kind == FecSymbolKind::Source
            {
                in_outage = (32..160).contains(&header.object_id);
            }
            let discard = match impairment {
                Impairment::Clean => false,
                Impairment::Loss => {
                    header.record_kind == RecordKind::CopperList
                        && header.symbol_kind == FecSymbolKind::Source
                        && header.object_id == 20
                }
                Impairment::Outage => in_outage,
            };
            if discard {
                status.dropped += 1;
                continue;
            }
            router
                .receive_datagram(&packet[..len], |event| {
                    if let SessionEvent::Manifest(manifest) = &event {
                        println!(
                            "Session sender={} id={:02x?}",
                            manifest.identity.sender_id, manifest.identity.session_id
                        );
                        archive = Some(NativeArchive::new(
                            path,
                            manifest.clone(),
                            SLAB_BYTES,
                            65536,
                        )?);
                    }
                    if let Some(writer) = archive.as_mut()
                        && let Some(list) = writer.accept(&event)?
                    {
                        status.latest = Some(list.id);
                    }
                    match event {
                        SessionEvent::Gap { gap, .. } => {
                            status.gaps += 1;
                            println!(
                                "Missing CopperLists {}..={}: {:?}",
                                gap.first_id, gap.last_id, gap.reason
                            );
                        }
                        SessionEvent::VerifiedAnchor { anchor, .. } => {
                            status.anchor = Some(anchor.copperlist_id)
                        }
                        _ => {}
                    }
                    Ok::<(), cu29_logstream::Error>(())
                })
                .map_err(|error| format!("Receiver consumer failed: {error:?}"))?;
        } else {
            thread::sleep(Duration::from_millis(1));
        }
        if last_status.elapsed() >= Duration::from_millis(500) {
            print_status(&status, router.stats().datagrams_seen);
            last_status = Instant::now();
        }
        if stop_at.is_some_and(|id| status.latest.is_some_and(|latest| latest >= id))
            || (seen_packet && last_packet.elapsed() >= Duration::from_millis(idle_ms))
        {
            break;
        }
        if !seen_packet && started.elapsed() >= Duration::from_secs(15) {
            return Err("No UDP traffic arrived within 15 seconds".into());
        }
    }
    archive.ok_or("No session manifest received")?.finish()?;
    print_status(&status, router.stats().datagrams_seen);
    if !matches!(impairment, Impairment::Clean) && status.dropped == 0 {
        return Err("Impairment scenario did not discard any packets".into());
    }
    println!("Archive closed at receiver's known boundary; unobserved sender tail is unknown.");
    Ok(())
}

fn print_status(status: &Status, packets: usize) {
    println!(
        "packets={packets} archived={:?} verified_anchor={:?} gaps={} demo_drops={}",
        status.latest, status.anchor, status.gaps, status.dropped
    );
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
        } => sender(remote, &log_base, iterations),
        Command::Receiver {
            listen,
            log_base,
            ready_file,
            impairment,
            stop_at,
            idle_ms,
        } => receiver(
            listen,
            &log_base,
            ready_file.as_deref(),
            impairment,
            stop_at,
            idle_ms,
        ),
        Command::Verify {
            sender,
            received,
            expect,
            iterations,
        } => verify(&sender, &received, expect, iterations),
    }
}
