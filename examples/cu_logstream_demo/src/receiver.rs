use crate::{Impairment, Result, prepare_log};
use cu_logstream_demo::{
    DataSet, SLAB_BYTES,
    telemetry::{Frame, Publisher, RecordingState, Status},
};
use cu29_logstream::{
    CuStreamRx, FecSymbolKind, FiniteObjectLimits, NativeArchive, RecordKind, SessionEvent,
    SessionRouter, SessionRouterLimits, WirePacket,
};
use cu29_logstream_udp::CuUdpLogStreamConfig;
use std::{
    fs,
    net::SocketAddr,
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant},
};

#[derive(clap::Args)]
pub struct ReceiverOptions {
    #[arg(long, default_value = "127.0.0.1:7447")]
    pub listen: SocketAddr,
    #[arg(long)]
    pub log_base: PathBuf,
    /// Write the bound endpoint after socket setup (used by the demo launcher).
    #[arg(long)]
    pub ready_file: Option<PathBuf>,
    #[arg(long, value_enum, default_value_t = Impairment::Clean)]
    pub impairment: Impairment,
    /// Stop this receiver after archiving this CopperList, for the restart scenario.
    #[arg(long)]
    pub stop_at: Option<u64>,
    /// Exit after this much silence; this does not prove the sender's tail is complete.
    #[arg(long, default_value_t = 1000, value_parser = clap::value_parser!(u64).range(1..))]
    pub idle_ms: u64,
}

pub fn run(
    options: &ReceiverOptions,
    mut telemetry: Option<&mut Publisher>,
    stop: &AtomicBool,
) -> Result<()> {
    let ReceiverOptions {
        listen,
        log_base: path,
        ready_file,
        impairment,
        stop_at,
        idle_ms,
    } = options;
    let quiet = telemetry.is_some();
    prepare_log(path)?;
    let mut socket = CuUdpLogStreamConfig::new(*listen);
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
    if !quiet {
        println!("Receiver listening at {address}: {}", path.display());
    }
    let started = Instant::now();
    let mut last_packet = started;
    let mut last_status = started;
    let mut seen_packet = false;
    let mut first_packet = None;
    let mut in_outage = false;
    let mut packet = [0; 1200];
    while !stop.load(Ordering::Acquire) {
        if let Some(len) = rx
            .try_recv(&mut packet)
            .map_err(|error| format!("UDP receive: {error:?}"))?
        {
            last_packet = Instant::now();
            status.last_packet = Some(last_packet);
            seen_packet = true;
            let first = *first_packet.get_or_insert(last_packet);
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
                Impairment::Bootstrap => {
                    last_packet.duration_since(first) < Duration::from_millis(300)
                }
            };
            if discard {
                status.dropped += 1;
                continue;
            }
            router
                .receive_datagram(&packet[..len], |event| {
                    if let SessionEvent::Manifest(manifest) = &event {
                        if !quiet {
                            println!(
                                "Session sender={} id={:02x?}",
                                manifest.identity.sender_id, manifest.identity.session_id
                            );
                        }
                        status.identity = Some(manifest.identity);
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
                        status.archived += 1;
                        status.state = RecordingState::Recording;
                        if let Some(publisher) = telemetry.as_deref_mut() {
                            publisher.publish(Frame {
                                identity: status.identity.expect("archived frame has a manifest"),
                                received_at: Instant::now(),
                                copperlist: list,
                            });
                        }
                    }
                    match event {
                        SessionEvent::Gap { gap, .. } => {
                            status.gaps += 1;
                            if !quiet {
                                println!(
                                    "Missing CopperLists {}..={}: {:?}",
                                    gap.first_id, gap.last_id, gap.reason
                                );
                            }
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
        status.packets = router.stats().datagrams_seen;
        if let Some(publisher) = telemetry.as_deref_mut() {
            publisher.set_status(status);
        }
        if !quiet && last_status.elapsed() >= Duration::from_millis(500) {
            print_status(&status, router.stats().datagrams_seen);
            last_status = Instant::now();
        }
        if stop_at.is_some_and(|id| status.latest.is_some_and(|latest| latest >= id))
            || (seen_packet && last_packet.elapsed() >= Duration::from_millis(*idle_ms))
        {
            break;
        }
        if !seen_packet && started.elapsed() >= Duration::from_secs(15) {
            return Err("No UDP traffic arrived within 15 seconds".into());
        }
    }
    if let Some(archive) = archive {
        archive.finish()?;
    } else if !stop.load(Ordering::Acquire) {
        return Err("No session manifest received".into());
    }
    status.state = RecordingState::Closed;
    if let Some(publisher) = telemetry {
        publisher.set_status(status);
    }
    if !quiet {
        print_status(&status, router.stats().datagrams_seen);
    }
    if !stop.load(Ordering::Acquire)
        && !matches!(impairment, Impairment::Clean)
        && status.dropped == 0
    {
        return Err("Impairment scenario did not discard any packets".into());
    }
    if !quiet {
        println!("Archive closed at receiver's known boundary; unobserved sender tail is unknown.");
    }
    Ok(())
}

fn print_status(status: &Status, packets: usize) {
    println!(
        "packets={packets} archived={:?} verified_anchor={:?} gaps={} demo_drops={}",
        status.latest, status.anchor, status.gaps, status.dropped
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_logstream::telemetry::telemetry_channel;

    #[test]
    fn native_archive_is_identical_with_fast_stalled_or_disconnected_reader() {
        const COUNT: u64 = 96;
        let logs = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs");
        fs::create_dir_all(&logs).unwrap();
        let directory = tempfile::Builder::new()
            .prefix("telemetry-test-")
            .tempdir_in(logs)
            .unwrap();
        for mode in ["disabled", "fast", "stalled", "disconnected"] {
            let ready = directory.path().join(format!("{mode}.endpoint"));
            let received = directory.path().join(format!("{mode}.copper"));
            let sender = directory.path().join(format!("{mode}-sender.copper"));
            let options = ReceiverOptions {
                listen: "127.0.0.1:0".parse().unwrap(),
                log_base: received.clone(),
                ready_file: Some(ready.clone()),
                impairment: Impairment::Clean,
                stop_at: Some(COUNT - 1),
                idle_ms: 2000,
            };
            let (publisher, reader) =
                telemetry_channel::<Frame, Status>(4.try_into().unwrap(), Status::default());
            let mut publisher = (mode != "disabled").then_some(publisher);
            let mut reader = Some(reader);
            if mode == "disconnected" {
                drop(reader.take());
            }
            thread::scope(|scope| {
                let worker = scope.spawn(move || {
                    let result = run(&options, publisher.as_mut(), &AtomicBool::new(false))
                        .map_err(|e| e.to_string());
                    drop(publisher);
                    result
                });
                let fast_reader = if mode == "fast" {
                    let mut reader = reader.take().unwrap();
                    Some(scope.spawn(move || {
                        let mut accounted = 0;
                        loop {
                            assert!(reader.wait_timeout(Duration::from_secs(5)));
                            reader.status();
                            while let Some(update) = reader.try_read() {
                                accounted += update.missed + 1;
                            }
                            if reader.is_closed() {
                                while let Some(update) = reader.try_read() {
                                    accounted += update.missed + 1;
                                }
                                return accounted;
                            }
                        }
                    }))
                } else {
                    None
                };
                let started = Instant::now();
                let address = loop {
                    if let Ok(text) = fs::read_to_string(&ready)
                        && let Ok(address) = text.trim().parse()
                    {
                        break address;
                    }
                    if worker.is_finished() {
                        panic!(
                            "Receiver exited before binding: {:?}",
                            worker.join().unwrap()
                        );
                    }
                    assert!(
                        started.elapsed() < Duration::from_secs(5),
                        "receiver did not bind"
                    );
                    thread::sleep(Duration::from_millis(5));
                };
                cu_logstream_demo::run_sender(address, &sender, COUNT, 0).unwrap();
                worker.join().unwrap().unwrap();
                if let Some(worker) = fast_reader {
                    assert_eq!(worker.join().unwrap(), COUNT);
                }
            });
            if mode == "stalled" {
                let reader = reader.as_mut().unwrap();
                let status = reader.status();
                assert_eq!(status.archived, COUNT);
                assert_eq!(status.state, RecordingState::Closed);
                let update = reader.try_read().unwrap();
                assert_eq!(update.missed, COUNT - 4);
                assert_eq!(update.frame.copperlist.id, COUNT - 4);
            }
            crate::verify(&sender, &received, crate::Expectation::Complete, COUNT).unwrap();
        }
    }
}
