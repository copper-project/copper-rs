//! Demo-only CLI and packet impairment. Copper owns the receiver and twin lifecycle.
use crate::{Impairment, Result, prepare_log};
use cu_logstream_demo::telemetry::Status;
use cu_logstream_demo::twin::Twin;
use cu29_logstream::{
    CuStreamRx, CuStreamRxError, CuTwin, CuTwinReader, FecSymbolKind, RecordKind, WirePacket,
};
use cu29_logstream_udp::CuUdpLogStreamConfig;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

#[derive(clap::Args)]
pub struct ReceiverOptions {
    #[arg(long, default_value = "127.0.0.1:7447")]
    pub listen: SocketAddr,
    #[arg(long)]
    pub log_base: PathBuf,
    /// Write the bound endpoint for the demo launcher.
    #[arg(long)]
    pub ready_file: Option<PathBuf>,
    #[arg(long, value_enum, default_value_t = Impairment::Clean)]
    pub impairment: Impairment,
    #[arg(long)]
    pub stop_at: Option<u64>,
    #[arg(long, default_value_t = 1000, value_parser = clap::value_parser!(u64).range(1..))]
    pub idle_ms: u64,
    #[arg(long)]
    pub archive_only: bool,
}

#[derive(Debug)]
pub struct ImpairmentStats {
    started: Instant,
    last_packet_ms: AtomicU64,
    seen_packet: AtomicBool,
    dropped: AtomicUsize,
}

impl ImpairmentStats {
    pub fn should_stop(&self, options: &ReceiverOptions, status: Status) -> Result<bool> {
        if !self.seen_packet.load(Ordering::Acquire) {
            if self.started.elapsed() >= Duration::from_secs(15) {
                return Err("No UDP traffic arrived within 15 seconds".into());
            }
            return Ok(false);
        }
        let elapsed = self.started.elapsed().as_millis() as u64;
        Ok(options
            .stop_at
            .is_some_and(|id| status.latest.is_some_and(|latest| latest >= id))
            || elapsed.saturating_sub(self.last_packet_ms.load(Ordering::Relaxed))
                >= options.idle_ms)
    }
}

#[derive(Debug)]
struct ImpairedRx<R> {
    rx: R,
    impairment: Impairment,
    stats: Arc<ImpairmentStats>,
    first_packet: Option<Instant>,
    in_outage: bool,
}

impl<R: CuStreamRx> CuStreamRx for ImpairedRx<R> {
    fn try_recv(
        &mut self,
        packet: &mut [u8],
    ) -> std::result::Result<Option<usize>, CuStreamRxError> {
        let Some(len) = self.rx.try_recv(packet)? else {
            return Ok(None);
        };
        let now = Instant::now();
        self.stats.last_packet_ms.store(
            self.stats.started.elapsed().as_millis() as u64,
            Ordering::Relaxed,
        );
        self.stats.seen_packet.store(true, Ordering::Release);
        let first = *self.first_packet.get_or_insert(now);
        let wire = WirePacket::decode(&packet[..len])
            .map_err(|_| CuStreamRxError::Failed("Invalid demo packet"))?;
        let header = wire.header;
        if header.record_kind == RecordKind::CopperList
            && header.symbol_kind == FecSymbolKind::Source
        {
            self.in_outage = (32..160).contains(&header.object_id);
        }
        let discard = match self.impairment {
            Impairment::Clean => false,
            Impairment::Loss => {
                header.record_kind == RecordKind::CopperList
                    && header.symbol_kind == FecSymbolKind::Source
                    && header.object_id == 20
            }
            Impairment::Outage => self.in_outage,
            Impairment::Bootstrap => now.duration_since(first) < Duration::from_millis(300),
        };
        if discard {
            self.stats.dropped.fetch_add(1, Ordering::Relaxed);
            return Ok(None);
        }
        Ok(Some(len))
    }
}

pub fn start(
    options: &ReceiverOptions,
) -> Result<(CuTwin<Twin>, CuTwinReader<Twin>, Arc<ImpairmentStats>)> {
    prepare_log(&options.log_base)?;
    let mut socket = CuUdpLogStreamConfig::new(options.listen);
    socket.recv_buffer_bytes = Some(262144);
    let (_, rx) = socket.open()?;
    let address = rx.local_addr()?;
    let stats = Arc::new(ImpairmentStats {
        started: Instant::now(),
        last_packet_ms: AtomicU64::new(0),
        seen_packet: AtomicBool::new(false),
        dropped: AtomicUsize::new(0),
    });
    let rx = ImpairedRx {
        rx,
        impairment: options.impairment,
        stats: stats.clone(),
        first_packet: None,
        in_outage: false,
    };
    let builder = Twin::twin(rx).with_log_path(&options.log_base);
    let builder = if options.archive_only {
        builder.archive_only()
    } else {
        builder
    };
    let (twin, reader) = builder.spawn()?;
    if let Some(ready) = &options.ready_file {
        use std::io::Write;
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(ready)?;
        writeln!(file, "{address}")?;
    }
    Ok((twin, reader, stats))
}

pub fn run(options: &ReceiverOptions) -> Result<()> {
    let (mut twin, mut reader, impairment) = start(options)?;
    println!(
        "Receiver listening at {}: {}",
        options.listen,
        options.log_base.display()
    );
    loop {
        let status = reader.status();
        while reader.try_read().is_some() {}
        if reader.is_closed() || impairment.should_stop(options, status)? {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let status = twin.stop()?;
    if status.identity.is_none() {
        return Err("No session manifest received".into());
    }
    if !options.archive_only && status.archived > 0 && status.twin.reconstructed == 0 {
        return Err("Live Copper twin produced no reconstructed frames".into());
    }
    if status.twin.divergences != 0 {
        return Err("Live Copper twin diverged".into());
    }
    if !matches!(options.impairment, Impairment::Clean)
        && impairment.dropped.load(Ordering::Relaxed) == 0
    {
        return Err("Impairment scenario did not discard any packets".into());
    }
    println!(
        "packets={} archived={:?} verified_anchor={:?} gaps={} demo_drops={}",
        status.packets,
        status.latest,
        status.anchor,
        status.gaps,
        impairment.dropped.load(Ordering::Relaxed)
    );
    println!(
        "Copper twin: {:?}, reconstructed={}, verified={}, replay_queue_drops={}",
        status.twin.state,
        status.twin.reconstructed,
        status.twin.verified,
        status.twin.queue_overflows
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_logstream_demo::telemetry::RecordingState;
    use cu_logstream_demo::{ITERATIONS, run_sender};

    #[test]
    fn native_archive_is_identical_with_fast_stalled_or_disconnected_reader() {
        const COUNT: u64 = 96;
        let logs = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs");
        std::fs::create_dir_all(&logs).unwrap();
        let directory = tempfile::tempdir_in(logs).unwrap();
        for mode in ["disabled", "fast", "stalled", "disconnected"] {
            let received = directory.path().join(format!("{mode}.copper"));
            let sender = directory.path().join(format!("{mode}-sender.copper"));
            let mut socket = CuUdpLogStreamConfig::new("127.0.0.1:0".parse().unwrap());
            socket.recv_buffer_bytes = Some(262144);
            let (_, rx) = socket.open().unwrap();
            let address = rx.local_addr().unwrap();
            let builder = Twin::twin(rx)
                .with_log_path(&received)
                .with_frame_capacity(4.try_into().unwrap());
            let builder = if mode == "disabled" {
                builder.archive_only()
            } else {
                builder
            };
            let (mut twin, reader) = builder.spawn().unwrap();
            let mut reader = Some(reader);
            if mode == "disconnected" {
                drop(reader.take());
            }
            std::thread::scope(|scope| {
                let producer = scope.spawn(|| run_sender(address, &sender, COUNT, 0).unwrap());
                if mode == "fast" {
                    let reader = reader.as_mut().unwrap();
                    let deadline = Instant::now() + Duration::from_secs(10);
                    while reader.status().archived < COUNT {
                        assert!(Instant::now() < deadline);
                        while let Some(update) = reader.try_read() {
                            let list = &update.frame.copperlist;
                            assert_eq!(
                                list.msgs.get_derived_output().payload().unwrap().0,
                                list.msgs.get_sum_output().payload().unwrap().0 % ITERATIONS
                            );
                        }
                        std::thread::sleep(Duration::from_millis(1));
                    }
                }
                producer.join().unwrap();
                std::thread::sleep(Duration::from_millis(100));
            });
            let status = twin.stop().unwrap();
            assert_eq!(status.archived, COUNT);
            assert_eq!(status.state, RecordingState::Closed);
            if mode == "stalled" {
                let reader = reader.as_mut().unwrap();
                let update = reader.try_read().unwrap();
                assert_eq!(update.missed, COUNT - 4);
                assert_eq!(update.frame.copperlist.id, COUNT - 4);
            }
            crate::verify(&sender, &received, crate::Expectation::Complete, COUNT).unwrap();
        }
    }
}
