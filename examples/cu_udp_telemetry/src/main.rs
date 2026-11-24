//! Run a robot and ground station over loopback, keeping both native archives.
use cu_udp_telemetry::{MAX_PAYLOAD, PlainUdpRx, PlainUdpTx};
use cu29::prelude::*;
use cu29_logstream::TelemetryBufferConfig;
use cu29_logstream::protocol::CuTelemetryTx;
use cu29_logstream_udp::CuUdpLogStreamConfig;
use std::error::Error;
use std::path::Path;
use std::time::{Duration, Instant};

#[copper_runtime(config = "copperconfig.ron")]
struct Robot {}

mod ground {
    use cu29::prelude::*;
    #[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
    struct Replay {}
    pub type Twin = default::Replay;
}

const CYCLES: u64 = 48;

fn run_robot<T: CuTelemetryTx + 'static>(
    make_tx: impl FnOnce(RobotClock) -> CuResult<T> + Send + 'static,
    log: &Path,
) -> Result<(), Box<dyn Error>> {
    let (clock, mock) = RobotClock::mock();
    let builder = Robot::builder()
        .with_clock(clock)
        .with_log_path(log, Some(cu_logstream_demo::SLAB_BYTES))?;
    let buffers = TelemetryBufferConfig {
        capture_count: 1,
        keyframe_count: 1,
        structured_log_count: 1,
        ..TelemetryBufferConfig::new(MAX_PAYLOAD, MAX_PAYLOAD)
    };
    #[cfg(feature = "host")]
    let robot = builder.with_telemetry_tx(make_tx, buffers).build()?;
    #[cfg(not(feature = "host"))]
    let robot = builder
        .with_telemetry_tx(cu29_logstream::new_session_id(), make_tx, buffers)
        .build()?;
    let mut running = robot.start()?;
    for id in 0..CYCLES {
        mock.set_value((id + 1) * cu_logstream_demo::TICK_NS);
        running.run_one_iteration()?;
        std::thread::sleep(Duration::from_millis(10));
    }
    drop(running.stop()?);
    Ok(())
}

fn loopback<T: CuTelemetryTx + 'static>(
    dir: &Path,
    make_tx: impl FnOnce(RobotClock, PlainUdpTx<cu29_logstream_udp::CuUdpLogStreamTx>) -> CuResult<T>
    + Send
    + 'static,
) -> Result<cu29_logstream::CuTwinStatus, Box<dyn Error>> {
    std::fs::create_dir_all(dir)?;
    let (_, rx) = CuUdpLogStreamConfig::new("127.0.0.1:0".parse()?).open()?;
    let address = rx.local_addr()?;
    let (mut twin, mut frames) = ground::Twin::from_telemetry(PlainUdpRx::new(rx))
        .with_log_path(dir.join("ground.copper"))
        .spawn()?;
    let mut sender = CuUdpLogStreamConfig::new("127.0.0.1:0".parse()?);
    sender.remote_addr = Some(address);
    let (tx, _) = sender.open()?;
    let udp = PlainUdpTx::new(tx.ok_or("missing UDP sender")?);
    run_robot(move |clock| make_tx(clock, udp), &dir.join("robot.copper"))?;

    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        while let Some(read) = frames.try_read() {
            let list = &read.frame.copperlist;
            if list.id == CYCLES - 1 {
                let angles = cu_logstream_demo::tasks::JointAngles::at_tick(list.id);
                if list.msgs.get_encoders_output().payload() != Some(&angles)
                    || list.msgs.get_kinematics_output().payload().is_none()
                {
                    return Err("final twin frame did not contain the captured angles and reconstructed pose".into());
                }
                return Ok(twin.stop()?);
            }
        }
        frames.status();
        if frames.is_closed() || Instant::now() >= deadline {
            let status = twin.stop()?;
            return Err(format!("timed out waiting for final twin frame: {status:?}").into());
        }
        frames.wait_timeout(Duration::from_millis(50));
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let dir = std::env::args_os()
        .nth(1)
        .ok_or("pass a fresh archive directory")?;
    let status = loopback(Path::new(&dir), |_, tx| Ok(tx))?;
    println!(
        "Archived {} CopperLists, {} gaps and {} structured logs.",
        status.archived, status.gaps, status.structured_logs
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_logstream::protocol::{CuTelemetryRecord, TxMode};

    struct SkipOne<T>(T);
    impl<T: CuTelemetryTx> CuTelemetryTx for SkipOne<T> {
        fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
            if matches!(record, CuTelemetryRecord::CopperList { id: 17, .. }) {
                return Ok(());
            }
            self.0.send(record)
        }

        fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
            self.0.poll(mode)
        }
    }

    #[cfg(not(feature = "host"))]
    #[test]
    fn threadless_udp_recovers_at_the_next_keyframe() -> Result<(), Box<dyn Error>> {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
        std::fs::create_dir_all(&root)?;
        let dir = tempfile::tempdir_in(root)?;
        let status = loopback(dir.path(), |_, udp| Ok(SkipOne(udp)))?;
        assert_eq!(status.gaps, 1);
        assert_eq!(status.archived, CYCLES - 1);
        assert_eq!(status.recovery_point, Some(32));
        assert!(status.structured_logs > 0);
        Ok(())
    }

    #[cfg(feature = "host")]
    #[test]
    fn fec_keeps_every_capture_while_plain_udp_recovers_from_loss() -> Result<(), Box<dyn Error>> {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
        std::fs::create_dir_all(&root)?;
        let dir = tempfile::tempdir_in(root)?;
        use cu29_logstream::protocol::{
            BestEffort, CuTelemetryEvent, CuTelemetryPoll, CuTelemetryRx, Fanout,
        };
        use cu29_logstream::{FecTelemetryRx, FecTelemetryTx, LogStreamPlan};

        let config =
            CuConfig::deserialize_ron(include_str!("../../cu_logstream_demo/copperconfig.ron"))?;
        let plan = LogStreamPlan::resolve(&config.log_streaming.unwrap().destinations[0])?;
        let (_, rx) = CuUdpLogStreamConfig::new("127.0.0.1:0".parse()?).open()?;
        let mut link = CuUdpLogStreamConfig::new("127.0.0.1:0".parse()?);
        link.remote_addr = Some(rx.local_addr()?);
        let (tx, _) = link.open()?;
        let transport = tx.unwrap();
        let mut receiver = FecTelemetryRx::new(rx, Default::default(), None)?;
        let fec_captures = std::thread::spawn(move || -> CuResult<Vec<u64>> {
            let deadline = Instant::now() + Duration::from_secs(5);
            let mut ids = Vec::new();
            while Instant::now() < deadline && ids.last() != Some(&(CYCLES - 1)) {
                let outcome = receiver.try_recv(|event| {
                    if let CuTelemetryEvent::Record(CuTelemetryRecord::CopperList { id, .. }) =
                        event
                    {
                        ids.push(id);
                    }
                    Ok(())
                })?;
                if outcome == CuTelemetryPoll::Pending {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
            Ok(ids)
        });
        let status = loopback(dir.path(), move |clock, udp| {
            assert!(!clock.is_mock());
            let fec = FecTelemetryTx::new(transport, plan, clock)?;
            Ok(Fanout(
                fec,
                BestEffort::new(SkipOne(udp), |err| panic!("UDP output failed: {err}")),
            ))
        })?;
        assert_eq!(
            fec_captures.join().unwrap()?,
            (0..CYCLES).collect::<Vec<_>>()
        );
        assert_eq!(status.gaps, 1);
        assert_eq!(status.archived, CYCLES - 1);
        assert!(status.structured_logs > 0);
        assert_eq!(status.recovery_point, Some(32));
        let ground = dir.path().join("ground.copper");
        assert!(
            cu_logstream_demo::read_lists(&ground)?
                .iter()
                .all(|list| list.id != 17)
        );
        assert!(
            cu_logstream_demo::read_keyframes(&ground)?
                .iter()
                .any(|frame| frame.culistid == 32)
        );
        Ok(())
    }
}
