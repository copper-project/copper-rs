use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_msp_bridge::{MspRequestBatch, MspResponseBatch};
use cu_msp_lib::commands::MspCommandCode;
use cu_msp_lib::structs::{MspRc, MspResponse};
use cu_msp_lib::{MspPacket, MspParser};
use nix::fcntl::{fcntl, FcntlArg, OFlag};
use nix::pty::openpty;
use std::fs::File;
use std::io::{Read, Write};
use std::os::unix::io::AsRawFd;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct CuMspBridgeLoopbackApp {}

fn main() {
    if let Err(err) = drive() {
        eprintln!("cu-msp-bridge-loopback failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let logger_path = PathBuf::from("logs/cu_msp_bridge_loopback.copper");
    let ctx = basic_copper_setup(&logger_path, Some(8 * 1024 * 1024), true, None)?;

    let mut config = read_configuration("copperconfig.ron")?;
    let emulator = FlightControllerEmulator::new()?;
    emulator.configure_bridge(&mut config)?;

    let mut app = CuMspBridgeLoopbackAppBuilder::new()
        .with_context(&ctx)
        .with_config(config)
        .build()?;

    app.start_all_tasks()?;
    for _ in 0..3 {
        app.run_one_iteration()?;
        if tasks::state::was_validated() {
            break;
        }
        thread::sleep(Duration::from_millis(10));
    }
    app.stop_all_tasks()?;

    if !tasks::state::was_validated() {
        return Err(CuError::from(
            "MSP flight controller emulator did not validate the response",
        ));
    }

    drop(emulator);
    Ok(())
}

/// Emulates a simple MSP FC responding to RC packets
struct FlightControllerEmulator {
    slave_path: String,
    running: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<()>>,
}

impl FlightControllerEmulator {
    fn new() -> CuResult<Self> {
        let pty = openpty(None, None).map_err(|err| {
            CuError::new_with_cause(
                "failed to create pseudo terminal for MSP flight controller emulator",
                err,
            )
        })?;

        let slave_path = std::fs::read_link(format!("/proc/self/fd/{}", pty.slave.as_raw_fd()))
            .map_err(|err| {
                CuError::new_with_cause(
                    "failed to retrieve slave PTY path for MSP flight controller emulator",
                    err,
                )
            })?
            .to_string_lossy()
            .into_owned();

        // Drop the slave handle – CuMspBridge will open it by path.
        drop(pty.slave);

        let running = Arc::new(AtomicBool::new(true));
        let thread_running = running.clone();
        let worker = thread::spawn(move || flight_controller_worker(thread_running, pty.master));

        Ok(Self {
            slave_path,
            running,
            worker: Some(worker),
        })
    }

    fn configure_bridge(&self, config: &mut CuConfig) -> CuResult<()> {
        let graph = config.get_graph_mut(None)?;
        let node_id = graph
            .get_node_id_by_name("msp_bridge")
            .ok_or_else(|| CuError::from("config missing `msp_bridge` node"))?;
        let node = graph
            .get_node_mut(node_id)
            .ok_or_else(|| CuError::from("unable to mutate msp_bridge node"))?;
        node.set_param("device", self.slave_path.clone());
        Ok(())
    }
}

impl Drop for FlightControllerEmulator {
    fn drop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
    }
}

fn flight_controller_worker(running: Arc<AtomicBool>, master_fd: std::os::unix::io::OwnedFd) {
    let mut file = File::from(master_fd);
    if let Err(err) = set_nonblocking(&file) {
        eprintln!("MSP emulator failed to mark PTY non-blocking: {err}");
        return;
    }

    let mut parser = MspParser::new();
    let mut buffer = [0u8; 512];

    while running.load(Ordering::SeqCst) {
        match file.read(&mut buffer) {
            Ok(0) => {
                thread::sleep(Duration::from_millis(5));
            }
            Ok(n) => {
                for &byte in &buffer[..n] {
                    if let Ok(Some(packet)) = parser.parse(byte) {
                        let command: MspCommandCode = packet.cmd.into();
                        if command == MspCommandCode::MSP_RC {
                            if let Err(err) = write_rc_response(&mut file) {
                                eprintln!("MSP emulator failed to echo RC packet: {err}");
                                return;
                            }
                        }
                    }
                }
            }
            Err(err) if err.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(5));
            }
            Err(err) => {
                eprintln!("MSP emulator PTY read failed: {err}");
                return;
            }
        }
    }
}

fn set_nonblocking(file: &File) -> CuResult<()> {
    let fd = file.as_raw_fd();
    let flags = OFlag::from_bits_truncate(
        fcntl(fd, FcntlArg::F_GETFL)
            .map_err(|err| CuError::new_with_cause("failed to read PTY flags", err))?,
    );
    fcntl(fd, FcntlArg::F_SETFL(flags | OFlag::O_NONBLOCK))
        .map_err(|err| CuError::new_with_cause("failed to set PTY non-blocking mode", err))?;
    Ok(())
}

fn write_rc_response(file: &mut File) -> std::io::Result<()> {
    let mut rc = MspRc::new();
    rc.set_roll(1234);
    rc.set_pitch(1500);
    rc.set_yaw(1600);

    let response = MspResponse::MspRc(rc);
    let packet: MspPacket = response.into();
    let size = packet.packet_size_bytes();
    let mut data = vec![0u8; size];
    packet.serialize(&mut data).expect("serialize MSP response");
    file.write_all(&data)?;
    file.flush()
}

mod tasks {
    use super::*;

    pub mod state {
        use std::sync::atomic::{AtomicBool, Ordering};

        static VALIDATED: AtomicBool = AtomicBool::new(false);

        pub fn mark_valid() {
            VALIDATED.store(true, Ordering::SeqCst);
        }

        pub fn was_validated() -> bool {
            VALIDATED.load(Ordering::SeqCst)
        }
    }

    #[derive(Default)]
    pub struct LoopbackSource {
        sent: bool,
    }

    impl Freezable for LoopbackSource {}

    impl CuSrcTask for LoopbackSource {
        type Output<'m> = CuMsg<MspRequestBatch>;

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self { sent: false })
        }

        fn process(&mut self, _clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
            if self.sent {
                output.clear_payload();
            } else {
                let mut batch = MspRequestBatch::new();
                batch.push(cu_msp_lib::structs::MspRequest::MspRc);
                output.set_payload(batch);
                self.sent = true;
            }
            Ok(())
        }
    }

    #[derive(Default)]
    pub struct LoopbackSink;

    impl Freezable for LoopbackSink {}

    impl CuSinkTask for LoopbackSink {
        type Input<'m> = CuMsg<MspResponseBatch>;

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
            if let Some(batch) = input.payload() {
                if let Some(MspResponse::MspRc(rc)) = batch.0.first() {
                    if rc.channels[0] == 1234 {
                        state::mark_valid();
                    }
                }
            }
            Ok(())
        }
    }
}
