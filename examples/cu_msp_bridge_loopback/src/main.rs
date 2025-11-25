use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_msp_bridge::{MspRequestBatch, MspResponseBatch};
use cu_msp_lib::commands::MspCommandCode;
use cu_msp_lib::structs::{MspRc, MspResponse};
use cu_msp_lib::{MspPacket, MspParser};
use nix::errno::Errno;
use nix::fcntl::{fcntl, FcntlArg, OFlag};
use nix::pty::openpty;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::os::unix::fs::symlink;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct CuMspBridgeLoopbackApp {}

fn main() {
    if let Err(err) = drive() {
        error!("cu-msp-bridge-loopback failed: {}", err.to_string());
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let logs_dir = manifest_dir.join("logs");
    fs::create_dir_all(&logs_dir).map_err(|err| {
        CuError::new_with_cause(
            "failed to create logs directory for MSP loopback example",
            err,
        )
    })?;
    let logger_path = logs_dir.join("cu_msp_bridge_loopback.copper");
    let ctx = basic_copper_setup(&logger_path, Some(16 * 1024 * 1024), true, None)?;

    let config_path = manifest_dir.join("copperconfig.ron");
    let config_path_str = config_path
        .to_str()
        .ok_or_else(|| CuError::from("copperconfig.ron path is not valid UTF-8"))?
        .to_string();
    let mut config = read_configuration(&config_path_str)?;
    let emulator = FlightControllerEmulator::new(&logs_dir)?;
    emulator.configure_bridge(&mut config)?;
    install_ctrlc_cleanup(emulator.symlink_path.clone())?;

    let mut app = CuMspBridgeLoopbackAppBuilder::new()
        .with_context(&ctx)
        .with_config(config)
        .build()?;

    app.start_all_tasks()?;
    for i in 0..3 {
        debug!("Running iteration {}", i);
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

    cleanup_symlink(&emulator.symlink_path);
    drop(emulator);
    Ok(())
}

/// Emulates a simple MSP FC responding to RC packets
struct FlightControllerEmulator {
    symlink_path: PathBuf,
    running: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<()>>,
}

impl FlightControllerEmulator {
    fn new(logs_dir: &Path) -> CuResult<Self> {
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

        let symlink_path = logs_dir.join("msp_bridge_loopback_slave");
        if let Some(parent) = symlink_path.parent() {
            fs::create_dir_all(parent).map_err(|err| {
                CuError::new_with_cause(
                    "failed to ensure MSP flight controller emulator symlink directory exists",
                    err,
                )
            })?;
        }
        if symlink_path.exists() {
            fs::remove_file(&symlink_path).map_err(|err| {
                CuError::new_with_cause(
                    "failed to remove stale MSP flight controller emulator symlink",
                    err,
                )
            })?;
        }
        symlink(&slave_path, &symlink_path).map_err(|err| {
            CuError::new_with_cause(
                "failed to create MSP flight controller emulator symlink",
                err,
            )
        })?;

        let running = Arc::new(AtomicBool::new(true));
        let thread_running = running.clone();
        let worker = thread::spawn(move || flight_controller_worker(thread_running, pty.master));

        Ok(Self {
            symlink_path,
            running,
            worker: Some(worker),
        })
    }

    fn configure_bridge(&self, config: &mut CuConfig) -> CuResult<()> {
        let device_path = self.symlink_path.to_string_lossy().into_owned();
        let graph = config.get_graph_mut(None)?;
        let node_id = graph
            .get_node_id_by_name("msp_bridge")
            .ok_or_else(|| CuError::from("config missing `msp_bridge` node"))?;
        let node = graph
            .get_node_mut(node_id)
            .ok_or_else(|| CuError::from("unable to mutate msp_bridge node"))?;
        node.set_param("device", device_path.clone());

        if let Some(bridge_cfg) = config.bridges.iter_mut().find(|b| b.id == "msp_bridge") {
            let component = bridge_cfg.config.get_or_insert_with(ComponentConfig::new);
            component.set("device", device_path);
        } else {
            return Err(CuError::from("config missing `msp_bridge` bridge entry"));
        }
        Ok(())
    }
}

impl Drop for FlightControllerEmulator {
    fn drop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
        cleanup_symlink(&self.symlink_path);
    }
}

fn flight_controller_worker(running: Arc<AtomicBool>, master_fd: std::os::unix::io::OwnedFd) {
    let mut file = File::from(master_fd);
    if let Err(err) = set_nonblocking(&file) {
        error!(
            "MSP emulator failed to mark PTY non-blocking: {}",
            err.to_string()
        );
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
            Err(err) if err.raw_os_error() == Some(Errno::EIO as i32) => {
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
    let flags = OFlag::from_bits_truncate(
        fcntl(file, FcntlArg::F_GETFL)
            .map_err(|err| CuError::new_with_cause("failed to read PTY flags", err))?,
    );
    fcntl(file, FcntlArg::F_SETFL(flags | OFlag::O_NONBLOCK))
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

fn install_ctrlc_cleanup(symlink_path: PathBuf) -> CuResult<()> {
    ctrlc::set_handler(move || {
        cleanup_symlink(&symlink_path);
        std::process::exit(130);
    })
    .map_err(|err| CuError::new_with_cause("failed to install Ctrl-C handler", err))
}

fn cleanup_symlink(path: &Path) {
    if path.exists() {
        let _ = fs::remove_file(path);
    }
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
                debug!("LoopbackSource: completed sending MSP request");
                output.clear_payload();
            } else {
                let mut batch = MspRequestBatch::new();
                debug!("Pushing MSP_RC request");
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
                debug!(
                    "LoopbackSink: received MSP response batch with {} responses",
                    batch.0.len()
                );
                if let Some(MspResponse::MspRc(rc)) = batch.0.first() {
                    debug!("Received RC response: {}", rc);
                    if rc.channels[0] == 1234 {
                        state::mark_valid();
                    }
                }
            }
            Ok(())
        }
    }
}
