use clap::{Args, Parser, Subcommand};
use cu_linux_resources::{SERIAL0_BAUDRATE_KEY, SERIAL0_DEV_KEY, SERIAL0_TIMEOUT_MS_KEY};
use cu_sensor_payloads::RangeObservation;
use cu29::prelude::*;
use cu29::units::si::length::meter;
use serialport::SerialPort;
use std::fs;
use std::io::ErrorKind;
use std::path::{Path, PathBuf};
use std::sync::{
    Arc, LazyLock, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, Instant};

const DEFAULT_BAUDRATE: u32 = 115_200;
const DEFAULT_SERIAL_TIMEOUT_MS: u64 = 20;
const DEFAULT_SETUP_TIMEOUT_MS: u64 = 100;
const DEFAULT_NETWORK_ID: &str = "REYAX123";
const DEFAULT_CPIN: &str = "FABC0002EEDCAA90FABC0002EEDCAA90";
const DEFAULT_ROBOT_ADDRESS: &str = "ROBOT001";
const DEFAULT_FIXED_ANCHOR_ADDRESS: &str = "ANCH0001";
const LOG_SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProbeSnapshot {
    pub count: u64,
    pub last: Option<RangeObservation>,
}

static SNAPSHOT: LazyLock<Mutex<ProbeSnapshot>> =
    LazyLock::new(|| Mutex::new(ProbeSnapshot::default()));

pub fn reset_probe_snapshot() {
    *SNAPSHOT.lock().expect("snapshot mutex poisoned") = ProbeSnapshot::default();
}

pub fn probe_snapshot() -> ProbeSnapshot {
    *SNAPSHOT.lock().expect("snapshot mutex poisoned")
}

pub mod tasks {
    use super::*;

    #[derive(Default, Reflect)]
    pub struct RangeCaptureSink;

    impl Freezable for RangeCaptureSink {}

    impl CuSinkTask for RangeCaptureSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(RangeObservation);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let Some(observation) = input.payload() else {
                return Ok(());
            };

            let mut snapshot = SNAPSHOT.lock().expect("snapshot mutex poisoned");
            snapshot.count = snapshot.count.saturating_add(1);
            snapshot.last = Some(*observation);
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct Ryuw122ProbeApp {}

#[derive(Parser, Debug)]
#[command(
    name = "cu-ryuw122-probe",
    about = "Configure and probe a robot-side REYAX RYUW122 ranging link"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    Robot(SerialPortArgs),
    Anchor(SerialPortArgs),
    Probe(ProbeArgs),
}

#[derive(Args, Debug, Clone)]
struct SerialPortArgs {
    #[arg(value_name = "PORT")]
    port: PathBuf,

    #[arg(long, default_value_t = DEFAULT_BAUDRATE)]
    baudrate: u32,
}

#[derive(Args, Debug, Clone)]
struct ProbeArgs {
    #[command(flatten)]
    serial: SerialPortArgs,

    #[arg(long)]
    duration_s: Option<u64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DeviceProfile {
    Robot,
    Anchor,
}

impl DeviceProfile {
    fn label(self) -> &'static str {
        match self {
            Self::Robot => "robot",
            Self::Anchor => "fixed anchor",
        }
    }

    fn vendor_mode_value(self) -> u8 {
        match self {
            Self::Robot => 1,
            Self::Anchor => 0,
        }
    }

    fn vendor_mode_label(self) -> &'static str {
        match self {
            Self::Robot => "initiator",
            Self::Anchor => "responder",
        }
    }

    fn address(self) -> &'static str {
        match self {
            Self::Robot => DEFAULT_ROBOT_ADDRESS,
            Self::Anchor => DEFAULT_FIXED_ANCHOR_ADDRESS,
        }
    }
}

pub fn run_cli() -> CuResult<()> {
    match Cli::parse().command {
        Command::Robot(args) => configure_modem(DeviceProfile::Robot, &args),
        Command::Anchor(args) => configure_modem(DeviceProfile::Anchor, &args),
        Command::Probe(args) => run_probe(&args),
    }
}

fn configure_modem(profile: DeviceProfile, args: &SerialPortArgs) -> CuResult<()> {
    let mut port = open_serial_port(&args.port, args.baudrate, DEFAULT_SETUP_TIMEOUT_MS)?;
    let _ = read_response(
        port.as_mut(),
        Duration::from_millis(40),
        Duration::from_millis(120),
    );

    expect_contains(&send_command(port.as_mut(), "AT")?, "+OK", "probing modem")?;

    for command in setup_commands(profile) {
        let response = send_command(port.as_mut(), &command)?;
        expect_contains(&response, "+OK", &format!("running `{command}`"))?;
        println!("{command} -> {}", one_line(&response));
    }

    for (command, expected) in verify_queries(profile) {
        let response = send_command(port.as_mut(), &command)?;
        expect_contains(&response, &expected, &format!("verifying `{command}`"))?;
        println!("{command} -> {}", one_line(&response));
    }

    println!(
        "configured {} {} as vendor {} ({}) on network {}",
        profile.label(),
        args.port.display(),
        profile.vendor_mode_label(),
        profile.address(),
        DEFAULT_NETWORK_ID,
    );
    Ok(())
}

fn run_probe(args: &ProbeArgs) -> CuResult<()> {
    change_to_manifest_dir()?;
    reset_probe_snapshot();

    let mut config = cu29::read_configuration("copperconfig.ron")?;
    override_serial_resource(
        &mut config,
        &args.serial.port,
        args.serial.baudrate,
        DEFAULT_SERIAL_TIMEOUT_MS,
    );

    let log_path = prepare_log_path()?;
    let mut app = Ryuw122ProbeApp::builder()
        .with_config(config)
        .with_log_path(&log_path, LOG_SLAB_SIZE)?
        .build()?;

    let running = Arc::new(AtomicBool::new(true));
    let running_for_signal = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_for_signal.store(false, Ordering::Relaxed);
    })
    .map_err(|err| CuError::new_with_cause("failed to install Ctrl-C handler", err))?;

    println!(
        "probing fixed anchor {} from robot modem {}",
        DEFAULT_FIXED_ANCHOR_ADDRESS,
        args.serial.port.display(),
    );
    println!("writing Copper log to {}", log_path.display());
    if let Some(duration_s) = args.duration_s {
        println!("running for {duration_s}s");
    } else {
        println!("press Ctrl-C to stop");
    }

    let started_at = Instant::now();
    let deadline = args.duration_s.map(Duration::from_secs);

    app.start_all_tasks()?;

    let mut last_count = 0;
    let mut run_error = None;
    while running.load(Ordering::Relaxed)
        && deadline
            .map(|duration| started_at.elapsed() < duration)
            .unwrap_or(true)
    {
        if let Err(err) = app.run_one_iteration() {
            run_error = Some(err);
            break;
        }

        let snapshot = probe_snapshot();
        if snapshot.count != last_count {
            if let Some(last) = snapshot.last {
                println!("[{:>4}] {}", snapshot.count, format_observation(last),);
            }
            last_count = snapshot.count;
        }

        thread::sleep(Duration::from_millis(5));
    }

    app.stop_all_tasks()?;

    if let Some(err) = run_error {
        return Err(err);
    }

    let snapshot = probe_snapshot();
    let Some(last) = snapshot.last else {
        return Err(CuError::from(format!(
            "no range observations captured; configure the robot with `just robot [port]`, configure the fixed anchor with `just anchor [port]`, and re-run `just probe [port]` for {}",
            args.serial.port.display()
        )));
    };

    println!(
        "captured {} observations; last {}",
        snapshot.count,
        format_observation(last),
    );
    Ok(())
}

fn setup_commands(profile: DeviceProfile) -> Vec<String> {
    let mut commands = vec![
        format!("AT+MODE={}", profile.vendor_mode_value()),
        format!("AT+NETWORKID={DEFAULT_NETWORK_ID}"),
        format!("AT+ADDRESS={}", profile.address()),
        format!("AT+CPIN={DEFAULT_CPIN}"),
    ];

    match profile {
        DeviceProfile::Robot => commands.push("AT+RSSI=1".to_string()),
        DeviceProfile::Anchor => commands.push("AT+TAGD=0,0".to_string()),
    }

    commands
}

fn verify_queries(profile: DeviceProfile) -> Vec<(String, String)> {
    let mut queries = vec![
        (
            "AT+MODE?".to_string(),
            format!("+MODE={}", profile.vendor_mode_value()),
        ),
        (
            "AT+NETWORKID?".to_string(),
            format!("+NETWORKID={DEFAULT_NETWORK_ID}"),
        ),
        (
            "AT+ADDRESS?".to_string(),
            format!("+ADDRESS={}", profile.address()),
        ),
        ("AT+CPIN?".to_string(), "+CPIN=".to_string()),
    ];

    match profile {
        DeviceProfile::Robot => queries.push(("AT+RSSI?".to_string(), "+RSSI=1".to_string())),
        DeviceProfile::Anchor => queries.push(("AT+TAGD?".to_string(), "+TAGD=0,0".to_string())),
    }

    queries
}

fn open_serial_port(port: &Path, baudrate: u32, timeout_ms: u64) -> CuResult<Box<dyn SerialPort>> {
    serialport::new(port.display().to_string(), baudrate)
        .timeout(Duration::from_millis(timeout_ms))
        .open()
        .map_err(|err| CuError::new_with_cause("failed to open serial port", err))
}

fn send_command(port: &mut dyn SerialPort, command: &str) -> CuResult<String> {
    let mut bytes = command.as_bytes().to_vec();
    bytes.extend_from_slice(b"\r\n");
    port.write_all(&bytes)
        .map_err(|err| CuError::new_with_cause("failed to write modem command", err))?;
    port.flush()
        .map_err(|err| CuError::new_with_cause("failed to flush modem command", err))?;

    read_response(port, Duration::from_millis(50), Duration::from_millis(500))
}

fn read_response(
    port: &mut dyn SerialPort,
    quiet_period: Duration,
    overall_timeout: Duration,
) -> CuResult<String> {
    let started_at = Instant::now();
    let mut last_data_at = None;
    let mut response = Vec::new();
    let mut scratch = [0_u8; 256];

    while started_at.elapsed() < overall_timeout {
        match port.read(&mut scratch) {
            Ok(0) => {}
            Ok(n) => {
                response.extend_from_slice(&scratch[..n]);
                last_data_at = Some(Instant::now());
            }
            Err(err)
                if matches!(
                    err.kind(),
                    ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
                ) => {}
            Err(err) => {
                return Err(CuError::new_with_cause(
                    "failed to read modem response",
                    err,
                ));
            }
        }

        if !response.is_empty()
            && last_data_at
                .map(|last| last.elapsed() >= quiet_period)
                .unwrap_or(false)
        {
            break;
        }
    }

    Ok(String::from_utf8_lossy(&response).trim().to_string())
}

fn expect_contains(response: &str, needle: &str, context: &str) -> CuResult<()> {
    if response.contains("+ERR") {
        return Err(CuError::from(format!(
            "{context} failed with modem error: {}",
            one_line(response)
        )));
    }
    if !response.contains(needle) {
        return Err(CuError::from(format!(
            "{context} expected `{needle}` but got `{}`",
            one_line(response)
        )));
    }
    Ok(())
}

fn one_line(response: &str) -> String {
    response
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .collect::<Vec<_>>()
        .join(" | ")
}

fn change_to_manifest_dir() -> CuResult<()> {
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR"))
        .map_err(|err| CuError::new_with_cause("failed to switch to example directory", err))
}

fn prepare_log_path() -> CuResult<PathBuf> {
    let log_path = PathBuf::from("logs/cu_ryuw122_probe.copper");
    if let Some(parent) = log_path.parent() {
        fs::create_dir_all(parent)
            .map_err(|err| CuError::new_with_cause("failed to create log directory", err))?;
    }
    Ok(log_path)
}

fn override_serial_resource(config: &mut CuConfig, port: &Path, baudrate: u32, timeout_ms: u64) {
    let Some(bundle) = config
        .resources
        .iter_mut()
        .find(|bundle| bundle.id == "linux")
    else {
        return;
    };

    let bundle_config = bundle.config.get_or_insert_with(ComponentConfig::new);
    bundle_config.set(SERIAL0_DEV_KEY, port.display().to_string());
    bundle_config.set(SERIAL0_BAUDRATE_KEY, baudrate);
    bundle_config.set(SERIAL0_TIMEOUT_MS_KEY, timeout_ms);
}

fn format_observation(observation: RangeObservation) -> String {
    let distance_m = observation.distance.get::<meter>();
    let rssi = observation
        .rssi_dbm
        .map(|value| format!("{value} dBm"))
        .unwrap_or_else(|| "n/a".to_string());

    format!(
        "anchor={} distance={distance_m:.2}m rssi={rssi}",
        observation.peer_id.as_str(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_setup_matches_probe_defaults() {
        assert_eq!(
            setup_commands(DeviceProfile::Robot),
            vec![
                "AT+MODE=1".to_string(),
                format!("AT+NETWORKID={DEFAULT_NETWORK_ID}"),
                format!("AT+ADDRESS={DEFAULT_ROBOT_ADDRESS}"),
                format!("AT+CPIN={DEFAULT_CPIN}"),
                "AT+RSSI=1".to_string(),
            ]
        );
    }

    #[test]
    fn anchor_setup_matches_probe_target() {
        assert_eq!(
            setup_commands(DeviceProfile::Anchor),
            vec![
                "AT+MODE=0".to_string(),
                format!("AT+NETWORKID={DEFAULT_NETWORK_ID}"),
                format!("AT+ADDRESS={DEFAULT_FIXED_ANCHOR_ADDRESS}"),
                format!("AT+CPIN={DEFAULT_CPIN}"),
                "AT+TAGD=0,0".to_string(),
            ]
        );
    }
}
