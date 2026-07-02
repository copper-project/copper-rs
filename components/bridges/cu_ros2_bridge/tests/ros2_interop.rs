use cu_ros2_bridge::Ros2Bridge;
use cu29::prelude::*;
use std::error::Error;
use std::io;
use std::net::TcpStream;
use std::process::{Child, Command, Output, Stdio};
use std::thread;
use std::time::{Duration, Instant};

tx_channels! {
    struct InteropTxChannels : InteropTxId {
        out => i32,
    }
}

rx_channels! {
    struct InteropRxChannels : InteropRxId {
        unused => i32,
    }
}

type InteropBridge = Ros2Bridge<InteropTxChannels, InteropRxChannels>;

#[test]
fn ros2_echo_reads_copper_published_std_msgs_int32_over_zenoh() -> Result<(), Box<dyn Error>> {
    if let Err(reason) = probe_ros2_interop_dependencies() {
        eprintln!(
            "warning: skipping ROS 2 interop test: {reason}. Install ROS 2 with std_msgs and rmw_zenoh_cpp to exercise real ROS 2 bridge interoperability."
        );
        return Ok(());
    }

    run_ros2_interop()
}

fn run_ros2_interop() -> Result<(), Box<dyn Error>> {
    let domain_id = env_u32("ROS_DOMAIN_ID", 0)?;
    let endpoint = "tcp/127.0.0.1:7447";
    let topic = format!("/copper_ros2_bridge_interop_{}", std::process::id());

    stop_ros2_daemon();

    let mut zenohd = ensure_zenoh_router(endpoint, domain_id)?;

    let mut echo = spawn_ros2_echo(&topic, domain_id)?;
    thread::sleep(Duration::from_millis(750));

    let mut bridge_cfg = ComponentConfig::default();
    bridge_cfg.set("domain_id", domain_id);
    bridge_cfg.set("namespace", "copper".to_string());
    bridge_cfg.set("node", "ros2_interop_test".to_string());
    bridge_cfg.set("zenoh_config_json", zenoh_client_config_json(endpoint));

    let tx_channels = [BridgeChannelConfig::from_static(
        &InteropTxChannels::OUT,
        Some(topic),
        None,
    )];
    let mut bridge = <InteropBridge as CuBridge>::new(Some(&bridge_cfg), &tx_channels, &[], ())?;
    let ctx = CuContext::new_with_clock();
    bridge.start(&ctx)?;

    let result = publish_until_echo_receives(&mut bridge, &ctx, &mut echo);
    bridge.stop(&ctx)?;

    if let Some(router) = zenohd.as_mut() {
        router.kill_and_wait();
    }

    result
}

fn probe_ros2_interop_dependencies() -> Result<(), String> {
    require_command_success(Command::new("ros2").arg("--help"), "ros2 --help")?;
    require_command_success(
        Command::new("ros2").args(["pkg", "prefix", "rmw_zenoh_cpp"]),
        "ros2 pkg prefix rmw_zenoh_cpp",
    )?;
    require_command_success(
        Command::new("ros2").args(["pkg", "prefix", "std_msgs"]),
        "ros2 pkg prefix std_msgs",
    )?;
    Ok(())
}

fn require_command_success(command: &mut Command, label: &str) -> Result<(), String> {
    let output = command
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .output()
        .map_err(|err| format!("{label} failed to spawn: {err}"))?;

    if output.status.success() {
        Ok(())
    } else {
        Err(format!(
            "{label} exited with {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        ))
    }
}

fn ensure_zenoh_router(
    endpoint: &str,
    domain_id: u32,
) -> Result<Option<ChildGuard>, Box<dyn Error>> {
    if tcp_endpoint_reachable(endpoint) {
        return Ok(None);
    }

    let mut command = Command::new("ros2");
    command
        .args(["run", "rmw_zenoh_cpp", "rmw_zenohd"])
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut guard = ChildGuard::spawn(command, "rmw_zenohd")?;
    wait_for_tcp_endpoint(endpoint, Duration::from_secs(10), Some(&mut guard))?;
    Ok(Some(guard))
}

fn spawn_ros2_echo(topic: &str, domain_id: u32) -> Result<ChildGuard, Box<dyn Error>> {
    let mut command = Command::new("ros2");
    command
        .args(["topic", "echo", "--once", topic, "std_msgs/msg/Int32"])
        .env("ROS_DOMAIN_ID", domain_id.to_string())
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    ChildGuard::spawn(command, "ros2 topic echo")
}

fn publish_until_echo_receives(
    bridge: &mut InteropBridge,
    ctx: &CuContext,
    echo: &mut ChildGuard,
) -> Result<(), Box<dyn Error>> {
    let deadline = Instant::now() + Duration::from_secs(15);

    while Instant::now() < deadline {
        let msg = CuMsg::new(Some(42i32));
        bridge.send(ctx, &InteropTxChannels::OUT, &msg)?;

        if echo.try_wait()?.is_some() {
            let output = echo.wait_with_output()?;
            let stdout = String::from_utf8_lossy(&output.stdout);
            let stderr = String::from_utf8_lossy(&output.stderr);

            if !output.status.success() {
                return Err(boxed_error(format!(
                    "ros2 topic echo exited with {}.\nstdout:\n{}\nstderr:\n{}",
                    output.status, stdout, stderr
                )));
            }

            if stdout.contains("data: 42") {
                return Ok(());
            }

            return Err(boxed_error(format!(
                "ros2 topic echo exited without the expected sample.\nstdout:\n{}\nstderr:\n{}",
                stdout, stderr
            )));
        }

        thread::sleep(Duration::from_millis(100));
    }

    echo.kill_and_wait();
    Err(boxed_error(
        "timed out waiting for ros2 topic echo to receive data: 42",
    ))
}

fn zenoh_client_config_json(endpoint: &str) -> String {
    format!(
        r#"{{
            mode: "client",
            connect: {{ endpoints: ["{endpoint}"] }},
            scouting: {{ multicast: {{ enabled: false }}, gossip: {{ enabled: false }} }}
        }}"#
    )
}

fn wait_for_tcp_endpoint(
    endpoint: &str,
    timeout: Duration,
    mut child: Option<&mut ChildGuard>,
) -> Result<(), Box<dyn Error>> {
    let addr = endpoint
        .strip_prefix("tcp/")
        .ok_or_else(|| boxed_error(format!("expected tcp/ endpoint, got {endpoint}")))?;
    let deadline = Instant::now() + timeout;

    while Instant::now() < deadline {
        if TcpStream::connect(addr).is_ok() {
            return Ok(());
        }

        if let Some(child) = child.as_deref_mut()
            && let Some(status) = child.try_wait()?
        {
            let output = child.wait_with_output()?;
            return Err(boxed_error(format!(
                "{} exited before {endpoint} became reachable: {status}\nstdout:\n{}\nstderr:\n{}",
                child.name,
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            )));
        }

        thread::sleep(Duration::from_millis(100));
    }

    Err(boxed_error(format!("timed out waiting for {endpoint}")))
}

fn tcp_endpoint_reachable(endpoint: &str) -> bool {
    endpoint
        .strip_prefix("tcp/")
        .map(|addr| TcpStream::connect(addr).is_ok())
        .unwrap_or(false)
}

fn stop_ros2_daemon() {
    let _ = Command::new("ros2")
        .args(["daemon", "stop"])
        .env("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

fn env_u32(name: &str, default: u32) -> Result<u32, Box<dyn Error>> {
    match std::env::var(name) {
        Ok(value) => value
            .parse::<u32>()
            .map_err(|err| boxed_error(format!("{name} must be a u32, got {value:?}: {err}"))),
        Err(std::env::VarError::NotPresent) => Ok(default),
        Err(err) => Err(boxed_error(format!("failed to read {name}: {err}"))),
    }
}

fn boxed_error(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::other(message.into()))
}

struct ChildGuard {
    child: Option<Child>,
    name: &'static str,
}

impl ChildGuard {
    fn spawn(mut command: Command, name: &'static str) -> Result<Self, Box<dyn Error>> {
        let child = command
            .spawn()
            .map_err(|err| boxed_error(format!("failed to spawn {name}: {err}")))?;
        Ok(Self {
            child: Some(child),
            name,
        })
    }

    fn try_wait(&mut self) -> io::Result<Option<std::process::ExitStatus>> {
        match self.child.as_mut() {
            Some(child) => child.try_wait(),
            None => Ok(None),
        }
    }

    fn wait_with_output(&mut self) -> io::Result<Output> {
        self.child
            .take()
            .expect("child already consumed")
            .wait_with_output()
    }

    fn kill_and_wait(&mut self) {
        if let Some(mut child) = self.child.take() {
            if matches!(child.try_wait(), Ok(None)) {
                let _ = child.kill();
            }
            let _ = child.wait();
        }
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        self.kill_and_wait();
    }
}
