use bincode::{Decode, Encode};
use cu29::config::{BridgeChannelConfigRepresentation, ComponentConfig, CuConfig};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};
use std::vec::Vec;

const REPORT_WINDOW_CAPACITY: usize = 4_096;
pub const BRIDGE_ID: &str = "zenoh";
pub const CHANNEL_ID: &str = "latency";
pub const LATENCY_ROUTE: &str = "benchmark/zenoh/latency";

fn report_interval() -> CuDuration {
    CuDuration::from_secs(1)
}

fn wall_clock_now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_else(|_| std::time::Duration::from_secs(0))
        .as_nanos() as u64
}

fn wall_clock_latency(sent_at_wall_ns: u64) -> CuDuration {
    CuDuration::from_nanos(wall_clock_now_ns().saturating_sub(sent_at_wall_ns))
}

fn percentile(sorted: &[u64], num: u64, den: u64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let len = sorted.len();
    let rank = ((len - 1) as u128 * num as u128) / den as u128;
    sorted[rank as usize]
}

fn format_duration(ns: u64) -> String {
    if ns >= 1_000_000 {
        format!("{:.3}ms", ns as f64 / 1_000_000.0)
    } else if ns >= 1_000 {
        format!("{:.3}us", ns as f64 / 1_000.0)
    } else {
        format!("{ns}ns")
    }
}

#[derive(Debug, Clone)]
struct LatencyProbe {
    last_report_at: Option<CuTime>,
    last_seq: Option<u64>,
    last_latency_ns: Option<u64>,
    samples_ns: Vec<u64>,
    jitter_ns: Vec<u64>,
    dropped_messages: u64,
    max_seq_gap: u64,
}

impl Default for LatencyProbe {
    fn default() -> Self {
        Self::new()
    }
}

impl LatencyProbe {
    fn new() -> Self {
        Self {
            last_report_at: None,
            last_seq: None,
            last_latency_ns: None,
            samples_ns: Vec::with_capacity(REPORT_WINDOW_CAPACITY),
            jitter_ns: Vec::with_capacity(REPORT_WINDOW_CAPACITY),
            dropped_messages: 0,
            max_seq_gap: 1,
        }
    }

    fn observe(&mut self, now: CuTime, seq: u64, latency: CuDuration) {
        let latency_ns = latency.as_nanos();

        if let Some(last_seq) = self.last_seq {
            let gap = seq.saturating_sub(last_seq);
            self.max_seq_gap = self.max_seq_gap.max(gap);
            if gap > 1 {
                self.dropped_messages = self.dropped_messages.saturating_add(gap - 1);
            }
        }
        self.last_seq = Some(seq);
        if self.samples_ns.len() < REPORT_WINDOW_CAPACITY {
            self.samples_ns.push(latency_ns);
        }
        if let Some(last_latency_ns) = self.last_latency_ns
            && self.jitter_ns.len() < REPORT_WINDOW_CAPACITY
        {
            self.jitter_ns.push(latency_ns.abs_diff(last_latency_ns));
        }
        self.last_latency_ns = Some(latency_ns);

        let Some(last_report_at) = self.last_report_at else {
            self.last_report_at = Some(now);
            return;
        };

        if now - last_report_at < report_interval() || self.samples_ns.is_empty() {
            return;
        }

        self.samples_ns.sort_unstable();
        self.jitter_ns.sort_unstable();

        let samples = self.samples_ns.len();
        let min_ns = *self.samples_ns.first().unwrap_or(&0);
        let p50_ns = percentile(&self.samples_ns, 50, 100);
        let p95_ns = percentile(&self.samples_ns, 95, 100);
        let p99_ns = percentile(&self.samples_ns, 99, 100);
        let max_ns = *self.samples_ns.last().unwrap_or(&0);
        let mean_ns = (self
            .samples_ns
            .iter()
            .map(|&sample| sample as u128)
            .sum::<u128>()
            / samples as u128) as u64;
        let variance_ns = self
            .samples_ns
            .iter()
            .map(|&sample| {
                let delta = sample as f64 - mean_ns as f64;
                delta * delta
            })
            .sum::<f64>()
            / samples as f64;
        let stddev_ns = variance_ns.sqrt().round() as u64;
        let jitter_p95_ns = percentile(&self.jitter_ns, 95, 100);
        let jitter_max_ns = *self.jitter_ns.last().unwrap_or(&0);

        let line = format!(
            "[\x1b[94mlatency\x1b[0m] \x1b[92msamples\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mmin\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mp50\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mp95\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mp99\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mmax\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mmean\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mstddev\x1b[0m \x1b[36m{}\x1b[0m | \x1b[92mjitter\x1b[0m \x1b[93mp95\x1b[0m \x1b[36m{}\x1b[0m \x1b[93mmax\x1b[0m \x1b[36m{}\x1b[0m | \x1b[92mdropped\x1b[0m \x1b[36m{}\x1b[0m \x1b[92mmax_gap\x1b[0m \x1b[36m{}\x1b[0m",
            samples,
            format_duration(min_ns),
            format_duration(p50_ns),
            format_duration(p95_ns),
            format_duration(p99_ns),
            format_duration(max_ns),
            format_duration(mean_ns),
            format_duration(stddev_ns),
            format_duration(jitter_p95_ns),
            format_duration(jitter_max_ns),
            self.dropped_messages,
            self.max_seq_gap,
        );
        println!("{}", line);

        self.last_report_at = Some(now);
        self.samples_ns.clear();
        self.jitter_ns.clear();
        self.dropped_messages = 0;
        self.max_seq_gap = 1;
    }
}

pub mod messages {
    use super::*;

    #[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct LatencySample {
        pub seq: u64,
        pub sent_at_wall_ns: u64,
    }
}

pub struct BenchRunOptions {
    pub log_path: PathBuf,
    pub instance_id: u32,
    pub transport: BenchTransport,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchEndpointRole {
    Tx,
    Rx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchTransport {
    Shmem,
    Udp,
    Tcp,
}

impl BenchTransport {
    pub fn parse(raw: &str) -> CuResult<Self> {
        match raw {
            "shmem" => Ok(Self::Shmem),
            "udp" => Ok(Self::Udp),
            "tcp" => Ok(Self::Tcp),
            other => Err(CuError::from(format!(
                "invalid --transport '{other}', expected shmem, udp, or tcp"
            ))),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Shmem => "shmem",
            Self::Udp => "udp",
            Self::Tcp => "tcp",
        }
    }

    fn connect_endpoint(self, role: BenchEndpointRole) -> Option<&'static str> {
        match (self, role) {
            (Self::Shmem, BenchEndpointRole::Tx) => {
                Some("unixsock-stream//tmp/cu29_zenoh_bridge_bench.sock?rel=0")
            }
            (Self::Udp, BenchEndpointRole::Tx) => Some("udp/127.0.0.1:19001?rel=0"),
            (Self::Tcp, BenchEndpointRole::Tx) => Some("tcp/127.0.0.1:19000?rel=1"),
            _ => None,
        }
    }

    fn listen_endpoint(self, role: BenchEndpointRole) -> Option<&'static str> {
        match (self, role) {
            (Self::Shmem, BenchEndpointRole::Rx) => {
                Some("unixsock-stream//tmp/cu29_zenoh_bridge_bench.sock?rel=0")
            }
            (Self::Udp, BenchEndpointRole::Rx) => Some("udp/127.0.0.1:19001?rel=0"),
            (Self::Tcp, BenchEndpointRole::Rx) => Some("tcp/127.0.0.1:19000?rel=1"),
            _ => None,
        }
    }

    fn uses_shared_memory(self) -> bool {
        matches!(self, Self::Shmem)
    }

    fn zenoh_config_json(self, role: BenchEndpointRole) -> &'static str {
        match (self, role) {
            (Self::Shmem, BenchEndpointRole::Tx) => {
                r#"
                    {
                      mode: "peer",
                      scouting: {
                        multicast: { enabled: false },
                      },
                      listen: { endpoints: [] },
                      connect: {
                        timeout_ms: -1,
                        exit_on_failure: false,
                        endpoints: ["unixsock-stream//tmp/cu29_zenoh_bridge_bench.sock?rel=0"],
                      },
                      transport: {
                        shared_memory: {
                          enabled: true,
                          transport_optimization: {
                            enabled: true,
                            message_size_threshold: 1,
                          },
                        },
                      },
                    }
                "#
            }
            (Self::Shmem, BenchEndpointRole::Rx) => {
                r#"
                    {
                      mode: "peer",
                      scouting: {
                        multicast: { enabled: false },
                      },
                      connect: { endpoints: [] },
                      listen: { endpoints: ["unixsock-stream//tmp/cu29_zenoh_bridge_bench.sock?rel=0"] },
                      transport: {
                        shared_memory: {
                          enabled: true,
                          transport_optimization: {
                            enabled: true,
                            message_size_threshold: 1,
                          },
                        },
                      },
                    }
                "#
            }
            (Self::Udp, BenchEndpointRole::Tx) => {
                r#"
                    {
                      mode: "peer",
                      scouting: {
                        multicast: { enabled: false },
                      },
                      listen: { endpoints: [] },
                      connect: {
                        timeout_ms: -1,
                        exit_on_failure: false,
                        endpoints: ["udp/127.0.0.1:19001?rel=0"],
                      },
                    }
                "#
            }
            (Self::Udp, BenchEndpointRole::Rx) => {
                r#"
                    {
                      mode: "peer",
                      scouting: {
                        multicast: { enabled: false },
                      },
                      connect: { endpoints: [] },
                      listen: { endpoints: ["udp/127.0.0.1:19001?rel=0"] },
                    }
                "#
            }
            (Self::Tcp, BenchEndpointRole::Tx) => {
                r#"
                    {
                      mode: "peer",
                      scouting: {
                        multicast: { enabled: false },
                      },
                      listen: { endpoints: [] },
                      connect: {
                        timeout_ms: -1,
                        exit_on_failure: false,
                        endpoints: ["tcp/127.0.0.1:19000?rel=1"],
                      },
                    }
                "#
            }
            (Self::Tcp, BenchEndpointRole::Rx) => {
                r#"
                    {
                      mode: "peer",
                      scouting: {
                        multicast: { enabled: false },
                      },
                      connect: { endpoints: [] },
                      listen: { endpoints: ["tcp/127.0.0.1:19000?rel=1"] },
                    }
                "#
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct BenchEndpointSummary {
    pub rate_target_hz: Option<u64>,
    pub route: String,
    pub transport: BenchTransport,
    pub listen_endpoint: Option<&'static str>,
    pub connect_endpoint: Option<&'static str>,
}

pub fn bench_endpoint_summary(
    config: &CuConfig,
    role: BenchEndpointRole,
    transport: BenchTransport,
) -> CuResult<BenchEndpointSummary> {
    let rate_target_hz = config
        .runtime
        .as_ref()
        .and_then(|runtime| runtime.rate_target_hz);

    let bridge = config
        .bridges
        .iter()
        .find(|bridge| bridge.id == BRIDGE_ID)
        .ok_or_else(|| {
            CuError::from(format!("benchmark config is missing bridge '{BRIDGE_ID}'"))
        })?;

    let route = bridge
        .channels
        .iter()
        .find_map(|channel| match (role, channel) {
            (BenchEndpointRole::Tx, BridgeChannelConfigRepresentation::Tx { id, route, .. })
                if id == CHANNEL_ID =>
            {
                Some(route.clone().unwrap_or_else(|| LATENCY_ROUTE.to_string()))
            }
            (BenchEndpointRole::Rx, BridgeChannelConfigRepresentation::Rx { id, route, .. })
                if id == CHANNEL_ID =>
            {
                Some(route.clone().unwrap_or_else(|| LATENCY_ROUTE.to_string()))
            }
            _ => None,
        })
        .ok_or_else(|| {
            CuError::from(format!(
                "benchmark config is missing {:?} channel '{}'",
                role, CHANNEL_ID
            ))
        })?;

    Ok(BenchEndpointSummary {
        rate_target_hz,
        route,
        transport,
        listen_endpoint: transport.listen_endpoint(role),
        connect_endpoint: transport.connect_endpoint(role),
    })
}

impl BenchEndpointSummary {
    pub fn transport_details(&self) -> String {
        let mut details = format!("transport={}", self.transport.as_str());
        if let Some(endpoint) = self.listen_endpoint {
            details.push_str(" listen=");
            details.push_str(endpoint);
        }
        if let Some(endpoint) = self.connect_endpoint {
            details.push_str(" connect=");
            details.push_str(endpoint);
        }
        if self.transport.uses_shared_memory() {
            details.push_str(" shared_memory=enabled");
        }
        details
    }
}

pub fn load_bench_config(
    config_ron: &str,
    role: BenchEndpointRole,
    transport: BenchTransport,
) -> CuResult<CuConfig> {
    let mut config = CuConfig::deserialize_ron(config_ron)
        .map_err(|err| CuError::new_with_cause("failed to parse embedded benchmark config", err))?;

    let bridge = config
        .bridges
        .iter_mut()
        .find(|bridge| bridge.id == BRIDGE_ID)
        .ok_or_else(|| {
            CuError::from(format!("benchmark config is missing bridge '{BRIDGE_ID}'"))
        })?;

    let bridge_config = bridge.config.get_or_insert_with(ComponentConfig::default);
    bridge_config.set(
        "zenoh_config_json",
        transport.zenoh_config_json(role).to_string(),
    );

    Ok(config)
}

pub fn prepare_bench_transport(role: BenchEndpointRole, transport: BenchTransport) -> CuResult<()> {
    if transport != BenchTransport::Shmem || role != BenchEndpointRole::Rx {
        return Ok(());
    }

    let Some(endpoint) = transport.listen_endpoint(role) else {
        return Ok(());
    };
    let Some(socket_path) = endpoint.strip_prefix("unixsock-stream/") else {
        return Ok(());
    };
    let socket_path = socket_path
        .split_once('?')
        .map(|(path, _)| path)
        .unwrap_or(socket_path);

    let socket_path = Path::new(socket_path);
    if !socket_path.exists() {
        return Ok(());
    }

    fs::remove_file(socket_path).map_err(|err| {
        CuError::from(format!(
            "failed to remove stale shared-memory socket '{}'",
            socket_path.display()
        ))
        .add_cause(err.to_string().as_str())
    })?;

    Ok(())
}

pub fn default_log_path(file_name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join(file_name)
}

pub fn parse_run_options(default_log_file: &str) -> CuResult<BenchRunOptions> {
    let mut log_path = default_log_path(default_log_file);
    let mut instance_id = 0u32;
    let mut transport = BenchTransport::Shmem;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--log" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --log"))?;
                log_path = PathBuf::from(value);
            }
            "--instance-id" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --instance-id"))?;
                instance_id = value.parse::<u32>().map_err(|e| {
                    CuError::from(format!("invalid --instance-id '{value}'"))
                        .add_cause(e.to_string().as_str())
                })?;
            }
            "--transport" => {
                let value = args
                    .next()
                    .ok_or_else(|| CuError::from("missing value for --transport"))?;
                transport = BenchTransport::parse(&value)?;
            }
            other => {
                return Err(CuError::from(format!(
                    "unsupported argument '{other}', expected --log, --instance-id, or --transport"
                )));
            }
        }
    }

    if let Some(parent) = log_path.parent() {
        fs::create_dir_all(parent).map_err(|e| {
            CuError::from(format!(
                "failed to create log directory '{}'",
                parent.display()
            ))
            .add_cause(e.to_string().as_str())
        })?;
    }

    Ok(BenchRunOptions {
        log_path,
        instance_id,
        transport,
    })
}

pub mod bridges {
    use super::messages;
    use cu_zenoh_bridge::ZenohBridge;
    use cu29::prelude::*;

    tx_channels! {
        pub struct BenchTxChannels : BenchTxId {
            latency => messages::LatencySample = "benchmark/zenoh/latency",
        }
    }

    rx_channels! {
        pub struct BenchRxChannels : BenchRxId {
            latency => messages::LatencySample = "benchmark/zenoh/latency",
        }
    }

    pub type BenchZenohBridge = ZenohBridge<BenchTxChannels, BenchRxChannels>;
}

pub mod tasks {
    use super::LatencyProbe;
    use super::messages::LatencySample;
    use super::{wall_clock_latency, wall_clock_now_ns};
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct LatencySource {
        seq: u64,
    }

    impl Freezable for LatencySource {}

    impl CuSrcTask for LatencySource {
        type Output<'m> = output_msg!(LatencySample);
        type Resources<'r> = ();

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self { seq: 0 })
        }

        fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            output.set_payload(LatencySample {
                seq: self.seq,
                sent_at_wall_ns: wall_clock_now_ns(),
            });
            output.tov = Tov::Time(ctx.now());
            self.seq = self.seq.wrapping_add(1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct LatencySink {
        #[reflect(ignore)]
        probe: LatencyProbe,
    }

    impl Freezable for LatencySink {}

    impl CuSinkTask for LatencySink {
        type Input<'m> = input_msg!(LatencySample);
        type Resources<'r> = ();

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                probe: LatencyProbe::new(),
            })
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let Some(sample) = input.payload() else {
                return Ok(());
            };

            let now = _ctx.now();
            let latency = wall_clock_latency(sample.sent_at_wall_ns);
            self.probe.observe(now, sample.seq, latency);
            Ok(())
        }
    }
}
