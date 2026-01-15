use dora_node_api::arrow::array::UInt64Array;
use dora_node_api::dora_core::config::DataId;
use dora_node_api::{DoraNode, Event, EventStream, Metadata, MetadataParameters};
use eyre::{bail, ContextCompat, Result};
use std::collections::BTreeMap;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const CHAIN_LEN: usize = 8;
const TASK_COUNT: usize = 1 + CHAIN_LEN + CHAIN_LEN;
const TASK_ORDER: [&str; TASK_COUNT + 1] = [
    "src", "ct-0", "gpio-0", "ct-1", "gpio-1", "ct-2", "gpio-2", "ct-3", "gpio-3", "ct-4",
    "gpio-4", "ct-5", "gpio-5", "ct-6", "gpio-6", "ct-7", "gpio-7", "End2End",
];

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let role = Role::from_id(node.id().as_ref())?;
    match role {
        Role::Source => run_source(&mut node, &mut events),
        Role::Chain(index) => run_chain(&mut node, &mut events, index),
        Role::Gpio(index) => run_gpio(&mut node, &mut events, index),
        Role::Stats => run_stats(&mut events),
    }
}

enum Role {
    Source,
    Chain(usize),
    Gpio(usize),
    Stats,
}

impl Role {
    fn from_id(id: &str) -> Result<Self> {
        if id == "src" {
            return Ok(Self::Source);
        }
        if id == "stats" {
            return Ok(Self::Stats);
        }
        if let Some(rest) = id.strip_prefix("ct-") {
            let index: usize = rest.parse().map_err(|_| eyre::eyre!("invalid ct index"))?;
            if index < CHAIN_LEN {
                return Ok(Self::Chain(index));
            }
        }
        if let Some(rest) = id.strip_prefix("gpio-") {
            let index: usize = rest
                .parse()
                .map_err(|_| eyre::eyre!("invalid gpio index"))?;
            if index < CHAIN_LEN {
                return Ok(Self::Gpio(index));
            }
        }
        bail!("unknown node id: {id}");
    }
}

fn run_source(node: &mut DoraNode, events: &mut EventStream) -> Result<()> {
    let out_id = DataId::from("out".to_string());
    let stats_id = DataId::from("stats".to_string());
    let mut seq = 0u64;
    let mut state = false;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { metadata, .. } => {
                let recv_ns = now_ns();
                let sender_ns = metadata_ns(&metadata).unwrap_or(recv_ns);
                let hop_latency = recv_ns.saturating_sub(sender_ns);
                send_stat(node, &stats_id, hop_latency)?;

                state = !state;
                seq = seq.wrapping_add(1);
                let send_ns = now_ns();
                let msg = UInt64Array::from(vec![seq, state as u64, send_ns, send_ns]);

                node.send_output(out_id.clone(), MetadataParameters::default(), msg)?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}

fn run_chain(node: &mut DoraNode, events: &mut EventStream, index: usize) -> Result<()> {
    let chain_id = DataId::from("chain".to_string());
    let gpio_id = DataId::from("gpio".to_string());
    let stats_id = DataId::from("stats".to_string());
    let send_chain = index + 1 < CHAIN_LEN;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { data, .. } => {
                let (seq, state, origin_ns, last_send_ns) = decode_msg(&data)?;
                let hop_latency = now_ns().saturating_sub(last_send_ns);
                if send_chain {
                    let send_ns = now_ns();
                    let msg = UInt64Array::from(vec![seq, state, origin_ns, send_ns]);
                    node.send_output(chain_id.clone(), MetadataParameters::default(), msg)?;
                }

                let send_ns = now_ns();
                let msg = UInt64Array::from(vec![seq, state, origin_ns, send_ns]);
                node.send_output(gpio_id.clone(), MetadataParameters::default(), msg)?;

                send_stat(node, &stats_id, hop_latency)?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}

fn run_gpio(node: &mut DoraNode, events: &mut EventStream, index: usize) -> Result<()> {
    let stats_id = DataId::from("stats".to_string());
    let end2end_id = DataId::from("end2end".to_string());
    let record_end2end = index + 1 == CHAIN_LEN;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { data, .. } => {
                let (_, _, origin_ns, last_send_ns) = decode_msg(&data)?;
                let hop_latency = now_ns().saturating_sub(last_send_ns);

                if record_end2end {
                    let end_to_end = now_ns().saturating_sub(origin_ns);
                    send_stat(node, &end2end_id, end_to_end)?;
                }

                send_stat(node, &stats_id, hop_latency)?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}

fn run_stats(events: &mut EventStream) -> Result<()> {
    let mut stats = StatsStore::new(Duration::from_secs(1));

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => {
                let sample = decode_stat(&data)?;
                stats.record(id.as_ref(), sample);
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}

fn decode_msg(data: &dora_node_api::ArrowData) -> Result<(u64, u64, u64, u64)> {
    let array = data
        .as_any()
        .downcast_ref::<UInt64Array>()
        .context("expected UInt64Array for caterpillar message")?;
    if array.len() < 4 {
        bail!("caterpillar message requires 4 u64 values");
    }
    Ok((
        array.value(0),
        array.value(1),
        array.value(2),
        array.value(3),
    ))
}

fn decode_stat(data: &dora_node_api::ArrowData) -> Result<u64> {
    let array = data
        .as_any()
        .downcast_ref::<UInt64Array>()
        .context("expected UInt64Array for stats sample")?;
    if array.is_empty() {
        bail!("stats sample is empty");
    }
    Ok(array.value(0))
}

fn send_stat(node: &mut DoraNode, output_id: &DataId, sample_ns: u64) -> Result<()> {
    let payload = UInt64Array::from(vec![sample_ns]);
    node.send_output(output_id.clone(), MetadataParameters::default(), payload)?;
    Ok(())
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_else(|_| Duration::from_secs(0))
        .as_nanos() as u64
}

fn metadata_ns(metadata: &Metadata) -> Option<u64> {
    metadata
        .timestamp()
        .get_time()
        .to_system_time()
        .duration_since(UNIX_EPOCH)
        .ok()
        .map(|duration| duration.as_nanos() as u64)
}

struct StatsStore {
    stats: BTreeMap<String, Stat>,
    last_report: Instant,
    report_interval: Duration,
}

impl StatsStore {
    fn new(report_interval: Duration) -> Self {
        Self {
            stats: BTreeMap::new(),
            last_report: Instant::now(),
            report_interval,
        }
    }

    fn record(&mut self, name: &str, sample_ns: u64) {
        self.stats
            .entry(name.to_string())
            .or_insert_with(Stat::default)
            .record(sample_ns);

        if self.last_report.elapsed() >= self.report_interval {
            self.last_report = Instant::now();
            println!("\n{}", self.format_table());
        }
    }

    fn format_table(&self) -> String {
        let mut rows: Vec<[String; 7]> = Vec::new();
        rows.push([
            "Task".to_string(),
            "Min".to_string(),
            "Max".to_string(),
            "Mean".to_string(),
            "Stddev".to_string(),
            "Jitter".to_string(),
            "Max Jitter".to_string(),
        ]);

        for name in TASK_ORDER {
            if let Some(stat) = self.stats.get(name) {
                if stat.count > 0 {
                    let mean = stat.mean();
                    let stddev = stat.stddev();
                    let jitter_avg = stat.jitter_avg();
                    rows.push([
                        name.to_string(),
                        format_ns(stat.min_ns),
                        format_ns(stat.max_ns),
                        format_ns(mean),
                        format_ns(stddev),
                        format_ns(jitter_avg),
                        format_ns(stat.jitter_max),
                    ]);
                    continue;
                }
            }

            rows.push([
                name.to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
                "-".to_string(),
            ]);
        }

        let mut widths = [0usize; 7];
        for row in &rows {
            for (idx, cell) in row.iter().enumerate() {
                widths[idx] = widths[idx].max(cell.len());
            }
        }

        let mut out = String::new();
        for (row_idx, row) in rows.iter().enumerate() {
            let line = format!(
                "{:<w0$}  {:>w1$}  {:>w2$}  {:>w3$}  {:>w4$}  {:>w5$}  {:>w6$}",
                row[0],
                row[1],
                row[2],
                row[3],
                row[4],
                row[5],
                row[6],
                w0 = widths[0],
                w1 = widths[1],
                w2 = widths[2],
                w3 = widths[3],
                w4 = widths[4],
                w5 = widths[5],
                w6 = widths[6],
            );
            out.push_str(&line);
            if row_idx + 1 < rows.len() {
                out.push('\n');
            }
        }
        out
    }
}

#[derive(Default)]
struct Stat {
    count: u64,
    min_ns: u64,
    max_ns: u64,
    mean_ns: f64,
    m2: f64,
    last_ns: Option<u64>,
    jitter_sum: u128,
    jitter_max: u64,
    jitter_count: u64,
}

impl Stat {
    fn record(&mut self, sample_ns: u64) {
        self.count += 1;
        if self.count == 1 {
            self.min_ns = sample_ns;
            self.max_ns = sample_ns;
            self.mean_ns = sample_ns as f64;
            self.m2 = 0.0;
        } else {
            self.min_ns = self.min_ns.min(sample_ns);
            self.max_ns = self.max_ns.max(sample_ns);
            let delta = sample_ns as f64 - self.mean_ns;
            self.mean_ns += delta / self.count as f64;
            let delta2 = sample_ns as f64 - self.mean_ns;
            self.m2 += delta * delta2;
        }

        if let Some(prev) = self.last_ns {
            let jitter = sample_ns.abs_diff(prev);
            self.jitter_sum += jitter as u128;
            self.jitter_max = self.jitter_max.max(jitter);
            self.jitter_count += 1;
        }
        self.last_ns = Some(sample_ns);
    }

    fn mean(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            self.mean_ns.round() as u64
        }
    }

    fn stddev(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            (self.m2 / self.count as f64).sqrt().round() as u64
        }
    }

    fn jitter_avg(&self) -> u64 {
        if self.jitter_count == 0 {
            0
        } else {
            (self.jitter_sum / self.jitter_count as u128) as u64
        }
    }
}

fn format_ns(ns: u64) -> String {
    if ns >= 1_000_000 {
        format!("{:.3} ms", ns as f64 / 1_000_000.0)
    } else if ns >= 1_000 {
        format!("{:.3} us", ns as f64 / 1_000.0)
    } else {
        format!("{} ns", ns)
    }
}
