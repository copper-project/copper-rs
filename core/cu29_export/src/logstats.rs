use crate::copperlists_reader;
use cu29::clock::{CuDuration, OptionCuTime};
use cu29::config::{CuConfig, CuGraph, Flavor};
use cu29::curuntime::{CuExecutionLoop, CuExecutionUnit, compute_runtime_plan};
use cu29::monitoring::CuDurationStatistics;
use cu29::prelude::{CopperListTuple, CuMsgMetadataTrait, CuPayloadRawBytes};
use cu29::{CuError, CuResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use std::path::Path;

const LOGSTATS_SCHEMA_VERSION: u32 = 1;
const MAX_LATENCY_NS: u64 = 10_000_000_000;

#[derive(Debug, Serialize, Deserialize)]
pub struct LogStats {
    pub schema_version: u32,
    pub config_signature: String,
    pub mission: Option<String>,
    pub edges: Vec<EdgeLogStats>,
    pub perf: PerfStats,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgeLogStats {
    pub src: String,
    pub dst: String,
    pub msg: String,
    pub samples: u64,
    pub none_samples: u64,
    pub valid_time_samples: u64,
    pub total_raw_bytes: u64,
    pub avg_raw_bytes: Option<f64>,
    pub rate_hz: Option<f64>,
    pub throughput_bytes_per_sec: Option<f64>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PerfStats {
    pub samples: u64,
    pub valid_time_samples: u64,
    pub end_to_end: DurationStats,
    pub jitter: DurationStats,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DurationStats {
    pub min_ns: Option<u64>,
    pub max_ns: Option<u64>,
    pub mean_ns: Option<f64>,
    pub stddev_ns: Option<f64>,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct EdgeKey {
    src: String,
    dst: String,
    msg: String,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct SrcMsgKey {
    src: String,
    msg: String,
}

#[derive(Clone, Debug)]
struct OutputSlot {
    edges: Vec<EdgeKey>,
}

#[derive(Debug, Default, Clone)]
struct EdgeAccumulator {
    samples: u64,
    none_samples: u64,
    valid_time_samples: u64,
    total_raw_bytes: u64,
    min_end_ns: Option<u64>,
    max_end_ns: Option<u64>,
}

impl EdgeAccumulator {
    fn record_sample(&mut self, payload_bytes: Option<u64>, end_time_ns: Option<u64>) {
        self.samples = self.samples.saturating_add(1);
        if let Some(bytes) = payload_bytes {
            self.total_raw_bytes = self.total_raw_bytes.saturating_add(bytes);
        } else {
            self.none_samples = self.none_samples.saturating_add(1);
        }

        if let Some(end_ns) = end_time_ns {
            self.valid_time_samples = self.valid_time_samples.saturating_add(1);
            self.min_end_ns = Some(self.min_end_ns.map_or(end_ns, |min| min.min(end_ns)));
            self.max_end_ns = Some(self.max_end_ns.map_or(end_ns, |max| max.max(end_ns)));
        }
    }

    fn finalize(self, key: EdgeKey) -> EdgeLogStats {
        let payload_samples = self.samples.saturating_sub(self.none_samples);
        let avg_raw_bytes = if payload_samples > 0 {
            Some(self.total_raw_bytes as f64 / payload_samples as f64)
        } else {
            None
        };

        let (rate_hz, throughput_bytes_per_sec) = if self.valid_time_samples >= 2 {
            match (self.min_end_ns, self.max_end_ns) {
                (Some(min_ns), Some(max_ns)) if max_ns > min_ns => {
                    let duration_ns = max_ns - min_ns;
                    let duration_secs = duration_ns as f64 / 1_000_000_000.0;
                    let intervals = (self.valid_time_samples - 1) as f64;
                    (
                        Some(intervals / duration_secs),
                        Some(self.total_raw_bytes as f64 / duration_secs),
                    )
                }
                _ => (None, None),
            }
        } else {
            (None, None)
        };

        EdgeLogStats {
            src: key.src,
            dst: key.dst,
            msg: key.msg,
            samples: self.samples,
            none_samples: self.none_samples,
            valid_time_samples: self.valid_time_samples,
            total_raw_bytes: self.total_raw_bytes,
            avg_raw_bytes,
            rate_hz,
            throughput_bytes_per_sec,
        }
    }
}

#[derive(Debug)]
struct PerfAccumulator {
    stats: CuDurationStatistics,
    samples: u64,
    valid_time_samples: u64,
}

impl PerfAccumulator {
    fn new() -> Self {
        Self {
            stats: CuDurationStatistics::new(CuDuration(MAX_LATENCY_NS)),
            samples: 0,
            valid_time_samples: 0,
        }
    }

    fn record_sample(&mut self, latency: Option<CuDuration>) {
        self.samples = self.samples.saturating_add(1);
        if let Some(latency) = latency {
            self.stats.record(latency);
            self.valid_time_samples = self.valid_time_samples.saturating_add(1);
        }
    }

    fn finalize(&self) -> PerfStats {
        let end_to_end = duration_stats_from(&self.stats);
        let jitter = if self.stats.len() >= 2 {
            jitter_stats_from(&self.stats)
        } else {
            DurationStats::default()
        };

        PerfStats {
            samples: self.samples,
            valid_time_samples: self.valid_time_samples,
            end_to_end,
            jitter,
        }
    }
}

pub fn compute_logstats<P>(
    mut reader: impl Read,
    config: &CuConfig,
    mission: Option<&str>,
) -> CuResult<LogStats>
where
    P: CopperListTuple + CuPayloadRawBytes,
{
    let signature = build_graph_signature(config, mission)?;
    let output_slots = build_output_slots(config, mission)?;
    let mut edge_accumulators = build_edge_accumulators(config, mission)?;
    let mut perf = PerfAccumulator::new();
    let mut warned_lengths = false;

    for culist in copperlists_reader::<P>(&mut reader) {
        let payload_sizes = culist.msgs.payload_raw_bytes();
        let cumsgs = culist.msgs.cumsgs();

        if !warned_lengths
            && (payload_sizes.len() != cumsgs.len() || payload_sizes.len() != output_slots.len())
        {
            eprintln!(
                "Warning: output mapping length mismatch (sizes={}, msgs={}, slots={})",
                payload_sizes.len(),
                cumsgs.len(),
                output_slots.len()
            );
            warned_lengths = true;
        }

        let count = payload_sizes
            .len()
            .min(cumsgs.len())
            .min(output_slots.len());

        for idx in 0..count {
            let slot = &output_slots[idx];
            if slot.edges.is_empty() {
                continue;
            }
            let payload_bytes = payload_sizes[idx];
            let end_time_ns = extract_end_time_ns(cumsgs[idx].metadata());
            for edge in &slot.edges {
                if let Some(acc) = edge_accumulators.get_mut(edge) {
                    acc.record_sample(payload_bytes, end_time_ns);
                }
            }
        }

        perf.record_sample(compute_end_to_end_latency(&cumsgs));
    }

    let edges = edge_accumulators
        .into_iter()
        .map(|(key, acc)| acc.finalize(key))
        .collect();

    Ok(LogStats {
        schema_version: LOGSTATS_SCHEMA_VERSION,
        config_signature: signature,
        mission: mission.map(|value| value.to_string()),
        edges,
        perf: perf.finalize(),
    })
}

pub fn write_logstats(stats: &LogStats, path: &Path) -> CuResult<()> {
    let file = File::create(path)
        .map_err(|e| CuError::new_with_cause("Failed to create logstats output", e))?;
    serde_json::to_writer_pretty(file, stats)
        .map_err(|e| CuError::new_with_cause("Failed to serialize logstats", e))?;
    Ok(())
}

fn build_output_slots(config: &CuConfig, mission: Option<&str>) -> CuResult<Vec<OutputSlot>> {
    let graph = config.get_graph(mission)?;
    let packs = collect_output_packs(graph)?;
    let edges_by_src = build_edges_by_src_msg(graph);
    let mut slots = Vec::new();

    for pack in packs {
        for msg in pack.msg_types {
            let edges = edges_by_src
                .get(&SrcMsgKey {
                    src: pack.src.clone(),
                    msg: msg.clone(),
                })
                .cloned()
                .unwrap_or_default();
            slots.push(OutputSlot { edges });
        }
    }

    Ok(slots)
}

fn build_edge_accumulators(
    config: &CuConfig,
    mission: Option<&str>,
) -> CuResult<HashMap<EdgeKey, EdgeAccumulator>> {
    let graph = config.get_graph(mission)?;
    let mut acc = HashMap::new();
    for cnx in graph.edges() {
        let key = EdgeKey {
            src: cnx.src.clone(),
            dst: cnx.dst.clone(),
            msg: cnx.msg.clone(),
        };
        acc.entry(key).or_default();
    }
    Ok(acc)
}

fn build_edges_by_src_msg(graph: &CuGraph) -> HashMap<SrcMsgKey, Vec<EdgeKey>> {
    let mut map: HashMap<SrcMsgKey, Vec<EdgeKey>> = HashMap::new();
    for cnx in graph.edges() {
        let key = SrcMsgKey {
            src: cnx.src.clone(),
            msg: cnx.msg.clone(),
        };
        let edge = EdgeKey {
            src: cnx.src.clone(),
            dst: cnx.dst.clone(),
            msg: cnx.msg.clone(),
        };
        map.entry(key).or_default().push(edge);
    }
    map
}

#[derive(Debug)]
struct OutputPackInfo {
    culist_index: u32,
    src: String,
    msg_types: Vec<String>,
}

fn collect_output_packs(graph: &CuGraph) -> CuResult<Vec<OutputPackInfo>> {
    let plan = compute_runtime_plan(graph)?;
    let mut packs = Vec::new();
    collect_output_packs_from_loop(&plan, graph, &mut packs)?;
    packs.sort_by_key(|pack| pack.culist_index);
    Ok(packs)
}

fn collect_output_packs_from_loop(
    loop_unit: &CuExecutionLoop,
    graph: &CuGraph,
    packs: &mut Vec<OutputPackInfo>,
) -> CuResult<()> {
    for step in &loop_unit.steps {
        match step {
            CuExecutionUnit::Step(step) => {
                if let Some(output_pack) = &step.output_msg_pack {
                    let node = graph
                        .get_node(step.node_id)
                        .ok_or_else(|| CuError::from("Missing node for output pack"))?;
                    packs.push(OutputPackInfo {
                        culist_index: output_pack.culist_index,
                        src: node.get_id(),
                        msg_types: output_pack.msg_types.clone(),
                    });
                }
            }
            CuExecutionUnit::Loop(inner) => {
                collect_output_packs_from_loop(inner, graph, packs)?;
            }
        }
    }
    Ok(())
}

fn compute_end_to_end_latency(
    msgs: &[&dyn cu29::prelude::ErasedCuStampedData],
) -> Option<CuDuration> {
    let start = msgs
        .first()
        .and_then(|msg| extract_start_time_ns(msg.metadata()))?;
    let end = msgs
        .last()
        .and_then(|msg| extract_end_time_ns(msg.metadata()))?;

    if end < start {
        return None;
    }
    Some(CuDuration::from_nanos(end - start))
}

fn extract_start_time_ns(meta: &dyn CuMsgMetadataTrait) -> Option<u64> {
    option_time_ns(meta.process_time().start)
}

fn extract_end_time_ns(meta: &dyn CuMsgMetadataTrait) -> Option<u64> {
    option_time_ns(meta.process_time().end)
}

fn option_time_ns(value: OptionCuTime) -> Option<u64> {
    Option::<CuDuration>::from(value).map(|t| t.as_nanos())
}

fn duration_stats_from(stats: &CuDurationStatistics) -> DurationStats {
    if stats.is_empty() {
        return DurationStats::default();
    }
    DurationStats {
        min_ns: Some(stats.min().as_nanos()),
        max_ns: Some(stats.max().as_nanos()),
        mean_ns: Some(stats.mean().as_nanos() as f64),
        stddev_ns: Some(stats.stddev().as_nanos() as f64),
    }
}

fn jitter_stats_from(stats: &CuDurationStatistics) -> DurationStats {
    DurationStats {
        min_ns: Some(stats.jitter_min().as_nanos()),
        max_ns: Some(stats.jitter_max().as_nanos()),
        mean_ns: Some(stats.jitter_mean().as_nanos() as f64),
        stddev_ns: Some(stats.jitter_stddev().as_nanos() as f64),
    }
}

fn build_graph_signature(config: &CuConfig, mission: Option<&str>) -> CuResult<String> {
    let graph = config.get_graph(mission)?;
    let mut parts = Vec::new();
    parts.push(format!("mission={}", mission.unwrap_or("default")));

    let mut nodes: Vec<_> = graph.get_all_nodes();
    nodes.sort_by(|a, b| a.1.get_id().cmp(&b.1.get_id()));
    for (_, node) in nodes {
        parts.push(format!(
            "node|{}|{}|{}",
            node.get_id(),
            node.get_type(),
            flavor_label(node.get_flavor())
        ));
    }

    let mut edges: Vec<String> = graph
        .edges()
        .map(|cnx| {
            format!(
                "edge|{}|{}|{}",
                format_endpoint(cnx.src.as_str(), cnx.src_channel.as_deref()),
                format_endpoint(cnx.dst.as_str(), cnx.dst_channel.as_deref()),
                cnx.msg
            )
        })
        .collect();
    edges.sort();
    parts.extend(edges);

    let joined = parts.join("\n");
    Ok(format!("fnv1a64:{:016x}", fnv1a64(joined.as_bytes())))
}

fn flavor_label(flavor: Flavor) -> &'static str {
    match flavor {
        Flavor::Task => "task",
        Flavor::Bridge => "bridge",
    }
}

fn format_endpoint(node: &str, channel: Option<&str>) -> String {
    match channel {
        Some(ch) => format!("{node}/{ch}"),
        None => node.to_string(),
    }
}

fn fnv1a64(data: &[u8]) -> u64 {
    const OFFSET_BASIS: u64 = 0xcbf29ce484222325;
    const PRIME: u64 = 0x100000001b3;
    let mut hash = OFFSET_BASIS;
    for byte in data {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge_key() -> EdgeKey {
        EdgeKey {
            src: "src".to_string(),
            dst: "dst".to_string(),
            msg: "Msg".to_string(),
        }
    }

    #[test]
    fn edge_stats_average_and_rate() {
        let mut acc = EdgeAccumulator::default();
        acc.record_sample(Some(100), Some(1_000_000_000));
        acc.record_sample(Some(300), Some(2_000_000_000));
        let stats = acc.finalize(edge_key());

        assert_eq!(stats.samples, 2);
        assert_eq!(stats.none_samples, 0);
        assert_eq!(stats.total_raw_bytes, 400);
        assert!((stats.avg_raw_bytes.unwrap() - 200.0).abs() < 1e-6);
        assert!((stats.rate_hz.unwrap() - 1.0).abs() < 1e-6);
        assert!((stats.throughput_bytes_per_sec.unwrap() - 400.0).abs() < 1e-6);
    }

    #[test]
    fn edge_stats_handles_missing_times() {
        let mut acc = EdgeAccumulator::default();
        acc.record_sample(Some(64), None);
        let stats = acc.finalize(edge_key());
        assert_eq!(stats.samples, 1);
        assert_eq!(stats.valid_time_samples, 0);
        assert!(stats.rate_hz.is_none());
        assert!(stats.throughput_bytes_per_sec.is_none());
    }

    #[test]
    fn perf_stats_skip_missing_latency() {
        let mut perf = PerfAccumulator::new();
        perf.record_sample(Some(CuDuration::from_nanos(1_000)));
        perf.record_sample(None);
        let stats = perf.finalize();

        assert_eq!(stats.samples, 2);
        assert_eq!(stats.valid_time_samples, 1);
        assert_eq!(stats.end_to_end.min_ns, Some(1_000));
        assert_eq!(stats.end_to_end.max_ns, Some(1_000));
        assert_eq!(stats.jitter.min_ns, None);
    }
}
