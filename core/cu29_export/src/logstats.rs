use crate::copperlists_reader;
use cu29::clock::{CuDuration, OptionCuTime};
use cu29::config::{CuConfig, CuGraph, DEFAULT_MISSION_ID, Flavor};
use cu29::curuntime::{CuExecutionUnit, CuStepPhase};
use cu29::monitoring::CuDurationStatistics;
use cu29::planner::{PlanEntityKind, assemble_runtime_plan_with};
use cu29::prelude::{CopperListTuple, CuMsgMetadataTrait, CuPayloadRawBytes};
use cu29::{CuError, CuResult};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::Read;
use std::path::Path;

const LOGSTATS_SCHEMA_VERSION: u32 = 2;
const MAX_LATENCY_NS: u64 = 10_000_000_000;
const MAX_QUANTILE_SAMPLES: usize = 2_048;
const MAX_REPRESENTATIVE_TRACES: usize = 256;
const MIN_CURRENT_CLUSTER_GAP_NS: u64 = 100_000;

#[derive(Debug, Serialize, Deserialize)]
pub struct LogStats {
    pub schema_version: u32,
    pub config_signature: String,
    pub mission: Option<String>,
    pub edges: Vec<EdgeLogStats>,
    pub perf: PerfStats,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schedule: Option<ScheduleLogStats>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ScheduleLogStats {
    pub stages: Vec<StageLogStats>,
    pub traces: Vec<ExecutionTrace>,
    pub residual_before: DurationStats,
    pub resource_overlaps: Vec<ResourceOverlapStats>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct StageLogStats {
    pub origin: String,
    pub samples: u64,
    pub durations: DurationStats,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExecutionTrace {
    pub kind: String,
    pub culist_id: u64,
    /// Sum of recorded process intervals in this CopperList.
    pub duration_ns: u64,
    /// Wall-clock span from the first through last retained interval.
    pub wall_span_ns: u64,
    /// Message slots carrying timestamps outside the current execution cluster.
    pub excluded_intervals: u32,
    pub residual_before_ns: Option<u64>,
    pub intervals: Vec<ExecutionInterval>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExecutionInterval {
    pub origin: String,
    pub start_ns: u64,
    pub end_ns: u64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ResourceOverlapStats {
    pub resource: String,
    pub left: String,
    pub right: String,
    pub occurrences: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgeLogStats {
    pub src: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub src_channel: Option<String>,
    pub dst: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dst_channel: Option<String>,
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub p50_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub p95_ns: Option<u64>,
}

#[derive(Debug, Default)]
struct TimingAccumulator {
    samples: u64,
    min: Option<u64>,
    max: Option<u64>,
    sum: f64,
    sum_squares: f64,
    quantiles: WeightedQuantiles,
}

impl TimingAccumulator {
    fn record(&mut self, value: u64) {
        self.samples = self.samples.saturating_add(1);
        self.min = Some(self.min.map_or(value, |current| current.min(value)));
        self.max = Some(self.max.map_or(value, |current| current.max(value)));
        let value_f64 = value as f64;
        self.sum += value_f64;
        self.sum_squares += value_f64 * value_f64;
        self.quantiles.record(value);
    }

    fn stats(&self) -> DurationStats {
        if self.samples == 0 {
            return DurationStats::default();
        }
        let mean = self.sum / self.samples as f64;
        let variance = (self.sum_squares / self.samples as f64 - mean * mean).max(0.0);
        DurationStats {
            min_ns: self.min,
            max_ns: self.max,
            mean_ns: Some(mean),
            stddev_ns: Some(variance.sqrt()),
            p50_ns: self.quantiles.quantile(0.50),
            p95_ns: self.quantiles.quantile(0.95),
        }
    }
}

#[derive(Debug, Default)]
struct WeightedQuantiles {
    samples: Vec<u64>,
    seen: u64,
}

impl WeightedQuantiles {
    fn record(&mut self, value: u64) {
        self.seen = self.seen.saturating_add(1);
        if self.samples.len() < MAX_QUANTILE_SAMPLES {
            self.samples.push(value);
            return;
        }
        let candidate = splitmix64(self.seen) % self.seen;
        if candidate < MAX_QUANTILE_SAMPLES as u64 {
            self.samples[candidate as usize] = value;
        }
    }

    fn quantile(&self, quantile: f64) -> Option<u64> {
        if self.samples.is_empty() {
            return None;
        }
        let mut values = self.samples.clone();
        values.sort_unstable();
        let index = ((values.len().saturating_sub(1)) as f64 * quantile).round() as usize;
        values.get(index).copied()
    }
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct EdgeKey {
    src: String,
    src_channel: Option<String>,
    dst: String,
    dst_channel: Option<String>,
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
            src_channel: key.src_channel,
            dst: key.dst,
            dst_channel: key.dst_channel,
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
        let jitter = jitter_stats_from(&self.stats);

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
    let graph = config.get_graph(mission)?;
    let signature = build_graph_signature(graph, mission);
    let output_slots = build_output_slots::<P>(config, graph, mission)?;
    let mut edge_accumulators = build_edge_accumulators(graph);
    let mut perf = PerfAccumulator::new();
    let resource_bindings = build_resource_bindings::<P>(config, graph);
    let mut stage_accumulators = BTreeMap::<String, TimingAccumulator>::new();
    let mut trace_duration_accumulator = TimingAccumulator::default();
    let mut residual_accumulator = TimingAccumulator::default();
    let mut representative_traces = Vec::<ExecutionTrace>::new();
    let mut worst_trace: Option<ExecutionTrace> = None;
    let mut active_intervals = Vec::<ExecutionInterval>::new();
    let mut overlap_counts = BTreeMap::<(String, String, String), u64>::new();
    let mut previous_end_ns = None;
    let mut trace_count = 0u64;
    let mut warned_lengths = false;

    for culist in copperlists_reader::<P>(&mut reader) {
        let payload_sizes = culist.msgs.payload_raw_bytes();
        let cumsgs = culist.msgs.cumsgs();

        let payload_len = payload_sizes.len();
        let msg_len = cumsgs.len();
        let slot_len = output_slots.len();
        if !warned_lengths && (payload_len != msg_len || payload_len != slot_len) {
            eprintln!(
                "Warning: output mapping length mismatch (sizes={}, msgs={}, slots={})",
                payload_len, msg_len, slot_len
            );
            warned_lengths = true;
        }

        let count = payload_len.min(msg_len).min(slot_len);

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

        if let Some(trace) =
            build_execution_trace(culist.id, &cumsgs, P::get_all_task_ids(), previous_end_ns)
        {
            for interval in &trace.intervals {
                stage_accumulators
                    .entry(interval.origin.clone())
                    .or_default()
                    .record(interval.end_ns - interval.start_ns);
            }
            trace_duration_accumulator.record(trace.duration_ns);
            if let Some(residual) = trace.residual_before_ns {
                residual_accumulator.record(residual);
            }
            record_resource_overlaps(
                &trace,
                &resource_bindings,
                &mut active_intervals,
                &mut overlap_counts,
            );
            previous_end_ns = trace.intervals.iter().map(|interval| interval.end_ns).max();
            trace_count = trace_count.saturating_add(1);
            sample_trace(&mut representative_traces, trace.clone(), trace_count);
            if worst_trace
                .as_ref()
                .is_none_or(|worst| trace.duration_ns > worst.duration_ns)
            {
                worst_trace = Some(trace);
            }
        }
    }

    let edges = edge_accumulators
        .into_iter()
        .map(|(key, acc)| acc.finalize(key))
        .collect();

    let duration_p50 = trace_duration_accumulator
        .stats()
        .p50_ns
        .unwrap_or_default();
    let typical_trace = representative_traces
        .into_iter()
        .min_by_key(|trace| trace.duration_ns.abs_diff(duration_p50));
    let mut traces = Vec::new();
    if let Some(mut typical) = typical_trace {
        typical.kind = "typical".to_string();
        traces.push(typical);
    }
    if let Some(mut worst) = worst_trace
        && traces
            .first()
            .is_none_or(|typical| typical.culist_id != worst.culist_id)
    {
        worst.kind = "slowest".to_string();
        traces.push(worst);
    }

    let stages = stage_accumulators
        .into_iter()
        .map(|(origin, accumulator)| StageLogStats {
            origin,
            samples: accumulator.samples,
            durations: accumulator.stats(),
        })
        .collect();
    let resource_overlaps = overlap_counts
        .into_iter()
        .map(
            |((resource, left, right), occurrences)| ResourceOverlapStats {
                resource,
                left,
                right,
                occurrences,
            },
        )
        .collect();

    Ok(LogStats {
        schema_version: LOGSTATS_SCHEMA_VERSION,
        config_signature: signature,
        mission: mission.map(|value| value.to_string()),
        edges,
        perf: perf.finalize(),
        schedule: Some(ScheduleLogStats {
            stages,
            traces,
            residual_before: residual_accumulator.stats(),
            resource_overlaps,
        }),
    })
}

fn build_execution_trace(
    culist_id: u64,
    msgs: &[&dyn cu29::prelude::ErasedCuStampedData],
    origins: &[&str],
    previous_end_ns: Option<u64>,
) -> Option<ExecutionTrace> {
    let mut grouped = BTreeMap::<String, (u64, u64)>::new();
    for (msg, origin) in msgs.iter().zip(origins.iter()) {
        let (Some(start), Some(end)) = (
            extract_start_time_ns(msg.metadata()),
            extract_end_time_ns(msg.metadata()),
        ) else {
            continue;
        };
        if end < start {
            continue;
        }
        grouped
            .entry((*origin).to_string())
            .and_modify(|range| {
                range.0 = range.0.min(start);
                range.1 = range.1.max(end);
            })
            .or_insert((start, end));
    }
    if grouped.is_empty() {
        return None;
    }
    let mut intervals = grouped
        .into_iter()
        .map(|(origin, (start_ns, end_ns))| ExecutionInterval {
            origin,
            start_ns,
            end_ns,
        })
        .collect::<Vec<_>>();
    intervals.sort_by_key(|interval| (interval.start_ns, interval.end_ns));
    let original_count = intervals.len();
    let intervals = select_current_execution_cluster(intervals);
    let start_ns = intervals.first()?.start_ns;
    let end_ns = intervals.iter().map(|interval| interval.end_ns).max()?;
    let duration_ns = intervals
        .iter()
        .map(|interval| interval.end_ns.saturating_sub(interval.start_ns))
        .sum();
    Some(ExecutionTrace {
        kind: String::new(),
        culist_id,
        duration_ns,
        wall_span_ns: end_ns.saturating_sub(start_ns),
        excluded_intervals: original_count.saturating_sub(intervals.len()) as u32,
        residual_before_ns: previous_end_ns.and_then(|end| start_ns.checked_sub(end)),
        intervals,
    })
}

fn select_current_execution_cluster(intervals: Vec<ExecutionInterval>) -> Vec<ExecutionInterval> {
    if intervals.len() < 2 {
        return intervals;
    }
    let total_process_ns: u64 = intervals
        .iter()
        .map(|interval| interval.end_ns.saturating_sub(interval.start_ns))
        .sum();
    let split_gap_ns = total_process_ns
        .saturating_mul(4)
        .max(MIN_CURRENT_CLUSTER_GAP_NS);
    let mut ranges = Vec::new();
    let mut cluster_start = 0;
    for index in 1..intervals.len() {
        let gap = intervals[index]
            .start_ns
            .saturating_sub(intervals[index - 1].end_ns);
        if gap > split_gap_ns {
            ranges.push(cluster_start..index);
            cluster_start = index;
        }
    }
    ranges.push(cluster_start..intervals.len());
    let selected = ranges
        .into_iter()
        .max_by_key(|range| (range.len(), intervals[range.end - 1].end_ns))
        .unwrap_or(0..intervals.len());
    intervals[selected].to_vec()
}

fn sample_trace(samples: &mut Vec<ExecutionTrace>, trace: ExecutionTrace, seen: u64) {
    if samples.len() < MAX_REPRESENTATIVE_TRACES {
        samples.push(trace);
        return;
    }
    let hash = splitmix64(trace.culist_id);
    let candidate = hash % seen;
    if candidate < MAX_REPRESENTATIVE_TRACES as u64 {
        samples[candidate as usize] = trace;
    }
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e3779b97f4a7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}

fn build_resource_bindings<P: CopperListTuple>(
    config: &CuConfig,
    graph: &CuGraph,
) -> HashMap<String, Vec<String>> {
    let mut bindings = HashMap::new();
    for origin in P::get_all_task_ids() {
        if bindings.contains_key(*origin) {
            continue;
        }
        let mut targets: Vec<String> = if let Some(rest) = origin.strip_prefix("bridge::") {
            let bridge_id = rest.split("::").next().unwrap_or_default();
            config
                .bridges
                .iter()
                .find(|bridge| bridge.id == bridge_id)
                .and_then(|bridge| bridge.resources.as_ref())
                .map(|resources| resources.values().cloned().collect())
                .unwrap_or_default()
        } else {
            graph
                .get_node_id_by_name(origin)
                .and_then(|node_id| graph.get_node(node_id))
                .and_then(|node| node.get_resources())
                .map(|resources| resources.values().cloned().collect())
                .unwrap_or_default()
        };
        targets.sort();
        targets.dedup();
        bindings.insert((*origin).to_string(), targets);
    }
    bindings
}

fn record_resource_overlaps(
    trace: &ExecutionTrace,
    bindings: &HashMap<String, Vec<String>>,
    active: &mut Vec<ExecutionInterval>,
    counts: &mut BTreeMap<(String, String, String), u64>,
) {
    let Some(trace_start) = trace
        .intervals
        .iter()
        .map(|interval| interval.start_ns)
        .min()
    else {
        return;
    };
    active.retain(|interval| interval.end_ns > trace_start);

    for (index, interval) in trace.intervals.iter().enumerate() {
        for other in active.iter().chain(trace.intervals[..index].iter()) {
            if interval.origin == other.origin
                || interval.start_ns >= other.end_ns
                || other.start_ns >= interval.end_ns
            {
                continue;
            }
            let Some(left_resources) = bindings.get(&interval.origin) else {
                continue;
            };
            let Some(right_resources) = bindings.get(&other.origin) else {
                continue;
            };
            for resource in left_resources {
                if right_resources.contains(resource) {
                    let (left, right) = if interval.origin <= other.origin {
                        (interval.origin.clone(), other.origin.clone())
                    } else {
                        (other.origin.clone(), interval.origin.clone())
                    };
                    *counts.entry((resource.clone(), left, right)).or_default() += 1;
                }
            }
        }
    }
    active.extend(trace.intervals.iter().cloned());
}

pub fn write_logstats(stats: &LogStats, path: &Path) -> CuResult<()> {
    let file = File::create(path)
        .map_err(|e| CuError::new_with_cause("Failed to create logstats output", e))?;
    serde_json::to_writer_pretty(file, stats)
        .map_err(|e| CuError::new_with_cause("Failed to serialize logstats", e))?;
    Ok(())
}

fn build_output_slots<P: CopperListTuple>(
    config: &CuConfig,
    graph: &CuGraph,
    mission: Option<&str>,
) -> CuResult<Vec<OutputSlot>> {
    let specs = P::get_output_specs();
    if specs.is_empty() {
        return build_output_slots_from_plan(config, graph, mission);
    }
    Ok(specs
        .iter()
        .map(|spec| OutputSlot {
            edges: graph
                .edges()
                .filter(|edge| edge.msg == spec.msg_type && edge_matches_origin(edge, spec.task_id))
                .map(edge_key_from_connection)
                .collect(),
        })
        .collect())
}

fn edge_matches_origin(edge: &cu29::config::Cnx, origin: &str) -> bool {
    if edge.src == origin {
        return true;
    }
    let Some(rest) = origin.strip_prefix("bridge::") else {
        return false;
    };
    let mut parts = rest.split("::");
    let (Some(bridge), Some(direction), Some(channel), None) =
        (parts.next(), parts.next(), parts.next(), parts.next())
    else {
        return false;
    };
    direction == "rx" && edge.src == bridge && edge.src_channel.as_deref() == Some(channel)
}

fn edge_key_from_connection(cnx: &cu29::config::Cnx) -> EdgeKey {
    EdgeKey {
        src: cnx.src.clone(),
        src_channel: cnx.src_channel.clone(),
        dst: cnx.dst.clone(),
        dst_channel: cnx.dst_channel.clone(),
        msg: cnx.msg.clone(),
    }
}

fn build_output_slots_from_plan(
    config: &CuConfig,
    graph: &CuGraph,
    mission: Option<&str>,
) -> CuResult<Vec<OutputSlot>> {
    // Share the generated-runtime construction path (bridge stages, slot
    // indices) so this cannot drift from the compiled plan; honor the mission's
    // configured plan heuristic.
    let mission = mission.unwrap_or(DEFAULT_MISSION_ID);
    let heuristic = config.plan_heuristic_for(mission);
    let plan = assemble_runtime_plan_with(config, graph, &heuristic, mission)?;

    let mut packs: Vec<(u32, String, Vec<String>)> = Vec::new();
    for unit in &plan.execution.steps {
        let CuExecutionUnit::Step(step) = unit else {
            continue;
        };
        // Anytime refine steps reuse their base step's culist slot; counting
        // them would emit duplicate slots and misalign every later slot.
        if step.phase == CuStepPhase::AnytimeRefine {
            continue;
        }
        let Some(output_pack) = &step.output_msg_pack else {
            continue;
        };
        let entity = &plan.entities[step.node_id as usize];
        let origin = match entity.kind {
            PlanEntityKind::Task { .. } => entity.label.clone(),
            PlanEntityKind::BridgeRx { .. } | PlanEntityKind::BridgeTx { .. } => {
                format!("bridge::{}", entity.label)
            }
        };
        packs.push((
            output_pack.culist_index,
            origin,
            output_pack.msg_types.clone(),
        ));
    }

    packs.sort_by_key(|(culist_index, _, _)| *culist_index);
    Ok(packs
        .into_iter()
        .flat_map(|(_, origin, msg_types)| {
            msg_types.into_iter().map(move |msg| OutputSlot {
                edges: graph
                    .edges()
                    .filter(|edge| edge.msg == msg && edge_matches_origin(edge, &origin))
                    .map(edge_key_from_connection)
                    .collect(),
            })
        })
        .collect())
}

fn build_edge_accumulators(graph: &CuGraph) -> HashMap<EdgeKey, EdgeAccumulator> {
    let mut acc = HashMap::new();
    for cnx in graph.edges() {
        let key = EdgeKey {
            src: cnx.src.clone(),
            src_channel: cnx.src_channel.clone(),
            dst: cnx.dst.clone(),
            dst_channel: cnx.dst_channel.clone(),
            msg: cnx.msg.clone(),
        };
        acc.entry(key).or_default();
    }
    acc
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
    end.checked_sub(start).map(CuDuration::from_nanos)
}

fn extract_start_time_ns(meta: &dyn CuMsgMetadataTrait) -> Option<u64> {
    option_time_ns(meta.process_time().start)
}

fn extract_end_time_ns(meta: &dyn CuMsgMetadataTrait) -> Option<u64> {
    option_time_ns(meta.process_time().end)
}

fn option_time_ns(value: OptionCuTime) -> Option<u64> {
    Option::<cu29::clock::CuTime>::from(value).map(|t| t.as_nanos())
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
        p50_ns: None,
        p95_ns: None,
    }
}

fn jitter_stats_from(stats: &CuDurationStatistics) -> DurationStats {
    if stats.len() < 2 {
        return DurationStats::default();
    }
    DurationStats {
        min_ns: Some(stats.jitter_min().as_nanos()),
        max_ns: Some(stats.jitter_max().as_nanos()),
        mean_ns: Some(stats.jitter_mean().as_nanos() as f64),
        stddev_ns: Some(stats.jitter_stddev().as_nanos() as f64),
        p50_ns: None,
        p95_ns: None,
    }
}

fn build_graph_signature(graph: &CuGraph, mission: Option<&str>) -> String {
    let mut parts = Vec::new();
    parts.push(format!("mission={}", mission.unwrap_or("default")));

    let mut nodes: Vec<_> = graph.get_all_nodes();
    nodes.sort_by_key(|a| a.1.get_id());
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
    format!("fnv1a64:{:016x}", fnv1a64(joined.as_bytes()))
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

    #[test]
    fn plan_output_slots_skip_anytime_refine_duplicates() {
        // A foreground anytime task (max_refines: 2) expands into one base step
        // plus refine steps that reuse the base's culist slot.
        let config = CuConfig::deserialize_ron(
            r#"(
                tasks: [
                    (id: "src", type: "demo::Src"),
                    (id: "any", type: "demo::Any", anytime: (max_refines: 2)),
                    (id: "sink", type: "demo::Sink"),
                ],
                cnx: [
                    (src: "src", dst: "any", msg: "u32"),
                    (src: "any", dst: "sink", msg: "u32"),
                ],
            )"#,
        )
        .expect("valid anytime config");
        let graph = config.get_graph(None).unwrap();

        let plan = assemble_runtime_plan_with(
            &config,
            graph,
            &config.plan_heuristic_for(DEFAULT_MISSION_ID),
            DEFAULT_MISSION_ID,
        )
        .unwrap();
        // The regression can only trigger if refine steps are actually present.
        assert!(
            plan.execution.steps.iter().any(|unit| matches!(
                unit,
                CuExecutionUnit::Step(step) if step.phase == CuStepPhase::AnytimeRefine
            )),
            "config must exercise anytime refine steps"
        );
        // One slot per emitted culist slot: sum of msg types over non-refine
        // steps, and each culist index appears exactly once.
        let mut expected_indices = Vec::new();
        let mut expected_slots = 0usize;
        for unit in &plan.execution.steps {
            if let CuExecutionUnit::Step(step) = unit
                && step.phase != CuStepPhase::AnytimeRefine
                && let Some(pack) = &step.output_msg_pack
            {
                expected_indices.push(pack.culist_index);
                expected_slots += pack.msg_types.len();
            }
        }
        let mut deduped = expected_indices.clone();
        deduped.sort_unstable();
        deduped.dedup();
        assert_eq!(deduped.len(), expected_indices.len(), "culist indices dup");

        let slots = build_output_slots_from_plan(&config, graph, None).unwrap();
        assert_eq!(slots.len(), expected_slots);
    }

    fn edge_key() -> EdgeKey {
        EdgeKey {
            src: "src".to_string(),
            src_channel: None,
            dst: "dst".to_string(),
            dst_channel: None,
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

    #[test]
    fn timing_accumulator_reports_bounded_quantiles() {
        let mut timings = TimingAccumulator::default();
        for value in 1..=10_000 {
            timings.record(value);
        }
        let stats = timings.stats();
        assert_eq!(stats.min_ns, Some(1));
        assert_eq!(stats.max_ns, Some(10_000));
        assert!(stats.p50_ns.unwrap().abs_diff(5_000) < 250);
        assert!(stats.p95_ns.unwrap().abs_diff(9_500) < 250);
        assert!(timings.quantiles.samples.len() <= MAX_QUANTILE_SAMPLES);
    }

    #[test]
    fn resource_overlap_requires_time_and_declared_target_overlap() {
        let trace = ExecutionTrace {
            kind: String::new(),
            culist_id: 1,
            duration_ns: 30,
            wall_span_ns: 30,
            excluded_intervals: 0,
            residual_before_ns: None,
            intervals: vec![
                ExecutionInterval {
                    origin: "left".to_string(),
                    start_ns: 10,
                    end_ns: 30,
                },
                ExecutionInterval {
                    origin: "right".to_string(),
                    start_ns: 20,
                    end_ns: 40,
                },
            ],
        };
        let bindings = HashMap::from([
            ("left".to_string(), vec!["gpu0".to_string()]),
            ("right".to_string(), vec!["gpu0".to_string()]),
        ]);
        let mut active = Vec::new();
        let mut counts = BTreeMap::new();
        record_resource_overlaps(&trace, &bindings, &mut active, &mut counts);
        assert_eq!(
            counts.get(&("gpu0".to_string(), "left".to_string(), "right".to_string())),
            Some(&1)
        );
    }

    #[test]
    fn current_execution_cluster_excludes_carried_forward_slots() {
        let intervals = vec![
            ExecutionInterval {
                origin: "stale_bridge".to_string(),
                start_ns: 100,
                end_ns: 200,
            },
            ExecutionInterval {
                origin: "source".to_string(),
                start_ns: 10_000_000,
                end_ns: 10_000_500,
            },
            ExecutionInterval {
                origin: "sink".to_string(),
                start_ns: 10_000_600,
                end_ns: 10_001_000,
            },
        ];
        let selected = select_current_execution_cluster(intervals);
        assert_eq!(selected.len(), 2);
        assert_eq!(selected[0].origin, "source");
        assert_eq!(selected[1].origin, "sink");
    }
}
