//! Benchmark KPIs from one recorded run.
//!
//! `kpi [log base]`, default `<crate>/logs/autoware.copper`. One typed walk over the
//! copperlists yields every KPI; `CuDurationStatistics` does the aggregation and
//! `cu29_export`'s `compute_logstats` writes the per-edge view next to the CSVs.
//!
//! Timings come from the recorded `process_time` and `tov`. A copperlist is one full
//! pass over the graph, so a sample and everything it triggers share one list and the
//! chain KPIs need no cross-list matching.

use cu_autoware::payload;
use cu29::prelude::*;
use cu29_export::copperlists_reader;
use cu29_export::logstats::{compute_logstats, write_logstats};
use std::collections::HashMap;
use std::fmt::Display;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

gen_cumsgs!("copperconfig.ron");

/// Reference-system deadlines. Copper never drops, so overload lands in these instead.
const RT0_DEADLINE_MS: u64 = 200;
const RT1_DEADLINE_MS: u64 = 200;
const RT2_DEADLINE_MS: u64 = 100;
/// behavior_planner's configured period.
const PLANNER_PERIOD_MS: u64 = 100;
/// `CuDurationStatistics` spreads its buckets linearly over one ceiling (1024 of them
/// under std), so every series gets a ceiling on the order of its own values rather than
/// one global ceiling that would quantize the small series away.
const CEILING_FACTOR: u64 = 4;

/// Fig. 4 cost model, in µs. Each entry is a callback the node can charge in one pass,
/// gated by the copperlist slot whose payload says the callback ran: a node's own slot
/// for its main callback, an input's slot for a cache callback. `id#1` is a second
/// output port. Summed over a pass, this is what the node's `process_time` should cost.
const NODES: &[(&str, &[(&str, f64)])] = &[
    ("front_lidar", &[("front_lidar", 100.0)]),
    ("rear_lidar", &[("rear_lidar", 100.0)]),
    ("point_cloud_map", &[("point_cloud_map", 100.0)]),
    ("visualizer", &[("visualizer", 100.0)]),
    ("lanelet2_map", &[("lanelet2_map", 100.0)]),
    (
        "euclidean_cluster_settings",
        &[("euclidean_cluster_settings", 100.0)],
    ),
    (
        "points_transformer_front",
        &[("points_transformer_front", 10100.0)],
    ),
    (
        "points_transformer_rear",
        &[("points_transformer_rear", 10100.0)],
    ),
    (
        "point_cloud_fusion",
        &[
            ("points_transformer_rear", 2100.0),
            ("point_cloud_fusion", 10005.0),
        ],
    ),
    (
        "voxel_grid_downsampler",
        &[("voxel_grid_downsampler", 10100.0)],
    ),
    ("ray_ground_filter", &[("ray_ground_filter", 10100.0)]),
    (
        "euclidean_cluster_detector",
        &[
            ("euclidean_cluster_detector", 10100.0),
            ("euclidean_cluster_detector#1", 10200.0),
        ],
    ),
    (
        "object_collision_estimator",
        &[("object_collision_estimator", 10100.0)],
    ),
    (
        "point_cloud_map_loader",
        &[("point_cloud_map_loader", 10100.0)],
    ),
    (
        "ndt_localizer",
        &[
            ("voxel_grid_downsampler", 2100.0),
            ("ndt_localizer", 10100.0),
        ],
    ),
    (
        "lanelet2_global_planner",
        &[
            ("ndt_localizer", 2100.0),
            ("lanelet2_global_planner", 10200.0),
        ],
    ),
    (
        "lanelet2_map_loader",
        &[
            ("lanelet2_global_planner", 2100.0),
            ("lanelet2_map_loader", 10200.0),
        ],
    ),
    ("parking_planner", &[("parking_planner", 10100.0)]),
    ("lane_planner", &[("lane_planner", 10100.0)]),
    (
        "behavior_planner",
        &[
            ("object_collision_estimator", 1.0),
            ("ndt_localizer", 1.0),
            ("lanelet2_global_planner", 1.0),
            ("lanelet2_map_loader", 1.0),
            ("parking_planner", 1.0),
            ("lane_planner", 1.0),
            ("behavior_planner", 10100.0),
        ],
    ),
    ("mpc_controller", &[("mpc_controller", 10100.0)]),
    (
        "vehicle_interface",
        &[("behavior_planner", 2100.0), ("vehicle_interface", 10100.0)],
    ),
    ("vehicle_dbw", &[("vehicle_interface", 1000.0)]),
    (
        "intersection_output",
        &[("euclidean_cluster_detector#1", 1000.0)],
    ),
];

/// Flat copperlist slot names, `id` then `id#1`... for the ports of a multi-port node.
fn slot_names() -> Vec<String> {
    let mut seen: HashMap<&str, usize> = HashMap::new();
    CuMsgs::get_all_task_ids()
        .iter()
        .map(|id| {
            let port = seen.entry(id).or_insert(0);
            let name = if *port == 0 {
                (*id).to_string()
            } else {
                format!("{id}#{port}")
            };
            *port += 1;
            name
        })
        .collect()
}

/// The modelled cost of one callback, in ns. The table is the single source of truth for
/// the RT1 composition as well as for the err% column.
fn charge_ns(node: &str, gate: &str) -> u64 {
    NODES
        .iter()
        .find(|(id, _)| *id == node)
        .and_then(|(_, charges)| charges.iter().find(|(slot, _)| *slot == gate))
        .map(|(_, us)| (us * 1e3) as u64)
        .unwrap_or_else(|| panic!("no modelled charge for {node} gated on {gate}"))
}

fn ceiling(charges: &[(&str, f64)]) -> CuDuration {
    let total: f64 = charges.iter().map(|(_, us)| us).sum();
    CuDuration((CEILING_FACTOR as f64 * total * 1e3) as u64)
}

fn tov_ns(tov: Tov) -> Option<u64> {
    match tov {
        Tov::Time(t) => Some(t.as_nanos()),
        _ => None,
    }
}

fn span(process_time: PartialCuTimeRange) -> Option<(u64, u64)> {
    let start: Option<CuTime> = process_time.start.into();
    let end: Option<CuTime> = process_time.end.into();
    match (start, end) {
        (Some(start), Some(end)) if end >= start => Some((start.as_nanos(), end.as_nanos())),
        _ => None,
    }
}

fn ms(d: CuDuration) -> f64 {
    d.as_nanos() as f64 / 1e6
}

/// `CuDurationStatistics` interpolates inside linear buckets, so a percentile can land
/// just past the largest sample of a tight distribution. A reported one never should.
fn pct(s: &CuDurationStatistics, percentile: f64) -> f64 {
    ms(s.percentile(percentile)).min(ms(s.max()))
}

/// Execution alignment: (matched, missing, duplicate, unmatched) for sensor samples
/// against the updates each should have produced exactly once.
fn alignment(samples: &[u64], updates: &HashMap<u64, u32>) -> (usize, usize, u32, usize) {
    let matched = samples
        .iter()
        .filter(|seq| updates.contains_key(seq))
        .count();
    let duplicate = updates.values().map(|count| count - 1).sum();
    (
        matched,
        samples.len() - matched,
        duplicate,
        updates.len().saturating_sub(matched),
    )
}

/// One latency series: the samples for the CSV and the statistics for the report.
struct Series {
    rows: String,
    stats: CuDurationStatistics,
}

impl Series {
    fn new(header: &str, deadline_ms: u64) -> Self {
        Self {
            rows: format!("{header}\n"),
            stats: CuDurationStatistics::new(CuDuration::from_millis(deadline_ms * CEILING_FACTOR)),
        }
    }

    /// `at_ns` is the sample's own time of validity, already relative to app start.
    fn record(&mut self, seq: u64, at_ns: u64, value_ns: u64) {
        let _ = writeln!(
            self.rows,
            "{seq},{:.3},{:.3}",
            at_ns as f64 / 1e6,
            value_ns as f64 / 1e6
        );
        self.stats.record(CuDuration(value_ns));
    }

    fn report(&self, name: &str) {
        let s = &self.stats;
        println!(
            "{name:<26}{:>6}{:>10.2}{:>10.2}{:>10.2}{:>10.2}",
            s.len(),
            ms(s.mean()),
            pct(s, 0.5),
            pct(s, 0.99),
            ms(s.max())
        );
    }
}

/// Everything one walk of the log accumulates.
struct Kpis {
    slot_of: HashMap<String, usize>,
    /// RT1 ends inside point_cloud_fusion's cache callback, whose end is not separately
    /// observable, so its modelled cost completes the chain.
    rt1_cache_ns: u64,
    hot_path: Series,
    rt1: Series,
    rt2: Series,
    planner_period: Series,
    measured: Vec<CuDurationStatistics>,
    modelled_us: Vec<f64>,
    culists: u64,
    late: u64,
    /// Firings whose `tov` or `process_time` was not usable: a timing gap, not a miss.
    timing_gaps: u64,
    planner_ticks: u64,
    front_seqs: Vec<u64>,
    estimator_updates: HashMap<u64, u32>,
    last_planner_tov: Option<u64>,
    span_ns: u64,
}

impl Kpis {
    fn new() -> Self {
        Self {
            slot_of: slot_names()
                .into_iter()
                .enumerate()
                .map(|(index, name)| (name, index))
                .collect(),
            rt1_cache_ns: charge_ns("point_cloud_fusion", "points_transformer_rear"),
            hot_path: Series::new("seq,t_ms,latency_ms", RT0_DEADLINE_MS),
            rt1: Series::new("seq,t_ms,latency_ms", RT1_DEADLINE_MS),
            rt2: Series::new("seq,t_ms,latency_ms", RT2_DEADLINE_MS),
            planner_period: Series::new("seq,t_ms,period_ms", PLANNER_PERIOD_MS),
            measured: NODES
                .iter()
                .map(|(_, charges)| CuDurationStatistics::new(ceiling(charges)))
                .collect(),
            modelled_us: vec![0.0; NODES.len()],
            culists: 0,
            late: 0,
            timing_gaps: 0,
            planner_ticks: 0,
            front_seqs: Vec::new(),
            estimator_updates: HashMap::new(),
            last_planner_tov: None,
            span_ns: 0,
        }
    }

    fn record_pass(&mut self, msgs: &CuMsgs) {
        let flat = msgs.cumsgs();
        let present: Vec<bool> = flat.iter().map(|msg| msg.payload().is_some()).collect();
        self.culists += 1;
        self.span_ns = self.span_ns.max(
            flat.iter()
                .filter_map(|msg| span(msg.metadata().process_time()))
                .map(|(_, end)| end)
                .max()
                .unwrap_or(0),
        );

        // (a) and (b): the update is a firing whether or not it is timeable, and the
        // estimator's output carries the front_lidar tov it came from.
        let estimator = msgs.get_object_collision_estimator_output();
        if let Some(sample) = estimator.payload() {
            *self.estimator_updates.entry(sample.seq).or_default() += 1;
            match (tov_ns(estimator.tov), span(estimator.metadata.process_time)) {
                (Some(at), Some((_, end))) => {
                    let latency = end.saturating_sub(at);
                    self.hot_path.record(sample.seq, at, latency);
                    if latency > RT0_DEADLINE_MS * 1_000_000 {
                        self.late += 1;
                    }
                }
                _ => self.timing_gaps += 1,
            }
        }
        if let Some(sample) = msgs.get_front_lidar_output().payload() {
            self.front_seqs.push(sample.seq);
        }

        // (d) RT1 ends at point_cloud_fusion's cache callback, which runs first in that
        // node's process span: the observable part runs to the span start and the
        // modelled cache cost completes it.
        let rear = msgs.get_points_transformer_rear_output();
        let fusion = msgs.get_point_cloud_fusion_output();
        if let (Some(sample), Some(at), Some((start, _))) = (
            rear.payload(),
            tov_ns(rear.tov),
            span(fusion.metadata.process_time),
        ) {
            self.rt1
                .record(sample.seq, at, start.saturating_sub(at) + self.rt1_cache_ns);
        }

        // (e) RT2 runs from the planner tick to the command sink, which does get a
        // copperlist slot and therefore a recorded process_time.
        let planner = msgs.get_behavior_planner_output();
        if let (Some(sample), Some(at)) = (planner.payload(), tov_ns(planner.tov)) {
            self.planner_ticks += 1;
            if let Some((_, end)) = span(msgs.get_vehicle_dbw_output().metadata.process_time) {
                self.rt2.record(sample.seq, at, end.saturating_sub(at));
            }
            if let Some(previous) = self.last_planner_tov {
                self.planner_period
                    .record(sample.seq, at, at.saturating_sub(previous));
            }
            self.last_planner_tov = Some(at);
        }

        // (f) per node, measured against the cost the model says this pass charged.
        for (index, (node, charges)) in NODES.iter().enumerate() {
            let cost: f64 = charges
                .iter()
                .filter(|(gate, _)| present[self.slot_of[*gate]])
                .map(|(_, us)| us)
                .sum();
            if cost == 0.0 {
                continue;
            }
            let Some((start, end)) = span(flat[self.slot_of[*node]].metadata().process_time())
            else {
                continue;
            };
            self.measured[index].record(CuDuration(end - start));
            self.modelled_us[index] += cost;
        }
    }

    /// (node, n, mean, p50, p99, target, err%) in ms, for the nodes that charged something.
    fn node_rows(&self) -> Vec<(&'static str, u64, f64, f64, f64, f64, f64)> {
        NODES
            .iter()
            .enumerate()
            .filter_map(|(index, (node, _))| {
                let s = &self.measured[index];
                if s.is_empty() {
                    return None;
                }
                let target = self.modelled_us[index] / s.len() as f64 / 1e3;
                let mean = ms(s.mean());
                Some((
                    *node,
                    s.len(),
                    mean,
                    pct(s, 0.5),
                    pct(s, 0.99),
                    target,
                    (mean - target) / target * 100.0,
                ))
            })
            .collect()
    }

    fn report(&self, log_base: &Path) {
        println!(
            "cu_autoware KPIs — {} copperlists over {:.1}s ({})",
            self.culists,
            self.span_ns as f64 / 1e9,
            log_base.display()
        );
        println!(
            "\nchain                          n      mean       p50       p99       max   (ms)"
        );
        self.hot_path.report("hot path RT0");
        self.rt1.report("RT1 rear -> fusion*");
        self.rt2.report("RT2 planner -> dbw");
        println!(
            "* RT1 is the rear edge tov to point_cloud_fusion's process start plus its \
             {:.1}ms cache cost, modelled from the cost table: the cache and the trigger \
             share one process span.",
            self.rt1_cache_ns as f64 / 1e6
        );

        let (matched, missing, duplicate, unmatched) =
            alignment(&self.front_seqs, &self.estimator_updates);
        println!(
            "\nalignment  front_lidar {} -> estimator updates {matched}, missing {missing}, duplicate {duplicate}, unmatched {unmatched}, timing gaps {}",
            self.front_seqs.len(),
            self.timing_gaps
        );
        println!(
            "late       {} of {} hot-path samples over {RT0_DEADLINE_MS}ms",
            self.late,
            self.hot_path.stats.len()
        );

        let period = &self.planner_period.stats;
        if period.is_empty() {
            println!(
                "planner    {} ticks, no period to measure",
                self.planner_ticks
            );
        } else {
            println!(
                "planner    {} ticks, period mean {:.2}ms vs {PLANNER_PERIOD_MS}ms, jitter mean {:.2}ms max {:.2}ms, drift {:+.1}ms",
                self.planner_ticks,
                ms(period.mean()),
                ms(period.jitter_mean()),
                ms(period.jitter_max()),
                (ms(period.mean()) - PLANNER_PERIOD_MS as f64) * period.len() as f64
            );
        }

        println!("\nnode                           n      mean       p99    target      err%");
        println!("(target is the per-pass mean of the gated cost model; err% compares means)");
        for (node, n, mean, _, p99, target, err) in self.node_rows() {
            println!("{node:<26}{n:>6}{mean:>10.3}{p99:>10.3}{target:>10.3}{err:>+10.1}");
        }
    }

    fn nodes_csv(&self) -> String {
        let mut csv = String::from("node,n,mean_ms,p50_ms,p99_ms,target_ms,err_pct\n");
        for (node, n, mean, p50, p99, target, err) in self.node_rows() {
            let _ = writeln!(
                csv,
                "{node},{n},{mean:.4},{p50:.4},{p99:.4},{target:.4},{err:.2}"
            );
        }
        csv
    }
}

fn fail(message: impl Display) -> ! {
    eprintln!("kpi: {message}");
    std::process::exit(1);
}

fn reader(log_base: &Path) -> CuResult<UnifiedLoggerIOReader> {
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|e| CuError::new_with_cause(&format!("{}", log_base.display()), e))?;
    let UnifiedLogger::Read(logger) = logger else {
        return Err(CuError::from(format!(
            "{}: opened for writing",
            log_base.display()
        )));
    };
    Ok(UnifiedLoggerIOReader::new(
        logger,
        UnifiedLogType::CopperList,
    ))
}

fn write(dir: &Path, name: &str, content: &str) {
    let path = dir.join(name);
    if let Err(e) = fs::write(&path, content) {
        fail(format!("{}: {e}", path.display()));
    }
    println!("  {}", path.display());
}

fn main() {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let log_base = std::env::args().nth(1).map_or_else(
        || crate_dir.join("logs").join("autoware.copper"),
        PathBuf::from,
    );

    let mut kpis = Kpis::new();
    let source = reader(&log_base).unwrap_or_else(|e| fail(e));
    for culist in copperlists_reader::<CuMsgs>(source) {
        kpis.record_pass(&culist.msgs);
    }
    if kpis.culists == 0 {
        fail(format!("{}: no copperlists recorded", log_base.display()));
    }

    kpis.report(&log_base);

    let out_dir = crate_dir.join("analysis").join("data");
    if let Err(e) = fs::create_dir_all(&out_dir) {
        fail(format!("{}: {e}", out_dir.display()));
    }
    println!("\nwrote");
    write(&out_dir, "hotpath.csv", &kpis.hot_path.rows);
    write(&out_dir, "rt1.csv", &kpis.rt1.rows);
    write(&out_dir, "rt2.csv", &kpis.rt2.rows);
    write(&out_dir, "bp_period.csv", &kpis.planner_period.rows);
    write(&out_dir, "nodes.csv", &kpis.nodes_csv());

    // The per-edge / all-passes view is already a core feature; don't reimplement it.
    let config_path = crate_dir.join("copperconfig.ron");
    let config = config_path
        .to_str()
        .ok_or_else(|| CuError::from("crate path is not UTF-8"))
        .and_then(read_configuration)
        .unwrap_or_else(|e| fail(e));
    let logstats = reader(&log_base)
        .and_then(|source| compute_logstats::<CuMsgs>(source, &config, None))
        .unwrap_or_else(|e| fail(e));
    let logstats_path = out_dir.join("logstats.json");
    if let Err(e) = write_logstats(&logstats, &logstats_path) {
        fail(e);
    }
    println!("  {}", logstats_path.display());
}

#[cfg(test)]
mod tests {
    use super::*;
    use payload::{RefLaneSample, RefSample};

    const MS: u64 = 1_000_000;

    fn sample(seq: u64) -> RefSample {
        RefSample {
            seq,
            ..Default::default()
        }
    }

    fn t(ns: u64) -> CuTime {
        CuDuration(ns).into()
    }

    /// A fired message: payload, time of validity, and the producer's process span.
    fn fired<P: CuMsgPayload>(msg: &mut CuMsg<P>, payload: P, tov: u64, start: u64, end: u64) {
        msg.set_payload(payload);
        msg.tov = Tov::Time(t(tov));
        timed(msg, start, end);
    }

    /// A pass a node ran but produced nothing: the runtime still stamps the span.
    fn timed<P: CuMsgPayload>(msg: &mut CuMsg<P>, start: u64, end: u64) {
        msg.metadata.process_time.start = t(start).into();
        msg.metadata.process_time.end = t(end).into();
    }

    fn node_index(id: &str) -> usize {
        NODES
            .iter()
            .position(|(node, _)| *node == id)
            .unwrap_or_else(|| panic!("{id} is not in the cost table"))
    }

    /// The pass where the lidar chain is fresh, times in ms. Tuple-field access needs
    /// literal indices; `test_culist_slot_layout` pins the ones used here. Tuple index n
    /// is flat slot n below the two-port node (17.0/17.1 = slots 17/18) and slot n+1 above.
    fn fresh_pass() -> CuMsgs {
        let mut msgs = CuMsgs::default();
        fired(
            &mut msgs.0.0,
            sample(7),
            1000 * MS,
            1000 * MS,
            1000 * MS + 100_000,
        );
        fired(&mut msgs.0.1, sample(7), 1000 * MS, 1000 * MS, 1010 * MS);
        fired(
            &mut msgs.0.2,
            sample(5),
            1002 * MS,
            1002 * MS,
            1002 * MS + 100_000,
        );
        fired(&mut msgs.0.3, sample(5), 1002 * MS, 1002 * MS, 1012 * MS);
        fired(&mut msgs.0.4, sample(7), 1000 * MS, 1012 * MS, 1024 * MS);
        fired(&mut msgs.0.17.0, sample(7), 1000 * MS, 1030 * MS, 1050 * MS);
        fired(
            &mut msgs.0.17.1,
            RefLaneSample(sample(7)),
            1000 * MS,
            1030 * MS,
            1050 * MS,
        );
        fired(&mut msgs.0.19, sample(7), 1000 * MS, 1140 * MS, 1150 * MS);
        fired(&mut msgs.0.20, sample(3), 1160 * MS, 1160 * MS, 1170 * MS);
        timed(&mut msgs.0.23, 1190 * MS, 1191 * MS);
        msgs
    }

    /// The pass where both lidars are gated off: only the settings lane and the planner
    /// fire, and the gated nodes still carry a span.
    fn gated_pass() -> CuMsgs {
        let mut msgs = CuMsgs::default();
        timed(&mut msgs.0.0, 1200 * MS, 1200 * MS);
        timed(&mut msgs.0.1, 1200 * MS, 1200 * MS);
        timed(&mut msgs.0.3, 1200 * MS, 1200 * MS);
        timed(&mut msgs.0.4, 1200 * MS, 1200 * MS);
        timed(&mut msgs.0.19, 1200 * MS, 1200 * MS);
        fired(
            &mut msgs.0.16,
            sample(40),
            1205 * MS,
            1205 * MS,
            1205 * MS + 100_000,
        );
        timed(&mut msgs.0.17.0, 1210 * MS, 1220 * MS);
        fired(
            &mut msgs.0.17.1,
            RefLaneSample(sample(40)),
            1205 * MS,
            1210 * MS,
            1220 * MS,
        );
        fired(&mut msgs.0.20, sample(4), 1330 * MS, 1330 * MS, 1340 * MS);
        timed(&mut msgs.0.23, 1360 * MS, 1361 * MS);
        msgs
    }

    /// The fixtures above address the copperlist by literal tuple index; if the graph
    /// ever renumbers its slots this is what catches it.
    #[test]
    fn test_culist_slot_layout() {
        let slots = slot_names();
        for (index, name) in [
            (0, "front_lidar"),
            (1, "points_transformer_front"),
            (2, "rear_lidar"),
            (3, "points_transformer_rear"),
            (4, "point_cloud_fusion"),
            (16, "euclidean_cluster_settings"),
            (17, "euclidean_cluster_detector"),
            (18, "euclidean_cluster_detector#1"),
            (20, "object_collision_estimator"),
            (21, "behavior_planner"),
            (24, "vehicle_dbw"),
        ] {
            assert_eq!(slots[index], name, "slot {index}");
        }
    }

    /// The cost model has to name real copperlist slots and cover every node, or a config
    /// edit silently moves the err% column off its subject.
    #[test]
    fn test_cost_model_matches_the_graph() {
        let slots = slot_names();
        for (node, charges) in NODES {
            assert!(slots.iter().any(|slot| slot == node), "no slot '{node}'");
            for (gate, _) in *charges {
                assert!(
                    slots.iter().any(|slot| slot == gate),
                    "{node}: unknown gate '{gate}'"
                );
            }
        }
        let mut covered: Vec<&str> = NODES.iter().map(|(node, _)| *node).collect();
        covered.sort_unstable();
        let mut ids = CuMsgs::get_all_task_ids().to_vec();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(covered, ids);
    }

    /// Every callback of Fig. 4, once: the 185.0ms pass PLAN.md quotes.
    #[test]
    fn test_cost_model_totals_the_fig4_pass() {
        let total: f64 = NODES
            .iter()
            .flat_map(|(_, charges)| charges.iter())
            .map(|(_, us)| us)
            .sum();
        assert!((total - 185_011.0).abs() < 1e-6, "{total}us");
    }

    #[test]
    fn test_fresh_pass_yields_every_chain() {
        let mut kpis = Kpis::new();
        kpis.record_pass(&fresh_pass());

        // estimator end 1150 - front_lidar tov 1000.
        assert_eq!(kpis.hot_path.stats.max(), CuDuration(150 * MS));
        // fusion process start 1012 - rear edge tov 1002, plus the 2.1ms cache cost.
        assert_eq!(kpis.rt1.stats.max(), CuDuration(10 * MS + 2_100_000));
        // dbw end 1191 - planner tov 1160.
        assert_eq!(kpis.rt2.stats.max(), CuDuration(31 * MS));
        assert_eq!(kpis.planner_ticks, 1);
        assert!(
            kpis.planner_period.stats.is_empty(),
            "no period on one tick"
        );
        assert_eq!(kpis.late, 0);
        assert_eq!(kpis.timing_gaps, 0);
        assert_eq!(
            alignment(&kpis.front_seqs, &kpis.estimator_updates),
            (1, 0, 0, 0)
        );
    }

    #[test]
    fn test_gated_pass_advances_only_what_fired() {
        let mut kpis = Kpis::new();
        kpis.record_pass(&fresh_pass());
        kpis.record_pass(&gated_pass());

        for node in [
            "front_lidar",
            "points_transformer_front",
            "points_transformer_rear",
            "point_cloud_fusion",
            "object_collision_estimator",
        ] {
            assert_eq!(
                kpis.measured[node_index(node)].len(),
                1,
                "{node} charged a gated pass"
            );
        }
        // Lane 1 of the intersection ran on both passes, lane 0 only on the fresh one.
        let detector = node_index("euclidean_cluster_detector");
        assert_eq!(kpis.measured[detector].len(), 2);
        assert!((kpis.modelled_us[detector] - (20_300.0 + 10_200.0)).abs() < 1e-6);

        assert_eq!(kpis.hot_path.stats.len(), 1);
        assert_eq!(kpis.rt1.stats.len(), 1);
        assert_eq!(kpis.rt2.stats.len(), 2);
        assert_eq!(kpis.planner_ticks, 2);
        // Planner tov 1160 then 1330.
        assert_eq!(kpis.planner_period.stats.max(), CuDuration(170 * MS));
        assert_eq!(
            alignment(&kpis.front_seqs, &kpis.estimator_updates),
            (1, 0, 0, 0)
        );
    }

    /// A firing with no usable timestamp is a timing gap, never a missing firing.
    #[test]
    fn test_untimed_firing_counts_as_a_gap_not_a_miss() {
        let mut msgs = fresh_pass();
        msgs.0.19.tov = Tov::None;
        let mut kpis = Kpis::new();
        kpis.record_pass(&msgs);

        assert_eq!(kpis.timing_gaps, 1);
        assert!(kpis.hot_path.stats.is_empty());
        assert_eq!(
            alignment(&kpis.front_seqs, &kpis.estimator_updates),
            (1, 0, 0, 0)
        );
    }

    #[test]
    fn test_alignment_counts_duplicates_and_strays() {
        let updates = HashMap::from([(1, 1), (2, 2), (9, 1)]);
        // Sample 3 never reached the estimator; 2 arrived twice; 9 has no sample.
        assert_eq!(alignment(&[1, 2, 3], &updates), (2, 1, 1, 1));
        assert_eq!(alignment(&[], &HashMap::new()), (0, 0, 0, 0));
    }
}
