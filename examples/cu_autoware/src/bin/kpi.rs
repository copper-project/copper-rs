//! Benchmark KPIs from one recorded run.
//!
//! `kpi [log base] [out dir]`, defaults `<crate>/logs/autoware.copper` and
//! `<crate>/analysis/data`; `KPI_EXPECT=<n>` also asserts the copperlist count. One typed
//! walk over the copperlists yields every KPI; `CuDurationStatistics` does the aggregation
//! and `cu29_export`'s `compute_logstats` writes the per-edge view next to the CSVs.
//!
//! Timings come from the recorded `process_time` and `tov`. A copperlist is one full
//! pass over the graph, so a sample and everything it triggers share one list and the
//! chain KPIs need no cross-list matching.

use cu_autoware::payload;
use cu29::prelude::*;
use cu29_export::logstats::{compute_logstats, write_logstats};
use cu29_export::{copperlists_reader, runtime_lifecycle_reader};
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
/// The RT1/RT2 chains run in tens of ms, so sizing their buckets off their 200/100ms
/// deadlines would collapse the whole distribution into one bucket. Overshoot lands in
/// the top bucket and `max()` stays exact.
const RT1_CEILING_MS: u64 = 64;
const RT2_CEILING_MS: u64 = 128;

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

/// One latency series: the samples for the CSV, the statistics for the report, and the
/// deadline the series is judged against.
struct Series {
    rows: String,
    stats: CuDurationStatistics,
    deadline_ns: u64,
    late: u64,
}

impl Series {
    fn new(header: &str, ceiling_ms: u64, deadline_ms: u64) -> Self {
        Self {
            rows: format!("{header}\n"),
            stats: CuDurationStatistics::new(CuDuration::from_millis(ceiling_ms)),
            deadline_ns: deadline_ms * 1_000_000,
            late: 0,
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
        if value_ns > self.deadline_ns {
            self.late += 1;
        }
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
            hot_path: Series::new(
                "seq,t_ms,latency_ms",
                RT0_DEADLINE_MS * CEILING_FACTOR,
                RT0_DEADLINE_MS,
            ),
            rt1: Series::new("seq,t_ms,latency_ms", RT1_CEILING_MS, RT1_DEADLINE_MS),
            rt2: Series::new("seq,t_ms,latency_ms", RT2_CEILING_MS, RT2_DEADLINE_MS),
            planner_period: Series::new(
                "seq,t_ms,period_ms",
                PLANNER_PERIOD_MS * CEILING_FACTOR,
                PLANNER_PERIOD_MS,
            ),
            measured: NODES
                .iter()
                .map(|(_, charges)| CuDurationStatistics::new(ceiling(charges)))
                .collect(),
            modelled_us: vec![0.0; NODES.len()],
            culists: 0,
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
                    self.hot_path.record(sample.seq, at, end.saturating_sub(at));
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

    fn report(&self, log_base: &Path, ids: (u64, u64)) {
        println!(
            "cu_autoware KPIs — {} copperlists (ids {}..={}) over {:.1}s ({})",
            self.culists,
            ids.0,
            ids.1,
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
            "late       RT0 {}/{} over {RT0_DEADLINE_MS}ms, RT1 {}/{} over {RT1_DEADLINE_MS}ms, RT2 {}/{} over {RT2_DEADLINE_MS}ms",
            self.hot_path.late,
            self.hot_path.stats.len(),
            self.rt1.late,
            self.rt1.stats.len(),
            self.rt2.late,
            self.rt2.stats.len()
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

fn reader(log_base: &Path, section: UnifiedLogType) -> CuResult<UnifiedLoggerIOReader> {
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
    Ok(UnifiedLoggerIOReader::new(logger, section))
}

/// The run's own witnesses that the log is whole and belongs to this config: the RON the
/// app was built from, and the shutdown record `cu_autoware::run` writes last. Without
/// them a truncated log reports on its prefix and a stale one reports on the wrong graph.
fn provenance(log_base: &Path) -> CuResult<(String, bool)> {
    let source = reader(log_base, UnifiedLogType::RuntimeLifecycle)?;
    let (mut config, mut complete) = (None, false);
    for record in runtime_lifecycle_reader(source) {
        match record.event {
            RuntimeLifecycleEvent::Instantiated {
                effective_config_ron,
                ..
            } => config = Some(effective_config_ron),
            RuntimeLifecycleEvent::ShutdownCompleted => complete = true,
            _ => {}
        }
    }
    config
        .map(|config| (config, complete))
        .ok_or_else(|| CuError::from("no Instantiated record in the runtime lifecycle section"))
}

/// What decides which node lands in which copperlist slot. The recorded RON cannot be
/// compared verbatim — `ComponentConfig` is a `HashMap`, so the two processes serialize
/// its keys in different orders — and per-node values do not move slots anyway.
fn layout_witness(config: &CuConfig) -> String {
    let tasks: Vec<String> = config
        .graphs
        .get_default_mission_graph()
        .map(|graph| {
            graph
                .get_all_nodes()
                .into_iter()
                .map(|(_, node)| node.get_id())
                .collect()
        })
        .unwrap_or_else(|e| fail(e));
    format!("{tasks:?}")
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
    let mut args = std::env::args().skip(1);
    let log_base = args.next().map_or_else(
        || crate_dir.join("logs").join("autoware.copper"),
        PathBuf::from,
    );
    let out_dir = args
        .next()
        .map_or_else(|| crate_dir.join("analysis").join("data"), PathBuf::from);

    let config_path = crate_dir.join("copperconfig.ron");
    let config = config_path
        .to_str()
        .ok_or_else(|| CuError::from("crate path is not UTF-8"))
        .and_then(read_configuration)
        .unwrap_or_else(|e| fail(e));

    let mut kpis = Kpis::new();
    let source = reader(&log_base, UnifiedLogType::CopperList).unwrap_or_else(|e| fail(e));
    let (mut first, mut last, mut gaps) = (None, 0u64, 0u64);
    for culist in copperlists_reader::<CuMsgs>(source) {
        match first {
            None => first = Some(culist.id),
            Some(_) if culist.id != last + 1 => gaps += 1,
            _ => {}
        }
        last = culist.id;
        kpis.record_pass(&culist.msgs);
    }

    // `cu29_export`'s reader turns any decode failure into a silent end of iteration, so
    // nothing below this point may run on a log that was not read whole: the report would
    // be plausible and wrong. Three witnesses, cheapest first.
    let Some(first) = first else {
        fail(format!("{}: no copperlists recorded", log_base.display()));
    };
    if first != 0 || gaps > 0 || kpis.culists != last + 1 {
        fail(format!(
            "{}: non-contiguous copperlists — {} of them, ids {first}..={last}, {gaps} gap(s)",
            log_base.display(),
            kpis.culists
        ));
    }
    let (logged_ron, complete) = provenance(&log_base).unwrap_or_else(|e| fail(e));
    if !complete {
        fail(format!(
            "{}: no shutdown record — the log is truncated or the run did not finish",
            log_base.display()
        ));
    }
    let logged = read_configuration_str(logged_ron, None).unwrap_or_else(|e| fail(e));
    if layout_witness(&logged) != layout_witness(&config) {
        fail(format!(
            "{}: recorded under a different graph or plan order than {}",
            log_base.display(),
            config_path.display()
        ));
    }
    if let Ok(expected) = std::env::var("KPI_EXPECT") {
        let expected: u64 = expected
            .parse()
            .unwrap_or_else(|e| fail(format!("KPI_EXPECT: {e}")));
        if kpis.culists != expected {
            fail(format!(
                "{}: {} copperlists, KPI_EXPECT says {expected}",
                log_base.display(),
                kpis.culists
            ));
        }
    }

    kpis.report(&log_base, (first, last));

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
    let logstats = reader(&log_base, UnifiedLogType::CopperList)
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

    fn t(ns: u64) -> CuTime {
        CuDuration(ns).into()
    }

    /// The payload a fixture synthesizes for a slot, whatever type that slot carries.
    trait FixturePayload: CuMsgPayload {
        fn of_seq(seq: u64) -> Self;
    }

    impl FixturePayload for RefSample {
        fn of_seq(seq: u64) -> Self {
            RefSample {
                seq,
                ..Default::default()
            }
        }
    }

    impl FixturePayload for RefLaneSample {
        fn of_seq(seq: u64) -> Self {
            RefLaneSample(RefSample::of_seq(seq))
        }
    }

    impl FixturePayload for () {
        fn of_seq(_: u64) -> Self {}
    }

    /// One copperlist slot, written without naming its payload type: slot numbering follows
    /// the execution plan, so a pinned `runtime.plan.order` moves types between slots.
    trait Slot {
        /// Payload, time of validity and process span. `tov: None` is a firing the log
        /// could not time.
        fn fire(&mut self, port: usize, seq: u64, tov: Option<u64>, start: u64, end: u64);
        /// A pass the node ran without producing anything: the runtime still stamps the span.
        fn timed(&mut self, port: usize, start: u64, end: u64);
    }

    impl<P: FixturePayload> Slot for CuMsg<P> {
        fn fire(&mut self, port: usize, seq: u64, tov: Option<u64>, start: u64, end: u64) {
            self.set_payload(P::of_seq(seq));
            self.tov = tov.map_or(Tov::None, |ns| Tov::Time(t(ns)));
            self.timed(port, start, end);
        }

        fn timed(&mut self, port: usize, start: u64, end: u64) {
            assert_eq!(port, 0, "single-port slot");
            self.metadata.process_time.start = t(start).into();
            self.metadata.process_time.end = t(end).into();
        }
    }

    impl<A: Slot, B: Slot> Slot for (A, B) {
        fn fire(&mut self, port: usize, seq: u64, tov: Option<u64>, start: u64, end: u64) {
            match port {
                0 => self.0.fire(0, seq, tov, start, end),
                1 => self.1.fire(0, seq, tov, start, end),
                _ => panic!("no port {port}"),
            }
        }

        fn timed(&mut self, port: usize, start: u64, end: u64) {
            match port {
                0 => self.0.timed(0, start, end),
                1 => self.1.timed(0, start, end),
                _ => panic!("no port {port}"),
            }
        }
    }

    /// `&mut msgs.0.N` for a runtime N. The tuple is heterogeneous, so the index has to be
    /// a literal somewhere; this is the one place it is, and every arm coerces to `Slot`
    /// whatever type the plan put there.
    macro_rules! tuple_slots {
        ($($index:tt)*) => {
            fn tuple_slot(msgs: &mut CuMsgs, index: usize) -> &mut dyn Slot {
                match index {
                    $($index => &mut msgs.0.$index,)*
                    _ => panic!("copperlist has no slot {index}"),
                }
            }
        };
    }
    tuple_slots!(0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23);

    /// (tuple index, port) of the flat slot named `name`, e.g. `euclidean_cluster_detector#1`.
    fn slot_of(name: &str) -> (usize, usize) {
        let ids = CuMsgs::get_all_task_ids();
        let (mut index, mut port) = (0, 0);
        for (flat, slot) in slot_names().iter().enumerate() {
            if flat > 0 {
                if ids[flat] == ids[flat - 1] {
                    port += 1;
                } else {
                    index += 1;
                    port = 0;
                }
            }
            if slot == name {
                return (index, port);
            }
        }
        panic!("no copperlist slot named '{name}'");
    }

    fn fire(msgs: &mut CuMsgs, slot: &str, seq: u64, tov: Option<u64>, start: u64, end: u64) {
        let (index, port) = slot_of(slot);
        tuple_slot(msgs, index).fire(port, seq, tov, start, end);
    }

    fn timed(msgs: &mut CuMsgs, slot: &str, start: u64, end: u64) {
        let (index, port) = slot_of(slot);
        tuple_slot(msgs, index).timed(port, start, end);
    }

    fn node_index(id: &str) -> usize {
        NODES
            .iter()
            .position(|(node, _)| *node == id)
            .unwrap_or_else(|| panic!("{id} is not in the cost table"))
    }

    /// The pass where the lidar chain is fresh, times in ms.
    fn fresh_pass() -> CuMsgs {
        let mut msgs = CuMsgs::default();
        let m = &mut msgs;
        fire(
            m,
            "front_lidar",
            7,
            Some(1000 * MS),
            1000 * MS,
            1000 * MS + 100_000,
        );
        fire(
            m,
            "points_transformer_front",
            7,
            Some(1000 * MS),
            1000 * MS,
            1010 * MS,
        );
        fire(
            m,
            "rear_lidar",
            5,
            Some(1002 * MS),
            1002 * MS,
            1002 * MS + 100_000,
        );
        fire(
            m,
            "points_transformer_rear",
            5,
            Some(1002 * MS),
            1002 * MS,
            1012 * MS,
        );
        fire(
            m,
            "point_cloud_fusion",
            7,
            Some(1000 * MS),
            1012 * MS,
            1024 * MS,
        );
        fire(
            m,
            "euclidean_cluster_detector",
            7,
            Some(1000 * MS),
            1030 * MS,
            1050 * MS,
        );
        fire(
            m,
            "euclidean_cluster_detector#1",
            7,
            Some(1000 * MS),
            1030 * MS,
            1050 * MS,
        );
        fire(
            m,
            "object_collision_estimator",
            7,
            Some(1000 * MS),
            1140 * MS,
            1150 * MS,
        );
        fire(
            m,
            "behavior_planner",
            3,
            Some(1160 * MS),
            1160 * MS,
            1170 * MS,
        );
        timed(m, "vehicle_dbw", 1190 * MS, 1191 * MS);
        msgs
    }

    /// The pass where both lidars are gated off: only the settings lane and the planner
    /// fire, and the gated nodes still carry a span.
    fn gated_pass() -> CuMsgs {
        let mut msgs = CuMsgs::default();
        let m = &mut msgs;
        for node in [
            "front_lidar",
            "points_transformer_front",
            "points_transformer_rear",
            "point_cloud_fusion",
            "object_collision_estimator",
        ] {
            timed(m, node, 1200 * MS, 1200 * MS);
        }
        fire(
            m,
            "euclidean_cluster_settings",
            40,
            Some(1205 * MS),
            1205 * MS,
            1205 * MS + 100_000,
        );
        timed(m, "euclidean_cluster_detector", 1210 * MS, 1220 * MS);
        fire(
            m,
            "euclidean_cluster_detector#1",
            40,
            Some(1205 * MS),
            1210 * MS,
            1220 * MS,
        );
        fire(
            m,
            "behavior_planner",
            4,
            Some(1330 * MS),
            1330 * MS,
            1340 * MS,
        );
        timed(m, "vehicle_dbw", 1360 * MS, 1361 * MS);
        msgs
    }

    /// Slot numbering follows the execution plan, so nothing here may depend on a
    /// particular one. What must hold under any legal order: one uniquely named flat slot
    /// per task output, every one of them reachable through `tuple_slot`, and the
    /// detector's two ports adjacent in the same tuple slot.
    #[test]
    fn test_culist_slot_layout() {
        let names = slot_names();
        let mut unique = names.clone();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), names.len(), "duplicate slot name");

        let mut msgs = CuMsgs::default();
        let mut tuples = 0;
        for name in &names {
            let (index, port) = slot_of(name);
            tuple_slot(&mut msgs, index).timed(port, 0, 0);
            tuples = tuples.max(index + 1);
        }
        assert_eq!(tuples, NODES.len(), "tuple_slots! must list every slot");

        let (index, port) = slot_of("euclidean_cluster_detector#1");
        assert_eq!(port, 1);
        assert_eq!(slot_of("euclidean_cluster_detector"), (index, 0));
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
        assert_eq!(kpis.hot_path.late, 0);
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
        fire(
            &mut msgs,
            "object_collision_estimator",
            7,
            None,
            1140 * MS,
            1150 * MS,
        );
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
