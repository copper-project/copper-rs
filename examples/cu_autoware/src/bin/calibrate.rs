//! Turns the reference system's per-node execution times into `crunch_limit` values for
//! this host, and with `--apply` writes them into `copperconfig.ron`.
//!
//! Run it in release: the crunch is the workload under measurement, and a debug build
//! calibrates a different one. `crunch` is `#[inline(never)]` so this binary and the app
//! execute the same codegen of it; the authoritative per-node check is step 4's logged
//! `process_time`, which measured every load-carrying class within about 1% of target.
//!
//! The search also steers on the median while the KPIs pay the mean, which puts calibrate
//! on the optimistic side of whatever the app ends up charging.

use chrono::Local;
use cu_autoware::tasks::crunch;
use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// (node id, config key, target execution time in µs). Fig. 4 of the RTNS'25 paper; the
/// single authority for `--apply`. Nodes sharing a target share one measured limit.
///
/// `behavior_planner`'s `cache_crunch_limit` is deliberately absent: its 0.001ms input
/// callback is a pointer assignment, below what a timed search can resolve, so the config
/// keeps a fixed small limit there.
const TARGETS: &[(&str, &str, u64)] = &[
    ("front_lidar", "crunch_limit", 100),
    ("rear_lidar", "crunch_limit", 100),
    ("point_cloud_map", "crunch_limit", 100),
    ("visualizer", "crunch_limit", 100),
    ("lanelet2_map", "crunch_limit", 100),
    ("euclidean_cluster_settings", "crunch_limit", 100),
    ("vehicle_dbw", "crunch_limit", 1000),
    ("intersection_output", "crunch_limit", 1000),
    ("point_cloud_fusion", "cache_crunch_limit", 2100),
    ("ndt_localizer", "cache_crunch_limit", 2100),
    ("lanelet2_global_planner", "cache_crunch_limit", 2100),
    ("lanelet2_map_loader", "cache_crunch_limit", 2100),
    ("vehicle_interface", "cache_crunch_limit", 2100),
    ("point_cloud_fusion", "crunch_limit", 10005),
    ("points_transformer_front", "crunch_limit", 10100),
    ("points_transformer_rear", "crunch_limit", 10100),
    ("voxel_grid_downsampler", "crunch_limit", 10100),
    ("point_cloud_map_loader", "crunch_limit", 10100),
    ("ray_ground_filter", "crunch_limit", 10100),
    ("object_collision_estimator", "crunch_limit", 10100),
    ("mpc_controller", "crunch_limit", 10100),
    ("parking_planner", "crunch_limit", 10100),
    ("lane_planner", "crunch_limit", 10100),
    ("ndt_localizer", "crunch_limit", 10100),
    ("vehicle_interface", "crunch_limit", 10100),
    ("behavior_planner", "crunch_limit", 10100),
    ("euclidean_cluster_detector", "crunch_limit0", 10100),
    ("lanelet2_global_planner", "crunch_limit", 10200),
    ("lanelet2_map_loader", "crunch_limit", 10200),
    ("euclidean_cluster_detector", "crunch_limit1", 10200),
];

/// Timed runs per candidate limit; the median is what the search steers on.
const RUNS: usize = 15;
/// The search aims at 0.3%; `--apply` refuses to write anything worse than the 5% bar.
/// 1% used to be the bar, which is the whole gap between the 10.1ms and 10.2ms classes:
/// the search could stop early on either side of it and hand the wider class the smaller
/// limit. It is only an early exit, so a tighter bar costs probes, not convergence.
const TOLERANCE: f64 = 0.003;
const APPLY_BAR: f64 = 5.0;
const PROBE_LIMIT: u64 = 20_000;
const WARMUP: Duration = Duration::from_millis(500);
const HEADER_MARK: &str = "// Calibrated on ";
const HEADER_NOTE: &str =
    "// Limits are host-measured: rerun `cargo run --release --bin calibrate -- --apply`.";

/// Median and spread (max-min over median, in %) of `RUNS` timed crunches, in µs.
fn measure(limit: u64) -> (f64, f64) {
    let mut samples: Vec<f64> = (0..RUNS)
        .map(|_| {
            let start = Instant::now();
            crunch(limit);
            start.elapsed().as_secs_f64() * 1e6
        })
        .collect();
    samples.sort_by(f64::total_cmp);
    let median = samples[RUNS / 2];
    (median, (samples[RUNS - 1] - samples[0]) / median * 100.0)
}

/// Binary search for the limit costing `target_us`. `cost` only has to be monotonic.
fn search(target_us: f64, mut cost: impl FnMut(u64) -> f64) -> u64 {
    // The cruncher is O(n*sqrt(n)), so one probe places the first guess within a factor.
    let probe = cost(PROBE_LIMIT);
    let guess = (PROBE_LIMIT as f64 * (target_us / probe).powf(2.0 / 3.0)).max(1.0) as u64;
    let (mut lo, mut hi) = (guess / 2 + 1, guess * 2 + 1);
    while lo > 1 && cost(lo) > target_us {
        lo /= 2;
    }
    while cost(hi) < target_us {
        hi *= 2;
    }
    while lo + 1 < hi {
        let mid = lo + (hi - lo) / 2;
        let measured = cost(mid);
        if (measured - target_us).abs() / target_us <= TOLERANCE {
            return mid;
        }
        if measured < target_us {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

fn cpu_model() -> String {
    fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|info| {
            info.lines()
                .find(|line| line.starts_with("model name"))
                .and_then(|line| line.split_once(':'))
                .map(|(_, model)| model.trim().to_string())
        })
        .unwrap_or_else(|| "unknown CPU".to_string())
}

/// Drops a header this binary generated, and only that: a hand-written comment survives
/// even if the generated block was edited down to one line.
fn strip_header(text: &str) -> &str {
    let mut rest = text;
    loop {
        let line = rest.lines().next().unwrap_or("");
        if !line.starts_with(HEADER_MARK) && line != HEADER_NOTE {
            return rest;
        }
        rest = rest[line.len()..].strip_prefix('\n').unwrap_or("");
    }
}

fn limit_of(
    targets: &[(&str, &str, u64)],
    limits: &[(u64, u64)],
    node: &str,
    key: &str,
) -> Option<u64> {
    let (_, _, target) = targets.iter().find(|(n, k, _)| *n == node && *k == key)?;
    limits
        .iter()
        .find(|(class, _)| class == target)
        .map(|(_, limit)| *limit)
}

/// `        "crunch_limit": 123, // note` with the value swapped, anything else untouched.
fn rewrite_line(line: &str, limit_of: impl Fn(&str) -> Option<u64>) -> Option<String> {
    let (head, tail) = line.split_once("\": ")?;
    let limit = limit_of(head.trim_start().strip_prefix('"')?)?;
    let digits = tail.len() - tail.trim_start_matches(|c: char| c.is_ascii_digit()).len();
    (digits > 0).then(|| format!("{head}\": {limit}{}", &tail[digits..]))
}

/// The rewritten config and the number of values replaced. Node scope comes from the
/// preceding `id:` line, which is what makes the two `RefIntersection` lanes separable.
fn rewrite(body: &str, targets: &[(&str, &str, u64)], limits: &[(u64, u64)]) -> (String, usize) {
    let mut node = "";
    let mut rewrites = 0;
    let mut out = String::new();
    for line in body.lines() {
        if let Some(rest) = line.trim_start().strip_prefix("id: \"") {
            node = rest.split('"').next().unwrap_or("");
        }
        match rewrite_line(line, |key| limit_of(targets, limits, node, key)) {
            Some(rewritten) => {
                out.push_str(&rewritten);
                rewrites += 1;
            }
            None => out.push_str(line),
        }
        out.push('\n');
    }
    (out, rewrites)
}

fn apply(limits: &[(u64, u64)]) {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("copperconfig.ron");
    let text = fs::read_to_string(&path).expect("copperconfig.ron must be readable");
    let (body, rewrites) = rewrite(strip_header(&text), TARGETS, limits);
    if rewrites != TARGETS.len() {
        eprintln!(
            "copperconfig.ron: rewrote {rewrites} of {} calibrated values, so the config and \
             TARGETS disagree on node ids or keys. Nothing written.",
            TARGETS.len()
        );
        std::process::exit(1);
    }
    let out = format!(
        "{HEADER_MARK}{}, {}.\n{HEADER_NOTE}\n{body}",
        cpu_model(),
        Local::now().format("%Y-%m-%d")
    );
    // fmtron's canonical form has no trailing newline.
    fs::write(&path, out.trim_end()).expect("copperconfig.ron must be writable");
    println!("applied {rewrites} limits to {}", path.display());
}

fn main() {
    if cfg!(debug_assertions) {
        eprintln!("calibrate measures the release workload: cargo run --release --bin calibrate");
        std::process::exit(1);
    }
    let mut apply_to_config = false;
    for arg in std::env::args().skip(1) {
        if arg != "--apply" {
            eprintln!("unknown argument '{arg}'. usage: calibrate [--apply]");
            std::process::exit(2);
        }
        apply_to_config = true;
    }

    // The app runs the crunch back to back, so calibrate against a warm core: a cold one
    // reads several percent slow here, which is the whole error budget.
    let warmup = Instant::now();
    while warmup.elapsed() < WARMUP {
        crunch(PROBE_LIMIT);
    }

    let mut classes: Vec<u64> = TARGETS.iter().map(|(_, _, target)| *target).collect();
    classes.sort_unstable();
    classes.dedup();

    println!("class_ms     limit  median_ms    err%  spread%");
    let mut limits = Vec::new();
    let mut worst = 0.0f64;
    for target in classes {
        let limit = search(target as f64, |limit| measure(limit).0);
        let (median, spread) = measure(limit);
        let err = (median - target as f64) / target as f64 * 100.0;
        println!(
            "{:8.3}  {limit:8}  {:9.3}  {err:+6.2}  {spread:7.2}",
            target as f64 / 1000.0,
            median / 1000.0
        );
        limits.push((target, limit));
        worst = worst.max(err.abs());
    }

    if apply_to_config {
        if worst > APPLY_BAR {
            eprintln!(
                "worst class is {worst:.2}% off target, over the {APPLY_BAR}% bar. Nothing written."
            );
            std::process::exit(1);
        }
        apply(&limits);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_TARGETS: &[(&str, &str, u64)] = &[
        ("front_lidar", "crunch_limit", 100),
        ("euclidean_cluster_detector", "crunch_limit0", 10100),
        ("euclidean_cluster_detector", "crunch_limit1", 10200),
    ];
    const TEST_LIMITS: &[(u64, u64)] = &[(100, 11), (10100, 22), (10200, 33)];

    const BODY: &str = "\
(
    tasks: [
        (
            id: \"front_lidar\",
            config: {
                \"period_ms\": 200,
                \"crunch_limit\": 1, // stale
            },
        ),
        (
            id: \"euclidean_cluster_detector\",
            config: {
                \"crunch_limit0\": 2,
                \"crunch_limit1\": 3,
            },
        ),
    ],
)";

    /// Pins the search against a synthetic O(n^1.5) cost, so it never depends on the clock.
    #[test]
    fn test_search_converges_on_a_synthetic_cost() {
        let cost = |limit: u64| (limit as f64).powf(1.5) / 1000.0;
        for target in [100.0, 2100.0, 10_200.0] {
            let limit = search(target, cost);
            let err = (cost(limit) - target).abs() / target;
            assert!(err <= TOLERANCE, "limit {limit} for {target}us: err {err}");
        }
    }

    /// A staircase cost whose treads are 1000us apart: nothing lands within TOLERANCE of a
    /// target between two of them, so the search has to exhaust the bisection and return
    /// the cheapest limit that still meets the target.
    #[test]
    fn test_search_falls_back_to_the_first_limit_meeting_a_plateau_target() {
        let cost = |limit: u64| (limit / 1000 * 1000) as f64;
        let target = 10_500.0;
        let limit = search(target, cost);
        assert!(
            cost(limit) >= target,
            "limit {limit} undershoots the target"
        );
        assert!(
            cost(limit - 1) < target,
            "limit {limit} is not the cheapest one meeting it"
        );
    }

    #[test]
    fn test_rewrite_replaces_mapped_values_only() {
        let (out, rewrites) = rewrite(BODY, TEST_TARGETS, TEST_LIMITS);
        assert_eq!(rewrites, TEST_TARGETS.len());
        assert!(out.contains("                \"crunch_limit\": 11, // stale\n"));
        assert!(
            out.contains("\"period_ms\": 200,"),
            "unmapped key rewritten"
        );
        assert!(out.contains("\"crunch_limit0\": 22,") && out.contains("\"crunch_limit1\": 33,"));
    }

    /// A renamed node or a stale TARGETS entry has to show up as a short count, which is
    /// what `apply` refuses to write on.
    #[test]
    fn test_rewrite_counts_short_when_a_key_is_missing() {
        let body = BODY.replace("                \"crunch_limit1\": 3,\n", "");
        let (_, rewrites) = rewrite(&body, TEST_TARGETS, TEST_LIMITS);
        assert_eq!(rewrites, TEST_TARGETS.len() - 1);
    }

    #[test]
    fn test_strip_header_only_drops_generated_lines() {
        let hand_written = "// hand written\n(\n)";
        assert_eq!(strip_header(hand_written), hand_written);
        let generated =
            format!("{HEADER_MARK}some cpu, 2026-01-01.\n{HEADER_NOTE}\n{hand_written}");
        assert_eq!(strip_header(&generated), hand_written);
        // Half the block edited away must still not eat the comment under it.
        let half = format!("{HEADER_MARK}some cpu, 2026-01-01.\n{hand_written}");
        assert_eq!(strip_header(&half), hand_written);
    }
}
