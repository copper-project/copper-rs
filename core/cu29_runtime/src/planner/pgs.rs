//! Profile-guided scheduling inputs: the application contract and the
//! execution profile extracted from a recorded log.

use crate::config::CuConfig;
use crate::config::Flavor;
use crate::config::MAX_RT_PRIORITY;
use crate::config::SchedulingPolicy;
use alloc::collections::BTreeMap;
use alloc::collections::BTreeSet;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::String;
use alloc::string::ToString;
use alloc::vec::Vec;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use serde::Deserialize;
use serde::Serialize;

const PROFILE_VERSION: u32 = 1;

/// What the application asks of a schedule: the chains it measures, the CPUs
/// it may use, and the capacity it may explore. Read only by planning tools.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuContract {
    /// Latency chains, each measured inside one CopperList.
    pub chains: Vec<CuChain>,
    /// Expected delivery rate of sources, for the rate criterion.
    #[serde(default)]
    pub sources: Vec<CuSourceRate>,
    /// CPUs the plan may pin workers to.
    pub cpus: Vec<usize>,
    /// Largest number of CopperLists a candidate may keep in flight.
    pub max_in_flight: u32,
    /// Fraction of each deadline and worker window reserved for timing
    /// variation. A value of `0.2` keeps twenty percent free.
    #[serde(default = "default_headroom")]
    pub headroom: f64,
    /// Scheduling policy of every worker thread a candidate creates.
    #[serde(default)]
    pub worker_policy: SchedulingPolicy,
}

/// A latency chain from a source task to a sink task, measured per
/// CopperList as the sink's process end minus the source's time of validity
/// (its process start when no time of validity is set).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuChain {
    pub id: String,
    pub source: String,
    pub sink: String,
    pub deadline_ms: u32,
}

/// The period a source is expected to deliver at.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuSourceRate {
    pub task: String,
    pub period_ms: u32,
}

fn default_headroom() -> f64 {
    0.2
}

impl CuContract {
    /// Checks the contract against one mission of `config`: every chain and
    /// source names a task of the mission, every sink is reachable from its
    /// source, and the CPU set and capacity are usable.
    pub fn validate(&self, config: &CuConfig, mission: Option<&str>) -> CuResult<()> {
        let graph = config.get_graph(mission)?;
        let tasks: BTreeSet<String> = graph
            .get_all_nodes()
            .into_iter()
            .filter(|(_, node)| node.get_flavor() == Flavor::Task)
            .map(|(_, node)| node.get_id())
            .collect();
        let mut ids = BTreeSet::new();
        for chain in &self.chains {
            if chain.id.is_empty() || !ids.insert(&chain.id) {
                return Err(CuError::from(format!(
                    "Chain id '{}' is empty or used twice",
                    chain.id
                )));
            }
            for task in [&chain.source, &chain.sink] {
                if !tasks.contains(task) {
                    return Err(CuError::from(format!(
                        "Chain '{}' names unknown task '{task}'",
                        chain.id
                    )));
                }
            }
            if chain.deadline_ms == 0 {
                return Err(CuError::from(format!(
                    "Chain '{}' needs a positive deadline",
                    chain.id
                )));
            }
            let mut seen = BTreeSet::new();
            let mut queue = VecDeque::from([chain.source.as_str()]);
            while let Some(node) = queue.pop_front() {
                if !seen.insert(node) {
                    continue;
                }
                for edge in graph.edges().filter(|edge| edge.src == node) {
                    queue.push_back(edge.dst.as_str());
                }
            }
            if !seen.contains(chain.sink.as_str()) {
                return Err(CuError::from(format!(
                    "Chain '{}': task '{}' is not reachable from '{}'",
                    chain.id, chain.sink, chain.source
                )));
            }
        }
        let mut source_tasks = BTreeSet::new();
        for source in &self.sources {
            if !source_tasks.insert(source.task.as_str()) {
                return Err(CuError::from(format!(
                    "Source rate for task '{}' is listed twice",
                    source.task
                )));
            }
            if !tasks.contains(&source.task) {
                return Err(CuError::from(format!(
                    "Source rate names unknown task '{}'",
                    source.task
                )));
            }
            if source.period_ms == 0 {
                return Err(CuError::from(format!(
                    "Source '{}' needs a positive period",
                    source.task
                )));
            }
        }
        if self.cpus.is_empty() {
            return Err(CuError::from("The contract needs at least one CPU"));
        }
        if self.cpus.iter().collect::<BTreeSet<_>>().len() != self.cpus.len() {
            return Err(CuError::from("The contract lists a CPU twice"));
        }
        if self.max_in_flight == 0 {
            return Err(CuError::from("max_in_flight must be positive"));
        }
        if !(0.0..1.0).contains(&self.headroom) {
            return Err(CuError::from(
                "headroom must be a fraction of the deadline in [0, 1)",
            ));
        }
        match self.worker_policy {
            SchedulingPolicy::Fifo { priority } | SchedulingPolicy::RoundRobin { priority }
                if priority > MAX_RT_PRIORITY.saturating_sub(2) =>
            {
                return Err(CuError::from(format!(
                    "worker_policy priority {priority} leaves no room for the high-priority worker and dispatcher"
                )));
            }
            _ => {}
        }
        Ok(())
    }

    pub fn deserialize_ron(text: &str) -> CuResult<Self> {
        ron::from_str(text).map_err(|e| CuError::new_with_cause("Could not parse contract", e))
    }

    pub fn serialize_ron(&self) -> CuResult<String> {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
            .map_err(|e| CuError::new_with_cause("Could not serialize contract", e))
    }

    #[cfg(feature = "std")]
    pub fn read(path: &std::path::Path) -> CuResult<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| CuError::new_with_cause("Could not read contract", e))?;
        Self::deserialize_ron(&text)
    }

    #[cfg(feature = "std")]
    pub fn write(&self, path: &std::path::Path) -> CuResult<()> {
        std::fs::write(path, self.serialize_ron()?)
            .map_err(|e| CuError::new_with_cause("Could not write contract", e))
    }
}

/// Costs and chain latencies measured on one recorded run, keyed by the plan
/// step keys of the mission the run executed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuProfile {
    pub version: u32,
    /// Identity of the graph the run executed; a profile applies only to
    /// configs with the same signature.
    pub config_signature: String,
    pub mission: String,
    /// CopperLists in the measured window.
    pub copperlists: u64,
    /// Wall time from the first recorded process start to the last end.
    pub window_ns: u64,
    /// Wall span of each CopperList's process steps.
    pub copperlist_span: CuCostStats,
    pub operations: BTreeMap<String, CuOperationProfile>,
    pub chains: BTreeMap<String, CuChainProfile>,
    pub sources: BTreeMap<String, CuSourceProfile>,
}

/// One operation's cost, split by whether it produced an output payload.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuOperationProfile {
    pub fired: CuCostStats,
    pub skipped: CuCostStats,
    /// Fired invocations per second over the window.
    pub firing_rate_hz: f64,
    /// Which CopperLists the operation fires in, when that repeats.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firing: Option<CuFiringPattern>,
}

/// An operation firing in the CopperLists whose id modulo `period` is one
/// of `phases`, as observed over at least two periods.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuFiringPattern {
    pub period: u32,
    pub phases: Vec<u32>,
}

impl CuFiringPattern {
    /// The smallest period under which `fired` (indexed by CopperList id
    /// from `first_id`) repeats, checked over every pair of ids one period
    /// apart; `None` below two periods of evidence or above `max_period`.
    pub fn detect(first_id: u64, fired: &[bool], max_period: u32) -> Option<Self> {
        (1..=max_period as usize)
            .filter(|&period| fired.len() >= 2 * period)
            .find(|&period| (period..fired.len()).all(|i| fired[i] == fired[i - period]))
            .map(|period| Self {
                period: period as u32,
                phases: (0..period)
                    .filter(|&i| fired[i])
                    .map(|i| ((first_id + i as u64) % period as u64) as u32)
                    .collect(),
            })
    }

    /// The fraction of CopperLists with `id % modulus == offset` the
    /// operation fires in.
    pub fn probability(&self, modulus: u32, offset: u32) -> f64 {
        let g = gcd(self.period, modulus);
        let hits = self
            .phases
            .iter()
            .filter(|&&phase| phase % g == offset % g)
            .count() as f64;
        hits / f64::from(self.period / g)
    }
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Summary of a set of durations, in nanoseconds; zeros when empty.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuCostStats {
    pub samples: u64,
    pub min_ns: u64,
    pub p50_ns: u64,
    pub mean_ns: f64,
    pub p95_ns: u64,
    pub p99_ns: u64,
    pub max_ns: u64,
}

impl CuCostStats {
    /// Summarizes `samples`, which it sorts in place.
    pub fn from_samples(samples: &mut [u64]) -> Self {
        if samples.is_empty() {
            return Self::default();
        }
        samples.sort_unstable();
        let percentile = |q: f64| {
            // Nearest-rank index; `core` has no `round`.
            let index = ((samples.len() - 1) as f64 * q + 0.5) as usize;
            samples[index]
        };
        Self {
            samples: samples.len() as u64,
            min_ns: samples[0],
            p50_ns: percentile(0.50),
            mean_ns: samples.iter().map(|&v| v as f64).sum::<f64>() / samples.len() as f64,
            p95_ns: percentile(0.95),
            p99_ns: percentile(0.99),
            max_ns: samples[samples.len() - 1],
        }
    }
}

/// Measured latency of one chain.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuChainProfile {
    pub deadline_ms: u32,
    pub latency: CuCostStats,
    /// Samples over the deadline.
    pub misses: u64,
}

/// Delivered rate of one source against the contract's period.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuSourceProfile {
    pub period_ms: u32,
    pub fired: u64,
    /// Intervals of the period over the source's firing span (at least the
    /// run's window less one period).
    pub expected: f64,
    /// `(fired - 1) / expected`: the intervals between firings over the
    /// intervals the period allows; 1 for a source firing on its period.
    pub delivered_rate: f64,
}

/// A stable identity of one mission graph: its nodes, types and edges. A
/// profile applies only to a config with the same signature.
pub fn graph_signature(graph: &crate::config::CuGraph, mission: Option<&str>) -> String {
    let mut parts = Vec::new();
    parts.push(format!("mission={}", mission.unwrap_or("default")));
    let mut nodes: Vec<_> = graph.get_all_nodes();
    nodes.sort_by_key(|a| a.1.get_id());
    for (_, node) in nodes {
        let flavor = match node.get_flavor() {
            Flavor::Task => "task",
            Flavor::Bridge => "bridge",
        };
        parts.push(format!(
            "node|{}|{}|{flavor}|kind={:?}|background={}|pool={}|anytime={:?}",
            node.get_id(),
            node.get_type(),
            node.get_declared_task_kind(),
            node.is_background(),
            node.background_pool(),
            node.anytime(),
        ));
        if let Some(resources) = node.get_resources() {
            let mut resources: Vec<_> = resources.iter().collect();
            resources.sort_by_key(|(binding, _)| *binding);
            for (binding, resource) in resources {
                parts.push(format!("resource|{}|{binding}|{resource}", node.get_id()));
            }
        }
        if let Some(config) = node.get_instance_config() {
            let mut values: Vec<_> = config.0.iter().collect();
            values.sort_by_key(|(key, _)| *key);
            for (key, value) in values {
                parts.push(format!("config|{}|{key}|{value}", node.get_id()));
            }
        }
    }
    let endpoint = |node: &str, channel: Option<&str>| match channel {
        Some(channel) => format!("{node}/{channel}"),
        None => node.to_string(),
    };
    let mut edges: Vec<String> = graph
        .edges()
        .map(|cnx| {
            format!(
                "edge|{}|{}|{}",
                endpoint(cnx.src.as_str(), cnx.src_channel.as_deref()),
                endpoint(cnx.dst.as_str(), cnx.dst_channel.as_deref()),
                cnx.msg
            )
        })
        .collect();
    edges.sort();
    parts.extend(edges);
    let joined = parts.join("\n");
    let mut hash: u64 = 0xcbf29ce484222325;
    for byte in joined.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("fnv1a64:{hash:016x}")
}

impl CuProfile {
    pub fn new(config_signature: String, mission: String) -> Self {
        Self {
            version: PROFILE_VERSION,
            config_signature,
            mission,
            copperlists: 0,
            window_ns: 0,
            copperlist_span: CuCostStats::default(),
            operations: BTreeMap::new(),
            chains: BTreeMap::new(),
            sources: BTreeMap::new(),
        }
    }

    pub fn serialize_ron(&self) -> CuResult<String> {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
            .map_err(|e| CuError::new_with_cause("Could not serialize profile", e))
    }

    pub fn deserialize_ron(text: &str) -> CuResult<Self> {
        let profile: Self = ron::from_str(text)
            .map_err(|e| CuError::new_with_cause("Could not parse profile", e))?;
        if profile.version != PROFILE_VERSION {
            return Err(CuError::from(format!(
                "Unsupported profile version {}; expected {PROFILE_VERSION}",
                profile.version
            )));
        }
        Ok(profile)
    }

    #[cfg(feature = "std")]
    pub fn read(path: &std::path::Path) -> CuResult<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| CuError::new_with_cause("Could not read profile", e))?;
        Self::deserialize_ron(&text)
    }

    #[cfg(feature = "std")]
    pub fn write(&self, path: &std::path::Path) -> CuResult<()> {
        std::fs::write(path, self.serialize_ron()?)
            .map_err(|e| CuError::new_with_cause("Could not write profile", e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> CuConfig {
        CuConfig::deserialize_ron(
            r#"(
            tasks: [(id: "src", type: "Source"), (id: "work", type: "Work"), (id: "sink", type: "Sink"), (id: "other", type: "Source")],
            cnx: [(src: "src", dst: "work", msg: "u32"), (src: "work", dst: "sink", msg: "u32"), (src: "other", dst: "__nc__", msg: "u32")],
        )"#,
        )
        .unwrap()
    }

    fn contract() -> CuContract {
        CuContract::deserialize_ron(
            r#"(
            chains: [(id: "hot", source: "src", sink: "sink", deadline_ms: 10)],
            sources: [(task: "src", period_ms: 20)],
            cpus: [0, 1],
            max_in_flight: 2,
        )"#,
        )
        .unwrap()
    }

    #[test]
    fn contract_parses_with_defaults_and_validates() {
        let contract = contract();
        assert_eq!(contract.headroom, 0.2);
        contract.validate(&config(), None).unwrap();
        assert_eq!(
            CuContract::deserialize_ron(&contract.serialize_ron().unwrap()).unwrap(),
            contract
        );
    }

    #[test]
    fn contract_rejects_unknown_tasks_unreachable_sinks_and_bad_values() {
        let config = config();
        let mutations: [fn(&mut CuContract); 9] = [
            |c| c.chains[0].sink = "missing".into(),
            |c| c.chains[0].sink = "other".into(),
            |c| c.chains[0].deadline_ms = 0,
            |c| c.chains.push(c.chains[0].clone()),
            |c| c.sources[0].task = "missing".into(),
            |c| c.sources.push(c.sources[0].clone()),
            |c| c.cpus = vec![1, 1],
            |c| c.max_in_flight = 0,
            |c| c.headroom = 1.0,
        ];
        for (index, mutate) in mutations.iter().enumerate() {
            let mut invalid = contract();
            mutate(&mut invalid);
            assert!(invalid.validate(&config, None).is_err(), "mutation {index}");
        }
    }

    #[test]
    fn cost_stats_summarize_and_profile_round_trips() {
        let mut samples = [5u64, 1, 4, 2, 3];
        let stats = CuCostStats::from_samples(&mut samples);
        assert_eq!(
            (stats.samples, stats.min_ns, stats.p50_ns, stats.max_ns),
            (5, 1, 3, 5)
        );
        assert_eq!(stats.mean_ns, 3.0);
        assert_eq!(CuCostStats::from_samples(&mut []), CuCostStats::default());
        let mut profile = CuProfile::new("sig".into(), "default".into());
        profile.operations.insert(
            "mission:default|task:src|phase:whole".into(),
            CuOperationProfile {
                fired: stats,
                skipped: CuCostStats::default(),
                firing_rate_hz: 50.0,
                firing: None,
            },
        );
        let text = profile.serialize_ron().unwrap();
        assert_eq!(CuProfile::deserialize_ron(&text).unwrap(), profile);
        assert!(CuProfile::deserialize_ron(&text.replace("version: 1", "version: 9")).is_err());
    }

    #[test]
    fn a_firing_pattern_is_detected_and_projected_on_another_modulus() {
        // Fires in CopperLists 3, 6, 9, ... observed from id 2.
        let fired: Vec<bool> = (2u64..40).map(|id| id.is_multiple_of(3)).collect();
        let pattern = CuFiringPattern::detect(2, &fired, 64).unwrap();
        assert_eq!((pattern.period, pattern.phases.clone()), (3, vec![0]));
        assert_eq!(pattern.probability(3, 0), 1.0);
        assert_eq!(pattern.probability(3, 1), 0.0);
        // Against a window of 4, every offset fires in a third of its CopperLists.
        assert!((pattern.probability(4, 1) - 1.0 / 3.0).abs() < 1e-9);
        // Below two periods of evidence there is no pattern.
        assert!(CuFiringPattern::detect(0, &[true, false, false, true, false], 64).is_none());
        let every = CuFiringPattern::detect(0, &[true; 8], 64).unwrap();
        assert_eq!((every.period, every.phases), (1, vec![0]));
    }
}
