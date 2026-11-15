//! Candidate plans from a profile and a contract: the response-time model
//! with the worker as its unit, a constraint-ranked score, and a local search
//! over which worker runs each occurrence and in which order.
//!
//! Workers form a pipeline: no zero-lag edge leads from a later worker back
//! to an earlier one, so a worker may run ahead of the others up to
//! `max_in_flight` instead of every worker finishing a CopperList before any
//! starts the next. Under a real-time policy each CPU gets a second worker
//! one priority above the base one, for the chains with short deadlines.

use super::CuContract;
use super::CuMissionPlan;
use super::CuPlan;
use super::CuPlanPlacement;
use super::CuPlanThread;
use super::CuPlanWorker;
use super::CuProfile;
use crate::config::CuConfig;
use crate::config::MAX_RT_PRIORITY;
use crate::config::SchedulingPolicy;
use crate::config::TaskKind;
use alloc::collections::BTreeMap;
use alloc::collections::BTreeSet;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::String;
use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use serde::Deserialize;
use serde::Serialize;

/// Response times past this many periods count as unbounded.
const RESPONSE_CAP_PERIODS: u64 = 20;
/// Moves without improvement before a restart jumps back to its best plan.
const PATIENCE: usize = 120;
/// Unconditional moves applied after such a jump.
const KICK: usize = 3;
/// Two candidates whose sum tier differs by less than this fraction count as
/// the same candidate.
const MIN_SEPARATION: f64 = 0.01;
/// A delivered rate this close to nominal counts as kept.
const RATE_TOLERANCE: f64 = 0.005;
/// The resolution of the rate tier.
const RATE_STEP: f64 = 0.02;
/// Deterministic search seed; search tuning is deliberately not user-facing.
const SEARCH_SEED: u64 = 20260914;
/// Moves per deterministic local-search restart.
#[cfg(not(test))]
const SEARCH_MOVES: usize = 2000;
#[cfg(test)]
const SEARCH_MOVES: usize = 300;
/// Deterministic local-search restarts.
#[cfg(not(test))]
const SEARCH_RESTARTS: usize = 8;
#[cfg(test)]
const SEARCH_RESTARTS: usize = 3;

/// What the proposer is asked to do.
#[derive(Clone, Debug)]
pub struct ProposeRequest<'a> {
    pub config: &'a CuConfig,
    pub mission: &'a str,
    pub contract: &'a CuContract,
    pub profile: &'a CuProfile,
    /// How many distinct candidates to return.
    pub candidates: usize,
}

/// One candidate: its plan and what the model predicts for it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CuCandidate {
    pub plan: CuPlan,
    pub prediction: CuPrediction,
}

/// The model's prediction for one plan of one mission.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPrediction {
    /// The constraint-ranked score tiers; smallest first is better.
    pub score: Vec<f64>,
    pub chains: BTreeMap<String, CuChainPrediction>,
    pub workers: BTreeMap<String, CuWorkerPrediction>,
    /// Predicted delivered fraction per contract source.
    pub sources: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuChainPrediction {
    pub latency_ns: u64,
    pub deadline_ns: u64,
    /// `latency / deadline`.
    pub ratio: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuWorkerPrediction {
    pub cpu: usize,
    /// Busy share of the window: the worker's own cost plus that of the
    /// higher-priority workers on its CPU.
    pub load: f64,
    /// Response time of one cycle, `None` when it does not converge.
    pub response_ns: Option<u64>,
    /// Delivered fraction of cycles.
    pub rate: f64,
}

/// The model's view of one mission plan's inventory: cost per unit,
/// dependency edges, and what each chain and source maps to.
///
/// A unit is one occurrence, or an anytime base occurrence with its refine
/// occurrences: the executor runs those phases together on one worker, so
/// the search never separates them.
struct Model {
    inventory: CuMissionPlan,
    /// Occurrence indices of each unit, in phase order.
    units: Vec<Vec<usize>>,
    /// Expected cost of each unit per cycle, in nanoseconds.
    cost: Vec<u64>,
    /// Cost of each unit in a cycle where it fires, in nanoseconds.
    fired_cost: Vec<u64>,
    /// Zero-lag predecessor units of each unit.
    preds: Vec<Vec<usize>>,
    /// CopperList period, the dispatch granularity `g`.
    copperlist_ns: u64,
    /// Cycle period `T`.
    cycle_ns: u64,
    /// Per chain: `(source occurrence, sink occurrence)` per CL offset.
    chains: Vec<Vec<(usize, usize)>>,
    chain_deadlines: Vec<u32>,
    /// Per contract source: its occurrences.
    sources: Vec<Vec<usize>>,
    /// Per unit: it lies on a chain whose deadline is within its source's
    /// period, so it may run on a higher-tier worker.
    short: Vec<bool>,
    /// The units on each such chain's paths; a chain stays within one tier.
    short_paths: Vec<Vec<usize>>,
    /// Per window cycle and unit: whether the unit fires there (always,
    /// without a firing pattern).
    fires: Vec<Vec<bool>>,
    cpus: Vec<usize>,
    /// CopperLists in flight at once, at least one cycle's worth.
    max_in_flight: u32,
    /// Expected cost of each unit in each cycle of the window, the longest
    /// source period, from its firing pattern.
    window_cost: Vec<Vec<u64>>,
    policy: SchedulingPolicy,
    headroom: f64,
}

/// A candidate under search: an ordered list of occurrences per worker. Lane
/// `l` runs on `cpus[l % cpus.len()]` at tier `l / cpus.len()`; tier 1 is
/// the higher-priority worker of that CPU.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct Assignment {
    lanes: Vec<Vec<usize>>,
}

struct Evaluation {
    score: Vec<f64>,
    /// Per chain, the latency the score used.
    chain_latency: Vec<u64>,
    response: Vec<Option<u64>>,
    rate: Vec<f64>,
    load: Vec<f64>,
    cycle_rate: f64,
}

impl Model {
    fn new(
        request: &ProposeRequest<'_>,
        inventory: CuMissionPlan,
        max_in_flight: u32,
    ) -> CuResult<Self> {
        let contract = request.contract;
        let profile = request.profile;
        let copperlist_ns = match request
            .config
            .runtime
            .as_ref()
            .and_then(|runtime| runtime.rate_target_hz)
        {
            Some(rate) => 1_000_000_000 / rate.max(1),
            None => profile
                .window_ns
                .checked_div(profile.copperlists.max(1))
                .unwrap_or(0)
                .max(1),
        };
        let background: BTreeSet<&str> = inventory
            .background
            .iter()
            .map(|entry| entry.task.as_str())
            .collect();
        // A refine occurrence joins the unit of its task's base occurrence in
        // the same CopperList; the profile measures the whole job under the
        // base key.
        let mut units: Vec<Vec<usize>> = Vec::new();
        let mut unit_of = vec![usize::MAX; inventory.steps.len()];
        for (index, step) in inventory.steps.iter().enumerate() {
            if is_refine(&step.key) {
                let base = inventory.steps.iter().position(|other| {
                    other.copperlist == step.copperlist
                        && !is_refine(&other.key)
                        && task_of_key(&other.key) == task_of_key(&step.key)
                });
                let Some(base) = base.map(|b| unit_of[b]).filter(|&u| u != usize::MAX) else {
                    return Err(CuError::from(format!(
                        "Refine step '{}' has no base step before it",
                        step.key
                    )));
                };
                units[base].push(index);
                unit_of[index] = base;
            } else {
                unit_of[index] = units.len();
                units.push(vec![index]);
            }
        }
        let mut cost = Vec::with_capacity(units.len());
        let mut fired_cost = Vec::with_capacity(units.len());
        let mut firing = Vec::with_capacity(units.len());
        for unit in &units {
            let step = &inventory.steps[unit[0]];
            let task = task_of_key(&step.key);
            if task.is_some_and(|task| background.contains(task)) {
                // The gateway only publishes and dispatches; the compute runs
                // on its pool.
                cost.push(0);
                fired_cost.push(0);
                firing.push(None);
                continue;
            }
            let operation = profile.operations.get(&step.key).ok_or_else(|| {
                CuError::from(format!("The profile has no measurement for '{}'", step.key))
            })?;
            // The fraction of CopperLists the operation fires in, from counts,
            // so an overloaded profiling run (slower CopperLists) does not
            // understate it.
            let fired = (operation.fired.samples as f64 / profile.copperlists.max(1) as f64)
                .clamp(0.0, 1.0);
            let expected =
                operation.fired.mean_ns * fired + operation.skipped.mean_ns * (1.0 - fired);
            cost.push(expected as u64);
            fired_cost.push(operation.fired.mean_ns.max(operation.skipped.mean_ns) as u64);
            firing.push(
                operation
                    .firing
                    .clone()
                    .map(|pattern| (pattern, operation.fired.mean_ns, operation.skipped.mean_ns)),
            );
        }
        // The window is the longest source period; a unit's cost in each of
        // its CopperLists follows its firing pattern, or its average without.
        let copperlists_per_cycle = inventory.copperlists_per_cycle;
        let window_ns = contract
            .sources
            .iter()
            .map(|source| u64::from(source.period_ms) * 1_000_000)
            .max()
            .unwrap_or(copperlist_ns)
            .max(copperlist_ns);
        let window_cycles = window_ns.div_ceil(copperlist_ns * u64::from(copperlists_per_cycle));
        let window_cls = (window_cycles * u64::from(copperlists_per_cycle)) as u32;
        let probability = |cycle: u64, unit: usize| {
            firing[unit].as_ref().map(|(pattern, _, _)| {
                let offset = cycle as u32 * copperlists_per_cycle
                    + inventory.steps[units[unit][0]].copperlist;
                pattern.probability(window_cls, offset)
            })
        };
        let window_cost: Vec<Vec<u64>> = (0..window_cycles)
            .map(|cycle| {
                (0..units.len())
                    .map(|unit| match (probability(cycle, unit), &firing[unit]) {
                        (Some(p), Some((_, fired_ns, skipped_ns))) => {
                            (fired_ns * p + skipped_ns * (1.0 - p)) as u64
                        }
                        _ => cost[unit],
                    })
                    .collect()
            })
            .collect();
        let fires: Vec<Vec<bool>> = (0..window_cycles)
            .map(|cycle| {
                (0..units.len())
                    .map(|unit| probability(cycle, unit).is_none_or(|p| p > 0.0))
                    .collect()
            })
            .collect();
        let mut preds: Vec<Vec<usize>> = vec![Vec::new(); units.len()];
        for edge in &inventory.dependencies {
            let (from, to) = (unit_of[edge.from as usize], unit_of[edge.to as usize]);
            if edge.cycle_lag == 0 && from != to && !preds[to].contains(&from) {
                preds[to].push(from);
            }
        }
        // Running an anytime task's phases together must not close a cycle
        // through another component (a resource or state edge into a
        // refinement and out of the base).
        if let Some(unit) = first_unit_on_a_cycle(&preds) {
            return Err(CuError::from(format!(
                "Step '{}' and its refinements cannot run together: an edge leads out of one of its phases and back into an earlier one through another component",
                inventory.steps[units[unit][0]].key
            )));
        }
        // A task's unit at each CopperList offset.
        let units_of = |task: &str| -> Vec<usize> {
            (0..inventory.copperlists_per_cycle)
                .filter_map(|offset| {
                    inventory
                        .steps
                        .iter()
                        .position(|step| {
                            step.copperlist == offset
                                && !is_refine(&step.key)
                                && task_of_key(&step.key) == Some(task)
                        })
                        .map(|index| unit_of[index])
                })
                .collect()
        };
        let mut chains = Vec::new();
        for chain in &contract.chains {
            let sources = units_of(&chain.source);
            let sinks = units_of(&chain.sink);
            if sources.len() != inventory.copperlists_per_cycle as usize
                || sinks.len() != sources.len()
            {
                return Err(CuError::from(format!(
                    "Chain '{}' names a task without a whole-phase step",
                    chain.id
                )));
            }
            chains.push(sources.into_iter().zip(sinks).collect());
        }
        let sources = contract
            .sources
            .iter()
            .map(|source| units_of(&source.task))
            .collect();
        // A chain due within its source's period cannot be pipelined over
        // periods: the units on its paths may take a higher-priority worker.
        let mut succs: Vec<Vec<usize>> = vec![Vec::new(); units.len()];
        for (to, from) in preds.iter().enumerate() {
            for &from in from {
                succs[from].push(to);
            }
        }
        let mut short = vec![false; units.len()];
        let mut short_paths = Vec::new();
        for (chain, pairs) in chains.iter().enumerate() {
            let spec = &contract.chains[chain];
            let period = contract
                .sources
                .iter()
                .find(|source| source.task == spec.source)
                .map(|source| source.period_ms);
            if period.is_some_and(|period| spec.deadline_ms <= period) {
                for &(source, sink) in pairs {
                    let (forward, backward) = (reachable(&succs, source), reachable(&preds, sink));
                    let path: Vec<usize> = (0..units.len())
                        .filter(|&u| forward[u] && backward[u])
                        .collect();
                    for &unit in &path {
                        short[unit] = true;
                    }
                    short_paths.push(path);
                }
            }
        }
        Ok(Self {
            cycle_ns: copperlist_ns * u64::from(copperlists_per_cycle),
            inventory,
            units,
            cost,
            fired_cost,
            preds,
            copperlist_ns,
            chains,
            chain_deadlines: contract
                .chains
                .iter()
                .map(|chain| chain.deadline_ms)
                .collect(),
            sources,
            short,
            short_paths,
            fires,
            cpus: contract.cpus.clone(),
            max_in_flight: max_in_flight.max(copperlists_per_cycle),
            window_cost,
            policy: contract.worker_policy,
            headroom: contract.headroom,
        })
    }

    /// Workers per CPU: a second one a priority above the base worker under
    /// a real-time policy.
    fn tiers(&self) -> usize {
        match self.policy {
            SchedulingPolicy::Fifo { priority } | SchedulingPolicy::RoundRobin { priority }
                if priority < MAX_RT_PRIORITY =>
            {
                2
            }
            _ => 1,
        }
    }

    fn lane_cpu(&self, lane: usize) -> usize {
        self.cpus[lane % self.cpus.len()]
    }

    fn lane_tier(&self, lane: usize) -> usize {
        lane / self.cpus.len()
    }

    fn worker_id(&self, lane: usize) -> String {
        let cpu = self.lane_cpu(lane);
        if self.lane_tier(lane) == 0 {
            format!("cpu{cpu}")
        } else {
            format!("cpu{cpu}-hi")
        }
    }

    fn worker_policy(&self, lane: usize) -> SchedulingPolicy {
        match self.policy {
            SchedulingPolicy::Fifo { priority } if self.lane_tier(lane) > 0 => {
                SchedulingPolicy::Fifo {
                    priority: priority + 1,
                }
            }
            SchedulingPolicy::RoundRobin { priority } if self.lane_tier(lane) > 0 => {
                SchedulingPolicy::RoundRobin {
                    priority: priority + 1,
                }
            }
            policy => policy,
        }
    }

    fn dispatcher(&self) -> CuPlanThread {
        let policy = match self.policy {
            SchedulingPolicy::Fifo { priority } => SchedulingPolicy::Fifo {
                priority: priority + 2,
            },
            SchedulingPolicy::RoundRobin { priority } => SchedulingPolicy::RoundRobin {
                priority: priority + 2,
            },
            policy => policy,
        };
        CuPlanThread { cpu: None, policy }
    }

    /// Window load of the higher-tier lanes on `lane`'s CPU.
    fn higher_load(&self, lane: usize, window_load: &[u64]) -> u64 {
        let cpus = self.cpus.len();
        (0..window_load.len())
            .filter(|&h| h % cpus == lane % cpus && h / cpus > lane / cpus)
            .map(|h| window_load[h])
            .sum()
    }

    /// Whether a lane order respects every zero-lag edge among its own
    /// occurrences, the whole cycle graph stays acyclic, the lanes form a
    /// pipeline (an edge from a later lane back to an earlier one would hold
    /// both to one CopperList at a time), higher-tier lanes carry only
    /// short-deadline work, which is what they preempt for, and a
    /// short-deadline chain stays within one tier, since the lower tier runs
    /// whole cycles behind the higher one.
    fn is_valid(&self, assignment: &Assignment) -> bool {
        let n = self.units.len();
        let lanes = assignment.lanes.len();
        let mut lane_of = vec![usize::MAX; n];
        for (l, lane) in assignment.lanes.iter().enumerate() {
            for &o in lane {
                lane_of[o] = l;
            }
        }
        if lane_of.contains(&usize::MAX) {
            return false;
        }
        if self.tiers() > 1 {
            if (0..n).any(|o| self.lane_tier(lane_of[o]) > 0 && !self.short[o]) {
                return false;
            }
            if self.short_paths.iter().any(|path| {
                path.iter()
                    .any(|&o| self.lane_tier(lane_of[o]) != self.lane_tier(lane_of[path[0]]))
            }) {
                return false;
            }
        }
        let mut incoming = vec![0usize; n];
        let mut succs: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (to, preds) in self.preds.iter().enumerate() {
            for &from in preds {
                succs[from].push(to);
                incoming[to] += 1;
            }
        }
        for lane in &assignment.lanes {
            for pair in lane.windows(2) {
                succs[pair[0]].push(pair[1]);
                incoming[pair[1]] += 1;
            }
        }
        if kahn_count(&succs, incoming) != n {
            return false;
        }
        let mut lane_succs: Vec<Vec<usize>> = vec![Vec::new(); lanes];
        let mut lane_incoming = vec![0usize; lanes];
        for (to, preds) in self.preds.iter().enumerate() {
            for &from in preds {
                let (a, b) = (lane_of[from], lane_of[to]);
                if a != b && !lane_succs[a].contains(&b) {
                    lane_succs[a].push(b);
                    lane_incoming[b] += 1;
                }
            }
        }
        kahn_count(&lane_succs, lane_incoming) == lanes
    }

    /// Response-time analysis per worker, then the cycle's timeline composed
    /// along dependencies, then the constraint-ranked score.
    fn evaluate(&self, assignment: &Assignment) -> Option<Evaluation> {
        let lanes = &assignment.lanes;
        let g = self.copperlist_ns;
        let period = self.cycle_ns;
        let lane_cost: Vec<u64> = lanes
            .iter()
            .map(|lane| lane.iter().map(|&o| self.cost[o]).sum())
            .collect();
        // A lane's busy share of the window is its own work plus that of the
        // higher-priority lanes on its CPU, which preempt it.
        let window_ns = period * self.window_cost.len() as u64;
        let window_load: Vec<u64> = lanes
            .iter()
            .map(|lane| {
                lane.iter()
                    .map(|&o| self.window_cost.iter().map(|cycle| cycle[o]).sum::<u64>())
                    .sum()
            })
            .collect();
        let mut response = Vec::with_capacity(lanes.len());
        let mut rate = Vec::with_capacity(lanes.len());
        let mut load = Vec::with_capacity(lanes.len());
        let mut inflate = Vec::with_capacity(lanes.len());
        for (l, &cost) in lane_cost.iter().enumerate() {
            let r = fixpoint(cost, &[], g, RESPONSE_CAP_PERIODS * period);
            response.push(r);
            let higher = self.higher_load(l, &window_load);
            let busy = (window_load[l] + higher) as f64 / window_ns as f64;
            // A lane full past the headroom's share of the window has no room
            // for the costs' tails and falls behind; measured costs' p95 is
            // two to three times their mean on the Autoware replica.
            let headroom = ((1.0 - self.headroom) / busy.max(1e-9)).min(1.0);
            rate.push(
                match r {
                    Some(r) if r <= period => 1.0,
                    Some(r) => period as f64 / (r + g / 2) as f64,
                    None => 0.0,
                }
                .min(headroom),
            );
            load.push(busy);
            // The typical stretch of a segment under preemption, as in the
            // paper: divided by the share the higher lanes leave.
            inflate.push(if higher < window_ns {
                (window_ns as f64 / (window_ns - higher) as f64).min(RESPONSE_CAP_PERIODS as f64)
            } else {
                RESPONSE_CAP_PERIODS as f64
            });
        }
        // Chains are timed in a cycle where everything fires: a long chain's
        // latency is paid in the cycles it runs, not on average.
        let (starts, ends) = self.timeline(lanes, &self.fired_cost, &inflate)?;
        let (_, expected_ends) = self.timeline(lanes, &self.cost, &inflate)?;
        // In steady state a lane may run behind another by whole cycles; the
        // second window of a two-window timeline gives each lane's cycle
        // time, waits included, and what a chain across lanes pays for it.
        let per_window = self.window_cost.len();
        let n = self.units.len();
        let (window_starts, window_ends) = self.window_timeline(lanes, &inflate, 2)?;
        let last_end = |lane: &[usize], window: usize| {
            let cycle = window * per_window + per_window - 1;
            lane.iter()
                .map(|&o| window_ends[cycle * n + o])
                .max()
                .unwrap_or(0)
        };
        for (l, lane) in lanes.iter().enumerate() {
            let span = last_end(lane, 1) - last_end(lane, 0);
            if span > window_ns {
                rate[l] = rate[l].min(window_ns as f64 / span as f64);
            }
        }
        let mut chain_ratio = Vec::with_capacity(self.chains.len());
        let mut chain_latency = Vec::with_capacity(self.chains.len());
        for (chain, pairs) in self.chains.iter().enumerate() {
            let deadline = u64::from(self.contract_deadline(chain)) * 1_000_000;
            let latency = pairs
                .iter()
                .map(|&(source, sink)| {
                    let steady = (per_window..2 * per_window)
                        .filter(|&cycle| self.fires[cycle % per_window][source])
                        .map(|cycle| {
                            window_ends[cycle * n + sink]
                                .saturating_sub(window_starts[cycle * n + source])
                        })
                        .max()
                        .unwrap_or(0);
                    ends[sink].saturating_sub(starts[source]).max(steady)
                })
                .max()
                .unwrap_or(0);
            chain_ratio.push(latency as f64 / deadline as f64);
            chain_latency.push(latency);
        }
        // Admission is gated by the whole cycle: the slowest lane throttles
        // every source, wherever the source itself runs, and so does the
        // cycle's critical path when fewer cycles than its length fit in
        // flight (`max_in_flight` CopperLists over `k` per cycle).
        let makespan = expected_ends.iter().copied().max().unwrap_or(0);
        let cycles_in_flight =
            f64::from(self.max_in_flight) / f64::from(self.inventory.copperlists_per_cycle.max(1));
        let pipelined = makespan as f64 / cycles_in_flight;
        let cycle_rate =
            rate.iter()
                .copied()
                .fold(1.0f64, f64::min)
                .min(if pipelined <= period as f64 {
                    1.0
                } else {
                    period as f64 / (pipelined + g as f64 / 2.0)
                });
        let source_rate: Vec<f64> = self.sources.iter().map(|_| cycle_rate).collect();
        let rate_deficit: f64 = source_rate.iter().map(|&r| rate_deficit(r)).sum();
        let sum: f64 = chain_ratio.iter().sum();
        let max_load = load.iter().copied().fold(0.0f64, f64::max);
        let score = vec![
            round6(rate_deficit),
            chain_ratio.iter().filter(|&&r| r > 1.0).count() as f64,
            chain_ratio
                .iter()
                .filter(|&&r| r > 1.0 - self.headroom)
                .count() as f64,
            round6(sum),
            round6(max_load),
        ];
        Some(Evaluation {
            score,
            chain_latency,
            response,
            rate,
            load,
            cycle_rate,
        })
    }

    /// Each lane runs its units in order; a unit starts when its lane is free
    /// and every predecessor has ended, and takes its cost stretched by the
    /// lane's `inflate` factor.
    fn timeline(
        &self,
        lanes: &[Vec<usize>],
        cost: &[u64],
        inflate: &[f64],
    ) -> Option<(Vec<u64>, Vec<u64>)> {
        let n = self.units.len();
        let mut starts = vec![0u64; n];
        let mut ends = vec![0u64; n];
        let mut done = vec![false; n];
        let mut lane_free = vec![0u64; lanes.len()];
        let mut progressed = true;
        let mut remaining = n;
        while remaining > 0 && progressed {
            progressed = false;
            for (l, lane) in lanes.iter().enumerate() {
                let Some(&o) = lane.iter().find(|&&o| !done[o]) else {
                    continue;
                };
                if self.preds[o].iter().any(|&p| !done[p]) {
                    continue;
                }
                let release = self.preds[o].iter().map(|&p| ends[p]).max().unwrap_or(0);
                let start = lane_free[l].max(release);
                starts[o] = start;
                ends[o] = start + (cost[o] as f64 * inflate[l]) as u64;
                lane_free[l] = ends[o];
                done[o] = true;
                remaining -= 1;
                progressed = true;
            }
        }
        (remaining == 0).then_some((starts, ends))
    }

    /// The lanes over `windows` windows: cycles released on the grid and
    /// held back by the ring, lanes in order across cycles, a unit waiting
    /// for its zero-lag predecessors of the same cycle, costs from the
    /// firing pattern stretched by `inflate`. Starts and ends are indexed by
    /// `cycle * units + unit`.
    fn window_timeline(
        &self,
        lanes: &[Vec<usize>],
        inflate: &[f64],
        windows: usize,
    ) -> Option<(Vec<u64>, Vec<u64>)> {
        let n = self.units.len();
        let per_window = self.window_cost.len();
        let cycles = per_window * windows;
        let ring =
            (self.max_in_flight / self.inventory.copperlists_per_cycle.max(1)).max(1) as usize;
        let mut starts = vec![0u64; n * cycles];
        let mut ends = vec![0u64; n * cycles];
        let mut done = vec![false; n * cycles];
        let mut remaining = vec![n; cycles];
        let mut cycle_end = vec![0u64; cycles];
        let mut admission = vec![0u64; cycles];
        let mut lane_free = vec![0u64; lanes.len()];
        let mut position = vec![0usize; lanes.len()];
        let mut progressed = true;
        while progressed {
            progressed = false;
            for (l, lane) in lanes.iter().enumerate() {
                while position[l] < lane.len() * cycles {
                    let (cycle, o) = (position[l] / lane.len(), lane[position[l] % lane.len()]);
                    let at = |unit: usize| cycle * n + unit;
                    if cycle >= ring && remaining[cycle - ring] > 0 {
                        break;
                    }
                    if self.preds[o].iter().any(|&p| !done[at(p)]) {
                        break;
                    }
                    if admission[cycle] == 0 {
                        admission[cycle] = (cycle as u64 * self.cycle_ns)
                            .max(if cycle >= ring {
                                cycle_end[cycle - ring]
                            } else {
                                0
                            })
                            .max(if cycle > 0 { admission[cycle - 1] } else { 0 });
                    }
                    let release = self.preds[o]
                        .iter()
                        .map(|&p| ends[at(p)])
                        .fold(admission[cycle], u64::max);
                    let start = lane_free[l].max(release);
                    starts[at(o)] = start;
                    ends[at(o)] = start
                        + (self.window_cost[cycle % per_window][o] as f64 * inflate[l]) as u64;
                    lane_free[l] = ends[at(o)];
                    cycle_end[cycle] = cycle_end[cycle].max(ends[at(o)]);
                    done[at(o)] = true;
                    remaining[cycle] -= 1;
                    position[l] += 1;
                    progressed = true;
                }
            }
        }
        remaining.iter().all(|&r| r == 0).then_some((starts, ends))
    }

    fn contract_deadline(&self, chain: usize) -> u32 {
        self.chain_deadlines[chain]
    }
}

fn round6(value: f64) -> f64 {
    (value * 1e6 + 0.5) as u64 as f64 / 1e6
}

/// The units reachable from `start` over `adj`, `start` included.
fn reachable(adj: &[Vec<usize>], start: usize) -> Vec<bool> {
    let mut seen = vec![false; adj.len()];
    let mut stack = vec![start];
    seen[start] = true;
    while let Some(u) = stack.pop() {
        for &v in &adj[u] {
            if !seen[v] {
                seen[v] = true;
                stack.push(v);
            }
        }
    }
    seen
}

/// The connected component of each unit, ignoring edge direction.
fn components(preds: &[Vec<usize>]) -> Vec<usize> {
    let n = preds.len();
    let mut succs: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (to, from) in preds.iter().enumerate() {
        for &from in from {
            succs[from].push(to);
        }
    }
    let mut component = vec![usize::MAX; n];
    let mut count = 0;
    for start in 0..n {
        if component[start] != usize::MAX {
            continue;
        }
        let mut stack = vec![start];
        component[start] = count;
        while let Some(u) = stack.pop() {
            for &v in preds[u].iter().chain(&succs[u]) {
                if component[v] == usize::MAX {
                    component[v] = count;
                    stack.push(v);
                }
            }
        }
        count += 1;
    }
    component
}

/// How many nodes a topological walk over `succs` reaches; fewer than all
/// means a cycle.
fn kahn_count(succs: &[Vec<usize>], mut incoming: Vec<usize>) -> usize {
    let mut ready: VecDeque<usize> = (0..succs.len()).filter(|&i| incoming[i] == 0).collect();
    let mut seen = 0;
    while let Some(node) = ready.pop_front() {
        seen += 1;
        for &next in &succs[node] {
            incoming[next] -= 1;
            if incoming[next] == 0 {
                ready.push_back(next);
            }
        }
    }
    seen
}

/// Smallest `x = c + sum ceil((x + g) / T) C` over `hp`; `None` past `cap`.
fn fixpoint(c: u64, hp: &[(u64, u64)], g: u64, cap: u64) -> Option<u64> {
    let mut x = c;
    loop {
        let next = c + hp
            .iter()
            .map(|&(cost, period)| (x + g).div_ceil(period.max(1)) * cost)
            .sum::<u64>();
        if next == x {
            return Some(x);
        }
        if next > cap {
            return None;
        }
        x = next;
    }
}

/// The task id inside a step key, `None` for bridge steps.
fn task_of_key(key: &str) -> Option<&str> {
    key.split('|').find_map(|part| part.strip_prefix("task:"))
}

fn is_refine(key: &str) -> bool {
    key.contains("|phase:refine:")
}

/// A unit that no topological order can place, if the unit graph has a cycle.
fn first_unit_on_a_cycle(preds: &[Vec<usize>]) -> Option<usize> {
    let mut incoming: Vec<usize> = preds.iter().map(Vec::len).collect();
    let mut succs: Vec<Vec<usize>> = vec![Vec::new(); preds.len()];
    for (to, from) in preds.iter().enumerate() {
        for &from in from {
            succs[from].push(to);
        }
    }
    let mut ready: VecDeque<usize> = (0..preds.len()).filter(|&i| incoming[i] == 0).collect();
    while let Some(unit) = ready.pop_front() {
        for &next in &succs[unit] {
            incoming[next] -= 1;
            if incoming[next] == 0 {
                ready.push_back(next);
            }
        }
    }
    // Every node left is on a cycle or downstream of one; strip the
    // downstream ones (no remaining successor of theirs is itself left) so a
    // cycle member is named.
    let mut left: Vec<bool> = incoming.iter().map(|&n| n > 0).collect();
    loop {
        let stripped = (0..preds.len()).find(|&i| left[i] && succs[i].iter().all(|&s| !left[s]));
        match stripped {
            Some(i) => left[i] = false,
            None => break,
        }
    }
    (0..preds.len()).find(|&i| left[i])
}

/// A source within `RATE_TOLERANCE` of its period keeps its rate; a deficit
/// counts in steps of `RATE_STEP`, so the tiers below decide between plans
/// whose rates the model cannot tell apart.
pub(super) fn rate_deficit(rate: f64) -> f64 {
    let deficit = (1.0 - RATE_TOLERANCE - rate).max(0.0);
    (deficit / RATE_STEP).ceil() * RATE_STEP
}

/// A deterministic pseudo-random sequence for the search (splitmix64).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
}

impl Model {
    /// The start: a pipeline. Connected components in deadline order, each in
    /// earliest-start order; a component that is short-deadline work
    /// throughout goes whole to the least loaded higher-tier lane, the rest
    /// are cut into one stage per CPU by load.
    fn start_plan(&self) -> Assignment {
        let n = self.units.len();
        let cpus = self.cpus.len();
        let tiers = self.tiers();
        let mut incoming = vec![0usize; n];
        let mut succs: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (to, preds) in self.preds.iter().enumerate() {
            for &from in preds {
                succs[from].push(to);
                incoming[to] += 1;
            }
        }
        let mut ready: VecDeque<usize> = (0..n).filter(|&i| incoming[i] == 0).collect();
        let mut order = Vec::with_capacity(n);
        while let Some(o) = ready.pop_front() {
            order.push(o);
            for &next in &succs[o] {
                incoming[next] -= 1;
                if incoming[next] == 0 {
                    ready.push_back(next);
                }
            }
        }
        let mut asap = vec![0u64; n];
        let mut position = vec![0usize; n];
        for (i, &o) in order.iter().enumerate() {
            position[o] = i;
            asap[o] = self.preds[o]
                .iter()
                .map(|&p| asap[p] + self.cost[p])
                .max()
                .unwrap_or(0);
        }
        let component = components(&self.preds);
        let count = component.iter().max().map_or(0, |&c| c + 1);
        let unit_load = |o: usize| self.window_cost.iter().map(|cycle| cycle[o]).sum::<u64>();
        let mut members: Vec<Vec<usize>> = vec![Vec::new(); count];
        let mut deadline = vec![u64::MAX; count];
        for o in 0..n {
            members[component[o]].push(o);
        }
        for (chain, pairs) in self.chains.iter().enumerate() {
            for &(_, sink) in pairs {
                let c = &mut deadline[component[sink]];
                *c = (*c).min(u64::from(self.chain_deadlines[chain]) * 1_000_000);
            }
        }
        let mut components: Vec<usize> = (0..count).collect();
        for c in &components {
            members[*c].sort_by_key(|&o| (asap[o], position[o]));
        }
        let comp_load: Vec<u64> = members
            .iter()
            .map(|units| units.iter().map(|&o| unit_load(o)).sum())
            .collect();
        components.sort_by_key(|&c| (deadline[c], core::cmp::Reverse(comp_load[c]), c));
        let share = comp_load.iter().sum::<u64>() / cpus as u64;
        let mut lanes = vec![Vec::new(); cpus * tiers];
        let mut lane_load = vec![0u64; cpus * tiers];
        let mut low = Vec::new();
        for &c in &components {
            if tiers > 1 && members[c].iter().all(|&o| self.short[o]) {
                let lane = cpus
                    + (0..cpus)
                        .min_by_key(|&i| (lane_load[cpus + i], i))
                        .expect("at least one CPU");
                lanes[lane].extend(&members[c]);
                lane_load[lane] += comp_load[c];
            } else {
                low.extend(&members[c]);
            }
        }
        let capacity = |stage: usize| {
            share.saturating_sub(if tiers > 1 {
                lane_load[cpus + stage]
            } else {
                0
            })
        };
        let (mut stage, mut acc, mut boundary) = (0, 0u64, capacity(0));
        for o in low {
            let w = unit_load(o);
            if stage + 1 < cpus && acc + w / 2 > boundary {
                stage += 1;
                boundary += capacity(stage);
            }
            lanes[stage].push(o);
            acc += w;
        }
        Assignment { lanes }
    }

    /// One random move: an occurrence moved to another lane and position, or
    /// two neighbours on one lane swapped. Invalid results are rejected by
    /// the caller through `is_valid`.
    fn neighbour(&self, assignment: &Assignment, rng: &mut Rng) -> Assignment {
        let mut lanes = assignment.lanes.clone();
        let occupied: Vec<usize> = (0..lanes.len()).filter(|&l| !lanes[l].is_empty()).collect();
        if occupied.is_empty() {
            return assignment.clone();
        }
        if rng.below(2) == 0 || lanes.len() == 1 {
            let from = occupied[rng.below(occupied.len())];
            let at = rng.below(lanes[from].len());
            let o = lanes[from].remove(at);
            // A unit keeps its tier: a short chain must not straddle tiers.
            let tier = self.lane_tier(from);
            let cpus = self.cpus.len();
            let to = tier * cpus + rng.below(cpus);
            let position = rng.below(lanes[to].len() + 1);
            lanes[to].insert(position, o);
        } else {
            let wide: Vec<usize> = occupied
                .iter()
                .copied()
                .filter(|&l| lanes[l].len() > 1)
                .collect();
            if wide.is_empty() {
                return assignment.clone();
            }
            let lane = wide[rng.below(wide.len())];
            let at = rng.below(lanes[lane].len() - 1);
            lanes[lane].swap(at, at + 1);
        }
        Assignment { lanes }
    }

    /// Hill climbing from the list schedule and from seeded random starts;
    /// every valid assignment evaluated is kept for the final ranking.
    fn search(&self, seed: u64, moves: usize, restarts: usize) -> Vec<(Vec<f64>, Assignment)> {
        let mut seen: BTreeMap<Assignment, Option<Vec<f64>>> = BTreeMap::new();
        let score = |assignment: &Assignment,
                     seen: &mut BTreeMap<Assignment, Option<Vec<f64>>>|
         -> Option<Vec<f64>> {
            if let Some(known) = seen.get(assignment) {
                return known.clone();
            }
            let value = if self.is_valid(assignment) {
                self.evaluate(assignment).map(|evaluation| evaluation.score)
            } else {
                None
            };
            seen.insert(assignment.clone(), value.clone());
            value
        };
        let start = self.start_plan();
        for attempt in 0..restarts.max(1) {
            let mut rng = Rng(seed.wrapping_add(attempt as u64));
            let mut current = if attempt == 0 {
                start.clone()
            } else {
                let mut random = start.clone();
                for _ in 0..self.units.len() * 2 {
                    let candidate = self.neighbour(&random, &mut rng);
                    if self.is_valid(&candidate) {
                        random = candidate;
                    }
                }
                random
            };
            let mut current_score = score(&current, &mut seen);
            let mut best = current.clone();
            let mut best_score = current_score.clone();
            let mut stall = 0;
            for _ in 0..moves {
                let candidate = self.neighbour(&current, &mut rng);
                let candidate_score = score(&candidate, &mut seen);
                let better = match (&candidate_score, &current_score) {
                    (Some(c), Some(cur)) => better_score(c, cur),
                    (Some(_), None) => true,
                    _ => false,
                };
                if better {
                    current = candidate;
                    current_score = candidate_score;
                    stall = 0;
                    if best_score
                        .as_ref()
                        .is_none_or(|b| better_score(current_score.as_ref().unwrap(), b))
                    {
                        best = current.clone();
                        best_score = current_score.clone();
                    }
                } else {
                    stall += 1;
                    if stall >= PATIENCE {
                        current = best.clone();
                        for _ in 0..KICK {
                            let kicked = self.neighbour(&current, &mut rng);
                            if self.is_valid(&kicked) {
                                current = kicked;
                            }
                        }
                        current_score = score(&current, &mut seen);
                        stall = 0;
                    }
                }
            }
        }
        let mut scored: Vec<(Vec<f64>, Assignment)> = seen
            .into_iter()
            .filter_map(|(assignment, score)| score.map(|score| (score, assignment)))
            .collect();
        scored.sort_by(|a, b| {
            if better_score(&a.0, &b.0) {
                core::cmp::Ordering::Less
            } else if better_score(&b.0, &a.0) {
                core::cmp::Ordering::Greater
            } else {
                a.1.cmp(&b.1)
            }
        });
        scored
    }

    /// The plan an assignment describes, with the contract's placement.
    fn materialize(&self, assignment: &Assignment, max_in_flight: u32) -> CuMissionPlan {
        let mut mission = self.inventory.clone();
        mission.max_in_flight = max_in_flight.max(mission.copperlists_per_cycle);
        mission.workers = assignment
            .lanes
            .iter()
            .enumerate()
            .filter(|(_, lane)| !lane.is_empty())
            .map(|(l, lane)| CuPlanWorker {
                id: self.worker_id(l),
                placement: CuPlanPlacement::Thread {
                    cpu: Some(self.lane_cpu(l)),
                    policy: self.worker_policy(l),
                },
                steps: lane
                    .iter()
                    .flat_map(|&u| self.units[u].iter().map(|&o| o as u32))
                    .collect(),
            })
            .collect();
        mission.dispatcher = Some(self.dispatcher());
        mission
    }

    fn prediction(&self, assignment: &Assignment, contract: &CuContract) -> CuPrediction {
        let evaluation = self
            .evaluate(assignment)
            .expect("a ranked assignment evaluates");
        let chains = contract
            .chains
            .iter()
            .zip(&self.chains)
            .enumerate()
            .map(|(index, (chain, _))| {
                let deadline_ns = u64::from(chain.deadline_ms) * 1_000_000;
                let latency_ns = evaluation.chain_latency[index];
                (
                    chain.id.clone(),
                    CuChainPrediction {
                        latency_ns,
                        deadline_ns,
                        ratio: latency_ns as f64 / deadline_ns as f64,
                    },
                )
            })
            .collect();
        let workers = assignment
            .lanes
            .iter()
            .enumerate()
            .filter(|(_, lane)| !lane.is_empty())
            .map(|(l, _)| {
                (
                    self.worker_id(l),
                    CuWorkerPrediction {
                        cpu: self.lane_cpu(l),
                        load: evaluation.load[l],
                        response_ns: evaluation.response[l],
                        rate: evaluation.rate[l],
                    },
                )
            })
            .collect();
        let sources = contract
            .sources
            .iter()
            .map(|source| (source.task.clone(), evaluation.cycle_rate))
            .collect();
        CuPrediction {
            score: evaluation.score,
            chains,
            workers,
            sources,
        }
    }
}

/// Lexicographic comparison, smaller is better.
fn better_score(a: &[f64], b: &[f64]) -> bool {
    for (x, y) in a.iter().zip(b) {
        if x < y {
            return true;
        }
        if x > y {
            return false;
        }
    }
    false
}

/// Whether two scores describe the same candidate for reporting: every tier
/// before the sum tier equal, and the sum tier within `MIN_SEPARATION`.
fn same_candidate(a: &[f64], b: &[f64], sum_tier: usize) -> bool {
    a[..sum_tier] == b[..sum_tier]
        && (a[sum_tier] - b[sum_tier]).abs() <= MIN_SEPARATION * b[sum_tier].abs().max(1e-9)
}

/// Proposes up to `request.candidates` distinct plans for one mission.
///
/// Cycle size, in-flight depth, dispatcher placement, and search budget are
/// derived deterministically. Every returned plan passes [`CuPlan::validate`].
pub fn propose(request: &ProposeRequest<'_>) -> CuResult<Vec<CuCandidate>> {
    if request.candidates == 0 {
        return Err(CuError::from(
            "At least one placement candidate is required",
        ));
    }
    request
        .contract
        .validate(request.config, Some(request.mission))?;
    let graph = request.config.get_graph(Some(request.mission))?;
    let signature = super::graph_signature(graph, Some(request.mission));
    if request.profile.config_signature != signature {
        return Err(CuError::from(format!(
            "The profile was recorded on another graph ({}); this config's mission '{}' is {signature}",
            request.profile.config_signature, request.mission
        )));
    }
    let baseline = CuPlan::from_config(request.config)?;
    let mut cycles = vec![1];
    if request.contract.max_in_flight >= 2
        && graph
            .get_all_nodes()
            .iter()
            .any(|(_, node)| node.get_declared_task_kind() == Some(TaskKind::Stateless))
    {
        cycles.push(2);
    }
    let mut candidates = Vec::new();
    for copperlists_per_cycle in cycles {
        for max_in_flight in copperlists_per_cycle..=request.contract.max_in_flight {
            candidates.extend(propose_shape(
                request,
                copperlists_per_cycle,
                max_in_flight,
                &baseline.concurrent_resources,
            )?);
        }
    }
    candidates.sort_by(|left, right| {
        if better_score(&left.prediction.score, &right.prediction.score) {
            core::cmp::Ordering::Less
        } else if better_score(&right.prediction.score, &left.prediction.score) {
            core::cmp::Ordering::Greater
        } else {
            core::cmp::Ordering::Equal
        }
    });
    let mut chosen: Vec<CuCandidate> = Vec::new();
    for candidate in candidates {
        if chosen.len() >= request.candidates {
            break;
        }
        if chosen
            .iter()
            .any(|known| same_candidate(&candidate.prediction.score, &known.prediction.score, 3))
        {
            continue;
        }
        chosen.push(candidate);
    }
    if chosen.is_empty() {
        return Err(CuError::from(
            "No valid plan was found on this contract's CPUs",
        ));
    }
    Ok(chosen)
}

fn propose_shape(
    request: &ProposeRequest<'_>,
    copperlists_per_cycle: u32,
    max_in_flight: u32,
    concurrent_resources: &[String],
) -> CuResult<Vec<CuCandidate>> {
    let exported = CuPlan::from_config_cyclic_with_resources(
        request.config,
        copperlists_per_cycle,
        concurrent_resources,
    )?;
    let inventory = exported
        .missions
        .get(request.mission)
        .ok_or_else(|| CuError::from(format!("Unknown mission '{}'", request.mission)))?
        .clone();
    let model = Model::new(request, inventory, max_in_flight)?;
    let seed = SEARCH_SEED ^ (u64::from(copperlists_per_cycle) << 32) ^ u64::from(max_in_flight);
    let scored = model.search(seed, SEARCH_MOVES, SEARCH_RESTARTS);
    let mut chosen: Vec<(Vec<f64>, Assignment)> = Vec::new();
    for (score, assignment) in scored {
        if chosen.len() >= request.candidates {
            break;
        }
        if chosen
            .iter()
            .any(|(known, _)| same_candidate(&score, known, 3))
        {
            continue;
        }
        chosen.push((score, assignment));
    }
    let mut candidates = Vec::with_capacity(chosen.len());
    for (_, assignment) in chosen {
        let mission = model.materialize(&assignment, max_in_flight);
        let mut plan = exported.clone();
        plan.missions.insert(request.mission.to_string(), mission);
        let mut prepared = request.config.clone();
        plan.provide_capacity(&mut prepared);
        if plan.validate(&prepared).is_err() {
            continue;
        }
        candidates.push(CuCandidate {
            prediction: model.prediction(&assignment, request.contract),
            plan,
        });
    }
    Ok(candidates)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::CuChain;
    use crate::planner::CuCostStats;
    use crate::planner::CuOperationProfile;
    use crate::planner::CuSourceRate;

    fn config() -> CuConfig {
        CuConfig::deserialize_ron(
            r#"(
            runtime: (rate_target_hz: 100),
            logging: (copperlist_count: 4),
            tasks: [(id: "src", type: "Source"), (id: "left", type: "Left"),
                (id: "right", type: "Right", kind: stateless_task), (id: "sink", type: "Sink")],
            cnx: [(src: "src", dst: "left", msg: "u32"), (src: "src", dst: "right", msg: "u32"),
                (src: "left", dst: "sink", msg: "u32"), (src: "right", dst: "sink", msg: "u32")],
        )"#,
        )
        .unwrap()
    }

    fn contract(cpus: Vec<usize>) -> CuContract {
        CuContract {
            chains: vec![CuChain {
                id: "hot".into(),
                source: "src".into(),
                sink: "sink".into(),
                deadline_ms: 10,
            }],
            sources: vec![CuSourceRate {
                task: "src".into(),
                period_ms: 10,
            }],
            cpus,
            max_in_flight: 2,
            headroom: 0.2,
            worker_policy: SchedulingPolicy::Fair,
        }
    }

    /// `left` and `right` each cost 3 ms every CopperList; the rest is free.
    fn profile() -> CuProfile {
        let graph = config().get_graph(Some("default")).unwrap().clone();
        let mut profile = CuProfile::new(
            crate::planner::graph_signature(&graph, Some("default")),
            "default".into(),
        );
        profile.copperlists = 100;
        profile.window_ns = 1_000_000_000;
        for (task, cost) in [
            ("src", 100_000),
            ("left", 3_000_000),
            ("right", 3_000_000),
            ("sink", 100_000),
        ] {
            let mut samples = vec![cost; 100];
            profile.operations.insert(
                format!("mission:default|task:{task}|phase:whole"),
                CuOperationProfile {
                    fired: CuCostStats::from_samples(&mut samples),
                    skipped: CuCostStats::default(),
                    firing_rate_hz: 100.0,
                    firing: None,
                },
            );
        }
        profile
    }

    fn request<'a>(
        config: &'a CuConfig,
        contract: &'a CuContract,
        profile: &'a CuProfile,
    ) -> ProposeRequest<'a> {
        ProposeRequest {
            config,
            mission: "default",
            contract,
            profile,
            candidates: 3,
        }
    }

    #[test]
    fn parallel_branches_land_on_two_cpus_and_shorten_the_chain() {
        let config = config();
        let profile = profile();
        let one = contract(vec![0]);
        let serial = propose(&request(&config, &one, &profile)).unwrap();
        let two = contract(vec![0, 1]);
        let parallel = propose(&request(&config, &two, &profile)).unwrap();
        let serial_latency = serial[0].prediction.chains["hot"].latency_ns;
        let parallel_latency = parallel[0].prediction.chains["hot"].latency_ns;
        assert_eq!(serial_latency, 6_200_000);
        assert_eq!(parallel_latency, 3_200_000);
        assert_eq!(parallel[0].plan.missions["default"].workers.len(), 2);
        for candidate in serial.iter().chain(&parallel) {
            candidate.plan.validate(&config).unwrap();
            assert_eq!(candidate.prediction.sources["src"], 1.0);
        }
        // Same request, same answer.
        let again = propose(&request(&config, &two, &profile)).unwrap();
        assert_eq!(again, parallel);
    }

    #[test]
    fn a_chain_is_timed_in_a_cycle_where_its_steps_fire() {
        let config = config();
        let mut profile = profile();
        // `left` fires in one CopperList out of ten: cheap on average, still
        // 3 ms in the cycles the chain runs through it.
        let left = profile
            .operations
            .get_mut("mission:default|task:left|phase:whole")
            .unwrap();
        left.fired = CuCostStats::from_samples(&mut [3_000_000; 10]);
        left.skipped = CuCostStats::from_samples(&mut vec![1_000; 90]);
        let one = contract(vec![0]);
        let serial = propose(&request(&config, &one, &profile)).unwrap();
        assert_eq!(serial[0].prediction.chains["hot"].latency_ns, 6_200_000);
        let load = serial[0].prediction.workers["cpu0"].load;
        assert!((0.32..0.36).contains(&load), "{load}");
    }

    #[test]
    fn two_copperlists_per_cycle_let_the_stateless_task_overlap() {
        let config = config();
        let profile = profile();
        let two = contract(vec![0, 1, 2]);
        let candidates = propose(&request(&config, &two, &profile)).unwrap();
        let best = &candidates[0];
        let mission = &best.plan.missions["default"];
        assert_eq!(
            (mission.copperlists_per_cycle, mission.max_in_flight),
            (2, 2)
        );
        best.plan.validate(&config).unwrap();
        // Both CopperLists of the cycle fit in one 20 ms cycle with three CPUs.
        assert!(best.prediction.chains["hot"].latency_ns <= 6_200_000);
        assert!(best.prediction.workers.values().all(|w| w.load < 1.0));
    }

    #[test]
    fn anytime_phases_stay_together_on_one_worker() {
        let config = CuConfig::deserialize_ron(
            r#"(
            runtime: (rate_target_hz: 100),
            logging: (copperlist_count: 2),
            tasks: [(id: "src", type: "Source"),
                (id: "refiner", type: "Refiner", anytime: (max_refines: 2)),
                (id: "sink", type: "Sink")],
            cnx: [(src: "src", dst: "refiner", msg: "u32"), (src: "refiner", dst: "sink", msg: "u32")],
        )"#,
        )
        .unwrap();
        let graph = config.get_graph(Some("default")).unwrap();
        let mut profile = CuProfile::new(
            crate::planner::graph_signature(graph, Some("default")),
            "default".into(),
        );
        profile.copperlists = 100;
        profile.window_ns = 1_000_000_000;
        for (key, cost) in [
            ("mission:default|task:src|phase:whole", 100_000),
            ("mission:default|task:refiner|phase:base", 3_000_000),
            ("mission:default|task:sink|phase:whole", 100_000),
        ] {
            let mut samples = vec![cost; 100];
            profile.operations.insert(
                key.into(),
                CuOperationProfile {
                    fired: CuCostStats::from_samples(&mut samples),
                    skipped: CuCostStats::default(),
                    firing_rate_hz: 100.0,
                    firing: None,
                },
            );
        }
        let mut contract = contract(vec![0, 1]);
        contract.chains[0].sink = "sink".into();
        contract.max_in_flight = 1;
        let candidates = propose(&request(&config, &contract, &profile)).unwrap();
        for candidate in &candidates {
            let mission = &candidate.plan.missions["default"];
            let refine = |key: &str| {
                mission
                    .steps
                    .iter()
                    .position(|s| s.key.contains(key))
                    .unwrap() as u32
            };
            let (base, one, two) = (refine("phase:base"), refine("refine:1"), refine("refine:2"));
            let worker = mission
                .workers
                .iter()
                .find(|w| w.steps.contains(&base))
                .unwrap();
            let at = |step| worker.steps.iter().position(|&s| s == step).unwrap();
            assert_eq!((at(one), at(two)), (at(base) + 1, at(base) + 2));
        }
        assert_eq!(candidates[0].prediction.chains["hot"].latency_ns, 3_200_000);
    }

    #[test]
    fn fifo_workers_come_in_two_tiers_and_lanes_form_a_pipeline() {
        let config = config();
        let profile = profile();
        let mut contract = contract(vec![0, 1]);
        contract.worker_policy = SchedulingPolicy::Fifo { priority: 60 };
        let candidates = propose(&request(&config, &contract, &profile)).unwrap();
        for candidate in &candidates {
            let mission = &candidate.plan.missions["default"];
            let mut lane_of = BTreeMap::new();
            for (l, worker) in mission.workers.iter().enumerate() {
                let priority = match worker.placement {
                    CuPlanPlacement::Thread {
                        policy: SchedulingPolicy::Fifo { priority },
                        ..
                    } => priority,
                    _ => panic!("{}: not a FIFO thread", worker.id),
                };
                assert_eq!(priority, if worker.id.ends_with("-hi") { 61 } else { 60 });
                // The chain is due within its 10 ms period, so its whole
                // graph sits on higher workers.
                assert!(worker.id.ends_with("-hi"), "{}", worker.id);
                for &step in &worker.steps {
                    lane_of.insert(step, l);
                }
            }
            // The lanes must still be a pipeline.
            let mut edges: BTreeSet<(usize, usize)> = mission
                .dependencies
                .iter()
                .filter(|edge| edge.cycle_lag == 0)
                .map(|edge| (lane_of[&edge.from], lane_of[&edge.to]))
                .filter(|(a, b)| a != b)
                .collect();
            let mut lanes: BTreeSet<usize> = lane_of.values().copied().collect();
            while let Some(&first) = lanes.iter().find(|&&l| !edges.iter().any(|&(_, b)| b == l)) {
                lanes.remove(&first);
                edges.retain(|&(a, _)| a != first);
            }
            assert!(lanes.is_empty(), "lanes wait on each other: {edges:?}");
            assert_eq!(
                mission.dispatcher.as_ref().unwrap().policy,
                SchedulingPolicy::Fifo { priority: 62 }
            );
        }
        assert_eq!(candidates[0].prediction.chains["hot"].latency_ns, 3_200_000);
    }

    #[test]
    fn a_missing_measurement_is_an_error() {
        let config = config();
        let mut profile = profile();
        profile
            .operations
            .remove("mission:default|task:left|phase:whole");
        let one = contract(vec![0]);
        assert!(propose(&request(&config, &one, &profile)).is_err());
    }
}
