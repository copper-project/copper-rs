//! Portable periodic schedule representation and structural validation.

use super::AssembledPlan;
use super::PlanEntityKind;
use super::explicit_schedule::execution_keys;
use crate::config::CuConfig;
use crate::config::CuGraph;
use crate::config::DEFAULT_BACKGROUND_POOL;
use crate::config::MAX_NICE;
use crate::config::MAX_RT_PRIORITY;
use crate::config::MIN_NICE;
use crate::config::MIN_RT_PRIORITY;
use crate::config::SchedulingPolicy;
use crate::config::TaskKind;
use crate::curuntime::CuExecutionUnit;
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

/// A repeating execution graph for one mission, indexed by CopperList id.
///
/// `steps` is a stable inventory of occurrences. Each worker lists its exact
/// execution order. Dependencies add precedence across workers or cycles.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuMissionPlan {
    /// Number of consecutive CopperLists represented by one schedule cycle.
    pub copperlists_per_cycle: u32,
    /// Largest number of CopperLists admitted but not yet committed.
    pub max_in_flight: u32,
    /// Each concrete process step occurs once per CopperList in this inventory.
    pub steps: Vec<CuPlanStep>,
    /// Ordered work and placement for each worker.
    pub workers: Vec<CuPlanWorker>,
    /// Placement of the thread that admits and commits CopperLists.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dispatcher: Option<CuPlanThread>,
    /// One entry per background task in the mission.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub background: Vec<CuPlanBackground>,
    /// Additional precedence edges.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub dependencies: Vec<CuPlanDependency>,
}

/// One occurrence of a concrete process step within a schedule cycle.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPlanStep {
    /// Opaque stable identity exported from the generated process plan.
    pub key: String,
    /// Zero-based CopperList offset within the repeating cycle.
    pub copperlist: u32,
}

/// One sequential worker. Every inventory index belongs to exactly one worker.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPlanWorker {
    /// Unique worker id; names the thread and appears in diagnostics.
    pub id: String,
    /// Execution context assigned to this worker.
    pub placement: CuPlanPlacement,
    /// Inventory indices in the order this worker executes them.
    pub steps: Vec<u32>,
}

/// Where a worker runs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum CuPlanPlacement {
    /// The application's main thread.
    Main,
    /// A dedicated thread with an optional CPU pin and scheduling policy.
    Thread {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        cpu: Option<usize>,
        #[serde(default)]
        policy: SchedulingPolicy,
    },
}

/// CPU pin and scheduling policy of a plan-owned thread.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPlanThread {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cpu: Option<usize>,
    #[serde(default)]
    pub policy: SchedulingPolicy,
}

/// How one background task's compute runs and is observed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPlanBackground {
    /// Task id.
    pub task: String,
    /// `runtime.thread_pools` entry running its compute.
    pub pool: String,
    /// Largest number of compute jobs in flight at once.
    pub max_running: u32,
    /// Result published by the in-CopperList gateway.
    pub result: CuPlanBackgroundResult,
}

/// Which compute result a background gateway publishes into CopperList `n`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum CuPlanBackgroundResult {
    /// Publish the newest completed result without waiting.
    Sampled,
    /// Wait for the result dispatched for CopperList `n - lag`.
    Lag { lag: u32 },
}

/// Precedence from `from` to `to`, addressed by inventory index.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CuPlanDependency {
    /// Predecessor occurrence index.
    pub from: u32,
    /// Dependent occurrence index.
    pub to: u32,
    /// Number of schedule cycles between predecessor and dependent.
    pub cycle_lag: u32,
}

pub(super) struct PlanShape {
    keys: Vec<String>,
    required: BTreeSet<(usize, usize)>,
    components: Vec<Vec<usize>>,
    background: Vec<CuPlanBackground>,
}

impl PlanShape {
    pub(super) fn new(
        plan: &AssembledPlan,
        config: &CuConfig,
        graph: &CuGraph,
        mission: &str,
        concurrent_resources: &[String],
    ) -> CuResult<Self> {
        let keys = execution_keys(plan, mission)?;
        let mut producers = BTreeMap::new();
        let mut entities: BTreeMap<_, Vec<usize>> = BTreeMap::new();
        let mut components: BTreeMap<_, Vec<usize>> = BTreeMap::new();
        let mut resources: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
        let mut background = Vec::new();

        for (index, unit) in plan.execution.steps.iter().enumerate() {
            let CuExecutionUnit::Step(step) = unit else {
                return Err(CuError::from("Nested execution loops cannot be exported"));
            };
            if let Some(output) = &step.output_msg_pack {
                producers.insert(output.culist_index, index);
            }
            entities.entry(step.node_id).or_default().push(index);
            let (component, bindings) = match plan.entities[step.node_id as usize].kind {
                PlanEntityKind::Task {
                    original_node_id,
                    task_index,
                } => {
                    let node = graph.get_node(original_node_id).ok_or_else(|| {
                        CuError::from(format!("Task node {original_node_id} not found"))
                    })?;
                    if node.is_background() {
                        if entities[&step.node_id].len() == 1 {
                            background.push(CuPlanBackground {
                                task: node.get_id(),
                                pool: node.background_pool().to_string(),
                                max_running: 1,
                                result: CuPlanBackgroundResult::Sampled,
                            });
                        }
                        continue;
                    }
                    if node.get_declared_task_kind() == Some(TaskKind::Stateless) {
                        continue;
                    }
                    ((0, task_index), node.get_resources())
                }
                PlanEntityKind::BridgeRx {
                    bridge_config_index,
                    ..
                }
                | PlanEntityKind::BridgeTx {
                    bridge_config_index,
                    ..
                } => (
                    (1, bridge_config_index),
                    config.bridges[bridge_config_index].resources.as_ref(),
                ),
            };
            components.entry(component).or_default().push(index);
            for resource in bindings.into_iter().flat_map(|map| map.values()) {
                if !concurrent_resources.contains(resource) {
                    resources.entry(resource).or_default().push(index);
                }
            }
        }

        let mut required = BTreeSet::new();
        for indices in entities.values() {
            for pair in indices.windows(2) {
                required.insert((pair[0], pair[1]));
            }
        }
        for (index, unit) in plan.execution.steps.iter().enumerate() {
            let CuExecutionUnit::Step(step) = unit else {
                continue;
            };
            for input in &step.input_msg_indices_types {
                let producer = producers
                    .get(&input.culist_index)
                    .ok_or_else(|| CuError::from("Missing process-step producer"))?;
                required.insert((*producer, index));
            }
        }

        let mut components: Vec<Vec<usize>> = components.into_values().collect();
        for (_, mut steps) in resources {
            steps.sort_unstable();
            steps.dedup();
            let owners = components
                .iter()
                .filter(|component| component.iter().any(|step| steps.contains(step)))
                .count();
            if owners > 1 {
                components.push(steps);
            }
        }

        Ok(Self {
            keys,
            required,
            components,
            background,
        })
    }

    pub(super) fn cyclic_plan(self, copperlists_per_cycle: u32) -> CuResult<CuMissionPlan> {
        if copperlists_per_cycle <= 1 {
            return self.serial_plan();
        }
        let per_cl = self.keys.len();
        let occurrence = |index: usize, offset: u32| step_id(offset as usize * per_cl + index);
        let mut dependencies = BTreeSet::new();
        for offset in 0..copperlists_per_cycle {
            for &(from, to) in &self.required {
                dependencies.insert(CuPlanDependency {
                    from: occurrence(from, offset)?,
                    to: occurrence(to, offset)?,
                    cycle_lag: 0,
                });
            }
        }
        for component in &self.components {
            let (Some(&first), Some(&last)) = (component.first(), component.last()) else {
                continue;
            };
            for offset in 0..copperlists_per_cycle {
                for pair in component.windows(2) {
                    dependencies.insert(CuPlanDependency {
                        from: occurrence(pair[0], offset)?,
                        to: occurrence(pair[1], offset)?,
                        cycle_lag: 0,
                    });
                }
                if offset + 1 < copperlists_per_cycle {
                    dependencies.insert(CuPlanDependency {
                        from: occurrence(last, offset)?,
                        to: occurrence(first, offset + 1)?,
                        cycle_lag: 0,
                    });
                }
            }
            dependencies.insert(CuPlanDependency {
                from: occurrence(last, copperlists_per_cycle - 1)?,
                to: occurrence(first, 0)?,
                cycle_lag: 1,
            });
        }
        let mut steps = Vec::with_capacity(per_cl * copperlists_per_cycle as usize);
        for offset in 0..copperlists_per_cycle {
            steps.extend(self.keys.iter().map(|key| CuPlanStep {
                key: key.clone(),
                copperlist: offset,
            }));
        }
        Ok(CuMissionPlan {
            copperlists_per_cycle,
            max_in_flight: copperlists_per_cycle,
            workers: vec![CuPlanWorker {
                id: "w0".into(),
                placement: CuPlanPlacement::Thread {
                    cpu: None,
                    policy: SchedulingPolicy::Fair,
                },
                steps: (0..steps.len()).map(step_id).collect::<CuResult<_>>()?,
            }],
            steps,
            dispatcher: None,
            background: self.background,
            dependencies: dependencies.into_iter().collect(),
        })
    }

    pub(super) fn serial_plan(self) -> CuResult<CuMissionPlan> {
        let mut dependencies = BTreeSet::new();
        for (from, to) in self.required {
            dependencies.insert(CuPlanDependency {
                from: step_id(from)?,
                to: step_id(to)?,
                cycle_lag: 0,
            });
        }
        for component in &self.components {
            for pair in component.windows(2) {
                dependencies.insert(CuPlanDependency {
                    from: step_id(pair[0])?,
                    to: step_id(pair[1])?,
                    cycle_lag: 0,
                });
            }
            if let (Some(first), Some(last)) = (component.first(), component.last()) {
                dependencies.insert(CuPlanDependency {
                    from: step_id(*last)?,
                    to: step_id(*first)?,
                    cycle_lag: 1,
                });
            }
        }
        let order = (0..self.keys.len())
            .map(step_id)
            .collect::<CuResult<Vec<_>>>()?;
        Ok(CuMissionPlan {
            copperlists_per_cycle: 1,
            max_in_flight: 1,
            steps: self
                .keys
                .into_iter()
                .map(|key| CuPlanStep { key, copperlist: 0 })
                .collect(),
            workers: vec![CuPlanWorker {
                id: "main".into(),
                placement: CuPlanPlacement::Main,
                steps: order,
            }],
            dispatcher: None,
            background: self.background,
            dependencies: dependencies.into_iter().collect(),
        })
    }
}

fn step_id(index: usize) -> CuResult<u32> {
    u32::try_from(index).map_err(|_| CuError::from("Execution plan has too many steps"))
}

fn validate_policy(what: &str, policy: SchedulingPolicy) -> CuResult<()> {
    match policy {
        SchedulingPolicy::Fifo { priority } | SchedulingPolicy::RoundRobin { priority } => {
            if !(MIN_RT_PRIORITY..=MAX_RT_PRIORITY).contains(&priority) {
                return Err(CuError::from(format!(
                    "{what}: real-time priority {priority} is out of range ({MIN_RT_PRIORITY}..={MAX_RT_PRIORITY})"
                )));
            }
        }
        SchedulingPolicy::Nice(nice) => {
            if !(MIN_NICE..=MAX_NICE).contains(&nice) {
                return Err(CuError::from(format!(
                    "{what}: niceness {nice} is out of range ({MIN_NICE}..={MAX_NICE})"
                )));
            }
        }
        SchedulingPolicy::Fair => {}
    }
    Ok(())
}

impl CuMissionPlan {
    /// Whether this is the serial subset.
    pub fn is_serial(&self) -> bool {
        self.copperlists_per_cycle == 1
            && self.max_in_flight == 1
            && self.workers.len() == 1
            && self.workers[0].placement == CuPlanPlacement::Main
    }

    /// Reasons this plan's CopperList contents can depend on timing.
    pub fn nondeterminism(&self) -> Vec<String> {
        self.background
            .iter()
            .filter(|entry| entry.result == CuPlanBackgroundResult::Sampled)
            .map(|entry| {
                format!(
                    "background task '{}' publishes its newest result (sampled)",
                    entry.task
                )
            })
            .collect()
    }

    pub(super) fn serial_keys(&self) -> CuResult<Vec<String>> {
        if !self.is_serial() {
            return Err(CuError::from(
                "Serial keys require one main worker, one CopperList per cycle, and one in flight.",
            ));
        }
        self.workers[0]
            .steps
            .iter()
            .map(|&index| {
                self.steps
                    .get(index as usize)
                    .map(|step| step.key.clone())
                    .ok_or_else(|| CuError::from("Unknown worker step"))
            })
            .collect()
    }

    pub(super) fn validate(&self, config: &CuConfig, shape: &PlanShape) -> CuResult<Vec<usize>> {
        if self.copperlists_per_cycle == 0 {
            return Err(CuError::from("copperlists_per_cycle must be positive"));
        }
        if self.max_in_flight == 0 {
            return Err(CuError::from("max_in_flight must be positive"));
        }
        if self.max_in_flight < self.copperlists_per_cycle {
            return Err(CuError::from(format!(
                "max_in_flight {} is smaller than copperlists_per_cycle {}",
                self.max_in_flight, self.copperlists_per_cycle
            )));
        }
        let storage = config
            .logging
            .as_ref()
            .and_then(|logging| logging.copperlist_count)
            .unwrap_or(super::DEFAULT_COPPERLIST_COUNT);
        if self.max_in_flight as usize > storage {
            return Err(CuError::from(format!(
                "max_in_flight {} exceeds the {storage} preallocated CopperLists",
                self.max_in_flight
            )));
        }
        let expected = shape
            .keys
            .len()
            .checked_mul(self.copperlists_per_cycle as usize)
            .ok_or_else(|| CuError::from("Execution plan step count overflow"))?;
        if self.steps.len() != expected {
            return Err(CuError::from(format!(
                "Expected {expected} process-step occurrences, found {}",
                self.steps.len()
            )));
        }

        let by_key: BTreeMap<_, _> = shape
            .keys
            .iter()
            .enumerate()
            .map(|(index, key)| (key, index))
            .collect();
        let mut occurrences = BTreeMap::new();
        for (index, step) in self.steps.iter().enumerate() {
            let key = *by_key
                .get(&step.key)
                .ok_or_else(|| CuError::from(format!("Unknown process step '{}'", step.key)))?;
            if step.copperlist >= self.copperlists_per_cycle {
                return Err(CuError::from(format!(
                    "CopperList offset {} is outside the schedule cycle",
                    step.copperlist
                )));
            }
            if occurrences.insert((key, step.copperlist), index).is_some() {
                return Err(CuError::from(format!(
                    "Duplicate process step '{}' in CopperList {}",
                    step.key, step.copperlist
                )));
            }
        }

        if self.workers.is_empty() {
            return Err(CuError::from("Execution plan needs at least one worker"));
        }
        let mut assigned = vec![false; self.steps.len()];
        let mut ids = BTreeSet::new();
        let mut edges = vec![Vec::new(); self.steps.len()];
        for worker in &self.workers {
            if worker.id.is_empty() || !ids.insert(worker.id.as_str()) {
                return Err(CuError::from(format!(
                    "Worker id '{}' is empty or used twice",
                    worker.id
                )));
            }
            match &worker.placement {
                CuPlanPlacement::Main => {
                    if !self.is_serial() || self.dispatcher.is_some() {
                        return Err(CuError::from(
                            "A main-thread worker is only valid in a serial plan",
                        ));
                    }
                }
                CuPlanPlacement::Thread { policy, .. } => {
                    validate_policy(&format!("Worker '{}'", worker.id), *policy)?;
                }
            }
            if worker.steps.is_empty() {
                return Err(CuError::from(format!(
                    "Worker '{}' has no steps",
                    worker.id
                )));
            }
            for &index in &worker.steps {
                let seen = assigned
                    .get_mut(index as usize)
                    .ok_or_else(|| CuError::from(format!("Unknown worker step index {index}")))?;
                if core::mem::replace(seen, true) {
                    return Err(CuError::from(format!(
                        "Step index {index} is assigned more than once"
                    )));
                }
            }
            for pair in worker.steps.windows(2) {
                edges[pair[0] as usize].push((pair[1] as usize, 0));
            }
            if let (Some(first), Some(last)) = (worker.steps.first(), worker.steps.last()) {
                edges[*last as usize].push((*first as usize, 1));
            }
        }
        if let Some(dispatcher) = &self.dispatcher {
            validate_policy("Dispatcher", dispatcher.policy)?;
        }
        if assigned.iter().any(|assigned| !assigned) {
            return Err(CuError::from(
                "Every process-step occurrence must be assigned to a worker",
            ));
        }

        let expected_background: BTreeSet<_> =
            shape.background.iter().map(|entry| &entry.task).collect();
        let declared: BTreeSet<_> = self.background.iter().map(|entry| &entry.task).collect();
        if declared.len() != self.background.len() || declared != expected_background {
            return Err(CuError::from(format!(
                "background must list exactly the mission's background tasks: {expected_background:?}"
            )));
        }
        for entry in &self.background {
            let known = entry.pool == DEFAULT_BACKGROUND_POOL
                || config.runtime.as_ref().is_some_and(|runtime| {
                    runtime
                        .thread_pools
                        .iter()
                        .any(|pool| pool.id == entry.pool)
                });
            if !known {
                return Err(CuError::from(format!(
                    "Background task '{}': unknown thread pool '{}'",
                    entry.task, entry.pool
                )));
            }
            if entry.max_running != 1 {
                return Err(CuError::from(format!(
                    "Background task '{}': this runtime requires max_running 1",
                    entry.task
                )));
            }
            if let CuPlanBackgroundResult::Lag { lag } = entry.result
                && lag != 1
            {
                return Err(CuError::from(format!(
                    "Background task '{}': this runtime supports result lag 1",
                    entry.task
                )));
            }
        }

        for dependency in &self.dependencies {
            if dependency.from as usize >= edges.len() || dependency.to as usize >= edges.len() {
                return Err(CuError::from("Dependency references an unknown step index"));
            }
            edges[dependency.from as usize].push((dependency.to as usize, dependency.cycle_lag));
        }
        let order = topological_order(&edges)?;
        let mut rank = vec![0; order.len()];
        for (position, &step) in order.iter().enumerate() {
            rank[step] = position;
        }
        let require = |from: usize, to: usize, lag: u32| -> CuResult<()> {
            if !reaches(&edges, from, to, lag) {
                return Err(CuError::from(format!(
                    "Missing precedence: '{}' (CL {}) must finish before '{}' (CL {}, cycle lag {lag})",
                    self.steps[from].key,
                    self.steps[from].copperlist,
                    self.steps[to].key,
                    self.steps[to].copperlist,
                )));
            }
            Ok(())
        };
        for copperlist in 0..self.copperlists_per_cycle {
            for &(from, to) in &shape.required {
                require(
                    occurrences[&(from, copperlist)],
                    occurrences[&(to, copperlist)],
                    0,
                )?;
            }
        }
        for component in &shape.components {
            let mut endpoints = Vec::new();
            for copperlist in 0..self.copperlists_per_cycle {
                let mut steps: Vec<_> = component
                    .iter()
                    .map(|&key| occurrences[&(key, copperlist)])
                    .collect();
                steps.sort_by_key(|&step| rank[step]);
                for pair in steps.windows(2) {
                    require(pair[0], pair[1], 0)?;
                }
                if let (Some(&first), Some(&last)) = (steps.first(), steps.last()) {
                    endpoints.push((first, last));
                }
            }
            for pair in endpoints.windows(2) {
                require(pair[0].1, pair[1].0, 0)?;
            }
            if let (Some(first), Some(last)) = (endpoints.first(), endpoints.last()) {
                require(last.1, first.0, 1)?;
            }
        }
        Ok(order)
    }

    pub(super) fn layout_keys(&self, order: &[usize]) -> Vec<String> {
        order
            .iter()
            .map(|&index| &self.steps[index])
            .filter(|step| step.copperlist == 0)
            .map(|step| step.key.clone())
            .collect()
    }
}

fn topological_order(edges: &[Vec<(usize, u32)>]) -> CuResult<Vec<usize>> {
    let mut incoming = vec![0usize; edges.len()];
    for outgoing in edges {
        for &(to, lag) in outgoing {
            if lag == 0 {
                incoming[to] += 1;
            }
        }
    }
    let mut ready: VecDeque<_> = incoming
        .iter()
        .enumerate()
        .filter_map(|(index, &count)| (count == 0).then_some(index))
        .collect();
    let mut order = Vec::with_capacity(edges.len());
    while let Some(from) = ready.pop_front() {
        order.push(from);
        for &(to, lag) in &edges[from] {
            if lag == 0 {
                incoming[to] -= 1;
                if incoming[to] == 0 {
                    ready.push_back(to);
                }
            }
        }
    }
    if order.len() != edges.len() {
        return Err(CuError::from(
            "Execution plan contains a dependency/worker-order cycle",
        ));
    }
    Ok(order)
}

fn reaches(edges: &[Vec<(usize, u32)>], from: usize, to: usize, lag: u32) -> bool {
    if edges[from].contains(&(to, lag)) {
        return true;
    }
    let mut visited = vec![false; edges.len() * (lag as usize + 1)];
    let mut ready = vec![(from, 0u32)];
    visited[from] = true;
    while let Some((node, elapsed)) = ready.pop() {
        for &(next, distance) in &edges[node] {
            let Some(next_lag) = elapsed.checked_add(distance).filter(|&sum| sum <= lag) else {
                continue;
            };
            if next == to && next_lag == lag {
                return true;
            }
            let index = next_lag as usize * edges.len() + next;
            if !visited[index] {
                visited[index] = true;
                ready.push((next, next_lag));
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::CuPlan;
    use crate::planner::DistributedSchedule;
    use crate::planner::ExplicitSchedule;
    use crate::planner::LaneOccurrence;
    use crate::planner::assemble_runtime_plan_for_mission;

    fn chain() -> CuConfig {
        CuConfig::deserialize_ron(
            r#"(
                runtime: (thread_pools: [(id: "rt", threads: 4, affinity: [0, 1])]),
                tasks: [(id: "src", type: "Source"), (id: "sink", type: "Sink")],
                cnx: [(src: "src", dst: "sink", msg: "u32")],
            )"#,
        )
        .unwrap()
    }

    fn worker(index: u32, steps: Vec<u32>) -> CuPlanWorker {
        CuPlanWorker {
            id: format!("w{index}"),
            placement: CuPlanPlacement::Thread {
                cpu: Some(index as usize),
                policy: SchedulingPolicy::Fifo {
                    priority: 60 - index as u8,
                },
            },
            steps,
        }
    }

    fn two_cl_pipeline() -> (CuConfig, CuPlan) {
        let config = chain();
        let mut plan = CuPlan::from_config(&config).unwrap();
        let mission = plan.missions.get_mut("default").unwrap();
        let first = mission.steps.clone();
        mission.steps.extend(first.into_iter().map(|mut step| {
            step.copperlist = 1;
            step
        }));
        mission.copperlists_per_cycle = 2;
        mission.max_in_flight = 2;
        mission.workers = vec![worker(0, vec![0, 2]), worker(1, vec![1, 3])];
        mission.dispatcher = Some(CuPlanThread {
            cpu: Some(0),
            policy: SchedulingPolicy::Fifo { priority: 70 },
        });
        mission.dependencies = vec![
            CuPlanDependency {
                from: 0,
                to: 1,
                cycle_lag: 0,
            },
            CuPlanDependency {
                from: 2,
                to: 3,
                cycle_lag: 0,
            },
        ];
        (config, plan)
    }

    #[test]
    fn multicore_plan_round_trips_and_resolves_exact_workers() {
        let (mut config, plan) = two_cl_pipeline();
        plan.validate(&config).unwrap();
        let text = plan.serialize_ron().unwrap();
        assert!(!text.contains("version"));
        let loaded = CuPlan::deserialize_ron(&text).unwrap();
        assert_eq!(loaded, plan);
        ExplicitSchedule::new(loaded).apply(&mut config).unwrap();
        assert_eq!(CuPlan::from_config(&config).unwrap(), plan);

        let assembled =
            assemble_runtime_plan_for_mission(&config, config.get_graph(None).unwrap(), "default")
                .unwrap();
        let schedule = DistributedSchedule::from_assembled(&assembled).unwrap();
        assert_eq!(schedule.worker_lengths, [2, 2]);
        assert_eq!(
            schedule.completion,
            [vec![(0, 1), (1, 1)], vec![(0, 2), (1, 2)]]
        );
        assert!(schedule.dependencies.iter().all(|(from, to, _)| {
            schedule.owner[*from] != schedule.owner[*to] && schedule.publish[*from]
        }));
        let lanes = assembled.lanes.unwrap();
        assert_eq!((lanes.copperlists_per_cycle, lanes.max_in_flight), (2, 2));
        assert_eq!(
            lanes.occurrences,
            [(0, 0), (1, 0), (0, 1), (1, 1)]
                .map(|(step, copperlist)| LaneOccurrence { step, copperlist })
        );
        assert_eq!(lanes.workers[0].occurrences, [0, 2]);
        assert_eq!(lanes.workers[1].occurrences, [1, 3]);
    }

    #[test]
    fn pipeline_and_equivalent_explicit_plan_resolve_identically() {
        let mut pipeline_config = CuConfig::deserialize_ron(
            r#"(
                runtime: (planner: (kind: Pipeline, config: { "max_in_flight": 2 })),
                logging: (copperlist_count: 2),
                tasks: [(id: "src", type: "Source"), (id: "mid", type: "Mid"),
                    (id: "sink", type: "Sink")],
                cnx: [(src: "src", dst: "mid", msg: "u32"),
                    (src: "mid", dst: "sink", msg: "u32")],
            )"#,
        )
        .unwrap();
        let plan = CuPlan::from_config(&pipeline_config).unwrap();
        let pipeline = assemble_runtime_plan_for_mission(
            &pipeline_config,
            pipeline_config.get_graph(None).unwrap(),
            "default",
        )
        .unwrap();
        ExplicitSchedule::new(plan)
            .apply(&mut pipeline_config)
            .unwrap();
        let explicit = assemble_runtime_plan_for_mission(
            &pipeline_config,
            pipeline_config.get_graph(None).unwrap(),
            "default",
        )
        .unwrap();
        assert_eq!(pipeline.lanes, explicit.lanes);
        assert_eq!(
            super::execution_keys(&pipeline, "default").unwrap(),
            super::execution_keys(&explicit, "default").unwrap()
        );
    }

    #[test]
    fn rejects_malformed_inventory_assignment_placement_and_capacity() {
        let (config, plan) = two_cl_pipeline();
        for mutation in 0..15 {
            let mut invalid = plan.clone();
            let mission = invalid.missions.get_mut("default").unwrap();
            match mutation {
                0 => _ = mission.steps.pop(),
                1 => mission.steps[0].key = "unknown".into(),
                2 => mission.steps[0].copperlist = 2,
                3 => mission.steps[2] = mission.steps[0].clone(),
                4 => _ = mission.workers[0].steps.pop(),
                5 => mission.workers[0].steps.push(0),
                6 => mission.workers[0].steps.push(99),
                7 => mission.workers[0].placement = CuPlanPlacement::Main,
                8 => {
                    mission.workers[0].placement = CuPlanPlacement::Thread {
                        cpu: None,
                        policy: SchedulingPolicy::Fifo { priority: 100 },
                    }
                }
                9 => mission.workers[1].id = mission.workers[0].id.clone(),
                10 => mission.dependencies[0].to = 99,
                11 => mission.max_in_flight = 3,
                12 => {
                    mission.dispatcher = Some(CuPlanThread {
                        cpu: None,
                        policy: SchedulingPolicy::Nice(40),
                    })
                }
                13 => mission.max_in_flight = 1,
                _ => mission.copperlists_per_cycle = 0,
            }
            assert!(invalid.validate(&config).is_err(), "mutation {mutation}");
        }
    }

    #[test]
    fn rejects_missing_precedence_cycles_and_state_recurrence() {
        let (config, plan) = two_cl_pipeline();
        let mut missing = plan.clone();
        missing
            .missions
            .get_mut("default")
            .unwrap()
            .dependencies
            .clear();
        assert!(missing.validate(&config).is_err());

        let mut cyclic = plan.clone();
        cyclic
            .missions
            .get_mut("default")
            .unwrap()
            .dependencies
            .push(CuPlanDependency {
                from: 1,
                to: 0,
                cycle_lag: 0,
            });
        assert!(cyclic.validate(&config).is_err());

        let mut recurrence = plan;
        recurrence.missions.get_mut("default").unwrap().workers =
            (0..4).map(|index| worker(index, vec![index])).collect();
        assert!(recurrence.validate(&config).is_err());
        recurrence
            .missions
            .get_mut("default")
            .unwrap()
            .dependencies
            .extend([
                CuPlanDependency {
                    from: 0,
                    to: 2,
                    cycle_lag: 0,
                },
                CuPlanDependency {
                    from: 1,
                    to: 3,
                    cycle_lag: 0,
                },
                CuPlanDependency {
                    from: 2,
                    to: 0,
                    cycle_lag: 1,
                },
                CuPlanDependency {
                    from: 3,
                    to: 1,
                    cycle_lag: 1,
                },
            ]);
        recurrence.validate(&config).unwrap();
    }

    fn fork_join(features_kind: &str) -> CuConfig {
        CuConfig::deserialize_ron(&format!(
            r#"(
                logging: (copperlist_count: 2),
                tasks: [(id: "source", type: "Source"), (id: "filter", type: "Filter"),
                    (id: "features", type: "Features", kind: {features_kind}),
                    (id: "fuse", type: "Fuse")],
                cnx: [(src: "source", dst: "filter", msg: "u32"),
                    (src: "source", dst: "features", msg: "u32"),
                    (src: "filter", dst: "fuse", msg: "u32"),
                    (src: "features", dst: "fuse", msg: "u32")],
            )"#
        ))
        .unwrap()
    }

    fn overlapping_features(config: &CuConfig) -> CuPlan {
        let mut plan = CuPlan::from_config(config).unwrap();
        let mission = plan.missions.get_mut("default").unwrap();
        let first = mission.steps.clone();
        let per_cl = first.len() as u32;
        mission.steps.extend(first.iter().cloned().map(|mut step| {
            step.copperlist = 1;
            step
        }));
        mission.copperlists_per_cycle = 2;
        mission.max_in_flight = 2;
        let index = |task: &str, cl: u32| {
            let needle = format!("task:{task}|");
            first
                .iter()
                .position(|step| step.key.contains(&needle))
                .unwrap() as u32
                + cl * per_cl
        };
        let ordered = [
            ("source", 0),
            ("filter", 0),
            ("source", 1),
            ("filter", 1),
            ("fuse", 0),
            ("fuse", 1),
        ]
        .map(|(task, cl)| index(task, cl));
        mission.workers = vec![
            worker(0, ordered.to_vec()),
            worker(1, vec![index("features", 0)]),
            worker(2, vec![index("features", 1)]),
        ];
        mission.dependencies = (0..2)
            .flat_map(|cl| {
                [("source", "features"), ("features", "fuse")].map(|(from, to)| CuPlanDependency {
                    from: index(from, cl),
                    to: index(to, cl),
                    cycle_lag: 0,
                })
            })
            .collect();
        plan
    }

    #[test]
    fn only_stateless_task_invocations_may_overlap_across_copperlists() {
        let stateless = fork_join("stateless_task");
        overlapping_features(&stateless)
            .validate(&stateless)
            .unwrap();

        let stateful = fork_join("task");
        let error = overlapping_features(&stateful)
            .validate(&stateful)
            .unwrap_err();
        assert!(
            error.to_string().contains("Missing precedence")
                && error.to_string().contains("task:features"),
            "{error}"
        );
    }

    #[test]
    fn shared_resources_require_order_unless_declared_concurrent() {
        let config = CuConfig::deserialize_ron(
            r#"(
                logging: (copperlist_count: 2),
                resources: [(id: "board", provider: "Board")],
                tasks: [(id: "left", type: "Source", kind: source,
                        resources: {"bus": "board.i2c"}),
                    (id: "right", type: "Source", kind: source,
                        resources: {"bus": "board.i2c"}),
                    (id: "alone", type: "Source", kind: source,
                        resources: {"bus": "board.spi"})],
            )"#,
        )
        .unwrap();
        let mut plan = CuPlan::from_config(&config).unwrap();
        let mission = plan.missions.get_mut("default").unwrap();
        let index = |needle: &str| {
            mission
                .steps
                .iter()
                .position(|step| step.key.contains(needle))
                .unwrap() as u32
        };
        let (left, right, alone) = (
            index("task:left|"),
            index("task:right|"),
            index("task:alone|"),
        );
        let shared = |edge: &CuPlanDependency| {
            edge.from != edge.to
                && [left, right].contains(&edge.from)
                && [left, right].contains(&edge.to)
        };
        mission.max_in_flight = 2;
        mission.workers = vec![worker(0, vec![left, alone]), worker(1, vec![right])];
        mission.dependencies.retain(|edge| !shared(edge));
        assert!(plan.validate(&config).is_err());
        plan.concurrent_resources = vec!["board.i2c".into()];
        plan.validate(&config).unwrap();
    }

    #[test]
    fn bridge_channels_require_shared_instance_recurrence() {
        let config = CuConfig::deserialize_ron(
            r#"(
                logging: (copperlist_count: 2),
                tasks: [(id: "left", type: "Sink"), (id: "right", type: "Sink")],
                bridges: [(id: "radio", type: "Radio",
                    channels: [Rx(id: "a"), Rx(id: "b")])],
                cnx: [(src: "radio/a", dst: "left", msg: "u32"),
                    (src: "radio/b", dst: "right", msg: "u32")],
            )"#,
        )
        .unwrap();
        let mut plan = CuPlan::from_config(&config).unwrap();
        let mission = plan.missions.get_mut("default").unwrap();
        let index = |needle: &str| {
            mission
                .steps
                .iter()
                .position(|step| step.key.contains(needle))
                .unwrap() as u32
        };
        let (a, b, left, right) = (
            index("bridge:radio:rx:a|"),
            index("bridge:radio:rx:b|"),
            index("task:left|"),
            index("task:right|"),
        );
        mission.max_in_flight = 2;
        mission.workers = vec![worker(0, vec![a, left]), worker(1, vec![b, right])];
        mission.dependencies = vec![
            CuPlanDependency {
                from: a,
                to: left,
                cycle_lag: 0,
            },
            CuPlanDependency {
                from: b,
                to: right,
                cycle_lag: 0,
            },
        ];
        assert!(plan.validate(&config).is_err());
        plan.missions
            .get_mut("default")
            .unwrap()
            .dependencies
            .extend([
                CuPlanDependency {
                    from: a,
                    to: b,
                    cycle_lag: 0,
                },
                CuPlanDependency {
                    from: b,
                    to: a,
                    cycle_lag: 1,
                },
            ]);
        plan.validate(&config).unwrap();
    }

    #[test]
    fn background_policy_is_complete_and_only_lag_one_is_deterministic() {
        let config = CuConfig::deserialize_ron(
            r#"(
                runtime: (thread_pools: [(id: "vision", threads: 1)]),
                tasks: [(id: "src", type: "Source"),
                    (id: "detector", type: "Detector", background: (pool: "vision")),
                    (id: "sink", type: "Sink")],
                cnx: [(src: "src", dst: "detector", msg: "u32"),
                    (src: "detector", dst: "sink", msg: "u32")],
            )"#,
        )
        .unwrap();
        let plan = CuPlan::from_config(&config).unwrap();
        assert_eq!(plan.missions["default"].nondeterminism().len(), 1);
        let mut deterministic = plan.clone();
        deterministic
            .missions
            .get_mut("default")
            .unwrap()
            .background[0]
            .result = CuPlanBackgroundResult::Lag { lag: 1 };
        deterministic.validate(&config).unwrap();
        assert!(
            deterministic.missions["default"]
                .nondeterminism()
                .is_empty()
        );

        for mutation in 0..5 {
            let mut invalid = plan.clone();
            let entries = &mut invalid.missions.get_mut("default").unwrap().background;
            match mutation {
                0 => entries.clear(),
                1 => entries[0].pool = "missing".into(),
                2 => entries[0].max_running = 0,
                3 => entries[0].max_running = 2,
                _ => entries[0].result = CuPlanBackgroundResult::Lag { lag: 2 },
            }
            assert!(invalid.validate(&config).is_err(), "mutation {mutation}");
        }
    }

    #[test]
    fn missions_and_unknown_schema_fields_are_rejected() {
        let config = CuConfig::deserialize_ron(
            r#"(
                missions: [(id: "alpha"), (id: "beta")],
                tasks: [(id: "src", type: "Source", kind: source)],
            )"#,
        )
        .unwrap();
        let mut plan = CuPlan::from_config(&config).unwrap();
        plan.missions.remove("alpha");
        assert!(plan.validate(&config).is_err());
        assert!(CuPlan::deserialize_ron("(version: 1, missions: {})").is_err());
        assert!(CuPlan::deserialize_ron("(missions: {}, unknown: 0)").is_err());
    }
}
