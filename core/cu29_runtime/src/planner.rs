//! Shared assembly of the exact generated CopperList execution plan.
//!
//! This module is intentionally hidden from the public documentation. It is an
//! implementation contract shared by the proc macro and first-party tooling,
//! not a stable application-facing scheduler API.

use crate::config::{
    BridgeChannelConfigRepresentation, ConfigGraphs, CuConfig, CuDirection, CuGraph,
    DEFAULT_MISSION_ID, Flavor, Node, NodeId, PlanHeuristic,
};
use crate::curuntime::{
    CuExecutionLoop, CuExecutionStep, CuExecutionUnit, CuInputMsg, CuOutputPack, CuStepPhase,
    CuTaskType, expand_anytime_steps, find_task_type_for_id,
};
use alloc::boxed::Box;
use alloc::collections::{BTreeMap, BTreeSet, VecDeque};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use cu29_traits::{CuError, CuResult};

/// Default number of preallocated CopperLists compiled into a runtime.
///
/// Code generation and plan tooling share this value so the displayed
/// in-flight bound cannot drift from the generated executor.
#[doc(hidden)]
pub const DEFAULT_COPPERLIST_COUNT: usize = 2;

/// Stable identity for one generated execution entity.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanEntityKind {
    Task {
        original_node_id: NodeId,
        task_index: usize,
    },
    BridgeRx {
        bridge_config_index: usize,
        channel_config_index: usize,
    },
    BridgeTx {
        bridge_config_index: usize,
        channel_config_index: usize,
    },
}

/// Metadata for a node in the synthetic graph consumed by the scheduler.
#[doc(hidden)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlanEntity {
    pub key: String,
    pub label: String,
    pub kind: PlanEntityKind,
}

/// The canonical generated plan plus the stable identity of every plan node.
#[doc(hidden)]
pub struct AssembledPlan {
    pub execution: CuExecutionLoop,
    /// Indexed by the `NodeId` used in `execution`.
    pub entities: Vec<PlanEntity>,
    /// Indexed by the plan `NodeId`; bridge stages contain `None`.
    pub plan_to_original: Vec<Option<NodeId>>,
}

/// The only decision a heuristic makes: a total step order over plan `NodeId`s.
#[doc(hidden)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StepOrder(pub Vec<NodeId>);

/// A plan heuristic after resolution: ids are already mapped to plan `NodeId`s.
///
/// Two-stage on purpose ("parse, don't validate"): the serde-facing
/// [`PlanHeuristic`] carries task names; this resolved form carries plan node
/// ids, so invalid states are unrepresentable once [`ResolvedPlanHeuristic::resolve`]
/// has run.
#[doc(hidden)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResolvedPlanHeuristic {
    TopoBfs,
    /// Task plan node ids in the pinned order (bridge stages are woven in by
    /// [`ResolvedPlanHeuristic::order`]).
    Pinned(Vec<NodeId>),
}

impl ResolvedPlanHeuristic {
    /// Map a config-level [`PlanHeuristic`] onto `plan_graph`, turning task RON
    /// ids into plan node ids. All id errors surface here, before ordering.
    #[doc(hidden)]
    pub fn resolve(
        heuristic: &PlanHeuristic,
        plan_graph: &CuGraph,
        mission: &str,
    ) -> CuResult<Self> {
        match heuristic {
            PlanHeuristic::TopoBfs => Ok(ResolvedPlanHeuristic::TopoBfs),
            PlanHeuristic::Pinned(ids) => Ok(ResolvedPlanHeuristic::Pinned(resolve_pinned_ids(
                plan_graph, ids, mission,
            )?)),
        }
    }

    /// Produce the step order for `plan_graph`. The only pluggable point.
    #[doc(hidden)]
    pub fn order(&self, plan_graph: &CuGraph) -> CuResult<StepOrder> {
        match self {
            ResolvedPlanHeuristic::TopoBfs => topo_bfs_order(plan_graph),
            ResolvedPlanHeuristic::Pinned(tasks) => pinned_order(plan_graph, tasks),
        }
    }
}

fn resolve_pinned_ids(graph: &CuGraph, ids: &[String], mission: &str) -> CuResult<Vec<NodeId>> {
    let mut task_ids: BTreeMap<String, NodeId> = BTreeMap::new();
    let mut bridge_labels: BTreeSet<String> = BTreeSet::new();
    for (node_id, node) in graph.get_all_nodes() {
        match node.get_flavor() {
            Flavor::Task => {
                task_ids.insert(node.get_id(), node_id);
            }
            Flavor::Bridge => {
                bridge_labels.insert(node.get_id());
            }
        }
    }
    let valid_ids = || {
        let mut names: Vec<String> = task_ids.keys().cloned().collect();
        names.sort();
        names.join(", ")
    };

    let mut resolved = Vec::with_capacity(ids.len());
    let mut seen: BTreeSet<NodeId> = BTreeSet::new();
    for id in ids {
        if let Some(&node_id) = task_ids.get(id) {
            if !seen.insert(node_id) {
                return Err(CuError::from(format!(
                    "Mission '{mission}': Pinned plan lists task '{id}' more than once."
                )));
            }
            resolved.push(node_id);
        } else if bridge_labels.contains(id) {
            return Err(CuError::from(format!(
                "Mission '{mission}': Pinned plan lists bridge stage '{id}'; pin only task ids: [{}].",
                valid_ids()
            )));
        } else {
            return Err(CuError::from(format!(
                "Mission '{mission}': Pinned plan lists unknown task '{id}'; valid task ids: [{}].",
                valid_ids()
            )));
        }
    }

    if resolved.len() != task_ids.len() {
        let mut missing: Vec<String> = task_ids
            .iter()
            .filter(|(_, node_id)| !seen.contains(node_id))
            .map(|(name, _)| name.clone())
            .collect();
        missing.sort();
        return Err(CuError::from(format!(
            "Mission '{mission}': Pinned plan must list every task exactly once; missing: [{}].",
            missing.join(", ")
        )));
    }

    Ok(resolved)
}

/// Today's plan walk reduced to a pure ordering decision.
///
/// The order is not a textbook BFS: it emerges from an outer source queue in
/// `node_ids` order, a per-node petgraph BFS, an early abort when a step's
/// inputs are not yet planned, and `handled`-gated neighbor enqueueing. This
/// reproduces that walk exactly, minus the culist/input bookkeeping (which
/// `plan_from_order` now performs).
fn topo_bfs_order(graph: &CuGraph) -> CuResult<StepOrder> {
    let mut order: Vec<NodeId> = Vec::new();
    let mut planned: BTreeSet<NodeId> = BTreeSet::new();

    let mut queue: VecDeque<NodeId> = VecDeque::new();
    for node_id in graph.node_ids() {
        if find_task_type_for_id(graph, node_id)? == CuTaskType::Source {
            queue.push_back(node_id);
        }
    }

    while let Some(start_node) = queue.pop_front() {
        for node_id in graph.bfs_nodes(start_node) {
            if planned.contains(&node_id) {
                continue;
            }
            if topo_bfs_branch(graph, node_id, &mut order, &mut planned)? {
                for neighbor in graph.get_neighbor_ids(node_id, CuDirection::Outgoing) {
                    queue.push_back(neighbor);
                }
            }
        }
    }

    Ok(StepOrder(order))
}

/// One branch of the walk: emit nodes reachable from `starting_point` whose
/// inputs are already planned, aborting at the first step that is not yet ready.
/// Returns whether any node was emitted (the walk's `handled` flag).
fn topo_bfs_branch(
    graph: &CuGraph,
    starting_point: NodeId,
    order: &mut Vec<NodeId>,
    planned: &mut BTreeSet<NodeId>,
) -> CuResult<bool> {
    let mut handled = false;
    for id in graph.bfs_nodes(starting_point) {
        if find_task_type_for_id(graph, id)? != CuTaskType::Source {
            let mut edge_ids = graph.get_dst_edges(id).unwrap_or_default();
            edge_ids.sort();
            let mut ready = true;
            for edge_id in edge_ids {
                let edge = graph
                    .edge(edge_id)
                    .unwrap_or_else(|| panic!("Missing edge {edge_id} for node {id}"));
                let pid = graph
                    .get_node_id_by_name(edge.src.as_str())
                    .unwrap_or_else(|| {
                        panic!("Missing source node '{}' for edge {edge_id}", edge.src)
                    });
                if !planned.contains(&pid) {
                    ready = false;
                    break;
                }
            }
            if !ready {
                return Ok(handled);
            }
        }
        // The historical walk had a re-visit path here that reordered an
        // already-planned step. It is unreachable for validated configs: a node
        // is planned only once all its producers are planned, so planned ⟹ all
        // ancestors planned; branches start only at unplanned nodes and BFS
        // visits each node once, so a branch never reaches a planned node.
        if planned.contains(&id) {
            unreachable!("plan re-visit path reached for node {id}");
        }
        order.push(id);
        planned.insert(id);
        handled = true;
    }
    Ok(handled)
}

/// Complete a pinned task order into a full step order.
///
/// Each bridge rx stage lands immediately before its earliest consumer task in
/// the pinned order; each bridge tx stage immediately after the last producer
/// task feeding it. Stages that do not attach to a pinned task fall to the ends
/// so `check_order` can surface the precedence problem.
fn pinned_order(graph: &CuGraph, pinned_tasks: &[NodeId]) -> CuResult<StepOrder> {
    let mut task_position: BTreeMap<NodeId, usize> = BTreeMap::new();
    for (position, &task) in pinned_tasks.iter().enumerate() {
        task_position.insert(task, position);
    }

    let mut before: BTreeMap<usize, Vec<NodeId>> = BTreeMap::new();
    let mut after: BTreeMap<usize, Vec<NodeId>> = BTreeMap::new();
    let mut leading: Vec<NodeId> = Vec::new();
    let mut trailing: Vec<NodeId> = Vec::new();

    for (node_id, node) in graph.get_all_nodes() {
        if node.get_flavor() != Flavor::Bridge {
            continue;
        }
        match find_task_type_for_id(graph, node_id)? {
            CuTaskType::Source => {
                match graph
                    .get_neighbor_ids(node_id, CuDirection::Outgoing)
                    .into_iter()
                    .filter_map(|consumer| task_position.get(&consumer).copied())
                    .min()
                {
                    Some(pos) => before.entry(pos).or_default().push(node_id),
                    None => leading.push(node_id),
                }
            }
            CuTaskType::Sink => {
                match graph
                    .get_neighbor_ids(node_id, CuDirection::Incoming)
                    .into_iter()
                    .filter_map(|producer| task_position.get(&producer).copied())
                    .max()
                {
                    Some(pos) => after.entry(pos).or_default().push(node_id),
                    None => trailing.push(node_id),
                }
            }
            CuTaskType::Regular => trailing.push(node_id),
        }
    }

    for stages in before.values_mut() {
        stages.sort_unstable();
    }
    for stages in after.values_mut() {
        stages.sort_unstable();
    }
    leading.sort_unstable();
    trailing.sort_unstable();

    let mut order = Vec::new();
    order.append(&mut leading);
    for (position, &task) in pinned_tasks.iter().enumerate() {
        if let Some(stages) = before.get(&position) {
            order.extend(stages.iter().copied());
        }
        order.push(task);
        if let Some(stages) = after.get(&position) {
            order.extend(stages.iter().copied());
        }
    }
    order.append(&mut trailing);

    Ok(StepOrder(order))
}

/// Shared legality gate for a step order over `graph`.
///
/// Rejects unknown ids, duplicate nodes, missing nodes, and precedence
/// violations (a step scheduled before one of its inputs). Errors name the
/// mission and the offending task ids.
#[doc(hidden)]
pub fn check_order(graph: &CuGraph, order: &StepOrder, mission: &str) -> CuResult<()> {
    let mut position: Vec<Option<usize>> = vec![None; graph.node_count()];

    for (index, &node_id) in order.0.iter().enumerate() {
        let slot = position.get_mut(node_id as usize).ok_or_else(|| {
            CuError::from(format!(
                "Mission '{mission}': plan order references unknown node id {node_id}."
            ))
        })?;
        if slot.is_some() {
            return Err(CuError::from(format!(
                "Mission '{mission}': task '{}' appears more than once in the plan order.",
                node_name(graph, node_id)
            )));
        }
        *slot = Some(index);
    }

    let mut missing: Vec<String> = Vec::new();
    for node_id in graph.node_ids() {
        if position[node_id as usize].is_none() {
            missing.push(node_name(graph, node_id));
        }
    }
    if !missing.is_empty() {
        missing.sort();
        return Err(CuError::from(format!(
            "Mission '{mission}': execution plan could not include all nodes. Missing: {}. Check for loopback or missing source connections.",
            missing.join(", ")
        )));
    }

    for edge in graph.edges() {
        let (Some(src), Some(dst)) = (
            graph.get_node_id_by_name(edge.src.as_str()),
            graph.get_node_id_by_name(edge.dst.as_str()),
        ) else {
            continue;
        };
        if position[src as usize] >= position[dst as usize] {
            return Err(CuError::from(format!(
                "Mission '{mission}': task '{}' is scheduled before its input '{}'.",
                node_name(graph, dst),
                node_name(graph, src)
            )));
        }
    }

    Ok(())
}

/// Materialize a validated step order into the concrete execution plan.
///
/// Walks the order once, assigning culist output indices in order and resolving
/// each step's input pack from the already-materialized producers.
#[doc(hidden)]
pub fn plan_from_order(graph: &CuGraph, order: &StepOrder) -> CuResult<CuExecutionLoop> {
    let mut plan: Vec<CuExecutionUnit> = Vec::new();
    let mut next_culist_output_index = 0u32;

    for &id in &order.0 {
        let node_ref = graph
            .get_node(id)
            .ok_or_else(|| CuError::from(format!("Node id {id} not found")))?;
        let task_type = find_task_type_for_id(graph, id)?;
        let mut input_msg_indices_types: Vec<CuInputMsg> = Vec::new();
        let output_msg_pack: Option<CuOutputPack>;

        match task_type {
            CuTaskType::Source => {
                let msg_types = graph.get_node_output_msg_types_by_id(id)?;
                if msg_types.is_empty() {
                    return Err(CuError::from(format!(
                        "Source node '{}' has no declared outputs",
                        node_ref.get_id()
                    )));
                }
                output_msg_pack = Some(CuOutputPack {
                    culist_index: next_culist_output_index,
                    msg_types,
                });
                next_culist_output_index += 1;
            }
            CuTaskType::Sink => {
                collect_step_inputs(graph, id, &plan, &mut input_msg_indices_types)?;
                output_msg_pack = Some(CuOutputPack {
                    culist_index: next_culist_output_index,
                    msg_types: Vec::from(["()".to_string()]),
                });
                next_culist_output_index += 1;
            }
            CuTaskType::Regular => {
                collect_step_inputs(graph, id, &plan, &mut input_msg_indices_types)?;
                let msg_types = graph.get_node_output_msg_types_by_id(id)?;
                if msg_types.is_empty() {
                    return Err(CuError::from(format!(
                        "Regular node '{}' has no declared outputs",
                        node_ref.get_id()
                    )));
                }
                output_msg_pack = Some(CuOutputPack {
                    culist_index: next_culist_output_index,
                    msg_types,
                });
                next_culist_output_index += 1;
            }
        }

        sort_inputs_by_connection_order(&mut input_msg_indices_types);
        plan.push(CuExecutionUnit::Step(Box::new(CuExecutionStep {
            node_id: id,
            node: node_ref.clone(),
            task_type,
            phase: CuStepPhase::default(),
            input_msg_indices_types,
            output_msg_pack,
        })));
    }

    Ok(CuExecutionLoop {
        steps: plan,
        loop_count: None,
    })
}

/// Resolve a step's input pack from the already-materialized producers.
///
/// `check_order` guarantees every producer precedes this node, so each pack is
/// present.
fn collect_step_inputs(
    graph: &CuGraph,
    id: NodeId,
    plan: &[CuExecutionUnit],
    input_msg_indices_types: &mut Vec<CuInputMsg>,
) -> CuResult<()> {
    let mut edge_ids = graph.get_dst_edges(id).unwrap_or_default();
    edge_ids.sort();
    for edge_id in edge_ids {
        let edge = graph
            .edge(edge_id)
            .unwrap_or_else(|| panic!("Missing edge {edge_id} for node {id}"));
        let pid = graph
            .get_node_id_by_name(edge.src.as_str())
            .unwrap_or_else(|| panic!("Missing source node '{}' for edge {edge_id}", edge.src));
        let output_pack = find_output_pack_from_nodeid(pid, plan).ok_or_else(|| {
            CuError::from(format!(
                "Plan materialization: input from node {pid} is not available before node {id}"
            ))
        })?;
        let msg_type = edge.msg.as_str();
        let src_port = output_pack
            .msg_types
            .iter()
            .position(|msg| msg == msg_type)
            .unwrap_or_else(|| {
                panic!("Missing output port for message type '{msg_type}' on node {pid}")
            });
        input_msg_indices_types.push(CuInputMsg {
            culist_index: output_pack.culist_index,
            msg_type: msg_type.to_string(),
            src_port,
            edge_id,
            connection_order: edge.order,
        });
    }
    Ok(())
}

fn find_output_pack_from_nodeid(
    node_id: NodeId,
    steps: &[CuExecutionUnit],
) -> Option<CuOutputPack> {
    for step in steps {
        match step {
            CuExecutionUnit::Loop(loop_unit) => {
                if let Some(output_pack) = find_output_pack_from_nodeid(node_id, &loop_unit.steps) {
                    return Some(output_pack);
                }
            }
            CuExecutionUnit::Step(step) if step.node_id == node_id => {
                return step.output_msg_pack.clone();
            }
            _ => {}
        }
    }
    None
}

/// Preserve the original serialized connection order across missions.
///
/// Edge ids are assigned per mission graph, so they are not stable enough to
/// describe a shared input layout when missions selectively include connections.
fn sort_inputs_by_connection_order(input_msg_indices_types: &mut [CuInputMsg]) {
    input_msg_indices_types.sort_by_key(|input| input.connection_order);
}

fn node_name(graph: &CuGraph, node_id: NodeId) -> String {
    graph
        .get_node(node_id)
        .map(|node| node.get_id())
        .unwrap_or_else(|| format!("node_id_{node_id}"))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ChannelDirection {
    Rx,
    Tx,
}

fn channel_is_used(
    graph: &CuGraph,
    bridge_id: &str,
    channel_id: &str,
    direction: ChannelDirection,
) -> bool {
    graph.edges().any(|connection| match direction {
        ChannelDirection::Rx => {
            connection.src == bridge_id && connection.src_channel.as_deref() == Some(channel_id)
        }
        ChannelDirection::Tx => {
            connection.dst == bridge_id && connection.dst_channel.as_deref() == Some(channel_id)
        }
    })
}

fn inferred_output_name(node: &Node, task_type: CuTaskType) -> String {
    let rust_type = node.get_type();
    if node.anytime().is_some() {
        return format!(
            "<<{rust_type} as cu29::cutask_anytime::CuAnytimeTask>::Output<'static> as cu29::cutask::CuSingleOutputMsg>::Payload"
        );
    }
    let task_trait = match task_type {
        CuTaskType::Source => "cu29::cutask::CuSrcTask",
        CuTaskType::Regular => "cu29::cutask::CuTask",
        CuTaskType::Sink => unreachable!("sinks do not have inferred outputs"),
    };
    format!(
        "<<{rust_type} as {task_trait}>::Output<'static> as cu29::cutask::CuSingleOutputMsg>::Payload"
    )
}

/// Assemble the generated plan for the default mission with the default
/// (`TopoBfs`) heuristic. Kept for tooling that reads a single graph.
#[doc(hidden)]
pub fn assemble_runtime_plan(config: &CuConfig, graph: &CuGraph) -> CuResult<AssembledPlan> {
    assemble_runtime_plan_with(config, graph, &PlanHeuristic::TopoBfs, DEFAULT_MISSION_ID)
}

/// Assemble the same synthetic task/bridge graph used by generated runtimes,
/// ordering it with the given (config-level) heuristic for `mission`.
#[doc(hidden)]
pub fn assemble_runtime_plan_with(
    config: &CuConfig,
    graph: &CuGraph,
    heuristic: &PlanHeuristic,
    mission: &str,
) -> CuResult<AssembledPlan> {
    let mut plan_graph = CuGraph::default();
    let mut entities = Vec::new();
    let mut plan_to_original = Vec::new();
    let mut original_to_plan = Vec::new();
    original_to_plan.resize(graph.node_count(), None);

    let mut task_index = 0usize;
    for (original_node_id, node) in graph.get_all_nodes() {
        if node.get_flavor() != Flavor::Task {
            continue;
        }
        let plan_node_id = plan_graph.add_node(node.clone())?;
        debug_assert_eq!(plan_node_id as usize, entities.len());
        original_to_plan[original_node_id as usize] = Some(plan_node_id);
        plan_to_original.push(Some(original_node_id));
        entities.push(PlanEntity {
            key: format!("task:{}", node.get_id()),
            label: node.get_id(),
            kind: PlanEntityKind::Task {
                original_node_id,
                task_index,
            },
        });
        task_index += 1;
    }

    // A declared task kind permits output inference when an output is marked
    // unconnected without an explicit message type. The macro later parses
    // this projection as Rust syntax when it builds the CopperList tuple.
    for (original_node_id, node) in graph.get_all_nodes() {
        if node.get_flavor() != Flavor::Task || node.get_declared_task_kind().is_none() {
            continue;
        }
        let task_type = find_task_type_for_id(graph, original_node_id)?;
        if task_type == CuTaskType::Sink
            || !graph
                .get_node_output_msg_types_by_id(original_node_id)?
                .is_empty()
        {
            continue;
        }
        let plan_node_id = original_to_plan[original_node_id as usize]
            .expect("task was mirrored into the plan graph");
        let message_type = inferred_output_name(node, task_type);
        plan_graph
            .get_node_mut(plan_node_id)
            .expect("mirrored task is present")
            .add_nc_output(&message_type, usize::MAX);
    }

    // The generated bridge representation creates all used receive stages,
    // then all used transmit stages, for each bridge in config order.
    let mut channel_nodes: Vec<(usize, usize, ChannelDirection, NodeId)> = Vec::new();
    for (bridge_config_index, bridge) in config.bridges.iter().enumerate() {
        if graph.get_node_id_by_name(&bridge.id).is_none() {
            continue;
        }
        for direction in [ChannelDirection::Rx, ChannelDirection::Tx] {
            for (channel_config_index, channel) in bridge.channels.iter().enumerate() {
                let (channel_id, channel_direction) = match channel {
                    BridgeChannelConfigRepresentation::Rx { id, .. } => (id, ChannelDirection::Rx),
                    BridgeChannelConfigRepresentation::Tx { id, .. } => (id, ChannelDirection::Tx),
                };
                if channel_direction != direction
                    || !channel_is_used(graph, &bridge.id, channel_id, direction)
                {
                    continue;
                }

                let direction_label = match direction {
                    ChannelDirection::Rx => "rx",
                    ChannelDirection::Tx => "tx",
                };
                let label = format!("{}::{direction_label}::{channel_id}", bridge.id);
                let synthetic_type = match direction {
                    ChannelDirection::Rx => "__CuBridgeRxChannel",
                    ChannelDirection::Tx => "__CuBridgeTxChannel",
                };
                let mut node = Node::new(&label, synthetic_type);
                node.set_flavor(Flavor::Bridge);
                let plan_node_id = plan_graph.add_node(node)?;
                debug_assert_eq!(plan_node_id as usize, entities.len());
                plan_to_original.push(None);
                entities.push(PlanEntity {
                    key: format!("bridge:{}:{direction_label}:{channel_id}", bridge.id),
                    label,
                    kind: match direction {
                        ChannelDirection::Rx => PlanEntityKind::BridgeRx {
                            bridge_config_index,
                            channel_config_index,
                        },
                        ChannelDirection::Tx => PlanEntityKind::BridgeTx {
                            bridge_config_index,
                            channel_config_index,
                        },
                    },
                });
                channel_nodes.push((
                    bridge_config_index,
                    channel_config_index,
                    direction,
                    plan_node_id,
                ));
            }
        }
    }

    for connection in graph.edges() {
        let src_plan = if let Some(channel_id) = connection.src_channel.as_deref() {
            find_channel_plan_node(
                config,
                &channel_nodes,
                &connection.src,
                channel_id,
                ChannelDirection::Rx,
            )?
        } else {
            let original_id = graph.get_node_id_by_name(&connection.src).ok_or_else(|| {
                CuError::from(format!("Unknown source node '{}'", connection.src))
            })?;
            original_to_plan[original_id as usize].ok_or_else(|| {
                CuError::from(format!("Source node '{}' is not a task", connection.src))
            })?
        };
        let dst_plan = if let Some(channel_id) = connection.dst_channel.as_deref() {
            find_channel_plan_node(
                config,
                &channel_nodes,
                &connection.dst,
                channel_id,
                ChannelDirection::Tx,
            )?
        } else {
            let original_id = graph.get_node_id_by_name(&connection.dst).ok_or_else(|| {
                CuError::from(format!("Unknown destination node '{}'", connection.dst))
            })?;
            original_to_plan[original_id as usize].ok_or_else(|| {
                CuError::from(format!(
                    "Destination node '{}' is not a task",
                    connection.dst
                ))
            })?
        };

        plan_graph
            .connect_ext_with_order(
                src_plan,
                dst_plan,
                &connection.msg,
                connection.missions.clone(),
                None,
                None,
                connection.order,
            )
            .map_err(|error| CuError::from(error.to_string()))?;
    }

    // Choice (order) then bookkeeping (materialize): one legality gate, one
    // shared materializer, for every heuristic and every consumer.
    let resolved = ResolvedPlanHeuristic::resolve(heuristic, &plan_graph, mission)?;
    let order = resolved.order(&plan_graph)?;
    check_order(&plan_graph, &order, mission)?;
    let mut execution = plan_from_order(&plan_graph, &order)?;
    expand_anytime_steps(&mut execution)?;
    Ok(AssembledPlan {
        execution,
        entities,
        plan_to_original,
    })
}

fn find_channel_plan_node(
    config: &CuConfig,
    channel_nodes: &[(usize, usize, ChannelDirection, NodeId)],
    bridge_id: &str,
    channel_id: &str,
    direction: ChannelDirection,
) -> CuResult<NodeId> {
    channel_nodes
        .iter()
        .find_map(
            |(bridge_index, channel_index, candidate_direction, node_id)| {
                let bridge = &config.bridges[*bridge_index];
                let channel = &bridge.channels[*channel_index];
                (bridge.id == bridge_id
                    && channel.id() == channel_id
                    && *candidate_direction == direction)
                    .then_some(*node_id)
            },
        )
        .ok_or_else(|| {
            CuError::from(format!(
                "Bridge channel '{bridge_id}/{channel_id}' is missing from the execution plan"
            ))
        })
}

/// Sorted mission views used by proc-macro generation and visualizers.
#[doc(hidden)]
pub fn mission_graphs(config: &CuConfig) -> Vec<(String, &CuGraph)> {
    match &config.graphs {
        ConfigGraphs::Simple(graph) => vec![("default".to_string(), graph)],
        ConfigGraphs::Missions(graphs) => {
            let mut missions: Vec<_> = graphs
                .iter()
                .map(|(mission, graph)| (mission.clone(), graph))
                .collect();
            missions.sort_by(|left, right| left.0.cmp(&right.0));
            missions
        }
    }
}

/// Return a stable key for a concrete serial step.
#[doc(hidden)]
pub fn step_key(
    mission: &str,
    entity: &PlanEntity,
    phase: CuStepPhase,
    refine_ordinal: Option<u32>,
) -> String {
    let phase = match phase {
        CuStepPhase::Whole => "whole".to_string(),
        CuStepPhase::AnytimeBase => "base".to_string(),
        CuStepPhase::AnytimeRefine => format!("refine:{}", refine_ordinal.unwrap_or(0)),
    };
    format!("mission:{mission}|{}|phase:{phase}", entity.key)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curuntime::CuExecutionUnit;

    fn config(ron: &str) -> CuConfig {
        CuConfig::deserialize_ron(ron).expect("valid planner test config")
    }

    fn step_labels(plan: &AssembledPlan) -> Vec<String> {
        plan.execution
            .steps
            .iter()
            .map(|unit| match unit {
                CuExecutionUnit::Step(step) => plan.entities[step.node_id as usize].label.clone(),
                CuExecutionUnit::Loop(_) => panic!("unexpected nested loop"),
            })
            .collect()
    }

    #[test]
    fn plans_diamond_fan_in_with_stable_input_order() {
        let config = config(
            r#"(
                tasks: [
                    (id: "left", type: "demo::Left"),
                    (id: "right", type: "demo::Right"),
                    (id: "join", type: "demo::Join"),
                    (id: "sink", type: "demo::Sink"),
                ],
                cnx: [
                    (src: "right", dst: "join", msg: "demo::RightMsg"),
                    (src: "left", dst: "join", msg: "demo::LeftMsg"),
                    (src: "join", dst: "sink", msg: "demo::Joined"),
                ],
            )"#,
        );
        let graph = config.get_graph(None).unwrap();
        let plan = assemble_runtime_plan(&config, graph).unwrap();
        assert_eq!(step_labels(&plan), ["left", "right", "join", "sink"]);
        let join = plan
            .execution
            .steps
            .iter()
            .find_map(|unit| match unit {
                CuExecutionUnit::Step(step) if step.node.get_id() == "join" => Some(step),
                _ => None,
            })
            .unwrap();
        assert_eq!(join.input_msg_indices_types.len(), 2);
        assert_eq!(join.input_msg_indices_types[0].msg_type, "demo::RightMsg");
        assert_eq!(join.input_msg_indices_types[1].msg_type, "demo::LeftMsg");
    }

    #[test]
    fn inserts_bridge_rx_and_tx_channel_stages() {
        let config = config(
            r#"(
                tasks: [
                    (id: "task", type: "demo::Task"),
                ],
                bridges: [
                    (
                        id: "radio",
                        type: "demo::Radio",
                        channels: [Rx(id: "incoming"), Tx(id: "outgoing")],
                    ),
                ],
                cnx: [
                    (src: "radio/incoming", dst: "task", msg: "demo::In"),
                    (src: "task", dst: "radio/outgoing", msg: "demo::Out"),
                ],
            )"#,
        );
        let graph = config.get_graph(None).unwrap();
        let plan = assemble_runtime_plan(&config, graph).unwrap();
        assert_eq!(
            step_labels(&plan),
            ["radio::rx::incoming", "task", "radio::tx::outgoing"]
        );
        assert!(matches!(
            plan.entities[1].kind,
            PlanEntityKind::BridgeRx { .. }
        ));
        assert!(matches!(
            plan.entities[2].kind,
            PlanEntityKind::BridgeTx { .. }
        ));
    }

    #[test]
    fn synthesizes_declared_unconnected_output() {
        let config = config(
            r#"(
                tasks: [(id: "generated", type: "demo::Generated", kind: source)],
                cnx: [],
            )"#,
        );
        let graph = config.get_graph(None).unwrap();
        let plan = assemble_runtime_plan(&config, graph).unwrap();
        let CuExecutionUnit::Step(step) = &plan.execution.steps[0] else {
            panic!("expected one generated step")
        };
        let output = step.output_msg_pack.as_ref().unwrap();
        assert_eq!(output.culist_index, 0);
        assert!(output.msg_types[0].contains("CuSingleOutputMsg"));
        assert!(output.msg_types[0].contains("CuSrcTask"));
    }

    // ---- Pinned resolution and ordering ----

    fn pinned_graph() -> CuConfig {
        // radio(rx incoming, tx outgoing) feeding/consuming a small task chain.
        config(
            r#"(
                tasks: [
                    (id: "cam", type: "demo::Cam"),
                    (id: "ekf", type: "demo::Ekf"),
                    (id: "motor", type: "demo::Motor"),
                ],
                bridges: [(
                    id: "radio",
                    type: "demo::Radio",
                    channels: [Rx(id: "incoming"), Tx(id: "outgoing")],
                )],
                cnx: [
                    (src: "radio/incoming", dst: "cam", msg: "demo::In"),
                    (src: "cam", dst: "ekf", msg: "demo::Frame"),
                    (src: "ekf", dst: "motor", msg: "demo::State"),
                    (src: "motor", dst: "radio/outgoing", msg: "demo::Cmd"),
                ],
            )"#,
        )
    }

    #[test]
    fn pinned_plan_weaves_bridge_stages_and_matches_task_order() {
        let config = pinned_graph();
        let graph = config.get_graph(None).unwrap();
        let heuristic = PlanHeuristic::Pinned(vec![
            "cam".to_string(),
            "ekf".to_string(),
            "motor".to_string(),
        ]);
        let plan = assemble_runtime_plan_with(&config, graph, &heuristic, "nominal").unwrap();
        assert_eq!(
            step_labels(&plan),
            [
                "radio::rx::incoming",
                "cam",
                "ekf",
                "motor",
                "radio::tx::outgoing"
            ]
        );
    }

    #[test]
    fn pinned_plan_rejects_bad_id_lists() {
        let config = pinned_graph();
        let graph = config.get_graph(None).unwrap();

        let missing = PlanHeuristic::Pinned(vec!["cam".to_string(), "ekf".to_string()]);
        let err = assemble_runtime_plan_with(&config, graph, &missing, "nominal")
            .err()
            .unwrap();
        assert!(err.to_string().contains("missing"), "{err}");
        assert!(err.to_string().contains("nominal"), "{err}");

        let stage = PlanHeuristic::Pinned(vec![
            "radio::rx::incoming".to_string(),
            "cam".to_string(),
            "ekf".to_string(),
            "motor".to_string(),
        ]);
        let err = assemble_runtime_plan_with(&config, graph, &stage, "nominal")
            .err()
            .unwrap();
        assert!(err.to_string().contains("bridge stage"), "{err}");

        let unknown = PlanHeuristic::Pinned(vec![
            "cam".to_string(),
            "ekf".to_string(),
            "motor".to_string(),
            "ghost".to_string(),
        ]);
        let err = assemble_runtime_plan_with(&config, graph, &unknown, "nominal")
            .err()
            .unwrap();
        assert!(err.to_string().contains("unknown task 'ghost'"), "{err}");

        let dup = PlanHeuristic::Pinned(vec![
            "cam".to_string(),
            "cam".to_string(),
            "ekf".to_string(),
        ]);
        let err = assemble_runtime_plan_with(&config, graph, &dup, "nominal")
            .err()
            .unwrap();
        assert!(err.to_string().contains("more than once"), "{err}");
    }

    #[test]
    fn check_order_flags_precedence_and_missing() {
        let config = build_config(&["s", "k"], &[("s", "k", "m")]);
        let graph = config.get_graph(None).unwrap();
        let s = graph.get_node_id_by_name("s").unwrap();
        let k = graph.get_node_id_by_name("k").unwrap();

        // Consumer before producer.
        let err = check_order(graph, &StepOrder(vec![k, s]), "nominal").unwrap_err();
        assert!(
            err.to_string().contains("scheduled before its input"),
            "{err}"
        );

        // A node left out of the order.
        let err = check_order(graph, &StepOrder(vec![s]), "nominal").unwrap_err();
        assert!(err.to_string().contains("Missing"), "{err}");
    }

    // ---- Differential golden test: legacy walk vs new pipeline ----

    fn build_config(nodes: &[&str], edges: &[(&str, &str, &str)]) -> CuConfig {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let mut ids: BTreeMap<String, NodeId> = BTreeMap::new();
        for &name in nodes {
            let id = graph.add_node(Node::new(name, "demo::T")).unwrap();
            ids.insert(name.to_string(), id);
        }
        for &(src, dst, msg) in edges {
            graph.connect(ids[src], ids[dst], msg).unwrap();
        }
        config
    }

    fn corpus() -> Vec<(String, CuConfig)> {
        // Bridge stages are just synthetic source/sink nodes to the walk, so a
        // corpus of source/regular/sink DAGs covers the bridge case too.
        let mut cases = vec![
            (
                "chain".to_string(),
                build_config(
                    &["s", "r1", "r2", "k"],
                    &[("s", "r1", "m0"), ("r1", "r2", "m1"), ("r2", "k", "m2")],
                ),
            ),
            (
                "fanout_shared_msg".to_string(),
                build_config(
                    &["s", "a", "b", "c"],
                    &[("s", "a", "m"), ("s", "b", "m"), ("s", "c", "n")],
                ),
            ),
            (
                "fanin_multisource".to_string(),
                build_config(
                    &["s1", "s2", "s3", "k"],
                    &[("s2", "k", "m2"), ("s1", "k", "m1"), ("s3", "k", "m3")],
                ),
            ),
            (
                "diamond".to_string(),
                build_config(
                    &["s", "a", "b", "j", "k"],
                    &[
                        ("s", "a", "m0"),
                        ("s", "b", "m1"),
                        ("a", "j", "ma"),
                        ("b", "j", "mb"),
                        ("j", "k", "mj"),
                    ],
                ),
            ),
            (
                "bridge_like".to_string(),
                build_config(
                    &["rx1", "rx2", "r", "tx1", "tx2"],
                    &[
                        ("rx1", "r", "m1"),
                        ("rx2", "r", "m2"),
                        ("r", "tx1", "o1"),
                        ("r", "tx2", "o2"),
                    ],
                ),
            ),
            (
                "multisource_layers".to_string(),
                build_config(
                    &["s1", "s2", "r1", "r2", "k"],
                    &[
                        ("s1", "r1", "a"),
                        ("s2", "r1", "b"),
                        ("s1", "r2", "c"),
                        ("s2", "r2", "d"),
                        ("r1", "k", "e"),
                        ("r2", "k", "f"),
                    ],
                ),
            ),
            (
                "side_branch".to_string(),
                build_config(
                    &["s", "r1", "r2", "r3", "k1", "k2"],
                    &[
                        ("s", "r1", "m0"),
                        ("r1", "r2", "m1"),
                        ("r1", "r3", "m2"),
                        ("r2", "k1", "m3"),
                        ("r3", "k2", "m4"),
                    ],
                ),
            ),
        ];
        // Deterministic layered DAGs with LCG-varied edges.
        for seed in 0u64..6 {
            cases.push((format!("layered_{seed}"), layered_dag(seed)));
        }
        cases
    }

    fn layered_dag(seed: u64) -> CuConfig {
        let layers = [2usize, 3, 3, 2];
        let mut state = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let mut next = || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            state >> 33
        };
        let name = |layer: usize, idx: usize| format!("n{layer}_{idx}");
        let mut nodes = Vec::new();
        for (layer, count) in layers.iter().enumerate() {
            for idx in 0..*count {
                nodes.push(name(layer, idx));
            }
        }
        let node_refs: Vec<&str> = nodes.iter().map(|s| s.as_str()).collect();
        let mut edges: Vec<(String, String, String)> = Vec::new();
        for layer in 0..layers.len() - 1 {
            for from in 0..layers[layer] {
                // guarantee at least one outgoing edge per non-last node
                let mut connected = false;
                for to in 0..layers[layer + 1] {
                    if next() % 2 == 0 || (to == layers[layer + 1] - 1 && !connected) {
                        edges.push((
                            name(layer, from),
                            name(layer + 1, to),
                            format!("m{layer}_{from}_{to}"),
                        ));
                        connected = true;
                    }
                }
            }
            // guarantee at least one incoming edge per next-layer node
            for to in 0..layers[layer + 1] {
                if !edges.iter().any(|(_, d, _)| *d == name(layer + 1, to)) {
                    edges.push((
                        name(layer, 0),
                        name(layer + 1, to),
                        format!("f{layer}_{to}"),
                    ));
                }
            }
        }
        let edge_refs: Vec<(&str, &str, &str)> = edges
            .iter()
            .map(|(s, d, m)| (s.as_str(), d.as_str(), m.as_str()))
            .collect();
        build_config(&node_refs, &edge_refs)
    }

    fn assert_same_plan(name: &str, config: &CuConfig) {
        let graph = config.get_graph(None).unwrap();
        let legacy = compute_runtime_plan_legacy(graph).expect("legacy plan");
        let fresh = crate::curuntime::compute_runtime_plan(graph).expect("new plan");
        assert_eq!(
            legacy.steps.len(),
            fresh.steps.len(),
            "{name}: step count differs"
        );
        for (index, (a, b)) in legacy.steps.iter().zip(fresh.steps.iter()).enumerate() {
            let (CuExecutionUnit::Step(a), CuExecutionUnit::Step(b)) = (a, b) else {
                panic!("{name}: unexpected nested loop");
            };
            assert_eq!(a.node_id, b.node_id, "{name}: step {index} node id");
            assert_eq!(a.phase, b.phase, "{name}: step {index} phase");
            let (oa, ob) = (a.output_msg_pack.as_ref(), b.output_msg_pack.as_ref());
            assert_eq!(
                oa.map(|p| p.culist_index),
                ob.map(|p| p.culist_index),
                "{name}: step {index} culist index"
            );
            assert_eq!(
                oa.map(|p| &p.msg_types),
                ob.map(|p| &p.msg_types),
                "{name}: step {index} output msg types"
            );
            assert_eq!(
                a.input_msg_indices_types.len(),
                b.input_msg_indices_types.len(),
                "{name}: step {index} input arity"
            );
            for (ia, ib) in a
                .input_msg_indices_types
                .iter()
                .zip(b.input_msg_indices_types.iter())
            {
                assert_eq!(ia.culist_index, ib.culist_index, "{name}: input culist");
                assert_eq!(ia.msg_type, ib.msg_type, "{name}: input msg");
                assert_eq!(ia.src_port, ib.src_port, "{name}: input src_port");
                assert_eq!(ia.edge_id, ib.edge_id, "{name}: input edge_id");
                assert_eq!(
                    ia.connection_order, ib.connection_order,
                    "{name}: input connection_order"
                );
            }
        }
    }

    #[test]
    fn topo_bfs_matches_legacy_walk_over_corpus() {
        for (name, config) in corpus() {
            assert_same_plan(&name, &config);
        }
    }

    // Verbatim pre-refactor walk, kept only to prove `TopoBfs` reproduces it.
    fn find_output_pack_from_nodeid_legacy(
        node_id: NodeId,
        steps: &[CuExecutionUnit],
    ) -> Option<CuOutputPack> {
        for step in steps {
            match step {
                CuExecutionUnit::Loop(loop_unit) => {
                    if let Some(pack) =
                        find_output_pack_from_nodeid_legacy(node_id, &loop_unit.steps)
                    {
                        return Some(pack);
                    }
                }
                CuExecutionUnit::Step(step) if step.node_id == node_id => {
                    return step.output_msg_pack.clone();
                }
                _ => {}
            }
        }
        None
    }

    fn plan_tasks_tree_branch_legacy(
        graph: &CuGraph,
        mut next_culist_output_index: u32,
        starting_point: NodeId,
        plan: &mut Vec<CuExecutionUnit>,
    ) -> CuResult<(u32, bool)> {
        let mut handled = false;
        for id in graph.bfs_nodes(starting_point) {
            let node_ref = graph.get_node(id).unwrap();
            let mut input_msg_indices_types: Vec<CuInputMsg> = Vec::new();
            let output_msg_pack: Option<CuOutputPack>;
            let task_type = find_task_type_for_id(graph, id)?;
            match task_type {
                CuTaskType::Source => {
                    let msg_types = graph.get_node_output_msg_types_by_id(id)?;
                    if msg_types.is_empty() {
                        return Err(CuError::from(format!(
                            "Source node '{}' has no declared outputs",
                            node_ref.get_id()
                        )));
                    }
                    output_msg_pack = Some(CuOutputPack {
                        culist_index: next_culist_output_index,
                        msg_types,
                    });
                    next_culist_output_index += 1;
                }
                CuTaskType::Sink => {
                    let mut edge_ids = graph.get_dst_edges(id).unwrap_or_default();
                    edge_ids.sort();
                    for edge_id in edge_ids {
                        let edge = graph
                            .edge(edge_id)
                            .unwrap_or_else(|| panic!("Missing edge {edge_id} for node {id}"));
                        let pid =
                            graph
                                .get_node_id_by_name(edge.src.as_str())
                                .unwrap_or_else(|| {
                                    panic!("Missing source node '{}' for edge {edge_id}", edge.src)
                                });
                        let output_pack = find_output_pack_from_nodeid_legacy(pid, plan);
                        if let Some(output_pack) = output_pack {
                            let msg_type = edge.msg.as_str();
                            let src_port = output_pack
                                .msg_types
                                .iter()
                                .position(|msg| msg == msg_type)
                                .unwrap_or_else(|| {
                                    panic!(
                                        "Missing output port for message type '{msg_type}' on node {pid}"
                                    )
                                });
                            input_msg_indices_types.push(CuInputMsg {
                                culist_index: output_pack.culist_index,
                                msg_type: msg_type.to_string(),
                                src_port,
                                edge_id,
                                connection_order: edge.order,
                            });
                        } else {
                            return Ok((next_culist_output_index, handled));
                        }
                    }
                    output_msg_pack = Some(CuOutputPack {
                        culist_index: next_culist_output_index,
                        msg_types: Vec::from(["()".to_string()]),
                    });
                    next_culist_output_index += 1;
                }
                CuTaskType::Regular => {
                    let mut edge_ids = graph.get_dst_edges(id).unwrap_or_default();
                    edge_ids.sort();
                    for edge_id in edge_ids {
                        let edge = graph
                            .edge(edge_id)
                            .unwrap_or_else(|| panic!("Missing edge {edge_id} for node {id}"));
                        let pid =
                            graph
                                .get_node_id_by_name(edge.src.as_str())
                                .unwrap_or_else(|| {
                                    panic!("Missing source node '{}' for edge {edge_id}", edge.src)
                                });
                        let output_pack = find_output_pack_from_nodeid_legacy(pid, plan);
                        if let Some(output_pack) = output_pack {
                            let msg_type = edge.msg.as_str();
                            let src_port = output_pack
                                .msg_types
                                .iter()
                                .position(|msg| msg == msg_type)
                                .unwrap_or_else(|| {
                                    panic!(
                                        "Missing output port for message type '{msg_type}' on node {pid}"
                                    )
                                });
                            input_msg_indices_types.push(CuInputMsg {
                                culist_index: output_pack.culist_index,
                                msg_type: msg_type.to_string(),
                                src_port,
                                edge_id,
                                connection_order: edge.order,
                            });
                        } else {
                            return Ok((next_culist_output_index, handled));
                        }
                    }
                    let msg_types = graph.get_node_output_msg_types_by_id(id)?;
                    if msg_types.is_empty() {
                        return Err(CuError::from(format!(
                            "Regular node '{}' has no declared outputs",
                            node_ref.get_id()
                        )));
                    }
                    output_msg_pack = Some(CuOutputPack {
                        culist_index: next_culist_output_index,
                        msg_types,
                    });
                    next_culist_output_index += 1;
                }
            }

            sort_inputs_by_connection_order(&mut input_msg_indices_types);
            if let Some(pos) = plan
                .iter()
                .position(|step| matches!(step, CuExecutionUnit::Step(s) if s.node_id == id))
            {
                let mut step = plan.remove(pos);
                if let CuExecutionUnit::Step(ref mut s) = step {
                    s.input_msg_indices_types = input_msg_indices_types;
                }
                plan.push(step);
            } else {
                let step = CuExecutionStep {
                    node_id: id,
                    node: node_ref.clone(),
                    task_type,
                    phase: CuStepPhase::default(),
                    input_msg_indices_types,
                    output_msg_pack,
                };
                plan.push(CuExecutionUnit::Step(Box::new(step)));
            }
            handled = true;
        }
        Ok((next_culist_output_index, handled))
    }

    fn compute_runtime_plan_legacy(graph: &CuGraph) -> CuResult<CuExecutionLoop> {
        let mut plan = Vec::new();
        let mut next_culist_output_index = 0u32;
        let mut queue: VecDeque<NodeId> = VecDeque::new();
        for node_id in graph.node_ids() {
            if find_task_type_for_id(graph, node_id)? == CuTaskType::Source {
                queue.push_back(node_id);
            }
        }
        while let Some(start_node) = queue.pop_front() {
            for node_id in graph.bfs_nodes(start_node) {
                let already = plan
                    .iter()
                    .any(|unit| matches!(unit, CuExecutionUnit::Step(s) if s.node_id == node_id));
                if already {
                    continue;
                }
                let (new_index, handled) = plan_tasks_tree_branch_legacy(
                    graph,
                    next_culist_output_index,
                    node_id,
                    &mut plan,
                )?;
                next_culist_output_index = new_index;
                if !handled {
                    continue;
                }
                for neighbor in graph.get_neighbor_ids(node_id, CuDirection::Outgoing) {
                    queue.push_back(neighbor);
                }
            }
        }
        let mut planned_nodes = BTreeSet::new();
        for unit in &plan {
            if let CuExecutionUnit::Step(step) = unit {
                planned_nodes.insert(step.node_id);
            }
        }
        let mut missing = Vec::new();
        for node_id in graph.node_ids() {
            if !planned_nodes.contains(&node_id) {
                if let Some(node) = graph.get_node(node_id) {
                    missing.push(node.get_id().to_string());
                } else {
                    missing.push(format!("node_id_{node_id}"));
                }
            }
        }
        if !missing.is_empty() {
            missing.sort();
            return Err(CuError::from(format!(
                "Execution plan could not include all nodes. Missing: {}. Check for loopback or missing source connections.",
                missing.join(", ")
            )));
        }
        Ok(CuExecutionLoop {
            steps: plan,
            loop_count: None,
        })
    }
}
