//! Shared assembly of the exact generated CopperList execution plan.
//!
//! This module is intentionally hidden from the public documentation. It is an
//! implementation contract shared by the proc macro and first-party tooling,
//! not a stable application-facing scheduler API.

use crate::config::{
    BridgeChannelConfigRepresentation, ConfigGraphs, CuConfig, CuGraph, Flavor, Node, NodeId,
};
use crate::curuntime::{
    CuExecutionLoop, CuStepPhase, CuTaskType, compute_runtime_plan, expand_anytime_steps,
    find_task_type_for_id,
};
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

/// Assemble the same synthetic task/bridge graph used by generated runtimes.
#[doc(hidden)]
pub fn assemble_runtime_plan(config: &CuConfig, graph: &CuGraph) -> CuResult<AssembledPlan> {
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

    let mut execution = compute_runtime_plan(&plan_graph)?;
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
}
