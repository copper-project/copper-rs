//! An out-of-tree execution planner: `runtime.planner` accepts any crate
//! implementing `CuPlanner`, resolved from the application's `build.rs` via
//! `cu29::planner::emit_plan`.

use cu29::CuResult;
use cu29::config::{ComponentConfig, CuDirection, CuGraph, NodeId};
use cu29::planner::{CuPlanner, StepOrder};
use std::collections::BTreeSet;

/// Kahn's algorithm with an alphabetical tie-break on the node id.
pub struct Alphabetical;

impl CuPlanner for Alphabetical {
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Alphabetical)
    }

    fn plan(&self, graph: &CuGraph) -> CuResult<StepOrder> {
        let mut order: Vec<NodeId> = Vec::new();
        let mut planned: BTreeSet<NodeId> = BTreeSet::new();
        while order.len() < graph.node_count() {
            let next = graph
                .get_all_nodes()
                .into_iter()
                .filter(|(id, _)| !planned.contains(id))
                .filter(|(id, _)| {
                    graph
                        .get_neighbor_ids(*id, CuDirection::Incoming)
                        .iter()
                        .all(|input| planned.contains(input))
                })
                .min_by_key(|(_, node)| node.get_id())
                .map(|(id, _)| id)
                .expect("a task graph is acyclic, so some node is always ready");
            planned.insert(next);
            order.push(next);
        }
        Ok(StepOrder(order))
    }
}
