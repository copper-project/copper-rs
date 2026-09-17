//! Offline example planner that writes a `TaskOrder` configuration fragment.

use cu29::config::{CuDirection, Flavor, read_configuration};
use std::collections::BTreeSet;
use std::path::PathBuf;

fn main() {
    let mut args = std::env::args_os().skip(1);
    let input = PathBuf::from(
        args.next()
            .expect("usage: custom-planner <input.ron> <output.ron>"),
    );
    let output = PathBuf::from(
        args.next()
            .expect("usage: custom-planner <input.ron> <output.ron>"),
    );
    assert!(
        args.next().is_none(),
        "usage: custom-planner <input.ron> <output.ron>"
    );

    let config = read_configuration(input.to_str().expect("input path must be UTF-8"))
        .expect("failed to read Copper config");
    let graph = config
        .get_graph(None)
        .expect("the example planner accepts a single graph");
    let task_count = graph
        .get_all_nodes()
        .into_iter()
        .filter(|(_, node)| node.get_flavor() == Flavor::Task)
        .count();
    let mut planned = BTreeSet::new();
    let mut order = Vec::with_capacity(task_count);
    while order.len() < task_count {
        let (node_id, node) = graph
            .get_all_nodes()
            .into_iter()
            .filter(|(node_id, node)| {
                node.get_flavor() == Flavor::Task && !planned.contains(node_id)
            })
            .filter(|(node_id, _)| {
                graph
                    .get_neighbor_ids(*node_id, CuDirection::Incoming)
                    .into_iter()
                    .all(|input| planned.contains(&input))
            })
            .min_by_key(|(_, node)| node.get_id())
            .expect("a validated task graph always has a ready task");
        planned.insert(node_id);
        order.push(node.get_id());
    }

    let formatted_order = order
        .iter()
        .map(|task| format!("                    {task:?},"))
        .collect::<Vec<_>>()
        .join("\n");
    let fragment = format!(
        "(\n    runtime: (\n        planner: (\n            kind: TaskOrder,\n            config: {{\n                \"order\": [\n{formatted_order}\n                ],\n            }},\n        ),\n    ),\n)"
    );
    std::fs::write(&output, fragment).expect("failed to write TaskOrder config");
}
