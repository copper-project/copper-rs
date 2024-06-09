use crate::copperlist::CuListsManager;
use crate::config::{CuConfig, NodeId};
use crate::config::{Node, NodeInstanceConfig};
use crate::CuResult;
use crate::clock::RobotClock;
use petgraph::visit::Walker;
use petgraph::prelude::*;

// CT is a tuple of all the tasks
// CL is the type of the copper list
pub struct CuRuntime<CT, CL: Sized + PartialEq, const NBCL: usize> {
    pub task_instances: CT,
    pub copper_lists: CuListsManager<CL, NBCL>,
    pub clock: RobotClock,
}

impl<CT, CL: Sized + PartialEq, const NBCL: usize> CuRuntime<CT, CL, NBCL> {
    pub fn new(
        config: &CuConfig,
        tasks_instanciator: impl Fn(Vec<Option<&NodeInstanceConfig>>) -> CuResult<CT>,
    ) -> CuResult<Self> {
        let all_instances_configs: Vec<Option<&NodeInstanceConfig>> = config
            .get_all_nodes()
            .iter()
            .map(|node_config| node_config.get_instance_config())
            .collect();
        let task_instances = tasks_instanciator(all_instances_configs)?;
        Ok(Self {
            task_instances,
            copper_lists: CuListsManager::new(),
            clock: RobotClock::default(),
        })
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CuTaskType {
    Source,
    Sink,
    Regular,
}

/// Steps give:
/// NodeId: node id of the task to execute
/// Node: node instance
/// CuTaskType: type of the task
/// Option<String>: input message type
/// u32: index in the culist of the input message
/// Option<String>: output message type
/// u32: index in the culist of the output message
pub struct CuExecutionStep {
    pub node_id: NodeId,
    pub node: Node,
    pub task_type: CuTaskType,
    pub input_msg_type: Option<String>,
    pub culist_input_index: Option<u32>,
    pub output_msg_type: Option<String>,
    pub culist_output_index: Option<u32>,
}

fn find_output_index_from_nodeid(
    node_id: NodeId,
    steps: &Vec<CuExecutionStep>,
) -> Option<u32> {
    for step in steps {
        if step.node_id == node_id {
            return step.culist_output_index;
        }
    }
    None
}

pub fn compute_runtime_plan(config: &CuConfig) -> CuResult<Vec<CuExecutionStep>> {
    let mut next_culist_output_index = 0u32;

    // prob not exactly what we want but to get us started
    let mut visitor = petgraph::visit::Bfs::new(&config.graph, 0.into());

    let mut result = Vec::new();

    while let Some(node) = visitor.next(&config.graph) {
        let id = node.index() as NodeId;
        let node = config.get_node(id).unwrap();
        let mut input_msg_type : Option<String> = None;
        let mut output_msg_type : Option<String> = None;

        let mut culist_input_index : Option<u32> = None;
        let mut culist_output_index : Option<u32> = None;

        // if a node has no parent it means it is a source
        let task_type = if config.graph.neighbors_directed(id.into(), petgraph::Direction::Incoming).count() == 0 {
            output_msg_type = Some(config.graph.edge_weight(EdgeIndex::new(config.get_src_edges(id)[0])).unwrap().clone());
            culist_output_index = Some(next_culist_output_index);
            next_culist_output_index += 1;
            CuTaskType::Source
        } else if config.graph.neighbors_directed(id.into(), petgraph::Direction::Outgoing).count() == 0 {
            input_msg_type = Some(config.graph.edge_weight(EdgeIndex::new(config.get_dst_edges(id)[0])).unwrap().clone());
            // get the node from where the message is coming from
            let parent = config.graph.neighbors_directed(id.into(), petgraph::Direction::Incoming).next().unwrap();
            culist_input_index = find_output_index_from_nodeid(parent.index() as NodeId, &result);
            CuTaskType::Sink
        } else {
            output_msg_type = Some(config.graph.edge_weight(EdgeIndex::new(config.get_src_edges(id)[0])).unwrap().clone());
            culist_output_index = Some(next_culist_output_index);
            next_culist_output_index += 1;
            input_msg_type = Some(config.graph.edge_weight(EdgeIndex::new(config.get_dst_edges(id)[0])).unwrap().clone());
            // get the node from where the message is coming from
            let parent = config.graph.neighbors_directed(id.into(), petgraph::Direction::Incoming).next().unwrap();
            culist_input_index = find_output_index_from_nodeid(parent.index() as NodeId, &result);
            CuTaskType::Regular
        };
        let step = CuExecutionStep {
            node_id: id,
            node: node.clone(),
            task_type,
            input_msg_type,
            culist_input_index,
            output_msg_type,
            culist_output_index,
        };
        result.push(step);
    }
    Ok(result)
}

//tests
#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Node;
    use crate::cutask::{CuMsg, CuSrcTask};
    use crate::cutask::{CuSinkTask, CuTaskLifecycle};
    use crate::clock::RobotClock;
    pub struct TestSource {}

    impl CuTaskLifecycle for TestSource {
        fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl CuSrcTask for TestSource {
        type Output = ();
        fn process(&mut self, clock: &RobotClock, _empty_msg: &mut CuMsg<Self::Output>) -> CuResult<()> {
            Ok(())
        }
    }

    pub struct TestSink {}

    impl CuTaskLifecycle for TestSink {
        fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl CuSinkTask for TestSink {
        type Input = ();

        fn process(&mut self, clock: &RobotClock, _input: &mut CuMsg<Self::Input>) -> CuResult<()> {
            Ok(())
        }
    }

    // Those should be generated by the derive macro
    type Tasks = (TestSource, TestSink);
    type Msgs = ((),);

    fn tasks_instanciator(
        all_instances_configs: Vec<Option<&NodeInstanceConfig>>,
    ) -> CuResult<Tasks> {
        Ok((
            TestSource::new(all_instances_configs[0])?,
            TestSink::new(all_instances_configs[1])?,
        ))
    }

    #[test]
    fn test_runtime_instanciation() {
        let mut config = CuConfig::default();
        config.add_node(Node::new("a", "TestSource"));
        config.add_node(Node::new("b", "TestSink"));
        config.connect(0, 1, "()");
        let runtime = CuRuntime::<Tasks, Msgs, 2>::new(&config, tasks_instanciator);
        assert!(runtime.is_ok());
    }
}
