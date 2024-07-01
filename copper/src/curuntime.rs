use crate::clock::{ClockProvider, RobotClock};
use crate::config::{CuConfig, NodeId};
use crate::config::{Node, NodeInstanceConfig};
use crate::copperlist::{CopperList, CuListsManager};
use crate::CuResult;
use copper_traits::CopperListPayload;
use petgraph::prelude::*;

/// This is the main structure that will be injected as a member of the Application struct.
/// CT is the tuple of all the tasks in order of execution.
/// CL is the type of the copper list, representing the input/output messages for all the tasks.
pub struct CuRuntime<CT, P: CopperListPayload, const NBCL: usize> {
    /// The tuple of all the tasks in order of execution.
    pub task_instances: CT,

    /// Copper lists hold in order all the input/output messages for all the tasks.
    pub copper_lists: CuListsManager<P, fn(&CopperList<P>), NBCL>,

    /// The base clock the runtime will be using to record time.
    pub clock: RobotClock,
}

/// To be able to share the clock we make the runtime a clock provider.:w
impl<CT, P: CopperListPayload, const NBCL: usize> ClockProvider for CuRuntime<CT, P, NBCL> {
    fn get_clock(&self) -> RobotClock {
        self.clock.clone()
    }
}

/// Small helper function to do nothing for the drop callback.
pub fn no_action<P: CopperListPayload>(_: &CopperList<P>) {}

impl<CT, P: CopperListPayload + 'static, const NBCL: usize> CuRuntime<CT, P, NBCL> {
    pub fn new(
        clock: RobotClock,
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
            copper_lists: CuListsManager::new(no_action), // FIXME: here add the cleanup logic
            clock,
        })
    }
}

/// Copper tasks can be of 3 types:
/// - Source: only producing output messages (usually used for drivers)
/// - Regular: processing input messages and producing output messages, more like compute nodes.
/// - Sink: only consuming input messages (usually used for actuators)
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CuTaskType {
    Source,
    Regular,
    Sink,
}

/// This structure represents a step in the execution plan.
pub struct CuExecutionStep {
    /// NodeId: node id of the task to execute
    pub node_id: NodeId,
    /// Node: node instance
    pub node: Node,
    /// CuTaskType: type of the task
    pub task_type: CuTaskType,
    /// Option<String>: input message type
    pub input_msg_type: Option<String>,
    /// u32: index in the culist of the input message
    pub culist_input_index: Option<u32>,
    /// Option<String>: output message type
    pub output_msg_type: Option<String>,
    /// u32: index in the culist of the output message
    pub culist_output_index: Option<u32>,
}

fn find_output_index_from_nodeid(node_id: NodeId, steps: &Vec<CuExecutionStep>) -> Option<u32> {
    for step in steps {
        if step.node_id == node_id {
            return step.culist_output_index;
        }
    }
    None
}

/// This is the main heuristics to compute an execution plan at compilation time.
/// TODO: Make that heuristic plugable.
pub fn compute_runtime_plan(config: &CuConfig) -> CuResult<Vec<CuExecutionStep>> {
    let mut next_culist_output_index = 0u32;

    // prob not exactly what we want but to get us started
    let mut visitor = Bfs::new(&config.graph, 0.into());

    let mut result = Vec::new();

    while let Some(node) = visitor.next(&config.graph) {
        let id = node.index() as NodeId;
        let node = config.get_node(id).unwrap();
        let mut input_msg_type: Option<String> = None;
        let mut output_msg_type: Option<String> = None;

        let mut culist_input_index: Option<u32> = None;
        let mut culist_output_index: Option<u32> = None;

        // if a node has no parent it means it is a source
        let task_type = if config.graph.neighbors_directed(id.into(), Incoming).count() == 0 {
            output_msg_type = Some(
                config
                    .graph
                    .edge_weight(EdgeIndex::new(config.get_src_edges(id)[0]))
                    .unwrap()
                    .clone(),
            );
            culist_output_index = Some(next_culist_output_index);
            next_culist_output_index += 1;
            CuTaskType::Source
        } else if config.graph.neighbors_directed(id.into(), Outgoing).count() == 0 {
            // this is a Sink.
            input_msg_type = Some(
                config
                    .graph
                    .edge_weight(EdgeIndex::new(config.get_dst_edges(id)[0]))
                    .unwrap()
                    .clone(),
            );
            // get the node from where the message is coming from
            let parent = config
                .graph
                .neighbors_directed(id.into(), Incoming)
                .next()
                .unwrap();
            // Find the source of the incoming message
            culist_input_index = find_output_index_from_nodeid(parent.index() as NodeId, &result);
            CuTaskType::Sink
        } else {
            output_msg_type = Some(
                config
                    .graph
                    .edge_weight(EdgeIndex::new(config.get_src_edges(id)[0]))
                    .unwrap()
                    .clone(),
            );
            culist_output_index = Some(next_culist_output_index);
            next_culist_output_index += 1;
            input_msg_type = Some(
                config
                    .graph
                    .edge_weight(EdgeIndex::new(config.get_dst_edges(id)[0]))
                    .unwrap()
                    .clone(),
            );
            // get the node from where the message is coming from
            let parent = config
                .graph
                .neighbors_directed(id.into(), Incoming)
                .next()
                .unwrap();
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
    use crate::clock::RobotClock;
    use crate::config::Node;
    use crate::cutask::{CuMsg, CuSrcTask};
    use crate::cutask::{CuSinkTask, CuTaskLifecycle};
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
        fn process(
            &mut self,
            clock: &RobotClock,
            _empty_msg: &mut CuMsg<Self::Output>,
        ) -> CuResult<()> {
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
        let runtime =
            CuRuntime::<Tasks, Msgs, 2>::new(RobotClock::default(), &config, tasks_instanciator);
        assert!(runtime.is_ok());
    }
}
