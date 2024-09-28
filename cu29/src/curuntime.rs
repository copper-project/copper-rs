//! CuRuntime is the heart of what copper is running on the robot.
//! It is exposed to the user via the `copper_runtime` macro injecting it as a field in their application struct.
//!

use crate::clock::{ClockProvider, RobotClock};
use crate::config::{Cnx, CuConfig, NodeId};
use crate::config::{ComponentConfig, Node};
use crate::copperlist::{CopperList, CopperListState, CuListsManager};
use crate::CuResult;
use cu29_log_derive::debug;
use cu29_traits::CopperListPayload;
use cu29_traits::WriteStream;
use petgraph::prelude::*;
use std::fmt::Debug;

/// This is the main structure that will be injected as a member of the Application struct.
/// CT is the tuple of all the tasks in order of execution.
/// CL is the type of the copper list, representing the input/output messages for all the tasks.
pub struct CuRuntime<CT, P: CopperListPayload, const NBCL: usize> {
    /// The tuple of all the tasks in order of execution.
    pub task_instances: CT,

    /// Copper lists hold in order all the input/output messages for all the tasks.
    pub copper_lists_manager: CuListsManager<P, NBCL>,

    /// The base clock the runtime will be using to record time.
    pub clock: RobotClock, // TODO: remove public at some point

    /// Logger
    logger: Box<dyn WriteStream<CopperList<P>>>,
}

/// To be able to share the clock we make the runtime a clock provider.
impl<CT, P: CopperListPayload, const NBCL: usize> ClockProvider for CuRuntime<CT, P, NBCL> {
    fn get_clock(&self) -> RobotClock {
        self.clock.clone()
    }
}

impl<CT, P: CopperListPayload + 'static, const NBCL: usize> CuRuntime<CT, P, NBCL> {
    pub fn new(
        clock: RobotClock,
        config: &CuConfig,
        tasks_instanciator: impl Fn(Vec<Option<&ComponentConfig>>) -> CuResult<CT>,
        logger: impl WriteStream<CopperList<P>> + 'static,
    ) -> CuResult<Self> {
        let all_instances_configs: Vec<Option<&ComponentConfig>> = config
            .get_all_nodes()
            .iter()
            .map(|node_config| node_config.get_instance_config())
            .collect();
        let task_instances = tasks_instanciator(all_instances_configs)?;

        let runtime = Self {
            task_instances,
            copper_lists_manager: CuListsManager::new(), // placeholder
            clock,
            logger: Box::new(logger),
        };

        Ok(runtime)
    }

    pub fn available_copper_lists(&self) -> usize {
        NBCL - self.copper_lists_manager.len()
    }

    pub fn end_of_processing(&mut self, culistid: u32) {
        debug!("End of processing for CL #{}", culistid);
        let mut is_top = true;
        let mut nb_done = 0;
        self.copper_lists_manager.iter_mut().for_each(|cl| {
            if cl.id == culistid && cl.get_state() == CopperListState::Processing {
                cl.change_state(CopperListState::DoneProcessing);
            }
            // if we have a series of copper lists that are done processing at the top of the circular buffer
            // serialize them all and Free them.
            if is_top && cl.get_state() == CopperListState::DoneProcessing {
                cl.change_state(CopperListState::BeingSerialized);
                debug!("Logging CL #{}", cl.id);
                self.logger.log(&cl).unwrap();
                cl.change_state(CopperListState::Free);
                nb_done += 1;
            } else {
                is_top = false;
            }
        });
        for _ in 0..nb_done {
            let cl = self.copper_lists_manager.pop();
            debug!("Popped CL #{}", cl.unwrap().id);
        }
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

    /// the indices in the copper list of the input messages and their types
    pub input_msg_indices_types: Vec<(u32, String)>,

    /// the index in the copper list of the output message and its type
    pub output_msg_index_type: Option<(u32, String)>,
}

impl Debug for CuExecutionStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(format!("   CuExecutionStep: {}\n", self.node_id).as_str())?;
        f.write_str(format!("       task_type: {:?}\n", self.task_type).as_str())?;
        f.write_str(
            format!(
                "       input_msg_types: {:?}\n",
                self.input_msg_indices_types
            )
            .as_str(),
        )?;
        f.write_str(
            format!("       output_msg_type: {:?}\n", self.output_msg_index_type).as_str(),
        )?;
        Ok(())
    }
}

/// This structure represents a loop in the execution plan.
/// It is used to represent a sequence of Execution units (loop or steps) that are executed
/// multiple times.
/// if loop_count is None, the loop is infinite.
pub struct CuExecutionLoop {
    pub steps: Vec<CuExecutionUnit>,
    pub loop_count: Option<u32>,
}

impl Debug for CuExecutionLoop {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("CuExecutionLoop:\n")?;
        for step in &self.steps {
            match step {
                CuExecutionUnit::Step(step) => {
                    step.fmt(f)?;
                }
                CuExecutionUnit::Loop(l) => {
                    l.fmt(f)?;
                }
            }
        }

        f.write_str(format!("   count: {:?}", self.loop_count).as_str())?;
        Ok(())
    }
}

/// This structure represents a step in the execution plan.
#[derive(Debug)]
pub enum CuExecutionUnit {
    Step(CuExecutionStep),
    Loop(CuExecutionLoop),
}

fn find_output_index_type_from_nodeid(
    node_id: NodeId,
    steps: &Vec<CuExecutionUnit>,
) -> Option<(u32, String)> {
    for step in steps {
        match step {
            CuExecutionUnit::Loop(loop_unit) => {
                if let Some(index) = find_output_index_type_from_nodeid(node_id, &loop_unit.steps) {
                    return Some(index);
                }
            }
            CuExecutionUnit::Step(step) => {
                if step.node_id == node_id {
                    return step.output_msg_index_type.clone();
                }
            }
        }
    }
    None
}

fn find_task_type_for_id(graph: &StableDiGraph<Node, Cnx, NodeId>, node_id: NodeId) -> CuTaskType {
    if graph.neighbors_directed(node_id.into(), Incoming).count() == 0 {
        CuTaskType::Source
    } else if graph.neighbors_directed(node_id.into(), Outgoing).count() == 0 {
        CuTaskType::Sink
    } else {
        CuTaskType::Regular
    }
}

/// Explores a subbranch and build the partial plan out of it.
fn plan_tasks_tree_branch(
    config: &CuConfig,
    mut next_culist_output_index: u32,
    starting_point: NodeId,
    plan: &mut Vec<CuExecutionUnit>,
) -> u32 {
    // prob not exactly what we want but to get us started
    let mut visitor = Bfs::new(&config.graph, starting_point.into());

    while let Some(node) = visitor.next(&config.graph) {
        let id = node.index() as NodeId;
        let node = config.get_node(id).unwrap();

        let mut input_msg_indices_types: Vec<(u32, String)> = Vec::new();
        let mut output_msg_index_type: Option<(u32, String)> = None;

        let task_type = find_task_type_for_id(&config.graph, id);

        match task_type {
            CuTaskType::Source => {
                output_msg_index_type = Some((
                    next_culist_output_index,
                    config
                        .graph
                        .edge_weight(EdgeIndex::new(config.get_src_edges(id)[0]))
                        .unwrap()
                        .msg
                        .clone(),
                ));
                next_culist_output_index += 1;
            }
            CuTaskType::Sink => {
                let parents: Vec<NodeIndex> = config
                    .graph
                    .neighbors_directed(id.into(), Incoming)
                    .collect();

                for parent in parents {
                    let index_type =
                        find_output_index_type_from_nodeid(parent.index() as NodeId, &plan);
                    if let Some(index_type) = index_type {
                        input_msg_indices_types.push(index_type);
                    } else {
                        // here do not add this node yet, wait for the other inputs to do it with all the inputs earliers in the copper list.
                        return next_culist_output_index;
                    }
                }
            }
            CuTaskType::Regular => {
                let parents: Vec<NodeIndex> = config
                    .graph
                    .neighbors_directed(id.into(), Incoming)
                    .collect();

                for parent in parents {
                    let index_type =
                        find_output_index_type_from_nodeid(parent.index() as NodeId, &plan);
                    if let Some(index_type) = index_type {
                        input_msg_indices_types.push(index_type);
                    } else {
                        // here do not add this node yet, wait for the other inputs to do it with all the inputs earliers in the copper list.
                        return next_culist_output_index;
                    }
                }
                output_msg_index_type = Some((
                    next_culist_output_index,
                    config
                        .graph
                        .edge_weight(EdgeIndex::new(config.get_src_edges(id)[0]))
                        .unwrap()
                        .msg
                        .clone(),
                ));
                next_culist_output_index += 1;
            }
        }

        // Sort the input messages by index
        // It means that the tuple presented as input to the merging task
        // depends on the order of *declaration* in the node section of the config file.
        input_msg_indices_types.sort_by(|a, b| a.0.cmp(&b.0));

        // Try to see if we did not already add this node to the plan
        if let Some(pos) = plan.iter().position(|step| {
            if let CuExecutionUnit::Step(ref s) = step {
                s.node_id == id
            } else {
                false
            }
        }) {
            // modify the existing step and put it back in the plan as the current step as it needs this subsequent output.
            let mut step = plan.remove(pos);
            if let CuExecutionUnit::Step(ref mut s) = step {
                s.input_msg_indices_types = input_msg_indices_types;
            }
            plan.push(step);
        } else {
            // ok this is just a new step
            let step = CuExecutionStep {
                node_id: id,
                node: node.clone(),
                task_type,
                input_msg_indices_types,
                output_msg_index_type,
            };
            plan.push(CuExecutionUnit::Step(step));
        }
    }
    next_culist_output_index
}

/// This is the main heuristics to compute an execution plan at compilation time.
/// TODO: Make that heuristic plugable.
pub fn compute_runtime_plan(config: &CuConfig) -> CuResult<CuExecutionLoop> {
    // find all the sources.
    let nodes_to_visit = config
        .graph
        .node_indices()
        .filter(|node_id| {
            let id = node_id.index() as NodeId;
            let task_type = find_task_type_for_id(&config.graph, id);
            task_type == CuTaskType::Source
        })
        .collect::<Vec<NodeIndex>>();

    let mut next_culist_output_index = 0u32;
    let mut plan: Vec<CuExecutionUnit> = Vec::new();

    for node_index in &nodes_to_visit {
        next_culist_output_index = plan_tasks_tree_branch(
            config,
            next_culist_output_index,
            node_index.index() as NodeId,
            &mut plan,
        );
    }

    Ok(CuExecutionLoop {
        steps: plan,
        loop_count: None, // if in not unit testing, the main loop is infinite
    })
}

//tests
#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::RobotClock;
    use crate::config::Node;
    use crate::cutask::{CuSinkTask, CuTaskLifecycle};
    use crate::cutask::{CuSrcTask, Freezable};
    use bincode::Encode;

    pub struct TestSource {}

    impl Freezable for TestSource {}

    impl CuTaskLifecycle for TestSource {
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl CuSrcTask<'_> for TestSource {
        type Output = ();
        fn process(&mut self, _clock: &RobotClock, _empty_msg: Self::Output) -> CuResult<()> {
            Ok(())
        }
    }

    pub struct TestSink {}

    impl Freezable for TestSink {}

    impl CuTaskLifecycle for TestSink {
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }
    }

    impl CuSinkTask<'_> for TestSink {
        type Input = ();

        fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
            Ok(())
        }
    }

    // Those should be generated by the derive macro
    type Tasks = (TestSource, TestSink);
    type Msgs = ((),);

    fn tasks_instanciator(all_instances_configs: Vec<Option<&ComponentConfig>>) -> CuResult<Tasks> {
        Ok((
            TestSource::new(all_instances_configs[0])?,
            TestSink::new(all_instances_configs[1])?,
        ))
    }

    #[derive(Debug)]
    struct FakeWriter {}

    impl<E: Encode> WriteStream<E> for FakeWriter {
        fn log(&mut self, _obj: &E) -> CuResult<()> {
            Ok(())
        }
    }

    #[test]
    fn test_runtime_instanciation() {
        let mut config = CuConfig::default();
        config.add_node(Node::new("a", "TestSource"));
        config.add_node(Node::new("b", "TestSink"));
        config.connect(0, 1, "()");
        let runtime = CuRuntime::<Tasks, Msgs, 2>::new(
            RobotClock::default(),
            &config,
            tasks_instanciator,
            FakeWriter {},
        );
        assert!(runtime.is_ok());
    }

    #[test]
    fn test_copperlists_manager_lifecycle() {
        let mut config = CuConfig::default();
        config.add_node(Node::new("a", "TestSource"));
        config.add_node(Node::new("b", "TestSink"));
        config.connect(0, 1, "()");
        let mut runtime = CuRuntime::<Tasks, Msgs, 2>::new(
            RobotClock::default(),
            &config,
            tasks_instanciator,
            FakeWriter {},
        )
        .unwrap();

        // Now emulates the generated runtime
        {
            let copperlists = &mut runtime.copper_lists_manager;
            let culist0 = copperlists
                .create()
                .expect("Ran out of space for copper lists");
            // FIXME: error handling.
            let id = culist0.id;
            assert_eq!(id, 0);
            culist0.change_state(CopperListState::Processing);
            assert_eq!(runtime.available_copper_lists(), 1);
        }

        {
            let copperlists = &mut runtime.copper_lists_manager;
            let culist1 = copperlists
                .create()
                .expect("Ran out of space for copper lists"); // FIXME: error handling.
            let id = culist1.id;
            assert_eq!(id, 1);
            culist1.change_state(CopperListState::Processing);
            assert_eq!(runtime.available_copper_lists(), 0);
        }

        {
            let copperlists = &mut runtime.copper_lists_manager;
            let culist2 = copperlists.create();
            assert!(culist2.is_none());
            assert_eq!(runtime.available_copper_lists(), 0);
        }

        // Free in order, should let the top of the stack be serialized and freed.
        runtime.end_of_processing(1);
        assert_eq!(runtime.available_copper_lists(), 1);

        // Readd a CL
        {
            let copperlists = &mut runtime.copper_lists_manager;
            let culist2 = copperlists
                .create()
                .expect("Ran out of space for copper lists"); // FIXME: error handling.
            let id = culist2.id;
            assert_eq!(id, 2);
            culist2.change_state(CopperListState::Processing);
            assert_eq!(runtime.available_copper_lists(), 0);
        }

        // Free out of order, the #0 first
        runtime.end_of_processing(0);
        // Should not free up the top of the stack
        assert_eq!(runtime.available_copper_lists(), 0);

        // Free up the top of the stack
        runtime.end_of_processing(2);
        // This should free up 2 CLs

        assert_eq!(runtime.available_copper_lists(), 2);
    }
}
