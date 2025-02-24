//! CuRuntime is the heart of what copper is running on the robot.
//! It is exposed to the user via the `copper_runtime` macro injecting it as a field in their application struct.
//!

use crate::config::{Cnx, CuConfig, NodeId};
use crate::config::{ComponentConfig, Node};
use crate::copperlist::{CopperList, CopperListState, CuListsManager};
use crate::monitoring::CuMonitor;
use cu29_clock::{ClockProvider, RobotClock};
use cu29_log_runtime::LoggerRuntime;
use cu29_traits::CopperListTuple;
use cu29_traits::CuResult;
use cu29_traits::WriteStream;
use cu29_unifiedlog::UnifiedLoggerWrite;
use std::sync::{Arc, Mutex};

use petgraph::prelude::*;
use std::fmt::Debug;

/// Just a simple struct to hold the various bits needed to run a Copper application.
pub struct CopperContext {
    pub unified_logger: Arc<Mutex<UnifiedLoggerWrite>>,
    pub logger_runtime: LoggerRuntime,
    pub clock: RobotClock,
}

/// This is the main structure that will be injected as a member of the Application struct.
/// CT is the tuple of all the tasks in order of execution.
/// CL is the type of the copper list, representing the input/output messages for all the tasks.
pub struct CuRuntime<CT, P: CopperListTuple, M: CuMonitor, const NBCL: usize> {
    /// The tuple of all the tasks in order of execution.
    pub tasks: CT,

    pub monitor: M,

    /// Copper lists hold in order all the input/output messages for all the tasks.
    pub copper_lists_manager: CuListsManager<P, NBCL>,

    /// The base clock the runtime will be using to record time.
    pub clock: RobotClock, // TODO: remove public at some point

    /// Logger
    logger: Option<Box<dyn WriteStream<CopperList<P>>>>,
}

/// To be able to share the clock we make the runtime a clock provider.
impl<CT, P: CopperListTuple, M: CuMonitor, const NBCL: usize> ClockProvider
    for CuRuntime<CT, P, M, NBCL>
{
    fn get_clock(&self) -> RobotClock {
        self.clock.clone()
    }
}

impl<CT, P: CopperListTuple + 'static, M: CuMonitor, const NBCL: usize> CuRuntime<CT, P, M, NBCL> {
    pub fn new(
        clock: RobotClock,
        config: &CuConfig,
        tasks_instanciator: impl Fn(Vec<Option<&ComponentConfig>>) -> CuResult<CT>,
        monitor_instanciator: impl Fn(&CuConfig) -> M,
        logger: impl WriteStream<CopperList<P>> + 'static,
    ) -> CuResult<Self> {
        let all_instances_configs: Vec<Option<&ComponentConfig>> = config
            .get_all_nodes()
            .iter()
            .map(|(_, node)| node.get_instance_config())
            .collect();
        let tasks = tasks_instanciator(all_instances_configs)?;

        let monitor = monitor_instanciator(config);

        // Needed to declare type explicitly as `cargo check` was failing without it
        let logger_: Option<Box<dyn WriteStream<CopperList<P>>>> =
            if let Some(logging_config) = &config.logging {
                if logging_config.enable_task_logging {
                    Some(Box::new(logger))
                } else {
                    None
                }
            } else {
                Some(Box::new(logger))
            };

        let runtime = Self {
            tasks,
            monitor,
            copper_lists_manager: CuListsManager::new(), // placeholder
            clock,
            logger: logger_,
        };

        Ok(runtime)
    }

    pub fn available_copper_lists(&self) -> usize {
        NBCL - self.copper_lists_manager.len()
    }

    pub fn end_of_processing(&mut self, culistid: u32) {
        let mut is_top = true;
        let mut nb_done = 0;
        self.copper_lists_manager.iter_mut().for_each(|cl| {
            if cl.id == culistid && cl.get_state() == CopperListState::Processing {
                cl.change_state(CopperListState::DoneProcessing);
            }
            // if we have a series of copper lists that are done processing at the top of the circular buffer
            // serialize them all and Free them.
            if is_top && cl.get_state() == CopperListState::DoneProcessing {
                if let Some(logger) = &mut self.logger {
                    cl.change_state(CopperListState::BeingSerialized);
                    logger.log(cl).unwrap();
                }
                cl.change_state(CopperListState::Free);
                nb_done += 1;
            } else {
                is_top = false;
            }
        });
        for _ in 0..nb_done {
            let _ = self.copper_lists_manager.pop();
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
        f.write_str(format!("   CuExecutionStep: Node Id: {}\n", self.node_id).as_str())?;
        f.write_str(format!("                  task_type: {:?}\n", self.node.get_type()).as_str())?;
        f.write_str(format!("                       task: {:?}\n", self.task_type).as_str())?;
        f.write_str(
            format!(
                "              input_msg_types: {:?}\n",
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

pub fn find_task_type_for_id(
    graph: &StableDiGraph<Node, Cnx, NodeId>,
    node_id: NodeId,
) -> CuTaskType {
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
        let output_msg_index_type: Option<(u32, String)>;

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
                        find_output_index_type_from_nodeid(parent.index() as NodeId, plan);
                    if let Some(index_type) = index_type {
                        input_msg_indices_types.push(index_type);
                    } else {
                        // here do not add this node yet, wait for the other inputs to do it with all the inputs earliers in the copper list.
                        return next_culist_output_index;
                    }
                }
                // Here we create an artificial "end node" for this sink to record the metadata associated with it.
                output_msg_index_type = Some((
                    next_culist_output_index,
                    "()".to_string(), // empty type
                ));
                next_culist_output_index += 1;
            }
            CuTaskType::Regular => {
                let parents: Vec<NodeIndex> = config
                    .graph
                    .neighbors_directed(id.into(), Incoming)
                    .collect();

                for parent in parents {
                    let index_type =
                        find_output_index_type_from_nodeid(parent.index() as NodeId, plan);
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
        input_msg_indices_types.sort_by(|a, b| {
            let (a_id, _) = a;
            let (b_id, _) = b;
            a_id.cmp(b_id)
        });

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
/// TODO: Make that heuristic pluggable.
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
    use crate::config::Node;
    use crate::cutask::CuSinkTask;
    use crate::cutask::{CuSrcTask, Freezable};
    use crate::monitoring::NoMonitor;
    use bincode::Encode;

    pub struct TestSource {}

    impl Freezable for TestSource {}

    impl CuSrcTask<'_> for TestSource {
        type Output = ();
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, _empty_msg: Self::Output) -> CuResult<()> {
            Ok(())
        }
    }

    pub struct TestSink {}

    impl Freezable for TestSink {}

    impl CuSinkTask<'_> for TestSink {
        type Input = ();

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

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

    fn monitor_instanciator(_config: &CuConfig) -> NoMonitor {
        NoMonitor {}
    }

    #[derive(Debug)]
    struct FakeWriter {}

    impl<E: Encode> WriteStream<E> for FakeWriter {
        fn log(&mut self, _obj: &E) -> CuResult<()> {
            Ok(())
        }
    }

    #[test]
    fn test_runtime_instantiation() {
        let mut config = CuConfig::default();
        config.add_node(Node::new("a", "TestSource"));
        config.add_node(Node::new("b", "TestSink"));
        config.connect(0, 1, "()");
        let runtime = CuRuntime::<Tasks, Msgs, NoMonitor, 2>::new(
            RobotClock::default(),
            &config,
            tasks_instanciator,
            monitor_instanciator,
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
        let mut runtime = CuRuntime::<Tasks, Msgs, NoMonitor, 2>::new(
            RobotClock::default(),
            &config,
            tasks_instanciator,
            monitor_instanciator,
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

    #[test]
    fn test_config_cnx_id_assignment() {
        let mut config = CuConfig::default();
        let src1_id = config.add_node(Node::new("a", "Source1"));
        let src2_id = config.add_node(Node::new("b", "Source2"));
        let sink_id = config.add_node(Node::new("c", "Sink"));

        assert_eq!(src1_id, 0);
        assert_eq!(src2_id, 1);

        // note that the source2 connection is before the source1
        config.connect(src2_id, sink_id, "type1");
        config.connect(src1_id, sink_id, "type2");

        let src1_edge_id = *config.get_src_edges(src1_id).first().unwrap();
        let src2_edge_id = *config.get_src_edges(src2_id).first().unwrap();

        // the edge id depends on the order the connection is created, not
        // on the node id!
        assert_eq!(src1_edge_id, 1);
        assert_eq!(src2_edge_id, 0);
    }
}
