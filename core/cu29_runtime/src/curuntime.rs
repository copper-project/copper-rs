//! CuRuntime is the heart of what copper is running on the robot.
//! It is exposed to the user via the `copper_runtime` macro injecting it as a field in their application struct.
//!

use crate::config::{ComponentConfig, Node};
use crate::config::{CuConfig, CuGraph, NodeId};
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
use petgraph::visit::VisitMap;
use petgraph::visit::Visitable;
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
        mission: Option<&str>,
        tasks_instanciator: impl Fn(Vec<Option<&ComponentConfig>>) -> CuResult<CT>,
        monitor_instanciator: impl Fn(&CuConfig) -> M,
        logger: impl WriteStream<CopperList<P>> + 'static,
    ) -> CuResult<Self> {
        let graph = config.get_graph(mission)?;
        let all_instances_configs: Vec<Option<&ComponentConfig>> = graph
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

pub fn find_task_type_for_id(graph: &CuGraph, node_id: NodeId) -> CuTaskType {
    if graph.0.neighbors_directed(node_id.into(), Incoming).count() == 0 {
        CuTaskType::Source
    } else if graph.0.neighbors_directed(node_id.into(), Outgoing).count() == 0 {
        CuTaskType::Sink
    } else {
        CuTaskType::Regular
    }
}

/// This function gets the input node by using the input step plan id, to get the edge that
/// connects the input to the output in the config graph
fn find_edge_with_plan_input_id(
    plan: &[CuExecutionUnit],
    graph: &CuGraph,
    plan_id: u32,
    output_node_id: NodeId,
) -> usize {
    let input_node = plan
        .get(plan_id as usize)
        .expect("Input step should've been added to plan before the step that receives the input");
    let CuExecutionUnit::Step(input_step) = input_node else {
        panic!("Expected input to be from a step, not a loop");
    };
    let input_node_id = input_step.node_id;

    graph
        .0
        .edges_connecting(input_node_id.into(), output_node_id.into())
        .map(|edge| edge.id().index())
        .next()
        .expect("An edge connecting the input to the output should exist")
}

/// The connection id used here is the index of the config graph edge that equates to the wanted
/// connection
fn sort_inputs_by_cnx_id(
    input_msg_indices_types: &mut [(u32, String)],
    plan: &[CuExecutionUnit],
    graph: &CuGraph,
    curr_node_id: NodeId,
) {
    input_msg_indices_types.sort_by(|(a_index, _), (b_index, _)| {
        let a_edge_id = find_edge_with_plan_input_id(plan, graph, *a_index, curr_node_id);
        let b_edge_id = find_edge_with_plan_input_id(plan, graph, *b_index, curr_node_id);
        a_edge_id.cmp(&b_edge_id)
    });
}
/// Explores a subbranch and build the partial plan out of it.
fn plan_tasks_tree_branch(
    graph: &CuGraph,
    mut next_culist_output_index: u32,
    starting_point: NodeId,
    plan: &mut Vec<CuExecutionUnit>,
) -> (u32, bool) {
    #[cfg(feature = "macro_debug")]
    eprintln!("-- starting branch from node {starting_point}");

    let mut visitor = Bfs::new(&graph.0, starting_point.into());
    let mut handled = false;

    while let Some(node) = visitor.next(&graph.0) {
        let id = node.index() as NodeId;
        let node_ref = graph.get_node(id).unwrap();
        #[cfg(feature = "macro_debug")]
        eprintln!("  Visiting node: {node_ref:?}");

        let mut input_msg_indices_types: Vec<(u32, String)> = Vec::new();
        let output_msg_index_type: Option<(u32, String)>;
        let task_type = find_task_type_for_id(graph, id);

        match task_type {
            CuTaskType::Source => {
                #[cfg(feature = "macro_debug")]
                eprintln!("    → Source node, assign output index {next_culist_output_index}");
                output_msg_index_type = Some((
                    next_culist_output_index,
                    graph
                        .0
                        .edge_weight(EdgeIndex::new(graph.get_src_edges(id).unwrap()[0]))
                        .unwrap() // FIXME(gbin): Error handling
                        .msg
                        .clone(),
                ));
                next_culist_output_index += 1;
            }
            CuTaskType::Sink => {
                let parents: Vec<NodeIndex> =
                    graph.0.neighbors_directed(id.into(), Incoming).collect();
                #[cfg(feature = "macro_debug")]
                eprintln!("    → Sink with parents: {parents:?}");
                for parent in &parents {
                    let pid = parent.index() as NodeId;
                    let index_type = find_output_index_type_from_nodeid(pid, plan);
                    if let Some(index_type) = index_type {
                        #[cfg(feature = "macro_debug")]
                        eprintln!("      ✓ Input from {pid} ready: {index_type:?}");
                        input_msg_indices_types.push(index_type);
                    } else {
                        #[cfg(feature = "macro_debug")]
                        eprintln!("      ✗ Input from {pid} not ready, returning");
                        return (next_culist_output_index, handled);
                    }
                }
                output_msg_index_type = Some((next_culist_output_index, "()".to_string()));
                next_culist_output_index += 1;
            }
            CuTaskType::Regular => {
                let parents: Vec<NodeIndex> =
                    graph.0.neighbors_directed(id.into(), Incoming).collect();
                #[cfg(feature = "macro_debug")]
                eprintln!("    → Regular task with parents: {parents:?}");
                for parent in &parents {
                    let pid = parent.index() as NodeId;
                    let index_type = find_output_index_type_from_nodeid(pid, plan);
                    if let Some(index_type) = index_type {
                        #[cfg(feature = "macro_debug")]
                        eprintln!("      ✓ Input from {pid} ready: {index_type:?}");
                        input_msg_indices_types.push(index_type);
                    } else {
                        #[cfg(feature = "macro_debug")]
                        eprintln!("      ✗ Input from {pid} not ready, returning");
                        return (next_culist_output_index, handled);
                    }
                }
                output_msg_index_type = Some((
                    next_culist_output_index,
                    graph
                        .0
                        .edge_weight(EdgeIndex::new(graph.get_src_edges(id).unwrap()[0])) // FIXME(gbin): Error handling and multimission
                        .unwrap()
                        .msg
                        .clone(),
                ));
                next_culist_output_index += 1;
            }
        }

        sort_inputs_by_cnx_id(&mut input_msg_indices_types, plan, graph, id);

        if let Some(pos) = plan
            .iter()
            .position(|step| matches!(step, CuExecutionUnit::Step(s) if s.node_id == id))
        {
            #[cfg(feature = "macro_debug")]
            eprintln!("    → Already in plan, modifying existing step");
            let mut step = plan.remove(pos);
            if let CuExecutionUnit::Step(ref mut s) = step {
                s.input_msg_indices_types = input_msg_indices_types;
            }
            plan.push(step);
        } else {
            #[cfg(feature = "macro_debug")]
            eprintln!("    → New step added to plan");
            let step = CuExecutionStep {
                node_id: id,
                node: node_ref.clone(),
                task_type,
                input_msg_indices_types,
                output_msg_index_type,
            };
            plan.push(CuExecutionUnit::Step(step));
        }

        handled = true;
    }

    #[cfg(feature = "macro_debug")]
    eprintln!("-- finished branch from node {starting_point} with handled={handled}");
    (next_culist_output_index, handled)
}

/// This is the main heuristics to compute an execution plan at compilation time.
/// TODO(gbin): Make that heuristic pluggable.
pub fn compute_runtime_plan(graph: &CuGraph) -> CuResult<CuExecutionLoop> {
    #[cfg(feature = "macro_debug")]
    eprintln!("[runtime plan]");
    let visited = graph.0.visit_map();
    let mut plan = Vec::new();
    let mut next_culist_output_index = 0u32;

    let mut queue: std::collections::VecDeque<NodeId> = graph
        .node_indices()
        .iter()
        .filter(|&node| find_task_type_for_id(graph, node.index() as NodeId) == CuTaskType::Source)
        .map(|node| node.index() as NodeId)
        .collect();

    #[cfg(feature = "macro_debug")]
    eprintln!("Initial source nodes: {queue:?}");

    while let Some(start_node) = queue.pop_front() {
        if visited.is_visited(&start_node) {
            #[cfg(feature = "macro_debug")]
            eprintln!("→ Skipping already visited source {start_node}");
            continue;
        }

        #[cfg(feature = "macro_debug")]
        eprintln!("→ Starting BFS from source {start_node}");
        let mut bfs = Bfs::new(&graph.0, start_node.into());

        while let Some(node_index) = bfs.next(&graph.0) {
            let node_id = node_index.index() as NodeId;
            let already_in_plan = plan
                .iter()
                .any(|unit| matches!(unit, CuExecutionUnit::Step(s) if s.node_id == node_id));
            if already_in_plan {
                #[cfg(feature = "macro_debug")]
                eprintln!("    → Node {node_id} already planned, skipping");
                continue;
            }

            #[cfg(feature = "macro_debug")]
            eprintln!("    Planning from node {node_id}");
            let (new_index, handled) =
                plan_tasks_tree_branch(graph, next_culist_output_index, node_id, &mut plan);
            next_culist_output_index = new_index;

            if !handled {
                #[cfg(feature = "macro_debug")]
                eprintln!("    ✗ Node {node_id} was not handled, skipping enqueue of neighbors");
                continue;
            }

            #[cfg(feature = "macro_debug")]
            eprintln!("    ✓ Node {node_id} handled successfully, enqueueing neighbors");
            for neighbor in graph.0.neighbors(node_index) {
                if !visited.is_visited(&neighbor) {
                    let nid = neighbor.index() as NodeId;
                    #[cfg(feature = "macro_debug")]
                    eprintln!("      → Enqueueing neighbor {nid}");
                    queue.push_back(nid);
                }
            }
        }
    }

    Ok(CuExecutionLoop {
        steps: plan,
        loop_count: None,
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
        let graph = config.get_graph_mut(None).unwrap();
        graph.add_node(Node::new("a", "TestSource")).unwrap();
        graph.add_node(Node::new("b", "TestSink")).unwrap();
        graph.connect(0, 1, "()").unwrap();
        let runtime = CuRuntime::<Tasks, Msgs, NoMonitor, 2>::new(
            RobotClock::default(),
            &config,
            None,
            tasks_instanciator,
            monitor_instanciator,
            FakeWriter {},
        );
        assert!(runtime.is_ok());
    }

    #[test]
    fn test_copperlists_manager_lifecycle() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        graph.add_node(Node::new("a", "TestSource")).unwrap();
        graph.add_node(Node::new("b", "TestSink")).unwrap();
        graph.connect(0, 1, "()").unwrap();
        let mut runtime = CuRuntime::<Tasks, Msgs, NoMonitor, 2>::new(
            RobotClock::default(),
            &config,
            None,
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
    fn test_runtime_task_input_order() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src1_id = graph.add_node(Node::new("a", "Source1")).unwrap();
        let src2_id = graph.add_node(Node::new("b", "Source2")).unwrap();
        let sink_id = graph.add_node(Node::new("c", "Sink")).unwrap();

        assert_eq!(src1_id, 0);
        assert_eq!(src2_id, 1);

        // note that the source2 connection is before the source1
        let src1_type = "src1_type";
        let src2_type = "src2_type";
        graph.connect(src2_id, sink_id, src2_type).unwrap();
        graph.connect(src1_id, sink_id, src1_type).unwrap();

        let src1_edge_id = *graph.get_src_edges(src1_id).unwrap().first().unwrap();
        let src2_edge_id = *graph.get_src_edges(src2_id).unwrap().first().unwrap();
        // the edge id depends on the order the connection is created, not
        // on the node id, and that is what determines the input order
        assert_eq!(src1_edge_id, 1);
        assert_eq!(src2_edge_id, 0);

        let runtime = compute_runtime_plan(graph).unwrap();
        let sink_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == sink_id => Some(step),
                _ => None,
            })
            .unwrap();

        // since the src2 connection was added before src1 connection, the src2 type should be
        // first
        assert_eq!(sink_step.input_msg_indices_types[0].1, src2_type);
        assert_eq!(sink_step.input_msg_indices_types[1].1, src1_type);
    }

    #[test]
    fn test_runtime_plan_diamond_case1() {
        // more complex topology that tripped the scheduler
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let cam0_id = graph
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask"))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask"))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new("broadcast", "tasks::MergingSinkTask"))
            .unwrap();

        // case 1 order
        graph.connect(cam0_id, broadcast_id, "i32").unwrap();
        graph.connect(cam0_id, inf0_id, "i32").unwrap();
        graph.connect(inf0_id, broadcast_id, "f32").unwrap();

        let edge_cam0_to_broadcast = *graph.get_src_edges(cam0_id).unwrap().first().unwrap();
        let edge_cam0_to_inf0 = graph.get_src_edges(cam0_id).unwrap()[1];

        assert_eq!(edge_cam0_to_inf0, 0);
        assert_eq!(edge_cam0_to_broadcast, 1);

        let runtime = compute_runtime_plan(graph).unwrap();
        let broadcast_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == broadcast_id => Some(step),
                _ => None,
            })
            .unwrap();

        assert_eq!(broadcast_step.input_msg_indices_types[0].1, "i32");
        assert_eq!(broadcast_step.input_msg_indices_types[1].1, "f32");
    }

    #[test]
    fn test_runtime_plan_diamond_case2() {
        // more complex topology that tripped the scheduler variation 2
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let cam0_id = graph
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask"))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask"))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new("broadcast", "tasks::MergingSinkTask"))
            .unwrap();

        // case 2 order
        graph.connect(cam0_id, inf0_id, "i32").unwrap();
        graph.connect(cam0_id, broadcast_id, "i32").unwrap();
        graph.connect(inf0_id, broadcast_id, "f32").unwrap();

        let edge_cam0_to_inf0 = *graph.get_src_edges(cam0_id).unwrap().first().unwrap();
        let edge_cam0_to_broadcast = graph.get_src_edges(cam0_id).unwrap()[1];

        assert_eq!(edge_cam0_to_broadcast, 0);
        assert_eq!(edge_cam0_to_inf0, 1);

        let runtime = compute_runtime_plan(graph).unwrap();
        let broadcast_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == broadcast_id => Some(step),
                _ => None,
            })
            .unwrap();

        assert_eq!(broadcast_step.input_msg_indices_types[0].1, "i32");
        assert_eq!(broadcast_step.input_msg_indices_types[1].1, "f32");
    }
}
