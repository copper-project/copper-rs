//! CuRuntime is the heart of what copper is running on the robot.
//! It is exposed to the user via the `copper_runtime` macro injecting it as a field in their application struct.
//!

use crate::config::{ComponentConfig, Flavor, Node, DEFAULT_KEYFRAME_INTERVAL};
use crate::config::{CuConfig, CuGraph, NodeId, RuntimeConfig};
use crate::copperlist::{CopperList, CopperListState, CuListZeroedInit, CuListsManager};
use crate::cutask::{BincodeAdapter, Freezable};
use crate::monitoring::CuMonitor;
use cu29_clock::{ClockProvider, CuTime, RobotClock};
use cu29_traits::CuResult;
use cu29_traits::WriteStream;
use cu29_traits::{CopperListTuple, CuError};

use bincode::enc::write::{SizeWriter, SliceWriter};
use bincode::enc::EncoderImpl;
use bincode::error::EncodeError;
use bincode::{Decode, Encode};
use core::fmt::{Debug, Formatter};
use petgraph::prelude::*;
use petgraph::visit::VisitMap;
use petgraph::visit::Visitable;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::boxed::Box;
    pub use alloc::collections::VecDeque;
    pub use alloc::format;
    pub use alloc::string::String;
    pub use alloc::string::ToString;
    pub use alloc::vec::Vec;
    pub use core::fmt::Result as FmtResult;
}

#[cfg(feature = "std")]
mod imp {
    pub use cu29_log_runtime::LoggerRuntime;
    pub use cu29_unifiedlog::UnifiedLoggerWrite;
    pub use rayon::ThreadPool;
    pub use std::collections::VecDeque;
    pub use std::fmt::Result as FmtResult;
    pub use std::sync::{Arc, Mutex};
}

use imp::*;

/// Just a simple struct to hold the various bits needed to run a Copper application.
#[cfg(feature = "std")]
pub struct CopperContext {
    pub unified_logger: Arc<Mutex<UnifiedLoggerWrite>>,
    pub logger_runtime: LoggerRuntime,
    pub clock: RobotClock,
}

/// Manages the lifecycle of the copper lists and logging.
pub struct CopperListsManager<P: CopperListTuple + Default, const NBCL: usize> {
    pub inner: CuListsManager<P, NBCL>,
    /// Logger for the copper lists (messages between tasks)
    pub logger: Option<Box<dyn WriteStream<CopperList<P>>>>,
}

impl<P: CopperListTuple + Default, const NBCL: usize> CopperListsManager<P, NBCL> {
    pub fn end_of_processing(&mut self, culistid: u32) -> CuResult<()> {
        let mut is_top = true;
        let mut nb_done = 0;
        for cl in self.inner.iter_mut() {
            if cl.id == culistid && cl.get_state() == CopperListState::Processing {
                cl.change_state(CopperListState::DoneProcessing);
            }
            if is_top && cl.get_state() == CopperListState::DoneProcessing {
                if let Some(logger) = &mut self.logger {
                    cl.change_state(CopperListState::BeingSerialized);
                    logger.log(cl)?;
                }
                cl.change_state(CopperListState::Free);
                nb_done += 1;
            } else {
                is_top = false;
            }
        }
        for _ in 0..nb_done {
            let _ = self.inner.pop();
        }
        Ok(())
    }

    pub fn available_copper_lists(&self) -> usize {
        NBCL - self.inner.len()
    }
}

/// Manages the frozen tasks state and logging.
pub struct KeyFramesManager {
    /// Where the serialized tasks are stored following the wave of execution of a CL.
    inner: KeyFrame,

    /// Logger for the state of the tasks (frozen tasks)
    logger: Option<Box<dyn WriteStream<KeyFrame>>>,

    /// Capture a keyframe only each...
    keyframe_interval: u32,
}

impl KeyFramesManager {
    fn is_keyframe(&self, culistid: u32) -> bool {
        self.logger.is_some() && culistid.is_multiple_of(self.keyframe_interval)
    }

    pub fn reset(&mut self, culistid: u32, clock: &RobotClock) {
        if self.is_keyframe(culistid) {
            self.inner.reset(culistid, clock.now());
        }
    }

    pub fn freeze_task(&mut self, culistid: u32, task: &impl Freezable) -> CuResult<usize> {
        if self.is_keyframe(culistid) {
            if self.inner.culistid != culistid {
                panic!("Freezing task for a different culistid");
            }
            self.inner
                .add_frozen_task(task)
                .map_err(|e| CuError::from(format!("Failed to serialize task: {e}")))
        } else {
            Ok(0)
        }
    }

    pub fn end_of_processing(&mut self, culistid: u32) -> CuResult<()> {
        if self.is_keyframe(culistid) {
            let logger = self.logger.as_mut().unwrap();
            logger.log(&self.inner)
        } else {
            Ok(())
        }
    }
}

/// This is the main structure that will be injected as a member of the Application struct.
/// CT is the tuple of all the tasks in order of execution.
/// CL is the type of the copper list, representing the input/output messages for all the tasks.
pub struct CuRuntime<CT, P: CopperListTuple, M: CuMonitor, const NBCL: usize> {
    /// The base clock the runtime will be using to record time.
    pub clock: RobotClock, // TODO: remove public at some point

    /// The tuple of all the tasks in order of execution.
    pub tasks: CT,

    /// For backgrounded tasks.
    #[cfg(feature = "std")]
    pub threadpool: Arc<ThreadPool>,

    /// The runtime monitoring.
    pub monitor: M,

    /// The logger for the copper lists (messages between tasks)
    pub copperlists_manager: CopperListsManager<P, NBCL>,

    /// The logger for the state of the tasks (frozen tasks)
    pub keyframes_manager: KeyFramesManager,

    /// The runtime configuration controlling the behavior of the run loop
    pub runtime_config: RuntimeConfig,
}

/// To be able to share the clock we make the runtime a clock provider.
impl<CT, P: CopperListTuple + CuListZeroedInit + Default, M: CuMonitor, const NBCL: usize>
    ClockProvider for CuRuntime<CT, P, M, NBCL>
{
    fn get_clock(&self) -> RobotClock {
        self.clock.clone()
    }
}

/// A KeyFrame is recording a snapshot of the tasks state before a given copperlist.
/// It is a double encapsulation: this one recording the culistid and another even in
/// bincode in the serialized_tasks.
#[derive(Encode, Decode)]
pub struct KeyFrame {
    // This is the id of the copper list that this keyframe is associated with (recorded before the copperlist).
    pub culistid: u32,
    // This is the timestamp when the keyframe was created, using the robot clock.
    pub timestamp: CuTime,
    // This is the bincode representation of the tuple of all the tasks.
    pub serialized_tasks: Vec<u8>,
}

impl KeyFrame {
    fn new() -> Self {
        KeyFrame {
            culistid: 0,
            timestamp: CuTime::default(),
            serialized_tasks: Vec::new(),
        }
    }

    /// This is to be able to avoid reallocations
    fn reset(&mut self, culistid: u32, timestamp: CuTime) {
        self.culistid = culistid;
        self.timestamp = timestamp;
        self.serialized_tasks.clear();
    }

    /// We need to be able to accumulate tasks to the serialization as they are executed after the step.
    fn add_frozen_task(&mut self, task: &impl Freezable) -> Result<usize, EncodeError> {
        let cfg = bincode::config::standard();
        let mut sizer = EncoderImpl::<_, _>::new(SizeWriter::default(), cfg);
        BincodeAdapter(task).encode(&mut sizer)?;
        let need = sizer.into_writer().bytes_written as usize;

        let start = self.serialized_tasks.len();
        self.serialized_tasks.resize(start + need, 0);
        let mut enc =
            EncoderImpl::<_, _>::new(SliceWriter::new(&mut self.serialized_tasks[start..]), cfg);
        BincodeAdapter(task).encode(&mut enc)?;
        Ok(need)
    }
}

impl<
        CT,
        P: CopperListTuple + CuListZeroedInit + Default + 'static,
        M: CuMonitor,
        const NBCL: usize,
    > CuRuntime<CT, P, M, NBCL>
{
    // FIXME(gbin): this became REALLY ugly with no-std
    #[cfg(feature = "std")]
    pub fn new(
        clock: RobotClock,
        config: &CuConfig,
        mission: Option<&str>,
        tasks_instanciator: impl for<'c> Fn(
            Vec<Option<&'c ComponentConfig>>,
            Arc<ThreadPool>,
        ) -> CuResult<CT>,
        monitor_instanciator: impl Fn(&CuConfig) -> M,
        copperlists_logger: impl WriteStream<CopperList<P>> + 'static,
        keyframes_logger: impl WriteStream<KeyFrame> + 'static,
    ) -> CuResult<Self> {
        let graph = config.get_graph(mission)?;
        let all_instances_configs: Vec<Option<&ComponentConfig>> = graph
            .get_all_nodes()
            .iter()
            .map(|(_, node)| node.get_instance_config())
            .collect();

        // TODO: make that configurable

        let threadpool = Arc::new(
            rayon::ThreadPoolBuilder::new()
                .num_threads(2) // default to 4 threads if not specified
                .build()
                .expect("Could not create the threadpool"),
        );

        let tasks = tasks_instanciator(all_instances_configs, threadpool.clone())?;
        let monitor = monitor_instanciator(config);

        let (copperlists_logger, keyframes_logger, keyframe_interval) = match &config.logging {
            Some(logging_config) if logging_config.enable_task_logging => (
                Some(Box::new(copperlists_logger) as Box<dyn WriteStream<CopperList<P>>>),
                Some(Box::new(keyframes_logger) as Box<dyn WriteStream<KeyFrame>>),
                logging_config.keyframe_interval.unwrap(), // it is set to a default at parsing time
            ),
            Some(_) => (None, None, 0), // explicit no enable logging
            None => (
                // default
                Some(Box::new(copperlists_logger) as Box<dyn WriteStream<CopperList<P>>>),
                Some(Box::new(keyframes_logger) as Box<dyn WriteStream<KeyFrame>>),
                DEFAULT_KEYFRAME_INTERVAL,
            ),
        };

        let copperlists_manager = CopperListsManager {
            inner: CuListsManager::new(),
            logger: copperlists_logger,
        };

        let keyframes_manager = KeyFramesManager {
            inner: KeyFrame::new(),
            logger: keyframes_logger,
            keyframe_interval,
        };

        let runtime_config = config.runtime.clone().unwrap_or_default();

        let runtime = Self {
            tasks,
            threadpool,
            monitor,
            clock,
            copperlists_manager,
            keyframes_manager,
            runtime_config,
        };

        Ok(runtime)
    }

    #[cfg(not(feature = "std"))]
    pub fn new(
        clock: RobotClock,
        config: &CuConfig,
        mission: Option<&str>,
        tasks_instanciator: impl for<'c> Fn(Vec<Option<&'c ComponentConfig>>) -> CuResult<CT>,
        monitor_instanciator: impl Fn(&CuConfig) -> M,
        copperlists_logger: impl WriteStream<CopperList<P>> + 'static,
        keyframes_logger: impl WriteStream<KeyFrame> + 'static,
    ) -> CuResult<Self> {
        let graph = config.get_graph(mission)?;
        let all_instances_configs: Vec<Option<&ComponentConfig>> = graph
            .get_all_nodes()
            .iter()
            .map(|(_, node)| node.get_instance_config())
            .collect();

        let tasks = tasks_instanciator(all_instances_configs)?;

        let monitor = monitor_instanciator(config);

        let (copperlists_logger, keyframes_logger, keyframe_interval) = match &config.logging {
            Some(logging_config) if logging_config.enable_task_logging => (
                Some(Box::new(copperlists_logger) as Box<dyn WriteStream<CopperList<P>>>),
                Some(Box::new(keyframes_logger) as Box<dyn WriteStream<KeyFrame>>),
                logging_config.keyframe_interval.unwrap(), // it is set to a default at parsing time
            ),
            Some(_) => (None, None, 0), // explicit no enable logging
            None => (
                // default
                Some(Box::new(copperlists_logger) as Box<dyn WriteStream<CopperList<P>>>),
                Some(Box::new(keyframes_logger) as Box<dyn WriteStream<KeyFrame>>),
                DEFAULT_KEYFRAME_INTERVAL,
            ),
        };

        let copperlists_manager = CopperListsManager {
            inner: CuListsManager::new(),
            logger: copperlists_logger,
        };

        let keyframes_manager = KeyFramesManager {
            inner: KeyFrame::new(),
            logger: keyframes_logger,
            keyframe_interval,
        };

        let runtime_config = config.runtime.clone().unwrap_or_default();

        let runtime = Self {
            tasks,
            #[cfg(feature = "std")]
            threadpool,
            monitor,
            clock,
            copperlists_manager,
            keyframes_manager,
            runtime_config,
        };

        Ok(runtime)
    }
}

/// Copper tasks can be of 4 types:
/// - Source: only producing output messages (usually used for drivers)
/// - Regular: processing input messages and producing output messages, more like compute nodes.
/// - Sink: only consuming input messages (usually used for actuators)
/// - Bridge: Like a regular task but gets conceptually "split" into a Source and a Sink
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CuTaskType {
    Source,
    Regular,
    Sink,
    Bridge,
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
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
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
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
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
    let node = graph.get_node(node_id).unwrap();
    if node.get_flavor() == Flavor::Bridge {
        return CuTaskType::Bridge;
    }
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
) -> (u32, bool, Option<NodeId>) {
    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("-- starting branch from node {starting_point}");

    let mut visitor = Bfs::new(&graph.0, starting_point.into());
    let mut handled = false;
    let mut resched: Option<NodeId> = None;

    while let Some(node) = visitor.next(&graph.0) {
        let id = node.index() as NodeId;
        let node_ref = graph.get_node(id).unwrap();
        #[cfg(all(feature = "std", feature = "macro_debug"))]
        eprintln!("  Visiting node: {node_ref:?}");

        let mut input_msg_indices_types: Vec<(u32, String)> = Vec::new();
        let output_msg_index_type: Option<(u32, String)>;
        let task_type = find_task_type_for_id(graph, id);

        match task_type {
            CuTaskType::Source => {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
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
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Sink with parents: {parents:?}");
                for parent in &parents {
                    let pid = parent.index() as NodeId;
                    let index_type = find_output_index_type_from_nodeid(pid, plan);
                    if let Some(index_type) = index_type {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✓ Input from {pid} ready: {index_type:?}");
                        input_msg_indices_types.push(index_type);
                    } else {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✗ Input from {pid} not ready, returning");
                        return (next_culist_output_index, handled, resched);
                    }
                }
                output_msg_index_type = Some((next_culist_output_index, "()".to_string()));
                next_culist_output_index += 1;
            }
            CuTaskType::Bridge => {
                // A bridge acts as both a source and a sink.
                // We need to check if we've already scheduled the source side.
                // Source side: no inputs, produces output
                // Sink side: has inputs, produces ()

                let parents: Vec<NodeIndex> =
                    graph.0.neighbors_directed(id.into(), Incoming).collect();

                // Check if all parent outputs are ready in the plan
                let mut all_parents_ready = true;
                for parent in &parents {
                    let pid = parent.index() as NodeId;
                    if find_output_index_type_from_nodeid(pid, plan).is_none() {
                        all_parents_ready = false;
                        break;
                    }
                }

                // Determine if this is the source side or sink side visit
                let is_source_side = find_output_index_type_from_nodeid(id, plan).is_none();

                if is_source_side {
                    // Source side: produce output for downstream tasks
                    // But only if this bridge has outgoing edges
                    let src_edges = graph.get_src_edges(id).unwrap_or_default();
                    if !src_edges.is_empty() {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("    → Bridge node (source side), assign output index {next_culist_output_index}");
                        output_msg_index_type = Some((
                            next_culist_output_index,
                            graph
                                .0
                                .edge_weight(EdgeIndex::new(src_edges[0]))
                                .unwrap() // FIXME(gbin): Error handling
                                .msg
                                .clone(),
                        ));
                        next_culist_output_index += 1;

                        // If this bridge has incoming connections, we need to schedule the sink side later
                        if !parents.is_empty() {
                            resched = Some(id);
                        }
                    } else {
                        // This bridge has no outgoing edges, so it's sink-only
                        // Even though is_source_side is true (no output in plan yet),
                        // we should treat it as a sink
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("    → Bridge node (sink-only, no outgoing edges)");

                        if !all_parents_ready {
                            #[cfg(all(feature = "std", feature = "macro_debug"))]
                            eprintln!("      ✗ Not all inputs ready, returning");
                            return (next_culist_output_index, handled, resched);
                        }

                        // Collect inputs
                        for parent in &parents {
                            let pid = parent.index() as NodeId;
                            let index_type = find_output_index_type_from_nodeid(pid, plan).unwrap();
                            #[cfg(all(feature = "std", feature = "macro_debug"))]
                            eprintln!("      ✓ Input from {pid} ready: {index_type:?}");
                            input_msg_indices_types.push(index_type);
                        }

                        output_msg_index_type = Some((next_culist_output_index, "()".to_string()));
                        next_culist_output_index += 1;
                    }
                } else {
                    // Sink side: consume inputs from upstream tasks
                    #[cfg(all(feature = "std", feature = "macro_debug"))]
                    eprintln!("    → Bridge node (sink side) with parents: {parents:?}");

                    // Check if all inputs are ready
                    if !all_parents_ready {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✗ Not all inputs ready, returning");
                        return (next_culist_output_index, handled, resched);
                    }

                    // Collect inputs
                    for parent in &parents {
                        let pid = parent.index() as NodeId;
                        let index_type = find_output_index_type_from_nodeid(pid, plan).unwrap();
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✓ Input from {pid} ready: {index_type:?}");
                        input_msg_indices_types.push(index_type);
                    }

                    output_msg_index_type = Some((next_culist_output_index, "()".to_string()));
                    next_culist_output_index += 1;
                }
            }

            CuTaskType::Regular => {
                let parents: Vec<NodeIndex> =
                    graph.0.neighbors_directed(id.into(), Incoming).collect();
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Regular task with parents: {parents:?}");
                for parent in &parents {
                    let pid = parent.index() as NodeId;
                    let index_type = find_output_index_type_from_nodeid(pid, plan);
                    if let Some(index_type) = index_type {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✓ Input from {pid} ready: {index_type:?}");
                        input_msg_indices_types.push(index_type);
                    } else {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      ✗ Input from {pid} not ready, returning");
                        return (next_culist_output_index, handled, resched);
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

        // For bridges, we allow the same node to appear twice (source side + sink side)
        // For other task types, we update existing entries
        let should_update_existing = task_type != CuTaskType::Bridge;

        if should_update_existing {
            if let Some(pos) = plan
                .iter()
                .position(|step| matches!(step, CuExecutionUnit::Step(s) if s.node_id == id))
            {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Already in plan at position {pos}, modifying in place");
                // Update in place to preserve ordering
                if let CuExecutionUnit::Step(ref mut s) = plan[pos] {
                    s.input_msg_indices_types = input_msg_indices_types;
                }
            } else {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
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
        } else {
            // Bridge: always add a new step
            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    → New bridge step added to plan");
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

    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("-- finished branch from node {starting_point} with handled={handled} and resched={resched:?}");
    (next_culist_output_index, handled, resched)
}

/// This is the main heuristics to compute an execution plan at compilation time.
/// TODO(gbin): Make that heuristic pluggable.
pub fn compute_runtime_plan(graph: &CuGraph) -> CuResult<CuExecutionLoop> {
    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("[runtime plan]");
    let visited = graph.0.visit_map();
    let mut plan = Vec::new();
    let mut next_culist_output_index = 0u32;

    let mut queue: VecDeque<NodeId> = graph
        .node_indices()
        .iter()
        .filter(|&node| {
            let node_id = node.index() as NodeId;
            match find_task_type_for_id(graph, node_id) {
                CuTaskType::Source => true,
                CuTaskType::Bridge => {
                    // Only add bridges that have outgoing edges (act as sources)
                    !graph.get_src_edges(node_id).unwrap_or_default().is_empty()
                }
                _ => false,
            }
        })
        .map(|node| node.index() as NodeId)
        .collect();

    #[cfg(all(feature = "std", feature = "macro_debug"))]
    eprintln!("Initial source nodes: {queue:?}");

    while let Some(start_node) = queue.pop_front() {
        if visited.is_visited(&start_node) {
            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("→ Skipping already visited source {start_node}");
            continue;
        }

        #[cfg(all(feature = "std", feature = "macro_debug"))]
        eprintln!("→ Starting BFS from source {start_node}");
        let mut bfs = Bfs::new(&graph.0, start_node.into());

        while let Some(node_index) = bfs.next(&graph.0) {
            let node_id = node_index.index() as NodeId;
            let task_type = find_task_type_for_id(graph, node_id);

            // For bridges, we need to check if BOTH sides are already in the plan
            // For other tasks, we check if they're in the plan at all
            let already_in_plan = if task_type == CuTaskType::Bridge {
                // Count how many times this bridge appears in the plan
                let bridge_count = plan
                    .iter()
                    .filter(|unit| matches!(unit, CuExecutionUnit::Step(s) if s.node_id == node_id))
                    .count();

                // Check if this bridge has both incoming and outgoing edges
                let has_outgoing = !graph.get_src_edges(node_id).unwrap_or_default().is_empty();
                let has_incoming = graph.0.neighbors_directed(node_id.into(), Incoming).count() > 0;

                if has_outgoing && has_incoming {
                    // Bridge can appear twice (source + sink)
                    bridge_count >= 2
                } else {
                    // Bridge is source-only or sink-only, can appear once
                    bridge_count >= 1
                }
            } else {
                plan.iter()
                    .any(|unit| matches!(unit, CuExecutionUnit::Step(s) if s.node_id == node_id))
            };

            if already_in_plan {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Node {node_id} already planned, skipping");
                continue;
            }

            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    Planning from node {node_id}");
            let (new_index, handled, resched) =
                plan_tasks_tree_branch(graph, next_culist_output_index, node_id, &mut plan);
            next_culist_output_index = new_index;

            if !handled {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    ✗ Node {node_id} was not handled, skipping enqueue of neighbors");
                continue;
            }

            if let Some(resched_node) = resched {
                #[cfg(all(feature = "std", feature = "macro_debug"))]
                eprintln!("    → Rescheduling node {resched_node}");
                queue.push_back(resched_node);
            }

            #[cfg(all(feature = "std", feature = "macro_debug"))]
            eprintln!("    ✓ Node {node_id} handled successfully, enqueueing neighbors");
            for neighbor in graph.0.neighbors(node_index) {
                if !visited.is_visited(&neighbor) {
                    let nid = neighbor.index() as NodeId;
                    if nid == start_node {
                        #[cfg(all(feature = "std", feature = "macro_debug"))]
                        eprintln!("      → Don't enqueue the starting node again {nid}");
                        continue;
                    }
                    #[cfg(all(feature = "std", feature = "macro_debug"))]
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
    use crate::config::{Flavor, Node};
    use crate::cutask::CuSinkTask;
    use crate::cutask::{CuSrcTask, Freezable};
    use crate::monitoring::NoMonitor;
    use bincode::Encode;
    use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks};
    use serde_derive::Serialize;

    pub struct TestSource {}

    impl Freezable for TestSource {}

    impl CuSrcTask for TestSource {
        type Output<'m> = ();
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            _empty_msg: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            Ok(())
        }
    }

    pub struct TestSink {}

    impl Freezable for TestSink {}

    impl CuSinkTask for TestSink {
        type Input<'m> = ();

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
            Ok(())
        }
    }

    // Those should be generated by the derive macro
    type Tasks = (TestSource, TestSink);

    #[derive(Debug, Encode, Decode, Serialize, Default)]
    struct Msgs(());

    impl ErasedCuStampedDataSet for Msgs {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for Msgs {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    impl CuListZeroedInit for Msgs {
        fn init_zeroed(&mut self) {}
    }

    #[cfg(feature = "std")]
    fn tasks_instanciator(
        all_instances_configs: Vec<Option<&ComponentConfig>>,
        _threadpool: Arc<ThreadPool>,
    ) -> CuResult<Tasks> {
        Ok((
            TestSource::new(all_instances_configs[0])?,
            TestSink::new(all_instances_configs[1])?,
        ))
    }

    #[cfg(not(feature = "std"))]
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
        graph
            .add_node(Node::new("a", "TestSource", Flavor::Task))
            .unwrap();
        graph
            .add_node(Node::new("b", "TestSink", Flavor::Task))
            .unwrap();
        graph.connect(0, 1, "()").unwrap();
        let runtime = CuRuntime::<Tasks, Msgs, NoMonitor, 2>::new(
            RobotClock::default(),
            &config,
            None,
            tasks_instanciator,
            monitor_instanciator,
            FakeWriter {},
            FakeWriter {},
        );
        assert!(runtime.is_ok());
    }

    #[test]
    fn test_copperlists_manager_lifecycle() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        graph
            .add_node(Node::new("a", "TestSource", Flavor::Task))
            .unwrap();
        graph
            .add_node(Node::new("b", "TestSink", Flavor::Task))
            .unwrap();
        graph.connect(0, 1, "()").unwrap();

        let mut runtime = CuRuntime::<Tasks, Msgs, NoMonitor, 2>::new(
            RobotClock::default(),
            &config,
            None,
            tasks_instanciator,
            monitor_instanciator,
            FakeWriter {},
            FakeWriter {},
        )
        .unwrap();

        // Now emulates the generated runtime
        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist0 = copperlists
                .inner
                .create()
                .expect("Ran out of space for copper lists");
            // FIXME: error handling.
            let id = culist0.id;
            assert_eq!(id, 0);
            culist0.change_state(CopperListState::Processing);
            assert_eq!(copperlists.available_copper_lists(), 1);
        }

        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist1 = copperlists
                .inner
                .create()
                .expect("Ran out of space for copper lists"); // FIXME: error handling.
            let id = culist1.id;
            assert_eq!(id, 1);
            culist1.change_state(CopperListState::Processing);
            assert_eq!(copperlists.available_copper_lists(), 0);
        }

        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist2 = copperlists.inner.create();
            assert!(culist2.is_none());
            assert_eq!(copperlists.available_copper_lists(), 0);
            // Free in order, should let the top of the stack be serialized and freed.
            let _ = copperlists.end_of_processing(1);
            assert_eq!(copperlists.available_copper_lists(), 1);
        }

        // Readd a CL
        {
            let copperlists = &mut runtime.copperlists_manager;
            let culist2 = copperlists
                .inner
                .create()
                .expect("Ran out of space for copper lists"); // FIXME: error handling.
            let id = culist2.id;
            assert_eq!(id, 2);
            culist2.change_state(CopperListState::Processing);
            assert_eq!(copperlists.available_copper_lists(), 0);
            // Free out of order, the #0 first
            let _ = copperlists.end_of_processing(0);
            // Should not free up the top of the stack
            assert_eq!(copperlists.available_copper_lists(), 0);

            // Free up the top of the stack
            let _ = copperlists.end_of_processing(2);
            // This should free up 2 CLs

            assert_eq!(copperlists.available_copper_lists(), 2);
        }
    }

    #[test]
    fn test_runtime_task_input_order() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let src1_id = graph
            .add_node(Node::new("a", "Source1", Flavor::Task))
            .unwrap();
        let src2_id = graph
            .add_node(Node::new("b", "Source2", Flavor::Task))
            .unwrap();
        let sink_id = graph
            .add_node(Node::new("c", "Sink", Flavor::Task))
            .unwrap();

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
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask", Flavor::Task))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask", Flavor::Task))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new(
                "broadcast",
                "tasks::MergingSinkTask",
                Flavor::Task,
            ))
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
            .add_node(Node::new("cam0", "tasks::IntegerSrcTask", Flavor::Task))
            .unwrap();
        let inf0_id = graph
            .add_node(Node::new("inf0", "tasks::Integer2FloatTask", Flavor::Task))
            .unwrap();
        let broadcast_id = graph
            .add_node(Node::new(
                "broadcast",
                "tasks::MergingSinkTask",
                Flavor::Task,
            ))
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

    #[test]
    fn test_degenerated_bridge_bridge_case() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let brg_id = graph
            .add_node(Node::new("a", "Bridge", Flavor::Bridge))
            .unwrap();

        graph.connect(brg_id, brg_id, "MyType").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let steps: Vec<CuExecutionUnit> = runtime.steps;

        assert_eq!(steps.len(), 2);
        let CuExecutionUnit::Step(step) = &steps[0] else {
            panic!("Expected a step");
        };
        // Src side
        assert_eq!(step.task_type, CuTaskType::Bridge);
        let Some((idx, typ)) = &step.output_msg_index_type else {
            panic!("Expected an output msg index");
        };
        assert_eq!(*idx, 0);
        assert_eq!(typ, "MyType");
        let CuExecutionUnit::Step(step) = &steps[1] else {
            panic!("Expected a step");
        };
        // Sink side
        assert_eq!(step.task_type, CuTaskType::Bridge);
        let (idx, typ) = &step.input_msg_indices_types[0];
        assert_eq!(*idx, 0);
        assert_eq!(typ, "MyType");
        let Some((idx, typ)) = &step.output_msg_index_type else {
            panic!("Expected an output msg index");
        };
        assert_eq!(*idx, 1);
        assert_eq!(typ, "()");
    }

    #[test]
    fn test_bridge_task_bridge_case() {
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let brg_id = graph
            .add_node(Node::new("b", "Bridge", Flavor::Bridge))
            .unwrap();
        let tsk_id = graph
            .add_node(Node::new("t", "Task", Flavor::Task))
            .unwrap();

        graph.connect(brg_id, tsk_id, "IncomingMsg").unwrap();
        graph.connect(tsk_id, brg_id, "OutgoingMsg").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let steps = runtime.steps;

        assert_eq!(steps.len(), 3);

        let CuExecutionUnit::Step(step) = &steps[0] else {
            panic!("Expected a step");
        };
        // Src side
        assert_eq!(step.task_type, CuTaskType::Bridge);
        let Some((idx, typ)) = &step.output_msg_index_type else {
            panic!("Expected an output msg index");
        };
        assert_eq!(*idx, 0);
        assert_eq!(typ, "IncomingMsg");

        let CuExecutionUnit::Step(step) = &steps[1] else {
            panic!("Expected a step");
        };
        // Task (middle step)
        assert_eq!(step.task_type, CuTaskType::Regular);

        let (idx, typ) = &step.input_msg_indices_types[0];
        assert_eq!(*idx, 0); // Reads from bridge output at index 0
        assert_eq!(typ, "IncomingMsg");

        let Some((idx, typ)) = &step.output_msg_index_type else {
            panic!("Expected an output msg index");
        };
        assert_eq!(*idx, 1); // Outputs to index 1
        assert_eq!(typ, "OutgoingMsg");

        // Step 2: Bridge sink side
        let CuExecutionUnit::Step(step) = &steps[2] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);

        let (idx, typ) = &step.input_msg_indices_types[0];
        assert_eq!(*idx, 1); // Reads from task output at index 1
        assert_eq!(typ, "OutgoingMsg");

        let Some((idx, typ)) = &step.output_msg_index_type else {
            panic!("Expected an output msg index");
        };
        assert_eq!(*idx, 2); // Bridge sink outputs () at index 2
        assert_eq!(typ, "()");
    }

    #[test]
    fn test_bridge_with_multiple_tasks() {
        // Test: Bridge -> Task1 -> Task2 -> Bridge
        // This tests a bridge with a longer pipeline between source and sink
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let brg_id = graph
            .add_node(Node::new("bridge", "Bridge", Flavor::Bridge))
            .unwrap();
        let tsk1_id = graph
            .add_node(Node::new("task1", "Task1", Flavor::Task))
            .unwrap();
        let tsk2_id = graph
            .add_node(Node::new("task2", "Task2", Flavor::Task))
            .unwrap();

        graph.connect(brg_id, tsk1_id, "InputMsg").unwrap();
        graph.connect(tsk1_id, tsk2_id, "IntermediateMsg").unwrap();
        graph.connect(tsk2_id, brg_id, "OutputMsg").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let steps = runtime.steps;

        assert_eq!(steps.len(), 4);

        // Step 0: Bridge source
        let CuExecutionUnit::Step(step) = &steps[0] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 0);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().1, "InputMsg");

        // Step 1: Task1
        let CuExecutionUnit::Step(step) = &steps[1] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Regular);
        assert_eq!(step.input_msg_indices_types[0].0, 0);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 1);

        // Step 2: Task2
        let CuExecutionUnit::Step(step) = &steps[2] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Regular);
        assert_eq!(step.input_msg_indices_types[0].0, 1);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 2);

        // Step 3: Bridge sink
        let CuExecutionUnit::Step(step) = &steps[3] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);
        assert_eq!(step.input_msg_indices_types[0].0, 2);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 3);
    }

    #[test]
    fn test_two_bridges_in_series() {
        // Test: Bridge1 -> Task -> Bridge2
        // This tests two separate bridges in a pipeline
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let brg1_id = graph
            .add_node(Node::new("bridge1", "Bridge1", Flavor::Bridge))
            .unwrap();
        let tsk_id = graph
            .add_node(Node::new("task", "Task", Flavor::Task))
            .unwrap();
        let brg2_id = graph
            .add_node(Node::new("bridge2", "Bridge2", Flavor::Bridge))
            .unwrap();

        graph.connect(brg1_id, tsk_id, "Msg1").unwrap();
        graph.connect(tsk_id, brg2_id, "Msg2").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let steps = runtime.steps;

        // Expected: Bridge1_src -> Task -> Bridge1_sink -> Bridge2_src -> Bridge2_sink
        // But wait, Bridge1 has no inputs, so no sink side!
        // And Bridge2 has no outputs, so it's only sink side!
        // Actually: Bridge1_src -> Task -> Bridge2_sink
        assert_eq!(steps.len(), 3);

        // Step 0: Bridge1 source only (no feedback)
        let CuExecutionUnit::Step(step) = &steps[0] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 0);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().1, "Msg1");

        // Step 1: Task
        let CuExecutionUnit::Step(step) = &steps[1] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Regular);
        assert_eq!(step.input_msg_indices_types[0].0, 0);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 1);

        // Step 2: Bridge2 sink only (no outputs to graph)
        let CuExecutionUnit::Step(step) = &steps[2] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);
        assert_eq!(step.input_msg_indices_types[0].0, 1);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 2);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().1, "()");
    }

    #[test]
    fn test_bridge_diamond_pattern() {
        // Test: Bridge -> Task1 -> Task3
        //                     \-> Task2 -/
        //                               -> Bridge (sink)
        // This tests a bridge with a diamond pattern in between
        let mut config = CuConfig::default();
        let graph = config.get_graph_mut(None).unwrap();
        let brg_id = graph
            .add_node(Node::new("bridge", "Bridge", Flavor::Bridge))
            .unwrap();
        let tsk1_id = graph
            .add_node(Node::new("task1", "Task1", Flavor::Task))
            .unwrap();
        let tsk2_id = graph
            .add_node(Node::new("task2", "Task2", Flavor::Task))
            .unwrap();
        let tsk3_id = graph
            .add_node(Node::new("task3", "Task3", Flavor::Task))
            .unwrap();

        graph.connect(brg_id, tsk1_id, "InputMsg").unwrap();
        graph.connect(brg_id, tsk2_id, "InputMsg").unwrap();
        graph.connect(tsk1_id, tsk3_id, "Msg1").unwrap();
        graph.connect(tsk2_id, tsk3_id, "Msg2").unwrap();
        graph.connect(tsk3_id, brg_id, "OutputMsg").unwrap();

        let runtime = compute_runtime_plan(graph).unwrap();
        let steps = runtime.steps;

        assert_eq!(steps.len(), 5);

        // Step 0: Bridge source
        let CuExecutionUnit::Step(step) = &steps[0] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().0, 0);

        // Steps 1 & 2: Task1 and Task2 (order may vary)
        // Both should read from index 0
        for step in steps.iter().take(2 + 1).skip(1) {
            let CuExecutionUnit::Step(step) = step else {
                panic!("Expected a step");
            };
            assert_eq!(step.task_type, CuTaskType::Regular);
            assert_eq!(step.input_msg_indices_types[0].0, 0);
        }

        // Step 3: Task3 (merging task)
        let CuExecutionUnit::Step(step) = &steps[3] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Regular);
        assert_eq!(step.input_msg_indices_types.len(), 2); // Two inputs
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().1, "OutputMsg");

        // Step 4: Bridge sink
        let CuExecutionUnit::Step(step) = &steps[4] else {
            panic!("Expected a step");
        };
        assert_eq!(step.task_type, CuTaskType::Bridge);
        assert_eq!(step.input_msg_indices_types[0].1, "OutputMsg");
        assert_eq!(step.output_msg_index_type.as_ref().unwrap().1, "()");
    }
}
