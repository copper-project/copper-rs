# Copper-rs Deep Research: Internals, Architecture, and Automotive Stack Integration

> **Purpose:** This document is a complete technical reference for understanding copper-rs internals
> before implementing production-grade CAN, UDS, and SOME/IP stacks on top of it. A reader of only
> this document and plan.md should have everything needed to implement those stacks correctly.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Core Concepts: The Task Graph Model](#2-core-concepts-the-task-graph-model)
3. [Configuration System (RON Format)](#3-configuration-system-ron-format)
4. [Task Traits and Lifecycle](#4-task-traits-and-lifecycle)
5. [Message System: CuMsg, CuStampedData, CuMsgMetadata](#5-message-system-cumsg-custampeddata-cumsgmetadata)
6. [CopperList: The In-Flight Data Bus](#6-copperlist-the-in-flight-data-bus)
7. [The proc-macro Code Generator (`#[copper_runtime]`)](#7-the-proc-macro-code-generator-copper_runtime)
8. [CuRuntime and the Execution Engine](#8-curuntime-and-the-execution-engine)
9. [Clock System: CuClock, CuInstant, CuDuration](#9-clock-system-cuclock-cuinstant-cuduration)
10. [Memory Management and the Unified Log](#10-memory-management-and-the-unified-log)
11. [SOA Derive and Data-Oriented Design](#11-soa-derive-and-data-oriented-design)
12. [Bridges: ROS2, Zenoh, iceoryx2](#12-bridges-ros2-zenoh-iceoryx2)
13. [Monitoring System](#13-monitoring-system)
14. [Embedded / no_std Support](#14-embedded--no_std-support)
15. [Resources System](#15-resources-system)
16. [Simulation Mode](#16-simulation-mode)
17. [Component Ecosystem](#17-component-ecosystem)
18. [Error Handling](#18-error-handling)
19. [Performance Characteristics and Real-time Guarantees](#19-performance-characteristics-and-real-time-guarantees)
20. [Automotive Protocol Integration: Fit Analysis](#20-automotive-protocol-integration-fit-analysis)

---

## 1. System Overview

Copper-rs is a **deterministic, zero-allocation robotics runtime and SDK** written in Rust. It is
described by its authors as "what a game engine is to games — build, run, and replay your entire
robot deterministically." Key properties:

- **Sub-microsecond inter-task latency** via a compile-time-typed in-process message bus
- **Deterministic replay** — every run is bit-for-bit reproducible by replaying the unified log
- **Targets the full spectrum**: Linux x86-64, aarch64, WASM, bare-metal ARM Cortex-M, RISC-V
- **No heap allocation in the hot path** — memory preallocated at startup, ring-buffer style
- **Proc-macro driven DAG wiring** — the graph is resolved at compile time, zero runtime overhead
- **ROS2, Zenoh, iceoryx2 bridges** — progressive migration path from ROS2 ecosystems
- Built on: Rust 1.80+, bincode 2 (serialization), petgraph (DAG), bevy_reflect (reflection)

### Crate Topology (workspace)

```
core/
  cu29                — public prelude, app helpers, pool, re-exports
  cu29_runtime        — CuRuntime, CuListsManager, cutask traits, CuMsg, CopperList, config
  cu29_traits         — CuError, CuResult, WriteStream, CopperListTuple, ErasedCuStampedData
  cu29_clock          — CuInstant, CuDuration, CuTimeRange, Tov; arch-specific counters
  cu29_derive         — #[copper_runtime] proc-macro (code generator)
  cu29_base_derive    — helper derive macros (CuTaskLifecycleCallbacks etc.)
  cu29_soa_derive     — #[derive(Soa)] for Structure-of-Arrays layout
  cu29_reflect_derive — bevy_reflect integration helpers
  cu29_log            — CuLog (structured task-side logging)
  cu29_log_derive     — #[derive(CuLog)] proc-macro
  cu29_log_runtime    — log serde/replay infrastructure
  cu29_unifiedlog     — mmap-backed binary log writer/reader
  cu29_helpers        — basic_copper_setup(), init_copper_app!()
  cu29_export         — export/replay tooling
  cu29_intern_strs    — interned string constants
  cu29_value          — Value enum (dynamic typed data for config)
  cu29_units          — type-safe physical units

components/
  bridges/            — cu_ros2_bridge, cu_iceoryx2_bridge, cu_zenoh_bridge, etc.
  sources/            — IMU, LiDAR, GPS, camera, serial, etc.
  sinks/              — GPIO, servo, Zenoh output, etc.
  tasks/              — AHRS, PID, alignment, AprilTag, rate limiter
  libs/               — cu_embedded_registry, cu_sdlogger, cu_transform, cu_tuimon
  monitors/           — cu_bevymon, cu_consolemon, cu_logmon, cu_safetymon
  payloads/           — cu_gnss_payloads, cu_ros2_payloads
  testing/            — test helpers

examples/             — 30+ examples including embedded skeletons
```

---

## 2. Core Concepts: The Task Graph Model

Copper models computation as a **Directed Acyclic Graph (DAG)** of tasks connected by typed
message edges. This graph is fully resolved at compile time.

### Node Types

| Type | Trait | In-Edges | Out-Edges | Description |
|------|-------|----------|-----------|-------------|
| Source | `CuSrcTask<'cl>` | 0 | ≥1 | Produces messages (sensor, driver, timer) |
| Regular | `CuTask<'cl>` | ≥1 | ≥1 | Transforms messages |
| Sink | `CuSinkTask<'cl>` | ≥1 | 0 | Consumes messages (actuator, logger) |

Node type is **automatically inferred** by the macro by examining edge counts in the graph.

### The Graph (petgraph)

```rust
// core/cu29_runtime/src/config.rs
pub struct CuGraph(pub StableDiGraph<Node, Cnx, NodeId>);
```

Nodes are `Node` structs (id, type string, config, resources, missions). Edges are `Cnx` structs
(src id, dst id, message type string, missions, src/dst channel for bridges).

### CopperList — the per-cycle data bus

Each execution cycle creates one **CopperList**: a preallocated tuple of `CuMsg<T>` slots, one
per edge in the graph. All tasks in a cycle read from and write to slots in the same CopperList.
This is the entire inter-task communication mechanism — no queues, no channels, no locks.

### Background Tasks

Tasks can be marked `background: true` in config. They run on a `ThreadPoolBundle` threadpool
and are wrapped in `CuAsyncTask<T, Output>`. Background tasks run concurrently with the main
execution loop but still communicate via the same CopperList mechanism.

---

## 3. Configuration System (RON Format)

### File Format

Copper uses **RON (Rusty Object Notation)** `.ron` files. The schema maps to `CuConfigRepresentation`:

```rust
// core/cu29_runtime/src/config.rs:1463
struct CuConfigRepresentation {
    tasks:     Option<Vec<Node>>,
    resources: Option<Vec<ResourceBundleConfig>>,
    bridges:   Option<Vec<BridgeConfig>>,
    cnx:       Option<Vec<SerializedCnx>>,
    monitors:  Option<Vec<MonitorConfig>>,
    logging:   Option<LoggingConfig>,
    runtime:   Option<RuntimeConfig>,
    missions:  Option<Vec<MissionsConfig>>,
    includes:  Option<Vec<IncludesConfig>>,
}
```

### Minimal Example (3-node pipeline)

```ron
(
    tasks: [
        (id: "src",  type: "myapp::tasks::MySource"),
        (id: "proc", type: "myapp::tasks::MyProcessor"),
        (id: "sink", type: "myapp::tasks::MySink"),
    ],
    cnx: [
        (src: "src",  dst: "proc", msg: "myapp::messages::Frame"),
        (src: "proc", dst: "sink", msg: "myapp::messages::Result"),
    ],
)
```

### Full Node Definition

```rust
pub struct Node {
    id: String,                              // unique identifier
    type_: Option<String>,                   // Rust type path
    config: Option<ComponentConfig>,         // arbitrary KV params -> task.new()
    resources: Option<HashMap<String, String>>, // resource bundle bindings
    missions: Option<Vec<String>>,           // missions this node applies to
    background: Option<bool>,                // run on threadpool
    run_in_sim: Option<bool>,                // include/exclude in sim mode
    logging: Option<NodeLogging>,            // per-node log config
    flavor: Flavor,                          // Task | Bridge
    nc_outputs: Vec<String>,                 // intentionally unconnected outputs
}
```

### ComponentConfig — Per-Node Parameters

```rust
pub struct ComponentConfig(pub HashMap<String, Value>);
```

Tasks receive their `ComponentConfig` in `new()`. The `Value` enum handles: bool, i64, f64,
String, Array, Map. Tasks call `config.get::<T>("key")` to extract typed values.

### Connection (Edge) Definition

```rust
pub struct Cnx {
    pub src: String,                   // source node id (or "bridge_id/channel_id")
    pub dst: String,                   // dest node id   (or "bridge_id/channel_id")
    pub msg: String,                   // Rust type path of payload
    pub missions: Option<Vec<String>>, // missions this edge applies to
    pub src_channel: Option<String>,   // bridge channel name on src side
    pub dst_channel: Option<String>,   // bridge channel name on dst side
    pub order: usize,                  // original serialized index
}
```

### Missions

Multiple named execution graphs in one config file. Tasks and connections carry
`missions: ["name"]` tags. The macro generates separate code for each mission.

### Logging Config

```rust
pub struct LoggingConfig {
    pub enable_task_logging: bool,      // default: true
    pub slab_size_mib: Option<u64>,     // mmap slab size
    pub section_size_mib: Option<u64>,  // per-section size
    pub keyframe_interval: Option<u32>, // for replay seeking
}
```

### Runtime Config

```rust
pub struct RuntimeConfig {
    pub rate_target_hz: Option<u64>, // CopperList execution rate cap
}
```

---

## 4. Task Traits and Lifecycle

All task traits live in `core/cu29_runtime/src/cutask.rs`.

### CuSrcTask (Source — no inputs)

```rust
pub trait CuSrcTask<'cl>: Sized {
    type Output: CuMsgPack<'cl>;
    type Resources<'a>: ResourceBindings;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>;
    fn start(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn preprocess(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }  // async-safe
    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()>;
    fn postprocess(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) } // async-safe
    fn stop(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
}
```

### CuTask (Regular — transforms input to output)

```rust
pub trait CuTask<'cl>: Sized {
    type Input: CuMsgPack<'cl>;
    type Output: CuMsgPack<'cl>;
    type Resources<'a>: ResourceBindings;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>;
    fn start(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn preprocess(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn process(&mut self, clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()>;
    fn postprocess(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn stop(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
}
```

### CuSinkTask (Sink — consumes, no output)

```rust
pub trait CuSinkTask<'cl>: Sized {
    type Input: CuMsgPack<'cl>;
    type Resources<'a>: ResourceBindings;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>;
    fn start(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn preprocess(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn process(&mut self, clock: &RobotClock, input: Self::Input) -> CuResult<()>;
    fn postprocess(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
    fn stop(&mut self, clock: &RobotClock) -> CuResult<()> { Ok(()) }
}
```

### Lifecycle Sequence

```
startup:
  for each task: task.start(clock)

per-cycle (hot loop):
  for each task in topological order:
    task.preprocess(clock)        ← allowed to do async I/O (called outside critical section)
    [timestamp start]
    task.process(clock, input, output)   ← CRITICAL SECTION — must be deterministic, no blocking
    [timestamp end → stored in CuMsgMetadata.process_time]
    task.postprocess(clock)       ← allowed to do async I/O

shutdown:
  for each task (reverse order): task.stop(clock)
```

### Multi-Input / Multi-Output

```rust
// Multi-output source (2 output channels)
type Output<'m> = output_msg!(SensorFrame, DiagPayload);
// In process():
output.0.set_payload(frame);
output.1.set_payload(diag);

// Multi-input regular task
type Input<'m> = input_msg!('m, SensorFrame, ImuData);
// In process():
if let Some(frame) = input.0.payload() { ... }
if let Some(imu) = input.1.payload() { ... }
```

The `input_msg!` / `output_msg!` macros expand to tuples of `&mut CuMsg<T>` references
into the CopperList. The runtime ensures all inputs are present before calling `process()`.

---

## 5. Message System: CuMsg, CuStampedData, CuMsgMetadata

### CuStampedData<T, M> (the canonical message type, aliased as CuMsg<T>)

```rust
// core/cu29_runtime/src/cutask.rs
pub struct CuStampedData<T, M>
where
    T: CuMsgPayload,
    M: Metadata,
{
    payload: Option<T>,         // The actual data (None = no data yet / dropped)
    pub tov: Tov,               // Time of Validity: None | One(CuInstant) | Range(CuTimeRange)
    pub metadata: M,            // Processing metadata (timing, status text)
}
```

`CuMsg<T>` is the alias used everywhere: `type CuMsg<T> = CuStampedData<T, CuMsgMetadata>`.

### CuMsgMetadata

```rust
pub struct CuMsgMetadata {
    pub process_time: PartialCuTimeRange,  // start/end timestamp of task.process() call
    pub status_txt: CuCompactString,       // short human-readable status for monitoring
}
```

`PartialCuTimeRange` holds `Option<CuInstant>` start and end, automatically populated by the
runtime around each `process()` call.

### Tov (Time of Validity)

```rust
pub enum Tov {
    None,
    Time(CuInstant),
    Range(CuTimeRange),  // CuTimeRange { start: CuInstant, end: CuInstant }
}
```

The Tov is set by the source/task producing the message to indicate when the data was sampled.
This is separate from `process_time` (when the CPU ran the task).

### CuMsgPayload Trait Bound

Any payload type `T` must implement `CuMsgPayload`:

```rust
pub trait CuMsgPayload:
    Default + Debug + Clone + Encode + Decode + Serialize + DeserializeOwned + 'static
{}
// Auto-implemented for all types satisfying the bounds.
```

This means all payloads must be: default-constructable, serializable (bincode + serde), cloneable.
For `no_std` contexts, `alloc` is used instead of `std`.

### ErasedCuStampedData (type-erased interface for monitoring/logging)

```rust
pub trait ErasedCuStampedData {
    fn payload(&self) -> Option<&dyn erased_serde::Serialize>;
    fn payload_reflect(&self) -> Option<&dyn Reflect>;  // feature = "reflect"
    fn tov(&self) -> Tov;
    fn metadata(&self) -> &dyn CuMsgMetadataTrait;
}
```

---

## 6. CopperList: The In-Flight Data Bus

### Structure

```rust
// core/cu29_runtime/src/copperlist.rs
pub struct CopperList<P: CopperListTuple> {
    pub id: u64,               // monotonically increasing cycle counter
    state: CopperListState,    // state machine
    pub msgs: P,               // GENERATED by macro: tuple of CuMsg<T> for each edge
}
```

`P` is a compile-time tuple type like `(CuMsg<FramePayload>, CuMsg<ResultPayload>, ())` — one
slot per connection in the graph. The `()` slots represent tasks with no output (sinks).

### CopperListState (state machine)

```rust
pub enum CopperListState {
    Free,              // available for reuse
    Initialized,       // reset and ready for a new cycle
    Processing,        // currently being processed by tasks
    DoneProcessing,    // all tasks ran, ready for logging
    BeingSerialized,   // being written to unified log
}
```

### CuListsManager — the ring buffer

```rust
pub struct CuListsManager<P: CopperListTuple, const N: usize> {
    data: Box<[CopperList<P>; N]>,  // N pre-allocated CopperLists (default N=2)
    length: usize,
    insertion_index: usize,
    current_cl_id: u64,
}
```

- `N` is the const generic buffer count (default 2, "double buffering")
- Memory is **heap-allocated once at startup** via `alloc_zeroed` with proper `Layout`
- The buffer is iterable in **reverse** (newest-first) or ascending order
- A `CopperList` is never freed; it cycles through states and is reused

### Memory Layout Philosophy

All inter-task data for one cycle lives in one contiguous allocation. The macro generates the
exact `P` tuple type at compile time so the size is known statically. No dynamic dispatch,
no Box<dyn>, no heap allocation per message.

The `CopperListTuple` trait requires `Default + Debug + Encode + Decode`. The `CopperLiskMask`
(512-task bitmask) is present for future use (selective remote transmission).

### Data Path: Source → Task → Sink

```
1. Runtime calls CuListsManager::get_or_alloc() → gets next Free CopperList
2. CopperList.change_state(Initialized)
3. For src in topological order:
     a. preprocess(clock)
     b. [record process_time.start]
     c. process(clock, &mut copperlist.msgs.N)   ← source writes into slot N
     d. [record process_time.end into msgs.N.metadata.process_time]
     e. postprocess(clock)
4. For regular tasks (same loop):
     a. preprocess(clock)
     b. [record process_time.start]
     c. process(clock, &copperlist.msgs.M, &mut copperlist.msgs.N)  ← read M, write N
     d. postprocess(clock)
5. For sinks:
     a. process(clock, &copperlist.msgs.M)   ← read M, produce nothing
6. CopperList.change_state(DoneProcessing)
7. Serialize CopperList to unified log (if logging enabled)
8. CopperList.change_state(BeingSerialized → Free)
```

---

## 7. The proc-macro Code Generator (`#[copper_runtime]`)

### Invocation

```rust
#[copper_runtime(config = "robot.ron", sim_mode = false)]
struct MyRobot {
    // user fields
}
```

### What It Injects Into the Struct

```rust
struct MyRobot {
    // ...user fields...
    copper_runtime: CuRuntime<
        CuTasks,          // tuple of all task instances
        CuBridges,        // tuple of all bridge instances
        CuStampedDataSet, // the P type for CopperList<P>
        MonitorType,      // NoMonitor or tuple of monitors
        2                 // CLNB: CopperList buffer count
    >,
    runtime_lifecycle_stream: Option<Box<dyn WriteStream<RuntimeLifecycleRecord>>>,
}
```

### Generated Types

```rust
// CopperList payload tuple (one slot per graph edge)
type CuStampedDataSet = (CuMsg<FramePayload>, CuMsg<ResultPayload>, ());

// Task tuple (one per node)
type CuTasks = (SourceTask, ProcessorTask, SinkTask);
// Background tasks wrapped:
type CuTasks = (SourceTask, CuAsyncTask<HeavyTask, OutputType>, SinkTask);

// Bridge tuple
type CuBridges = (Ros2Bridge, ZenohBridge);

// Monitor type
type MonitorType = (BevyMonitor, SafetyMonitor);  // or NoMonitor
```

### Generated impl blocks

```rust
impl MyRobot {
    // Initialization
    pub fn new(config: &CuConfig, context: &CopperContext) -> CuResult<Self>;

    // Execution loop entry point
    pub fn run(&mut self) -> CuResult<()>;

    // Mission-specific run (if missions defined)
    pub fn run_mission_default(&mut self) -> CuResult<()>;
    pub fn run_mission_sim(&mut self) -> CuResult<()>;
}
```

### CuExecutionLoop (compile-time execution plan)

The macro computes a topological sort of the DAG and generates a `CuExecutionLoop`:

```rust
pub struct CuExecutionLoop {
    pub steps: Vec<CuExecutionUnit>,
    pub loop_count: Option<u32>,
}

pub enum CuExecutionUnit {
    Step(Box<CuExecutionStep>),
    Loop(CuExecutionLoop),   // for nested loops (future feature)
}

pub struct CuExecutionStep {
    pub node_id: NodeId,
    pub node: Node,
    pub task_type: CuTaskType,  // Source | Regular | Sink
    pub input_msg_indices_types: Vec<CuInputMsg>,   // which CopperList slots to read
    pub output_msg_pack: Option<CuOutputPack>,       // which CopperList slots to write
}
```

The generated `run()` method walks this plan: it never allocates, never does dynamic dispatch
on the critical path.

---

## 8. CuRuntime and the Execution Engine

### CuRuntime<Tasks, Bridges, P, Monitor, CLNB>

```rust
pub struct CuRuntime<Tasks, Bridges, P, Monitor, const CLNB: usize>
where P: CopperListTuple
{
    // task instances tuple
    tasks: Tasks,
    // bridge instances tuple  
    bridges: Bridges,
    // ring buffer of CopperLists
    copper_lists_manager: CuListsManager<P, CLNB>,
    // monitor instance
    monitor: Monitor,
    // clock
    clock: RobotClock,
    // logging stream
    cl_stream: Option<Box<dyn WriteStream<CopperList<P>>>>,
    // execution plan (generated by macro)
    plan: CuExecutionLoop,
}
```

### run() / process() mechanics

The hot loop:
1. Get next CopperList from ring buffer
2. Stamp CopperList id (monotonic counter)
3. For each step in the execution plan:
   - Record `process_time.start` into the relevant CuMsg metadata
   - Call `task.process(clock, inputs, outputs)` with direct references into the CopperList
   - Record `process_time.end`
4. Notify monitor (if any) with the completed CopperList
5. Serialize CopperList to unified log (bincode, async via background thread)
6. Enforce `rate_target_hz` sleep if configured

### Rate Limiting

If `runtime.rate_target_hz` is set, the runtime sleeps after each cycle to maintain the
target rate. Uses `CuDuration` arithmetic against `CuInstant::now()`.

---

## 9. Clock System: CuClock, CuInstant, CuDuration

### Architecture-specific hardware counters

`cu29_clock` has **per-architecture raw counter implementations**:

| File | Architecture | Implementation |
|------|-------------|----------------|
| `x86_64.rs` | x86-64 | `rdtsc` via `core::arch::x86_64::_rdtsc()` |
| `aarch64.rs` | ARM64 | `CNTVCT_EL0` system register |
| `cortexm.rs` | Cortex-M (bare-metal) | `DWT.CYCCNT` or callback |
| `riscv64.rs` | RISC-V 64 | `rdtime` CSR |
| `wasm.rs` | WASM | `performance.now()` |
| `fallback.rs` | other | `std::time::Instant` |

All counters are converted to nanoseconds via a **calibration** module that measures the counter
frequency at startup.

### Types

```rust
pub struct CuInstant(u64);   // nanoseconds since arbitrary epoch (monotonic)
pub struct CuDuration(pub u64); // nanosecond duration, always positive

pub enum Tov {
    None,
    Time(CuInstant),
    Range(CuTimeRange),
}

pub struct CuTimeRange {
    pub start: CuInstant,
    pub end: CuInstant,
}

pub struct PartialCuTimeRange {
    pub start: Option<CuInstant>,
    pub end: Option<CuInstant>,
}
```

### RobotClock

`RobotClock` is a wrapper around the raw counter that provides:
- `now() -> CuInstant`
- `from_epoch(&self) -> CuDuration` — time since robot start
- Shared-reference counter via `Arc<AtomicU64>` for cross-thread access
- On embedded: the counter can be **injected via a callback** (for RTC or external time source)

`portable_atomic::AtomicU64` is used to support 32-bit embedded platforms where `AtomicU64` is
not natively available (emulated with CAS loop on ARMv6/v7).

---

## 10. Memory Management and the Unified Log

### CuListsManager Allocation

The ring buffer is allocated **once** at startup:
```rust
let layout = Layout::new::<[CopperList<P>; N]>();
let ptr = unsafe { alloc_zeroed(layout) };
```
No allocation occurs in the main loop. CopperLists are reused by state machine cycling.

### Unified Log (cu29_unifiedlog)

The unified log is the **replay backbone** of copper. It is a **memory-mapped file** (mmap) 
organized in sections.

```
File layout:
  [MainHeader]          — magic, page_size, first_section_offset
  [SectionHeader]       — magic[2], type (CopperList|StructuredLog|...), length, closed flag
  [bincode-encoded data — bincode 2 format]
  [SectionHeader]
  [bincode-encoded data]
  ...
  [EndOfLogMarker]
```

```rust
pub struct MainHeader {
    pub first_section_offset: u16,  // page-aligned start
    pub page_size: u16,
}

pub struct SectionHeader {
    pub magic: [u8; 2],
    pub entry_type: UnifiedLogType,  // CopperList | StructuredLog | LifecycleEvent | ...
    // length, closed flag
}
```

**Writing** is via `WriteStream<E: Encode>` — each call appends a bincode-encoded entry to the
current section. When a section fills, a new one is started.

**Reading** is via `ReadStream` — iterates sections, decodes entries. This is used for replay
and offline analysis.

### Slab Size

Default: 10 MiB. Configurable via `logging.slab_size_mib`. The slab is the pre-allocated mmap
region. Overflow handling: new sections are created; the log is a chain of section slabs.

### Log Types

```rust
pub enum UnifiedLogType {
    CopperList,          // per-cycle CopperList (all task messages + metadata)
    StructuredLog,       // CuLog entries (task-side log calls)
    LifecycleEvent,      // start/stop events
    // ...
}
```

### Structured Logging (CuLog)

Tasks can log structured data:
```rust
debug!(
    "camera latency: {}",
    copper_log(latency_ns)
);
```

The `#[derive(CuLog)]` macro generates a `CuLogStructured` type. Log entries are written to
the unified log (not stdout) for zero-overhead logging. A separate `cu_logmon` / `cu_logviz`
tool processes the log offline.

---

## 11. SOA Derive and Data-Oriented Design

```rust
#[derive(Soa)]
struct SensorSample {
    timestamp: u64,
    value: f32,
    quality: u8,
}
// Generates:
pub struct SensorSampleSoa<const N: usize> {
    pub len: usize,
    pub timestamp: [u64; N],
    pub value: [f32; N],
    pub quality: [u8; N],
}
```

**Why SOA?** For batch sensor data (e.g., LiDAR point clouds, IMU burst reads), SOA layout
enables SIMD vectorization. The fixed-size const-generic array fits in a `CuMsg<T>` payload
without any heap allocation, because `N` is known at compile time.

**Nested SOA**: fields annotated `#[soa(nested)]` create nested `XxxSoaStorage<N>` types,
enabling hierarchical data-oriented structures.

---

## 12. Bridges: ROS2, Zenoh, iceoryx2

Bridges are a **first-class concept** in copper — not bolted-on adapters. They appear in
config as `bridges:` entries and connect to the task graph via named channels.

### Bridge Architecture

```rust
pub trait CuBridge {
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> where Self: Sized;
    fn start(&mut self) -> CuResult<()>;
    fn stop(&mut self) -> CuResult<()>;
}

// Bridge channels are Rx (inbound to copper) or Tx (outbound from copper)
// Each channel has an id, an optional route (topic/path/service name), and ComponentConfig
```

Bridge channels appear in the `cnx:` array as `"bridge_id/channel_id"`:
```ron
cnx: [
    (src: "ros_bridge/sensor_topic", dst: "filter_task", msg: "payloads::SensorMsg"),
    (src: "filter_task", dst: "ros_bridge/output_topic", msg: "payloads::Filtered"),
]
```

### iceoryx2 Bridge (Zero-Copy IPC)

- Uses iceoryx2's publish/subscribe with two service types:
  - `ipc::Service` — shared memory between processes
  - `local::Service` — same-process
- Serialization: **bincode 2** (compact binary)
- Payload size is fixed at channel config time (`payload_size` parameter)
- Suitable for inter-process communication with near-zero latency

### Zenoh Bridge (Cloud/Edge)

- Wire formats per channel: `Bincode` (compact), `Json` (human-readable), `Cbor`
- Flexible key-expression routing (zenoh topics)
- Suitable for: cloud telemetry, remote debug, cross-machine communication

### ROS2 Bridge

```rust
pub trait RosBridgeAdapter: Sized {
    type CopperMsg: CuMsgPayload;
    type RosMsg;
    fn to_copper(ros_msg: Self::RosMsg) -> CuResult<Self::CopperMsg>;
    fn from_copper(cu_msg: &Self::CopperMsg) -> CuResult<Self::RosMsg>;
}
```

- `RosPayloadRegistry`: `HashMap<TypeId, Box<dyn RosCodec>>` for dynamic dispatch at bridge boundary
- Serialization: **CDR** (ROS2 standard)
- Enables progressive migration from ROS2 to pure copper

---

## 13. Monitoring System

Monitors receive the completed CopperList after each cycle and can observe all messages/metadata.

```rust
pub trait CuMonitor: Sized {
    fn new(
        config: Option<&ComponentConfig>,
        robot_descriptor: &CuMonitorMetadata,
        runtime: &CuRuntime<...>,
    ) -> CuResult<Self>;
    fn process(&mut self, clock: &RobotClock, copperlist: &CopperList<P>) -> CuResult<()>;
    fn start(&mut self, clock: &RobotClock) -> CuResult<()>;
    fn stop(&mut self, clock: &RobotClock) -> CuResult<()>;
}
```

Built-in monitors:
- `cu_bevymon` — real-time Bevy-based visualization
- `cu_consolemon` — TUI terminal monitor
- `cu_logmon` — file-based log monitor
- `cu_safetymon` — fault detection / safety watchdog

Multiple monitors compose as a tuple: `(BevyMon, SafetyMon)`.

---

## 14. Embedded / no_std Support

Copper has **first-class no_std support** for bare-metal deployment.

### Feature Flags

- `std` (default) — uses std library
- Without `std` — activates `extern crate alloc` paths throughout
- `cu29_clock` — no `std` clock: uses `DWT.CYCCNT` on Cortex-M, callback on others
- `cu29_unifiedlog` — std-only feature (logging requires filesystem)
- `cu_embedded_registry` — no_std task registry for dynamic task lookup on embedded

### Embedded Entry Point Pattern

```rust
#![no_std]
#![no_main]

use embedded_alloc::LlffHeap;
use portable_atomic::{AtomicU64, Ordering};

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

static RTC_NANOS: AtomicU64 = AtomicU64::new(0);

fn rtc_clock_callback() -> u64 {
    RTC_NANOS.load(Ordering::Relaxed)
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // Init 128KB heap
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    
    // Create robot with injected clock
    let clock = RobotClock::from_callback(rtc_clock_callback);
    let mut robot = MyRobot::new_with_clock(&config, clock).unwrap();
    robot.run().unwrap();
    loop {}
}
```

### Targets Supported

| Target | Architecture | Notes |
|--------|-------------|-------|
| Linux x86-64 | x86-64 | Full std, mmap log |
| Linux aarch64 | ARM64 | Full std, Raspberry Pi |
| RP2350 skeleton | ARM Cortex-M33 | no_std, embedded_alloc |
| Generic Cortex-M | ARMv7-M / ARMv6-M | no_std |
| RISC-V 64 | RV64 | no_std capable |
| WASM | wasm32 | Browser/WASI target |

### Constraints on Embedded

- No unified log (filesystem not available)
- No dynamic dispatch for task lookup → `cu_embedded_registry` provides compile-time registry
- Heap size is fixed (128 KB typical for RP2350)
- No background tasks (no threadpool) in minimal baremetal configs
- Clock must be injected via callback from hardware timer/RTC

---

## 15. Resources System

Resources are **hardware abstractions** (GPIO pins, I2C buses, SPI devices, serial ports) that
are owned by the runtime and borrowed by tasks.

```rust
pub trait ResourceBindings: Sized {
    fn from_bindings(
        resources: &mut ResourceManager,
        mapping: &ResourceMapping,
    ) -> CuResult<Self>;
}

// Example task with GPIO resource
pub struct MyTask {
    pin: OutputPin,
}

impl<'cl> CuSinkTask<'cl> for MyTask {
    type Resources<'a> = (OutputPin,);
    // Resources are acquired from ResourceManager by the generated code
}
```

### ResourceManager

The macro generates resource bundle initialization code:
```rust
let mut resources = ResourceManager::new();
resources.register_bundle(I2CBundle::new(config)?);
resources.register_bundle(ThreadPoolBundle::new(config)?); // for background tasks
```

Tasks declare their resource needs via the `Resources` associated type, and the macro
generates code to retrieve and pass them during `new()`.

---

## 16. Simulation Mode

`#[copper_runtime(config = "robot.ron", sim_mode = true)]` generates alternate types:

- Source tasks → replaced by `CuSimSrcTask<T>` (reads from replay log instead of hardware)
- Sink tasks → replaced by `CuSimSinkTask<Input>` (no-op, records expected outputs)
- Regular tasks → same (they are pure logic, no hardware)

This enables **hardware-in-the-loop** and **software-in-the-loop** testing from recorded data.
The `run_in_sim: false` node flag prevents stubbing for nodes that should run even in simulation.

---

## 17. Component Ecosystem

### Sources (hardware drivers → copper messages)

| Component | Protocol/Hardware | Payload |
|-----------|------------------|---------|
| IMU (BMI088, MPU9250, WT901) | SPI/I2C | `ImuMsg` |
| LiDAR (Hesai, Livox, VLP16) | UDP/Ethernet | `PointCloud` |
| GPS/GNSS (ublox) | UART/USB | `GnssMsg` |
| Barometer | I2C | `BaroMsg` |
| Magnetometer | I2C | `MagMsg` |
| Camera | V4L2/USB | `ImageMsg` |
| Encoder | GPIO/SPI | `EncoderMsg` |
| ELRS/CRSF RC | UART | `ChannelData` |
| Feetech servo feedback | UART | `ServoFeedback` |

### Sinks (copper messages → hardware actuation)

| Component | Protocol/Hardware | Input |
|-----------|------------------|-------|
| RPGpio | GPIO | `GpioCmd` |
| Servo controller | PWM/SPI | `ServoCmd` |
| Motor driver | PWM | `MotorCmd` |
| Zenoh output | Network | any |

### Processing Tasks

| Component | Function |
|-----------|----------|
| AHRS fusion | IMU + mag → attitude (quaternion) |
| Message alignment | Synchronizes multi-stream inputs by timestamp |
| AprilTag detection | Camera → tag poses |
| PID controller | Error → control command |
| Rate limiter | Passthrough with rate enforcement |
| Image thresholder | Camera → binary image |

---

## 18. Error Handling

```rust
pub type CuResult<T> = Result<T, CuError>;

pub struct CuError {
    cause: String,
    #[cfg(feature = "std")]
    backtrace: Option<std::backtrace::Backtrace>,
}

impl CuError {
    pub fn new(msg: impl Into<String>) -> Self;
    pub fn add_cause(self, cause: impl Into<String>) -> Self;  // for chaining
}
```

The `add_cause` pattern chains context (similar to `anyhow::context()`). The macro generates
error chains so failures show which task failed and why.

Tasks should return `Err(CuError::new("description"))` on unrecoverable errors. The runtime
propagates errors up to `run()` which returns `CuResult<()>`.

---

## 19. Performance Characteristics and Real-time Guarantees

### What copper guarantees

1. **Zero allocation in the hot path** — all CopperList memory is preallocated at startup
2. **No dynamic dispatch** for task execution — the generated code calls `tasks.0.process()`,
   `tasks.1.process()`, etc. — direct monomorphized calls
3. **No locks in the critical path** — single-threaded execution per CopperList cycle
4. **Deterministic execution order** — topological sort is fixed at compile time
5. **Timestamped every message** — `process_time` is hardware-timestamp based (rdtsc/CNTVCT)
6. **Full reproducibility** — replaying the unified log produces bit-identical outputs

### What copper does NOT guarantee (yet)

1. **WCET (Worst Case Execution Time)** analysis — not built in
2. **Priority inheritance** — no RTOS scheduler integration
3. **Memory protection** — runs in user space on Linux (no MPU enforcement)
4. **Interrupt-driven execution** — the loop is polling-based (can integrate with RTIC/Embassy
   for interrupt-driven embedded but requires manual integration)

### Performance Numbers (from README/code comments)

- Sub-microsecond inter-task latency for simple pass-through tasks on x86-64
- The caterpillar example measures 8-stage pipeline latency end-to-end
- `rdtsc` counter resolution: ~0.3 ns on modern x86-64
- `CuListsManager` ring buffer: double-buffered (N=2 default), no contention

### Real-time Considerations

- `rate_target_hz` provides **soft** rate control (sleep-based, not hard deadline)
- For hard real-time, the intent is to run copper under a PREEMPT_RT Linux kernel or on
  bare-metal with RTIC/Embassy providing interrupt context
- The `preprocess`/`postprocess` separation intentionally isolates I/O from the deterministic
  `process()` critical section

---

## 20. Automotive Protocol Integration: Fit Analysis

### What copper provides that automotive stacks need

| Need | Copper Mechanism |
|------|-----------------|
| Typed message passing | `CuMsg<T>` with compile-time typed edges |
| Timestamping | `Tov` + `process_time` on every message |
| Zero-copy in-process | `CopperList` shared tuple reference |
| Deterministic execution | Fixed topological order, no heap in hot path |
| Cross-process IPC | iceoryx2 bridge (zero-copy SHM) |
| Network communication | Zenoh bridge (pub/sub) |
| Hardware abstraction | Resources system (HAL) |
| Embedded support | no_std, Cortex-M, RISC-V targets |
| Logging/replay | Unified log (bincode mmap) |
| Configuration | RON config with per-node params |
| Monitoring | Monitor trait hooks into every cycle |

### CAN Bus Integration Model

CAN requires:
1. **Frame reception** (hardware interrupt or socket) → copper **Source task**
2. **Frame transmission** (write to CAN socket/hardware) → copper **Sink task**
3. **Signal decoding** (CAN frame → application messages) → copper **Regular task**
4. **Network management** (NM frames, heartbeat) → copper **Background task** or **Source/Sink pair**

The CAN source task would use `socketcan` (Linux) or direct register access (embedded) to read
frames, package them as `CuMsg<CanFrame>`, and emit them. A decoder task would parse the DBC/
ARXML signal definitions and produce typed application messages.

### UDS Integration Model

UDS (ISO 14229) is a request-response protocol over CAN (or DoIP over Ethernet):
1. **Transport layer** (ISO 15765-2 / DoIP) → Source + Sink tasks wrapping the transport
2. **Service dispatcher** — Regular task routing requests to service handlers
3. **Service handlers** — Regular tasks (one per UDS service: 0x22 ReadDataByIdentifier, etc.)
4. **Session/security state machine** → Background task maintaining session state

The request-response nature of UDS maps to a **copper pipeline** where each diagnostic request
flows through the graph as a `CuMsg<UdsRequest>` → processing → `CuMsg<UdsResponse>`.

### SOME/IP Integration Model

SOME/IP (AUTOSAR) has three patterns:
1. **Method calls** (request/response) → Source (Rx) + Sink (Tx) + dispatcher tasks
2. **Events** (fire-and-forget publish) → Source tasks consuming SOME/IP event messages
3. **Fields** (getter/setter/notifier) → hybrid of method + event

The SOME/IP transport (UDP/TCP over Ethernet) maps naturally to copper Sources/Sinks. The
Service Discovery (SOME/IP-SD) runs as a background task maintaining the service registry.

### Copper Constraints to Respect in Automotive Stacks

1. **Payloads must implement `CuMsgPayload`** (`Default + Debug + Clone + Encode + Decode`)
2. **No allocation in `process()`** — build frame buffers in `preprocess()` or preallocate
3. **Tov semantics** — set the Tov to the actual hardware reception timestamp, not process time
4. **Error handling** — return `CuError` variants, not panics
5. **Config-driven** — bus name, bitrate, filters should live in `ComponentConfig`, not code
6. **Bridge vs Task** — cross-process or cross-ECU communication should use the bridge mechanism;
   in-process signal processing should use tasks
7. **SOA for bulk frames** — for high-frequency CAN buses (1 Mbit/s, 8000 fps), use `#[derive(Soa)]`
   on frame structs to enable batched processing

---

*This document was produced by deep static analysis of the copper-rs codebase at
`~/sandboxes/copper-rs` (v0.13, March 2026).*
