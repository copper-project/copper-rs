# Isaac Sim integration

Implemented: a native Copper graph controls Jetbot in an Isaac Sim container using
shared-memory RGB/lidar observations and a Unix command socket. Live GUI/headless
operation and forced-stop override were validated on an RTX 3090 during integration.
Offline sensor recovery and deterministic replay remain unfinished; protocol tests
alone do not establish them. Launch instructions: [example README](../../examples/cu_isaac_sim/README.md).

## Boundary and graph

```text
Isaac container                         Native Copper
physics + camera/lidar → shared buffers → simulation callbacks
                      ↔ Unix socket    ↔ time, leases, wheel commands

camera → image statistics → telemetry sink
lidar  → forward clearance → stop controller → motor sink
```

The container pins Isaac 6.1.0 and mounts Python code directly. Copper runs natively
so ordinary tasks use the existing SDK and monitor. Docker owns simulator/GPU
integration; processing tasks know only standard Copper payloads. The example adds
no simulator dependency to core or embedded code.

[Scenario JSON](../../examples/cu_isaac_sim/isaac/scenario.json) fixes the premade
warehouse, forklift target, Jetbot spawn, calibration, sensor mounts, and rates.
Physics is 60 Hz; sensing/control is 10 Hz; camera output is 320 × 240 RGB. World
coordinates are meters with +Z up; chassis +X is forward. Lidar uses complete
`Example_Rotary` scans, world-frame Cartesian output, and a 0.3 m minimum range so
the 0.65 m stop threshold remains observable. Scene setup validates settling,
floor, approach corridor, camera variation, scan coverage, and lidar/collider
agreement before the control demonstration.

## Simulation-time stepping

Isaac owns advancement. At each observation boundary it finishes physics/sensor
work, fills leased buffers, and publishes sequence/timing metadata. Copper sets
its mock `RobotClock` to simulation time, executes one iteration, and returns a
command with the same sequence/time. Isaac waits for that command before advancing.
This keeps execution order tied to simulation rather than socket latency.

Sensor ToV records acquisition time; accumulated lidar includes per-point offsets.
The adapter converts world points into the lidar frame at the observation boundary.
It does not deskew each point using historical chassis poses.

The generated runtime uses `sim_mode = true`. Source callbacks inject observations
and return `ExecutedBySim`; the motor callback intercepts actuation. Ordinary image,
clearance, and controller tasks return `ExecuteByRuntime`. The
`--override-controller=stop` callback replaces the controller output with zero wheel
commands while both sensor branches continue. Hardware lifecycle access is suppressed
through simulation callbacks.

## Buffer ownership and backpressure

The versioned newline-delimited JSON socket protocol is bounded to 64 KiB per
metadata message. Large arrays stay in shared memory mapped by Rust and Python;
Python uses mmap/NumPy over Copper buffer descriptors. Session paths are mounted
at the same absolute location in the container.

| Data | Representation |
| --- | --- |
| Camera | `CuImage<CuSharedMemoryBuffer<u8>>`, packed RGB with explicit format/stride |
| Transport lidar | 20-byte little-endian records: XYZ float32, intensity float32, acquisition offset uint32 |
| Task lidar | One bounded conversion to `PointCloudSoaHandle`; at most 524,288 points per scan |

Pools are created before the loop: eight image/cloud slots for four retained
CopperLists. Python writes exclusively while holding a lease; published contents
remain immutable until **all** runtime/logger handles release them. A socket
acknowledgment alone cannot authorize reuse. The adapter waits for recording
capacity before granting another lease, outside task execution and simulation time.
Recording drains before the final simulator acknowledgment.

Sequence/timing violations, oversized scans, exhausted leases, disconnects, and
sensor timeouts abort. Isaac sets wheel velocity to zero and pauses on exit,
including loss of its native peer. The launcher places pool files inside a temporary
session through `TMPDIR` and removes that directory after shutdown.

## Recorded state and unfinished replay

The graph enables full handle-content logging for pixels and points. Unified logs
and matching scenario JSON live under the example's `logs/`; JSONL observations
and commands plus simulator reports live under `state/output/{gui,headless}/`.
Keep each `.copper` base and numbered slabs together. A report is simulator evidence,
not proof that a recording can recover data after shared-memory removal.

Remaining work, in dependency order:

1. Add an offline reader and regression that recovers pixels, points, acquisition
   times, and calibration after both processes exit and session buffers disappear.
2. Replay sensor inputs through the real image-statistics, clearance, and controller
   tasks without Isaac/GPU or actuator effects. Restore simulation time and keyframes;
   compare derived values/commands across a keyframe and from a nonzero keyframe.
3. Add a replay stop-threshold override, identify the modified configuration in its
   output log, and compare decisions on identical observations. `just replay` and
   `just inspect` were proposed but are not present in the example's command menu.

The target guarantee is replay of recorded observations. Repeatability of fresh
Isaac physics/rendering runs requires separate validation.

## Code and evidence

| Concern | Pointer |
| --- | --- |
| Native runtime, simulation callbacks, recording | [`src/main.rs`](../../examples/cu_isaac_sim/src/main.rs), [graph](../../examples/cu_isaac_sim/copperconfig.ron) |
| Leases, protocol validation, sensor conversion | [`src/transport.rs`](../../examples/cu_isaac_sim/src/transport.rs) |
| Ordinary processing/control | [`src/tasks.rs`](../../examples/cu_isaac_sim/src/tasks.rs) |
| Simulator stepping and fail-safe | [`isaac/transport.py`](../../examples/cu_isaac_sim/isaac/transport.py) |
| Assets, sensors, geometry checks | [`scene.py`](../../examples/cu_isaac_sim/isaac/scene.py), [`smoke.py`](../../examples/cu_isaac_sim/isaac/smoke.py) |
| Python/Rust protocol coverage, no simulator required | [`tests/protocol.rs`](../../examples/cu_isaac_sim/tests/protocol.rs), [`tests/protocol.py`](../../examples/cu_isaac_sim/tests/protocol.py) |
| Existing replay model to reuse | [caterpillar resim](../../examples/cu_caterpillar/src/resim.rs), [simulation API](../../core/cu29_runtime/src/simulation.rs) |

Historical validation: the 130-observation approach traveled about 1.59 m and
stopped with about 0.68 m collider clearance. The forced-stop run returned 30 zero
commands with only settling motion. Those measurements characterize that scenario
and host, not general simulator performance. Initial integration was split into
PRs #1525 (shared image payload), #1524 (scene/sensors), and #1526 (native graph/IPC).
