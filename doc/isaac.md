# Isaac Sim integration: Jetbot, sensors, and replay

Status (2026-10-18): the live demo works. The user confirmed the driven GUI
approach and stop. Docker GPU/Vulkan access, local XWayland rendering, warehouse
assets, onboard RGB, complete RTX lidar scans, shared-memory transport, and the
native Copper graph are validated on the RTX 3090. Headless checks also verify
simulation-clock stepping and the forced-stop controller override. Full sensor-log
recovery and deterministic replay are the next milestone.

## Resume here

Continue on `gbin/isaac-integration`. The PR stack is:

1. [#1525: CuImage shared-memory payload fix](https://github.com/copper-project/copper-rs/pull/1525)
   — `gbin/fix-cuimage-shared-memory`, based on `master`.
2. [#1524: container, scene, and sensors](https://github.com/copper-project/copper-rs/pull/1524)
   — `gbin/base-isaac-setup-scene`, based on the CuImage branch.
3. [#1526: native graph and shared-memory transport](https://github.com/copper-project/copper-rs/pull/1526)
   — `gbin/isaac-integration`, based on the scene branch.

Run these commands from `examples/cu_isaac_sim`:

```bash
just run
just run headless 130
just run headless 30 --override-controller=stop
```

The default GUI run continues until its window closes, keeping sensors and
ConsoleMon live after Jetbot stops at the forklift. `just run gui 130` and the
default `just run headless` produce 130 observations and control iterations, drive
about 1.59 m, and stop with about 0.68 m of collider clearance. Bounded GUI runs
pause the local window for inspection afterward.
ConsoleMon shows the native graph in the terminal; Docker and Isaac output is
redirected to the example's `logs/isaac.log`, replaced on each `just run`.
Use `tail -f logs/isaac.log` from another terminal to inspect simulator startup
and warnings.
The launcher uses the optimized debug profile and background CopperList recording.
Graph statuses show the frame, point count, RGB summary, clearance, and drive/stop
decision. The overview evidence capture runs after control, and the wall-clock frame cap is
disabled without changing the fixed simulation rates.
Keep `state/` caches and the example's `logs/` when resuming. A completed run's
`state/output/{gui,headless}/report.json` identifies its native log base under
`copper.log`; retain that base, its numbered slabs, and matching scenario JSON.

Next work, in order:

1. Add an offline log reader and a regression test that recovers actual RGB
   pixels, lidar points, acquisition times, and calibration after both processes
   have exited and the session's shared-memory directory has been removed.
   Full handle logging is enabled already; recovery has not yet been verified.
2. Replay recorded sensor inputs through the real image-statistics, clearance,
   and controller tasks without Isaac, a GPU, or actuator side effects. Restore
   simulation time and keyframes; verify derived values and commands across a
   keyframe boundary and when resuming from a nonzero keyframe.
3. Add a replay stop-threshold override and compare decisions on identical
   observations. Provide the planned `just replay` and `just inspect` commands,
   then finish their documentation.

Implementation entry points are the example's
[`src/main.rs`](../examples/cu_isaac_sim/src/main.rs) (runtime and callbacks),
[`src/transport.rs`](../examples/cu_isaac_sim/src/transport.rs) (leases and conversion),
[`src/tasks.rs`](../examples/cu_isaac_sim/src/tasks.rs) (ordinary tasks), and
[`isaac/transport.py`](../examples/cu_isaac_sim/isaac/transport.py) (simulator stepping).
The Python/Rust protocol suite in `tests/` runs without Isaac or a GPU.

## Goal

Build `examples/cu_isaac_sim`: NVIDIA's Jetbot drives toward a parked forklift in a
premade warehouse, Copper detects it using RTX lidar, and the controller stops the
robot. A mounted camera records the approach alongside the point cloud and motor
commands. Reuse the environment and an obstacle already in it; scene authoring is
outside this example's scope.

The example should demonstrate three things:

- Running ordinary Copper processing and control tasks against simulated hardware.
- Overriding sensor, actuator, and ordinary task behavior through the simulation API.
- Recording full sensor data and replaying the Copper graph without Isaac or a GPU.

## Agreed choices

| Area | Decision |
| --- | --- |
| Platform | Local Arch Linux host; Isaac in Docker, Copper running natively |
| Isaac version | Pin `nvcr.io/nvidia/isaac-sim:6.1.0` |
| Copper version | Current checkout; reuse existing runtime APIs |
| Robot | NVIDIA Jetbot asset with differential drive |
| Scene | NVIDIA's premade small warehouse with forklifts |
| Demo obstacle | An existing parked forklift; fixed Jetbot spawn and heading |
| Presentation | Local Isaac window by default; headless recording option |
| Connectivity | Local compute; NVIDIA-hosted asset downloads are acceptable |
| Sensors | Color camera and RTX rotary lidar |
| Initial rates | 60 Hz physics; 10 Hz sensors and Copper control |
| Camera resolution | 320 × 240 |
| IPC | Shared-memory sensor buffers and a Unix control socket |
| Lidar payload | Standard Copper point cloud after one bounded conversion |
| Compatibility | New example only; no older Isaac release support in this pass |

Keep transport, GPU readback, and sensor conversion in the simulation adapter,
outside ordinary processing tasks. Preserve static typed wiring and existing core
and `no_std` behavior.

### Premade environment

Load this USD reference relative to Isaac's configured asset root:

```text
/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd
```

This compact industrial scene supplies the environment and parked forklifts. Reuse
its floor, props, and lighting. Select an existing forklift body that intersects the
lidar's forward sensing region, then record its USD prim path and a fixed Jetbot
spawn pose in the example configuration. Verify a clear approach, traversable
floor, and working collision geometry before fixing the demo trajectory. Select
the target once during integration, rather than searching for an obstacle at each
launch. Set an external overview camera to show Jetbot, the target, and the stop;
the onboard camera records the robot's view.

The example's fixed scene and sensor settings live in
[`isaac/scenario.json`](../examples/cu_isaac_sim/isaac/scenario.json). World units
are meters with +Z up; Jetbot's local +X is forward. The selected target is
`/World/warehouse/Forklift/S_ForkliftBody`. Sensor prims are children of the
rigid chassis, `/World/jetbot/chassis`, so they follow its simulated motion.
The overview opens inside the building and frames the approach. The simulator
check settles Jetbot at rest, probes the approach corridor and floor using PhysX
raycasts, and compares lidar returns with the selected forklift collider. These
checks precede the driven stop demonstration.

Jetbot starts at `[7.5, -1.5, 0.04]` with 180° yaw. The overview camera is at
`[8.0, -5.0, 2.5]`, looking toward `[5.6, -1.6, 0.75]`. The approach has a
verified 30 cm wide corridor ending 55 cm before the forklift, with a level floor.
The camera and lidar mount translations relative to the chassis are
`[0.075, 0, 0.18]` and `[0, 0, 0.30]`. Both run at 10 Hz; physics runs at 60 Hz.
The onboard camera looks along chassis +X and produces 320 × 240 RGB. The RTX
`Example_Rotary` profile accumulates complete Cartesian scans in world coordinates.
Override its 1 m minimum range to 0.3 m so the 0.65 m controller threshold remains
observable during the approach.

`just headless` verifies settling, floor and corridor raycasts, nonuniform camera
pixels, and three complete scans with distinct timestamps 100 ms apart. On the
local RTX 3090, the lidar reports approximately 461,000 valid points per scan and
359.96° coverage; forward returns at 2.23 m agree with the forklift collider ray
at 2.25 m. Outputs include `overview.png`, `camera.png`, `lidar.npz`, and
`report.json` under the example's `state/output/headless/`. `just gui` runs the
same checks, then pauses with the overview selected for visual inspection.

`just run` launches the native graph and local window until the window closes;
`just run gui 130` and `just run headless 130` bound the exchange to 130
observations. Every observation
produces one control iteration, approximately 100 ms apart in simulation time.
The validated run travels 1.59 m, stops when lidar clearance crosses 0.65 m, and
ends with 0.68 m of PhysX collider clearance. The forced-stop callback produces
30 zero wheel commands while both sensor branches continue processing; measured
chassis movement is about 0.04 mm of settling. `commands.jsonl` and
`observations.jsonl` stream commands and publication metadata beside the scene
report. Native full-payload logs and scenario JSON live in the example's `logs/`.

NVIDIA also provides office, hospital, and larger warehouse environments. The small
warehouse with forklifts is the default to keep integration and rendering demands
modest. Loading a USD reference adds little application code, but asset downloads,
shader warmup, VRAM use, and rendering time still need measurement. Cache assets
and shaders and validate the selected scene with both sensors enabled. A primitive
scene may serve the earliest transport smoke test; the delivered demo uses the
premade warehouse.

## Architecture

```text
Isaac container                           Native Copper process
  Jetbot + scene
  camera / lidar  ── shared buffers ────►  simulation adapter
                  ◄── Unix socket ─────►  timing, leases, commands
                                           │
                                           ▼
                                         Copper graph
  wheel actuation ◄── motor commands ────  simulated motor sink
```

The Copper graph has two branches:

```text
camera source → image statistics → telemetry sink
lidar source  → forward clearance → stop controller → motor sink
```

Both sensor streams and the derived outputs are logged. Image statistics provide a
small example of ordinary processing on the camera stream; lidar drives the stop
decision.

### Isaac adapter and timing

Use mounted Python code for the local GUI and headless entrypoint. Share scene
creation and adapter code between both modes. Load the premade warehouse and Jetbot
assets, attach the sensors, and persist shader and asset caches across launches.

Isaac owns simulation advancement. At each control boundary:

1. Complete the scheduled physics and sensor work; obtain observations with their
   actual simulation timestamps. Warm up rendering before starting the recorded run.
2. Finish writing the leased shared buffers and notify Copper that they are ready.
3. Copper receives the observations before its iteration and sets
   `RobotClock::mock()` from simulation time.
4. Run one Copper iteration with the simulation callback.
5. Return the motor command to Isaac and apply it to subsequent physics steps.

Keep commands associated with the observation sequence that produced them. Sensor
`Tov` describes acquisition time, including a range for an accumulated lidar scan,
rather than socket arrival or processing time. Match the rotary lidar's scan rate
and sensor tick rate to obtain complete scans.

### Shared-memory ownership

Reuse `CuSharedMemoryPool`, `CuSharedMemoryBuffer`, and Copper's buffer descriptors.
Python maps the backing files with `mmap` and NumPy. Bind the session directory into
the container at the same absolute path and arrange matching user permissions.
Create the pools before the simulation loop and size them for bounded payloads and
the runtime's retained CopperLists.

The Unix socket carries descriptors, leases, frame metadata, commands, and
acknowledgements. Large sensor arrays stay in shared memory.

- Camera output uses `CuImage<CuSharedMemoryBuffer<u8>>`, carrying the actual pixel
  format and stride.
- Lidar arrives in shared arrays and undergoes one bounded conversion into
  `PointCloudSoaHandle`, preserving the standard Copper payload for consumers.
- Python has exclusive write access while filling a lease. After publication,
  Copper reads immutable contents until the lease is released.
- A socket acknowledgement alone does not make a buffer reusable. CopperList and
  logger retention must also release their handles before Python writes it again.
- Missing data, transport failure, or capacity exhaustion stops the run with a zero
  motor command. Isaac must stop locally on a lost connection, even when Copper
  cannot send a final command.

Shared memory avoids bulk socket transfers; GPU readback, the lidar conversion,
and recording still have costs. Measure them separately from task execution.

The implementation uses a versioned, newline-delimited JSON protocol bounded to
64 KiB per metadata message. The native process grants one camera buffer and one
lidar buffer for each observation sequence. Python maps Copper's descriptors and
publishes packed RGB plus 20-byte little-endian lidar records: XYZ float32,
intensity float32, and acquisition offset uint32. Bulk sensor bytes never pass
through the socket. The next command carries the same sequence and simulation
time; no further physics advances while Isaac waits for it.

Pools are created before the loop, with eight image/cloud slots and four
CopperLists. The adapter converts world points into the lidar frame at the
observation boundary, preserving per-point acquisition times. It does not deskew
each point using a historical robot pose. Sensor capacity is 524,288 points per
complete scan. Overflow or exhausted leases aborts the run. The launcher sets
`TMPDIR` to its temporary session directory so existing Copper pool files appear
under the Docker bind mount, without changing the core pool API.

### Task overrides

Build the generated runtime with `sim_mode = true`. In its generated `SimStep`
callback:

- Sensor `Process` callbacks fill output messages from Isaac observations and
  return `SimOverride::ExecutedBySim`.
- Motor `Process` reads the controller's command for Isaac and returns
  `SimOverride::ExecutedBySim`.
- Simulated hardware lifecycle callbacks suppress hardware access as needed.
- Ordinary processing and controller callbacks return
  `SimOverride::ExecuteByRuntime`.

Add `--override-controller=stop`: the callback writes a zero command at the
controller's process step and returns `ExecutedBySim`. This demonstrates replacing
an ordinary task while using the same compiled graph and downstream wiring.

## Recording and replay

Record full camera pixels, lidar points, acquisition timestamps, derived values,
motor commands, and calibration, including sensor transforms and coordinate-frame
conventions. Enable task logging and full handle contents. Write logs under the
example's own `logs/` directory, keeping each base `.copper` path and its slabs
together.

Replay injects recorded sensor inputs, runs the real processing and control tasks,
and suppresses actuator side effects. Use recorded simulation time and restore
state through `Freezable` and keyframes. Full sensor data must remain readable after
both original processes have exited and their shared-memory files are gone.

Support changing the stop threshold during replay so users can compare controller
decisions on identical observations. Identify the replay configuration in the new
log. Deterministic Copper replay of recorded observations is the guarantee to test;
fresh Isaac physics and rendering runs are validated separately.

Proposed example-local commands:

| Command | Purpose |
| --- | --- |
| `just run` | Launch the local Isaac window and Copper demo |
| `just run headless 130` | Record the same scenario without a window |
| `just replay` | Recompute processing and control from a recorded log |
| `just inspect` | Inspect recorded sensors and derived outputs |

## Implementation milestones

- [x] Validate Docker daemon access, NVIDIA Container Toolkit, GPU/Vulkan access,
      and Isaac's compatibility checker.
- [x] Validate the local window through XWayland before building the integration.
- [x] Load the premade warehouse and Jetbot; select and document the existing
      forklift target, collision geometry, fixed spawn pose, and overview camera.
- [x] Render valid camera frames and complete lidar scans in the warehouse.
- [x] Implement shared-memory leases and the Unix socket exchange.
- [x] Connect the Copper graph, simulation clock, and hardware overrides.
- [x] Demonstrate stopping before collision and `--override-controller=stop`
      in headless runs; the user also confirmed the driven GUI demonstration.
- [ ] Record and recover full sensor contents after process shutdown.
- [ ] Implement replay, threshold comparison, and inspection.
- [ ] Document launch, recording, replay, and first-run cache behavior.

The local host has an RTX 3090 with 24 GB VRAM, approximately 64 GiB RAM, and
sufficient free disk space. NVIDIA's container compatibility checker passes;
headless warehouse rendering and the local XWayland UI work. Arch is outside
NVIDIA's documented Ubuntu Linux platform matrix, so the example retains explicit
scene and sensor checks.

## Acceptance and verification

- GPU smoke test: the premade warehouse loads, both sensors produce valid
  observations, and Jetbot stops before the selected forklift in GUI and headless
  runs. Confirm that the detected surface and collision geometry agree.
- IPC tests: Python/Rust layout agreement, frame ordering, exclusive ownership,
  retained logger handles, disconnects, and capacity exhaustion.
- Logging test: recover actual pixels and points after shutdown and removal of
  session buffers; verify timestamps and calibration.
- Replay test: identical recorded inputs reproduce derived outputs and commands
  over multiple iterations and a keyframe boundary. Resume from a nonzero keyframe
  and verify state continuity.
- Override test: the forced-stop callback produces zero motor commands while
  sensor processing and logging continue.
- Follow the repository workflow: remove generated lockfiles before implementation
  passes, run `just fmt` after edits, and run focused example checks plus the
  relevant root verification target before pushing.

## References

- [Isaac Sim 6.1 container setup](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_container.html)
  documents the pinned image and local GUI setup. NVIDIA recommends its workstation
  installation for the full GUI experience, so validate the container window early.
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)
  and the compatibility checker define the initial platform checks.
- [Jetbot differential-drive example](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_simulation/mobile_robot_controllers.html).
- [Premade environment assets](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/assets/usd_assets_environments.html)
  lists the small warehouse with forklifts, larger warehouses, office, and hospital.
- [Asset loading and caching](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/assets/usd_assets_overview.html)
  describes first-load delays and asset dependencies.
- [Camera sensor API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/sensors/isaacsim_sensors_camera.html)
  and [RTX lidar API and timing](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/sensors/isaacsim_sensors_rtx_lidar.html).
- [Copper simulation callbacks](../core/cu29_runtime/src/simulation.rs),
  [shared-memory pools](../core/cu29_runtime/src/pool.rs), and
  [sensor payloads](../components/payloads/cu_sensor_payloads/src/lib.rs).
- [Existing simulator callback example](../examples/cu_rp_balancebot/src/sim_driver.rs)
  and [recording/replay reference](../examples/cu_caterpillar/src/resim.rs).
