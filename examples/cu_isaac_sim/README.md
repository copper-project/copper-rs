# Isaac Sim and native Copper

Load NVIDIA's warehouse with forklifts, place Jetbot on a fixed approach, and
validate its onboard camera and RTX lidar. The example uses
`nvcr.io/nvidia/isaac-sim:6.1.0` directly with mounted Python code; no image build
is needed. A native Copper graph consumes shared-memory sensor observations and
returns wheel commands, one iteration per observation in simulation time.
The [integration plan](../../doc/isaac.md) tracks recording and replay milestones.

## Launch

Run from this directory:

```bash
just compatibility
just gui
```

Requires a running Docker daemon accessible to your user, an NVIDIA driver,
NVIDIA Container Toolkit configured for Docker, and X11 or XWayland for the local
window. `DISPLAY` and `XAUTHORITY` must describe your desktop session. The launcher
mounts the existing Xauthority cookie without changing `xhost` permissions.
`compatibility` runs NVIDIA's GPU/Vulkan checker; look for
`System checking result: PASSED`. Docker downloads the pinned image automatically
if needed; `just pull` downloads it separately.

Launching the container sets `ACCEPT_EULA=Y` for the
[NVIDIA EULA](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_container.html#container-deployment).
Privacy consent is disabled. Asset loading requires access to NVIDIA's servers.
Initial image downloads and shader compilation can take several minutes.

`just gui` opens an overview inside the warehouse, framing Jetbot's approach to
the parked forklift. The check settles the robot, verifies the floor and approach
with PhysX raycasts, and collects a 320 × 240 onboard image and three complete RTX
lidar scans at 10 Hz. It compares forward lidar returns with the forklift collider
and requires at least 350° of scan coverage. Simulation then pauses for inspection.

The viewport camera menu offers `/World/Overview` and the onboard camera at
`/World/jetbot/chassis/Camera`. Select the onboard camera to inspect the robot's
view of the forklift. The robot remains stationary during this sensor check.

For the same validation without a window:

```bash
just headless
```

A blocked approach, unsupported floor, unstable spawn, missing image, incomplete
scan, incorrect scan timing, or lidar/collider disagreement fails the check.

## Copper control

Run the native graph and local Isaac window together:

```bash
just run
```

ConsoleMon displays the live Copper graph in the terminal. Docker and Isaac stdout
and stderr go to `logs/isaac.log`, replaced on each run. To follow simulator output
from another terminal:

```bash
tail -f logs/isaac.log
```

Noninteractive runs use ConsoleMon's text output instead of the terminal UI.
The standalone `just gui`, `just headless`, and `just compatibility` checks keep
their output in the terminal.

The graph shows the camera frame number, lidar point count, mean RGB, forward
clearance in meters, and the controller's `DRIVE`, `STOP obstacle`, or
`STOP no clearance` status. The controller override shows `STOP override`.

The launcher builds `cu-isaac-sim` with the workspace's `debug-optimized` profile,
starts a native Unix socket server, and mounts
its temporary shared-memory directory into Docker at the same absolute path.
After the scene and sensor checks, Isaac advances physics until a fresh camera
frame and complete lidar scan are available, publishes their lease, and waits
for the matching Copper command before advancing again. `RobotClock::mock()`
tracks physics simulation time; sensor `Tov` uses rendered acquisition time and
the lidar scan interval. Camera timestamps come from the render product's
`IsaacReadSimulationTime` annotator.

The default GUI run keeps sensors, control, and ConsoleMon active until you close
the Isaac window. Jetbot stays stopped at the forklift while live telemetry
continues. Supply a count for a bounded GUI run (`just run gui 130`), which pauses
the window for inspection afterward. `just run headless` defaults to 130
observations (13 simulated seconds). The native
transport expects 320 × 240 RGB and 10 Hz observations. For a headless run, or to override the
ordinary controller with the simulation callback:

```bash
just run headless 30
just run headless 30 --override-controller=stop
```

`copperconfig.ron` wires camera → image statistics → telemetry and lidar →
forward clearance → stop controller → motor output. The hardware callbacks
inject sensors and collect outputs; the processing and controller tasks execute
normally. Clear returns farther than 0.65 m command both wheels at 5 rad/s;
near or absent returns command zero. The validated 130-observation run travels
1.59 m and stops with 0.68 m of collider clearance. The forced-stop override keeps processing
sensors but replaces the controller output with zero.

Shared camera bytes stay behind `CuImage<CuSharedMemoryBuffer<u8>>` handles.
The adapter converts at most 524,288 lidar points into a preallocated standard
`PointCloudSoaHandle`. Points are expressed in the lidar frame at the observation
boundary, with +X forward and +Z up; per-point acquisition times are retained.
The conversion uses the observed lidar pose, without per-point motion deskew.
Eight image/cloud slots cover four retained CopperLists. The pool grants an
image slot again only after all runtime and logger handles have released it.
Sequence errors, overflow, invalid timing, socket failures, and sensor timeouts
abort the exchange. Isaac sets zero wheel velocity and pauses on every exit.

`state/output/{gui,headless}/commands.jsonl` streams returned commands, simulation
times, clearance, and mean RGB. `control.json` records the final count and last
command; bounded runs also include their full command list. Continuous runs keep
that list on disk to bound memory use. `observations.jsonl` preserves each publication
sequence, sensor acquisition times, point count, and lidar pose. Full sensor payloads and derived messages are
logged under this example's `logs/cu-isaac-*.copper`, with a matching scenario
JSON. Session buffers are removed after shutdown. Replay and standalone sensor
inspection are the next milestone.

`state/output/{gui,headless}/performance.json` measures control-loop throughput,
simulation/wall-time ratio, rendered steps, sensor readback and packing, and time
waiting for native commands. Startup and evidence-image capture are excluded.
The native log's matching `.performance.json` measures point conversion, graph
iterations, shutdown, and dropped recording counts. Background serialization uses
Copper's preallocated CopperList slots; recording drains before the final simulator
acknowledgment, and dropped records fail the run.
The adapter waits for recording capacity before granting another sensor lease,
keeping storage backpressure outside the task graph and simulation time.

Headless runs disable the unused viewport, and the overview evidence image is
rendered after control completes. Copper-driven runs disable Isaac's wall-clock
frame cap while keeping fixed 60 Hz physics and 10 Hz sensor timestamps.

## Scene settings

[`isaac/scenario.json`](isaac/scenario.json) fixes the target, spawn, camera poses,
sensor mounts, and rates. Coordinates are meters in a +Z-up world; Jetbot's local
+X points forward. The selected collider is
`/World/warehouse/Forklift/S_ForkliftBody`. Jetbot spawns at `[7.5, -1.5, 0.04]`
with 180° yaw and approximately 2.25 m of clearance to the forklift.

Both sensors are parented to the simulated rigid chassis. Their local positions
are `[0.075, 0, 0.18]` for the camera and `[0, 0, 0.30]` for lidar. The camera looks
along chassis +X with +Z up. `Example_Rotary` runs at 10 Hz with full-scan
accumulation, Cartesian output, and world-frame points. Its minimum range is
overridden to 0.3 m so the 0.65 m stop threshold remains observable. Physics runs at 60 Hz.
The corridor check covers a 30 cm width and ends 55 cm before the forklift.

## State and evidence

Continuous GUI sessions finish with report status `stopped`; bounded checks use
`passed`. The overview image is captured at the end of bounded checks.

`state/` is ignored by Git and owned by the invoking host user. GUI and headless
checks write separate `state/output/{gui,headless}/` directories:

- `report.json`: status, fixed scenario, floor/corridor checks, settled pose, scan
  timestamps, point counts, angular coverage, and lidar/collider distances.
- `overview.png`: 1280 × 720 view of Jetbot and the approach.
- `camera.png`: 320 × 240 onboard RGB image.
- `lidar.npz`: last complete scan's `points_world_m` (N × 3, float32),
  `time_offset_ns`, and `timestamp_ns`. Add each point's offset to the timestamp
  to obtain its acquisition time in simulation nanoseconds.

Check the report's status before using evidence from a previous launch. These
files are simulator validation artifacts. Copper runs additionally produce
`control.json` and unified logs.

Caches persist across launches. Native NVIDIA logs live in `state/logs/` and
`state/kit/logs/`; the writable home and Kit data live in `state/home/` and
`state/kit/data/`. `just shell` opens the same container for inspection.
The container uses your host UID/GID with supplementary group `1234` to access
Isaac's installed files, and enables local Hub startup for asset caching.

The [NVIDIA container guide](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_container.html)
documents the image, compatibility checker, and local GUI workflow. Arch is
outside the documented Ubuntu platform matrix; the scene and sensor checks
exercise the configuration on the local host.
