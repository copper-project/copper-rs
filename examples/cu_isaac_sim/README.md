# Isaac Sim scene and sensors

Load NVIDIA's warehouse with forklifts, place Jetbot on a fixed approach, and
validate its onboard camera and RTX lidar. The example uses
`nvcr.io/nvidia/isaac-sim:6.1.0` directly with mounted Python code; no image build
is needed. The [integration plan](../../doc/isaac.md) describes the Copper adapter
and replay work that follows.

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

## Scene settings

[`isaac/scenario.json`](isaac/scenario.json) fixes the target, spawn, camera poses,
sensor mounts, and rates. Coordinates are meters in a +Z-up world; Jetbot's local
+X points forward. The selected collider is
`/World/warehouse/Forklift/S_ForkliftBody`. Jetbot spawns at `[7.5, -1.5, 0.04]`
with 180° yaw and approximately 2.25 m of clearance to the forklift.

Both sensors are parented to the simulated rigid chassis. Their local positions
are `[0.075, 0, 0.18]` for the camera and `[0, 0, 0.30]` for lidar. The camera looks
along chassis +X with +Z up. `Example_Rotary` runs at 10 Hz with full-scan
accumulation, Cartesian output, and world-frame points. Physics runs at 60 Hz.
The corridor check covers a 30 cm width and ends 55 cm before the forklift.

## State and evidence

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
files are simulator validation artifacts; Copper recording and replay follow in
the next milestone.

Caches persist across launches. Native NVIDIA logs live in `state/logs/` and
`state/kit/logs/`; the writable home and Kit data live in `state/home/` and
`state/kit/data/`. `just shell` opens the same container for inspection.
The container uses your host UID/GID with supplementary group `1234` to access
Isaac's installed files, and enables local Hub startup for asset caching.

The [NVIDIA container guide](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_container.html)
documents the image, compatibility checker, and local GUI workflow. Arch is
outside the documented Ubuntu platform matrix; the scene and sensor checks
exercise the configuration on the local host.
