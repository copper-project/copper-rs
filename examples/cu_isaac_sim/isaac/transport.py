"""Exclusive Copper leases; metadata on the Unix socket, sensor arrays in mmap files."""

import json
import itertools
import mmap
from pathlib import Path
import socket
import time

MAX_METADATA = 65_536


class Connection:
    def __init__(self, session, timeout=60.0):
        self.session = Path(session).resolve()
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(timeout)
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.socket.connect(str(self.session / "control.sock"))
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() >= deadline:
                    self.socket.close()
                    raise TimeoutError("Copper control socket unavailable")
                time.sleep(0.02)
        self.reader = self.socket.makefile("rb")
        self.mappings = {}

    def send(self, message):
        encoded = json.dumps(message, allow_nan=False, separators=(",", ":")).encode() + b"\n"
        if len(encoded) > MAX_METADATA:
            raise RuntimeError("Control metadata exceeds protocol limit")
        self.socket.sendall(encoded)

    def receive(self, expected):
        line = self.reader.readline(MAX_METADATA + 1)
        if len(line) > MAX_METADATA or not line.endswith(b"\n"):
            raise RuntimeError("Copper disconnected or metadata overflow")
        message = json.loads(line)
        if message.get("type") != expected:
            raise RuntimeError(f"Expected {expected}, received {message}")
        return message

    def buffer(self, descriptor, expected_bytes):
        path = Path(descriptor["path"]).resolve()
        if (
            path.parent != self.session or not descriptor.get("__cu_shm_handle__")
            or descriptor["element_type"] != "u8"
            or descriptor["len_elements"] != expected_bytes
            or descriptor["offset_bytes"] < 0
        ):
            raise RuntimeError("Invalid Copper buffer descriptor")
        if path not in self.mappings:
            with path.open("r+b") as backing:
                self.mappings[path] = mmap.mmap(backing.fileno(), 0)
        mapping = self.mappings[path]
        offset = descriptor["offset_bytes"]
        if offset + expected_bytes > len(mapping):
            raise RuntimeError("Copper buffer descriptor exceeds backing file")
        return mapping, offset

    def close(self):
        self.reader.close()
        self.socket.close()
        for mapping in self.mappings.values():
            mapping.close()
        self.mappings.clear()


def exchange(app, sim, stage, config, camera, lidar, session, observations, output):
    """Advance Isaac only after a matching native command; stop locally on every exit."""
    import numpy as np
    import omni.replicator.core as rep
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
    from pxr import Gf
    from scene import world_transform

    robot = WheeledRobot(
        prim_path="/World/jetbot", name="copper_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    )
    robot.initialize()
    robot.post_reset()
    camera_time = rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime")
    camera_time.initialize(resetOnStop=True)
    camera_time.attach([str(camera.render_product.GetPath())])
    connection = None
    commands = []
    command_count = 0
    last_command = None
    closed = False
    initial_position = list(world_transform(stage, config["chassis"]).ExtractTranslation())
    final_position = initial_position
    observation_trace = (output / "observations.jsonl").open("w")
    command_trace = (output / "commands.jsonl").open("w")
    last_scan = round(sim.current_time * 1e9)
    last_camera = last_scan
    point_dtype = np.dtype([("xyz", "<f4", (3,)), ("intensity", "<f4"), ("offset_ns", "<u4")])
    performance = dict(render_steps=0, render_s=0.0, sensor_s=0.0, command_wait_s=0.0, lease_wait_s=0.0)
    started = None
    initial_simulation_time = sim.current_time
    try:
        connection = Connection(session)
        connection.send({"type": "hello", "version": 1, "observations": observations, "scenario": config})
        started = time.perf_counter()
        for sequence in range(observations) if observations else itertools.count():
            tick = time.perf_counter()
            lease = connection.receive("lease")
            performance["lease_wait_s"] += time.perf_counter() - tick
            if lease["sequence"] != sequence or lease["point_bytes"] != point_dtype.itemsize:
                raise RuntimeError("Unexpected lease sequence or lidar layout")
            camera_mapping, camera_offset = connection.buffer(lease["camera"], 320 * 240 * 3)
            lidar_mapping, lidar_offset = connection.buffer(lease["lidar"], lease["max_points"] * point_dtype.itemsize)
            deadline = time.monotonic() + 60.0
            for _ in range(config["physics_hz"]):
                if not app.is_running():
                    if observations == 0:
                        closed = True
                        break
                    raise RuntimeError("Simulator closed before the requested observations completed")
                if time.monotonic() > deadline:
                    raise RuntimeError("Simulator closed or sensor acquisition timed out")
                tick = time.perf_counter()
                sim.step(render=True)
                performance["render_s"] += time.perf_counter() - tick
                performance["render_steps"] += 1
                if not app.is_running():
                    if observations == 0:
                        closed = True
                        break
                    raise RuntimeError("Simulator closed before the requested observations completed")
                tick = time.perf_counter()
                raw, _ = lidar.get_data("generic-model-output")
                if raw is None or raw.size == 0:
                    performance["sensor_s"] += time.perf_counter() - tick
                    continue
                gmo = parse_generic_model_output_data(raw)
                stamp = int(gmo.timestampNs)
                if not gmo.scanComplete or not gmo.numElements or stamp <= last_scan:
                    performance["sensor_s"] += time.perf_counter() - tick
                    continue
                rgb, _ = camera.get_data("rgb")
                timing = camera_time.get_data()
                if rgb is None or not timing or "simulationTime" not in timing:
                    performance["sensor_s"] += time.perf_counter() - tick
                    continue
                camera_stamp = round(timing["simulationTime"] * 1e9)
                if camera_stamp <= last_camera:
                    performance["sensor_s"] += time.perf_counter() - tick
                    continue
                xyz = np.column_stack((gmo.x, gmo.y, gmo.z)).astype(np.float32)
                intensity = np.asarray(gmo.scalar)
                valid = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity) & ((np.asarray(gmo.flags) & 64) != 0)
                count = int(valid.sum())
                if count == 0 or count > lease["max_points"]:
                    raise RuntimeError(f"Lidar lease capacity exceeded or scan empty: {count}")
                offsets = np.asarray(gmo.timeOffsetNs)[valid]
                if np.any(offsets < 0) or np.any(offsets > np.iinfo(np.uint32).max):
                    raise RuntimeError("Lidar acquisition offset exceeds protocol capacity")
                scan_end = stamp + int(offsets.max())
                now = round(sim.current_time * 1e9)
                # The renderer can expose a scan ahead of the physics boundary while accumulating.
                # Publish only completed acquisitions whose whole interval is now in the past.
                if scan_end > now or camera_stamp > now:
                    performance["sensor_s"] += time.perf_counter() - tick
                    continue
                image = rgb.numpy()
                if image.shape != (240, 320, 3) or image.dtype != np.uint8:
                    raise RuntimeError(f"Unexpected camera layout: {image.shape} {image.dtype}")
                destination = np.ndarray(image.shape, dtype=np.uint8, buffer=camera_mapping, offset=camera_offset)
                np.copyto(destination, image)
                del destination
                destination = np.ndarray((count,), dtype=point_dtype, buffer=lidar_mapping, offset=lidar_offset)
                destination["xyz"] = xyz[valid]
                destination["intensity"] = intensity[valid]
                destination["offset_ns"] = offsets
                del destination
                pose = world_transform(stage, config["chassis"] + "/Lidar")
                axes = [list(pose.TransformDir(axis).GetNormalized()) for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))]
                publication = {
                    "type": "frame", "sequence": sequence, "simulation_time_ns": now,
                    "camera_time_ns": camera_stamp, "scan_start_ns": stamp,
                    "scan_end_ns": scan_end, "points": count,
                    "origin_world_m": list(pose.ExtractTranslation()), "axes_world": axes,
                }
                observation_trace.write(json.dumps(publication, allow_nan=False) + "\n")
                performance["sensor_s"] += time.perf_counter() - tick
                tick = time.perf_counter()
                connection.send(publication)
                # All NumPy views have been discarded. No writes until Rust grants another lease.
                command = connection.receive("command")
                performance["command_wait_s"] += time.perf_counter() - tick
                if command["sequence"] != sequence or command["simulation_time_ns"] != now:
                    raise RuntimeError("Stale Copper command")
                velocities = np.array([command["left_rad_s"], command["right_rad_s"]])
                if not np.isfinite(velocities).all() or (abs(velocities) > 10.0).any():
                    raise RuntimeError("Invalid Copper wheel command")
                robot.apply_wheel_actions(ArticulationAction(joint_velocities=velocities))
                command_trace.write(json.dumps(command, allow_nan=False) + "\n")
                command_count += 1
                last_command = command
                # Bounded runs retain their existing summary; continuous runs stream to disk.
                if observations:
                    commands.append(command)
                final_position = list(world_transform(stage, config["chassis"]).ExtractTranslation())
                last_scan, last_camera = stamp, camera_stamp
                break
            else:
                raise RuntimeError(f"No fresh complete sensor observation within one simulated second: camera={camera_time.get_data()}, last_scan={last_scan}")
            if closed:
                break
        # Stop before the final handshake. Peer exit is safe even if its reply is lost.
        if app.is_running():
            robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.zeros(2)))
        connection.send({"type": "finish"})
        finished = connection.receive("finished")
        if finished["observations"] != command_count:
            raise RuntimeError("Copper iteration count differs from observations")
        hit = {"hit": False}
        if app.is_running():
            from omni.physx import get_physx_scene_query_interface

            pose = world_transform(stage, config["chassis"] + "/Lidar")
            direction = pose.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
            hit = get_physx_scene_query_interface().raycast_closest(tuple(pose.ExtractTranslation()), tuple(direction), 6.0)
        return {
            "observations": command_count, "commands": commands, "last_command": last_command,
            "log": finished["log"],
            "initial_chassis_world_m": initial_position,
            "final_chassis_world_m": final_position,
            "travel_m": float(np.linalg.norm(np.asarray(final_position) - initial_position)),
            "final_collider": hit["collision"] if hit["hit"] else None,
            "final_collider_distance_m": float(hit["distance"]) if hit["hit"] else None,
        }
    finally:
        elapsed = time.perf_counter() - started if started is not None else None
        # Closing the window resets SimulationContext's time; use the last accepted frame.
        simulation_s = (
            last_command["simulation_time_ns"] / 1e9 - initial_simulation_time
            if last_command is not None else 0.0
        )
        observation_trace.close()
        command_trace.close()
        if app.is_running():
            robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.zeros(2)))
            sim.pause()
            camera_time.detach()
        if connection is not None:
            connection.close()
        (output / "control.json").write_text(json.dumps({
            "observations": command_count, "commands": commands, "last_command": last_command,
        }, indent=2) + "\n")
        if elapsed is not None:
            performance.update(
                observations=command_count, elapsed_s=elapsed, simulation_s=simulation_s,
                observations_per_second=command_count / elapsed,
                realtime_factor=simulation_s / elapsed,
            )
            (output / "performance.json").write_text(json.dumps(performance, indent=2) + "\n")
