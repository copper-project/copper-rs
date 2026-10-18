"""Validate the fixed Jetbot approach, onboard RGB camera, and complete RTX lidar scans."""

import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("gui", "headless"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--observations", type=int, default=30)
    args = parser.parse_args()
    if not 1 <= args.observations <= 10_000:
        parser.error("--observations must be between 1 and 10000")
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "report.json"
    report = {"status": "starting", "mode": args.mode}
    report_path.write_text(json.dumps(report, indent=2) + "\n")

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.mode == "headless",
            "enable_motion_bvh": args.session is not None,
            "width": 1280,
            "height": 720,
            "extra_args": ["--/persistent/app/usd/muteUsdDiagnostics=false"],
        },
        experience="/isaac-sim/apps/isaacsim.exp.full.kit",
    )
    try:
        import numpy as np
        import omni.replicator.core as rep
        from isaacsim.core.api import SimulationContext
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
        from omni.kit.viewport.utility import get_active_viewport
        from PIL import Image
        from scene import check_approach, create_sensors, load_scene, world_transform

        started = time.monotonic()
        stage, config, assets = load_scene()
        report.update(status="warming_up", scenario=config, assets=assets)
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        camera, lidar = create_sensors(stage, config)
        overview_product = rep.create.render_product("/World/Overview", (1280, 720))
        overview_rgb = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
        overview_rgb.attach([overview_product])
        if args.mode == "gui":
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError("No active Isaac viewport.")
            viewport.camera_path = "/World/Overview"

        sim = SimulationContext(
            physics_dt=1.0 / config["physics_hz"],
            rendering_dt=1.0 / config["physics_hz"],
            stage_units_in_meters=1.0,
            backend="numpy",
        )
        sim.initialize_physics()
        initial = world_transform(stage, config["chassis"]).ExtractTranslation()
        sim.play()
        for _ in range(config["physics_hz"]):
            sim.step(render=True)
        settled = world_transform(stage, config["chassis"]).ExtractTranslation()
        displacement = np.asarray(settled) - np.asarray(initial)
        if np.linalg.norm(displacement[:2]) > 0.05 or abs(displacement[2]) > 0.10:
            raise RuntimeError(f"Jetbot did not settle at its spawn: displacement {displacement}")
        approach = check_approach(stage, config)
        report.update(status="checking_sensors", approach=approach, settled_chassis_world_m=list(settled))
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        print(f"Approach clear; forklift collider at {approach['target_distance_m']:.3f} m", flush=True)

        scans = []
        camera_frame = None
        last_timestamp = None
        capture_after_ns = round(sim.current_time * 1e9)
        points = None
        offsets = None
        # At most four simulated seconds, bounded additionally by wall-clock timeout.
        for _ in range(config["physics_hz"] * 4):
            if not app.is_running():
                raise RuntimeError("Isaac closed before sensor validation completed.")
            if time.monotonic() - started > args.timeout:
                raise TimeoutError("Sensor warmup exceeded the timeout.")
            sim.step(render=True)
            rgb_data, _ = camera.get_data("rgb")
            if rgb_data is not None:
                candidate = rgb_data.numpy()
                if (
                    candidate.shape == (*config["camera_resolution_hw"], 3)
                    and candidate.dtype == np.uint8 and np.ptp(candidate) > 0
                ):
                    camera_frame = candidate.copy()
            raw, _ = lidar.get_data("generic-model-output")
            if raw is None or raw.size == 0:
                continue
            gmo = parse_generic_model_output_data(raw)
            if (
                not gmo.numElements or not gmo.scanComplete
                or gmo.timestampNs < capture_after_ns or gmo.timestampNs == last_timestamp
            ):
                continue
            xyz = np.column_stack((gmo.x, gmo.y, gmo.z)).astype(np.float32)
            valid = np.isfinite(xyz).all(axis=1) & ((np.asarray(gmo.flags) & 64) != 0)
            points = np.ascontiguousarray(xyz[valid])
            offsets = np.asarray(gmo.timeOffsetNs)[valid].copy()
            if len(points) < 100:
                raise RuntimeError(f"Complete lidar scan has too few valid points: {len(points)}")
            origin = np.asarray(approach["lidar_origin_world_m"])
            direction = np.asarray(approach["forward_world"])
            delta = points - origin
            forward = delta @ direction
            lateral = delta @ np.array([-direction[1], direction[0], 0.0])
            band = (forward > 0.1) & (abs(lateral) < 0.08) & (abs(delta[:, 2]) < 0.10)
            if not np.any(band):
                raise RuntimeError("No lidar returns in the forward forklift sensing region.")
            nearest = float(forward[band].min())
            if abs(nearest - approach["target_distance_m"]) > 0.20:
                raise RuntimeError(f"Lidar/collider disagreement: {nearest:.3f} m vs {approach['target_distance_m']:.3f} m")
            azimuths = np.sort(np.arctan2(delta[:, 1], delta[:, 0]))
            largest_gap = np.diff(np.r_[azimuths, azimuths[0] + 2 * np.pi]).max()
            coverage = float(360.0 - np.degrees(largest_gap))
            if coverage < 350.0:
                raise RuntimeError(f"Complete lidar scan covers only {coverage:.1f}°")
            timestamp = int(gmo.timestampNs)
            period = 1.0 / config["sensor_hz"]
            if last_timestamp is not None and abs((timestamp - last_timestamp) / 1e9 - period) > 1.1 / config["physics_hz"]:
                raise RuntimeError("Lidar acquisition timestamps do not match the configured scan rate.")
            last_timestamp = timestamp
            scans.append({
                "scan_index": int(gmo.scanIdx),
                "timestamp_ns": timestamp,
                "point_time_min_ns": timestamp + int(offsets.min()),
                "point_time_max_ns": timestamp + int(offsets.max()),
                "complete": True,
                "valid_points": len(points),
                "azimuth_coverage_deg": round(coverage, 3),
                "forward_distance_m": nearest,
            })
            if len(scans) >= 3 and camera_frame is not None:
                break
        if camera_frame is None or len(scans) < 3:
            raise RuntimeError(f"Sensor validation incomplete: camera={camera_frame is not None}, complete_scans={len(scans)}")
        if args.session is not None:
            from transport import exchange

            report.update(status="running_copper")
            report_path.write_text(json.dumps(report, indent=2) + "\n")
            report["copper"] = exchange(
                app, sim, stage, config, camera, lidar,
                args.session, args.observations, args.output,
            )
        sim.pause()
        rep.orchestrator.step(rt_subframes=4, delta_time=0.0)
        overview = np.asarray(overview_rgb.get_data())[:, :, :3]
        if overview.shape != (720, 1280, 3) or not np.ptp(overview):
            raise RuntimeError("Overview camera produced no valid RGB frame.")
        Image.fromarray(overview).save(args.output / "overview.png")
        Image.fromarray(camera_frame).save(args.output / "camera.png")
        np.savez(
            args.output / "lidar.npz", points_world_m=points, time_offset_ns=offsets,
            timestamp_ns=last_timestamp,
        )
        report.update(
            status="passed", elapsed_seconds=round(time.monotonic() - started, 3),
            camera={"path": config["chassis"] + "/Camera", "resolution_hw": config["camera_resolution_hw"], "tick_hz": config["sensor_hz"]},
            lidar={"path": config["chassis"] + "/Lidar", "config": config["lidar_config"], "tick_hz": config["sensor_hz"], "points_frame": "world", "scans": scans},
        )
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        label = f"Copper run passed ({args.observations} observations)" if args.session else "Scene/sensor check passed"
        print(f"{label}: {args.output}", flush=True)
        if args.mode == "gui":
            viewport.camera_path = "/World/Overview"
            while app.is_running():
                app.update()
    except BaseException as error:
        report.update(status="failed", error=str(error))
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
