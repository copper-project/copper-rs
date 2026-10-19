"""Fixed warehouse approach and chassis-mounted Isaac sensors."""

import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def load_scene():
    import omni.usd
    from isaacsim.storage.native import get_assets_root_path

    config = json.loads(Path(__file__).with_name("scenario.json").read_text())
    root = get_assets_root_path()
    if not root:
        raise RuntimeError("Isaac asset root unavailable.")
    assets = {
        "warehouse": root + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
        "jetbot": root + "/Isaac/Robots_Multiphysics/NVIDIA/Jetbot/jetbot.usda",
    }
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(config["physics_hz"])
    for name, asset in assets.items():
        prim = stage.DefinePrim(f"/World/{name}", "Xform")
        prim.GetReferences().AddReference(asset)
        if not any(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())):
            raise RuntimeError(f"No mesh geometry loaded from {asset}")
    robot = UsdGeom.Xformable(stage.GetPrimAtPath("/World/jetbot"))
    robot.ClearXformOpOrder()
    robot.AddTranslateOp().Set(Gf.Vec3d(*config["spawn_position_m"]))
    robot.AddRotateZOp().Set(config["spawn_yaw_deg"])
    if not stage.GetPrimAtPath(config["chassis"]).HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError("Jetbot chassis is not a rigid body.")
    target = stage.GetPrimAtPath(config["target"])
    if not target or not target.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError("Selected forklift body has no collision geometry.")
    camera = UsdGeom.Camera.Define(stage, "/World/Overview")
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    camera.CreateFocalLengthAttr(12.0)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*config["overview_eye_m"]),
            Gf.Vec3d(*config["overview_target_m"]),
            Gf.Vec3d(0, 0, 1),
        ).GetInverse()
    )
    return stage, config, assets


def create_sensors(stage, config):
    from isaacsim.sensors.experimental.rtx import CameraSensor, Lidar, LidarSensor, RtxCamera

    camera_path = config["chassis"] + "/Camera"
    camera = RtxCamera(camera_path, tick_rate=config["sensor_hz"])
    camera.camera.set_focal_lengths(12.0)
    camera.camera.set_clipping_ranges(0.01, 100.0)
    # USD cameras look along -Z with +Y up; the robot's forward axis is +X.
    eye = Gf.Vec3d(*config["camera_translation_m"])
    transform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_path))
    transform.ClearXformOpOrder()
    transform.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(eye, eye + Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    camera_sensor = CameraSensor(
        camera, resolution=tuple(config["camera_resolution_hw"]), annotators=["rgb"]
    )
    lidar = Lidar.create(
        config["chassis"] + "/Lidar",
        config=config["lidar_config"],
        translations=np.array(config["lidar_translation_m"]),
        tick_rate=config["sensor_hz"],
        accumulate_outputs=True,
        aux_output_level="FULL",
        attributes={
            "omni:sensor:Core:scanRateBaseHz": float(config["sensor_hz"]),
            "omni:sensor:Core:elementsCoordsType": "CARTESIAN",
            "omni:sensor:Core:outputFrameOfReference": "WORLD",
        },
    )
    lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])
    return camera_sensor, lidar_sensor


def world_transform(stage, path):
    return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def check_approach(stage, config):
    from omni.physx import get_physx_scene_query_interface

    query = get_physx_scene_query_interface()
    transform = world_transform(stage, config["chassis"])
    origin = transform.Transform(Gf.Vec3d(*config["lidar_translation_m"]))
    direction = transform.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
    side = transform.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
    hit = query.raycast_closest(tuple(origin), tuple(direction), 6.0)
    if not hit["hit"] or hit["collision"] != config["target"]:
        raise RuntimeError(f"Forward ray missed selected forklift: {hit}")
    distance = float(hit["distance"])
    if distance < 1.0:
        raise RuntimeError(f"Insufficient starting clearance: {distance:.3f} m")
    # Check a 30 cm wide corridor at wheel, chassis, and lidar heights.
    corridor_hits = []
    for offset in (-0.15, 0.0, 0.15):
        for height in (0.07, 0.15, float(origin[2])):
            start = Gf.Vec3d(origin[0], origin[1], height) + side * offset + direction * 0.20
            ray = query.raycast_closest(tuple(start), tuple(direction), distance - 0.55)
            if ray["hit"]:
                raise RuntimeError(f"Approach corridor blocked: {ray}")
            corridor_hits.append({"offset_m": offset, "height_m": height})
    floor_hits = []
    for forward in np.linspace(0.25, distance - 0.55, 9):
        for offset in (-0.15, 0.15):
            point = origin + direction * float(forward) + side * offset
            ray = query.raycast_closest((point[0], point[1], 0.5), (0.0, 0.0, -1.0), 1.0)
            if not ray["hit"] or abs(ray["position"][2]) > 0.02:
                raise RuntimeError(f"Approach lacks a level floor: {ray}")
            floor_hits.append({"position_m": list(ray["position"]), "collision": ray["collision"]})
    return {
        "target": config["target"],
        "target_distance_m": distance,
        "lidar_origin_world_m": list(origin),
        "forward_world": list(direction),
        "corridor_half_width_m": 0.15,
        "corridor_clearance_m": 0.55,
        "corridor_rays": len(corridor_hits),
        "floor_samples": floor_hits,
    }
