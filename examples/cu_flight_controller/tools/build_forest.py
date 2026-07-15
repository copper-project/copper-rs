"""Build the local flight-controller forest scene and a spawn-point preview.

Run through `just forest-assets`; the script is deterministic so geometry and
tree placement can be reviewed or regenerated without hand-editing the GLB.
"""

from __future__ import annotations

import math
import random
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
SOURCE = ASSETS / "forest_source"
BLEND_OUTPUT = ASSETS / "forest_intermediate.blend"
GLB_OUTPUT = ASSETS / "forest.glb"
PREVIEW_OUTPUT = ASSETS / "forest_spawn_preview.png"

SEED = 0xC029
SPAWN_X = -10.0
# Blender +Y exports to glTF/Bevy -Z. The quad starts at Bevy Z=20.
SPAWN_Y = -20.0

TERRAIN_X_MIN = -115.0
TERRAIN_X_MAX = 115.0
TERRAIN_Y_MIN = -90.0
TERRAIN_Y_MAX = 545.0
TERRAIN_X_STEPS = 59
TERRAIN_Y_STEPS = 160


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        if collection.name != "Collection":
            bpy.data.collections.remove(collection)
    root_collection = bpy.context.scene.collection.children.get("Collection")
    if root_collection is not None:
        root_collection.name = "ForestWorld"


def make_material(
    name: str,
    color: tuple[float, float, float, float],
    roughness: float = 0.9,
    metallic: float = 0.0,
) -> bpy.types.Material:
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    material.diffuse_color = color
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    return material


def raw_terrain_height(x: float, y: float) -> float:
    rolling = 1.55 * math.sin((y + 35.0) / 63.0)
    crossing = 1.05 * math.sin((x - 12.0) / 31.0) * math.cos((y + 10.0) / 48.0)
    long_wave = 0.8 * math.sin((x + y) / 82.0)
    ridge_west = 2.1 * math.exp(-(((x + 72.0) / 42.0) ** 2 + ((y - 245.0) / 150.0) ** 2))
    ridge_east = 1.7 * math.exp(-(((x - 70.0) / 38.0) ** 2 + ((y - 390.0) / 120.0) ** 2))
    hollow = -1.2 * math.exp(-(((x + 5.0) / 55.0) ** 2 + ((y - 330.0) / 85.0) ** 2))
    return rolling + crossing + long_wave + ridge_west + ridge_east + hollow


RAW_SPAWN_HEIGHT = raw_terrain_height(SPAWN_X, SPAWN_Y)


def terrain_height(x: float, y: float) -> float:
    distance = math.hypot(x - SPAWN_X, y - SPAWN_Y)
    flatten = min(1.0, max(0.0, (distance - 12.0) / 35.0))
    flatten = flatten * flatten * (3.0 - 2.0 * flatten)
    return (raw_terrain_height(x, y) - RAW_SPAWN_HEIGHT) * flatten


def link_object(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    for owner in list(obj.users_collection):
        owner.objects.unlink(obj)
    collection.objects.link(obj)


def create_terrain(material: bpy.types.Material) -> bpy.types.Object:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int, int]] = []
    for row_index in range(TERRAIN_Y_STEPS):
        y = (
            TERRAIN_Y_MIN
            + (TERRAIN_Y_MAX - TERRAIN_Y_MIN) * row_index / (TERRAIN_Y_STEPS - 1)
        )
        for column_index in range(TERRAIN_X_STEPS):
            x = (
                TERRAIN_X_MIN
                + (TERRAIN_X_MAX - TERRAIN_X_MIN)
                * column_index
                / (TERRAIN_X_STEPS - 1)
            )
            vertices.append((x, y, terrain_height(x, y)))
    for row_index in range(TERRAIN_Y_STEPS - 1):
        for column_index in range(TERRAIN_X_STEPS - 1):
            a = row_index * TERRAIN_X_STEPS + column_index
            b = a + 1
            c = a + TERRAIN_X_STEPS + 1
            d = a + TERRAIN_X_STEPS
            faces.append((a, b, c, d))

    mesh = bpy.data.meshes.new("ForestTerrainMesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(material)
    mesh.update()
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    terrain = bpy.data.objects.new("ForestTerrain", mesh)
    return terrain


def create_launch_clearing(material: bpy.types.Material) -> bpy.types.Object:
    segments = 48
    radius = 11.0
    z = terrain_height(SPAWN_X, SPAWN_Y) + 0.025
    vertices = [(SPAWN_X, SPAWN_Y, z)]
    for index in range(segments):
        angle = math.tau * index / segments
        uneven_radius = radius * (0.94 + 0.06 * math.sin(angle * 5.0))
        x = SPAWN_X + uneven_radius * math.cos(angle)
        y = SPAWN_Y + uneven_radius * math.sin(angle)
        vertices.append((x, y, terrain_height(x, y) + 0.03))
    faces = []
    for index in range(segments):
        faces.append((0, index + 1, (index + 1) % segments + 1))
    mesh = bpy.data.meshes.new("LaunchClearingMesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(material)
    mesh.update()
    return bpy.data.objects.new("LaunchClearing", mesh)


def create_trail(material: bpy.types.Material) -> bpy.types.Object:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int, int]] = []
    segments = 85
    for index in range(segments):
        t = index / (segments - 1)
        y = SPAWN_Y + 505.0 * t
        center_x = SPAWN_X + 6.0 * math.sin(t * math.tau * 1.35) + 2.0 * math.sin(t * math.tau * 4.0)
        half_width = 2.2 - 0.7 * t
        for side in (-1.0, 1.0):
            x = center_x + side * half_width
            vertices.append((x, y, terrain_height(x, y) + 0.035))
    for index in range(segments - 1):
        a = index * 2
        faces.append((a, a + 1, a + 3, a + 2))
    mesh = bpy.data.meshes.new("ForestTrailMesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(material)
    mesh.update()
    return bpy.data.objects.new("ForestTrail", mesh)


def import_prototype(
    filename: str,
    collection: bpy.types.Collection,
    bark_material: bpy.types.Material,
    foliage_material: bpy.types.Material,
    stone_material: bpy.types.Material,
) -> list[bpy.types.Object]:
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(SOURCE / filename))
    imported = [obj for obj in bpy.data.objects if obj not in before]
    meshes = [obj for obj in imported if obj.type == "MESH"]
    if not meshes:
        raise RuntimeError(f"{filename} did not contain a mesh")
    for obj in imported:
        if obj.type != "MESH":
            bpy.data.objects.remove(obj, do_unlink=True)
            continue
        link_object(obj, collection)
        obj.name = f"SOURCE_{Path(filename).stem}_{obj.name}"
        for slot in obj.material_slots:
            source_name = slot.material.name.lower() if slot.material else ""
            if "wood" in source_name or "bark" in source_name:
                slot.material = bark_material
            elif "leaf" in source_name or "grass" in source_name or "plant" in source_name:
                slot.material = foliage_material
            else:
                slot.material = stone_material
        obj.hide_render = True
        obj.hide_set(True)
    return meshes


def prototype_bottom(prototype: list[bpy.types.Object]) -> float:
    return min(corner[2] for obj in prototype for corner in obj.bound_box)


def spawn_prototype(
    prototype: list[bpy.types.Object],
    collection: bpy.types.Collection,
    name: str,
    x: float,
    y: float,
    scale: float,
    yaw: float,
) -> None:
    bottom = prototype_bottom(prototype)
    base_z = terrain_height(x, y) - bottom * scale
    for part_index, source_obj in enumerate(prototype):
        obj = source_obj.copy()
        obj.data = source_obj.data
        obj.name = name if len(prototype) == 1 else f"{name}_{part_index}"
        obj.hide_render = False
        obj.hide_viewport = False
        obj.hide_set(False)
        obj.location = (x, y, base_z)
        obj.scale = (scale, scale, scale)
        obj.rotation_euler[2] += yaw
        collection.objects.link(obj)


def populate_forest(
    rng: random.Random,
    tree_prototypes: list[list[bpy.types.Object]],
    rock_prototypes: list[list[bpy.types.Object]],
    bush_prototypes: list[list[bpy.types.Object]],
    trees: bpy.types.Collection,
    understory: bpy.types.Collection,
) -> tuple[int, int, int]:
    tree_count = 0
    spacing = 11.7
    y = TERRAIN_Y_MIN + 9.0
    row = 0
    while y < TERRAIN_Y_MAX - 8.0:
        x = TERRAIN_X_MIN + 8.0 + (row % 2) * spacing * 0.5
        while x < TERRAIN_X_MAX - 8.0:
            tx = x + rng.uniform(-3.2, 3.2)
            ty = y + rng.uniform(-3.2, 3.2)
            spawn_distance = math.hypot(tx - SPAWN_X, ty - SPAWN_Y)
            trail_x = SPAWN_X + 6.0 * math.sin(max(0.0, (ty - SPAWN_Y) / 505.0) * math.tau * 1.35)
            in_launch_funnel = ty < 12.0 and abs(tx - SPAWN_X) < 7.0
            on_trail = abs(tx - trail_x) < 2.8 and rng.random() < 0.8
            edge_density_bonus = abs(tx) / TERRAIN_X_MAX * 0.12
            if (
                spawn_distance > 17.0
                and not in_launch_funnel
                and not on_trail
                and rng.random() > 0.23 - edge_density_bonus
            ):
                prototype = rng.choice(tree_prototypes)
                scale = rng.uniform(8.4, 12.8)
                spawn_prototype(
                    prototype,
                    trees,
                    f"Pine_{tree_count:04d}",
                    tx,
                    ty,
                    scale,
                    rng.uniform(0.0, math.tau),
                )
                tree_count += 1
            x += spacing
        y += spacing
        row += 1

    rock_count = 0
    for _ in range(72):
        x = rng.uniform(TERRAIN_X_MIN + 6.0, TERRAIN_X_MAX - 6.0)
        y = rng.uniform(TERRAIN_Y_MIN + 6.0, TERRAIN_Y_MAX - 6.0)
        if math.hypot(x - SPAWN_X, y - SPAWN_Y) < 18.0:
            continue
        spawn_prototype(
            rng.choice(rock_prototypes),
            understory,
            f"Rock_{rock_count:03d}",
            x,
            y,
            rng.uniform(1.2, 3.8),
            rng.uniform(0.0, math.tau),
        )
        rock_count += 1

    bush_count = 0
    for _ in range(180):
        x = rng.uniform(TERRAIN_X_MIN + 4.0, TERRAIN_X_MAX - 4.0)
        y = rng.uniform(TERRAIN_Y_MIN + 4.0, TERRAIN_Y_MAX - 4.0)
        if math.hypot(x - SPAWN_X, y - SPAWN_Y) < 14.0:
            continue
        spawn_prototype(
            rng.choice(bush_prototypes),
            understory,
            f"Bush_{bush_count:03d}",
            x,
            y,
            rng.uniform(1.2, 2.8),
            rng.uniform(0.0, math.tau),
        )
        bush_count += 1

    return tree_count, rock_count, bush_count


def look_at(obj: bpy.types.Object, target: Vector) -> None:
    obj.rotation_euler = (target - obj.location).to_track_quat("-Z", "Y").to_euler()


def setup_preview() -> None:
    scene = bpy.context.scene
    try:
        scene.render.engine = "BLENDER_EEVEE_NEXT"
    except TypeError:
        scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(PREVIEW_OUTPUT)
    scene.render.film_transparent = False
    scene.render.image_settings.color_mode = "RGBA"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = 1.15

    world = bpy.data.worlds.new("ForestMistWorld")
    scene.world = world
    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    background = nodes.get("Background")
    background.inputs["Color"].default_value = (0.32, 0.44, 0.48, 1.0)
    background.inputs["Strength"].default_value = 0.85

    fog_material = bpy.data.materials.new(name="Preview atmospheric mist")
    fog_material.use_nodes = True
    fog_nodes = fog_material.node_tree.nodes
    fog_links = fog_material.node_tree.links
    fog_nodes.remove(fog_nodes.get("Principled BSDF"))
    fog_volume = fog_nodes.new("ShaderNodeVolumePrincipled")
    fog_volume.inputs["Density"].default_value = 0.0022
    fog_volume.inputs["Color"].default_value = (0.62, 0.72, 0.68, 1.0)
    fog_volume.inputs["Anisotropy"].default_value = 0.16
    fog_links.new(fog_volume.outputs["Volume"], fog_nodes.get("Material Output").inputs["Volume"])
    bpy.ops.mesh.primitive_cube_add(
        location=(0.0, (TERRAIN_Y_MIN + TERRAIN_Y_MAX) * 0.5, 23.0),
        scale=(118.0, (TERRAIN_Y_MAX - TERRAIN_Y_MIN) * 0.5, 25.0),
    )
    fog_box = bpy.context.object
    fog_box.name = "PreviewFogVolume"
    fog_box.data.materials.append(fog_material)

    bpy.ops.object.light_add(type="SUN", location=(30.0, -20.0, 80.0))
    sun = bpy.context.object
    sun.name = "PreviewSun"
    sun.data.energy = 4.0
    sun.data.angle = math.radians(18.0)
    sun.rotation_euler = (math.radians(28.0), math.radians(-22.0), math.radians(-28.0))

    bpy.ops.object.light_add(type="AREA", location=(-8.0, -12.0, 9.0))
    fill = bpy.context.object
    fill.name = "PreviewClearingFill"
    fill.data.energy = 1_500.0
    fill.data.shape = "DISK"
    fill.data.size = 16.0
    look_at(fill, Vector((SPAWN_X, 8.0, 2.5)))

    camera_data = bpy.data.cameras.new("SpawnPreviewCamera")
    camera = bpy.data.objects.new("SpawnPreviewCamera", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = (SPAWN_X - 1.2, SPAWN_Y - 4.5, 2.7)
    camera_data.lens = 28.0
    camera_data.sensor_width = 32.0
    camera_data.clip_start = 0.05
    camera_data.clip_end = 800.0
    look_at(camera, Vector((SPAWN_X + 2.0, SPAWN_Y + 38.0, 4.3)))
    scene.camera = camera


def export_glb(export_collections: list[bpy.types.Collection]) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    selected = []
    for collection in export_collections:
        for obj in collection.all_objects:
            if obj.type == "MESH":
                obj.hide_set(False)
                obj.select_set(True)
                selected.append(obj)
    if not selected:
        raise RuntimeError("forest export did not select any meshes")
    bpy.context.view_layer.objects.active = selected[0]
    bpy.ops.export_scene.gltf(
        filepath=str(GLB_OUTPUT),
        export_format="GLB",
        use_selection=True,
        export_cameras=False,
        export_lights=False,
        export_yup=True,
        export_apply=True,
        export_materials="EXPORT",
    )


def main() -> None:
    clear_scene()
    rng = random.Random(SEED)
    scene_root = bpy.context.scene.collection.children["ForestWorld"]
    source_collection = bpy.data.collections.new("SourcePrototypes")
    terrain_collection = bpy.data.collections.new("Terrain")
    tree_collection = bpy.data.collections.new("Trees")
    understory_collection = bpy.data.collections.new("Understory")
    for collection in (source_collection, terrain_collection, tree_collection, understory_collection):
        scene_root.children.link(collection)
    source_collection.hide_render = True

    floor_material = make_material("Forest floor", (0.16, 0.25, 0.09, 1.0), 1.0)
    clearing_material = make_material("Needle clearing", (0.26, 0.18, 0.085, 1.0), 1.0)
    trail_material = make_material("Forest trail", (0.20, 0.12, 0.055, 1.0), 1.0)
    bark_material = make_material("Pine bark", (0.22, 0.095, 0.04, 1.0), 0.98)
    stone_material = make_material("Mossy stone", (0.30, 0.35, 0.23, 1.0), 0.96)
    foliage_materials = [
        make_material("Pine needles deep", (0.045, 0.25, 0.085, 1.0), 0.92),
        make_material("Pine needles blue", (0.055, 0.29, 0.15, 1.0), 0.92),
        make_material("Pine needles light", (0.11, 0.34, 0.13, 1.0), 0.94),
        make_material("Pine needles warm", (0.13, 0.29, 0.075, 1.0), 0.94),
    ]

    terrain_collection.objects.link(create_terrain(floor_material))
    terrain_collection.objects.link(create_launch_clearing(clearing_material))
    terrain_collection.objects.link(create_trail(trail_material))

    tree_files = [
        "tree_pineTallA_detailed.glb",
        "tree_pineTallB_detailed.glb",
        "tree_pineTallC_detailed.glb",
        "tree_pineTallD_detailed.glb",
        "tree_pineRoundA.glb",
        "tree_pineRoundC.glb",
    ]
    tree_prototypes = [
        import_prototype(
            filename,
            source_collection,
            bark_material,
            foliage_materials[index % len(foliage_materials)],
            stone_material,
        )
        for index, filename in enumerate(tree_files)
    ]
    rock_prototypes = [
        import_prototype(
            filename,
            source_collection,
            bark_material,
            foliage_materials[0],
            stone_material,
        )
        for filename in ("rock_largeA.glb", "rock_largeC.glb", "rock_tallB.glb")
    ]
    bush_prototypes = [
        import_prototype(
            filename,
            source_collection,
            bark_material,
            foliage_materials[2 + index],
            stone_material,
        )
        for index, filename in enumerate(("plant_bush.glb", "plant_bushDetailed.glb"))
    ]

    counts = populate_forest(
        rng,
        tree_prototypes,
        rock_prototypes,
        bush_prototypes,
        tree_collection,
        understory_collection,
    )
    print(f"Generated forest: {counts[0]} trees, {counts[1]} rocks, {counts[2]} bushes")

    setup_preview()
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_OUTPUT))
    export_glb([terrain_collection, tree_collection, understory_collection])
    bpy.context.scene.render.filepath = str(PREVIEW_OUTPUT)
    bpy.ops.render.render(write_still=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_OUTPUT))


if __name__ == "__main__":
    main()
