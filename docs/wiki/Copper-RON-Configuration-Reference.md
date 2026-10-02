# Copper RON Configuration Reference

This guide explains how to configure a Copper application using RON files. It summarises the top-level sections and options supported by the current runtime configuration schema.

## Basic Structure

A Copper configuration is a RON file containing tasks, connections and optional sections such as resources, bridges, monitoring, logging, missions, includes, or runtime settings. The minimal form looks like:

```ron
(
    tasks: [],
    cnx: [],
)
```

Most examples name the file `copperconfig.ron` and reference it with the `#[copper_runtime(config = "...")]` attribute.

## Compile-Time Constants

The optional top-level `constants` array generates Rust constants as part of
`#[copper_runtime]`. A numeric constant has a required `id` and `value`, plus optional
placement and representation fields:

```ron
constants: [
    (id: "MAX_DETECTIONS", storage: usize, value: 64),
    (
        id: "CAMERA_OFFSET",
        module: "frames::camera",
        quantity: length,
        unit: millimeter,
        value: [180.0, 0.0, 310.0],
    ),
]
```

- `module` defaults to `constants`; an explicit relative Rust module path creates a
  generated module tree.
- `storage` defaults to `f32` and accepts Rust integer types, `f32`, or `f64`.
- `quantity` selects an SI quantity type. `unit` defaults to that quantity's coherent SI
  unit, and configured values are normalized when compiled.

A typed expression constant replaces `value` and all numeric representation fields with
both `type` and `expression`:

```ron
(
    id: "WORLD_TO_ROBOT",
    module: "transforms",
    type: "cu_transform::TypedTransform3D<f32, cu_transform::WorldFrame, cu_transform::RobotFrame>",
    expression: "cu_transform::TypedTransform3D::<f32, cu_transform::WorldFrame, cu_transform::RobotFrame>::from_translation_euler_xyz(crate::frames::robot::TRANSLATION, crate::frames::robot::ROTATION)",
)
```

The expression may call const constructors and methods, use struct literals, and reference
other constants. Rust checks the declared type and const-evaluability. Paths use normal Rust
scope in the generated module; prefer dependency-qualified paths and `crate::...` for
application or generated items.

Constants are baked into the binary and merge through `includes` with root-first
precedence on `(module, id)`. A runtime configuration that changes a compiled constant
produces a warning and does not replace the baked value.

## Tasks

Tasks are declared under the `tasks` array. Each task entry is a tuple with the following fields:

- `id` – unique identifier of the task.
- `type` – fully qualified Rust type of the task implementation.
- `kind` *(optional but recommended)* – declares which Copper task trait the type implements:
  `source`, `task`, or `sink`.
- `config` *(optional)* – map of parameters passed to the task.
- `resources` *(optional)* – map of task-local resource bindings to `bundle.resource` references.
- `missions` *(optional)* – list of mission ids in which this task is active.
- `background` *(optional)* – boolean. `true` runs an eligible source or regular task on the background thread pool. Omit it or set `false` for foreground execution.
- `run_in_sim` *(optional)* – boolean controlling whether a source or sink uses the real implementation in simulation mode instead of a generated stub. **Since v1.1:** generated source stubs preserve the complete output pack for multi-output sources, so the simulation callback can fill every port in place. It is ignored for regular tasks.
- `logging` *(optional)* – per-task message logging settings:
  - `enabled` – boolean, defaults to `true`.
  - `codec` – default logging codec id for this task's outputs.
  - `codecs` – map from message type to logging codec id.

Example:

```ron
(
    id: "task1",
    type: "tasks::ExampleTask",
    kind: task,
    background: true,
    resources: { "i2c": "board.i2c0" },
    logging: (enabled: false),
)
```

`kind` avoids relying on DAG shape to distinguish `CuSrcTask`, `CuTask`, and
`CuSinkTask`. Older configs still infer this when the graph is unambiguous, but explicit
`kind` is the preferred form.

## Resources

Top-level `resources` declare resource bundles that tasks and bridges can bind to:

```ron
resources: [
    (
        id: "board",
        provider: "cu_linux_resources::LinuxResources",
        config: { "serial_device": "/dev/ttyUSB0" },
    ),
]
```

Each resource bundle entry supports:

- `id` – bundle id used by task and bridge `resources` maps.
- `provider` – Rust type that provides the bundle.
- `config` *(optional)* – provider-specific configuration map.
- `missions` *(optional)* – mission ids in which the bundle is available.

Tasks and bridges refer to bundle slots with `bundle.resource` strings:

```ron
resources: { "serial": "board.serial0" }
```

## Bridges

Top-level `bridges` declare bridge components and their graph-facing channels:

```ron
bridges: [
    (
        id: "radio",
        type: "bridges::RadioBridge",
        resources: { "serial": "board.serial0" },
        channels: [
            Rx(id: "telemetry_in", route: "robot/telemetry"),
            Tx(id: "commands_out", route: "robot/commands"),
        ],
    ),
]
```

Each bridge supports:

- `id` – bridge id.
- `type` – Rust bridge type.
- `config` *(optional)* – bridge-specific configuration map.
- `resources` *(optional)* – bridge-local resource bindings to `bundle.resource` references.
- `missions` *(optional)* – mission ids in which the bridge is active.
- `run_in_sim` *(optional)* – boolean. Bridges run as their real implementation in simulation by default.
- `channels` – list of `Rx(...)` and `Tx(...)` channel declarations.

Each `Rx` or `Tx` channel has an `id`, plus optional `route` and `config` fields. `Rx` channels produce messages into the Copper graph and can be used as connection sources. `Tx` channels consume messages from the graph and can be used as connection destinations.

## Connections

Connections are declared under `cnx` as tuples describing edges between tasks and bridge channels:

- `src` – id of the source task.
- `dst` – id of the destination task.
- `msg` – Rust type of the message carried.
- `missions` *(optional)* – restrict the connection to specific missions.

Bridge channel endpoints use `bridge_id/channel_id` syntax. A bridge `Rx` channel can appear as `src`; a bridge `Tx` channel can appear as `dst`.

If a source or regular task intentionally produces an output that nobody consumes in the
current mission, declare it explicitly with `dst: "__nc__"`. That keeps the output message
type known to the runtime without adding a fake sink.

Example:

```ron
(src: "task0", dst: "task1", msg: "i32")
(src: "radio/telemetry_in", dst: "task1", msg: "messages::Telemetry")
(src: "task1", dst: "radio/commands_out", msg: "messages::Command")
```

## Monitoring

The optional `monitor` section selects one monitoring component and its parameters. The `monitors` spelling accepts a list:

```ron
monitor: (
    type: "cu_consolemon::CuConsoleMon",
    config: { "verbosity": 2 },
)

monitors: [
    (
        type: "cu_consolemon::CuConsoleMon",
        config: { "verbosity": 2 },
    ),
]
```

## Logging

The `logging` section tunes Copper’s structured log. Options mirror the fields of `LoggingConfig` defined in `core/cu29_runtime/src/config.rs`:

- `enable_task_logging` – controls per-task logging (defaults to `true`).
- `copperlist_count` – number of preallocated CopperLists compiled into the runtime. If set, it must be at least `1`.
- `slab_size_mib` – size of each memory mapped slab in MiB.
- `section_size_mib` – pre‑allocated size per log section in MiB.
- `keyframe_interval` – number of copperlists between two keyframes (default `100`).
- `codecs` – named logging codec specs reusable by per-task `logging.codec` and `logging.codecs`.

Example from `examples/cu_logging_size`:

```ron
logging: (
    slab_size_mib: 1024,
    section_size_mib: 100,
)
```

Example with codecs:

```ron
(
    logging: (
        slab_size_mib: 64,
        section_size_mib: 8,
        codecs: [
            (
                id: "png",
                type: "cu_png_codec::CuPngCodec",
            ),
        ],
    ),
    tasks: [
        (
            id: "image_task",
            type: "tasks::ImagePassThrough",
            logging: (
                codecs: {
                    "cu_sensor_payloads::CuImage<Vec<u8>>": "png",
                },
            ),
        ),
    ],
    cnx: [],
)
```

The library validates that `section_size_mib` does not exceed `slab_size_mib`. Log file paths are supplied by the generated app builder, typically with `with_log_path(...)`; `file` and `level` are not `logging` RON fields.

## Runtime Settings

Runtime behaviour can be adjusted with the `runtime` section. Currently only `rate_target_hz` is available:

```ron
runtime: (
    rate_target_hz: 2,
)
```

This acts as a rate limiter for Copper list execution. `rate_target_hz` must be between `1` and `1_000_000_000` when set.

On std targets, enabling the `high-precision-limiter` crate feature keeps the same
configuration field but changes the implementation to a tighter hybrid sleep/spin limiter
for higher-rate loops.

## Missions

Configurations may define multiple missions, each representing an alternative task graph:

```ron
missions: [ (id: "A"), (id: "B") ]
```

Tasks, resources, bridges and connections can specify a `missions` array to belong only to selected missions. See `examples/cu_missions/copperconfig.ron` for a complete example.

## Modular Configuration

Copper supports composition of configurations using the `includes` section. Each include specifies a path and optional parameter substitutions:

```ron
includes: [
    (
        path: "base.ron",
        params: { "id": "left", "pin": 4 },
    ),
]
```

Included files are processed recursively. Parameters are substituted in the included text using the `{{param}}` syntax. Merging behaviour and further details are described in the [Modular Configuration guide](Modular-Configuration).

**Since v1.1:** An include can be selected by Cargo features at compile time:

```ron
includes: [
    (
        path: "graph/jetson_mipi.ron",
        when: Feature("jetson-mipi"),
    ),
]
```

Supported predicates are `Feature("name")`, `Not(predicate)`, `All([predicates])`, and
`Any([predicates])`. A rejected include is skipped before its file is read, parsed, or
validated, so its tasks, connections, and message types do not enter generated Rust.

Applications using feature predicates must forward their Cargo feature set from `build.rs`:

```rust
fn main() {
    cu29_build::emit_cargo_features();
}
```

Add `cu29-build` under `[build-dependencies]`. Keep the matching component dependency
optional in `Cargo.toml`; target-specific dependencies can additionally be placed under a
`[target.'cfg(...)'.dependencies]` table. See `examples/cu_feature_gated_camera`.

## Multi-Copper Instance Overrides

Multi-Copper umbrella configs may declare an optional `instance_overrides_root`:

```ron
(
    subsystems: [
        (id: "robot", config: "robot_base.ron"),
    ],
    interconnects: [],
    instance_overrides_root: "instances",
)
```

This enables a convention-based lookup for per-instance config overlays:

```text
instances/<instance_id>/<subsystem_id>.ron
```

For `instance_id = 17` and subsystem `robot`, Copper probes:

```text
instances/17/robot.ron
```

If the file exists, it is applied as an overlay on top of the subsystem's base config.

Overlay files currently support a `set` array. Each entry targets one existing task, resource,
or bridge config map:

```ron
(
    set: [
        (
            path: "tasks/imu/config",
            value: {
                "gyro_bias": [0.012, -0.004, 0.008],
            },
        ),
        (
            path: "resources/board/config",
            value: {
                "imu_bus": "/dev/robot17-imu",
            },
        ),
        (
            path: "bridges/radio/config",
            value: {
                "mtu": 64,
            },
        ),
    ],
)
```

Rules:

- Supported paths are only `tasks/<id>/config`, `resources/<id>/config`, and `bridges/<id>/config`.
- `value` must be a config map.
- The overlay merges keys into the existing config map for that object.
- Unknown ids are rejected as configuration errors.
- Instance overlays do not change topology. They only mutate existing config blocks.

See `examples/cu_instance_overrides_demo/` for a minimal working example.

## Example

A more complete configuration demonstrating several features is provided in `examples/modular_config_example/copperconfig.ron`:

```ron
(
    tasks: [],
    cnx: [],
    monitor: ( type: "cu_consolemon::CuConsoleMon" ),
    logging: ( enable_task_logging: true, keyframe_interval: 100 ),
    includes: [
        ( path: "base.ron", params: {} ),
        ( path: "motors.ron", params: { "id": "left",  "pin": 4, "direction": "forward" } ),
        ( path: "motors.ron", params: { "id": "right", "pin": 5, "direction": "reverse" } ),
    ],
)
```

## Further Reading

For advanced composition rules and best practices see the [Modular Configuration guide](Modular-Configuration).
