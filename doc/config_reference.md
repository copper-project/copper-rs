# Copper RON Configuration Reference

This guide explains how to configure a Copper application using RON files. It summarises the available sections and options based on the source code and example configurations.

## Basic Structure

A Copper configuration is a RON file containing tasks, connections and optional sections such as monitoring, logging or runtime settings. The minimal form looks like:

```ron
(
    tasks: [],
    cnx: [],
)
```

Most examples name the file `copperconfig.ron` and reference it with the `#[copper_runtime(config = "...")]` attribute.

## Tasks

Tasks are declared under the `tasks` array. Each task entry is a tuple with the following fields:

- `id` – unique identifier of the task.
- `type` – fully qualified Rust type of the task implementation.
- `config` *(optional)* – map of parameters passed to the task.
- `missions` *(optional)* – list of mission ids in which this task is active.
- `background` *(optional)* – when set to `true`, the task runs on a background thread.
- `logging` *(optional)* – `{ enabled: bool }` to enable or disable message logging for this task.

Example:

```ron
(
    id: "task1",
    type: "tasks::ExampleTask",
    background: true,
    logging: (enabled: false),
)
```

## Connections

Connections are declared under `cnx` as tuples describing edges between tasks:

- `src` – id of the source task.
- `dst` – id of the destination task.
- `msg` – Rust type of the message carried.
- `missions` *(optional)* – restrict the connection to specific missions.
- `store` *(optional)* – when true, messages on this edge are stored in the log.

Example:

```ron
(src: "task0", dst: "task1", msg: "i32", store: true)
```

## Monitoring

The optional `monitor` section selects a monitoring component and its parameters:

```ron
monitor: (
    type: "cu_consolemon::CuConsoleMon",
    config: { "verbosity": 2 },
)
```

## Logging

The `logging` section tunes Copper’s structured log. Options mirror the fields of `LoggingConfig` defined in `core/cu29_runtime/src/config.rs`:

- `enable_task_logging` – controls per-task logging (defaults to `true`).
- `slab_size_mib` – size of each memory mapped slab in MiB.
- `section_size_mib` – pre‑allocated size per log section in MiB.
- `keyframe_interval` – number of copperlists between two keyframes (default `100`).

Example from `examples/cu_logging_size`:

```ron
logging: (
    slab_size_mib: 1024,
    section_size_mib: 100,
)
```

The library validates that `section_size_mib` does not exceed `slab_size_mib`.

## Runtime Settings

Runtime behaviour can be adjusted with the `runtime` section. Currently only `rate_target_hz` is available:

```ron
runtime: (
    rate_target_hz: 2,
)
```

This acts as a rate limiter for Copper list execution.

## Missions

Configurations may define multiple missions, each representing an alternative task graph:

```ron
missions: [ (id: "A"), (id: "B") ]
```

Tasks and connections can specify a `missions` array to belong only to selected missions. See `examples/cu_missions/copperconfig.ron` for a complete example.

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

Included files are processed recursively. Parameters are substituted in the included text using the `{{param}}` syntax. Merging behaviour and further details are described in `doc/modular_config.md`.

## Example

A more complete configuration demonstrating several features is provided in `examples/modular_config_example/main_config.ron`:

```ron
(
    tasks: [],
    cnx: [],
    monitor: ( type: "cu_consolemon::CuConsoleMon" ),
    logging: ( file: "robot.copper", level: "debug" ),
    includes: [
        ( path: "base.ron", params: {} ),
        ( path: "motors.ron", params: { "id": "left",  "pin": 4, "direction": "forward" } ),
        ( path: "motors.ron", params: { "id": "right", "pin": 5, "direction": "reverse" } ),
    ],
)
```

## Further Reading

For advanced composition rules and best practices see the [Modular Configuration guide](doc/modular_config.md).

