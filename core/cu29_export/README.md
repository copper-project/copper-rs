# `cu29-export`

Copper log export helpers and Python-facing log readers.

This crate is for offline analysis of `.copper` logs. That distinction matters:
using Python here is fine because it does not put Python on the runtime hot path.

If you want to run task logic in Python, that is a different feature entirely:
see
[`cu-python-task`](/home/gbin/projects/copper/copper-rs.python/components/tasks/cu_python_task/README.md).

## What This Crate Provides

- the `run_cli::<P>()` logreader entrypoint used by Copper examples and templates
- structured log export helpers
- CopperList export helpers
- optional Python bindings for iterating logs without going through JSON first

## Selecting a Recorded Run

A unified log can contain multiple recorded runs after a mission change or an
appended restart. Each `Instantiated` lifecycle record begins a recorded run.
Its CopperList IDs and clock can start over independently of the previous run.

Use the application's logreader binary to list the runs and select one:

```sh
logreader logs/robot.copper list-runs
logreader logs/robot.copper --run 1 fsck --dump-runtime-lifecycle
logreader logs/robot.copper --run 1 extract-copperlists
logreader logs/robot.copper --run 1 extract-text-log target/debug/cu29_log_index
logreader logs/robot.copper --run 1 log-stats --output run-1.json
logreader logs/robot.copper --run 1 export-mcap --output run-1.mcap
```

Indices are zero-based and follow `Instantiated` record order. They are distinct
from the recorded `instance_id`, which can repeat across process restarts.
`list-runs` reads lifecycle metadata without decoding application payloads.
It shows the mission, application, runtime instance ID, start time, and whether
`ShutdownCompleted` was recorded.

Single-run logs select their run automatically. Logs from standalone
writers with no `Instantiated` record are treated as one implicit run.
Multi-run logs require `--run` for extraction, fsck, statistics, and MCAP
export. The selection includes the runtime's initial stream reservations and all
of its CL, keyframe, lifecycle, and structured-log sections. A multi-run log
whose startup section ordering cannot be recognized is listed but rejected for
selection.

The selected run supplies the recorded configuration used by logging codecs.
Statistics also default to that configuration and its recorded mission; `--config`
and `--mission` provide explicit overrides. Without a recorded configuration,
statistics use `copperconfig.ron`. Use a logreader and string index built for the
application version that produced the selected run.

`fsck` checks CopperList IDs within the selected run, reports decoding errors,
and returns an error for repeated or decreasing IDs and an unclean final log close.
An ID reset at the next `Instantiated` belongs to that next run.

## Python Support

Python support lives behind the `python` feature and is not supported on macOS in
this workspace.

There are two Python-facing patterns:

### 1. Generic structured log reading

`libcu29_export` can expose:

- `struct_log_iterator_bare(...)`
- `struct_log_iterator_unified(...)`
- `runtime_lifecycle_iterator_unified(...)`

This is useful when you want Python to inspect Copper's structured text logs or
runtime lifecycle records.

The example script at
[`examples/cu_standalone_structlog/readlog.py`](/home/gbin/projects/copper/copper-rs.python/examples/cu_standalone_structlog/readlog.py)
shows the basic import pattern.

### 2. App-specific typed CopperList reading

CopperLists are application-specific, so a Python module that reads them must know
the generated tuple type for that application.

The intended pattern is:

1. call `gen_cumsgs!("copperconfig.ron")` in the application
2. expose a small `#[pymodule]` wrapper in that app
3. call `copperlist_iterator_unified_typed_py::<YourGeneratedType>(...)`

See
[`examples/cu_flight_controller/src/python_module.rs`](https://github.com/copper-project/extra-examples/blob/master/examples/cu_flight_controller/src/python_module.rs)
and
[`examples/cu_flight_controller/python/print_gnss_from_log.py`](https://github.com/copper-project/extra-examples/blob/master/examples/cu_flight_controller/python/print_gnss_from_log.py)
for the reference implementation.

## Feature Flags

- `python`: Rust-side helpers for embedding/exposing Python log readers
- `python-extension-module`: only for building the Python extension itself
- `mcap`: MCAP export support

## Recommendation

Use Python here for post-processing, data mining, notebooks, and analysis scripts.
That is a reasonable workflow.

Do not confuse that with putting Python inside a Copper control loop. Offline export
and runtime Python tasks have very different tradeoffs.
