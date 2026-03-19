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
[`examples/cu_flight_controller/src/python_module.rs`](/home/gbin/projects/copper/copper-rs.python/examples/cu_flight_controller/src/python_module.rs)
and
[`examples/cu_flight_controller/python/print_gnss_from_log.py`](/home/gbin/projects/copper/copper-rs.python/examples/cu_flight_controller/python/print_gnss_from_log.py)
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
