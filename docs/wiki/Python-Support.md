# Python Support

Copper has two Python stories and they should not be confused.

## 1. Offline Python Log Analysis

Copper can expose recorded logs to Python after the run:

- structured text logs through `cu29-export`
- runtime lifecycle records through `cu29-export`
- app-specific typed CopperLists through a small PyO3 module in the application

This works well for:

- notebooks
- data mining
- offline inspection
- ad hoc visualization
- exporting recorded robot data into a Python-heavy analysis workflow

Because Python only touches already-recorded data, this does **not** affect the
robot's runtime latency, jitter, or determinism.

Reference example:

- `examples/cu_flight_controller/src/python_module.rs`
- `examples/cu_flight_controller/python/print_gnss_from_log.py`

## 2. Runtime Python Task Prototyping

Copper also has first-class support for delegating one task body to Python through
`cu-python-task`, but this is **strongly not recommended** for production.

Why:

- every call crosses a Rust/Python boundary
- every call allocates
- task inputs/state/outputs are copied into owned values
- latency and jitter get much worse
- the middleware overhead is bad
- Python's allocation behavior is fundamentally hostile to realtime execution

Compared to a native Rust Copper task, the performance is abysmal.

If you put Python on the live execution path, you are giving up the core things
Copper is trying to guarantee.

That said, if you are in a bind with a large python codebase,
it is the way to go so you can migrate progressively.

### Process Mode

In `process` mode, Copper spawns a separate Python interpreter process and talks to
it through length-prefixed CBOR frames over stdin/stdout.

Pros:

- the GIL is not inside the Copper process
- failures are more isolated

Cons:

- extra serialization and copying
- extra allocations
- IPC overhead
- scheduler/process jitter
- depends on `cbor2`

### Embedded Mode

In `embedded` mode, Copper calls Python directly in-process through PyO3.

Pros:

- no extra process
- no CBOR IPC layer
- typically less overhead than `process`

Cons:

- the GIL is now inside the Copper process
- Python exceptions happen inside the runtime process
- still allocates constantly
- still destroys realtime characteristics
- not supported on macOS in this workspace

## Recommendation

Use runtime Python only as a temporary prototyping bridge for one task:

1. get the algorithm right in Python
2. validate the behavior
3. rewrite it in Rust immediately afterward

If helpful, use an LLM to draft the Rust translation, then review and tighten the
result like normal Rust code.

That is the intended lane for Copper's Python task support.
