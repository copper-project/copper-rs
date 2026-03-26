# `cu_python_task_demo`

This example shows how Copper can run one task from Python while the rest of the
application stays in Rust.

It demonstrates the execution modes provided by
[`cu-python-task`](/home/gbin/projects/copper/copper-rs.python/components/tasks/cu_python_task/README.md):

- `process`: Python runs in a separate interpreter and exchanges CBOR frames with Rust
- `embedded`: Python runs in-process through PyO3 under the GIL

## What The Demo Does

The graph is intentionally small:

- `LeftSource` publishes an incrementing integer
- `RightSource` publishes an incrementing tag
- `ExamplePythonTask` combines both inputs in `python/task.py`
- `SummarySink` and `StateSink` capture the outputs for tests

The Python task keeps mutable state across calls and writes two outputs:

- a per-cycle summary with the doubled left input
- an accumulated state snapshot

## Files To Read

- [`src/lib.rs`](/home/gbin/projects/copper/copper-rs.python/examples/cu_python_task_demo/src/lib.rs): Rust messages, task aliases, runtime wiring, and end-to-end tests
- [`python/task.py`](/home/gbin/projects/copper/copper-rs.python/examples/cu_python_task_demo/python/task.py): the Python algorithm body
- [`copperconfig.ron`](/home/gbin/projects/copper/copper-rs.python/examples/cu_python_task_demo/copperconfig.ron): graph definition

## Running It

Process mode:

```bash
cargo run -p cu-python-task-demo -- process
```

Embedded mode:

```bash
cargo run -p cu-python-task-demo -- embedded
```

Run the end-to-end tests:

```bash
cargo test -p cu-python-task-demo
```

Notes:

- process mode needs the Python package `cbor2`
- in process mode, any payload field typed as `CuHandle<CuSharedMemoryBuffer<T>>`
  is exported to Python by shared-memory descriptor automatically; this demo's
  default payloads are ordinary values, so it exercises the normal path
- embedded mode is not supported on macOS in this workspace
- the demo rewrites the Python task `script` config to an absolute path in Rust so
  `cargo run -p cu-python-task-demo` works from the workspace root; relative
  script paths are otherwise resolved from the process current working directory

## The Python Contract

The user script must export:

```python
def process(ctx, inp, state, output):
    ...
```

It may also export optional lifecycle hooks:

```python
def start(ctx, state):
    ...

def stop(ctx, state):
    ...
```

If those hooks are omitted, Copper treats them as no-ops.

In this demo:

- `ctx` exposes Copper callback metadata and clock access
- `inp` is a 2-tuple containing the left and right input messages
- `state` is a mutable object with `calls`, `total`, and `last_tag`
- `output` is a 2-slot container for the output messages
- `start` and `stop` are implemented only to demonstrate the optional hook shape

Message payloads are exposed as attribute-style objects, so the example code can
write:

```python
output[0].payload.doubled = left_value * 2
```

If an output payload has not been materialized yet, touching `output[i].payload`
creates a default payload object automatically.

## Why This Exists

This feature is for rapid experimentation only.

If you put Python on Copper's execution path, you give up the main reasons to use
Copper there in the first place: low latency, low jitter, tight allocation
control, and predictable realtime behavior. The middleware overhead is bad, the
allocation behavior is bad, performance is abysmal relative to a native Rust task,
and the resulting stack is not something you should ship as a serious realtime robot.

The practical use case is:

1. iterate on one algorithm in Python until the behavior is right
2. rewrite it in Rust immediately afterward
3. if helpful, use an LLM to draft the Rust translation, then review it properly

That is the lane this example is meant to support.
