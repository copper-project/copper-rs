# `cu-python-task`

Python-backed Copper tasks for algorithm prototyping.

## Read This First

This crate is intentionally hostile to Copper's normal performance model.

Copper is built around static Rust types, preallocation, deterministic scheduling, and
very low jitter. A Python task throws that away on the hot path:

- every call crosses a Rust/Python boundary
- every call allocates
- inputs, state, and outputs are cloned into owned values
- the runtime loses the latency and jitter characteristics Copper is designed for

In practice, performance is abysmal compared to a native Rust Copper task.

Use this only to experiment on one task that you want to iterate on quickly in Python.
Once the behavior is right, rewrite it in Rust. If you want, use an LLM-assisted
translation as a starting point, then review the generated Rust carefully.

This is strongly not recommended for production or realtime control loops.

## What The Crate Does

`PyTask<I, S, O>` is a regular `CuTask` implementation whose logic lives in a Python
script exposing:

```python
def process(input, state, output):
    ...
```

Copper still owns scheduling, logging, and task lifecycle. The Python function only
implements the algorithm body.

The task config supports two parameters:

- `script`: path to the Python file. Defaults to `python/task.py`. Relative
  paths are resolved against the process current working directory.
- `mode`: `"process"` or `"embedded"`. Defaults to `"process"`.

Example RON:

```ron
(
    id: "py",
    type: "tasks::ExamplePythonTask",
    config: {
        "script": "python/task.py",
        "mode": "process",
    },
)
```

If you launch the app from a workspace root or another directory, use an
absolute `script` path or rewrite the config programmatically before startup.

## Execution Modes

### `process`

The Python task runs in a separate interpreter process.

- Copper spawns `python3` or `python`
- requests and responses are sent as length-prefixed CBOR frames over stdin/stdout
- the Python side uses `cbor2` to decode and encode those frames

Tradeoffs:

- Pro: the GIL stays out of the Copper process
- Pro: a Python crash or import failure is isolated to the child process
- Con: every cycle pays extra serialization, copying, allocations, IPC overhead, and
  process scheduling jitter
- Con: you now depend on `cbor2`, and the pure-Python backend is slower still

### `embedded`

The Python task runs inside the Copper process through PyO3.

- requests are converted into `cu29_value::Value`
- those values are converted again into Python objects
- the Python `process(...)` function runs under the GIL
- the returned Python objects are converted back into Rust values

Tradeoffs:

- Pro: no child process and no CBOR IPC layer
- Pro: usually less overhead than `process`
- Con: the GIL is now in the Copper process
- Con: Python exceptions and runtime behavior happen inside the same process as the runtime
- Con: this still allocates constantly and still destroys realtime behavior
- Con: this workspace does not support embedded mode on macOS

If you are choosing between the two, the practical answer is usually:

- `process` if you want better isolation while you experiment
- `embedded` if you want slightly less overhead and are willing to accept the GIL in-process

Neither is a good production answer.

## Data Contract

Inputs and outputs are exposed to Python as mutable attribute-style objects:

- `msg.payload` gives access to the message payload when present
- `state` is a mutable object that is preserved between calls
- `output` is a mutable tuple/list-like container of output messages

When an output message has no payload yet, the bootstrap layer lazily materializes a
default payload object the first time Python accesses `output[i].payload`.

The whole contract is shown in the demo at
[`examples/cu_python_task_demo`](/home/gbin/projects/copper/copper-rs.python/examples/cu_python_task_demo/README.md).

## Recommendation

Treat this crate as a disposable prototyping bridge:

1. try an idea in Python
2. validate the behavior on one task
3. move the final implementation back to Rust as soon as possible

Do not build your robot architecture around Python tasks.
