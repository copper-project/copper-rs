# run_in_sim Demo

This demo shows how to control task behavior in simulation using `run_in_sim` and a simulation callback.

What the demo does:

- A Source task produces a counter value (simulated via the callback).
- A Regular processing task doubles that value.
- A Sink task prints the doubled value.

In this implementation:

- The Source is simulated (a placeholder is used), and the simulation callback sets the output payload each iteration.
- The Sink runs its real implementation in simulation (so its real `process` runs and prints).
- The Regular task always runs its real code in simulation; `run_in_sim` is ignored for regular tasks.

## Why this matters

When running in simulation, sources and sinks can be either:

- Replaced by a placeholder to avoid using real hardware, or
- Executed with their real implementation.

The `run_in_sim` flag determines which behavior is used for sources and sinks during simulation. This demo shows a mixed
setup where:

- The source is simulated (the simulator must provide data).
- The sink runs for real (you see the real code’s output).

This lets you inject synthetic inputs at the beginning of the pipeline while keeping the real behavior at the output
end.

## How the simulation callback is used

The runtime generates a `SimStep` enum that indicates which task and lifecycle stage is happening. The demo’s callback
does this:

- For the Source `Process` step: it sets the output payload (e.g., a monotonically increasing counter) and returns
  `ExecutedBySim`, meaning “the simulator handled it; skip the real implementation.”
- For the Regular task `Process` step: it returns `ExecuteByRuntime` so the real code runs and doubles the value.
- For the Sink `Process` step: it returns `ExecuteByRuntime` so the real sink runs and prints the result.
- For all other lifecycle steps (New, Start, Preprocess, Postprocess, Stop): it returns `ExecuteByRuntime`.

## Running the demo

From your workspace, run the example using your preferred method, for example:

- `cargo run -p cu_run_in_sim`
- or change into the example’s directory and `cargo run`

You should see output similar to:

```
17:32:05 [DEBUG] (1) cu29_log: 0 ns [Debug]: Logger created at logs/run_in_sim.copper. This is a simulation.
17:32:05 [DEBUG] (1) cu29_log: CuConfig: Using the original configuration the project was compiled with:  ...
[MySink] got value = 2
[MySink] got value = 4
[MySink] got value = 6
[MySink] got value = 8
[MySink] got value = 10
Flushing the unified Logger ... 
Unified Logger flushed.
```

