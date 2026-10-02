## Copper: Task Lifecycle

### Here is a quick illustration of a task lifecycle

![Copper Task Lifecycle](imgs/lifecycle.svg)

The framework is designed to give as many chances as possible for the tasks to do its heavy lifting out of the critical path.

* The **time critical part** will try to be scheduled as back to back as possible between tasks on the same thread (low latency / low jitter)
* The **best effort part** will try be scheduled by packing them to minimize the interference with the critical path (high throughput / high jitter).

In the critical loop, no allocation should occur, messages are pre-allocated and reused.

## Latched state updates

For low-rate "sticky" state, do not keep re-sending the full value every cycle and do not assume Copper will implicitly remember the previous output. Instead, send a payload that explicitly describes how downstream state should evolve.

Copper provides:

* `CuLatchedStateUpdate<T>` with `NoChange`, `Set(T)`, and `Clear`
* `CuLatchedState<T>` as the consumer-side cache

Semantics:

* `NoChange` means "keep whatever value was already latched"
* `Set(T)` means "replace the latched value with this one"
* `Clear` means "the previously latched value is no longer valid"

Typical producer pattern:

* emit `Set(...)` when calibration / transforms / metadata first become available
* emit `NoChange` on later cycles while the cached value stays valid
* emit `Clear` if the cached value must be invalidated

Typical consumer pattern:

* store a `CuLatchedState<T>` inside the task struct
* when an update message arrives, call `state.update(update)`
* read the cached value with `state.get()` and skip work until it exists

```rust
pub struct DepthProjector {
    calibration: CuLatchedState<Calibration>,
}

fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>,
           output: &mut Self::Output<'_>) -> CuResult<()> {
    if let Some(update) = input.payload() {
        self.calibration.update(update);
    }

    let Some(calibration) = self.calibration.get() else {
        return Ok(());
    };

    // Use the cached calibration until a future Set(...) or Clear arrives.
    output.set_payload(project_depth_with(calibration));
    Ok(())
}
```

This is useful for calibration bundles, static transforms, map updates, and other state that is usually unchanged but must still be deterministic in logs and replay. Because the updates are part of the message stream, replay and resimulation see the same state transitions as the live run.

Note on freeze / thaw: Copper periodically serializes component state so replay can seek from a nearby keyframe instead of replaying from the beginning. Keyframe capture does not stop or drain the graph, including background and `parallel-rt` work, but remains deterministic on replay. Because `freeze()` runs at the component's normal point in the execution wave, implementations should be side-effect-free and must not allocate on the real-time path.
