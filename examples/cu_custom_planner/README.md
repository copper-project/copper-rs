# Offline planner example

This example runs a deterministic `TaskOrder` generated outside Copper's proc
macro. Regenerate the checked-in `task_order.ron` from the application graph:

```bash
cargo run -p custom-planner -- examples/cu_custom_planner/copperconfig.ron examples/cu_custom_planner/task_order.ron
```

The generated fragment is included by `copperconfig.ron`, so planning remains a
compile-time input and the runtime performs no dynamic planner lookup.
