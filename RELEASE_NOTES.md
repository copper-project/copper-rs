# Release notes

## 1.2.2 and 1.1.4

Fix the background-task behavior to match the task specification: regular tasks configured with `background` now skip dispatch when the input payload is empty. Each completed background result is emitted once, including when collected on a tick with an empty input. Previously, empty inputs could launch background jobs and the last completed result could be emitted again on subsequent dispatches.

**Compatibility warning:** applications that rely on processing empty inputs or repeating the last completed result must opt into the old behavior. Add `background_process_empty: true` to each affected task in `copperconfig.ron`:

```ron
(
    id: "worker",
    type: "my_crate::Worker",
    background: true,
    background_process_empty: true,
)
```

Keep any existing background pool configuration; add the policy field alongside it. This option applies to regular background tasks. If constructing the wrapper directly, use `CuAsyncTask::<Task, Output, true>` to restore the old behavior. Omitting the option uses the corrected behavior.
