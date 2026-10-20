# Release notes

## 1.2.5

This patch backports fixes to Copper 1.2 while preserving its public APIs, configuration, and v1 log encapsulation.

### Fixes

- The flight controller simulator keeps `cu-zed` and `cu-vitfly` on their compatible 1.1 release line, preventing their registry dependencies from selecting Copper 1.3 alongside the local 1.2 runtime.
- Failed or panicking iterations release their unfinished CopperList before shutdown. Completed output still drains, partial output stays unpublished, and the application can restart. ([#1531](https://github.com/copper-project/copper-rs/pull/1531), [#1533](https://github.com/copper-project/copper-rs/pull/1533))
- Shared-memory images implement `Clone` and `Default` without requiring the underlying storage to implement those traits; cloning retains the buffer handle. ([#1525](https://github.com/copper-project/copper-rs/pull/1525))
- Bevy's demo keeps the runtime on the main thread, including when asynchronous CopperList I/O is enabled. Bevy's derive dependency and Kornia's 0.1 image/tensor dependencies are pinned to compatible releases. ([#1528](https://github.com/copper-project/copper-rs/pull/1528), [#1411](https://github.com/copper-project/copper-rs/pull/1411), [#1427](https://github.com/copper-project/copper-rs/pull/1427))
- ROS 2 publishers advertise their actual best-effort sensor-data QoS. Reliable subscribers can now detect incompatible publishers during discovery. ([#1399](https://github.com/copper-project/copper-rs/pull/1399))
- `fsck` computes recording timing from all message slots and reports unavailable rates when no positive duration was recorded. ([#1467](https://github.com/copper-project/copper-rs/pull/1467))
- Live diagnostics stay in the monitor's LOG pane; formatting `CuError` avoids an intermediate allocation; live twin archives accept explicit archive and receiver sizing. These previously merged backports now receive published patch versions. ([#1494](https://github.com/copper-project/copper-rs/pull/1494), [#1480](https://github.com/copper-project/copper-rs/pull/1480), [#1479](https://github.com/copper-project/copper-rs/pull/1479))
- Transform, monitor, and RRT* loops satisfy newer Clippy checks. ([#1499](https://github.com/copper-project/copper-rs/pull/1499), [#1506](https://github.com/copper-project/copper-rs/pull/1506))

### Packages

The following packages advance to **1.2.5**: `cu29`, `cu29-runtime`, `cu29-derive`, `cu29-traits`, `cu29-log-runtime`, `cu29-logstream`, `cu29-export`, `cu-tuimon`, `cu-transform`, `cu-sensor-payloads`, `cu-bevymon`, `cu-ros2-bridge`, and `cu-rrt-star`. Other package versions remain unchanged, including `cargo-cunew` 1.2.4. `cu29` signals this maintenance release without republishing every crate.

### Upgrading

Use `cu29 = "~1.2.5"`, then run `cargo update`. Update directly selected affected components to `~1.2.5` as needed. The generated runtime requires `cu29-runtime` and `cu29-derive` 1.2.5 or newer within the 1.2 line. Existing log encodings and the Kornia 0.1 conversion API are preserved; use the producing application's matching decoder for replay.

## 1.2.4

This patch releases `cargo-cunew` 1.2.4 for Copper 1.2.

### Fixes

- Project generation selects each Copper dependency's latest stable, non-yanked version within the 1.2 release line. All generated Copper requirements use `~`. This fixes the unavailable `cu29-build = "~1.2.3"` requirement after selective runtime patch releases.
- Installing the scaffold tool builds 78 packages on Linux, down from 269 in the previous published tool's current dependency graph. Bundled templates use Liquid directly, and registry queries use synchronous HTTPS with Rustls. Git repository initialization uses the installed `git` command or can be skipped with `--no-vcs`.
- CI builds and runs both generated templates against crates.io on release-branch changes, after GitHub releases, and daily. A separate check enforces a 100-package installer budget and rejects the removed heavy dependencies.

### Upgrading

Run `cargo install cargo-cunew --version '~1.2.4' --force`. Generate your project with `cargo cunew hello_copper`, enter the generated `hello_copper` directory, and run `cargo run`.

For an existing generated project that fails to resolve `cu29-build`, change its build dependency to `cu29-build = "~1.2.1"`. Keep `cu29 = "~1.2.3"`; the runtime and other Copper crates retain their existing compatible versions.

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
