# Release notes

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
