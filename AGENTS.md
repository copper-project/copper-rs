# AGENTS.md

## Project Identity

`copper-rs` is a Rust robotics runtime + SDK built around:

- Declarative task graphs in `copperconfig.ron`
- Compile-time runtime generation via `#[copper_runtime(...)]`
- Deterministic execution and replay
- Unified binary logging (`.copper`) plus separate string indexes/logreaders
- A mixed `std` + `no_std` workspace that spans desktop, SBC, and bare-metal targets

The shortest accurate mental model is: "game engine for robots, but the scene graph is a task graph and the engine is generated from RON config."

## Canonical Sources

Read these first when catching up:

1. Root overview: `README.md`
2. CI-aligned commands and local workflows: `justfile`, `CONTRIBUTING.md`
3. Narrative design docs in the sibling wiki checkout: `../copper-rs.wiki/`
4. Tutorial/book material in the sibling book checkout: `../copper-rs-book/book/src/`

Authority order when sources disagree:

1. Code
2. API docs / rustdoc surface
3. Wiki / book narrative docs

Most useful wiki pages:

- `../copper-rs.wiki/Copper-Runtime-Overview.md`
- `../copper-rs.wiki/Copper-Application-Overview.md`
- `../copper-rs.wiki/Copper-RON-Configuration-Reference.md`
- `../copper-rs.wiki/Task-Lifecycle.md`
- `../copper-rs.wiki/Resources.md`
- `../copper-rs.wiki/Project-Templates.md`
- `../copper-rs.wiki/Supported-Platforms.md`
- `../copper-rs.wiki/Available-Components.md`

Most useful book chapters:

- `../copper-rs-book/book/src/ch04-project-structure.md`
- `../copper-rs-book/book/src/ch13-logging-replay.md`
- `../copper-rs-book/book/src/ch18-justfile.md`
- `../copper-rs-book/book/src/ch21-resources.md`

## Workspace Map

- `core/`
  - Main runtime and proc-macro crates.
  - Start with `core/cu29`, `core/cu29_runtime`, `core/cu29_derive`, `core/cu29_traits`.
- `components/`
  - Reusable building blocks grouped as `sources`, `sinks`, `tasks`, `bridges`, `payloads`, `monitors`, `res`, `libs`.
- `examples/`
  - This repo’s best executable documentation. Many features are easier to learn from examples than from API docs.
- `templates/`
  - `cu_project` for single-crate apps, `cu_full` for multi-crate workspaces.
- `support/`
  - CI helpers, cross/deploy support, Docker, `cargo_cubuild`, docs generation helpers.

## Architectural Facts That Matter

- The primary user-facing crate is `core/cu29/src/lib.rs`.
- The `cu29` prelude re-exports nearly everything application authors touch.
- Copper’s design center is static structure, not dynamic runtime discovery.
  - A robot is a static thing.
  - Prefer types, compile-time wiring, and proc-macro/codegen enforcement.
  - Avoid magical runtime string matching, dynamic service discovery, or "things appear later" patterns.
- Runtime generation is proc-macro-driven:
  - `#[copper_runtime(config = "...")]` reads a RON config at compile time.
  - It generates application/builder types and computes the execution plan.
  - Missions in config generate mission-specific builders/modules.
- Config parsing and graph representation live in `core/cu29_runtime/src/config.rs`.
- The proc macro implementation is in `core/cu29_derive/src/lib.rs`.
- `gen_cumsgs!("...")` generates the logreader decode type for a config.
- Logging is a first-class runtime concern, not an optional afterthought:
  - Runtime logs `.copper` binary data
  - Text log strings are interned and reconstructed with `target/debug/cu29_log_index`
  - Replay/resim flows are part of the design, not bolt-ons
- `std` vs `no_std` is a real design axis across the workspace. Do not assume host-only APIs are available everywhere.

## Files To Inspect First For Common Tasks

If the task is about runtime generation:

- `core/cu29_derive/src/lib.rs`
- `core/cu29_runtime/src/config.rs`
- `core/cu29_runtime/src/curuntime.rs`

If the task is about task APIs, message flow, or lifecycle:

- `core/cu29_runtime/src/cutask.rs`
- `core/cu29_runtime/src/cubridge.rs`
- `core/cu29_runtime/src/app.rs`
- `../copper-rs.wiki/Task-Lifecycle.md`

If the task is about resources / HAL wiring:

- `core/cu29_runtime/src/resource.rs`
- `examples/cu_resources_test/`
- `../copper-rs.wiki/Resources.md`

If the task is about logging / export / replay:

- `core/cu29_helpers/src/lib.rs`
- `core/cu29_export/`
- `core/cu29_unifiedlog/`
- `examples/cu_caterpillar/src/logreader.rs`
- `examples/cu_caterpillar/src/resim.rs`

If the task is about templates / DX:

- `templates/cu_project/`
- `templates/cu_full/`
- `templates/README.md`
- Template checks in root `justfile`

If the task is about embedded / no_std:

- `.github/workflows/reusable-embedded.yml`
- `support/ci/embedded_crates.py`
- `examples/cu_rp2350_skeleton/`
- `components/res/cu_micoairh743/`

## Reference Examples

Use these as the quickest way to regain context:

- `examples/cu_caterpillar/`
  - Minimal but realistic Copper app.
  - Good reference for `#[copper_runtime]`, logger setup, determinism, resim, and justfile helpers.
- `examples/cu_flight_controller/`
  - Canonical higher-complexity integration example.
  - Good reference for resources, simulation mode, and real application structure.
- `examples/cu_rp_balancebot/`
  - Canonical simulation/demo app.
  - Good reference when validating the "try Copper without hardware" workflow.
- `examples/cu_resources_test/`
  - Best resource-binding + missions example.
  - Shows mission-specific builders, resource bundles, owned vs shared resources, bridge resources.
- `examples/cu_rp2350_skeleton/`
  - Best embedded skeleton.

Default canonical example set for orientation and regression thinking:

- `examples/cu_caterpillar/`
- `examples/cu_flight_controller/`
- `examples/cu_rp_balancebot/`

## Commands That Match CI

Preferred local checks from repo root:

- `just`
- `just pr-check`
- `just lint`
- `just test`
- `just std-ci`
- `just nostd-ci`

Useful focused tools:

- `just expand-runtime pkg=<crate> bin=<bin> [features=...]`
- `just expand-soa`
- `just rtsan-smoke pkg=<pkg> bin=<bin> [args=...]`

Important CI facts:

- Root CI entrypoint: `.github/workflows/general.yml`
- Main branch in CI config is `master`
- `just std-ci` mirrors the std path
- `just nostd-ci` mirrors the embedded/no_std path
- `examples/cu_caterpillar` has a determinism regression test used in CI
- Windows CI uses a reduced workspace scope (`core`) compared to Linux/macOS

## Platform Priorities

Treat platform support priorities as:

1. Linux host first
2. macOS second
3. no_std must not be casually broken
4. Windows has minimal support expectations

Implications:

- Prefer validating host workflows on Linux first.
- If changing shared crates, macros, or traits, consider `no_std` impact immediately.
- macOS regressions still matter because they are exercised in CI.
- Do not assume feature parity or deep support on Windows.

## Practical Development Notes

- Do not paper over issues just to make something work.
  - No hacks to hide a deeper design/runtime problem.
  - No shortcut that creates spaghetti code or weakens the architecture.
- Prefer `just` targets over inventing ad hoc command sequences.
- After each edit pass, run `just fmt` from the repo root before moving on.
- When touching Rust code, avoid clippy-denied cleanup mistakes that keep recurring here:
  - do not keep redundant same-type casts such as `u64` to `u64`
  - prefer `.is_multiple_of(...)` over `% ... == 0` when checking divisibility
- Use `cargo expand` when proc-macro behavior is unclear.
- Many examples/apps create logs under their own `logs/` directories.
- `basic_copper_setup(...)` is the common logger/runtime bootstrap helper, but examples may customize around it.
- Some examples intentionally disable task logging in config even though a unified log slab is still allocated.
- Do not introduce environment variables as invisible API unless explicitly requested.
  - Prefer constants.
  - If something may need to be user-configurable, ask whether it belongs in RON config.
  - Do not assume every tuning knob belongs in config; over-configuration is a design smell here.
- When a feature touches both docs and behavior, update the wiki/book sources if this checkout expects them to stay in sync.
- Default workspace members are only the core crates. Full workspace changes often need explicit workspace checks.
- The workspace is large and hardware-heavy. For many tasks, package-focused verification is more pragmatic than a full workspace run.

## Runtime And Memory Constraints

- Copper should never allocate on the real-time stack/path.
- Copper should be very cautious about startup allocation volume.
  - `no_std` targets may have extremely small memory budgets.
- Prefer preallocation, fixed capacity, and compile-time sizing where practical.
- Avoid memory copies when a zero-copy or borrow-based design is available.
- Changes that increase allocation count, heap pressure, or copy count need explicit justification.

## Logging And Debugging Workflow

- When Copper runs, the unified log is the primary debugging artifact.
- Use the main log to debug robots, algorithms, timing, and message flow before adding new instrumentation.
- Do not add extra ad hoc text logs just to inspect message values.
  - Copper already logs task messages as CopperLists when task logging is enabled.
  - Extract the recorded messages instead of printing them manually.
- If temporary instrumentation is still needed, use Copper structured logging:
  - `debug!`
  - `info!`
  - `warn!`
  - `error!`
- Do not invent custom text dump formats and do not parse hand-written text output when structured logs already exist.

### What To Enable In Config

There are two relevant logging controls in RON:

1. Global/task-log capture:

```ron
logging: (
    enable_task_logging: true,
    slab_size_mib: 16,
    keyframe_interval: 3,
)
```

Useful fields from `core/cu29_runtime/src/config.rs`:

- `enable_task_logging`
- `slab_size_mib`
- `section_size_mib`
- `keyframe_interval`

Notes:

- `enable_task_logging` defaults to `true`.
- `section_size_mib` must not exceed `slab_size_mib`.
- Be conservative with slab/section sizes, especially for small-memory targets.

2. Per-task / per-bridge message logging:

```ron
(
    id: "ahrs",
    type: "cu_ahrs::CuAhrs",
    logging: (enabled: true),
)
```

This is how you selectively keep or suppress message logging at node level.

Good references:

- `examples/cu_flight_controller/copperconfig.ron`
  - selective `logging: (enabled: true|false)` on tasks and bridges
  - root logging block with `slab_size_mib`, `section_size_mib`, `enable_task_logging`
- `examples/cu_caterpillar/copperconfig.ron`
  - root logging block with keyframe/slab/section sizing
- `examples/cu_resources_test/copperconfig.ron`
  - example with `logging: (enable_task_logging: false)`

### Preferred Debugging Paths

For message values and pipeline state:

- Use the app logreader built on `cu29_export::run_cli`.
- Current CLI path:
  - `extract-copperlists`
  - optionally `--export-format json|csv`
- This gives the recorded message payloads from the unified log.

For replaying failures:

- If you have a log, consider resimulation before adding instrumentation.
- Copper can replay a recorded log back through the application deterministically.
- This is a first-class debugging path for robot/algorithm failures.
- Good references:
  - `examples/cu_caterpillar/src/resim.rs`
  - `examples/cu_rp_balancebot/src/resim.rs`
  - `examples/cu_bridge_test/src/resim.rs`
- In practice, resim is done with the normal runtime macro in simulation mode:
  - `#[copper_runtime(config = "...", sim_mode = true)]`
- Keyframes matter here:
  - recorded task state can be restored
  - replay can resume from frozen task state rather than only from message streams

For deep log/session introspection:

- There is a remote debug API for querying replayed state and timeline data.
- Treat it as the higher-power path when you need more than raw message export.
- This is the path for querying:
  - timeline position / copperlists
  - task state
  - schema/type information
  - watch/search/inspect/read operations
- Good references:
  - `core/cu29_runtime/src/remote_debug.rs`
  - `examples/cu_remote_debug_session/src/main.rs`
  - `examples/cu_remote_debug_session/README.md`
- Important remote methods called out in the current API docs include:
  - `nav.seek`
  - `nav.step`
  - `nav.replay`
  - `timeline.get_cl`
  - `state.inspect`
  - `state.read`
  - `state.search`
  - `schema.get_type`
  - `schema.get_payload_map`

Practical rule:

- If something failed and you already have a log:
  - first try extracting copperlists / structured logs
  - then consider resim
  - if you need arbitrary state inspection across time, use the remote debug API

For structured runtime/task logs:

- Use `extract-text-log` with the generated string index, usually `target/debug/cu29_log_index`.
- These logs are structured and reconstructable from the same unified log.

For offline analysis:

- JSON dump is acceptable when needed.
- Prefer the Python API when querying structured logs programmatically; it is better than scraping rebuilt text.
- Python support lives in `core/cu29_export` behind the `python` feature.
- Current Python entry points in `core/cu29_export/src/lib.rs`:
  - `struct_log_iterator_unified(...)`
  - `struct_log_iterator_bare(...)`
- CLI support for this workflow is expected to improve, but the unified log + Python path already exists.

### Logging Defaults To Remember

- First ask: do I already have this in the CopperList or structured log?
- If yes, extract/query it. Do not add more logs.
- If no, and temporary observability is justified, add `debug!`/`info!` rather than custom text output.
- If message data is missing, check config first:
  - global `logging: (enable_task_logging: true, ...)`
  - node-level `logging: (enabled: true)` where needed
- If a bug is reproducible from a log, prefer replay/resim over speculative live debugging.

## Design Biases

- Static over dynamic.
- Compile-time guarantees over runtime validation where feasible.
- Macros/codegen/types over stringly-typed late binding.
- Explicit architecture over magical convenience layers.
- Small, understandable abstractions over flexible but leaky "framework" indirection.

In practice this means:

- Avoid adding runtime lookup systems keyed by strings if the set is known at compile time.
- Avoid hidden control through env vars.
- Avoid adding config fields without a real user need.
- Avoid designs that imply dynamic topology, runtime component appearance, or mutable graph shape.
- Treat proc macros and generated code as a feature, not as something to bypass with ad hoc runtime glue.

## How To Read A Copper App Quickly

1. Open `Cargo.toml` for crate features/deps.
2. Open the app’s `copperconfig.ron`.
3. Find the `#[copper_runtime(config = "...")]` entrypoint in `main.rs`.
4. Read task/bridge/resource implementations referenced by the config.
5. If logs or replay matter, inspect the app’s `logreader.rs` / `resim.rs`.
6. If missions are present, verify mission-specific bindings and generated builders.

## Current High-Signal Observations

- The repo is both framework code and productized examples/templates.
- Proc-macro expansion is central; a lot of "where is this defined?" questions are answered by generated code.
- Resources are now a major abstraction, not a side feature.
- Determinism, replay, and export are part of the intended value proposition and should be preserved when changing runtime behavior.
- This project is explicitly cross-platform and cross-target; avoid accidentally regressing `no_std` or embedded crates when changing shared traits or macros.

## Questions To Confirm Next Time

These are the highest-value maintainer questions still open after reading the code/docs:

1. Are there areas in the workspace that are effectively dormant or experimental and should not be treated as stable surfaces?
2. If an agent changes behavior in `core/`, the expected follow-up is to also update `../copper-rs.wiki` and `../copper-rs-book`, then tell you and make PRs there.
