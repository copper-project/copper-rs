# V1 API Surface

This file defines the Copper V1 public contract. Anything not listed as stable is not covered by V1 semver guarantees.

## Labels

- `stable`: covered by V1 semver.
- `experimental`: usable, but may change without a major version bump.
- `internal`: public only because proc macros, generated code, tests, or rustdoc need a path.
- `deprecated`: still callable, but not part of new V1 design.

## Stable

- `cu29::prelude`: canonical import surface for application crates.
- `#[copper_runtime(...)]`: generated application runtime entrypoint.
- `gen_cumsgs!("...")`: generated logreader decode type.
- Generated application builders:
  - `App::builder()`
  - `with_clock(...)`
  - `with_log_path(...)`
  - `with_logger(...)`
  - `with_resources(...)`
  - `with_instance_id(...)`
  - `build()`
- Application traits:
  - `CuApplication`
  - `CuStdApplication`
  - `CuSimApplication`
  - `CuRecordedReplayApplication`
  - `CuDistributedReplayApplication`
  - `CuSubsystemMetadata`
- Task and bridge authoring APIs:
  - `CuSrcTask`
  - `CuTask`
  - `CuSinkTask`
  - `CuBridge`
  - `CuMsg`
  - `CuMsgPayload`
  - `CuMsgMetadata`
  - `input_msg!`
  - `output_msg!`
  - `BridgeChannel`
  - `BridgeChannelSet`
- Resource APIs:
  - `resources!`
  - `bundle_resources!`
  - `ResourceBindings`
  - `ResourceBundle`
  - `ResourceBundleDecl`
  - `ResourceManager`
  - `ResourceKey`
  - `Owned`
  - `Borrowed`
- Config model and RON schema:
  - `CuConfig`
  - `MultiCopperConfig`
  - task, bridge, resource, monitor, runtime, logging, mission, and include config structs
  - `read_configuration`
  - `read_multi_configuration`
- Logging/export/replay APIs:
  - `CuLogEntry`
  - `CuLogLevel`
  - `LoggerRuntime`
  - `UnifiedLogWrite`
  - `UnifiedLogRead`
  - `SectionStorage`
  - `stream_write`
  - `cu29_export::run_cli`
  - `cu29_export::copperlists_reader`
  - `cu29_export::runtime_lifecycle_reader`
  - `cu29_export::structlog_reader`
  - `cu29_export::textlog_dump`
  - `cu29::replay::ReplayCli`
  - `cu29::replay::ReplayArgs`
- Core utility types used directly by applications:
  - `CuResult`
  - `CuError`
  - `RobotClock`
  - `RobotClockMock`
  - `CuTime`
  - `CuDuration`
  - `Tov`
  - `CuContext`
  - `CopperList`
  - `Freezable`
  - `CuArray`
  - `CuArrayVec`
  - `CuHandle`
  - `CuHostMemoryPool`
  - `CuPool`
  - `Value`

## Experimental

- `remote-debug` feature and `cu29::remote_debug`.
- `parallel-rt` feature and parallel executor APIs.
- `async-cl-io` feature and async CopperList I/O internals.
- Runtime performance knobs:
  - `sysclock-perf`
  - `high-precision-limiter`
- Low-level logging codec registry APIs in `cu29::logcodec`.
- Low-level monitoring probes and allocation counters.
- Direct unified-log section/header structs.

## Internal

- Direct fields on `CuRuntime`.
- `CuRuntimeParts`.
- `CuRuntimeBuilder`.
- `TasksInstantiator`.
- `BridgesInstantiator`.
- `MonitorInstantiator`.
- `ProcessStepOutcome`.
- `ProcessStepResult`.
- `SyncCopperListsManager`.
- `AsyncCopperListsManager`.
- `OwnedCopperListSubmission`.
- Generated mission modules and generated helper functions.
- Direct task tuple and bridge tuple access through `copper_runtime_mut()`.

## Deprecated

- None.

## API Snapshots

The checked-in V1 audit baseline lives under `api/v1/`.

Run:

```bash
just api-check
```

Refresh intentionally:

```bash
just api-update
```
