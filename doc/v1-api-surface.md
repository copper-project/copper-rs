# V1 API Surface

This file defines the Copper V1 public contract. Anything not listed as stable is not covered by V1 semver guarantees.

## Labels

- `stable`: covered by V1 semver.
- `experimental`: usable, but may change without a major version bump.
- `internal`: public only because proc macros, generated code, tests, or rustdoc need a path.
- `deprecated`: still callable, but not part of new V1 design.

The logreader CLI parser types (`cu29_export::LogReaderCli`, `Command`, and
`ExportFormat`) are experimental. `list-runs` and the global `--run`
option select recorded runs identified by `Instantiated` lifecycle records.

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
  - `CuStatelessTask`
  - `CuSinkTask`
  - `CuBridge`
  - `CuMsg`
  - `CuMsgPayload`
  - `CuMsgMetadata`
  - `input_msg!`
  - `output_msg!`
  - `BridgeChannel`
  - `BridgeChannelSet`
- Safety case authoring APIs:
  - `#[safety_case("...")]`
  - `safety_check!`
  - `safety_check_eq!`
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

- `self-describing` feature, `cu29::value_decode`, and
  `cu29_value::decode`: `ValueDecodeDescription` builds portable wire/schema
  descriptions and decodes native payload bytes to `Value` trees offline.
- `cu29::prelude::{ValueDecode, ValueDecodeSpec, ValueDecodeDescription,
  ValueDecodeLimits}` with `self-describing` enabled. The companion trait and
  static wire recipes are supplied by `cu-bincode`.
- Standard `ValueDecode` implementations for Copper time, compact strings,
  quantities, `CuArray`, and `CuArrayVec`.

- Background empty-input dispatch policy: `background_process_empty` and the defaulted
  `CuAsyncTask<T, O, const PROCESS_EMPTY: bool = false>` parameter.
  Empty inputs are now skipped by default while completed results are collected once.
  This corrects a specification bug to match the intended behavior. Set
  `background_process_empty: true` to opt into dispatching empty inputs.

- LogStream session manifests and `ReceiverRequirements` decoder geometry/bounds.
- `remote-debug` feature and `cu29::remote_debug`.
- `parallel-rt` feature and parallel executor APIs.
- `async-cl-io` feature and async CopperList I/O internals.
- `safety-ids` feature and `cu29::safety` metadata collection/export APIs.
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
- `CuListZeroedInit`: hook for initializing pool storage in place and
  resetting per-cycle metadata. Generated message datasets implement it.
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
