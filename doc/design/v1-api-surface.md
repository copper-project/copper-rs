# V1 API Surface

This file defines the Copper V1 public contract. Anything not listed as stable is not covered by V1 semver guarantees.

## Labels

- `stable`: covered by V1 semver.
- `experimental`: usable, but may change without a major version bump.
- `internal`: public only because proc macros, generated code, tests, or rustdoc need a path.
- `deprecated`: still callable, but not part of new V1 design.

The logreader CLI parser types (`cu29_export::LogReaderCli`, `Command`, and
`ExportFormat`) are experimental. `list-runs` and the global `--run`
option select constructions discovered from section context, including runs whose
`Instantiated` marker has rolled out; see [run identity](log-storage.md).

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
  - `CuStatelessTask` (`Reflect + Send + Sync`; no `Freezable` requirement)
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

### Clock synchronization

The clock surface includes `cu29_clock::{CuInstant, sync}`, `RobotClockMock::raw_now`,
and `RobotClock::{raw_now, busy_wait_for, busy_wait_until, sync_status, origin,
new_with_frequency}`. Raw reads, busy waits, status, and origin are available without
`clock-sync`; live discipline and known-frequency construction require it.
`ClockSync` owns discipline; `ClockSnapshot` preserves mapping, quality, and origin
for replay. `ClockOrigin`, `ClockSource`, and `ClockEpoch` separate provenance from
timestamp interpretation. Named-reference constructors are
`from_ref_time_with_origin` and `from_ref_time_with_rtc_and_origin`.

`CuInstant` is raw counter time; `CuTime` is execution time. The instance-free
`CuInstant::now()`/`Instant` API and free busy-wait functions are removed. Existing
`RobotClock` constructors/mock controls retain their source compatibility,
including the legacy mock `now()` counter view. See [clock synchronization](clock-sync.md)
for epoch, monotonicity, error bounds, replay, and migration details. Core discipline
uses allocation-free `SyncError`; runtime adapters return `CuError`.

## Experimental

- `cu29_graph_view::{render_config_svg, render_multi_config_svg}`: host-side,
  in-memory SVG rendering. The SVG markup and layout are presentation output, not
  a stable serialization format.

- `cu29::logstream::CuTwinBuilder`: `with_slab_size`, `with_section_size`,
  `with_receiver_limits`, `with_replay_capacity`, and `with_log_capacity` configure
  receiver-side archive and buffering budgets. `SessionRouterLimits::default()`
  supplies the single-sender 1200-byte-MTU, 64-symbol profile.

- `self-describing-logs` feature for automatic startup catalog recording;
  `decode-catalog` for host-side `cu29::value_decode`, and
  `cu29_value::decode`: `ValueDecodeDescription` builds portable wire/schema
  descriptions and decodes native payload bytes to `Value` trees offline.
- `cu29::prelude::{ValueDecode, ValueDecodeSpec}` are always available. The
  companion trait and static wire operations are supplied by `cu-bincode`;
  `#[bincode(describe)]` generates descriptions for individual `Encode` types.
  `cu29::prelude::{ValueDecodeDescription, ValueDecodeLimits}` requires
  `decode-catalog` for offline tooling.
- `cu29::value_decode_catalog` and `cu29_value::catalog`: versioned shared payload
  catalogs and offline decompression. The startup writer borrows native recipes,
  shares one graph across compiled mission slot maps, and compresses into the
  static metadata section using fixed working memory. Catalog version 1 uses
  a bincode-encoded version field, typed metadata, Heatshrink compression, and a
  length/checksum footer; see [catalog design](self-describing-logs.md).
- `cu29_export::catalog`: experimental shared catalog loading and fallible
  run-scoped standalone CopperList value readers. `ValueDecodeDescription::validate`
  checks complete graphs; `decode_at` decodes selected bindings without graph copies.
  Catalog CLI formats/decoder selection and registration-free Python catalog
  functions are experimental. Internal shared allocation budgets are hidden.
- `cu29-value-types` and its `cu29_value` / `cu29::prelude` re-exports:
  `Quantity`, `QuantityMetadata`, `StorageUnit`, `TimeStorageUnit`, and
  `ValueMetadata`. `ValueDecode::METADATA` attaches the Copper-owned vocabulary
  to schemas; `ValueDecodeMetadata` preserves future IDs in portable descriptions.
  `Scalar` and `RecordShape` are shared with the codec and retain their existing
  `ValueDecodeScalar` and `ValueDecodeShape` aliases in `cu29_value::decode`.
- Standard `ValueDecode` implementations for Copper time, compact strings,
  quantities, `CuArray`, `CuArrayVec`, and pooled buffer handles.

- Background empty-input dispatch policy: `background_process_empty` and the defaulted
  `CuAsyncTask<T, O, const PROCESS_EMPTY: bool = false>` parameter.
  Empty inputs are skipped by default while completed results are collected once. Set
  `background_process_empty: true` to opt into dispatching empty inputs.

- LogStream session manifests and `ReceiverRequirements` decoder geometry/bounds.
  Generated twins seal local application metadata and, with `self-describing-logs`,
  the shared decode catalog before recording captures. Archive config capture is deferred.
  `CuTwinBuilder::with_log_rollover(bytes)` bounds receiver archive storage using
  the unified logger's `OverwriteOldest` policy.
  Session manifests carry the producing construction's `SectionContext`; archive
  sections retain its run, instance and mission indices across rollover. Generated
  twins validate the sender mission against their compiled mission index.
- `remote-debug` feature and `cu29::remote_debug`.
- `parallel-rt` feature and parallel executor APIs.
- `async-cl-io` feature and async CopperList I/O internals.
- `safety-ids` feature and `cu29::safety` metadata collection/export APIs.
- Runtime performance knobs:
  - `sysclock-perf`
  - `high-precision-limiter`
- Low-level logging codec registry APIs in `cu29::logcodec`.
- Low-level monitoring probes and allocation counters.
- Direct unified-log section/header structs, `ApplicationMetadata`, `SectionContext`,
  `CapacityPolicy`, and the metadata-aware unreleased format v3
  ([storage design](log-storage.md)). Lifecycle records
  use `RuntimeStopReason` and `RuntimeLifecycleOperation`.

## Internal

- `cu29_graph_view::run_cli`: executable entry point; hidden from rustdoc.

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
- `cu29_unifiedlog::byte_log`: shared storage-adapter implementation.
- Direct task tuple and bridge tuple access through `copper_runtime_mut()`.

## Deprecated

- None.

## API Snapshots

The checked-in V1 audit baseline lives under [`api/v1/`](../../api/v1/).

Use `just api-check` to verify the baseline and `just api-update` only for an
intentional contract update. See [contributor workflow](../../CONTRIBUTING.md).
