# Clock synchronization and replay

Copper separates physical elapsed time from execution timestamps. A reference
disciplines the execution mapping; the local raw counter remains unchanged. This
lets synchronized message timestamps coexist with stable pacing/deadlines and lets
replay restore recorded time without live reference I/O.

## Timelines and provenance

| API/type | Meaning |
| --- | --- |
| `RobotClock::raw_now() -> CuInstant` | Undisciplined local counter converted to nanoseconds using this clock's calibration/origin; captures, elapsed time, acquisition deadlines |
| `now()`, `recent()`, `ctx.now() -> CuTime` | Execution time: local zero/reference origin when unconfigured, reference epoch when synchronized; message ToV |
| `CuDuration` | Interval on either timeline; execution intervals may include slew |
| `ClockSnapshot::at(raw)` | Explicit conversion through a recorded execution curve |
| `origin() -> ClockOrigin` | Source and epoch metadata, including optional TAI–UTC offset; no counter read or reference I/O |

Clones share calibration, mapping, and monotonic floor. Independently constructed
clocks have separate raw origins: compare raw instants only within one clock and
its clones. Rust prevents mixed raw/execution arithmetic, but does not distinguish
raw instants from different clocks.

`ClockDomain { id, identity, session }` identifies the reference; providers must
supply consistent epoch/time-scale semantics. `ClockSource` records provenance,
while `ClockEpoch` describes timestamp interpretation. Parent nanoseconds remain
in the provider's epoch, including GPS or application-defined epochs. Core performs
no calendar/UTC conversion. Linux PTP adapters normalize to TAI. Comparability
requires the same usable domain and epoch, not just nanosecond representation.

## Discipline and failure behavior

Startup acquires `Locked` before consumer start hooks: at least two accepted
observations and estimated error within `max_error`. First acquisition may phase
step to align the epoch. Subsequent correction/resync remains continuous and
uses bounded slew with a positive total rate. Live `now()`/`recent()` never decrease
but may repeat; earlier timestamps are never rewritten. Mock/replay controls can
move time backward deliberately.

```text
execution = time_anchor + (raw - raw_anchor) × (1 + rate_ppb / 1e9)
error = sample_uncertainty + ceil(sample_age_ns × drift_bound_ppb / 1e9)
      + abs(remaining_phase_error_ns)
```

The controller keeps eight observations and publishes an integer affine curve.
One controller owns discipline for a shared clock. Readers use fixed-point
conversion and atomic shared state without allocation, reference I/O, or a blocking
lock. Publication may return `ReadersActive` for deferred retry.

| Condition | Result |
| --- | --- |
| Reference unavailable | Holdover on last curve; uncertainty grows against raw time |
| Error budget or sample age exceeded | Expired; managed runtime rejects the next iteration before consumer hooks |
| Expiry during an iteration | Running iteration is not interrupted |
| `now()` after expiry | Still infallible/monotonic, but no usable synchronization guarantee; no fallback to local epoch |
| Wrong domain, stale/future/duplicate/reordered or inconsistent sample | Observation rejected |
| Reference discontinuity or changed domain/session | Managed execution fails; restart reacquires |
| Resync of aligned clock | Slew, not step; large phase error may exceed acquisition timeout |
| Different reference epoch after alignment | Rejected; selecting a new epoch requires a new clock |

`sync_status()` exposes domain/state, sample age, estimated error/budget, phase
offset, and drift. It is available without `clock-sync`, returning `None` without
counter reads or mapping storage. With discipline, quality ages even between samples.
For the same physical event, peer timestamp differences fit their combined error
bounds only if reference/capture/oscillator assumptions hold.

## Managed runtime integration

Enable `cu29/clock-sync`; select a typed resource through `runtime.clock.parent`,
for example `"ptp.reference"` from `cu_ptp::LinuxPtpBundle`. Code generation resolves
an owned `ClockReferenceBundle::Reference`. The bundle supplies `create_clock()`
unless the builder supplies `with_clock(clock)`; the configured parent disciplines
that injected clock too. `with_clock_reference(provider, maintenance_config)` is
the manual managed alternative and conflicts with a configured parent.

| RON field | Default / constraint |
| --- | --- |
| `max_error_ns` | Required, positive |
| `acquisition_timeout_ns` | 10 seconds |
| `sample_interval_ns` | 1 second |
| `max_age_ns` | 5 seconds |
| `drift_bound_ppb` | 100,000 (100 ppm) |
| `max_slew_ppb` | 500,000 (500 ppm additional phase correction) |

At 100 ppm, drift consumes 100 μs per second before capture/phase error; choose
cadence and budget together. Feature enablement alone keeps local time. Configuring
`runtime.clock` without the feature is an error. Managed sync currently requires
the Serial planner; Pipeline is rejected.

Startup polls during acquisition. On `std`, a maintenance worker performs reference
I/O into a fixed-capacity mailbox; the runtime publishes corrections and checks
health between iterations. On bare metal, bounded foreground polling occupies that
boundary. Shutdown joins maintenance and stops the provider. Tasks never poll the
reference inside `process()`.

## Reference boundary

| Adapter | Responsibility |
| --- | --- |
| `LinuxPtpBundle` / `LinuxPhc` (`cu-ptp/linux-phc`) | Read PHC and ptp4l management health, verify selected port PHC, combine configured reference error, reported offset, and bracketed capture uncertainty; UTC needs a valid offset |
| `LinuxSystemPtpBundle` | System-clock reference for software-timestamped experiments; external setup supplies synchronization |
| `BoardPtp<HZ>` | BSP start/poll/domain/read/stop hooks and known positive counter frequency; BSP handles counter setup, rollover, bounded service |
| Custom `ClockReference` | Supply observations/domain/health and export through a resource bundle or builder |

The adapters consume externally synchronized references; they do not implement a
PTP servo or steer the PHC. Cortex-M retains its counter backend's single-reader
contract. Standalone users own `ClockSync::{new, observe, reference_lost, update,
status, resync}` and cadence/acquisition/shutdown themselves. Core supports `no_std`,
fixed storage, and allocation-free `SyncError`; runtime adapters translate to `CuError`.

Zenoh carries ToV unchanged: no rebasing, domain validation, or added clock metadata.
Drivers/applications must establish compatible device/peer epochs and quality.
Statime transport, automatic PTP setup, and RTC/GNSS adapters remain outside this
implementation.

## Replay and migration

Managed execution records the initial mapping and corrections/quality as
`ClockSyncRecord { culistid, snapshot }` in `RuntimeLifecycle`. The ID identifies
the first CopperList using the curve. Recorded/debug/distributed replay restore
snapshots without starting live reference maintenance. Snapshots include origin
metadata; seeking is not constrained by live monotonicity.

The raw-time API replaces instance-free `CuInstant::now()`/`Instant` and free
busy-wait functions with `clock.raw_now()` and `clock.busy_wait_*()`. Use a clone of
the application's clock for deadlines. Stamp ToV with `ctx.now()`; copying raw
nanoseconds into `CuTime` does not convert epochs. Physical pacing must use raw
time once execution time can slew. `RobotClockMock::now()` retains its legacy raw
counter view; compare execution time through the associated `RobotClock` instead.

Existing `CuTime` encoding is unchanged. ClockSync lifecycle events require a
matching reader built with `clock-sync`; old logs acquire no corrections
retroactively. Container compatibility is independently governed by the
[log format](log-storage.md).

## Code and coverage

| Concern | Pointer |
| --- | --- |
| Counter calibration, timelines, clocks | [`cu29_clock/src/lib.rs`](../../core/cu29_clock/src/lib.rs), [`raw_counter/`](../../core/cu29_clock/src/raw_counter/) |
| Domain/quality/snapshot types | [`sync.rs`](../../core/cu29_clock/src/sync.rs), [`origin.rs`](../../core/cu29_clock/src/origin.rs) |
| Curve publication/controller and deterministic counter tests | [`sync/live.rs`](../../core/cu29_clock/src/sync/live.rs) |
| Runtime acquisition/maintenance | [`maintenance.rs`](../../core/cu29_runtime/src/clock_sync/maintenance.rs), [RON validation](../../core/cu29_runtime/src/config.rs) |
| Mocked acquisition before consumer start and provider shutdown | [`tests/clock_sync.rs`](../../core/cu29_runtime/tests/clock_sync.rs) |
| PTP capture uncertainty, health, binary management fixtures | [`cu_ptp/src/linux.rs`](../../components/res/cu_ptp/src/linux.rs) |
| Correction recording/restoration | [`clock_sync.rs`](../../core/cu29_runtime/src/clock_sync.rs), [replay](../../core/cu29_runtime/src/replay.rs), [generated runtime](../../core/cu29_derive/src/runtime_macro.rs) |

Configuration examples and adapter setup: [cu-ptp README](../../components/res/cu_ptp/README.md).
