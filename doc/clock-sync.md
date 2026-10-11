# Copper Clock synchronization and guarantees

## Two timelines

| API | Timeline and guarantee |
| --- | --- |
| `clock.raw_now() -> CuInstant` | Undisciplined local counter, converted to nanoseconds using this clock's own origin and calibration. Use for captures, physical elapsed time, acquisition deadlines and sample age. Synchronization never changes it. |
| `clock.now()`, `clock.recent()`, `ctx.now() -> CuTime` | Execution time. An unconfigured clock starts at zero, or the initial time requested by `from_ref_time*`. A synchronized clock follows its reference epoch; PTP adapters normalize to TAI. |
| `CuDuration` | An interval on either timeline. Execution intervals can reflect slew; raw intervals do not. |

Clones share the raw origin/calibration and execution mapping. Independently
constructed clocks have separate raw timelines: compare `CuInstant`s only within
one clock and its clones. Rust rejects mixed raw/execution arithmetic, comparisons
and implicit conversions, but does not distinguish two clocks' raw instants.
Raw busy-wait deadlines and `ClockObservation.raw_local` require `CuInstant`;
message ToV requires `CuTime`. `ClockSnapshot::at(raw)` explicitly maps an instant
through a recorded execution curve.

## Synchronized clock contract

- **Shared epoch:** ordinary execution timestamps use the selected reference's
  epoch. Comparable peers must have the same `ClockDomain` (`id`, root `identity`,
  `session`) and time scale. The domain fields identify the reference; providers
  are responsible for supplying consistent epoch/time-scale semantics.
- **Startup:** managed acquisition reaches `Locked` before consumer `start`
  hooks. Lock requires at least two accepted observations and estimated error
  within `max_error`. Construction alone does not acquire synchronization.
- **Monotonicity:** first acquisition may align the epoch by a phase step.
  Thereafter live corrections and resync preserve continuity and correct phase
  by bounded slew, with positive total rate. `now()` and `recent()` share a
  monotonic floor across clones: readings do not decrease, but may repeat.
  Earlier stamped timestamps are never rewritten. Mock controls and offline
  replay can deliberately move time backwards.
- **Accuracy:** quality includes upstream/capture uncertainty, drift since the
  last sample, and remaining phase error. `Locked` and usable `Holdover` fit
  `max_error`. For the same physical event, two peers' timestamp difference is
  bounded by their combined errors, provided the reference and oscillator bounds
  are valid. Nanosecond representation does not imply nanosecond accuracy.
- **Loss and expiry:** an unavailable reference enters holdover on the last
  curve. Exceeding the error budget or maximum sample age expires synchronization.
  The managed runtime rejects the next iteration before consumer hooks; it does
  not interrupt an iteration already running. `now()` remains infallible and
  monotonic after expiry, but has no usable synchronization guarantee and never
  falls back to a local epoch.

`clock.sync_status()` returns domain, state, age, estimated error, budget, signed
phase offset and drift; it returns `None` before discipline is published.
The method and its `SyncStatus` return type are available in both feature modes.
Without `clock-sync`, it returns `None` without reading the counter or adding
mapping storage. Tasks can inspect quality with identical code before and after
enabling synchronization. Live discipline and reference maintenance remain gated.
Quality ages against raw time even without a new observation. Wrong domains,
stale/future/duplicate/reordered samples and inconsistent measurements are
rejected. Detected reference discontinuity or a changed domain/session fails
managed execution; restart reacquires. Resync on an already aligned clock slews
rather than steps, so a large offset can prevent acquisition within the timeout.

The published curve uses integer arithmetic:

```text
execution = time_anchor + (raw - raw_anchor) * (1 + rate_ppb / 1_000_000_000)
estimated_error = sample_uncertainty
                + ceil(sample_age_ns * drift_bound_ppb / 1_000_000_000)
                + abs(remaining_phase_error_ns)
```

The controller keeps eight observations. One controller owns each clock; clones
share coherent mapping and quality. Publication can return `ReadersActive` and
be deferred. Reads do fixed-point conversion and atomic shared-state access,
without allocation, reference I/O or a blocking lock.

## Default path: configure one reference resource

Enable `cu29/clock-sync` and, for Linux adapters, `cu-ptp/linux-phc`. Add this to
an application's graph configuration:

```ron
resources: [
    (
        id: "ptp",
        provider: "cu_ptp::LinuxPtpBundle",
        config: {
            "device": "/dev/ptp0",
            "management": "/var/run/ptp4lro",
            "reference_error_ns": 10_000,
        },
    ),
],
runtime: (
    clock: (
        parent: "ptp.reference",
        max_error_ns: 100_000,
        sample_interval_ns: 100_000_000,
    ),
),
```

The numeric bounds are examples to validate on the actual hardware. Use the
ordinary builder and lifecycle; stamp ToV with `ctx.now()`. `runtime.clock.parent`
is resolved to an owned, typed `ClockReferenceBundle::Reference` resource at code
generation. The provider constructs the base clock through `create_clock()` unless
`with_clock(clock)` supplies one. Selecting a parent disciplines that clock too.
Clock synchronization requires the default Serial planner; Pipeline is rejected.

| Setting | Default |
| --- | --- |
| `max_error_ns` | Required, positive |
| `acquisition_timeout_ns` | 10 seconds |
| `sample_interval_ns` | 1 second |
| `max_age_ns` | 5 seconds |
| `drift_bound_ppb` | 100,000 (100 ppm) |
| `max_slew_ppb` | 500,000 (500 ppm additional phase correction) |

The error budget can expire before the age ceiling. At the default 100 ppm,
100 microseconds is consumed in one second even before capture and phase error;
choose cadence and bounds together. Features enable capabilities; configuration
selects behavior. Enabling the feature alone keeps the local epoch. Declaring
`runtime.clock` without the feature is an error.

Startup polls while acquiring. With `std`, ongoing reference I/O runs on a worker
and feeds a fixed-capacity mailbox; the runtime publishes corrections and checks
health before iterations. On bare metal, bounded foreground polling runs between
iterations. Shutdown joins maintenance and stops the provider. Tasks never poll
the reference in `process()`.

## Reference adapters and manual control

`cu-ptp` consumes an already synchronized reference; it does not implement a PTP
servo or steer the PHC:

- `LinuxPtpBundle` / `LinuxPhc` read a PHC and ptp4l management health, verify the
  selected port's PHC, and combine operator-supplied `reference_error_ns`, reported
  offset and bracketed-capture uncertainty. Unusable/stale upstream quality stops
  fresh samples. UTC conversion requires a valid UTC-to-TAI offset.
- `LinuxSystemPtpBundle` reads the system clock for software-timestamped local
  experiments. The external PTP/system-clock setup supplies synchronization.
- `BoardPtp<HZ>` wraps BSP `start/poll/domain/read/stop` hooks and a known positive
  architecture-counter frequency. The application supplies its resource bundle;
  the BSP initializes the counter, extends rollovers and bounds service work.
  Cortex-M retains the counter backend's single-reader contract.

For a custom managed reference, implement `ClockReference` and export it from a
`ClockReferenceBundle`, or use
`App::builder().with_clock_reference(provider, maintenance_config)`.
An injected reference conflicts with a configured RON parent.

Standalone users enable `cu29-clock/clock-sync` and own `ClockSync` directly:
`new(&clock, domain, config)`, `observe(sample)`, `reference_lost()`, `update()`,
`status()` and `resync(domain)`. Fallible core methods return `Result<_, SyncError>`;
providers use `CuResult`. The caller owns acquisition, cadence, health checks and
shutdown. Core discipline supports `no_std` and uses fixed observation storage.

A sensor following the same PTP root can supply comparable device timestamps,
but its driver must verify device quality/domain and normalize the time scale.
This stack adds no automatic PTP configuration or validation to sensor drivers.

## Transport independence

Clock synchronization consumes an externally synchronized PTP reference independently of Zenoh.
The existing bridge carries `Tov::Time` and both `Tov::Range` endpoints unchanged,
without rebasing, clock-domain validation or new clock metadata. The application
is responsible for ensuring peers share a usable epoch before comparing their
timestamps. Clock synchronization requires no Zenoh bridge or session.

## Replay

Managed execution records the initial mapping and subsequent corrections/quality
as `ClockSyncRecord`s in `RuntimeLifecycle`, associated with the first CopperList
using each curve. Recorded replay, debugger and distributed replay restore them
without starting live reference I/O. Replay explicitly restores recorded time;
live monotonicity does not constrain seeking through a recording.

## v1.2 → v1.3 compatibility and migration

This section covers clock-related changes in this stack. Apps using only
`ctx.now()`, `RobotClock::now()` / `recent()`, `CuTime`, `CuDuration`, existing
constructors, mock controls and `with_clock` need no source changes for these
APIs. Synchronization is opt-in; unconfigured apps retain local execution time.
The following low-level changes apply even without `clock-sync`:

| Incompatibility | Required change in application code |
| --- | --- |
| `CuInstant::now()` and the `Instant` alias are removed. | Replace `Instant` with `CuInstant` and obtain readings from `clock.raw_now()` or `ctx.clock.raw_now()`. Pass or store a clone of the application's clock wherever a standalone instant was previously read. |
| Free functions `busy_wait_for` and `busy_wait_until` are removed. | Call `clock.busy_wait_for(duration)` or `clock.busy_wait_until(deadline)`. Build the deadline from that same clock's `raw_now()` plus a `CuDuration`. |
| Counter origin and calibration are instance-owned, rather than global. | Share clock clones for raw measurements and deadlines. Do not construct a second clock to read or wait on the first clock's raw timeline, or assume constructing a clock recalibrates existing clocks. |

For example, migrate a v1.2 raw deadline:

```rust,ignore
// v1.2
let deadline = Instant::now() + delay;
busy_wait_until(deadline);

// v1.3: clock is the application's RobotClock, or a clone of it.
let deadline = clock.raw_now() + delay;
clock.busy_wait_until(deadline);
```

Keep message timestamps on `CuTime`, obtained from `ctx.now()`. A raw reading is
not a message timestamp; copying its nanoseconds into `CuTime` does not convert
epochs. When enabling synchronization, replace physical pacing or timeout logic
based on execution time with `raw_now()` / `CuInstant`; execution time now uses
the shared epoch and its intervals can include slew. Remove assumptions that
`ctx.now()` means time since startup. Tests comparing execution time against
`RobotClockMock::now()` should instead read the associated `RobotClock::now()`;
the mock accessor retains its legacy counter view. Use `mock.raw_now()` for raw
comparisons.

`CuTime` encoding and existing lifecycle event discriminants are preserved by
this stack. New synchronized recordings contain an experimental `ClockSync`
lifecycle event that v1.2 readers cannot decode: rebuild logreaders/replay tools
against v1.3 with `clock-sync` to consume it. Existing recordings gain no clock
corrections retroactively; their timestamps keep their recorded epoch. This is
not a guarantee of compatibility for unrelated log-format changes in v1.3.

Clock synchronization adds no Zenoh feature or attachment-format change. Existing bridge code needs
no migration for this stack; shared timestamp comparability depends on the
external synchronization setup.

## Future work

Future work could add Statime PTP transport over the existing Zenoh session and explicitly
selected software or hardware reference backends. This requires transport timestamp
capture, delay/asymmetry bounds and downstream quality propagation. The stack adds
no Statime integration, `ptp`/`ptp-sync` feature, PHC steering, direct serial
transport, bare-metal Zenoh frontend or RTC/GNSS reference adapter.
