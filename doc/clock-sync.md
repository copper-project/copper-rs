# Clock synchronization: minimal API proposal

Status: design sketch; new APIs below are proposed, not implemented.

**Contract:** configuring synchronization makes `RobotClock::now()` and
`ctx.now()` return the shared reference time in nanoseconds. With PTP, this is
PTP/TAI time in a common epoch across robots and sensors. Startup acquires sync
before consumers run; ongoing corrections preserve monotonicity. Existing code
that stamps messages with `ctx.now()` gets synchronized timestamps.

## Delivery stages

| Version | Deliverable |
| --- | --- |
| v0 (agreed) | Discipline RobotClock's `now()` against an already synchronized PTP reference: Linux PHC and board adapters, acquisition, continuous maintenance and health. |
| v1 (proposal) | Add a Statime transport adapter over the existing Zenoh session. Discipline a software PTP clock on either platform, or an explicitly selected Linux/MCU hardware clock. |
| Later adapters | RTC and GNSS/PPS observations through the same core API. |

## Default simple path: configure a resource

On Linux and bare metal, select one parent resource and an acceptable error.
The generated runtime creates and disciplines one RobotClock. After acquisition,
its ordinary `now()` uses the parent's epoch and tracks its offset and drift.

**Linux:**

```ron
(
    resources: [
        (
            id: "ptp",
            provider: "cu_ptp::LinuxPtpBundle",
            config: { "device": "/dev/ptp0", "management": "/var/run/ptp4l" },
        ),
    ],
    runtime: (
        clock: (parent: "ptp.reference", max_error_ns: 100_000),
    ),
    // Existing tasks, bridges and connections follow.
)
```

**MCU/bare metal:** the same clock configuration selects a board resource.

```ron
(
    resources: [
        (id: "ptp", provider: "crate::board::PtpBundle"),
    ],
    runtime: (
        clock: (parent: "ptp.reference", max_error_ns: 100_000),
    ),
    // Existing tasks, bridges and connections follow.
)
```

Both bundles export `reference` with the same typed reference/controller
contract, plus local-counter setup for the generated builder. Linux selects the
hosted counter; the board bundle supplies raw-counter initialization and its
known frequency. Resources supply these inputs; the runtime constructs the
execution clock. Normal board/HAL initialization stays in the BSP.

Application code uses its usual builder and lifecycle on either platform:

```rust,ignore
App::builder().build()?.run_until_shutdown()?;
```

These RON fields and bundle contracts are proposed. Resolve the parent and
local-counter setup at code generation; one parent per runtime. In v1 a Zenoh
reference provider attaches to the named existing bridge session during startup
and disciplines an overlay by default. Resolve that typed session handle at
code generation too. PHC discipline is an explicit provider/backend choice.

Defaults: acquisition timeout 10 seconds, reference sampling 1 Hz, maximum sample
age 5 seconds, relative drift bound 100 ppm. `max_error_ns` is required. The age
limit is a ceiling: the error budget can expire sooner. Override cadence/bounds
for the actual board/link; servo gains stay internal.

### Access from tasks and drivers

```rust,ignore
let timestamp = ctx.now();             // synchronized, monotonic PTP/TAI time
output.tov = Tov::Time(timestamp);      // existing timestamping code
let quality = ctx.clock.sync_status(); // domain, state and estimated error
```

`now()`, `recent()`, clock clones and runtime message/process/log timestamps use
the same disciplined timeline. Robots following the same reference share an
epoch; their readings agree within their combined error budgets. `sync_status()`
returns `None` for an unsynchronized clock. Replay restores recorded clock
discipline and never starts live parent I/O.

### Features

Keep observation/discipline arithmetic in `cu29-clock` without platform dependencies.
Add one opt-in `cu29/clock-sync` feature for runtime integration, enabling the
corresponding `cu29-clock/clock-sync` feature for clock storage and discipline.
Standalone/manual users enable the clock crate's feature directly. Put PHC/Statime
dependencies in a `cu-ptp` component:
`linux-phc` for the Linux adapter, `ptp` for protocol/overlay support on either
platform. Zenoh synchronization is an opt-in bridge feature `ptp-sync`; it
enables the protocol pieces. Features enable capabilities; RON selects behavior.
Unconfigured applications retain their local epoch and behavior.

### RobotClock storage and read mapping

Select the clock representation at compile time:

- Each clock owns its hardware-counter calibration. `raw_now()` is available
  with or without synchronization and clones retain the same counter origin.
- With `cu29-clock/clock-sync` disabled, `now()`/`recent()` subtract the clock's
  raw anchor and add its requested initial time.
- With the feature enabled, use shared mapping state containing
  `raw_anchor`, `time_anchor` and fixed-point `rate`. Both `now()` and `recent()`
  evaluate this mapping, whether or not a synchronization parent is configured.
  Clock clones share the published mapping and synchronization status.

The mapping is:

```text
now = time_anchor + rate * (raw - raw_anchor)
```

Here `raw` is undisciplined local counter time expressed in nanoseconds. Initialize
an unconfigured clock with `raw_anchor` at construction, `time_anchor = 0` and
`rate = 1`, preserving the local epoch. The `from_ref_time` constructors instead
set `time_anchor` to their requested initial time. Mocks initialize both anchors
to zero and the rate to one, preserving their existing control semantics.

Acquisition establishes the parent's epoch through these same anchors; subsequent
updates preserve continuity and adjust the rate as specified below. The mapping
owns the output epoch: feature-enabled builds have no separate output-offset member
or additional output-offset subtraction. Feature availability alone does not
select a parent or change the clock to a shared epoch.

`raw_now()` returns `CuInstant` on the undisciplined counter timeline. `now()`
and `recent()` return `CuTime` on the execution timeline. Raw instants and
execution timestamps are distinct types; raw deadlines and observations accept
`CuInstant`, and intervals on either timeline use `CuDuration`. A recorded
`ClockSnapshot::at(instant)` explicitly maps a raw instant to execution `CuTime`. Reference captures, acquisition timeouts and sample aging
use the raw timeline; discipline updates never alter it. Replay restores the recorded mapping
and synchronization status into the same shared state.

## One parent per clock

Each synchronized robot has one configured parent; parents can serve several
children. Wiring is explicit and acyclic. Changing parent requires reacquisition.

```text
                  PTP grandmaster
                   /           \
          host / MCU PTP clock  PTP lidar
                   |
          disciplined RobotClock.now()
```

`ClockSync` estimates parent time from an undisciplined local counter, then
controls the offset/rate used by `RobotClock::now()`. Sample that raw counter
when measuring drift, so clock correction never feeds back into the estimator.
PTP sensor and bridge timestamps in the same domain already use the same epoch.

## What the default path runs underneath

The Linux and board providers adapt their hardware to the same maintenance
contract. This is the low-level API below, driven by the generated runtime:

| Default path | Low-level operation |
| --- | --- |
| Build from the parent resource | Resolve typed local-counter setup; construct one RobotClock with the hosted counter or `new_with_frequency(hz)`. Its clones share discipline state. |
| Apply configuration | Set `SyncConfig` error, age, drift and slew bounds; cadence and acquisition timeout belong to runtime maintenance. |
| Start | Start required transport/reference controller. Obtain the parent domain and call `ClockSync::new(&clock, domain, config)`. Feed paired raw/reference samples through `observe` and `update` until `Locked`; then start consumers. |
| Maintain | Poll/sample, call `observe`, then `update` to publish continuous offset/rate corrections. Check `status()` before running consumers. |
| Read | `ctx.now()` reads the raw counter and evaluates the current disciplined clock curve. Ordinary ToV stamping uses that result. |
| Lose/reset reference | Continue monotonic holdover within the error/age limits. Expiry or a detected parent step/session change stops consumers with a runtime error. Restart calls `resync(parent.domain())` and reacquires without jumping an existing clock. |
| Stop | Call the controller's `stop` before releasing its transport. |

Linux maintenance uses a worker; bare-metal maintenance uses bounded foreground
polling between iterations. Both run outside `process`. Startup also polls while
acquiring, before the first iteration. Replay restores recorded clock discipline
state and skips live reference maintenance. Acquisition timeouts and sample age
use raw time, independent of clock correction. Acquisition timeout is an error.

## Monotonic discipline

On first acquisition, establish the shared epoch before publishing synchronized
time to consumers. During operation, preserve the current clock value when
changing its rate; correct phase error by bounded slew. Rate stays positive.
Default maximum additional phase-correction slew is 500 ppm, configurable as
`max_slew_ppb: 500_000`. Initial alignment is the only permitted phase step.

Reported error includes reference/capture uncertainty, drift extrapolation and
remaining phase error between the published `now()` curve and the parent.
`Locked` requires that total to fit `max_error`. Previously stamped times remain
unchanged. A resync on an existing clock preserves continuity; if it cannot
reacquire within the budget/timeout, starting consumers fails.

During reference loss, `now()` continues the last disciplined curve. Exceeding
the budget marks synchronization expired and stops the managed runtime;
`now()` remains infallible and monotonic. There is no fallback to a local epoch.

## Escape hatches

### Custom reference, runtime-managed

Implement a concrete typed `ClockReference` provider for RTC, GNSS/PPS or an
exotic nanosecond counter. It supplies local-counter setup and reference
start/poll/stop, domain and paired `ClockObservation`s through the same provider
contract. Configure it as a resource, or inject it programmatically:

```rust,ignore
App::builder()
    .with_clock_reference(provider)
    .build()?
    .run_until_shutdown()?;
```

Maintenance, health checks and consumer access remain runtime-managed. Choose
either an injected provider or a RON parent; conflicting selections are errors.
Existing `with_clock(clock)` optionally overrides the provider's local-counter
setup with an application-supplied base clock. It injects execution time;
`with_clock_reference` selects the synchronization parent. With a parent
selected, the injected clock's `now()` is disciplined too.

`clock.raw_now()` exposes the undisciplined local counter for capture, controller
bookkeeping and physical timing. It is a low-level escape hatch, not the clock
used to stamp synchronized messages.

### Fully manual synchronization

For standalone use or custom lifecycle/control, construct `RobotClock`, the
parent adapter and `ClockSync` yourself. Own polling, acquisition, health checks,
resynchronization and shutdown. The following examples demonstrate this path.

### Core API (`cu29-clock`, also `no_std`)

```rust,ignore
struct ClockObservation {
    raw_local: CuInstant,      // undisciplined local time at the measured event
    parent_ns: u64,            // parent time at that SAME event
    uncertainty: CuDuration,   // reference error + capture/read/transport error
    domain: ClockDomain,       // identifies epoch, time scale and clock session
}

struct SyncConfig {
    max_error: CuDuration,     // application's acceptable timestamp error
    max_age: CuDuration,       // maximum age of the last accepted observation
    drift_bound_ppb: u32,      // relative oscillator error bound during holdover
    max_slew_ppb: u32,         // bound on additional phase-correction slew
}

impl ClockSync {
    fn new(clock: &RobotClock, parent: ClockDomain, config: SyncConfig)
        -> CuResult<Self>;     // attach to this clock's shared discipline state
    fn observe(&mut self, sample: ClockObservation) -> CuResult<()>;
    fn update(&mut self) -> CuResult<SyncStatus>; // acquire/slew the actual now()
    fn resync(&mut self, parent: ClockDomain) -> CuResult<()>;
    fn status(&self) -> SyncStatus;              // ages against raw time
}
```

One controller owns discipline for a clock; clones share its published state.
`SyncStatus` reports state (`Acquiring | Locked | Holdover | Expired`), domain,
sample age, total estimated error, signed phase offset in ns and drift in ppb.
Use a bounded observation window. Reject wrong domains, stale/out-of-order
samples and inconsistent measurements. Parent discontinuity invalidates lock.

The clock evaluates `time_anchor + rate * (raw - raw_anchor)` using integer
nanoseconds, fixed-point rate and signed wide intermediates. Online curve
updates preserve continuity; estimator changes never directly step `now()`.
Holdover uncertainty grows by `elapsed_ns * drift_bound_ppb / 1_000_000_000`.
Reject arithmetic overflow and out-of-range reference timestamps.

### Manual setup: Linux or bare metal

Linux:

```rust,ignore
let clock = RobotClock::new();
// Proposed adapter: PHC reads + upstream health/time properties from ptp4l.
let mut parent = LinuxPhc::open("/dev/ptp0", "/var/run/ptp4l")?;
```

Bare metal:

```rust,ignore
// New constructor: existing architecture counter, known ticks/second, no sleep.
let clock = RobotClock::new_with_frequency(board::RAW_COUNTER_HZ)?;
let mut parent = board::ptp_reference()?; // hardware clock or software PTP clock
```

Then the same controller on either platform:

```rust,ignore
parent.start()?;
let mut sync = ClockSync::new(&clock, parent.domain(), SyncConfig {
    max_error: CuDuration::from_micros(100),
    max_age: CuDuration::from_secs(5),
    drift_bound_ppb: 100_000,
    max_slew_ppb: 500_000,
})?;

// Own cadence/timeout: repeat until Locked before exposing clock to consumers.
parent.poll()?;
sync.observe(parent.sample(&clock)?)?; // sample uses raw_now(), not now()
let health = sync.update()?;          // actually disciplines clock.now()

// Once Locked, ordinary clock reads and stamps use the shared epoch.
let timestamp = clock.now();
// Keep poll/observe/update running outside process(); check health.

sync.resync(parent.domain())?; // reacquire while preserving existing continuity
// Repeat acquisition with consumers stopped; parent.stop() at shutdown.
```

[`ptp4l`](https://www.linuxptp.org/documentation/ptp4l/) runs the protocol and
disciplines the PHC; the [kernel PHC API](https://docs.kernel.org/driver-api/ptp.html)
exposes clock reads/adjustments. Copper's v0 adapter reads it and cross-timestamps
against `raw_now()`, using bracketed reads when precise correlation is unavailable.
It includes upstream error plus capture error and stops supplying fresh samples
when upstream lock/quality is unusable. Copper disciplines its RobotClock in
software. Normalize reference time to PTP/TAI; UTC requires known leap offsets.

`board::ptp_reference` is a BSP integration point: use an existing PTP service or
Statime with native Ethernet. It can expose an adjustable hardware timer or a
software PTP clock backed by a monotonic counter. Statime's embedded support and
STM32 example are described in its [crate documentation](https://docs.rs/statime/latest/statime/).
The proposed `crate::board::PtpBundle` supplies this counter setup and wraps
`board::ptp_reference` for runtime maintenance.
The BSP handles timer rollover/correlation. ISR captures/enqueues; foreground
runs the service. Keep the current Cortex-M DWT backend's single-reader contract.
Core estimation/discipline uses fixed storage and needs no OS/thread.

## Where PTP, sources and Zenoh fit

| Piece | Responsibility / placement |
| --- | --- |
| `ClockSync`, `RobotClock` | Transport-independent estimation and monotonic discipline of `now()` in `cu29-clock`. |
| PTP reference adapter | Own clock/health handles through a resource bundle; produce `ClockObservation`. |
| Clock reference bundle | Supply raw-counter setup and parent/controller handles; generated runtime constructs and maintains the synchronized RobotClock used by all consumers. |
| PTP service | v0 Linux: externally configured `ptp4l`. Bare metal: board service or Statime. v1: Statime plus a Copper Zenoh transport adapter. |
| Lidar source | Configure supported PTP mode; normalize device timestamps to the shared domain, check quality/range and stamp ToV in the same epoch as `ctx.now()`. Keep the ordinary `CuSrcTask` lifecycle. |
| Zenoh bridge | v0: carry ordinary synchronized ToV plus domain/quality metadata. v1: dedicated PTP event/general routes on its existing session, timestamped before graph queues. |

In v0, Copper and a lidar follow the same PTP reference as siblings. Device PTP
mode and profile are configured through the device driver. Native PTP uses its
standard transport; ordinary sensor drivers keep their existing task lifecycle.

Parent I/O/estimation runs in runtime-managed Linux maintenance or bare-metal
foreground, outside `process`. Preallocate discipline state; publish coherent
clock-curve updates at iteration boundaries. Synchronized `now()` adds fixed-point
conversion and shared-state reads, with no parent I/O, allocation or blocking
lock. Feature-disabled builds retain the current read path. Record initial epoch,
curve updates and quality outside `process` for deterministic replay.

Start reference sampling at 1 Hz. Adjust to the error budget: 20 ppm
consumes 100 us in five seconds before reference/transport error.

## v1 proposal: PTP transport over Zenoh + local disciplined clock

```text
parent PTP time -- Zenoh/radio/serial --> follower PTP clock
                                          /          \
                                RobotClock.now()   local native PTP
                                                     to lidar (optional)
```

Use Statime's PTP messages, state machine and servo with a Copper transport
adapter: encapsulate event/general packets over statically configured parent and
child routes. Correlate sequence IDs and capture software Tx/Rx timestamps at
publication/callback boundaries. Preserve root time properties and accumulate
upstream uncertainty. This is a custom PTP transport between Copper peers;
standard PTP devices join through a native Ethernet port/service.

Clock backend selection is explicit:

| Backend | Discipline target |
| --- | --- |
| Software (Linux and bare metal) | `statime::OverlayClock` over a read-only raw-counter adapter. Statime disciplines this PTP reference; the v0 controller disciplines RobotClock's `now()` from it. |
| Linux PHC | Open `/dev/ptpN` with `clock-steering`; servo adjusts its phase/frequency. One controller owns discipline. |
| MCU hardware timer | BSP implements `statime::Clock` for an adjustable timer. |

The software backend needs no kernel clock device or PTP-capable Ethernet MAC.
Each backend supplies the reference for the same RobotClock discipline API:
ordinary `ctx.now()` follows the Zenoh parent's time. The raw-counter adapter
must read `raw_now()`, keeping the two controllers independent. Physical
downstream PTP service requires a native transport and suitable timestamping.
Apply the initial-alignment/bounded-slew policy to Copper-controlled PTP
backends too; resync preserves an existing clock's continuity. Large errors
invalidate lock and require reacquisition. Enforce this around Statime's clock
interface, since an overlay permits steps. If Copper controls a PHC, configure
local PTP serving without a competing servo and propagate upstream loss/degraded
quality to downstream clocks.

Reuse candidates:

- [Statime `OverlayClock`](https://docs.rs/statime/latest/statime/struct.OverlayClock.html): software phase/frequency adjustment over a read-only clock.
- [Statime filters](https://docs.rs/statime/latest/statime/filters/index.html): measurement filtering/clock servo; retain integer/fixed-point timestamp conversion at the Copper boundary.
- [`clock-steering::UnixClock`](https://docs.rs/clock-steering/latest/clock_steering/unix/struct.UnixClock.html): Linux PHC reads, phase steps and frequency control. Account for its frequency units when adapting to Statime.

These provide reusable clock/protocol pieces; the Copper Zenoh adapter is new
integration work. Validate attainable error on the actual radio/serial path:
queueing, retransmissions and delay asymmetry limit software timestamp accuracy.
Zenoh supports [serial links](https://spec.zenoh.io/spec/1.0.0/transport/links.html);
this checkout's bridge currently enables TCP/UDP/Unix sockets. Direct serial
transport and a bare-metal Zenoh frontend require separate transport integration.

## Compatibility and first implementation

- Existing constructors, `now()`, mocks, `with_clock` and `CuSrcTask` remain
  source-compatible. Unconfigured clocks retain their local epoch/behavior;
  opting into sync makes `now()` use the shared reference epoch. Mark new APIs
  experimental.
- Add typed `runtime.clock` configuration and generated lifecycle integration;
  both platforms use the resource-configured path. Build resources and resolve
  their local-counter setup before constructing the execution clock and
  consumers. Manual polling and clock injection are optional escape hatches.
- Base-counter initialization/calibration must not disturb existing clocks.
  `new_with_frequency` takes a positive raw-counter frequency on bare metal.
- v0 implements actual `now()` discipline, read-only Linux PTP reference adapter,
  known-frequency constructor, runtime maintenance and Linux/board reference
  bundles with examples of the default path. Clock discipline over Zenoh belongs
  to v1. RTC/GNSS remain later reference adapters through the custom-provider
  contract.
- Version synced Zenoh attachments for ToV domain/session and uncertainty.
  ToV already uses shared time: preserve both range endpoints on send/receive,
  validate domain/session and account for sender uncertainty when comparing to
  receiver time. A foreign domain or unusable individual timestamp yields
  `Tov::None`, preserving origin and logging the reason. Clock expiry follows the
  runtime error policy above. Synced peers upgrade together; retain attachment
  version 1 operation for unsynchronized clocks.
- Verify two robots with different boot times/raw-counter rates get comparable
  `now()` values, stay monotonic through positive/negative correction and resync,
  and keep `recent()`/clones on the same timeline. Also verify total error includes
  pending slew, upstream loss despite fresh PHC reads, expiry, parent reset, timer
  rollover, unchanged shared-domain ToV ranges and deterministic replay on host
  and `no_std`. v1 adds software-overlay continuity, exclusive PHC control and
  transport loss/reordering/asymmetry checks.
