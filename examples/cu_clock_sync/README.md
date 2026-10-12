# See a PTP-synchronized RobotClock live

Run this demo on one Linux machine. It starts a local PTP reference and opens
ConsoleMon's **CLK** panel so you can see synchronization working immediately.
No second machine or external grandmaster is needed, and the shipped config
already enables `allow_local_master`.

## Quick start

You need Rust, an active Ethernet interface with `/dev/ptp0`, and read access to
that device. On Arch Linux or CachyOS, install the demo tools once:

```sh
sudo pacman -S --needed linuxptp ethtool just
```

The packages are [linuxptp](https://archlinux.org/packages/extra/x86_64/linuxptp/),
[ethtool](https://archlinux.org/packages/extra/x86_64/ethtool/), and
[just](https://archlinux.org/packages/extra/x86_64/just/).

From this example's directory, in an interactive terminal:

```sh
just
```

The [justfile](justfile) finds the interface owning `/dev/ptp0`, starts ptp4l as a local
master, synchronizes the PHC from your system clock with phc2sys, and runs the
Copper app. It asks for sudo to start the daemons and stops the daemons it started
when the demo exits. It also handles management-socket access. If local PTP
services are already running, it reuses them and restores the socket's group on exit.
The master advertises PTP on the selected Ethernet network; use a test network.
Daemon output is saved to `logs/ptp4l.log` and `logs/phc2sys.log` when the demo
starts them.

For a master started by the demo, the current system leap-second table supplies
TAI minus UTC. The demo advertises that offset as valid before starting phc2sys,
so CLK can display local time. The source remains **Internal oscillator**: this
local experiment does not claim GNSS or atomic-clock provenance. Existing PTP
services retain their own metadata. If the system leap table is missing or expired,
the offset stays unknown and CLK shows **Local time: Unavailable**.

The steps are separate recipes: `interface`, `local-start`, `local-ready`,
`run`, and `local-stop`. There is no separate setup script.
If your PTP services and socket permissions are already configured, `cargo run`
opens the monitor directly.
For another PHC number or a machine without hardware PTP, see
[Linux setup](docs/linux-setup.md).

## What you should see

After acquisition, CLK opens automatically with **Clock / reference** on the
left and **Timestamp uncertainty** on the right. Use arrow keys to scroll on
narrow terminals. Look for:

- **State: LOCKED**, within **Uncertainty limit**, the configured maximum.
- **Robot time** increasing on the PTP / TAI timeline.
- **Source** showing the upstream source advertised by ptp4l, independently of **Epoch**.
- **Local time** as an ISO date to seconds in the host timezone, when **TAI − UTC** is known.
- The CopperList counter at the bottom right increasing as the tasks execute at 20 Hz.
- **Offset from reference**, positive when ahead and negative when behind.
- **Uncertainty bound**, with each contribution shown below it.
- Live estimated drift, sample age, and reference identity (the PTP grandmaster).

Press **Ctrl-C** or **q** to stop. The other tabs show the task graph, hops,
task latency, bandwidth, memory usage, and logs. Startup announces acquisition
and the log path; failures print an error in the terminal. You do not need to
decode a log to see whether it works.
Absolute accuracy in this local demo comes from your system clock.

**Uncertainty bound includes configured allowances.** The demo configures
`reference_error_ns: 1000000`, so it starts with a conservative 1 ms reference
uncertainty. Copper adds capture uncertainty, the phase offset, and an allowance
for drift since the latest sample. For example, a 46 µs phase offset and a
600 ms-old sample at the default 100 ppm drift bound give roughly a 1.1 ms bound.
**Offset from reference** shows the estimated reference-versus-RobotClock difference; it does
not measure the PHC's accuracy against an independent clock. Running locally
does not establish absolute accuracy. Choose a smaller reference bound only
when measurements justify it.

**Uncertainty limit** is the maximum uncertainty accepted by this configuration,
not a precision target. Above the demo's 20 ms limit, synchronization expires and
Copper stops before the next task iteration. Old samples can also expire sync.
Raising this limit permits more uncertainty; it does not improve clock accuracy.

## The application wiring

The PTP-specific configuration in [copperconfig.ron](copperconfig.ron) is:

```ron
resources: [(
    id: "ptp",
    provider: "cu_ptp::LinuxPtpBundle",
    config: {
        "device": "/dev/ptp0",
        "management": "/var/run/ptp4l",
        "reference_error_ns": 1000000,
        "allow_local_master": true,
    },
)],
runtime: (clock: (parent: "ptp.reference", max_error_ns: 20000000)),
```

LinuxPTP only permits the PHC identity query on its read-write management socket,
`/var/run/ptp4l`. Copper sends GET requests; `ptp4lro` rejects this query even when
PTP is synchronized. `just` sets up access automatically.

The builder creates the clock from the reference. Copper acquires it before
starting tasks and maintains synchronization while the app runs.
[main.rs](src/main.rs) just builds and runs the app. Every task uses ordinary
`ctx.now()`; the source in [tasks.rs](src/tasks.rs) stamps its measurement with:

```rust
output.set_payload(42);
output.tov = Tov::Time(ctx.now());
```

Here the measurement is generated when `process` runs. A real sensor should use
its capture timestamp when available. The timestamp belongs on `tov`, alongside
the measurement payload. ConsoleMon reads this same clock on its UI thread.

## Next: two machines

Once the local demo works, follow the
[network grandmaster setup](docs/linux-setup.md#follow-an-existing-grandmaster).
Set `allow_local_master` to false on each follower, select its PHC, and have both
machines follow the same grandmaster and domain. The task code stays the same.

## Optional: inspect recordings

The app also saves Copper logs in this example's `logs/` directory. From here:

```sh
cargo run --bin clock-sync-logreader -- logs/ptp.copper extract-copperlists
```

For startup failures or lifecycle events:

```sh
cargo run --bin clock-sync-logreader -- logs/ptp.copper fsck --dump-runtime-lifecycle
```

The example uses a 1 ms upstream error bound and a 20 ms total error budget.
Choose bounds from your measurements; they cannot improve the reference's
accuracy. Loss of reference enters bounded holdover; exceeding error or age
limits fails the next iteration before tasks run. Restarting reacquires the
reference. Clock sync is experimental and requires the Serial planner.

Replay applications and logreaders must enable `clock-sync`. Recorded replay
restores clock corrections without opening the PTP device.
