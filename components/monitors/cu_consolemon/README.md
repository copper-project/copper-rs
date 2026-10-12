# ConsoleMon

<img src="doc/tasks.png" alt="Tasks"></img>

Note: This is part of the Copper project. See the main project page for context.

The ConsoleMon is a simple monitor that display the current state of the robot to the console in an TUI.
It is useful for monitoring a robot quickly over ssh or in a terminal.

## Usage

Add it as a dependency in your `Cargo.toml`:

```tom
[dependencies]
cu-consolemon = "*"
```

And in you copperconfig.ron:

```ron
(
    tasks: [
        ( ...
        ),
     ],
    cnx: [
        ( ... ),
    ],

    monitor: (
        type: "cu_consolemon::CuConsoleMon", // <== Here
    )
)

```

The monitor offers these screens (some are feature-dependent):

- **SysInfo**: A quick system information screen (CPU, Memory, Distrib ...)
- **DAG**: A Directed Acyclic Graph of the tasks with their real time error status and short string info.
- **HOP**: Explore a task and its neighbors with live status and timing.
- **CLK**: Live robot time and origin, available for local and synchronized clocks.
- **Latencies**: A list of the tasks with their real time latencies & assorted statistics (Jitter, Min, Max, Avg).
- **Bandwidth**: CopperList serialization and IO statistics.
- **Memory Pools**: A list of the memory pools with their real time usage and statistics (Pool ID, Used/Total, Buffer Size, Handles in Use, Handles/sec).
- **Debug Output** [`debug_pane`](#debug_pane-feature): The shared `cu_tuimon` log pane showing Copper live logs and captured `stderr`.

## `debug_pane` feature

Enabled by default. Disable with `default-features = false`. Displays real-time logs
from Copper structured logging (`debug!`, `info!`, `warn!`, `error!`) and captured `stderr`.

## CLK panel

CLK always shows the application's shared `RobotClock`, its live time, and its
origin. Ordinary clocks show LOCAL with a local monotonic or mock counter. The
optional `clock-sync` feature enables clock discipline; synchronized clocks open
CLK initially. It has two bordered panes side by side: **Clock / reference**
for live time and alignment, and **Timestamp uncertainty** for bounds and limits.
Use arrow keys to scroll on narrow terminals.

| Metric | Meaning |
| --- | --- |
| State | LOCAL uses a local counter; ACQUIRING awaits synchronization; LOCKED follows the reference; HOLDOVER continues without the reference; EXPIRED requires reacquisition. |
| Robot time | Current shared `RobotClock` time in ns. Task timestamps from `ctx.now()` use this same clock. |
| Origin | Local monotonic counter, mock counter, or external reference. |
| Offset from reference | Signed robot time minus reference time in µs: positive is ahead, negative is behind. |
| Estimated drift | Estimated clock rate difference in ppm. |
| Sample age | Elapsed time since the latest reference sample in ms. |
| Reference identity | Reference provider's clock identifier. |
| Domain / session | Reference domain and session identifiers; the provider determines the time scale. |
| Uncertainty bound | Conservative total timestamp uncertainty in ms: reference uncertainty + aging uncertainty + offset magnitude. Includes configured allowances; it is not measured total clock error. |
| Reference uncertainty | Reported or configured reference uncertainty plus capture uncertainty in ms. |
| Aging uncertainty | Uncertainty accumulated since the latest sample in µs, using the drift bound. |
| Offset magnitude | Absolute offset from the reference in µs. |
| Drift bound | Rate uncertainty allowance in ppm used to calculate aging uncertainty. |
| Uncertainty limit | Configured maximum uncertainty in ms. Exceeding it expires LOCKED or HOLDOVER; sample-age limits can also expire synchronization. This is not a precision target. |

Unavailable indicates that no usable uncertainty estimate exists or that the clock
has no external synchronization. Clock sampling and formatting happen on the UI
thread; no per-iteration clock telemetry is added to the task graph.

Try the [standalone PTP demo](../../../examples/cu_clock_sync/README.md).
