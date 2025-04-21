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

The monitor has 4 screens:

- **SysInfo**: A quick system information screen (CPU, Memory, Distrib ...)
- **DAG**: A Directed Acyclic Graph of the tasks with their real time error status and short string info.
- **Latencies**: A list of the tasks with their real time latencies & assorted statistics (Jitter, Min, Max, Avg).
- **Memory Pools**: A list of the memory pools with their real time usage and statistics (Pool ID, Used/Total, Buffer Size, Handles in Use, Handles/sec).
- **Debug Output** [`debug_pane`](#debug_pane-feature): A pane that displays debug logs in real-time.

## `debug_pane` feature

Enabled by default. Disable with `default-features = false`. Displays real-time logs
from [log](https://crates.io/crates/log), [cu29-log](https://crates.io/crates/cu29-log) and `stderr`.
