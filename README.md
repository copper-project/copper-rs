<h1 align="center">
  <img src="https://github.com/copper-project/copper-rs/blob/master/doc/static/cu29.png?raw=true" width="60" alt="Copper logo" />
  <br />
  Copper Runtime &amp; SDK
</h1>

<p align="center">
  A Rust runtime for building, running, recording, and deterministically replaying
  static robot task graphs, from Linux to bare metal.
</p>

<p align="center">
  <a href="https://github.com/copper-project/copper-rs/actions/workflows/general.yml"><img src="https://img.shields.io/github/check-runs/copper-project/copper-rs/master?nameFilter=CI%20Status&amp;label=CI%2FCD" alt="CI/CD status" /></a>
  <a href="https://crates.io/crates/cu29"><img src="https://img.shields.io/crates/v/cu29.svg" alt="cu29 on crates.io" /></a>
  <img src="https://img.shields.io/badge/Rust-1.95+-orange.svg" alt="Rust 1.95 or newer" />
  <a href="https://copper-project.github.io/copper-rs-book/"><img src="https://img.shields.io/badge/book-read-blue" alt="Read the Copper book" /></a>
  <a href="https://copper-project.github.io/copper-rs/"><img src="https://img.shields.io/badge/docs-read-blue" alt="Copper documentation" /></a>
  <a href="https://github.com/copper-project/copper-rs/wiki/Copper-Release-Notes"><img src="https://img.shields.io/badge/release_notes-read-blue" alt="Copper release notes" /></a>
  <a href="https://discord.gg/VkCG7Sb9Kw"><img src="https://img.shields.io/discord/1305916875741597826?logo=discord" alt="Copper Discord" /></a>
</p>

<p align="center">
  <strong><a href="https://cdn.copper-robotics.com/demo/balancebot/index.html">▶ Try Copper in your browser</a></strong>
  &nbsp;·&nbsp;
  <strong><a href="#build-your-first-copper-app">Build your first app</a></strong>
</p>

## Why Copper

| Built for robots | Built to ship |
| --- | --- |
| **Static by design:** task graphs are declared in RON and wired at compile time. | **Runs anywhere:** Linux, macOS, SBCs, and bare-metal microcontrollers. |
| **Realtime first:** zero-allocation, data-oriented execution on the hot path. | **Deterministic replay:** record a run, reproduce it, and inspect it offline. |
| **Rust-first:** ergonomic task APIs with compile-time guarantees. | **Interoperable:** connect to ROS 2 through Zenoh and migrate progressively. |

Already flying, driving, swimming, spacefaring, and powering humanoids.

## See It Run

These are the same Copper applications used on physical robots, recompiled for the
browser. The simulator runs beside a live view of the Copper task graph and latency.

<table>
  <tr>
    <td width="50%" valign="top">
      <a href="https://cdn.copper-robotics.com/demo/balancebot/index.html">
        <img src="https://raw.githubusercontent.com/copper-project/copper-rs/master/doc/static/demo-balancebot-browser.png" alt="BalanceBot browser demo" width="100%" />
      </a>
      <br />
      <strong><a href="https://cdn.copper-robotics.com/demo/balancebot/index.html">BalanceBot</a></strong>
      <br />
      A self-balancing robot simulation using the application that runs on Raspberry Pi hardware.
      <br />
      <a href="https://github.com/copper-project/extra-examples/tree/master/examples/cu_rp_balancebot">Source code</a>
    </td>
    <td width="50%" valign="top">
      <a href="https://cdn.copper-robotics.com/demo/flight-controller/index.html">
        <img src="https://raw.githubusercontent.com/copper-project/copper-rs/master/doc/static/demo-flight-controller-browser.png" alt="Flight controller browser demo" width="100%" />
      </a>
      <br />
      <strong><a href="https://cdn.copper-robotics.com/demo/flight-controller/index.html">Flight Controller</a></strong>
      <br />
      A quadcopter simulation using the control stack deployed on STM32H7 flight hardware.
      <br />
      <a href="https://github.com/copper-project/extra-examples/tree/master/examples/cu_flight_controller">Source code</a>
    </td>
  </tr>
</table>

Watch more robots built with Copper in the
[community showcase](https://youtu.be/weV_JYaUsmo), or explore the
[cross-framework benchmarks](https://github.com/copper-project/copper-rs/tree/master/benchmarks).

## Build Your First Copper App

Install the latest stable Rust toolchain, then:

```bash
cargo install cargo-cunew
cargo cunew hello_copper
cd hello_copper
cargo run
```

The generated app has a typed `source → task → sink` graph that prints its
first messages and records `logs/hello-copper.copper`. Start with
`copperconfig.ron`, `src/main.rs`, and `src/tasks.rs`; the generated `justfile`
also provides helpers for logs, CopperLists, topology (`just graph`), the exact
generated process schedule (`just sched`), post-execution timing from the default
Copper log (`just sched-log`), and replay. Host projects also include
`just pgs-baseline`, `just pgs-optimize`, `just pgs-candidate`, and
`just pgs-measure`; the starter `schedule.ron` is a conservative placeholder to tune
for the real workload. Project creation asks whether the target is bare metal/no_std
and omits PGS scaffolding for that choice.

## How Copper Fits Together

| Stage | What happens |
| --- | --- |
| **Declare** | Define the static task graph in `copperconfig.ron`. |
| **Generate** | `#[copper_runtime]` wires the graph at compile time. |
| **Run** | Execute deterministically with a zero-allocation hot path. |
| **Record** | Capture messages, timing, and state in a unified `.copper` log. |
| **Inspect** | Replay a recorded run, export its data, and inspect its state. |

The robot is a static thing: Copper turns its declared graph into a purpose-built
runtime, then records messages, timing, and state into one replayable log.
`RobotClock` makes time explicit across hardware, simulation, and replay.

## What Sets Copper Apart

- <img src="doc/static/version-0.13.svg" alt="v0.13" align="absmiddle" />&nbsp;&nbsp;&nbsp;**[Time Travel Debugging](https://copper-project.github.io/copper-rs-book/logging-replay.html#remote-debug-and-time-traveler)**<br>
  Seek through a recording, restore task state, and re-execute your code with recorded inputs and robot time.

- <img src="doc/static/version-1.0.svg" alt="v1.0" align="absmiddle" />&nbsp;&nbsp;&nbsp;**[Typed Lossless Codecs](https://copper-project.github.io/copper-rs-book/logging-replay.html#typed-logging-codecs)**<br>
  Choose lossless compression per payload type in the task graph; Copper handles encoding and decoding for recording and replay.

- <img src="doc/static/version-1.1.svg" alt="v1.1" align="absmiddle" />&nbsp;&nbsp;&nbsp;**[Anytime Computation](examples/cu_anytime_rrt_star)**<br>
  Produce a base result, then refine it within time or quality budgets, with refinement steps in the deterministic execution plan.

- <img src="doc/static/version-1.2.svg" alt="v1.2" align="absmiddle" />&nbsp;&nbsp;&nbsp;**[Live Twinning Telemetry In Challenging Environments](https://copper-project.github.io/copper-rs-book/logstream-telemetry.html)**<br>
  Run selected robot tasks on the ground to reconstruct outputs you never transmitted, while keeping a replayable archive.

- <img src="doc/static/version-1.3.svg" alt="v1.3" align="absmiddle" />&nbsp;&nbsp;&nbsp;**[Profile Guided Scheduling (PGS)](https://copper-project.github.io/copper-rs-book/profile-guided-scheduling.html)**<br>
  Profile-Guided Scheduling turns measured timings, sensor rates, and deadlines into candidate schedules. Measure, select, and compile one into your runtime.

- <img src="doc/static/version-1.3.svg" alt="v1.3" align="absmiddle" />&nbsp;&nbsp;&nbsp;**[First Class Clock Sync With Replay](https://copper-project.github.io/copper-rs-book/clock-sync.html)**<br>
  Synchronize `RobotClock` to PTP or custom references, track uncertainty, and replay clock corrections.

## Explore Copper

<table>
  <tr>
    <td><strong>Learn</strong></td>
    <td><a href="https://copper-project.github.io/copper-rs-book/">Book</a> · <a href="https://copper-project.github.io/copper-rs/Copper-Runtime-Overview">Runtime overview</a> · <a href="https://docs.rs/cu29">API docs</a></td>
  </tr>
  <tr>
    <td><strong>Build</strong></td>
    <td><a href="https://copper-project.github.io/copper-rs/Project-Templates">Project templates</a> · <a href="https://copper-project.github.io/copper-rs/Copper-RON-Configuration-Reference">RON reference</a> · <a href="https://cdn.copper-robotics.com/catalog/index.html">Component catalog</a></td>
  </tr>
  <tr>
    <td><strong>Go deeper</strong></td>
    <td><a href="https://github.com/copper-project/copper-rs/tree/master/examples">Examples</a> · <a href="https://copper-project.github.io/copper-rs-book/logging-replay.html">Logging and replay</a> · <a href="https://copper-project.github.io/copper-rs/Python-Support">Python support</a></td>
  </tr>
  <tr>
    <td><strong>Project</strong></td>
    <td><a href="https://copper-project.github.io/copper-rs/Supported-Platforms">Supported platforms</a> · <a href="https://copper-project.github.io/copper-rs/Copper-Release-Notes">Release notes</a></td>
  </tr>
  <tr>
    <td><strong>Community</strong></td>
    <td><a href="https://github.com/copper-project/copper-rs/blob/master/CONTRIBUTING.md">Contributing</a> · <a href="https://github.com/copper-project/copper-rs/discussions">GitHub Discussions</a> · <a href="https://discord.gg/VkCG7Sb9Kw">Discord</a></td>
  </tr>
</table>
