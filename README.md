<h1 style="color: #b87333;"><img src="https://github.com/copper-project/copper-rs/blob/master/doc/static/cu29.png?raw=true" width="60" /> &nbsp;&nbsp;&nbsp;&nbsp; Copper Runtime & SDK</h1>

[![copper](https://github.com/gbin/copper-project/actions/workflows/general.yml/badge.svg)](https://github.com/gbin/copper-project/actions/workflows/general.yml)
![GitHub last commit](https://img.shields.io/github/last-commit/copper-project/copper-rs)
![](https://img.shields.io/badge/Rust-1.80+-orange.svg)
[![dependency status](https://deps.rs/repo/github/copper-project/copper-rs/status.svg)](https://deps.rs/repo/github/copper-project/copper-rs)
[![Discord](https://img.shields.io/discord/1305916875741597826?logo=discord)](https://discord.gg/VkCG7Sb9Kw)
[![Book](https://img.shields.io/badge/book-Docs-blue)](https://copper-project.github.io/copper-rs-book/)
[![Documentation](https://img.shields.io/badge/docs-available-brightgreen)](https://copper-project.github.io/copper-rs)

<blockquote>
🤖&nbsp&nbsp&nbsp&nbsp
  <em style="font-size: 1.2em;">
    Copper is to robots what a game engine is to games - build, run, and replay your entire robot deterministically.
  </em>
</blockquote>

### Why Copper

<p><strong style="color: #b87333;">🦀 Rust-first</strong> – ergonomic & safe
<p><strong style="color: #b87333;">⚡ Sub-microsecond latency</strong> – zero-alloc, data-oriented runtime
<p><strong style="color: #b87333;">⏱️ Deterministic replay</strong> – every run, bit-for-bit identical
<p><strong style="color: #b87333;">🧠 Interoperable with ROS2</strong> – bridges via Zenoh opening the path for a progressive migration.
<p><strong style="color: #b87333;">🪶 Runs anywhere</strong> – from Linux servers, workstations, SBC to bare-metal MPUs
<p><strong style="color: #b87333;">📦 Built to ship</strong> – one stack from simulation to production

### Showcasing copper-rs

✈️ Flying | 🚗 Driving | 🌊 Swimming | 🚀 Spacefaring | 🤖 Humanoids

[![Copper in virtual action](https://github.com/user-attachments/assets/27ab8b08-ef17-4283-a1cf-8d4271595ef6)](https://youtu.be/weV_JYaUsmo)

### You don't have a real robot yet? Try it in our toy example right now in a sim environment!

[![Copper in virtual action](https://img.youtube.com/vi/kC6sGRZUxLE/maxresdefault.jpg)](https://youtu.be/kC6sGRZUxLE)

Here is a Copper-based robot in action in a Bevy simulation environment!
The realistic sim is created using [Bevy](https://crates.io/crates/bevy) (A Rust Game Engine)
and [Avian3d](https://crates.io/crates/avian3d) (Physics Engine in Rust).

On your mac or linux machine (x86-64 or Arm) just run ...

```bash
$ cargo install cu-rp-balancebot
$ balancebot-sim
```

... to try it locally.

The source code for this demo is available in the [examples/cu_rp_balancebot](examples/cu_rp_balancebot) directory.

## Get Started

- Start a new project from templates: [Project Templates](https://copper-project.github.io/copper-rs/Project-Templates)
- See a full task graph + runtime walkthrough: [Copper Application Overview](https://copper-project.github.io/copper-rs/Copper-Application-Overview)
- Build and deploy an application: [Build and Deploy a Copper Application](https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application)
- RON configuration reference: [Copper RON Configuration Reference](https://copper-project.github.io/copper-rs/Copper-RON-Configuration-Reference)

## Documentation

[Link to the full documentation](https://copper-project.github.io/copper-rs/)

- Runtime concepts and SDK features: [Copper Runtime Overview](https://copper-project.github.io/copper-rs/Copper-Runtime-Overview)
- Task lifecycle: [Task Lifecycle](https://copper-project.github.io/copper-rs/Task-Lifecycle)
- Modular configuration: [Modular Configuration](https://copper-project.github.io/copper-rs/Modular-Configuration)
- Task automation: [Task Automation with just](https://copper-project.github.io/copper-rs/Task-Automation-with-Just)
- Supported platforms: [Supported Platforms](https://copper-project.github.io/copper-rs/Supported-Platforms)
- Bare-metal development: [Baremetal Development](https://copper-project.github.io/copper-rs/Baremetal-Development)
- Available components: [Available Components](https://copper-project.github.io/copper-rs/Available-Components)
- FAQ: [FAQ](https://copper-project.github.io/copper-rs/FAQ)
- Release notes: [Copper Release Notes](https://copper-project.github.io/copper-rs/Copper-Release-Notes)
- Roadmap: [Roadmap](https://copper-project.github.io/copper-rs/Roadmap)

## Python Support

Copper has two very different Python stories:

- Offline Python log analysis: use `cu29-export` and app-specific PyO3 modules such as
  [examples/cu_flight_controller](examples/cu_flight_controller). This is a reasonable
  workflow because Python stays off the runtime hot path.
- Runtime Python task prototyping: use
  [components/tasks/cu_python_task](components/tasks/cu_python_task) and
  [examples/cu_python_task_demo](examples/cu_python_task_demo). This is for
  experimentation only and is strongly not recommended for production or realtime
  robots.

Putting Python inside a Copper task defeats the performance model Copper is built
for: it adds allocations, latency, jitter, and middleware overhead, and it ruins
the realtime characteristics of the stack. The intended use is to sketch one task
in Python, get the behavior right, then rewrite it in Rust.


## Citation

If you use Copper-rs in your research, please cite it as:

```bibtex
@misc{copperrs2026,
  author       = {Guillaume Binet and Copper Project contributors},
  title        = {Copper-rs: A deterministic runtime and SDK for robotics},
  year         = {2026},
  howpublished = {GitHub repository},
  url          = {https://github.com/copper-project/copper-rs},
  note         = {Version v0.13 or latest}
}
```

## Project

> [!NOTE]
> We are looking for contributors to help us build the best robotics framework possible. If you are interested, please
> join us on [Discord](https://discord.gg/VkCG7Sb9Kw) or open an issue.
