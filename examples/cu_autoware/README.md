# Copper vs. LaME Autoware reference-system benchmark

This example compares Copper with the LaME executor using the same synthetic callback
graph and the execution costs from Figure 4 of the RTNS 2025 LaME paper. It is a
modified Autoware *reference system*, not a complete autonomous-driving Autoware stack.
The comparison is close apples-to-apples at the workload/chain level; it is not a claim
that the runtimes have identical execution models.

## Prerequisites

- Linux on x86-64 with at least eight logical CPUs numbered `0-7`
- Rust and this repository's normal Copper build prerequisites
- `just`, Python 3, and a running Docker daemon
- A kernel that permits `SCHED_DEADLINE` in a container with `SYS_NICE`
- Internet access for the first Docker build

No host ROS installation is needed. The image uses ROS 2 Humble on Ubuntu 22.04 and pins
[the LaME source](https://github.com/rtenlab/reference-system-latency-management) at
`c17dfa091dff2ad10e369bdb34c415ddce8405f0`. The first image build can take a while
because it builds LaME's modified `rclcpp`.

The runner does not use `--privileged`, change scheduler sysctls, write host cgroups, or
change CPU frequency. Copper and LaME's deadline workers both retain the host's full CPU
affinity. This is required because Linux rejects `SCHED_DEADLINE` admission after an
ordinary Docker container narrows a task below its scheduler root domain. LaME's
controller still uses its upstream `4-7` affinity. The container preserves LaME's
upstream reservations but disables its privileged deadline-bandwidth reclaim flag. For
lower-noise publication runs you may manually isolate CPUs and select a fixed performance
governor, but those are deliberately not hidden prerequisites.

## Run the comparison

From this directory:

```sh
just copper
just lame
just compare
```

The vanilla graph remains the default. A second three-command path builds the same crate
with the `hybrid-background` feature and compares a manually optimized static schedule:

```sh
just copper-hybrid
just lame
just compare-hybrid
```

Both benchmark commands default to a 60-second measured phase. Reasonable
overrides remain command-line arguments rather than environment variables:

```sh
just copper seconds=120
just lame seconds=120
just compare
```

`just copper` builds the release binaries, calibrates the synthetic work on the current
host into `analysis/data/copper/copperconfig.ron`, records a unified Copper log under this
example's `logs/`, and extracts chain KPIs. `just lame` builds the pinned container,
calibrates LaME's identical workload classes inside it, performs LaME's fixed 15-second
profiling phase and 5-second pause, and then records the requested managed phase.

`just copper-hybrid` selects `copperconfig-hybrid.ron` through the Cargo feature. It
partitions the 36 callbacks into seven independent periodic regions. Only those regions
are scheduled across two statically balanced one-thread pools; every causal continuation
inside a region executes inline on the same worker. Critical-path callbacks run before
non-critical fan-out work. This is the example-local schedule that Copper's future
profile-guided optimizer is expected to generate automatically, and it does not modify
Copper's scheduling internals. Task-state keyframes are compiled out because workers may
be active at CopperList boundaries, while CopperList logging remains available for KPI
extraction. The KPI pass validates callback costs and exact chain sequences. LaME's
`SCHED_DEADLINE` policy remains a reported runtime difference.

Results are kept side by side:

```text
analysis/data/
├── copper/       # raw chain CSVs, process samples, generated RON, summary.json
├── copper-hybrid/ # the feature-gated, statically fused two-worker variant
├── lame/         # raw console log, raw responses, chain CSVs, process samples, summary.json
├── comparison.csv
├── comparison.md
├── comparison-hybrid.csv
└── comparison-hybrid.md
```

The comparable real-time chains are:

| Report | Copper endpoint | LaME chain | Deadline |
| --- | --- | ---: | ---: |
| RT0 | front lidar → behavior-planner input | 1 (source chain 0) | 200 ms |
| RT1 | rear lidar → point-cloud-fusion input | 0 (source chain 3) | 200 ms |
| RT2 | behavior-planner timer → vehicle DBW | 2 (source chain 1) | 100 ms |

The report includes sample count, mean, p50, p99, maximum, deadline misses, sampled mean
CPU, and peak RSS; 100% CPU means one fully occupied logical core. Keep the raw files when comparing machines: Copper runs natively while
LaME runs in a container, and the original paper evaluated LaME on NVIDIA Jetson hardware.

## Copper vs. LaME results

(on a desktop class Ryzen CPU)

| System | Chain | Deadline |    n |               Mean |                p50 |                 p99 |                 Max |   Misses |        Mean CPU |          Peak RSS |
| ------ | ----- | -------: | ---: | -----------------: | -----------------: | ------------------: | ------------------: | -------: | --------------: | ----------------: |
| LaME   | rt0   |   200 ms |  298 |         102.470 ms |         102.015 ms |          111.081 ms |          113.556 ms |        0 |          202.6% |          88.9 MiB |
| copper | rt0   |   200 ms |  300 | 51.048 ms [x2.007] | 51.041 ms [x1.999] |  51.857 ms [x2.142] |  52.623 ms [x2.158] | 0 [same] | 177.1% [x1.144] | 28.3 MiB [x3.141] |
| LaME   | rt1   |   200 ms |  299 |         102.652 ms |          96.573 ms |          165.932 ms |          173.784 ms |        0 |          202.6% |          88.9 MiB |
| copper | rt1   |   200 ms |  300 | 12.346 ms [x8.314] | 12.351 ms [x7.819] | 12.467 ms [x13.310] | 12.524 ms [x13.876] | 0 [same] | 177.1% [x1.144] | 28.3 MiB [x3.141] |
| LaME   | rt2   |   100 ms |  600 |          48.284 ms |          47.835 ms |           56.243 ms |           75.951 ms |        0 |          202.6% |          88.9 MiB |
| copper | rt2   |   100 ms |  600 | 21.359 ms [x2.261] | 21.352 ms [x2.240] |  21.666 ms [x2.596] |  22.526 ms [x3.372] | 0 [same] | 177.1% [x1.144] | 28.3 MiB [x3.141] |

Reference: [Latency Management for ROS 2: An Online Multi-Core Scheduling Approach](https://intra.engr.ucr.edu/~hyoseung/pdf/RTNS25_LaME.pdf).
