#!/usr/bin/env python3
"""Figures from the CSVs `kpi` writes. Usage: plots.py [data dir] [figs dir]."""

import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "data")
FIGS = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "figs")


def column(name, *fields):
    """Columns of a CSV, or None when the run did not produce it."""
    path = os.path.join(DATA, name)
    if not os.path.exists(path):
        print(f"skipping {name}: not found")
        return None
    with open(path) as handle:
        rows = [r for r in csv.DictReader(handle) if all(r.get(f) for f in fields)]
    parsed = []
    for row in rows:
        try:
            parsed.append([float(row[field]) for field in fields])
        except ValueError:
            continue
    if not parsed:
        print(f"skipping {name}: empty")
        return None
    return [list(col) for col in zip(*parsed)]


def figure(name, plot, xlabel, ylabel, title):
    fig, axes = plt.subplots(figsize=(6, 3.2))
    plot(axes)
    axes.set_xlabel(xlabel)
    axes.set_ylabel(ylabel)
    axes.set_title(title)
    axes.grid(True, linewidth=0.3, alpha=0.5)
    fig.tight_layout()
    path = os.path.join(FIGS, name)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"wrote {path}")


def chain_box():
    chains = [
        ("hot path", column("hotpath.csv", "latency_ms")),
        ("RT1", column("rt1.csv", "latency_ms")),
        ("RT2", column("rt2.csv", "latency_ms")),
    ]
    chains = [(label, series[0]) for label, series in chains if series]
    if not chains:
        return
    figure(
        "latency_box.png",
        lambda axes: axes.boxplot(
            [values for _, values in chains],
            labels=[label for label, _ in chains],
            showfliers=True,
        ),
        "",
        "latency (ms)",
        "Chain latency",
    )


def hotpath_series():
    series = column("hotpath.csv", "t_ms", "latency_ms")
    if not series:
        return
    t, latency = series
    figure(
        "hotpath_series.png",
        lambda axes: axes.plot([x / 1000 for x in t], latency, linewidth=0.8),
        "time (s)",
        "latency (ms)",
        "Hot-path latency",
    )


def planner_period():
    series = column("bp_period.csv", "t_ms", "period_ms")
    if not series:
        return
    t, period = series

    def plot(axes):
        axes.plot([x / 1000 for x in t], period, linewidth=0.8)
        axes.axhline(100.0, color="black", linewidth=0.8, linestyle="--")

    figure(
        "planner_period.png",
        plot,
        "time (s)",
        "period (ms)",
        "behavior_planner period vs 100ms",
    )


def proc_series():
    series = column("proc.csv", "t_s", "cpu_pct", "rss_mb")
    if not series:
        return
    t, cpu, rss = series
    # proc.csv comes from the sampler, the rest from kpi: catch a mixed pair of runs.
    hotpath = column("hotpath.csv", "t_ms")
    if hotpath and hotpath[0]:
        kpi_span = hotpath[0][-1] / 1000
        # The kpi span legitimately trails the process span by startup plus up to one
        # 200ms sensor period, so short runs need an absolute floor on the tolerance.
        tolerance = max(0.05 * max(t[-1], kpi_span), 0.5)
        if kpi_span and abs(t[-1] - kpi_span) > tolerance:
            print(
                f"warning: proc.csv spans {t[-1]:.1f}s but hotpath.csv spans "
                f"{kpi_span:.1f}s - CSVs look like different runs"
            )
    figure(
        "cpu.png",
        lambda axes: axes.plot(t, cpu, linewidth=0.8),
        "time (s)",
        "CPU (%)",
        "Process CPU",
    )
    figure(
        "rss.png",
        lambda axes: axes.plot(t, rss, linewidth=0.8),
        "time (s)",
        "RSS (MB)",
        "Process memory",
    )


if __name__ == "__main__":
    os.makedirs(FIGS, exist_ok=True)
    chain_box()
    hotpath_series()
    planner_period()
    proc_series()
