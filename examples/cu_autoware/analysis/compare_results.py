#!/usr/bin/env python3
"""Summarize either runner and produce the side-by-side CSV/Markdown report."""

import argparse
import csv
import json
import math
import platform
from pathlib import Path
import statistics

CHAINS = {
    "rt0": ("front lidar → behavior input", 200.0),
    "rt1": ("rear lidar → fusion input", 200.0),
    "rt2": ("behavior timer → vehicle DBW", 100.0),
}


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def latency_values(path):
    with path.open(newline="") as file:
        return [float(row["latency_ms"]) for row in csv.DictReader(file)]


def process_metrics(path):
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    cpu = [float(row["cpu_pct"]) for row in rows]
    rss = [float(row["rss_mb"]) for row in rows]
    return {
        "mean_cpu_pct": statistics.fmean(cpu) if cpu else None,
        "peak_rss_mb": max(rss) if rss else None,
    }


def summarize(directory, system):
    result = {"system": system, "chains": {}, "process": process_metrics(directory / "proc.csv")}
    for chain, (description, deadline) in CHAINS.items():
        filename = "hotpath.csv" if system == "copper" and chain == "rt0" else f"{chain}.csv"
        values = latency_values(directory / filename)
        result["chains"][chain] = {
            "description": description,
            "deadline_ms": deadline,
            "n": len(values),
            "mean_ms": statistics.fmean(values),
            "p50_ms": percentile(values, 0.50),
            "p99_ms": percentile(values, 0.99),
            "max_ms": max(values),
            "deadline_misses": sum(value > deadline for value in values),
        }
    result["platform"] = {"system": platform.system(), "release": platform.release(), "machine": platform.machine()}
    (directory / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def report(base):
    systems = [json.loads((base / name / "summary.json").read_text()) for name in ("copper", "lame")]
    fields = ["system", "chain", "description", "deadline_ms", "n", "mean_ms", "p50_ms", "p99_ms", "max_ms", "deadline_misses", "mean_cpu_pct", "peak_rss_mb"]
    rows = []
    for system in systems:
        for chain, metrics in system["chains"].items():
            row = {"system": system["system"], "chain": chain, **metrics}
            row.update(system["process"])
            rows.append(row)
    with (base / "comparison.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# Copper vs. LaME results",
        "",
        "| System | Chain | Deadline | n | Mean | p50 | p99 | Max | Misses | Mean CPU | Peak RSS |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['system']} | {row['chain']} | {row['deadline_ms']:.0f} ms | {row['n']} | "
            f"{row['mean_ms']:.3f} ms | {row['p50_ms']:.3f} ms | {row['p99_ms']:.3f} ms | "
            f"{row['max_ms']:.3f} ms | {row['deadline_misses']} | "
            f"{row['mean_cpu_pct']:.1f}% | {row['peak_rss_mb']:.1f} MiB |"
        )
    lines += ["", "CPU/RSS are process samples at roughly 10 Hz. LaME samples are from the managed phase after its 15 s profile and 5 s pause.", ""]
    (base / "comparison.md").write_text("\n".join(lines))
    print("\n".join(lines))


parser = argparse.ArgumentParser()
sub = parser.add_subparsers(dest="command", required=True)
one = sub.add_parser("summarize")
one.add_argument("directory", type=Path)
one.add_argument("system", choices=("copper", "lame"))
both = sub.add_parser("compare")
both.add_argument("base", type=Path)
args = parser.parse_args()
if args.command == "summarize":
    summarize(args.directory, args.system)
else:
    report(args.base)
