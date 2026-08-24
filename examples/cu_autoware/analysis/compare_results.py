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


def markdown_table(headers, rows, right_aligned):
    widths = []
    for column, header in enumerate(headers):
        minimum = 4 if column in right_aligned else 3
        widths.append(max(minimum, len(header), *(len(row[column]) for row in rows)))

    def render(row):
        cells = []
        for column, value in enumerate(row):
            align = str.rjust if column in right_aligned else str.ljust
            cells.append(align(value, widths[column]))
        return f"| {' | '.join(cells)} |"

    separators = [
        "-" * (width - 1) + ":" if column in right_aligned else "-" * width
        for column, width in enumerate(widths)
    ]
    return [render(headers), render(separators), *(render(row) for row in rows)]


def comparison_marker(value, reference):
    if value == reference:
        return "[same]"
    if value == 0:
        return "[xinf]"
    return f"[x{reference / value:.3f}]"


def summarize(directory, system):
    result = {"system": system, "chains": {}, "process": process_metrics(directory / "proc.csv")}
    for chain, (description, deadline) in CHAINS.items():
        filename = "hotpath.csv" if system.startswith("copper") and chain == "rt0" else f"{chain}.csv"
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


def report(base, copper_dir, output):
    challenger = json.loads((base / copper_dir / "summary.json").read_text())
    reference = json.loads((base / "lame" / "summary.json").read_text())
    fields = ["system", "chain", "description", "deadline_ms", "n", "mean_ms", "p50_ms", "p99_ms", "max_ms", "deadline_misses", "mean_cpu_pct", "peak_rss_mb"]
    rows = []
    for chain in CHAINS:
        for system in (reference, challenger):
            metrics = system["chains"][chain]
            name = "copper" if system["system"].startswith("copper") else system["system"]
            row = {"system": name, "chain": chain, **metrics}
            row.update(system["process"])
            rows.append(row)
    with (base / f"{output}.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    headers = ("System", "Chain", "Deadline", "n", "Mean", "p50", "p99", "Max", "Misses", "Mean CPU", "Peak RSS")
    table_rows = []
    for row in rows:
        reference_chain = reference["chains"][row["chain"]]
        is_challenger = row["system"] == "copper"

        def compared(value, reference_value, formatted):
            if not is_challenger:
                return formatted
            return f"{formatted} {comparison_marker(value, reference_value)}"

        table_rows.append(
            (
                row["system"],
                row["chain"],
                f"{row['deadline_ms']:.0f} ms",
                str(row["n"]),
                compared(row["mean_ms"], reference_chain["mean_ms"], f"{row['mean_ms']:.3f} ms"),
                compared(row["p50_ms"], reference_chain["p50_ms"], f"{row['p50_ms']:.3f} ms"),
                compared(row["p99_ms"], reference_chain["p99_ms"], f"{row['p99_ms']:.3f} ms"),
                compared(row["max_ms"], reference_chain["max_ms"], f"{row['max_ms']:.3f} ms"),
                compared(row["deadline_misses"], reference_chain["deadline_misses"], str(row["deadline_misses"])),
                compared(row["mean_cpu_pct"], reference["process"]["mean_cpu_pct"], f"{row['mean_cpu_pct']:.1f}%"),
                compared(row["peak_rss_mb"], reference["process"]["peak_rss_mb"], f"{row['peak_rss_mb']:.1f} MiB"),
            )
        )
    lines = ["# Copper vs. LaME results", "", *markdown_table(headers, table_rows, set(range(2, len(headers))))]
    lines += [
        "",
        "LaME is the reference row for each chain. Bracketed factors on Copper are LaME divided by Copper, so values above x1.000 are improvements and values below x1.000 are regressions; `[same]` means both values are equal.",
        "",
        "CPU/RSS are process samples at roughly 10 Hz; 100% CPU is one fully occupied logical core. LaME samples are from the managed phase after its 15 s profile and 5 s pause.",
        "",
    ]
    (base / f"{output}.md").write_text("\n".join(lines))
    print("\n".join(lines))


parser = argparse.ArgumentParser()
sub = parser.add_subparsers(dest="command", required=True)
one = sub.add_parser("summarize")
one.add_argument("directory", type=Path)
one.add_argument("system", choices=("copper", "copper-hybrid", "lame"))
both = sub.add_parser("compare")
both.add_argument("base", type=Path)
both.add_argument("--copper-dir", default="copper")
both.add_argument("--output", default="comparison")
args = parser.parse_args()
if args.command == "summarize":
    summarize(args.directory, args.system)
else:
    report(args.base, args.copper_dir, args.output)
