#!/usr/bin/env python3
"""Extract managed-phase LaME response-time samples from its console output."""

import csv
from pathlib import Path
import re
import sys

raw = Path(sys.argv[1])
out = Path(sys.argv[2])
lines = raw.read_text(errors="replace").splitlines()
deadline_errors = [line for line in lines if "Error setting deadline" in line]
if deadline_errors:
    raise SystemExit(f"{raw}: LaME failed to enter SCHED_DEADLINE ({deadline_errors[0]})")
markers = [i for i, line in enumerate(lines) if "Executor and all timers start." in line]
if len(markers) < 2:
    raise SystemExit(f"{raw}: expected two executor-start markers, found {len(markers)}")

pattern = re.compile(r"^RT Chain: (\d+) prio: \d+ Instance: (\d+) Response Time: (\d+)")
# The controller reindexes the three source chains in managed-priority order.
mapping = {1: ("rt0", 200.0), 2: ("rt1", 200.0), 0: ("rt2", 100.0)}
rows = {name: [] for name, _ in mapping.values()}
all_rows = []
for line in lines[markers[1] + 1 :]:
    match = pattern.search(line)
    if not match:
        continue
    chain, instance, latency_us = map(int, match.groups())
    all_rows.append((chain, instance, latency_us / 1000.0))
    if chain in mapping:
        rows[mapping[chain][0]].append((instance, latency_us / 1000.0))

if not all(rows.values()):
    missing = [name for name, values in rows.items() if not values]
    raise SystemExit(f"{raw}: no managed-phase samples for {', '.join(missing)}")

out.mkdir(parents=True, exist_ok=True)
with (out / "responses.csv").open("w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["chain_id", "instance", "latency_ms"])
    writer.writerows(all_rows)
for name, values in rows.items():
    with (out / f"{name}.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["seq", "latency_ms"])
        writer.writerows(values)
