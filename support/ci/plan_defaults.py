#!/usr/bin/env python3

"""Discover the local Copper log and typed logreader used by `just plan-log`."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


def fail(message: str) -> None:
    raise SystemExit(f"plan-log: {message}")


def copper_log_bases(directory: Path) -> list[Path]:
    logs = directory / "logs"
    bases: set[Path] = set()
    for path in logs.glob("*.copper"):
        name = re.sub(r"_\d+\.copper$", ".copper", path.name)
        bases.add(logs / name)
    return sorted(bases)


def choose_log(candidates: list[Path]) -> Path:
    if not candidates:
        fail("no logs/*.copper slabs found; run the example first")
    preferred = [
        path
        for path in candidates
        if "resim" not in path.stem and "replay" not in path.stem
    ]
    if preferred:
        candidates = preferred
    non_compute = [path for path in candidates if "compute" not in path.stem]
    if non_compute:
        candidates = non_compute
    if len(candidates) != 1:
        names = ", ".join(path.name for path in candidates)
        fail(f"multiple plausible logs ({names}); use `just plan log=... bin=...`")
    return candidates[0]


def choose_logreader(targets: list[dict[str, Any]], log: Path) -> dict[str, Any]:
    candidates = [
        target
        for target in targets
        if "bin" in target.get("kind", []) and "logreader" in target.get("name", "")
    ]
    if not candidates:
        fail("this example has no typed logreader binary")
    wants_compute = "compute" in log.stem
    matched = [
        target
        for target in candidates
        if ("compute" in target["name"]) == wants_compute
    ]
    if matched:
        candidates = matched
    if len(candidates) != 1:
        names = ", ".join(target["name"] for target in candidates)
        fail(f"multiple plausible logreaders ({names}); use `just plan log=... bin=...`")
    return candidates[0]


def main() -> None:
    if len(sys.argv) != 2:
        fail("expected the example directory")
    directory = Path(sys.argv[1]).resolve()
    metadata = json.loads(
        subprocess.check_output(
            ["cargo", "metadata", "--format-version", "1", "--no-deps"],
            cwd=directory,
            text=True,
            encoding="utf-8",
        )
    )
    packages = [
        package
        for package in metadata.get("packages", [])
        if Path(package["manifest_path"]).resolve().parent == directory
    ]
    if len(packages) != 1:
        fail(f"could not identify one Cargo package in {directory}")
    log = choose_log(copper_log_bases(directory))
    target = choose_logreader(packages[0].get("targets", []), log)
    required_features = target.get("required-features", [])
    print(target["name"])
    print(",".join(required_features))
    print(log)


if __name__ == "__main__":
    main()
