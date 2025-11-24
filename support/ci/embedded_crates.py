#!/usr/bin/env python3

"""Helpers for working with embedded-only workspace members."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from typing import Any, Dict, List, Optional


def _load_embedded_packages() -> List[Dict[str, Any]]:
    metadata_json = subprocess.check_output(
        ["cargo", "metadata", "--format-version", "1", "--no-deps"],
        text=True,
        encoding="utf-8",
    )
    metadata = json.loads(metadata_json)

    embedded: List[Dict[str, Any]] = []
    for pkg in metadata.get("packages", []):
        pkg_metadata = pkg.get("metadata") or {}
        copper_meta = pkg_metadata.get("copper", {})
        if not copper_meta.get("embedded_only"):
            continue

        features = copper_meta.get("embedded_features") or []
        if isinstance(features, str):
            features = [features]

        embedded.append(
            {
                "name": pkg["name"],
                "manifest_path": pkg["manifest_path"],
                "target": copper_meta.get("embedded_target"),
                "config": copper_meta.get("embedded_config"),
                "no_default_features": bool(
                    copper_meta.get("embedded_no_default_features", False)
                ),
                "features": features,
            }
        )

    embedded.sort(key=lambda pkg: pkg["name"])
    return embedded


def _print_excludes(packages: List[Dict[str, Any]]) -> None:
    excludes = " ".join(f"--exclude {pkg['name']}" for pkg in packages)
    print(excludes)


def _print_names(packages: List[Dict[str, Any]]) -> None:
    for pkg in packages:
        print(pkg["name"])


def _print_json(packages: List[Dict[str, Any]]) -> None:
    json.dump(packages, sys.stdout)


def _run_action(
    packages: List[Dict[str, Any]], action: str, toolchain: Optional[str]
) -> None:
    for pkg in packages:
        cmd = ["cargo"]
        if toolchain:
            cmd.append(f"+{toolchain}")
        cmd.extend([action, "-p", pkg["name"]])
        if pkg["no_default_features"]:
            cmd.append("--no-default-features")
        if pkg["features"]:
            cmd.extend(["--features", " ".join(pkg["features"])])
        if pkg["target"]:
            cmd.extend(["--target", pkg["target"]])
        if pkg["config"]:
            cmd.extend(["--config", pkg["config"]])
        if action == "clippy":
            cmd.extend(["--", "--deny", "warnings"])

        print(f"Running: {shlex.join(cmd)}", flush=True)
        subprocess.run(cmd, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Utilities for embedded-only workspace members."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="Print newline-separated embedded package names.")
    subparsers.add_parser("json", help="Print embedded package metadata as JSON.")
    subparsers.add_parser(
        "excludes", help="Print cargo --exclude flags for embedded packages."
    )

    run_parser = subparsers.add_parser(
        "run", help="Execute cargo for each embedded package."
    )
    run_parser.add_argument(
        "--action",
        choices=("clippy", "build"),
        default="clippy",
        help="Cargo subcommand to run (default: clippy).",
    )
    run_parser.add_argument(
        "--toolchain",
        default="stable",
        help=(
            "Rust toolchain channel to use with cargo commands (default: stable). "
            "Set to 'default' to rely on the runner's default toolchain."
        ),
    )

    args = parser.parse_args()
    packages = _load_embedded_packages()

    if args.command == "list":
        _print_names(packages)
    elif args.command == "json":
        _print_json(packages)
    elif args.command == "excludes":
        _print_excludes(packages)
    elif args.command == "run":
        toolchain = None if args.toolchain == "default" else args.toolchain
        _run_action(packages, args.action, toolchain)
    else:
        parser.error(f"unsupported command: {args.command}")


if __name__ == "__main__":
    main()
