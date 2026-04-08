#!/usr/bin/env python3

"""Helpers for working with embedded-only workspace members."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from typing import Any, Dict, List, Optional


def _normalize_list(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, list):
        return [item for item in value if isinstance(item, str)]
    return []


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
        environments = set(_normalize_list(copper_meta.get("environments")))
        if "embedded" not in environments or "host" in environments:
            continue

        features = _normalize_list(copper_meta.get("embedded_features"))
        compile_targets = _normalize_list(copper_meta.get("embedded_targets"))

        embedded.append(
            {
                "name": pkg["name"],
                "manifest_path": pkg["manifest_path"],
                "compile_targets": compile_targets,
                "config": copper_meta.get("embedded_config"),
                "build_release": bool(copper_meta.get("embedded_build_release", False)),
                "no_default_features": bool(
                    copper_meta.get("embedded_no_default_features", False)
                ),
                "features": features,
                "targets": [
                    {
                        "kind": target.get("kind", []),
                        "name": target["name"],
                        "required_features": target.get("required-features", []),
                    }
                    for target in pkg.get("targets", [])
                ],
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
        compile_targets = pkg["compile_targets"] or [None]
        for compile_target in compile_targets:
            cmd = ["cargo"]
            if toolchain:
                cmd.append(f"+{toolchain}")
            cmd.extend([action, "-p", pkg["name"]])
            if pkg["no_default_features"]:
                cmd.append("--no-default-features")
            if pkg["features"]:
                cmd.extend(["--features", " ".join(pkg["features"])])
            if compile_target:
                cmd.extend(["--target", compile_target])
            if pkg["config"]:
                cmd.extend(["--config", pkg["config"]])
            if action == "build":
                enabled_features = set(pkg["features"])
                eligible_bins = [
                    target["name"]
                    for target in pkg.get("targets", [])
                    if "bin" in target.get("kind", [])
                    and set(target.get("required_features", [])).issubset(enabled_features)
                ]
                for bin_name in eligible_bins:
                    cmd.extend(["--bin", bin_name])
            # Some embedded crates are expected to be linked with optimizations to fit
            # target memory; allow opting into release builds via package metadata.
            if action == "build" and pkg.get("build_release"):
                cmd.append("--release")
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
