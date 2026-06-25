#!/usr/bin/env python3

"""Print cargo --exclude flags for packages omitted from default workspace CI."""

from __future__ import annotations

import argparse
import json
import subprocess
from typing import Any, Dict, List, Optional


def _normalize_list(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, list):
        return [item for item in value if isinstance(item, str)]
    return []


def _cargo_command(toolchain: Optional[str]) -> List[str]:
    if toolchain:
        return ["rustup", "run", toolchain, "cargo"]
    return ["cargo"]


def _load_excluded_packages(toolchain: Optional[str]) -> List[Dict[str, Any]]:
    metadata_json = subprocess.check_output(
        _cargo_command(toolchain) + ["metadata", "--format-version", "1", "--no-deps"],
        text=True,
        encoding="utf-8",
    )
    metadata = json.loads(metadata_json)

    excluded: List[Dict[str, Any]] = []
    for pkg in metadata.get("packages", []):
        pkg_metadata = pkg.get("metadata") or {}
        copper_meta = pkg_metadata.get("copper", {})
        environments = set(_normalize_list(copper_meta.get("environments")))
        embedded_only = "embedded" in environments and "host" not in environments
        if embedded_only or bool(copper_meta.get("ci_exclude_workspace", False)):
            excluded.append(
                {
                    "name": pkg["name"],
                    "manifest_path": pkg["manifest_path"],
                }
            )

    excluded.sort(key=lambda pkg: pkg["name"])
    return excluded


def _print_excludes(packages: List[Dict[str, Any]]) -> None:
    print(" ".join(f"--exclude {pkg['name']}" for pkg in packages))


def _print_names(packages: List[Dict[str, Any]]) -> None:
    for pkg in packages:
        print(pkg["name"])


def _add_toolchain_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--toolchain",
        default="default",
        help="Rust toolchain channel to use with cargo metadata.",
    )


def _resolve_toolchain(requested: str) -> Optional[str]:
    if requested == "default":
        return None
    return requested


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Utilities for default workspace CI exclusions."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    excludes_parser = subparsers.add_parser(
        "excludes", help="Print cargo --exclude flags."
    )
    _add_toolchain_argument(excludes_parser)

    list_parser = subparsers.add_parser("list", help="Print package names.")
    _add_toolchain_argument(list_parser)

    args = parser.parse_args()
    packages = _load_excluded_packages(_resolve_toolchain(args.toolchain))

    if args.command == "excludes":
        _print_excludes(packages)
    elif args.command == "list":
        _print_names(packages)
    else:
        parser.error(f"unsupported command: {args.command}")


if __name__ == "__main__":
    main()
