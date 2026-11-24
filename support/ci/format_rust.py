#!/usr/bin/env python3

"""Format the Copper workspace with cargo fmt."""

import argparse
import json
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", help="Rust toolchain for cargo fmt")
    parser.add_argument("--check", action="store_true", help="Check without editing files")
    args = parser.parse_args()

    cargo = ["cargo"]
    if args.toolchain:
        cargo.append(f"+{args.toolchain}")
    fmt_args = ["--", "--check"] if args.check else []

    if sys.platform != "win32":
        return subprocess.call([*cargo, "fmt", "--all", *fmt_args])

    # Stable cargo fmt passes all target paths to rustfmt at once, which can
    # exceed the Windows process command-line limit. See rust-lang/rustfmt#6934.
    metadata = subprocess.run(
        [*cargo, "metadata", "--format-version", "1", "--no-deps"],
        capture_output=True,
        encoding="utf-8",
    )
    if metadata.returncode:
        sys.stderr.write(metadata.stderr)
        return metadata.returncode

    for package in json.loads(metadata.stdout)["packages"]:
        status = subprocess.call([*cargo, "fmt", "-p", package["name"], *fmt_args])
        if status:
            return status
    return 0


if __name__ == "__main__":
    sys.exit(main())
