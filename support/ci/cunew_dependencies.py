#!/usr/bin/env python3
"""Keep the project bootstrap tool small enough for a cold cargo install."""

import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    args = parser.parse_args()
    host = subprocess.check_output(
        ["rustc", f"+{args.toolchain}", "-vV"], text=True
    ).split("host: ")[1].splitlines()[0]
    tree = subprocess.check_output(
        [
            "cargo", f"+{args.toolchain}", "tree", "-p", "cargo-cunew",
            "--target", host, "--edges", "normal,build", "--prefix", "none",
            "--format", "{p}",
        ], text=True
    )
    packages = {line.removesuffix(" (*)") for line in tree.splitlines()}
    forbidden = {"cargo-generate", "rhai", "git2", "libgit2-sys", "openssl-sys", "reqwest", "tokio"}
    found = sorted({line.split()[0] for line in packages} & forbidden)
    print(f"cargo-cunew builds {len(packages)} packages on {host} (budget: 100)")
    if found or len(packages) > 100:
        raise SystemExit(f"cargo-cunew dependency budget exceeded; forbidden packages: {found}")


if __name__ == "__main__":
    main()
