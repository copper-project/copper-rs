#!/usr/bin/env python3
"""Run OpenZL compression per columnar stream using profile mapping."""

import argparse
import json
import subprocess
from pathlib import Path

PROFILE_MAP = {
    ".u16": "le-u16",
    ".u32": "le-u32",
    ".u64": "le-u64",
    ".i16": "le-i16",
    ".i32": "le-i32",
    ".i64": "le-i64",
}

SERIAL_SUFFIXES = [
    ".u8",
    ".i8",
    ".f32",
    ".f64",
    ".bytes",
]


def choose_profile(stream_name: str) -> str:
    for suffix, profile in PROFILE_MAP.items():
        if stream_name.endswith(suffix):
            return profile
    for suffix in SERIAL_SUFFIXES:
        if stream_name.endswith(suffix):
            return "serial"
    return "serial"


def run(cmd):
    res = subprocess.run(cmd, capture_output=True, text=True)
    return res.returncode, res.stdout.strip(), res.stderr.strip()


def compress_file(path: Path, profile: str, out_path: Path, strict: bool) -> tuple[int | None, str | None]:
    if path.stat().st_size == 0:
        out_path.write_bytes(b"")
        return 0, None

    cmd = [
        "zli",
        "compress",
        "--profile",
        profile,
        str(path),
        "--output",
        str(out_path),
    ]
    if strict:
        cmd.insert(2, "--strict")

    code, out, err = run(cmd)
    if code != 0:
        return None, err or out
    return out_path.stat().st_size, None


def iter_streams(base_dir: Path, structured_dir: Path, copper_dir: Path):
    for subdir in [structured_dir, copper_dir]:
        for file_path in sorted(subdir.glob("*.bin")):
            rel = file_path.relative_to(base_dir)
            name = file_path.name
            stream_name = name[:-4] if name.endswith(".bin") else name
            yield file_path, rel, stream_name


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenZL per-stream compression evaluation.")
    parser.add_argument("--base-dir", default="/tmp/cu29-archive-bench")
    parser.add_argument("--structured-dir", default=None)
    parser.add_argument("--copper-dir", default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--result-json", default=None)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    base_dir = Path(args.base_dir)
    structured_dir = Path(args.structured_dir) if args.structured_dir else base_dir / "structured"
    copper_dir = Path(args.copper_dir) if args.copper_dir else base_dir / "copperlist"
    output_dir = Path(args.output_dir) if args.output_dir else base_dir / "openzl"
    result_json = Path(args.result_json) if args.result_json else base_dir / "openzl_results.json"

    output_dir.mkdir(parents=True, exist_ok=True)

    results = []
    errors = []

    for file_path, rel, stream_name in iter_streams(base_dir, structured_dir, copper_dir):
        profile = choose_profile(stream_name)
        out_path = output_dir / f"{rel.as_posix()}.zl"
        out_path.parent.mkdir(parents=True, exist_ok=True)

        compressed_size, error = compress_file(file_path, profile, out_path, args.strict)
        results.append({
            "stream": stream_name,
            "path": str(rel),
            "profile": profile,
            "raw_bytes": file_path.stat().st_size,
            "compressed_bytes": compressed_size,
        })
        if error:
            errors.append({
                "stream": stream_name,
                "path": str(rel),
                "profile": profile,
                "error": error,
            })

    summary = {
        "base_dir": str(base_dir),
        "output_dir": str(output_dir),
        "streams": results,
        "errors": errors,
    }
    result_json.write_text(json.dumps(summary, indent=2))

    ok = [r for r in results if r["compressed_bytes"] is not None]
    raw_total = sum(r["raw_bytes"] for r in ok)
    comp_total = sum(r["compressed_bytes"] for r in ok)

    print(f"Streams compressed: {len(ok)}")
    print(f"Raw total bytes: {raw_total}")
    print(f"Compressed total bytes: {comp_total}")
    if errors:
        print(f"Errors: {len(errors)} (see {result_json})")
    else:
        print("No errors")
    print(f"Summary: {result_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
