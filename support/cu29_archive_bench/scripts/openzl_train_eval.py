#!/usr/bin/env python3
"""Train OpenZL per stream and compress with the trained compressor."""

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


def compress_with_profile(path: Path, profile: str, out_path: Path) -> tuple[int | None, str | None]:
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
    parser = argparse.ArgumentParser(description="OpenZL per-stream training evaluation.")
    parser.add_argument("--base-dir", default="/tmp/cu29-archive-bench")
    parser.add_argument("--structured-dir", default=None)
    parser.add_argument("--copper-dir", default=None)
    parser.add_argument("--train-dir", default=None)
    parser.add_argument("--compress-dir", default=None)
    parser.add_argument("--result-json", default=None)
    parser.add_argument("--max-time-secs", type=int, default=None)
    args = parser.parse_args()

    base_dir = Path(args.base_dir)
    structured_dir = Path(args.structured_dir) if args.structured_dir else base_dir / "structured"
    copper_dir = Path(args.copper_dir) if args.copper_dir else base_dir / "copperlist"
    train_dir = Path(args.train_dir) if args.train_dir else base_dir / "openzl_trained"
    compress_dir = Path(args.compress_dir) if args.compress_dir else base_dir / "openzl_trained_compressed"
    result_json = Path(args.result_json) if args.result_json else base_dir / "openzl_trained_results.json"

    train_dir.mkdir(parents=True, exist_ok=True)
    compress_dir.mkdir(parents=True, exist_ok=True)

    results = []
    errors = []

    for file_path, rel, stream_name in iter_streams(base_dir, structured_dir, copper_dir):
        profile = choose_profile(stream_name)
        train_out = train_dir / f"{rel.as_posix()}.zli"
        train_out.parent.mkdir(parents=True, exist_ok=True)
        compress_out = compress_dir / f"{rel.as_posix()}.zl"
        compress_out.parent.mkdir(parents=True, exist_ok=True)

        raw_size = file_path.stat().st_size
        if raw_size == 0:
            train_out.write_bytes(b"")
            compress_out.write_bytes(b"")
            results.append({
                "stream": stream_name,
                "path": str(rel),
                "profile": profile,
                "raw_bytes": raw_size,
                "trained_compressor": str(train_out),
                "compressed_bytes": 0,
                "train_ok": True,
                "fallback_profile": False,
            })
            continue

        train_cmd = [
            "zli",
            "train",
            "--profile",
            profile,
            str(file_path),
            "--output",
            str(train_out),
            "--force",
        ]
        if args.max_time_secs is not None:
            train_cmd.extend(["--max-time-secs", str(args.max_time_secs)])

        code, out, err = run(train_cmd)
        if code != 0:
            error_text = err or out
            if "No trainable graph found in compressor" in error_text:
                compressed_size, comp_err = compress_with_profile(file_path, profile, compress_out)
                if comp_err:
                    errors.append({
                        "stage": "compress-fallback",
                        "stream": stream_name,
                        "path": str(rel),
                        "profile": profile,
                        "error": comp_err,
                    })
                    results.append({
                        "stream": stream_name,
                        "path": str(rel),
                        "profile": profile,
                        "raw_bytes": raw_size,
                        "trained_compressor": str(train_out),
                        "compressed_bytes": None,
                        "train_ok": False,
                        "fallback_profile": True,
                    })
                    continue
                results.append({
                    "stream": stream_name,
                    "path": str(rel),
                    "profile": profile,
                    "raw_bytes": raw_size,
                    "trained_compressor": str(train_out),
                    "compressed_bytes": compressed_size,
                    "train_ok": False,
                    "fallback_profile": True,
                })
                continue

            errors.append({
                "stage": "train",
                "stream": stream_name,
                "path": str(rel),
                "profile": profile,
                "error": error_text,
            })
            results.append({
                "stream": stream_name,
                "path": str(rel),
                "profile": profile,
                "raw_bytes": raw_size,
                "trained_compressor": str(train_out),
                "compressed_bytes": None,
                "train_ok": False,
                "fallback_profile": False,
            })
            continue

        code, out, err = run([
            "zli",
            "compress",
            "--compressor",
            str(train_out),
            str(file_path),
            "--output",
            str(compress_out),
        ])
        if code != 0:
            errors.append({
                "stage": "compress",
                "stream": stream_name,
                "path": str(rel),
                "profile": profile,
                "error": err or out,
            })
            results.append({
                "stream": stream_name,
                "path": str(rel),
                "profile": profile,
                "raw_bytes": raw_size,
                "trained_compressor": str(train_out),
                "compressed_bytes": None,
                "train_ok": True,
                "fallback_profile": False,
            })
            continue

        results.append({
            "stream": stream_name,
            "path": str(rel),
            "profile": profile,
            "raw_bytes": raw_size,
            "trained_compressor": str(train_out),
            "compressed_bytes": compress_out.stat().st_size,
            "train_ok": True,
            "fallback_profile": False,
        })

    summary = {
        "base_dir": str(base_dir),
        "train_dir": str(train_dir),
        "compress_dir": str(compress_dir),
        "streams": results,
        "errors": errors,
    }
    result_json.write_text(json.dumps(summary, indent=2))

    ok = [r for r in results if r["compressed_bytes"] is not None]
    raw_total = sum(r["raw_bytes"] for r in ok)
    comp_total = sum(r["compressed_bytes"] for r in ok)

    print(f"Streams trained+compressed: {len(ok)}")
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
