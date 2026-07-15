#!/usr/bin/env python3
"""Extract replayable VitFly samples and UI-matching previews from a Copper log.

Each exported ``.npz`` contains the exact tensor and context needed by the
original PyTorch ViTFly model, plus the source depth map and Copper outputs. A
same-named ``.png`` renders the depth and arrow exactly like the simulator UI.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
from PIL import Image

MODEL_HEIGHT = 60
MODEL_WIDTH = 90
DEFAULT_MODEL_MAX_DEPTH_M = 12.5
DEFAULT_INVALID_DEPTH = 0.8
DEFAULT_PREVIEW_MAX_DEPTH_M = 20.0
PROGRESS_INTERVAL_SECONDS = 5.0


def workspace_paths() -> tuple[Path, Path, Path]:
    crate_dir = Path(__file__).resolve().parents[1]
    workspace_dir = crate_dir.parent.parent
    target_dir = workspace_dir / "target" / "debug"
    return crate_dir, workspace_dir, target_dir


def parse_args() -> argparse.Namespace:
    crate_dir, _, _ = workspace_paths()
    parser = argparse.ArgumentParser(
        description=(
            "Extract raw depth, exact VitFly model inputs, Copper outputs, and "
            "UI-style previews from a flight-compute unified log."
        )
    )
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        default=crate_dir / "logs" / "flight_compute_sim.copper",
        help=(
            "unified log base (default: flight-controller "
            "logs/flight_compute_sim.copper; pass the base, not an individual "
            "_N.copper slab)"
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=crate_dir / "logs" / "vit-extracted",
        help="output directory (default: flight-controller logs/vit-extracted)",
    )
    parser.add_argument(
        "--every",
        type=positive_int,
        default=1,
        help="export every Nth complete sample (default: 1)",
    )
    parser.add_argument(
        "--limit",
        type=positive_int,
        help="stop after this many exported samples",
    )
    parser.add_argument("--start-cl", type=int, help="first CopperList id to consider")
    parser.add_argument("--end-cl", type=int, help="last CopperList id to consider")
    parser.add_argument(
        "--model-max-depth-m",
        type=positive_float,
        default=DEFAULT_MODEL_MAX_DEPTH_M,
        help="normalization depth used by the Copper task (default: 12.5)",
    )
    parser.add_argument(
        "--invalid-depth",
        type=float,
        default=DEFAULT_INVALID_DEPTH,
        help="normalized replacement for non-finite/non-positive depth (default: 0.8)",
    )
    parser.add_argument(
        "--preview-max-depth-m",
        type=positive_float,
        default=DEFAULT_PREVIEW_MAX_DEPTH_M,
        help="far distance used by the simulator preview (default: 20.0)",
    )
    return parser.parse_args()


def positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return value


def positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def resolve_input_path(path: Path) -> Path:
    if not path.is_absolute():
        path = Path.cwd() / path
    # Unified log bases are commonly symlinks to the first `_0.copper` slab.
    # Preserve the base name: resolving that symlink would make Copper append a
    # second `_0` and look for `flight_compute_sim_0_0.copper`.
    return Path(os.path.abspath(path))


def unified_log_exists(base: Path) -> bool:
    if base.exists():
        return True
    first_slab = base.with_name(f"{base.stem}_0{base.suffix}")
    return first_slab.exists()


def normalized_model_input(
    depth_m: np.ndarray,
    max_depth_m: float,
    invalid_depth: float,
) -> np.ndarray:
    """Match cu-vitfly's normalize-then-bilinear-resize preprocessing."""
    source = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(source) & (source > np.float32(0.0))
    normalized = np.full(source.shape, np.float32(invalid_depth), dtype=np.float32)
    normalized[valid] = np.clip(
        source[valid] / np.float32(max_depth_m),
        np.float32(0.0),
        np.float32(1.0),
    )

    source_height, source_width = normalized.shape
    scale_x = np.float32(source_width) / np.float32(MODEL_WIDTH)
    scale_y = np.float32(source_height) / np.float32(MODEL_HEIGHT)
    output_x = np.arange(MODEL_WIDTH, dtype=np.float32)
    output_y = np.arange(MODEL_HEIGHT, dtype=np.float32)
    source_x = np.clip(
        (output_x + np.float32(0.5)) * scale_x - np.float32(0.5),
        np.float32(0.0),
        np.float32(source_width - 1),
    )
    source_y = np.clip(
        (output_y + np.float32(0.5)) * scale_y - np.float32(0.5),
        np.float32(0.0),
        np.float32(source_height - 1),
    )
    x0 = np.floor(source_x).astype(np.intp)
    y0 = np.floor(source_y).astype(np.intp)
    x1 = np.minimum(x0 + 1, source_width - 1)
    y1 = np.minimum(y0 + 1, source_height - 1)
    wx = (source_x - x0.astype(np.float32))[None, :]
    wy = (source_y - y0.astype(np.float32))[:, None]

    top = normalized[y0[:, None], x0[None, :]] * (np.float32(1.0) - wx)
    top += normalized[y0[:, None], x1[None, :]] * wx
    bottom = normalized[y1[:, None], x0[None, :]] * (np.float32(1.0) - wx)
    bottom += normalized[y1[:, None], x1[None, :]] * wx
    resized = top * (np.float32(1.0) - wy) + bottom * wy
    return np.ascontiguousarray(resized[None, None, :, :], dtype=np.float32)


def scalar_first_quaternion(pose_rpy_rad: np.ndarray) -> np.ndarray:
    """Match cu-vitfly's aerospace Euler to scalar-first quaternion conversion."""
    roll, pitch, yaw = np.asarray(pose_rpy_rad, dtype=np.float32)
    sr = np.sin(np.float32(0.5) * roll)
    cr = np.cos(np.float32(0.5) * roll)
    sp = np.sin(np.float32(0.5) * pitch)
    cp = np.cos(np.float32(0.5) * pitch)
    sy = np.sin(np.float32(0.5) * yaw)
    cy = np.cos(np.float32(0.5) * yaw)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float32,
    )[None, :]


def render_preview(
    depth_m: np.ndarray,
    velocity_mps: np.ndarray | None,
    preview_max_depth_m: float,
) -> np.ndarray:
    """Port the flight simulator's depth colors and VitFly arrow pixel-for-pixel."""
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(depth) & (depth <= np.float32(preview_max_depth_m))
    scaled = np.clip(
        depth / np.float32(preview_max_depth_m),
        np.float32(0.0),
        np.float32(1.0),
    )
    intensity_f = np.sqrt(np.float32(1.0) - scaled) * np.float32(255.0)
    intensity_f[~valid] = np.float32(0.0)
    intensity = np.floor(intensity_f + np.float32(0.5)).astype(np.uint8)
    pixels = np.repeat(intensity[:, :, None], 3, axis=2)
    if velocity_mps is not None:
        draw_vitfly_arrow(pixels, np.asarray(velocity_mps, dtype=np.float32))
    return pixels


def draw_vitfly_arrow(pixels: np.ndarray, velocity_mps: np.ndarray) -> None:
    height, width, _ = pixels.shape
    left = float(velocity_mps[1])
    up = float(velocity_mps[2])
    if not math.isfinite(left) or not math.isfinite(up) or width == 0 or height == 0:
        return

    center = (width * 0.5, height * 0.5)
    end = (
        min(max(center[0] - left * width / 3.0, 0.0), width - 1.0),
        min(max(center[1] - up * height / 3.0, 0.0), height - 1.0),
    )
    draw_preview_line(pixels, center, end, radius=2)

    direction_x = center[0] - end[0]
    direction_y = center[1] - end[1]
    length_squared = direction_x * direction_x + direction_y * direction_y
    if length_squared < 1.0:
        return
    inverse_length = 1.0 / math.sqrt(length_squared)
    direction_x *= inverse_length
    direction_y *= inverse_length
    perpendicular = (-direction_y, direction_x)
    base = (end[0] + direction_x * 12.0, end[1] + direction_y * 12.0)
    draw_preview_line(
        pixels,
        end,
        (base[0] + perpendicular[0] * 7.0, base[1] + perpendicular[1] * 7.0),
        radius=2,
    )
    draw_preview_line(
        pixels,
        end,
        (base[0] - perpendicular[0] * 7.0, base[1] - perpendicular[1] * 7.0),
        radius=2,
    )


def draw_preview_line(
    pixels: np.ndarray,
    start: tuple[float, float],
    end: tuple[float, float],
    radius: int,
) -> None:
    height, width, _ = pixels.shape
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    steps = max(int(math.ceil(max(abs(delta_x), abs(delta_y), 1.0))), 1)
    for step in range(steps + 1):
        fraction = step / steps
        x = rust_round(start[0] + delta_x * fraction)
        y = rust_round(start[1] + delta_y * fraction)
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if offset_x * offset_x + offset_y * offset_y > radius * radius:
                    continue
                pixel_x = x + offset_x
                pixel_y = y + offset_y
                if 0 <= pixel_x < width and 0 <= pixel_y < height:
                    pixels[pixel_y, pixel_x] = (255, 32, 32)


def rust_round(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def optional_vector(sample: dict[str, Any], name: str) -> np.ndarray | None:
    value = sample[name]
    if value is None:
        return None
    return np.asarray(value, dtype=np.float32)


def sample_arrays(
    sample: dict[str, Any],
    model_max_depth_m: float,
    invalid_depth: float,
) -> dict[str, np.ndarray] | None:
    pose = optional_vector(sample, "pose_rpy_rad")
    task_velocity = optional_vector(sample, "task_velocity_mps")
    preview_velocity = optional_vector(sample, "preview_velocity_mps")
    desired_speed = sample["desired_speed_mps"]
    if (
        pose is None
        or task_velocity is None
        or preview_velocity is None
        or desired_speed is None
    ):
        return None

    height = int(sample["height"])
    width = int(sample["width"])
    stride = int(sample["stride"])
    depth_with_stride = np.frombuffer(sample["depth_f32_le"], dtype="<f4")
    expected = height * stride
    if depth_with_stride.size != expected:
        raise ValueError(
            f"CL {sample['cl_id']} has {depth_with_stride.size} depth values; expected {expected}"
        )
    depth_m = np.ascontiguousarray(depth_with_stride.reshape(height, stride)[:, :width])
    desired_velocity = np.asarray([[desired_speed]], dtype=np.float32)
    model_input = normalized_model_input(depth_m, model_max_depth_m, invalid_depth)
    attitude = scalar_first_quaternion(pose)
    command_velocity = optional_vector(sample, "command_velocity_mps")
    if command_velocity is None:
        command_velocity = np.full(3, np.nan, dtype=np.float32)
    model_output = np.full(3, np.nan, dtype=np.float32)
    if math.isfinite(float(desired_speed)) and float(desired_speed) > 0.0:
        model_output = task_velocity / np.float32(desired_speed)

    return {
        "depth_m": depth_m,
        "model_input": model_input,
        "desired_velocity": desired_velocity,
        "attitude": attitude,
        "pose_rpy_rad": pose,
        "copper_model_output": model_output,
        "copper_task_velocity_mps": task_velocity,
        "copper_command_velocity_mps": command_velocity,
        "preview_velocity_mps": preview_velocity,
        "cl_id": np.asarray(sample["cl_id"], dtype=np.uint64),
        "frame_seq": np.asarray(sample["frame_seq"], dtype=np.uint64),
        "tov_ns": np.asarray(sample["tov_ns"] or 0, dtype=np.uint64),
    }


def preload_zed_wrapper(target_dir: Path) -> None:
    """Make Cargo's locally built ZED C wrapper visible to the extension loader."""
    candidates = [
        Path("/usr/local/zed/lib/libsl_zed_c.so"),
        Path("/opt/zed-sdk/lib/libsl_zed_c.so"),
    ]
    candidates.extend(
        sorted(
            target_dir.glob("build/zed-sdk-sys-*/out/zed-c-api-build/libsl_zed_c.so"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
    )
    for candidate in candidates:
        if not candidate.exists():
            continue
        try:
            ctypes.CDLL(str(candidate), mode=ctypes.RTLD_GLOBAL)
            return
        except OSError:
            continue


def import_copper_binding(target_dir: Path):
    preload_zed_wrapper(target_dir)
    sys.path.insert(0, str(target_dir))
    try:
        import libcu_flight_controller_export
    except ImportError as error:
        raise RuntimeError(
            "failed to import libcu_flight_controller_export; build it first with:\n"
            "  just -f examples/cu_flight_controller/justfile py-vitfly-build"
        ) from error
    if not hasattr(libcu_flight_controller_export, "vitfly_sample_iterator_unified"):
        raise RuntimeError(
            "libcu_flight_controller_export was built without the VitFly schema; rebuild it with:\n"
            "  just -f examples/cu_flight_controller/justfile py-vitfly-build"
        )
    return libcu_flight_controller_export


def main() -> int:
    args = parse_args()
    _, _, target_dir = workspace_paths()
    log_path = resolve_input_path(args.log)
    output_dir = resolve_input_path(args.output)
    if not unified_log_exists(log_path):
        print(f"Copper log base not found: {log_path}", file=sys.stderr)
        return 2
    if not math.isfinite(args.invalid_depth):
        print("--invalid-depth must be finite", file=sys.stderr)
        return 2

    manifest_path = output_dir / "manifest.jsonl"
    dataset_path = output_dir / "dataset.json"
    if manifest_path.exists() or dataset_path.exists():
        print(
            f"output already contains an extraction manifest: {output_dir}\n"
            "choose a new --output directory",
            file=sys.stderr,
        )
        return 2
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        binding = import_copper_binding(target_dir)
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 2

    print(f"Extracting VitFly samples from {log_path}", flush=True)
    print(f"Writing samples to {output_dir}", flush=True)

    started_at = time.monotonic()
    last_progress_at = started_at
    copperlists_seen = 0
    complete_seen = 0
    exported = 0
    skipped_incomplete = 0
    manifest_rows: list[dict[str, Any]] = []
    iterator = binding.vitfly_sample_iterator_unified(str(log_path))
    for sample in iterator:
        cl_id = int(sample["cl_id"])
        if args.start_cl is not None and cl_id < args.start_cl:
            continue
        if args.end_cl is not None and cl_id > args.end_cl:
            break

        copperlists_seen += 1
        now = time.monotonic()
        if now - last_progress_at >= PROGRESS_INTERVAL_SECONDS:
            elapsed = now - started_at
            rate = exported / elapsed if elapsed > 0.0 else 0.0
            print(
                f"  processed {copperlists_seen:,} CopperLists through CL {cl_id:,}; "
                f"wrote {exported:,} samples in {elapsed:.0f}s "
                f"({rate:.1f} samples/s)",
                flush=True,
            )
            last_progress_at = now

        arrays = sample_arrays(sample, args.model_max_depth_m, args.invalid_depth)
        if arrays is None:
            skipped_incomplete += 1
            continue
        if complete_seen % args.every != 0:
            complete_seen += 1
            continue
        complete_seen += 1

        basename = f"sample_{exported:06d}_cl_{cl_id:010d}"
        npz_name = f"{basename}.npz"
        preview_name = f"{basename}.png"
        npz_path = output_dir / npz_name
        preview_path = output_dir / preview_name
        if npz_path.exists() or preview_path.exists():
            raise FileExistsError(f"refusing to replace existing sample {basename}")

        np.savez_compressed(npz_path, **arrays)
        preview = render_preview(
            arrays["depth_m"],
            arrays["preview_velocity_mps"],
            args.preview_max_depth_m,
        )
        Image.fromarray(preview, mode="RGB").save(preview_path)

        row = {
            "index": exported,
            "cl_id": cl_id,
            "frame_seq": int(sample["frame_seq"]),
            "tov_ns": sample["tov_ns"],
            "npz": npz_name,
            "preview": preview_name,
            "desired_speed_mps": sample["desired_speed_mps"],
            "copper_model_output": arrays["copper_model_output"].tolist(),
            "copper_task_velocity_mps": arrays["copper_task_velocity_mps"].tolist(),
            "copper_command_velocity_mps": sample["command_velocity_mps"],
            "preview_velocity_mps": arrays["preview_velocity_mps"].tolist(),
        }
        manifest_rows.append(row)
        exported += 1
        if args.limit is not None and exported >= args.limit:
            break

    with manifest_path.open("x", encoding="utf-8") as manifest:
        for row in manifest_rows:
            manifest.write(json.dumps(row, allow_nan=False, separators=(",", ":")))
            manifest.write("\n")
    dataset = {
        "format_version": 1,
        "source_log": str(log_path),
        "sample_count": exported,
        "skipped_incomplete": skipped_incomplete,
        "every": args.every,
        "model_input": {
            "shape": [1, 1, MODEL_HEIGHT, MODEL_WIDTH],
            "dtype": "float32",
            "max_depth_m": args.model_max_depth_m,
            "invalid_depth": args.invalid_depth,
            "processing": "normalize_source_then_half_pixel_bilinear_resize",
        },
        "attitude": {"shape": [1, 4], "order": "wxyz", "dtype": "float32"},
        "desired_velocity": {"shape": [1, 1], "unit": "m/s", "dtype": "float32"},
        "depth_m": {"shape": "[source_height, source_width]", "dtype": "float32"},
        "preview": {
            "max_depth_m": args.preview_max_depth_m,
            "arrow_axes": "positive left points image-left; positive up points image-up",
            "arrow_source": "conditioned command when present, otherwise VitFly task velocity",
        },
    }
    with dataset_path.open("x", encoding="utf-8") as metadata_file:
        json.dump(dataset, metadata_file, indent=2)
        metadata_file.write("\n")

    elapsed = time.monotonic() - started_at
    print(
        f"Exported {exported:,} samples to {output_dir} in {elapsed:.1f}s "
        f"({skipped_incomplete:,} incomplete frames skipped)",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
