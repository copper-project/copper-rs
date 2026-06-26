#!/usr/bin/env python3
"""Read typed GNSS CopperLists from a flight-controller unified log.

This script demonstrates the app-specific PyO3 module pattern used for offline log
analysis. Python is inspecting recorded data here, not participating in the live
runtime.
"""

import math
from pathlib import Path
import sys


def resolve_paths() -> tuple[Path, Path]:
    crate_dir = Path(__file__).resolve().parents[1]
    workspace_dir = crate_dir.parent.parent
    target_dir = workspace_dir / "target" / "debug"
    return crate_dir, target_dir


def format_tov(msg) -> str:
    kind = getattr(msg.tov, "kind", "none")
    if kind == "time":
        return f"{msg.tov.time_ns / 1_000_000_000.0:.9f}s"
    if kind == "range":
        return (
            f"{msg.tov.start_ns / 1_000_000_000.0:.9f}s"
            f"-{msg.tov.end_ns / 1_000_000_000.0:.9f}s"
        )
    return "none"


def format_angle(quantity) -> tuple[float, str]:
    value = quantity.value
    unit = getattr(quantity, "unit", "")
    if unit == "rad":
        return math.degrees(value), "deg"
    return value, unit


def main() -> int:
    crate_dir, target_dir = resolve_paths()
    sys.path.insert(0, str(target_dir))

    try:
        import libcu_flight_controller_export
    except ImportError as exc:
        print(
            "failed to import libcu_flight_controller_export; "
            "build it first with:\n"
            "  cargo rustc -p cu-flight-controller --no-default-features "
            "--features python-bindings --lib --crate-type cdylib",
            file=sys.stderr,
        )
        print(f"import error: {exc}", file=sys.stderr)
        return 1

    default_log = crate_dir / "logs" / "flight_controller_sim.copper"
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_log
    if not log_path.is_absolute():
        log_path = Path.cwd() / log_path

    print(f"reading: {log_path}")
    for record in libcu_flight_controller_export.runtime_lifecycle_iterator_unified(str(log_path)):
        if record.event.kind == "mission_started":
            print(
                f"runtime mission start: ts={record.timestamp_ns} "
                f"mission={record.event.mission}"
            )
            break

    count = 0
    for copperlist in libcu_flight_controller_export.copperlist_iterator_unified(str(log_path)):
        for msg in copperlist.messages:
            if msg.task_id not in {"gnss_ublox", "gnss_fix_sink"}:
                continue
            if msg.payload is None or not hasattr(msg.payload, "position"):
                continue

            lat, lat_unit = format_angle(msg.payload.position.latitude)
            lon, lon_unit = format_angle(msg.payload.position.longitude)
            extras = []
            if hasattr(msg.payload, "num_satellites_used"):
                extras.append(f"sats={msg.payload.num_satellites_used}")
            if hasattr(msg.payload, "gnss_fix_ok"):
                extras.append(f"fix_ok={msg.payload.gnss_fix_ok}")
            extra_text = f" {' '.join(extras)}" if extras else ""

            print(
                f"id={copperlist.id} tov={format_tov(msg)} task={msg.task_id} "
                f"lat={lat:.7f} {lat_unit} "
                f"lon={lon:.7f} {lon_unit}{extra_text}"
            )
            count += 1

    print(f"printed {count} GNSS fixes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
