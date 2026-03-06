#!/usr/bin/env python3
from pathlib import Path
import sys


def resolve_paths() -> tuple[Path, Path]:
    crate_dir = Path(__file__).resolve().parents[1]
    workspace_dir = crate_dir.parent.parent
    target_dir = workspace_dir / "target" / "debug"
    return crate_dir, target_dir


def main() -> int:
    crate_dir, target_dir = resolve_paths()
    sys.path.insert(0, str(target_dir))

    try:
        import libcu_flight_controller_export
    except ImportError as exc:
        print(
            "failed to import libcu_flight_controller_export; "
            "build it first with:\n"
            "  cargo build -p cu-flight-controller --no-default-features "
            "--features python-bindings --lib",
            file=sys.stderr,
        )
        print(f"import error: {exc}", file=sys.stderr)
        return 1

    default_log = crate_dir / "logs" / "flight_controller_sim.copper"
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_log
    if not log_path.is_absolute():
        log_path = Path.cwd() / log_path

    print(f"reading: {log_path}")
    count = 0
    for copperlist in libcu_flight_controller_export.copperlist_iterator_unified(str(log_path)):
        for msg in copperlist.messages:
            if msg.task_id != "gnss_fix_sink":
                continue

            print(
                f"id={copperlist.id} task={msg.task_id} "
                f"lat={msg.payload.latitude.value:.7f} {msg.payload.latitude.unit} "
                f"lon={msg.payload.longitude.value:.7f} {msg.payload.longitude.unit}"
            )
            count += 1

    print(f"printed {count} GNSS fixes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
