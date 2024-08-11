#!/usr/bin/env python3
# This is an example script using the python bindings for the Copper log file format.
from pathlib import Path
import sys

crate_dir = Path(__file__).resolve().parent
target_dir = crate_dir.parent.parent / "target" / "debug"  # Use "release" if you did a release build
sys.path.append(str(target_dir))


import cu29_export
from datetime import datetime, timedelta

log_file_path = "logfile.bin"
index_file_path = target_dir / "cu29_log_index"

log_iterator = cu29_export.PyLogIterator(str(log_file_path), str(index_file_path))
all_strings = log_iterator.all_strings()


for log_entry in log_iterator:

    message = all_strings[log_entry.msg_index()]

    try:
        formatted_message = message.format(*log_entry.params())
    except IndexError as e:
        formatted_message = f"Error formatting message: {e}"

    nanoseconds = log_entry.time_ns()
    elapsed_time = timedelta(seconds=nanoseconds // 1_000_000_000, microseconds=(nanoseconds % 1_000_000_000) // 1_000)

    print(f"{elapsed_time}: {formatted_message}")
