#!/usr/bin/env python3
# This is an example script using the python bindings for the Copper log file format.
from pathlib import Path
import sys

crate_dir = Path(__file__).resolve().parent
target_dir = crate_dir.parent.parent / "target" / "debug"  # Use "release" if you did a release build
sys.path.append(str(target_dir))


import libcu29_export

log_file_path = "logfile.bin"
index_file_path = target_dir / "cu29_log_index"

log_iterator, all_strings = libcu29_export.struct_log_iterator_bare(log_file_path, str(index_file_path))


for log_entry in log_iterator:
    message = all_strings[log_entry.msg_index()]
    parameters = log_entry.params()

    try:
        formatted_message = message.format(*parameters)
    except IndexError as e:
        formatted_message = f"Error formatting message: {e}"

    print(f"{log_entry.ts()}: {formatted_message}")
