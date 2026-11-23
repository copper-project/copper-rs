"""Summarize direct Copper core calls diagnosed by the Ferrocene firmware build."""

import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
hits = set()
diagnostics = 0
core_instantiation_seen = False
for line in sys.stdin:
    event = json.loads(line)
    if event["reason"] != "compiler-message":
        continue
    message = event["message"]
    if message["level"] == "error":
        print(message["rendered"], file=sys.stderr)
    if (message["code"] or {}).get("code") != "ferrocene::unvalidated":
        continue
    diagnostics += 1
    spans = message["spans"]
    # A private audit root can emit local warnings without traversing Copper.
    # The pinned compiler attaches instantiation notes during its post-mono pass.
    callers = [
        (Path(span["file_name"]).resolve(), span["line_start"])
        for span in spans
        if span["is_primary"]
    ]
    core_callers = [(path, line) for path, line in callers if path.is_relative_to(ROOT / "core")]
    if core_callers and any("instantiated by" in child["message"] for child in message["children"]):
        core_instantiation_seen = True
    for definition in spans:
        if not re.search(r"/library/(core|alloc)/src/", definition["file_name"]):
            continue
        name = re.fullmatch(r"`(.+)` is unvalidated", definition["label"] or "")
        if not name:
            continue
        for path, line_number in core_callers:
            hits.add((str(path.relative_to(ROOT)), line_number, name[1]))

if not diagnostics:
    sys.exit("No Ferrocene audit diagnostics received. Check build output and audit activation.")
if not core_instantiation_seen:
    sys.exit(f"Inconclusive audit: {diagnostics} diagnostics, none confirming traversal into Copper core.")
for path, line_number, name in sorted(hits):
    print(f"{path}:{line_number}  {name}")
print(f"{len(hits)} distinct call-site and callee pairs from Copper core into core/alloc ({diagnostics} total lint diagnostics)")
