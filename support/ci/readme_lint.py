#!/usr/bin/env python3

"""Lint README structure for core/components/examples crate documentation."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Iterable, List, Optional


REQUIRED_HEADINGS = {
    "core": ["Overview", "Usage", "Compatibility", "Links"],
    "components": [
        "Overview",
        "Interface",
        "Configuration",
        "Usage",
        "Compatibility",
        "Links",
    ],
    "examples": ["Overview", "Prerequisites", "Run", "Expected Output", "Links"],
}


def _first_heading_level(lines: List[str]) -> Optional[int]:
    in_fence = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("```"):
            in_fence = not in_fence
        if in_fence:
            continue
        if not stripped:
            continue
        match = re.match(r"^(#{1,6})\s+.+$", stripped)
        if match:
            return len(match.group(1))
    return None


def _h2_headings(lines: List[str]) -> List[str]:
    in_fence = False
    headings: List[str] = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("```"):
            in_fence = not in_fence
        if in_fence:
            continue
        match = re.match(r"^##\s+(.+?)\s*$", stripped)
        if match:
            headings.append(match.group(1).strip())
    return headings


def _infer_type(path: Path) -> Optional[str]:
    parts = path.parts
    for idx, part in enumerate(parts):
        if part == "core" and idx + 2 < len(parts):
            return "core"
        if part == "components" and idx + 3 < len(parts):
            return "components"
        if part == "examples" and idx + 2 < len(parts):
            return "examples"
    return None


def _iter_repo_readmes(repo_root: Path) -> Iterable[Path]:
    patterns = [
        "core/*/README.md",
        "components/*/*/README.md",
        "examples/*/README.md",
    ]
    for pattern in patterns:
        for path in sorted(repo_root.glob(pattern)):
            yield path


def _lint_file(path: Path) -> List[str]:
    errors: List[str] = []
    doc_type = _infer_type(path)
    if doc_type is None:
        return errors

    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    level = _first_heading_level(lines)
    if level is None:
        errors.append(f"{path}: first heading must be H1")
        return errors
    if level != 1:
        errors.append(f"{path}: first heading must be H1")

    h2 = _h2_headings(lines)
    required = REQUIRED_HEADINGS[doc_type]

    for heading in required:
        if heading not in h2:
            errors.append(f"{path}: missing required heading: {heading}")

    indices = [h2.index(heading) for heading in required if heading in h2]
    if len(indices) >= 2 and indices != sorted(indices):
        errors.append(
            f"{path}: heading order violation: required order is {', '.join(required)}"
        )

    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate README section templates.")
    parser.add_argument(
        "--files",
        nargs="*",
        default=None,
        help="Optional explicit README files to validate.",
    )
    args = parser.parse_args()

    if args.files:
        candidates = [Path(p) for p in args.files]
    else:
        repo_root = Path(__file__).resolve().parents[2]
        candidates = list(_iter_repo_readmes(repo_root))

    all_errors: List[str] = []
    for path in candidates:
        all_errors.extend(_lint_file(path))

    if all_errors:
        for err in all_errors:
            print(err)
        print(f"README lint failed with {len(all_errors)} error(s).")
        sys.exit(1)

    print(f"README lint passed for {len(candidates)} file(s).")


if __name__ == "__main__":
    main()
