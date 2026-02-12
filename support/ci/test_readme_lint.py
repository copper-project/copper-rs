#!/usr/bin/env python3

from __future__ import annotations

import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LINT_SCRIPT = REPO_ROOT / "support" / "ci" / "readme_lint.py"


def _run_lint(*files: Path) -> subprocess.CompletedProcess[str]:
    cmd = ["python3", str(LINT_SCRIPT), "--files", *[str(f) for f in files]]
    return subprocess.run(cmd, capture_output=True, text=True)


class ReadmeLintTests(unittest.TestCase):
    def test_core_template_passes(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            readme = Path(td) / "core" / "demo" / "README.md"
            readme.parent.mkdir(parents=True, exist_ok=True)
            readme.write_text(
                textwrap.dedent(
                    """\
                    # demo

                    ## Overview
                    ok

                    ## Usage
                    ok

                    ## Compatibility
                    ok

                    ## Links
                    - x
                    """
                ),
                encoding="utf-8",
            )
            res = _run_lint(readme)
            self.assertEqual(res.returncode, 0, msg=res.stdout + res.stderr)

    def test_missing_required_heading_fails(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            readme = Path(td) / "components" / "tasks" / "demo" / "README.md"
            readme.parent.mkdir(parents=True, exist_ok=True)
            readme.write_text(
                textwrap.dedent(
                    """\
                    # demo

                    ## Overview
                    ok

                    ## Interface
                    ok

                    ## Usage
                    ok

                    ## Compatibility
                    ok

                    ## Links
                    - x
                    """
                ),
                encoding="utf-8",
            )
            res = _run_lint(readme)
            self.assertNotEqual(res.returncode, 0)
            self.assertIn("missing required heading", res.stdout)

    def test_heading_order_fails(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            readme = Path(td) / "examples" / "demo" / "README.md"
            readme.parent.mkdir(parents=True, exist_ok=True)
            readme.write_text(
                textwrap.dedent(
                    """\
                    # demo

                    ## Overview
                    ok

                    ## Run
                    ok

                    ## Prerequisites
                    ok

                    ## Expected Output
                    ok

                    ## Links
                    - x
                    """
                ),
                encoding="utf-8",
            )
            res = _run_lint(readme)
            self.assertNotEqual(res.returncode, 0)
            self.assertIn("heading order violation", res.stdout)

    def test_missing_h1_fails(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            readme = Path(td) / "core" / "demo" / "README.md"
            readme.parent.mkdir(parents=True, exist_ok=True)
            readme.write_text(
                textwrap.dedent(
                    """\
                    ## Overview
                    ok

                    ## Usage
                    ok

                    ## Compatibility
                    ok

                    ## Links
                    - x
                    """
                ),
                encoding="utf-8",
            )
            res = _run_lint(readme)
            self.assertNotEqual(res.returncode, 0)
            self.assertIn("first heading must be H1", res.stdout)


if __name__ == "__main__":
    unittest.main()
