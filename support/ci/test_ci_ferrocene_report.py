"""Regression checks for preserving and qualifying shareable lint diagnostics."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import ferrocene_report


class FerroceneReportTests(unittest.TestCase):
    def test_retains_source_spans_and_nested_diagnostics(self):
        record = {
            "reason": "compiler-message",
            "package_id": "path+file:///copper#cu29@1.4.0-dev",
            "message": {
                "code": {"code": "ferrocene::unvalidated"},
                "message": "call to an unvalidated function",
                "spans": [{"file_name": "src/main.rs", "line_start": 42}],
                "children": [{"message": "the called function is not prevalidated"}],
                "rendered": "warning: call to an unvalidated function\n  --> src/main.rs:42\n",
            },
        }
        self.assertEqual(ferrocene_report.lint_diagnostics(json.dumps(record)), [record])

    def test_ignores_build_noise_and_other_lints(self):
        for line in ["", "not JSON", "[]", '{"reason":"build-finished"}',
                     '{"reason":"compiler-message","message":{"code":{"code":"dead_code"}}}']:
            with self.subTest(line=line):
                self.assertEqual(ferrocene_report.lint_diagnostics(line), [])

    def test_failed_build_keeps_report_and_attempts_remaining_scope(self):
        class Build:
            def __init__(self, status):
                self.status = status
                self.stdout = iter(['{"reason":"build-finished","success":false}\n'])

            def wait(self):
                return self.status

        with tempfile.TemporaryDirectory(dir=ferrocene_report.ROOT / "target") as temp:
            output_dir = Path(temp)
            summary = output_dir / "summary.md"
            with patch.object(ferrocene_report.subprocess, "check_output", side_effect=["Ferrocene compiler", "commit"]), \
                 patch.object(ferrocene_report.subprocess, "Popen", side_effect=[Build(101), Build(0)]) as builds, \
                 patch.dict(ferrocene_report.os.environ, {"GITHUB_STEP_SUMMARY": str(summary)}), \
                 contextlib.redirect_stdout(io.StringIO()):
                status = ferrocene_report.build_report(output_dir)
            self.assertEqual(status, 1)
            self.assertEqual(builds.call_count, 2)
            results = json.loads((output_dir / "diagnostics.json").read_text())
            self.assertEqual([result["exit_code"] for result in results], [101, 0])
            report = (output_dir / "report.md").read_text()
            self.assertIn("diagnostics are partial", report)
            self.assertIn("not a compliance result", report)
            self.assertIn("no annotated validation root", report)
            self.assertEqual(summary.read_text(), report)
            self.assertTrue((output_dir / "criticalup.toml").is_file())


if __name__ == "__main__":
    unittest.main()
