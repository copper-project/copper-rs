"""Verify that published pages use reviewed repository sources."""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "support/ci/wiki_site.py"


class WikiSiteTests(unittest.TestCase):
    def setUp(self):
        target = REPO_ROOT / "target"
        target.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=target)
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.output = self.root / "build"

    def run_site(self):
        return subprocess.run(
            [sys.executable, str(SCRIPT), "--source", str(self.source),
             "--workdir", str(self.output)],
            capture_output=True, text=True,
        )

    def test_reviewed_release_notes_keep_public_page_and_navigation(self):
        (self.source / "Copper-Release-Notes.md").write_text(
            "# v1.2.2 and v1.1.4\n\nbackground_process_empty: true\n"
        )
        (self.source / "Home.md").write_text(
            "[Release notes](Copper-Release-Notes)\n"
        )
        (self.source / "_Sidebar.md").write_text(
            "- [Release notes](Copper-Release-Notes)\n"
        )
        result = self.run_site()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            (self.output / "docs/Copper-Release-Notes.md").read_text(),
            (self.source / "Copper-Release-Notes.md").read_text(),
        )
        self.assertIn("Copper-Release-Notes.md", (self.output / "mkdocs.yml").read_text())
        self.assertIn("Copper-Release-Notes.md", (self.output / "docs/index.md").read_text())
        self.assertFalse((self.output / "docs/_Sidebar.md").exists())

    def test_missing_release_notes_fail_before_clearing_previous_build(self):
        self.output.mkdir()
        sentinel = self.output / "previous-build"
        sentinel.write_text("keep")
        result = self.run_site()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Missing release notes", result.stderr)
        self.assertEqual(sentinel.read_text(), "keep")

    def test_build_directory_cannot_erase_sources(self):
        (self.source / "Copper-Release-Notes.md").write_text("# Released\n")
        self.output = self.source
        result = self.run_site()
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue((self.source / "Copper-Release-Notes.md").exists())


if __name__ == "__main__":
    unittest.main()
