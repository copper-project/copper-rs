import unittest

from ci_changes import needs_code_checks


class ChangeClassificationTests(unittest.TestCase):
    def test_docs_only(self):
        self.assertFalse(needs_code_checks(["README.md", "components/tasks/foo/README.md", "docs/logo.svg"]))

    def test_code_mixed_with_docs(self):
        self.assertTrue(needs_code_checks(["README.md", "core/cu29/src/lib.rs"]))

    def test_build_and_ci_inputs(self):
        for path in ["Cargo.toml", "copperconfig.ron", ".github/workflows/general.yml", "support/docker/Dockerfile.ci-ubuntu26", "unknown-file"]:
            with self.subTest(path=path):
                self.assertTrue(needs_code_checks([path]))

    def test_empty_diff_runs_checks(self):
        self.assertTrue(needs_code_checks([]))


if __name__ == "__main__":
    unittest.main()
