"""Exercise image reuse and publication planning without Docker or network access."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class ImagePlanTests(unittest.TestCase):
    def plan(self, *, event="pull_request", fork=False, refresh=False,
             changed="README.md", exists=True, ref="refs/pull/1/merge",
             before="base", fetch_fails=False):
        artifacts = ROOT / "target" / "ci-image-tests"
        artifacts.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=artifacts) as temporary:
            directory = Path(temporary)
            for name, body in {
                "git": '''case "$1" in
fetch) test "$FETCH_FAILS" != true ;;
diff|diff-tree) printf '%s\\n' "$CHANGED_FILES" ;;
*) exit 99 ;;
esac
''',
                "docker": 'test "$IMAGE_EXISTS" = true\n',
            }.items():
                executable = directory / name
                executable.write_text("#!/usr/bin/env bash\n" + body)
                executable.chmod(0o755)
            output = directory / "output"
            env = dict(os.environ, PATH=f"{directory}:{os.environ['PATH']}",
                       GITHUB_OUTPUT=str(output), GITHUB_EVENT_NAME=event,
                       GITHUB_REPOSITORY="copper-project/copper-rs",
                       PR_HEAD_REPO="fork/copper-rs" if fork else "copper-project/copper-rs",
                       GITHUB_SHA="head", BASE_SHA="base", BEFORE_SHA=before,
                       GITHUB_REF=ref, GITHUB_RUN_ID="123", GITHUB_RUN_ATTEMPT="2",
                       CI_IMAGE_TAG="ubuntu-26.04", REFRESH_IMAGE=str(refresh).lower(),
                       CHANGED_FILES=changed, IMAGE_EXISTS=str(exists).lower(),
                       FETCH_FAILS=str(fetch_fails).lower())
            subprocess.run(["bash", str(ROOT / "support/ci/prepare_ci_image.sh")],
                           env=env, check=True, capture_output=True, text=True)
            return output.read_text()

    def test_reuse_published_image(self):
        self.assertEqual(self.plan(), "image=ghcr.io/copper-project/copper-rs-ci:ubuntu-26.04\nshould_build=false\n")

    def test_fork_uses_host_without_publishing(self):
        result = self.plan(fork=True, changed="support/docker/Dockerfile.ci-ubuntu26")
        self.assertIn("image=\nshould_build=false\n", result)

    def test_fork_image_validation_builds_without_usable_image(self):
        result = self.plan(fork=True, refresh=True)
        self.assertIn("image=\nshould_build=true\n", result)
        self.assertNotIn("-run-", result)

    def test_image_inputs_force_a_build(self):
        for changed in ["support/docker/Dockerfile.ci-ubuntu26",
                        ".github/actions/ci-image/action.yml",
                        ".github/workflows/build-ci-images.yml",
                        "support/ci/prepare_ci_image.sh"]:
            with self.subTest(changed=changed):
                result = self.plan(changed=changed)
                self.assertIn("should_build=true", result)
                self.assertIn("ubuntu-26.04-run-123-2", result)
                self.assertNotIn("\nghcr.io/copper-project/copper-rs-ci:ubuntu-26.04\n", result)

    def test_missing_image_builds(self):
        self.assertIn("should_build=true", self.plan(exists=False))

    def test_refresh_master_updates_stable_tag(self):
        result = self.plan(event="schedule", refresh=True, ref="refs/heads/master")
        self.assertIn("\nghcr.io/copper-project/copper-rs-ci:ubuntu-26.04\n", result)

    def test_manual_branch_refresh_does_not_update_stable_tag(self):
        result = self.plan(event="workflow_dispatch", refresh=True, ref="refs/heads/topic")
        self.assertIn("should_build=true", result)
        self.assertNotIn("\nghcr.io/copper-project/copper-rs-ci:ubuntu-26.04\n", result)

    def test_master_push_detects_docker_changes(self):
        for before in ["base", "0" * 40]:
            with self.subTest(before=before):
                result = self.plan(event="push", ref="refs/heads/master", before=before,
                                   changed="support/docker/Dockerfile.ci-ubuntu26")
                self.assertIn("should_build=true", result)
                self.assertIn("\nghcr.io/copper-project/copper-rs-ci:ubuntu-26.04\n", result)

    def test_base_fetch_failure_fails_closed(self):
        for event in ["pull_request", "push"]:
            with self.subTest(event=event):
                with self.assertRaises(subprocess.CalledProcessError):
                    self.plan(event=event, fetch_fails=True)


if __name__ == "__main__":
    unittest.main()
