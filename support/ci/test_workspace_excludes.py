"""Regression tests for workspace selection on CI runner operating systems."""

import json
import unittest
from unittest.mock import patch

import workspace_excludes


class WorkspaceExcludesTest(unittest.TestCase):
    def excluded_names(self, platform):
        packages = [
            ("zed-sdk-sys", {"ci_exclude_workspace_os": ["macos"]}),
            ("zed-sdk", {"ci_exclude_workspace_os": ["macos"]}),
            ("cu-zed", {"environments": ["host"], "host_os": ["linux"]}),
            ("embedded", {"environments": ["embedded"]}),
            ("always-excluded", {"ci_exclude_workspace": True}),
            ("windows-excluded", {"ci_exclude_workspace_os": ["windows"]}),
            ("dual-environment", {"environments": ["host", "embedded"]}),
            ("ordinary", {}),
        ]
        metadata = {
            "packages": [
                {
                    "name": name,
                    "manifest_path": f"/{name}/Cargo.toml",
                    "metadata": {"copper": copper},
                }
                for name, copper in packages
            ]
        }
        with (
            patch.object(workspace_excludes.sys, "platform", platform),
            patch.object(
                workspace_excludes.subprocess,
                "check_output",
                return_value=json.dumps(metadata),
            ),
        ):
            return [
                package["name"]
                for package in workspace_excludes._load_excluded_packages(None)
            ]

    def test_macos_excludes_sdk_wrappers_and_keeps_portable_source(self):
        self.assertEqual(
            self.excluded_names("darwin"),
            ["always-excluded", "embedded", "zed-sdk", "zed-sdk-sys"],
        )

    def test_linux_keeps_sdk_wrappers(self):
        self.assertEqual(
            self.excluded_names("linux"), ["always-excluded", "embedded"]
        )

    def test_windows_uses_rust_os_name_and_keeps_sdk_wrappers(self):
        self.assertEqual(
            self.excluded_names("win32"),
            ["always-excluded", "embedded", "windows-excluded"],
        )


if __name__ == "__main__":
    unittest.main()
