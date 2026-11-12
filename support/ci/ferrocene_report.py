#!/usr/bin/env python3
"""Collect shareable Ferrocene lint diagnostics from representative applications."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = ROOT / "target/ferrocene-report"
LINT = "ferrocene::unvalidated"
BUILDS = {
    "host": ["-p", "cu-nologging-task", "--bin", "nologging-tasks"],
    "baremetal": [
        "-p", "cu-min-baremetal", "--bin", "cu-min-baremetal",
        "--no-default-features", "--target", "aarch64-unknown-none",
        "--config", "examples/cu_min_baremetal/.cargo/config-nostd.toml",
    ],
}


def lint_diagnostics(line: str) -> list[dict]:
    """Retain native lint messages, including their spans and nested notes."""
    try:
        record = json.loads(line)
    except json.JSONDecodeError:
        return []
    if not isinstance(record, dict) or record.get("reason") != "compiler-message":
        return []
    message = record["message"]
    if (message.get("code") or {}).get("code") != LINT:
        return []
    return [record]


def build_report(output_dir: Path) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / "criticalup.toml", output_dir / "criticalup.toml")
    compiler = subprocess.check_output(
        ["rustc", "+ferrocene", "--version", "--verbose"], cwd=ROOT, text=True
    ).strip()
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    sections = [
        "# Ferrocene unvalidated-call report",
        f"Commit: `{commit}`",
        "Compiler:\n```text\n" + compiler + "\n```",
        "Baseline: see the attached criticalup.toml. Builds use the dev profile.",
        "## Scope and interpretation",
        "This is a compiler lint boundary report, not a complete inventory of "
        "uncertified APIs or safety certification evidence. The host example "
        "exercises generated runtime wiring and logging; the bare-metal example "
        "checks the no_std configuration on aarch64-unknown-none.",
        "The lint checks calls from prevalidated functions (including ordinary "
        "main entrypoints). It stops at unvalidated boundaries: Copper and "
        "third-party functions are not assumed prevalidated. The bare-metal "
        "example uses no_main and has no annotated validation root. Zero "
        "diagnostics therefore does not establish certified-subset compliance. "
        "Other features, targets, uninstantiated generics, and unmarked function "
        "bodies are not exhaustively audited. No prevalidated annotations or "
        "lint suppressions are added to Copper for this report.",
        "Reference: https://public-docs.ferrocene.dev/main/safety-manual/core/subset.html",
    ]
    results = []
    for name, args in BUILDS.items():
        command = [
            "cargo", "+ferrocene", "rustc", *args, "--message-format=json",
            "--", f"--force-warn={LINT}",
        ]
        findings = []
        print(f"Running: {shlex.join(command)}", flush=True)
        with (output_dir / f"{name}.jsonl").open("w") as raw, (
            output_dir / f"{name}.stderr.log"
        ).open("w") as stderr:
            process = subprocess.Popen(
                command, cwd=ROOT, stdout=subprocess.PIPE, stderr=stderr, text=True
            )
            assert process.stdout is not None
            for line in process.stdout:
                raw.write(line)
                findings.extend(lint_diagnostics(line))
            status = process.wait()
        print((output_dir / f"{name}.stderr.log").read_text(), flush=True)
        results.append({"scope": name, "command": command, "exit_code": status,
                        "diagnostics": findings})
        sections.extend([
            f"## {name}",
            "```sh\n" + shlex.join(command) + "\n```",
            f"Build exit code: {status}. Lint diagnostics: {len(findings)}.",
        ])
        if status:
            sections.append("Build failed; diagnostics are partial. See raw build logs.")
        if not findings:
            sections.append("No lint diagnostics emitted in this scope; this is not a compliance result.")
        for record in findings:
            message = record["message"]
            sections.append("```text\n" + (message.get("rendered") or message["message"]).rstrip() + "\n```")
    (output_dir / "diagnostics.json").write_text(json.dumps(results, indent=2) + "\n")
    report = "\n\n".join(sections) + "\n"
    (output_dir / "report.md").write_text(report)
    if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
        with Path(summary).open("a") as output:
            output.write(report)
    # Findings are advisory; compilation errors still fail CI after saving the report.
    return int(any(result["exit_code"] for result in results))


if __name__ == "__main__":
    raise SystemExit(build_report(REPORT_DIR))
