#!/usr/bin/env python3
"""Install cargo-cunew, build registry-backed templates, and run their apps."""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import tomllib


def run(command, **kwargs):
    print("+ " + " ".join(map(str, command)), flush=True)
    subprocess.run(command, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--source", choices=("checkout", "published"), default="published")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = root / "target" / "released-template-check" / args.source
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    cargo = ["cargo", f"+{args.toolchain}"]
    install = cargo + ["install", "--force", "--root", str(output / "install"),
                       "--target-dir", str(root / "target" / "cunew-install")]
    if args.source == "checkout":
        install += ["--path", str(root / "support" / "cargo_cunew")]
    else:
        install += ["cargo-cunew"]
    started = time.monotonic()
    run(install)
    print(f"Installed cargo-cunew in {time.monotonic() - started:.1f} seconds", flush=True)
    generator = output / "install" / "bin" / "cargo-cunew"
    run([str(generator), "--version"])
    env = os.environ.copy()
    env["CARGO_TARGET_DIR"] = str(root / "target" / "released-template-build")
    for template in ("project", "workspace"):
        project = output / ("hello_copper" if template == "project" else "hello-workspace")
        run([str(generator), str(project), "--template", template, "--no-vcs"], cwd=root)
        metadata = json.loads(subprocess.check_output(
            cargo + ["metadata", "--format-version", "1"], cwd=project, env=env, text=True
        ))
        workspace_members = set(metadata["workspace_members"])
        for manifest_path in project.rglob("Cargo.toml"):
            manifest = tomllib.loads(manifest_path.read_text())
            sections = [manifest.get("dependencies", {}), manifest.get("build-dependencies", {}), manifest.get("workspace", {}).get("dependencies", {})]
            for section in sections:
                for name, dependency in section.items():
                    if name.startswith(("cu29", "cu-")) and isinstance(dependency, dict) and "version" in dependency:
                        assert dependency["version"].startswith("~"), (name, dependency)
        for package in metadata["packages"]:
            if package["name"].startswith(("cu29", "cu-")) and package["id"] not in workspace_members:
                assert package["source"] and package["source"].startswith("registry+"), package["name"]
        package = "hello_copper" if template == "project" else "cu_example_app"
        run(cargo + ["build", "--bins"], cwd=project, env=env)
        run(cargo + ["build", "-p", package, "--features", "sim-debug", "--bins"], cwd=project, env=env)
        app_dir = project if template == "project" else project / "apps" / "cu_example_app"
        app = Path(env["CARGO_TARGET_DIR"]) / "debug" / ("hello-copper" if template == "project" else package)
        with (output / f"{template}-run.log").open("w") as log:
            process = subprocess.Popen([str(app)], cwd=app_dir, stdout=log, stderr=log)
            try:
                time.sleep(2)
                if process.poll() is not None:
                    raise RuntimeError(f"{package} exited before the shutdown signal")
                process.send_signal(signal.SIGINT)
                if process.wait(timeout=15) != 0:
                    raise RuntimeError(f"{package} failed: see {log.name}")
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
        assert list((app_dir / "logs").glob("*.copper")), f"{package} did not record a Copper log"
        print(f"{template}: built registry dependencies, ran, recorded logs, and shut down successfully", flush=True)


if __name__ == "__main__":
    main()
