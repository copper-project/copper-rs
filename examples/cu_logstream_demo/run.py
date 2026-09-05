#!/usr/bin/env python3
"""Drive real sender/receiver processes; inspect native archives with Rust tools."""

import argparse
import pathlib
import subprocess
import time

HERE = pathlib.Path(__file__).resolve().parent


def run(binary, scenario):
    directory = HERE / "logs" / f"{scenario}-{time.time_ns()}"
    directory.mkdir(parents=True)
    children = []

    def spawn(*args):
        child = subprocess.Popen([str(binary), *map(str, args)], cwd=HERE)
        children.append(child)
        return child

    def wait(child):
        code = child.wait(timeout=20)
        if code:
            raise RuntimeError(f"Demo process exited with {code}")

    def receiver(name, listen="127.0.0.1:0", impairment="clean", stop=False):
        ready = directory / f"{name}.endpoint"
        args = ["receiver", "--listen", listen, "--log-base", directory / f"{name}.copper",
                "--ready-file", ready, "--impairment", impairment]
        if stop:
            args += ["--stop-at", "64"]
        child = spawn(*args)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError("Receiver exited before becoming ready")
            if ready.exists() and (address := ready.read_text().strip()):
                return child, address
            time.sleep(0.01)
        raise TimeoutError("Receiver did not bind its socket")

    def verify(name, expectation):
        wait(spawn("verify", "--sender", directory / "sender.copper", "--received",
                   directory / f"{name}.copper", "--expect", expectation,
                   "--iterations", "1" if scenario == "idle" else "256"))

    try:
        if scenario == "idle":
            ground, address = receiver("received", impairment="bootstrap")
            sender = spawn("sender", "--remote", address, "--log-base", directory / "sender.copper",
                           "--iterations", "1", "--idle-ms", "1000")
        elif scenario == "late":
            # Reserve an endpoint using the actual receiver, then close it before
            # starting the sender. This receiver receives no data or manifest.
            initial, address = receiver("reservation")
            initial.terminate()
            initial.wait(timeout=5)
            sender = spawn("sender", "--remote", address, "--log-base", directory / "sender.copper")
            time.sleep(0.6)
            ground, _ = receiver("received", address)
        else:
            ground, address = receiver("received", impairment=scenario if scenario in ("loss", "outage") else "clean",
                                       stop=scenario == "restart")
            sender = spawn("sender", "--remote", address, "--log-base", directory / "sender.copper")
        if scenario == "restart":
            wait(ground)
            if sender.poll() is not None:
                raise RuntimeError("Sender finished before receiver restart")
            time.sleep(0.4)
            restarted, _ = receiver("restarted", address)
            wait(sender)
            wait(restarted)
            verify("received", "prefix")
            verify("restarted", "late")
            replay_names = ["received", "restarted"]
        else:
            wait(sender)
            wait(ground)
            verify("received", {"idle": "complete", "clean": "complete", "loss": "complete", "outage": "outage", "late": "late"}[scenario])
            replay_names = ["received"]

        logreader = binary.with_name("cu-logstream-demo-logreader")
        replay = binary.with_name("cu-logstream-demo-resim")
        for name in replay_names:
            subprocess.run([str(logreader), str(directory / f"{name}.copper"), "fsck"], check=True, timeout=20, cwd=HERE)
            subprocess.run([str(replay), "--log-base", str(directory / f"{name}.copper"),
                            "--replay-log-base", str(directory / f"{name}-replay.copper")], check=True, timeout=20, cwd=HERE)
        print(f"PASS {scenario}: {directory}", flush=True)
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=["clean", "loss", "outage", "late", "restart", "idle", "all"], default="clean", nargs="?")
    parser.add_argument("--binary", type=pathlib.Path, required=True)
    args = parser.parse_args()
    binary = args.binary.resolve()
    for scenario in (["clean", "loss", "outage", "late", "restart", "idle"] if args.scenario == "all" else [args.scenario]):
        run(binary, scenario)
