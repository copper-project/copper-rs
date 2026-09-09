#!/usr/bin/env python3
"""Drive real sender/receiver processes; inspect native archives with Rust tools."""

import argparse
import pathlib
import socket
import subprocess
import time

HERE = pathlib.Path(__file__).resolve().parent


def run(binary, scenario, two_way=False):
    mode = "two-way" if two_way else "one-way"
    directory = HERE / "logs" / f"{mode}-{scenario}-{time.time_ns()}"
    directory.mkdir(parents=True)
    children = []
    reservation = None
    if two_way:
        reservation = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reservation.bind(("127.0.0.1", 0))
        host, port = reservation.getsockname()
        robot_address = f"{host}:{port}"

    def spawn(*args):
        if two_way and args[0] == "sender":
            reservation.close()
            args = (*args, "--bind", robot_address)
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
        if two_way:
            args += ["--feedback-to", robot_address]
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
                   "--iterations", "1" if scenario == "idle" else "256",
                   *(["--require-robot-logs"] if scenario in ("clean", "loss") else [])))

    try:
        if scenario == "idle":
            ground, address = receiver("received", impairment="bootstrap")
            sender = spawn("sender", "--remote", address, "--log-base", directory / "sender.copper",
                           "--iterations", "1", "--idle-ms", "1000")
        elif scenario == "late":
            # Observe a real prefix with a separate receiver. A launch-time sleep
            # can join while CL0 is still retained and recover it, so it does not
            # establish the missing history required by this scenario.
            initial, address = receiver("reservation", stop=True)
            sender = spawn("sender", "--remote", address, "--log-base", directory / "sender.copper")
            wait(initial)
            if sender.poll() is not None:
                raise RuntimeError("Sender finished before late receiver startup")
            # This fresh process gets no archive or decoder state from the probe.
            ground, _ = receiver("received", address)
        else:
            ground, address = receiver("received", impairment=scenario if scenario in ("loss", "lossy", "outage") else "clean",
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
            verify("received", {"idle": "complete", "clean": "complete", "loss": "complete", "lossy": "lossy", "outage": "outage", "late": "late"}[scenario])
            replay_names = ["received"]

        logreader = binary.with_name("cu-logstream-demo-logreader")
        replay = binary.with_name("cu-logstream-demo-resim")
        for name in replay_names:
            subprocess.run([str(logreader), str(directory / f"{name}.copper"), "fsck"], check=True, timeout=20, cwd=HERE)
            subprocess.run([str(replay), "--log-base", str(directory / f"{name}.copper"),
                            "--replay-log-base", str(directory / f"{name}-replay.copper")], check=True, timeout=20, cwd=HERE)
        print(f"PASS {mode} {scenario}: {directory}", flush=True)
    finally:
        if reservation is not None:
            reservation.close()
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
    parser.add_argument("scenario", choices=["clean", "loss", "lossy", "outage", "late", "restart", "idle", "all"], default="clean", nargs="?")
    parser.add_argument("--binary", type=pathlib.Path, required=True)
    parser.add_argument("--two-way", action="store_true", help="Return receiver reports to a feedback-enabled sender")
    args = parser.parse_args()
    binary = args.binary.resolve()
    for scenario in (["clean", "loss", "lossy", "outage", "late", "restart", "idle"] if args.scenario == "all" else [args.scenario]):
        run(binary, scenario, args.two_way)
