"""Sample CI resources every 15 seconds and publish observed minimum headroom."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


ROOT = Path(os.environ.get("GITHUB_WORKSPACE", ".")).resolve()
STATE = ROOT / ".ci-resources"
GIB = 1024**3


def sample():
    result = {"disk_free_gib": shutil.disk_usage(ROOT).free / GIB}
    if Path("/proc/meminfo").exists():
        memory = dict(line.split(":", 1) for line in Path("/proc/meminfo").read_text().splitlines())
        result["memory_available_gib"] = int(memory["MemAvailable"].split()[0]) * 1024 / GIB
    with (STATE / "samples.jsonl").open("a") as output:
        output.write(json.dumps(result) + "\n")


def main():
    phase = sys.argv[1]
    if phase == "start":
        STATE.mkdir(exist_ok=True)
        (STATE / "started").write_text(str(time.time()))
        (STATE / "stop").unlink(missing_ok=True)
        (STATE / "samples.jsonl").write_text("")
        sample()
        subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "monitor"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    elif phase == "monitor":
        while not (STATE / "stop").exists():
            time.sleep(15)
            if not (STATE / "stop").exists():
                sample()
    elif phase == "finish":
        if not (STATE / "started").exists():
            return
        (STATE / "stop").touch()
        sample()
        samples = [json.loads(line) for line in (STATE / "samples.jsonl").read_text().splitlines()]
        elapsed = (time.time() - float((STATE / "started").read_text())) / 60
        lines = ["### CI resource measurements", "", f"Measured job duration: {elapsed:.1f} minutes", "",
                 "| Resource | Start | Minimum sampled | Finish |", "| --- | ---: | ---: | ---: |"]
        for key, label in [("disk_free_gib", "Free workspace disk (GiB)"),
                           ("memory_available_gib", "Available Linux memory (GiB)")]:
            if key in samples[0]:
                lines.append(f"| {label} | {samples[0][key]:.2f} | {min(s[key] for s in samples):.2f} | {samples[-1][key]:.2f} |")
        lines.extend(["", "Samples are taken every 15 seconds, after checkout through the final step. Container pulls and post-job cache saves are outside this measurement."])
        report = "\n".join(lines) + "\n"
        print(report)
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as output:
            output.write(report)
    else:
        raise ValueError(f"Unknown phase: {phase}")


if __name__ == "__main__":
    main()
