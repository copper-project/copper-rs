"""Skip expensive CI only when every changed path is known documentation."""

import os
from pathlib import PurePosixPath
import subprocess


def needs_code_checks(paths):
    for path in paths:
        p = PurePosixPath(path)
        if p.suffix == ".md" or path.startswith("docs/"):
            continue
        if path in {"LICENSE", "NOTICE", ".github/CODEOWNERS"}:
            continue
        return True
    return not paths


def main():
    base = os.environ["BASE_SHA"]
    head = os.environ["HEAD_SHA"]
    if not base or set(base) == {"0"}:
        code = True
    else:
        # Checkout contains GitHub's PR merge commit (or the pushed commit).
        # Compare it with the event's base without cloning the full history.
        subprocess.run(["git", "fetch", "--no-tags", "--depth=1", "origin", base], check=True)
        args = ["git", "diff", "--name-only", "-z", base, head]
        paths = subprocess.check_output(args).decode().rstrip("\0").split("\0")
        code = needs_code_checks(paths)
    value = str(code).lower()
    print(f"Code checks required: {value}")
    with open(os.environ["GITHUB_OUTPUT"], "a") as output:
        output.write(f"code={value}\n")


if __name__ == "__main__":
    main()
