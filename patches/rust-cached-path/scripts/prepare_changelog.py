import os
from datetime import datetime
from pathlib import Path

TAG = os.environ["TAG"]


def main():
    changelog = Path("CHANGELOG.md")

    with changelog.open() as f:
        lines = f.readlines()

    insert_index: int
    for i in range(len(lines)):
        line = lines[i]
        if line.startswith("## Unreleased"):
            insert_index = i + 1
        elif line.startswith(f"## [{TAG}]"):
            print("CHANGELOG already up-to-date")
            return
        elif line.startswith("## [v"):
            break
    else:
        raise RuntimeError("Couldn't find 'Unreleased' section")

    lines.insert(insert_index, "\n")
    lines.insert(
        insert_index + 1,
        f"## [{TAG}](https://github.com/epwalsh/rust-cached-path/releases/tag/{TAG}) - "
        f"{datetime.now().strftime('%Y-%m-%d')}\n",
    )

    with changelog.open("w") as f:
        f.writelines(lines)


if __name__ == "__main__":
    main()
