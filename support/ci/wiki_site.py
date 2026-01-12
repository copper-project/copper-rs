#!/usr/bin/env python3

"""Prepare the Copper GitHub wiki content for MkDocs."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
from pathlib import Path
from typing import Dict, Iterable, List

WIKI_REPO = "https://github.com/copper-project/copper-rs.wiki.git"
WIKI_WEB_URL = "https://github.com/copper-project/copper-rs/wiki/"

LINK_RE = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")


def _run(cmd: List[str]) -> None:
    subprocess.run(cmd, check=True)


def _clone_repo(repo_url: str, dest: Path) -> None:
    if dest.exists():
        shutil.rmtree(dest)
    _run(["git", "clone", "--depth", "1", repo_url, str(dest)])


def _page_map(wiki_dir: Path) -> Dict[str, str]:
    pages = {}
    for path in wiki_dir.glob("*.md"):
        if path.name.startswith("_"):
            continue
        pages[path.stem] = f"{path.stem}.md"
    if "Home" in pages:
        pages["Home"] = "index.md"
    return pages


def _rewrite_url(url: str, pages: Dict[str, str]) -> str:
    if url.startswith(WIKI_WEB_URL):
        url = url[len(WIKI_WEB_URL) :]

    if url.startswith(("http://", "https://", "mailto:", "ftp://", "data:")):
        return url
    if url.startswith(("#", "/")):
        return url

    if "#" in url:
        base, anchor = url.split("#", 1)
    else:
        base, anchor = url, ""

    if "/" not in base and base in pages:
        resolved = pages[base]
        if anchor:
            resolved = f"{resolved}#{anchor}"
        return resolved

    return url


def _rewrite_link_target(target: str, pages: Dict[str, str]) -> str:
    parts = target.split(maxsplit=1)
    url = parts[0]
    rest = f" {parts[1]}" if len(parts) > 1 else ""
    return f"{_rewrite_url(url, pages)}{rest}"


def _rewrite_markdown_links(text: str, pages: Dict[str, str]) -> str:
    def _replace(match: re.Match[str]) -> str:
        label = match.group(1)
        target = match.group(2).strip()
        normalized = label.strip().lower()
        if "bleeding edge api documentation" in normalized:
            return "[API](api/)"
        return f"[{label}]({_rewrite_link_target(target, pages)})"

    return LINK_RE.sub(_replace, text)


def _rewrite_markdown_files(docs_dir: Path, pages: Dict[str, str]) -> None:
    for path in docs_dir.rglob("*.md"):
        content = path.read_text(encoding="utf-8")
        updated = _rewrite_markdown_links(content, pages)
        if updated != content:
            path.write_text(updated, encoding="utf-8")


def _copy_wiki(src_dir: Path, docs_dir: Path) -> None:
    if docs_dir.exists():
        shutil.rmtree(docs_dir)
    shutil.copytree(src_dir, docs_dir, ignore=shutil.ignore_patterns(".git"))
    sidebar = docs_dir / "_Sidebar.md"
    if sidebar.exists():
        sidebar.unlink()


def _parse_sidebar(sidebar_path: Path, pages: Dict[str, str]) -> List[Dict[str, object]]:
    nav: List[Dict[str, object]] = []
    current_section: List[Dict[str, object]] | None = None

    for raw_line in sidebar_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue

        if line.startswith("**") and line.endswith("**"):
            section = line.strip("*").strip()
            current_section = []
            nav.append({section: current_section})
            continue

        match = re.match(r"- \[([^\]]+)\]\(([^)]+)\)", line)
        if not match:
            continue
        title = match.group(1).strip()
        target = match.group(2).strip()
        item = {title: _rewrite_link_target(target, pages)}
        if current_section is None:
            nav.append(item)
        else:
            current_section.append(item)

    return nav


def _yaml_quote(value: str) -> str:
    escaped = value.replace('"', '\\"')
    return f'"{escaped}"'


def _dump_nav(nav: Iterable[Dict[str, object]], indent: int = 0) -> List[str]:
    lines: List[str] = []
    prefix = "  " * indent
    for item in nav:
        for key, value in item.items():
            quoted_key = _yaml_quote(str(key))
            if isinstance(value, list):
                lines.append(f"{prefix}- {quoted_key}:")
                lines.extend(_dump_nav(value, indent + 1))
            else:
                lines.append(
                    f"{prefix}- {quoted_key}: {_yaml_quote(str(value))}"
                )
    return lines


def _write_mkdocs_config(
    path: Path, nav: List[Dict[str, object]], site_url: str
) -> None:
    lines = [
        "site_name: Copper Wiki",
        f"site_url: {site_url}",
        "repo_url: https://github.com/copper-project/copper-rs",
        "docs_dir: docs",
        "site_dir: site",
        "use_directory_urls: true",
        "theme:",
        "  name: readthedocs",
        "markdown_extensions:",
        "  - tables",
        "  - fenced_code",
        "  - toc:",
        "      permalink: true",
        "nav:",
    ]
    lines.extend(_dump_nav(nav, indent=1))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _resolve_workdir(repo_root: Path, workdir: str) -> Path:
    candidate = Path(workdir)
    if candidate.is_absolute():
        return candidate
    return repo_root / candidate


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Prepare the GitHub wiki content for MkDocs."
    )
    parser.add_argument(
        "--repo",
        default=WIKI_REPO,
        help="Git URL for the wiki repository.",
    )
    parser.add_argument(
        "--site-url",
        default="https://copper-project.github.io/copper-rs/",
        help="Published site URL for MkDocs metadata.",
    )
    parser.add_argument(
        "--workdir",
        default="build/wiki",
        help="Working directory (relative to repo root).",
    )

    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    workdir = _resolve_workdir(repo_root, args.workdir)

    if workdir.exists():
        shutil.rmtree(workdir)
    workdir.mkdir(parents=True)

    src_dir = workdir / "src"
    docs_dir = workdir / "docs"

    _clone_repo(args.repo, src_dir)
    pages = _page_map(src_dir)

    _copy_wiki(src_dir, docs_dir)

    home = docs_dir / "Home.md"
    if home.exists():
        home.rename(docs_dir / "index.md")

    _rewrite_markdown_files(docs_dir, pages)

    sidebar_path = src_dir / "_Sidebar.md"
    if sidebar_path.exists():
        nav = _parse_sidebar(sidebar_path, pages)
    else:
        nav = [{name: path} for name, path in sorted(pages.items())]

    _write_mkdocs_config(workdir / "mkdocs.yml", nav, args.site_url)


if __name__ == "__main__":
    main()
