#!/usr/bin/env python3
"""Generate timeline/data.js from git history of all related repos."""

import subprocess
import json
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
MAIN_REPO  = SCRIPT_DIR.parent.parent          # micromouse/
REPO_BASE  = MAIN_REPO.parent           # Rust/

# filter_author=False  →  include every author's commits (with mine: true/false)
# filter_author=True   →  only include commits by Bananasmoothii (all get mine: true)
REPOS = [
    {
        "name":          "micromouse",
        "label":         "Micromouse",
        "path":          MAIN_REPO,
        "color":         "#818cf8",
        "filter_author": False,
    },
    {
        "name":          "vl53l0x",
        "label":         "VL53L0X driver",
        "path":          REPO_BASE / "vl53l0x",
        "color":         "#fb923c",
        "filter_author": True,
    },
    {
        "name":          "vl53l1",
        "label":         "VL53L1X driver",
        "path":          REPO_BASE / "vl53l1",
        "color":         "#34d399",
        "filter_author": True,
    },
    {
        "name":          "mpu9250",
        "label":         "MPU9250 driver",
        "path":          REPO_BASE / "mpu9250-forked",
        "color":         "#c084fc",
        "filter_author": True,
    },
]


def detect_main_branch(path: Path) -> str:
    for candidate in ("master", "main"):
        r = subprocess.run(["git", "rev-parse", "--verify", candidate],
                           cwd=path, capture_output=True)
        if r.returncode == 0:
            return candidate
    return ""


def git_hashes(path: Path, *extra_args: str) -> set[str]:
    r = subprocess.run(
        ["git", "log", "--format=%H", *extra_args],
        cwd=path, capture_output=True, text=True,
    )
    return set(r.stdout.strip().splitlines())


def get_commits(path: Path, filter_author: bool) -> list[dict]:
    author_args = ["--author=Bananasmoothii"] if filter_author else []

    try:
        result = subprocess.run(
            ["git", "log", "--all", "--format=%H|%ad|%an|%s",
             "--date=format:%Y-%m-%dT%H:%M:%S", *author_args],
            cwd=path, capture_output=True, text=True, check=True,
        )
    except subprocess.CalledProcessError as e:
        print(f"  git error: {e.stderr.strip()}", file=sys.stderr)
        return []

    # Commits reachable from the main branch (for the merged flag)
    mb = detect_main_branch(path)
    merged_hashes = git_hashes(path, mb, *author_args) if mb else set()

    # If showing all authors, we need to know which commits are by Bananasmoothii
    my_hashes: set[str] = set()
    if not filter_author:
        my_hashes = git_hashes(path, "--all", "--author=Bananasmoothii")

    commits = []
    for line in result.stdout.strip().splitlines():
        parts = line.split("|", 3)   # hash | date | author_name | subject (subject may contain |)
        if len(parts) == 4:
            full_hash, date, author, message = parts
            mine = filter_author or (full_hash in my_hashes)
            commits.append({
                "hash":    full_hash[:8],
                "date":    date,
                "author":  author,
                "message": message,
                "merged":  full_hash in merged_hashes,
                "mine":    mine,
            })
    return commits


def media_date(filepath: Path) -> str | None:
    name = filepath.name
    # PXL_YYYYMMDD_HHMMSSmmm (Google Photos)
    m = re.search(r"(202\d)(\d{2})(\d{2})[_\-T](\d{2})(\d{2})(\d{2})", name)
    if m:
        y, mo, d, h, mi, s = m.groups()
        return f"{y}-{mo}-{d}T{h}:{mi}:{s}"
    # Any 8-digit date: YYYYMMDD
    m = re.search(r"(202\d)(\d{2})(\d{2})", name)
    if m:
        y, mo, d = m.groups()
        return f"{y}-{mo}-{d}T12:00:00"
    # Fall back to file mtime
    try:
        from datetime import datetime, timezone
        mtime = filepath.stat().st_mtime
        dt = datetime.fromtimestamp(mtime, tz=timezone.utc)
        return dt.strftime("%Y-%m-%dT%H:%M:%S")
    except OSError:
        return None


def main():
    data: dict = {"repos": [], "media": []}

    for cfg in REPOS:
        p = cfg["path"]
        if not p.exists():
            print(f"  skip {cfg['name']}: {p} not found")
            continue
        commits = get_commits(p, filter_author=cfg["filter_author"])
        mine_count  = sum(1 for c in commits if c["mine"])
        other_count = len(commits) - mine_count
        label = cfg["label"]
        if other_count:
            print(f"  {label}: {mine_count} mine + {other_count} other = {len(commits)} commits")
        else:
            print(f"  {label}: {len(commits)} commits")
        data["repos"].append({
            "name":    cfg["name"],
            "label":   cfg["label"],
            "color":   cfg["color"],
            "commits": commits,
        })

    photo_exts = {".jpg", ".jpeg", ".png", ".webp", ".gif", ".avif"}
    video_exts = {".mp4", ".mov", ".webm", ".avi", ".mkv"}

    notes_file = SCRIPT_DIR / "notes.json"
    notes: dict[str, str] = {}
    if notes_file.exists():
        notes = json.loads(notes_file.read_text(encoding="utf-8"))

    media_dir = SCRIPT_DIR / "media"
    if media_dir.exists():
        found_files: set[str] = set()
        for f in sorted(media_dir.iterdir()):
            ext = f.suffix.lower()
            if ext in photo_exts:
                kind = "photo"
            elif ext in video_exts:
                kind = "video"
            else:
                continue
            date = media_date(f)
            if not date:
                print(f"  Skipped (no date): {f.name}")
                continue
            item: dict = {"path": f"media/{f.name}", "date": date, "type": kind}
            if f.name in notes:
                item["note"] = notes[f.name]
            data["media"].append(item)
            found_files.add(f.name)
            note_tag = " [note]" if f.name in notes else ""
            print(f"  {kind.capitalize()}: {f.name} -> {date}{note_tag}")

        for name in notes:
            if name not in found_files:
                print(f"  Note ignored (no media): {name}")

    js = (
        "// Auto-generated by generate_data.py — do not edit manually\n"
        f"const TIMELINE_DATA = {json.dumps(data, indent=2, ensure_ascii=False)};\n"
    )
    out = SCRIPT_DIR / "data.js"
    out.write_text(js, encoding="utf-8")
    print(f"\nWritten to {out}")


if __name__ == "__main__":
    main()
