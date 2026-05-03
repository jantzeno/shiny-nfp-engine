#!/usr/bin/env python3
"""Verify checked-in C++ tests are either compiled or intentionally excluded."""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
XMAKE_FILE = REPO_ROOT / "xmake.lua"

INTENTIONALLY_EXCLUDED = {
    "tests/support/mtg_fixture.cpp": (
        "fixture implementation moved to tests/fixtures/export_surface/mtg_fixture.cpp"
    ),
}


def listed_test_sources() -> set[str]:
    text = XMAKE_FILE.read_text(encoding="utf-8")
    return set(re.findall(r'add_files\("((?:tests/)[^"]+\.cpp)"\)', text))


def checked_in_test_sources() -> set[str]:
    return {
        path.relative_to(REPO_ROOT).as_posix()
        for path in (REPO_ROOT / "tests").rglob("*.cpp")
    }


def main() -> int:
    listed = listed_test_sources()
    checked_in = checked_in_test_sources()
    excluded = set(INTENTIONALLY_EXCLUDED)

    missing = sorted(checked_in - listed - excluded)
    missing_exclusions = sorted(excluded - checked_in)
    listed_exclusions = sorted(listed & excluded)
    stale_listed = sorted(listed - checked_in)

    if not (missing or missing_exclusions or listed_exclusions or stale_listed):
        return 0

    print("test manifest mismatch:", file=sys.stderr)
    if missing:
        print("  checked-in tests not listed or excluded:", file=sys.stderr)
        for path in missing:
            print(f"    {path}", file=sys.stderr)
    if missing_exclusions:
        print("  exclusions for files that no longer exist:", file=sys.stderr)
        for path in missing_exclusions:
            print(f"    {path}", file=sys.stderr)
    if listed_exclusions:
        print("  tests are both listed and intentionally excluded:", file=sys.stderr)
        for path in listed_exclusions:
            print(f"    {path}", file=sys.stderr)
    if stale_listed:
        print("  listed tests that no longer exist:", file=sys.stderr)
        for path in stale_listed:
            print(f"    {path}", file=sys.stderr)

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
