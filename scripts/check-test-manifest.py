#!/usr/bin/env python3
"""Verify checked-in C++ tests are either compiled or intentionally excluded."""

from __future__ import annotations

import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
XMAKE_FILE = REPO_ROOT / "xmake.lua"

INTENTIONALLY_EXCLUDED = {
    "tests/regression/genetic_search_observer.cpp": (
        "legacy jostle/genetic observer API removed from current search surface"
    ),
    "tests/regression/masonry_trace.cpp": (
        "legacy masonry search API removed from current search surface"
    ),
    "tests/regression/search_observer.cpp": (
        "legacy jostle observer API removed from current search surface"
    ),
    "tests/unit/benchmark_contract.cpp": (
        "legacy benchmark support headers removed from current tool surface"
    ),
    "tests/regression/concave_bin_concave_piece.cpp": (
        "legacy nonconvex NFP API removed from current NFP surface"
    ),
    "tests/regression/degenerate_triangle_pair.cpp": (
        "legacy convex NFP result API removed from current NFP surface"
    ),
    "tests/regression/l_shape_sliding.cpp": (
        "legacy nonconvex NFP API removed from current NFP surface"
    ),
    "tests/unit/convex_nfp.cpp": (
        "legacy convex IFP/NFP result API removed from current NFP surface"
    ),
    "tests/unit/decomposition.cpp": (
        "legacy decomposition engine/cache API removed from current decomposition surface"
    ),
    "tests/unit/decomposition_headers.cpp": (
        "legacy generic cache-store header contract removed from current cache surface"
    ),
    "tests/unit/genetic_search.cpp": (
        "legacy genetic search API removed from current production-search surface"
    ),
    "tests/unit/masonry.cpp": (
        "legacy masonry packing API removed from current packing surface"
    ),
    "tests/unit/masonry_search.cpp": (
        "legacy masonry search API removed from current search surface"
    ),
    "tests/unit/nonconvex_fixtures.cpp": (
        "legacy nonconvex NFP API removed from current NFP surface"
    ),
    "tests/unit/nonconvex_nfp.cpp": (
        "legacy nonconvex NFP/engine API removed from current NFP surface"
    ),
    "tests/unit/nfp_cache.cpp": (
        "legacy generic cache-store NFP overloads removed from current cache surface"
    ),
    "tests/unit/nfp_headers.cpp": (
        "legacy milestone header contract references removed NFP/cache API surface"
    ),
    "tests/unit/orbital_verifier.cpp": (
        "legacy orbital verifier API removed from current NFP surface"
    ),
    "tests/unit/packing.cpp": (
        "legacy constructive decoder API removed from current packing surface"
    ),
    "tests/unit/placement.cpp": (
        "legacy placement engine API removed from current placement surface"
    ),
    "tests/unit/profile_decode_benchmark_contract.cpp": (
        "legacy benchmark support headers removed from current tool surface"
    ),
    "tests/unit/search.cpp": (
        "legacy jostle search API removed from current search surface"
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
