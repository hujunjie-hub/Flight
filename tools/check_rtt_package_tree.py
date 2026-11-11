#!/usr/bin/env python3
"""Check RT-Thread's generated package entrypoints without assuming `latest` names.

RT-Thread package directory suffixes depend on the selected version. A pinned
package may not live under a `*-latest` directory, so the BSP must validate the
generated package metadata entrypoints instead of hard-coding latest paths.
"""
from pathlib import Path

REQUIRED_GENERATED_FILES = (
    Path("packages/SConscript"),
    Path("packages/Kconfig"),
)


def missing_generated_metadata(root: Path) -> list[str]:
    root = Path(root)
    return [str(rel).replace("\\", "/") for rel in REQUIRED_GENERATED_FILES
            if not (root / rel).is_file()]


def package_tree_ready(root: Path) -> bool:
    return not missing_generated_metadata(root)


def main() -> int:
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=".")
    args = ap.parse_args()
    missing = missing_generated_metadata(Path(args.root))
    report = {
        "status": "PASS" if not missing else "BLOCKED",
        "required_generated_files": [str(p).replace("\\", "/") for p in REQUIRED_GENERATED_FILES],
        "missing": missing,
        "note": "This checks RT-Thread-generated entrypoints only. It does not prove package lock, package contents, compile or firmware link.",
    }
    print(json.dumps(report, indent=2))
    return 0 if not missing else 2


if __name__ == "__main__":
    raise SystemExit(main())
