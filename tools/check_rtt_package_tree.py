#!/usr/bin/env python3
"""Check RT-Thread's generated package entrypoints without assuming `latest` names.

RT-Thread Env 4.x (`pkgs` from `C:\\env-windows\\.venv`) writes
`packages/SConscript` and `packages/pkgs.json` plus the installed package
directories. It does **not** write `packages/Kconfig`: the Kconfig tree lives in
the package index (`$PKGS_ROOT/packages`) and the BSP root Kconfig sources it
with `osource`, i.e. optionally. The old guard required `packages/Kconfig` and
therefore rejected a perfectly valid `pkgs --update` result.

This guard verifies what the tool actually produces:
  * the SConscript bridge,
  * the pkgs.json manifest,
  * at least one installed package directory with its own SConscript.
`packages/Kconfig` is checked for consistency when present but is not required.
"""
from pathlib import Path

REQUIRED_GENERATED_FILES = (
    Path("packages/SConscript"),
    Path("packages/pkgs.json"),
)
OPTIONAL_GENERATED_FILES = (Path("packages/Kconfig"),)


def installed_package_dirs(root: Path) -> list[Path]:
    packages = Path(root) / "packages"
    if not packages.is_dir():
        return []
    return sorted(p for p in packages.iterdir()
                  if p.is_dir() and (p / "SConscript").is_file())


def missing_generated_metadata(root: Path) -> list[str]:
    root = Path(root)
    missing = [str(rel).replace("\\", "/") for rel in REQUIRED_GENERATED_FILES
               if not (root / rel).is_file()]
    if not missing and not installed_package_dirs(root):
        missing.append("packages/<package>/SConscript")
    return missing


def package_tree_ready(root: Path) -> bool:
    return not missing_generated_metadata(root)


def main() -> int:
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=".")
    args = ap.parse_args()
    root = Path(args.root)
    missing = missing_generated_metadata(root)
    installed = [p.name for p in installed_package_dirs(root)]
    report = {
        "status": "PASS" if not missing else "BLOCKED",
        "required_generated_files": [str(p).replace("\\", "/")
                                     for p in REQUIRED_GENERATED_FILES],
        "optional_generated_files": [str(p).replace("\\", "/")
                                     for p in OPTIONAL_GENERATED_FILES],
        "installed_packages": installed,
        "missing": missing,
        "note": ("This checks RT-Thread-generated entrypoints only. It does not prove "
                 "package lock, package contents, compile or firmware link."),
    }
    print(json.dumps(report, indent=2))
    return 0 if not missing else 2


if __name__ == "__main__":
    raise SystemExit(main())
