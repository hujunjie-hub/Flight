"""Verify that the RT-Thread package selection for the firmware link is pinned.

The H723 firmware link needs `packages/SConscript`, which the RT-Thread package
manager generates from `.config`. Every selected package must be locked to an
exact version here; `VER="latest"` is not reproducible (spec/00 §4) and makes
`check_target_build.py` depend on whatever the package registry served that day.

This check does not install anything. Exit codes: 0 = fully pinned,
1 = at least one selected package is unresolved or the lock disagrees with
.config, 2 = BLOCKED because `.config` has no package selection.
"""
from pathlib import Path
import json
import re
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
LOCK = ROOT / "tools/rt_packages.lock.json"
CONFIG = ROOT / ".config"
OUT = ROOT / "build/m4_m5_implementation/evidence/rt_packages_lock.json"


def selected_packages(config_text):
    selection = {}
    for line in config_text.splitlines():
        match = re.fullmatch(r"CONFIG_PKG_USING_([A-Za-z0-9_]+)=y", line.strip())
        if not match:
            continue
        symbol = match.group(1)
        if symbol.endswith("_LATEST_VERSION") or symbol.endswith("_CUSTOM_VERSION"):
            continue
        selection[symbol] = "latest" if f"CONFIG_PKG_USING_{symbol}_LATEST_VERSION=y" in config_text else "pinned"
    return selection


def main():
    if not CONFIG.is_file() or not LOCK.is_file():
        print(json.dumps({"status": "BLOCKED",
                          "reason": ".config or tools/rt_packages.lock.json missing"},
                         indent=2))
        return 2
    config_text = CONFIG.read_text(encoding="utf-8")
    lock = json.loads(LOCK.read_text(encoding="utf-8"))
    locked = {entry["symbol"]: entry for entry in lock.get("packages", [])}
    selection = selected_packages(config_text)

    problems = []
    for symbol, state in sorted(selection.items()):
        entry = locked.get(symbol)
        if entry is None:
            problems.append(f"{symbol}: selected in .config but absent from the lock")
            continue
        pinned = entry.get("pinned_version")
        if not pinned or pinned == "latest":
            problems.append(f"{symbol}: lock has no exact pinned_version (selected={state})")
            continue
        configured = re.search(
            rf'^CONFIG_PKG_{re.escape(symbol)}_VER="([^"]*)"', config_text, re.M)
        if not configured or configured.group(1) != pinned:
            actual = configured.group(1) if configured else "<missing>"
            problems.append(
                f"{symbol}: .config version {actual!r} does not match pinned {pinned!r}")
    for symbol in locked:
        if symbol not in selection:
            problems.append(f"{symbol}: locked but no longer selected in .config")

    report = {
        "status": "PASS" if not problems else "FAIL",
        "scope": "RT-THREAD_PACKAGES_FOR_FIRMWARE_LINK",
        "lock_status": lock.get("status"),
        "selected_packages": sorted(selection),
        "problems": problems,
        "note": "A pass only means the selection is pinned; installing packages/ "
                "still requires the RT-Thread env tool as documented in "
                "spec/validation/rt_thread_packages_reproducibility.md.",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
