"""Verify that the RT-Thread package selection for the firmware link is pinned.

The H723 firmware link needs `packages/SConscript`, which the RT-Thread package
manager generates from `.config`. Every selected package must be locked to an
exact version and commit here; `VER="latest"` is not reproducible (spec/00 §4)
and makes `check_target_build.py` depend on whatever the package registry served
that day.

The verifier is intentionally strict:

* lock status must be RESOLVED with a named approver;
* every selected package needs a `pinned_version` (never "latest") and a 40-hex
  `pinned_commit`;
* `CONFIG_PKG_USING_<NAME>_LATEST_VERSION` must no longer be enabled;
* the `.config` version must equal the locked version;
* the lock must not carry packages that are no longer selected.

Exit codes: 0 = fully pinned, 1 = at least one problem, 2 = BLOCKED because
`.config` or the lock file is missing. Installation itself is not performed here.
"""
from pathlib import Path
import json
import re
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
LOCK = ROOT / "tools/rt_packages.lock.json"
CONFIG = ROOT / ".config"
OUT = ROOT / "build/m4_m5_implementation/evidence/rt_packages_lock.json"
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


def selected_packages(config_text):
    """Map CONFIG_PKG_USING_<NAME> -> 'latest' | 'custom'."""
    selection = {}
    for line in config_text.splitlines():
        match = re.fullmatch(r"CONFIG_PKG_USING_([A-Za-z0-9_]+)=y", line.strip())
        if not match:
            continue
        symbol = match.group(1)
        if symbol.endswith("_LATEST_VERSION") or symbol.endswith("_CUSTOM_VERSION"):
            continue
        latest_on = f"CONFIG_PKG_USING_{symbol}_LATEST_VERSION=y" in config_text
        selection[symbol] = "latest" if latest_on else "custom"
    return selection


def verify(config_text, lock):
    """Return the list of problems for a (.config, lock) pair."""
    problems = []
    status = lock.get("status")
    if status != "RESOLVED":
        problems.append(f"lock status is {status!r}, expected 'RESOLVED'")
    if status == "RESOLVED":
        if not lock.get("resolved_by"):
            problems.append("lock status is RESOLVED but resolved_by is empty")
        if not lock.get("resolved_at_utc"):
            problems.append("lock status is RESOLVED but resolved_at_utc is empty")

    packages = lock.get("packages") or []
    locked = {}
    for entry in packages:
        symbol = entry.get("symbol")
        if not symbol:
            problems.append("lock entry without a symbol")
            continue
        if symbol in locked:
            problems.append(f"{symbol}: duplicate lock entry")
        locked[symbol] = entry

    selection = selected_packages(config_text)
    for symbol, state in sorted(selection.items()):
        entry = locked.get(symbol)
        if entry is None:
            problems.append(f"{symbol}: selected in .config but absent from the lock")
            continue
        pinned = entry.get("pinned_version")
        if not pinned or pinned == "latest":
            problems.append(f"{symbol}: lock has no exact pinned_version (selected={state})")
        commit = entry.get("pinned_commit")
        if not commit or not COMMIT_RE.fullmatch(str(commit)):
            problems.append(f"{symbol}: pinned_commit is not a 40-hex commit")
        if state == "latest":
            problems.append(
                f"{symbol}: CONFIG_PKG_USING_{symbol}_LATEST_VERSION is still enabled")
        configured = re.search(
            rf'^CONFIG_PKG_{re.escape(symbol)}_VER="([^"]*)"', config_text, re.M)
        version = configured.group(1) if configured else None
        if version is None:
            problems.append(f"{symbol}: CONFIG_PKG_{symbol}_VER is missing in .config")
        elif pinned and version != pinned:
            problems.append(
                f"{symbol}: .config version {version!r} does not match pinned {pinned!r}")
    for symbol in sorted(locked):
        if symbol not in selection:
            problems.append(f"{symbol}: locked but no longer selected in .config")
    return problems


def main():
    if not CONFIG.is_file() or not LOCK.is_file():
        print(json.dumps({"status": "BLOCKED",
                          "reason": ".config or tools/rt_packages.lock.json missing"},
                         indent=2))
        return 2
    config_text = CONFIG.read_text(encoding="utf-8")
    lock = json.loads(LOCK.read_text(encoding="utf-8"))
    problems = verify(config_text, lock)
    report = {
        "status": "PASS" if not problems else "FAIL",
        "scope": "RT-THREAD_PACKAGES_FOR_FIRMWARE_LINK",
        "lock_status": lock.get("status"),
        "selected_packages": sorted(selected_packages(config_text)),
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
