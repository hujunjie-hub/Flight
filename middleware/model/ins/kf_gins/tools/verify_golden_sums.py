"""Verify build/validation/kf_gins_golden/SHA256SUMS.txt against the files.

Exits non-zero and names every stale/missing entry.  Run from the Flight root:

    python middleware/model/ins/kf_gins/tools/verify_golden_sums.py
"""

import hashlib
import os
import sys

from pathlib import Path

ROOT = str(next(p for p in Path(__file__).resolve().parents if (p / "SConstruct").is_file()))
SUMS = os.path.join(ROOT, "build", "validation", "kf_gins_golden", "SHA256SUMS.txt")


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    if not os.path.exists(SUMS):
        print("missing manifest:", SUMS)
        return 1

    checked = 0
    stale = 0
    for raw in open(SUMS, encoding="utf-8"):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            print("malformed line:", line)
            stale += 1
            continue
        expected, rel = parts
        path = os.path.join(ROOT, rel.replace("/", os.sep))
        if not os.path.exists(path):
            print("MISSING", rel)
            stale += 1
            continue
        actual = sha256(path)
        checked += 1
        if actual != expected:
            print("STALE  ", rel, "expected", expected[:16], "actual", actual[:16])
            stale += 1

    print("golden manifest: %d files checked, %d stale/missing" % (checked, stale))
    return 0 if stale == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
