#!/usr/bin/env python3
"""Negative control for the Flight ABI checker (T191-A-ABI evidence).

The ABI tool is only trustworthy if it actually detects layout drift.  This
control copies the production headers to a scratch directory, changes one
field width in NavStateCore, runs the same checker against the mutated copy,
and asserts that the result is FAIL with concrete size/offset mismatches.

It never touches the real headers and removes the scratch tree afterwards.
Run from the Flight project root:
  python tests/abi/negative_control.py
"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
sys.path.insert(0, str(HERE))
import abi_report as abi  # noqa: E402

SCRATCH = HERE / "_negative_control_tmp"
MUTATION_BEFORE = """typedef struct {
  uint32_t valid_mask;
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float acceleration_ned_mps2[3];
  float q_nb_wxyz[4];"""
MUTATION_AFTER = """typedef struct {
  uint64_t valid_mask;
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float acceleration_ned_mps2[3];
  float q_nb_wxyz[4];"""


def main():
    if SCRATCH.exists():
        shutil.rmtree(SCRATCH)
    contracts = SCRATCH / abi.CONTRACTS.relative_to(ROOT)
    for header in [*abi.HEADERS, abi.LOCK_FILE, abi.CONTRACTS / "product_contract/abi_lock.h"]:
        copy = SCRATCH / header.relative_to(ROOT)
        copy.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(header, copy)

    base_header = contracts / "base_types/common_base.h"
    text = base_header.read_text(encoding="utf-8")
    if MUTATION_BEFORE not in text:
        raise SystemExit("negative control: NavStateCore anchor not found")
    base_header.write_text(text.replace(MUTATION_BEFORE, MUTATION_AFTER, 1),
                           encoding="utf-8", newline="\n")

    abi.CONTRACTS = contracts
    abi.HEADERS = [SCRATCH / header.relative_to(ROOT) for header in abi.HEADERS]
    abi.EVIDENCE = SCRATCH / "evidence"
    abi.LOCK_FILE = SCRATCH / abi.LOCK_FILE.relative_to(ROOT)

    sys.argv = ["abi_report.py"]
    try:
        code = abi.main()
    finally:
        pass

    compare = json.loads((SCRATCH / "evidence" / "abi_compare.json").read_text(encoding="utf-8"))
    mismatches = compare["spec_layout_comparison"]["mismatches"]
    nav = [m for m in mismatches if m["type"] == "NavStateCore"]
    lock_problems = compare.get("abi_lock_check", {}).get("problems", [])
    ok = (code == 1 and compare["status"] == "FAIL"
          and (bool(mismatches) or bool(lock_problems)))
    summary = {
        "negative_control": "PASS" if ok else "FAIL",
        "checker_exit_code": code,
        "checker_status": compare["status"],
        "mismatch_count": len(mismatches),
        "lock_problems": len(lock_problems),
        "navstate_mismatches": nav[:4],
    }
    print(json.dumps(summary, ensure_ascii=False))
    shutil.rmtree(SCRATCH, ignore_errors=True)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
