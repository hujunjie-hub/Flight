"""Produce the LF re-baseline proposal for CRLF-pinned spec/ABI evidence.

Nothing is rewritten: the script reads the committed blobs, proves that the
pinned hashes are the CRLF variants of the same content, and emits

  build/validation/rebaseline_proposal/proposal.json
  build/validation/rebaseline_proposal/abi_lock.diff
  build/validation/rebaseline_proposal/summary.md

so the ABI/spec owner can approve an exact change instead of a guess. The ABI
layout itself is compared through tests/abi/evidence/abi_compare.json when
present; a mismatch there would mean real ABI drift, not a line-ending issue.
"""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / "build/validation/rebaseline_proposal"
LOCK_HEADER = ROOT / "middleware/contracts/product_contract/abi_lock.h"
DIALECT = ROOT / "spec/validation/dialect_report.json"
ABI_COMPARE = ROOT / "tests/abi/evidence/abi_compare.json"
PINNED = (
    ("spec/contracts/FCU.xml", "spec_xml_sha256", DIALECT),
    ("spec/contracts/message_registry.json",
     "ABI_LOCK_MESSAGE_REGISTRY_SHA256", LOCK_HEADER),
)


def blob(path):
    return subprocess.check_output(["git", "cat-file", "blob", f"HEAD:{path}"], cwd=ROOT)


def eol_variants(data):
    return {"lf": hashlib.sha256(data).hexdigest(),
            "crlf": hashlib.sha256(data.replace(b"\n", b"\r\n")).hexdigest()}


def pinned_value(path, kind):
    if kind == "spec_xml_sha256":
        return json.loads(path.read_text(encoding="utf-8"))["spec_xml_sha256"]
    match = re.search(r'#define\s+' + kind + r'\s+"([0-9a-f]{64})"',
                      path.read_text(encoding="utf-8"))
    return match.group(1) if match else None


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"scope": "LF_REBASELINE_PROPOSAL", "status": "PROPOSED",
              "files": [], "abi": {}}
    for relative, kind, evidence in PINNED:
        data = blob(relative)
        variants = eol_variants(data)
        pinned = pinned_value(evidence, kind)
        report["files"].append({
            "path": relative,
            "pinned_field": f"{evidence.relative_to(ROOT).as_posix()}:{kind}",
            "pinned_sha256": pinned,
            "committed_blob_sha256_lf": variants["lf"],
            "crlf_variant_sha256": variants["crlf"],
            "pinned_matches": ("lf" if pinned == variants["lf"] else
                               "crlf" if pinned == variants["crlf"] else "none"),
            "proposed_sha256": variants["lf"],
        })

    if ABI_COMPARE.is_file():
        # Prefer the committed evidence: a locally regenerated report can be
        # STALE_BASELINE if the spec/layout evidence was rewritten by another
        # check, and that must not hide the real 143-type comparison.
        try:
            raw = blob("tests/abi/evidence/abi_compare.json")
            source = "HEAD:tests/abi/evidence/abi_compare.json"
        except subprocess.CalledProcessError:
            raw = ABI_COMPARE.read_bytes()
            source = ABI_COMPARE.relative_to(ROOT).as_posix()
        compare = json.loads(raw)
        layout = compare.get("spec_layout_comparison", {})
        report["abi"] = {
            "source": source,
            "compared_types": layout.get("compared_types"),
            "compared_field_offsets": layout.get("compared_field_offsets"),
            "missing_types": layout.get("missing_types"),
            "extra_types": layout.get("extra_types"),
            "mismatches": layout.get("mismatches"),
            "layout_status": layout.get("status"),
            "arm_hash": compare.get("abi_hashes", {}).get("arm"),
            "host_hash": compare.get("abi_hashes", {}).get("host"),
        }

    lock_text = LOCK_HEADER.read_text(encoding="utf-8")
    old = None
    match = re.search(r'(#define\s+ABI_LOCK_MESSAGE_REGISTRY_SHA256\s+")[0-9a-f]{64}(")',
                      lock_text)
    if match:
        old = re.search(r'"([0-9a-f]{64})"', match.group(0)).group(1)
    new = next((item["proposed_sha256"] for item in report["files"]
                if item["path"] == "spec/contracts/message_registry.json"), None)
    report["abi_lock_proposal"] = {"file": "middleware/contracts/product_contract/abi_lock.h",
                                   "macro": "ABI_LOCK_MESSAGE_REGISTRY_SHA256",
                                   "old": old, "new": new,
                                   "applied": False,
                                   "requires_approval": True}
    if old and new and match:
        diff = (
            "--- a/middleware/contracts/product_contract/abi_lock.h\n"
            "+++ b/middleware/contracts/product_contract/abi_lock.h\n"
            f"-#define ABI_LOCK_MESSAGE_REGISTRY_SHA256 \"{old}\"\n"
            f"+#define ABI_LOCK_MESSAGE_REGISTRY_SHA256 \"{new}\"\n"
        )
        (OUT / "abi_lock.diff").write_text(diff, encoding="utf-8")

    (OUT / "proposal.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    lines = ["# LF re-baseline proposal", "",
             "Status: PROPOSED. No tracked file was modified.", "",
             "| file | pinned | committed blob (LF) | CRLF variant | proposed |",
             "|---|---|---|---|---|"]
    for item in report["files"]:
        lines.append("| `{path}` | `{pinned}` ({match}) | `{lf}` | `{crlf}` | `{proposed}` |".format(
            path=item["path"], pinned=item["pinned_sha256"], match=item["pinned_matches"],
            lf=item["committed_blob_sha256_lf"], crlf=item["crlf_variant_sha256"],
            proposed=item["proposed_sha256"]))
    abi = report["abi"]
    if abi:
        lines += ["", "ABI layout comparison: {} types, {} field offsets, "
                      "{} mismatches, status {}.".format(
                          abi.get("compared_types"), abi.get("compared_field_offsets"),
                          len(abi.get("mismatches") or []), abi.get("layout_status"))]
    lines += ["", "After approval: regenerate `spec/validation/dialect_report.json` with "
                  "`python spec/tools/check_dialect.py` (pymavlink) and apply "
                  "`abi_lock.diff` through the ABI review."]
    (OUT / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((OUT / "summary.md").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
