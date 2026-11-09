"""Refresh the KF-GINS upstream lock from the verified local import.

Run from the Flight root (the script only reads the imported files and the
recorded provenance values; it never touches the network):

    python middleware/model/ins/kf_gins/tools/gen_locks.py

Regenerating the hashes is the intended way to detect an accidental edit of the
vendored upstream copy.
"""

import hashlib
import json
import os
import sys

from pathlib import Path

PKG = Path(__file__).resolve().parents[1]

PROVENANCE = {
    "repository": "https://github.com/i2Nav-WHU/KF-GINS",
    "commit": "8291a93e49de513fe9d21f819500d39082ded611",
    "tree": "6d255a9c2c909ec9348a4bd37ec8eaf6f33f50b7",
    "commit_date": "2026-06-10T12:39:11+08:00",
    "author": "liqiang <wlq@whu.edu.cn>",
    "subject": "fix a bug of gnss velocity equation in the algorithm docs",
    "branches_containing_commit": ["origin/main"],
    "status_porcelain": "",
    "submodules": [],
    "archive": {
        "command": "git archive --format=tar.gz --prefix=KF-GINS-8291a93e49de513fe9d21f819500d39082ded611/ "
                   "8291a93e49de513fe9d21f819500d39082ded611",
        "format": "tar.gz",
        "bytes": 56254907,
        "sha256": "183a86c099126412b0d20a0f72de1625d9ecc73f01b5bcab16f69b6221a30dd7",
        "note": "includes ThirdParty/ (Eigen, yaml-cpp, abseil); those trees are NOT imported",
    },
    "license": {"spdx": "GPL-3.0-or-later", "file": "LICENSE",
                "sha256": "230184f60bae2feaf244f10a8bac053c8ff33a183bcc365b4d8b876d2b7f4809"},
}

IMPORTED = [
    ("src/kf-gins/gi_engine.cpp", "upstream/src/kf-gins/gi_engine.cpp"),
    ("src/kf-gins/gi_engine.h", "upstream/src/kf-gins/gi_engine.h"),
    ("src/kf-gins/insmech.cpp", "upstream/src/kf-gins/insmech.cpp"),
    ("src/kf-gins/insmech.h", "upstream/src/kf-gins/insmech.h"),
    ("src/kf-gins/kf_gins_types.h", "upstream/src/kf-gins/kf_gins_types.h"),
    ("src/common/angle.h", "upstream/src/common/angle.h"),
    ("src/common/earth.h", "upstream/src/common/earth.h"),
    ("src/common/logging.h", "upstream/src/common/logging.h"),
    ("src/common/rotation.h", "upstream/src/common/rotation.h"),
    ("src/common/types.h", "upstream/src/common/types.h"),
]

# Values recorded in spec/validation/upstream_sources.json (the spec baseline).
RECORDED = {
    "src/kf-gins/gi_engine.cpp": "efbbf8a66a189b6aae3e4fb063cc9c6af6dac66d9ff38f14ab61c690e864ba2d",
    "src/kf-gins/gi_engine.h": "5579f2aecc6527bd2267b5b97217b11dd50bb281cd748208bd2bdca13dd0c750",
    "src/kf-gins/insmech.cpp": "8a910e0dab271d288a1be66c23436d7bcbf454d6db89a074b327c7a661fe3c9f",
    "src/kf-gins/insmech.h": "9da6bdc445d29b85674077ac47ce20e6ea41cc7334f639e768edb03b4c950246",
    "src/kf-gins/kf_gins_types.h": "752e92f544efc28c691f1d2954d4fb3262ba89b02b0902e657e5c01d51520dae",
    "src/common/angle.h": "ad2a7a9b3a90d6222f53c227683d762f034408bd6cdd8c5668123c89ebc35eb0",
    "src/common/earth.h": "31970d453771eefd8a892c33a43dd7b80fa9c29747b8ab62d4cd5a6965896b9a",
    "src/common/logging.h": "18fdb3481ef16954bcc3210d409f3158c18bad7a18082e184dc378472dbc9957",
    "src/common/rotation.h": "f2fefd9415f8c849e5611d25bbe97646e97d6bad79523b54fe6f446f5d89d2b0",
    "src/common/types.h": "d30b49b09f39ab4b9c225c295c421fa4571f381576567793c1598eb612415aeb",
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 16), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_upstream_lock():
    files = []
    for upstream_path, local_path in IMPORTED:
        local = os.path.join(PKG, local_path.replace("/", os.sep))
        actual = sha256(local)
        recorded = RECORDED[upstream_path]
        files.append({
            "upstream_path": upstream_path,
            "local_path": local_path,
            "sha256": actual,
            "spec_recorded_sha256": recorded,
            "matches_spec_record": actual == recorded,
        })
    return {
        "schema": "kf-gins-upstream-lock/1",
        "generated_by": "middleware/model/ins/kf_gins/tools/gen_locks.py",
        "generated_at": "2026-11-04",
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3",
        "import_status": "IMPORTED_CONTROLLED_TRIM",
        "import_note": "Pristine files are byte-identical to the locked commit. The 22-state firmware "
                       "adaptation lives in middleware/model/ins/kf_gins/trimmed_core and must not be described as upstream.",
        **PROVENANCE,
        "files": files,
        "not_imported": {
            "src/kf_gins.cpp": "desktop entry point (file IO, console, YAML)",
            "src/fileio/*": "desktop data set loader/saver",
            "CMakeLists.txt": "desktop CMake build; firmware uses SCons",
            "ThirdParty/eigen-3.3.9": "upstream reference dependency only; H723 dependency decision is "
                                      "PLATFORM-owned (spec 00 §12.2, Eigen 5.0.1 candidate)",
            "ThirdParty/yaml-cpp-0.7.0": "desktop configuration parsing only",
            "ThirdParty/abseil-cpp-20220623.1": "desktop tooling only",
        },
        "third_party_reference": [
            {"name": "Eigen", "version": "3.3.9", "license": "MPL-2.0",
             "upstream_path": "ThirdParty/eigen-3.3.9", "imported": False,
             "note": "used only to build the pristine upstream Host reference in this session"},
            {"name": "yaml-cpp", "version": "0.7.0", "license": "MIT",
             "upstream_path": "ThirdParty/yaml-cpp-0.7.0", "imported": False,
             "license_sha256": "c06fd4ee9241e3b6841fcee5d471475be412c822ddf2876af17127acd426c011"},
            {"name": "Abseil C++", "version": "20220623.1", "license": "Apache-2.0",
             "upstream_path": "ThirdParty/abseil-cpp-20220623.1", "imported": False,
             "license_sha256": "f54fff0b905df5b3464527c652a30e903b172d6dcab4d89b5e6f105d5e4a4603"},
        ],
    }


def main():
    lock = build_upstream_lock()
    mismatches = [entry for entry in lock["files"] if not entry["matches_spec_record"]]
    if mismatches:
        for entry in mismatches:
            print("SHA-256 mismatch:", entry["local_path"], file=sys.stderr)
        return 1

    with (PKG / "UPSTREAM_LOCK.json").open("w", encoding="utf-8") as handle:
        json.dump(lock, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("UPSTREAM_LOCK.json: %d verified files" % len(lock["files"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
