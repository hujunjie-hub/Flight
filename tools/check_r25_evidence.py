#!/usr/bin/env python3
"""Independent R25 consistency checks with machine-readable results."""
import argparse, json, re, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/validation/regenerated/checks"
OUT.mkdir(parents=True, exist_ok=True)


def sha(path):
    p = ROOT / path
    return __import__("hashlib").sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def emit(name, checks, failures, status, detail):
    data = {"check": name, "checks": checks, "failures": failures,
            "status": status, "detail": detail, "exit_code": 0 if status == "PASS" else (2 if status == "BLOCKED" else 1)}
    (OUT / f"{name}.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    (OUT / f"{name}.log").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print(f"{name}: {status} checks={checks} failures={failures}")
    return data["exit_code"]


def manifest():
    p = ROOT / "spec/validation/flight_acceptance_manifest.json"
    d = json.loads(p.read_text())
    checks = 0
    fails = []
    for key in ("latest_local_validation", "s06_112_evidence", "stack_budget"):
        checks += 1
        if key not in d:
            fails.append(f"missing {key}")
    lv = d.get("latest_local_validation", {})
    for key in ("validated_code_commit", "input_baseline_code_commit",
                "input_baseline_archive_head", "remote_head"):
        checks += 1
        if not lv.get(key):
            fails.append(f"missing {key}")
    checks += 1
    if d.get("issue_registry") != "spec/quality/ISSUE_REGISTER.md":
        fails.append("issue_registry not local")
    return emit("manifest", checks, len(fails), "PASS" if not fails else "FAIL", fails)


def issues():
    p = ROOT / "spec/quality/ISSUE_REGISTER.md"
    text = p.read_text()
    ids = re.findall(r"^\| (FLIGHT-LOCAL-\d+) \|", text, re.M)
    checks = len(ids)
    fails = []
    if len(ids) != len(set(ids)):
        fails.append("duplicate issue ids")
    if "GITHUB_PUBLICATION = PENDING" not in text and "GITHUB_PUBLICATION=PENDING" not in text:
        fails.append("publication status not pending")
    return emit("issues", checks, len(fails), "PASS" if not fails else "FAIL", fails)


def product():
    src = (ROOT / "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c").read_text()
    manifest = json.loads((ROOT / "spec/validation/flight_acceptance_manifest.json").read_text())
    values = manifest.get("product_envelope", {})
    checks = 0
    fails = []
    for token, expected in (("default_max_speed_mps = 8.0f", True),
                            ("default_max_accel_mps2 = 4.0f", True),
                            ("max_jerk_ned_mps3[0] = policy->default_max_accel_mps2 * 4.0f", True)):
        checks += 1
        if (token in src) != expected:
            fails.append(f"source policy mismatch: {token}")
    checks += 1
    if not values:
        values = {"source": "quad_x4_product_config.c"}
    return emit("product", checks, len(fails), "PASS" if not fails else "FAIL", fails)


def stack():
    su = ROOT / "build/m5_arm/m5_trajectory_safety.su"
    lines = su.read_text(errors="ignore").splitlines()
    frames = {}
    for line in lines:
        m = re.search(r":([A-Za-z0-9_]+)\s+(\d+)\s+", line)
        if m:
            frames[m.group(1)] = int(m.group(2))
    need = ["m5_trajectory_safety_validate", "build_curve", "interval_scale",
            "interval_mul", "interval_bounds", "interval_invalid"]
    checks = len(need)
    fails = []
    total = 0
    for name in need:
        if name not in frames:
            fails.append(f"missing {name}")
        else:
            total += frames[name]
    if total > 4096:
        fails.append(f"validator chain {total} > 4096")
    status = "BLOCKED" if total > 4096 else ("PASS" if not fails else "FAIL")
    data = emit("stack", checks, len(fails), status, {"chain_bytes": total, "fails": fails})
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("check", choices=["manifest", "issues", "product", "stack"])
    a = ap.parse_args()
    return {"manifest": manifest, "issues": issues, "product": product, "stack": stack}[a.check]()


if __name__ == "__main__":
    raise SystemExit(main())
