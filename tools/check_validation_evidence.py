#!/usr/bin/env python3
"""Generic R27 evidence checks: manifest, issues, product, stack callgraph."""
import argparse, hashlib, json, re, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/validation/regenerated/checks"
OUT.mkdir(parents=True, exist_ok=True)


def sha(path):
    p = ROOT / path
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def emit(name, checks, failures, status, detail):
    data = {"check": name, "checks": checks, "failures": failures,
            "status": status, "detail": detail,
            "exit_code": 0 if status == "PASS" else (2 if status == "BLOCKED" else 1)}
    (OUT / f"{name}.json").write_text(json.dumps(data, indent=2) + "\n")
    (OUT / f"{name}.log").write_text(json.dumps(data, indent=2) + "\n")
    print(f"{name}: {status} checks={checks} failures={failures}")
    return data["exit_code"]


def manifest():
    d = json.loads((ROOT / "spec/validation/flight_acceptance_manifest.json").read_text())
    lv = d.get("latest_local_validation", {})
    checks = 0; fails = []
    for key, expected in (("run_id", "M5-06-R27-20261111"),
                          ("validated_code_commit", "b2a4c3779e9a2ccc22aebe209dce435b74ccc7d6"),
                          ("branch", "opt/stage-02-m5-06-safety-validator")):
        checks += 1
        if lv.get(key) != expected:
            fails.append(f"{key}: {lv.get(key)} != {expected}")
    checks += 1
    if d.get("issue_registry") != "spec/quality/ISSUE_REGISTER.md":
        fails.append("issue_registry mismatch")
    return emit("manifest", checks, len(fails), "PASS" if not fails else "FAIL", fails)


def issues():
    text = (ROOT / "spec/quality/ISSUE_REGISTER.md").read_text()
    ids = re.findall(r"^\| (FLIGHT-LOCAL-\d+) \|", text, re.M)
    checks = len(ids); fails = []
    if len(ids) != len(set(ids)): fails.append("duplicate ids")
    if "GITHUB_PUBLICATION = PENDING" not in text and "GITHUB_PUBLICATION=PENDING" not in text:
        fails.append("publication pending missing")
    for n in range(35, 42):
        checks += 1
        if f"FLIGHT-LOCAL-{n}" not in text: fails.append(f"missing {n}")
    return emit("issues", checks, len(fails), "PASS" if not fails else "FAIL", fails)


def product():
    src = (ROOT / "middleware/model/fms/quad_x4/flight_policy/quad_x4_product_config.c").read_text()
    checks = 0; fails = []
    for token in ("default_max_speed_mps = 8.0f", "default_max_accel_mps2 = 4.0f",
                  "max_jerk_ned_mps3[0] = policy->default_max_accel_mps2 * 4.0f",
                  "valid_mask = 0x7Fu"):
        checks += 1
        if token not in src: fails.append(f"missing {token}")
    return emit("product", checks, len(fails), "PASS" if not fails else "FAIL", fails)


def parse_disassembly():
    dis = ROOT / "build/m5_arm/m5_trajectory_safety.dis"
    if not dis.is_file():
        exe = "arm-none-eabi-objdump.exe"
        subprocess.check_call([exe, "-d", str(ROOT / "build/m5_arm/m5_trajectory_safety.o")],
                              stdout=dis.open("w"), cwd=ROOT)
    text = dis.read_text(errors="ignore")
    funcs = {}; cur = None; addr_to_name = {}
    for line in text.splitlines():
        m = re.match(r"^([0-9a-f]+) <([^>]+)>:", line)
        if m:
            cur = m.group(2); funcs[cur] = {"sp": 0, "calls": []}; addr_to_name[int(m.group(1), 16)] = cur
            continue
        if cur is None:
            continue
        if re.search(r"\bsubw?\s+sp,\s*(sp,\s*)?#\d+", line):
            n = int(re.search(r"#(\d+)", line).group(1)); funcs[cur]["sp"] += n
        for pat, word in ((r"\bpush\s*\{([^}]*)\}", 4),
                          (r"\bvpush\s*\{([^}]*)\}", 8),
                          (r"\bstmdb\s+sp!,\s*\{([^}]*)\}", 4)):
            mm = re.search(pat, line)
            if mm:
                regs = re.findall(r"r\d+|lr|pc|d\d+", mm.group(1))
                funcs[cur]["sp"] += word * len(regs)
        m = re.search(r"\bbl\s+([0-9a-f]+)\s+<([^>]+)>", line)
        if m:
            funcs[cur]["calls"].append(m.group(2))
    return funcs


def stack():
    funcs = parse_disassembly()
    root = "m5_trajectory_safety_validate"
    if root not in funcs:
        return emit("stack", 0, 1, "UNKNOWN", ["root function missing"])
    best = {"bytes": -1, "path": []}
    def walk(name, total, path):
        nonlocal best
        if name not in funcs:
            return
        total += funcs[name]["sp"]; path = path + [name]
        if total > best["bytes"]:
            best = {"bytes": total, "path": path}
        for child in funcs[name]["calls"]:
            if child in funcs and child not in path:
                walk(child, total, path)
    walk(root, 0, [])
    su = {}
    for line in (ROOT / "build/m5_arm/m5_trajectory_safety.su").read_text(errors="ignore").splitlines():
        m = re.search(r":([A-Za-z0-9_]+)\s+(\d+)\s+", line)
        if m: su[m.group(1)] = int(m.group(2))
    detail = {"chain_bytes": best["bytes"], "path": best["path"],
              "su_vs_sp": {n: {"su": su.get(n), "sp": funcs.get(n, {}).get("sp")}
                           for n in ("interval_scale", "interval_mul")}}
    status = "BLOCKED" if best["bytes"] > 4096 else "PASS"
    return emit("stack", len(best["path"]), 1 if status == "BLOCKED" else 0, status, detail)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("check", choices=["manifest", "issues", "product", "stack"])
    a = ap.parse_args()
    return {"manifest": manifest, "issues": issues, "product": product, "stack": stack}[a.check]()


if __name__ == "__main__":
    sys.exit(main())
