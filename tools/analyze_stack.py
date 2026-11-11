#!/usr/bin/env python3
"""Strict Cortex-M7 stack call-graph analyzer for the M5-06 validator.

Inputs:
  * ``.su`` file from ``-fstack-usage``
  * ``objdump -d`` text for the same object

The analyzer never returns PASS when it sees a dynamic/indirect call whose
target cannot be resolved, a recursion, a malformed function, or a missing
root. It reports the known lower bound separately from the conservative
status. This is a static analysis, not an H723 stack-watermark measurement.
"""
import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SU = ROOT / "build/m5_arm/m5_trajectory_safety.su"
DEFAULT_DIS = ROOT / "build/m5_arm/m5_trajectory_safety.dis"
LIMIT = 4096


def parse_su(text):
    frames = {}
    malformed = []
    for line in text.splitlines():
        if not line.strip():
            continue
        match = re.match(
            r"^(?P<file>.*?):(?P<line>\d+):(?P<col>\d+):(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s+"
            r"(?P<bytes>\d+)\s+(?P<kind>static|dynamic|bounded)(?:,(?P<flags>.*))?$",
            line.strip())
        if not match:
            malformed.append(line.strip())
            continue
        name = match.group("name")
        if name in frames:
            malformed.append("duplicate .su function " + name)
            continue
        frames[name] = {
            "su_bytes": int(match.group("bytes")),
            "kind": match.group("kind"),
            "source": f"{match.group('file')}:{match.group('line')}",
            "flags": (match.group("flags") or "").strip(),
        }
    return frames, malformed


def _reg_count(regs):
    count = 0
    for part in [x.strip() for x in regs.split(",") if x.strip()]:
        if re.fullmatch(r"[rd]\d+", part):
            count += 1
        elif re.fullmatch(r"[rd]\d+-[rd]\d+", part):
            first, last = part.split("-")
            a = int(first[1:])
            b = int(last[1:])
            count += abs(b - a) + 1
        elif re.fullmatch(r"\d+-\d+", part):
            a, b = part.split("-")
            count += abs(int(b) - int(a)) + 1
        elif part in ("lr", "pc", "sp", "fp", "ip"):
            count += 1
        else:
            count += 1
    return count


def _imm(token):
    token = token.strip()
    match = re.search(r"#(-?0x[0-9a-fA-F]+|-?\d+)", token)
    return int(match.group(1), 0) if match else None


def parse_disassembly(text):
    functions = {}
    current = None
    order = []
    for raw in text.splitlines():
        header = re.match(r"^\s*([0-9a-fA-F]+)\s+<([^>]+)>:\s*$", raw)
        if header:
            current = header.group(2)
            functions[current] = {
                "start": int(header.group(1), 16),
                "sp": 0,
                "max_sp": 0,
                "calls": [],
                "tail_calls": [],
                "unknown_calls": [],
                "dynamic_sp": False,
            }
            order.append(current)
            continue
        if current is None:
            continue
        line = raw.split("@")[0].strip()
        if not line or ":" not in line:
            continue
        # Instruction text after "addr:\tbytes\tmnemonic operands".
        parts = re.split(r"\s{2,}|\t+", line.strip(), maxsplit=1)
        if len(parts) < 2:
            continue
        insn = parts[1].strip()
        insn = re.sub(r"^[0-9a-fA-F ]+\s+", "", insn)
        mnemonic = insn.split()[0].lower() if insn.split() else ""
        mnemonic = mnemonic.split(".")[0]
        operands = insn[len(insn.split()[0]):].strip() if insn.split() else ""
        entry = functions[current]

        if mnemonic in ("push", "vpush", "stmdb"):
            regs = re.search(r"\{([^}]*)\}", operands)
            if regs:
                word = 8 if mnemonic == "vpush" else 4
                entry["sp"] += word * _reg_count(regs.group(1))
                entry["max_sp"] = max(entry["max_sp"], entry["sp"])
        elif mnemonic in ("sub", "subw") and re.match(r"sp\s*,\s*(sp\s*,\s*)?#", operands):
            value = _imm(operands)
            if value is None:
                entry["unknown_calls"].append("dynamic sub sp")
                entry["dynamic_sp"] = True
            else:
                entry["sp"] += value
                entry["max_sp"] = max(entry["max_sp"], entry["sp"])
        elif mnemonic in ("add", "addw") and re.match(r"sp\s*,\s*(sp\s*,\s*)?#", operands):
            value = _imm(operands)
            if value is None:
                entry["unknown_calls"].append("dynamic add sp")
                entry["dynamic_sp"] = True
            else:
                entry["sp"] = max(0, entry["sp"] - value)
        elif mnemonic in ("add", "addw", "sub", "subw") and re.match(r"sp\s*,", operands):
            if re.search(r"\br\d+|\bip\b|\blr\b", operands):
                entry["unknown_calls"].append("dynamic sp adjustment")
                entry["dynamic_sp"] = True
        elif mnemonic in ("pop", "vpop", "ldmia"):
            regs = re.search(r"\{([^}]*)\}", operands)
            if regs:
                word = 8 if mnemonic == "vpop" else 4
                entry["sp"] = max(0, entry["sp"] - word * _reg_count(regs.group(1)))
        elif mnemonic in ("sub", "subw") and re.search(r"\bsp\b", operands):
            if re.search(r"\br\d+|\bip\b|\blr\b", operands):
                entry["unknown_calls"].append("dynamic sp adjustment")
                entry["dynamic_sp"] = True
        elif mnemonic in ("mov", "movw") and re.match(r"sp\s*,", operands):
            # Epilogue `mov sp, r7` is not a dynamic allocation; `mov sp, rX`
            # other than r7 is unresolved.
            if not re.match(r"sp\s*,\s*r7\b", operands):
                entry["unknown_calls"].append("mov sp from register")
                entry["dynamic_sp"] = True

        call = re.search(r"\b(bl|blx)\s+([0-9a-fA-F]+)\s+<([^>]+)>", insn)
        if call:
            kind, _addr, target = call.groups()
            if kind == "blx" and re.fullmatch(r"[a-z]+\d*|ip|lr|r\d+", target):
                entry["unknown_calls"].append("indirect blx " + target)
            else:
                entry["calls"].append(target)
        elif re.search(r"\bblx\s+(r\d+|ip|lr)\b", insn) or re.search(r"\bbx\s+(r\d+|ip)\b", insn):
            entry["unknown_calls"].append("indirect branch " + insn)
        else:
            branch = re.search(r"^\s*b(?:\.w|\.n)?\s+([0-9a-fA-F]+)\s+<([^>]+)>", insn)
            if branch:
                target = branch.group(2)
                if "+" not in target and target != current:
                    entry["tail_calls"].append(target)
                elif "+" not in target and target == current:
                    # A self branch is a loop, not a call. If it has no +offset,
                    # treat it as an unknown control-flow edge only when it is
                    # clearly outside the body; local labels normally carry +off.
                    pass
    return functions, order


def analyze(su_text, dis_text, limit=LIMIT, root_name="m5_trajectory_safety_validate"):
    su, su_malformed = parse_su(su_text)
    funcs, order = parse_disassembly(dis_text)
    errors = list(su_malformed)
    if not su:
        errors.append("no .su frames")
    if not funcs:
        errors.append("no disassembly functions")
    for name, info in funcs.items():
        if name not in su:
            errors.append(f"function missing from .su: {name}")
    for name in su:
        if name not in funcs:
            errors.append(f"function missing from disassembly: {name}")
    # Resolve calls to known functions; external libm calls are unknown stack.
    edges = {}
    unknown = []
    for name, info in funcs.items():
        children = []
        for target in info["calls"]:
            if target in funcs:
                children.append(target)
            else:
                unknown.append({"caller": name, "target": target,
                                "kind": "external-or-imported"})
        for target in info["tail_calls"]:
            if target in funcs:
                children.append(target)
            else:
                unknown.append({"caller": name, "target": target,
                                "kind": "tail-call-external"})
        for reason in info["unknown_calls"]:
            unknown.append({"caller": name, "target": None, "kind": reason})
        edges[name] = sorted(set(children))
    if root_name not in funcs:
        return {
            "schema_version": "1.0",
            "status": "UNKNOWN",
            "reason": "root function missing",
            "limit_bytes": limit,
            "known_lower_bound_bytes": None,
            "worst_chain": [],
            "unknown_edges": unknown,
            "errors": errors,
            "frames": {},
        }
    # DFS all simple paths; detect cycles and mark recursion unknown.
    best = {"bytes": -1, "path": []}
    recursion = []
    def walk(node, total, path, active):
        nonlocal best
        info = funcs[node]
        frame = max(info["max_sp"], info["sp"])
        total2 = total + frame
        path2 = path + [node]
        if total2 > best["bytes"]:
            best = {"bytes": total2, "path": path2}
        for child in edges.get(node, []):
            if child in active:
                recursion.append(path2 + [child])
                continue
            walk(child, total2, path2, active | {child})
    walk(root_name, 0, [], {root_name})
    # Known lower bound ignores unknown external frames; that is why a
    # below-limit result with unknown edges is still UNKNOWN, never PASS.
    callgraph_status = "UNKNOWN" if (unknown or recursion or errors) else "COMPLETE"
    if best["bytes"] > limit:
        status = "BLOCKED"
    elif callgraph_status != "COMPLETE":
        status = "UNKNOWN"
    else:
        status = "PASS"
    frames = {}
    mismatches = []
    for name, info in funcs.items():
        su_bytes = su.get(name, {}).get("su_bytes")
        actual = max(info["max_sp"], info["sp"])
        frames[name] = {"su": su_bytes, "sp": actual, "kind": su.get(name, {}).get("kind")}
        if su_bytes is not None and su_bytes != actual:
            mismatches.append({"function": name, "su": su_bytes, "sp": actual,
                               "delta": actual - su_bytes})
    return {
        "schema_version": "1.0",
        "status": status,
        "callgraph_status": callgraph_status,
        "reason": ("known lower bound exceeds budget" if status == "BLOCKED"
                   else "unknown call/control-flow edges present" if status == "UNKNOWN"
                   else "all reachable calls resolved and within budget"),
        "limit_bytes": limit,
        "known_lower_bound_bytes": best["bytes"],
        "worst_chain": best["path"],
        "frames": frames,
        "su_vs_sp_mismatches": mismatches,
        "edges": edges,
        "unknown_edges": unknown,
        "recursion": recursion,
        "errors": errors,
        "scope": "STATIC_OBJECT_ANALYSIS_ONLY; NOT_H723_STACK_WATERMARK",
    }


def run_files(su_path, dis_path, limit=LIMIT):
    su_path = Path(su_path)
    dis_path = Path(dis_path)
    if not su_path.is_file():
        return {"status": "UNKNOWN", "reason": f"missing .su: {su_path}"}
    if not dis_path.is_file():
        return {"status": "UNKNOWN", "reason": f"missing disassembly: {dis_path}"}
    return analyze(su_path.read_text(errors="replace"),
                   dis_path.read_text(errors="replace"), limit=limit)


def self_test():
    cases = 0
    failures = []
    su = (
        "x.c:1:1:root 100 static\n"
        "x.c:2:1:helper 40 static\n"
        "x.c:3:1:leaf 8 static\n"
    )
    dis = (
        "00000000 <root>:\n"
        "   0:\tb500      \tpush\t{lr}\n"
        "   2:\tb082      \tsub\tsp, #8\n"
        "   4:\tf7ff fffc \tbl\t0 <helper>\n"
        "   8:\tb002      \tadd\tsp, #8\n"
        "   a:\tbd00      \tpop\t{pc}\n"
        "0000000c <helper>:\n"
        "   c:\tb510      \tpush\t{r4, lr}\n"
        "   e:\tf7ff fffd \tbl\t0 <leaf>\n"
        "  12:\tbd10      \tpop\t{r4, pc}\n"
        "00000014 <leaf>:\n"
        "  14:\t4770      \tbx\tlr\n"
    )
    base = analyze(su, dis, root_name="root")
    cases += 1
    if base["status"] != "PASS":
        failures.append("valid fixture not PASS: " + str(base))
    # vpush must be counted (d8-d15 = 64 bytes).
    su_v = su.replace("x.c:2:1:helper 40 static", "x.c:2:1:helper 72 static")
    dis_v = dis.replace("   c:\tb510      \tpush\t{r4, lr}\n",
                        "   c:\ted2d 8b10 \tvpush\t{d8-d15}\n  10:\tb510      \tpush\t{r4, lr}\n")
    dis_v = dis_v.replace("   e:\tf7ff fffd \tbl\t0 <leaf>", "  12:\tf7ff fffd \tbl\t0 <leaf>")
    v = analyze(su_v, dis_v, root_name="root")
    cases += 1
    if v["frames"].get("helper", {}).get("sp") != 72:
        failures.append("vpush not counted: " + str(v["frames"].get("helper")))
    # Indirect call must be UNKNOWN.
    dis_i = dis.replace("   e:\tf7ff fffd \tbl\t0 <leaf>",
                        "   e:\t4788      \tblx\tr1")
    i = analyze(su, dis_i, root_name="root")
    cases += 1
    if i["status"] != "UNKNOWN":
        failures.append("indirect call not UNKNOWN")
    # Tail call must create an edge.
    dis_t = dis.replace("   e:\tf7ff fffd \tbl\t0 <leaf>",
                        "   e:\tf7ff bffd \tb.w\t0 <leaf>")
    t = analyze(su, dis_t, root_name="root")
    cases += 1
    if "leaf" not in t.get("edges", {}).get("helper", []):
        failures.append("tail call edge missing")
    # Dynamic sub sp must be UNKNOWN.
    dis_d = dis.replace("  2:\tb082      \tsub\tsp, #8",
                        "  2:\t44ad      \tadd\tsp, r5")
    d = analyze(su, dis_d, root_name="root")
    cases += 1
    if d["status"] != "UNKNOWN":
        failures.append("dynamic sp not UNKNOWN")
    # .su mismatch must be reported.
    s = analyze(su.replace("helper 40 static", "helper 48 static"), dis, root_name="root")
    cases += 1
    if not s["su_vs_sp_mismatches"]:
        failures.append(".su mismatch not reported")
    # Missing root must be UNKNOWN.
    m = analyze(su, dis, root_name="missing_root")
    cases += 1
    if m["status"] != "UNKNOWN":
        failures.append("missing root not UNKNOWN")
    # Malformed .su must fail closed.
    q = analyze(su + "not a stack line\n", dis, root_name="root")
    cases += 1
    if "not a stack line" not in " ".join(q["errors"]):
        failures.append("malformed .su not reported")
    # Budget breach must be BLOCKED, not PASS.
    su_big = su.replace("root 100 static", "root 5000 static")
    b = analyze(su_big, dis.replace("sub\tsp, #8", "sub\tsp, #5000"), root_name="root")
    cases += 1
    if b["status"] != "BLOCKED":
        failures.append("budget breach not BLOCKED")
    if failures:
        for failure in failures:
            print("STACK_SELFTEST_FAIL:", failure, file=sys.stderr)
        return 1
    print(f"STACK_ANALYSIS_SELF_TEST_PASS ({cases} negative cases)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--su", default=str(DEFAULT_SU))
    ap.add_argument("--dis", default=str(DEFAULT_DIS))
    ap.add_argument("--limit", type=int, default=LIMIT)
    ap.add_argument("--output")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    result = run_files(args.su, args.dis, limit=args.limit)
    text = json.dumps(result, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)
    if result.get("status") == "BLOCKED":
        return 2
    if result.get("status") != "PASS":
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())\n