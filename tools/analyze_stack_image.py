#!/usr/bin/env python3
"""Strict whole-image stack call-graph analyzer for a linked Cortex-M7 image.

Unlike ``analyze_stack.py`` (one relocatable object), this consumes the final
link output, so every ``bl`` resolves to a real function and imported libm/libc
edges stop being "unknown". That is what closes the B06 "external edge" gap:
the residual uncertainty becomes the *link configuration*, not the call graph.

Frame accounting uses the disassembly SP arithmetic for every function and
cross-checks project functions against their ``.su`` records (a mismatch is
reported, never hidden). The result is static evidence only: it is not an H723
stack watermark, not WCET and not a RTA.
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_stack as A  # noqa: E402

LIMIT = 4096
ROOT_FN = "m5_trajectory_safety_validate"


SU_RE = re.compile(
    r"^(?P<file>.*?):(?P<line>\d+):(?P<col>\d+):"
    r"(?P<name>[A-Za-z_][A-Za-z0-9_.$]*)\s+(?P<bytes>\d+)\s+"
    r"(?P<kind>static|dynamic|bounded)(?:,(?P<flags>.*))?$")


def load_su(su_dir):
    """Load .su records, accepting GCC clone suffixes (.constprop.0, .isra.0).

    ``analyze_stack.parse_su`` intentionally only accepts C identifiers; at -O2
    GCC emits cloned symbols with dots, so the image analyzer needs the wider
    form instead of treating its own build output as malformed.
    """
    frames = {}
    if not su_dir:
        return frames
    for path in sorted(Path(su_dir).glob("*.su")):
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            line = raw.strip()
            if not line:
                continue
            match = SU_RE.match(line)
            if not match:
                raise SystemExit(f"malformed .su {path}: {line}")
            name = match.group("name")
            frames.setdefault(name, {
                "su_bytes": int(match.group("bytes")),
                "kind": match.group("kind"),
                "source": f"{match.group('file')}:{match.group('line')}",
                "file": path.name,
            })
    return frames


def incoming_argument_bytes(instructions):
    """Recognize an unconditional GCC Thumb-2 split-argument copy.

    GCC's ARM .su excludes the space for the register portion of an incoming
    aggregate. The linked SP maximum still includes it. Require an initial
    allocation and a store of the argument registers into that exact space.
    """
    if not instructions:
        return 0
    mnemonic, operands, _ = instructions[0]
    initial = re.fullmatch(r"sp, (?:sp, )?#(8|16)", operands)
    if mnemonic != "sub" or not initial:
        return 0
    size = int(initial.group(1))
    registers = "r2, r3" if size == 8 else "r0, r1, r2, r3"
    frame = None
    address = None
    for mnemonic, operands, sp in instructions[1:]:
        # Recognize only the straight-line prologue before any call or branch.
        if frame is None and (mnemonic in ("push", "vpush") or
                (mnemonic == "stmdb" and operands.startswith("sp!,")) or
                (mnemonic in ("sub", "subw") and
                 re.fullmatch(r"sp, (?:sp, )?#\d+", operands))):
            continue
        set_frame = re.fullmatch(r"r7, sp, #(\d+)", operands)
        set_address = re.fullmatch(r"(r[0-3]|ip), r7, #(\d+)", operands)
        if mnemonic in ("add", "addw") and set_frame and frame is None:
            frame = -sp + int(set_frame.group(1))
        elif mnemonic in ("add", "addw") and set_address and frame is not None:
            register, offset = set_address.groups()
            if register in registers.split(", "):
                break
            address = (register, frame + int(offset))
            continue
        elif mnemonic in ("sub", "subw") and address:
            register, value = address
            subtract = re.fullmatch(rf"{register}, {register}, #(\d+)", operands)
            if not subtract:
                break
            address = (register, value - int(subtract.group(1)))
            continue
        elif mnemonic == "stmia" and address:
            register, offset = address
            return size if offset == -size and operands == f"{register}, {{{registers}}}" else 0
        elif (mnemonic not in ("str", "vstr") or frame is None or
              not re.fullmatch(r"(?:r[0-3]|[ds]\d+), \[r[0-7](?:, #-?\d+)?\]", operands)):
            break
        address = None
    return 0


def analyze(dis_text, su_frames, root_name=ROOT_FN, limit=LIMIT):
    funcs, _ = A.parse_disassembly(dis_text)
    if not funcs:
        return {"status": "UNKNOWN", "reason": "no functions in disassembly"}
    frames = {}
    mismatches = []
    argument_saves = {}
    for name, info in funcs.items():
        sp = max(info["max_sp"], info["sp"])
        frames[name] = {"sp": sp, "su": su_frames.get(name, {}).get("su_bytes"),
                        "kind": su_frames.get(name, {}).get("kind")}
        su = frames[name]["su"]
        if su is not None and su != sp:
            delta = sp - su
            if delta > 0 and delta == incoming_argument_bytes(info["instructions"]):
                argument_saves[name] = delta
            else:
                mismatches.append({"function": name, "su": su, "sp": sp, "delta": delta})
    edges = {}
    unknown = []
    for name, info in funcs.items():
        children = []
        for target in info["calls"]:
            if target in funcs:
                children.append(target)
            else:
                unknown.append({"caller": name, "target": target, "kind": "external"})
        for target in info["tail_calls"]:
            if target in funcs:
                children.append(target)
            else:
                unknown.append({"caller": name, "target": target, "kind": "tail-external"})
        for reason in info["unknown_calls"]:
            unknown.append({"caller": name, "target": None, "kind": reason})
        if frames[name]["kind"] == "dynamic":
            unknown.append({"caller": name, "target": None, "kind": "dynamic .su frame"})
        edges[name] = sorted(set(children))
    if root_name not in funcs:
        return {"status": "UNKNOWN", "reason": f"root {root_name} not found",
                "function_count": len(funcs), "unknown_edges": unknown}

    for mismatch in mismatches:
        unknown.append({"caller": mismatch["function"], "target": None,
                        "kind": ".su and disassembly disagree"})

    colour = {}
    cycles = []
    back_edges = set()

    def detect(node, path):
        colour[node] = 1
        for child in edges.get(node, []):
            if colour.get(child) == 1:
                cycles.append(path + [node, child])
                back_edges.add((node, child))
            elif colour.get(child, 0) == 0:
                detect(child, path + [node])
        colour[node] = 2

    detect(root_name, [])
    memo = {}

    def longest(node):
        if node in memo:
            return memo[node]
        best_child, best_val = None, 0
        for child in edges.get(node, []):
            if (node, child) in back_edges:
                continue
            value, _ = longest(child)
            if value > best_val:
                best_child, best_val = child, value
        memo[node] = (frames[node]["sp"] + best_val, best_child)
        return memo[node]

    total, _ = longest(root_name)
    chain, cur = [], root_name
    while cur is not None:
        chain.append(cur)
        cur = longest(cur)[1]

    reachable = set()
    stack = [root_name]
    while stack:
        node = stack.pop()
        if node in reachable:
            continue
        reachable.add(node)
        stack.extend(edges.get(node, []))

    # An unresolved edge in a function that cannot be reached from the root
    # cannot contribute to the root's stack. Only reachable unknowns, and
    # recursion on a reachable path, make the graph incomplete.
    unknown_reachable = [u for u in unknown if u["caller"] in reachable]
    cycles_reachable = [c for c in cycles if any(node in reachable for node in c)]
    callgraph_complete = not unknown_reachable and not cycles_reachable
    if total > limit:
        status = "BLOCKED"
    elif not callgraph_complete:
        status = "UNKNOWN"
    else:
        status = "PASS"
    return {
        "schema_version": "1.0",
        "status": status,
        "callgraph_status": "COMPLETE" if callgraph_complete else "UNKNOWN",
        "root": root_name,
        "limit_bytes": limit,
        "known_lower_bound_bytes": total,
        "worst_chain": chain,
        "function_count": len(funcs),
        "edge_count": sum(len(v) for v in edges.values()),
        "reachable_functions": len(reachable),
        "unknown_edges": unknown,
        "unknown_edges_total": len(unknown),
        "unknown_edges_reachable": unknown_reachable,
        "recursion": cycles,
        "recursion_reachable": cycles_reachable,
        "su_vs_sp_mismatches": mismatches,
        "incoming_argument_save_bytes": argument_saves,
        "frames": {name: {"sp": info["sp"], "su": info["su"]}
                   for name, info in sorted(frames.items(), key=lambda kv: -kv[1]["sp"])},
        "edges": edges,
        "scope": "LINKED_IMAGE_STATIC_ANALYSIS; NOT_H723_STACK_WATERMARK",
    }


def emit(result, out_json, csv_dir):
    text = json.dumps(result, indent=2)
    if out_json:
        Path(out_json).parent.mkdir(parents=True, exist_ok=True)
        Path(out_json).write_text(text + "\n", encoding="utf-8")
    if csv_dir:
        d = Path(csv_dir)
        d.mkdir(parents=True, exist_ok=True)
        with (d / "image_frames.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["function", "sp_bytes", "su_bytes", "delta"])
            for name, info in result.get("frames", {}).items():
                su = info["su"]
                w.writerow([name, info["sp"], "" if su is None else su,
                            "" if su is None else info["sp"] - su])
        with (d / "image_unknown_edges.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["caller", "target", "kind"])
            for edge in result.get("unknown_edges", []):
                w.writerow([edge["caller"], edge.get("target"), edge["kind"]])
    print(text)
    if result.get("status") == "BLOCKED":
        return 2
    if result.get("status") != "PASS":
        return 3
    return 0


def self_test():
    dis = (
        "00000000 <root>:\n"
        "   0:\tb500      \tpush\t{lr}\n"
        "   2:\tb082      \tsub\tsp, #8\n"
        "   4:\tf000 fffa \tbl\t0 <helper>\n"
        "   8:\tf000 fffa \tbl\t0 <fmax>\n"
        "   c:\tb002      \tadd\tsp, #8\n"
        "   e:\tbd00      \tpop\t{pc}\n"
        "00000010 <helper>:\n"
        "  10:\tb510      \tpush\t{r4, lr}\n"
        "  12:\t4770      \tbx\tlr\n"
        "00000014 <fmax>:\n"
        "  14:\t4770      \tbx\tlr\n"
        "00000018 <dead>:\n"
        "  18:\t4718      \tbx\tr3\n"
    )
    cases, failures = 0, []
    ok = analyze(dis, {"root": {"su_bytes": 12}, "helper": {"su_bytes": 8},
                       "fmax": {"su_bytes": 0}}, root_name="root", limit=4096)
    cases += 1
    if ok["status"] != "PASS":
        failures.append("complete image not PASS: " + str(ok["status"]))
    cases += 1
    if ok["known_lower_bound_bytes"] != 20:
        failures.append("lower bound wrong: " + str(ok["known_lower_bound_bytes"]))
    cases += 1
    if not (ok["unknown_edges_total"] > 0 and not ok["unknown_edges_reachable"]):
        failures.append("unreachable unknown edge wrongly treated as reachable")
    blocked = analyze(dis, {}, root_name="root", limit=10)
    cases += 1
    if blocked["status"] != "BLOCKED":
        failures.append("over-budget image not BLOCKED")
    unknown = analyze(dis.replace("bl\t0 <helper>", "bl\t0 <missing>"), {}, root_name="root", limit=4096)
    cases += 1
    if unknown["status"] != "UNKNOWN":
        failures.append("unresolved call not UNKNOWN")
    mismatch = analyze(dis, {"root": {"su_bytes": 99}}, root_name="root", limit=4096)
    cases += 1
    if mismatch["status"] != "UNKNOWN" or not mismatch["su_vs_sp_mismatches"]:
        failures.append("reachable .su mismatch did not make the result UNKNOWN")
    # GCC may omit the register arguments copied below the incoming SP from
    # its .su estimate. The measured frame must still include those bytes.
    spill = dis.replace("  10:\tb510      \tpush\t{r4, lr}\n",
                        "  10:\tb082      \tsub\tsp, #8\n"
                        "  12:\tb580      \tpush\t{r7, lr}\n"
                        "  14:\taf01      \tadd\tr7, sp, #4\n"
                        "  16:\tf107 0104 \tadd.w\tr1, r7, #4\n"
                        "  1a:\te881 000c \tstmia.w\tr1, {r2, r3}\n")
    adjusted = analyze(spill, {"helper": {"su_bytes": 8, "kind": "static"}},
                       root_name="root", limit=4096)
    cases += 1
    if adjusted["status"] != "PASS" or adjusted["known_lower_bound_bytes"] != 28:
        failures.append("register argument spill did not reconcile the frame")
    for bad in (spill.replace("r1, r7, #4", "r1, r7, #0"),
                spill.replace("{r2, r3}", "{r1, r3}"),
                spill.replace("stmia.w\tr1,", "stmia.w\tr0,")):
        adjusted = analyze(bad, {"helper": {"su_bytes": 8, "kind": "static"}},
                           root_name="root", limit=4096)
        cases += 1
        if adjusted["status"] != "UNKNOWN":
            failures.append("unexplained frame discrepancy was accepted")
    loads = dis.replace("   4:\tf000 fffa \tbl\t0 <helper>",
                        "   4:\te8b1 000c \tldmia.w\tr1!, {r2, r3}\n"
                        "   6:\tb082      \tsub\tsp, #8\n"
                        "   8:\tf000 fffa \tbl\t0 <helper>")
    adjusted = analyze(loads, {"root": {"su_bytes": 20}}, root_name="root", limit=4096)
    cases += 1
    if adjusted["status"] != "PASS" or adjusted["known_lower_bound_bytes"] != 28:
        failures.append("load through r1 was counted as an SP adjustment")
    dynamic = analyze(dis, {"root": {"su_bytes": 12, "kind": "dynamic"}},
                      root_name="root", limit=4096)
    cases += 1
    if dynamic["status"] != "UNKNOWN":
        failures.append("reachable unbounded dynamic .su frame did not make the result UNKNOWN")
    recursive = dis.replace("bl\t0 <helper>", "bl\t0 <root>")
    cases += 1
    try:
        result = analyze(recursive, {}, root_name="root", limit=4096)
        if result["status"] != "UNKNOWN" or not result["recursion_reachable"]:
            failures.append("reachable recursion did not make the result UNKNOWN")
    except RecursionError:
        failures.append("reachable recursion crashed stack analysis")
    if failures:
        for failure in failures:
            print("STACK_IMAGE_SELFTEST_FAIL:", failure, file=sys.stderr)
        return 1
    print(f"STACK_IMAGE_SELF_TEST_PASS ({cases} cases)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dis", help="objdump -d -C text of the linked image")
    ap.add_argument("--su-dir", help="directory of project .su files for cross-check")
    ap.add_argument("--root", default=ROOT_FN)
    ap.add_argument("--limit", type=int, default=LIMIT)
    ap.add_argument("--output")
    ap.add_argument("--csv-dir")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if not args.dis:
        ap.error("--dis is required")
    text = Path(args.dis).read_text(encoding="utf-8", errors="replace")
    result = analyze(text, load_su(args.su_dir), root_name=args.root, limit=args.limit)
    return emit(result, args.output, args.csv_dir)


if __name__ == "__main__":
    sys.exit(main())
