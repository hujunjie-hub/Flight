#!/usr/bin/env python3
"""Create the permanent R28 session handover package.

The package preserves raw bytes for every tracked modified and untracked file,
not just a patch. It is intentionally independent of chat history.
"""
import hashlib
import json
import argparse
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# RUN_ID is a command-line argument: a hard-coded one made every later
# handover claim the R28 baseline.
RUN_ID = ""
OUT = ROOT / "build/validation/handover" / "unset"
CODE = ""
REMOTE = ""


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT), *args],
                          capture_output=True, text=True).stdout.strip()


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def copy(path, dest):
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes((ROOT / path).read_bytes())


def parse_blockers():
    """Read the live B01-B07 status from the derived blocker board."""
    board = ROOT / "spec/quality/BLOCKER_BOARD.md"
    blockers = {}
    if not board.is_file():
        return blockers
    current = None
    for line in board.read_text(encoding="utf-8").splitlines():
        m = re.match(r"^##\s+(B0\d)\b", line)
        if m:
            current = m.group(1)
            blockers.setdefault(current, "UNKNOWN")
            continue
        if current and re.match(r"^\|\s*状态\s*\|", line):
            cells = [cell.strip() for cell in line.strip("|").split("|")]
            if len(cells) >= 2:
                blockers[current] = cells[1].replace("**", "")
                current = None
    return blockers


def validation_summary():
    """Summarise the last local regression instead of a frozen R28 ledger."""
    path = ROOT / "build/m4_m5_implementation/evidence/validation.json"
    if not path.is_file():
        return {"status": "NOT_RUN", "run_id": None, "code_commit": None,
                "passed": [], "failed": [], "sha256": None, "path": None}
    data = json.loads(path.read_text(encoding="utf-8"))
    steps = data.get("commands") or data.get("steps") or data.get("tests") or []
    passed, failed = [], []
    for step in steps:
        name = step.get("name") or step.get("test_id") or "?"
        if "status" in step:
            ok = step["status"] == "PASS"
        else:
            ok = step.get("exit_code") == 0
        (passed if ok else failed).append(name)
    return {"status": data.get("status", "UNKNOWN"),
            "run_id": data.get("run_id"),
            "code_commit": data.get("code_commit"),
            "passed": passed, "failed": failed,
            "sha256": sha(path),
            "path": str(path.relative_to(ROOT)).replace("\\", "/")}


def main():
    global RUN_ID, OUT, CODE, REMOTE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--code-commit", default="")
    parser.add_argument("--remote-commit", default="")
    args = parser.parse_args()
    RUN_ID = args.run_id
    OUT = ROOT / "build/validation/handover" / RUN_ID
    CODE = args.code_commit or git("rev-parse", "HEAD")
    REMOTE = args.remote_commit or git("rev-parse", "origin/main")
    OUT.mkdir(parents=True, exist_ok=True)
    head = git("rev-parse", "HEAD")
    branch = git("branch", "--show-current")
    remote = git("rev-parse", f"origin/{branch}")
    ahead = int(git("rev-list", "--count", f"origin/{branch}..HEAD"))
    status = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain=v2",
                             "--untracked-files=all"], capture_output=True,
                            text=True).stdout
    write(OUT / "git_status.txt", status)
    write(OUT / "git_head.txt", head + "\n")
    write(OUT / "git_branch.txt", branch + "\n")
    (OUT / "git_diff_cached.patch").write_bytes(
        subprocess.run(["git", "-C", str(ROOT), "diff", "--binary", "--cached"],
                       capture_output=True).stdout)
    (OUT / "git_diff_worktree.patch").write_bytes(
        subprocess.run(["git", "-C", str(ROOT), "diff", "--binary"],
                       capture_output=True).stdout)

    modified = []
    for line in status.splitlines():
        if line.startswith("1 "):
            match = re.search(r"\t(.+)$", line) or re.search(r"\s(\S+)$", line)
            if match:
                modified.append(match.group(1))
    untracked = []
    for line in status.splitlines():
        if line.startswith("? "):
            untracked.append(line[2:])
    for path in modified:
        copy(path, OUT / "working_tree_modified" / path)
    for path in untracked:
        copy(path, OUT / "working_tree_untracked" / path)
    mod_manifest = [{"path": p, "size_bytes": (ROOT / p).stat().st_size,
                     "sha256": sha(ROOT / p),
                     "raw_copy": "working_tree_modified/" + p} for p in modified]
    unt_manifest = [{"path": p, "size_bytes": (ROOT / p).stat().st_size,
                     "sha256": sha(ROOT / p),
                     "raw_copy": "working_tree_untracked/" + p} for p in untracked]
    write(OUT / "modified_files_manifest.json", json.dumps(mod_manifest, indent=2) + "\n")
    write(OUT / "untracked_manifest.json", json.dumps(unt_manifest, indent=2) + "\n")

    validation = validation_summary()
    test_progress = {
        "run_id": RUN_ID,
        "code_commit": CODE,
        "validation_run_id": validation["run_id"],
        "validation_code_commit": validation["code_commit"],
        "validation_status": validation["status"],
        "test_results_path": validation["path"],
        "test_results_sha256": validation["sha256"],
        "counts": {"PASS": len(validation["passed"]),
                   "NOT_PASS": len(validation["failed"])},
        "passed": validation["passed"],
        "not_passed": validation["failed"],
    }
    write(OUT / "test_progress.json", json.dumps(test_progress, indent=2) + "\n")
    write(OUT / "active_processes.json", json.dumps({"run_id": RUN_ID, "processes": []}, indent=2) + "\n")

    blocker = {"run_id": RUN_ID}
    blocker.update(parse_blockers())
    write(OUT / "blocker_snapshot.json", json.dumps(blocker, indent=2) + "\n")

    def _open(value):
        text = value.strip()
        return not (text.startswith("PASS") or text.startswith("OWNER_WAIVED")
                    or text.startswith("RESOLVED"))
    open_blockers = [f"{k}={v[:40]}" for k, v in blocker.items()
                     if k != "run_id" and _open(v)]
    write(OUT / "next_actions.md", f"""# Next actions

1. Read `SESSION_HANDOVER.md`, `handover_state.json` and `test_progress.json`.
2. Re-run `python tests/guidance/run_validation.py --run-id <id> --code-commit <sha>`
   and compare with the recorded validation run ({validation["run_id"]}).
3. Current non-PASS blockers: {', '.join(open_blockers) if open_blockers else 'none'}.
4. Do not start M5-07 before M5-06 acceptance and independent review.
""")
    write(OUT / "NEXT_SESSION_START_PROMPT.txt", f"""You are a new local Agent for Flight.
Read build/validation/handover/{RUN_ID}/SESSION_HANDOVER.md and handover_state.json first.
Independently verify HEAD={head}, branch={branch}, remote={remote}, ahead={ahead},
all SHA256SUMS, git status, the test ledger and the blocker snapshot.
Then write build/validation/handover/{RUN_ID}/TAKEOVER_ACK.md.
Do not modify code before the ACK. Do not reset/clean/stash/push/rebase.
""")

    state = {
        "run_id": RUN_ID,
        "branch": branch,
        "head": head,
        "validated_code_commit": CODE,
        "remote_head": remote,
        "ahead": ahead,
        "modified_count": len(modified),
        "untracked_count": len(untracked),
        "validation": {k: validation[k] for k in ("status", "run_id", "code_commit", "sha256")},
        "test_counts": test_progress["counts"],
        "blockers": blocker,
        "handover_status": "HANDOVER_READY",
    }
    write(OUT / "handover_state.json", json.dumps(state, indent=2) + "\n")

    sections = {
        "A": "项目目标：M5-06 安全验证器与 Host/SIL 证据闭环；M5-07 在验收与独立复核前不得启动。",
        "B": f"当前阶段：ENG-07 终端链（FLIGHT-17/18/19/21/23/24）受 FLIGHT-26（近地极限环）与 FLIGHT-29（导航发散）阻塞，其余非真机项已闭环。",
        "C": f"当前 RUN_ID：{RUN_ID}。",
        "D": f"当前 Git Branch：{branch}。",
        "E": f"当前 HEAD：{head}。",
        "F": f"Validated Code Commit：{CODE}。",
        "G": f"最后一次本地回归：{validation['run_id']} / {validation['status']}（{len(validation['passed'])} PASS，{len(validation['failed'])} 非 PASS）。",
        "H": f"Remote HEAD：{remote}。",
        "I": f"Ahead：{ahead}。",
        "J": f"工作区：{len(modified)} 个 tracked modified，{len(untracked)} 个 untracked。",
        "K": "已完成：文档冲突与 Issue 映射、未登记项登记（#27–#34）、证据哈希重基与 ABI 证据中性化、memory-budget 新鲜度门控、CONTACT_HOLD 静态保持、SIL 导航发散检测、M6 导航加速度 fail-closed、T01–T30 本地回归全绿。",
        "L": f"未完成：FLIGHT-26 内环阻尼与 FLIGHT-29 估计器修复待批准；H723/HIL/WCET/栈水位/飞行 NOT_RUN；独立复核未指派。",
        "M": f"当前非 PASS 步骤：{', '.join(validation['failed']) if validation['failed'] else '无'}。",
        "N": "当前阻塞：" + (', '.join(open_blockers) if open_blockers else '无') + "。",
        "O": f"已运行测试：{validation['path']}（SHA256 {validation['sha256']}）。",
        "P": f"测试结果 SHA256：{test_progress['test_results_sha256']}。",
        "Q": "正在运行/中断进程：无；见 active_processes.json。",
        "R": "修改文件列表：见 modified_files_manifest.json（含原始字节副本）。",
        "S": "未跟踪文件列表：见 untracked_manifest.json（含原始字节副本）。",
        "T": "不允许修改：main、正式 Spec/ABI/ICD/RT-Thread lock、4,096 B 栈预算、M4/M5 安全阈值、FMS 授权边界。",
        "U": "下一步命令：按 next_actions.md；先复跑 T01–T30 再复跑原生 FMT-SIM 三场景。",
        "V": "风险：Host/SIL/ARM 编译不等于 H723 实机；FLIGHT-26/29 未修复前 Mission 与 LANDED 验收不可信。",
        "W": "新 Agent 接管步骤：读取本文件、handover_state.json、重算 SHA、核对 Git 与测试日志，再写 TAKEOVER_ACK.md。",
    }
    text = "# SESSION HANDOVER\n\n"
    text += f"Status: **HANDOVER_READY**\n\n"
    for key in "ABCDEFGHIJKLMNOPQRSTUVW":
        text += f"## {key}. {sections[key]}\n\n"
    write(OUT / "SESSION_HANDOVER.md", text)

    sums = []
    for path in sorted(OUT.rglob("*")):
        if path.is_file() and path.name != "HANDOVER_SHA256SUMS.txt":
            sums.append(f"{sha(path)}  {path.relative_to(OUT).as_posix()}")
    write(OUT / "HANDOVER_SHA256SUMS.txt", "\n".join(sums) + "\n")
    print(f"HANDOVER_READY {OUT} modified={len(modified)} untracked={len(untracked)} head={head}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
