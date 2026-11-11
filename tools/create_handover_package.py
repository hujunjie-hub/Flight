#!/usr/bin/env python3
"""Create the permanent R28 session handover package.

The package preserves raw bytes for every tracked modified and untracked file,
not just a patch. It is intentionally independent of chat history.
"""
import hashlib
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN_ID = "M5-06-R28-20261111"
OUT = ROOT / "build/validation/handover" / RUN_ID
CODE = "31463cb351d2f55e5da978c75db64e9d25fc2523"
REMOTE = "51e7e923419733f5aa3b21efa1e46588629623ed"


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


def main():
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

    results_path = ROOT / "build/validation/regenerated/r28_test_results.json"
    results = json.loads(results_path.read_text(encoding="utf-8")) if results_path.is_file() else {"tests": []}
    test_progress = {
        "run_id": RUN_ID,
        "code_commit": CODE,
        "test_results_path": "build/validation/regenerated/r28_test_results.json",
        "test_results_sha256": sha(results_path) if results_path.is_file() else None,
        "counts": {},
        "tests": results.get("tests", []),
    }
    for test in results.get("tests", []):
        key = test.get("status", "UNKNOWN")
        test_progress["counts"][key] = test_progress["counts"].get(key, 0) + 1
    write(OUT / "test_progress.json", json.dumps(test_progress, indent=2) + "\n")
    write(OUT / "active_processes.json", json.dumps({"run_id": RUN_ID, "processes": []}, indent=2) + "\n")

    blocker = {
        "run_id": RUN_ID,
        "B01": "FAIL/BLOCKED",
        "B02": "FAIL/BLOCKED",
        "B03": "FAIL/BLOCKED",
        "B04": "BLOCKED",
        "B05": "WAITING_INDEPENDENT_REVIEW",
        "B06": "BLOCKED (4248 B > 4096 B; callgraph UNKNOWN)",
        "B07": "NOT_RUN",
        "FLIGHT-LOCAL-35": "FIXED_WAITING_RETEST",
        "FLIGHT-LOCAL-36": "FIXED_WAITING_RETEST",
        "FLIGHT-LOCAL-37": "FIXED_WAITING_RETEST",
        "FLIGHT-LOCAL-38": "FIXED_WAITING_RETEST",
        "FLIGHT-LOCAL-39": "FIXED_WAITING_RETEST",
        "FLIGHT-LOCAL-40": "FIXED_WAITING_RETEST",
        "FLIGHT-LOCAL-41": "BLOCKED / R22 bytes NOT_RECOVERABLE",
    }
    write(OUT / "blocker_snapshot.json", json.dumps(blocker, indent=2) + "\n")

    write(OUT / "next_actions.md", """# Next actions

1. Run `python tools/check_validation_evidence.py --check all` after the archive report exists.
2. Build the R28 ZIP and bundle from the archive head; write the external report.
3. Re-run archive, handover and manifest checks against the final metadata commit.
4. Do not start M5-07 before M5-06 acceptance and MR-009 independent APPROVAL.
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
        "tooling_commit": "5e1b8e5d6ea10d0620004f23567b2502c8aae1c4",
        "remote_head": remote,
        "ahead": ahead,
        "r27_archive_head": "753c45994cb9cba12d60ce58acb04a9dcaa9b97a",
        "r27_zip_sha256": "8482a166deb44d0ebca1b2c372d6635b32c57f8225f6744064d5363fb4ae9819",
        "modified_count": len(modified),
        "untracked_count": len(untracked),
        "test_counts": test_progress["counts"],
        "blockers": blocker,
        "handover_status": "HANDOVER_READY",
    }
    write(OUT / "handover_state.json", json.dumps(state, indent=2) + "\n")

    sections = {
        "A": "项目目标：M5-06 安全验证器与 Host/SIL 证据闭环，不进入 M5-07。",
        "B": "当前阶段：R28 严格证据、MR-009 测试、B06 分析与归档准备。",
        "C": f"当前 RUN_ID：{RUN_ID}。",
        "D": f"当前 Git Branch：{branch}。",
        "E": f"当前 HEAD：{head}。",
        "F": f"Validated Code Commit：{CODE}。",
        "G": "Tooling Commit：5e1b8e5d6ea10d0620004f23567b2502c8aae1c4。",
        "H": f"Remote HEAD：{remote}。",
        "I": f"Ahead：{ahead}。",
        "J": f"工作区：{len(modified)} 个 tracked modified，{len(untracked)} 个 untracked；均为 R27 失败证据/保留文件。",
        "K": "已完成：交接规范、strict collector、统一 checker、MR-009 专用测试、B06 分析、R28 文档与测试台账。",
        "L": "未完成：最终 ZIP/Bundle/外部报告；TAKEOVER_ACK；最终全量 checker/archive 复验。",
        "M": "当前失败：spec_layout、packages_lock、production_abi。",
        "N": "当前阻塞：B01-B07；FLIGHT-LOCAL-41 R22 原始字节；S06-100..102。",
        "O": "已运行测试：见 test_progress.json 与 r28_test_results.json。",
        "P": f"测试结果 SHA256：{test_progress['test_results_sha256']}；原始日志路径见每条 test 记录。",
        "Q": "正在运行/中断进程：无；见 active_processes.json。",
        "R": "修改文件列表：见 modified_files_manifest.json（含原始字节副本）。",
        "S": "未跟踪文件列表：见 untracked_manifest.json（含原始字节副本）。",
        "T": "不允许修改：main、PR #2、正式 Spec/ABI/ICD/RT-Thread lock、4,096 B 栈预算、安全阈值、R27 归档原始字节。",
        "U": "下一步命令：生成 R28 ZIP/Bundle/报告；然后运行 `python tools/check_validation_evidence.py --check all`。",
        "V": "风险：Host/SIL 不等于 H723；B06 UNKNOWN 边；R22 bytes 不可恢复；归档元数据提交不能伪装成 ZIP 内容。",
        "W": "新 Agent 接管步骤：读取本文件、handover_state.json、重算所有 SHA、核对 Git 和测试日志，再写 TAKEOVER_ACK.md。",
    }
    text = "# R28 SESSION HANDOVER\n\n"
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
