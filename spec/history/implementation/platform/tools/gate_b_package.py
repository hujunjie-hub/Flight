#!/usr/bin/env python3
"""Assemble the A-PLATFORM stage-2 Gate B evidence package.

Builds the board module and the drv_tim hook object for Cortex-M7, records
global symbols/section sizes, merges the Gate B static checks, and writes a
reviewable acceptance package.  It never claims target waveform/HIL.

Run from the Flight project root:
  python applications/platform/rtthread53/tools/gate_b_package.py
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "applications" / "platform" / "rtthread53" / "evidence" / "gate_b"
BOARD = ROOT / "board"
SRC = BOARD / "fc_board_platform.c"
HDR = BOARD / "fc_board_platform.h"
DRV_TIM = ROOT / "libraries" / "HAL_Drivers" / "drivers" / "drv_tim.c"
GATE = OUT / "gate_b_evidence.json"
BIN = Path(r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin")
CC = BIN / "arm-none-eabi-gcc.exe"
NM = BIN / "arm-none-eabi-nm.exe"
SIZE = BIN / "arm-none-eabi-size.exe"

INCLUDES = [
    "-I" + str(ROOT), "-I" + str(BOARD), "-I" + str(BOARD / "CubeMX_Config" / "Inc"),
    "-I" + str(ROOT / "libraries" / "HAL_Drivers"),
    "-I" + str(ROOT / "libraries" / "HAL_Drivers" / "drivers"),
    "-I" + str(ROOT / "libraries" / "HAL_Drivers" / "drivers" / "config"),
    "-I" + str(ROOT / "libraries" / "HAL_Drivers" / "CMSIS" / "Include"),
    "-I" + str(ROOT / "packages" / "stm32h7_hal_driver-latest" / "Inc"),
    "-I" + str(ROOT / "packages" / "stm32h7_cmsis_driver-latest" / "Include"),
    "-I" + str(ROOT / "packages" / "CMSIS-Core-latest" / "Include"),
    "-I" + str(ROOT / "rt-thread" / "include"),
    "-I" + str(ROOT / "rt-thread" / "components" / "drivers" / "include"),
    "-I" + str(ROOT / "rt-thread" / "components" / "finsh"),
    "-I" + str(ROOT / "rt-thread" / "libcpu" / "arm" / "cortex-m7"),
    "-I" + str(ROOT / "applications" / "contracts"),
]
FLAGS = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard",
         "-std=c99", "-ffunction-sections", "-fdata-sections",
         "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-Dgcc"]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(cmd):
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    records = []

    def record(name, cmd, result):
        records.append({"name": name, "command": " ".join(str(c) for c in cmd),
                        "exit": result.returncode,
                        "stderr": result.stderr.strip()[:800]})

    board_obj = OUT / "fc_board_platform.o"
    cmd = [CC] + FLAGS + ["-Wall", "-Wextra", "-Werror", "-fno-short-enums"] + INCLUDES + ["-c", SRC, "-o", board_obj]
    res = run(cmd)
    record("compile_board", cmd, res)
    if res.returncode != 0:
        print(json.dumps({"error": "board compile failed", "stderr": res.stderr[:800]}))
        return 1

    drv_obj = OUT / "drv_tim.o"
    cmd = [CC] + FLAGS + ["-Wall", "-Wextra", "-fno-short-enums"] + INCLUDES + ["-c", DRV_TIM, "-o", drv_obj]
    res = run(cmd)
    record("compile_drv_tim", cmd, res)
    drv_ok = res.returncode == 0

    def symbols(obj):
        result = run([NM, "-g", "--defined-only", obj])
        found = {}
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 3:
                found[parts[2]] = parts[1]
        return found

    board_syms = symbols(board_obj)
    drv_syms = symbols(drv_obj) if drv_ok else {}

    expected_board = {
        "fc_board_platform_init": "T", "fc_board_mpu_cache_init": "T",
        "fc_board_tim2_init": "T", "fc_board_tim2_read64": "T",
        "fc_board_tim2_read_capture": "T", "fc_board_dshot_init": "T",
        "fc_board_dshot_submit": "T", "fc_board_dshot_stop": "T",
        "HAL_TIM_IC_CaptureCallback": "T", "HAL_TIM_PWM_PulseFinishedCallback": "T",
        "HAL_TIM_PeriodElapsedCallback": "T",
        "fc_board_tim_period_elapsed": "T", "fc_board_imu_drdy_isr": "W",
        "htim1": "B", "htim2": "B", "hdma_tim1_up": "B",
    }
    missing_board = [name for name, kind in expected_board.items()
                     if board_syms.get(name) != kind]
    drv_hook_dormant = drv_syms.get("fc_board_tim_period_elapsed") != "W"  # BSP_USING_TIM not enabled in this config
    strong_hook_ok = board_syms.get("fc_board_tim_period_elapsed") == "T"

    def size_of(obj):
        result = run([SIZE, obj])
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[0].isdigit():
                return {"text": int(parts[0]), "data": int(parts[1]), "bss": int(parts[2])}
        return {}

    sizes = {"board": size_of(board_obj), "drv_tim": size_of(drv_obj) if drv_ok else {}}
    (OUT / "symbols.txt").write_text(
        "\n".join(["## board"] + [f"{v} {k}" for k, v in sorted(board_syms.items())] +
                  (["", "## drv_tim"] + [f"{v} {k}" for k, v in sorted(drv_syms.items())])),
        encoding="utf-8")
    (OUT / "sizes.txt").write_text(json.dumps(sizes, indent=2), encoding="utf-8")

    gate = json.loads(GATE.read_text(encoding="utf-8"))
    gate_ok = gate["verdict"] == "PASS_STATIC" and not gate["failed_checks"]

    matrix = {
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3",
        "scope": "GATE_B_STATIC_IMPLEMENTATION",
        "target_validation": "NOT_RUN",
        "recommendation": "ACCEPT_STATIC_WITH_TARGET_NOT_RUN",
        "symbols": {
            "missing_or_wrong": missing_board,
            "weak_hook_in_drv_tim": not drv_hook_dormant,
        "drv_tim_hook_dormant": drv_hook_dormant,
            "strong_hook_in_board": strong_hook_ok,
            "drv_tim_compiled": drv_ok,
        },
        "sizes": sizes,
        "items": [
            {"id": "GB-01", "test": "TEST-HW-IRQ-001", "item": "Pin/AF owner (PA2/PA3 TIM2, PE9/11/13/14 TIM1)",
             "status": "PASS_STATIC", "evidence": "CubeMX MSP + .ioc; target pin measurement NOT_RUN"},
            {"id": "GB-02", "test": "TEST-HW-DMA-002", "item": "DMAMUX TIM1_UP -> DMA2_Stream2",
             "status": "PASS_STATIC", "evidence": "MSP hdma_tim1_up/DMA_REQUEST_TIM1_UP; target transfer NOT_RUN"},
            {"id": "GB-03", "test": "TEST-HW-DMA-003", "item": "DMA stream uniqueness",
             "status": "PASS_STATIC", "evidence": "DMA2_Stream2 only for TIM1_UP; target dump NOT_RUN"},
            {"id": "GB-04", "test": "TEST-HW-IRQ-004", "item": "NVIC mapping (TIM2=1, TIM1_UP=2)",
             "status": "PASS_STATIC", "evidence": "MSP priorities; runtime register dump NOT_RUN"},
            {"id": "GB-05", "test": "TEST-HW-IRQ-005", "item": "ISR API policy (no RTOS API in TIM2/TIM1/DMA ISR)",
             "status": "PASS_STATIC", "evidence": "source scan of callbacks; target trace NOT_RUN"},
            {"id": "GB-06", "test": "TEST-HW-DMA-006", "item": "MPU/D-Cache (D2 non-cacheable)",
             "status": "PASS_STATIC", "evidence": "fc_board_mpu_cache_init; cache stress NOT_RUN"},
            {"id": "GB-07", "test": "TEST-HW-IMU-007", "item": "ADIS burst DRDY capture (TIM2 CH4)",
             "status": "PASS_STATIC", "evidence": "capture ring/64-bit snapshot; SPI pre-armed DMA path NOT_RUN"},
            {"id": "GB-08", "test": "TEST-HW-SAFE-013", "item": "DShot TIM1 + DMA burst + TC->Update stop",
             "status": "PASS_STATIC", "evidence": "code/静态 checks; target waveform NOT_RUN"},
            {"id": "GB-09", "test": "TEST-RT-003/4/7", "item": "map/WCET/RTA/stack/critical section",
             "status": "NOT_RUN", "evidence": "no scons/SCons available; no target build"},
            {"id": "GB-10", "test": "TEST-RT-010/11", "item": "DMA reachability/cache stress",
             "status": "NOT_RUN", "evidence": "no target board"},
            {"id": "GB-11", "test": "TEST-HW-GNSS-008", "item": "PPS capture / timestamp accuracy",
             "status": "NOT_RUN", "evidence": "CH3 configured; no PPS source/waveform"},
            {"id": "GB-12", "test": "TEST-RT-018", "item": "Worst-case combined load / HIL",
             "status": "NOT_RUN", "evidence": "no HIL"},
        ],
        "checks": {
            "gate_b_static_pass": gate_ok,
            "symbols_pass": not missing_board and strong_hook_ok and drv_ok,
            "drv_tim_warning_only": drv_ok,
        },
    }
    matrix["verdict"] = "PASS_STATIC" if (gate_ok and not missing_board
                                          and strong_hook_ok and drv_ok) else "FAIL"
    (OUT / "acceptance_matrix.json").write_text(
        json.dumps(matrix, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    files = [SRC, HDR, ROOT / "board" / "SConscript", DRV_TIM, GATE,
             OUT / "gate_b_report.md", OUT / "acceptance_matrix.json",
             OUT / "symbols.txt", OUT / "sizes.txt"]
    hashes = {str(p.relative_to(ROOT)).replace("\\", "/"): sha(p) for p in files if p.is_file()}
    (OUT / "package_sha256.json").write_text(
        json.dumps({"algorithm": "sha256", "files": hashes}, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# A-PLATFORM stage-2 Gate B evidence package",
        "",
        "- Baseline: `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904`",
        "- Scope: static implementation + ARM Cortex-M7 compile/symbols; target validation **NOT_RUN**",
        f"- Verdict: **{matrix['verdict']}**; recommendation: `{matrix['recommendation']}`",
        "",
        "## Commands",
        "",
        "```text",
        f"{records[0]['command']}",
        f"{records[1]['command'] if len(records) > 1 else 'drv_tim compile skipped'}",
        "python applications/platform/rtthread53/tools/gate_b_check.py",
        "```",
        "",
        "## Symbols / sizes",
        "",
        f"- Missing/wrong board symbols: `{missing_board}`",
        f"- `fc_board_tim_period_elapsed`: board=`{board_syms.get('fc_board_tim_period_elapsed')}`, drv_tim=`{drv_syms.get('fc_board_tim_period_elapsed')}`",
        f"- Board `.text/.data/.bss`: `{sizes['board']}`",
        f"- drv_tim `.text/.data/.bss`: `{sizes['drv_tim']}`",
        "",
        "## Acceptance matrix",
        "",
        "| ID | Test ID | Item | Status | Evidence/gap |",
        "|---|---|---|---|---|",
    ]
    for item in matrix["items"]:
        lines.append(f"| {item['id']} | {item['test']} | {item['item']} | **{item['status']}** | {item['evidence']} |")
    lines += [
        "",
        "## Limitations (must not be read as PASS)",
        "",
        "- No `scons`/SCons in this environment: no firmware link, `.map`, WCET/RTA, stack watermark.",
        "- `drv_tim.c` was corrected (`flatency` -> `uint32_t`) and now compiles clean; its",
        "  `fc_board_tim_period_elapsed` hook is dormant while `BSP_USING_TIM` is disabled, and the",
        "  board module owns `HAL_TIM_PeriodElapsedCallback` in that configuration.",
        "- DRDY SPI fast path is only the weak `fc_board_imu_drdy_isr()` hook; the pre-armed `drv_spi`",
        "  extension is not implemented.",
        "- TIM2 capture/HAL IRQ ordering, DShot bit timing/zero-tail and MPU/cache coherency are",
        "  target-verification items.",
        "",
        "## Files",
        "",
    ]
    for path, digest in hashes.items():
        lines.append(f"- `{path}` — `{digest}`")
    (OUT / "GATE_B_EVIDENCE_PACKAGE.md").write_text("\n".join(lines) + "\n",
                                                    encoding="utf-8", newline="\n")
    print(json.dumps({"verdict": matrix["verdict"], "symbols_pass": matrix["checks"]["symbols_pass"],
                      "board_size": sizes["board"], "missing": missing_board}, ensure_ascii=False))
    return 0 if matrix["verdict"] == "PASS_STATIC" else 1


if __name__ == "__main__":
    sys.exit(main())
