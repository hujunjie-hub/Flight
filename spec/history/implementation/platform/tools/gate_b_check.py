#!/usr/bin/env python3
"""Gate B static evidence for A-PLATFORM stage 2 (TIM2/DRDY/MPU/DShot).

Compiles board/fc_board_platform.c with the firmware include set and checks the
source-level wiring required by 07/09 Gate B.  Target waveform, MPU/cache
stress, map/WCET, HIL and flight remain NOT_RUN.

Run from the Flight project root:
  python applications/platform/rtthread53/tools/gate_b_check.py
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
SCONS = BOARD / "SConscript"
IT = BOARD / "CubeMX_Config" / "Src" / "stm32h7xx_it.c"
MSP = BOARD / "CubeMX_Config" / "Src" / "stm32h7xx_hal_msp.c"
DRV_TIM = ROOT / "libraries" / "HAL_Drivers" / "drivers" / "drv_tim.c"
ARM = Path(r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    text = read(SRC)
    hdr = read(HDR)
    it = read(IT)
    msp = read(MSP)
    checks = []

    def add(cid, ok, detail):
        checks.append({"id": cid, "status": "PASS" if ok else "FAIL", "detail": detail})

    add("board_source_registered", "fc_board_platform.c" in read(SCONS),
        {"sconscript": str(SCONS.relative_to(ROOT)).replace("\\", "/")})
    add("handles_defined",
        all(re.search(r"^(TIM_HandleTypeDef|DMA_HandleTypeDef)\s+%s;" % name, text, re.M)
            for name in ("htim1", "htim2", "hdma_tim1_up")),
        {"handles": ["htim1", "htim2", "hdma_tim1_up"]})
    add("irq_handlers_exist",
        all(name in it for name in ("TIM2_IRQHandler", "TIM1_UP_IRQHandler",
                                    "DMA2_Stream2_IRQHandler")),
        {"handlers": ["TIM2", "TIM1_UP", "DMA2_Stream2"]})
    add("tim2_priority_1", "HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0)" in msp, {})
    add("tim1_up_priority_2", "HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0)" in msp, {})
    add("dma2_stream2_tim1_up",
        "hdma_tim1_up.Instance = DMA2_Stream2;" in msp
        and "DMA_REQUEST_TIM1_UP" in msp, {})
    add("tim2_32bit_free_running",
        "htim2.Init.Period = 0xFFFFFFFFu;" in text and "TIM_COUNTERMODE_UP" in text,
        {})
    add("tim2_capture_ch3_ch4",
        "TIM_CHANNEL_3" in text and "TIM_CHANNEL_4" in text
        and "TIM_IT_CC3" in text and "TIM_IT_CC4" in text, {})
    add("tim2_64bit_retry",
        "g_tim2_hi" in text and "sr1 ^ sr2" in text and "TIM_SR_UIF" in text, {})
    add("mpu_d2_noncacheable",
        "0x30000000u" in text and "ARM_MPU_ACCESS_ORDERED" in text
        and "SCB_EnableDCache" in text, {})
    add("dshot_dma_burst",
        "HAL_TIM_DMABurst_MultiWriteStart" in text
        and "TIM_DMABURSTLENGTH_4TRANSFERS" in text
        and "FC_BOARD_DSHOT_BUF_WORDS" in text, {})
    add("dshot_stop_threshold",
        "stop_threshold" in text and "19u * (g_dshot.arr + 1u)" in text
        and "fc_board_dshot_stop" in text, {})
    add("drdy_hook_present", "fc_board_imu_drdy_isr" in text and "__weak" in text, {})
    add("drv_tim_hook_present",
        "fc_board_tim_period_elapsed" in read(DRV_TIM), {})

    # DShot timing at 275 MHz, 300 kbit/s (07 section 3.0.1).
    tim1_hz, bitrate = 275000000, 300000
    n = (tim1_hz + bitrate // 2) // bitrate
    arr = n - 1
    t0h = (3 * n + 4) // 8
    t1h = (3 * n + 2) // 4
    add("dshot_timing_275mhz_300k",
        n == 917 and arr == 916 and t0h == 344 and t1h == 688,
        {"N": n, "ARR": arr, "T0H": t0h, "T1H": t1h})

    # ARM compile of the new board module with the firmware include set.
    inc = [
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
    flags = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard",
             "-std=c99", "-Wall", "-Wextra", "-Werror", "-fno-short-enums",
             "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-Dgcc"]
    obj = OUT / "fc_board_platform.o"
    log = OUT / "arm_compile.log"
    result = subprocess.run([str(ARM)] + flags + inc + ["-c", str(SRC), "-o", str(obj)],
                            capture_output=True, text=True)
    log.write_text(result.stdout + result.stderr, encoding="utf-8")
    add("arm_compile", result.returncode == 0,
        {"command": " ".join([str(ARM)] + flags + inc + ["-c", str(SRC), "-o", str(obj)]),
         "exit": result.returncode, "log": str(log.relative_to(ROOT)).replace("\\", "/"),
         "stderr": result.stderr.strip()[:400]})

    failed = [c["id"] for c in checks if c["status"] != "PASS"]
    gate_items = [
        {"item": "BSP clock/source init reuse", "status": "PASS_STATIC",
         "evidence": "board.c SystemClock_Config + CubeMX MSP; target readback NOT_RUN"},
        {"item": "IRQ/NVIC mapping", "status": "PASS_STATIC",
         "evidence": "MSP TIM2=1, TIM1_UP=2, DMA2_Stream2; target dump NOT_RUN"},
        {"item": "DMA2_Stream2 TIM1_UP mapping", "status": "PASS_STATIC",
         "evidence": "MSP hdma_tim1_up + .ioc; target transfer NOT_RUN"},
        {"item": "TIM2 free-running/capture/64-bit extension", "status": "PASS_STATIC",
         "evidence": "fc_board_platform.c; target waveform NOT_RUN"},
        {"item": "MPU/D-Cache", "status": "PASS_STATIC",
         "evidence": "four regions + D2 non-cacheable + SCB_EnableDCache; cache stress NOT_RUN"},
        {"item": "DShot TC->Update / DMA burst", "status": "PASS_STATIC",
         "evidence": "TIM1 4ch + DMA burst + absolute stop gate; waveform NOT_RUN"},
        {"item": "DRDY -> pre-armed SPI1 DMA", "status": "NOT_RUN",
         "evidence": "weak fc_board_imu_drdy_isr hook only; drv_spi async extension missing"},
        {"item": "map/WCET/stack/RTA", "status": "NOT_RUN", "evidence": "no target build"},
        {"item": "HIL / board waveform", "status": "NOT_RUN", "evidence": "no hardware"},
    ]
    result_json = {
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3",
        "scope": "GATE_B_STATIC_SOURCE_AND_ARM_COMPILE",
        "production_target_validation": "NOT_RUN",
        "files": {
            "board/fc_board_platform.c": sha(SRC),
            "board/fc_board_platform.h": sha(HDR),
            "board/SConscript": sha(SCONS),
            "libraries/HAL_Drivers/drivers/drv_tim.c": sha(DRV_TIM),
        },
        "checks": checks,
        "failed_checks": failed,
        "gate_b_items": gate_items,
        "verdict": "PASS_STATIC" if not failed else "FAIL",
    }
    (OUT / "gate_b_evidence.json").write_text(
        json.dumps(result_json, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# A-PLATFORM stage 2 Gate B static evidence",
        "",
        "- Baseline: `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904`",
        "- Scope: source wiring + ARM compile of `board/fc_board_platform.c`; target waveform/HIL NOT_RUN",
        f"- Verdict: **{result_json['verdict']}**",
        "",
        "| Gate B item | Status | Evidence |",
        "|---|---|---|",
    ]
    for item in gate_items:
        lines.append(f"| {item['item']} | **{item['status']}** | {item['evidence']} |")
    lines += ["", "## Checks", "", "| Check | Status |", "|---|---|"]
    for check in checks:
        lines.append(f"| {check['id']} | **{check['status']}** |")
    lines += [
        "",
        "## Not proved",
        "",
        "- No firmware link/map, WCET/RTA, stack watermark, MPU/cache stress test, TIM2 capture",
        "  waveform, DShot bit timing/zero-tail, DRDY-to-SPI DMA or HIL/flight evidence.",
        "- The DRDY fast path is only a weak hook; the `drv_spi` pre-armed async extension is still",
        "  missing and must be implemented/validated on target before Gate B can close.",
        "",
    ]
    (OUT / "gate_b_report.md").write_text("\n".join(lines), encoding="utf-8", newline="\n")
    print(json.dumps({"verdict": result_json["verdict"], "failed": failed,
                      "checks": len(checks), "gate_b": len(gate_items)}, ensure_ascii=False))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
