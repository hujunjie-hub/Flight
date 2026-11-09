#!/usr/bin/env python3
"""Platform asset inventory / gap scan for T191-A-PLATFORM preparation.

Reads the existing board/, libraries/HAL_Drivers/ and .config, records factual
evidence (file presence, enabled Kconfig symbols, IRQ handlers implemented in
the CubeMX IT file, .ioc signals) and evaluates a fixed requirement list from
spec 03/05/07.  It does NOT enable drivers, modify board files or build the
firmware; every "ENABLE_NEEDED"/"MISSING" row is work for the platform task.

Run from the Flight project root:
  python applications/platform/rtthread53/tools/platform_inventory.py
Outputs:
  applications/platform/rtthread53/evidence/platform_asset_scan.json
  applications/platform/rtthread53/evidence/platform_gap.md
"""
from __future__ import annotations

import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "applications" / "platform" / "rtthread53" / "evidence"
DRIVERS = ROOT / "libraries" / "HAL_Drivers" / "drivers"
BOARD = ROOT / "board"
CONFIG = ROOT / ".config"
IT_FILE = BOARD / "CubeMX_Config" / "Src" / "stm32h7xx_it.c"
IOC = BOARD / "CubeMX_Config" / "Flight.ioc"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""


def config_symbols() -> dict:
    symbols = {}
    for line in read(CONFIG).splitlines():
        match = re.match(r"^(CONFIG_[A-Za-z0-9_]+)=(.*)$", line.strip())
        if match:
            symbols[match.group(1)] = match.group(2)
    return symbols


def irq_handlers() -> list:
    return re.findall(r"^void\s+(\w+_IRQHandler)\s*\(\s*void\s*\)", read(IT_FILE), re.M)


def ioc_facts() -> dict:
    text = read(IOC)
    signals = sorted(set(re.findall(r"^(?:Mcu\.)?(P[A-K]\d+)\.(?:Signal|GPIO_Label)=(\S+)", text, re.M)))
    dma = sorted(set(re.findall(r"^Dma\.(\w+)\.\d+\.Instance=(\w+)", text, re.M)))
    peripherals = sorted(set(re.findall(r"^(TIM\d+|SPI\d|I2C\d|USART\d|UART\d|SDMMC\d|OCTOSPI\d)\.", text, re.M)))
    return {
        "peripherals_declared": peripherals,
        "dma_bindings": [{"request": a, "stream": b} for a, b in dma],
        "pin_signals": [{"pin": a, "signal": b} for a, b in signals],
    }


def file_state(name: str) -> dict:
    path = DRIVERS / name
    return {"path": str(path.relative_to(ROOT)).replace("\\", "/"), "exists": path.is_file()}


def main() -> int:
    symbols = config_symbols()
    irqs = irq_handlers()
    ioc = ioc_facts()
    drivers = {p.name: p.stat().st_size for p in DRIVERS.glob("*.c")}
    board_text = read(BOARD / "board.c") + read(ROOT / "libraries" / "HAL_Drivers" / "drv_common.c")
    board_platform = read(BOARD / "fc_board_platform.c")

    enabled = {key: value for key, value in symbols.items() if value == "y"}
    driver_rows = []
    for name in [
        "drv_spi.c", "drv_hard_i2c.c", "drv_tim.c", "drv_pwm.c", "drv_usart.c",
        "drv_sdmmc.c", "drv_qspi.c", "drv_dma.c", "drv_flash/drv_flash_h7.c",
    ]:
        state = file_state(name)
        state["size"] = drivers.get(Path(name).name)
        driver_rows.append(state)

    checks = {
        "system_clock_config": "SystemClock_Config" in board_text,
        "cache_enable_macro": symbols.get("BSP_SCB_ENABLE_D_CACHE") == "y",
        "mpu_config": "ARM_MPU_SetRegionEx" in board_platform,
        "tim2_init": "fc_board_tim2_init" in board_platform,
        "tim2_capture": "TIM_IT_CC3" in board_platform and "TIM_IT_CC4" in board_platform,
        "dshot_driver": "HAL_TIM_DMABurst_MultiWriteStart" in board_platform,
        "rtos_adapter_fc_os": any((ROOT / "applications" / "platform").rglob("fc_os*.h")),
        "time_service_core": (ROOT / "applications" / "services" / "time" / "fc_time_service.c").is_file(),
        "raw_ingress_core": (ROOT / "applications" / "services" / "raw_ingress" / "fc_raw_ingress.c").is_file(),
        "snapshot_channel": any((ROOT / "applications" / "services").rglob("fc_snapshot*.c")),
    }

    def status(present: bool, enabled_flag: bool | None = None) -> str:
        if not present:
            return "MISSING"
        if enabled_flag is False:
            return "ENABLE_NEEDED"
        return "REUSE"

    uart_enabled = any(key.startswith("CONFIG_BSP_USING_UART") and value == "y" for key, value in enabled.items())
    requirements = [
        ("SYS-CLOCK", "System clock tree / HSE / PLL", "board/board.c", status(checks["system_clock_config"]),
         "reuse SystemClock_Config; target clock evidence NOT_RUN"),
        ("SYS-MPU", "MPU regions (DMA non-cacheable/executable attrs)", "board/fc_board_platform.c, 03 section 6/7",
         status(checks["mpu_config"]), "static regions added; target MPU/cache stress NOT_RUN"),
        ("SYS-CACHE", "D-Cache enable + maintenance", "board/fc_board_platform.c",
         status("SCB_EnableDCache" in board_platform), "cache enabled after MPU; target stress NOT_RUN"),
        ("TIME-TICK", "TIM2 32-bit free-running 10 MHz + UIF 64-bit extension", "board/fc_board_platform.c",
         status(checks["tim2_init"]), "source implemented; target waveform NOT_RUN"),
        ("TIME-CAP", "TIM2 CH4 DRDY / CH3 PPS capture", "board/fc_board_platform.c",
         status(checks["tim2_capture"]), "capture implemented; DRDY pre-armed SPI DMA still NOT_RUN"),
        ("TIME-SVC", "Monotonic/PPS/UTC TimeService core", "applications/services/time",
         status(checks["time_service_core"]), "host-tested; board tick source binding pending"),
        ("RAW-SVC", "Raw ingress lease ring (160B slots)", "applications/services/raw_ingress",
         status(checks["raw_ingress_core"]), "host-tested; DMA publish binding pending"),
        ("SNAP-SVC", "Double-slot latest-value snapshot protocol", "03 cross-task snapshot",
         status(checks["snapshot_channel"]), "not delivered in this slice"),
        ("OS-ADP", "RTOS adapter fc_os_* static task/IPC API", "03 section 5.2",
         status(checks["rtos_adapter_fc_os"]), "not delivered in this slice"),
        ("BUS-SPI", "SPI1 IMU bus + async DMA fast path", "drv_spi.c / .ioc",
         status((DRIVERS / "drv_spi.c").is_file(), "CONFIG_RT_USING_SPI" in enabled),
         "driver exists; Kconfig/board binding not enabled"),
        ("BUS-I2C", "I2C1/I2C2 baro/mag", "drv_hard_i2c.c",
         status((DRIVERS / "drv_hard_i2c.c").is_file(), "CONFIG_RT_USING_I2C" in enabled),
         "driver exists; Kconfig/board binding not enabled"),
        ("LINK-UART", "USART2/3/4 RC/GCS/GNSS + DMA rings", "drv_usart.c / .ioc",
         "ENABLE_NEEDED", "only UART1 console enabled; other links are design inputs"),
        ("OUT-DSHOT", "TIM1 UP DMA burst + DShot TC->Update", "board/fc_board_platform.c",
         status(checks["dshot_driver"]), "source implemented; target waveform NOT_RUN"),
        ("OUT-PWM", "PWM output fallback", "drv_pwm.c",
         status((DRIVERS / "drv_pwm.c").is_file(), "CONFIG_RT_USING_PWM" in enabled),
         "driver exists; not enabled"),
        ("STO-SD", "SDMMC1 IDMA storage", "drv_sdmmc.c",
         status((DRIVERS / "drv_sdmmc.c").is_file(), "CONFIG_RT_USING_SDIO" in enabled),
         "driver exists; not enabled"),
        ("STO-NOR", "OCTOSPI1 W25Q64", "drv_qspi.c",
         status((DRIVERS / "drv_qspi.c").is_file(), "BSP_USING_QSPI" in enabled),
         "driver exists; not enabled"),
        ("IRQ-TABLE", "NVIC vector/priority ownership", "stm32h7xx_it.c",
         "STUB" if {"TIM2_IRQHandler", "DMA2_Stream2_IRQHandler"} <= set(irqs) else "MISSING",
         "handlers call HAL with CubeMX handles; peripheral init/priority ownership not verified"),
    ]

    result = {
        "baseline": "V1.9.1-GLOBAL-AUDIT-R3",
        "scope": "DOCUMENT_STATIC + PLATFORM_ASSET_INVENTORY",
        "production_validation": "NOT_RUN",
        "config_enabled_highlights": {
            key: value for key, value in enabled.items()
            if key.startswith(("CONFIG_RT_USING_", "CONFIG_BSP_USING_", "CONFIG_BSP_SCB"))
            and key in {
                "CONFIG_RT_USING_SERIAL", "CONFIG_RT_USING_SPI", "CONFIG_RT_USING_I2C",
                "CONFIG_RT_USING_PWM", "CONFIG_RT_USING_SDIO", "CONFIG_RT_USING_CACHE",
                "CONFIG_RT_USING_CAN", "CONFIG_BSP_USING_UART", "CONFIG_BSP_USING_UART1",
                "CONFIG_BSP_USING_SPI", "CONFIG_BSP_USING_I2C1", "CONFIG_BSP_USING_TIM",
            }
        },
        "drivers": driver_rows,
        "irq_handlers": sorted(set(irqs)),
        "ioc": ioc,
        "checks": checks,
        "requirements": [
            {"id": rid, "requirement": text, "evidence": evidence, "status": state, "next": nxt}
            for rid, text, evidence, state, nxt in requirements
        ],
    }

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "platform_asset_scan.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# T191-A-PLATFORM gap matrix (independent preparation)",
        "",
        "- Baseline: `V1.9.1-GLOBAL-AUDIT-R3`",
        "- Scope: static inventory of existing `board/`, `libraries/HAL_Drivers/`, `.config`, `.ioc`, IT handlers",
        "- Production target validation: **NOT_RUN** (no board build/waveform/cache test in this task)",
        "",
        "| ID | Requirement | Evidence | Status | Next action |",
        "|---|---|---|---|---|",
    ]
    for rid, text, evidence, state, nxt in requirements:
        lines.append(f"| {rid} | {text} | `{evidence}` | **{state}** | {nxt} |")
    lines += [
        "",
        "## Factual inputs",
        "",
        f"- Drivers present: {len(drivers)} `.c` files under `libraries/HAL_Drivers/drivers`.",
        f"- Enabled RT-Thread bus config: "
        f"SPI={enabled.get('CONFIG_RT_USING_SPI', 'n')}, I2C={enabled.get('CONFIG_RT_USING_I2C', 'n')}, "
        f"PWM={enabled.get('CONFIG_RT_USING_PWM', 'n')}, SDIO={enabled.get('CONFIG_RT_USING_SDIO', 'n')}, "
        f"CAN={enabled.get('CONFIG_RT_USING_CAN', 'n')}.",
        f"- `.ioc` peripherals declared: {', '.join(ioc['peripherals_declared']) or 'none parsed'}.",
        f"- IRQ handlers present: {len(result['irq_handlers'])}.",
        "",
        "## Delivered in this slice (services, host-tested)",
        "",
        "- `applications/services/time` - monotonic mapping + PPS discipline + UTC mapping.",
        "- `applications/services/raw_ingress` - 160B raw slot lease ring for M2.",
        "",
        "## Not executed",
        "",
        "- Kconfig/.config driver enablement, board MPU/D-Cache programming, TIM2/DRIVE/PPS capture,",
        "  DShot TC->Update, SD/NOR storage, and any target build/waveform/cache stress test.",
        "- Every target-dependent row must still pass Gate B/E on the real board.",
        "",
    ]
    (OUT / "platform_gap.md").write_text("\n".join(lines), encoding="utf-8", newline="\n")
    print(json.dumps({
        "requirements": len(requirements),
        "missing": sum(1 for row in requirements if row[3] == "MISSING"),
        "enable_needed": sum(1 for row in requirements if row[3] == "ENABLE_NEEDED"),
        "reuse": sum(1 for row in requirements if row[3] == "REUSE"),
        "output": str(OUT.relative_to(ROOT)).replace("\\", "/"),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
