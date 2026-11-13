# T191-A-ABI Host/ARM production ABI evidence

- Baseline: `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Port `0x00010906` / Product `0x00010906`
- Profile: QUAD_X4 (`0x00010001`)
- Scope: `HOST_PRODUCTION_ABI` (production headers compiled in C11 and C++14 for ARM Cortex-M7 and x86_64 host)
- Result: **PASS**
- Independent review: NOT_RUN (reviewer D)
- Target firmware build, consumer recompilation, WCET, HIL and flight: NOT_RUN

## ABI hashes

| Target | Pointer bits | ABI hash |
|---|---|---|
| ARM Cortex-M7 EABI hard-float | 32 | `12b64f6368aad4b96b5bdd21f382a1babc810a2df96680af7853273e2e7f66c4` |
| x86_64 MinGW Host | 64 | `0edc9d8baaca7a826d9e807132f0cf232fd55f7a9b646d4d9a82aab4c288eacf` |

Canonical ABI text is `TYPE/FIELD` ordered by type name and field name; see
`tests/abi/abi_report.py::canonical_abi`.

## Types and fields

- Public types probed: **143**
- Public fields probed: **986**
- Private types excluded: `MpcOutputSnapshot`

## Key sizes (ARM / Host, bytes)

| Type | ARM | Host | ARM align |
|---|---|---|---|
| ContractHeader | 88 | 88 | 8 |
| NavigationSnapshot | 224 | 224 | 8 |
| MotionReference | 1536 | 1536 | 8 |
| ControllerInput | 64 | 112 | 8 |
| FmsInputView | 200 | 256 | 8 |
| FmsOutputView | 4584 | 4584 | 8 |
| ActuatorDemand | 152 | 152 | 8 |
| ProductMissionPayload | 1040 | 1040 | 4 |
| AuthorizedMissionSnapshot | 1184 | 1184 | 8 |
| ProductEstimatorConfigPayload | 2384 | 2384 | 8 |

## Cross-check against spec/validation/abi_layout.json

- Status: **PASS**
- Compared types: **143**
- Compared field offsets: **981**
- Mismatches: **0**
- Missing types: **0**
- Extra types: **0**
- Spec compiler: arm-none-eabi-gcc.exe (GNU Tools for STM32 14.3.rel1.20251027-0700) 14.3.1 20250623


## Host vs ARM layout delta

- Identical layouts: **139**
- Pointer-dependent layouts (expected): **4**
- Unexpected differences: **0**

| Type | ARM size | Host size | Differing fields |
|---|---|---|---|
| ControllerInput | 64 | 112 | 13 |
| FmsInputSnapshot | 200 | 256 | 0 |
| FmsInputView | 200 | 256 | 18 |
| GuidanceInput | 72 | 120 | 15 |

`FmsInputSnapshot` is a typedef alias of `FmsInputView` and inherits its layout
without independent fields. Pointer-bearing Port views are local to the calling
process and have separate ARM and Host layouts.

## Message registry

- Registry entries: **55**, macros checked: **55**
- Problems: **0**

## Header hygiene and self-containment

- Headers checked for forbidden includes / `void*`: **8**
  (problems: **0**)
- Registered messages carrying pointer fields: **0**
- Headers compiled standalone (ARM C99 + Host C++14): **8**
  (failures: **0**)

## Firmware build flags

- `rtconfig.py` layout flags required: `-mcpu=cortex-m7`, `-mthumb`, `-mfpu=fpv5-d16`, `-mfloat-abi=hard`, `-fno-short-enums`
- Missing: **0** (GNU Arm defaults to short enums; the public ICD stores 32-bit status enums)

## ABI lock

- Lock file: `middleware/contracts/product_contract/abi_lock.h`
- Status: **PASS**
- Message registry SHA256 bound by the lock: `dd06554c240567d5f81b51e4aed39336ccf3e469c6c95a952d1a2d766bbc49e7`

## Artifacts

- `tests/abi/evidence/abi_arm.json`
- `tests/abi/evidence/abi_host.json`
- `tests/abi/evidence/abi_compare.json`
- `tests/abi/evidence/commands.json`
- `tests/abi/evidence/abi_probe.c` (exact compiled probe)
- `tests/abi/evidence/abi_canonical_arm.txt` / `abi_canonical_host.txt`
  (their SHA256 equals the ABI hashes above)

## What this evidence does not prove

Compiling the headers and matching the normative layout does not prove a
target firmware build, consumer recompilation, static-memory/WCET budget,
target register/waveform behaviour, SIL/HIL or flight safety.  Those remain
NOT_RUN in this report. See spec 09 for system and flight evidence.
