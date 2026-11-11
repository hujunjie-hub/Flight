# OWNER Approval Request — RT-Thread H723 Package Baseline

Status: **CANDIDATES ONLY / WAITING_OWNER — LOCK REMAINS UNRESOLVED**

Scope: firmware build dependencies for STM32H723ZGT6

## 1. Decision requested

Approve the candidate package versions below, or provide replacements. Approval
is only permission to attempt installation and compatibility validation. It is
not evidence that RT-Thread env can fetch the packages or that Flight links.

## 2. Current blocker

`.config` selects all three packages with `VER="latest"` and the corresponding
`_LATEST_VERSION` option enabled. `tools/rt_packages.lock.json` correctly stays
`UNRESOLVED`; `tools/check_rt_packages_lock.py` must continue to fail.

## 3. Candidate versions

| Package symbol | Candidate tag | Candidate commit | Upstream | Required file |
|---|---|---|---|---|
| CMSIS_CORE | 5.9.0 | `61e36449f53c25ef7825c40f7dd93685736f457f` | ARM-software/CMSIS_5 | `CMSIS/Core/Include/core_cm7.h` |
| STM32H7_CMSIS_DRIVER | v1.10.7 | `05ea8164ca19102bb5c9eebf0c45f7ac2cd1e35c` | STMicroelectronics/cmsis_device_h7 | `Include/stm32h723xx.h` |
| STM32H7_HAL_DRIVER | v1.11.6 | `33ca46547d40b281a98b5ca497efc89dc628a4a7` | STMicroelectronics/stm32h7xx_hal_driver | `Inc/stm32h7xx_hal.h` |

These are candidates documented in `spec/validation/rt_thread_packages_candidates.md`.
They are not yet verified as selectable by the installed RT-Thread package
index. License texts, complete transitive dependencies, installed tree hashes,
and actual H723 link compatibility remain unverified.

## 4. Required checks after approval

1. Record OWNER approval for each package version and its license.
2. Confirm each exact tag resolves to the listed commit using `git ls-remote`
   or a fetched tag object.
3. Install the project-compatible RT-Thread env/package manager.
4. Change `.config` from `latest` only after approval; disable each
   `_LATEST_VERSION` symbol.
5. Update `tools/rt_packages.lock.json` with exact versions/commits, approver,
   approval ID and UTC time. Do not mark RESOLVED yet.
6. Execute `pkgs --update` using the approved environment and preserve complete
   stdout/stderr and exit code.
7. Record actual package directories, commits, licenses and per-file/tree SHA256.
8. Run `python tools/check_rt_packages_lock.py`.
9. Run the package-tree entrypoint check.
10. Run the real H723 firmware link and inspect ELF, MAP, symbols, flash/RAM and
    linker warnings.

A lock-file PASS alone is not a firmware-link PASS. A successful package fetch
alone is not a firmware-link PASS.

## 5. Build-system correction already prepared

R28 `SConstruct` hard-coded package directories ending in `-latest`. That
would reject a correctly pinned versioned package directory. The R29 patch
replaces this with checks for RT-Thread-generated `packages/SConscript` and
`packages/Kconfig` entrypoints. SCons will then validate referenced package
subdirectories during the actual build. The separate package lock checker
remains responsible for exact version/commit selection.

This build-script change does not install packages and does not alter the
formal dependency baseline.

## 6. Approval record

| Package | Decision | Approved version | Approver | Approval ID | UTC |
|---|---|---|---|---|---|
| CMSIS_CORE | PENDING | PENDING | PENDING | PENDING | PENDING |
| STM32H7_CMSIS_DRIVER | PENDING | PENDING | PENDING | PENDING | PENDING |
| STM32H7_HAL_DRIVER | PENDING | PENDING | PENDING | PENDING | PENDING |

Until all three are approved and installed/verified, B03 remains FAIL and B04
remains BLOCKED.
