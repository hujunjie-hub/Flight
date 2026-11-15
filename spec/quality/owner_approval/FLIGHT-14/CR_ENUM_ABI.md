# FLIGHT-14 Release Enum-Size ABI — Change Request Package

Status: **OPEN / AWAITING ABI OWNER DECISION**

Scope: QUAD_X4 / STM32H723ZGT6 production Release build with CubeCLT
GNU Tools for STM32 14.3.1. Host/SIL evidence only; H723 target and HIL remain
NOT_RUN.

## 1. Reproduced facts

| Object | `Tag_ABI_enum_size` | Origin |
|---|---|---|
| project `board.o` | **`int`** | project build with `-fno-short-enums` |
| CubeCLT `crtbegin.o` / `crtend.o` | **`small`** | toolchain multilib |
| CubeCLT `libc.a` / `libm.a` / `libgcc.a` members | **`small`** | toolchain multilib |
| linked `Flight.elf` | **`small`** | toolchain startup object wins |
| linker warnings | 228 | `arm-none-eabi-ld` enum-size mismatch |

Reproduced on 2026-11-15 with `arm-none-eabi-readelf -A` against the current
checkout (`board.o` = int, `crtbegin.o` = small, ELF attribute summary = small).

The production header ABI check (`tests/abi/abi_report.py`) is a separate gate and
passes: 144 production types, 0 spec mismatches, 0 unexpected host/ARM
differences, `abi_lock` PASS. That gate compares layouts of our own types; it
does not model how libc objects were compiled.

## 2. Why this matters and why it may not

`small` lets a compiler use 8-bit enums where it can; `int` forces 32-bit. A
mismatch is only observable where an enum value crosses an object boundary
between the two conventions.

- Our public headers are always compiled with `-fno-short-enums` (locked by
  `abi_lock.h`), so every struct layout and every enum-typed field is 32-bit on
  our side.
- The mismatching objects are startup code and libc. They do not consume our
  public enum types: our calls into libc pass integers, pointers and
  `size_t`, not enum-typed values, and libc does not return our enums.
- Therefore the current risk is *latent*: it becomes real if new code passes an
  enum-typed value through a libc API or through a callback implemented on the
  other side of the boundary.

The linker reports the enum policy for the whole object. The diagnostic does
not identify a call or prove that any argument has incompatible layout.

## 3. Options

### Option A — keep the toolchain, make the hazard explicit (recommended first step)

- Keep `-fno-short-enums` and the ABI lock unchanged.
- T18 checks the exact 228 warning objects, runtime file hashes and load order,
  120 selected archive members, and 65 cross-runtime symbols against
  [`release_enum_link_baseline.json`](../../../../tests/abi/release_enum_link_baseline.json).
  A change fails T18 and preserves the full linker log. A matching run prints
  the count and retains the raw diagnostics in `scons.log`.
- To check a fresh default Release link separately, save the SCons output and
  run `python tools/check_enum_link.py <release-log> Flight.map` from the repo
  root. The check requires the original `LINK Flight.elf` line and `-cref` map.
- The map detects new named runtime interfaces. A source or type-level check
  for arguments and indirect callbacks through existing symbols is still open.
  In the current `snprintf` calls, the GCS calibration enum values are cast to
  `unsigned` before `%u` formatting.
- Risk: none to the binary; residual risk stays latent but is now measured.
- Effort: small, Host-only.

### Option B — rebuild the multilib with 32-bit enums

- Requires maintaining a custom newlib/CubeCLT multilib and re-validating
  startup, constructors and C++ runtime.
- Removes the attribute mismatch completely.
- Risk: toolchain divergence from the vendor package; every target build and
  startup path must be re-qualified.
- Effort: large, and it changes the build baseline for every target artifact.

### Option C — switch to a toolchain whose multilib is 32-bit-enum by default

- No such vendor package is identified today, and it would move the whole
  validated toolchain identity (`arm-none-eabi-gcc 14.2.1` in the pinned
  evidence vs `14.3.1` locally).
- Not recommended.

## 4. Decision criteria for the ABI owner

1. Is `Tag_ABI_enum_size` part of the accepted production ABI contract for this
   product? If yes, only Option B closes the issue.
2. Does any current call path pass an enum-typed value across the libc
   boundary? A source audit says no; that audit should be part of the decision
   record.
3. Is a permanently warning-laden link acceptable if the warnings are baselined
   and monitored?

## 5. Test plan for whatever is approved

- `arm-none-eabi-readelf -A` assertion on `board.o`, `crtbegin.o`, `crtend.o`
  and the linked ELF, recorded in the build evidence.
- Link-warning count equality against the recorded baseline (new warning = fail).
- `tests/abi/abi_report.py` (144 types, lock PASS) and Spec Audit must stay
  green.
- Full default Release link, symbol/segment comparison, and the existing startup
  checks.
- H723 target run stays NOT_RUN and must not be claimed.

## 6. What the owner is being asked to decide

1. Decide whether the current diagnostic baseline is acceptable as a temporary
   disposition.
2. Decide whether the enum-size attribute is contractual and approve an
   isolated 32-bit-enum multilib qualification if needed.
