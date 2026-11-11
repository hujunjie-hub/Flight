# B06 Validator Stack Remediation Design — R29

Status: **DESIGN BASELINE / NO ARCHITECTURE CHANGE APPROVED**

## 1. Known facts

R28's Cortex-M7 disassembly report gives this known nested frame sum:

| Function | Actual SP bytes |
|---|---:|
| `m5_trajectory_safety_validate` | 1,112 |
| `build_curve` | 2,824 |
| `interval_scale` | 112 |
| `interval_mul` | 88 |
| `interval_bounds` | 64 |
| `interval_invalid` | 48 |
| **Known lower bound** | **4,248** |

Planner Worker limit is 4,096 B; known lower bound already exceeds it by 152 B.
Call graph is UNKNOWN because imported/libm and other external edges are not
resolved to their actual target frames. Therefore 4,248 B is not a complete
worst-case stack figure and cannot be treated as a final maximum.

Static RAM headroom is 232 B. A fixed workspace of 1.5–2 KiB cannot fit without
an OWNER-approved memory architecture change and a full static budget review.

## 2. Source-level cause to investigate first

`build_curve()` currently holds several arrays and interval temporaries in one
large frame, including `t_power[6]`, `dt_power[6]`, per-axis `c[6]` and
`local_power[6]`, plus `c_i[6]`, `t_power_i[6]`, `dt_power_i[6]`,
`local_power_i[6]`, and `bezier_i[6]` inside the axis loop. The compiler's
actual frame is 2,824 B.

The first optimization candidate is reducing simultaneous live storage, not
adding a helper blindly. A helper can increase nested stack if the caller's
large locals remain live across the call.

## 3. Required experiment order

A. Establish baseline with the exact GNU Tools for STM32 compiler and flags:

- `.su` for every translation unit;
- full `objdump -d -C`;
- linker map and resolved library objects;
- per-function SP accounting;
- complete call graph from the validator root;
- explicit unresolved indirect/external calls.

B. Produce a live-range map for `build_curve` arrays and temporaries. For each
array, record first write, last read, size, alignment and whether the next phase
can reuse the storage without aliasing.

C. Test three variants independently, never combine them before measurement:

1. **Axis helper extraction**: move one axis's coefficient/interval work into a
   helper whose stack frame is measured. Confirm compiler does not inline it
   back and caller does not retain equivalent temporaries.
2. **In-place interval-array reuse**: reuse arrays only after proving the
   previous phase's last read. Add assertions/tests for aliasing and axis
   independence. No static/global storage.
3. **Fixed workspace**: do not implement without OWNER approval because 232 B
   static headroom is insufficient for a 1.5–2 KiB workspace.

D. For each variant, report exact frame deltas, complete worst path, external
frames, RAM delta, WCET delta, reentrancy and concurrency impact.

## 4. Hard acceptance criteria

- The full call graph must be COMPLETE; UNKNOWN is not PASS.
- All compiler-generated stack adjustments, callee-save pushes, VFP pushes,
  dynamic allocations, tail calls and indirect calls must be accounted for.
- The final complete worst-case chain must be below 4,096 B and preserve an
  explicitly agreed caller/interrupt margin.
- The 232 B static headroom must remain within its approved ceiling.
- No lower stack budget, reduced DFS capacity or reduced safety work budget.
- Host measurements do not replace Cortex-M7 target disassembly or H723 runtime
  stack watermark.

If no variant satisfies these criteria, B06 remains BLOCKED and the system must
not proceed to M5-07.
