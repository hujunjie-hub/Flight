# MR-009 R17 独立审查材料（NOT_APPROVED）

状态：**WAITING_INDEPENDENT_REVIEW**。本文件只整理审查义务与证据，不构成批准，
Agent 不得自行标记 APPROVED。

## 1. 审查范围

1. R14b/R15/R16 的 jerk 上界：`60/d³`、Bernstein 原区间、幂基尾段。
2. Bernstein-to-power：`p0=b0`、`p1=2(b1-b0)`、`p2=b0-2b1+b2`。
3. 扩展尾坐标：`u_tail = 1 + gamma_2*(end_time/duration)`。
4. consumer binary32 jerk：`j(s)=6c3+24c4*s+60c5*s²`，真实 C 系数。
5. Fraction 连续最大值：端点 + 驻点 `s*=-c4/(5c5)`。
6. S06-108 时间路径；S06-112 连续 jerk；S06-113 实际注入误差。
7. FP 假设、异常值、溢出/次正规、fail-closed。

## 2. 已提供的证据

| 证据 | 结果 |
|---|---|
| S06-112 | 360 checked / 1080 component checks / 20 fail-closed / 0 failures；旧 Bernstein-as-power 248/300 欠界 |
| S06-113 | 76 checked / 18 actual-error checks；mode2 error/γ₂ 3.618–4.059 |
| S06-108 | 96 cases / 72 wrong-local checks / 0 failures |
| S06-110d | 5 子区间 × 4 状态，解析 jerk ≤ bound |
| 证明 | `spec/history/implementation/guidance/M5_06_7_JERK_INTERVAL_PROOF.md` |
| 日志 | `build/m4_m5_implementation/evidence/m5_host.log`；SHA256 见 r16/r17_test_results |

## 3. 审查义务清单

- [ ] 独立推导 `60/d³` 与 `Δ³B`，确认没有把 `d` 与全段 `h` 混用。
- [ ] 独立推导幂基转换，确认旧公式被明确否定。
- [ ] 独立检查 `u_tail` 坐标，确认 `ratio*(1+γ₂)` 不再出现。
- [ ] 独立检查 consumer 多项式系数来源是真实 C binary32，不是 Python 副本。
- [ ] 独立复算 Fraction 端点/驻点最大值，确认无容差。
- [ ] 检查 S06-112 三类上界是否分别覆盖各自区间，total 是否为最大值。
- [ ] 检查 S06-113 注入的是 additive seconds，实际误差不是 1 微秒。
- [ ] 检查任意区间失效都走 NUMERIC_FAILURE，不静默 PASS。
- [ ] 检查结论没有越过 Host/SIL 范围。

## 4. 未满足/待审查项

- 独立审查者尚未签字；
- S06-100..102 仍 NOT_RUN；
- 4,096 B 产品化预算不满足（4,232 B）；MR-009 数学证明不等于产品化。
- 审查者必须明确 APPROVED / REJECTED / NEEDS_INFO，不得以 CI PASS 代替。

## 5. 结论栏

```text
MR-009 status: APPROVED / REJECTED / NEEDS_INFO
Reviewer: __________  Date: ______
Scope: Host/SIL M5-06 only; excludes H723 runtime/WCET/HIL/flight
```
