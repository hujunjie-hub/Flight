# OWNER / ABI Reviewer Approval Request — Message Registry Hash Rebaseline

Status: **AWAITING_ABI_APPROVAL — ABI LOCK NOT MODIFIED**

Request owner: Flight OWNER and authorized ABI reviewer
Scope: one hash constant only; no public type/layout/offset/ICD change

## 1. Decision requested

Approve updating only `ABI_LOCK_MESSAGE_REGISTRY_SHA256` in
`middleware/contracts/product_contract/abi_lock.h` from the historical CRLF
byte hash to the canonical LF Git blob hash. This is an ABI-lock evidence
rebaseline, not an ABI layout change.

## 2. Exact evidence

| Evidence | Value |
|---|---|
| Locked CRLF registry SHA256 | `e2893aebd8c1121b1ede80591849730ec722c5f9e0616443507c2ff99179bf41` |
| Current Git/LF registry SHA256 | `dd06554c240567d5f81b51e4aed39336ccf3e469c6c95a952d1a2d766bbc49e7` |
| ARM ABI hash | `12b64f6368aad4b96b5bdd21f382a1babc810a2df96680af7853273e2e7f66c4` |
| Host ABI hash | `0edc9d8baaca7a826d9e807132f0cf232fd55f7a9b646d4d9a82aab4c288eacf` |
| Compared public types | 143 |
| Compared field offsets | 981 |
| Layout mismatches | 0 |
| Unexpected Host/ARM differences | 0 |

The R28 failure is specifically the `message_registry` byte hash. The ARM and
Host canonical layout hashes match the existing ABI lock. The source analysis
is recorded in `spec/validation/eol_hash_rebaseline_analysis.md` and R28's
`tests/abi/evidence/abi_compare.json`.

## 3. Exact permitted diff

Only this macro value may change:

```c
#define ABI_LOCK_MESSAGE_REGISTRY_SHA256 "e2893aebd8c1121b1ede80591849730ec722c5f9e0616443507c2ff99179bf41"
```

Proposed value:

```c
#define ABI_LOCK_MESSAGE_REGISTRY_SHA256 "dd06554c240567d5f81b51e4aed39336ccf3e469c6c95a952d1a2d766bbc49e7"
```

No structure, typedef, enum, field order, packing, alignment, message ID,
contract version or ICD document may change under this approval.

## 4. Required validation after approval

1. Record the approval ID and reviewer.
2. Generate a one-line proposed diff using
   `python tests/abi/abi_report.py --emit-lock`.
3. Inspect the diff; abort if any other line changes.
4. Run `python tests/abi/abi_report.py` for ARM and Host evidence.
5. Confirm 143 types, 981 offsets, zero mismatches and zero unexpected
   Host/ARM differences.
6. Re-run ABI negative controls and source SHA manifest generation.
7. Attach raw compiler commands, compiler versions, logs and SHA256.

## 5. Approval record

- Decision: PENDING
- ABI reviewer: PENDING
- OWNER: PENDING
- Approval ID: PENDING
- UTC time: PENDING
- Conditions: PENDING

Until approved and independently rechecked, B02 remains FAIL. Do not run
`--emit-lock` before approval.
