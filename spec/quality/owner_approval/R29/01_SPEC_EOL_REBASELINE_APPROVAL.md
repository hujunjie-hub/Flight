# OWNER Approval Request — Spec LF Hash Rebaseline

Status: **AWAITING_OWNER_APPROVAL — NO BASELINE MODIFIED**

Request owner: Flight project OWNER / Spec authority
Prepared from: R28 archive evidence and committed Git blobs
Scope: evidence hash representation only; no FCU.xml semantic or wire-schema change

## 1. Decision requested

Approve regenerating the dialect evidence from the repository's canonical LF
working tree, replacing the historical CRLF byte hash in generated evidence.
Do not edit `spec/contracts/FCU.xml` content as part of this request.

## 2. Exact evidence

| Artifact | Existing pinned SHA256 | Current Git/LF SHA256 | Finding |
|---|---|---|---|
| `spec/contracts/FCU.xml` | `1be7adc3b849bc89f812ad66353368ae2c8c6290f2ea97168ee710a7145446ed` | `17d6e980731244852e415eb7cb7a124033160ccd7119f751c0dbe52078ce4321` | Same text, CRLF vs LF bytes |

The repository policy `.gitattributes` specifies `* text=auto eol=lf`.
The existing `spec/validation/eol_hash_rebaseline_analysis.md` records the
reproduction command and byte-level comparison. The current audit failure is
therefore a stale generated-evidence hash under the canonical LF checkout,
not evidence of a changed MAVLink definition.

## 3. Safety and compatibility impact

- No field, message ID, wire encoding, unit, enum or schema version is changed.
- The generated dialect report and its dependent hash references are refreshed.
- The canonical Git blob remains authoritative.
- The old CRLF hash and R28 failure log remain preserved as historical evidence.

## 4. Exact implementation after approval

1. Record OWNER approval ID, approver and UTC timestamp in the change request.
2. In a clean LF checkout, run `python spec/tools/check_dialect.py`.
3. Review the generated `spec/validation/dialect_report.json` diff.
4. Run `python spec/tools/audit_spec.py --round 3 --label R29`.
5. Run the optimized negative test for the audit tool.
6. Confirm `git diff --check` and verify no FCU.xml semantic diff.
7. Update dependent generated reports only where they explicitly bind the old
   `spec_xml_sha256`; do not bulk-rewrite unrelated historical reports.
8. Record exact commands, tool versions, output logs and SHA256 in R29.

## 5. Approval record

- Decision: PENDING
- Approver: PENDING
- Approval ID: PENDING
- UTC time: PENDING
- Conditions: PENDING

Until the fields above are completed by the authorized OWNER, B01 remains FAIL.
