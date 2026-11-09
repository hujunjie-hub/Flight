# -*- coding: utf-8 -*-
"""Rebuild spec/validation/change_summary.json for the current baseline.

Per user instruction (T191-D-REVIEW follow-up), the R3 change summary is
archived in r3_archive/. This script writes a baseline manifest: current
SHA256 of every R3-listed spec input plus the fresh OUTSIDE_SPEC_INTEGRITY
snapshot. It does not recompute added/removed line counts because the R3
file contents are not available; those fields stay null and say so.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = ROOT / 'spec'
VAL = SPEC / 'validation'
ARCHIVE = Path(__file__).resolve().parent / 'r3_archive'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    r3 = json.loads((ARCHIVE / 'change_summary.json').read_text(encoding='utf-8'))
    protected = json.loads((VAL / 'protected_sources_report.json').read_text(encoding='utf-8'))
    files = []
    for row in r3['files']:
        p = ROOT / row['path']
        current = sha(p) if p.is_file() else None
        files.append({
            'path': row['path'],
            'change': 'UNCHANGED' if current == row['sha256'] else 'MODIFIED',
            'sha256': current,
            'r3_sha256': row['sha256'],
            'added_lines': None,
            'removed_lines': None,
            'note': 'line delta not recomputed (R3 content not retained)' if current != row['sha256'] else '',
        })
    current_inputs = {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in SPEC.rglob('*')
                      if p.is_file() and 'validation' not in p.relative_to(SPEC).parts and '__pycache__' not in p.parts}
    listed = {f['path'] for f in files}
    added = [k for k in current_inputs if k not in listed]
    removed = [f['path'] for f in files if f['sha256'] is None]
    payload = {
        'change_set': 'T191-D-REVIEW-POSTFIX',
        'baseline': 'V1.9.1-GLOBAL-AUDIT-R3 / POST-REVIEW-REBUILD 2026-11-04',
        'supersedes': 'spec/validation/T191-D-REVIEW/r3_archive/change_summary.json (R3 original)',
        'issues': 78,
        'rounds': r3.get('rounds', []),
        'scope': 'SPEC_ONLY',
        'production_firmware_build': 'NOT_RUN',
        'independent_human_review': 'A/B/C opinions recorded; Gate A and user approval NOT_RUN',
        'rebuild_note': 'Rebuilt on the 07-corrected tree per user instruction; R3 original archived. File-level hashes only, line deltas not recomputed.',
        'outside_spec': {
            'scope': protected['scope'],
            'status': protected['status'],
            'baseline_files': protected['baseline_files'],
            'current_files': protected['current_files'],
            'tree_sha256': protected['tree_sha256'],
            'note': 'checkpoint at rebuild time; implementation work is ongoing',
        },
        'files': files,
        'added_since_r3': added,
        'removed_since_r3': removed,
    }
    (VAL / 'change_summary.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'files': len(files), 'modified': sum(1 for f in files if f['change'] == 'MODIFIED'),
                      'added': len(added), 'removed': len(removed)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
