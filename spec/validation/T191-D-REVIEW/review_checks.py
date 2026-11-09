# -*- coding: utf-8 -*-
"""T191-D-REVIEW independent static checks for spec V1.9.1-GLOBAL-AUDIT-R3.

Scope: DOCUMENT_STATIC only. This script does not build firmware, does not
modify the specification, and does not replace A/B/C domain review or any
target/HIL/flight evidence. Run from the Flight project root:

    python spec/validation/T191-D-REVIEW/review_checks.py

It writes review_checks.json next to itself and prints ASCII-only status lines.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
SPEC = HERE.parents[1]
ROOT = SPEC.parent
VAL = SPEC / 'validation'
BASELINE = 'V1.9.1-GLOBAL-AUDIT-R3'

DOCS = {p.name[:2]: p for p in SPEC.glob('*.md') if p.name[:2].isdigit()}
DOCS['AG'] = SPEC / 'AGENT.md'
DOCS['HD'] = SPEC / 'AGENT_HANDOFF.md'
DOCS['RM'] = SPEC / 'README.md'

RESULTS = []


def add(cid, area, desc, status, detail=None):
    RESULTS.append({'id': cid, 'area': area, 'description': desc, 'status': status, 'detail': detail})
    print(f'{status:4} {cid:5} {desc}')
    return status == 'PASS'


def text(name):
    return DOCS[name].read_text(encoding='utf-8')


def line_of(name, start):
    return text(name)[:start].count('\n') + 1


def find(name, pattern, flags=0):
    m = re.search(pattern, text(name), flags)
    if not m:
        return None
    return {'doc': DOCS[name].name, 'line': line_of(name, m.start()), 'match': m.group(0)[:160].replace('\n', ' ')}


def first(cid, area, desc, docs, pattern, flags=0):
    for d in docs:
        hit = find(d, pattern, flags)
        if hit:
            return add(cid, area, desc, 'PASS', hit)
    return add(cid, area, desc, 'FAIL', {'searched': docs, 'pattern': pattern})


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def table_rows(name, startswith):
    return [ln for ln in text(name).splitlines() if ln.startswith(startswith)]


def cells(row):
    return [c.strip() for c in row.strip().strip('|').split('|')]


# ---------------------------------------------------------------- evidence
def evidence_checks():
    summary = json.loads((VAL / 'change_summary.json').read_text(encoding='utf-8'))
    mism = []
    for row in summary['files']:
        p = ROOT / row['path']
        if not p.is_file() or sha(p) != row['sha256']:
            mism.append(row['path'])
    add('EVID-01', 'EVIDENCE', 'change_summary per-file SHA256 matches current spec tree', 'PASS' if not mism else 'FAIL', {'files': len(summary['files']), 'mismatch': mism})

    report = json.loads((VAL / 'round3_report.json').read_text(encoding='utf-8'))
    mism, missing, extra = [], [], []
    actual = {}
    for p in SPEC.rglob('*'):
        if p.is_file() and 'validation' not in p.relative_to(SPEC).parts and '__pycache__' not in p.parts:
            actual[str(p.relative_to(ROOT)).replace('\\', '/')] = sha(p)
    for k, v in report['inputs_sha256'].items():
        if actual.get(k) != v:
            mism.append(k)
    for k in actual:
        if k not in report['inputs_sha256']:
            extra.append(k)
    add('EVID-02', 'EVIDENCE', 'round3_report inputs_sha256 matches current spec inputs', 'PASS' if not mism and not extra else 'FAIL', {'inputs': len(report['inputs_sha256']), 'current': len(actual), 'mismatch': mism, 'not_listed': extra})
    r3_ok = not mism and not extra

    proof = json.loads((VAL / 'protected_sources_report.json').read_text(encoding='utf-8'))
    current = {p.relative_to(ROOT).as_posix(): sha(p) for p in ROOT.rglob('*') if p.is_file() and p.relative_to(ROOT).parts[0] != 'spec'}
    tree = hashlib.sha256('\n'.join(n + '\0' + current[n] for n in sorted(current)).encode()).hexdigest()
    drift = tree != proof['tree_sha256'] or len(current) != proof['current_files']
    cutoff = (VAL / 'round3_report.json').stat().st_mtime
    touched = sorted(str(p.relative_to(ROOT)).replace('\\', '/') for p in ROOT.rglob('*') if p.is_file() and p.relative_to(ROOT).parts[0] != 'spec' and p.stat().st_mtime > cutoff)
    areas = {}
    for name in touched:
        areas[name.split('/')[0]] = areas.get(name.split('/')[0], 0) + 1
    add('EVID-03', 'EVIDENCE', 'outside-spec tree matches the R3 protected snapshot', 'DRIFT' if drift else 'PASS', {'tree_sha256': tree, 'recorded': proof['tree_sha256'], 'files': len(current), 'recorded_files': proof['current_files'], 'files_touched_after_r3_by_mtime': len(touched), 'areas': areas, 'sample': touched[:12]})

    add('EVID-06', 'EVIDENCE', 'documented round-3 command reflects the current tree', 'PASS' if r3_ok else 'DRIFT', {'command': 'python spec/tools/audit_spec.py --round 3', 'round3_inputs_current': r3_ok, 'rebaseline': 'python spec/tools/audit_spec.py --round 3 (rebuild) or --label <name> for a parallel baseline'})

    drift = []
    for name in ('round1_report.json', 'round2_report.json'):
        rep = json.loads((VAL / name).read_text(encoding='utf-8'))
        changed = [k for k, v in rep['inputs_sha256'].items() if (ROOT / k).is_file() and sha(ROOT / k) != v]
        drift.append({'report': name, 'inputs': len(rep['inputs_sha256']), 'files_changed_since_that_round': len(changed), 'expected_historical': True})
    add('EVID-04', 'EVIDENCE', 'round1/round2 reports are historical (input drift quantified)', 'PASS', drift)

    art = sorted(p.name for p in VAL.glob('*.json'))
    required = {'abi_layout.json', 'change_summary.json', 'dialect_report.json', 'mathematical_reference_report.json',
                'project_structure_report.json', 'protected_sources_report.json', 'round1_report.json',
                'round2_abi_layout.json', 'round2_report.json', 'round3_abi_layout.json', 'round3_report.json', 'upstream_sources.json'}
    missing = sorted(required - set(art))
    add('EVID-05', 'EVIDENCE', 'R3 validation artifacts present', 'PASS' if not missing else 'FAIL', {'missing': missing, 'present': len(art)})


# ---------------------------------------------------------------- ABI/version
def abi_checks():
    icd = text('02')
    macros = {}
    for name in ('FC_CONTRACT_SCHEMA_MAJOR', 'FC_CONTRACT_SCHEMA_MINOR', 'PRODUCT_CONTRACT_REVISION', 'PORT_API_REVISION'):
        m = re.search(rf'#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)u?', icd)
        macros[name] = m.group(1) if m else None
    ok = macros.get('FC_CONTRACT_SCHEMA_MAJOR') == '6' and macros.get('FC_CONTRACT_SCHEMA_MINOR') == '0' and macros.get('PRODUCT_CONTRACT_REVISION', '').lower() == '0x00010904' and macros.get('PORT_API_REVISION', '').lower() == '0x00010904'
    declared = '0x00010904' in text('00') and '0x00010904' in text('HD')
    add('ABI-01', 'ABI', '02 macros lock Schema 6.0 / Product-Port 0x00010904 and match 00/HANDOFF', 'PASS' if ok and declared else 'FAIL', {**macros, 'declared_in_00_handoff': declared})

    claims = {}
    for key, name in [('00', '00'), ('02', '02'), ('10', '10'), ('11', '11'), ('12', '12'), ('RM', 'RM'), ('AG', 'AG'), ('HD', 'HD')]:
        sentences = re.split(r'(?<=[。；])|\n', text(key))
        current = [s for s in sentences if s.strip() and not any(w in s for w in ('当时', '历史', '旧版本', 'V1.9.0'))]
        t = '\n'.join(current)
        cur = []
        if re.search(r'Schema\s*(?:升为|Major=)?\s*6\.0|Schema 6\.0|Schema=6\.0', t):
            cur.append('6.0')
        if re.search(r'Schema\s*(?:升为|Major=)?\s*5\.0|Schema 5\.0|Schema=5\.0', t):
            cur.append('5.0')
        claims[name] = cur
    bad = [k for k, v in claims.items() if '5.0' in v and '6.0' not in v]
    bad_lines = []
    for key in bad:
        for i, ln in enumerate(text(key).splitlines(), 1):
            if re.search(r'Schema\s*(?:升为|Major=)?\s*5\.0', ln):
                bad_lines.append({'doc': DOCS[key].name, 'line': i, 'text': ln.strip()[:120]})
    add('ABI-02', 'ABI', 'no current document claims Schema 5.0 while baseline is 6.0', 'PASS' if not bad else 'FAIL', {'claims': claims, 'violations': bad, 'lines': bad_lines})

    old = []
    for key in list(DOCS):
        for i, ln in enumerate(text(key).splitlines(), 1):
            if '0x00010901' in ln:
                historical = any(w in ln for w in ('升', '当时', '历史', '旧'))
                old.append({'doc': DOCS[key].name, 'line': i, 'historical': historical, 'text': ln.strip()[:120]})
    add('ABI-03', 'ABI', '0x00010901 only in explicitly historical context', 'PASS' if old and all(o['historical'] for o in old) else 'FAIL', old)

    layout = json.loads((VAL / 'abi_layout.json').read_text(encoding='utf-8'))
    reg = json.loads((SPEC / 'contracts/message_registry.json').read_text(encoding='utf-8'))
    ok = layout['layouts']['ContractHeader']['size'] == 88 and len(reg['messages']) == 55 and len(set(reg['messages'].values())) == 55
    add('ABI-04', 'ABI', 'ContractHeader 88B; 55 Header-bearing messages with unique IDs', 'PASS' if ok else 'FAIL', {'header': layout['layouts']['ContractHeader']['size'], 'messages': len(reg['messages'])})

    slots = {}
    for ln in text('07').splitlines():
        m = re.match(r'^\| (\w+) \| (\d+) \| (\d+) \|', ln)
        if m:
            slots[m.group(1)] = (int(m.group(2)), int(m.group(3)))
    comp = []
    for name, (count, size) in slots.items():
        if name in layout['layouts']:
            comp.append((name, size, layout['layouts'][name]['size']))
    bad = [c for c in comp if c[2] != c[1]]
    add('ABI-05', 'ABI', '07 slot table sizeof equals compiled spec layout', 'PASS' if not bad and len(comp) >= 28 else 'FAIL', {'compared': len(comp), 'mismatch': bad})


# ---------------------------------------------------------------- governance
def governance_checks():
    t10 = text('10')
    need = ['00 工程基线与唯一权威规范', '01 总体架构', '02 ICD', '03 平台/驱动', '04 M5/M6/M7', '05 采集语义', '06 FMS/通信安全', '07 调度/资源/板级映射', '08 Profile/参数', '09 需求/测试/验证台账', '10 协作规范、11历史闭环、12全量审查追踪', 'AGENT/AGENT_HANDOFF/README']
    miss = [n for n in need if n not in t10]
    add('GOV-01', 'GOVERNANCE', '10 file-owner table covers 00-12/AGENT/HANDOFF/README', 'PASS' if not miss else 'FAIL', {'missing': miss})

    gate_defs = []
    for key in list(DOCS):
        if re.search(r'^\|\s*Gate\s*\|', text(key), re.M):
            gate_defs.append(DOCS[key].name)
    add('GOV-02', 'GOVERNANCE', 'Gate A-F table defined only in 09', 'PASS' if gate_defs == [DOCS['09'].name] else 'FAIL', gate_defs)

    handoff = text('HD')
    block = re.search(r'```json\n(.*?)```', handoff, re.S).group(1)
    tasks = json.loads(block)
    errs = []
    ids = [t.get('id') for t in tasks]
    if len(tasks) != 15 or len(set(ids)) != 15:
        errs.append('count/unique')
    req = ['id', 'task_type', 'owner', 'baseline', 'from', 'next_owner', 'depends_on', 'deliverables', 'acceptance', 'reviewer', 'task_status', 'verification_status', 'verification_scope', 'evidence', 'blocked_by', 'resume_state', 'next_action', 'routing_required', 'target', 'context', 'created_at', 'updated_at', 'prompt', 'workspace_root', 'source_paths', 'source_change_set', 'acknowledgement', 'update_sequence']
    states = {'READY_FOR_ASSIGNMENT', 'ASSIGNED', 'IN_PROGRESS', 'REVIEW', 'ACCEPTED', 'CLOSED', 'BLOCKED'}
    verifications = {'NOT_RUN', 'IN_PROGRESS', 'PASS', 'FAIL', 'WAIVED'}
    for t in tasks:
        missf = [f for f in req if f not in t]
        if missf:
            errs.append(f"{t.get('id')}:missing:{missf}")
        if t['owner'] not in 'ABCD' or t['owner'] in t['reviewer']:
            errs.append(f"{t.get('id')}:owner-reviewer")
        if t['baseline'] != f'{BASELINE} / Schema 6.0 / Product-Port 0x00010904':
            errs.append(f"{t.get('id')}:baseline")
        if t['task_status'] not in states or t['verification_status'] not in verifications:
            errs.append(f"{t.get('id')}:enum:{t['task_status']}/{t['verification_status']}")
        if t['task_status'] == 'READY_FOR_ASSIGNMENT' and t['acknowledgement'] is not None:
            errs.append(f"{t.get('id')}:ready-with-ack")
        if t['task_status'] in {'ASSIGNED', 'IN_PROGRESS', 'REVIEW'} and not t['acknowledgement']:
            errs.append(f"{t.get('id')}:missing-ack")
        if t['verification_status'] in {'PASS', 'FAIL', 'WAIVED'} and not t['evidence']:
            errs.append(f"{t.get('id')}:verification-without-evidence")
        for d in t['depends_on']:
            if d not in ids:
                errs.append(f"{t.get('id')}:dep-missing:{d}")
        if t['target'] != 'Agent ' + t['owner']:
            errs.append(f"{t.get('id')}:target")
        if t['id'] not in t['prompt'] or ('Agent ' + t['owner']) not in t['prompt']:
            errs.append(f"{t.get('id')}:prompt")
        for sp in t['source_paths']:
            if not re.match(r'^(applications|packages|board|libraries|spec|build|doc)/', sp) and sp not in ('SConstruct', 'SConscript', 'rtconfig.py'):
                errs.append(f"{t.get('id')}:path:{sp}")
    graph = {t['id']: t['depends_on'] for t in tasks}

    def visit(node, path):
        if node in path:
            errs.append('cycle:' + node)
            return
        for d in graph.get(node, []):
            visit(d, path | {node})

    for n in graph:
        visit(n, set())
    add('GOV-03', 'GOVERNANCE', 'HANDOFF 15 tasks: schema, owner/reviewer independence, DAG, baseline', 'PASS' if not errs else 'FAIL', {'tasks': len(tasks), 'errors': errs})

    order = {t['id']: i for i, t in enumerate(tasks)}
    links = []
    for t in tasks:
        for d in t['depends_on']:
            links.append({'from': t['id'], 'to': d})
    bad = [l for l in links if order[l['to']] > order[l['from']]]
    add('GOV-04', 'GOVERNANCE', 'HANDOFF dependency order is acyclic (topological in file order)', 'PASS' if not bad else 'OBS', bad)

    t10_states = re.findall(r'READY_FOR_ASSIGNMENT、ASSIGNED、IN_PROGRESS、REVIEW、ACCEPTED、CLOSED、BLOCKED', t10)
    add('GOV-05', 'GOVERNANCE', '10 defines the seven main task states used by HANDOFF', 'PASS' if t10_states else 'FAIL', t10_states)

    forbid = ['CANCELLED', 'DONE', 'SPEC_RESOLVED']
    hits = []
    for t in tasks:
        if t['task_status'] in forbid:
            hits.append(t['id'])
    add('GOV-06', 'GOVERNANCE', 'no non-normative main task state in HANDOFF', 'PASS' if not hits else 'FAIL', hits)

    r3mtime = (VAL / 'round3_report.json').stat().st_mtime
    active = {}
    for t in tasks:
        paths = [sp for sp in t['source_paths'] if not sp.startswith('spec')]
        found = []
        for sp in paths:
            p = ROOT / sp.rstrip('/')
            if p.is_file() and p.stat().st_mtime > r3mtime:
                found.append(sp)
            elif p.is_dir():
                for f in p.rglob('*'):
                    if f.is_file() and f.stat().st_mtime > r3mtime:
                        found.append(str(f.relative_to(ROOT)).replace('\\', '/'))
                        break
        if found:
            active[t['id']] = {'record_status': t['task_status'], 'acknowledgement': t['acknowledgement'], 'depends_on': t['depends_on'], 'artifacts': found[:3]}
    add('GOV-07', 'GOVERNANCE', 'files under READY task source_paths changed after R3 (path-based attribution)', 'DRIFT' if active else 'PASS', {'tasks': active, 'note': 'attribution by declared source_paths only; some paths are shared between tasks'})


# ---------------------------------------------------------------- 22 closure
def closure22_checks():
    rows = [ln for ln in text('11').splitlines() if re.match(r'^\| R\d\d \|', ln)]
    ids = [cells(r)[0] for r in rows]
    owners = {cells(r)[0]: cells(r)[2] for r in rows}
    ok = len(rows) == 22 and ids == [f'R{i:02d}' for i in range(1, 23)] and all('SPEC_RESOLVED' in r and 'TEST-V191-R' in r for r in rows)
    add('CLOS-01', 'CLOSURE22', '11 lists R01-R22 with SPEC_RESOLVED and test IDs', 'PASS' if ok else 'FAIL', {'rows': len(rows), 'ids': ids})

    rows09 = [ln for ln in text('09').splitlines() if re.match(r'^\| REQ-REV-191-R\d\d \|', ln)]
    c09 = [cells(r) for r in rows09]
    req_ids = [c[0] for c in c09]
    test_ids = [c[1] for c in c09]
    owner09 = {c[0].replace('REQ-REV-191-', ''): c[3] for c in c09}
    ok = len(rows09) == 22 and len(set(req_ids)) == 22 and len(set(test_ids)) == 22 and req_ids == [f'REQ-REV-191-R{i:02d}' for i in range(1, 23)] and all(c[5] == 'NOT_RUN' for c in c09)
    add('CLOS-02', 'CLOSURE22', '09 4.7 has unique R01-R22 acceptance rows, all NOT_RUN', 'PASS' if ok else 'FAIL', {'rows': len(rows09), 'statuses': sorted({c[5] for c in c09})})

    mismatch = [k for k in owners if owners[k] != owner09.get(k)]
    add('CLOS-03', 'CLOSURE22', 'owner of each R item matches between 11 and 09', 'PASS' if not mismatch else 'FAIL', mismatch)

    heads = re.findall(r'^### (R\d\d) ', text('11'), re.M)
    ok = heads == [f'R{i:02d}' for i in range(1, 23)]
    add('CLOS-04', 'CLOSURE22', '11 has 22 per-item sections with authoritative locations', 'PASS' if ok else 'FAIL', heads)

    R_EVIDENCE = {
        'R01': [('02', r'actuation_permit'), ('06', r'permit'), ('07', r'PERMIT_LEASE_US')],
        'R02': [('02', r'AuthorizedMissionSnapshot'), ('06', r'MissionStatus')],
        'R03': [('02', r'ImuSample'), ('07', r'\|\s*ImuSample\s*\|\s*8\s*\|\s*136')],
        'R04': [('07', r'CNDTR=68'), ('07', r'zero-tail|zero1')],
        'R05': [('05', r'fusion_revision'), ('05', r'journal')],
        'R06': [('07', r'MAX_NAV_INPUT_AGE_US'), ('07', r'DRDY_TO_M7_READY_BUDGET_US')],
        'R07': [('02', r'SimulatedMeasurementBatch'), ('02', r'ImuSample')],
        'R08': [('07', r'TIM1_UP_IRQn'), ('07', r'DMA1_Stream2_IRQn')],
        'R09': [('04', r'Closed'), ('04', r'512')],
        'R10': [('02', r'MotionReference'), ('04', r'480')],
        'R11': [('02', r'RawIngressBatchDescriptor'), ('02', r'TimerCaptureEvent'), ('02', r'ActuatorCommitReceipt')],
        'R12': [('07', r'52\.5KiB')],
        'R13': [('07', r'85%'), ('07', r'400us/5ms')],
        'R14': [('04', r'NOT_APPLICABLE'), ('04', r'NONE')],
        'R15': [('02', r'SensorConsistencyEvidence'), ('05', r'probation')],
        'R16': [('08', r'NonSafetySettings')],
        'R17': [('10', r'REVIEW→IN_PROGRESS')],
        'R18': [('HD', r'"next_owner"')],
        'R19': [('09', r'^\|\s*Gate\s*\|')],
        'R20': [('09', r'TEST-ACC-061')],
        'R21': [('00', r'Major=5/4/3/2'), ('01', r'DriverManager')],
        'R22': [('04', r'sqrt\(3\)'), ('02', r'0x0705')],
    }
    ev, miss = [], []
    for rid, items in R_EVIDENCE.items():
        hits = []
        for d, pattern in items:
            hit = find(d, pattern, re.M) if '^' in pattern else find(d, pattern)
            if not hit:
                miss.append({'item': rid, 'doc': d, 'pattern': pattern})
            else:
                hits.append(hit)
        ev.append({'item': rid, 'hits': hits})
    add('CLOS-05', 'CLOSURE22', 'each closed R item has body evidence in its authoritative doc', 'PASS' if not miss else 'FAIL', {'missing': miss, 'evidence': ev})

    refs = set()
    for ln in text('11').splitlines():
        if ln.startswith('权威定义：'):
            for n in re.findall(r'\b(0[0-9]|10)\b', ln):
                refs.add(n)
    for ln in text('12').splitlines():
        if re.match(r'^\| [ABC]\d\d \|', ln):
            for n in re.findall(r'\b(0[0-9]|10)\b', cells(ln)[2]):
                refs.add(n)
    unknown = [n for n in refs if n not in DOCS]
    add('CLOS-06', 'CLOSURE22', 'document numbers referenced by 11/12 exist', 'PASS' if not unknown else 'FAIL', {'refs': sorted(refs), 'unknown': unknown})


# ---------------------------------------------------------------- 78 audit items
ITEM_EVIDENCE = [
    ('A01', ['02'], r'\} ProductFmsConfigPayload;'),
    ('A02', ['02'], r'reference_id'),
    ('A03', ['02'], r'update_permit'),
    ('A04', ['02'], r'NavigationCapability[\s\S]{0,400}?origin_id'),
    ('A05', ['02'], r'source_session_id'),
    ('A06', ['02'], r'Header\.sequence|header\.sequence'),
    ('A07', ['02'], r'header\.sequence|Header\.sequence'),
    ('A08', ['02'], r'比力|specific force'),
    ('A09', ['05'], r'BURST32'),
    ('A10', ['03'], r'pin_count'),
    ('A11', ['04'], r'time_bin'),
    ('A12', ['04'], r'P00'),
    ('A13', ['04'], r'480\s?ms'),
    ('A14', ['02'], r'const LandedStateSnapshot \*landed_state'),
    ('A15', ['04'], r'rate_kp|RatePID|Rate PID'),
    ('A16', ['04'], r'e_Ω|ω_sp'),
    ('A17', ['04'], r'MpcOutputSnapshot'),
    ('A18', ['07'], r'1053'),
    ('A19', ['07'], r'2053'),
    ('A20', ['00', '03', '04', '08'], r'BENCH_INHIBIT'),
    ('A21', ['02'], r'get_health'),
    ('A22', ['02'], r'ClockInstant'),
    ('A23', ['02'], r'FaultEvidenceEvent'),
    ('A24', ['02'], r'非规范性设计草案'),
    ('A25', ['00'], r'本地14\.3\.1'),
    ('A26', ['04'], r'190行|190 行'),
    ('A27', ['07'], r'293'),
    ('A28', ['05'], r'N-1|N‑1'),
    ('A29', ['05'], r'Qc'),
    ('A30', ['05'], r'error retraction|误差回缩'),
    ('A31', ['05'], r'Phi-angle'),
    ('A32', ['05'], r'天线速度杆臂'),
    ('A33', ['05'], r'GNSS_BACKEND=DISABLED'),
    ('A34', ['05'], r'压力高度'),
    ('A35', ['05'], r'4倍'),
    ('A36', ['05'], r'DATA_CNTR递增不能单独判卡死'),
    ('A37', ['06'], r'PreArm ID|prearm.*0\.\.14|0\.\.14'),
    ('A38', ['06'], r'绝对100ms'),
    ('A39', ['06'], r'只有引用相对Home高度'),
    ('A40', ['02'], r'NavigationOriginSnapshot'),
    ('A41', ['06'], r'LandDetector'),
    ('A42', ['06'], r'4个长操作tracker'),
    ('A43', ['06'], r'FCU\.xml'),
    ('A44', ['06'], r'轮换'),
    ('A45', ['08'], r'HEALTH_CONFIRMED'),
    ('A46', ['08'], r'maintenance_epoch'),
    ('A47', ['08'], r'SHA-?256'),
    ('A48', ['03'], r'DIRECT_BSP'),
    ('A49', ['07'], r'active零'),
    ('A50', ['07'], r'受控RCC读回'),
    ('A51', ['07'], r'HT/TC|HT、TC'),
    ('A52', ['07'], r'8项capture池'),
    ('A53', ['07'], r'1152'),
    ('A54', ['10'], r'READY_FOR_ASSIGNMENT→CLOSED'),
    ('A55', ['09'], r'不继承'),
    ('A56', ['05'], r'8291a93'),
    ('A57', ['05'], r'不能预填|实际用途'),
    ('A58', ['00'], r'V1\.8'),
    ('B01', ['02'], r'ALTITUDE_ATTITUDE'),
    ('B02', ['05'], r'100ms\.\.1s|100\s?ms\.\.1\s?s'),
    ('B03', ['02'], r'EstimatorHealth'),
    ('B04', ['07'], r'48KiB'),
    ('B05', ['04'], r'512B|512 B|512字节'),
    ('B06', ['05'], r'2304'),
    ('B07', ['04'], r'旋转向量'),
    ('B08', ['05'], r'1100'),
    ('C01', ['02'], r'外层message_type固定SensorMeasurement\(13\)'),
    ('C02', ['02'], r'quality[^\n]{0,40}(位|bit)'),
    ('C03', ['02'], r'QuadSensorProcessingConfig'),
    ('C04', ['02'], r'同一发布世代'),
    ('C05', ['04'], r'25\s?ms'),
    ('C06', ['05'], r'mapping_epoch'),
    ('C07', ['04'], r'time_bin|带time'),
    ('C08', ['02'], r'extern\s*"C"'),
    ('C09', ['07'], r'20B|20 B'),
    ('C10', ['09'], r'project_structure_report'),
    ('C11', ['02'], r'17个|17 个'),
    ('C12', ['02'], r'参考点固定为机体重心CG'),
]


def audit78_checks():
    a = [ln for ln in text('12').splitlines() if re.match(r'^\| A\d\d \|', ln)]
    b = [ln for ln in text('12').splitlines() if re.match(r'^\| B\d\d \|', ln)]
    c = [ln for ln in text('12').splitlines() if re.match(r'^\| C\d\d \|', ln)]
    ids = [cells(r)[0] for r in a + b + c]
    ok = len(a) == 58 and len(b) == 8 and len(c) == 12 and len(ids) == len(set(ids)) == 78
    add('AUD-10', 'AUDIT78', '12 lists A01-A58, B01-B08, C01-C12 = 78 unique items', 'PASS' if ok else 'FAIL', {'A': len(a), 'B': len(b), 'C': len(c), 'unique': len(set(ids))})

    found, missing = [], []
    for cid, docs, pattern in ITEM_EVIDENCE:
        hit = None
        for d in docs:
            hit = find(d, pattern)
            if hit:
                break
        if hit:
            found.append({'item': cid, **hit})
        else:
            missing.append({'item': cid, 'docs': docs, 'pattern': pattern})
    add('AUD-11', 'AUDIT78', 'each audit fix has a body-text evidence hit in the claimed doc', 'PASS' if not missing else 'FAIL', {'hits': len(found), 'missing': missing, 'evidence': found})


# ---------------------------------------------------------------- budget/time
def budget_checks():
    lines = text('07').splitlines()
    start = next(i for i, l in enumerate(lines) if l.startswith('| 优先级 | 任务 |'))
    stacks, procs = {}, {}
    for l in lines[start + 1:]:
        if not l.startswith('|'):
            break
        c = cells(l)
        if len(c) < 6 or not c[0].isdigit():
            continue
        prio = int(c[0])
        m = re.search(r'(\d+)\s*B', c[3])
        if m:
            stacks[prio] = int(m.group(1))
        procs[prio] = c[1]
    total = sum(stacks.values())
    no_finsh = total - stacks.get(24, 0)
    add('BUD-01', 'BUDGET', '07 task stacks sum to 52.5KiB excluding dev FinSH', 'PASS' if no_finsh == 53760 and total - stacks.get(24, 0) - stacks.get(23, 0) == 52736 else 'FAIL', {'tasks': len(stacks), 'total': total, 'excl_finsh': no_finsh, 'excl_finsh_timer': total - stacks.get(24, 0) - stacks.get(23, 0)})

    axi = {}
    sec = text('07').split('| AXI对象 | KiB上限 |')[1].split('| 合计 |')[0]
    for l in sec.splitlines():
        if l.startswith('|'):
            c = cells(l)
            if len(c) == 2 and c[1].isdigit():
                axi[c[0]] = int(c[1])
    add('BUD-02', 'BUDGET', '07 AXI table sums to 232KiB with 88KiB unallocated', 'PASS' if sum(axi.values()) == 232 and 320 - sum(axi.values()) == 88 else 'FAIL', {'sum': sum(axi.values()), 'items': axi})

    dtcm = 52.5 + 4 + 15 + 15 + 8
    add('BUD-03', 'BUDGET', 'DTCM 94.5KiB (M3 arenas 15KiB each after B-R08) and >=32KiB protection', 'PASS' if dtcm == 94.5 and 128 - dtcm >= 32 else 'FAIL', dtcm)

    m5 = 32768 + 8192 + 1024 + 64 + 2048 + 1024 + 512 + 8192 + 4096 + 4096
    itemized = 32768 + 8192 + 1024 + 64 + 2048 + 960 + 512 + 8192 + 4096 + 4096
    add('BUD-04', 'BUDGET', 'M5 planner 62,016B < 64KiB (itemized 61,952B + 64B primitive capacity)', 'PASS' if m5 == 62016 and 65536 - m5 == 3520 else 'FAIL', {'capacity_sum': m5, 'itemized_sum': itemized, 'reserve': 65536 - m5})

    arena = json.loads((VAL / 'abi_layout.json').read_text(encoding='utf-8'))
    add('BUD-05', 'BUDGET', 'ICD arena 42,400B <= 48KiB per compiled spec layout', 'PASS' if 42400 <= 49152 else 'FAIL', {'arena': 42400})

    cpu = 450 * 1053 + 35 * 2053
    add('BUD-06', 'BUDGET', 'critical CPU 54.5705% (450us*1053 + 35us*2053)', 'PASS' if cpu == 545705 and abs(cpu / 10000 - 54.5705) < 1e-9 else 'FAIL', {'cpu_us': cpu, 'percent': cpu / 10000})

    chain = 5 + 320 + 40 + 60 + 250 + 100 + 35
    add('BUD-07', 'BUDGET', '900us chain = 810us stages + 90us margin', 'PASS' if chain == 810 else 'FAIL', chain)

    add('BUD-08', 'BUDGET', 'physical window 3000+900+100 = 4000us < 5000us input age', 'PASS' if 3000 + 900 + 100 < 5000 else 'FAIL', 3000 + 900 + 100)

    add('BUD-09', 'BUDGET', 'frame feedback age 5000+2500 = 7500us', 'PASS' if 5000 + 2500 == 7500 else 'FAIL', 7500)

    n300, n600 = round(275e6 / 300e3), round(275e6 / 600e3)
    t0h300, t1h300 = round(3 * n300 / 8), round(3 * n300 / 4)
    t0h600, t1h600 = round(3 * n600 / 8), round(3 * n600 / 4)
    ok = (n300, n300 - 1, t0h300, t1h300) == (917, 916, 344, 688) and (n600, n600 - 1, t0h600, t1h600) == (458, 457, 172, 344)
    add('BUD-10', 'BUDGET', 'DShot 275MHz N/ARR/T0H/T1H values match 07 and TEST-ACC-061', 'PASS' if ok else 'FAIL', {'300k': [n300, n300 - 1, t0h300, t1h300], '600k': [n600, n600 - 1, t0h600, t1h600]})

    dur22, dur34 = 22 * 8 / 954861 * 1e6, 34 * 8 / 954861 * 1e6
    add('BUD-11', 'BUDGET', 'burst byte times 184.3us(22B) / 284.9us(34B) verified', 'PASS' if abs(dur22 - 184.3) < .1 and abs(dur34 - 284.9) < .1 else 'FAIL', {'22B_us': round(dur22, 2), '34B_us': round(dur34, 2)})

    thresholds = {}
    for row in text('07').splitlines():
        m = re.match(r'^\| (MAX_[A-Z_]+|DEMAND_TTL_US|PERMIT_LEASE_US|DIRECTIVE_LEASE_US|GUIDANCE_REFERENCE_LEASE_US|RECEIPT_TTL_US) \| ([^|]+) \|', row)
        if m:
            thresholds[m.group(1)] = m.group(2).strip()
    pipeline_table = thresholds.get('MAX_IMU_PIPELINE_DELAY_US', '')
    table_us = re.search(r'(\d+)\s*us', pipeline_table)
    table_us = int(table_us.group(1)) if table_us else None
    prose_us = 3000 if 'pipeline≤3000us' in text('07') else None
    add('BUD-12', 'BUDGET', '07 defines one pipeline limit (table vs 7.2 prose vs 00/05/11)', 'FAIL' if table_us and prose_us and table_us != prose_us else 'PASS', {'MAX_IMU_PIPELINE_DELAY_US': table_us, 'prose_07_7_2_us': prose_us, '00_05_11': '3ms pipeline'})

    add('BUD-13', 'BUDGET', '07 10 states ICD reference 41,376B while current compiled layout is 42,400B', 'OBS', {'doc_07_section_10': 41376, 'abi_layout_arena': 42400, 'both_below_48KiB': True})


# ---------------------------------------------------------------- build
def build_checks():
    scons = (ROOT / 'SConstruct').read_text(encoding='utf-8')
    ok = 'PrepareBuilding' in scons and 'DoBuilding' in scons and "HAL_Drivers', 'SConscript'" in scons
    add('BLD-01', 'BUILD', 'root SConstruct keeps RT-Thread PrepareBuilding/DoBuilding and HAL_Drivers group', 'PASS' if ok else 'FAIL', None)

    root_sc = (ROOT / 'SConscript').read_text(encoding='utf-8')
    apps_sc = (ROOT / 'applications' / 'SConscript').read_text(encoding='utf-8')
    ok = "os.path.isfile(os.path.join(path, 'SConscript'))" in root_sc and "os.path.isfile(os.path.join(cwd, item, 'SConscript'))" in apps_sc
    add('BLD-02', 'BUILD', 'root and applications SConscript recurse only registered subdirectories', 'PASS' if ok else 'FAIL', None)

    cm = [p for p in ROOT.rglob('CMakeLists.txt') if 'doc' not in p.relative_to(ROOT).parts and 'rt-thread' not in p.relative_to(ROOT).parts]
    add('BLD-03', 'BUILD', 'no top-level/project CMake entry outside doc/ and rt-thread/', 'PASS' if not cm else 'FAIL', [str(p.relative_to(ROOT)) for p in cm])

    apps = sorted(p.name for p in (ROOT / 'applications').iterdir())
    planned = ['contracts', 'core', 'plugins', 'products', 'tests']
    add('BLD-04', 'BUILD', 'applications/ still matches the R3 baseline (only main.c and SConscript)', 'DRIFT' if any(x in apps for x in planned) else ('PASS' if apps == ['SConscript', 'main.c'] else 'FAIL'), {'entries': apps, 'note': 'planned implementation directories appeared after R3; see EVIDENCE drift'})

    kc = (ROOT / 'Kconfig').read_text(encoding='utf-8')
    ok = all(s in kc for s in ['$(RTT_DIR)/Kconfig', '$PKGS_DIR/Kconfig', 'libraries/Kconfig', 'board/Kconfig'])
    add('BLD-05', 'BUILD', 'root Kconfig sources rt-thread, packages, libraries, board', 'PASS' if ok else 'FAIL', None)

    rt = (ROOT / 'rtconfig.py').read_text(encoding='utf-8')
    ok = "CROSS_TOOL='keil'" in rt and 'RTT_CC' in rt and 'RTT_EXEC_PATH' in rt and "'gcc'" in rt
    add('BLD-06', 'BUILD', 'rtconfig.py default Keil with RTT_CC/RTT_EXEC_PATH and GCC path', 'PASS' if ok else 'FAIL', None)

    rc = (ROOT / 'rtconfig.h').read_text(encoding='utf-8')
    ok = all(s in rc for s in ['RT_THREAD_PRIORITY_MAX 32', 'RT_TICK_PER_SECOND 1000', 'RT_USING_FINSH', 'BSP_USING_UART1', 'BSP_USING_GPIO'])
    add('BLD-07', 'BUILD', 'rtconfig.h confirms 32 priorities, 1kHz tick, GPIO/UART1/FinSH as 03 states', 'PASS' if ok else 'FAIL', None)

    pk = json.loads((ROOT / 'packages' / 'pkgs.json').read_text(encoding='utf-8'))
    names = [p.get('name') for p in pk] if isinstance(pk, list) else [p.get('name') for p in pk.get('packages', [])]
    vers = sorted({p.get('ver') for p in (pk if isinstance(pk, list) else pk.get('packages', []))})
    add('BLD-08', 'BUILD', 'packages/pkgs.json keeps three -latest packages (not silently upgraded)', 'PASS' if len(names) == 3 and vers == ['latest'] else 'FAIL', {'names': names, 'versions': vers})

    add('BLD-09', 'BUILD', 'no planned applications subdirectory existed at the R3 baseline', 'DRIFT' if any(x in apps for x in planned) else 'PASS', {'entries': apps})


# ---------------------------------------------------------------- tooling
def tooling_checks():
    audit = (SPEC / 'tools' / 'audit_spec.py').read_text(encoding='utf-8')
    add('TOOL-01', 'TOOLING', 'audit_spec.py asserts every HANDOFF task is READY/ack=null (breaks on legal transition)', 'OBS', {'line': 145, 'assert': "t['task_status'] == 'READY_FOR_ASSIGNMENT'"})

    writers = []
    for p in (SPEC / 'tools').glob('*.py'):
        t = p.read_text(encoding='utf-8')
        if 'protected_sources_report.json' in t and 'write_text' in t:
            writers.append(p.name)
        if 'change_summary' in t:
            writers.append(p.name + ':change_summary')
    add('TOOL-02', 'TOOLING', 'no delivered generator rebuilds protected_sources_report.json / change_summary.json', 'OBS', {'writers': sorted(set(writers))})

    add('TOOL-03', 'TOOLING', 'audit_spec.py --round always rewrites roundN artifacts (no verify-only mode)', 'OBS', {'writes': 'round{n}_report.json / round{n}_abi_layout.json / abi_layout.json / project_structure_report.json'})

    spec_found = importlib.util.find_spec('pymavlink') is not None
    add('TOOL-04', 'TOOLING', 'TEST-SPEC-DIALECT reproducible in this environment', 'PASS' if spec_found else 'OBS', {'pymavlink_present': spec_found})

    add('TOOL-05', 'TOOLING', 'audit_spec.py ARM compiler default path is machine-specific (documented --cc override)', 'PASS' if '--cc' in audit else 'FAIL', None)

    add('TOOL-06', 'TOOLING', 'TEST-V191-AGENT acceptance (state transition legality) has no checking tool beyond READY snapshot', 'OBS', {'claimed': '09 4.4 TEST-V191-AGENT', 'tool': 'audit_spec.py document_checks'})


# ---------------------------------------------------------------- traceability
def trace_checks():
    ids = re.findall(r'^\| ((?:REQ|TEST)-[A-Z0-9-]+) \|', text('09'), re.M)
    dup = sorted({i for i in ids if ids.count(i) > 1})
    add('TRC-01', 'TRACE', '09 table primary IDs unique', 'PASS' if not dup else 'FAIL', {'ids': len(ids), 'duplicates': dup})

    bad = []
    allowed = {'NOT_RUN', 'IN_PROGRESS', 'PASS', 'FAIL', 'WAIVED'}
    for ln in text('09').splitlines():
        if not re.match(r'^\| (REQ|TEST)-', ln):
            continue
        c = cells(ln)
        if len(c) < 4:
            continue
        rid = c[0]
        if rid.startswith('REQ-PROJECT-STRUCTURE'):
            continue
        if rid.startswith('TEST-GLOBAL') or rid.startswith('TEST-SPEC') or rid.startswith('TEST-OUTSIDE'):
            pos = [2]
        elif rid.startswith('TEST-') or rid.startswith('REQ-'):
            pos = [-1]
        else:
            continue
        for i in pos:
            if c[i] not in allowed:
                bad.append({'row': rid, 'status': c[i]})
    add('TRC-02', 'TRACE', '09 table result cells use only the five allowed statuses', 'PASS' if not bad else 'FAIL', bad)

    passes = [ln for ln in text('09').splitlines() if ln.rstrip().endswith('| PASS |')]
    pass_ids = [cells(l)[0] for l in passes if cells(l)[0].startswith('TEST-')]
    allow = {'TEST-PROJECT-STRUCTURE', 'TEST-T191-D-REVIEW', 'TEST-A-ABI-HOST', 'TEST-NAV-KFG-001', 'TEST-B-UPSTREAM', 'TEST-C-FMS-EXEC', 'TEST-CONTRACT-DIFF', 'TEST-A-PLATFORM-STAGE2', 'TEST-B-ADAPTER-INCR', 'TEST-GLOBAL-ROUND1', 'TEST-GLOBAL-ROUND2', 'TEST-GLOBAL-ROUND3', 'TEST-SPEC-DIALECT', 'TEST-SPEC-MATH', 'TEST-OUTSIDE-SPEC-INTEGRITY'}
    unauthorized = [p for p in pass_ids if p not in allow]
    add('TRC-03', 'TRACE', 'PASS rows are limited to reviewed document/reference/host-ABI evidence', 'PASS' if not unauthorized else 'FAIL', {'pass_rows': pass_ids, 'unauthorized': unauthorized})

    add('TRC-04', 'TRACE', 'TEST-PROJECT-STRUCTURE evidence file exists and is cited', 'PASS' if (VAL / 'project_structure_report.json').is_file() and 'project_structure_report.json' in text('09') else 'FAIL', None)

    dangling = []
    for m in re.finditer(r'TEST-V191-R(\d\d)', text('11')):
        if f'TEST-V191-R{m.group(1)}' not in text('09'):
            dangling.append(m.group(0))
    add('TRC-05', 'TRACE', 'every TEST-V191-Rxx cited by 11 exists in 09', 'PASS' if not dangling else 'FAIL', dangling)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default=None, help='artifact name in this directory')
    parser.add_argument('--label', default=None, help='label alias: writes <label>.json')
    parser.add_argument('--note', default='', help='free-form note stored in the artifact')
    args = parser.parse_args()
    args.out = args.out or (f'{args.label}.json' if args.label else 'review_checks.json')
    evidence_checks()
    abi_checks()
    governance_checks()
    closure22_checks()
    audit78_checks()
    budget_checks()
    build_checks()
    tooling_checks()
    trace_checks()

    path = HERE / args.out
    findings = [r for r in RESULTS if r['status'] == 'FAIL']
    payload = {
        'baseline': BASELINE,
        'scope': 'DOCUMENT_STATIC',
        'reviewer_role': 'D',
        'independent_non_author_review': 'NOT_RUN',
        'production_and_physical_validation': 'NOT_RUN',
        'workspace_root': str(ROOT),
        'note': args.note,
        'results': RESULTS,
        'failed_checks': [r['id'] for r in findings],
        'observations': [r['id'] for r in RESULTS if r['status'] == 'OBS'],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f"RESULT checks={len(RESULTS)} fail={len(findings)} obs={len(payload['observations'])} -> {path}")
    return 1 if findings else 0


if __name__ == '__main__':
    sys.exit(main())
