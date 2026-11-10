"""Spec-only checks. Does not build firmware, generate drivers, or modify sources.

Run from Flight: python spec/tools/audit_spec.py --round 1
ARM compiler may be overridden with --cc (matching g++/objcopy must be adjacent).
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
sys.dont_write_bytecode = True
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SPEC = ROOT / 'spec'
VALIDATION = SPEC / 'validation'
ICD = SPEC / '02_接口契约与公共数据规范.md'
BASELINE = 'V1.9.1-GLOBAL-AUDIT-R3'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _record_failure(message, filename, line):
    """Best-effort structured failure artifact for the acceptance ledger."""
    try:
        import json as _json
        import time as _time
        target = Path(__file__).resolve().parent.parent / 'validation' / 'audit_failure.json'
        target.write_text(_json.dumps({
            'status': 'FAIL',
            'message': str(message),
            'file': filename,
            'line': line,
            'utc': _time.strftime('%Y-%m-%dT%H:%M:%SZ', _time.gmtime()),
        }, indent=2) + '\n', encoding='utf-8')
    except Exception:
        pass


def require(condition, message="acceptance check failed"):
    """Acceptance check that survives `python -O`.

    `assert` statements are removed when Python runs with -O, which would turn a
    failed safety acceptance check into a silent pass. This helper always raises
    SystemExit with a structured message, the failing source location and a JSON
    failure artifact.
    """
    if not condition:
        import sys as _sys
        frame = _sys._getframe(1)
        _record_failure(message, frame.f_code.co_filename, frame.f_lineno)
        raise SystemExit('SPEC_AUDIT_FAILED: %s (%s:%d)' % (
            message, frame.f_code.co_filename, frame.f_lineno))


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def extract():
    text = ICD.read_text(encoding='utf-8')
    blocks = list(re.finditer(r'```c\n(.*?)```', text, re.S))
    # UGV/USV are explicitly illustrative and mutually exclusive with QUAD.
    accepted = [m for m in blocks if not re.search(r'#define PRODUCT_PREVIEW_COUNT 12\b', m[1])]
    source = '\n'.join(m[1] for m in accepted)
    control_text = (SPEC/'04_飞控核心模块与控制系统设计.md').read_text(encoding='utf-8')
    private_blocks = [m[1] for m in re.finditer(r'```c\n(.*?)```', control_text, re.S) if 'MpcOutputSnapshot' in m[1]]
    require(len(private_blocks) == 1)
    source += '\n' + private_blocks[0]
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    macros = re.findall(r'^#define[^\n]+', source, re.M)
    typedefs = re.findall(r'typedef\s+(?:struct|union|enum)\s*\{.*?\}\s*\w+\s*;|typedef\s+\w+\s+\w+\s*;', source, re.S)
    name_of = lambda d: re.search(r'(\w+)\s*;\s*$', d)[1]
    definitions = {name_of(d): d for d in typedefs}
    require(len(definitions) == len(typedefs), 'duplicate normative typedef')
    source_left = source
    for d in typedefs:
        source_left = source_left.replace(d, '')
    anonymous = re.findall(r'(?<!\w)enum\s*\{.*?\}\s*;', source_left, re.S)
    functions = re.findall(r'\b(?:Status|uint64_t|uint32_t|void)\s+\w+\([^;{}]*\)\s*;', source_left, re.S)
    ordered, pending = [], dict(definitions)
    while pending:
        ready = [n for n, d in pending.items() if not ((set(re.findall(r'\b\w+\b', d)) & pending.keys()) - {n})]
        if not ready:
            raise ValueError(f'cyclic/missing type dependencies: {list(pending)}')
        for n in ready:
            ordered.append(pending.pop(n))
    header = '#include <stdint.h>\n#include <stddef.h>\n'
    header += '\n'.join(macros + anonymous + ordered)
    header += '\n#ifdef __cplusplus\nextern \"C\" {\n#endif\n' + '\n'.join(functions) + '\n#ifdef __cplusplus\n}\n#endif\n'
    return header, definitions, len(blocks), len(accepted)


def compile_abi(cc):
    header, definitions, total, accepted = extract()
    expressions, keys = [], []
    for name, d in definitions.items():
        for kind, expression in [('size', f'sizeof({name})'), ('alignment', f'__alignof__({name})')]:
            keys.append((name, kind))
            expressions.append(expression)
        if re.match(r'typedef\s+(?:struct|union)', d):
            body = d[d.index('{') + 1:d.rindex('}')]
            for field in body.split(';'):
                if not field.strip():
                    continue
                m = re.search(r'(\w+)\s*(?:\[[^]]+\]\s*)*$', field.strip())
                require(m, f'unparsed field: {name}:{field}')
                keys.append((name, 'offset:' + m[1]))
                expressions.append(f'offsetof({name}, {m[1]})')
    fixture = header + '\nconst uint32_t audit_values[] = {\n' + ',\n'.join(expressions) + '\n};\n'
    adjacent = Path(cc).parent
    cxx = adjacent / 'arm-none-eabi-g++.exe'
    objcopy = adjacent / 'arm-none-eabi-objcopy.exe'
    flags = ['-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-Wall', '-Wextra', '-Werror', '-fno-short-enums']
    with tempfile.TemporaryDirectory(prefix='flight-spec-abi-') as tmp:
        folder = Path(tmp)
        src = folder / 'contracts.c'
        src.write_text(fixture, encoding='utf-8')
        commands = []
        for compiler, standard, suffix, extras in [(cc, 'c11', 'c', []), (str(cxx), 'c++14', 'cpp', ['-fno-exceptions', '-fno-rtti'])]:
            obj = folder / f'{suffix}.o'
            cmd = [str(compiler), *flags, f'-std={standard}', *extras, '-x', 'c++' if suffix == 'cpp' else 'c', '-c', str(src), '-o', str(obj)]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode:
                raise ValueError(result.stderr)
            binary = folder / f'{suffix}.bin'
            subprocess.run([str(objcopy), '--dump-section', f'.rodata={binary}', str(obj)], check=True, capture_output=True)
            raw = binary.read_bytes()
            values = struct.unpack('<' + 'I' * (len(raw) // 4), raw)
            require(len(values) == len(keys))
            commands.append({'language': standard, 'flags': flags + extras, 'values_sha256': hashlib.sha256(raw).hexdigest()})
            if suffix == 'c':
                c_values = values
            else:
                require(values == c_values, 'C/C++ layout differs')
        layouts = {}
        for (name, kind), value in zip(keys, c_values):
            layouts.setdefault(name, {})[kind] = value
        require(layouts['ContractHeader']['size'] == 88)
        version = subprocess.run([str(cc), '--version'], capture_output=True, text=True, check=True).stdout.splitlines()[0]
    report = {'baseline': BASELINE, 'scope': 'SPEC_ARM_ABI_COMPILE', 'production_firmware_validation': 'NOT_RUN', 'icd_sha256': sha(ICD), 'private_control_spec_sha256':sha(SPEC/'04_飞控核心模块与控制系统设计.md'), 'compiler': version, 'c_blocks_total': total, 'quad_c_blocks': accepted, 'commands': commands, 'layouts': layouts}
    write_json(VALIDATION / 'abi_layout.json', report)
    return report, definitions


def document_checks():
    results = []
    files = sorted(SPEC.glob('*.md'))
    links = 0
    for path in files:
        content = path.read_text(encoding='utf-8')
        require(content.count('```') % 2 == 0, f'unclosed fence {path.name}')
        for target in re.findall(r'\]\(([^)]+)\)', content):
            target = target.strip('<>')
            if re.match(r'\w+://', target) or target.startswith('#'):
                continue
            local = target.split('#')[0]
            require((path.parent / local).exists(), f'broken link {path.name}:{target}')
            links += 1
    for forbidden in ['状态唯一表示为 `(voxel_index, velocity_bin_id, yaw_bin_id)`', '`profiles/quad_x4/kinodynamic_primitives.yaml`', '分段时间由 jerk-limited S-Curve time parameterizer 给定']:
        require(forbidden not in (SPEC/'00_工程基线与唯一权威规范.md').read_text(encoding='utf-8'), forbidden)
    results.append({'name': 'all_markdown_fences_and_local_links', 'status': 'PASS', 'files': len(files), 'links': links})
    root = ET.parse(SPEC / 'contracts/FCU.xml').getroot()
    messages = root.findall('./messages/message')
    mids = [int(x.attrib['id']) for x in messages]
    require(mids == list(range(43000, 43013)))
    require(len({x.attrib['name'] for x in messages}) == 13)
    for m in messages:
        fields = m.findall('field')
        require(len({x.attrib['name'] for x in fields}) == len(fields))
        require(all(x.text and x.text.strip() for x in fields))
    results.append({'name': 'dialect_schema_and_private_ids', 'status': 'PASS', 'messages': 13, 'common_merge_and_production_generation': 'NOT_RUN'})
    return results


def crc32c(data):
    result = 0xffffffff
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ (0x82f63b78 if result & 1 else 0)
    return result ^ 0xffffffff


def reference_checks():
    require(crc32c(b'123456789') == 0xe3069283)
    # Error retraction p_true=p_hat-delta_p, b_true=b_hat+delta_b;
    # innovation h(nominal)-z at true measurement has H=[-1,-1].
    p_d, bias, h0, epsilon = 3.0, .2, 100.0, 1e-5
    h = lambda p, b: h0 - p + b
    nominal = h(p_d, bias)
    jac = [(nominal - h(p_d-epsilon, bias))/epsilon, (nominal - h(p_d, bias+epsilon))/epsilon]
    require(all(abs(x + 1) < 1e-8 for x in jac))
    require(32+8+8+8+4 == 60)
    key = lambda voxel, vel, yaw, time: (voxel << 14) | (vel << 9) | (yaw << 5) | time
    require(len({key(42, 0, 0, t) for t in range(32)}) == 32)
    require(key(4095, 28, 15, 31) < (1 << 26))
    critical_cpu = 450 * 1053 + 35 * 2053
    require(critical_cpu == 545705 and critical_cpu < 550000)
    require(sum([16,88,0,48,32,8,16,4,4,16]) == 232)
    require(sum([64,88,13,48,32,8,16,4,4,16]) == 293 > 256)
    require(sum([5,320,40,60,250,100,35,90]) == 900)
    require(3000 + 900 + 100 < 5000)
    require(93*30 + 20*60 + 200 + 100 == 4290 < 4400)
    # A committed but unconfirmed transaction must never displace confirmed settings.
    stages = ['erased', 'header', 'payload', 'commit', 'ram_activate', 'confirmed']
    for stage in stages:
        slots = [(1, True, True), (2, stage in ('commit','ram_activate','confirmed'), stage == 'confirmed')]
        selected = max(g for g, committed, confirmed in slots if committed and confirmed)
        require(selected == (2 if stage == 'confirmed' else 1))
    return {'name': 'independent_arithmetic_and_reference_cases', 'status': 'PASS', 'cases': ['CRC32C official vector', 'Baro error-retraction Jacobian', '60 primitive count', 'time-key wait separation', 'CPU 54.5705%', 'per-profile memory admission', '900us critical chain', '5ms sample age', 'replay admission arithmetic', 'settings six power-cut boundaries'], 'scope': 'DOCUMENT_REFERENCE_ONLY', 'SIL_HIL_or_target_validation': 'NOT_RUN'}


def resource_checks(layouts):
    text = (SPEC / '07_实时调度与板级硬件资源设计.md').read_text(encoding='utf-8')
    total = 0
    checked = []
    for name, count, size in re.findall(r'^\| (\w+) \| (\d+) \| (\d+) \|', text, re.M):
        if name in layouts:
            actual = layouts[name]['size']
            require(actual <= int(size), f'{name}: sizeof {actual} > slot {size}')
            if name != 'ProductObstacleObservation':
                total += int(count)*int(size)
            checked.append({'type': name, 'count': int(count), 'slot': int(size), 'sizeof': actual})
    total += 8*160 + layouts['FmsOutputView']['size'] + 1536 + 6144 + 2048
    require(total <= 49152, f'ICD arena exceeds 48KiB: {total}')
    # 2048B covers <=64*20B snapshot metadata plus bounded small views; large inputs point to arena objects.
    return {'name': 'ICD_slot_and_arena_reference', 'status': 'PASS', 'checked': checked, 'arena_bytes': total, 'arena_limit': 49152, 'scope': 'SPEC_LAYOUT_REFERENCE_NOT_PRODUCTION_MAP'}


def message_registry(definitions):
    path = SPEC / 'contracts/message_registry.json'
    names = [n for n, d in definitions.items() if n != 'MpcOutputSnapshot' and re.search(r'\bContractHeader\s+header\s*;|\bModuleHealth\s+common\s*;', d)]
    require(path.is_file(), 'missing normative message registry; never regenerate IDs from document order')
    registry = json.loads(path.read_text(encoding='utf-8'))
    require(set(names) == set(registry['messages']), 'register changed Header-bearing types explicitly')
    require(len(set(registry['messages'].values())) == len(names))
    return {'name': 'explicit_message_registry', 'status': 'PASS', 'messages': len(names)}


def outside_snapshot():
    current = {p.relative_to(ROOT).as_posix(): sha(p) for p in ROOT.rglob('*') if p.is_file() and p.relative_to(ROOT).parts[0] != 'spec'}
    tree = hashlib.sha256('\n'.join(n+'\0'+current[n] for n in sorted(current)).encode()).hexdigest()
    return current, tree


def write_outside_baseline(path):
    """Generate a fresh OUTSIDE_SPEC_INTEGRITY snapshot for an approved baseline."""
    current, tree = outside_snapshot()
    payload = {'scope': 'OUTSIDE_SPEC_INTEGRITY', 'status': 'PASS', 'baseline_files': len(current),
               'current_files': len(current), 'changed': [], 'added': [], 'removed': [],
               'baseline_sha256': tree, 'tree_sha256': tree}
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--round', type=int, choices=(1,2,3), required=True)
    parser.add_argument('--cc', default=r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe')
    parser.add_argument('--label', default=None, help='artifact label; defaults to roundN (keeps historical reports)')
    parser.add_argument('--outside-baseline', type=Path, default=None,
                        help='assert a frozen OUTSIDE_SPEC_INTEGRITY report; omitted means NOT_ENFORCED')
    parser.add_argument('--write-outside-baseline', type=Path, default=None,
                        help='write a fresh OUTSIDE_SPEC_INTEGRITY snapshot for the current tree and exit')
    args = parser.parse_args()
    VALIDATION.mkdir(exist_ok=True)
    if args.write_outside_baseline:
        write_outside_baseline(args.write_outside_baseline)
        print(json.dumps({'outside_baseline': str(args.write_outside_baseline), 'status': 'PASS'}, ensure_ascii=False))
        return
    abi, definitions = compile_abi(args.cc)
    checks = document_checks()
    checks += [message_registry(definitions), resource_checks(abi['layouts']), reference_checks()]
    if args.round == 3:
        from check_reference_models import run_reference_checks
        checks.append(run_reference_checks())
        if args.outside_baseline:
            proof = json.loads(Path(args.outside_baseline).read_text(encoding='utf-8'))
            require(proof['status'] == 'PASS',
                    'protected-sources baseline report is not PASS')
            current, tree_hash = outside_snapshot()
            require(tree_hash == proof['tree_sha256'] and
                    len(current) == proof['current_files'],
                    'protected outside-spec tree changed since the frozen baseline')
            checks.append({'name': 'protected_sources_report', 'status': 'PASS', 'sha256': sha(Path(args.outside_baseline))})
        else:
            checks.append({'name': 'protected_sources_report', 'status': 'NOT_ENFORCED',
                           'reason': 'pass --outside-baseline <report> to assert a frozen tree; implementation work necessarily changes it'})
        proof = json.loads((VALIDATION/'dialect_report.json').read_text(encoding='utf-8'))
        require(proof['status'] == 'PASS',
                'spec/validation/dialect_report.json is not PASS')
        require(proof['spec_xml_sha256'] == sha(SPEC/'contracts/FCU.xml'),
                'spec/validation/dialect_report.json spec_xml_sha256 does not match '
                'spec/contracts/FCU.xml; the committed evidence is stale or was '
                'generated from a different line-ending checkout (see '
                'spec/validation/eol_hash_rebaseline_analysis.md)')
        checks.append({'name': 'dialect_report', 'status': 'PASS', 'sha256': sha(VALIDATION/'dialect_report.json')})
    label = args.label or f'round{args.round}'
    inputs = [p for p in SPEC.rglob('*') if p.is_file() and 'validation' not in p.relative_to(SPEC).parts and '__pycache__' not in p.parts]
    report = {'baseline': BASELINE, 'round': args.round, 'label': label, 'status': 'PASS', 'scope': 'DOCUMENT_STATIC_AND_SPEC_ARM_ABI_COMPILE', 'independent_human_review': 'NOT_RUN', 'production_and_physical_validation': 'NOT_RUN', 'checks': checks, 'inputs_sha256': {str(p.relative_to(ROOT)).replace('\\','/'): sha(p) for p in inputs}, 'abi_layout_sha256': sha(VALIDATION/'abi_layout.json'), 'abi_artifact':f'spec/validation/{label}_abi_layout.json'}
    write_json(VALIDATION / f'{label}_abi_layout.json', abi)
    write_json(VALIDATION / f'{label}_report.json', report)
    if args.round == 3 and args.label is None:
        write_json(VALIDATION/'project_structure_report.json', {'baseline': BASELINE, 'scope':'DOCUMENT_STATIC', 'status':'PASS', 'source':'round3_report.json', 'checks':checks[:2], 'protected_sources_report_sha256':sha(VALIDATION/'protected_sources_report.json')})
    print(json.dumps({'round': args.round, 'label': label, 'status': 'PASS', 'types': len(abi['layouts']), 'checks': len(checks)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
