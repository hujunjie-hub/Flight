"""Host execution + Cortex-M7 object/relocatable link checks. No board output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT/'build/validation/controller_navigation'
OUT.mkdir(parents=True, exist_ok=True)
ap = argparse.ArgumentParser()
ap.add_argument('--compile-only', action='store_true')
ap.add_argument('--target', choices=['host', 'arm', 'all'], default='all')
args = ap.parse_args()
EXE_EXT = '.exe' if os.name == 'nt' else ''


def tool_path(target, name):
    """Resolve native or ARM tools without assuming a Windows installation path."""
    prefix = 'arm-none-eabi-' if target == 'arm' else ''
    executable = prefix + name + EXE_EXT
    env_name = 'RTT_EXEC_PATH' if target == 'arm' else 'FLIGHT_HOST_BIN'
    candidates = []
    configured_dir = os.environ.get(env_name)
    if configured_dir:
        candidates.append(Path(configured_dir) / executable)
    discovered = shutil.which(executable)
    if discovered:
        candidates.append(Path(discovered))
    if os.name == 'nt':
        fallback = (r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin'
                    if target == 'arm' else r'D:\STM32CubeCLT\mingw64\bin')
        candidates.append(Path(fallback) / executable)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise SystemExit(
        f"Cannot find {executable}. Install the toolchain, add it to PATH, "
        f"or set {env_name} to its bin directory."
    )


includes = [
    'middleware/contracts',
    'middleware/model/fms',
    'middleware/model/ins',
    'middleware/model/control',
    'middleware/model/control/inner_loop/quad_x4',
    'middleware/model/ins/kf_gins',
    'middleware/model/ins/kf_gins/trimmed_core/include',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/public',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/private',
]
sources = []
for pattern in (
    'middleware/model/control/*.c',
    'middleware/model/control/inner_loop/quad_x4/*/*.c',
    'middleware/model/control/outer_loop/quad_x4/*/*.c',
    'middleware/model/ins/*.c',
    'middleware/model/ins/kf_gins/navigation_estimator/*.cpp',
    'middleware/model/ins/kf_gins/observation_replay/*.cpp',
    'middleware/model/ins/kf_gins/trimmed_core/src/*.cpp',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/src/*.c',
):
    sources.extend(sorted(ROOT.glob(pattern)))
sources.extend(ROOT / path for path in (
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/quad_workspace.c',
    'middleware/model/ins/kf_gins/backend_adapter/kf_gins_adapter.cpp',
))
commands = []
results = {
    'host_tests': 'NOT_RUN',
    'm3_bridge_tests': 'NOT_RUN',
    'arm_compile_link': 'NOT_RUN',
    'target_hardware': 'NOT_RUN',
    'flight': 'NOT_RUN',
}
lock = json.loads((
    ROOT / 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/UPSTREAM_LOCK.json'
).read_text())
if lock.get('schema_version') != 2 or lock.get('hash_algorithm') != 'git-blob-sha1':
    raise SystemExit('Unsupported OSQP source-lock schema or hash algorithm')
osqp_root = ROOT / 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated'
osqp_mismatches = []
# Verify the Git blob hashing implementation against the standard "abc" vector.
if hashlib.sha1(b'blob 3' + bytes((0,)) + b'abc').hexdigest() != 'f2ba8f84ab5c1bce84a7b441cb1959cfc7093b7f':
    raise SystemExit('Git blob SHA-1 self-test failed')

for rel, entry in sorted(lock['files'].items()):
    path = osqp_root / rel
    if not path.is_file():
        osqp_mismatches.append((rel, entry.get('git_blob_sha1', 'MISSING'), 'FILE_MISSING'))
        continue
    content = path.read_bytes()
    # Git object identity includes a NUL byte after the '<type> <size>' header.
    git_object = b'blob ' + str(len(content)).encode('ascii') + bytes((0,)) + content
    actual = hashlib.sha1(git_object).hexdigest()
    expected = entry.get('git_blob_sha1')
    if actual != expected:
        osqp_mismatches.append((rel, expected, actual))
        continue
    provenance = entry.get('provenance')
    if provenance == 'upstream':
        upstream = entry.get('upstream', {})
        if upstream.get('blob_sha1') != expected:
            osqp_mismatches.append((rel, 'UPSTREAM_PROVENANCE_MISMATCH', upstream.get('blob_sha1')))
    elif provenance == 'local_patch':
        if not entry.get('upstream', {}).get('blob_sha1') or not entry.get('patch_summary'):
            osqp_mismatches.append((rel, 'PATCH_METADATA_REQUIRED', 'MISSING'))
    elif provenance == 'generated':
        if not entry.get('generator'):
            osqp_mismatches.append((rel, 'GENERATOR_METADATA_REQUIRED', 'MISSING'))
    else:
        osqp_mismatches.append((rel, 'KNOWN_PROVENANCE_REQUIRED', str(provenance)))
if osqp_mismatches:
    print('OSQP embedded source/provenance lock mismatches:')
    for rel, expected, actual in osqp_mismatches:
        print(f'  {rel}: expected={expected} actual={actual}')
    raise SystemExit(f'{len(osqp_mismatches)} OSQP source lock mismatch(es)')


def run(cmd):
    command = list(map(str, cmd))
    try:
        process = subprocess.run(
            command, cwd=ROOT, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True
        )
    except OSError as error:
        commands.append({'command': command, 'exit_code': None, 'output': str(error)})
        raise SystemExit(f'Cannot run {command[0]}: {error}') from error
    commands.append({
        'command': command,
        'exit_code': process.returncode,
        'output': process.stdout,
    })
    if process.stdout:
        print(process.stdout, end='')
    if process.returncode:
        (OUT / 'commands.json').write_text(json.dumps(commands, indent=2))
        raise SystemExit(process.returncode)
    return process.stdout


targets = ('host', 'arm') if args.target == 'all' else (args.target,)
for target in targets:
    objdir = OUT / target
    objdir.mkdir(exist_ok=True)
    objects = []
    for source in sources:
        obj = objdir / (
            str(source.relative_to(ROOT)).replace('\\', '_').replace('/', '_') + '.o'
        )
        cpp = source.suffix == '.cpp'
        compiler = tool_path(target, 'g++' if cpp else 'gcc')
        flags = [
            '-O2', '-g', '-Wall', '-Wextra', '-Wshadow', '-fno-short-enums',
            '-ffunction-sections', '-fdata-sections',
            '-std=c++14' if cpp else '-std=c99',
        ]
        if cpp:
            flags += ['-fno-exceptions', '-fno-rtti', '-fno-threadsafe-statics']
        if 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/' not in source.as_posix():
            flags += ['-Werror']
        if target == 'arm':
            flags += [
                '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16',
                '-mfloat-abi=hard', '-fstack-usage',
            ]
        run([
            compiler, *flags, *[f'-I{ROOT / include}' for include in includes],
            '-c', source, '-o', obj,
        ])
        objects.append(obj)

    linker = tool_path(target, 'g++')
    if target == 'host' and not args.compile_only:
        test = ROOT / 'tests/algorithms/test_algorithms.cpp'
        exe = objdir / f'test_algorithms{EXE_EXT}'
        run([
            linker, '-std=c++14', '-O2', '-static', '-static-libgcc',
            '-static-libstdc++',
            *[f'-I{ROOT / include}' for include in includes],
            test, *objects, '-o', exe,
        ])
        output = run([exe])
        (OUT / 'host_tests.log').write_text(output)
        results['host_tests'] = 'PASS'

        bridge = ROOT / 'tests/navigation/test_m3_bridge.cpp'
        bridge_exe = objdir / f'test_m3_bridge{EXE_EXT}'
        bridge_objects = [
            obj for obj in objects
            if any(token in obj.name for token in (
                'ins_replay.cpp', 'ins_observation.cpp', 'kf_gins_trimmed_core_src_'
            ))
        ]
        run([
            linker, '-std=c++14', '-O2', '-static', '-static-libgcc',
            '-static-libstdc++',
            *[f'-I{ROOT / include}' for include in includes],
            bridge, *bridge_objects, '-o', bridge_exe,
        ])
        (OUT / 'bridge_tests.log').write_text(run([bridge_exe]))
        results['m3_bridge_tests'] = 'PASS'

    if target == 'arm':
        run([
            linker, '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16',
            '-mfloat-abi=hard', '-r', *objects, '-o', objdir / 'algorithms.o',
        ])
        run([tool_path('arm', 'size'), objdir / 'algorithms.o'])
        symbols = run([tool_path('arm', 'nm'), '-u', objdir / 'algorithms.o'])
        forbidden = ['malloc', 'calloc', 'realloc', '_Zn', '__cxa_throw', '_Unwind', '__atomic_']
        if any(symbol in symbols for symbol in forbidden):
            raise SystemExit('Unexpected heap, exception or atomic runtime dependency')
        objdump = run([tool_path('arm', 'objdump'), '-h', objdir / 'algorithms.o'])
        if '.init_array' in objdump:
            raise SystemExit('Unexpected global C++ initialization')
        mpc_objects = [
            obj for obj in objects
            if obj.name.endswith(('quad_workspace.c.o', 'mpc.c.o'))
        ]
        if len(mpc_objects) != 2:
            raise SystemExit(f'Expected two MPC objects, found {len(mpc_objects)}')
        sizes = run([tool_path('arm', 'size'), *mpc_objects])
        ram = sum(
            int(line.split()[1]) + int(line.split()[2])
            for line in sizes.splitlines()[1:] if line.strip()
        )
        if ram > 32768:
            raise SystemExit(f'MPC static RAM exceeds 32 KiB: {ram}')
        results.update(
            arm_compile_link='PASS',
            mpc_static_ram_bytes=ram,
            mpc_static_ram_budget_bytes=32768,
        )

(OUT / 'commands.json').write_text(json.dumps(commands, indent=2))
inputs = set(sources)
for folder in (
    'middleware/contracts', 'middleware/model/fms', 'middleware/model/control',
    'middleware/model/ins', 'middleware/model/ins/kf_gins/trimmed_core/include',
    'config', 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc',
):
    inputs.update(
        path for path in (ROOT / folder).rglob('*')
        if path.suffix in ('.h', '.hpp', '.inc', '.yaml', '.py')
    )
(OUT / 'source_hashes.json').write_text(json.dumps({
    str(source.relative_to(ROOT)): hashlib.sha256(source.read_bytes()).hexdigest()
    for source in sorted(inputs)
}, indent=2))
(OUT / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
print(f"PASS: {args.target} validation; target hardware and flight remain NOT_RUN")
