"""Run real Port/runtime integration on a deterministic Host scheduler."""
import hashlib
import argparse
import json
import os
from pathlib import Path
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
if not __debug__:
    raise SystemExit('Run the validation without Python -O.')
parser=argparse.ArgumentParser()
parser.add_argument('--sanitize',action='store_true')
args=parser.parse_args()
OUT=ROOT/('build/firmware_integration/host_sanitized' if args.sanitize else 'build/firmware_integration/host')
OUT.mkdir(parents=True,exist_ok=True)
summary_path=OUT/'summary.json'
summary_path.write_text(json.dumps({'runtime_integration':'INCOMPLETE'})+'\n',encoding='utf-8')
BIN=Path(os.environ.get('FLIGHT_HOST_BIN','D:/STM32CubeCLT/mingw64/bin' if os.name=='nt' else '/usr/bin'))
EXT='.exe' if os.name=='nt' else ''
includes=['tests/runtime/stubs', 'applications/tasks', 'applications/runtime',
          'middleware/contracts', 'middleware/communication', 'middleware/protocol', 'middleware/param',
          'middleware/sensor', 'middleware/storage', 'middleware/model/ins', 'middleware/model/fms',
          'middleware/model/fms/quad_x4', 'middleware/guidance',
          'middleware/model/control', 'middleware/model/control/inner_loop/quad_x4',
          'middleware/actuator', 'sensor_device', 'sensor_device/time_base',
          'libraries/HAL_Drivers/drivers', 'sensor_device/actuator',
          'sensor_device/imu', 'sensor_device/mag', 'sensor_device/barometer',
          'sensor_device/pmu', 'sensor_device/gnss',
          'middleware/model/ins/kf_gins', 'middleware/model/ins/kf_gins/trimmed_core/include',
          'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated', 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/public',
          'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/private', 'board']
sources = []
for pattern in ('middleware/sensor/*/*.c', 'middleware/model/fms/*.c', 'middleware/model/fms/quad_x4/*/*.c',
                'middleware/guidance/*/*.c', 'middleware/model/control/*.c', 'middleware/model/ins/*.c',
                'middleware/model/control/inner_loop/quad_x4/*/*.c', 'middleware/model/control/outer_loop/quad_x4/*/*.c',
                'middleware/model/ins/kf_gins/navigation_estimator/*.cpp', 'middleware/model/ins/kf_gins/observation_replay/*.cpp',
                'middleware/model/ins/kf_gins/trimmed_core/src/*.cpp', 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/src/*.c'):
    sources.extend(sorted(ROOT.glob(pattern)))
sources.extend(ROOT/path for path in (
    'middleware/model/fms/geofence_geometry/geofence_geometry.c', 'middleware/model/ins/kf_gins/backend_adapter/kf_gins_adapter.cpp',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/quad_workspace.c', 'middleware/protocol/mavlink/mavlink_frame.c',
    'middleware/protocol/mavlink/mavlink_hil.c',
    'middleware/protocol/mavlink/mavlink_action.c',
    'middleware/communication/sim/hil_link.c',
    'middleware/param/parameter_store.c',
    'middleware/storage/event_journal/event_journal.c',
    'middleware/protocol/unicore_ascii/unicore_ascii.c',
    'middleware/actuator/device_adapter/actuator_device.c', 'middleware/actuator/output_guard/actuator_guard.c',
    'libraries/HAL_Drivers/drivers/drv_wdt.c',
    'sensor_device/gnss/um982.c', 'sensor_device/imu/adis16505.c',
    'sensor_device/barometer/bmp585.c',
    'sensor_device/telemetry/elrs/elrs.c', 'sensor_device/telemetry/data_radio/data_radio.c',
    'sensor_device/telemetry/sim/hil_serial.c',
    'tests/runtime/test_runtime.c'))
commands=[]
inputs=set(sources)|{Path(__file__).resolve()}
for directory in includes:
    for suffix in ['*.h','*.hpp','*.inc']:
        inputs.update((ROOT/directory).glob(suffix))
inputs.update((ROOT/'applications/runtime').glob('*.c')) # included by scheduler fixture
inputs.update(path for path in (ROOT/'applications/tasks').rglob('*') if path.suffix in {'.c', '.h'})
inputs.update((ROOT/'tests/runtime/stubs/drivers').glob('*.h'))
for folder in ('contracts', 'communication', 'protocol', 'sensor', 'guidance', 'actuator', 'storage',
               'model/fms', 'model/control/inner_loop/quad_x4',
               'model/ins/kf_gins/observation_replay', 'model/ins/kf_gins/backend_adapter'):
    inputs.update(path for path in (ROOT/'middleware'/folder).rglob('*') if path.suffix in {'.h', '.hpp'})
inputs.add(ROOT/'middleware/communication/telemetry_link/telemetry_link.c')
inputs.update((ROOT/'sensor_device/telemetry').rglob('*.h'))
inputs.add(ROOT/'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/mpc_sparsity_pattern.h')
inputs.update((ROOT/'middleware/model/ins/kf_gins/trimmed_core/include').rglob('*.hpp'))
source_hashes={s.relative_to(ROOT).as_posix():hashlib.sha256(s.read_bytes()).hexdigest() for s in sorted(inputs)}
def run(cmd):
    command=list(map(str,cmd))
    try:
        p=subprocess.run(command,cwd=ROOT,capture_output=True,text=True)
    except OSError as error:
        commands.append({'command':command,'exit_code':None,'output':str(error)})
        raise SystemExit(f'Cannot run {command[0]}: {error}') from error
    commands.append({'command':command,'exit_code':p.returncode,'output':p.stdout+p.stderr})
    if p.returncode:
        print(p.stdout+p.stderr);raise SystemExit(p.returncode)
    return p.stdout
try:
    objects=[]
    for s in sources:
        obj=OUT/(s.relative_to(ROOT).as_posix().replace('/','_')+'.o')
        cpp=s.suffix=='.cpp'
        flags=['-std=c++14' if cpp else '-std=c99','-O2','-g','-Wall','-Wextra',
               '-Wno-unused-function','-fno-short-enums','-DFLIGHT_HOST_TEST','-DFLIGHT_LOGGER_ENABLE']
        if 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/' not in s.as_posix(): flags+=['-Werror']
        if s.name in {'adis16505.c', 'bmp585.c'}: flags+=['-DFLIGHT_DRIVER_CODEC_TEST']
        if cpp: flags+=['-fno-exceptions','-fno-rtti','-fno-threadsafe-statics']
        if args.sanitize:
            flags+=['-O1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
        run([BIN/(('g++' if cpp else 'gcc')+EXT),*flags,*['-I'+str(ROOT/i) for i in includes],'-c',s,'-o',obj])
        objects.append(obj)
    exe=OUT/('test_runtime'+EXT)
    linkflags=['-fsanitize=address,undefined'] if args.sanitize else ['-static','-static-libgcc','-static-libstdc++']
    run([BIN/('g++'+EXT),*linkflags,*objects,'-lm','-o',exe])
    output=run([exe])
finally:
    (OUT/'commands.json').write_text(json.dumps(commands,indent=2),encoding='utf-8')

for relative,digest in source_hashes.items():
    assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest, 'source changed during run: '+relative
print(output,end='')
log=OUT/'test_runtime.log'
log.write_text(output,encoding='utf-8')
summary_path.write_text(json.dumps({'runtime_integration':'PASS','hardware':'NOT_RUN',
 'flight':'NOT_RUN','log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),
 'source_hashes':source_hashes},indent=2)+'\n',encoding='utf-8')
