"""Check the three project source layers, names and dependency boundaries.

The rules mirror section 0 of the platform spec: device model pairs by category, module-owned backends and
private backend headers.
"""
import ast
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
LAYERS = ('sensor_device', 'middleware', 'applications')
SENSORS = {'imu', 'mag', 'barometer', 'pmu', 'gnss'}
DEVICE_MODULES = SENSORS | {'actuator', 'storage', 'telemetry', 'time_base'}
MODULE_LAYOUT = {
    'communication': {'telemetry_link', 'sim'},
    'protocol': {'mavlink', 'nmea', 'unicore_ascii'},
    'sensor': {'sample_acquisition', 'sample_normalization', 'calibration', 'measurement_filter', 'source_health'},
    'guidance': {'guidance_update', 'trajectory_planning', 'pilot_and_landing'},
    'actuator': {'output_guard', 'device_adapter'},
    'storage': {'event_journal'},
    'contracts': {'base_types', 'messages', 'ports', 'product_contract', 'contract_checks'},
}
MODULES = set(MODULE_LAYOUT) | {'model', 'param'}
MODEL_MODULES = {'ins', 'fms', 'control', 'plant'}
KF_GINS = 'model/ins/kf_gins'
OSQP = 'model/control/outer_loop/quad_x4/mpc_solver/osqp'
BACKENDS = (KF_GINS, OSQP)
IMPORTED_TREES = (f'middleware/{KF_GINS}/upstream/', f'middleware/{OSQP}/generated/')
SOURCE_SUFFIXES = {'.c', '.h', '.cpp', '.hpp', '.inc'}
errors = []


def check(condition, message):
    if not condition:
        errors.append(message)


def directories(path):
    return {p.name for p in path.iterdir() if p.is_dir()} if path.is_dir() else set()


def headers(path):
    return {p.name for p in path.rglob('*') if p.suffix in {'.h', '.hpp'}}


sensor = ROOT / 'sensor_device'
middleware = ROOT / 'middleware'
model = middleware / 'model'
application = ROOT / 'applications'
check(directories(sensor) == DEVICE_MODULES, 'expected sensor, actuator, storage, telemetry and time devices')
check(not (ROOT / 'board/actuator').exists(), 'actuator drivers belong under sensor_device/actuator')
storage = sensor / 'storage'
check(directories(storage) == {'W25Q64', 'SDMMC'}, 'expected W25Q64 and SDMMC storage directories')
check(not any(path.suffix in SOURCE_SUFFIXES for path in storage.glob('*')),
      'place storage sources in their media type directory')
check(not (ROOT / 'board/storage').exists(), 'storage drivers belong under sensor_device/storage')
telemetry = sensor / 'telemetry'
telemetry_devices = {'elrs': 'elrs', 'data_radio': 'data_radio', 'sim': 'hil_serial'}
check(directories(telemetry) == set(telemetry_devices), 'expected a directory for each telemetry device')
check(not any(path.suffix in SOURCE_SUFFIXES for path in telemetry.glob('*')),
      'place telemetry sources under their device directory')
for directory, device in telemetry_devices.items():
    path = telemetry / directory
    check(not directories(path) and {p.name for p in path.glob('*') if p.suffix in SOURCE_SUFFIXES} ==
          {f'{device}.c', f'{device}.h'}, f'keep telemetry implementation and header together under {directory}')
check(directories(middleware) == MODULES, 'expected M1-M7, protocol, parameters and contracts')
check(directories(application) == {'tasks', 'runtime'},
      'applications must group tasks and runtime')
for name, groups in MODULE_LAYOUT.items():
    directory = middleware / name
    check(directories(directory) == groups, f'unexpected functional directories under middleware/{name}')
    check(directory.is_dir() and not any(path.suffix in SOURCE_SUFFIXES for path in directory.iterdir()),
          f'place source and headers in the functional directories under middleware/{name}')
param = middleware / 'param'
check(not directories(param) and {p.name for p in param.glob('*') if p.suffix in SOURCE_SUFFIXES} ==
      {'parameter_store.c', 'parameter_store.h'}, 'keep the parameter store directly under middleware/param')
check(directories(model) == MODEL_MODULES, 'unexpected model-layer modules')
for name in MODEL_MODULES:
    for suffix in ('.c', '.h'):
        interface = model / name / f'{name}_interface{suffix}'
        check(interface.is_file(), f'missing public model interface: {interface}')
check(directories(model / 'control') == {'inner_loop', 'outer_loop'},
      'control models must be grouped by inner and outer loop')
for relative in ('ins', 'fms', 'plant', 'control/inner_loop', 'control/outer_loop'):
    check(bool(directories(model / relative)), f'missing model implementation directory: {relative}')
FMS_SHARED = {'fms_contract', 'geofence_geometry'}
check(FMS_SHARED <= directories(model / 'fms'), 'FMS needs shared contract and geometry directories')
for relative, groups in {
    'fms/quad_x4': {'flight_state_machine', 'mission_lifecycle', 'operator_authority', 'flight_safety', 'flight_policy'},
    'ins/kf_gins': {'navigation_estimator', 'observation_replay', 'backend_adapter', 'trimmed_core', 'upstream', 'tools'},
    'control/inner_loop/quad_x4': {'attitude_rate_control', 'control_allocation', 'simulation_config'},
    'control/outer_loop/quad_x4': {'position_tracking', 'prediction_model', 'mpc_solver'},
}.items():
    directory = model / relative
    check(groups <= directories(directory), f'missing functional directories under model/{relative}')
    check(directory.is_dir() and not any(path.suffix in SOURCE_SUFFIXES for path in directory.iterdir()),
          f'place model sources and headers in functional directories under model/{relative}')
for legacy in ('app', 'drivers', 'modules', 'middle'):
    check(not (ROOT / legacy).exists(), f'legacy source layer still exists: {legacy}')
for name in SENSORS:
    directory = sensor / name
    check(not directories(directory), f'keep headers and implementation together: {directory}')
    implementations = {p.stem for p in directory.glob('*.c')}
    device_headers = {p.stem for p in directory.glob('*.h')}
    check(bool(implementations) and implementations == device_headers,
          f'expected matching model source/header pairs under {name}')

task_names = {'sensor', 'mission', 'mpc', 'actuator', 'supervisor', 'logger', 'flash'}
task_root = application / 'tasks'
check({p.stem for p in application.glob('*.c')} == {'main'},
      'keep only the entry point at the application root')
check(directories(task_root) == task_names | {'communication'}, 'keep one directory per task family')
check({p.name for p in task_root.glob('*') if p.is_file() and p.suffix in SOURCE_SUFFIXES} ==
      {'task_manager.c', 'task_config.h'}, 'keep shared task management and configuration at the task root')
for name in sorted(task_names):
    check((task_root / name / f'task_{name}.c').is_file(), f'missing task implementation under tasks/{name}')
check({p.stem for p in (task_root / 'communication').glob('*.c')} ==
      {'task_comm', 'task_rc', 'task_radio', 'task_gcs', 'task_sim'},
      'keep the communication dispatcher, RC, radio, GCS and sim in separate files')
check(not directories(application / 'runtime'), 'keep runtime sources in one directory')
config = ROOT / 'config'
check(not directories(config), 'keep the selected product parameters directly in config')
check({p.stem for p in config.glob('*.yaml')} ==
      {'vehicle_model', 'sensor_alignment', 'estimator_tuning', 'controller_tuning',
       'calibration', 'kinodynamic_primitives'},
      'expected five CORE configuration files plus the M5 primitive manifest under config')

app_headers = headers(application)
sensor_headers = headers(sensor)
device_private_headers = sensor_headers - {'systime.h', 'sensor_reading.h'}
business_headers = set().union(*(headers(middleware / name)
                                 for name in MODULES - {'contracts'}))
backend_headers = {
    'middleware/model/ins/': (headers(middleware / KF_GINS) - headers(middleware / KF_GINS / 'upstream')) | {'ins_model.h'},
    'middleware/model/fms/': set().union(*(headers(model / 'fms' / name)
                                     for name in directories(model / 'fms') - FMS_SHARED))
    | {'fms_model.h'},
    'middleware/model/control/': (headers(model / 'control') - headers(middleware / OSQP / 'generated/inc/private'))
    - {'control_interface.h'},
}
prefix = re.compile(r'\b(?:fc_\w+|FC_\w+|Fc[A-Z]\w*)\b')
model_identifier = re.compile(r'(?<![a-z0-9])(?:quad_x4|kf_gins)(?![a-z0-9])', re.I)
for folder in ('board', 'config', 'tools'):
    for path in (ROOT / folder).rglob('*'):
        if 'CubeMX_Config' not in path.parts:
            check(not model_identifier.search(path.name),
                  f'model-specific filename outside model implementation: {path.relative_to(ROOT)}')
for layer in LAYERS:
    for path in (ROOT / layer).rglob('*'):
        relative = path.relative_to(ROOT).as_posix()
        check(path.suffix not in {'.o', '.su'}, f'build output in source tree: {relative}')
        if path.suffix not in SOURCE_SUFFIXES or relative.startswith(IMPORTED_TREES):
            continue
        check(re.fullmatch(r'[a-z][a-z0-9]*(?:_[a-z0-9]+)*', path.stem),
              f'use lower snake_case: {relative}')
        source = path.read_text(encoding='utf-8')
        code = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
        check(not prefix.search(code), f'legacy Flight prefix: {relative}')
        includes = {Path(name).name for name in re.findall(r'^\s*#include\s+[<"]([^>"]+)', code, re.M)}
        if not relative.startswith('middleware/model/'):
            check(not model_identifier.search(path.stem) and not model_identifier.search(code),
                  f'model-specific identifiers belong to the model implementation: {relative}')
        if relative.startswith('middleware/protocol/'):
            check(not (includes & (sensor_headers | app_headers | {'rtthread.h', 'rtdevice.h', 'sensor_sample.h'})),
                  f'protocol codecs must be independent of RT-Thread and sensor drivers: {relative}')
        for owner, private_headers in backend_headers.items():
            if not relative.startswith(owner):
                check(not (includes & private_headers),
                      f'backend headers are private to {owner}: {relative}')
        if layer == 'applications':
            check(not (includes & device_private_headers),
                  f'application depends on a physical device: {relative}')
            check(not re.search(r'\b(?:HAL_\w+|LL_\w+|TIM\d+|rt_device_\w+)\b', code),
                  f'device access belongs in a middleware adapter: {relative}')
        else:
            check(not includes & app_headers, f'lower layer depends on applications: {relative}')
        if layer == 'sensor_device':
            check(not includes & business_headers, f'sensor depends on business code: {relative}')
        if layer == 'middleware':
            check(not re.search(r'\b(?:HAL_\w+|LL_\w+|TIM\d+)\b', code),
                  f'hardware control belongs in the device or board layer: {relative}')
        if relative.startswith('middleware/communication/'):
            check(not re.search(r'\brt_device_\w+\b', code),
                  f'telemetry UART ownership belongs under sensor_device/telemetry: {relative}')
        if relative.startswith('middleware/actuator/output_guard/'):
            check(not includes & {'dshot.h', 'dshot_packet.h'},
                  f'DShot framing belongs at the device adapter: {relative}')
        if relative != 'sensor_device/time_base/timer_capture.c':
            check('TIM2->' not in code, f'TIM2 register access belongs in its driver: {relative}')

for relative in ('SConstruct', 'SConscript', 'board/SConscript', 'tests/target/SConscript',
                 *(f'{layer}/SConscript' for layer in LAYERS),
                 'middleware/model/SConscript', *(f'middleware/{backend}/SConscript' for backend in BACKENDS)):
    path = ROOT / relative
    check(path.is_file(), f'missing build entry: {relative}')
    if path.is_file():
        ast.parse(path.read_text(encoding='utf-8'), filename=relative)
for relative in ('Kconfig', 'middleware/Kconfig', 'middleware/model/Kconfig', 'applications/Kconfig'):
    path = ROOT / relative
    for reference in re.findall(r'^rsource "([^"]+)"', path.read_text(encoding='utf-8'), re.M):
        check((path.parent / reference).is_file(), f'broken Kconfig path: {relative} -> {reference}')

report = {'layout': 'SENSOR_DEVICE_MIDDLEWARE_APPLICATIONS', 'status': 'FAIL' if errors else 'PASS',
          'errors': errors, 'middleware_modules': sorted(MODULES),
          'model_modules': sorted(MODEL_MODULES), 'replaceable_backends': sorted(BACKENDS),
          'sensor_devices': sorted(SENSORS)}
output = ROOT / 'build/three_layer_refactor/layout_checks.json'
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
if errors:
    raise SystemExit('\n'.join(errors))
print('PASS: three source layers, device/module names and driver boundaries')
