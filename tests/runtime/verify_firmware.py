"""Strict ARM checks, real default SCons build, and a traceable artifact manifest."""
from datetime import datetime, timedelta, timezone
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
if not __debug__:
    raise SystemExit('Run the validation without Python -O.')
OUT=ROOT/'build/firmware_integration'
compiler=shutil.which('arm-none-eabi-gcc')
compiler_dir=Path(compiler).parent if compiler else Path(r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin')
ARM=(ROOT/(os.environ.get('RTT_EXEC_PATH') or compiler_dir)).resolve()
EXT='.exe' if os.name=='nt' else ''
RTT=(ROOT/(os.environ.get('RTT_ROOT') or 'rt-thread')).resolve()
OUT.mkdir(parents=True,exist_ok=True)
strict=OUT/'strict_arm';strict.mkdir(exist_ok=True)
manifest_path=OUT/'manifest.json'
manifest_path.write_text(json.dumps({'full_arm_build':'INCOMPLETE'})+'\n',encoding='utf-8')
commands=[]

def arm_tool(name):
    return ARM/('arm-none-eabi-'+name+EXT)

def scons_python():
    interpreter='Scripts/python.exe' if os.name=='nt' else 'bin/python'
    candidates=(Path(sys.executable), ROOT/'.venv'/interpreter, ROOT/'build/m4_m5_tools'/interpreter)
    for candidate in candidates:
        if candidate.is_file() and subprocess.run([candidate,'-c','import SCons'],capture_output=True).returncode==0:
            return candidate
    raise SystemExit('Install SCons in the active Python environment or the project .venv.')

def capture_source_hashes():
    inputs={ROOT/name for name in ('.config','rtconfig.h','rtconfig.py','README.md','PROJECT_STRUCTURE.md',
            'Kconfig','SConstruct','SConscript','tools/check_layout.py','spec/contracts/FCU.xml',
            'packages/deps.lock','packages/SConscript','tools/flight_mavsdk.py')}
    suffixes={'.c','.h','.cpp','.hpp','.s','.S','.inc','.lds','.py','.md','.json','.yaml','.ioc','.conf'}
    for folder in ('applications','middleware','sensor_device','board','libraries/HAL_Drivers',
                   'tests/runtime','tests/target','tests/algorithms','config'):
        inputs.update(p for p in (ROOT/folder).rglob('*') if p.is_file() and
                      (p.suffix in suffixes or p.name in {'Kconfig','SConscript'}))
    inputs.update(ROOT/path for path in (
        'rt-thread/components/legacy/usb/Kconfig',
        'rt-thread/components/legacy/usb/usbdevice/class/cdc_vcom.c',
        'rt-thread/components/legacy/usb/usbdevice/core/usbdevice.c',
        'tests/drivers/check_drivers.py'))
    return {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(inputs)}

def capture_artifacts():
    artifacts={}
    for name in ('Flight.elf','Flight.bin','Flight.map'):
        content=(ROOT/name).read_bytes()
        artifacts[name]={'bytes':len(content),'sha256':hashlib.sha256(content).hexdigest()}
    return artifacts

source_hashes=capture_source_hashes()
generation=json.loads((ROOT/'middleware/protocol/mavlink/wire_generation.json').read_text())
assert generation['input_sha256']==hashlib.sha256((ROOT/'spec/contracts/FCU.xml').read_bytes()).hexdigest()
assert generation['output_sha256']==hashlib.sha256((ROOT/'middleware/protocol/mavlink/fcu_mavlink_layout.h').read_bytes()).hexdigest()
config=(ROOT/'.config').read_text(encoding='utf-8')
header=(ROOT/'rtconfig.h').read_text(encoding='utf-8')
enabled=['FLIGHT_RUNTIME_ENABLE','RT_USING_ULOG','RT_USING_MESSAGEQUEUE',
         'INS_BACKEND_ENABLE','SENSOR_PROCESSING_ENABLE','INS_REPLAY_ENABLE',
         'SENSOR_INTEGRITY_ENABLE','FMS_ENABLE','GUIDANCE_ENABLE','CONTROLLER_ENABLE']
def config_enabled(symbol):
    configured=bool(re.search(r'^CONFIG_'+symbol+r'=y$',config,re.M))
    compiled=bool(re.search(r'^#define\s+'+symbol+r'\s*$',header,re.M))
    assert configured==compiled, f'{symbol} differs between .config and rtconfig.h'
    return compiled

for symbol in enabled:
    assert config_enabled(symbol),symbol

def run(cmd,name,env=None):
    p=subprocess.run(list(map(str,cmd)),cwd=ROOT,capture_output=True,text=True,env=env)
    (OUT/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
    commands.append({'name':name,'command':list(map(str,cmd)),'exit_code':p.returncode,
                     'log':f'build/firmware_integration/{name}.log'})
    (OUT/'verification_commands.json').write_text(json.dumps(commands,indent=2),encoding='utf-8')
    if p.returncode:
        print((p.stdout+p.stderr)[-4000:]);raise SystemExit(p.returncode)
    return p.stdout
includes=['.', 'board', 'board/CubeMX_Config/Inc', 'libraries/HAL_Drivers',
 'libraries/HAL_Drivers/drivers', 'libraries/HAL_Drivers/drivers/config',
 'packages/stm32h7_hal_driver-latest/Inc', 'packages/stm32h7_cmsis_driver-latest/Include',
 'packages/CMSIS-Core-latest/Include', 'rt-thread/include', 'rt-thread/components/drivers/include',
 'rt-thread/components/drivers/hwcrypto',
 'rt-thread/components/finsh', 'rt-thread/components/utilities/ulog', 'rt-thread/libcpu/arm/cortex-m7',
 'applications/runtime', 'applications/tasks', 'middleware/contracts', 'middleware/communication', 'middleware/protocol',
 'middleware/param',
 'middleware/sensor', 'middleware/storage', 'middleware/model/ins', 'middleware/model/fms', 'middleware/guidance',
 'middleware/model/control', 'middleware/actuator', 'sensor_device', 'sensor_device/time_base',
 'sensor_device/actuator',
 'sensor_device/imu', 'sensor_device/mag', 'sensor_device/barometer',
 'sensor_device/pmu', 'sensor_device/gnss']
flags=['-std=c99','-O2','-Wall','-Wextra','-Werror','-fno-short-enums','-fstack-usage',
 '-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-DSTM32H723xx','-DUSE_HAL_DRIVER','-Dgcc']
sources=sorted((ROOT/'applications/runtime').glob('*.c'))
hil_enabled=config_enabled('FLIGHT_SIM_LINK')
logger_enabled=config_enabled('FLIGHT_LOGGER_ENABLE')
nor_enabled=config_enabled('FLIGHT_NOR_LOG_ENABLE')
tuning_enabled=config_enabled('FLIGHT_BENCH_TUNING')
usb_enabled=config_enabled('BSP_USING_USBD')
config_enabled('FLIGHT_TARGET_UTEST')
assert not config_enabled('FLIGHT_SKIP_ABI_LOCK'), 'release verification requires the ABI lock'
if logger_enabled:
    includes.extend(('rt-thread/components/dfs/dfs_v1/include',
                     'rt-thread/components/libc/compilers/common/include',
                     'rt-thread/components/libc/compilers/common/extension/fcntl/octal'))
sources.extend(path for path in sorted((ROOT/'applications/tasks').rglob('*.c'))
               if (hil_enabled or path.name != 'task_sim.c') and
                  (logger_enabled or path.parent.name != 'logger') and
                  (nor_enabled or path.parent.name != 'flash'))
if nor_enabled:
    sources.append(ROOT/'middleware/storage/event_journal/event_journal.c')
if tuning_enabled:
    sources.append(ROOT/'middleware/param/parameter_store.c')
if hil_enabled:
    sources.extend((ROOT/'middleware/communication/sim/hil_link.c', ROOT/'middleware/protocol/mavlink/mavlink_hil.c',
                    ROOT/'sensor_device/telemetry/sim/hil_serial.c'))
sources += [ROOT/p for p in [
 'middleware/communication/telemetry_link/telemetry_link.c',
 'sensor_device/telemetry/elrs/elrs.c', 'sensor_device/telemetry/data_radio/data_radio.c',
 'middleware/protocol/mavlink/mavlink_frame.c',
 'middleware/protocol/mavlink/mavlink_action.c',
 'middleware/protocol/nmea/nmea.c',
 'middleware/sensor/sample_acquisition/sensor_baro_mag.c',
 'middleware/sensor/sample_acquisition/sensor_gnss.c',
 'middleware/actuator/output_guard/actuator_guard.c','middleware/protocol/unicore_ascii/unicore_ascii.c']]
include_flags=['-I'+str(RTT/Path(p).relative_to('rt-thread') if p.startswith('rt-thread/') else ROOT/p)
               for p in includes]
for source in sources:
    # FinSH stores commands through its generic syscall function pointer.
    export_flags=['-Wno-cast-function-type'] if source.name=='runtime.c' else []
    if source.name=='parameter_store.c':
        # The generated catalog also contains the boot configuration helper.
        export_flags.append('-Wno-unused-function')
    run([arm_tool('gcc'),*flags,*export_flags,*include_flags,
         '-c',source,'-o',strict/(source.stem+'.o')],'strict_'+source.stem)
    if source.name=='unicore_ascii.c':
        undefined=run([arm_tool('nm'),'-u',strict/(source.stem+'.o')],'native_symbols')
        assert not any(x in undefined for x in ['malloc','calloc','realloc','strtod','strtoul'])
build_env={**os.environ,'RTT_BUILD':'release','RTT_CC':'gcc','RTT_EXEC_PATH':str(ARM)}
toolchain=run([arm_tool('gcc'),'--version'],'toolchain').splitlines()[0]
run([scons_python(),'-m','SCons','-Q','-j8'],'release_build',env=build_env)
symbols=run([arm_tool('nm'),'-n','Flight.elf'],'symbols')
required=['runtime_start','runtime_data_get_heartbeat','estimator_push_imu','estimator_update',
          'fms_policy_update','guidance_update','controller_update','control_outer_update','actuator_poll',
          'ins_get_pending_origin','fms_authorize_origin_reset','runtime_take_event']
if tuning_enabled:
    required.extend(('parameter_store_set', 'runtime_apply_controller_parameters'))
if nor_enabled:
    required.append('event_journal_append')
if usb_enabled:
    required.extend(('OTG_HS_IRQHandler', 'board_usb_prepare'))
for name in required:
    assert re.search(r'\bT '+name+r'\r?$',symbols,re.M),name
if not tuning_enabled:
    assert not re.search(r'\b(?:parameter_store_set|runtime_apply_controller_parameters)$',symbols,re.M)
assert (ROOT/'Flight.bin').stat().st_size<=1024*1024
size=run([arm_tool('size'),'Flight.elf'],'size')
sections=run([arm_tool('readelf'),'-SW','Flight.elf'],'sections')
cores=re.search(r'\.estimator_cores\s+NOBITS\s+([0-9a-f]+)\s+[0-9a-f]+\s+([0-9a-f]+)',sections)
assert cores, 'estimator cores must have a NOLOAD section'
core_address,core_bytes=(int(value,16) for value in cores.groups())
assert 0 < core_bytes <= 30*1024 and 0x20000000 <= core_address <= 0x20020000-core_bytes
axi_end=int(re.search(r'^([0-9a-f]+) B _end$',symbols,re.M).group(1),16)
axi_headroom=0x24050000-axi_end
assert axi_headroom>=64*1024
artifacts=capture_artifacts()
checks={}
for key,path in [('runtime','host/summary.json'),('sanitizers','host_sanitized/summary.json'),('wire','wire/summary.json')]:
    report=json.loads((OUT/path).read_text(encoding='utf-8'))
    result=report.get('runtime_integration',report.get('wire_comparison'))
    assert result=='PASS',(key,report)
    for relative,digest in report.get('source_hashes',{}).items():
        assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest, 'stale '+key+': '+relative
    if key=='wire':
        assert report['input_sha256']==generation['input_sha256']
        assert report['host_log_sha256']==hashlib.sha256((OUT/'host/test_runtime.log').read_bytes()).hexdigest()
    else:
        log=OUT/Path(path).parent/'test_runtime.log'
        assert report['log_sha256']==hashlib.sha256(log.read_bytes()).hexdigest(), 'stale '+key+' log'
    checks[key]={'status':'PASS','evidence':'build/firmware_integration/'+path,
      'sha256':hashlib.sha256((OUT/path).read_bytes()).hexdigest()}
for key,relative in [
    ('algorithms','build/validation/controller_navigation/summary.json'),
    ('abi','tests/abi/evidence/abi_compare.json'),
    ('drivers','build/driver_delivery/driver_checks.json')]:
    path=ROOT/relative;report=json.loads(path.read_text(encoding='utf-8'))
    if key=='algorithms':
        assert report['host_tests']==report['arm_compile_link']==report['m3_bridge_tests']=='PASS'
        for source_name,digest in json.loads((path.parent/'source_hashes.json').read_text(encoding='utf-8')).items():
            assert hashlib.sha256((ROOT/source_name).read_bytes()).hexdigest()==digest, 'stale algorithms: '+source_name
    elif key=='abi':
        assert report['status']=='PASS' and not report['failures'], report['failures']
        arm_abi=json.loads((path.parent/'abi_arm.json').read_text(encoding='utf-8'))
        for relative,digest in arm_abi['header_sha256'].items():
            assert (ROOT/relative).is_file() and hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest, 'stale ABI: '+relative
    else:
        assert report['firmware_sha256']=={name:value['sha256'] for name,value in artifacts.items()}
    checks[key]={'status':'PASS','evidence':relative,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
layout=ROOT/'build/three_layer_refactor/layout_checks.json'
assert json.loads(layout.read_text(encoding='utf-8'))['status']=='PASS'
checks['layout']={'status':'PASS','evidence':layout.relative_to(ROOT).as_posix(),
  'sha256':hashlib.sha256(layout.read_bytes()).hexdigest()}
for key,relative,expected in [
    ('replay','build/firmware_integration/replay/test_replay.log',r'test_replay: ([1-9][0-9]*) checks, 0 failures'),
    ('health','build/firmware_integration/health/test_health.log',r'test_health: ([1-9][0-9]*) checks, 0 failures'),
    ('fms','build/firmware_integration/final_m4.log',r'checks=([1-9][0-9]*) failures=0')]:
    path=ROOT/relative;assert re.search(expected,path.read_text(encoding='utf-8'))
    checks[key]={'status':'PASS','evidence':relative,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
assert source_hashes==capture_source_hashes(), 'source changed during firmware verification'
assert artifacts==capture_artifacts(), 'artifacts changed during firmware verification'
manifest={'date':datetime.now(timezone(timedelta(hours=8))).date().isoformat(),
 'timezone':'Asia/Shanghai','scope':'THREE_LAYER_FIRMWARE_INTEGRATION',
 'source_layers':['sensor_device','middleware','applications'],
 'profile':'QUAD_X4','schema':'6.0','port_revision':'0x00010906','output_policy':'BENCH_INHIBIT',
 'remote_mutation':'FLIGHT_OPERATIONS_DISABLED_BY_PROFILE','default_modules_enabled':True,'full_arm_build':'PASS',
 'toolchain':toolchain,'build_type':'release','hil_enabled':hil_enabled,'logger_enabled':logger_enabled,
 'nor_event_ring_enabled':nor_enabled,'local_tuning_enabled':tuning_enabled,'usb_cdc_enabled':usb_enabled,
 'required_linked_symbols':required,'strict_arm_sources':[str(p.relative_to(ROOT)) for p in sources],
 'resources':{'estimator_cores_dtcm_bytes':core_bytes,'axi_static_headroom_bytes':axi_headroom},
 'artifacts':artifacts,'size_output':size,'checks':checks,
 'remaining_evidence':{'target_boot_stack_wcet_rta':'NOT_RUN','physical_waveforms':'NOT_RUN',
 'hil':'NOT_RUN','flight':'NOT_RUN','independent_review':'NOT_RUN'},
 'source_sha256':source_hashes}
manifest_path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'full_arm_build':'PASS','strict_arm_sources':len(sources),'artifacts':artifacts},indent=2))
