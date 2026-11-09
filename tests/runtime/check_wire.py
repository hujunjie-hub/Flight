"""Compare the firmware's C encoder against an independently generated SDK.

uv run --no-project --with pymavlink==2.4.49 python tests/runtime/check_wire.py
"""
import importlib.util
from importlib.metadata import version
import hashlib
import json
import math
from pathlib import Path
import re
import shutil

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
if not __debug__:
    raise SystemExit('Run the validation without Python -O.')
OUT=ROOT/'build/firmware_integration/wire'
OUT.mkdir(parents=True,exist_ok=True)
summary_path=OUT/'summary.json'
summary_path.write_text(json.dumps({'wire_comparison':'INCOMPLETE'})+'\n',encoding='utf-8')
sdk_version=version('pymavlink')
assert sdk_version=='2.4.49', f'Expected pymavlink 2.4.49, found {sdk_version}'
import pymavlink
from pymavlink.generator import mavgen

host=OUT.parent/'host'
runtime_summary=json.loads((host/'summary.json').read_text(encoding='utf-8'))
assert runtime_summary['runtime_integration']=='PASS', 'Host runtime validation has not passed'
for relative,digest in runtime_summary['source_hashes'].items():
    assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest, 'stale runtime source: '+relative
log_bytes=(host/'test_runtime.log').read_bytes()
log_hash=hashlib.sha256(log_bytes).hexdigest()
assert log_hash==runtime_summary['log_sha256'], 'Host runtime log changed after validation'
input_xml=(ROOT/'spec/contracts/FCU.xml').read_bytes()
(OUT/'FCU.xml').write_bytes(input_xml)
definitions=Path(pymavlink.__file__).parent/'dialects/v20'
for name in ['common.xml','standard.xml','minimal.xml']:
    shutil.copyfile(definitions/name,OUT/name)
assert mavgen.mavgen(mavgen.Opts(output=str(OUT/'fcu.py'),wire_protocol='2.0',language='Python3'),[str(OUT/'FCU.xml')])
spec=importlib.util.spec_from_file_location('firmware_test_fcu',OUT/'fcu.py')
sdk=importlib.util.module_from_spec(spec);spec.loader.exec_module(sdk)
cls=sdk.MAVLink_fcu_operation_status_message
fields={k:0 for k in cls.fieldnames}
fields.update(time_boot_us=0x0102030405060708,boot_id=0x1122334455667788,profile_id=0x10001,
              authority_epoch=9,directive_epoch=10,permit_epoch=11,flight_mode=4,authority=1)
mav=sdk.MAVLink(None,srcSystem=1,srcComponent=1);mav.seq=7
expected=cls(**fields).pack(mav)
log=log_bytes.decode('utf-8')
actual=bytes.fromhex(re.search(r'WIRE=([0-9a-f]+)',log).group(1))
assert actual==expected,(actual.hex(),expected.hex())
decoded=mav.decode(bytearray(actual))
assert decoded.boot_id==fields['boot_id'] and decoded.flight_mode==4
packets={}
for vector in re.findall(r'PACKET=([0-9a-f]+)',log):
    raw=bytes.fromhex(vector)
    message=mav.decode(bytearray(raw)) # generated SDK validates length/CRC_EXTRA/CRC
    assert message.get_srcSystem()==1 and message.get_srcComponent()==1
    packets[message.get_msgId()]=message
required = {0, 1, 24, 31, 74, 141, 245, *range(43000, 43013)}
assert required <= set(packets) <= required | {32}, set(packets)
assert packets[1].voltage_battery==65535 and packets[1].battery_remaining==-1
assert packets[0].base_mode & 1 and not packets[0].base_mode & 128
assert not packets[1].onboard_control_sensors_health & (1 << 28)
assert math.isnan(packets[74].airspeed)
assert math.isnan(packets[141].altitude_relative) and math.isnan(packets[141].altitude_amsl)
assert packets[141].altitude_terrain < -1000 and packets[141].bottom_clearance < 0
assert packets[24].fix_type==0 and packets[24].satellites_visible==255
assert packets[245].vtol_state==0 and packets[245].landed_state==0
assert abs(sum(value**2 for value in (packets[31].q1,packets[31].q2,packets[31].q3,packets[31].q4))-1)<1e-4
assert packets[43004].available_mask==0 and packets[43004].feedback_capability==0
assert packets[43004].output_policy_id==1 and packets[43004].reason_code==19
assert packets[43006].validity_mask==0 and packets[43007].validity_mask==0
assert packets[43010].check_state==2 and packets[43010].reason_code==19
assert packets[43011].valid_axes_mask==0 and packets[43011].receiver_failsafe==1
build=packets[43012]
assert build.schema_major==6 and build.product_revision==build.port_revision==0x00010906
assert build.digest_kind==3 and build.source_commit_valid==0 and not any(build.source_commit)
expected_abi=re.search(r'ABI_LOCK_ARM_HASH "([0-9a-f]+)"',
    (ROOT/'middleware/contracts/product_contract/abi_lock.h').read_text()).group(1)
assert bytes(build.sha256).hex()==expected_abi
summary_path.write_text(json.dumps({'wire_comparison':'PASS','dialect':'spec/contracts/FCU.xml',
    'generator':'pymavlink=='+sdk_version,'c_vector':actual.hex(),'sdk_vector':expected.hex(),
    'input_sha256':hashlib.sha256(input_xml).hexdigest(),
    'host_log_sha256':log_hash,
    'production_message_ids':sorted(packets),'production_semantics':'PASS'},indent=2)+'\n',encoding='utf-8')
print('PASS: C encoder matches SDK byte for byte; FCU and standard telemetry decoded and checked')
