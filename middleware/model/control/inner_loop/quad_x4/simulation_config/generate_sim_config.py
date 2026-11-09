"""Compile the SIM_ONLY controller fixture from authoritative JSON-subset YAML.
No runtime tuning or Flash parameter loading is introduced.
"""
from pathlib import Path
import hashlib
import json
import math

MODEL = Path(__file__).resolve().parent
ROOT = next(p for p in MODEL.parents if (p/'SConstruct').is_file())
config=ROOT/'config'
paths=[config/'vehicle_model.yaml',config/'controller_tuning.yaml']
vehicle,tuning=[json.loads(p.read_text()) for p in paths]
assert vehicle['status']==tuning['status']=='SIM_ONLY'
if 'inner_loop' in tuning or 'outer_loop' in tuning:
    inner = tuning['inner_loop']
    outer = tuning['outer_loop']
    assert not inner.keys() & outer.keys(), 'duplicate tuning key across loops'
    loop_tuning = {**inner, **outer}
else:
    loop_tuning = {k:v for k,v in tuning.items() if k not in ('schema_version','profile','status')}
assert loop_tuning.pop('mpc_structure')==dict(sample_time_s=.02,prediction_horizon=20,control_horizon=5)
fields={k:vehicle[k] for k in ('mass_kg','inertia_frd_kgm2','motor_position_frd_m','motor_yaw_torque_ratio_m')}
fields.update(loop_tuning)
def literal(value):
    if isinstance(value,list):return '{'+','.join(map(literal,value))+'}'
    assert isinstance(value,(int,float)) and math.isfinite(value)
    s=format(float(value),'.9g')
    return s+('f' if '.' in s or 'e' in s else '.0f')
digest=hashlib.sha256(b''.join(p.read_bytes() for p in paths)).hexdigest()
text='/* Generated from SIM_ONLY vehicle_model.yaml + controller_tuning.yaml. */\n'
text+='#include "control_backend.h"\n'
text+='static const ProductControllerConfig sim_config = {\n'
text+=' .profile_hash=PRODUCT_PROFILE_ID,.parameter_set_id=1,.parameter_hash=0x'+digest[:8]+'u,.config_version=1,\n .payload={\n'
text+='\n'.join('  .'+k+'='+literal(v)+',' for k,v in fields.items())+'\n }};\n'
text += '''
void control_backend_sim_config(ProductControllerConfig *out)
{
    if (out)
    {
        *out = sim_config;
    }
}
'''
(MODEL/'quad_x4_controller_sim_config.c').write_text(text, encoding='utf-8')
print('SIM_ONLY parameter hash:',digest)
