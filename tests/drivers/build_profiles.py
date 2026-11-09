"""Build the standard Flight product with its required UM982/PPS support.
Runtime loss is tested by fault injection, never by removing GNSS hardware.
"""
import json
import os
import re
from pathlib import Path
import shutil
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT=ROOT/'build/driver_delivery'
def build(name):
    env=os.environ.copy()
    env['RTT_CC']='gcc'
    env['RTT_EXEC_PATH']=str(Path(shutil.which('arm-none-eabi-gcc') or
      r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe').parent)
    result=subprocess.run(['uv','run','--with','scons','scons','-j','8'],env=env,cwd=ROOT,
                           capture_output=True,text=True)
    (OUT/(name+'_build.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode: raise RuntimeError(f'{name} build failed; see log')
    dest=OUT/name;dest.mkdir(exist_ok=True)
    for file in ['Flight.elf','Flight.bin','Flight.map']:
        shutil.copy2(ROOT/file,dest/file)
    return {'profile':name,'gnss_required':True,'pps_capture':True,'result':'FULL_ARM_BUILD_PASS'}

def main():
    OUT.mkdir(exist_ok=True,parents=True)
    text=(ROOT/'rtconfig.h').read_text(encoding='utf-8')
    config=(ROOT/'.config').read_text(encoding='utf-8')
    for symbol in ['BSP_USING_FLIGHT_DRIVERS','BSP_USING_UART4','BSP_UART4_RX_USING_DMA']:
        assert re.search(r'^#define\s+'+symbol+r'(?:\s|$)',text,re.M),symbol
        assert 'CONFIG_'+symbol+'=y' in config,symbol
    results=[build('flight_um982_standard')]
    (OUT/'build_profiles.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results,indent=2))
if __name__=='__main__':main()
