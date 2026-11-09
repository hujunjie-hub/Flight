"""Check fixed UM982 configuration, reject missing required UART/DMA, and
prevent the old hardware-optional requirement from returning to active specs.
Run with: uv run --with kconfiglib python tests/drivers/check_required_gnss.py
"""
import json
from pathlib import Path
import re
import subprocess
import kconfiglib

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT=ROOT/'build/gnss_required_correction'
CC=Path(r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe')
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    config=(ROOT/'.config').read_text(encoding='utf-8')
    header=(ROOT/'rtconfig.h').read_text(encoding='utf-8')
    required=['BSP_USING_FLIGHT_DRIVERS','BSP_USING_UART4','BSP_UART4_RX_USING_DMA']
    for symbol in required:
        assert 'CONFIG_'+symbol+'=y' in config,symbol
        assert re.search(r'^#define\s+'+symbol+r'(?:\s|$)',header,re.M),symbol
    k=kconfiglib.Kconfig(str(ROOT/'Kconfig'),warn=False)
    k.load_config(str(ROOT/'.config'))
    assert all(k.syms[s].str_value=='y' for s in required)
    modules=['FLIGHT_RUNTIME_ENABLE','INS_BACKEND_ENABLE','SENSOR_PROCESSING_ENABLE','INS_REPLAY_ENABLE',
             'SENSOR_INTEGRITY_ENABLE','FMS_ENABLE','GUIDANCE_ENABLE','CONTROLLER_ENABLE']
    for symbol in modules:
        assert symbol in k.syms and k.syms[symbol].str_value=='y',symbol
        assert len(k.syms[symbol].nodes)==1,'duplicate Kconfig definition: '+symbol
    for s in required[1:]:
        k.syms[s].set_value(0)
        assert k.syms[s].str_value=='y',f'{s} was incorrectly optional'
    ktext=(ROOT/'board/Kconfig').read_text(encoding='utf-8')
    assert 'config BSP_FLIGHT_GNSS' not in ktext
    cases=[]
    for name,macros,expected in [
        ('required_transport',required,True),
        ('missing_uart4',[required[0],required[2]],False),
        ('missing_uart4_dma',required[:2],False),
    ]:
        source='#include <stdint.h>\ntypedef struct USART_TypeDef USART_TypeDef;\n'
        source+=''.join('#define '+s+'\n' for s in macros)
        source+='#include "board_device.h"\n'
        p=subprocess.run([str(CC),'-std=c99','-fsyntax-only','-Iboard','-x','c','-'],
                         input=source,cwd=ROOT,text=True,capture_output=True)
        (OUT/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
        assert (p.returncode==0)==expected,name
        if not expected: assert 'Flight requires UM982' in p.stderr
        cases.append({'case':name,'compiler_exit':p.returncode,'expected_success':expected})
    for path in [ROOT/'sensor_device/time_base/timer_capture.c',ROOT/'libraries/HAL_Drivers/drivers/drv_usart_dma.c',
                 ROOT/'sensor_device/time_base/systime_rtthread.c',ROOT/'board/CubeMX_Config/Src/stm32h7xx_hal_msp.c']:
        assert 'BSP_FLIGHT_GNSS' not in path.read_text(encoding='utf-8'),path
    platform=(ROOT/'sensor_device/time_base/timer_capture.c').read_text(encoding='utf-8')
    assert 'TIM_DIER_CC3IE' in platform and 'TIM_CHANNEL_3' in platform
    assert 'cfg.gnss_pps_installed = 1u;' in (ROOT/'sensor_device/time_base/systime_rtthread.c').read_text(encoding='utf-8')
    docs=list((ROOT/'spec').glob('*.md'))
    forbidden=[
        'GNSS是可选导航观测源和可选外部UTC/频率参考',
        'GNSS可选；不与RC/GCS复用',
        'GNSS、PPS、RTC和外部同步脉冲均为Product Manifest可选能力',
        'GNSS_SUPPORT=0独立构建/启动',
        '固定`GNSS_BACKEND=DISABLED`（GNSS_SUPPORT=0）',
        'GNSS未装/无PPS时禁用CH3',
        '本默认配置GNSS_PPS_CAPTURE=0',
    ]
    for path in docs:
        text=path.read_text(encoding='utf-8')
        assert not any(s in text for s in forbidden),path
    for filename in ['00_工程基线与唯一权威规范.md','05_传感器采集时间基准与GNSS独立运行.md',
                     '07_实时调度与板级硬件资源设计.md','08_产品配置插件与参数管理.md']:
        text=(ROOT/'spec'/filename).read_text(encoding='utf-8')
        assert 'GNSS_SUPPORT=1' in text and 'GNSS_PPS_CAPTURE=1' in text,filename
    report={'fixed_hardware':'UM982 required; UART4 RX DMA + TIM2 CH3 always initialized',
            'kconfig_direct_disable_attempts':'forced back to enabled',
            'three_layer_modules_enabled_once':modules,
            'compile_guard_cases':cases,'active_spec_documents_checked':len(docs),
            'old_hardware_optional_clauses':'absent','target_execution':'NOT_RUN'}
    (OUT/'required_gnss_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
