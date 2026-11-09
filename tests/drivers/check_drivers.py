"""Host golden wire tests, strict ARM compilation, and linked resource audit.
No target execution, physical timing or flight authorization is implied.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT = ROOT / 'build/driver_delivery'
ARM = Path(os.environ.get('RTT_EXEC_PATH', r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin'))
HOST = Path(r'D:\STM32CubeCLT\mingw64\bin\gcc.exe')

def run(args, log):
    result = subprocess.run([str(a) for a in args], cwd=ROOT, text=True, capture_output=True)
    (OUT/log).write_text(result.stdout + result.stderr,encoding='utf-8')
    if result.returncode: raise RuntimeError(f'{log} failed: {result.stdout[-1000:]} {result.stderr[-1000:]}')
    return result.stdout

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    exe=OUT/'test_driver_codec.exe'
    run([HOST,'-std=c11','-Wall','-Wextra','-Werror','-Isensor_device/actuator',
         'tests/drivers/test_driver_codec.c','-o',exe],'codec_build.log')
    host=run([exe],'codec_test.log').strip()
    hooks_exe=OUT/'test_time_hooks.exe'
    run([HOST,'-std=c11','-Wall','-Wextra','-Werror',
         '-Imiddleware/contracts','-Isensor_device/time_base',
         'sensor_device/time_base/systime.c','tests/drivers/test_time_hooks.c',
         '-o',hooks_exe],'time_hooks_build.log')
    hooks=run([hooks_exe],'time_hooks_test.log').strip()
    outage_exe=OUT/'test_gnss_outage.exe'
    run([HOST,'-std=c11','-Wall','-Wextra','-Werror',
         '-Imiddleware/contracts','-Isensor_device/time_base',
         '-Isensor_device','-Imiddleware/sensor',
         'sensor_device/time_base/systime.c',
         'middleware/sensor/calibration/sensor_calibration.c','middleware/sensor/measurement_filter/sensor_filter.c',
         'middleware/sensor/sample_normalization/sensor_sample.c','tests/drivers/test_gnss_outage.c',
         '-lm','-o',outage_exe],'gnss_outage_build.log')
    outage=run([outage_exe],'gnss_outage_test.log').strip()
    includes=['.','board','sensor_device/actuator','sensor_device/time_base',
      'sensor_device/imu','sensor_device/mag','sensor_device/gnss',
      'sensor_device/barometer','sensor_device/pmu','middleware/actuator',
      'board/CubeMX_Config/Inc','libraries/HAL_Drivers',
      'libraries/HAL_Drivers/drivers','libraries/HAL_Drivers/drivers/config',
      'packages/stm32h7_hal_driver-latest/Inc','packages/stm32h7_cmsis_driver-latest/Include',
      'packages/CMSIS-Core-latest/Include',
      'rt-thread/include','rt-thread/components/drivers/include',
      'rt-thread/components/drivers/hwcrypto','rt-thread/components/finsh',
      'rt-thread/components/utilities/ulog',
      'rt-thread/libcpu/arm/cortex-m7','middleware/contracts']
    flags=['-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-std=c99',
           '-Wall','-Wextra','-Werror','-fno-short-enums','-DSTM32H723xx','-DUSE_HAL_DRIVER','-Dgcc']
    sources = [ROOT/path for path in ('board/board.c', 'board/board_identity.c',
                                     'libraries/HAL_Drivers/drivers/drv_usart_dma.c',
                                     'libraries/HAL_Drivers/drivers/drv_usart_v2.c')]
    config = (ROOT/'rtconfig.h').read_text(encoding='utf-8')
    usb_enabled = '#define BSP_USING_USBD\n' in config
    # The relocated BSP SDMMC driver is covered by the linked firmware audit below.
    sdmmc = ROOT/'sensor_device/storage/SDMMC/drv_sdmmc.c'
    for directory in ('sensor_device', 'board/diagnostics'):
        sources.extend(sorted(path for path in (ROOT/directory).rglob('*.c') if path != sdmmc))
    for source in sources:
        # FinSH stores commands through its generic syscall function pointer.
        export_flags = ['-Wno-cast-function-type'] if source.name == 'device_diagnostics.c' else []
        run([ARM/'arm-none-eabi-gcc.exe']+flags+export_flags+['-I'+p for p in includes]+
            ['-fstack-usage','-c',source,'-o',OUT/(source.stem+'_strict.o')],source.stem+'_strict.log')
    nm=run([ARM/'arm-none-eabi-nm.exe','-n','Flight.elf'],'linked_symbols.log')
    symbols={}
    for line in nm.splitlines():
        cols=line.split()
        if len(cols)==3: symbols[cols[2]]=(int(cols[0],16),cols[1])
    handlers=['TIM2_IRQHandler','TIM1_UP_IRQHandler','DMA2_Stream2_IRQHandler',
      'SPI1_IRQHandler','DMA1_Stream0_IRQHandler','DMA1_Stream1_IRQHandler',
      'USART1_IRQHandler','USART2_IRQHandler','USART3_IRQHandler','UART4_IRQHandler','DMA1_Stream2_IRQHandler',
      'DMA1_Stream5_IRQHandler','DMA1_Stream6_IRQHandler',
      'DMA1_Stream7_IRQHandler','I2C1_EV_IRQHandler','I2C4_EV_IRQHandler',
      'I2C2_EV_IRQHandler','DMA2_Stream0_IRQHandler','DMA2_Stream1_IRQHandler',
      'SDMMC1_IRQHandler','OCTOSPI1_IRQHandler']
    if usb_enabled:
        handlers.append('OTG_HS_IRQHandler')
        assert symbols['board_usb_prepare'][1] == 'T'
        if '#define RT_USB_DEVICE_CDC\n' in config:
            assert symbols['vcom_get_stored_serno'][1] == 'T'
            if '#define RT_USING_FINSH\n' in config:
                assert 'flight_console' in symbols
        ioc = dict(line.split('=', 1) for line in
                   (ROOT/'board/CubeMX_Config/Flight.ioc').read_text(encoding='utf-8').splitlines()
                   if '=' in line and not line.startswith('#'))
        assert ioc['PA11.Signal'] == 'USB_OTG_HS_DM' and ioc['PA11.GPIO_Label'] == 'USB0_N'
        assert ioc['PA12.Signal'] == 'USB_OTG_HS_DP' and ioc['PA12.GPIO_Label'] == 'USB0_P'
        assert ioc['PA9.Signal'] == 'USART1_TX' and ioc['PA10.Signal'] == 'USART1_RX'
        assert ioc['USB_OTG_HS.VirtualMode'] == 'Device_Only_FS'
        assert ioc['USB_OTG_HS.vbus_sensing_enable'] == 'DISABLE'
        assert ioc['RCC.USBCLockSelection'] == 'RCC_USBCLKSOURCE_PLL3'
        pll = [int(ioc['RCC.DIV'+factor+'3']) for factor in ('M', 'N', 'Q')]
        assert pll == [5, 96, 10] and int(ioc['RCC.USBFreq_Value']) == 48000000
        assert ioc['NVIC.OTG_HS_IRQn'].startswith(r'true\:12\:0')
    assert all(symbols[h][1]=='T' for h in handlers), 'weak/unbound driver IRQ'
    dma_begin=symbols['__device_dma_start'][0]; dma_end=symbols['__device_dma_end'][0]
    assert dma_begin==0x30000000 and dma_end-dma_begin<=12*1024
    assert symbols['g_imu_tx'][0]%32==0 and symbols['g_imu_rx'][0]%32==0
    assert symbols['g_dshot_buf'][0]%32==0
    for buffer in ['g_uart2_rx','g_uart2_tx','g_uart3_rx','g_uart4_rx']:
        address=symbols[buffer][0]
        assert dma_begin<=address<dma_end and address%32==0, buffer
    assert 'huart4' not in symbols # no parallel CubeMX UART handle
    linked_objects={line.split()[-1].replace('\\','/').rsplit('/',1)[-1]
                    for line in (ROOT/'Flight.map').read_text(encoding='utf-8').splitlines()
                    if line.startswith('LOAD ')}
    assert {'dev_serial_v2.o','drv_usart_v2.o'} <= linked_objects
    assert not {'dev_serial.o','drv_usart.o'} & linked_objects
    if usb_enabled:
        assert 'drv_usbd.o' in linked_objects
    result={'scope':'HOST_GOLDEN + STRICT_ARM_COMPILE + FULL_FIRMWARE_LINK_RESOURCE_AUDIT',
      'host':host,'time_hooks':hooks,'gnss_runtime_loss_recovery':outage,
      'gnss_required':True,'gnss_pps_capture_enabled':True,
      'strict_arm_sources':[str(s.relative_to(ROOT)).replace('\\','/') for s in sources],
      'strong_irq_handlers':handlers,'dma_arena_bytes':dma_end-dma_begin,
      'target_waveform_cache_wcet_hil':'NOT_RUN',
      'firmware_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['Flight.elf','Flight.bin','Flight.map']}}
    (OUT/'driver_checks.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))
if __name__=='__main__': main()
