"""Snapshot sources and audit the standard firmware's required UM982 resources."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT=ROOT/'build/driver_delivery'
ARM=Path(os.environ.get('RTT_EXEC_PATH',r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin'))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def symbols(image):
    text=subprocess.check_output([str(ARM/'arm-none-eabi-nm.exe'),'-n',str(image)],text=True)
    return {c[2]:(int(c[0],16),c[1]) for line in text.splitlines() if len(c:=line.split())==3}
def main():
    profiles={}
    for name in ['flight_um982_standard']:
        image=OUT/name/'Flight.elf'; sym=symbols(image)
        uart=sym['UART4_IRQHandler'][1]
        assert uart=='T'
        assert 'g_uart4_rx' in sym
        assert sym['DMA1_Stream2_IRQHandler'][1]=='T'
        dma=sym['__device_dma_end'][0]-sym['__device_dma_start'][0]
        assert 0<dma<=12288
        profiles[name]={'uart4_handler':uart,'uart4_rx_dma_allocated':'g_uart4_rx' in sym,
                        'dma_bytes':dma,'elf_sha256':sha(image)}
    paths=[ROOT/p for p in ['.config','rtconfig.h','README.md','board/board.c','board/board.h',
        'board/SConscript','board/Kconfig','board/board_identity.c',
        'board/board_identity.h','sensor_device/time_base/timer_capture.c','sensor_device/time_base/timer_capture.h',
        'board/board_device.h','board/CubeMX_Config/Src/stm32h7xx_hal_msp.c',
        'board/CubeMX_Config/Inc/stm32h7xx_hal_conf.h','board/linker_scripts/link.lds',
        'board/linker_scripts/link.sct','board/linker_scripts/link.icf',
        'libraries/HAL_Drivers/drivers/drv_spi.c','libraries/HAL_Drivers/drivers/drv_spi.h',
        'libraries/HAL_Drivers/drivers/SConscript','libraries/HAL_Drivers/drivers/Kconfig',
        'libraries/HAL_Drivers/drivers/drv_usart.c','libraries/HAL_Drivers/drivers/drv_usart_v2.c',
        'libraries/HAL_Drivers/drivers/drv_usart_dma.c','libraries/HAL_Drivers/drivers/drv_usart_dma.h',
        'libraries/HAL_Drivers/drivers/drv_wdt.c','libraries/HAL_Drivers/drivers/drv_wdt.h',
        'libraries/HAL_Drivers/drivers/drv_crypto.c','libraries/HAL_Drivers/drivers/drv_crypto.h',
        'libraries/HAL_Drivers/drivers/drv_hard_i2c.c',
        'packages/stm32h7_hal_driver-latest/SConscript',
        'sensor_device/time_base/systime.c','sensor_device/time_base/systime.h',
        'middleware/SConscript','sensor_device/SConscript',
        'sensor_device/time_base/README.md']]
    for directory in ['sensor_device','board/diagnostics','middleware/actuator',
                      'tests/drivers']:
        paths.extend(p for p in (ROOT/directory).rglob('*') if p.is_file() and
                     (p.suffix.lower() in ['.c','.h','.md','.json','.py'] or p.name in ['LICENSE','SConscript']))
    manifest={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(set(paths))}
    (OUT/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (OUT/'profile_resource_checks.json').write_text(json.dumps(profiles,indent=2)+'\n',encoding='utf-8')
    sizes=subprocess.check_output([str(ARM/'arm-none-eabi-size.exe'),str(ROOT/'Flight.elf')],text=True)
    (OUT/'firmware_size.txt').write_text(sizes,encoding='utf-8')
    assert sha(ROOT/'Flight.elf')==profiles['flight_um982_standard']['elf_sha256']
    checks=json.loads((OUT/'driver_checks.json').read_text())
    assert sha(ROOT/'Flight.elf')==checks['firmware_sha256']['Flight.elf']
    print(json.dumps(profiles,indent=2)); print(sizes)
if __name__=='__main__':main()
