import os
import sys
import rtconfig

RTT_ROOT = os.getenv('RTT_ROOT') or 'rt-thread'

sys.path = sys.path + [os.path.join(RTT_ROOT, 'tools')]
try:
    from building import *
except ImportError:
    print('Cannot load RT-Thread build tools. Check RTT_ROOT.')
    print(RTT_ROOT)
    exit(-1)

def bsp_pkg_check():
    check_paths = [
        os.path.join("packages", "CMSIS-Core-latest"),
        os.path.join("packages", "stm32h7_cmsis_driver-latest"),
        os.path.join("packages", "stm32h7_hal_driver-latest")
    ]

    need_update = not all(os.path.exists(p) for p in check_paths)

    if need_update:
        print("\n===============================================================================")
        print("Dependency packages missing, please running 'pkgs --update'...")
        print("If no packages are fetched, run 'pkgs --upgrade' first, then 'pkgs --update'...")
        print("===============================================================================")
        exit(1)

RegisterPreBuildingAction(bsp_pkg_check)

TARGET = 'Flight.' + rtconfig.TARGET_EXT

DefaultEnvironment(tools=[])
env = Environment(tools = ['mingw'],
    AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
    CC = rtconfig.CC, CFLAGS = rtconfig.CFLAGS,
    AR = rtconfig.AR, ARFLAGS = '-rc',
    CXX = rtconfig.CXX, CXXFLAGS = rtconfig.CXXFLAGS,
    LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

if rtconfig.PLATFORM in ['iccarm']:
    env.Replace(CCCOM = ['$CC $CFLAGS $CPPFLAGS $_CPPDEFFLAGS $_CPPINCFLAGS -o $TARGET $SOURCES'])
    env.Replace(ARFLAGS = [''])
    env.Replace(LINKCOM = env["LINKCOM"] + ' --map Flight.map')

Export('env')
Export('RTT_ROOT')
Export('rtconfig')

SDK_ROOT = os.path.abspath('./')

if os.path.exists(SDK_ROOT + '/libraries'):
    libraries_path_prefix = SDK_ROOT + '/libraries'
else:
    libraries_path_prefix = os.path.dirname(SDK_ROOT) + '/libraries'

# prepare building environment
objs = PrepareBuilding(env, RTT_ROOT, has_libcpu=False)

rtconfig.BSP_LIBRARY_TYPE = None

# include drivers
objs.extend(SConscript(os.path.join(libraries_path_prefix, 'HAL_Drivers', 'SConscript'),variant_dir='build/libraries/HAL_Drivers', duplicate=0))

# Keep the RTOS, Flight task owners, models and HIL stream at the default -O2.
if rtconfig.PLATFORM == 'gcc' and rtconfig.BUILD == 'release':
    size_optimized_groups = {
        'STM32H7-HAL', 'Finsh', 'UTest', 'Utilities', 'FlightBoardServices',
        'NorFlashStorage', 'SdmmcStorage', 'MavlinkProtocol', 'GnssProtocols',
    }
    for group in Projects:
        if group['name'] in size_optimized_groups:
            group['LOCAL_CFLAGS'] = group.get('LOCAL_CFLAGS', '') + ' -Os'

# The link step emits three artifacts: the ELF image plus the linker map and
# the raw binary produced by the post action. Register all of them on the
# program node so they are rebuilt when missing and dropped by 'scons -c',
# keeping every output under the single 'Flight.*' name.
def flight_outputs(target, source, env):
    return target + [env.File('Flight.bin'), env.File('Flight.map')], source

env.Append(PROGEMITTER=[flight_outputs])
if rtconfig.PLATFORM == 'gcc':
    env.Depends(TARGET, 'board/linker_scripts/link.lds')

# make a building
DoBuilding(TARGET, objs)
