"""Collect the Flight source layers and BSP/package build inputs."""
import os
Import('env')
from building import *

objs = []

# Relative child scripts inherit the BSP build variant from PrepareBuilding.
for directory in ('board', 'sensor_device', 'middleware', 'applications'):
    objs.extend(SConscript(os.path.join(directory, 'SConscript')))
if GetDepend('FLIGHT_TARGET_UTEST'):
    objs.extend(SConscript('tests/target/SConscript'))
objs.extend(SConscript(os.path.join(GetCurrentDir(), 'packages', 'SConscript')))

Return('objs')
