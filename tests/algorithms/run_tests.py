"""Host execution + Cortex-M7 object/relocatable link checks. No board output."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
OUT=ROOT/'build/validation/controller_navigation'
OUT.mkdir(parents=True,exist_ok=True)
ap=argparse.ArgumentParser()
ap.add_argument('--compile-only',action='store_true')
ap.add_argument('--target',choices=['host','arm','all'],default='all')
args=ap.parse_args()
includes=['middleware/contracts',
          'middleware/model/fms','middleware/model/ins','middleware/model/control','middleware/model/control/inner_loop/quad_x4',
          'middleware/model/ins/kf_gins','middleware/model/ins/kf_gins/trimmed_core/include',
          'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated','middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/public',
          'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/private']
sources = []
for pattern in ('middleware/model/control/*.c', 'middleware/model/control/inner_loop/quad_x4/*/*.c',
                'middleware/model/control/outer_loop/quad_x4/*/*.c', 'middleware/model/ins/*.c',
                'middleware/model/ins/kf_gins/navigation_estimator/*.cpp', 'middleware/model/ins/kf_gins/observation_replay/*.cpp',
                'middleware/model/ins/kf_gins/trimmed_core/src/*.cpp',
                'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/src/*.c'):
    sources.extend(sorted(ROOT.glob(pattern)))
sources.extend(ROOT/path for path in (
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/quad_workspace.c',
    'middleware/model/ins/kf_gins/backend_adapter/kf_gins_adapter.cpp',
))
commands=[]
results={'host_tests':'NOT_RUN','arm_compile_link':'NOT_RUN','target_hardware':'NOT_RUN','flight':'NOT_RUN'}
lock=json.loads((ROOT/'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/UPSTREAM_LOCK.json').read_text())
for rel,digest in lock['files'].items():
    assert hashlib.sha256((ROOT/'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated'/rel).read_bytes()).hexdigest()==digest, rel
def run(cmd):
    p=subprocess.run(list(map(str,cmd)),cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    commands.append({'command':list(map(str,cmd)),'exit_code':p.returncode,'output':p.stdout})
    if p.stdout: print(p.stdout,end='')
    if p.returncode:
        (OUT/'commands.json').write_text(json.dumps(commands,indent=2))
        raise SystemExit(p.returncode)
    return p.stdout

for target,binpath in [('host','D:/STM32CubeCLT/mingw64/bin'),('arm','D:/STM32CubeCLT/GNU-tools-for-STM32/bin')]:
    if args.target!='all' and args.target!=target: continue
    objdir=OUT/target;objdir.mkdir(exist_ok=True)
    objects=[]
    for source in sources:
        obj=objdir/(str(source.relative_to(ROOT)).replace('\\','_').replace('/','_')+'.o')
        cpp=source.suffix=='.cpp'
        compiler=Path(binpath)/(('arm-none-eabi-' if target=='arm' else '')+('g++' if cpp else 'gcc')+'.exe')
        flags=['-O2','-g','-Wall','-Wextra','-Wshadow','-fno-short-enums','-ffunction-sections','-fdata-sections',
               '-std=c++14' if cpp else '-std=c99']
        if cpp:flags+=['-fno-exceptions','-fno-rtti','-fno-threadsafe-statics']
        if 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/' not in source.as_posix():flags+=['-Werror']
        if target=='arm':flags+=['-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-fstack-usage']
        run([compiler,*flags,*[f'-I{ROOT/i}' for i in includes],'-c',source,'-o',obj]);objects.append(obj)
    linker=Path(binpath)/(('arm-none-eabi-' if target=='arm' else '')+'g++.exe')
    if target=='host' and not args.compile_only:
        test=ROOT/'tests/algorithms/test_algorithms.cpp'
        exe=objdir/'test_algorithms.exe'
        run([linker,'-std=c++14','-O2','-static','-static-libgcc','-static-libstdc++',
             *[f'-I{ROOT/i}' for i in includes],test,*objects,'-o',exe])
        output=run([exe]);(OUT/'host_tests.log').write_text(output)
        results['host_tests']='PASS'
        bridge=ROOT/'tests/navigation/test_m3_bridge.cpp'
        bridge_exe=objdir/'test_m3_bridge.exe'
        bridge_objects=[p for p in objects if any(s in p.name for s in
                        ('ins_replay.cpp','ins_observation.cpp','kf_gins_trimmed_core_src_'))]
        run([linker,'-std=c++14','-O2','-static','-static-libgcc','-static-libstdc++',
             *[f'-I{ROOT/i}' for i in includes],bridge,*bridge_objects,'-o',bridge_exe])
        (OUT/'bridge_tests.log').write_text(run([bridge_exe]))
        results['m3_bridge_tests']='PASS'
    if target=='arm':
        run([linker,'-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-r',*objects,'-o',objdir/'algorithms.o'])
        run([Path(binpath)/'arm-none-eabi-size.exe',objdir/'algorithms.o'])
        symbols=run([Path(binpath)/'arm-none-eabi-nm.exe','-u',objdir/'algorithms.o'])
        forbidden=['malloc','calloc','realloc','_Zn','__cxa_throw','_Unwind','__atomic_']
        if any(x in symbols for x in forbidden):raise SystemExit('Unexpected heap, exception or atomic runtime dependency')
        objdump=run([Path(binpath)/'arm-none-eabi-objdump.exe','-h',objdir/'algorithms.o'])
        if '.init_array' in objdump:raise SystemExit('Unexpected global C++ initialization')
        mpc_objects=[obj for obj in objects if obj.name.endswith(('quad_workspace.c.o', 'mpc.c.o'))]
        assert len(mpc_objects)==2
        sizes=run([Path(binpath)/'arm-none-eabi-size.exe',*mpc_objects])
        ram=sum(int(line.split()[1])+int(line.split()[2]) for line in sizes.splitlines()[1:] if line.strip())
        if ram>32768:raise SystemExit(f'MPC static RAM exceeds 32 KiB: {ram}')
        results.update(arm_compile_link='PASS',mpc_static_ram_bytes=ram,mpc_static_ram_budget_bytes=32768)
(OUT/'commands.json').write_text(json.dumps(commands,indent=2))
inputs=set(sources)
for folder in ('middleware/contracts','middleware/model/fms',
               'middleware/model/control','middleware/model/ins','middleware/model/ins/kf_gins/trimmed_core/include',
               'config','middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc'):
    inputs.update(p for p in (ROOT/folder).rglob('*') if p.suffix in ('.h','.hpp','.inc','.yaml','.py'))
(OUT/'source_hashes.json').write_text(json.dumps({str(s.relative_to(ROOT)):hashlib.sha256(s.read_bytes()).hexdigest() for s in sorted(inputs)},indent=2))
(OUT/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
print('PASS: Host algorithms and Cortex-M7 compile/link checks')
