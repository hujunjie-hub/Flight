import re, sys

BS = chr(92)
BSLASH = '[' + '/' + BS + ']'

def agg(path):
    total = {}
    last = None
    pat = re.compile(r'^\s*(\S*)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+(.*)$')
    for ln in open(path, encoding='utf-8', errors='ignore'):
        ln = ln.rstrip()
        if not ln.strip():
            continue
        m = pat.match(ln)
        if not m:
            continue
        sec, sz, rest = m.group(1), int(m.group(2), 16), m.group(3).strip()
        if not (sec.startswith('.text') or sec.startswith('.rodata')):
            continue
        mm = re.search(r'(\S+\.(?:o|obj|a\([^)]*\)))\s*$', rest)
        obj = mm.group(1) if mm else last
        if not obj:
            continue
        last = obj
        name = obj.replace(BS, '/').split('/')[-1]
        name = re.sub(r'lib_a-', '', name)
        name = re.sub(r'\.(c|cpp|S|s)?\.(o|obj)$', '', name)
        total[name] = total.get(name, 0) + sz
    return total

a = agg(sys.argv[1])  # scons
b = agg(sys.argv[2])  # cmake

def group(n):
    if re.match(r'^(cp-demangle|guard|vterminate|atexit|del_op|new_op|eh_alloc|pure|term)', n):
        return 'libstdc++'
    if re.match(r'^(stm32h7xx_|startup_|system_stm32)', n):
        return 'HAL/CMSIS'
    if re.match(r'^(dtoa|mprec|strtod|svfwprintf|wmem|wcst|trim|locale|mprec|sdidtbl|ctype|fvwrite|vfiprintf|vfprintf|nano-vfprintf|mem-|cclass)', n):
        return 'libc-extra'
    if re.match(r'^(pow|sqrt|sin|cos|atan|exp|log|floor|ceil|fabs|fmod|hypot|remainder|erf|tanh|j0|j1|yn|lgam|cbrt|asinh|acosh|atanh|cosh|sincos|math)', n):
        return 'libm'
    if re.match(r'^(unwind|dvmd|bp|clz|aeabi|adddf|addsub|div|mul|fix|float|trunc|cmp|neg|extends|eq)', n):
        return 'libgcc'
    return 'other'

ga, gb = {}, {}
for k, v in a.items():
    ga[group(k)] = ga.get(group(k), 0) + v
for k, v in b.items():
    gb[group(k)] = gb.get(group(k), 0) + v
print('== group rollup (text+rodata) ==')
for g in sorted(set(ga) | set(gb)):
    print(f"{g:<12} scons={ga.get(g,0):7d}  cmake={gb.get(g,0):7d}  diff={ga.get(g,0)-gb.get(g,0):+7d}")
print()

delta = {}
for k in set(a) | set(b):
    d = a.get(k, 0) - b.get(k, 0)
    if abs(d) > 1500:
        delta[k] = d
print(f"{'scons-only KB':>12}  {'obj':<42} {'scons':>7} {'cmake':>7}")
for k, d in sorted(delta.items(), key=lambda kv: kv[1]):
    print(f"{d/1024:12.1f}  {k:<42} {a.get(k,0):7d} {b.get(k,0):7d}")
print("TOTAL text+rodata  scons:", sum(a.values()), " cmake:", sum(b.values()))
