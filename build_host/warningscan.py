import json, subprocess, re, os, tempfile, sys

BS = chr(92)
cc = json.load(open('build-check/compile_commands.json', encoding='utf-8'))

def norm(p):
    return p.replace(BS, '/')

proj = [e for e in cc
        if re.search(r'/(applications|middleware|board|libraries)/', norm(e['file']))
        and not re.search(r'/dist/|KF-GINS/ThirdParty|rt-thread/', norm(e['file']))]
print(f"scan {len(proj)} project TUs", file=sys.stderr)

warns = {}
for e in proj:
    cmd = e['command']
    cmd = cmd.replace('-O2', '-O2 -Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Wnull-dereference')
    out = tempfile.mktemp(suffix='.o').replace(BS, '/')
    cmd = re.sub(r'-o\s+(\S+)', lambda m: '-o ' + out, cmd)
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True, cwd=e['directory'])
    if os.path.exists(out):
        os.remove(out)
    for line in (r.stderr or '').splitlines():
        m = re.match(r'(.+?:\d+(?::\d+)?):\s*warning:', line)
        if m:
            key = norm(m.group(1))
            warns.setdefault(key, []).append(line.strip())

for f, ws in sorted(warns.items(), key=lambda kv: -len(kv[1])):
    print(f"### {f}  ({len(ws)})")
    for w in ws[:3]:
        print("   ", w[:220])
