import re
p = "build_host/swd_monitor.py"
src = open(p, encoding="utf-8").read()
lines = src.splitlines()
for i, l in enumerate(lines):
    if l.startswith("CLI = "):
        lines[i] = 'CLI = r"D:\\STM32CubeCLT\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe"'
        break
open(p, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
print([l for l in lines if l.startswith("CLI")][0])
