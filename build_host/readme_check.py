# -*- coding: utf-8 -*-
"""
README 反引号路径引用存在性检查 (2026-10-03 随 README 全量审计引入)

扫描 12 个项目自有 README (排除上游), 提取行内反引号里形似文件/目录的
token, 按三级上下文解析: (1) 相对 README 所在目录 (2) 相对仓库根
(3) 相对常见根目录 (middleware/ build_host/ applications/ drivers/ doc/
build_host/data/) 的短名/异目录速记。均不存在才报 broken。

已知可接受的散文速记 (不算 broken, 语境自明): 裸文件名 (baro_calib.c)、
build_host/ 前缀句中的脚本名、代码围栏内容 (已剔除)。
"""
import os
import re

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
READMES = [
    "README.MD", "build_host/README.md", "middleware/README.MD",
    "middleware/Sensor_Preprocessing/filter_calib/README.md",
    "middleware/Control/attitude_so3/README.md",
    "middleware/Control/position_mpc/README.md",
    "middleware/Navigation/gins/README.md", "middleware/Sensor_Preprocessing/param_calib/README.md",
    "middleware/Protocol/README.md", "middleware/Protocol/mavlink/README.md",
    "middleware/Protocol/nmea/README.md", "middleware/Control/so3/README.md",
]
EXT = (".c", ".h", ".cpp", ".hpp", ".py", ".md", ".bat", ".ps1", ".exe",
       ".gdb", ".ocd", ".json", ".xml", ".ioc", ".lds", ".icf", ".sct",
       ".png", ".pdf", ".xlsx", ".yaml", ".cfg", ".csv", ".bin", ".txt")
SEARCH = [ROOT, os.path.join(ROOT, "middleware"), os.path.join(ROOT, "build_host"),
          os.path.join(ROOT, "libraries", "HAL_Drivers", "drivers"),
          os.path.join(ROOT, "applications"), os.path.join(ROOT, "build_host", "data"),
          os.path.join(ROOT, "doc"),
          # 重组后的二级目录组: 裸文件名速记只扫 SEARCH 根一层子目录,
          # 故凡再嵌一层的组目录须在此登记 (Sensor_Drivers、
          # Navigation/gins、Sensor_Preprocessing 各组目录)
          os.path.join(ROOT, "middleware", "Sensor_Preprocessing"),
          os.path.join(ROOT, "middleware", "Sensor_Drivers"),
          os.path.join(ROOT, "middleware", "Navigation")]


def main():
    bad = []
    for r in READMES:
        path = os.path.join(ROOT, r)
        if not os.path.exists(path):
            bad.append((r, "<README 本身缺失>"))
            continue
        base = os.path.dirname(path)
        text = open(path, encoding="utf-8").read()
        text = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
        for tok in re.findall(r"`([^`\n]+)`", text):
            if "*" in tok or tok.startswith(("http", "#", "-", "msh>", "..")):
                continue
            last = tok.rstrip("/").rsplit("/", 1)[-1]
            if "." not in last or not tok.endswith(EXT):
                continue
            found = (os.path.exists(os.path.normpath(os.path.join(base, tok)))
                     or os.path.exists(os.path.normpath(os.path.join(ROOT, tok)))
                     or any(os.path.exists(os.path.join(s, tok)) for s in SEARCH))
            if not found and "/" not in tok:      # 裸文件名全局速记
                found = any(os.path.exists(os.path.join(sub, tok))
                            for s in SEARCH for sub in ([s] + [
                                os.path.join(s, d) for d in os.listdir(s)
                                if os.path.isdir(os.path.join(s, d))]))
            if not found:
                bad.append((r, tok))

    if bad:
        print(f"broken refs: {len(bad)}")
        for r, t in bad:
            print(f"  {r}: `{t}`")
        raise SystemExit(1)
    print(f"all path refs in {len(READMES)} READMEs exist")


if __name__ == "__main__":
    main()
