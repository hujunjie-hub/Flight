#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""磁标定会话所需符号/结构体偏移提取 (pyelftools, 每次重编后跑一次).

输出 JSON 到 build_host/magcal_syms.json, 供 swd_magcal_session.py 消费.
注意 DW_AT_data_member_location 的值是 ListContainer, 取末元素
(首元素 0x23 是 DW_OP 操作码, 不是偏移).
"""
import json
import sys

from elftools.elf.elffile import ELFFile

ELF = r"..\cmake-build-debug\Flight.elf" if len(sys.argv) < 2 else sys.argv[1]

WANT_VARS = {
    # file 尾串 -> 变量名  (用 decl_file 尾串区分同名 static, 如 s_cal/s_par/ctx)
    "mag_calib.c": ["s_cal", "s_par", "g_magcal_swd_req"],
    "calib_store.c": ["s_store"],
    "mag_data.c": ["ctx"],
    "gins_bridge.cpp": ["g_nan", "s_engine", "g_run", "g_sol"],
    "gins_wdt.c": ["g_wdt_test_hold"],
}


def die_name(die):
    n = die.attributes.get("DW_AT_name")
    return n.value.decode() if n else None


def member_offset(die):
    loc = die.attributes.get("DW_AT_data_member_location")
    if loc is None:
        return 0
    v = loc.value
    if isinstance(v, list):
        return v[-1]          # ListContainer: 末元素才是偏移
    return int(v)


def main():
    out = {"vars": {}, "types": {}}

    with open(ELF, "rb") as f:
        elf = ELFFile(f)
        dwarf = elf.get_dwarf_info()

        # 1) 符号表拿地址 (static 符号也在)
        syms = {}
        for sec in elf.iter_sections():
            if sec.header["sh_type"] != "SHT_SYMTAB":
                continue
            for s in sec.iter_symbols():
                if s.name and s["st_value"]:
                    syms.setdefault(s.name, []).append(s["st_value"])

        # 2) DWARF: 按 CU 文件名分桶, 记录 static 变量的声明文件
        cu_files = {}
        for cu in dwarf.iter_CUs():
            top = cu.get_top_DIE()
            p = top.attributes.get("DW_AT_name")
            if p is None:
                continue
            fname = p.value.decode().replace("\\", "/").split("/")[-1]
            cu_files[fname] = cu

        for fname, names in WANT_VARS.items():
            cu = cu_files.get(fname)
            if cu is None:
                print(f"[warn] CU not found: {fname}")
                continue
            for die in cu.iter_DIEs():
                if die.tag != "DW_TAG_variable":
                    continue
                nm = die_name(die)
                if nm not in names:
                    continue
                addr = die.attributes.get("DW_AT_location")
                if addr is None or addr.form != "DW_FORM_exprloc":
                    continue
                # exprloc [DW_OP_addr imm] 手工解 (小端)
                expr = addr.value
                if not expr or expr[0] != 0x03:
                    continue
                a = int.from_bytes(expr[1:5], "little")
                if a == 0:
                    continue
                out["vars"][f"{fname}:{nm}"] = a
                print(f"var  {fname}:{nm} = 0x{a:08X}")

        # 3) 结构体成员偏移
        WANT_TYPES = {
            "calib_data": None, "ell_fit_acc": None, "mag_sample": None,
            "mag_data_status": None, "gins_solution": None,
        }
        seen = set()
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag != "DW_TAG_structure_type":
                    continue
                nm = die_name(die)
                if nm not in WANT_TYPES or nm in seen:
                    continue
                sz = die.attributes.get("DW_AT_byte_size")
                members = {"__size": sz.value if sz else -1}
                for ch in die.iter_children():
                    if ch.tag != "DW_TAG_member":
                        continue
                    members[die_name(ch)] = member_offset(ch)
                out["types"][nm] = members
                seen.add(nm)
                print(f"type {nm} size={members['__size']}: " +
                      ", ".join(f"{k}@0x{v:X}" for k, v in members.items()
                                if k != "__size"))

    # 4) g_sol = g_nan + 偏移 (gins_bridge.cpp 内同 CU 静态结构体数组, DWARF 变量名
    #    可能缺失, 由 nm 符号补) — 引擎/解快照的绝对地址单独算
    with open(r"magcal_syms.json", "w") as f:
        json.dump(out, f, indent=1)
    print("written magcal_syms.json")


if __name__ == "__main__":
    main()
