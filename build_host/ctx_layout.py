#!/usr/bin/env python3
"""One-shot: dump member offsets of the anonymous ctx structs in
imu_data.c / mag_data.c from DWARF (record_ring migration changed layout)."""
from elftools.elf.elffile import ELFFile

ELF = '../cmake-build-debug/Flight.elf'

with open(ELF, 'rb') as f:
    elf = ELFFile(f)
    dwarf = elf.get_dwarf_info()
    for cu in dwarf.iter_CUs():
        for die in cu.iter_DIEs():
            if die.tag != 'DW_TAG_variable':
                continue
            name = die.attributes.get('DW_AT_name')
            if not name or name.value != b'ctx':
                continue
            ref = die.attributes.get('DW_AT_type')
            if ref is None:
                continue
            tdie = cu.get_DIE_from_refaddr(ref.value + cu.cu_offset)
            # resolve typedef chain
            while tdie.tag == 'DW_TAG_typedef':
                r2 = tdie.attributes.get('DW_AT_type')
                tdie = cu.get_DIE_from_refaddr(r2.value + cu.cu_offset)
            if tdie.tag != 'DW_TAG_structure_type':
                continue
            members = []
            for c in tdie.iter_children():
                if c.tag != 'DW_TAG_member':
                    continue
                mn = c.attributes.get('DW_AT_name')
                loc = c.attributes.get('DW_AT_data_member_location')
                off = loc.value if loc else 0
                if isinstance(off, list):
                    off = off[-1]
                members.append((mn.value.decode() if mn else '?', off))
            if any(m[0] in ('st', 'last') for m in members) and \
               any(m[0] in ('ring', 'rb') for m in members):
                comp = cu.get_top_DIE().attributes.get('DW_AT_name')
                print((comp.value.decode() if comp else '?'), members)
