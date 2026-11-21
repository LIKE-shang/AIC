#!/usr/bin/env python3
"""Read-only ELF audit. Requires pyelftools/capstone, never loads or executes .so."""
import hashlib
from pathlib import Path
from elftools.elf.elffile import ELFFile
from capstone import Cs, CS_ARCH_ARM64, CS_MODE_ARM

root = Path(__file__).resolve().parents[2]
p = root/'uav_mission/lib/liblib.so'
print('SHA256:', hashlib.sha256(p.read_bytes()).hexdigest())
with p.open('rb') as f:
    e = ELFFile(f)
    print('Machine:', e['e_machine'])
    symbols = e.get_section_by_name('.symtab')
    for name in ['_Z15lib_pwm_controlii', 'lib_ctrl_pwm']:
        s = symbols.get_symbol_by_name(name)[0]
        sec = e.get_section(s['st_shndx'])
        off = s['st_value']-sec['sh_addr']
        data = sec.data()[off:off+s['st_size']]
        print(name, hex(s['st_value']), 'size', s['st_size'])
        if name.startswith('_Z'):
            for i in Cs(CS_ARCH_ARM64, CS_MODE_ARM).disasm(data, s['st_value']):
                print('0x%x: %s %s' % (i.address, i.mnemonic, i.op_str))
        else:
            print('initial bytes:', data.hex())
    print('NEEDED:', [t.needed for t in e.get_section_by_name('.dynamic').iter_tags()
                      if t.entry.d_tag == 'DT_NEEDED'])
