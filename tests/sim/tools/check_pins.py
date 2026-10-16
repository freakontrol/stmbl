#!/usr/bin/env python3
"""Fail when a sim component's pins differ from the firmware component it stands in for.

The templates and board configs link pins by name, so a sim hv/ls/io must
carry exactly the firmware's pins. Run by the Makefile on every build.

usage: check_pins.py firmware.c sim.c
"""
import re
import sys


def pins(path):
    out = []
    with open(path) as f:
        for line in f:
            m = re.search(r'HAL_PINA?\((\w*)(,\s*\d*)?\)', line)
            if m:
                out.append(m.group(0).replace(' ', ''))
    return out


fw, sim = pins(sys.argv[1]), pins(sys.argv[2])
if fw != sim:
    missing = [p for p in fw if p not in sim]
    extra = [p for p in sim if p not in fw]
    print('%s: pins differ from %s' % (sys.argv[2], sys.argv[1]))
    if missing:
        print('  missing: ' + ' '.join(missing))
    if extra:
        print('  extra:   ' + ' '.join(extra))
    if not missing and not extra:
        print('  same pins, different order')
    sys.exit(1)
