#!/usr/bin/env python3
# m33mu -- an ARMv8-M Emulator
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Test aid for nvm284*.txt, not a firmware change: m33mu programs and erases
flash instantly, so an NVM write takes far less time than on the chip.  This
adds a busy wait of <us> microseconds (default 3000) at the end of the
flash_safe_execute() callback in targets/f1/furi_hal/furi_hal_nvm_storage.c,
inside the region where pico-sdk's default safe zone has interrupts disabled.
Edits the firmware tree in place; rebuild after it.

usage: nvm_flash_delay.py <firmware tree> [us]
"""
import sys

if len(sys.argv) < 2:
    sys.exit(__doc__)
path = sys.argv[1] + "/targets/f1/furi_hal/furi_hal_nvm_storage.c"
us = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
src = open(path).read()
old = "        flash_range_program(args->addr, (const uint8_t*)args->buffer, args->size);\n    }\n}\n"
if "test aid: flash time" in src:
    sys.exit(f"{path}: already has the delay")
if src.count(old) != 1:
    sys.exit(f"{path}: the flash_safe_execute callback is not as expected")
src = src.replace(old, old[:-2] + f"    busy_wait_us_32({us}); // test aid: flash time, interrupts disabled\n}}\n")
if "#include <hardware/timer.h>" not in src:
    src = "#include <hardware/timer.h>\n" + src
open(path, "w").write(src)
