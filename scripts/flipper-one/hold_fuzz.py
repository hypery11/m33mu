#!/usr/bin/env python3
# m33mu -- an ARMv8-M Emulator
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Random I2C1 master scripts for the Flipper One MCU I2C slave
(mcu-firmware#263): register writes and reads on the intercom (LED colour
registers 0x0310..0x0317 and the version register 0x0080), with the I2C1 IRQ
held off at random points (irq_hold_us).  Each read carries the bytes a
correct slave returns, as '# expect: ..', from a model of the registers that
applies the transactions in bus order.

usage: hold_fuzz.py gen <seed> <transactions> [x] [nohold] > script.txt
           x       also write + repeated START + read in one transaction
           nohold  the same script without its irq_hold_us lines
       hold_fuzz.py check <script.txt> <master log>
           prints the reads that differ, exit 1 if any
"""
import random
import re
import sys


def gen(seed, n, combined=False, holds=True):
    rnd = random.Random(seed)
    regs = bytearray(8)  # 0x0310..0x0317, all 0 after boot
    mode = (" combined" if combined else "") + ("" if holds else " no-holds")
    out = [
        f"# hold_fuzz seed={seed} transactions={n}{mode}",
        "wait_log enabling I2C slave mode",
        "delay_ms 50",
    ]

    def expect(addr, count):
        if addr == 0x0080:
            return "01 00"[: 3 * count - 1]
        return " ".join(f"{regs[addr - 0x0310 + i]:02x}" for i in range(count))

    for _ in range(n):
        r = rnd.random()
        if r < 0.30:
            hold = rnd.randint(100, 3000)
            if holds:
                out.append(f"irq_hold_us {hold}")
        elif r < 0.40:
            out.append(f"delay_us {rnd.randint(0, 300)}")
        kind = rnd.choice(["w", "w", "w", "r", "v", "p"] + (["x"] if combined else []))
        if kind == "w":
            words = rnd.randint(1, 4)
            start = 0x0310 + 2 * rnd.randint(0, 4 - words)
            data = [rnd.randint(0, 255) for _ in range(2 * words)]
            for i, b in enumerate(data):
                regs[start - 0x0310 + i] = b
            out.append(f"write 69 {start >> 8:02x} {start & 0xff:02x} " + " ".join(f"{b:02x}" for b in data))
        elif kind == "r":
            words = rnd.randint(1, 4)
            start = 0x0310 + 2 * rnd.randint(0, 4 - words)
            out.append(f"write_read 69 {start >> 8:02x} {start & 0xff:02x} : {2 * words}   # expect: {expect(start, 2 * words)}")
        elif kind == "v":
            out.append(f"write_read 69 00 80 : 2   # expect: {expect(0x0080, 2)}")
        elif kind == "x":
            # write, then a repeated START and a read of what follows, in one transaction
            words = rnd.randint(1, 3)
            start = 0x0310 + 2 * rnd.randint(0, 3 - words)
            data = [rnd.randint(0, 255) for _ in range(2 * words)]
            for i, b in enumerate(data):
                regs[start - 0x0310 + i] = b
            nxt = start + 2 * words
            rd = 2 * rnd.randint(1, (0x0318 - nxt) // 2)
            out.append(f"write_read 69 {start >> 8:02x} {start & 0xff:02x} " + " ".join(f"{b:02x}" for b in data) + f" : {rd}   # expect: {expect(nxt, rd)}")
        else:
            words = rnd.randint(1, 4)
            start = 0x0310 + 2 * rnd.randint(0, 4 - words)
            out.append(f"write 69 {start >> 8:02x} {start & 0xff:02x}")
            out.append(f"read 69 {2 * words}   # expect: {expect(start, 2 * words)}")
    out.append("delay_ms 10")
    out.append(f"write_read 69 03 10 : 8   # expect: {expect(0x0310, 8)}")
    out.append(f"write_read 69 00 80 : 2   # expect: {expect(0x0080, 2)}")
    print("\n".join(out))


def check(script, log):
    want = {}
    holds = 0
    with open(script) as f:
        for i, line in enumerate(f, 1):
            if line.startswith("irq_hold_us"):
                holds += 1
            m = re.search(r"# expect: (.*)$", line)
            if m:
                want[i] = m.group(1).strip()
    got = {}
    with open(log) as f:
        for line in f:
            m = re.match(r"RESULT line=(\d+) .* -> (.*)$", line)
            if m:
                got[int(m.group(1))] = m.group(2).strip()
    bad = 0
    for ln, w in sorted(want.items()):
        g = got.get(ln, "(no RESULT)")
        if g != f"OK read: {w}":
            bad += 1
            print(f"line {ln}: want OK read: {w}, got {g}")
    print(f"reads checked {len(want)}, mismatches {bad}, holds {holds}")
    return bad


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "gen" and set(sys.argv[4:]) <= {"x", "nohold"}:
        flags = sys.argv[4:]
        gen(int(sys.argv[2]), int(sys.argv[3]), "x" in flags, "nohold" not in flags)
    elif len(sys.argv) == 4 and sys.argv[1] == "check":
        sys.exit(1 if check(sys.argv[2], sys.argv[3]) else 0)
    else:
        sys.exit(__doc__)
