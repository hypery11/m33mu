#!/usr/bin/env python3
# m33mu -- an ARMv8-M Emulator
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Look at the wrong reads of runs one 16-bit register at a time.

Each register in a wrong read of the LED colour registers 0x0310..0x0317 is
compared with that register's history in bus order:
  new      the value it has at that point of the script
  old-<k>  the value it had <k> writes before
  this     (write + read in one transfer) the value this transfer writes
  mixed    none of these: the two bytes come from different values
A read after data in the same transfer (write, repeated START, read) is
checked against the register pointer (the write's own address) and against
the next address (plain auto-increment, what hold_fuzz.py expects).

usage: classify_reads.py <script.txt> <master log> [<script.txt> <master log> ...]
"""
import re
import sys

BASE, N = 0x0310, 8


def results(log):
    got = {}
    for line in open(log):
        m = re.match(r"RESULT line=(\d+) .* -> OK read: (.*)$", line)
        if m:
            got[int(m.group(1))] = [int(x, 16) for x in m.group(2).split()]
    return got


def run(script, log, tally):
    got = results(log)
    regs = bytearray(N)
    hist = {r: [(0, 0)] for r in range(BASE, BASE + N, 2)}
    pointer = None
    out = []

    def write(addr, data):
        touched = []
        for i, b in enumerate(data):
            if 0 <= addr - BASE + i < N:
                regs[addr - BASE + i] = b
                r = (addr + i) & ~1
                if r not in touched:
                    touched.append(r)
        for r in touched:
            hist[r].append((regs[r - BASE], regs[r - BASE + 1]))

    def kinds(addr, value, this=None):
        ks = []
        for k in range(0, len(value) - 1, 2):
            r = addr + k
            if not BASE <= r < BASE + N:
                return None
            pair = (value[k], value[k + 1])
            h = hist[r]
            if this and pair == this.get(r):
                ks.append("this")
            elif pair == h[-1]:
                ks.append("new")
            elif pair in h:
                ks.append(f"old-{len(h) - 1 - max(i for i, v in enumerate(h) if v == pair)}")
            else:
                ks.append("mixed")
        return ks

    for ln, line in enumerate(open(script), 1):
        f = line.split("#")[0].split()
        m = re.search(r"# expect: (.*)$", line)
        want = [int(x, 16) for x in m.group(1).split()] if m else None
        if not f or f[0] not in ("write", "write_read", "read") or f[1] != "69":
            continue
        if f[0] == "read":
            addr, data, n = pointer, [], int(f[2])
        else:
            colon = f.index(":") if ":" in f else len(f)
            addr = int(f[2], 16) << 8 | int(f[3], 16)
            data = [int(x, 16) for x in f[4:colon]]
            n = int(f[colon + 1]) if f[0] == "write_read" else 0
            pointer = addr
        g = got.get(ln)
        text = " ".join(f)
        if f[0] == "write_read" and data:
            # write, repeated START, read
            if want is None or g == want:
                if want is not None:
                    tally["x next"] += 1
                write(addr, data)
                continue
            after = bytearray(regs)
            for i, b in enumerate(data):
                if 0 <= addr - BASE + i < N:
                    after[addr - BASE + i] = b
            this = {r: (after[r - BASE], after[r - BASE + 1]) for r in range(BASE, BASE + N, 2) if after[r - BASE:r - BASE + 2] != regs[r - BASE:r - BASE + 2]}
            ks = kinds(addr, g, this) if g else None  # "new" = the value before this write
            write(addr, data)
            if ks:
                ks = ["before" if k == "new" else k for k in ks]
            tally["x pointer" if ks and "mixed" not in ks else "x other"] += 1
            out.append(f"line {ln} {text}: pointer {' '.join(ks) if ks else 'no result'}")
            continue
        if f[0] == "write":
            write(addr, data)
            continue
        if want is None or g == want or addr == 0x0080:
            continue
        ks = kinds(addr, g) if g else None
        tally["plain"] += 1
        if ks:
            for k in ks:
                tally["reg " + ("old" if k.startswith("old") else k)] += 1
            if "new" in ks and any(k.startswith("old") for k in ks):
                tally["plain old+new"] += 1
            if ks[0].startswith("old") and all(k == "new" for k in ks[1:]):
                tally["plain first old"] += 1
        out.append(f"line {ln} {text}: {' '.join(ks) if ks else 'no result'}")
    return out


def main(args):
    if len(args) < 2 or len(args) % 2:
        sys.exit(__doc__)
    tally = {k: 0 for k in ("plain", "plain first old", "plain old+new", "reg new", "reg old", "reg mixed", "x next", "x pointer", "x other")}
    for i in range(0, len(args), 2):
        for line in run(args[i], args[i + 1], tally):
            print(f"{args[i].rsplit('/', 1)[-1]} {line}")
    print(f"wrong plain reads {tally['plain']}: only the first register old {tally['plain first old']}, "
          f"old and new registers {tally['plain old+new']}; registers new {tally['reg new']}, "
          f"old {tally['reg old']}, mixed {tally['reg mixed']}")
    print(f"write + read in one transfer: next address {tally['x next']}, pointer {tally['x pointer']}, "
          f"other {tally['x other']}")


if __name__ == "__main__":
    main(sys.argv[1:])
