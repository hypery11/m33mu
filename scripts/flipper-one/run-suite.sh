#!/bin/sh
# m33mu -- an ARMv8-M Emulator
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# The hold_fuzz suite: fuzz1..fuzz9 (with IRQ holds) and fuzz1nh, fuzz2nh
# (the same transactions as fuzz1/fuzz2, no holds).  Prints the wrong reads
# per script and the totals (384 reads with holds, 95 without).
#
# usage: run-suite.sh <image.uf2> <partition_table.uf2> [out dir]
#   M33MU, TIMEOUT and M33MU_* as for run-i2c1.sh
set -eu
if [ $# -lt 2 ]; then
    sed -n '5,10p' "$0" >&2
    exit 2
fi
here=$(cd "$(dirname "$0")" && pwd)
img=$1
pt=$2
out=${3:-$(mktemp -d "${TMPDIR:-/tmp}/f1-suite.XXXXXX")}
mkdir -p "$out"
: > "$out/suite.txt"
failed=0
for s in 1 2 3 4 5 6 7 8 9 1nh 2nh; do
    script=$here/i2c1/fuzz$s.txt
    if "$here/run-i2c1.sh" "$img" "$pt" "$script" "$out" > "$out/fuzz$s.sum" 2>&1; then
        python3 "$here/hold_fuzz.py" check "$script" "$out/fuzz$s.i2c1.log" > "$out/fuzz$s.check" || true
        echo "fuzz$s: $(tail -1 "$out/fuzz$s.check") | $(grep '^CLI after' "$out/fuzz$s.sum")" | tee -a "$out/suite.txt"
    else
        failed=$((failed + 1))
        echo "fuzz$s: run failed, see $out/fuzz$s.sum" | tee -a "$out/suite.txt"
    fi
done
awk '/reads checked/ { n = $4 + 0; m = $6 + 0; h = ($1 ~ /nh:$/) ? 2 : 1; wrong[h] += m; reads[h] += n }
     END { print "wrong reads: with holds " (wrong[1] + 0) "/" (reads[1] + 0) ", without holds " (wrong[2] + 0) "/" (reads[2] + 0)
           if (reads[1] != 384 || reads[2] != 95) print "not all reads checked: expected 384 and 95" }' "$out/suite.txt"
echo "logs: $out"
[ "$failed" -eq 0 ]
