#!/bin/sh
# m33mu -- an ARMv8-M Emulator
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# Run one I2C1 master script against a Flipper One MCU image and print its
# RESULT lines.  When the script has ended, "uptime" goes to the CLI on UART1
# to see that the firmware still answers.
#
# usage: run-i2c1.sh <image.uf2> <partition_table.uf2> <script.txt> [out dir]
#   M33MU    emulator binary (default build/m33mu)
#   TIMEOUT  wall-clock limit in seconds (default 300)
# Other M33MU_* variables pass through, e.g. M33MU_I2C1_TRACE=1 for the I2C1
# register trace in the script's log.
# The out dir (default a new one in $TMPDIR) gets <script>.i2c1.log (the
# script's log), <script>.rtt.log (firmware log) and <script>.emu.log.
# Exit status 1 when the script did not run to its end.
set -eu
if [ $# -lt 3 ]; then
    sed -n '5,16p' "$0" >&2
    exit 2
fi
img=$1
pt=$2
script=$3
out=${4:-$(mktemp -d "${TMPDIR:-/tmp}/f1-i2c1.XXXXXX")}
mkdir -p "$out"
m33mu=${M33MU:-build/m33mu}
if [ ! -x "$m33mu" ]; then
    echo "run-i2c1.sh: no emulator at $m33mu (set M33MU)" >&2
    exit 2
fi
name=$(basename "$script" .txt)
log=$out/$name.i2c1.log
rm -f "$log" "$out/$name.otp.bin" "$out/$name.rtt.log" "$out/$name.UART0" "$out/$name.UART1"

M33MU_RP2350_PT=$pt \
M33MU_RTT=$out/$name.rtt.log \
M33MU_OTP_FILE=$out/$name.otp.bin \
M33MU_UART_LINK_PREFIX=$out/$name. \
M33MU_I2C1_MASTER=$script \
M33MU_I2C1_MASTER_LOG=$log \
    "$m33mu" --cpu rp2350 --timeout "${TIMEOUT:-300}" "$img" > "$out/$name.emu.log" 2>&1 &
pid=$!
while kill -0 "$pid" 2>/dev/null; do
    if grep -q -e 'END of script' -e 'SCRIPT ERROR' "$log" 2>/dev/null; then
        break
    fi
    sleep 1
done

cli="no answer"
if kill -0 "$pid" 2>/dev/null && [ -e "$out/$name.UART1" ]; then
    cli=$(python3 - "$out/$name.UART1" <<'EOF'
import os, re, select, sys, time, tty
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd)
os.write(fd, b"\r\nuptime\r")
buf = b""
end = time.monotonic() + 5
while time.monotonic() < end:
    r, _, _ = select.select([fd], [], [], 0.2)
    if r:
        buf += os.read(fd, 4096)
    m = re.search(rb"Uptime[^\r\n]*\r?\n", buf)
    if m:
        break
m = re.search(rb"Uptime[^\r\n]*", buf)
print(m.group().decode() if m else "no answer")
EOF
) || cli="no answer"
fi
kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true

grep -E '^RESULT|END of script|SCRIPT ERROR' "$log" || echo "no results, see $out/$name.emu.log"
echo "CLI after: $cli"
echo "logs: $out"
grep -q 'END of script' "$log" 2>/dev/null
