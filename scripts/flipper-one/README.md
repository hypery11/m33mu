# Flipper One MCU: I2C1 test scripts

Scripts for the I2C1 scripted master (see
[docs/rp2350-flipper-one.md](../../docs/rp2350-flipper-one.md)), used for
flipperone-mcu-firmware
[#263](https://github.com/flipperdevices/flipperone-mcu-firmware/issues/263),
[#281](https://github.com/flipperdevices/flipperone-mcu-firmware/pull/281) and
[#284](https://github.com/flipperdevices/flipperone-mcu-firmware/pull/284).
They need `python3` and an m33mu build, nothing else.

| File | |
|---|---|
| `run-i2c1.sh` | one script on one image: its RESULT lines, then `uptime` on the CLI |
| `run-suite.sh` | the fuzz suite below, with the totals |
| `hold_fuzz.py` | makes the fuzz scripts and checks a run against their `# expect:` values |
| `classify_reads.py` | looks at the wrong reads one 16-bit register at a time (new, old, mixed) |
| `nvm_flash_delay.py` | test aid for `nvm284*.txt`: adds flash time to the NVM write in the firmware tree |
| `i2c1/` | the scripts |

## Running

The firmware build directory has the image and `partition_table.uf2`. From
the m33mu tree:

```sh
scripts/flipper-one/run-suite.sh <fw build>/flipperone-mcu-firmware.uf2 <fw build>/partition_table.uf2
scripts/flipper-one/run-i2c1.sh <fw build>/flipperone-mcu-firmware.uf2 <fw build>/partition_table.uf2 scripts/flipper-one/i2c1/hold263.txt
```

`M33MU` picks the emulator binary (default `build/m33mu`), `TIMEOUT` the
wall-clock limit of one run (default 300 s). A run takes about 20 s. The logs
go to a new directory under `$TMPDIR`, or to the one given as the last
argument. Other `M33MU_*` variables pass through: with `M33MU_I2C1_TRACE=1` the
script's log also has the firmware's I2C1 register accesses (INTR_STAT,
DATA_CMD, ...).

To check a single run or sort its wrong reads:

```sh
scripts/flipper-one/hold_fuzz.py check scripts/flipper-one/i2c1/fuzz7.txt <out>/fuzz7.i2c1.log
scripts/flipper-one/classify_reads.py scripts/flipper-one/i2c1/fuzz7.txt <out>/fuzz7.i2c1.log
```

## Scripts

- `fuzz1`..`fuzz9`: 80 random transactions each on the LED colour registers
  0x0310..0x0317 and the version register 0x0080, with 18 to 26 IRQ holds of
  100..3000 us. `fuzz7`..`fuzz9` also write and read in one transfer.
  `hold_fuzz.py gen <seed> 80` makes `fuzz1`..`fuzz6`,
  `hold_fuzz.py gen <seed> 80 x` makes `fuzz7`..`fuzz9`.
- `fuzz1nh`, `fuzz2nh`: `fuzz1` and `fuzz2` without the holds
  (`hold_fuzz.py gen <seed> 80 nohold`).
- `hold263`, `hold263b`, `hold263c`: the cases from #263, written by hand,
  with the expected values in the comments.
- `gap0`: a START at the same time as the STOP before it.
- `nvm284`, `nvm284r`: link LED brightness writes, which the firmware saves to
  NVM, with colour writes right after; `nvm284r` also reads back while the NVM
  write is running. m33mu programs flash instantly, so run
  `nvm_flash_delay.py <firmware tree>` and rebuild first, then
  `run-i2c1.sh` with the script and `hold_fuzz.py check` on its log.

`irq_hold_us` holds the I2C1 IRQ line low to the NVIC. It does not depend on
BASEPRI or PRIMASK, so a hold stands for "the I2C IRQ is late", whatever the
reason.

For a write and a read in one transfer, `hold_fuzz.py` expects the bytes after
the written ones (plain auto-increment). Where the firmware answers such a
read from the register pointer instead, the read counts as wrong.

These are emulator runs. See "Not modelled" in the doc for what the I2C1
model leaves out.
