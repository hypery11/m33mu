# Flipper One MCU firmware on the RP2350 profile

The `rp2350` profile runs the Flipper One MCU firmware
([flipperone-mcu-firmware](https://github.com/flipperdevices/flipperone-mcu-firmware),
`f1` and `f2` targets, RP2350B) up to its CLI. The models it needs beyond the
base profile are in `cpu/rp2350/`: a PL011 UART, multi-channel DMA, the
partition table, OTP rows, register stubs for blocks the firmware only
configures, and optional device models for the I2C buses. The device models
are off unless their environment variable is set.

## Build

Build as usual (see [Getting started](getting-started.md)). Nothing extra is
needed:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja -C build m33mu
```

## Running the firmware

The firmware build writes the image and `partition_table.uf2` (from
`targets/partition_table.json`) to its build directory. Pass the partition
table through `M33MU_RP2350_PT` so `rom_get_partition_table_info()` and
`rom_get_boot_info()` answer like the bootrom does after booting partition 0:

```sh
M33MU_RP2350_PT=partition_table.uf2 \
M33MU_RTT=rtt.log \
M33MU_OTP_FILE=otp.bin \
M33MU_UART_LINK_PREFIX=/tmp/m33mu- \
build/m33mu --cpu rp2350 --timeout 600 flipper_one_f1.uf2
```

Each UART opens a pty once the firmware enables it. With
`M33MU_UART_LINK_PREFIX` set, `/tmp/m33mu-UART0` and `/tmp/m33mu-UART1` link
to them. The stock `f1` and `f2` images have the CLI on UART1:

```sh
python3 -m serial.tools.miniterm /tmp/m33mu-UART1 115200
```

The baud rate on the host side does not matter. The firmware log goes to the
RTT file.

## Environment

General:

| Variable | Meaning |
|---|---|
| `M33MU_RP2350_PT=<file>` | picotool partition table, `.uf2` or `.bin` |
| `M33MU_RTT=<file\|->` | drain SEGGER RTT up-buffer 0 to a file or stdout |
| `M33MU_OTP_FILE=<file>` | OTP backing file for this run (default `rp2350_OTP.bin` in the working directory) |
| `M33MU_UART_LINK_PREFIX=<prefix>` | symlink each UART pty to `<prefix>UART<n>` |
| `M33MU_UART_LOG_PREFIX=<prefix>` | copy everything sent on UART`n` to `<prefix>UART<n>.txlog` |
| `M33MU_RP2350_CLK_PERI_HZ=<hz>` | clk_peri for UART bit timing (default 150 MHz) |

Device models:

| Variable | Meaning |
|---|---|
| `M33MU_INA219=1` | INA219 on the PIO I2C pads (`_ADDR`, `_SDA`, `_SCL`, `_SHUNT_UV`, `_BUS_MV`, `_LOG`) |
| `M33MU_INA219_EXTRA=0x40,...` | plain 16-bit register files at more addresses on the same pads |
| `M33MU_PCAL6416=1` | PCAL6416A on a DW_apb_i2c bus (`_BUS`, `_ADDR`, `_EXT`, `_INT`, `_RESET`, `_LOG`) |
| `M33MU_I2C1_MASTER=<script>` | I2C1 in slave mode with a scripted master, see below |
| `M33MU_I2C1_MASTER_LOG=<file>` | results of the script (default stdout) |
| `M33MU_I2C1_MASTER_HZ`, `_STRETCH_MS`, `_GAP_US` | SCL rate (400000), longest clock stretch (100), bus free time (50 us) |
| `M33MU_I2C1_TRACE=1` | also log the firmware's I2C1 register accesses and slave events |

The defaults of each model and the registers it covers are listed at the top
of its source file (`rp2350_ina219.c`, `rp2350_pcal6416.c`,
`rp2350_i2c1_master.c`).

Debugging: `M33MU_PL011_TRACE`, `M33MU_DMA_TRACE`, `M33MU_STUB_TRACE`,
`M33MU_NVIC_TRACE`, `M33MU_PIO_DUMP`, `M33MU_PC_SAMPLE=<cycles>`, and the
probes `M33MU_PROBE_LOG`, `M33MU_PROBE_FN`, `M33MU_PROBE_DMA_PIO`
(`rp2350_extra.c`).

## I2C1 scripted master

With `M33MU_I2C1_MASTER` set, I2C1 works in slave mode and a script plays
the CPU side of the bus (the firmware's I2C slave is at 0x69). One command per
line, `#` starts a comment, addresses and bytes in hex, counts and times in
decimal:

| Command | Meaning |
|---|---|
| `wait_log <text>` | wait until `<text>` shows up in the RTT log (needs `M33MU_RTT`) |
| `delay_ms <n>`, `delay_us <n>` | wait, emulated time |
| `gap_us <n>` | bus free time before each later START |
| `irq_hold_us <n>` | for the next `n` us the I2C1 IRQ line reads low to the NVIC, as if the firmware sat in a critical section; the FIFOs keep working |
| `write <addr> <byte>...` | START, address + W, the bytes, STOP |
| `read <addr> <n>` | START, address + R, `n` bytes, STOP |
| `write_read <addr> <byte>... : <n>` | write, repeated START, read `n` bytes, STOP |
| `log <text>` | copy `<text>` to the log |

Example: two short register writes while the IRQ is held off, then read them
back:

```
wait_log enabling I2C slave mode
delay_ms 50
irq_hold_us 3000
write 69 03 10 11 11
write 69 03 16 22 22
delay_ms 10
write_read 69 03 10 : 8
```

Each command logs a `RESULT` line with what came back. With the firmware at
`b2df0ff` (dev, 2026-11-09) the two writes run together
([mcu-firmware#263](https://github.com/flipperdevices/flipperone-mcu-firmware/issues/263)),
so the address of the second one lands in the registers after the first:

```
RESULT line=7 write_read 69 03 10 : 8 -> OK read: 11 11 03 16 22 22 00 00
```

An image that keeps the two writes apart reads `11 11 00 00 00 00 22 22`.

## Not modelled

- Time is emulated time. Durations in the logs are not board timings.
- The I2C models work at byte level: no SCL/SDA waveforms, set-up and hold
  times, glitch filter or arbitration.
- In I2C1 slave mode `IC_DATA_CMD` bit 11 (FIRST_DATA_BYTE) is set on the
  first byte after an address. Bit 10 (RESTART) always reads 0.
- The scripted master is not a model of the RK3576 I2C controller or of a
  Linux driver.
- The INA219 and PCAL6416A models answer the registers the firmware uses;
  they are not models of the silicon.
- USB is not covered here; use the UART CLI.
