/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * Behavioural model of an NXP PCAL6416A 16-bit I2C I/O
 * expander as a slave behind a DW_apb_i2c master (rp2350_extra.c).  It exists
 * to capture what a firmware's PCAL6416 driver puts on the bus and to answer
 * plausibly; it is NOT a model of the silicon.
 *
 * Environment:
 *   M33MU_PCAL6416=1             enable (default: off, the bus stays empty)
 *   M33MU_PCAL6416_LOG=<file>    register access log ("-" or unset: stdout)
 *   M33MU_PCAL6416_BUS=0         DW_apb_i2c instance (0 = I2C0, 1 = I2C1)
 *   M33MU_PCAL6416_ADDR=0x20     7-bit address (ADDR pin low; 0x21 = high)
 *   M33MU_PCAL6416_EXT=0xffff    level every port pin sees from outside while
 *                                it is high impedance (bit 8 + n = P1_n);
 *                                constant for the whole run
 *   M33MU_PCAL6416_INT=46        GPIO the INT output is wired to (-1: none)
 *   M33MU_PCAL6416_RESET=43      GPIO that drives RESET (-1: none)
 *
 * Bus side: byte level.  The master engine reports START + address, data
 * bytes and STOP with their times; the device ACKs its own address and every
 * byte written to it.  Nothing below a byte exists here (no SCL/SDA
 * waveforms, set-up/hold times, clock stretching, glitch filter, bus
 * time-out).
 *
 * Register side (PCAL6416A product data sheet Rev. 7.1, 30 August 2022,
 * sections 7.3 to 7.8, 8.1 and 8.2):
 *   write: first byte = command byte (register pointer), every further byte
 *          goes to the pointed register; the pointer then moves to the other
 *          register of the pair (00/01, 02/03 ... 4C/4D), so two data bytes
 *          fill a 16-bit pair, port 0 first when the command byte is even.
 *   read:  the pointed register, then the other one of the pair, and so on;
 *          the pointer is kept between transfers.
 *   00h/01h Input port     R, pin levels XOR Polarity inversion.  A pin
 *                          configured as output reads what it drives; any
 *                          other pin reads M33MU_PCAL6416_EXT.
 *   02h/03h Output port    R/W, power-on FFh; reads back what was written
 *   04h/05h Polarity inv.  R/W, power-on 00h
 *   06h/07h Configuration  R/W, power-on FFh (1 = input)
 *   40h-43h Drive strength R/W, power-on FFh; stored only
 *   44h/45h Input latch    R/W, power-on 00h; stored only
 *   46h/47h Pull enable    R/W, power-on 00h; stored only
 *   48h/49h Pull select    R/W, power-on FFh; stored only
 *   4Ah/4Bh Interrupt mask R/W, power-on FFh (1 = masked)
 *   4Ch/4Dh Interrupt stat R, pending and unmasked sources
 *   4Fh     Output config  R/W, power-on 00h, bit 0/1 = port 0/1 open drain
 *                          (an open-drain output only drives low); not part
 *                          of a pair, the pointer stays
 *   INT: an input pin is a pending source while its level differs from the
 *        Input port register content (the level at the last read of that
 *        port, or at reset).  INT is low while a pending source is unmasked;
 *        reading the port clears it.  With a constant EXT the only way to get
 *        there is the data sheet's "changing an I/O from an output to an input
 *        may cause a false interrupt".
 *   RESET: low restores the power-on register values and the device does not
 *        answer while it is low.  The pin is sampled only when the bus
 *        addresses the device or the MCU reads its GPIO inputs.
 *
 * Not modelled: input latching (44h/45h only store), pull-up/pull-down
 * resistors (46h-49h only store; EXT is the level), drive strength, port pin
 * timing (tv(Q), tv(INT), trst(INT)), level translation and the two supplies,
 * changing port inputs during a run, the reset pulse width.  Registers the
 * data sheet does not list (08h-3Fh, 4Eh, 50h and up) read FFh and ignore
 * writes; the data sheet does not say what the part does there.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rp2350/rp2350_pcal6416.h"
#include "rp2350/rp2350_mmio.h"
#include "rp2350/rp2350_timers.h"

struct pcal6416 {
    int enabled; /* -1 = environment not read yet */
    FILE *log;
    mm_u32 addr;
    mm_u32 bus;
    int int_pin;
    int reset_pin;
    mm_u16 ext;
    /* registers, [0] = port 0, [1] = port 1 */
    mm_u8 out[2];
    mm_u8 pol[2];
    mm_u8 cfg[2];
    mm_u8 drv[4];
    mm_u8 latch[2];
    mm_u8 pue[2];
    mm_u8 pud[2];
    mm_u8 mask[2];
    mm_u8 odcfg;
    mm_u8 in_reg[2]; /* Input port register content, for the interrupt compare */
    mm_u8 ptr;
    mm_u8 in_reset;
    mm_u8 int_low;
    /* transfer in progress */
    mm_u8 addressed;
    mm_u8 rw;
    mm_u8 xfer_reg;    /* register the first data byte went to / came from */
    mm_u32 n_data;     /* data bytes moved, the command byte not counted */
    mm_u32 n_wr_bytes; /* bytes received in a write transfer, command byte included */
    mm_u8 data[4];
    mm_u16 was;
    mm_u64 t_last;
    /* counters */
    mm_u64 n_start;
    mm_u64 n_match;
    mm_u64 n_other;
    mm_u64 n_wr;
    mm_u64 n_out_wr;
    mm_u64 n_rd;
    mm_u64 n_in_rd;
    mm_u64 n_int;
    mm_u64 n_reset;
    mm_u16 last_out;
};

static struct pcal6416 g = {.enabled = -1};

static mm_bool reg_paired(mm_u32 reg)
{
    return (reg <= 0x07u || (reg >= 0x40u && reg <= 0x4du)) ? MM_TRUE : MM_FALSE;
}

static mm_bool reg_exists(mm_u32 reg)
{
    return (reg_paired(reg) || reg == 0x4fu) ? MM_TRUE : MM_FALSE;
}

static mm_bool reg_read_only(mm_u32 reg)
{
    return (reg <= 0x01u || reg == 0x4cu || reg == 0x4du) ? MM_TRUE : MM_FALSE;
}

static const char *reg_name(mm_u32 reg)
{
    if (reg == 0x4fu) return "OUTPUT_CONFIG";
    if (!reg_paired(reg)) return "(none)";
    switch (reg & 0xfeu) {
    case 0x00u: return "INPUT";
    case 0x02u: return "OUTPUT";
    case 0x04u: return "POLARITY";
    case 0x06u: return "CONFIG";
    case 0x40u: return "DRIVE_P0";
    case 0x42u: return "DRIVE_P1";
    case 0x44u: return "INPUT_LATCH";
    case 0x46u: return "PULL_ENABLE";
    case 0x48u: return "PULL_SELECT";
    case 0x4au: return "INT_MASK";
    default: return "INT_STATUS";
    }
}

static mm_u64 to_us(mm_u64 cycles)
{
    mm_u64 per_us = mm_rp2350_cpu_hz() / 1000000ull;
    if (per_us == 0u) per_us = 1u;
    return cycles / per_us;
}

static void logf_head(mm_u64 t, const char *what)
{
    fprintf(g.log, "t_us=%-10llu %-5s", (unsigned long long)to_us(t), what);
}

static mm_u16 u16(const mm_u8 r[2])
{
    return (mm_u16)(((mm_u16)r[1] << 8) | r[0]);
}

/* Pins the device drives, and the level on all 16 pins. */
static mm_u16 pins_driven(void)
{
    return (mm_u16)~u16(g.cfg);
}

static mm_u16 pin_levels(void)
{
    mm_u16 outputs = pins_driven();
    mm_u16 out = u16(g.out);
    mm_u16 od = (mm_u16)(((g.odcfg & 1u) ? 0x00ffu : 0u) | ((g.odcfg & 2u) ? 0xff00u : 0u));
    mm_u16 low = (mm_u16)(outputs & ~out);
    mm_u16 high = (mm_u16)(outputs & out & ~od);
    return (mm_u16)((g.ext | high) & ~low);
}

static mm_u16 input_now(void)
{
    return (mm_u16)(pin_levels() ^ u16(g.pol));
}

static mm_u16 int_status(void)
{
    mm_u16 pending = (mm_u16)((input_now() ^ u16(g.in_reg)) & u16(g.cfg));
    return (mm_u16)(pending & ~u16(g.mask));
}

static void update_int(mm_u64 t)
{
    mm_u16 st = int_status();
    mm_u8 low = (st != 0u) ? 1u : 0u;
    if (low == g.int_low) return;
    g.int_low = low;
    if (low) g.n_int++;
    logf_head(t, "INT");
    if (low) fprintf(g.log, " asserted (low), status=0x%04x\n", (unsigned)st);
    else fprintf(g.log, " released (high)\n");
    fflush(g.log);
}

static void power_on(void)
{
    mm_u16 in;
    memset(g.out, 0xff, sizeof(g.out));
    memset(g.pol, 0x00, sizeof(g.pol));
    memset(g.cfg, 0xff, sizeof(g.cfg));
    memset(g.drv, 0xff, sizeof(g.drv));
    memset(g.latch, 0x00, sizeof(g.latch));
    memset(g.pue, 0x00, sizeof(g.pue));
    memset(g.pud, 0xff, sizeof(g.pud));
    memset(g.mask, 0xff, sizeof(g.mask));
    g.odcfg = 0u;
    g.ptr = 0u;
    in = input_now();
    g.in_reg[0] = (mm_u8)(in & 0xffu);
    g.in_reg[1] = (mm_u8)(in >> 8);
    g.int_low = 0u;
    g.addressed = 0u;
    g.n_data = 0u;
    g.n_wr_bytes = 0u;
}

static mm_u8 *reg_store(mm_u32 reg)
{
    switch (reg) {
    case 0x02u: case 0x03u: return &g.out[reg & 1u];
    case 0x04u: case 0x05u: return &g.pol[reg & 1u];
    case 0x06u: case 0x07u: return &g.cfg[reg & 1u];
    case 0x40u: case 0x41u: case 0x42u: case 0x43u: return &g.drv[reg & 3u];
    case 0x44u: case 0x45u: return &g.latch[reg & 1u];
    case 0x46u: case 0x47u: return &g.pue[reg & 1u];
    case 0x48u: case 0x49u: return &g.pud[reg & 1u];
    case 0x4au: case 0x4bu: return &g.mask[reg & 1u];
    case 0x4fu: return &g.odcfg;
    default: break;
    }
    return 0;
}

static mm_u8 reg_get(mm_u32 reg)
{
    const mm_u8 *p;
    if (reg <= 0x01u) return (mm_u8)(input_now() >> (8u * reg));
    if (reg == 0x4cu || reg == 0x4du) return (mm_u8)(int_status() >> (8u * (reg & 1u)));
    p = reg_store(reg);
    return (p != 0) ? *p : 0xffu;
}

/* The pair a register belongs to as one value, port 1 in the high byte. */
static mm_u16 pair16(mm_u32 reg)
{
    if (!reg_paired(reg)) return reg_get(reg);
    return (mm_u16)(((mm_u16)reg_get(reg | 1u) << 8) | reg_get(reg & ~1u));
}

static void load_env(void)
{
    const char *v = getenv("M33MU_PCAL6416");
    const char *p;
    g.enabled = (v != 0 && v[0] != '\0' && strcmp(v, "0") != 0) ? 1 : 0;
    if (!g.enabled) return;
    g.addr = 0x20u;
    g.bus = 0u;
    g.int_pin = 46;
    g.reset_pin = 43;
    g.ext = 0xffffu;
    if ((p = getenv("M33MU_PCAL6416_ADDR")) != 0 && p[0] != '\0') g.addr = (mm_u32)strtoul(p, 0, 0) & 0x7fu;
    if ((p = getenv("M33MU_PCAL6416_BUS")) != 0 && p[0] != '\0') g.bus = (mm_u32)strtoul(p, 0, 0) & 1u;
    if ((p = getenv("M33MU_PCAL6416_EXT")) != 0 && p[0] != '\0') g.ext = (mm_u16)strtoul(p, 0, 0);
    if ((p = getenv("M33MU_PCAL6416_INT")) != 0 && p[0] != '\0') g.int_pin = (int)strtol(p, 0, 0);
    if ((p = getenv("M33MU_PCAL6416_RESET")) != 0 && p[0] != '\0') g.reset_pin = (int)strtol(p, 0, 0);
    if (g.int_pin > 47) g.int_pin = -1;
    if (g.reset_pin > 47) g.reset_pin = -1;
    p = getenv("M33MU_PCAL6416_LOG");
    if (p != 0 && p[0] != '\0' && strcmp(p, "-") != 0) g.log = fopen(p, "w");
    if (g.log == 0) g.log = stdout;
    power_on();
    fprintf(g.log, "# PCAL6416A behavioural model (emulated, not silicon): bus=I2C%u addr=0x%02x INT=GPIO%d "
                   "RESET=GPIO%d ext=0x%04x; t_us = emulated time since reset; 16-bit values are "
                   "port 1 << 8 | port 0\n",
            (unsigned)g.bus, (unsigned)g.addr, g.int_pin, g.reset_pin, (unsigned)g.ext);
    fflush(g.log);
    printf("[PCAL6416] model on: bus=I2C%u addr=0x%02x INT=GPIO%d RESET=GPIO%d ext=0x%04x\n",
           (unsigned)g.bus, (unsigned)g.addr, g.int_pin, g.reset_pin, (unsigned)g.ext);
}

mm_bool mm_rp2350_pcal6416_enabled(void)
{
    if (g.enabled < 0) load_env();
    return g.enabled ? MM_TRUE : MM_FALSE;
}

mm_u32 mm_rp2350_pcal6416_bus(void)
{
    if (g.enabled < 0) load_env();
    return g.bus;
}

/* RESET is driven by an MCU pad: low when the pad's output enable is on and
 * its output level is 0, IO_BANK0 OEOVER/OUTOVER applied (the firmware makes
 * an open-drain output out of OEOVER = invert and OUTOVER = low). */
static void sample_reset(mm_u64 t)
{
    mm_u32 out_lo = 0u, out_hi = 0u, oe_lo = 0u, oe_hi = 0u;
    mm_u64 over[6];
    mm_u64 out, oe, bit;
    mm_u8 low;

    if (g.reset_pin < 0) return;
    bit = 1ull << g.reset_pin;
    mm_rp2350_sio_pad_state(&out_lo, &out_hi, &oe_lo, &oe_hi);
    mm_rp2350_gpio_overrides(over);
    out = (mm_u64)out_lo | ((mm_u64)out_hi << 32);
    oe = (mm_u64)oe_lo | ((mm_u64)oe_hi << 32);
    oe = ((oe ^ over[0]) & ~over[1]) | over[2];
    out = ((out ^ over[3]) & ~over[4]) | over[5];
    low = ((oe & bit) != 0u && (out & bit) == 0u) ? 1u : 0u;
    if (low == g.in_reset) return;
    g.in_reset = low;
    logf_head(t, "RESET");
    if (low) {
        g.n_reset++;
        power_on();
        fprintf(g.log, " low: registers back to their power-on values\n");
    } else {
        fprintf(g.log, " released\n");
    }
    fflush(g.log);
}

static void print_data(void)
{
    mm_u32 i;
    fprintf(g.log, " bytes=%lu data=", (unsigned long)g.n_data);
    for (i = 0; i < g.n_data && i < sizeof(g.data); ++i) fprintf(g.log, "%s%02x", i ? " " : "", (unsigned)g.data[i]);
    if (g.n_data > sizeof(g.data)) fprintf(g.log, " ...");
}

/* Close the bookkeeping of the transfer that a STOP or a new START ends. */
static void end_xfer(void)
{
    if (g.addressed) {
        if (g.rw) {
            if (g.n_data != 0u) {
                g.n_rd++;
                if (g.xfer_reg <= 0x01u) g.n_in_rd++;
                logf_head(g.t_last, "RD");
                fprintf(g.log, " reg=0x%02x %-13s", (unsigned)g.xfer_reg, reg_name(g.xfer_reg));
                print_data();
                if (g.n_data >= 2u && reg_paired(g.xfer_reg)) {
                    mm_u8 lo = (g.xfer_reg & 1u) ? g.data[1] : g.data[0];
                    mm_u8 hi = (g.xfer_reg & 1u) ? g.data[0] : g.data[1];
                    fprintf(g.log, " val=0x%04x", (unsigned)(((mm_u16)hi << 8) | lo));
                } else {
                    fprintf(g.log, " val=0x%02x", (unsigned)g.data[0]);
                }
                fprintf(g.log, "%s\n", reg_exists(g.xfer_reg) ? "" : "  (no such register)");
            }
        } else if (g.n_wr_bytes == 0u) {
            logf_head(g.t_last, "PROBE");
            fprintf(g.log, " address ACKed, no data\n");
        } else if (g.n_data == 0u) {
            logf_head(g.t_last, "PTR");
            fprintf(g.log, " reg=0x%02x %s%s\n", (unsigned)g.ptr, reg_name(g.ptr),
                    reg_exists(g.ptr) ? "" : "  (no such register)");
        } else {
            mm_u16 now = pair16(g.xfer_reg);
            g.n_wr++;
            logf_head(g.t_last, "WR");
            fprintf(g.log, " reg=0x%02x %-13s", (unsigned)g.xfer_reg, reg_name(g.xfer_reg));
            print_data();
            if (!reg_exists(g.xfer_reg)) {
                fprintf(g.log, "  IGNORED (no such register)\n");
            } else if (reg_read_only(g.xfer_reg)) {
                fprintf(g.log, "  IGNORED (read-only register)\n");
            } else {
                fprintf(g.log, " val=0x%04x was=0x%04x%s", (unsigned)now, (unsigned)g.was,
                        (now == g.was) ? " (unchanged)" : "");
                if ((g.xfer_reg & 0xfeu) == 0x02u || (g.xfer_reg & 0xfeu) == 0x06u) {
                    fprintf(g.log, "  pins driven: 0x%04x of outputs 0x%04x",
                            (unsigned)(pin_levels() & pins_driven()), (unsigned)pins_driven());
                }
                fprintf(g.log, "\n");
                if ((g.xfer_reg & 0xfeu) == 0x02u) {
                    g.n_out_wr++;
                    g.last_out = u16(g.out);
                }
            }
        }
        fflush(g.log);
    }
    g.addressed = 0u;
    g.n_data = 0u;
    g.n_wr_bytes = 0u;
}

mm_bool mm_rp2350_pcal6416_start(mm_u64 t, mm_u32 addr7, mm_bool read)
{
    if (g.enabled <= 0) return MM_FALSE;
    end_xfer(); /* a repeated START closes the transfer before it */
    g.n_start++;
    sample_reset(t);
    if (addr7 != g.addr) {
        g.n_other++;
        return MM_FALSE;
    }
    if (g.in_reset) return MM_FALSE;
    g.n_match++;
    g.addressed = 1u;
    g.rw = read ? 1u : 0u;
    g.t_last = t;
    return MM_TRUE;
}

mm_bool mm_rp2350_pcal6416_write(mm_u64 t, mm_u8 byte)
{
    if (g.enabled <= 0 || !g.addressed || g.rw) return MM_FALSE;
    g.t_last = t;
    if (g.n_wr_bytes == 0u) {
        g.ptr = byte;
    } else {
        mm_u8 *p = reg_store(g.ptr);
        if (g.n_data == 0u) {
            g.xfer_reg = g.ptr;
            g.was = pair16(g.ptr);
        }
        if (g.n_data < sizeof(g.data)) g.data[g.n_data] = byte;
        g.n_data++;
        if (p != 0) *p = (g.ptr == 0x4fu) ? (mm_u8)(byte & 0x03u) : byte;
        if (reg_paired(g.ptr)) g.ptr ^= 1u;
        update_int(t);
    }
    g.n_wr_bytes++;
    return MM_TRUE;
}

mm_u8 mm_rp2350_pcal6416_read(mm_u64 t, mm_bool master_ack)
{
    mm_u8 v;
    (void)master_ack; /* the next byte is only produced when the master asks for it */
    if (g.enabled <= 0 || !g.addressed || !g.rw) return 0xffu;
    g.t_last = t;
    v = reg_get(g.ptr);
    if (g.ptr <= 0x01u) g.in_reg[g.ptr] = v; /* reading a port clears its interrupt */
    if (g.n_data == 0u) g.xfer_reg = g.ptr;
    if (g.n_data < sizeof(g.data)) g.data[g.n_data] = v;
    g.n_data++;
    if (reg_paired(g.ptr)) g.ptr ^= 1u;
    update_int(t);
    return v;
}

void mm_rp2350_pcal6416_stop(mm_u64 t)
{
    (void)t;
    if (g.enabled <= 0) return;
    end_xfer();
}

void mm_rp2350_pcal6416_pads(mm_u64 *high, mm_u64 *low)
{
    if (g.enabled <= 0) return;
    sample_reset(mm_rp2350_now_cycles());
    if (g.int_pin < 0) return;
    if (g.int_low) {
        if (low != 0) *low |= 1ull << g.int_pin;
    } else {
        if (high != 0) *high |= 1ull << g.int_pin;
    }
}

void mm_rp2350_pcal6416_stats(void)
{
    if (g.enabled <= 0) return;
    fprintf(stderr, "[PCAL6416] starts=%llu addressed=%llu other_addr=%llu reg_writes=%llu (output=%llu, last=0x%04x) "
                    "reg_reads=%llu (input=%llu) int_asserted=%llu resets=%llu\n",
            (unsigned long long)g.n_start, (unsigned long long)g.n_match, (unsigned long long)g.n_other,
            (unsigned long long)g.n_wr, (unsigned long long)g.n_out_wr, (unsigned)g.last_out,
            (unsigned long long)g.n_rd, (unsigned long long)g.n_in_rd, (unsigned long long)g.n_int,
            (unsigned long long)g.n_reset);
    if (g.log != 0 && g.log != stdout) fflush(g.log);
}
