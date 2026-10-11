/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * Behavioural model of a TI INA219 current/power monitor as
 * an I2C slave on two GPIO pads.  It exists to capture what a firmware's
 * INA219 driver puts on the bus and to answer plausibly; it is NOT a model of
 * the silicon.
 *
 * Environment:
 *   M33MU_INA219=1            enable (default: off, the bus stays empty)
 *   M33MU_INA219_LOG=<file>   register access log ("-" or unset: stdout)
 *   M33MU_INA219_ADDR=0x45    7-bit address
 *   M33MU_INA219_SDA=22       SDA pad (GPIO number)
 *   M33MU_INA219_SCL=23       SCL pad
 *   M33MU_INA219_SHUNT_UV=60000   simulated voltage across the shunt, in uV
 *                                 (signed; 60 mV = 6 A through 10 mOhm)
 *   M33MU_INA219_BUS_MV=7600      simulated bus voltage, in mV
 *   M33MU_INA219_EXTRA=0x40,0x41  more 7-bit addresses on the same pads.  Each
 *                                 one is a plain register file, not an INA219:
 *                                 256 16-bit registers (all 0 at reset), its own
 *                                 pointer, a write stores the value and a read
 *                                 returns it, MSB first.  Logged as XWR/XRD.
 *
 * Bus side (pad level, so it works with a PIO bit-banged master):
 *   START/STOP/repeated START detection, address match with ACK, every
 *   written byte ACKed, read data shifted out MSB first until the master
 *   NAKs.  The device reacts one master clock step after an edge (that stands
 *   in for the SDA hold time).  No clock stretching, no bus timeout, no
 *   general call, no high-speed mode.
 *
 * Register side (INA219 datasheet SBOS448G, sections 8.5 and 8.6):
 *   write: first byte = register pointer, next two bytes = value, MSB first;
 *          the pointer is kept between transfers.  Further bytes are ignored.
 *   read:  the pointed register, MSB then LSB (repeating if the master keeps
 *          clocking).
 *   00h Configuration  R/W, power-on 399Fh (figure 19); RST (bit 15) restores
 *                      the power-on state and reads back 0.  Reads back what
 *                      was written, MODE included.
 *   01h Shunt voltage  R, LSB 10 uV, two's complement, clipped at the PGA
 *                      full scale of the configuration that started the
 *                      conversion (table 7): PG 00/01/10/11 = +-40/80/160/
 *                      320 mV = +-0FA0h/1F40h/3E80h/7D00h; the sign extension
 *                      into the unused upper bits follows from that.
 *   02h Bus voltage    R, (V / 4 mV) << 3 | CNVR << 1 | OVF (figure 24).
 *   03h Power          R, |Current| * BusVoltage / 5000 (equation 5; the
 *                      magnitude for a negative current is this model's
 *                      choice); reading it clears CNVR.
 *   04h Current        R, ShuntVoltage * Calibration / 4096 (equation 4),
 *                      truncated.
 *   05h Calibration    R/W, power-on 0000h, bit 0 always 0 (figure 27).
 *   Conversions: MODE 1/2/3 (triggered) start one conversion when the
 *   configuration is written; the result lands in the data registers after
 *   the conversion time of table 5 (84 us ... 68.1 ms; shunt + bus when both
 *   are converted), and CNVR is 0 in between.  MODE 5/6/7 (continuous, also
 *   the power-on mode) behave the same for the first conversion and return
 *   fresh values on every later read.  MODE 0/4 convert nothing; the data
 *   registers keep their last values.  Current and Power are recomputed from
 *   the latest Shunt and Bus values after every conversion.  OVF is set when
 *   Current or Power does not fit its register (the value is then saturated;
 *   the datasheet only says the data "may be meaningless").
 *
 * Not modelled: ADC resolution and averaging (BADC/SADC only set the
 * conversion time), noise, offset and gain error, the bus-voltage range
 * (BRNG is stored and ignored), supply or common-mode limits, the 4 us
 * register update delay, the 40 us recovery from power-down, the 28 ms bus
 * timeout, general-call reset, SMBus alert, any register above 05h (reads
 * 0000h, writes ignored).  Whether the MODE bits of a real part change once
 * a triggered conversion is done is not something the datasheet states; here
 * they do not.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rp2350/rp2350_ina219.h"
#include "rp2350/rp2350_mmio.h"

enum { ST_IDLE = 0, ST_ADDR, ST_ACK_ADDR, ST_RX, ST_ACK_RX, ST_TX, ST_TX_ACK, ST_DONE };

struct ina219 {
    int enabled; /* -1 = environment not read yet */
    FILE *log;
    mm_u32 addr;
    mm_u32 sda_pin;
    mm_u32 scl_pin;
    mm_i32 vshunt_uv;
    mm_u32 vbus_mv;
    /* time */
    mm_u64 cycles;
    /* registers */
    mm_u16 config;
    mm_u16 calib;
    mm_i16 shunt;
    mm_i16 current;
    mm_u16 power;
    mm_u16 bus;   /* 13-bit value, LSB 4 mV */
    mm_u8 cnvr;
    mm_u8 ovf;
    mm_u8 ptr;
    /* conversion in flight */
    mm_u8 conv_pending;
    mm_u8 conv_logged;
    mm_u16 conv_cfg;
    mm_u64 conv_ready_us;
    /* bit engine */
    mm_u8 st;
    mm_u8 scl_prev;
    mm_u8 sda_prev;
    mm_u8 sda_low;
    mm_u8 nbits;
    mm_u8 shift;
    mm_u8 rw;
    mm_u8 addressed;
    mm_u8 master_ack;
    mm_u8 txbyte;
    mm_u8 wr_hi;
    mm_u32 wr_count;
    mm_u32 rd_bytes;
    mm_u16 rd_val;
    mm_u8 rd_reg;
    mm_u8 rd_stale;
    /* counters */
    mm_u64 n_start;
    mm_u64 n_match;
    mm_u64 n_other;
    mm_u64 n_wr;
    mm_u64 n_rd;
    mm_u64 n_cfg_wr;
    mm_u64 n_stale;
    mm_u16 last_cfg_wr;
    /* extra register-file devices (M33MU_INA219_EXTRA) */
    mm_u8 extra[128];
    mm_u8 cur; /* address of the device in the current transfer */
    mm_u64 n_xwr;
    mm_u64 n_xrd;
    mm_u64 n_xprobe;
};

static struct ina219 g = {.enabled = -1};
static mm_u16 g_xreg[128][256];
static mm_u8 g_xptr[128];

static const char *const k_reg_name[6] = {"CONFIG", "SHUNT", "BUS", "POWER", "CURRENT", "CALIBRATION"};
static const mm_i32 k_pga_fs[4] = {4000, 8000, 16000, 32000}; /* in 10 uV LSBs */

static const char *reg_name(mm_u32 reg)
{
    return (reg < 6u) ? k_reg_name[reg] : "(none)";
}

static mm_u64 now_us(void)
{
    mm_u64 per_us = mm_rp2350_cpu_hz() / 1000000ull;
    if (per_us == 0u) per_us = 1u;
    return g.cycles / per_us;
}

/* Datasheet ADC setting table: conversion time in us for a BADC/SADC nibble. */
static mm_u32 adc_time_us(mm_u32 nibble)
{
    static const mm_u32 single[4] = {84u, 148u, 276u, 532u};
    static const mm_u32 avg[8] = {532u, 1060u, 2130u, 4260u, 8510u, 17020u, 34050u, 68100u};
    if ((nibble & 0x8u) == 0u) return single[nibble & 0x3u];
    return avg[nibble & 0x7u];
}

static const char *adc_text(mm_u32 nibble)
{
    static const char *const single[4] = {"9-bit", "10-bit", "11-bit", "12-bit"};
    static const char *const avg[8] = {"12-bit", "12-bit x2", "12-bit x4", "12-bit x8",
                                       "12-bit x16", "12-bit x32", "12-bit x64", "12-bit x128"};
    if ((nibble & 0x8u) == 0u) return single[nibble & 0x3u];
    return avg[nibble & 0x7u];
}

static const char *mode_text(mm_u32 mode)
{
    static const char *const t[8] = {"power-down", "shunt, triggered", "bus, triggered",
                                     "shunt+bus, triggered", "ADC off", "shunt, continuous",
                                     "bus, continuous", "shunt+bus, continuous"};
    return t[mode & 7u];
}

static mm_u32 conv_time_us(mm_u16 cfg)
{
    mm_u32 t = 0u;
    if (cfg & 0x1u) t += adc_time_us((cfg >> 3) & 0xfu);
    if (cfg & 0x2u) t += adc_time_us((cfg >> 7) & 0xfu);
    return t;
}

static void power_on(void)
{
    g.config = 0x399fu;
    g.calib = 0u;
    g.shunt = 0;
    g.current = 0;
    g.power = 0u;
    g.bus = 0u;
    g.cnvr = 0u;
    g.ovf = 0u;
    /* power-on mode is shunt+bus continuous: the first result is on its way */
    g.conv_pending = 1u;
    g.conv_logged = 0u;
    g.conv_cfg = g.config;
    g.conv_ready_us = now_us() + conv_time_us(g.config);
}

static void load_env(void)
{
    const char *v = getenv("M33MU_INA219");
    const char *p;
    g.enabled = (v != 0 && v[0] != '\0' && strcmp(v, "0") != 0) ? 1 : 0;
    if (!g.enabled) return;
    g.addr = 0x45u;
    g.sda_pin = 22u;
    g.scl_pin = 23u;
    g.vshunt_uv = 60000;
    g.vbus_mv = 7600u;
    if ((p = getenv("M33MU_INA219_ADDR")) != 0 && p[0] != '\0') g.addr = (mm_u32)strtoul(p, 0, 0) & 0x7fu;
    if ((p = getenv("M33MU_INA219_SDA")) != 0 && p[0] != '\0') g.sda_pin = (mm_u32)strtoul(p, 0, 0) % 48u;
    if ((p = getenv("M33MU_INA219_SCL")) != 0 && p[0] != '\0') g.scl_pin = (mm_u32)strtoul(p, 0, 0) % 48u;
    if ((p = getenv("M33MU_INA219_SHUNT_UV")) != 0 && p[0] != '\0') g.vshunt_uv = (mm_i32)strtol(p, 0, 0);
    if ((p = getenv("M33MU_INA219_BUS_MV")) != 0 && p[0] != '\0') g.vbus_mv = (mm_u32)strtoul(p, 0, 0);
    if ((p = getenv("M33MU_INA219_EXTRA")) != 0) {
        while (*p != '\0') {
            char *end = 0;
            unsigned long a = strtoul(p, &end, 0);
            if (end == p) break;
            if (a < 128u && a != g.addr) g.extra[a] = 1u;
            p = end;
            while (*p == ',' || *p == ' ') p++;
        }
    }
    p = getenv("M33MU_INA219_LOG");
    if (p != 0 && p[0] != '\0' && strcmp(p, "-") != 0) g.log = fopen(p, "w");
    if (g.log == 0) g.log = stdout;
    g.scl_prev = 1u;
    g.sda_prev = 1u;
    power_on();
    fprintf(g.log, "# INA219 behavioural model (emulated, not silicon): addr=0x%02x SDA=GPIO%u SCL=GPIO%u "
                   "vshunt=%lduV vbus=%lumV; t_us = emulated time since reset\n",
            (unsigned)g.addr, (unsigned)g.sda_pin, (unsigned)g.scl_pin, (long)g.vshunt_uv,
            (unsigned long)g.vbus_mv);
    for (mm_u32 a = 0; a < 128u; a++) {
        if (g.extra[a]) fprintf(g.log, "# extra register file at 0x%02x\n", (unsigned)a);
    }
    fflush(g.log);
    printf("[INA219] model on: addr=0x%02x SDA=GPIO%u SCL=GPIO%u vshunt=%lduV vbus=%lumV\n",
           (unsigned)g.addr, (unsigned)g.sda_pin, (unsigned)g.scl_pin, (long)g.vshunt_uv,
           (unsigned long)g.vbus_mv);
}

mm_bool mm_rp2350_ina219_enabled(void)
{
    if (g.enabled < 0) load_env();
    return g.enabled ? MM_TRUE : MM_FALSE;
}

void mm_rp2350_ina219_tick(mm_u64 cycles)
{
    g.cycles += cycles;
}

static void logf_head(const char *what)
{
    fprintf(g.log, "t_us=%-10llu %-5s", (unsigned long long)now_us(), what);
}

/* One conversion with configuration `cfg`: update the data registers. */
static void convert(mm_u16 cfg)
{
    mm_i32 fs = k_pga_fs[(cfg >> 11) & 0x3u];
    mm_i32 raw = g.vshunt_uv / 10;
    mm_i32 clipped = raw;
    mm_i64 cur;
    mm_i64 pwr;

    g.ovf = 0u;
    if (cfg & 0x1u) {
        if (clipped > fs) clipped = fs;
        if (clipped < -fs) clipped = -fs;
        g.shunt = (mm_i16)clipped;
    }
    if (cfg & 0x2u) {
        mm_u32 bd = g.vbus_mv / 4u;
        if (bd > 0x1fffu) bd = 0x1fffu;
        g.bus = (mm_u16)bd;
    }
    cur = ((mm_i64)g.shunt * (mm_i64)g.calib) / 4096;
    if (cur > 32767) { cur = 32767; g.ovf = 1u; }
    if (cur < -32768) { cur = -32768; g.ovf = 1u; }
    g.current = (mm_i16)cur;
    pwr = ((cur < 0 ? -cur : cur) * (mm_i64)g.bus) / 5000;
    if (pwr > 65535) { pwr = 65535; g.ovf = 1u; }
    g.power = (mm_u16)pwr;
    g.cnvr = 1u;

    if (!g.conv_logged) {
        logf_head("CONV");
        fprintf(g.log, " cfg=0x%04x mode=%u(%s):", (unsigned)cfg, (unsigned)(cfg & 7u), mode_text(cfg));
        if (cfg & 0x1u) {
            fprintf(g.log, " vshunt_in=%lduV pga_fs=+-%lduV -> SHUNT=0x%04x%s", (long)g.vshunt_uv,
                    (long)fs * 10, (unsigned)(mm_u16)g.shunt, (clipped != raw) ? " (CLIPPED)" : "");
        }
        if (cfg & 0x2u) {
            fprintf(g.log, " vbus_in=%lumV -> BUS=0x%04x", (unsigned long)g.vbus_mv,
                    (unsigned)((g.bus << 3) | (g.cnvr << 1) | g.ovf));
        }
        fprintf(g.log, " CURRENT=0x%04x POWER=0x%04x (CAL=0x%04x)%s\n", (unsigned)(mm_u16)g.current,
                (unsigned)g.power, (unsigned)g.calib, g.ovf ? " OVF" : "");
        fflush(g.log);
        if (cfg & 0x4u) g.conv_logged = 1u; /* continuous: log the first one only */
    }
}

/* Bring the data registers up to date with the virtual clock. */
static mm_bool update(void)
{
    if (!g.conv_pending) return MM_FALSE;
    if (now_us() < g.conv_ready_us) return MM_TRUE; /* still converting */
    convert(g.conv_cfg);
    if ((g.conv_cfg & 0x4u) == 0u) g.conv_pending = 0u; /* triggered: one shot */
    return MM_FALSE;
}

static void log_config(mm_u16 v)
{
    mm_u32 pg = (v >> 11) & 3u;
    fprintf(g.log, "  BRNG=%u(%uV) PG=%u(/%u,%umV) BADC=0x%x(%s) SADC=0x%x(%s) MODE=%u(%s)",
            (unsigned)((v >> 13) & 1u), ((v >> 13) & 1u) ? 32u : 16u, (unsigned)pg, 1u << pg, 40u << pg,
            (unsigned)((v >> 7) & 0xfu), adc_text((v >> 7) & 0xfu), (unsigned)((v >> 3) & 0xfu),
            adc_text((v >> 3) & 0xfu), (unsigned)(v & 7u), mode_text(v));
}

static void reg_write(mm_u32 reg, mm_u16 val)
{
    g.n_wr++;
    if (reg == 0u) (void)update(); /* a conversion that has finished lands first */
    logf_head("WR");
    fprintf(g.log, " reg=0x%02x %-11s val=0x%04x", (unsigned)reg, reg_name(reg), (unsigned)val);
    if (reg == 0u) {
        g.n_cfg_wr++;
        g.last_cfg_wr = val;
        log_config(val);
        fprintf(g.log, "\n");
        if (val & 0x8000u) {
            logf_head("RST");
            fprintf(g.log, " power-on state restored\n");
            power_on();
        } else {
            mm_u32 mode = val & 7u;
            g.config = val & 0x7fffu;
            if (mode == 0u || mode == 4u) {
                g.conv_pending = 0u; /* nothing converts; data registers keep their values */
            } else {
                g.cnvr = 0u;
                g.conv_pending = 1u;
                g.conv_logged = 0u;
                g.conv_cfg = g.config;
                g.conv_ready_us = now_us() + conv_time_us(g.config);
            }
        }
    } else if (reg == 5u) {
        g.calib = val & 0xfffeu;
        fprintf(g.log, "\n");
    } else {
        fprintf(g.log, "  IGNORED (%s)\n", (reg < 5u) ? "read-only register" : "no such register");
    }
    fflush(g.log);
}

static mm_u16 reg_read(mm_u32 reg)
{
    switch (reg) {
    case 0u: return g.config;
    case 1u: return (mm_u16)g.shunt;
    case 2u: return (mm_u16)((g.bus << 3) | (g.cnvr << 1) | g.ovf);
    case 3u: {
        mm_u16 v = g.power;
        g.cnvr = 0u; /* datasheet: reading the Power register clears CNVR */
        return v;
    }
    case 4u: return (mm_u16)g.current;
    case 5u: return g.calib;
    default: break;
    }
    return 0u;
}

/* Close the bookkeeping of the transfer that a STOP or a new START ends. */
static void end_xfer(void)
{
    if (g.addressed && g.cur != g.addr) {
        if (g.rw) {
            if (g.rd_bytes != 0u) {
                g.n_xrd++;
                logf_head("XRD");
                fprintf(g.log, " addr=0x%02x reg=0x%02x val=0x%04x bytes=%lu\n", (unsigned)g.cur,
                        (unsigned)g.rd_reg, (unsigned)g.rd_val, (unsigned long)g.rd_bytes);
            }
        } else if (g.wr_count == 0u) {
            g.n_xprobe++;
            logf_head("XPRB");
            fprintf(g.log, " addr=0x%02x address ACKed, no data\n", (unsigned)g.cur);
        }
        fflush(g.log);
    } else if (g.addressed) {
        if (g.rw) {
            if (g.rd_bytes != 0u) {
                g.n_rd++;
                logf_head("RD");
                fprintf(g.log, " reg=0x%02x %-11s val=0x%04x bytes=%lu%s%s\n", (unsigned)g.rd_reg,
                        reg_name(g.rd_reg), (unsigned)g.rd_val, (unsigned long)g.rd_bytes,
                        (g.rd_reg > 5u) ? "  (no such register)" : "",
                        g.rd_stale ? "  (conversion still running: previous result)" : "");
            }
        } else if (g.wr_count == 0u) {
            logf_head("PROBE");
            fprintf(g.log, " address ACKed, no data\n");
        } else if (g.wr_count == 1u) {
            logf_head("PTR");
            fprintf(g.log, " reg=0x%02x %s\n", (unsigned)g.ptr, reg_name(g.ptr));
        } else if (g.wr_count == 2u) {
            logf_head("WR?");
            fprintf(g.log, " reg=0x%02x %s: only one data byte (0x%02x), nothing written\n",
                    (unsigned)g.ptr, reg_name(g.ptr), (unsigned)g.wr_hi);
        } else if (g.wr_count > 3u) {
            logf_head("WR+");
            fprintf(g.log, " %lu extra byte(s) after the register value ignored\n",
                    (unsigned long)(g.wr_count - 3u));
        }
        fflush(g.log);
    }
    g.addressed = 0u;
    g.wr_count = 0u;
    g.rd_bytes = 0u;
    g.rd_stale = 0u;
}

static void rx_byte(mm_u8 b)
{
    if (g.cur != g.addr) {
        if (g.wr_count == 0u) g_xptr[g.cur] = b;
        else if (g.wr_count == 1u) g.wr_hi = b;
        else if (g.wr_count == 2u) {
            mm_u16 v = (mm_u16)(((mm_u16)g.wr_hi << 8) | b);
            g_xreg[g.cur][g_xptr[g.cur]] = v;
            g.n_xwr++;
            logf_head("XWR");
            fprintf(g.log, " addr=0x%02x reg=0x%02x val=0x%04x\n", (unsigned)g.cur, (unsigned)g_xptr[g.cur],
                    (unsigned)v);
        }
        g.wr_count++;
        return;
    }
    if (g.wr_count == 0u) g.ptr = b;
    else if (g.wr_count == 1u) g.wr_hi = b;
    else if (g.wr_count == 2u) reg_write(g.ptr, (mm_u16)(((mm_u16)g.wr_hi << 8) | b));
    g.wr_count++;
}

static void load_tx(void)
{
    if (g.cur != g.addr) {
        if (g.rd_bytes == 0u) {
            g.rd_reg = g_xptr[g.cur];
            g.rd_val = g_xreg[g.cur][g.rd_reg];
        }
        g.txbyte = (g.rd_bytes & 1u) ? (mm_u8)(g.rd_val & 0xffu) : (mm_u8)(g.rd_val >> 8);
        g.rd_bytes++;
        return;
    }
    if (g.rd_bytes == 0u) {
        mm_bool busy = update();
        g.rd_reg = g.ptr;
        g.rd_stale = (busy && g.ptr >= 1u && g.ptr <= 4u) ? 1u : 0u;
        if (g.rd_stale) g.n_stale++;
        g.rd_val = reg_read(g.ptr);
    }
    g.txbyte = (g.rd_bytes & 1u) ? (mm_u8)(g.rd_val & 0xffu) : (mm_u8)(g.rd_val >> 8);
    g.rd_bytes++;
}

static void drive_bit(mm_u32 bit)
{
    g.sda_low = bit ? 0u : 1u;
}

static void on_scl_rise(mm_u32 sda)
{
    switch (g.st) {
    case ST_ADDR:
    case ST_RX:
        g.shift = (mm_u8)((g.shift << 1) | (sda & 1u));
        if (g.nbits < 8u) g.nbits++;
        break;
    case ST_TX_ACK:
        g.master_ack = sda ? 0u : 1u;
        break;
    default:
        break;
    }
}

static void on_scl_fall(void)
{
    switch (g.st) {
    case ST_ADDR:
        if (g.nbits == 8u) {
            if ((mm_u32)(g.shift >> 1) == g.addr || g.extra[g.shift >> 1]) {
                g.cur = (mm_u8)(g.shift >> 1);
                if (g.cur == g.addr) g.n_match++;
                g.addressed = 1u;
                g.rw = g.shift & 1u;
                g.sda_low = 1u; /* ACK */
                g.st = ST_ACK_ADDR;
            } else {
                g.n_other++;
                g.st = ST_DONE;
            }
        }
        break;
    case ST_ACK_ADDR:
        if (g.rw) {
            load_tx();
            drive_bit((g.txbyte >> 7) & 1u);
            g.nbits = 1u;
            g.st = ST_TX;
        } else {
            g.sda_low = 0u;
            g.nbits = 0u;
            g.shift = 0u;
            g.st = ST_RX;
        }
        break;
    case ST_RX:
        if (g.nbits == 8u) {
            rx_byte(g.shift);
            g.sda_low = 1u; /* ACK */
            g.st = ST_ACK_RX;
        }
        break;
    case ST_ACK_RX:
        g.sda_low = 0u;
        g.nbits = 0u;
        g.shift = 0u;
        g.st = ST_RX;
        break;
    case ST_TX:
        if (g.nbits < 8u) {
            drive_bit((g.txbyte >> (7u - g.nbits)) & 1u);
            g.nbits++;
        } else {
            g.sda_low = 0u; /* release for the master's ACK/NAK */
            g.master_ack = 0u;
            g.st = ST_TX_ACK;
        }
        break;
    case ST_TX_ACK:
        if (g.master_ack) {
            load_tx();
            drive_bit((g.txbyte >> 7) & 1u);
            g.nbits = 1u;
            g.st = ST_TX;
        } else {
            g.st = ST_DONE; /* NAK: wait for STOP or a new START */
        }
        break;
    default:
        break;
    }
}

mm_u64 mm_rp2350_ina219_pads(mm_u64 levels)
{
    mm_u8 scl;
    mm_u8 sda;

    if (g.enabled <= 0) return 0u;
    scl = (mm_u8)((levels >> g.scl_pin) & 1u);
    sda = (mm_u8)((levels >> g.sda_pin) & 1u);
    if (scl != g.scl_prev) {
        if (scl) on_scl_rise(sda);
        else on_scl_fall();
    } else if (scl && sda != g.sda_prev) {
        end_xfer();
        g.sda_low = 0u;
        if (!sda) { /* START or repeated START */
            g.n_start++;
            g.nbits = 0u;
            g.shift = 0u;
            g.st = ST_ADDR;
        } else {    /* STOP */
            g.st = ST_IDLE;
        }
    }
    g.scl_prev = scl;
    g.sda_prev = sda;
    return g.sda_low ? (1ull << g.sda_pin) : 0u;
}

void mm_rp2350_ina219_stats(void)
{
    if (g.enabled <= 0) return;
    fprintf(stderr, "[INA219] starts=%llu addressed=%llu other_addr=%llu reg_writes=%llu (config=%llu, last=0x%04x) "
                    "reg_reads=%llu early_reads=%llu\n",
            (unsigned long long)g.n_start, (unsigned long long)g.n_match, (unsigned long long)g.n_other,
            (unsigned long long)g.n_wr, (unsigned long long)g.n_cfg_wr, (unsigned)g.last_cfg_wr,
            (unsigned long long)g.n_rd, (unsigned long long)g.n_stale);
    fprintf(stderr, "[INA219] extra register files: probes=%llu reg_writes=%llu reg_reads=%llu\n",
            (unsigned long long)g.n_xprobe, (unsigned long long)g.n_xwr, (unsigned long long)g.n_xrd);
    if (g.log != 0 && g.log != stdout) fflush(g.log);
}
