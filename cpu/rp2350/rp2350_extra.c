/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * Additions for running the Flipper One MCU firmware on the RP2350
 * profile:
 *   - register-file stubs for peripherals the profile did not map
 *     (BUSCTRL, ADC, PWM, ROSC, SHA256, POWMAN, OTP data, CoreSight, DFT,
 *     GLITCH_DETECTOR, TBMAN, CORESIGHT_TRACE); first access is logged.
 *   - a DW_apb_i2c model for I2C0/I2C1 with an empty bus: every master
 *     transfer aborts with ABRT_7B_ADDR_NOACK, like a bare Pico 2.
 *   - XIP aliases 0x14000000/0x1c000000 (no-cache / no-translate) that
 *     read the same flash array.
 *   - partition table: M33MU_RP2350_PT=<picotool partition .uf2|.bin>
 *     is parsed into the bootrom's partition table, so
 *     rom_get_partition_table_info() / rom_get_boot_info() answer like the
 *     real bootrom.  The image itself stays at flash offset 0 (no ATRANS).
 *   - SEGGER RTT: M33MU_RTT=<file|-> drains up-buffer 0 to a file/stdout.
 *   - optional, off by default: a master engine for a DW_apb_i2c instance
 *     that has a slave model attached (PCAL6416A, M33MU_PCAL6416=1), slave
 *     mode for I2C1 with a scripted external master (M33MU_I2C1_MASTER), and
 *     measurement probes (M33MU_PROBE_LOG / _FN / _DMA_PIO).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rp2350/rp2350_mmio.h"
#include "rp2350/rp2350_extra.h"
#include "rp2350/rp2350_pcal6416.h"
#include "rp2350/rp2350_i2c1_master.h"
#include "rp2350/rp2350_timers.h"
#include "m33mu/mmio.h"
#include "m33mu/memmap.h"
#include "m33mu/cpu.h"
#include "m33mu/nvic.h"

/* ------------------------------------------------------------------ */
/* generic register-file stubs                                         */
/* ------------------------------------------------------------------ */

#define STUB_REGS 0x400u /* 4 KiB of registers per block */

struct stub_block {
    mm_u32 base;
    mm_u32 size;
    const char *name;
    mm_u32 regs[STUB_REGS];
    mm_bool logged;
    mm_u32 kind;
};

enum { STUB_PLAIN = 0, STUB_ADC = 1, STUB_POWMAN = 2, STUB_ROSC = 3 };

static struct stub_block g_stubs[] = {
    {0x40068000u, 0x4000u, "BUSCTRL", {0}, MM_FALSE, STUB_PLAIN},
    {0x400a0000u, 0x4000u, "ADC", {0}, MM_FALSE, STUB_ADC},
    {0x400a8000u, 0x4000u, "PWM", {0}, MM_FALSE, STUB_PLAIN},
    {0x400e8000u, 0x4000u, "ROSC", {0}, MM_FALSE, STUB_ROSC},
    {0x400f8000u, 0x4000u, "SHA256", {0}, MM_FALSE, STUB_PLAIN},
    {0x40100000u, 0x4000u, "POWMAN", {0}, MM_FALSE, STUB_POWMAN},
    {0x40130000u, 0x10000u, "OTP_DATA", {0}, MM_FALSE, STUB_PLAIN},
    {0x40140000u, 0x10000u, "CORESIGHT", {0}, MM_FALSE, STUB_PLAIN},
    {0x40150000u, 0x4000u, "DFT", {0}, MM_FALSE, STUB_PLAIN},
    {0x40158000u, 0x4000u, "GLITCH_DET", {0}, MM_FALSE, STUB_PLAIN},
    {0x40160000u, 0x4000u, "TBMAN", {0}, MM_FALSE, STUB_PLAIN},
    {0x50700000u, 0x4000u, "CORESIGHT_TRACE", {0}, MM_FALSE, STUB_PLAIN},
};

static int g_stub_trace = -1;

static mm_bool stub_trace(void)
{
    if (g_stub_trace < 0) {
        const char *v = getenv("M33MU_STUB_TRACE");
        g_stub_trace = (v != 0 && v[0] != '\0') ? 1 : 0;
    }
    return g_stub_trace ? MM_TRUE : MM_FALSE;
}

static void stub_log(struct stub_block *b, const char *op, mm_u32 offset, mm_u32 value)
{
    if (!b->logged || stub_trace()) {
        printf("[STUB] %s %s +0x%04x (0x%08x) val=0x%08x%s\n", b->name, op,
               offset, b->base + offset, value, b->logged ? "" : " (first access)");
        b->logged = MM_TRUE;
    }
}

static mm_u32 stub_index(const struct stub_block *b, mm_u32 offset, mm_u32 *alias)
{
    mm_u32 base_off = offset;
    *alias = 0u;
    if (b->size == 0x4000u) {
        *alias = (offset >> 12) & 3u;
        base_off = offset & 0xfffu;
    }
    return (base_off / 4u) % STUB_REGS;
}

static mm_bool stub_read(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 *value_out)
{
    struct stub_block *b = (struct stub_block *)opaque;
    mm_u32 alias;
    mm_u32 idx;
    mm_u32 v;
    if (b == 0 || value_out == 0) return MM_FALSE;
    idx = stub_index(b, offset, &alias);
    v = b->regs[idx];
    if (b->kind == STUB_ADC) {
        if (idx == 0u) v |= (1u << 8);          /* CS.READY */
        if (idx == 1u) v = 0x7ffu;              /* RESULT: mid-scale */
        if (idx == 2u) v = (v & ~0x000f0f00u) | (1u << 8); /* FCS: EMPTY, LEVEL 0 */
        if (idx == 3u) v = 0x7ffu;              /* FIFO */
    } else if (b->kind == STUB_ROSC) {
        if (idx == 0x18u / 4u) v |= (1u << 31) | (1u << 12); /* STATUS.STABLE|ENABLED */
        if (idx == 0x1cu / 4u) v = (v * 1103515245u + 12345u) & 1u; /* RANDOMBIT */
    }
    if (!mmio_peek_mode()) stub_log(b, "rd", offset, v);
    v >>= (offset & 3u) * 8u;
    if (size_bytes == 1u) v &= 0xffu;
    else if (size_bytes == 2u) v &= 0xffffu;
    *value_out = v;
    return MM_TRUE;
}

static mm_bool stub_write(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 value)
{
    struct stub_block *b = (struct stub_block *)opaque;
    mm_u32 alias;
    mm_u32 idx;
    if (b == 0) return MM_FALSE;
    if (size_bytes == 1u) value = (value & 0xffu) * 0x01010101u;
    else if (size_bytes == 2u) value = (value & 0xffffu) * 0x00010001u;
    idx = stub_index(b, offset, &alias);
    switch (alias) {
    case 1u: b->regs[idx] ^= value; break;
    case 2u: b->regs[idx] |= value; break;
    case 3u: b->regs[idx] &= ~value; break;
    default: b->regs[idx] = value; break;
    }
    stub_log(b, "wr", offset, value);
    return MM_TRUE;
}

/* ------------------------------------------------------------------ */
/* DW_apb_i2c, empty bus                                               */
/* ------------------------------------------------------------------ */

#define IC_CON            0x00u
#define IC_TAR            0x04u
#define IC_DATA_CMD       0x10u
#define IC_INTR_STAT      0x2cu
#define IC_INTR_MASK      0x30u
#define IC_RAW_INTR_STAT  0x34u
#define IC_CLR_INTR       0x40u
#define IC_CLR_TX_ABRT    0x54u
#define IC_CLR_STOP_DET   0x60u
#define IC_CLR_START_DET  0x64u
#define IC_ENABLE         0x6cu
#define IC_STATUS         0x70u
#define IC_TXFLR          0x74u
#define IC_RXFLR          0x78u
#define IC_TX_ABRT_SOURCE 0x80u
#define IC_ENABLE_STATUS  0x9cu
#define IC_COMP_PARAM_1   0xf4u
#define IC_COMP_VERSION   0xf8u
#define IC_COMP_TYPE      0xfcu

#define RAW_TX_EMPTY  (1u << 4)
#define RAW_TX_ABRT   (1u << 6)
#define RAW_STOP_DET  (1u << 9)
#define RAW_START_DET (1u << 10)

struct dw_i2c {
    const char *name;
    mm_u32 regs[0x100u / 4u];
    mm_u32 raw;
    mm_u32 abrt_source;
    mm_bool logged;
    mm_u64 nacks;
    /* master engine, only used while a slave model is attached (dev) */
    mm_bool dev;
    mm_u32 reset_bit;    /* RESETS bit of this block */
    mm_u16 txf[16];
    mm_u32 tx_head, tx_n;
    mm_u8 rxf[16];
    mm_u32 rx_head, rx_n;
    mm_u8 phase;
    mm_u64 phase_end;
    mm_u16 cur;          /* command being shifted */
    mm_bool in_xfer;     /* START sent, STOP not yet: the master holds the bus */
    mm_bool xfer_read;
    mm_u64 n_start, n_tx, n_rx, n_stop, n_cut;
    /* slave mode with an external master (slv), I2C1 only */
    mm_bool slv;
    mm_u8 s_txf[16];
    mm_u32 s_tx_head, s_tx_n;
    mm_u16 s_rxf[16];        /* data byte | FIRST_DATA_BYTE */
    mm_u32 s_rx_head, s_rx_n;
    mm_u8 s_state;           /* SS_* */
    mm_bool s_repeated;      /* the START being decoded is a repeated START */
    mm_bool s_addressed;     /* our address was seen since the last START */
    mm_bool s_first;         /* the next byte received is the first after the address */
    mm_bool s_rx_data;       /* a data byte was received in this write */
    mm_bool s_rd_req;        /* RD_REQ raised for the byte the master waits for */
    mm_u32 s_en_status;      /* IC_ENABLE_STATUS bits 2:1, latched at IC_ENABLE 1 -> 0 */
    mm_u32 s_flush_cnt;      /* IC_TX_ABRT_SOURCE.TX_FLUSH_CNT */
    mm_u64 s_n_start, s_n_match, s_n_other, s_n_rx, s_n_tx, s_n_rd_req, s_n_rx_done, s_n_stop;
    mm_u64 s_n_irq, s_n_abrt, s_n_rx_over, s_n_tx_over, s_n_rx_under, s_n_locked;
};

static struct dw_i2c g_i2c[2] = {{.name = "I2C0"}, {.name = "I2C1"}};

/*
 * master engine for a bus with a slave model on it.
 *
 * Without a model (the default) the code further down is used unchanged:
 * every transfer aborts at once with ABRT_7B_ADDR_NOACK.  With a model the
 * instance becomes what pico-sdk's i2c_write_blocking / i2c_read_blocking
 * need from a DW_apb_i2c master:
 *   IC_DATA_CMD   16-entry TX FIFO of data + CMD/STOP/RESTART, 16-entry RX FIFO
 *   bus           START + address (10 bit times), one byte + ACK (9), STOP (1);
 *                 a bit time is IC_xS_SCL_HCNT + IC_xS_SCL_LCNT system clock
 *                 cycles.  A command without STOP leaves the bus held; the
 *                 next one continues, or starts over with a repeated START
 *                 when it has RESTART set or changes direction.
 *   interrupts    raw status only: TX_EMPTY (with IC_CON.TX_EMPTY_CTRL: FIFO
 *                 at or below IC_TX_TL and the last byte shifted out),
 *                 RX_FULL, TX_ABRT with IC_TX_ABRT_SOURCE (7B_ADDR_NOACK,
 *                 TXDATA_NOACK), STOP_DET, START_DET, RX_UNDER/RX_OVER/
 *                 TX_OVER, and their clear-on-read registers.  No IRQ line.
 *   IC_ENABLE     0 flushes both FIFOs; the bus stays held if it was (the SDK
 *                 clears and sets it around IC_TAR between the two halves of
 *                 a write-then-read)
 *   RESETS        asserting the block reset clears the register file (zeros,
 *                 not the documented reset values: i2c_init() writes every
 *                 register it relies on) and all state; registers ignore
 *                 writes while the reset is held
 * Not there: slave mode, 10-bit and general-call addressing, high speed,
 * SDA hold / spike / set-up counts, clock stretching and SCL low time-outs,
 * arbitration, the IRQ and DMA request lines, IC_ENABLE.ABORT, the exact
 * SCL period formula (the real block adds a few cycles per phase).
 */
#define IC_SS_SCL_HCNT 0x14u
#define IC_SS_SCL_LCNT 0x18u
#define IC_FS_SCL_HCNT 0x1cu
#define IC_FS_SCL_LCNT 0x20u
#define IC_RX_TL       0x38u
#define IC_TX_TL       0x3cu
#define IC_CLR_RX_UNDER 0x44u
#define IC_CLR_RX_OVER  0x48u
#define IC_CLR_TX_OVER  0x4cu

#define RAW_RX_UNDER (1u << 0)
#define RAW_RX_OVER  (1u << 1)
#define RAW_RX_FULL  (1u << 2)
#define RAW_TX_OVER  (1u << 3)

#define CMD_READ    (1u << 8)
#define CMD_STOP    (1u << 9)
#define CMD_RESTART (1u << 10)

enum { PH_IDLE = 0, PH_ADDR, PH_DATA, PH_STOP };

static mm_bool g_i2c_dev_any = MM_FALSE;

static mm_u64 i2c_bit_cycles(const struct dw_i2c *c)
{
    mm_bool std = (((c->regs[IC_CON / 4u] >> 1) & 3u) == 1u) ? MM_TRUE : MM_FALSE;
    mm_u64 bit = (mm_u64)(c->regs[(std ? IC_SS_SCL_HCNT : IC_FS_SCL_HCNT) / 4u] & 0xffffu) +
                 (mm_u64)(c->regs[(std ? IC_SS_SCL_LCNT : IC_FS_SCL_LCNT) / 4u] & 0xffffu);
    return (bit < 16u) ? 16u : bit;
}

static void i2c_eng_abort(struct dw_i2c *c, mm_u32 source, mm_u64 t)
{
    c->abrt_source |= source;
    c->raw |= RAW_TX_ABRT;
    c->tx_n = 0u; /* the TX FIFO is flushed and stays so until CLR_TX_ABRT */
    c->phase = PH_STOP; /* the master issues a STOP on an abort */
    c->phase_end = t + i2c_bit_cycles(c);
}

/* Advance the bus up to the current time. */
static void i2c_eng_run(struct dw_i2c *c)
{
    mm_u64 now = mm_rp2350_now_cycles();
    mm_u64 t = now; /* when an idle master takes the next command */

    for (;;) {
        mm_u64 bit = i2c_bit_cycles(c);
        mm_bool rd = (c->cur & CMD_READ) ? MM_TRUE : MM_FALSE;
        if (c->phase == PH_IDLE) {
            if (c->tx_n == 0u || (c->regs[IC_ENABLE / 4u] & 1u) == 0u || (c->regs[IC_CON / 4u] & 1u) == 0u) return;
            c->cur = c->txf[c->tx_head];
            c->tx_head = (c->tx_head + 1u) & 15u;
            c->tx_n--;
            rd = (c->cur & CMD_READ) ? MM_TRUE : MM_FALSE;
            if (!c->in_xfer || (c->cur & CMD_RESTART) != 0u || rd != c->xfer_read) {
                c->phase = PH_ADDR;
                c->phase_end = t + 10u * bit;
            } else {
                c->phase = PH_DATA;
                c->phase_end = t + 9u * bit;
            }
            continue;
        }
        if (now < c->phase_end) return;
        t = c->phase_end;
        if (c->phase == PH_ADDR) {
            mm_bool ack = mm_rp2350_pcal6416_start(t, c->regs[IC_TAR / 4u] & 0x7fu, rd);
            c->n_start++;
            c->raw |= RAW_START_DET;
            c->in_xfer = MM_TRUE;
            c->xfer_read = rd;
            if (!ack) {
                c->nacks++;
                i2c_eng_abort(c, 1u << 0, t); /* ABRT_7B_ADDR_NOACK */
            } else {
                c->phase = PH_DATA;
                c->phase_end = t + 9u * bit;
            }
        } else if (c->phase == PH_DATA) {
            if (rd) {
                mm_u8 b = mm_rp2350_pcal6416_read(t, (c->cur & CMD_STOP) ? MM_FALSE : MM_TRUE);
                c->n_rx++;
                if (c->rx_n < 16u) {
                    c->rxf[(c->rx_head + c->rx_n) & 15u] = b;
                    c->rx_n++;
                } else {
                    c->raw |= RAW_RX_OVER;
                }
            } else {
                c->n_tx++;
                if (!mm_rp2350_pcal6416_write(t, (mm_u8)(c->cur & 0xffu))) {
                    i2c_eng_abort(c, 1u << 3, t); /* ABRT_TXDATA_NOACK */
                    continue;
                }
            }
            if ((c->cur & CMD_STOP) != 0u) {
                c->phase = PH_STOP;
                c->phase_end = t + bit;
            } else {
                c->phase = PH_IDLE;
            }
        } else { /* PH_STOP */
            mm_rp2350_pcal6416_stop(t);
            c->n_stop++;
            c->in_xfer = MM_FALSE;
            c->raw |= RAW_STOP_DET;
            c->phase = PH_IDLE;
        }
    }
}

/* The block is disabled or reset: nothing more is shifted. */
static void i2c_eng_halt(struct dw_i2c *c, mm_bool release_bus)
{
    mm_u64 now = mm_rp2350_now_cycles();
    i2c_eng_run(c);
    if (c->phase == PH_STOP) {
        /* the STOP of a finished transfer still goes out */
        mm_rp2350_pcal6416_stop(now);
        c->n_stop++;
        c->in_xfer = MM_FALSE;
        c->raw |= RAW_STOP_DET;
    } else if (c->phase != PH_IDLE || (release_bus && c->in_xfer)) {
        /* cut off inside a byte, or reset while holding the bus */
        c->n_cut++;
        printf("[I2C] %s: %s with a transfer in progress (phase %u), bus released\n", c->name,
               release_bus ? "block reset" : "disabled", (unsigned)c->phase);
        mm_rp2350_pcal6416_stop(now);
        c->in_xfer = MM_FALSE;
    }
    c->phase = PH_IDLE;
    c->tx_n = 0u;
    c->tx_head = 0u;
    c->rx_n = 0u;
    c->rx_head = 0u;
}

static mm_u32 i2c_eng_raw(const struct dw_i2c *c)
{
    mm_u32 raw = c->raw & ~(RAW_TX_EMPTY | RAW_RX_FULL);
    mm_bool shifting = (c->phase == PH_ADDR || c->phase == PH_DATA) ? MM_TRUE : MM_FALSE;
    if (c->tx_n <= (c->regs[IC_TX_TL / 4u] & 0xffu) &&
        ((c->regs[IC_CON / 4u] & (1u << 8)) == 0u || !shifting)) {
        raw |= RAW_TX_EMPTY;
    }
    if (c->rx_n > (c->regs[IC_RX_TL / 4u] & 0xffu)) raw |= RAW_RX_FULL;
    return raw;
}

static mm_u32 i2c_eng_reg_read(struct dw_i2c *c, mm_u32 off)
{
    mm_bool peek = mmio_peek_mode();
    if (mm_rp2350_reset_asserted(c->reset_bit)) return 0u;
    i2c_eng_run(c);
    switch (off) {
    case IC_RAW_INTR_STAT: return i2c_eng_raw(c);
    case IC_INTR_STAT: return i2c_eng_raw(c) & c->regs[IC_INTR_MASK / 4u];
    case IC_TX_ABRT_SOURCE: return c->abrt_source;
    case IC_CLR_INTR:
        if (!peek) { c->raw = 0u; c->abrt_source = 0u; }
        return 0u;
    case IC_CLR_RX_UNDER:
        if (!peek) c->raw &= ~RAW_RX_UNDER;
        return 0u;
    case IC_CLR_RX_OVER:
        if (!peek) c->raw &= ~RAW_RX_OVER;
        return 0u;
    case IC_CLR_TX_OVER:
        if (!peek) c->raw &= ~RAW_TX_OVER;
        return 0u;
    case IC_CLR_TX_ABRT:
        if (!peek) { c->raw &= ~RAW_TX_ABRT; c->abrt_source = 0u; }
        return 0u;
    case IC_CLR_STOP_DET:
        if (!peek) c->raw &= ~RAW_STOP_DET;
        return 0u;
    case IC_CLR_START_DET:
        if (!peek) c->raw &= ~RAW_START_DET;
        return 0u;
    case IC_STATUS:
        return ((c->phase != PH_IDLE) ? ((1u << 0) | (1u << 5)) : 0u) | /* ACTIVITY | MST_ACTIVITY */
               ((c->tx_n < 16u) ? (1u << 1) : 0u) |                      /* TFNF */
               ((c->tx_n == 0u) ? (1u << 2) : 0u) |                      /* TFE */
               ((c->rx_n != 0u) ? (1u << 3) : 0u) |                      /* RFNE */
               ((c->rx_n == 16u) ? (1u << 4) : 0u);                      /* RFF */
    case IC_TXFLR: return c->tx_n;
    case IC_RXFLR: return c->rx_n;
    case IC_DATA_CMD: {
        mm_u32 v;
        if (c->rx_n == 0u) {
            if (!peek) c->raw |= RAW_RX_UNDER;
            return 0u;
        }
        v = c->rxf[c->rx_head];
        if (!peek) {
            c->rx_head = (c->rx_head + 1u) & 15u;
            c->rx_n--;
        }
        return v;
    }
    case IC_ENABLE_STATUS: return c->regs[IC_ENABLE / 4u] & 1u;
    case IC_COMP_PARAM_1: return 0x000f0fefu; /* 16-deep FIFOs */
    case IC_COMP_VERSION: return 0x3230312au;
    case IC_COMP_TYPE: return 0x44570140u;
    default:
        if (off >= 0x40u && off <= 0x68u) return 0u; /* other clear-on-read */
        return c->regs[(off & 0xffu) / 4u];
    }
}

static void i2c_eng_reg_write(struct dw_i2c *c, mm_u32 off, mm_u32 v)
{
    if (mm_rp2350_reset_asserted(c->reset_bit)) return;
    i2c_eng_run(c);
    if (off == IC_DATA_CMD) {
        if ((c->regs[IC_ENABLE / 4u] & 1u) == 0u) return;
        if ((c->raw & RAW_TX_ABRT) != 0u) return; /* FIFO held flushed until CLR_TX_ABRT */
        if (c->tx_n >= 16u) {
            c->raw |= RAW_TX_OVER;
            return;
        }
        c->txf[(c->tx_head + c->tx_n) & 15u] = (mm_u16)(v & 0x7ffu);
        c->tx_n++;
        i2c_eng_run(c);
        return;
    }
    if (off == IC_ENABLE) {
        mm_u32 old = c->regs[IC_ENABLE / 4u];
        c->regs[IC_ENABLE / 4u] = v;
        if ((old & 1u) != 0u && (v & 1u) == 0u) i2c_eng_halt(c, MM_FALSE);
        return;
    }
    if (off < 0x100u) c->regs[off / 4u] = v;
}

/*
 * slave mode of I2C1, with an external master on the bus.
 *
 * Off unless M33MU_I2C1_MASTER is set (rp2350_i2c1_master.c, a scripted
 * master in the role of the RK3576 CPU).  Without it I2C1 is the empty-bus
 * model further down, unchanged.  With it I2C1 is what a DW_apb_i2c slave
 * (IC_CON.MASTER_MODE = 0, IC_SLAVE_DISABLE = 0, IC_ENABLE = 1) shows its
 * firmware, after the RP2350 datasheet's I2C chapter and the register
 * descriptions in pico-sdk's hardware/regs/i2c.h.  Byte level: the master
 * reports START / repeated START, the address byte, each data byte and STOP
 * at their emulated times (rp2350_extra.h, mm_rp2350_i2c1_bus_*).
 *   address    7-bit compare with IC_SAR[6:0]; a match is ACKed, any other
 *              address NACKed
 *   receive    each byte the master writes is ACKed and goes into the 16-entry
 *              RX FIFO (IC_DATA_CMD read, bit 11 = FIRST_DATA_BYTE).  RX FIFO
 *              full: with IC_CON.RX_FIFO_FULL_HLD_CTRL SCL is held (clock
 *              stretching) until there is room, without it the byte is ACKed
 *              and lost (RX_OVER).  IC_SLV_DATA_NACK_ONLY = 1 NACKs data.
 *   transmit   a read address flushes whatever is left in the TX FIFO
 *              (TX_ABRT, ABRT_SLVFLUSH_TXFIFO, TX_FLUSH_CNT; the TX FIFO takes
 *              no data until IC_CLR_TX_ABRT is read).  For each byte the
 *              master wants while the TX FIFO is empty RD_REQ is raised and
 *              SCL held until IC_DATA_CMD is written; bytes already in the
 *              FIFO go out without RD_REQ.  The master's NACK sets RX_DONE.
 *              CMD = 1 written in slave mode: TX_ABRT, ABRT_SLVRD_INTX.
 *   interrupts RX_UNDER, RX_OVER, RX_FULL (level: IC_RXFLR > IC_RX_TL),
 *              TX_OVER, TX_EMPTY (level: IC_TXFLR <= IC_TX_TL; TX_EMPTY_CTRL
 *              is not looked at), RD_REQ, TX_ABRT, RX_DONE, ACTIVITY,
 *              STOP_DET (honours STOP_DET_IFADDRESSED), START_DET (START and
 *              repeated START, any address), RESTART_DET (repeated START
 *              followed by our address); their IC_CLR_* read-to-clear
 *              registers and IC_CLR_INTR.  IC_INTR_STAT = raw & IC_INTR_MASK;
 *              its OR is the level of I2C1_IRQ (37) to the NVIC.
 *   registers  datasheet reset values, also after every RESETS cycle;
 *              IC_CON, IC_SAR, the SCL counts, IC_FS_SPKLEN and
 *              IC_SLV_DATA_NACK_ONLY ignore writes while IC_ENABLE = 1;
 *              IC_RX_TL / IC_TX_TL saturate at 15 (16 entries)
 *   IC_ENABLE  1 -> 0 flushes both FIFOs and clears the interrupt status at
 *              once; a transfer in progress is dropped (further data bytes
 *              NACKed, reads find SDA released = 0xff) and IC_ENABLE_STATUS
 *              reports SLV_DISABLED_WHILE_BUSY / SLV_RX_DATA_LOST
 * Not there: master mode on this instance (behaves like the empty bus),
 * 10-bit addressing, general call (address 0 is NACKed), START BYTE, SMBus,
 * spike filter, set-up/hold times, DMA requests, arbitration, IC_ENABLE.ABORT
 * and TX_CMD_BLOCK, the delay with which the real block goes idle after
 * IC_ENABLE = 0, the second byte lane of IC_DATA_CMD (a byte read at 0x11
 * pops the FIFO again).
 */
#define IC_SAR                0x08u
#define IC_CLR_RD_REQ         0x50u
#define IC_CLR_RX_DONE        0x58u
#define IC_CLR_ACTIVITY       0x5cu
#define IC_CLR_GEN_CALL       0x68u
#define IC_SDA_HOLD           0x7cu
#define IC_SLV_DATA_NACK_ONLY 0x84u
#define IC_SDA_SETUP          0x94u
#define IC_ACK_GENERAL_CALL   0x98u
#define IC_FS_SPKLEN          0xa0u
#define IC_CLR_RESTART_DET    0xa8u

#define RAW_RD_REQ      (1u << 5)
#define RAW_RX_DONE     (1u << 7)
#define RAW_ACTIVITY    (1u << 8)
#define RAW_GEN_CALL    (1u << 11)
#define RAW_RESTART_DET (1u << 12)

#define CON_MASTER_MODE     (1u << 0)
#define CON_10BITADDR_SLAVE (1u << 3)
#define CON_SLAVE_DISABLE   (1u << 6)
#define CON_STOP_DET_IFADDR (1u << 7)
#define CON_RX_FIFO_FULL_HLD (1u << 9)

#define ABRT_SLVFLUSH_TXFIFO (1u << 13)
#define ABRT_SLVRD_INTX      (1u << 15)
#define DATA_FIRST_BYTE      (1u << 11)

#define I2C1_IRQ 37u

enum { SS_IDLE = 0, /* bus free, or the block took no part in this transfer */
       SS_ADDR,     /* START seen, address byte expected */
       SS_OTHER,    /* another address: waiting for STOP or repeated START */
       SS_RX,       /* addressed, master writes */
       SS_TX,       /* addressed, master reads */
       SS_TX_DONE,  /* the master NACKed the last byte */
       SS_OFF };    /* disabled or reset during the transfer: out until STOP */

static mm_bool g_i2c_slv_on = MM_FALSE;
static struct mm_nvic *g_extra_nvic = 0;

#define SLV_TRACE(...) do { if (mm_rp2350_i2c1_master_tracing()) mm_rp2350_i2c1_master_trace(__VA_ARGS__); } while (0)

static mm_bool slv_active(const struct dw_i2c *c)
{
    if (mm_rp2350_reset_asserted(c->reset_bit)) return MM_FALSE;
    if ((c->regs[IC_ENABLE / 4u] & 1u) == 0u) return MM_FALSE;
    return ((c->regs[IC_CON / 4u] & (CON_MASTER_MODE | CON_SLAVE_DISABLE)) == 0u) ? MM_TRUE : MM_FALSE;
}

static mm_bool slv_busy(const struct dw_i2c *c)
{
    return (c->s_state == SS_ADDR || c->s_state == SS_RX || c->s_state == SS_TX || c->s_state == SS_TX_DONE)
               ? MM_TRUE : MM_FALSE;
}

static mm_u32 slv_raw(const struct dw_i2c *c)
{
    mm_u32 raw = c->raw & ~(RAW_RX_FULL | RAW_TX_EMPTY);
    if ((c->regs[IC_ENABLE / 4u] & 1u) == 0u) return raw;
    if (c->s_rx_n > c->regs[IC_RX_TL / 4u]) raw |= RAW_RX_FULL;
    if (c->s_tx_n <= c->regs[IC_TX_TL / 4u]) raw |= RAW_TX_EMPTY;
    return raw;
}

static void slv_flush(struct dw_i2c *c)
{
    c->s_tx_head = c->s_tx_n = 0u;
    c->s_rx_head = c->s_rx_n = 0u;
}

static void slv_regs_reset(struct dw_i2c *c)
{
    memset(c->regs, 0, sizeof(c->regs));
    c->regs[IC_CON / 4u] = 0x65u;
    c->regs[IC_TAR / 4u] = 0x55u;
    c->regs[IC_SAR / 4u] = 0x55u;
    c->regs[IC_SS_SCL_HCNT / 4u] = 0x28u;
    c->regs[IC_SS_SCL_LCNT / 4u] = 0x2fu;
    c->regs[IC_FS_SCL_HCNT / 4u] = 0x06u;
    c->regs[IC_FS_SCL_LCNT / 4u] = 0x0du;
    c->regs[IC_INTR_MASK / 4u] = 0x8ffu;
    c->regs[IC_SDA_HOLD / 4u] = 0x1u;
    c->regs[IC_SDA_SETUP / 4u] = 0x64u;
    c->regs[IC_ACK_GENERAL_CALL / 4u] = 0x1u;
    c->regs[IC_FS_SPKLEN / 4u] = 0x7u;
    c->raw = 0u;
    c->abrt_source = 0u;
    c->s_flush_cnt = 0u;
    c->s_en_status = 0u;
    c->s_rd_req = MM_FALSE;
    slv_flush(c);
    if (c->s_state != SS_IDLE) c->s_state = SS_OFF;
}

static void slv_tx_abort(struct dw_i2c *c, mm_u32 source)
{
    c->s_flush_cnt = (c->s_flush_cnt + c->s_tx_n) & 0x1ffu;
    c->abrt_source |= source;
    c->raw |= RAW_TX_ABRT;
    slv_flush(c); /* "flushes/resets/empties the TX_FIFO and RX_FIFO" */
    c->s_n_abrt++;
    SLV_TRACE("TX_ABRT source=0x%05x", c->abrt_source);
}

/* The IRQ line, for the NVIC's level re-sampling (rp2350_irq_level). */
mm_bool mm_rp2350_extra_i2c_line(mm_u32 irq)
{
    const struct dw_i2c *c = &g_i2c[1];
    if (!g_i2c_slv_on || irq != I2C1_IRQ) return MM_FALSE;
    if (mm_rp2350_reset_asserted(c->reset_bit)) return MM_FALSE;
    if (mm_rp2350_i2c1_master_irq_held()) return MM_FALSE; /* script: irq_hold_us */
    return (slv_raw(c) & c->regs[IC_INTR_MASK / 4u]) != 0u ? MM_TRUE : MM_FALSE;
}

void mm_rp2350_extra_set_nvic(struct mm_nvic *nvic)
{
    g_extra_nvic = nvic;
}

mm_u64 mm_rp2350_extra_next_event(void)
{
    if (!g_i2c_slv_on) return (mm_u64)-1;
    return mm_rp2350_i2c1_master_next_event();
}

static void slv_update_irq(struct dw_i2c *c)
{
    mm_u32 word = I2C1_IRQ / 32u;
    mm_u32 mask = 1u << (I2C1_IRQ % 32u);
    if (g_extra_nvic == 0 || !mm_rp2350_extra_i2c_line(I2C1_IRQ)) return;
    if ((g_extra_nvic->active_mask[word] & mask) != 0u) return;  /* re-sampled on exception return */
    if ((g_extra_nvic->pending_mask[word] & mask) != 0u) return;
    mm_nvic_set_pending(g_extra_nvic, I2C1_IRQ, MM_TRUE);
    c->s_n_irq++;
}

/* bus side, called by the master at the time of each event */

void mm_rp2350_i2c1_bus_start(mm_bool repeated)
{
    struct dw_i2c *c = &g_i2c[1];
    if (!c->slv) return;
    if (!slv_active(c)) {
        c->s_state = SS_IDLE;
        return;
    }
    c->s_n_start++;
    c->raw |= RAW_START_DET | RAW_ACTIVITY;
    c->s_repeated = repeated;
    if (!repeated) c->s_addressed = MM_FALSE;
    c->s_rd_req = MM_FALSE;
    c->s_state = SS_ADDR;
    SLV_TRACE("%s -> START_DET", repeated ? "Sr" : "S");
}

int mm_rp2350_i2c1_bus_addr(mm_u32 addr7, mm_bool read)
{
    struct dw_i2c *c = &g_i2c[1];
    if (!c->slv || c->s_state != SS_ADDR) return MM_I2C_SLV_NACK;
    if (!slv_active(c)) {
        c->s_state = SS_OFF;
        return MM_I2C_SLV_NACK;
    }
    if ((c->regs[IC_CON / 4u] & CON_10BITADDR_SLAVE) != 0u || addr7 != (c->regs[IC_SAR / 4u] & 0x7fu)) {
        c->s_n_other++;
        c->s_state = SS_OTHER;
        SLV_TRACE("address 0x%02x%c not ours (IC_SAR=0x%03x): NACK", addr7, read ? 'R' : 'W', c->regs[IC_SAR / 4u]);
        return MM_I2C_SLV_NACK;
    }
    c->s_n_match++;
    c->s_addressed = MM_TRUE;
    if (c->s_repeated) c->raw |= RAW_RESTART_DET;
    if (read) {
        if (c->s_tx_n != 0u) slv_tx_abort(c, ABRT_SLVFLUSH_TXFIFO);
        c->s_state = SS_TX;
    } else {
        c->s_state = SS_RX;
        c->s_first = MM_TRUE;
        c->s_rx_data = MM_FALSE;
    }
    SLV_TRACE("address 0x%02x%c: ACK%s", addr7, read ? 'R' : 'W', c->s_repeated ? ", RESTART_DET" : "");
    return MM_I2C_SLV_ACK;
}

int mm_rp2350_i2c1_bus_write(mm_u8 byte)
{
    struct dw_i2c *c = &g_i2c[1];
    if (!c->slv || c->s_state != SS_RX || !slv_active(c)) return MM_I2C_SLV_NACK;
    if ((c->regs[IC_SLV_DATA_NACK_ONLY / 4u] & 1u) != 0u) return MM_I2C_SLV_NACK;
    if (c->s_rx_n >= 16u) {
        if ((c->regs[IC_CON / 4u] & CON_RX_FIFO_FULL_HLD) != 0u) return MM_I2C_SLV_HOLD;
        c->raw |= RAW_RX_OVER;
        c->s_n_rx_over++;
        return MM_I2C_SLV_ACK; /* acknowledged, and lost */
    }
    c->s_rxf[(c->s_rx_head + c->s_rx_n) & 15u] = (mm_u16)(byte | (c->s_first ? DATA_FIRST_BYTE : 0u));
    c->s_rx_n++;
    c->s_first = MM_FALSE;
    c->s_rx_data = MM_TRUE;
    c->s_n_rx++;
    return MM_I2C_SLV_ACK;
}

int mm_rp2350_i2c1_bus_read(mm_u8 *byte)
{
    struct dw_i2c *c = &g_i2c[1];
    if (!c->slv || c->s_state != SS_TX || !slv_active(c)) return MM_I2C_SLV_NACK;
    if (c->s_tx_n == 0u) {
        if (!c->s_rd_req) {
            c->s_rd_req = MM_TRUE;
            c->raw |= RAW_RD_REQ;
            c->s_n_rd_req++;
            SLV_TRACE("TX FIFO empty -> RD_REQ, SCL held");
        }
        return MM_I2C_SLV_HOLD;
    }
    *byte = c->s_txf[c->s_tx_head];
    c->s_tx_head = (c->s_tx_head + 1u) & 15u;
    c->s_tx_n--;
    c->s_rd_req = MM_FALSE;
    c->s_n_tx++;
    return MM_I2C_SLV_ACK;
}

void mm_rp2350_i2c1_bus_master_ack(mm_bool ack)
{
    struct dw_i2c *c = &g_i2c[1];
    if (!c->slv || c->s_state != SS_TX || ack) return;
    c->raw |= RAW_RX_DONE;
    c->s_n_rx_done++;
    c->s_state = SS_TX_DONE;
}

void mm_rp2350_i2c1_bus_stop(void)
{
    struct dw_i2c *c = &g_i2c[1];
    mm_u8 st = c->s_state;
    if (!c->slv) return;
    c->s_state = SS_IDLE;
    c->s_rd_req = MM_FALSE;
    if (st == SS_IDLE || st == SS_OFF || !slv_active(c)) return;
    if ((c->regs[IC_CON / 4u] & CON_STOP_DET_IFADDR) == 0u || c->s_addressed) {
        c->raw |= RAW_STOP_DET;
        SLV_TRACE("P -> STOP_DET");
    }
    c->s_addressed = MM_FALSE;
    c->s_n_stop++;
}

/* register side */

static const char *slv_reg_name(mm_u32 off)
{
    switch (off) {
    case IC_CON: return "IC_CON";
    case IC_SAR: return "IC_SAR";
    case IC_DATA_CMD: return "IC_DATA_CMD";
    case IC_INTR_STAT: return "IC_INTR_STAT";
    case IC_INTR_MASK: return "IC_INTR_MASK";
    case IC_RAW_INTR_STAT: return "IC_RAW_INTR_STAT";
    case IC_RX_TL: return "IC_RX_TL";
    case IC_TX_TL: return "IC_TX_TL";
    case IC_CLR_INTR: return "IC_CLR_INTR";
    case IC_CLR_RD_REQ: return "IC_CLR_RD_REQ";
    case IC_CLR_TX_ABRT: return "IC_CLR_TX_ABRT";
    case IC_CLR_STOP_DET: return "IC_CLR_STOP_DET";
    case IC_CLR_START_DET: return "IC_CLR_START_DET";
    case IC_CLR_RESTART_DET: return "IC_CLR_RESTART_DET";
    case IC_ENABLE: return "IC_ENABLE";
    case IC_STATUS: return "IC_STATUS";
    case IC_TXFLR: return "IC_TXFLR";
    case IC_RXFLR: return "IC_RXFLR";
    case IC_TX_ABRT_SOURCE: return "IC_TX_ABRT_SOURCE";
    case IC_ENABLE_STATUS: return "IC_ENABLE_STATUS";
    default: return "";
    }
}

static mm_u32 slv_reg_read_inner(struct dw_i2c *c, mm_u32 off, mm_bool peek)
{
    switch (off) {
    case IC_RAW_INTR_STAT: return slv_raw(c);
    case IC_INTR_STAT: return slv_raw(c) & c->regs[IC_INTR_MASK / 4u];
    case IC_TX_ABRT_SOURCE: return c->abrt_source | (c->s_flush_cnt << 23);
    case IC_CLR_INTR:
        if (!peek) {
            c->raw &= RAW_RX_FULL | RAW_TX_EMPTY; /* hardware-cleared ones are levels anyway */
            c->abrt_source = 0u;
        }
        return 0u;
    case IC_CLR_RX_UNDER: if (!peek) c->raw &= ~RAW_RX_UNDER; return 0u;
    case IC_CLR_RX_OVER: if (!peek) c->raw &= ~RAW_RX_OVER; return 0u;
    case IC_CLR_TX_OVER: if (!peek) c->raw &= ~RAW_TX_OVER; return 0u;
    case IC_CLR_RD_REQ: if (!peek) c->raw &= ~RAW_RD_REQ; return 0u;
    case IC_CLR_TX_ABRT:
        if (!peek) { c->raw &= ~RAW_TX_ABRT; c->abrt_source = 0u; }
        return 0u;
    case IC_CLR_RX_DONE: if (!peek) c->raw &= ~RAW_RX_DONE; return 0u;
    case IC_CLR_ACTIVITY: if (!peek && !slv_busy(c)) c->raw &= ~RAW_ACTIVITY; return 0u;
    case IC_CLR_STOP_DET: if (!peek) c->raw &= ~RAW_STOP_DET; return 0u;
    case IC_CLR_START_DET: if (!peek) c->raw &= ~RAW_START_DET; return 0u;
    case IC_CLR_GEN_CALL: if (!peek) c->raw &= ~RAW_GEN_CALL; return 0u;
    case IC_CLR_RESTART_DET: if (!peek) c->raw &= ~RAW_RESTART_DET; return 0u;
    case IC_STATUS:
        return (slv_busy(c) ? ((1u << 0) | (1u << 6)) : 0u) |   /* ACTIVITY | SLV_ACTIVITY */
               ((c->s_tx_n < 16u) ? (1u << 1) : 0u) |           /* TFNF */
               ((c->s_tx_n == 0u) ? (1u << 2) : 0u) |           /* TFE */
               ((c->s_rx_n != 0u) ? (1u << 3) : 0u) |           /* RFNE */
               ((c->s_rx_n == 16u) ? (1u << 4) : 0u);           /* RFF */
    case IC_TXFLR: return c->s_tx_n;
    case IC_RXFLR: return c->s_rx_n;
    case IC_DATA_CMD: {
        mm_u32 v;
        if (c->s_rx_n == 0u) {
            if (!peek) { c->raw |= RAW_RX_UNDER; c->s_n_rx_under++; }
            return 0u;
        }
        v = c->s_rxf[c->s_rx_head];
        if (!peek) {
            c->s_rx_head = (c->s_rx_head + 1u) & 15u;
            c->s_rx_n--;
        }
        return v;
    }
    case IC_ENABLE_STATUS:
        if ((c->regs[IC_ENABLE / 4u] & 1u) != 0u) return 1u;
        return c->s_en_status;
    case IC_COMP_PARAM_1: return 0x000f0fefu; /* 16-deep FIFOs */
    case IC_COMP_VERSION: return 0x3230312au;
    case IC_COMP_TYPE: return 0x44570140u;
    default:
        if (off >= 0x40u && off <= 0x68u) return 0u;
        return c->regs[(off & 0xffu) / 4u];
    }
}

static mm_u32 i2c_slv_reg_read(struct dw_i2c *c, mm_u32 off)
{
    mm_bool peek = mmio_peek_mode();
    mm_u32 v;
    if (mm_rp2350_reset_asserted(c->reset_bit)) return 0u;
    if (!peek) mm_rp2350_i2c1_master_run(); /* the bus up to now */
    v = slv_reg_read_inner(c, off, peek);
    if (!peek) {
        SLV_TRACE("rd +0x%02x %-18s -> 0x%08x", off, slv_reg_name(off), v);
        if (off == IC_DATA_CMD) mm_rp2350_i2c1_master_run(); /* room in the RX FIFO: a held byte goes on now */
    }
    return v;
}

static void i2c_slv_reg_write(struct dw_i2c *c, mm_u32 off, mm_u32 v)
{
    mm_bool en;
    if (mm_rp2350_reset_asserted(c->reset_bit)) return;
    mm_rp2350_i2c1_master_run();
    en = ((c->regs[IC_ENABLE / 4u] & 1u) != 0u) ? MM_TRUE : MM_FALSE;
    SLV_TRACE("wr +0x%02x %-18s <- 0x%08x", off, slv_reg_name(off), v);
    switch (off) {
    case IC_DATA_CMD:
        if (!en) return;
        if ((c->regs[IC_CON / 4u] & CON_MASTER_MODE) != 0u) {
            /* master mode: nobody else on this bus acknowledges, as before */
            c->raw |= RAW_TX_ABRT | RAW_STOP_DET | RAW_START_DET;
            c->abrt_source = 1u; /* ABRT_7B_ADDR_NOACK */
            c->nacks++;
            return;
        }
        if ((c->raw & RAW_TX_ABRT) != 0u) return; /* held flushed until IC_CLR_TX_ABRT */
        if ((v & CMD_READ) != 0u) { slv_tx_abort(c, ABRT_SLVRD_INTX); return; }
        if (c->s_tx_n >= 16u) {
            c->raw |= RAW_TX_OVER;
            c->s_n_tx_over++;
            return;
        }
        c->s_txf[(c->s_tx_head + c->s_tx_n) & 15u] = (mm_u8)(v & 0xffu);
        c->s_tx_n++;
        mm_rp2350_i2c1_master_run(); /* a held read goes on now */
        return;
    case IC_ENABLE: {
        mm_u32 old = c->regs[IC_ENABLE / 4u];
        c->regs[IC_ENABLE / 4u] = v & 0x5u; /* ENABLE, TX_CMD_BLOCK; ABORT is master only */
        if ((old & 1u) != 0u && (v & 1u) == 0u) {
            if (slv_busy(c)) {
                c->s_en_status = (1u << 1) | ((c->s_state == SS_RX && c->s_rx_data) ? (1u << 2) : 0u);
                c->s_state = SS_OFF;
                SLV_TRACE("disabled during a transfer: SLV_DISABLED_WHILE_BUSY%s",
                          (c->s_en_status & 4u) ? ", SLV_RX_DATA_LOST" : "");
            } else {
                c->s_en_status = 0u;
            }
            slv_flush(c);
            c->raw = 0u;
            c->abrt_source = 0u;
            c->s_flush_cnt = 0u;
            c->s_rd_req = MM_FALSE;
        } else if ((old & 1u) == 0u && (v & 1u) != 0u) {
            c->s_en_status = 0u;
        }
        return;
    }
    case IC_CON:
    case IC_SAR:
    case IC_SS_SCL_HCNT:
    case IC_SS_SCL_LCNT:
    case IC_FS_SCL_HCNT:
    case IC_FS_SCL_LCNT:
    case IC_FS_SPKLEN:
    case IC_SLV_DATA_NACK_ONLY:
        if (en) {
            c->s_n_locked++;
            SLV_TRACE("wr +0x%02x ignored: IC_ENABLE = 1", off);
            return;
        }
        if (off == IC_CON) v &= 0x3ffu;
        else if (off == IC_SAR) v &= 0x3ffu;
        else if (off == IC_FS_SPKLEN) { v &= 0xffu; if (v == 0u) v = 1u; }
        else if (off == IC_SLV_DATA_NACK_ONLY) v &= 1u;
        else v &= 0xffffu;
        c->regs[off / 4u] = v;
        return;
    case IC_RX_TL:
    case IC_TX_TL:
        v &= 0xffu;
        c->regs[off / 4u] = (v > 15u) ? 15u : v;
        return;
    case IC_INTR_MASK:
        c->regs[off / 4u] = v & 0x1fffu;
        return;
    default:
        if (off < 0x100u && !(off >= 0x40u && off <= 0x68u) && off != IC_CLR_RESTART_DET) c->regs[off / 4u] = v;
        return;
    }
}

void mm_rp2350_extra_i2c_reset(mm_u32 index)
{
    struct dw_i2c *c = &g_i2c[index & 1u];
    if (c->slv) { /* slave model */
        slv_regs_reset(c);
        SLV_TRACE("RESETS: block reset, registers at their reset values");
        return;
    }
    if (!c->dev) return; /* the empty-bus model keeps its registers, as before */
    i2c_eng_halt(c, MM_TRUE);
    memset(c->regs, 0, sizeof(c->regs));
    c->raw = 0u;
    c->abrt_source = 0u;
}

static mm_u32 i2c_reg_read(struct dw_i2c *c, mm_u32 off)
{
    mm_u32 v;
    if (c->slv) return i2c_slv_reg_read(c, off); /* slave mode, external master */
    if (c->dev) return i2c_eng_reg_read(c, off); /* slave model attached */
    switch (off) {
    case IC_RAW_INTR_STAT: return c->raw | RAW_TX_EMPTY;
    case IC_INTR_STAT: return (c->raw | RAW_TX_EMPTY) & c->regs[IC_INTR_MASK / 4u];
    case IC_TX_ABRT_SOURCE: return c->abrt_source;
    case IC_CLR_INTR:
        if (!mmio_peek_mode()) { c->raw = 0u; c->abrt_source = 0u; }
        return 0u;
    case IC_CLR_TX_ABRT:
        if (!mmio_peek_mode()) { c->raw &= ~RAW_TX_ABRT; c->abrt_source = 0u; }
        return 0u;
    case IC_CLR_STOP_DET:
        if (!mmio_peek_mode()) c->raw &= ~RAW_STOP_DET;
        return 0u;
    case IC_CLR_START_DET:
        if (!mmio_peek_mode()) c->raw &= ~RAW_START_DET;
        return 0u;
    case IC_STATUS: return (1u << 1) | (1u << 2); /* TFNF | TFE, idle */
    case IC_TXFLR: return 0u;
    case IC_RXFLR: return 0u;
    case IC_DATA_CMD: return 0u;
    case IC_ENABLE_STATUS: return c->regs[IC_ENABLE / 4u] & 1u;
    case IC_COMP_PARAM_1: return 0x000f0fefu; /* 16-deep FIFOs */
    case IC_COMP_VERSION: return 0x3230312au;
    case IC_COMP_TYPE: return 0x44570140u;
    default:
        if (off >= 0x40u && off <= 0x68u) return 0u; /* other clear-on-read */
        v = c->regs[(off & 0xffu) / 4u];
        return v;
    }
}

static void i2c_reg_write(struct dw_i2c *c, mm_u32 off, mm_u32 v)
{
    if (c->slv) { i2c_slv_reg_write(c, off, v); return; } /* slave mode, external master */
    if (c->dev) { i2c_eng_reg_write(c, off, v); return; } /* slave model attached */
    if (off == IC_DATA_CMD) {
        if ((c->regs[IC_ENABLE / 4u] & 1u) != 0u && (c->regs[IC_CON / 4u] & 1u) != 0u) {
            /* master: nobody acknowledges the address */
            c->raw |= RAW_TX_ABRT | RAW_STOP_DET | RAW_START_DET;
            c->abrt_source = 1u; /* ABRT_7B_ADDR_NOACK */
            c->nacks++;
            if (!c->logged) {
                printf("[I2C] %s: transfer to 0x%02x NACKed (empty bus model)\n", c->name,
                       c->regs[IC_TAR / 4u] & 0x3ffu);
                c->logged = MM_TRUE;
            }
        }
        return;
    }
    if (off < 0x100u) c->regs[off / 4u] = v;
}

static mm_bool i2c_read(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 *value_out)
{
    struct dw_i2c *c = (struct dw_i2c *)opaque;
    mm_u32 off = offset & 0xfffu;
    mm_u32 v;
    if (c == 0 || value_out == 0) return MM_FALSE;
    if (off >= 0x100u) { *value_out = 0u; return MM_TRUE; }
    v = i2c_reg_read(c, off & ~3u) >> ((off & 3u) * 8u);
    if (size_bytes == 1u) v &= 0xffu;
    else if (size_bytes == 2u) v &= 0xffffu;
    *value_out = v;
    return MM_TRUE;
}

static mm_bool i2c_write(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 value)
{
    struct dw_i2c *c = (struct dw_i2c *)opaque;
    mm_u32 alias = (offset >> 12) & 3u;
    mm_u32 off = offset & 0xfffu;
    mm_u32 reg = off & ~3u;
    mm_u32 cur;
    if (c == 0) return MM_FALSE;
    if (reg >= 0x100u) return MM_TRUE;
    if (size_bytes == 1u) value = (value & 0xffu) * 0x01010101u;
    else if (size_bytes == 2u) value = (value & 0xffffu) * 0x00010001u;
    if (alias != 0u) {
        cur = (reg == IC_DATA_CMD) ? 0u : c->regs[reg / 4u];
        if (alias == 1u) value = cur ^ value;
        else if (alias == 2u) value = cur | value;
        else value = cur & ~value;
    }
    i2c_reg_write(c, reg, value);
    return MM_TRUE;
}

/* ------------------------------------------------------------------ */
/* XIP aliases                                                          */
/* ------------------------------------------------------------------ */

static mm_u8 *g_flash = 0;
static mm_u32 g_flash_size = 0;
static struct mm_memmap *g_map = 0;

static mm_bool xip_alias_read(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 *value_out)
{
    mm_u32 v = 0u;
    mm_u32 i;
    (void)opaque;
    if (value_out == 0) return MM_FALSE;
    for (i = 0; i < size_bytes; ++i) {
        mm_u32 o = offset + i;
        mm_u8 b = 0xffu;
        if (g_flash != 0 && o < g_flash_size) b = g_flash[o];
        v |= (mm_u32)b << (8u * i);
    }
    *value_out = v;
    return MM_TRUE;
}

static mm_bool xip_alias_write(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 value)
{
    (void)opaque; (void)offset; (void)size_bytes; (void)value;
    return MM_TRUE; /* writes to XIP are ignored (no cache-as-SRAM) */
}

/* ------------------------------------------------------------------ */
/* partition table                                                      */
/* ------------------------------------------------------------------ */

#define PT_INFO_PT_INFO                   0x0001u
#define PT_INFO_PARTITION_LOCATION_AND_FLAGS 0x0010u
#define PT_INFO_PARTITION_ID              0x0020u
#define PT_INFO_PARTITION_FAMILY_IDS      0x0040u
#define PT_INFO_PARTITION_NAME            0x0080u
#define PT_INFO_SINGLE_PARTITION          0x8000u

static struct {
    mm_bool loaded;
    mm_u32 count;
    mm_u32 unpart;
    mm_u32 loc[16];
    mm_u32 flags[16];
    mm_u32 id_lo[16];
    mm_u32 id_hi[16];
} g_pt;

static mm_u32 rd32(const mm_u8 *p)
{
    return (mm_u32)p[0] | ((mm_u32)p[1] << 8) | ((mm_u32)p[2] << 16) | ((mm_u32)p[3] << 24);
}

/* Find the first PICOBIN block in buf with a PARTITION_TABLE item and take
 * the partitions from it.  Every read stays inside buf and inside the item;
 * a block or item that runs past the end, or a partition entry that runs
 * past its item, ends the parse with MM_FALSE and g_pt untouched. */
static mm_bool pt_parse_block(const mm_u8 *buf, size_t len)
{
    size_t i;
    for (i = 0; i + 8u <= len; i += 4u) {
        size_t p;
        if (rd32(buf + i) != 0xffffded3u) continue;
        p = i + 4u;
        while (p + 4u <= len) {
            mm_u32 hdr = rd32(buf + p);
            mm_u32 type = hdr & 0xffu;
            mm_u32 size = (type & 0x80u) ? ((hdr >> 8) & 0xffffu) : ((hdr >> 8) & 0xffu);
            size_t item_end;
            if (type == 0xffu) break; /* LAST */
            if (size == 0u) break;
            item_end = p + 4u * (size_t)size;
            if (item_end > len) return MM_FALSE;
            if ((type & 0x7fu) == 0x0au) {
                mm_u32 count = (hdr >> 24) & 0xffu;
                size_t q = p + 4u;
                mm_u32 unpart;
                mm_u32 loc[16], flags[16], id_lo[16], id_hi[16];
                mm_u32 n;
                if (q + 4u > item_end) return MM_FALSE;
                unpart = rd32(buf + q); q += 4u;
                for (n = 0; n < count && n < 16u; ++n) {
                    mm_u32 fl;
                    size_t extra;
                    if (q + 8u > item_end) return MM_FALSE;
                    loc[n] = rd32(buf + q);
                    fl = rd32(buf + q + 4u);
                    q += 8u;
                    flags[n] = fl;
                    id_lo[n] = id_hi[n] = 0u;
                    if (fl & 0x1u) {
                        if (q + 8u > item_end) return MM_FALSE;
                        id_lo[n] = rd32(buf + q);
                        id_hi[n] = rd32(buf + q + 4u);
                        q += 8u;
                    }
                    extra = 4u * ((fl >> 7) & 3u); /* extra families */
                    if (q + extra > item_end) return MM_FALSE;
                    q += extra;
                    if (fl & 0x1000u) {
                        size_t name;
                        if (q + 1u > item_end) return MM_FALSE;
                        name = (1u + (size_t)(buf[q] & 0x7fu) + 3u) & ~(size_t)3u;
                        if (q + name > item_end) return MM_FALSE;
                        q += name;
                    }
                }
                g_pt.unpart = unpart;
                memcpy(g_pt.loc, loc, n * sizeof(loc[0]));
                memcpy(g_pt.flags, flags, n * sizeof(flags[0]));
                memcpy(g_pt.id_lo, id_lo, n * sizeof(id_lo[0]));
                memcpy(g_pt.id_hi, id_hi, n * sizeof(id_hi[0]));
                g_pt.count = n;
                g_pt.loaded = MM_TRUE;
                return MM_TRUE;
            }
            p = item_end;
        }
    }
    return MM_FALSE;
}

static void pt_load_env(void)
{
    const char *path = getenv("M33MU_RP2350_PT");
    FILE *f;
    mm_u8 *buf;
    long len;
    if (path == 0 || path[0] == '\0') return;
    f = fopen(path, "rb");
    if (f == 0) { printf("[PT] cannot open %s\n", path); return; }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0 || len > (1 << 20)) { fclose(f); return; }
    buf = (mm_u8 *)malloc((size_t)len);
    if (buf == 0) { fclose(f); return; }
    if (fread(buf, 1, (size_t)len, f) != (size_t)len) { free(buf); fclose(f); return; }
    fclose(f);
    if (len >= 512 && rd32(buf) == 0x0a324655u) {
        /* UF2: concatenate payloads */
        long off;
        mm_u8 *flat = (mm_u8 *)calloc(1, (size_t)len);
        size_t fl = 0;
        if (flat == 0) { free(buf); return; }
        for (off = 0; off + 512 <= len; off += 512) {
            mm_u32 psz = rd32(buf + off + 16);
            if (psz > 476u) psz = 476u;
            if (fl + psz > (size_t)len) break;
            memcpy(flat + fl, buf + off + 32, psz);
            fl += psz;
        }
        (void)pt_parse_block(flat, fl);
        free(flat);
    } else {
        (void)pt_parse_block(buf, (size_t)len);
    }
    free(buf);
    if (g_pt.loaded) {
        mm_u32 n;
        printf("[PT] loaded %u partitions from %s\n", g_pt.count, path);
        for (n = 0; n < g_pt.count; ++n) {
            mm_u32 first = g_pt.loc[n] & 0x1fffu;
            mm_u32 last = (g_pt.loc[n] >> 13) & 0x1fffu;
            printf("[PT]   %u: flash 0x%06x-0x%06x id=%u flags=0x%08x\n", n, first * 4096u,
                   (last + 1u) * 4096u - 1u, g_pt.id_lo[n], g_pt.flags[n]);
        }
    } else {
        printf("[PT] no partition table found in %s\n", path);
    }
}

mm_bool mm_rp2350_extra_pt_loaded(void)
{
    return g_pt.loaded;
}

mm_u32 mm_rp2350_extra_pt_info(struct mm_memmap *map, enum mm_sec_state sec,
                               mm_u32 out_addr, mm_u32 out_words, mm_u32 flags_and_partition)
{
    mm_u32 flags = flags_and_partition & 0xffffu;
    mm_u32 single = (flags & PT_INFO_SINGLE_PARTITION) != 0u;
    mm_u32 pidx = (flags_and_partition >> 24) & 0xffu;
    mm_u32 supported = flags & (PT_INFO_PT_INFO | PT_INFO_PARTITION_LOCATION_AND_FLAGS |
                                PT_INFO_PARTITION_ID | PT_INFO_SINGLE_PARTITION);
    mm_u32 w = 0u;
    mm_u32 n;
#define PUT(val) do { if (w >= out_words) return (mm_u32)0u - 13u; \
        (void)mm_memmap_write(map, sec, out_addr + 4u * w, 4u, (val)); w++; } while (0)
    if (single && pidx >= g_pt.count) return (mm_u32)0u - 17u; /* BOOTROM_ERROR_NOT_FOUND */
    PUT(supported);
    if (flags & PT_INFO_PT_INFO) {
        PUT(g_pt.count | (1u << 8)); /* count, has-partition-table */
        PUT(g_pt.unpart);
    }
    for (n = 0; n < g_pt.count; ++n) {
        if (single && n != pidx) continue;
        if (flags & PT_INFO_PARTITION_LOCATION_AND_FLAGS) {
            PUT(g_pt.loc[n]);
            PUT(g_pt.flags[n]);
        }
        if ((flags & PT_INFO_PARTITION_ID) && (g_pt.flags[n] & 1u)) {
            PUT(g_pt.id_lo[n]);
            PUT(g_pt.id_hi[n]);
        }
    }
#undef PUT
    return w;
}

/* ------------------------------------------------------------------ */
/* SEGGER RTT up-buffer 0 drain                                         */
/* ------------------------------------------------------------------ */

static FILE *g_rtt_out = 0;
static mm_bool g_rtt_enabled = MM_FALSE;
static mm_u32 g_rtt_cb = 0;
static mm_u64 g_rtt_cycles = 0;

static mm_u8 *ram_ptr(mm_u32 addr, mm_u32 len)
{
    if (g_map == 0 || g_map->ram.buffer == 0) return 0;
    if (addr < g_map->ram.base) return 0;
    if ((mm_u64)addr - g_map->ram.base + len > g_map->ram.length) return 0;
    return (mm_u8 *)g_map->ram.buffer + (addr - g_map->ram.base);
}

static void rtt_scan(void)
{
    mm_u8 *base;
    mm_u32 i;
    if (g_map == 0) return;
    base = ram_ptr(g_map->ram.base, 16u);
    if (base == 0) return;
    for (i = 0; i + 64u < g_map->ram.length; i += 4u) {
        if (memcmp(base + i, "SEGGER RTT\0", 11) == 0) {
            g_rtt_cb = g_map->ram.base + i;
            printf("[RTT] control block at 0x%08x\n", g_rtt_cb);
            fflush(stdout);
            return;
        }
    }
}

static void rtt_drain(void)
{
    mm_u8 *cb;
    mm_u32 nup;
    mm_u32 pbuf, size, wr, rd;
    mm_u8 *up;
    if (g_rtt_cb == 0u) { rtt_scan(); if (g_rtt_cb == 0u) return; }
    cb = ram_ptr(g_rtt_cb, 24u + 24u);
    if (cb == 0 || memcmp(cb, "SEGGER RTT", 10) != 0) { g_rtt_cb = 0u; return; }
    nup = rd32(cb + 16);
    if (nup == 0u || nup > 16u) return;
    up = cb + 24;
    pbuf = rd32(up + 4);
    size = rd32(up + 8);
    wr = rd32(up + 12);
    rd = rd32(up + 16);
    if (size == 0u || wr >= size || rd >= size || wr == rd) return;
    {
        mm_u8 *data = ram_ptr(pbuf, size);
        if (data == 0) return;
        while (rd != wr) {
            mm_u32 end = (wr > rd) ? wr : size;
            fwrite(data + rd, 1, end - rd, g_rtt_out);
            if (g_i2c_slv_on) mm_rp2350_i2c1_master_rtt(data + rd, end - rd); /* wait_log */
            rd = (end == size) ? 0u : end;
        }
        fflush(g_rtt_out);
        up[16] = (mm_u8)rd; up[17] = (mm_u8)(rd >> 8); up[18] = (mm_u8)(rd >> 16); up[19] = (mm_u8)(rd >> 24);
    }
}

/* PC sampler: M33MU_PC_SAMPLE=<cycles> histograms core-0 PCs (1 KiB buckets
 * are too coarse, so exact PCs in a small open-addressing table). */
static const struct mm_cpu *g_pcs_cpu = 0;

void mm_rp2350_extra_set_cpu0(const struct mm_cpu *cpu)
{
    g_pcs_cpu = cpu;
}

#define PCS_SLOTS 4096u
static mm_u32 g_pcs_pc[PCS_SLOTS];
static mm_u32 g_pcs_n[PCS_SLOTS];
static mm_u64 g_pcs_every = 0, g_pcs_acc = 0, g_pcs_total = 0;

static void pc_sample(mm_u64 cycles)
{
    mm_u32 pc, h, i;
    g_pcs_acc += cycles;
    if (g_pcs_acc < g_pcs_every) return;
    g_pcs_acc = 0;
    pc = ((g_pcs_cpu != 0) ? g_pcs_cpu->r[15] : 0u) & ~1u;
    h = (pc >> 1) * 2654435761u;
    for (i = 0; i < 64u; ++i) {
        mm_u32 k = (h + i) % PCS_SLOTS;
        if (g_pcs_n[k] == 0u || g_pcs_pc[k] == pc) { g_pcs_pc[k] = pc; g_pcs_n[k]++; g_pcs_total++; return; }
    }
}

static void pc_report(void)
{
    mm_u32 top, i, best;
    if (g_pcs_total == 0u) return;
    fprintf(stderr, "[PCS] %llu samples, top PCs:\n", (unsigned long long)g_pcs_total);
    for (top = 0; top < 16u; ++top) {
        best = PCS_SLOTS;
        for (i = 0; i < PCS_SLOTS; ++i) if (g_pcs_n[i] && (best == PCS_SLOTS || g_pcs_n[i] > g_pcs_n[best])) best = i;
        if (best == PCS_SLOTS) break;
        fprintf(stderr, "[PCS] 0x%08x %5.1f%%\n", g_pcs_pc[best], 100.0 * g_pcs_n[best] / (double)g_pcs_total);
        g_pcs_n[best] = 0u;
    }
}

/* ------------------------------------------------------------------ */
/* measurement probes, all off unless M33MU_PROBE_LOG is set     */
/* ------------------------------------------------------------------ */
/*
 *   M33MU_PROBE_LOG=<file|->        where the lines go; nothing below is
 *                                   active without it
 *   M33MU_PROBE_FN=<addr>=<name>,.. log entry and return of the functions at
 *                                   these addresses with the time between
 *                                   them.  The entry is seen when a basic
 *                                   block starts at <addr>, the return when
 *                                   execution reaches the LR of the entry
 *                                   with the SP of the entry.  The time
 *                                   includes whatever preempted the thread
 *                                   and every sleep inside the function.
 *                                   For a function entered as an exception
 *                                   handler only the entry is logged.
 *   M33MU_PROBE_DMA_PIO=1           log every DMA run whose destination is a
 *                                   PIO TX FIFO, with the words written
 * t_us is the same clock as in the device model logs.
 */
#define PROBE_FN_MAX 8u
#define PROBE_FRAME_MAX 16u

static struct {
    int state; /* 0 = environment not read, 1 = on, -1 = off */
    FILE *log;
    mm_bool dma_pio;
    mm_u32 fn_n;
    mm_u32 fn_addr[PROBE_FN_MAX];
    char fn_name[PROBE_FN_MAX][40];
    struct {
        mm_bool used;
        mm_u32 fn;
        mm_u32 ret;
        mm_u32 sp;
        mm_u64 t0;
    } frame[PROBE_FRAME_MAX];
    mm_u64 n_enter, n_exit, n_dma;
} g_probe;

static mm_u64 probe_us(mm_u64 cycles)
{
    mm_u64 per_us = mm_rp2350_cpu_hz() / 1000000ull;
    return cycles / (per_us ? per_us : 1u);
}

static void probe_load_env(void)
{
    const char *path = getenv("M33MU_PROBE_LOG");
    const char *fn = getenv("M33MU_PROBE_FN");
    const char *dma = getenv("M33MU_PROBE_DMA_PIO");
    g_probe.state = -1;
    if (path == 0 || path[0] == '\0') return;
    g_probe.log = (strcmp(path, "-") == 0) ? stdout : fopen(path, "w");
    if (g_probe.log == 0) g_probe.log = stdout;
    g_probe.state = 1;
    g_probe.dma_pio = (dma != 0 && dma[0] != '\0' && strcmp(dma, "0") != 0) ? MM_TRUE : MM_FALSE;
    while (fn != 0 && *fn != '\0' && g_probe.fn_n < PROBE_FN_MAX) {
        char *end = 0;
        mm_u32 addr = (mm_u32)strtoul(fn, &end, 0);
        mm_u32 k = 0u;
        if (end == fn) break;
        g_probe.fn_addr[g_probe.fn_n] = addr & ~1u;
        if (*end == '=') {
            ++end;
            while (*end != '\0' && *end != ',' && k + 1u < sizeof(g_probe.fn_name[0])) g_probe.fn_name[g_probe.fn_n][k++] = *end++;
        }
        g_probe.fn_name[g_probe.fn_n][k] = '\0';
        if (k == 0u) snprintf(g_probe.fn_name[g_probe.fn_n], sizeof(g_probe.fn_name[0]), "0x%08x", addr & ~1u);
        g_probe.fn_n++;
        while (*end != '\0' && *end != ',') ++end;
        fn = (*end == ',') ? end + 1 : end;
    }
    fprintf(g_probe.log, "# m33mu probes (emulated time, t_us = system clock cycles / %llu):",
            (unsigned long long)(mm_rp2350_cpu_hz() / 1000000ull));
    {
        mm_u32 i;
        for (i = 0; i < g_probe.fn_n; ++i) fprintf(g_probe.log, " fn 0x%08x=%s", g_probe.fn_addr[i], g_probe.fn_name[i]);
    }
    fprintf(g_probe.log, "%s\n", g_probe.dma_pio ? " dma->pio" : "");
    fflush(g_probe.log);
}

mm_bool mm_rp2350_probe_fn_on(void)
{
    if (g_probe.state == 0) probe_load_env();
    return (g_probe.state > 0 && g_probe.fn_n != 0u) ? MM_TRUE : MM_FALSE;
}

/* Called by the CPU loop at the start of every basic block / instruction. */
void mm_rp2350_probe_fn_check(mm_u32 pc, mm_u32 lr, mm_u32 sp)
{
    mm_u32 i;
    mm_u64 now = mm_rp2350_now_cycles();
    pc &= ~1u;
    for (i = 0; i < PROBE_FRAME_MAX; ++i) {
        if (g_probe.frame[i].used && g_probe.frame[i].ret == pc && g_probe.frame[i].sp == sp) {
            mm_u64 d = now - g_probe.frame[i].t0;
            g_probe.frame[i].used = MM_FALSE;
            g_probe.n_exit++;
            fprintf(g_probe.log, "t_us=%-10llu EXIT  %s dur_us=%llu dur_cycles=%llu\n",
                    (unsigned long long)probe_us(now), g_probe.fn_name[g_probe.frame[i].fn],
                    (unsigned long long)probe_us(d), (unsigned long long)d);
            fflush(g_probe.log);
        }
    }
    for (i = 0; i < g_probe.fn_n; ++i) {
        mm_u32 k, slot = PROBE_FRAME_MAX;
        if (g_probe.fn_addr[i] != pc) continue;
        for (k = 0; k < PROBE_FRAME_MAX; ++k) {
            if (g_probe.frame[k].used && g_probe.frame[k].fn == i && g_probe.frame[k].sp == sp &&
                g_probe.frame[k].ret == (lr & ~1u)) {
                slot = PROBE_FRAME_MAX + 1u; /* the same activation, seen again */
                break;
            }
            if (!g_probe.frame[k].used && slot == PROBE_FRAME_MAX) slot = k;
        }
        if (slot > PROBE_FRAME_MAX) continue;
        if ((lr & 0xff000000u) == 0xff000000u) {
            /* entered as an exception handler: LR is an EXC_RETURN value, no
             * block ever starts there, so only the entry can be logged */
            g_probe.n_enter++;
            fprintf(g_probe.log, "t_us=%-10llu ENTER %s sp=0x%08x lr=0x%08x (exception entry, return not tracked)\n",
                    (unsigned long long)probe_us(now), g_probe.fn_name[i], sp, lr);
            fflush(g_probe.log);
            continue;
        }
        g_probe.n_enter++;
        fprintf(g_probe.log, "t_us=%-10llu ENTER %s sp=0x%08x lr=0x%08x%s\n", (unsigned long long)probe_us(now),
                g_probe.fn_name[i], sp, lr, (slot == PROBE_FRAME_MAX) ? " (no free slot, return not tracked)" : "");
        fflush(g_probe.log);
        if (slot == PROBE_FRAME_MAX) continue;
        g_probe.frame[slot].used = MM_TRUE;
        g_probe.frame[slot].fn = i;
        g_probe.frame[slot].ret = lr & ~1u;
        g_probe.frame[slot].sp = sp;
        g_probe.frame[slot].t0 = now;
    }
}

mm_bool mm_rp2350_probe_dma_on(void)
{
    if (g_probe.state == 0) probe_load_env();
    return (g_probe.state > 0 && g_probe.dma_pio) ? MM_TRUE : MM_FALSE;
}

/* One DMA run into a PIO TX FIFO: `words` are the values written. */
void mm_rp2350_probe_dma(mm_u32 ch, mm_u32 dst, mm_u32 step, const mm_u32 *words, mm_u32 n, mm_u32 n_total)
{
    mm_u32 i;
    mm_u32 pio = (dst - 0x50200000u) / 0x00100000u;
    mm_u32 sm = ((dst & 0xfffu) - 0x10u) / 4u;
    g_probe.n_dma++;
    fprintf(g_probe.log, "t_us=%-10llu DMA   ch=%u -> PIO%u.TXF%u n=%u x %uB data=",
            (unsigned long long)probe_us(mm_rp2350_now_cycles()), ch, pio, sm, n_total, step);
    for (i = 0; i < n; ++i) fprintf(g_probe.log, "%s%0*x", i ? " " : "", (int)(step * 2u), words[i]);
    fprintf(g_probe.log, "%s\n", (n < n_total) ? " ..." : "");
    fflush(g_probe.log);
}

void mm_rp2350_extra_tick(mm_u64 cycles)
{
    if (g_i2c_dev_any) { /* keep a bus with a slave model moving */
        if (g_i2c[0].dev && g_i2c[0].phase != PH_IDLE) i2c_eng_run(&g_i2c[0]);
        if (g_i2c[1].dev && g_i2c[1].phase != PH_IDLE) i2c_eng_run(&g_i2c[1]);
    }
    if (g_i2c_slv_on) { /* the scripted master on I2C1, and the IRQ line */
        mm_rp2350_i2c1_master_run();
        slv_update_irq(&g_i2c[1]);
    }
    if (g_pcs_every) pc_sample(cycles);
    if (!g_rtt_enabled) return;
    g_rtt_cycles += cycles;
    if (g_rtt_cycles < (1u << 18)) return; /* ~2 ms of virtual time */
    g_rtt_cycles = 0;
    rtt_drain();
}

void mm_rp2350_extra_flush(void)
{
    if (g_rtt_enabled) rtt_drain();
}

/* ------------------------------------------------------------------ */

mm_bool mm_rp2350_extra_register(struct mmio_bus *bus)
{
    struct mmio_region reg;
    size_t i;
    static const mm_u32 xip_alias[2] = {0x14000000u, 0x1c000000u}; /* 0x18000000 is XIP maintenance */
    for (i = 0; i < sizeof(g_stubs) / sizeof(g_stubs[0]); ++i) {
        memset(&reg, 0, sizeof(reg));
        reg.base = g_stubs[i].base;
        reg.size = g_stubs[i].size;
        reg.opaque = &g_stubs[i];
        reg.read = stub_read;
        reg.write = stub_write;
        if (!mmio_bus_register_region(bus, &reg)) {
            printf("[STUB] %s overlaps an existing model, not stubbed\n", g_stubs[i].name);
        }
    }
    for (i = 0; i < 2; ++i) {
        memset(&reg, 0, sizeof(reg));
        reg.base = i ? 0x40098000u : 0x40090000u;
        reg.size = 0x4000u;
        reg.opaque = &g_i2c[i];
        reg.read = i2c_read;
        reg.write = i2c_write;
        (void)mmio_bus_register_region(bus, &reg);
    }
    for (i = 0; i < 2; ++i) {
        memset(&reg, 0, sizeof(reg));
        reg.base = xip_alias[i];
        reg.size = 0x01000000u;
        reg.opaque = 0;
        reg.read = xip_alias_read;
        reg.write = xip_alias_write;
        (void)mmio_bus_register_region(bus, &reg);
    }
    g_i2c[0].reset_bit = 1u << 4;
    g_i2c[1].reset_bit = 1u << 5;
    if (mm_rp2350_pcal6416_enabled()) { /* slave model on one of the buses */
        g_i2c[mm_rp2350_pcal6416_bus() & 1u].dev = MM_TRUE;
        g_i2c_dev_any = MM_TRUE;
    }
    if (mm_rp2350_i2c1_master_enabled() && !g_i2c_slv_on) { /* I2C1 slave mode, external master */
        if (g_i2c[1].dev) {
            printf("[I2C] I2C1 already has the PCAL6416 model attached; M33MU_I2C1_MASTER ignored\n");
        } else {
            g_i2c[1].slv = MM_TRUE;
            g_i2c[1].s_state = SS_IDLE;
            slv_regs_reset(&g_i2c[1]);
            g_i2c_slv_on = MM_TRUE;
            printf("[I2C] I2C1: DW_apb_i2c slave mode model on, IRQ %u, bus driven by the scripted master\n", I2C1_IRQ);
        }
    }
    {
        const char *pcs = getenv("M33MU_PC_SAMPLE");
        const char *rtt = getenv("M33MU_RTT");
        if (pcs != 0 && pcs[0] != '\0') g_pcs_every = strtoull(pcs, 0, 0);
        if (rtt != 0 && rtt[0] != '\0' && !g_rtt_enabled) {
            if (strcmp(rtt, "-") == 0) g_rtt_out = stdout;
            else g_rtt_out = fopen(rtt, "wb");
            if (g_rtt_out == 0) g_rtt_out = stdout;
            g_rtt_enabled = MM_TRUE;
        }
    }
    return MM_TRUE;
}

void mm_rp2350_extra_flash_bind(struct mm_memmap *map, mm_u8 *flash, mm_u32 flash_size)
{
    g_map = map;
    g_flash = flash;
    g_flash_size = flash_size;
    if (!g_pt.loaded) pt_load_env();
    if (g_pt.loaded) {
        struct rp2350_partition_table *pt = mm_rp2350_partition_table_get_mut();
        mm_u32 n;
        if (pt != 0) {
            pt->partition_count = (mm_u8)g_pt.count;
            pt->permission_partition_count = (mm_u8)g_pt.count;
            pt->loaded = 1u;
            pt->unpartitioned_space_permissions_and_flags = g_pt.unpart;
            for (n = 0; n < g_pt.count && n < RP2350_PARTITION_TABLE_MAX_PARTITIONS; ++n) {
                pt->partitions[n].permissions_and_location = g_pt.loc[n];
                pt->partitions[n].permissions_and_flags = g_pt.flags[n];
            }
        }
    }
}

void mm_rp2350_extra_stats(void)
{
    pc_report();
    if (g_i2c[0].nacks || g_i2c[1].nacks) {
        fprintf(stderr, "[I2C] NACKed transfers: I2C0=%llu I2C1=%llu\n",
                (unsigned long long)g_i2c[0].nacks, (unsigned long long)g_i2c[1].nacks);
    }
    {
        mm_u32 i;
        for (i = 0; i < 2u; ++i) {
            const struct dw_i2c *c = &g_i2c[i];
            if (!c->dev) continue;
            fprintf(stderr, "[I2C] %s master engine: starts=%llu bytes_tx=%llu bytes_rx=%llu stops=%llu cut_short=%llu\n",
                    c->name, (unsigned long long)c->n_start, (unsigned long long)c->n_tx,
                    (unsigned long long)c->n_rx, (unsigned long long)c->n_stop, (unsigned long long)c->n_cut);
        }
    }
    if (g_i2c_slv_on) {
        const struct dw_i2c *c = &g_i2c[1];
        fprintf(stderr, "[I2C] I2C1 slave model: starts=%llu addressed=%llu other_addr=%llu bytes_rx=%llu bytes_tx=%llu "
                        "rd_req=%llu rx_done=%llu stops=%llu irq_pend=%llu tx_abrt=%llu rx_over=%llu tx_over=%llu "
                        "rx_under=%llu locked_writes=%llu\n",
                (unsigned long long)c->s_n_start, (unsigned long long)c->s_n_match, (unsigned long long)c->s_n_other,
                (unsigned long long)c->s_n_rx, (unsigned long long)c->s_n_tx, (unsigned long long)c->s_n_rd_req,
                (unsigned long long)c->s_n_rx_done, (unsigned long long)c->s_n_stop, (unsigned long long)c->s_n_irq,
                (unsigned long long)c->s_n_abrt, (unsigned long long)c->s_n_rx_over, (unsigned long long)c->s_n_tx_over,
                (unsigned long long)c->s_n_rx_under, (unsigned long long)c->s_n_locked);
        mm_rp2350_i2c1_master_stats();
    }
    if (g_probe.state > 0) {
        fprintf(stderr, "[PROBE] fn_enter=%llu fn_exit=%llu dma_pio=%llu\n", (unsigned long long)g_probe.n_enter,
                (unsigned long long)g_probe.n_exit, (unsigned long long)g_probe.n_dma);
        if (g_probe.log != 0 && g_probe.log != stdout) fflush(g_probe.log);
    }
}
