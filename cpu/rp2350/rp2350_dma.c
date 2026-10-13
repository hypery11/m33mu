/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * RP2350 DMA: 16 channels with the full alias register layout, chaining,
 * ring wrap, byte swap, INTR/INTE/INTF/INTS for the four DMA IRQ lines
 * (NVIC 10..13, level sensitive), CHAN_ABORT, MULTI_CHAN_TRIGGER and the
 * atomic register aliases.  (Replaces the channel-0-only model.)
 *
 * Timing is not modelled: a triggered channel runs to completion at once,
 * except that transfers to/from a PIO FIFO are paced by running the state
 * machine (mm_rp2350_pio_dreq_wait), as the original model did.  TIMER
 * pacing, sniffer CRC and the DMA MPU/SECCFG are register files only.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rp2350/rp2350_mmio.h"
#include "rp2350/rp2350_pio.h"
#include "rp2350/rp2350_dma.h"
#include "rp2350/rp2350_extra.h"
#include "m33mu/mmio.h"
#include "m33mu/memmap.h"

#define DMA_BASE 0x50000000u
#define DMA_ALIAS_SIZE 0x4000u
#define DMA_NCH 16u
#define DMA_IRQ0 10u

#define CTRL_EN        (1u << 0)
#define CTRL_INCR_R    (1u << 4)
#define CTRL_INCR_R_REV (1u << 5)
#define CTRL_INCR_W    (1u << 6)
#define CTRL_INCR_W_REV (1u << 7)
#define CTRL_RING_SEL  (1u << 12)
#define CTRL_IRQ_QUIET (1u << 23)
#define CTRL_BSWAP     (1u << 24)
#define CTRL_BUSY      (1u << 26)
#define CTRL_RW_MASK   0x03ffffffu  /* bits 25:0 writable */

struct dma_ch {
    mm_u32 read_addr;
    mm_u32 write_addr;
    mm_u32 count;       /* current (remaining) count */
    mm_u32 count_reload;/* TRANS_COUNT as written (mode in 31:28) */
    mm_u32 ctrl;
    mm_u64 transfers;
};

static struct {
    struct dma_ch ch[DMA_NCH];
    mm_u32 intr;
    mm_u32 inte[4];
    mm_u32 intf[4];
    mm_u32 misc[0x1000u / 4u]; /* everything else: plain register file */
    struct mm_nvic *nvic;
    int depth;
    int trace;
} g_dma;

static mm_bool dma_trace(void)
{
    if (g_dma.trace == 0) {
        const char *v = getenv("M33MU_DMA_TRACE");
        g_dma.trace = (v != 0 && v[0] != '\0') ? 1 : -1;
    }
    return g_dma.trace > 0 ? MM_TRUE : MM_FALSE;
}

static void dma_update_irq(void)
{
    mm_u32 i;
    if (g_dma.nvic == 0) return;
    for (i = 0; i < 4u; ++i) {
        mm_u32 ints = (g_dma.intr & g_dma.inte[i]) | g_dma.intf[i];
        mm_u32 irq = DMA_IRQ0 + i;
        mm_u32 word = irq / 32u, mask = 1u << (irq % 32u);
        if (ints == 0u) continue;
        if ((g_dma.nvic->active_mask[word] & mask) != 0u) continue;
        if ((g_dma.nvic->pending_mask[word] & mask) != 0u) continue;
        mm_nvic_set_pending(g_dma.nvic, irq, MM_TRUE);
    }
}

mm_bool mm_rp2350_dma_line(mm_u32 irq)
{
    mm_u32 i;
    if (irq < DMA_IRQ0 || irq > DMA_IRQ0 + 3u) return MM_FALSE;
    i = irq - DMA_IRQ0;
    return ((g_dma.intr & g_dma.inte[i]) | g_dma.intf[i]) != 0u ? MM_TRUE : MM_FALSE;
}

void mm_rp2350_dma_poll(void)
{
    /* level-sensitive lines: re-pend after the handler returned */
    dma_update_irq();
}

static mm_u32 bswap_val(mm_u32 v, mm_u32 step)
{
    if (step == 4u) return ((v & 0xffu) << 24) | ((v & 0xff00u) << 8) | ((v >> 8) & 0xff00u) | (v >> 24);
    if (step == 2u) return ((v & 0xffu) << 8) | ((v >> 8) & 0xffu);
    return v;
}

static mm_u32 next_addr(mm_u32 addr, mm_u32 step, mm_bool incr, mm_bool rev, mm_u32 ring, mm_bool ring_here)
{
    mm_u32 n;
    if (!incr) return addr;
    n = rev ? addr - step : addr + step;
    if (ring_here && ring != 0u) {
        mm_u32 m = (1u << ring) - 1u;
        n = (addr & ~m) | (n & m);
    }
    return n;
}

static void dma_trigger(mm_u32 c);

static void dma_run(mm_u32 c)
{
    struct dma_ch *ch = &g_dma.ch[c];
    struct mm_memmap *map = mm_memmap_current();
    mm_u32 size_sel = (ch->ctrl >> 2) & 3u;
    mm_u32 step = (size_sel == 0u) ? 1u : (size_sel == 1u) ? 2u : 4u;
    mm_bool ir = (ch->ctrl & CTRL_INCR_R) != 0u, irr = (ch->ctrl & CTRL_INCR_R_REV) != 0u;
    mm_bool iw = (ch->ctrl & CTRL_INCR_W) != 0u, iwr = (ch->ctrl & CTRL_INCR_W_REV) != 0u;
    mm_u32 ring = (ch->ctrl >> 8) & 0xfu;
    mm_bool ring_w = (ch->ctrl & CTRL_RING_SEL) != 0u;
    mm_u32 mode = (ch->count_reload >> 28) & 0xfu;
    mm_u32 n = ch->count & 0x0fffffffu;
    mm_u32 chain = (ch->ctrl >> 13) & 0xfu;
    mm_u32 i;
    /* M33MU_PROBE_DMA_PIO, destination PIOn TXFm (0x502/503/504x_xx10..1c) */
    mm_u32 probe_dst = ch->write_addr;
    mm_u32 probe_words[32];
    mm_u32 probe_n = 0u;
    mm_bool probe = MM_FALSE;
    if (mm_rp2350_probe_dma_on()) {
        mm_u32 blk = probe_dst & 0xfff00000u;
        mm_u32 off = probe_dst & 0x000fffffu;
        probe = ((blk == 0x50200000u || blk == 0x50300000u || blk == 0x50400000u) && off >= 0x10u && off < 0x20u)
                    ? MM_TRUE : MM_FALSE;
    }

    if (mode == 0xfu) {
        /* endless: run one reload's worth so the CPU makes progress */
        n = ch->count_reload & 0x0fffffffu;
    }
    if (dma_trace()) {
        printf("[DMA] ch%u run %u x %uB %08x -> %08x ctrl=%08x\n", c, n, step, ch->read_addr, ch->write_addr, ch->ctrl);
    }
    ch->ctrl |= CTRL_BUSY;
    for (i = 0; i < n && map != 0; ++i) {
        mm_u32 v = 0u;
        (void)mm_rp2350_pio_dreq_wait(ch->read_addr, MM_FALSE);
        (void)mm_rp2350_pio_dreq_wait(ch->write_addr, MM_TRUE);
        (void)mm_memmap_read(map, MM_SECURE, ch->read_addr, step, &v);
        if (ch->ctrl & CTRL_BSWAP) v = bswap_val(v, step);
        if (probe && probe_n < 32u) probe_words[probe_n++] = v;
        (void)mm_memmap_write(map, MM_SECURE, ch->write_addr, step, v);
        ch->read_addr = next_addr(ch->read_addr, step, ir, irr, ring, !ring_w);
        ch->write_addr = next_addr(ch->write_addr, step, iw, iwr, ring, ring_w);
        if ((ch->ctrl & CTRL_EN) == 0u) break; /* aborted from a side effect */
    }
    ch->transfers += i;
    if (probe) mm_rp2350_probe_dma(c, probe_dst, step, probe_words, probe_n, i);
    ch->count = (mode == 0xfu) ? ch->count : 0u;
    ch->ctrl &= ~CTRL_BUSY;
    if ((ch->ctrl & CTRL_IRQ_QUIET) == 0u) {
        g_dma.intr |= 1u << c;
        dma_update_irq();
    }
    if (mode == 1u) {
        dma_trigger(c); /* TRIGGER_SELF */
    } else if (chain != c) {
        dma_trigger(chain);
    }
}

static void dma_trigger(mm_u32 c)
{
    struct dma_ch *ch = &g_dma.ch[c & 0xfu];
    if ((ch->ctrl & CTRL_EN) == 0u) return;
    if (g_dma.depth > 64) return; /* runaway chain/self-trigger guard */
    ch->count = ch->count_reload & 0x0fffffffu;
    g_dma.depth++;
    dma_run(c & 0xfu);
    g_dma.depth--;
}

static mm_u32 ch_read(mm_u32 c, mm_u32 r)
{
    struct dma_ch *ch = &g_dma.ch[c];
    switch (r) {
    case 0x00: case 0x14: case 0x28: case 0x3c: return ch->read_addr;
    case 0x04: case 0x18: case 0x2c: case 0x34: return ch->write_addr;
    case 0x08: case 0x1c: case 0x24: case 0x38: return (ch->count_reload & 0xf0000000u) | (ch->count & 0x0fffffffu);
    default: return ch->ctrl; /* 0x0c 0x10 0x20 0x30 */
    }
}

static void ch_write(mm_u32 c, mm_u32 r, mm_u32 v)
{
    struct dma_ch *ch = &g_dma.ch[c];
    mm_bool trig = MM_FALSE;
    switch (r) {
    case 0x00: case 0x14: case 0x28: ch->read_addr = v; break;
    case 0x3c: ch->read_addr = v; trig = MM_TRUE; break;
    case 0x04: case 0x18: case 0x34: ch->write_addr = v; break;
    case 0x2c: ch->write_addr = v; trig = MM_TRUE; break;
    case 0x08: case 0x24: case 0x38: ch->count_reload = v; break;
    case 0x1c: ch->count_reload = v; trig = MM_TRUE; break;
    case 0x0c: ch->ctrl = (ch->ctrl & ~CTRL_RW_MASK) | (v & CTRL_RW_MASK); trig = MM_TRUE; break;
    default: ch->ctrl = (ch->ctrl & ~CTRL_RW_MASK) | (v & CTRL_RW_MASK); break;
    }
    /* a zero written to a trigger register is a null trigger */
    if (trig && v != 0u) dma_trigger(c);
}

static mm_u32 dma_reg_read(mm_u32 off)
{
    if (off < 0x400u) return ch_read(off / 0x40u, off & 0x3cu);
    switch (off) {
    case 0x400: case 0x410: case 0x420: case 0x430: return g_dma.intr;
    case 0x404: return g_dma.inte[0];
    case 0x414: return g_dma.inte[1];
    case 0x424: return g_dma.inte[2];
    case 0x434: return g_dma.inte[3];
    case 0x408: return g_dma.intf[0];
    case 0x418: return g_dma.intf[1];
    case 0x428: return g_dma.intf[2];
    case 0x438: return g_dma.intf[3];
    case 0x40c: return (g_dma.intr & g_dma.inte[0]) | g_dma.intf[0];
    case 0x41c: return (g_dma.intr & g_dma.inte[1]) | g_dma.intf[1];
    case 0x42c: return (g_dma.intr & g_dma.inte[2]) | g_dma.intf[2];
    case 0x43c: return (g_dma.intr & g_dma.inte[3]) | g_dma.intf[3];
    case 0x464: return 0u;            /* CHAN_ABORT: nothing in flight */
    case 0x468: return DMA_NCH;       /* N_CHANNELS */
    default:
        if (off >= 0x800u && off < 0xc00u) {
            mm_u32 c = (off - 0x800u) / 0x40u;
            if ((off & 0x3fu) == 0x04u) return g_dma.ch[c].count_reload & 0x0fffffffu; /* DBG_TCR */
            return 0u;
        }
        return g_dma.misc[(off & 0xfffu) / 4u];
    }
}

static void dma_reg_write(mm_u32 off, mm_u32 v)
{
    mm_u32 i;
    if (off < 0x400u) { ch_write(off / 0x40u, off & 0x3cu, v); return; }
    switch (off) {
    case 0x400: case 0x410: case 0x420: case 0x430:
    case 0x40c: case 0x41c: case 0x42c: case 0x43c:
        g_dma.intr &= ~v; break; /* write-1-to-clear */
    case 0x404: g_dma.inte[0] = v & 0xffffu; break;
    case 0x414: g_dma.inte[1] = v & 0xffffu; break;
    case 0x424: g_dma.inte[2] = v & 0xffffu; break;
    case 0x434: g_dma.inte[3] = v & 0xffffu; break;
    case 0x408: g_dma.intf[0] = v & 0xffffu; break;
    case 0x418: g_dma.intf[1] = v & 0xffffu; break;
    case 0x428: g_dma.intf[2] = v & 0xffffu; break;
    case 0x438: g_dma.intf[3] = v & 0xffffu; break;
    case 0x450: /* MULTI_CHAN_TRIGGER */
        for (i = 0; i < DMA_NCH; ++i) if (v & (1u << i)) dma_trigger(i);
        break;
    case 0x464: /* CHAN_ABORT */
        for (i = 0; i < DMA_NCH; ++i) if (v & (1u << i)) { g_dma.ch[i].count = 0u; g_dma.ch[i].ctrl &= ~CTRL_BUSY; }
        break;
    default:
        g_dma.misc[(off & 0xfffu) / 4u] = v; break;
    }
    dma_update_irq();
}

static mm_bool dma_mmio_read(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 *value_out)
{
    mm_u32 off = offset & 0xfffu;
    mm_u32 v;
    (void)opaque;
    if (value_out == 0 || size_bytes == 0u || size_bytes > 4u) return MM_FALSE;
    v = dma_reg_read(off & ~3u) >> ((off & 3u) * 8u);
    if (size_bytes == 1u) v &= 0xffu;
    else if (size_bytes == 2u) v &= 0xffffu;
    *value_out = v;
    return MM_TRUE;
}

static mm_bool dma_mmio_write(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 value)
{
    mm_u32 alias = (offset >> 12) & 3u;
    mm_u32 reg = offset & 0xffcu;
    (void)opaque;
    if (size_bytes == 0u || size_bytes > 4u) return MM_FALSE;
    if (size_bytes == 1u) value = (value & 0xffu) * 0x01010101u;
    else if (size_bytes == 2u) value = (value & 0xffffu) * 0x00010001u;
    if (alias != 0u) {
        mm_u32 cur = dma_reg_read(reg);
        /* write-1-to-clear registers see the mask itself */
        if (reg == 0x400u || reg == 0x410u || reg == 0x420u || reg == 0x430u) cur = 0u;
        if (alias == 1u) value = cur ^ value;
        else if (alias == 2u) value = cur | value;
        else value = cur & ~value;
    }
    dma_reg_write(reg, value);
    return MM_TRUE;
}

mm_bool mm_rp2350_dma_register(struct mmio_bus *bus)
{
    struct mmio_region reg;
    memset(&g_dma, 0, sizeof(g_dma));
    memset(&reg, 0, sizeof(reg));
    reg.base = DMA_BASE;
    reg.size = DMA_ALIAS_SIZE;
    reg.read = dma_mmio_read;
    reg.write = dma_mmio_write;
    return mmio_bus_register_region(bus, &reg);
}

void mm_rp2350_dma_set_nvic(struct mm_nvic *nvic)
{
    g_dma.nvic = nvic;
}

void mm_rp2350_dma_stats(void)
{
    mm_u32 i;
    for (i = 0; i < DMA_NCH; ++i) {
        if (g_dma.ch[i].transfers) {
            fprintf(stderr, "[DMA] ch%u: %llu transfers\n", i, (unsigned long long)g_dma.ch[i].transfers);
        }
    }
}
