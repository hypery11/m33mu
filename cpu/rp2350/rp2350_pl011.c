/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * RP2350 UART0/UART1: ARM PL011 model with FIFOs, baud-rate timing and
 * level-sensitive interrupts (replaces the flag-only model).
 *
 * Modelled:
 *   - 32-entry TX and RX FIFOs (1 entry when LCR_H.FEN = 0), 12-bit RX
 *     entries carrying FE/PE/BE/OE.
 *   - Virtual-time line rate: one character = (1 start + data + parity +
 *     stop) bit periods, bit period = 16 * (IBRD + FBRD/64) clk_peri cycles,
 *     converted to emulated core cycles with M33MU_RP2350_CLK_PERI_HZ
 *     (default 150 MHz, the pico-sdk default clk_peri).
 *   - RX: the host side (pty/stdio) is sampled once per character period; a
 *     character arriving while the RX FIFO is full is lost and raises OE
 *     (no flow control, like a real 2-wire link).
 *   - RXRIS: level >= IFLS.RXIFLSEL trigger (4/8/16/24/28), cleared by
 *     reading below it.  RTRIS: RX FIFO non-empty and no character received
 *     or read for 32 bit periods; cleared by draining the FIFO or ICR.RTIC.
 *   - TXRIS: set when the TX FIFO drains through IFLS.TXIFLSEL (transition,
 *     not level, as in the PL011 TRM), cleared by filling above it or ICR.
 *   - FR: RXFE/RXFF/TXFE/TXFF/BUSY.  RSR/ECR.  MIS = RIS & IMSC.
 *   - UARTINTR is a level: whenever MIS != 0 and the NVIC line is not active
 *     the IRQ (33 / 34) is pended again, so an ISR that returns without
 *     clearing its cause is re-entered (same as the real NVIC with a level
 *     interrupt).
 *   - Atomic register aliases (+0x1000 XOR, +0x2000 SET, +0x3000 CLR).
 * Not modelled: modem lines, IrDA, DMA requests, break generation, exact
 * PL011 clock-domain-crossing latencies.
 */
#define _DEFAULT_SOURCE 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "rp2350/rp2350_uart_spi.h"
#include "rp2350/rp2350_mmio.h"
#include "rp2350/rp2350_pl011.h"
#include "rp2350/rp2350_usb.h"
#include "rp2350/rp2350_extra.h"
#include "rp2350/rp2350_pio.h"
#include "rp2350/rp2350_ina219.h"
#include "rp2350/rp2350_dma.h"
#include "m33mu/mmio.h"
#include "m33mu/target_hal.h"

#define UART0_BASE 0x40070000u
#define UART1_BASE 0x40078000u
#define UART_ALIAS_SIZE 0x4000u
#define UART0_IRQ 33u
#define UART1_IRQ 34u

#define UARTDR    0x000u
#define UARTRSR   0x004u
#define UARTFR    0x018u
#define UARTILPR  0x020u
#define UARTIBRD  0x024u
#define UARTFBRD  0x028u
#define UARTLCR_H 0x02cu
#define UARTCR    0x030u
#define UARTIFLS  0x034u
#define UARTIMSC  0x038u
#define UARTRIS   0x03cu
#define UARTMIS   0x040u
#define UARTICR   0x044u
#define UARTDMACR 0x048u

#define CR_UARTEN (1u << 0)
#define CR_TXE    (1u << 8)
#define CR_RXE    (1u << 9)

#define LCR_PEN  (1u << 1)
#define LCR_STP2 (1u << 3)
#define LCR_FEN  (1u << 4)

#define FR_BUSY (1u << 3)
#define FR_RXFE (1u << 4)
#define FR_TXFF (1u << 5)
#define FR_RXFF (1u << 6)
#define FR_TXFE (1u << 7)

#define INT_RX (1u << 4)
#define INT_TX (1u << 5)
#define INT_RT (1u << 6)
#define INT_FE (1u << 7)
#define INT_PE (1u << 8)
#define INT_BE (1u << 9)
#define INT_OE (1u << 10)
#define INT_ALL 0x7ffu

#define DR_OE (1u << 11)

#define FIFO_DEPTH 32u

struct pl011 {
    mm_u32 base;
    mm_u32 reset_bit;
    mm_u32 irq;
    char label[8];
    struct mm_uart_io io;
    mm_bool io_open;
    FILE *txlog;

    /* registers */
    mm_u32 ibrd, fbrd, lcr_h, cr, ifls, imsc, ris, rsr, dmacr, ilpr;

    mm_u16 rxf[FIFO_DEPTH];
    mm_u32 rx_head, rx_count;
    mm_u8 txf[FIFO_DEPTH];
    mm_u32 tx_head, tx_count;

    mm_bool tx_shifting;     /* a character is on the wire */
    mm_u8 tx_shift;
    mm_u64 tx_done_at;       /* virtual cycle the shifting char completes */
    mm_u64 rx_next_at;       /* next host sample point */
    mm_u64 rx_last_at;       /* last RX FIFO activity (for RT) */
    mm_u32 rx_idle_polls;

    /* statistics */
    mm_u64 st_rx, st_tx, st_overrun, st_irq_pend;
};

static struct pl011 g_uart[2];
static struct mm_nvic *g_nvic = 0;
static mm_u64 g_now = 0;          /* virtual core cycles */
static mm_u64 g_next_event = 0;
static mm_u64 g_clk_peri_hz = 150000000ull;
static int g_trace = -1;

static mm_bool trace_on(void)
{
    if (g_trace < 0) {
        const char *v = getenv("M33MU_PL011_TRACE");
        g_trace = (v != 0 && v[0] != '\0') ? 1 : 0;
    }
    return g_trace ? MM_TRUE : MM_FALSE;
}

static mm_u32 fifo_depth(const struct pl011 *u)
{
    return (u->lcr_h & LCR_FEN) ? FIFO_DEPTH : 1u;
}

static mm_u32 trig_level(mm_u32 sel)
{
    switch (sel & 7u) {
    case 0: return 4u;
    case 1: return 8u;
    case 2: return 16u;
    case 3: return 24u;
    default: return 28u;
    }
}

static mm_u64 bit_cycles(const struct pl011 *u)
{
    /* divisor = IBRD + FBRD/64; bit = 16 * divisor clk_peri cycles */
    mm_u64 div64 = ((mm_u64)(u->ibrd & 0xffffu) << 6) | (u->fbrd & 0x3fu);
    mm_u64 hz = mm_rp2350_cpu_hz();
    mm_u64 c;
    if (div64 == 0u) div64 = 64u;
    c = (16u * div64 * hz) / (64u * (g_clk_peri_hz ? g_clk_peri_hz : hz));
    return c ? c : 1u;
}

static mm_u64 char_cycles(const struct pl011 *u)
{
    mm_u32 wlen = 5u + ((u->lcr_h >> 5) & 3u);
    mm_u32 bits = 1u + wlen + ((u->lcr_h & LCR_PEN) ? 1u : 0u) + ((u->lcr_h & LCR_STP2) ? 2u : 1u);
    return bits * bit_cycles(u);
}

static mm_bool clock_on(const struct pl011 *u)
{
    if (!mm_rp2350_clock_peri_enabled()) return MM_FALSE;
    if (mm_rp2350_reset_asserted(u->reset_bit)) return MM_FALSE;
    return MM_TRUE;
}

static void regs_reset(struct pl011 *u)
{
    u->ibrd = u->fbrd = u->lcr_h = 0u;
    u->cr = CR_TXE | CR_RXE;
    u->ifls = 0x12u;
    u->imsc = u->ris = u->rsr = u->dmacr = u->ilpr = 0u;
    u->rx_head = u->rx_count = 0u;
    u->tx_head = u->tx_count = 0u;
    u->tx_shifting = MM_FALSE;
}

static void update_irq(struct pl011 *u)
{
    mm_u32 mis;
    mm_u32 word, mask;
    if (g_nvic == 0) return;
    mis = u->ris & u->imsc;
    if (mis == 0u) return;
    word = u->irq / 32u;
    mask = 1u << (u->irq % 32u);
    if ((g_nvic->active_mask[word] & mask) != 0u) return;
    if ((g_nvic->pending_mask[word] & mask) != 0u) return;
    mm_nvic_set_pending(g_nvic, u->irq, MM_TRUE);
    u->st_irq_pend++;
}

static void update_rx_level(struct pl011 *u)
{
    mm_u32 trig = (u->lcr_h & LCR_FEN) ? trig_level((u->ifls >> 3) & 7u) : 1u;
    if (u->rx_count >= trig) u->ris |= INT_RX;
    else u->ris &= ~INT_RX;
    if (u->rx_count == 0u) u->ris &= ~INT_RT;
}

static void io_ensure(struct pl011 *u)
{
    mm_bool want = ((u->cr & CR_UARTEN) != 0u) && clock_on(u);
    if (want && !u->io_open) {
        if (mm_uart_io_open(&u->io, u->base)) {
            const char *pfx = getenv("M33MU_UART_LINK_PREFIX");
            const char *lpfx = getenv("M33MU_UART_LOG_PREFIX");
            if (mm_tui_is_active()) {
                mm_tui_attach_uart(u->label, u->io.name);
            }
            if (pfx != 0 && pfx[0] != '\0' && u->io.name[0] == '/') {
                char link[256];
                snprintf(link, sizeof(link), "%s%s", pfx, u->label);
                (void)unlink(link);
                if (symlink(u->io.name, link) == 0) {
                    printf("[PL011] %s -> %s\n", link, u->io.name);
                }
            }
            if (lpfx != 0 && lpfx[0] != '\0' && u->txlog == 0) {
                char path[256];
                snprintf(path, sizeof(path), "%s%s.txlog", lpfx, u->label);
                u->txlog = fopen(path, "wb");
            }
            fflush(stdout);
        }
        u->io_open = MM_TRUE;
    }
}

static void rx_push(struct pl011 *u, mm_u8 byte)
{
    mm_u32 depth = fifo_depth(u);
    u->st_rx++;
    if (u->rx_count >= depth) {
        /* overrun: character lost, OE flagged on the newest entry */
        mm_u32 last = (u->rx_head + u->rx_count - 1u) % FIFO_DEPTH;
        u->rxf[last] |= (mm_u16)DR_OE;
        u->rsr |= 8u;
        u->ris |= INT_OE;
        u->st_overrun++;
        if (trace_on()) printf("[PL011 %s] RX overrun, byte 0x%02x lost\n", u->label, byte);
        return;
    }
    u->rxf[(u->rx_head + u->rx_count) % FIFO_DEPTH] = byte;
    u->rx_count++;
    u->rx_last_at = g_now;
    update_rx_level(u);
}

static void tx_start_next(struct pl011 *u)
{
    if (u->tx_shifting || u->tx_count == 0u) return;
    if ((u->cr & (CR_UARTEN | CR_TXE)) != (CR_UARTEN | CR_TXE)) return;
    {
        mm_u32 trig = trig_level(u->ifls & 7u);
        mm_u32 before = u->tx_count;
        u->tx_shift = u->txf[u->tx_head];
        u->tx_head = (u->tx_head + 1u) % FIFO_DEPTH;
        u->tx_count--;
        u->tx_shifting = MM_TRUE;
        u->tx_done_at = g_now + char_cycles(u);
        /* TX interrupt on the transition through the trigger level */
        if (u->lcr_h & LCR_FEN) {
            if (before > trig && u->tx_count <= trig) u->ris |= INT_TX;
        } else if (u->tx_count == 0u) {
            u->ris |= INT_TX;
        }
    }
}

static void tx_finish(struct pl011 *u)
{
    u->tx_shifting = MM_FALSE;
    u->st_tx++;
    if (u->io_open) {
        mm_uart_io_queue_tx(&u->io, u->tx_shift);
        (void)mm_uart_io_flush(&u->io);
    }
    if (u->txlog) {
        fputc(u->tx_shift, u->txlog);
        fflush(u->txlog);
    }
}

static void recompute_next_event(void)
{
    mm_u64 next = (mm_u64)-1;
    int i;
    for (i = 0; i < 2; ++i) {
        struct pl011 *u = &g_uart[i];
        if (!u->io_open) continue;
        if (u->tx_shifting && u->tx_done_at < next) next = u->tx_done_at;
        if ((u->cr & (CR_UARTEN | CR_RXE)) == (CR_UARTEN | CR_RXE)) {
            if (u->rx_next_at < next) next = u->rx_next_at;
            if (u->rx_count != 0u && (u->ris & INT_RT) == 0u) {
                mm_u64 rt = u->rx_last_at + 32u * bit_cycles(u);
                if (rt < next) next = rt;
            }
        }
    }
    g_next_event = next;
}

static void service(struct pl011 *u)
{
    if (!u->io_open) return;
    if (!clock_on(u)) return;
    /* TX */
    if (u->tx_shifting && g_now >= u->tx_done_at) {
        tx_finish(u);
    }
    tx_start_next(u);
    /* RX: sample the host once per character time */
    if ((u->cr & (CR_UARTEN | CR_RXE)) == (CR_UARTEN | CR_RXE)) {
        if (g_now >= u->rx_next_at) {
            mm_u64 cc = char_cycles(u);
            if (!mm_uart_io_has_rx(&u->io)) {
                (void)mm_uart_io_poll(&u->io);
            }
            if (mm_uart_io_has_rx(&u->io)) {
                rx_push(u, mm_uart_io_read(&u->io));
                u->rx_idle_polls = 0u;
                u->rx_next_at = g_now + cc;
            } else {
                /* idle line: sample less often (still well under 1 ms) */
                if (u->rx_idle_polls < 64u) u->rx_idle_polls++;
                u->rx_next_at = g_now + cc * (u->rx_idle_polls < 8u ? 1u : 8u);
            }
        }
        if (u->rx_count != 0u && (u->ris & INT_RT) == 0u &&
            g_now >= u->rx_last_at + 32u * bit_cycles(u)) {
            u->ris |= INT_RT;
        }
    }
    update_irq(u);
}

void mm_rp2350_pl011_tick(mm_u64 cycles)
{
    g_now += cycles;
    if (g_now < g_next_event) return;
    service(&g_uart[0]);
    service(&g_uart[1]);
    recompute_next_event();
}

static mm_u32 read_reg(struct pl011 *u, mm_u32 off)
{
    switch (off) {
    case UARTDR: {
        mm_u32 v = 0u;
        if (u->rx_count != 0u) {
            v = u->rxf[u->rx_head];
            if (!mmio_peek_mode()) {
                u->rx_head = (u->rx_head + 1u) % FIFO_DEPTH;
                u->rx_count--;
                u->rx_last_at = g_now;
                u->rsr = (v >> 8) & 0xfu;
                update_rx_level(u);
                recompute_next_event();
            }
        }
        return v;
    }
    case UARTRSR: return u->rsr;
    case UARTFR: {
        mm_u32 fr = 0u;
        mm_u32 depth = fifo_depth(u);
        if (u->rx_count == 0u) fr |= FR_RXFE;
        if (u->rx_count >= depth) fr |= FR_RXFF;
        if (u->tx_count == 0u) fr |= FR_TXFE;
        if (u->tx_count >= depth) fr |= FR_TXFF;
        if (u->tx_count != 0u || u->tx_shifting) fr |= FR_BUSY;
        return fr;
    }
    case UARTILPR: return u->ilpr;
    case UARTIBRD: return u->ibrd;
    case UARTFBRD: return u->fbrd;
    case UARTLCR_H: return u->lcr_h;
    case UARTCR: return u->cr;
    case UARTIFLS: return u->ifls;
    case UARTIMSC: return u->imsc;
    case UARTRIS: return u->ris;
    case UARTMIS: return u->ris & u->imsc;
    case UARTDMACR: return u->dmacr;
    case 0xfe0u: return 0x11u;
    case 0xfe4u: return 0x10u;
    case 0xfe8u: return 0x34u;
    case 0xfecu: return 0x00u;
    case 0xff0u: return 0x0du;
    case 0xff4u: return 0xf0u;
    case 0xff8u: return 0x05u;
    case 0xffcu: return 0xb1u;
    default: return 0u;
    }
}

static void write_reg(struct pl011 *u, mm_u32 off, mm_u32 v)
{
    switch (off) {
    case UARTDR:
        if (u->tx_count < fifo_depth(u)) {
            mm_u32 trig = trig_level(u->ifls & 7u);
            u->txf[(u->tx_head + u->tx_count) % FIFO_DEPTH] = (mm_u8)v;
            u->tx_count++;
            if ((u->lcr_h & LCR_FEN) ? (u->tx_count > trig) : MM_TRUE) {
                u->ris &= ~INT_TX;
            }
        }
        tx_start_next(u);
        break;
    case UARTRSR: u->rsr = 0u; break;
    case UARTILPR: u->ilpr = v & 0xffu; break;
    case UARTIBRD: u->ibrd = v & 0xffffu; break;
    case UARTFBRD: u->fbrd = v & 0x3fu; break;
    case UARTLCR_H:
        if (((u->lcr_h ^ v) & LCR_FEN) != 0u) {
            /* FIFO enable change flushes the FIFOs */
            u->rx_head = u->rx_count = 0u;
            u->tx_head = u->tx_count = 0u;
        }
        u->lcr_h = v & 0xffu;
        update_rx_level(u);
        break;
    case UARTCR: u->cr = v & 0xff87u; break;
    case UARTIFLS: u->ifls = v & 0x3fu; update_rx_level(u); break;
    case UARTIMSC: u->imsc = v & INT_ALL; break;
    case UARTICR: u->ris &= ~(v & INT_ALL); update_rx_level(u);
        /* RX is a level: it comes straight back if the FIFO is still at
         * or above the trigger level */
        break;
    case UARTDMACR: u->dmacr = v & 7u; break;
    default: break;
    }
    io_ensure(u);
    if (off == UARTCR && (v & CR_UARTEN) == 0u) {
        /* nothing: FIFO contents are kept, like the PL011 */
    }
    update_irq(u);
    recompute_next_event();
}

static mm_bool pl011_read(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 *value_out)
{
    struct pl011 *u = (struct pl011 *)opaque;
    mm_u32 off = offset & 0xfffu;
    mm_u32 v;
    if (u == 0 || value_out == 0 || size_bytes == 0u || size_bytes > 4u) return MM_FALSE;
    if (offset >= UART_ALIAS_SIZE) return MM_FALSE;
    if (!clock_on(u)) { *value_out = 0u; return MM_TRUE; }
    v = read_reg(u, off & ~3u);
    v >>= (off & 3u) * 8u;
    if (size_bytes == 1u) v &= 0xffu;
    else if (size_bytes == 2u) v &= 0xffffu;
    *value_out = v;
    if (!mmio_peek_mode()) update_irq(u);
    return MM_TRUE;
}

static mm_bool pl011_write(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 value)
{
    struct pl011 *u = (struct pl011 *)opaque;
    mm_u32 alias = (offset >> 12) & 3u;
    mm_u32 off = offset & 0xfffu;
    mm_u32 reg = off & ~3u;
    mm_u32 v;
    if (u == 0 || size_bytes == 0u || size_bytes > 4u) return MM_FALSE;
    if (offset >= UART_ALIAS_SIZE) return MM_FALSE;
    if (!clock_on(u)) return MM_TRUE;
    /* RP2350 APB: narrow writes are replicated across the word */
    if (size_bytes == 1u) value = (value & 0xffu) * 0x01010101u;
    else if (size_bytes == 2u) value = (value & 0xffffu) * 0x00010001u;
    if (alias == 0u) {
        v = value;
    } else {
        /* atomic aliases are read-modify-write of the plain register;
         * they never pop the RX FIFO or push the TX FIFO */
        mm_u32 cur;
        if (reg == UARTDR || reg == UARTICR || reg == UARTRSR) {
            cur = 0u;
        } else {
            cur = read_reg(u, reg);
        }
        if (alias == 1u) v = cur ^ value;
        else if (alias == 2u) v = cur | value;
        else v = cur & ~value;
    }
    if (trace_on() && reg != UARTDR) {
        printf("[PL011 %s] wr %03x <= %08x (alias %u)\n", u->label, reg, v, alias);
    }
    write_reg(u, reg, v);
    return MM_TRUE;
}

void mm_rp2350_exit_report(void)
{
    static int done = 0;
    if (done) return;
    done = 1;
    mm_rp2350_extra_flush();
    mm_rp2350_pl011_stats();
    mm_rp2350_extra_stats();
    mm_rp2350_ina219_stats();
    mm_rp2350_dma_stats();
    if (getenv("M33MU_PIO_DUMP")) mm_rp2350_pio_dump();
    fflush(stdout);
    fflush(stderr);
}

static void pl011_atexit(void)
{
    mm_rp2350_exit_report();
}

static void pl011_sigterm(int sig)
{
    (void)sig;
    mm_rp2350_exit_report();
    _exit(143);
}

static mm_bool rp2350_irq_level(mm_u32 irq)
{
    if (irq == UART0_IRQ) return (g_uart[0].ris & g_uart[0].imsc) != 0u ? MM_TRUE : MM_FALSE;
    if (irq == UART1_IRQ) return (g_uart[1].ris & g_uart[1].imsc) != 0u ? MM_TRUE : MM_FALSE;
    return mm_rp2350_dma_line(irq);
}

void mm_rp2350_uart_init(struct mmio_bus *bus, struct mm_nvic *nvic)
{
    static int atexit_done = 0;
    struct mmio_region reg;
    int i;
    const char *clk = getenv("M33MU_RP2350_CLK_PERI_HZ");
    mm_rp2350_usb_set_nvic(nvic);
    g_nvic = nvic;
    mm_rp2350_dma_set_nvic(nvic);
    mm_nvic_set_level_source(rp2350_irq_level);
    if (!atexit_done) { atexit_done = 1; atexit(pl011_atexit); signal(SIGTERM, pl011_sigterm); signal(SIGINT, pl011_sigterm); }
    if (clk != 0 && clk[0] != '\0') {
        g_clk_peri_hz = strtoull(clk, 0, 0);
    }
    if (bus == 0) return;
    for (i = 0; i < 2; ++i) {
        struct pl011 *u = &g_uart[i];
        memset(u, 0, sizeof(*u));
        u->base = i ? UART1_BASE : UART0_BASE;
        u->reset_bit = i ? RP2350_RESET_UART1 : RP2350_RESET_UART0;
        u->irq = i ? UART1_IRQ : UART0_IRQ;
        snprintf(u->label, sizeof(u->label), "UART%d", i);
        mm_uart_instance_register(u->base, u->label);
        mm_uart_io_init(&u->io);
        regs_reset(u);
        memset(&reg, 0, sizeof(reg));
        reg.base = u->base;
        reg.size = UART_ALIAS_SIZE;
        reg.opaque = u;
        reg.read = pl011_read;
        reg.write = pl011_write;
        (void)mmio_bus_register_region(bus, &reg);
    }
}

void mm_rp2350_uart_reset(void)
{
    int i;
    for (i = 0; i < 2; ++i) {
        struct pl011 *u = &g_uart[i];
        regs_reset(u);
        if (u->io_open) {
            mm_uart_io_close(&u->io);
            u->io_open = MM_FALSE;
        }
    }
    g_next_event = 0;
}

void mm_rp2350_uart_poll(void)
{
    int i;
    for (i = 0; i < 2; ++i) {
        struct pl011 *u = &g_uart[i];
        if (mm_rp2350_reset_asserted(u->reset_bit) && (u->cr != (CR_TXE | CR_RXE) || u->lcr_h != 0u)) {
            regs_reset(u);
        }
        if (u->io_open) {
            (void)mm_uart_io_flush(&u->io);
        }
        /* level-sensitive line: re-pend after the ISR returned */
        update_irq(u);
    }
    mm_rp2350_dma_poll();
}

void mm_rp2350_pl011_stats(void)
{
    int i;
    for (i = 0; i < 2; ++i) {
        struct pl011 *u = &g_uart[i];
        if (!u->io_open) continue;
        fprintf(stderr, "[PL011 %s] rx=%llu tx=%llu overrun=%llu irq_pend=%llu ris=0x%03x imsc=0x%03x rxlvl=%u txlvl=%u\n",
                u->label, (unsigned long long)u->st_rx, (unsigned long long)u->st_tx,
                (unsigned long long)u->st_overrun, (unsigned long long)u->st_irq_pend,
                u->ris, u->imsc, u->rx_count, u->tx_count);
    }
}
