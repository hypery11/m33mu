/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <string.h>
#include <stdio.h>
#include "rp2350/rp2350_uart_spi.h"
#include "rp2350/rp2350_mmio.h"
#include "rp2350/rp2350_usb.h"
#include "m33mu/mmio.h"
#include "m33mu/target_hal.h"
#include "m33mu/spi_bus.h"

#define UART0_BASE 0x40070000u
#define UART1_BASE 0x40078000u
#define SPI0_BASE  0x40080000u
#define SPI1_BASE  0x40088000u

#define UART_SIZE 0x2000u
#define SPI_SIZE  0x1000u

#define UARTDR   0x000u
#define UARTRSR  0x004u
#define UARTFR   0x018u
#define UARTIBRD 0x024u
#define UARTFBRD 0x028u
#define UARTLCR_H 0x02cu
#define UARTCR   0x030u
#define UARTIMSC 0x038u
#define UARTRIS  0x03cu
#define UARTMIS  0x040u
#define UARTICR  0x044u

#define UARTCR_UARTEN (1u << 0)
#define UARTCR_TXE    (1u << 8)
#define UARTCR_RXE    (1u << 9)

#define UARTFR_CTS  (1u << 0)
#define UARTFR_DSR  (1u << 1)
#define UARTFR_DCD  (1u << 2)
#define UARTFR_BUSY (1u << 3)
#define UARTFR_RXFE (1u << 4)
#define UARTFR_TXFF (1u << 5)
#define UARTFR_RXFF (1u << 6)
#define UARTFR_TXFE (1u << 7)

#define SSPCR0  0x000u
#define SSPCR1  0x004u
#define SSPDR   0x008u
#define SSPSR   0x00cu
#define SSPCPSR 0x010u
#define SSPIMSC 0x014u
#define SSPRIS  0x018u
#define SSPMIS  0x01cu
#define SSPICR  0x020u
#define SSPDMACR 0x024u

#define SSPCR1_SSE (1u << 1)

#define SSPSR_TFE (1u << 0)
#define SSPSR_TNF (1u << 1)
#define SSPSR_RNE (1u << 2)
#define SSPSR_RFF (1u << 3)
#define SSPSR_BSY (1u << 4)

struct spi_inst {
    mm_u32 base;
    mm_u32 reset_bit;
    mm_u32 regs[SPI_SIZE / 4u];
    mm_u8 rx_data;
    mm_bool rx_valid;
    int bus;
};

static struct spi_inst spis[2];

static mm_bool spi_clock_on(struct spi_inst *s)
{
    if (s == 0) return MM_FALSE;
    if (!mm_rp2350_clock_peri_enabled()) return MM_FALSE;
    if (mm_rp2350_reset_asserted(s->reset_bit)) return MM_FALSE;
    return MM_TRUE;
}

static void spi_update_status(struct spi_inst *s)
{
    mm_u32 sr = 0u;
    if (s == 0) return;
    sr |= SSPSR_TFE | SSPSR_TNF;
    if (s->rx_valid) {
        sr |= SSPSR_RNE;
    }
    s->regs[SSPSR / 4u] = sr;
}

static mm_bool spi_read(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 *value_out)
{
    struct spi_inst *s = (struct spi_inst *)opaque;
    if (s == 0 || value_out == 0 || size_bytes == 0u || size_bytes > 4u) return MM_FALSE;
    if (offset + size_bytes > SPI_SIZE) return MM_FALSE;

    if (offset == SSPDR && size_bytes == 4u) {
        mm_u32 v = 0u;
        if (s->rx_valid) {
            v = (mm_u32)s->rx_data;
            if (!mmio_peek_mode()) {
                s->rx_valid = MM_FALSE;
            }
        }
        *value_out = v;
        if (!mmio_peek_mode()) {
            spi_update_status(s);
        }
        return MM_TRUE;
    }
    if (offset == SSPSR && size_bytes == 4u) {
        if (!mmio_peek_mode()) {
            spi_update_status(s);
        }
        *value_out = s->regs[SSPSR / 4u];
        return MM_TRUE;
    }

    *value_out = s->regs[offset / 4u];
    return MM_TRUE;
}

static mm_bool spi_write(void *opaque, mm_u32 offset, mm_u32 size_bytes, mm_u32 value)
{
    struct spi_inst *s = (struct spi_inst *)opaque;
    mm_u8 out;
    mm_u8 in;
    if (s == 0 || size_bytes == 0u || size_bytes > 4u) return MM_FALSE;
    if (offset + size_bytes > SPI_SIZE) return MM_FALSE;

    if (offset == SSPDR && size_bytes == 4u) {
        if (spi_clock_on(s) && (s->regs[SSPCR1 / 4u] & SSPCR1_SSE) != 0u) {
            out = (mm_u8)(value & 0xffu);
            in = mm_spi_bus_xfer(s->bus, out);
            s->rx_data = in;
            s->rx_valid = MM_TRUE;
            mm_spi_bus_end(s->bus);
        }
        spi_update_status(s);
        return MM_TRUE;
    }
    if (offset == SSPICR && size_bytes == 4u) {
        s->regs[SSPRIS / 4u] = 0u;
        s->regs[SSPMIS / 4u] = 0u;
        return MM_TRUE;
    }

    s->regs[offset / 4u] = value;
    spi_update_status(s);
    return MM_TRUE;
}

void mm_rp2350_spi_init(struct mmio_bus *bus, struct mm_nvic *nvic)
{
    struct mmio_region reg;
    (void)nvic;
    if (bus == 0) return;

    memset(&reg, 0, sizeof(reg));

    spis[0].base = SPI0_BASE;
    spis[0].reset_bit = RP2350_RESET_SPI0;
    spis[0].bus = 0;
    reg.base = SPI0_BASE;
    reg.size = SPI_SIZE;
    reg.opaque = &spis[0];
    reg.read = spi_read;
    reg.write = spi_write;
    (void)mmio_bus_register_region(bus, &reg);

    spis[1].base = SPI1_BASE;
    spis[1].reset_bit = RP2350_RESET_SPI1;
    spis[1].bus = 1;
    reg.base = SPI1_BASE;
    reg.opaque = &spis[1];
    (void)mmio_bus_register_region(bus, &reg);
}

void mm_rp2350_spi_reset(void)
{
    size_t i;
    for (i = 0; i < sizeof(spis) / sizeof(spis[0]); ++i) {
        memset(spis[i].regs, 0, sizeof(spis[i].regs));
        spis[i].rx_valid = MM_FALSE;
        spi_update_status(&spis[i]);
    }
}

void mm_rp2350_spi_poll(void)
{
    size_t i;
    for (i = 0; i < sizeof(spis) / sizeof(spis[0]); ++i) {
        if (!spi_clock_on(&spis[i])) {
            spis[i].rx_valid = MM_FALSE;
        }
        spi_update_status(&spis[i]);
    }
}
