/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef M33MU_CPU_RP2350_DMA_H
#define M33MU_CPU_RP2350_DMA_H

#include "m33mu/types.h"
#include "m33mu/mmio.h"
#include "m33mu/nvic.h"

mm_bool mm_rp2350_dma_register(struct mmio_bus *bus);
void mm_rp2350_dma_set_nvic(struct mm_nvic *nvic);
void mm_rp2350_dma_poll(void);
mm_bool mm_rp2350_dma_line(mm_u32 irq);
void mm_rp2350_dma_stats(void);

#endif
