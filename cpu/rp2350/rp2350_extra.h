/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef M33MU_CPU_RP2350_EXTRA_H
#define M33MU_CPU_RP2350_EXTRA_H

#include "m33mu/types.h"
#include "m33mu/mmio.h"
#include "m33mu/memmap.h"

mm_bool mm_rp2350_extra_register(struct mmio_bus *bus);
void mm_rp2350_extra_flash_bind(struct mm_memmap *map, mm_u8 *flash, mm_u32 flash_size);
mm_bool mm_rp2350_extra_pt_loaded(void);
mm_u32 mm_rp2350_extra_pt_info(struct mm_memmap *map, enum mm_sec_state sec,
                               mm_u32 out_addr, mm_u32 out_words, mm_u32 flags_and_partition);
void mm_rp2350_extra_tick(mm_u64 cycles);
void mm_rp2350_extra_flush(void);
void mm_rp2350_extra_stats(void);
/* core 0, for the PC sampler (M33MU_PC_SAMPLE) */
struct mm_cpu;
void mm_rp2350_extra_set_cpu0(const struct mm_cpu *cpu);

/* RESETS asserted the reset of I2C<index> (only matters for a bus with
 * a slave model attached). */
void mm_rp2350_extra_i2c_reset(mm_u32 index);

/* measurement probes (M33MU_PROBE_LOG, see rp2350_extra.c). */
mm_bool mm_rp2350_probe_fn_on(void);
void mm_rp2350_probe_fn_check(mm_u32 pc, mm_u32 lr, mm_u32 sp);
mm_bool mm_rp2350_probe_dma_on(void);
void mm_rp2350_probe_dma(mm_u32 ch, mm_u32 dst, mm_u32 step, const mm_u32 *words, mm_u32 n, mm_u32 n_total);

#endif
