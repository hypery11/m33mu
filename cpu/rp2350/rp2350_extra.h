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

#endif
