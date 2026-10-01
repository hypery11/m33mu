/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef M33MU_CPU_RP2350_PL011_H
#define M33MU_CPU_RP2350_PL011_H

#include "m33mu/types.h"

void mm_rp2350_pl011_tick(mm_u64 cycles);
void mm_rp2350_pl011_stats(void);
void mm_rp2350_exit_report(void);

#endif
