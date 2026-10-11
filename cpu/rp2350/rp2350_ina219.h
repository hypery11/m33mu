/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef M33MU_CPU_RP2350_INA219_H
#define M33MU_CPU_RP2350_INA219_H

#include "m33mu/types.h"

/*
 * An optional behavioural model of a TI INA219 as an I2C
 * slave on two GPIO pads (the Flipper One main bus, a PIO bit-banged master
 * on GPIO22 = SDA / GPIO23 = SCL).  Off unless M33MU_INA219 is set, see
 * rp2350_ina219.c for what is and is not modelled.
 */

/* MM_TRUE when M33MU_INA219 is set to something other than "" or "0". */
mm_bool mm_rp2350_ina219_enabled(void);

/* Virtual time base for the model (system clock cycles). */
void mm_rp2350_ina219_tick(mm_u64 cycles);

/*
 * Feed the current pad levels (bit n = GPIO n, after pull-ups and whatever the
 * master drives); returns the pads the device pulls low (open drain).
 */
mm_u64 mm_rp2350_ina219_pads(mm_u64 levels);

/* Counters, printed in the exit report. */
void mm_rp2350_ina219_stats(void);

#endif /* M33MU_CPU_RP2350_INA219_H */
