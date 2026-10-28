/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef M33MU_CPU_RP2350_I2C1_MASTER_H
#define M33MU_CPU_RP2350_I2C1_MASTER_H

#include "m33mu/types.h"

/*
 * A scripted I2C bus master on the I2C1 pins (the role of
 * the RK3576 CPU towards the Flipper One MCU firmware's I2C slave at 0x69).
 * It drives the DW_apb_i2c slave model of I2C1 in rp2350_extra.c.  Off
 * unless M33MU_I2C1_MASTER is set, see rp2350_i2c1_master.c.
 */

/* MM_TRUE when M33MU_I2C1_MASTER names a script (read once, on first call). */
mm_bool mm_rp2350_i2c1_master_enabled(void);

/* Advance the script and the bus up to the current emulated time. */
void mm_rp2350_i2c1_master_run(void);

/* Cycles from now to the next timed event (the end of a bus phase or a
 * delay, a stretch time-out); (mm_u64)-1 when nothing is scheduled. */
mm_u64 mm_rp2350_i2c1_master_next_event(void);

/* MM_TRUE while an irq_hold_us of the script is running: the I2C1 IRQ line
 * then reads low to the NVIC. */
mm_bool mm_rp2350_i2c1_master_irq_held(void);

/* Text the firmware wrote to its RTT log (for `wait_log`). */
void mm_rp2350_i2c1_master_rtt(const mm_u8 *data, mm_u32 len);

/* M33MU_I2C1_TRACE: slave-side events and register accesses go to the log. */
mm_bool mm_rp2350_i2c1_master_tracing(void);
void mm_rp2350_i2c1_master_trace(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

/* Counters, printed in the exit report. */
void mm_rp2350_i2c1_master_stats(void);

#endif /* M33MU_CPU_RP2350_I2C1_MASTER_H */
