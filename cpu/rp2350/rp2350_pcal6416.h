/* m33mu -- an ARMv8-M Emulator
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef M33MU_CPU_RP2350_PCAL6416_H
#define M33MU_CPU_RP2350_PCAL6416_H

#include "m33mu/types.h"

/*
 * An optional behavioural model of an NXP PCAL6416A 16-bit
 * I/O expander as a slave on a DW_apb_i2c bus (the Flipper One control bus,
 * I2C0 on GPIO20/21, address 0x20).  Off unless M33MU_PCAL6416 is set, see
 * rp2350_pcal6416.c for what is and is not modelled.
 */

/* MM_TRUE when M33MU_PCAL6416 is set to something other than "" or "0". */
mm_bool mm_rp2350_pcal6416_enabled(void);

/* DW_apb_i2c instance the device sits on (0 = I2C0, 1 = I2C1). */
mm_u32 mm_rp2350_pcal6416_bus(void);

/*
 * Byte-level slave interface, called by the I2C master engine in
 * rp2350_extra.c.  `t` is the time of the event in system clock cycles
 * (mm_rp2350_now_cycles() time base).
 *   start: START or repeated START plus the address byte; returns ACK
 *   write: one byte from the master; returns ACK
 *   read:  one byte to the master; master_ack says whether it was ACKed
 *   stop:  STOP condition
 */
mm_bool mm_rp2350_pcal6416_start(mm_u64 t, mm_u32 addr7, mm_bool read);
mm_bool mm_rp2350_pcal6416_write(mm_u64 t, mm_u8 byte);
mm_u8 mm_rp2350_pcal6416_read(mm_u64 t, mm_bool master_ack);
void mm_rp2350_pcal6416_stop(mm_u64 t);

/*
 * Pads the device (with the pull-up on its open-drain INT line) presents to
 * the MCU: bit n = GPIO n.  *high: pulled high unless the MCU drives the pin,
 * *low: pulled low by the device.
 */
void mm_rp2350_pcal6416_pads(mm_u64 *high, mm_u64 *low);

/* Counters, printed in the exit report. */
void mm_rp2350_pcal6416_stats(void);

#endif /* M33MU_CPU_RP2350_PCAL6416_H */
