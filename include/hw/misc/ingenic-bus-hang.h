/*
 * Ingenic SoCs: an access that never completes
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_BUS_HANG_H
#define HW_MISC_INGENIC_BUS_HANG_H

#include "exec/hwaddr.h"

/*
 * A register access on silicon that hangs the bus: no core executes another
 * instruction and nothing but the watchdog's reset, if it runs, brings the
 * SoC back. Here every CPU stops, the timers and devices run on, and a
 * system reset resumes the CPUs.
 */
void ingenic_bus_hang(const char *who, hwaddr addr, bool write);

#endif /* HW_MISC_INGENIC_BUS_HANG_H */
