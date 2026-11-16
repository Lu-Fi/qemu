/*
 * Ingenic SoCs: an access that never completes
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/cpu.h"
#include "system/cpus.h"
#include "hw/misc/ingenic-bus-hang.h"

void ingenic_bus_hang(const char *who, hwaddr addr, bool write)
{
    CPUState *cpu;

    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of 0x%" HWADDR_PRIx
                  " hangs the bus; only a reset recovers\n",
                  who, write ? "write" : "read", addr);
    CPU_FOREACH(cpu) {
        cpu->stop = true;
        qemu_cpu_kick(cpu);
    }
    cpu_stop_current();
}
