/*
 * Ingenic T30 display controller (DPU), the registers it reads out of reset
 *
 * A smaller DPU than the T40's at 0x13050000, 44 KiB wide. Its registers
 * were read on a T30X at U-Boot with the gate opened: the ones below are
 * not 0, every other register the vendor header lists reads 0. Nothing
 * writes to silicon here, so the bits a write keeps are not known and
 * writes are dropped, as they were before.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T30_DPU "ingenic-t30-dpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT30DpuState, INGENIC_T30_DPU)

#define T30_DPU_SIZE    0xb000

struct IngenicT30DpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
};

static const struct {
    uint16_t offset;
    uint32_t value;
} t30_dpu_regs[] = {
    { 0x2014, 0x00cf000c },     /* DC_COM_CONFIG */
    { 0x2018, 0x00000006 },     /* DC_PCFG_RD_CTRL */
    { 0x2020, 0x03190842 },     /* DC_OFIFO_PCFG */
    { 0x3200, 0x066204a8 },     /* layer 0 CSC */
    { 0x3204, 0x03410190 },
    { 0x3208, 0x00000812 },
    { 0x320c, 0x00800000 },
    { 0x3210, 0x066204a8 },     /* layer 1 CSC */
    { 0x3214, 0x03410190 },
    { 0x3218, 0x00000812 },
    { 0x321c, 0x00800000 },
    { 0x8000, 0x000000e0 },     /* DC_DISP_COM */
    { 0xa000, 0x00000800 },     /* DC_SLCD_CFG */
};

static uint64_t ingenic_t30_dpu_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    for (unsigned i = 0; i < ARRAY_SIZE(t30_dpu_regs); i++) {
        if (t30_dpu_regs[i].offset == (offset & ~3)) {
            return t30_dpu_regs[i].value >> ((offset & 3) * 8);
        }
    }
    return 0;
}

static void ingenic_t30_dpu_write(void *opaque, hwaddr offset, uint64_t value,
                                  unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "ingenic-t30-dpu: write 0x%" HWADDR_PRIx
                  " <- 0x%" PRIx64 " dropped\n", offset, value);
}

static const MemoryRegionOps ingenic_t30_dpu_ops = {
    .read = ingenic_t30_dpu_read,
    .write = ingenic_t30_dpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void ingenic_t30_dpu_init(Object *obj)
{
    IngenicT30DpuState *s = INGENIC_T30_DPU(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t30_dpu_ops, s,
                          TYPE_INGENIC_T30_DPU, T30_DPU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const TypeInfo ingenic_t30_dpu_info = {
    .name = TYPE_INGENIC_T30_DPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT30DpuState),
    .instance_init = ingenic_t30_dpu_init,
};

static void ingenic_t30_dpu_register_types(void)
{
    type_register_static(&ingenic_t30_dpu_info);
}

type_init(ingenic_t30_dpu_register_types)
