/*
 * Ingenic SHA-256 hardware accelerator emulation
 *
 * Register map (from vendor kernel ingenic-hash.h):
 *   0x00  HSCR   - Control (algorithm, enable, DMA start)
 *   0x04  HSSR   - Status (bit 1 = DMA done, write-1-to-clear)
 *   0x08  HSINTM - Interrupt mask (bit 1 = DMA done IRQ enable)
 *   0x0C  HSSA   - DMA source address (physical)
 *   0x10  HSTC   - Transfer count (64-byte blocks)
 *   0x14  HSDI   - Data input FIFO (CPU-fed, used by bootrom)
 *   0x18  HSDO   - Data output (digest, read N words)
 *
 * The T31 bootrom uses both CPU-fed (HSDI at 0x14) and DMA modes.
 * For SHA-256: HSCR algorithm select = 3, SHA mode bits [8:7] = 3.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/ingenic-hash.h"
#include "crypto/hash.h"
#include "exec/cpu-common.h"
#include "migration/vmstate.h"

#define HASH_HSCR    0x00
#define HASH_HSSR    0x04
#define HASH_HSINTM  0x08
#define HASH_HSSA    0x0C
#define HASH_HSTC    0x10
#define HASH_HSDI    0x14
#define HASH_HSDO    0x18

#define HSCR_DMA_START  (3 << 5)

static void ingenic_hash_compute(IngenicHashState *s)
{
    uint8_t *result = NULL;
    size_t result_len = 0;
    uint32_t data_len = s->fifo_pos;

    if (data_len == 0) {
        memset(s->digest, 0, sizeof(s->digest));
        s->digest_valid = true;
        return;
    }

    /* The Ingenic HASH hardware receives pre-padded SHA-256 blocks.
     * The ROM adds standard padding (0x80 + zeros + 64-bit big-endian
     * bit count) before feeding data.  Strip it so qcrypto_hash_bytes
     * (which adds its own padding) produces the correct result. */
    if (data_len >= 72 && data_len % 64 == 0) {
        uint64_t bitlen = (uint64_t)s->fifo[data_len - 8] << 56 |
                          (uint64_t)s->fifo[data_len - 7] << 48 |
                          (uint64_t)s->fifo[data_len - 6] << 40 |
                          (uint64_t)s->fifo[data_len - 5] << 32 |
                          (uint64_t)s->fifo[data_len - 4] << 24 |
                          (uint64_t)s->fifo[data_len - 3] << 16 |
                          (uint64_t)s->fifo[data_len - 2] << 8 |
                          (uint64_t)s->fifo[data_len - 1];
        uint32_t orig_bytes = (uint32_t)(bitlen / 8);
        if (orig_bytes < data_len && s->fifo[orig_bytes] == 0x80) {
            data_len = orig_bytes;
        }
    }

    fprintf(stderr, "HASH: strip %u -> %u, last8=[%02x%02x%02x%02x %02x%02x%02x%02x]\n",
            s->fifo_pos, data_len,
            s->fifo[s->fifo_pos-8], s->fifo[s->fifo_pos-7],
            s->fifo[s->fifo_pos-6], s->fifo[s->fifo_pos-5],
            s->fifo[s->fifo_pos-4], s->fifo[s->fifo_pos-3],
            s->fifo[s->fifo_pos-2], s->fifo[s->fifo_pos-1]);

    if (qcrypto_hash_bytes(QCRYPTO_HASH_ALGO_SHA256,
                           (const char *)s->fifo, data_len,
                           &result, &result_len, NULL) == 0) {
        if (result_len >= 32) {
            memcpy(s->digest, result, 32);
        }
        g_free(result);
    }
    s->digest_valid = true;
    s->digest_idx = 0;
}

static void ingenic_hash_dma(IngenicHashState *s)
{
    uint32_t bytes = s->reg_blkcnt * 64;

    if (bytes > INGENIC_HASH_FIFO_MAX) {
        bytes = INGENIC_HASH_FIFO_MAX;
    }

    s->fifo_pos = 0;
    cpu_physical_memory_read(s->reg_dma_addr, s->fifo, bytes);
    s->fifo_pos = bytes;

    ingenic_hash_compute(s);
    s->reg_status |= 2;
}

static uint64_t ingenic_hash_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicHashState *s = INGENIC_HASH(opaque);

    switch (offset) {
    case HASH_HSCR: {
        uint32_t v = s->reg_ctrl;
        fprintf(stderr, "HASH: R HSCR=0x%08x\n", v);
        return v;
    }
    case HASH_HSSR:
        /* Bit 0: FIFO ready (ROM polls this every 16 words).
         * Bit 1: DMA done (kernel driver uses this).
         * Always report ready since we process data synchronously. */
        return s->reg_status | 0x3;
    case HASH_HSINTM:
        return s->reg_intmask;
    case HASH_HSSA:
        return s->reg_dma_addr;
    case HASH_HSTC:
        return s->reg_blkcnt;
    case HASH_HSDO:
        if (!s->digest_valid && s->fifo_pos > 0) {
            ingenic_hash_compute(s);
            fprintf(stderr, "HASH: computed SHA-256 (%u bytes input): ",
                    s->fifo_pos);
            for (int k = 0; k < 8; k++)
                fprintf(stderr, "%02x%02x%02x%02x",
                        s->digest[k*4], s->digest[k*4+1],
                        s->digest[k*4+2], s->digest[k*4+3]);
            fprintf(stderr, "\n");
        }
        if (s->digest_idx < 8) {
            uint32_t w = (uint32_t)s->digest[s->digest_idx * 4] << 24 |
                         (uint32_t)s->digest[s->digest_idx * 4 + 1] << 16 |
                         (uint32_t)s->digest[s->digest_idx * 4 + 2] << 8 |
                         (uint32_t)s->digest[s->digest_idx * 4 + 3];
            fprintf(stderr, "HASH: HSDO[%u] = 0x%08x\n", s->digest_idx, w);
            s->digest_idx++;
            return w;
        }
        return 0;
    default:
        return 0;
    }
}

static void ingenic_hash_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    IngenicHashState *s = INGENIC_HASH(opaque);

    switch (offset) {
    case HASH_HSCR:
        fprintf(stderr, "HASH: W HSCR=0x%08x\n", (unsigned)value);
        s->reg_ctrl = (uint32_t)value;
        if (value & 1) {
            s->fifo_pos = 0;
            s->digest_valid = false;
            s->digest_idx = 0;
            s->started = true;
        }
        if (value & HSCR_DMA_START) {
            ingenic_hash_dma(s);
        }
        break;
    case HASH_HSSR:
        s->reg_status &= ~(uint32_t)value;
        break;
    case HASH_HSINTM:
        s->reg_intmask = (uint32_t)value;
        break;
    case HASH_HSSA:
        s->reg_dma_addr = (uint32_t)value;
        break;
    case HASH_HSTC:
        s->reg_blkcnt = (uint32_t)value;
        break;
    case HASH_HSDI:
        if (s->started && s->fifo_pos + 4 <= INGENIC_HASH_FIFO_MAX) {
            uint32_t v = (uint32_t)value;
            s->fifo[s->fifo_pos]     = v & 0xff;
            s->fifo[s->fifo_pos + 1] = (v >> 8) & 0xff;
            s->fifo[s->fifo_pos + 2] = (v >> 16) & 0xff;
            s->fifo[s->fifo_pos + 3] = (v >> 24) & 0xff;
            s->fifo_pos += 4;
            s->digest_valid = false;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_hash_ops = {
    .read = ingenic_hash_read,
    .write = ingenic_hash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_hash_reset_hold(Object *obj, ResetType type)
{
    IngenicHashState *s = INGENIC_HASH(obj);
    s->reg_ctrl = 0;
    s->reg_status = 0;
    s->reg_intmask = 0;
    s->reg_dma_addr = 0;
    s->reg_blkcnt = 0;
    s->fifo_pos = 0;
    s->started = false;
    s->digest_valid = false;
    s->digest_idx = 0;
}

static void ingenic_hash_realize(DeviceState *dev, Error **errp)
{
}

static void ingenic_hash_init(Object *obj)
{
    IngenicHashState *s = INGENIC_HASH(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_hash_ops, s,
                          TYPE_INGENIC_HASH, INGENIC_HASH_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_ingenic_hash = {
    .name = TYPE_INGENIC_HASH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reg_ctrl, IngenicHashState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_hash_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_hash_realize;
    dc->vmsd = &vmstate_ingenic_hash;
    rc->phases.hold = ingenic_hash_reset_hold;
}

static const TypeInfo ingenic_hash_type_info = {
    .name = TYPE_INGENIC_HASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicHashState),
    .instance_init = ingenic_hash_init,
    .class_init = ingenic_hash_class_init,
};

static void ingenic_hash_register_types(void)
{
    type_register_static(&ingenic_hash_type_info);
}

type_init(ingenic_hash_register_types)
