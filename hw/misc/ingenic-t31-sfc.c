/*
 * Ingenic T31 SPI Flash Controller (SFC) emulation
 *
 * Implements PIO (CPU mode) read from a backing flash image. The flash
 * image is loaded from the first -drive with if=mtd on the command line.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "migration/vmstate.h"
#include "hw/core/irq.h"
#include "hw/misc/ingenic-t31-sfc.h"
#include "system/block-backend.h"
#include "system/blockdev.h"

/*
 * Persist a range of flash_data back to the underlying disk image so
 * that env writes ('saveenv' from U-Boot, etc.) survive across QEMU
 * restarts. blk is NULL when no -drive was attached, in which case we
 * silently keep the changes only in RAM.
 *
 * Errors are logged but not fatal: a write-back failure shouldn't
 * crash the guest, since the in-memory copy is still consistent.
 */
static void ingenic_t31_sfc_writeback(IngenicT31SfcState *s,
                                       uint32_t offset, uint32_t len)
{
    BlockBackend *blk = s->blk;
    int ret;

    if (!blk) {
        return;
    }
    if (offset >= s->flash_size) {
        return;
    }
    if (offset + len > s->flash_size) {
        len = s->flash_size - offset;
    }
    ret = blk_pwrite(blk, offset, len, &s->flash_data[offset], 0);
    if (ret < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ingenic-t31-sfc: write-back failed at "
                      "offset 0x%" PRIx32 " len %" PRIu32 ": %s\n",
                      offset, len, strerror(-ret));
    }
}

static uint32_t ingenic_t31_sfc_sr(IngenicT31SfcState *s)
{
    /* SR bits 16-22 are the FIFO entry count read by the driver's
     * sfc_fifo_num() to decide how many words to drain. */
    uint32_t available = s->fifo_len - s->fifo_pos;
    if (available > 0x7f) {
        available = 0x7f;
    }
    return (s->sr & 0xffff) | (available << 16);
}

static void ingenic_t31_sfc_update_irq(IngenicT31SfcState *s)
{
    /*
     * SFC_INTC is a mask register: bit set = source MASKED. The
     * kernel writes 0x1f to mask all and 0 to unmask all.
     */
    qemu_set_irq(s->irq, (s->sr & ~s->intc) != 0);
}

/* Register offsets */
#define SFC_GLB             0x0000
#define SFC_DEV_CONF        0x0004
#define SFC_DEV_STA_EXP     0x0008
#define SFC_DEV_STA_RT      0x000C
#define SFC_DEV_STA_MSK     0x0010
#define SFC_TRAN_CONF0      0x0014
#define SFC_TRAN_LEN        0x002C
#define SFC_DEV_ADDR0       0x0030
#define SFC_DEV_ADDR_PLUS0  0x0048
#define SFC_MEM_ADDR        0x0060
#define SFC_TRIG            0x0064
#define SFC_SR              0x0068
#define SFC_SCR             0x006C
#define SFC_INTC            0x0070
#define SFC_CGE             0x0078
#define SFC_CMD_IDX         0x007C
#define SFC_COL_ADDR        0x0080
#define SFC_ROW_ADDR        0x0084
#define SFC_DES_ADDR        0x0090
#define SFC_GLB1            0x0094
#define SFC_DEV1_STA_RT     0x0098
#define SFC_TRAN_CONF1_BASE 0x009C
#define SFC_TRAN_CONF1_END  0x00B4
#define SFC_CDT_BASE        0x0800
#define SFC_CDT_END         0x0C00
#define SFC_DR              0x1000

/* SFC_TRIG bits */
#define TRIG_START      (1 << 0)
#define TRIG_STOP       (1 << 1)
#define TRIG_FLUSH      (1 << 2)

/* SFC_SR bits */
#define SR_RECE_REQ     (1 << 2)
#define SR_END          (1 << 4)

/* SFC_TRAN_CONF bits */
#define TRAN_CMD_MSK    0xFFFF
#define TRAN_DATEEN     (1 << 16)

/* SFC_GLB bits */
#define GLB_TRAN_DIR        (1 << 13)

/* SFC_SR bits */
#define SR_TRAN_REQ     (1 << 3)

/* SPI NOR commands */
#define SPI_CMD_READ            0x03
#define SPI_CMD_FAST_READ       0x0B
#define SPI_CMD_READ_ID         0x9F
#define SPI_CMD_READ_STATUS     0x05
#define SPI_CMD_WRITE_ENABLE    0x06
#define SPI_CMD_WRITE_DISABLE   0x04
#define SPI_CMD_PAGE_PROGRAM    0x02
#define SPI_CMD_ERASE_4K        0x20
#define SPI_CMD_ERASE_32K       0x52
#define SPI_CMD_ERASE_64K       0xD8
#define SPI_CMD_ERASE_CHIP      0x60

#define SFC_IOSIZE          0x2000
#define THRESHOLD            31

static void ingenic_t31_sfc_fill_fifo(IngenicT31SfcState *s)
{
    uint32_t addr = s->dev_addr[0] + s->flash_pos * 4;
    uint32_t remaining = s->words_total - s->flash_pos;
    uint32_t chunk = remaining > THRESHOLD ? THRESHOLD : remaining;
    uint32_t i;

    for (i = 0; i < chunk; i++) {
        uint32_t faddr = addr + i * 4;
        if (faddr + 4 <= s->flash_size) {
            memcpy(&s->fifo[i], &s->flash_data[faddr], 4);
        } else {
            s->fifo[i] = 0xFFFFFFFF;
        }
    }
    s->fifo_pos = 0;
    s->fifo_len = chunk;
}

static void ingenic_t31_sfc_do_transfer(IngenicT31SfcState *s)
{
    uint32_t cmd;
    uint32_t cdt_index = s->cmd_idx & 0x3F;
    uint32_t cdt_xfer = s->cdt[cdt_index * 4 + 1];

    if (cdt_xfer != 0) {
        cmd = cdt_xfer & 0xFF;
        s->dev_addr[0] = s->row_addr;
    } else {
        uint32_t cdt_word0 = s->cdt[cdt_index * 4];
        if (cdt_word0 != 0) {
            cmd = cdt_word0 & 0xFF;
            s->dev_addr[0] = s->row_addr;
        } else {
            cmd = s->tran_conf[0] & TRAN_CMD_MSK;
            if (s->row_addr != 0) {
                s->dev_addr[0] = s->row_addr;
            }
        }
    }

    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->sr = 0;

    switch (cmd) {
    case SPI_CMD_READ:
    case SPI_CMD_FAST_READ:
        s->words_total = (s->tran_len + 3) / 4;
        ingenic_t31_sfc_fill_fifo(s);
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_ID:
        s->fifo[0] = 0x001840EF;
        s->fifo_len = 1;
        s->fifo_pos = 0;
        s->words_total = 1;
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_STATUS:
        s->fifo[0] = s->write_enabled ? 0x02 : 0x00;
        s->fifo_len = 1;
        s->fifo_pos = 0;
        s->words_total = 1;
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_WRITE_ENABLE:
        s->write_enabled = true;
        s->sr = SR_END;
        break;

    case SPI_CMD_WRITE_DISABLE:
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_PAGE_PROGRAM:
        if (s->glb & GLB_TRAN_DIR) {
            s->words_total = (s->tran_len + 3) / 4;
            s->writing = true;
            s->sr = SR_TRAN_REQ;
        } else {
            s->sr = SR_END;
        }
        break;

    case SPI_CMD_ERASE_4K:
        if (s->write_enabled && s->dev_addr[0] + 4096 <= s->flash_size) {
            memset(&s->flash_data[s->dev_addr[0]], 0xFF, 4096);
            ingenic_t31_sfc_writeback(s, s->dev_addr[0], 4096);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_ERASE_32K:
        if (s->write_enabled && s->dev_addr[0] + 32768 <= s->flash_size) {
            memset(&s->flash_data[s->dev_addr[0]], 0xFF, 32768);
            ingenic_t31_sfc_writeback(s, s->dev_addr[0], 32768);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_ERASE_64K:
        if (s->write_enabled && s->dev_addr[0] + 65536 <= s->flash_size) {
            memset(&s->flash_data[s->dev_addr[0]], 0xFF, 65536);
            ingenic_t31_sfc_writeback(s, s->dev_addr[0], 65536);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_ERASE_CHIP:
        if (s->write_enabled) {
            memset(s->flash_data, 0xFF, s->flash_size);
            ingenic_t31_sfc_writeback(s, 0, s->flash_size);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    default:
        s->sr = SR_END;
        break;
    }
}

static uint64_t ingenic_t31_sfc_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(opaque);

    switch (offset) {
    case SFC_GLB:
        return s->glb;
    case SFC_DEV_CONF:
        return s->dev_conf;
    case SFC_DEV_STA_RT:
    case SFC_DEV_STA_EXP:
    case SFC_DEV_STA_MSK:
        return 0;
    case SFC_TRAN_CONF0:
        return s->tran_conf[0];
    case SFC_TRAN_LEN:
        return s->tran_len;
    case SFC_DEV_ADDR0:
        return s->dev_addr[0];
    case SFC_SR:
        return ingenic_t31_sfc_sr(s);
    case SFC_INTC:
        return s->intc;
    case SFC_CGE:
        return s->cge;
    case SFC_CMD_IDX:
        return s->cmd_idx;
    case SFC_COL_ADDR:
        return s->col_addr;
    case SFC_ROW_ADDR:
        return s->row_addr;

    case SFC_DR:
        if (s->fifo_pos < s->fifo_len) {
            uint32_t val = s->fifo[s->fifo_pos++];
            s->flash_pos++;
            if (s->fifo_pos >= s->fifo_len) {
                s->sr &= ~SR_RECE_REQ;
                if (s->flash_pos >= s->words_total) {
                    s->sr |= SR_END;
                } else {
                    ingenic_t31_sfc_fill_fifo(s);
                    s->sr |= SR_RECE_REQ;
                }
                ingenic_t31_sfc_update_irq(s);
            }
            return val;
        }
        return 0xFFFFFFFF;

    default:
        if (offset >= SFC_DES_ADDR && offset < SFC_TRAN_CONF1_END) {
            return s->v2_regs[(offset - SFC_DES_ADDR) / 4];
        }
        if (offset >= 0x18 && offset <= 0x28) {
            return 0;
        }
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_T31_SFC_CDT_ENTRIES * 4) {
                return s->cdt[idx];
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read (offset 0x%04" HWADDR_PRIx ")\n",
                      __func__, offset);
        return 0;
    }
}

static void ingenic_t31_sfc_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(opaque);

    switch (offset) {
    case SFC_DEV_STA_EXP:
    case SFC_DEV_STA_MSK:
        break;
    case SFC_GLB:
        s->glb = (uint32_t)value;
        break;
    case SFC_DEV_CONF:
        s->dev_conf = (uint32_t)value;
        break;
    case SFC_TRAN_CONF0:
        s->tran_conf[0] = (uint32_t)value;
        break;
    case SFC_TRAN_LEN:
        s->tran_len = (uint32_t)value;
        break;
    case SFC_DEV_ADDR0:
        s->dev_addr[0] = (uint32_t)value;
        break;
    case SFC_DEV_ADDR_PLUS0:
        s->dev_addr_plus[0] = (uint32_t)value;
        break;
    case SFC_MEM_ADDR:
        s->mem_addr = (uint32_t)value;
        break;
    case SFC_INTC:
        s->intc = (uint32_t)value;
        ingenic_t31_sfc_update_irq(s);
        break;
    case SFC_CGE:
        s->cge = (uint32_t)value;
        break;
    case SFC_CMD_IDX:
        s->cmd_idx = (uint32_t)value;
        break;
    case SFC_COL_ADDR:
        s->col_addr = (uint32_t)value;
        break;
    case SFC_ROW_ADDR:
        s->row_addr = (uint32_t)value;
        break;

    case SFC_TRIG:
        if (value & TRIG_FLUSH) {
            s->fifo_pos = 0;
            s->fifo_len = 0;
        }
        if (value & TRIG_STOP) {
            s->sr = 0;
        }
        if (value & TRIG_START) {
            ingenic_t31_sfc_do_transfer(s);
        }
        ingenic_t31_sfc_update_irq(s);
        break;

    case SFC_DR:
        if (s->writing) {
            uint32_t faddr = s->dev_addr[0] + s->flash_pos * 4;
            if (s->write_enabled && faddr + 4 <= s->flash_size) {
                uint32_t v = (uint32_t)value;
                s->flash_data[faddr]     &= v & 0xFF;
                s->flash_data[faddr + 1] &= (v >> 8) & 0xFF;
                s->flash_data[faddr + 2] &= (v >> 16) & 0xFF;
                s->flash_data[faddr + 3] &= (v >> 24) & 0xFF;
            }
            s->flash_pos++;
            if (s->flash_pos >= s->words_total) {
                /*
                 * End of a PAGE PROGRAM burst: flush the whole written
                 * range back to disk. Doing it once per burst avoids
                 * a blk_pwrite per 4-byte word.
                 */
                if (s->write_enabled || s->writing) {
                    uint32_t bytes = s->flash_pos * 4;
                    ingenic_t31_sfc_writeback(s, s->dev_addr[0], bytes);
                }
                s->writing = false;
                s->write_enabled = false;
                s->sr &= ~SR_TRAN_REQ;
                s->sr |= SR_END;
            } else if ((s->flash_pos % THRESHOLD) == 0) {
                s->sr |= SR_TRAN_REQ;
            }
            ingenic_t31_sfc_update_irq(s);
        }
        break;

    case SFC_SCR:
        s->sr &= ~(uint32_t)value;
        ingenic_t31_sfc_update_irq(s);
        break;

    default:
        if (offset >= SFC_DES_ADDR && offset < SFC_TRAN_CONF1_END) {
            s->v2_regs[(offset - SFC_DES_ADDR) / 4] = (uint32_t)value;
            break;
        }
        if (offset >= 0x18 && offset <= 0x28) {
            break;
        }
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_T31_SFC_CDT_ENTRIES * 4) {
                s->cdt[idx] = (uint32_t)value;
            }
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write (offset 0x%04" HWADDR_PRIx
                      ", value 0x%08" PRIx64 ")\n",
                      __func__, offset, value);
        break;
    }
}

static const MemoryRegionOps ingenic_t31_sfc_ops = {
    .read = ingenic_t31_sfc_read,
    .write = ingenic_t31_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_sfc_reset_hold(Object *obj, ResetType type)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(obj);

    s->glb = 0;
    s->dev_conf = 0;
    memset(s->tran_conf, 0, sizeof(s->tran_conf));
    s->tran_len = 0;
    memset(s->dev_addr, 0, sizeof(s->dev_addr));
    memset(s->dev_addr_plus, 0, sizeof(s->dev_addr_plus));
    s->mem_addr = 0;
    s->sr = 0;
    s->intc = 0x1f;
    s->cge = 0;
    s->cmd_idx = 0;
    s->col_addr = 0;
    s->row_addr = 0;
    memset(s->cdt, 0, sizeof(s->cdt));
    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->write_enabled = false;
    s->writing = false;
}

static void ingenic_t31_sfc_realize(DeviceState *dev, Error **errp)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(dev);
    DriveInfo *di;

    s->flash_size = INGENIC_T31_SFC_FLASH_SIZE;
    s->flash_data = g_malloc0(s->flash_size);
    memset(s->flash_data, 0xFF, s->flash_size);

    di = drive_get(IF_MTD, 0, 0);
    if (!di) {
        di = drive_get(IF_PFLASH, 0, 0);
    }
    if (!di) {
        di = drive_get(IF_NONE, 0, 0);
    }
    if (di) {
        BlockBackend *blk = blk_by_legacy_dinfo(di);
        int64_t size = blk_getlength(blk);

        if (size > s->flash_size) {
            size = s->flash_size;
        }
        if (blk_pread(blk, 0, size, s->flash_data, 0) < 0) {
            error_setg(errp, "failed to read SPI flash image");
            return;
        }
        /*
         * Request write permission so page-program and erase commands
         * can persist back to the backing image. If the image is
         * opened read-only (snapshot=on, --readonly, etc.) we silently
         * fall back to in-memory-only modifications.
         */
        if (blk_supports_write_perm(blk)) {
            uint64_t perm = BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE;
            if (blk_set_perm(blk, perm, BLK_PERM_ALL, NULL) == 0) {
                s->blk = blk;
            }
        }
    }
}

static void ingenic_t31_sfc_init(Object *obj)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_sfc_ops, s,
                          TYPE_INGENIC_T31_SFC, SFC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_t31_sfc_finalize(Object *obj)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(obj);

    g_free(s->flash_data);
}

static void ingenic_t31_sfc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_t31_sfc_realize;
    rc->phases.hold = ingenic_t31_sfc_reset_hold;
}

static const TypeInfo ingenic_t31_sfc_type_info = {
    .name = TYPE_INGENIC_T31_SFC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31SfcState),
    .instance_init = ingenic_t31_sfc_init,
    .instance_finalize = ingenic_t31_sfc_finalize,
    .class_init = ingenic_t31_sfc_class_init,
};

static void ingenic_t31_sfc_register_types(void)
{
    type_register_static(&ingenic_t31_sfc_type_info);
}

type_init(ingenic_t31_sfc_register_types)
