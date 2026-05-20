/*
 * Ingenic A1 SPI Flash Controller V2 (SFC) emulation
 *
 * Read-only flash model for the A1 XBurst2 SoC. Flash data is loaded
 * from a backing image at realize time but never written back -- erase
 * is a no-op in the flash buffer and page-program uses direct writes
 * (not AND-with-existing) since erase does not clear the buffer.
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
#include "hw/misc/ingenic-a1-sfc.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "exec/cpu-common.h"

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
#define SR_TRAN_REQ     (1 << 3)
#define SR_END          (1 << 4)

/* SFC_TRAN_CONF bits */
#define TRAN_CMD_MSK    0xFFFF
#define TRAN_DATEEN     (1 << 16)

/* SFC_GLB bits */
#define GLB_OP_MODE     (1 << 6)
#define GLB_TRAN_DIR    (1 << 13)
#define GLB_DES_EN      (1 << 15)

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

static uint32_t ingenic_a1_sfc_sr(IngenicA1SfcState *s)
{
    uint32_t available = s->fifo_len - s->fifo_pos;
    if (available > 0x7f) {
        available = 0x7f;
    }
    return (s->sr & 0xffff) | (available << 16);
}

static void ingenic_a1_sfc_update_irq(IngenicA1SfcState *s)
{
    /*
     * SFC_INTC is a mask register: bit set = source MASKED. The
     * kernel writes 0x1f to mask all and 0 to unmask all.
     */
    qemu_set_irq(s->irq, (s->sr & ~s->intc) != 0);
}

static uint32_t sfc_get_threshold(IngenicA1SfcState *s)
{
    uint32_t t = (s->glb >> 7) & 0x3F;
    return t ? t : 31;
}

static void ingenic_a1_sfc_fill_fifo(IngenicA1SfcState *s)
{
    uint32_t threshold = sfc_get_threshold(s);
    uint32_t addr = s->dev_addr[0] + s->flash_pos * 4;
    uint32_t remaining = s->words_total - s->flash_pos;
    uint32_t chunk = remaining > threshold ? threshold : remaining;
    uint32_t i;

    for (i = 0; i < chunk; i++) {
        uint32_t faddr = addr + i * 4;
        if (faddr < s->flash_size && faddr + 4 <= s->flash_size) {
            memcpy(&s->fifo[i], &s->flash_data[faddr], 4);
        } else {
            s->fifo[i] = 0xFFFFFFFF;
        }
    }
    s->fifo_pos = 0;
    s->fifo_len = chunk;
}

static void ingenic_a1_sfc_do_transfer(IngenicA1SfcState *s)
{
    uint32_t cmd;
    uint32_t cdt_index = s->cmd_idx & 0x3F;
    uint32_t cdt_xfer = s->cdt[cdt_index * 4 + 1];

    /*
     * DMA descriptor chain mode: the kernel SFC driver sets GLB.DES_EN
     * and writes a descriptor chain address to SFC_DES_ADDR (0x90).
     * Each descriptor has: next_des_addr, mem_addr, tran_len, link.
     * Walk the chain, copy flash data to guest RAM, then set END.
     */
    if ((s->glb & GLB_DES_EN) && !(s->glb & GLB_TRAN_DIR)) {
        uint32_t des_phys = s->v2_regs[0] & 0x1FFFFFFF;
        uint32_t flash_addr = s->dev_addr[0];

        if (des_phys) {
            uint32_t desc[4];
            cpu_physical_memory_read(des_phys, desc, 16);
            if (desc[1] || desc[2]) {
                uint32_t max_iter = 256;
                while (des_phys && max_iter--) {
                    cpu_physical_memory_read(des_phys, desc, 16);
                    uint32_t mem_phys = desc[1] & 0x1FFFFFFF;
                    uint32_t tran_len = desc[2];
                    if (mem_phys && tran_len > 0 &&
                        tran_len <= 16 * 1024 * 1024 &&
                        flash_addr < s->flash_size) {
                        uint32_t avail = s->flash_size - flash_addr;
                        uint32_t len = tran_len < avail ? tran_len : avail;
                        cpu_physical_memory_write(mem_phys,
                                                  &s->flash_data[flash_addr],
                                                  len);
                        flash_addr += len;
                    }
                    if (desc[3] == 0 || desc[0] == 0) {
                        break;
                    }
                    des_phys = desc[0] & 0x1FFFFFFF;
                }
                s->sr = SR_END;
                ingenic_a1_sfc_update_irq(s);
                return;
            }
        }
        /* No valid descriptor: fall through to PIO path */
    }

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
        if (s->row_addr) {
            s->dev_addr[0] = s->row_addr;
        }
        s->words_total = (s->tran_len + 3) / 4;
        ingenic_a1_sfc_fill_fifo(s);
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_ID:
        s->fifo[0] = 0x001840EF;   /* W25Q128 */
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
    case SPI_CMD_ERASE_32K:
    case SPI_CMD_ERASE_64K:
    {
        /*
         * Erase is a no-op in the flash buffer. We don't persist writes
         * back to the disk image, so erasing would destroy data (kernel,
         * rootfs) that the guest reads back later. Page-program uses
         * direct writes (not AND-with-existing) so erase state doesn't
         * matter for correctness within a single session.
         */
        uint32_t erase_sz = (cmd == SPI_CMD_ERASE_4K) ? 4096 :
                             (cmd == SPI_CMD_ERASE_32K) ? 32768 : 65536;
        uint32_t addr = s->dev_addr[0];

        if (s->write_enabled && addr < s->flash_size &&
            (s->flash_size - addr) >= erase_sz) {
            /* no-op: skip memset to preserve flash contents */
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;
    }

    case SPI_CMD_ERASE_CHIP:
        /* Chip erase is also a no-op -- same reasoning as sector erase */
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    default:
        s->sr = SR_END;
        break;
    }
}

static uint64_t ingenic_a1_sfc_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(opaque);

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
    case SFC_TRIG:
        return 0;
    case SFC_SR:
        return ingenic_a1_sfc_sr(s);
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
                    ingenic_a1_sfc_fill_fifo(s);
                    s->sr |= SR_RECE_REQ;
                }
                ingenic_a1_sfc_update_irq(s);
            }
            return val;
        }
        return 0xFFFFFFFF;

    default:
        /* V2 extended registers: DES_ADDR through TRAN_CONF1[5] */
        if (offset >= SFC_DES_ADDR && offset < SFC_TRAN_CONF1_END) {
            return s->v2_regs[(offset - SFC_DES_ADDR) / 4];
        }
        /* TRAN_CONF[1..5] at offsets 0x18..0x28 */
        if (offset >= 0x18 && offset <= 0x28) {
            return 0;
        }
        /* CDT read-back */
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_A1_SFC_CDT_ENTRIES * 4) {
                return s->cdt[idx];
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read (offset 0x%04" HWADDR_PRIx ")\n",
                      __func__, offset);
        return 0;
    }
}

static void ingenic_a1_sfc_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(opaque);

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
        ingenic_a1_sfc_update_irq(s);
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
            ingenic_a1_sfc_do_transfer(s);
        }
        ingenic_a1_sfc_update_irq(s);
        break;

    case SFC_DR:
        if (s->writing) {
            /*
             * Page-program uses direct write (not AND-with-existing)
             * since erase is a no-op and doesn't clear the buffer.
             */
            uint32_t faddr = s->dev_addr[0] + s->flash_pos * 4;
            if (s->write_enabled && faddr < s->flash_size &&
                faddr + 4 <= s->flash_size) {
                uint32_t v = (uint32_t)value;
                s->flash_data[faddr]     = v & 0xFF;
                s->flash_data[faddr + 1] = (v >> 8) & 0xFF;
                s->flash_data[faddr + 2] = (v >> 16) & 0xFF;
                s->flash_data[faddr + 3] = (v >> 24) & 0xFF;
            }
            s->flash_pos++;
            if (s->flash_pos >= s->words_total) {
                /* No writeback -- flash is read-only on disk */
                s->writing = false;
                s->write_enabled = false;
                s->sr &= ~SR_TRAN_REQ;
                s->sr |= SR_END;
            } else if ((s->flash_pos % sfc_get_threshold(s)) == 0) {
                s->sr |= SR_TRAN_REQ;
            }
            ingenic_a1_sfc_update_irq(s);
        }
        break;

    case SFC_SCR:
        s->sr &= ~(uint32_t)value;
        ingenic_a1_sfc_update_irq(s);
        break;

    default:
        /* V2 extended registers */
        if (offset >= SFC_DES_ADDR && offset < SFC_TRAN_CONF1_END) {
            s->v2_regs[(offset - SFC_DES_ADDR) / 4] = (uint32_t)value;
            break;
        }
        /* TRAN_CONF[1..5] at offsets 0x18..0x28 */
        if (offset >= 0x18 && offset <= 0x28) {
            break;
        }
        /* CDT writes */
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_A1_SFC_CDT_ENTRIES * 4) {
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

static const MemoryRegionOps ingenic_a1_sfc_ops = {
    .read = ingenic_a1_sfc_read,
    .write = ingenic_a1_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_a1_sfc_reset_hold(Object *obj, ResetType type)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(obj);

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
    s->des_addr = 0;
    s->glb1 = 0;
    s->dev1_sta_rt = 0;
    memset(s->tran_conf1, 0, sizeof(s->tran_conf1));
    memset(s->v2_regs, 0, sizeof(s->v2_regs));
    memset(s->cdt, 0, sizeof(s->cdt));
    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->write_enabled = false;
    s->writing = false;
}

static void ingenic_a1_sfc_realize(DeviceState *dev, Error **errp)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(dev);
    DriveInfo *di;

    s->flash_size = INGENIC_A1_SFC_FLASH_SIZE;
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
         * Flash is read-only: we do NOT hold a reference to blk and
         * never write back. Modifications (page-program) are kept only
         * in the in-memory flash_data buffer for the current session.
         */
    }
}

static void ingenic_a1_sfc_init(Object *obj)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_a1_sfc_ops, s,
                          TYPE_INGENIC_A1_SFC, INGENIC_A1_SFC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_a1_sfc_finalize(Object *obj)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(obj);

    g_free(s->flash_data);
}

static void ingenic_a1_sfc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_a1_sfc_realize;
    rc->phases.hold = ingenic_a1_sfc_reset_hold;
}

static const TypeInfo ingenic_a1_sfc_type_info = {
    .name = TYPE_INGENIC_A1_SFC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1SfcState),
    .instance_init = ingenic_a1_sfc_init,
    .instance_finalize = ingenic_a1_sfc_finalize,
    .class_init = ingenic_a1_sfc_class_init,
};

static void ingenic_a1_sfc_register_types(void)
{
    type_register_static(&ingenic_a1_sfc_type_info);
}

type_init(ingenic_a1_sfc_register_types)
