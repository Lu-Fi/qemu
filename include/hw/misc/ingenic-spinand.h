/*
 * Ingenic SoCs: a SPI NAND behind the SFC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_SPINAND_H
#define HW_MISC_INGENIC_SPINAND_H

#define INGENIC_SPINAND_PAGE    2048
#define INGENIC_SPINAND_OOB     128
#define INGENIC_SPINAND_RAW     (INGENIC_SPINAND_PAGE + INGENIC_SPINAND_OOB)
#define INGENIC_SPINAND_PPB     64
#define INGENIC_SPINAND_MAX_BLOCKS 16384

/* Persist len bytes of the page data at offset */
typedef void IngenicSpiNandWriteback(void *opaque, uint32_t offset,
                                     uint32_t len);

typedef struct IngenicSpiNand {
    uint32_t id;            /* (MID << 8) | DID; 0 = no NAND */
    uint32_t blocks;
    uint8_t *data;          /* the page data, the controller's flash array */
    uint8_t *oob;           /* the spare areas, in RAM only */
    uint8_t cache[INGENIC_SPINAND_RAW];
    uint8_t feat[4];        /* feature registers A0h B0h C0h D0h */
    uint8_t cmd;            /* the command a data phase belongs to */
    uint32_t addr;
    IngenicSpiNandWriteback *writeback;
    void *opaque;
} IngenicSpiNand;

static inline uint32_t ingenic_spinand_size(const IngenicSpiNand *n)
{
    return n->blocks * INGENIC_SPINAND_PPB * INGENIC_SPINAND_PAGE;
}

void ingenic_spinand_init(IngenicSpiNand *n, uint8_t *data,
                          IngenicSpiNandWriteback *writeback, void *opaque);
void ingenic_spinand_finalize(IngenicSpiNand *n);

/*
 * One command on the bus, with an address or dummy cycles or neither.
 * buf (len bytes) gets what the flash answers, 0xff past its end.
 */
void ingenic_spinand_command(IngenicSpiNand *n, uint8_t cmd, uint32_t addr,
                             bool dummy, uint8_t *buf, uint32_t len);

/* The len bytes a SET FEATURE or PROGRAM LOAD sent */
void ingenic_spinand_data(IngenicSpiNand *n, const uint8_t *buf,
                          uint32_t len);

#endif
