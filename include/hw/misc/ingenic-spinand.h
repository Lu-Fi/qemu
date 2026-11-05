/*
 * Ingenic SoCs: a SPI NAND behind the SFC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_SPINAND_H
#define HW_MISC_INGENIC_SPINAND_H

#define INGENIC_SPINAND_PAGE_MAX    4096
#define INGENIC_SPINAND_OOB_MAX     256
#define INGENIC_SPINAND_RAW         (INGENIC_SPINAND_PAGE_MAX + \
                                     INGENIC_SPINAND_OOB_MAX)
#define INGENIC_SPINAND_MAX_BLOCKS  16384
#define INGENIC_SPINAND_ID_MAX      6

/* How a part with two dies picks one */
typedef enum {
    INGENIC_SPINAND_DIE_NONE,
    INGENIC_SPINAND_DIE_WINBOND,    /* SOFTWARE DIE SELECT, C2h */
    INGENIC_SPINAND_DIE_MICRON,     /* feature D0h bit 6 */
} IngenicSpiNandDieSelect;

typedef struct IngenicSpiNandPart {
    const char *name;
    uint8_t id[INGENIC_SPINAND_ID_MAX];     /* MID, then the device ID */
    uint8_t id_len;
    bool id_after_byte;     /* READ ID answers after an address/dummy byte */
    uint16_t page;
    uint16_t oob;
    uint16_t ppb;           /* pages a block */
    uint16_t blocks;        /* in all, over every die */
    uint8_t planes;
    uint8_t dies;
    IngenicSpiNandDieSelect die_select;
    uint8_t ecc_mask;       /* the ECC status field in feature C0h */
    uint8_t ecc_corr;       /* what it reads after a corrected read */
    uint8_t ecc_uncorr;     /* and after an uncorrectable one */
    uint8_t ecc_ext;        /* a feature (or 7Ch, an opcode) with the count */
    uint8_t ecc_ext_val;    /* what that reads after a corrected read */
    uint32_t t_read_ns;     /* PAGE READ, to the cache (tRD) */
    uint32_t t_prog_ns;     /* PROGRAM EXECUTE (tPROG) */
    uint32_t t_erase_ns;    /* BLOCK ERASE (tBERS) */
} IngenicSpiNandPart;

typedef struct IngenicSpiNand {
    /* Properties of the SFC that carries the chip */
    uint32_t id;            /* nand-id: (MID << 8) | DID, or 3 ID bytes */
    char *part_name;        /* nand-part */
    uint32_t blocks;        /* nand-blocks: 0 = the part's */
    char *layout;           /* nand-layout: "data" or "raw" */
    bool timing;            /* nand-busy: operations hold OIP */

    IngenicSpiNandPart part;    /* the part, or one made up for nand-id */
    bool raw;               /* the image holds each page's spare area */
    uint32_t col_bits;      /* the column address; the next bit is plane */
    void *blk;
    uint64_t blk_len;
    uint8_t *data;          /* the page data, back to back */
    uint8_t *oob;           /* the spare areas, back to back */
    uint32_t oob_len;
    uint8_t cache[2][INGENIC_SPINAND_RAW];  /* one per plane */
    uint8_t feat[4];        /* feature registers A0h B0h C0h D0h */
    bool corrected;         /* the last PAGE READ mended bitflips */
    uint8_t die;            /* the die selected */
    uint8_t cmd;            /* the command a data phase belongs to */
    uint32_t addr;
    int64_t busy_until;     /* QEMU_CLOCK_VIRTUAL ns; OIP until then */
    GHashTable *ecc_faults; /* page -> INGENIC_SPINAND_FAULT_* */
    GArray *worn;           /* blocks a test marked bad */
} IngenicSpiNand;

extern const VMStateDescription vmstate_ingenic_spinand;

/* Add the chip's properties to the SFC that carries it */
void ingenic_spinand_add_properties(Object *owner, IngenicSpiNand *n);
/*
 * Load the chip from blk (or none) if nand-id or nand-part asks for one;
 * false with errp set on a bad configuration. A chip present leaves its
 * ID in n->id.
 */
bool ingenic_spinand_realize(IngenicSpiNand *n, void *blk, Error **errp);
void ingenic_spinand_finalize(IngenicSpiNand *n);

/*
 * The array again from the drive it writes through to, after a load: the
 * drive is then the one the state was saved with
 */
void ingenic_spinand_reread(IngenicSpiNand *n);

/*
 * One command on the bus, with an address or dummy cycles or neither.
 * buf (len bytes) gets what the flash answers, 0xff past its end.
 */
void ingenic_spinand_command(IngenicSpiNand *n, uint8_t cmd, uint32_t addr,
                             bool dummy, uint8_t *buf, uint32_t len);

/* The len bytes a SET FEATURE or PROGRAM LOAD sent */
void ingenic_spinand_data(IngenicSpiNand *n, const uint8_t *buf,
                          uint32_t len);

/* When the operation in flight ends (QEMU_CLOCK_VIRTUAL), 0 if idle */
int64_t ingenic_spinand_busy_until(IngenicSpiNand *n);

/*
 * The first len bytes of the boot flash, for a board that boots past
 * the bootrom: the SPI NAND's page data if n holds a chip, else blk's
 */
int ingenic_spinand_boot_read(IngenicSpiNand *n, void *blk, uint32_t len,
                              void *buf);

#endif
