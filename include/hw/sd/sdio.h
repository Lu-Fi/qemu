/*
 * SDIO card
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_SDIO_H
#define HW_SD_SDIO_H

#include "hw/sd/sd.h"
#include "qom/object.h"

#define TYPE_SDIO_CARD "sdio-card"
OBJECT_DECLARE_TYPE(SDIOCard, SDIOCardClass, SDIO_CARD)

#define SDIO_MAX_FUNCS  7

struct SDIOCardClass {
    /*< private >*/
    /* the sd-bus calls an SDIO card through this, as it does a memory card */
    SDCardClass parent_class;
    /*< public >*/

    uint16_t vendor;        /* CISTPL_MANFID, what drivers match on */
    uint16_t device;
    uint8_t num_funcs;
    uint8_t func_class;     /* FBR standard function interface code */
    uint16_t max_blksz;

    /*
     * Function 1-7 register space. A CMD52 moves one byte; a CMD53 hands
     * over its whole transfer at once, with @incr set when the address
     * counts up across it rather than staying on one register.
     */
    void (*read)(SDIOCard *card, unsigned fn, uint32_t addr, bool incr,
                 uint8_t *buf, size_t len);
    void (*write)(SDIOCard *card, unsigned fn, uint32_t addr, bool incr,
                  const uint8_t *buf, size_t len);
    /* CCCR I/O abort RES: the functions start over */
    void (*io_reset)(SDIOCard *card);
};

struct SDIOCard {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    uint16_t rca;
    uint8_t state;
    uint8_t io_enable;
    uint8_t int_enable;
    uint8_t int_pending;    /* bit n: function n wants service */
    uint8_t bus_if;
    uint8_t speed;
    uint16_t blksz[SDIO_MAX_FUNCS + 1];
    bool irq_level;

    /* the CMD53 whose data phase is under way */
    bool xfer_write;
    bool xfer_incr;
    uint8_t xfer_fn;
    uint32_t xfer_addr;
    uint32_t xfer_len;
    uint32_t xfer_pos;
    uint8_t *xfer_buf;
};

/* A function raises or drops its interrupt */
void sdio_card_set_irq(SDIOCard *card, unsigned fn, bool level);

#endif
