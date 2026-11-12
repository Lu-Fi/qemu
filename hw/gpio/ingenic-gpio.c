/*
 * Ingenic XBurst GPIO controller emulation
 *
 * The kernel ioremap()s one 64 KiB region and treats it as N ports, each
 * at i * port-stride (0x1000; 0x100 on the T10 and T20). Each port has
 * paired set/clear registers around a base value (PXMSK at 0x20, PXMSKS
 * at 0x24 sets bits, PXMSKC at 0x28 clears them).
 *
 * The T21, T23, T30 and T31 drivers stage a pin-function change in the
 * PZ shadow page at 0x7000 (PZINTS, PZMSKC, ...) and apply it to one
 * port by writing that port's number to PZGID2LD: only the pins staged
 * change. The T10 and T20 drivers write the port registers directly.
 *
 * A pad reads (PXPIN) the level the GPIO drives when it is an output
 * (INT 0, MSK 1, PAT1 0: PAT0) and otherwise the level outside drives
 * onto it, "pinval" (pulled high) until the in-pX properties say else.
 * An interrupt pad (INT 1) latches PXFLG on its trigger, PAT1:PAT0 =
 * 00 low level, 01 high level, 10 falling edge, 11 rising edge, and the
 * port interrupts while an unmasked pad has its flag set. Only pads a
 * test has driven latch: nothing is wired to the rest, and a chip's
 * interrupt line that idles low would otherwise read high and fire for
 * ever.
 *
 * Each family's pad registers from 0x60 on (pulls, drive strength, slew
 * rate, Schmitt triggers, glitch filters, power domain, ...) hold what the
 * guest writes, with set and clear ports at +4 and +8 as the pulls have;
 * the T32's PxUPD, off a 0x10 boundary, has none. The T30's, measured,
 * keep only the bits of the pins a port has (two a pin in the pulls and
 * drive strength), and its 0x100 drops a write. The T20's pull disable at
 * 0x60, which the T21's and T30's vendor U-Boot still writes, reads 0 and
 * drops writes on both. A reset gives each port
 * the values its part's bootrom leaves on silicon, as the bootrom's USB
 * read shows them, less what that bootrom writes itself: the pins it puts
 * in a function for flash and cards (from its run in the model, and on the
 * XBurst1 parts' PC2-4 and the T23's PB8-10, which a NOR boot leaves as
 * inputs) are inputs again, as it, or the boot's stand-in, sets them up
 * once more. The T23, T31 and T41 variants measured read alike; a T33 has
 * a T32LQ's, a T40N a T40XP's.
 *
 * Properties in-pa .. in-pf set the outside levels: with -global before
 * the machine runs (a card in its slot) or at run time (a test pressing a
 * button). A reset keeps them, as it does not reach outside the chip.
 * pin-pX, int-pX, msk-pX, pat1-pX, pat0-pX and flg-pX read the port back.
 *
 * Wires: "pad" outputs (32 per port) give the level each pad drives,
 * high while it is not a GPIO output, and "pull" inputs let an open-drain
 * device on the wire hold the pad low, as bit-banged I2C needs, or a card
 * socket's switch; what they hold a reset keeps, as it keeps in-pX.
 *
 * With "trace" set to a chardev, each pin whose function or level changes
 * writes a JSON line there: the virtual time in ns, the pin (PB14) and its
 * Linux number, what it now is (out, in, irq or func) with its level,
 * trigger or function, and the guest PC that changed it, as the CPU last
 * saved it (the start of its translation block), or "src":"ext" when the
 * change came from outside the guest (in-pX or a device on the wire). A
 * reset writes {"reset":...} first; until a pin is traced it keeps its
 * reset state.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/gpio/ingenic-gpio.h"
#include "migration/vmstate.h"

/* Per-port register offsets within a port page; the PZ page matches */
#define PXPIN       0x000
#define PXINT       0x010
#define PXINTS      0x014
#define PXINTC      0x018
#define PXMSK       0x020
#define PXMSKS      0x024
#define PXMSKC      0x028
#define PXPAT1      0x030
#define PXPAT1S     0x034
#define PXPAT1C     0x038
#define PXPAT0      0x040
#define PXPAT0S     0x044
#define PXPAT0C     0x048
#define PXFLG       0x050
#define PXFLGC      0x058
#define PZGID2LD    0x0F0

typedef struct IngenicGpioReset {
    uint8_t port;
    uint16_t reg;
    uint32_t value;
} IngenicGpioReset;

/* Which of a pad register's bits a port keeps, where measured */
enum { PAD_PIN = 1, PAD_LO, PAD_HI, PAD_RO };

typedef struct IngenicGpioPadBits {
    uint16_t reg;
    uint8_t bits;
} IngenicGpioPadBits;

struct IngenicGpioFamily {
    const char *name;
    uint16_t pads[16];                  /* its pad registers, ending in 0 */
    const IngenicGpioReset *reset;      /* ending in a port of 0xff */
    const uint32_t *pins;               /* the pins each port has */
    const IngenicGpioPadBits *bits;     /* ending in a reg of 0 */
    uint16_t absent;                    /* written by firmware, not there */
};

static const IngenicGpioReset ingenic_gpio_reset_t10[] = {
    { 0, PXMSK, 0x1fffffff }, { 0, PXPAT1, 0x1fffffff },
    { 0, 0x060, 0x007ff000 }, { 1, PXMSK, 0xfe37efff },
    { 1, PXPAT1, 0xfe37efff }, { 1, PXPAT0, 0x01c80000 },
    { 1, 0x060, 0xff06040e }, { 2, PXMSK, 0x0007fbff },
    { 2, PXPAT1, 0x0007fbff }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t20[] = {
    { 0, PXMSK, 0x1fdfffff }, { 0, PXPAT1, 0x1fdfffff },
    { 0, 0x060, 0x005ff000 }, { 1, PXMSK, 0xfe37efff },
    { 1, PXPAT1, 0xfe37efff }, { 1, PXPAT0, 0x01c80000 },
    { 1, 0x060, 0xff06040e }, { 2, PXMSK, 0x0007fbff },
    { 2, PXPAT1, 0x0007fbff }, { 2, 0x060, 0x00010000 }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t21[] = {
    { 0, PXMSK, 0x1fc7ffff }, { 0, PXPAT1, 0x1fc7ffff },
    { 0, 0x110, 0x05400000 }, { 0, 0x120, 0x01555000 },
    { 0, 0x130, 0xaaaaaaaa }, { 0, 0x140, 0x02aaa02a },
    { 0, 0x150, 0x00008000 }, { 0, 0x160, 0x00400000 },
    { 0, 0x180, 0x1fc7ffff }, { 1, PXMSK, 0x9e37efff },
    { 1, PXPAT1, 0x9e37efff }, { 1, PXPAT0, 0x01c80000 },
    { 1, 0x110, 0x10600000 }, { 1, 0x120, 0x80150064 },
    { 1, 0x130, 0xa8aaaaaa }, { 1, 0x140, 0x82aaaaaa },
    { 1, 0x160, 0x18000000 }, { 1, 0x180, 0x9fffefff },
    { 2, PXMSK, 0x0007fbff }, { 2, PXPAT1, 0x0007fbff },
    { 2, 0x110, 0x90050009 }, { 2, 0x120, 0x0000002a },
    { 2, 0x130, 0xaa8aaaaa }, { 2, 0x140, 0x0000002a },
    { 2, 0x160, 0x00078000 }, { 2, 0x180, 0x0007fbff }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t23[] = {
    { 0, PXMSK, 0x1987f0c0 }, { 0, PXPAT1, 0x1987f0c0 },
    { 0, 0x110, 0x19803000 }, { 0, 0x130, 0x95005000 },
    { 0, 0x140, 0x02828015 }, { 0, 0x150, 0x00008000 },
    { 1, PXMSK, 0xffffefff }, { 1, PXPAT1, 0xffffefff },
    { 1, 0x110, 0x01818000 }, { 1, 0x120, 0x80000000 },
    { 1, 0x130, 0x94559655 }, { 1, 0x140, 0x55555556 },
    { 1, 0x160, 0x60000000 }, { 2, PXMSK, 0x0003ffff },
    { 2, PXPAT1, 0x0003ffff }, { 2, 0x110, 0x00000001 },
    { 2, 0x120, 0x00000002 }, { 2, 0x130, 0x55555555 },
    { 2, 0x140, 0x00000005 }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t30[] = {
    { 0, PXMSK, 0x1fdfffff }, { 0, PXPAT1, 0x1fdfffff },
    { 0, 0x110, 0x05400000 }, { 0, 0x120, 0x00001000 },
    { 1, PXMSK, 0xfe37efff }, { 1, PXPAT1, 0xfe37efff },
    { 1, PXPAT0, 0x01c80000 }, { 1, 0x110, 0x00600000 },
    { 1, 0x120, 0x80150068 }, { 2, PXMSK, 0x1ffffbff },
    { 2, PXPAT1, 0x1ffffbff }, { 2, 0x110, 0x000a0009 },
    { 2, 0x120, 0x02a90001 }, { 3, PXMSK, 0x0fcff3fc },
    { 3, PXPAT1, 0x0fcff3fc }, { 3, 0x110, 0x00004000 },
    { 3, 0x120, 0x00000010 }, { 0, 0x100, 0x00000001 },
    { 0, 0x130, 0xaaaaaaaa }, { 0, 0x140, 0x02aaa2aa },
    { 0, 0x150, 0x00008000 }, { 0, 0x160, 0x00580000 },
    { 0, 0x180, 0x1fdfffff }, { 1, 0x130, 0xa8aaaaaa },
    { 1, 0x140, 0xaaaaaaaa }, { 1, 0x160, 0x78000000 },
    { 1, 0x180, 0xffffefff }, { 2, 0x130, 0xaa8aaaaa },
    { 2, 0x140, 0x02aaaaaa }, { 2, 0x160, 0x01060300 },
    { 2, 0x180, 0x1ffffbff }, { 3, 0x130, 0xaa0aaaa0 },
    { 3, 0x140, 0x00aaa0aa }, { 3, 0x180, 0x0fcff3fc }, { 0xff }
};
static const uint32_t ingenic_gpio_pins_t30[] = {
    0x1fdfffff, 0xffffefff, 0x1ffffbff, 0x0fcff3fc
};
static const IngenicGpioPadBits ingenic_gpio_bits_t30[] = {
    { 0x100, PAD_RO }, { 0x110, PAD_LO }, { 0x120, PAD_HI },
    { 0x130, PAD_LO }, { 0x140, PAD_HI }, { 0x150, PAD_PIN },
    { 0x160, PAD_PIN }, { 0x180, PAD_PIN }, { 0 }
};
static const IngenicGpioReset ingenic_gpio_reset_t31[] = {
    { 0, PXMSK, 0x1fc7ffc0 }, { 0, PXPAT1, 0x1fc7ffc0 },
    { 0, 0x110, 0x1fc33800 }, { 0, 0x130, 0xaaaaa000 },
    { 0, 0x140, 0x02aaa02a }, { 0, 0x150, 0x00008000 },
    { 0, 0x160, 0x00400000 }, { 1, PXMSK, 0xfe37efff },
    { 1, PXPAT1, 0xfe37efff }, { 1, PXPAT0, 0x01c80000 },
    { 1, 0x110, 0x67094800 }, { 1, 0x120, 0x98060400 },
    { 1, 0x160, 0x60000000 }, { 2, PXMSK, 0x07fffbff },
    { 2, PXPAT1, 0x07fffbff }, { 2, 0x110, 0x00004309 },
    { 2, 0x120, 0x00060002 }, { 2, 0x160, 0x001e0000 }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t32[] = {
    { 0, PXMSK, 0x1ffffc00 }, { 0, PXPAT1, 0x1ffffc00 },
    { 0, 0x110, 0x1f80fc00 }, { 0, 0x130, 0x55500000 },
    { 0, 0x140, 0x02aa955a }, { 1, PXMSK, 0xffffefff },
    { 1, PXPAT1, 0xffffefff }, { 1, 0x110, 0x01800000 },
    { 1, 0x130, 0x54559655 }, { 1, 0x140, 0x55555555 },
    { 2, PXMSK, 0x83fffbff }, { 2, PXPAT1, 0x83fffbff },
    { 2, 0x110, 0x00000001 }, { 2, 0x120, 0x00000002 },
    { 2, 0x130, 0x55855565 }, { 2, 0x140, 0x00555555 }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t40[] = {
    { 0, PXMSK, 0x1fffffff }, { 0, PXPAT1, 0x1fffffff },
    { 0, 0x110, 0x14000000 }, { 0, 0x130, 0x1f800000 },
    { 1, PXMSK, 0xffffefff }, { 1, PXPAT1, 0xffffefff },
    { 1, 0x110, 0x41800000 }, { 1, 0x120, 0x800000c0 },
    { 1, 0x170, 0x06000000 }, { 2, PXMSK, 0xffffffff },
    { 2, PXPAT1, 0xffffffff }, { 2, 0x110, 0x00200001 },
    { 2, 0x120, 0x00100002 }, { 2, 0x170, 0x00300000 },
    { 2, 0x180, 0x00300000 }, { 3, PXMSK, 0x0c3fffff },
    { 3, PXPAT1, 0x0c3fffff }, { 3, PXPAT0, 0x03c00000 }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_t41[] = {
    { 0, PXMSK, 0xffffffc0 }, { 0, PXPAT1, 0xffffffc0 },
    { 0, 0x110, 0xfff00000 }, { 0, 0x130, 0x000fffc0 },
    { 0, 0x140, 0xfff00000 }, { 1, PXMSK, 0xfe4fefff },
    { 1, PXPAT1, 0xffffefff }, { 1, 0x130, 0xffffefff },
    { 2, PXMSK, 0x001ffbff }, { 2, PXPAT1, 0x001ffbff },
    { 2, 0x110, 0x00000001 }, { 2, 0x120, 0x00000002 },
    { 2, 0x130, 0x00000003 }, { 3, PXMSK, 0x00001fff },
    { 3, PXPAT1, 0x00001fff }, { 3, 0x130, 0x00001fff }, { 0xff }
};
static const IngenicGpioReset ingenic_gpio_reset_a1[] = {
    { 0, PXMSK, 0xffffffff }, { 0, PXPAT1, 0xffffffff },
    { 1, PXMSK, 0xff03ffff }, { 1, PXPAT1, 0xff03ffff },
    { 1, 0x110, 0x00000a00 }, { 2, PXMSK, 0x0ff0ffff },
    { 2, PXPAT1, 0x0ff0ffff }, { 2, PXPAT0, 0x000f0000 },
    { 2, 0x110, 0x02400001 }, { 2, 0x120, 0x00000002 },
    { 2, 0x130, 0x0ff00000 }, { 4, PXMSK, 0x0001ffff },
    { 4, PXPAT1, 0x0001ffff }, { 4, 0x130, 0x0001ffff }, { 0xff }
};

static const IngenicGpioFamily ingenic_gpio_families[] = {
    /* pulls off, pull direction */
    { "t10", { 0x060, 0x090 }, ingenic_gpio_reset_t10 },
    { "t20", { 0x060, 0x090 }, ingenic_gpio_reset_t20 },
    /*
     * glitch filters 0-3, VDDIO, pulls up and down, drive strength low and
     * high, slew rate, Schmitt trigger, receiver enable
     */
    { "t21", { 0x070, 0x080, 0x090, 0x0a0, 0x100, 0x110, 0x120, 0x130,
               0x140, 0x150, 0x160, 0x180 }, ingenic_gpio_reset_t21,
      .absent = 0x060 },
    /* power domain, pulls, drive strength low and high, slew, Schmitt */
    { "t23", { 0x100, 0x110, 0x120, 0x130, 0x140, 0x150, 0x160 },
      ingenic_gpio_reset_t23 },
    /*
     * VDDIO, pulls and drive strength (two bits a pin: pins 0-15, then
     * 16-31), slew rate, Schmitt trigger, receiver enable
     */
    { "t30", { 0x100, 0x110, 0x120, 0x130, 0x140, 0x150, 0x160, 0x180 },
      ingenic_gpio_reset_t30, ingenic_gpio_pins_t30, ingenic_gpio_bits_t30,
      0x060 },
    { "t31", { 0x100, 0x110, 0x120, 0x130, 0x140, 0x150, 0x160 },
      ingenic_gpio_reset_t31 },
    /*
     * glitch filters, power domain, pulls, drive strength 0 and 1, slew,
     * Schmitt, shadow mode and its update
     */
    { "t32", { 0x070, 0x080, 0x090, 0x0a0, 0x100, 0x110, 0x120, 0x130,
               0x140, 0x150, 0x160, 0x1d0, 0x1e4 }, ingenic_gpio_reset_t32 },
    { "t33", { 0x070, 0x080, 0x090, 0x0a0, 0x100, 0x110, 0x120, 0x130,
               0x140, 0x150, 0x160, 0x1d0, 0x1e4 }, ingenic_gpio_reset_t32 },
    /*
     * glitch filters, power domain, pulls, drive strength 0-2, slew,
     * Schmitt, Schmitt 1, hold
     */
    { "t40", { 0x070, 0x080, 0x090, 0x0a0, 0x100, 0x110, 0x120, 0x130,
               0x140, 0x150, 0x160, 0x170, 0x180, 0x190 },
      ingenic_gpio_reset_t40 },
    /*
     * glitch filters, power domain, pulls, drive strength 0-2, slew,
     * Schmitt, drive strength 3, strong pull-up
     */
    { "t41", { 0x070, 0x080, 0x090, 0x0a0, 0x100, 0x110, 0x120, 0x130,
               0x140, 0x150, 0x160, 0x170, 0x1a0, 0x1b0 },
      ingenic_gpio_reset_t41 },
    { "a1", { 0x070, 0x080, 0x090, 0x0a0, 0x100, 0x110, 0x120, 0x130,
              0x140, 0x150, 0x160, 0x170, 0x180, 0x190 },
      ingenic_gpio_reset_a1 },
};

enum { R_INT, R_MSK, R_PAT1, R_PAT0, R_FLG, R_PIN };

static uint32_t *gpio_reg(IngenicGpioPort *p, int r)
{
    switch (r) {
    case R_INT:
        return &p->intr;
    case R_MSK:
        return &p->msk;
    case R_PAT1:
        return &p->pat1;
    case R_PAT0:
        return &p->pat0;
    case R_FLG:
        return &p->flg;
    default:
        return &p->in;
    }
}

/* The pad register @po names, and the port it is (0, set 4 or clear 8) */
static uint32_t *gpio_pad(IngenicGpioState *s, IngenicGpioPort *p,
                          hwaddr po, unsigned *op)
{
    for (const uint16_t *r = s->fam->pads; *r; r++) {
        if (po == *r || (!(*r & 0xf) && (po == *r + 4 || po == *r + 8))) {
            *op = po - *r;
            return &p->pad[(*r - INGENIC_GPIO_PAD_BASE) >> 4];
        }
    }
    return NULL;
}

/* Two bits a pin, for 16 pins */
static uint32_t gpio_pairs(uint32_t pins)
{
    uint32_t m = 0;

    for (unsigned i = 0; i < 16; i++) {
        if (pins & BIT(i)) {
            m |= 3u << (2 * i);
        }
    }
    return m;
}

/* The bits of pad register @reg that port @n keeps */
static uint32_t gpio_pad_keep(IngenicGpioState *s, unsigned n, hwaddr reg)
{
    for (const IngenicGpioPadBits *b = s->fam->bits; b && b->reg; b++) {
        if (b->reg != reg) {
            continue;
        }
        switch (b->bits) {
        case PAD_PIN:
            return s->fam->pins[n];
        case PAD_LO:
            return gpio_pairs(s->fam->pins[n] & 0xffff);
        case PAD_HI:
            return gpio_pairs(s->fam->pins[n] >> 16);
        default:
            return 0;
        }
    }
    return UINT32_MAX;
}

static uint32_t gpio_outputs(IngenicGpioPort *p)
{
    return ~p->intr & p->msk & ~p->pat1;
}

static uint32_t gpio_level(IngenicGpioPort *p)
{
    uint32_t out = gpio_outputs(p);

    return (out & p->pat0) | (~out & p->in);
}

/* Last, as a pad's wire can pull another pad of the port and update it */
static void gpio_drive(IngenicGpioState *s, unsigned n)
{
    IngenicGpioPort *p = &s->port[n];
    uint32_t drive = ~gpio_outputs(p) | p->pat0;
    uint32_t changed = drive ^ p->drive;

    p->drive = drive;
    while (changed) {
        int i = ctz32(changed);

        changed &= changed - 1;
        qemu_set_irq(s->pad[n * 32 + i], extract32(drive, i, 1));
    }
}

/*
 * What pin @i is, from INT, MSK, PAT1, PAT0 and the level (r[0..4]): a
 * device function's level is the device's business, not the trace's.
 */
static unsigned gpio_pin_state(const uint32_t *r, int i)
{
    unsigned intr = extract32(r[0], i, 1), msk = extract32(r[1], i, 1);
    unsigned pat = extract32(r[2], i, 1) << 1 | extract32(r[3], i, 1);
    unsigned level = extract32(r[4], i, 1);

    if (intr) {
        return 0x30 | pat << 2 | msk << 1 | level;
    }
    if (!msk) {
        return pat;
    }
    return pat & 2 ? 0x20 | level : 0x10 | (pat & 1);
}

static void gpio_trace(IngenicGpioState *s, unsigned n)
{
    static const char *const trig[] = { "low", "high", "falling", "rising" };
    IngenicGpioPort *p = &s->port[n];
    uint32_t now[5] = { p->intr, p->msk, p->pat1, p->pat0, gpio_level(p) };
    g_autoptr(GString) js = NULL;

    if (!qemu_chr_fe_backend_connected(&s->trace)) {
        return;
    }
    for (int i = 0; i < 32; i++) {
        unsigned was = gpio_pin_state(p->traced, i);
        unsigned st = gpio_pin_state(now, i);

        if (st == was) {
            continue;
        }
        if (!js) {
            js = g_string_new(NULL);
        }
        g_string_append_printf(js, "{\"t\":%" PRId64 ",\"pin\":\"P%c%d\","
                               "\"gpio\":%u,",
                               qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                               'A' + n, i, n * 32 + i);
        switch (st >> 4) {
        case 0:
            g_string_append_printf(js, "\"fn\":\"func\",\"func\":%u", st);
            break;
        case 1:
            g_string_append_printf(js, "\"fn\":\"out\",\"lvl\":%u", st & 1);
            break;
        case 2:
            g_string_append_printf(js, "\"fn\":\"in\",\"lvl\":%u", st & 1);
            break;
        default:
            g_string_append_printf(js, "\"fn\":\"irq\",\"trig\":\"%s\","
                                   "\"mask\":%u,\"lvl\":%u",
                                   trig[st >> 2 & 3], st >> 1 & 1, st & 1);
            break;
        }
        if (s->trace_ext || !current_cpu) {
            g_string_append(js, ",\"src\":\"ext\"}\n");
        } else {
            g_string_append_printf(js, ",\"pc\":\"0x%08" PRIx64 "\"}\n",
                                   (uint64_t)CPU_GET_CLASS(current_cpu)->
                                   get_pc(current_cpu));
        }
    }
    memcpy(p->traced, now, sizeof(now));
    if (js) {
        qemu_chr_fe_write_all(&s->trace, (uint8_t *)js->str, js->len);
    }
}

static void gpio_update(IngenicGpioState *s, unsigned n)
{
    IngenicGpioPort *p = &s->port[n];
    uint32_t level = gpio_level(p);
    uint32_t hit = ~(level ^ p->pat0) & p->intr & p->driven;

    p->flg |= hit & ~p->pat1;
    p->flg |= hit & p->pat1 & (level ^ p->last);
    p->last = level;
    qemu_set_irq(s->irq[n], !!(p->flg & p->intr & ~p->msk));
    gpio_trace(s, n);
    gpio_drive(s, n);
}

static void gpio_pull(void *opaque, int line, int level)
{
    IngenicGpioState *s = opaque;
    IngenicGpioPort *p = &s->port[line / 32];
    uint32_t bit = BIT(line % 32);
    bool ext = s->trace_ext;

    p->driven |= bit;
    p->in = (p->in & ~bit) | (level ? s->pinval & bit : 0);
    s->pulled[line / 32] = (s->pulled[line / 32] & ~bit) | (level ? 0 : bit);
    s->trace_ext = true;
    gpio_update(s, line / 32);
    s->trace_ext = ext;
}

/* PZGID2LD: apply the staged changes to port @n and forget them */
static void gpio_load(IngenicGpioState *s, unsigned n)
{
    if (n < s->nr_ports) {
        for (int r = R_INT; r <= R_PAT0; r++) {
            uint32_t *reg = gpio_reg(&s->port[n], r);

            *reg = (*reg | s->zset[r]) & ~s->zclr[r];
        }
        gpio_update(s, n);
    }
    memset(s->zset, 0, sizeof(s->zset));
    memset(s->zclr, 0, sizeof(s->zclr));
}

static void gpio_shadow_write(IngenicGpioState *s, hwaddr off, uint32_t v)
{
    int r = (off >> 4) - 1;                 /* 0x10 INT .. 0x40 PAT0 */

    if (off == PZGID2LD) {
        gpio_load(s, v & 7);
        return;
    }
    if (off < PXINT || off > PXPAT0C) {
        return;
    }
    switch (off & 0xf) {
    case 0x0:
        s->zset[r] = v;
        s->zclr[r] = ~v;
        break;
    case 0x4:
        s->zset[r] |= v;
        s->zclr[r] &= ~v;
        break;
    case 0x8:
        s->zclr[r] |= v;
        s->zset[r] &= ~v;
        break;
    }
}

static uint64_t ingenic_gpio_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    IngenicGpioState *s = INGENIC_GPIO(opaque);
    unsigned n = offset / s->port_stride, op;
    IngenicGpioPort *p;
    uint32_t *pad;

    if (offset >= INGENIC_GPIO_SHADOW_OFF || n >= s->nr_ports) {
        return 0;
    }
    p = &s->port[n];
    pad = gpio_pad(s, p, offset % s->port_stride, &op);
    if (pad) {
        return *pad;
    }
    switch (offset % s->port_stride) {
    case PXPIN:
        return gpio_level(p);
    case PXINT: case PXINTS: case PXINTC:
        return p->intr;
    case PXMSK: case PXMSKS: case PXMSKC:
        return p->msk;
    case PXPAT1: case PXPAT1S: case PXPAT1C:
        return p->pat1;
    case PXPAT0: case PXPAT0S: case PXPAT0C:
        return p->pat0;
    case PXFLG: case PXFLGC:
        return p->flg;
    default:
        return 0;
    }
}

static void ingenic_gpio_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    IngenicGpioState *s = INGENIC_GPIO(opaque);
    unsigned n = offset / s->port_stride, op;
    hwaddr po = offset % s->port_stride;
    uint32_t v = (uint32_t)value;
    IngenicGpioPort *p;
    uint32_t *pad;

    if (offset >= INGENIC_GPIO_SHADOW_OFF &&
        offset < INGENIC_GPIO_SHADOW_OFF + s->port_stride) {
        gpio_shadow_write(s, offset - INGENIC_GPIO_SHADOW_OFF, v);
        return;
    }
    if (n >= s->nr_ports) {
        return;
    }
    p = &s->port[n];
    pad = gpio_pad(s, p, po, &op);
    if (pad) {
        uint32_t keep = gpio_pad_keep(s, n, po - op);

        v = op == 4 ? *pad | v : op == 8 ? *pad & ~v : v;
        *pad = (*pad & ~keep) | (v & keep);
        return;
    }
    if (s->fam->absent && po >= s->fam->absent && po <= s->fam->absent + 8) {
        return;
    }
    switch (po) {
    case PXPIN:
        /* read-only on real HW; ignore writes */
        break;
    case PXINT:
        p->intr = v;
        break;
    case PXINTS:
        p->intr |= v;
        break;
    case PXINTC:
        p->intr &= ~v;
        break;
    case PXMSK:
        p->msk = v;
        break;
    case PXMSKS:
        p->msk |= v;
        break;
    case PXMSKC:
        p->msk &= ~v;
        break;
    case PXPAT1:
        p->pat1 = v;
        break;
    case PXPAT1S:
        p->pat1 |= v;
        break;
    case PXPAT1C:
        p->pat1 &= ~v;
        break;
    case PXPAT0:
        p->pat0 = v;
        break;
    case PXPAT0S:
        p->pat0 |= v;
        break;
    case PXPAT0C:
        p->pat0 &= ~v;
        break;
    case PXFLG:
        p->flg = v;
        break;
    case PXFLGC:
        p->flg &= ~v;
        break;
    case PZGID2LD:
        /* The T10 and T20 drivers write it here; nothing is staged */
        gpio_load(s, v & 7);
        return;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, offset, v);
        return;
    }
    gpio_update(s, n);
}

static const MemoryRegionOps ingenic_gpio_ops = {
    .read = ingenic_gpio_read,
    .write = ingenic_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* pin-pX, int-pX, ... (read only): opaque = n << 8 | r */
static void gpio_prop_get(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    IngenicGpioState *s = INGENIC_GPIO(obj);
    IngenicGpioPort *p = &s->port[GPOINTER_TO_UINT(opaque) >> 8];
    int r = GPOINTER_TO_UINT(opaque) & 0xff;
    uint32_t value = r == R_PIN ? gpio_level(p) : *gpio_reg(p, r);

    visit_type_uint32(v, name, &value, errp);
}

static void gpio_prop_get_in(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    visit_type_uint32(v, name, object_field_prop_ptr(obj, opaque), errp);
}

static void gpio_prop_set_in(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    IngenicGpioState *s = INGENIC_GPIO(obj);
    uint32_t *outside = object_field_prop_ptr(obj, opaque);
    unsigned n = outside - s->outside;
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    *outside = value;
    if (DEVICE(obj)->realized) {
        s->port[n].driven |= value ^ s->port[n].in;
        s->port[n].in = value;
        s->trace_ext = true;
        gpio_update(s, n);
        s->trace_ext = false;
    }
}

static const PropertyInfo gpio_prop_in = {
    .type = "uint32",
    .description = "the level outside drives onto each pad",
    .get = gpio_prop_get_in,
    .set = gpio_prop_set_in,
    .realized_set_allowed = true,
};

static void ingenic_gpio_realize(DeviceState *dev, Error **errp)
{
    static const char *const names[] = {
        [R_INT] = "int", [R_MSK] = "msk", [R_PAT1] = "pat1",
        [R_PAT0] = "pat0", [R_FLG] = "flg", [R_PIN] = "pin",
    };
    IngenicGpioState *s = INGENIC_GPIO(dev);
    const char *family = s->family ?: "t31";

    if (s->nr_ports < 1 || s->nr_ports > INGENIC_GPIO_MAX_PORTS) {
        error_setg(errp, "nr-ports must be 1 to %d", INGENIC_GPIO_MAX_PORTS);
        return;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(ingenic_gpio_families); i++) {
        if (!strcmp(ingenic_gpio_families[i].name, family)) {
            s->fam = &ingenic_gpio_families[i];
        }
    }
    if (!s->fam) {
        error_setg(errp, "unknown family '%s'", family);
        return;
    }
    for (unsigned n = 0; n < s->nr_ports; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[n]);
        for (int r = R_INT; r <= R_PIN; r++) {
            g_autofree char *prop = g_strdup_printf("%s-p%c", names[r],
                                                    'a' + n);

            object_property_add(OBJECT(dev), prop, "uint32", gpio_prop_get,
                                NULL, NULL, GUINT_TO_POINTER(n << 8 | r));
        }
    }
    qdev_init_gpio_out_named(dev, s->pad, "pad", s->nr_ports * 32);
    qdev_init_gpio_in_named(dev, gpio_pull, "pull", s->nr_ports * 32);
}

static void ingenic_gpio_reset_hold(Object *obj, ResetType type)
{
    IngenicGpioState *s = INGENIC_GPIO(obj);

    memset(s->port, 0, sizeof(s->port));
    memset(s->zset, 0, sizeof(s->zset));
    memset(s->zclr, 0, sizeof(s->zclr));
    for (const IngenicGpioReset *r = s->fam->reset; r->port != 0xff; r++) {
        IngenicGpioPort *p = &s->port[r->port];
        unsigned op;

        if (r->reg <= PXPAT0) {
            *gpio_reg(p, (r->reg >> 4) - 1) = r->value;
        } else {
            *gpio_pad(s, p, r->reg, &op) = r->value;
        }
    }
    for (unsigned n = 0; n < INGENIC_GPIO_MAX_PORTS; n++) {
        s->port[n].in = s->outside[n] & s->pinval & ~s->pulled[n];
        s->port[n].driven = (s->pinval & ~s->outside[n]) | s->pulled[n];
        s->port[n].last = s->port[n].in;
        s->port[n].drive = UINT32_MAX;
    }
}

static void ingenic_gpio_reset_exit(Object *obj, ResetType type)
{
    IngenicGpioState *s = INGENIC_GPIO(obj);

    if (qemu_chr_fe_backend_connected(&s->trace)) {
        g_autofree char *js = g_strdup_printf(
            "{\"t\":%" PRId64 ",\"reset\":true,\"ports\":%u}\n",
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), s->nr_ports);

        qemu_chr_fe_write_all(&s->trace, (uint8_t *)js, strlen(js));
    }
    for (unsigned n = 0; n < s->nr_ports; n++) {
        gpio_update(s, n);
    }
}

static void ingenic_gpio_init(Object *obj)
{
    IngenicGpioState *s = INGENIC_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_gpio_ops, s,
                          TYPE_INGENIC_GPIO,
                          INGENIC_GPIO_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (unsigned n = 0; n < INGENIC_GPIO_MAX_PORTS; n++) {
        s->outside[n] = UINT32_MAX;
    }
}

static const Property ingenic_gpio_props[] = {
    DEFINE_PROP_UINT32("port-stride", IngenicGpioState, port_stride,
                       0x1000),
    DEFINE_PROP_UINT32("nr-ports", IngenicGpioState, nr_ports, 3),
    DEFINE_PROP_UINT32("pinval", IngenicGpioState, pinval, 0xFFFFFFFF),
    DEFINE_PROP_STRING("family", IngenicGpioState, family),
    DEFINE_PROP_CHR("trace", IngenicGpioState, trace),
    DEFINE_PROP("in-pa", IngenicGpioState, outside[0], gpio_prop_in, uint32_t),
    DEFINE_PROP("in-pb", IngenicGpioState, outside[1], gpio_prop_in, uint32_t),
    DEFINE_PROP("in-pc", IngenicGpioState, outside[2], gpio_prop_in, uint32_t),
    DEFINE_PROP("in-pd", IngenicGpioState, outside[3], gpio_prop_in, uint32_t),
    DEFINE_PROP("in-pe", IngenicGpioState, outside[4], gpio_prop_in, uint32_t),
    DEFINE_PROP("in-pf", IngenicGpioState, outside[5], gpio_prop_in, uint32_t),
};

static const VMStateDescription vmstate_ingenic_gpio_port = {
    .name = TYPE_INGENIC_GPIO "/port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(intr, IngenicGpioPort),
        VMSTATE_UINT32(msk, IngenicGpioPort),
        VMSTATE_UINT32(pat1, IngenicGpioPort),
        VMSTATE_UINT32(pat0, IngenicGpioPort),
        VMSTATE_UINT32(flg, IngenicGpioPort),
        VMSTATE_UINT32_ARRAY(pad, IngenicGpioPort, INGENIC_GPIO_PADS),
        VMSTATE_UINT32(in, IngenicGpioPort),
        VMSTATE_UINT32(driven, IngenicGpioPort),
        VMSTATE_UINT32(last, IngenicGpioPort),
        VMSTATE_UINT32(drive, IngenicGpioPort),
        VMSTATE_UINT32_ARRAY(traced, IngenicGpioPort, 5),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_gpio = {
    .name = TYPE_INGENIC_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(port, IngenicGpioState, INGENIC_GPIO_MAX_PORTS,
                             1, vmstate_ingenic_gpio_port, IngenicGpioPort),
        VMSTATE_UINT32_ARRAY(outside, IngenicGpioState,
                             INGENIC_GPIO_MAX_PORTS),
        VMSTATE_UINT32_ARRAY(pulled, IngenicGpioState, INGENIC_GPIO_MAX_PORTS),
        VMSTATE_UINT32_ARRAY(zset, IngenicGpioState, 4),
        VMSTATE_UINT32_ARRAY(zclr, IngenicGpioState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_gpio_realize;
    dc->vmsd = &vmstate_ingenic_gpio;
    rc->phases.hold = ingenic_gpio_reset_hold;
    rc->phases.exit = ingenic_gpio_reset_exit;
    device_class_set_props(dc, ingenic_gpio_props);
}

static const TypeInfo ingenic_gpio_type_info = {
    .name = TYPE_INGENIC_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicGpioState),
    .instance_init = ingenic_gpio_init,
    .class_init = ingenic_gpio_class_init,
};

static void ingenic_gpio_register_types(void)
{
    type_register_static(&ingenic_gpio_type_info);
}

type_init(ingenic_gpio_register_types)
