/*
 * Ingenic CPM: the CPU clock that CPCCR and the PLLs give
 *
 * Checked against the CPM registers of real T10, T20, T21, T23, T30, T31,
 * T40, T41 and A1 boards, read back at the U-Boot prompt.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_CPM_CLK_H
#define HW_MISC_INGENIC_CPM_CLK_H

#include "qemu/bitops.h"

#define INGENIC_EXTCLK_HZ   24000000ULL

/* How a SoC family encodes its PLL control registers */
typedef enum {
    INGENIC_PLL_MNOD,       /* EXTCLK * M / N / OD1 / OD0 */
    INGENIC_PLL_MNOD_FRAC,  /* the same with a 24-bit fraction of M (T10, T20) */
    INGENIC_PLL_PLLREG,     /* EXTCLK * 2(M + 1) / ((N + 1) 2^OD0 (OD1 + 1)) */
    INGENIC_PLL_MNOD_OD1,   /* MNOD with OD1 and OD0 less one (T32Pro MPLL) */
} IngenicPllFormat;

/* A PLL's output in Hz, 0 while it is disabled */
static inline uint64_t ingenic_pll_hz(uint32_t reg, uint32_t frac,
                                      IngenicPllFormat format)
{
    uint64_t m = extract32(reg, 20, 12);
    uint64_t n = extract32(reg, 14, 6);
    uint64_t div;

    if (!(reg & 1)) {
        return 0;
    }
    if (format == INGENIC_PLL_PLLREG) {
        div = ((n + 1) << extract32(reg, 11, 3)) * (extract32(reg, 7, 4) + 1);
        return INGENIC_EXTCLK_HZ * 2 * (m + 1) / div;
    }
    if (format == INGENIC_PLL_MNOD_OD1) {
        div = n * (extract32(reg, 11, 3) + 1) * (extract32(reg, 8, 3) + 1);
    } else {
        div = n * extract32(reg, 11, 3) * extract32(reg, 8, 3);
    }
    if (!div) {
        return 0;
    }
    if (format == INGENIC_PLL_MNOD_FRAC) {
        return (INGENIC_EXTCLK_HZ * ((m << 24) + extract32(frac, 0, 24)) /
                div) >> 24;
    }
    return INGENIC_EXTCLK_HZ * m / div;
}

/* SCLK_A: EXTCLK or APLL as CPCCR selects, 0 while stopped */
static inline uint64_t ingenic_sclka_hz(uint32_t cpccr, uint64_t apll_hz)
{
    switch (extract32(cpccr, 30, 2)) {
    case 1:
        return INGENIC_EXTCLK_HZ;
    case 2:
        return apll_hz;
    }
    return 0;
}

/*
 * CPCCR feeds the CPU from SCLK_A or MPLL, divided by CDIV + 1. A stopped
 * source, as in a CPCCR that firmware has yet to set, leaves the core on
 * EXTCLK as it comes out of reset.
 */
static inline uint64_t ingenic_cpu_hz(uint32_t cpccr, uint64_t apll_hz,
                                      uint64_t mpll_hz)
{
    uint64_t hz = 0;

    switch (extract32(cpccr, 28, 2)) {
    case 1:
        hz = ingenic_sclka_hz(cpccr, apll_hz);
        break;
    case 2:
        hz = mpll_hz;
        break;
    }
    return hz ? hz / (extract32(cpccr, 0, 4) + 1) : INGENIC_EXTCLK_HZ;
}

#endif /* HW_MISC_INGENIC_CPM_CLK_H */
