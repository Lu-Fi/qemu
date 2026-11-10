/*
 * Ingenic eFuse: a chip's ID, the words computed from it, and a whole
 * array from a dump
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_EFUSE_ID_H
#define HW_MISC_INGENIC_EFUSE_ID_H

/*
 * The chip ID's signature: the first MD5 state word from the IV a108c309
 * b4f84b07 c3073cf8 d2162de9 over eFuse words 0-2 (12 bytes, MD5-padded).
 * A T30Z's word 13 and a T40's or T41's word 19, which their libimp
 * (and the T30Z's fast-boot SPL) check.
 */
uint32_t ingenic_efuse_chip_sign(const uint32_t id[3]);

/*
 * The MXU licence (word 15 of a T20, T21, T30 or T31): the first four MD5
 * state words XORed, from the IV a5e95a16 b4f84b07 c3073cf8 d2162de9 over
 * four copies of words 0-2 and (~h << 16 | h), h the low half of word 4,
 * zero-padded to 512 bytes; it reproduces a T30X's word 15.
 */
uint32_t ingenic_efuse_mxu_licence(const uint32_t id[3], uint16_t h);

/* "efuse-chipid": 24 hex digits, words 0 to 2 */
bool ingenic_efuse_parse_id(const char *hex, uint32_t id[3], Error **errp);

/*
 * "efuse-file": a chip's array as read out, little-endian words from word
 * 0; up to @max of them, their number in @n
 */
bool ingenic_efuse_load(const char *path, uint32_t *words, unsigned max,
                        unsigned *n, Error **errp);

#endif /* HW_MISC_INGENIC_EFUSE_ID_H */
