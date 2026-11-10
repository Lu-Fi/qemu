/*
 * Ingenic eFuse: a chip's ID, the words computed from it, and a whole
 * array from a dump
 *
 * The computed words are MD5 (RFC 1321) from an initial state of Ingenic's
 * own instead of MD5's.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/misc/ingenic-efuse-id.h"

static const uint32_t md5_k[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
    0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
    0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
    0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
    0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
    0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
    0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
    0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
    0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
};

static const uint8_t md5_s[16] = {
    7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21,
};

static void md5_block(uint32_t st[4], const uint8_t *blk)
{
    uint32_t a = st[0], b = st[1], c = st[2], d = st[3];

    for (int i = 0; i < 64; i++) {
        uint32_t f, t;
        int g;

        if (i < 16) {
            f = (b & c) | (~b & d);
            g = i;
        } else if (i < 32) {
            f = (d & b) | (~d & c);
            g = (5 * i + 1) % 16;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3 * i + 5) % 16;
        } else {
            f = c ^ (b | ~d);
            g = (7 * i) % 16;
        }
        t = d;
        d = c;
        c = b;
        b += rol32(a + f + md5_k[i] + ldl_le_p(blk + 4 * g),
                   md5_s[(i / 16) * 4 + i % 4]);
        a = t;
    }
    st[0] += a;
    st[1] += b;
    st[2] += c;
    st[3] += d;
}

/* MD5 of @msg, padded as MD5 pads it, from the state @iv */
static void md5_from(const uint32_t iv[4], const uint8_t *msg, size_t len,
                     uint32_t st[4])
{
    size_t blocks = (len + 8) / 64 + 1;
    g_autofree uint8_t *buf = g_malloc0(blocks * 64);

    memcpy(buf, msg, len);
    buf[len] = 0x80;
    stq_le_p(buf + blocks * 64 - 8, (uint64_t)len * 8);
    memcpy(st, iv, 4 * sizeof(uint32_t));
    for (size_t i = 0; i < blocks; i++) {
        md5_block(st, buf + 64 * i);
    }
}

uint32_t ingenic_efuse_chip_sign(const uint32_t id[3])
{
    static const uint32_t iv[4] = {
        0xa108c309, 0xb4f84b07, 0xc3073cf8, 0xd2162de9,
    };
    uint8_t msg[12];
    uint32_t st[4];

    for (int i = 0; i < 3; i++) {
        stl_le_p(msg + 4 * i, id[i]);
    }
    md5_from(iv, msg, sizeof(msg), st);
    return st[0];
}

uint32_t ingenic_efuse_mxu_licence(const uint32_t id[3], uint16_t h)
{
    static const uint32_t iv[4] = {
        0xa5e95a16, 0xb4f84b07, 0xc3073cf8, 0xd2162de9,
    };
    uint8_t msg[512] = { 0 };
    uint32_t st[4];

    for (int k = 0; k < 4; k++) {
        for (int i = 0; i < 3; i++) {
            stl_le_p(msg + 16 * k + 4 * i, id[i]);
        }
        stl_le_p(msg + 16 * k + 12, (uint32_t)~h << 16 | h);
    }
    md5_from(iv, msg, sizeof(msg), st);
    return st[0] ^ st[1] ^ st[2] ^ st[3];
}

bool ingenic_efuse_parse_id(const char *hex, uint32_t id[3], Error **errp)
{
    if (strlen(hex) != 24) {
        error_setg(errp, "efuse-chipid: 24 hex digits, words 0 to 2");
        return false;
    }
    for (int i = 0; i < 3; i++) {
        uint32_t w = 0;

        for (int j = 0; j < 8; j++) {
            int d = g_ascii_xdigit_value(hex[8 * i + j]);

            if (d < 0) {
                error_setg(errp, "efuse-chipid: '%c' is not a hex digit",
                           hex[8 * i + j]);
                return false;
            }
            w = w << 4 | d;
        }
        id[i] = w;
    }
    return true;
}

bool ingenic_efuse_load(const char *path, uint32_t *words, unsigned max,
                        unsigned *n, Error **errp)
{
    g_autofree char *buf = NULL;
    g_autoptr(GError) gerr = NULL;
    gsize len;

    if (!g_file_get_contents(path, &buf, &len, &gerr)) {
        error_setg(errp, "efuse-file: %s", gerr->message);
        return false;
    }
    if (len % 4 || len / 4 > max) {
        error_setg(errp, "efuse-file: %s holds %zu bytes, not up to %u"
                   " whole words", path, (size_t)len, max);
        return false;
    }
    for (gsize i = 0; i < len / 4; i++) {
        words[i] = ldl_le_p(buf + 4 * i);
    }
    *n = len / 4;
    return true;
}
