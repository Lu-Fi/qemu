/*
 * Ingenic HASH engine: MD5, SHA-1, SHA-224, SHA-256, SHA-384 and SHA-512
 *
 * A block engine: software pads the message, the engine runs the
 * compression function on each block, fed through HSDI or fetched by its
 * own DMA (HSSA, HSTC in 64-byte units), and HSDO reads the state back.
 *
 * As measured on the T31 and T41:
 *  - HSCR resets to 0x30000000 (PS 3); INIT and DMAS read 0. HSINTM has 2
 *    bits, HSSA and HSTC 32, and a DMA leaves both as written.
 *  - Message and state words are the algorithm's own: little-endian for
 *    MD5, big-endian for SHA. DIRVS byte-swaps each word written (or
 *    fetched), DORVS each word read from HSDO.
 *  - HSDO is a 16-word file holding the state after the last block, in
 *    order and zero past it: SHA-224 shows H7 as an 8th word, SHA-384/512
 *    give each word high half first. Reads walk it and wrap; a finished
 *    block starts them over. INIT neither clears it nor moves the index.
 *  - A finished block sets HSSR.ORD, the end of a DMA MRD as well; both
 *    clear by writing 1.
 *  - A DMA of half a SHA-384/512 block still hashes a whole one (T41),
 *    without MRD.
 * Not modelled: on the T31 a DMA after CPU-fed blocks does not always set
 * MRD, and half a SHA-512 block then wedges the engine.
 * The T33 puts HSBNUM at 0x14, moving HSDI and HSDO up a word, and counts
 * HSTC in words ("t33", from its vendor driver, unchecked).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-hash.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define HSCR        0x00
#define HSSR        0x04
#define HSINTM      0x08
#define HSSA        0x0c
#define HSTC        0x10
#define HSBNUM_T33  0x14

#define HSCR_DORVS  BIT(8)
#define HSCR_DIRVS  BIT(7)
#define HSCR_DMAS   BIT(6)
#define HSCR_DMAE   BIT(5)
#define HSCR_INIT   BIT(4)
#define HSCR_SEL(v) extract32(v, 1, 3)
#define HSCR_EN     BIT(0)
#define HSCR_MASK   0xf00001af
#define HSCR_RESET  0x30000000

#define HSSR_MRD    BIT(1)
#define HSSR_ORD    BIT(0)

enum { ALG_MD5, ALG_SHA1, ALG_SHA224, ALG_SHA256, ALG_SHA384, ALG_SHA512 };

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
static const uint32_t sha256_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};
static const uint64_t sha512_k[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL,
    0xb5c0fbcfec4d3b2fULL, 0xe9b5dba58189dbbcULL,
    0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
    0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL,
    0xd807aa98a3030242ULL, 0x12835b0145706fbeULL,
    0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL,
    0x9bdc06a725c71235ULL, 0xc19bf174cf692694ULL,
    0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
    0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL,
    0x2de92c6f592b0275ULL, 0x4a7484aa6ea6e483ULL,
    0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL,
    0xb00327c898fb213fULL, 0xbf597fc7beef0ee4ULL,
    0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
    0x06ca6351e003826fULL, 0x142929670a0e6e70ULL,
    0x27b70a8546d22ffcULL, 0x2e1b21385c26c926ULL,
    0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL,
    0x81c2c92e47edaee6ULL, 0x92722c851482353bULL,
    0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
    0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL,
    0xd192e819d6ef5218ULL, 0xd69906245565a910ULL,
    0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL,
    0x2748774cdf8eeb99ULL, 0x34b0bcb5e19b48a8ULL,
    0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
    0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL,
    0x748f82ee5defb2fcULL, 0x78a5636f43172f60ULL,
    0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL,
    0xbef9a3f7b2c67915ULL, 0xc67178f2e372532bULL,
    0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
    0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL,
    0x06f067aa72176fbaULL, 0x0a637dc5a2c898a6ULL,
    0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL,
    0x3c9ebe0a15c9bebcULL, 0x431d67c49c100d4cULL,
    0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
    0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL,
};

static const uint8_t md5_r[16] = {
    7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21,
};

static const uint32_t iv32[3][8] = {
    { 0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0 },
    { 0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939,
      0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4 },
    { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
      0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 },
};

static const uint64_t iv64[2][8] = {
    { 0xcbbb9d5dc1059ed8ULL, 0x629a292a367cd507ULL, 0x9159015a3070dd17ULL,
      0x152fecd8f70e5939ULL, 0x67332667ffc00b31ULL, 0x8eb44a8768581511ULL,
      0xdb0c2e0d64f98fa7ULL, 0x47b5481dbefa4fa4ULL },
    { 0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
      0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
      0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL },
};

static void md5_block(uint32_t *h, const uint32_t *m)
{
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3];

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
        b += rol32(a + f + md5_k[i] + m[g], md5_r[(i / 16) * 4 + i % 4]);
        a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
}

static void sha1_block(uint32_t *h, const uint32_t *m)
{
    uint32_t w[80], a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];

    for (int i = 0; i < 80; i++) {
        uint32_t f, k, t;

        w[i] = i < 16 ? m[i]
                      : rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        if (i < 20) {
            f = (b & c) | (~b & d);
            k = 0x5a827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ed9eba1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8f1bbcdc;
        } else {
            f = b ^ c ^ d;
            k = 0xca62c1d6;
        }
        t = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
}

static void sha256_block(uint32_t *h, const uint32_t *m)
{
    uint32_t w[64], v[8];

    for (int i = 0; i < 64; i++) {
        if (i < 16) {
            w[i] = m[i];
        } else {
            uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^
                          (w[i - 15] >> 3);
            uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^
                          (w[i - 2] >> 10);

            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
    }
    memcpy(v, h, sizeof(v));
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = v[7] + (ror32(v[4], 6) ^ ror32(v[4], 11) ^
                              ror32(v[4], 25)) +
                      ((v[4] & v[5]) ^ (~v[4] & v[6])) + sha256_k[i] + w[i];
        uint32_t t2 = (ror32(v[0], 2) ^ ror32(v[0], 13) ^ ror32(v[0], 22)) +
                      ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));

        memmove(v + 1, v, 7 * sizeof(v[0]));
        v[4] += t1;
        v[0] = t1 + t2;
    }
    for (int i = 0; i < 8; i++) {
        h[i] += v[i];
    }
}

static void sha512_block(uint64_t *h, const uint64_t *m)
{
    uint64_t w[80], v[8];

    for (int i = 0; i < 80; i++) {
        if (i < 16) {
            w[i] = m[i];
        } else {
            uint64_t s0 = ror64(w[i - 15], 1) ^ ror64(w[i - 15], 8) ^
                          (w[i - 15] >> 7);
            uint64_t s1 = ror64(w[i - 2], 19) ^ ror64(w[i - 2], 61) ^
                          (w[i - 2] >> 6);

            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
    }
    memcpy(v, h, sizeof(v));
    for (int i = 0; i < 80; i++) {
        uint64_t t1 = v[7] + (ror64(v[4], 14) ^ ror64(v[4], 18) ^
                              ror64(v[4], 41)) +
                      ((v[4] & v[5]) ^ (~v[4] & v[6])) + sha512_k[i] + w[i];
        uint64_t t2 = (ror64(v[0], 28) ^ ror64(v[0], 34) ^ ror64(v[0], 39)) +
                      ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));

        memmove(v + 1, v, 7 * sizeof(v[0]));
        v[4] += t1;
        v[0] = t1 + t2;
    }
    for (int i = 0; i < 8; i++) {
        h[i] += v[i];
    }
}

static unsigned hash_alg(IngenicHashState *s)
{
    return HSCR_SEL(s->hscr);
}

static unsigned hash_block_words(IngenicHashState *s)
{
    return hash_alg(s) >= ALG_SHA384 ? 32 : 16;
}

static void hash_update_irq(IngenicHashState *s)
{
    qemu_set_irq(s->irq, !!(s->hssr & s->hsintm));
}

static void hash_start(IngenicHashState *s)
{
    static const uint8_t iv[] = { 0, 0, 1, 2, 0, 1 };
    unsigned alg = hash_alg(s);

    if (alg <= ALG_SHA256) {
        memcpy(s->h32, iv32[iv[alg]], sizeof(s->h32));
    } else if (alg <= ALG_SHA512) {
        memcpy(s->h64, iv64[iv[alg]], sizeof(s->h64));
    }
    s->blk_n = 0;
}

/* A block is done: HSDO shows the new state, read from the top */
static void hash_publish(IngenicHashState *s)
{
    static const uint8_t words[] = { 4, 5, 8, 8 };
    unsigned alg = hash_alg(s);

    memset(s->out, 0, sizeof(s->out));
    if (alg >= ALG_SHA384) {
        for (int i = 0; i < 8; i++) {
            s->out[2 * i] = s->h64[i] >> 32;
            s->out[2 * i + 1] = s->h64[i];
        }
    } else {
        memcpy(s->out, s->h32, words[alg] * sizeof(uint32_t));
    }
    s->out_idx = 0;
    s->hssr |= HSSR_ORD;
}

static void hash_word(IngenicHashState *s, uint32_t v)
{
    unsigned alg = hash_alg(s);

    if (alg > ALG_SHA512) {
        return;
    }
    s->blk[s->blk_n++] = s->hscr & HSCR_DIRVS ? bswap32(v) : v;
    if (s->blk_n < hash_block_words(s)) {
        return;
    }
    s->blk_n = 0;
    switch (alg) {
    case ALG_MD5:
        md5_block(s->h32, s->blk);
        break;
    case ALG_SHA1:
        sha1_block(s->h32, s->blk);
        break;
    case ALG_SHA224:
    case ALG_SHA256:
        sha256_block(s->h32, s->blk);
        break;
    default: {
        uint64_t m[16];

        for (int i = 0; i < 16; i++) {
            m[i] = (uint64_t)s->blk[2 * i] << 32 | s->blk[2 * i + 1];
        }
        sha512_block(s->h64, m);
    }
    }
    hash_publish(s);
}

static void hash_dma(IngenicHashState *s)
{
    uint64_t len = (uint64_t)s->hstc * (s->t33 ? 4 : 64);
    uint32_t bsize = hash_block_words(s) * 4;
    uint64_t total = ROUND_UP(len, bsize);
    uint32_t buf[32];

    for (uint64_t done = 0; done < total; done += bsize) {
        hwaddr addr = (hwaddr)s->hssa + done;

        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, buf,
                               bsize) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hash: DMA read error at "
                          "0x%" HWADDR_PRIx "\n", addr);
            return;
        }
        for (int i = 0; i < bsize / 4; i++) {
            hash_word(s, le32_to_cpu(buf[i]));
        }
    }
    if (total == len) {
        s->hssr |= HSSR_MRD;
    }
}

static uint64_t ingenic_hash_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicHashState *s = INGENIC_HASH(opaque);
    hwaddr hsdo = s->t33 ? 0x1c : 0x18;

    if (offset == hsdo) {
        uint32_t v = s->out[s->out_idx];

        s->out_idx = (s->out_idx + 1) % ARRAY_SIZE(s->out);
        return s->hscr & HSCR_DORVS ? bswap32(v) : v;
    }
    switch (offset) {
    case HSCR:
        return s->hscr;
    case HSSR:
        return s->hssr;
    case HSINTM:
        return s->hsintm;
    case HSSA:
        return s->hssa;
    case HSTC:
        return s->hstc;
    case HSBNUM_T33:
        return s->t33 ? s->hsbnum : 0;
    }
    return 0;
}

static void ingenic_hash_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    IngenicHashState *s = INGENIC_HASH(opaque);
    hwaddr hsdi = s->t33 ? 0x18 : 0x14;
    uint32_t v = value;

    if (offset == hsdi) {
        if (s->hscr & HSCR_EN) {
            hash_word(s, v);
        }
        hash_update_irq(s);
        return;
    }
    switch (offset) {
    case HSCR:
        s->hscr = v & HSCR_MASK;
        if (v & HSCR_INIT) {
            hash_start(s);
        }
        if ((v & HSCR_DMAS) && (s->hscr & (HSCR_DMAE | HSCR_EN)) ==
                               (HSCR_DMAE | HSCR_EN)) {
            hash_dma(s);
        }
        break;
    case HSSR:
        s->hssr &= ~(v & (HSSR_MRD | HSSR_ORD));
        break;
    case HSINTM:
        s->hsintm = v & (HSSR_MRD | HSSR_ORD);
        break;
    case HSSA:
        s->hssa = v;
        break;
    case HSTC:
        s->hstc = v;
        break;
    case HSBNUM_T33:
        if (s->t33) {
            s->hsbnum = v;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hash: write of 0x%x to 0x%"
                      HWADDR_PRIx "\n", v, offset);
        return;
    }
    hash_update_irq(s);
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

    s->hscr = HSCR_RESET;
    s->hssr = s->hsintm = s->hssa = s->hstc = s->hsbnum = 0;
    memset(s->h32, 0, sizeof(s->h32));
    memset(s->h64, 0, sizeof(s->h64));
    memset(s->blk, 0, sizeof(s->blk));
    memset(s->out, 0, sizeof(s->out));
    s->blk_n = s->out_idx = 0;
}

static void ingenic_hash_reset_exit(Object *obj, ResetType type)
{
    hash_update_irq(INGENIC_HASH(obj));
}

static void ingenic_hash_init(Object *obj)
{
    IngenicHashState *s = INGENIC_HASH(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_hash_ops, s,
                          TYPE_INGENIC_HASH, INGENIC_HASH_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static int ingenic_hash_post_load(void *opaque, int version_id)
{
    IngenicHashState *s = opaque;

    if (s->blk_n >= ARRAY_SIZE(s->blk) || s->out_idx >= ARRAY_SIZE(s->out)) {
        return -EINVAL;
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_hash = {
    .name = TYPE_INGENIC_HASH,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = ingenic_hash_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(hscr, IngenicHashState),
        VMSTATE_UINT32(hssr, IngenicHashState),
        VMSTATE_UINT32(hsintm, IngenicHashState),
        VMSTATE_UINT32(hssa, IngenicHashState),
        VMSTATE_UINT32(hstc, IngenicHashState),
        VMSTATE_UINT32(hsbnum, IngenicHashState),
        VMSTATE_UINT32_ARRAY(h32, IngenicHashState, 8),
        VMSTATE_UINT64_ARRAY(h64, IngenicHashState, 8),
        VMSTATE_UINT32_ARRAY(blk, IngenicHashState, 32),
        VMSTATE_UINT32(blk_n, IngenicHashState),
        VMSTATE_UINT32_ARRAY(out, IngenicHashState, 16),
        VMSTATE_UINT32(out_idx, IngenicHashState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_hash_props[] = {
    DEFINE_PROP_BOOL("t33", IngenicHashState, t33, false),
};

static void ingenic_hash_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_hash;
    device_class_set_props(dc, ingenic_hash_props);
    rc->phases.hold = ingenic_hash_reset_hold;
    rc->phases.exit = ingenic_hash_reset_exit;
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
