/*
 * QTest testcase for the Ingenic AES engine
 *
 * Known-answer tests on every part that has the block: FIPS-197 appendix C
 * and NIST SP 800-38A (F.1 ECB, F.2 CBC) for 128, 192 and 256-bit keys,
 * encrypting and decrypting, driven two ways as the vendor code does.
 *  - DMA, as the SDK's jz-aes driver: ASCR clear, then mode and key
 *    length, IV and key words, key expansion, ASSA/ASDA/count, DMAS.
 *  - By register, as the U-Boot driver: four ASDI words, AESS, four ASDO
 *    words, a block at a time.
 * Register words are the cipher's bytes most significant first. Memory for
 * the DMA is those words little-endian (each four bytes reversed) on every
 * part but the T32 and T33, whose drivers set ASCR.EDIN (and EDOUT on the
 * T33) to take it in byte order. The T33 has its own register layout.
 * The expected values are the standards', not the model's.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define AES_BASE        0x13430000
#define SRC             0x00100000
#define DST             0x00200000

#define ASCR            0x00
#define ASSR            0x04
#define ASINTM          0x08
#define ASSA            0x0c

#define ASCR_EN         BIT(0)
#define ASCR_INIT_IV    BIT(1)
#define ASCR_KEYS       BIT(2)
#define ASCR_AESS       BIT(3)
#define ASCR_DECE       BIT(4)
#define ASCR_CBC        BIT(5)
#define ASCR_KEYL(n)    ((n) << 6)
#define ASCR_DMAE       BIT(8)
#define ASCR_DMAS       BIT(9)
#define ASCR_CLR        BIT(10)
#define ASCR_EDIN       BIT(12)
#define ASCR_EDOUT      BIT(13)

typedef struct {
    const char *part;
    const char *machine;
    bool t33;                   /* the T33's register layout */
    bool bytes;                 /* DMA memory in byte order with EDIN/EDOUT */
    uint32_t ed;                /* the EDIN/EDOUT bits that read back */
} AesPart;

static const AesPart parts[] = {
    { "t10", "ingenic-t10" },
    { "t20", "ingenic-t20" },
    { "t21", "ingenic-t21" },
    { "t23", "ingenic-t23" },
    { "t30", "ingenic-t30" },
    { "t31", "ingenic-t31" },
    { "t32", "ingenic-t32", .bytes = true, .ed = ASCR_EDIN },
    { "t33", "ingenic-t33", .t33 = true, .bytes = true,
      .ed = ASCR_EDIN | ASCR_EDOUT },
    { "t40", "ingenic-t40" },
    { "t41", "ingenic-t41" },
    { "a1", "ingenic-a1" },
};

typedef struct {
    const char *name;
    unsigned bits;
    bool cbc;
    const char *key;
    const char *iv;
    const char *pt;
    const char *ct;
} AesVec;

#define SP38A_PT \
    "6bc1bee22e409f96e93d7e117393172a" "ae2d8a571e03ac9c9eb76fac45af8e51" \
    "30c81c46a35ce411e5fbc1191a0a52ef" "f69f2445df4f9b17ad2b417be66c3710"
#define SP38A_IV "000102030405060708090a0b0c0d0e0f"
#define SP38A_K128 "2b7e151628aed2a6abf7158809cf4f3c"
#define SP38A_K192 "8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b"
#define SP38A_K256 \
    "603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4"
#define FIPS_PT "00112233445566778899aabbccddeeff"
#define FIPS_KEY \
    "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f"

static const AesVec vecs[] = {
    { "fips197-c1-ecb-128", 128, false, FIPS_KEY, NULL, FIPS_PT,
      "69c4e0d86a7b0430d8cdb78070b4c55a" },
    { "fips197-c2-ecb-192", 192, false, FIPS_KEY, NULL, FIPS_PT,
      "dda97ca4864cdfe06eaf70a0ec0d7191" },
    { "fips197-c3-ecb-256", 256, false, FIPS_KEY, NULL, FIPS_PT,
      "8ea2b7ca516745bfeafc49904b496089" },
    { "sp800-38a-f1.1-ecb-128", 128, false, SP38A_K128, NULL, SP38A_PT,
      "3ad77bb40d7a3660a89ecaf32466ef97" "f5d3d58503b9699de785895a96fdbaaf"
      "43b1cd7f598ece23881b00e3ed030688" "7b0c785e27e8ad3f8223207104725dd4" },
    { "sp800-38a-f1.3-ecb-192", 192, false, SP38A_K192, NULL, SP38A_PT,
      "bd334f1d6e45f25ff712a214571fa5cc" "974104846d0ad3ad7734ecb3ecee4eef"
      "ef7afd2270e2e60adce0ba2face6444e" "9a4b41ba738d6c72fb16691603c18e0e" },
    { "sp800-38a-f1.5-ecb-256", 256, false, SP38A_K256, NULL, SP38A_PT,
      "f3eed1bdb5d2a03c064b5a7e3db181f8" "591ccb10d410ed26dc5ba74a31362870"
      "b6ed21b99ca6f4f9f153e7b1beafed1d" "23304b7a39f9f3ff067d8d8f9e24ecc7" },
    { "sp800-38a-f2.1-cbc-128", 128, true, SP38A_K128, SP38A_IV, SP38A_PT,
      "7649abac8119b246cee98e9b12e9197d" "5086cb9b507219ee95db113a917678b2"
      "73bed6b8e3c1743b7116e69e22229516" "3ff1caa1681fac09120eca307586e1a7" },
    { "sp800-38a-f2.3-cbc-192", 192, true, SP38A_K192, SP38A_IV, SP38A_PT,
      "4f021db243bc633d7178183a9fa071e8" "b4d9ada9ad7dedf4e5e738763f69145a"
      "571b242012fb7ae07fa9baac3df102e0" "08b0e27988598881d920a9e64f5615cd" },
    { "sp800-38a-f2.5-cbc-256", 256, true, SP38A_K256, SP38A_IV, SP38A_PT,
      "f58c4c04d6e5f1ba779eabfb5f7bfbd6" "9cfc4e967edb808d679f777bc6702c7d"
      "39f23369a9d9bacfa530e26304231461" "b2eb05e2c39be9fcda6c19078c6a9d1b" },
};

typedef struct {
    QTestState *qts;
    const AesPart *p;
    unsigned asda, astc, asdi, asdo, asky, asiv;
    uint32_t keyd, aesd, dmad;
} Aes;

static size_t unhex(const char *s, uint8_t *out)
{
    size_t n = strlen(s) / 2;

    for (size_t i = 0; i < n; i++) {
        unsigned v;

        g_assert_cmpint(sscanf(s + 2 * i, "%2x", &v), ==, 1);
        out[i] = v;
    }
    return n;
}

static char *hex(const uint8_t *b, size_t n)
{
    GString *s = g_string_new(NULL);

    for (size_t i = 0; i < n; i++) {
        g_string_append_printf(s, "%02x", b[i]);
    }
    return g_string_free(s, false);
}

static void aes_reg_init(Aes *a)
{
    if (a->p->t33) {
        a->asda = 0x14;
        a->asdi = 0x20;
        a->asdo = 0x24;
        a->asky = 0x28;
        a->asiv = 0x2c;
        a->aesd = BIT(0);
        a->dmad = BIT(1);
        a->keyd = 0;
    } else {
        a->asda = 0x10;
        a->astc = 0x14;
        a->asdi = 0x18;
        a->asdo = 0x1c;
        a->asky = 0x20;
        a->asiv = 0x24;
        a->keyd = BIT(0);
        a->aesd = BIT(1);
        a->dmad = BIT(2);
    }
}

static uint32_t rd(Aes *a, unsigned reg)
{
    return qtest_readl(a->qts, AES_BASE + reg);
}

static void wr(Aes *a, unsigned reg, uint32_t v)
{
    qtest_writel(a->qts, AES_BASE + reg, v);
}

static void ascr_or(Aes *a, uint32_t bits)
{
    wr(a, ASCR, rd(a, ASCR) | bits);
}

/* Write back the status bits seen set: the drivers' way to clear them */
static void ssr_clear(Aes *a, uint32_t bits)
{
    wr(a, ASSR, rd(a, ASSR) & bits);
}

/* The labelled hex of an answer, so a failure names what it was */
static void expect(const AesPart *p, const AesVec *v, const char *what,
                   const uint8_t *got, const uint8_t *want, size_t n)
{
    g_autofree char *g = hex(got, n);
    g_autofree char *w = hex(want, n);
    g_autofree char *gl = g_strdup_printf("aes %s %s %s: %s", p->part, v->name,
                                          what, g);
    g_autofree char *wl = g_strdup_printf("aes %s %s %s: %s", p->part, v->name,
                                          what, w);

    g_assert_cmpstr(gl, ==, wl);
}

static void load_key_iv(Aes *a, const AesVec *v)
{
    uint8_t key[32], iv[16];

    if (v->cbc) {
        unhex(v->iv, iv);
        for (int i = 0; i < 4; i++) {
            wr(a, a->asiv, ldl_be_p(iv + 4 * i));
        }
        ascr_or(a, ASCR_INIT_IV);
    }
    unhex(v->key, key);
    for (unsigned i = 0; i < v->bits / 32; i++) {
        wr(a, a->asky, ldl_be_p(key + 4 * i));
    }
    ascr_or(a, ASCR_KEYS);
    if (!a->p->t33) {
        g_assert_cmphex(rd(a, ASSR) & a->keyd, ==, a->keyd);
        ssr_clear(a, a->keyd);
    }
}

static uint32_t keyl(const AesVec *v)
{
    return ASCR_KEYL((v->bits - 128) / 64);
}

/* The SDK driver: DMA of the whole message in 'chunks' starts */
static void dma_run(Aes *a, const AesVec *v, bool dec, unsigned chunks,
                    const uint8_t *in, uint8_t *out, size_t n)
{
    uint32_t mode = ASCR_EN | ASCR_DMAE | keyl(v) | (v->cbc ? ASCR_CBC : 0) |
                    (dec ? ASCR_DECE : 0);
    size_t per = n / chunks;
    g_autofree uint8_t *buf = g_malloc(n);

    ssr_clear(a, ~0u);
    wr(a, ASINTM, a->dmad | a->aesd | a->keyd);
    wr(a, ASCR, ASCR_CLR | ASCR_EN);
    wr(a, ASCR, mode);
    ascr_or(a, ASCR_EDIN | ASCR_EDOUT);
    g_assert_cmphex(rd(a, ASCR) & (ASCR_EDIN | ASCR_EDOUT), ==, a->p->ed);
    load_key_iv(a, v);

    memcpy(buf, in, n);
    if (!a->p->bytes) {
        for (size_t i = 0; i < n; i += 4) {
            stl_le_p(buf + i, ldl_be_p(in + i));
        }
    }
    qtest_memwrite(a->qts, SRC, buf, n);
    wr(a, ASSA, SRC);
    wr(a, a->asda, DST);
    for (unsigned c = 0; c < chunks; c++) {
        if (a->p->t33) {
            wr(a, 0x10, per / 4);
            wr(a, 0x18, per / 4);
            wr(a, 0x1c, per / 16);
        } else {
            wr(a, a->astc, per / 16);
        }
        wr(a, ASSA, SRC + c * per);
        wr(a, a->asda, DST + c * per);
        ascr_or(a, ASCR_DMAS);
        g_assert_cmphex(rd(a, ASSR) & a->dmad, ==, a->dmad);
        ssr_clear(a, a->dmad | a->aesd);
        g_assert_cmphex(rd(a, ASSR), ==, 0);
    }
    qtest_memread(a->qts, DST, buf, n);
    if (!a->p->bytes) {
        for (size_t i = 0; i < n; i += 4) {
            stl_be_p(out + i, ldl_le_p(buf + i));
        }
    } else {
        memcpy(out, buf, n);
    }
}

/* The U-Boot driver: a block through ASDI and ASDO at a time */
static void pio_run(Aes *a, const AesVec *v, bool dec,
                    const uint8_t *in, uint8_t *out, size_t n)
{
    uint32_t mode = ASCR_EN | ASCR_CLR | keyl(v) | (3u << 28);

    ssr_clear(a, ~0u);
    wr(a, ASINTM, a->p->t33 ? 1 : 3);
    wr(a, ASCR, mode);
    if (v->cbc) {
        ascr_or(a, ASCR_CBC);
    }
    if (dec) {
        ascr_or(a, ASCR_DECE);
    }
    load_key_iv(a, v);

    for (size_t b = 0; b < n; b += 16) {
        for (int i = 0; i < 4; i++) {
            wr(a, a->asdi, ldl_be_p(in + b + 4 * i));
        }
        ascr_or(a, ASCR_AESS);
        g_assert_cmphex(rd(a, ASSR) & a->aesd, ==, a->aesd);
        ssr_clear(a, a->aesd);
        g_assert_cmphex(rd(a, ASSR) & a->aesd, ==, 0);
        for (int i = 0; i < 4; i++) {
            stl_be_p(out + b + 4 * i, rd(a, a->asdo));
        }
    }
}

static void test_part(const void *data)
{
    const AesPart *p = data;
    Aes a = { .p = p };

    a.qts = qtest_initf("-machine %s", p->machine);
    aes_reg_init(&a);
    g_assert_cmphex(rd(&a, ASCR), ==, 0x30000000);

    for (int i = 0; i < ARRAY_SIZE(vecs); i++) {
        const AesVec *v = &vecs[i];
        uint8_t pt[64], ct[64], out[64];
        size_t n = unhex(v->pt, pt);

        g_assert_cmpuint(unhex(v->ct, ct), ==, n);
        for (int dec = 0; dec < 2; dec++) {
            const uint8_t *in = dec ? ct : pt;
            const uint8_t *want = dec ? pt : ct;
            const char *dir = dec ? "decrypt" : "encrypt";
            g_autofree char *w = NULL;

            memset(out, 0xa5, sizeof(out));
            dma_run(&a, v, dec, 1, in, out, n);
            w = g_strdup_printf("%s dma", dir);
            expect(p, v, w, out, want, n);

            if (n > 16) {
                memset(out, 0xa5, sizeof(out));
                dma_run(&a, v, dec, 2, in, out, n);
                g_free(w);
                w = g_strdup_printf("%s dma in two starts", dir);
                expect(p, v, w, out, want, n);
            }

            memset(out, 0xa5, sizeof(out));
            pio_run(&a, v, dec, in, out, n);
            g_free(w);
            w = g_strdup_printf("%s registers", dir);
            expect(p, v, w, out, want, n);
        }
    }
    qtest_quit(a.qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-aes/%s",
                                                parts[i].part);

        qtest_add_data_func(path, &parts[i], test_part);
    }
    return g_test_run();
}
