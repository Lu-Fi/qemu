/*
 * QTest testcase for the Ingenic DES engine
 *
 * Known-answer tests for single DES and triple DES (EDE), ECB and CBC,
 * both directions, on every part that has the block (not the T33). The
 * vectors are FIPS 81 appendix, FIPS 46-3 / SP 800-20 single blocks, the
 * SP 800-67 TDEA example, and the jz-des vendor driver's own TDES vectors
 * (checked here against OpenSSL). The ones marked "openssl" are not in a
 * standard; they were computed with OpenSSL's des-ede3 and des-ede3-cbc.
 *
 * Each is driven two ways as the vendor jz-des driver does: through the
 * registers (CR2 mode, key words, IV, CR1.EN, two DIN words, two DOUT
 * words per block), and by PDMA (a channel from memory to DIN on request
 * 0x2e, one from DOUT to memory on 0x2f, with CR2.DMAE). The driver sets
 * CR2.BE, so words are the data's bytes most significant first, and the
 * DMA moves those words as memory words.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define DES_BASE        0x10061000
#define SRC             0x00100000
#define DST             0x00200000

#define CR1             0x00
#define CR2             0x04
#define SR              0x08
#define KEY             0x10
#define IVL             0x28
#define IVR             0x2c
#define DIN             0x30
#define DOUT            0x34

#define CR2_DMAE        BIT(0)
#define CR2_TDES        BIT(1)
#define CR2_CBC         BIT(2)
#define CR2_DEC         BIT(3)
#define CR2_BE          BIT(4)

#define PDMA            0x13420000
#define PDMA_DMAC       (PDMA + 0x1000)
#define CH(n, r)        (PDMA + (n) * 0x20 + (r))
#define DSA             0x00
#define DTA             0x04
#define DTC             0x08
#define DRT             0x0c
#define DCS             0x10
#define DCM             0x14
#define DCS_GO          0x80000001
#define DCM_SAI         0x00800000
#define DCM_DAI         0x00400000
#define REQ_DIN         0x2e
#define REQ_DOUT        0x2f

typedef struct {
    const char *part;
    const char *machine;
} DesPart;

static const DesPart parts[] = {
    { "t10", "ingenic-t10" },
    { "t20", "ingenic-t20" },
    { "t21", "ingenic-t21" },
    { "t23", "ingenic-t23" },
    { "t30", "ingenic-t30" },
    { "t31", "ingenic-t31" },
    { "t32", "ingenic-t32" },
    { "t40", "ingenic-t40" },
    { "t41", "ingenic-t41" },
    { "a1", "ingenic-a1" },
};

typedef struct {
    const char *name;
    bool tdes;
    bool cbc;
    const char *key;            /* 8 bytes, or 24 for triple DES */
    const char *iv;
    const char *pt;
    const char *ct;
} DesVec;

#define NOW_IS "4e6f77206973207468652074696d6520666f7220616c6c20"
#define TDEA_KEY \
    "0123456789abcdef" "23456789abcdef01" "456789abcdef0123"
#define TDEA_PT "5468652071756663" "6b2062726f776e20" "666f78206a756d70"
#define VENDOR_KEY \
    "bede4bb50d7ba2b2" "319fb7c7c72f5f77" "5f72979f694f373c"

static const DesVec vecs[] = {
    /* FIPS 46-3 / classic: key 133457799BBCDFF1 */
    { "fips46-ecb", false, false, "133457799bbcdff1", NULL,
      "0123456789abcdef", "85e813540f0ab405" },
    /* SP 800-20 table 1 (variable plaintext) and table 2 (variable key) */
    { "sp800-20-vptext-ecb", false, false, "0101010101010101", NULL,
      "8000000000000000", "95f8a5e5dd31d900" },
    { "sp800-20-vkey-ecb", false, false, "8001010101010101", NULL,
      "0000000000000000", "95a8d72813daa94d" },
    /* FIPS 81 appendix: "Now is the time for all " */
    { "fips81-ecb", false, false, "0123456789abcdef", NULL, NOW_IS,
      "3fa40e8a984d4815" "6a271787ab8883f9" "893d51ec4b563b53" },
    { "fips81-cbc", false, true, "0123456789abcdef", "1234567890abcdef",
      NOW_IS,
      "e5c7cdde872bf27c" "43e934008c389c0f" "683788499a7c05f6" },
    /* SP 800-67 appendix B: "The qufck brown fox jump" */
    { "sp800-67-tdea-ecb", true, false, TDEA_KEY, NULL, TDEA_PT,
      "a826fd8ce53b855f" "cce21c8112256fe6" "68d5c05dd9b6b900" },
    /* openssl des-ede3-cbc, the same key and plaintext */
    { "sp800-67-tdea-cbc-openssl", true, true, TDEA_KEY, "1234567890abcdef",
      TDEA_PT,
      "38413d4ba2325cf1" "141f707471ac2ced" "57db530f0123b5ac" },
    /* K1 = K2 = K3 is single DES */
    { "tdea-3x-fips46-ecb", true, false,
      "133457799bbcdff1" "133457799bbcdff1" "133457799bbcdff1", NULL,
      "0123456789abcdef", "85e813540f0ab405" },
    /* jz-des's own TDES vectors (en_ecb_cpu_mode_tdes, en_cbc_cpu_mode_tdes) */
    { "jz-des-tdes-ecb-openssl", true, false, VENDOR_KEY, NULL,
      "c94cbdd7c75f89f8", "7f59ba1514836860" },
    { "jz-des-tdes-cbc-openssl", true, true, VENDOR_KEY, "054c45664aeb0008",
      "c94cbdd7c75f89f8", "8df4d80e129f752c" },
};

typedef struct {
    QTestState *qts;
    const DesPart *p;
} Des;

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

static uint32_t rd(Des *d, unsigned reg)
{
    return qtest_readl(d->qts, DES_BASE + reg);
}

static void wr(Des *d, unsigned reg, uint32_t v)
{
    qtest_writel(d->qts, DES_BASE + reg, v);
}

static void expect(const DesPart *p, const DesVec *v, const char *what,
                   const uint8_t *got, const uint8_t *want, size_t n)
{
    g_autofree char *g = hex(got, n);
    g_autofree char *w = hex(want, n);
    g_autofree char *gl = g_strdup_printf("des %s %s %s: %s", p->part, v->name,
                                          what, g);
    g_autofree char *wl = g_strdup_printf("des %s %s %s: %s", p->part, v->name,
                                          what, w);

    g_assert_cmpstr(gl, ==, wl);
}

/* The driver's order: CR2 mode, keys, IV, then enable */
static void setup(Des *d, const DesVec *v, bool dec, bool dma)
{
    uint8_t key[24];
    size_t kn = unhex(v->key, key);

    wr(d, CR1, 0);
    wr(d, CR2, 0);
    wr(d, CR2, CR2_BE | (v->tdes ? CR2_TDES : 0) | (v->cbc ? CR2_CBC : 0) |
               (dec ? CR2_DEC : 0) | (dma ? CR2_DMAE : 0));
    for (unsigned i = 0; i < 6; i++) {
        /* a single DES key is K1 alone; the rest are not used */
        wr(d, KEY + 4 * i, i < kn / 4 ? ldl_be_p(key + 4 * i) : 0);
    }
    if (v->cbc) {
        uint8_t iv[8];

        unhex(v->iv, iv);
        wr(d, IVL, ldl_be_p(iv));
        wr(d, IVR, ldl_be_p(iv + 4));
    }
    wr(d, CR1, 1);
}

static void pio_run(Des *d, const DesVec *v, bool dec,
                    const uint8_t *in, uint8_t *out, size_t n)
{
    setup(d, v, dec, false);
    for (size_t b = 0; b < n; b += 8) {
        wr(d, DIN, ldl_be_p(in + b));
        wr(d, DIN, ldl_be_p(in + b + 4));
        stl_be_p(out + b, rd(d, DOUT));
        stl_be_p(out + b + 4, rd(d, DOUT));
    }
    g_assert_cmphex(rd(d, SR) & 1, ==, 1);
    wr(d, CR1, 0);
}

static void ch_start(Des *d, int ch, uint32_t dsa, uint32_t dta, uint32_t dtc,
                     uint32_t drt, uint32_t dcm)
{
    qtest_writel(d->qts, CH(ch, DCS), 0);
    qtest_writel(d->qts, CH(ch, DSA), dsa);
    qtest_writel(d->qts, CH(ch, DTA), dta);
    qtest_writel(d->qts, CH(ch, DTC), dtc);
    qtest_writel(d->qts, CH(ch, DRT), drt);
    qtest_writel(d->qts, CH(ch, DCM), dcm);
    qtest_writel(d->qts, CH(ch, DCS), DCS_GO);
}

/* The vendor's pair: channel 5 memory to DIN, channel 6 DOUT to memory */
static void dma_run(Des *d, const DesVec *v, bool dec,
                    const uint8_t *in, uint8_t *out, size_t n)
{
    g_autofree uint32_t *buf = g_malloc(n);

    for (size_t i = 0; i < n; i += 4) {
        buf[i / 4] = cpu_to_le32(ldl_be_p(in + i));
    }
    qtest_memwrite(d->qts, SRC, buf, n);
    qtest_memset(d->qts, DST, 0xa5, n);
    qtest_writel(d->qts, PDMA_DMAC, 1);
    setup(d, v, dec, true);
    ch_start(d, 5, SRC, DES_BASE + DIN, n / 4, REQ_DIN, DCM_SAI);
    ch_start(d, 6, DES_BASE + DOUT, DST, n / 4, REQ_DOUT, DCM_DAI);
    g_assert_cmphex(qtest_readl(d->qts, CH(5, DTC)), ==, 0);
    g_assert_cmphex(qtest_readl(d->qts, CH(6, DTC)), ==, 0);
    qtest_memread(d->qts, DST, buf, n);
    for (size_t i = 0; i < n; i += 4) {
        stl_be_p(out + i, le32_to_cpu(buf[i / 4]));
    }
    wr(d, CR1, 0);
}

static void test_part(const void *data)
{
    const DesPart *p = data;
    Des d = { .p = p };

    d.qts = qtest_initf("-machine %s", p->machine);
    for (int i = 0; i < ARRAY_SIZE(vecs); i++) {
        const DesVec *v = &vecs[i];
        uint8_t pt[24], ct[24], out[24];
        size_t n = unhex(v->pt, pt);

        g_assert_cmpuint(unhex(v->ct, ct), ==, n);
        for (int dec = 0; dec < 2; dec++) {
            const uint8_t *in = dec ? ct : pt;
            const uint8_t *want = dec ? pt : ct;
            const char *dir = dec ? "decrypt" : "encrypt";
            g_autofree char *w = g_strdup_printf("%s registers", dir);

            memset(out, 0xa5, sizeof(out));
            pio_run(&d, v, dec, in, out, n);
            expect(p, v, w, out, want, n);

            g_free(w);
            w = g_strdup_printf("%s pdma", dir);
            memset(out, 0xa5, sizeof(out));
            dma_run(&d, v, dec, in, out, n);
            expect(p, v, w, out, want, n);
        }
    }
    qtest_quit(d.qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-des/%s",
                                                parts[i].part);

        qtest_add_data_func(path, &parts[i], test_part);
    }
    return g_test_run();
}
