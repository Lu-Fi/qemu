/*
 * QTest testcase for the T30's LDC against a corpus of its silicon's jobs
 *
 * A T30X's LDC was driven over /dev/mem with synthetic jobs (the sweeps of
 * one register at a time that find where the engine hangs): whether each
 * ended or never did is kept in ingenic-ldc-corpus-data.inc, as the
 * registers of the job and the part's answer. The model has to give the
 * same answer for every job, so a rule that moves a hang window by one
 * table value fails here. A job never reads its picture for that answer,
 * the source is zeroes.
 *
 * The jobs with a CRC32 run on noise pictures the same way, noise as the
 * probe tool made it (xorshift32 bytes), the output pre-filled with noise
 * too since the part leaves some bytes of it unwritten.
 *
 * The hang jobs run as a subset, the ones next to a change of outcome and
 * every 32nd, unless LDC_CORPUS_FULL is set.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"

#define T30_LDC         0x130a0000
#define SRC             0x00800000
#define DST             0x00a00000

#include "ingenic-ldc-corpus-data.inc"

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static void fill(QTestState *qts, uint64_t addr, size_t n, CorpusFill f,
                 bool *kept)
{
    g_autofree uint8_t *buf = g_malloc0(n);
    uint32_t x = f.arg | 1;

    switch (f.kind) {
    case 'K':
        if (*kept) {
            return;
        }
        *kept = true;
        break;
    case 'F':
        memset(buf, 0xff, n);
        break;
    case 'B':
        for (size_t i = 0; i < n; i++) {
            buf[i] = (i >> f.arg) & 1 ? 0xff : 0;
        }
        break;
    case 'N':
        for (size_t i = 0; i < n; i++) {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            buf[i] = x;
        }
        break;
    }
    qtest_memwrite(qts, addr, buf, n);
}

static void corpus_run(QTestState *qts, const CorpusJob *j, CorpusFill in,
                       bool *kept)
{
    size_t len = j->w * j->h * 3 / 2;

    fill(qts, SRC, len, in, kept);
    fill(qts, DST, len, j->out, kept);
    qtest_writel(qts, T30_LDC + 0x04, 1);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 200 * 1000 * 1000);
}

static void test_captures(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    static const uint8_t reg[] = { 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24 };
    int bad = 0;

    for (size_t n = 0; n < ARRAY_SIZE(corpus_jobs); n++) {
        const CorpusJob *j = &corpus_jobs[n];
        size_t len = j->w * j->h * 3 / 2;
        g_autofree uint8_t *buf = g_malloc(len);
        bool kept = false;

        qtest_writel(qts, T30_LDC + 0x00, 2);
        for (int i = 0; i < ARRAY_SIZE(j->ylut); i++) {
            qtest_writel(qts, T30_LDC + 0x100 + 4 * i, j->ylut[i]);
            qtest_writel(qts, T30_LDC + 0x500 + 4 * i, j->uvlut[i]);
        }
        for (int i = 0; i < ARRAY_SIZE(reg); i++) {
            qtest_writel(qts, T30_LDC + reg[i], j->regs[i]);
        }
        for (int i = 0; i < 4; i++) {
            qtest_writel(qts, T30_LDC + 0x3c + 4 * i, j->w);
        }
        qtest_writel(qts, T30_LDC + 0x08, j->h << 16 | j->w);
        qtest_writel(qts, T30_LDC + 0x2c, SRC);
        qtest_writel(qts, T30_LDC + 0x30, SRC + j->w * j->h);
        qtest_writel(qts, T30_LDC + 0x34, DST);
        qtest_writel(qts, T30_LDC + 0x38, DST + j->w * j->h);
        qtest_writel(qts, T30_LDC + 0x00, 0x38);
        if (j->a.kind) {
            if (j->wr_a) {
                qtest_writel(qts, T30_LDC + 0x18, j->wr_a);
            }
            corpus_run(qts, j, j->a, &kept);
            qtest_writel(qts, T30_LDC + 0x18, j->regs[3]);
        }
        corpus_run(qts, j, j->b, &kept);
        qtest_memread(qts, DST, buf, len);
        if (crc32_le(buf, len) != j->crc) {
            bad++;
            g_printerr("%s: silicon CRC 0x%08x, model 0x%08x\n", j->tag,
                       j->crc, crc32_le(buf, len));
        }
    }
    g_assert_cmpint(bad, ==, 0);
    qtest_quit(qts);
}

static void test_corpus(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    bool full = g_getenv("LDC_CORPUS_FULL");
    gsize len;
    guchar *stream = g_base64_decode(corpus_stream, &len);
    const guchar *p = stream;
    /* the registers as the part holds them: 0x08 to 0x48, both tables */
    uint8_t rf[0x900 - 0x08] = {};
    int bad = 0, run = 0;

    for (size_t s = 0; s < ARRAY_SIZE(corpus_sets); s++) {
        for (int j = 0; j < corpus_sets[s].n; j++) {
            bool hang = *p >> 7, fast = *p >> 6 & 1;
            int changes = *p++ & 0x3f;

            while (changes--) {
                size_t r = corpus_regs[*p++];
                uint32_t v = 0;

                for (int sh = 0;; sh += 7) {
                    v |= (uint32_t)(*p & 0x7f) << sh;
                    if (!(*p++ & 0x80)) {
                        break;
                    }
                }
                stl_le_p(rf + r - 0x08, v);
            }
            if (!full && !fast) {
                continue;
            }
            qtest_writel(qts, T30_LDC + 0x00, 2);
            qtest_memwrite(qts, T30_LDC + 0x08, rf, 0x48 - 0x08 + 4);
            qtest_memwrite(qts, T30_LDC + 0x100, rf + 0x100 - 0x08, 0x400);
            qtest_memwrite(qts, T30_LDC + 0x500, rf + 0x500 - 0x08, 0x400);
            qtest_writel(qts, T30_LDC + 0x00, 0x38);
            qtest_writel(qts, T30_LDC + 0x04, 1);
            qtest_writel(qts, T30_LDC + 0x00, 0x39);
            qtest_clock_step(qts, 20 * 1000 * 1000);
            uint32_t st = qtest_readl(qts, T30_LDC + 0x04);

            run++;
            if (((st & 2) && !(st & 1)) != hang) {
                bad++;
                g_printerr("%s job %d: silicon %s, model status 0x%08x\n",
                           corpus_sets[s].name, j, hang ? "hung" : "ended",
                           st);
            }
        }
    }
    g_assert_cmpint(p - stream, ==, len);
    g_assert_cmpint(bad, ==, 0);
    g_assert_cmpint(run, >, 0);
    g_free(stream);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-ldc-corpus/t30/hangs", test_corpus);
    qtest_add_func("ingenic-ldc-corpus/t30/captures", test_captures);
    return g_test_run();
}
