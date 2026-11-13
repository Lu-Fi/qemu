/*
 * QTest testcase for the Ingenic A1 HDMI controller and its monitor
 *
 * The receiver sense and PLL lock status, the DDC master reading the EDID
 * (both blocks, the segment pointer, the NACK of an absent slave), the
 * interrupt status and mask, the clock gate, the monitor unplugged, the
 * audio registers and the audio sink AIC1 plays into.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define HDMI            0x13380000
#define HDMI_PHY        0x10075000
#define H_RXSENSE       0x410
#define H_IRQ_STATUS    0x414
#define H_IRQ_MASK      0x614
#define H_DDC_SLAVE     0x1f800
#define H_DDC_OFFSET    0x1f804
#define H_DDC_RDATA     0x1f80c
#define H_DDC_CMD       0x1f810
#define H_DDC_SEGMENT   0x1f828
#define PHY_POWER       0x2c0
#define PHY_SENSE       0x334
#define VDAC            0x10076000
#define CPM_CLKGR0      0x10000030
#define GATE_HDMI       (1u << 27)
#define GATE_VDE        (1u << 28)
#define CPM_CPCCR       0x10000000
#define CPCCR_EXTCLK    0x55800000      /* SCLK_A and the CPU from EXTCLK */
#define CPM_I2S1TCDR    0x10000100
#define AIC1            0x10021000
#define AICFR           0x00
#define AICCR           0x04
#define AICSR           0x14
#define AICDR           0x34
#define H_INVIDCONF     0x4000
#define H_AUDSSTAT      0x4190
#define H_MC_CLKDIS     0x10004
#define H_AUD_CONF0     0xc400
#define H_AUD_N2        0xc804
#define H_AUD_CTS3      0xc814

/* The kernel leaves gate_hdmi and gate_vde open; reset has gate_vde shut */
static QTestState *a1_init(const char *args)
{
    QTestState *qts = qtest_initf("-machine ingenic-a1 %s", args);

    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) &
                 ~(GATE_HDMI | GATE_VDE));
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, HDMI + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, HDMI + off, v);
}

/* One DDC byte at SEG:OFF from SLAVE, and the status it left (cleared) */
static uint32_t ddc(QTestState *qts, uint32_t slave, uint32_t seg,
                    uint32_t off, uint32_t *status)
{
    wr(qts, H_DDC_SLAVE, slave);
    wr(qts, H_DDC_OFFSET, off);
    if (seg) {
        wr(qts, H_DDC_SEGMENT, seg);
    }
    wr(qts, H_DDC_CMD, seg ? 2 : 1);
    *status = rd(qts, H_IRQ_STATUS);
    wr(qts, H_IRQ_STATUS, *status);
    g_assert_cmpuint(rd(qts, H_IRQ_STATUS), ==, 0);
    return rd(qts, H_DDC_RDATA);
}

/* CLKGR0 bits 27 and 28 shut the window and the PHY, as on an A1N */
static void test_gate(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    uint32_t g;

    g_assert(qtest_readl(qts, CPM_CLKGR0) & GATE_VDE);
    g_assert_cmphex(rd(qts, 0), ==, 0);
    g_assert_cmphex(rd(qts, H_AUD_CONF0), ==, 0);
    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) &
                 ~(GATE_HDMI | GATE_VDE));
    g = qtest_readl(qts, CPM_CLKGR0);
    /* what an A1N reads with the gates open and nothing written */
    g_assert_cmphex(rd(qts, 0), ==, 0x21);
    g_assert_cmphex(rd(qts, H_AUD_CONF0), ==, 0x2f);
    g_assert_cmphex(rd(qts, 0x604), ==, 4);

    /* shut again: zeros, writes dropped, what was held comes back */
    wr(qts, 0x604, 0x33);
    qtest_writel(qts, HDMI_PHY + 0x40, 0x55);
    for (int i = 0; i < 3; i++) {
        static const uint32_t shut[] = { GATE_VDE, GATE_VDE | GATE_HDMI,
                                         GATE_HDMI };

        qtest_writel(qts, CPM_CLKGR0, g | shut[i]);
        g_assert_cmphex(rd(qts, 0), ==, 0);
        g_assert_cmphex(rd(qts, 0x604), ==, 0);
        g_assert_cmphex(qtest_readl(qts, HDMI_PHY + 0x40), ==, 0);
        wr(qts, 0x604, 0x5a);
        qtest_writel(qts, HDMI_PHY + 0x40, 0x66);
        qtest_writel(qts, CPM_CLKGR0, g);
        g_assert_cmphex(rd(qts, 0x604), ==, 0x33);
        g_assert_cmphex(qtest_readl(qts, HDMI_PHY + 0x40), ==, 0x55);
    }
    qtest_quit(qts);
}

static void test_sense_lock(void)
{
    QTestState *qts = a1_init("");

    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0x3c);
    qtest_writel(qts, HDMI_PHY + PHY_POWER, 4);
    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0x3e);
    g_assert_cmphex(qtest_readl(qts, HDMI_PHY + PHY_SENSE), ==, 0x55);
    qtest_writel(qts, HDMI_PHY + PHY_POWER, 0);
    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0x3c);
    g_assert_cmphex(qtest_readl(qts, HDMI_PHY + PHY_SENSE), ==, 0);
    qtest_quit(qts);
}

static void test_edid(void)
{
    QTestState *qts = a1_init("");
    static const uint8_t head[8] = { 0, 255, 255, 255, 255, 255, 255, 0 };
    uint32_t st, sum = 0;

    for (int i = 0; i < 8; i++) {
        g_assert_cmpuint(ddc(qts, 0x50, 0, i, &st), ==, head[i]);
        g_assert_cmpuint(st, ==, 2);
    }
    for (int i = 0; i < 128; i++) {
        sum += ddc(qts, 0x50, 0, i, &st);
    }
    g_assert_cmpuint(sum & 0xff, ==, 0);
    g_assert_cmpuint(ddc(qts, 0x50, 0, 126, &st), ==, 1);
    /* The second block, at 128 of segment 0, is CEA-861 */
    g_assert_cmpuint(ddc(qts, 0x50, 0, 128, &st), ==, 2);
    g_assert_cmpuint(ddc(qts, 0x50, 0, 129, &st), ==, 3);
    g_assert_cmpuint(ddc(qts, 0x50, 1, 0, &st), ==, 0);
    g_assert_cmpuint(st, ==, 2);
    sum = 0;
    for (int i = 128; i < 256; i++) {
        sum += ddc(qts, 0x50, 0, i, &st);
    }
    g_assert_cmpuint(sum & 0xff, ==, 0);
    qtest_quit(qts);
}

static void test_nack_mask(void)
{
    QTestState *qts = a1_init("");
    uint32_t st;

    /* The SCDC slave is not there: bit 0 */
    ddc(qts, 0x54, 0, 0, &st);
    g_assert_cmpuint(st, ==, 1);
    /* Masked, the status shows without the interrupt */
    wr(qts, H_IRQ_MASK, 3);
    ddc(qts, 0x50, 0, 0, &st);
    g_assert_cmpuint(st, ==, 2);
    g_assert_cmpuint(rd(qts, H_IRQ_MASK), ==, 3);
    qtest_quit(qts);
}

static void test_unplug(void)
{
    QTestState *qts = a1_init("");
    QDict *rsp;
    uint32_t st;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': '/machine/soc/hdmi', 'property': 'connected', "
                    "'value': false}}");
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0);
    ddc(qts, 0x50, 0, 0, &st);
    g_assert_cmpuint(st, ==, 1);
    qtest_quit(qts);
}

/* No monitor, output not started: status and PHY read 0; masks and DDC reset */
static void test_reset_values(void)
{
    QTestState *qts = a1_init("");
    static const uint32_t mask[] = { 0x18, 0x4, 0x10, 0x8, 0, 0x4 };
    static const struct {
        uint32_t off, val;
    } ddc_rv[] = {
        { 0x1f814, 0x40 }, { 0x1f81c, 0xb }, { 0x1f824, 1 },
        { 0x1f830, 0x6c }, { 0x1f838, 0x7f }, { 0x1f840, 0x11 },
        { 0x1f800, 0 }, { 0x1f810, 0 },
    };
    static const uint32_t id[] = { 0x21, 0x1a, 0xa0, 0x01, 0x9a, 0x22, 0xfe };
    QDict *rsp;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': '/machine/soc/hdmi', 'property': 'connected', "
                    "'value': false}}");
    qobject_unref(rsp);
    for (unsigned i = 0; i < 7; i++) {
        g_assert_cmphex(rd(qts, i * 4), ==, id[i]);
    }
    g_assert_cmphex(rd(qts, 0x1c), ==, 0);
    for (unsigned i = 0; i < 6; i++) {
        g_assert_cmphex(rd(qts, 0x400 + i * 4), ==, 0);
        g_assert_cmphex(rd(qts, 0x600 + i * 4), ==, mask[i]);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(ddc_rv); i++) {
        g_assert_cmphex(rd(qts, ddc_rv[i].off), ==, ddc_rv[i].val);
    }
    for (unsigned i = 0; i < 0x100; i += 4) {
        g_assert_cmphex(qtest_readl(qts, HDMI_PHY + i), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, HDMI_PHY + PHY_SENSE), ==, 0);
    qtest_quit(qts);
}

/* As an A1N reads it with the VO running: zero, and writes are not kept */
static void test_vdac(void)
{
    QTestState *qts = a1_init("");
    uint32_t off;

    for (off = 0x200; off < 0x300; off += 4) {
        g_assert_cmphex(qtest_readl(qts, VDAC + off), ==, 0);
    }
    qtest_writel(qts, VDAC + 0x288, 0xffffffff);
    qtest_writel(qts, VDAC + 0x294, 0x12345678);
    qtest_writel(qts, VDAC + 0x2a0, 1);
    g_assert_cmphex(qtest_readl(qts, VDAC + 0x288), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VDAC + 0x294), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VDAC + 0x2a0), ==, 0);
    qtest_quit(qts);
}

static uint64_t hdmi_prop(QTestState *qts, const char *prop)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/machine/soc/hdmi', 'property': %s } }",
                         prop);
    uint64_t v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_int(r, "return");
    qobject_unref(r);
    return v;
}

/* The state the vendor's firmware leaves while sample-audio 3 plays */
static void hdmi_audio_up(QTestState *qts)
{
    qtest_writel(qts, HDMI_PHY + PHY_POWER, 4);
    wr(qts, H_INVIDCONF, 0x78);
    wr(qts, H_AUD_CONF0, 0xa1);
}

/* 64 TX FIFO entries of 8-bit samples +64, -64, ... and the AIC1 started */
static void aic1_tone(QTestState *qts)
{
    qtest_writel(qts, AIC1 + AICFR, 1 | 24 << 16 | 7u << 24);
    for (int i = 0; i < 64; i++) {
        qtest_writel(qts, AIC1 + AICDR, i & 1 ? 0xc0 : 0x40);
    }
    qtest_writel(qts, AIC1 + AICCR, 2 | 1 << 12);
}

/* The audio registers: the ID block, what holds and what clears itself */
static void test_audio_regs(void)
{
    QTestState *qts = a1_init("-audio none");

    wr(qts, 0, 0xff);
    g_assert_cmphex(rd(qts, 0), ==, 0x21);
    /* 0xa1 is FIFO reset, I2S select and input on: bit 7 clears itself */
    wr(qts, H_AUD_CONF0, 0xa1);
    g_assert_cmphex(rd(qts, H_AUD_CONF0), ==, 0x21);
    wr(qts, H_AUD_CONF0 + 4, 0x10);
    wr(qts, H_AUD_CONF0 + 0xc, 4);
    g_assert_cmphex(rd(qts, H_AUD_CONF0 + 4), ==, 0x10);
    g_assert_cmphex(rd(qts, H_AUD_CONF0 + 0xc), ==, 4);
    g_assert_cmphex(rd(qts, H_AUD_CONF0 + 0x10), ==, 0x10);
    wr(qts, H_AUD_N2, 0x10);
    wr(qts, H_AUD_CTS3, 0x11);
    g_assert_cmphex(rd(qts, H_AUD_N2), ==, 0x10);
    g_assert_cmphex(rd(qts, H_AUD_CTS3), ==, 0x11);
    g_assert_cmphex(rd(qts, H_AUD_CTS3 + 8), ==, 0x11);
    /* 0x438c bit 3: the audio infoframe, a plain register */
    wr(qts, 0x438c, 0x1f);
    g_assert_cmphex(rd(qts, 0x438c), ==, 0x1f);
    g_assert_cmphex(rd(qts, H_AUDSSTAT), ==, 0);
    wr(qts, H_AUDSSTAT, 1);
    g_assert_cmphex(rd(qts, H_AUDSSTAT), ==, 0);
    qtest_quit(qts);
}

/* AIC1's frames reach the sink only with the audio path open, at AIC1's rate */
static void test_audio_sink(void)
{
    QTestState *qts = a1_init("-audio none");
    uint64_t frames, dropped;

    /* Nothing enabled: the frames are taken and dropped */
    aic1_tone(qts);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmpuint(hdmi_prop(qts, "audio-frames"), ==, 0);
    g_assert_cmpuint(hdmi_prop(qts, "audio-dropped"), >, 0);
    qtest_quit(qts);

    qts = a1_init("-audio none");
    hdmi_audio_up(qts);
    aic1_tone(qts);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmphex(rd(qts, H_AUDSSTAT), ==, 1);
    frames = hdmi_prop(qts, "audio-frames");
    g_assert_cmpuint(frames, >, 64);
    g_assert_cmpuint(hdmi_prop(qts, "audio-rate"), >, 0);
    /* The 64 entries, then silence (a TX underrun) */
    g_assert_cmpuint(hdmi_prop(qts, "audio-nonzero"), ==, 64);
    g_assert_cmpuint(hdmi_prop(qts, "audio-active"), ==, 64);
    g_assert_cmpuint(hdmi_prop(qts, "audio-crossings"), ==, 63);
    g_assert_cmpuint(hdmi_prop(qts, "audio-peak"), ==, 0x4000);
    g_assert_cmpuint(hdmi_prop(qts, "audio-dropped"), ==, 0);

    /* The audio clock stopped (MC_CLKDIS bit 3): dropped again */
    wr(qts, H_MC_CLKDIS, 8);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmpuint(hdmi_prop(qts, "audio-frames"), ==, frames);
    dropped = hdmi_prop(qts, "audio-dropped");
    g_assert_cmpuint(dropped, >, 0);
    g_assert_cmphex(rd(qts, H_AUDSSTAT), ==, 0);
    wr(qts, H_MC_CLKDIS, 0);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmpuint(hdmi_prop(qts, "audio-frames"), >, frames);

    /* No I2S input enabled, or DVI mode: dropped */
    frames = hdmi_prop(qts, "audio-frames");
    wr(qts, H_AUD_CONF0, 0x20);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmpuint(hdmi_prop(qts, "audio-frames"), ==, frames);
    wr(qts, H_AUD_CONF0, 0x21);
    wr(qts, H_INVIDCONF, 0x70);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmpuint(hdmi_prop(qts, "audio-frames"), ==, frames);
    qtest_quit(qts);
}

/* AIC1 plays at CPM I2S1's clock: half the divider, twice the frames */
static void test_audio_rate(void)
{
    uint64_t frames[2];

    for (int i = 0; i < 2; i++) {
        QTestState *qts = a1_init("-audio none");

        /* SCLK_A from EXTCLK: with nothing to boot the PLLs are off */
        qtest_writel(qts, CPM_CPCCR, CPCCR_EXTCLK);
        qtest_writel(qts, CPM_I2S1TCDR, 1u << 29 | (1 + i) << 20 | 2);
        hdmi_audio_up(qts);
        aic1_tone(qts);
        qtest_clock_step(qts, 100 * 1000 * 1000);
        frames[i] = hdmi_prop(qts, "audio-frames");
        qtest_quit(qts);
    }
    g_assert_cmpuint(frames[0], >, 64);
    g_assert_cmpfloat((double)frames[1] / frames[0], >, 1.9);
    g_assert_cmpfloat((double)frames[1] / frames[0], <, 2.1);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-hdmi/gate", test_gate);
    qtest_add_func("ingenic-a1-hdmi/sense-lock", test_sense_lock);
    qtest_add_func("ingenic-a1-hdmi/edid", test_edid);
    qtest_add_func("ingenic-a1-hdmi/nack-mask", test_nack_mask);
    qtest_add_func("ingenic-a1-hdmi/unplug", test_unplug);
    qtest_add_func("ingenic-a1-hdmi/reset-values", test_reset_values);
    qtest_add_func("ingenic-a1-hdmi/vdac", test_vdac);
    qtest_add_func("ingenic-a1-hdmi/audio-regs", test_audio_regs);
    qtest_add_func("ingenic-a1-hdmi/audio-sink", test_audio_sink);
    qtest_add_func("ingenic-a1-hdmi/audio-rate", test_audio_rate);
    return g_test_run();
}
