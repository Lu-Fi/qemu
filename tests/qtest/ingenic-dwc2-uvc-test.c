/*
 * QTest testcase for the DWC2 UVC host
 *
 * The test is the gadget: it drives the DWC2's device registers the way
 * the Linux driver does, answers the host's enumeration with a
 * configuration holding an MJPEG and an H.264 format, takes the stream
 * negotiation (probe, probe reply, commit) and sends bulk payloads with
 * UVC headers; the H.264 frames leave out the end-of-frame bit, as the 3.10
 * gadget does. The host must write the pictures of each format to its
 * output directory: from the second MJPEG frame on (the first only marks a
 * frame boundary), and for H.264 from the first IDR picture on, with a
 * frame of the other format left over from the first stream skipped.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qobject/qdict.h"
#include "hw/usb/dwc2-regs.h"

#define OTG             0x13500000
#define SETUP_BUF       0x01000000      /* setup packet, then OUT data */
#define IN_BUF          0x01001000
#define BULK_BUF        0x01002000
#define BULK_MPS        512
#define PROBE_LEN       26

#define EPENA_CNAK      (DXEPCTL_EPENA | DXEPCTL_CNAK)

static const uint8_t guid_h264[16] = {
    'H', '2', '6', '4', 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
};

typedef struct {
    QTestState *qts;
    uint8_t cfg[256];
    int cfg_len;
    uint8_t probe[PROBE_LEN];
    uint32_t arm;               /* the next SETUP or OUT data lands here */
    uint8_t pending_set;        /* control write whose data is expected */
    int committed_format;       /* 0 until COMMIT, then bFormatIndex */
    int stream_step;            /* payloads sent for the committed format */
} Gadget;

static uint32_t reg_r(Gadget *g, uint32_t off)
{
    return qtest_readl(g->qts, OTG + off);
}

static void reg_w(Gadget *g, uint32_t off, uint32_t val)
{
    qtest_writel(g->qts, OTG + off, val);
}

static void put(uint8_t *d, int *n, const uint8_t *src, int len)
{
    memcpy(d + *n, src, len);
    *n += len;
}

static void build_config(Gadget *g)
{
    uint8_t *c = g->cfg;
    int n = 0;
    const uint8_t cfg_head[] = { 9, 2, 0, 0, 1, 1, 0, 0xc0, 1 };
    const uint8_t vs_intf[] = { 9, 4, 1, 0, 1, 0x0e, 2, 0, 0 };
    const uint8_t mjpg_fmt[] = { 11, 0x24, 6, 1, 1, 0, 1, 0, 0, 0, 0 };
    uint8_t mjpg_frame[30] = { 30, 0x24, 7, 1, 0 };
    uint8_t h264_fmt[28] = { 28, 0x24, 0x10, 2, 1 };
    uint8_t h264_frame[30] = { 30, 0x24, 0x11, 1, 0 };
    const uint8_t ep[] = { 7, 5, 0x81, 2, 0x00, 0x02, 0 };

    stw_le_p(mjpg_frame + 5, 640);
    stw_le_p(mjpg_frame + 7, 360);
    stl_le_p(mjpg_frame + 21, 333333);
    mjpg_frame[25] = 1;
    stl_le_p(mjpg_frame + 26, 333333);
    memcpy(h264_fmt + 5, guid_h264, 16);
    h264_fmt[22] = 1;
    stw_le_p(h264_frame + 5, 640);
    stw_le_p(h264_frame + 7, 360);
    stl_le_p(h264_frame + 17, 400000);
    h264_frame[21] = 1;
    stl_le_p(h264_frame + 26, 400000);

    put(c, &n, cfg_head, sizeof(cfg_head));
    put(c, &n, vs_intf, sizeof(vs_intf));
    put(c, &n, mjpg_fmt, sizeof(mjpg_fmt));
    put(c, &n, mjpg_frame, sizeof(mjpg_frame));
    put(c, &n, h264_fmt, sizeof(h264_fmt));
    put(c, &n, h264_frame, sizeof(h264_frame));
    put(c, &n, ep, sizeof(ep));
    stw_le_p(c + 2, n);
    g->cfg_len = n;
}

/* Take the next SETUP packet and be ready for the data and status stages. */
static void arm_ep0_out(Gadget *g)
{
    reg_w(g, DOEPDMA(0), g->arm);
    reg_w(g, DOEPTSIZ(0), DOEPTSIZ0_SUPCNT(3) | DOEPTSIZ0_PKTCNT | 64);
    reg_w(g, DOEPCTL(0), EPENA_CNAK);
}

static void arm_ep0_in(Gadget *g, const uint8_t *data, int len)
{
    if (len) {
        qtest_memwrite(g->qts, IN_BUF, data, len);
    }
    reg_w(g, DIEPDMA(0), IN_BUF);
    reg_w(g, DIEPTSIZ(0), DIEPTSIZ0_PKTCNT((len + 63) / 64 ? : 1) | len);
    reg_w(g, DIEPCTL(0), EPENA_CNAK);
}

static void send_payload(Gadget *g, const uint8_t *data, int len)
{
    uint32_t pkts = len ? (len + BULK_MPS - 1) / BULK_MPS : 1;

    qtest_memwrite(g->qts, BULK_BUF, data, len);
    reg_w(g, DIEPDMA(1), BULK_BUF);
    reg_w(g, DIEPTSIZ(1), (pkts << 19) | len);
    reg_w(g, DIEPCTL(1), DXEPCTL_EPENA | DXEPCTL_CNAK | DXEPCTL_USBACTEP |
          DXEPCTL_EPTYPE_BULK | BULK_MPS);
}

/* One frame as two payloads: a header and half the picture, then the rest. */
static void frame_payloads(const uint8_t *pic, int len, int fid, bool eof,
                           uint8_t *out0,
                           int *len0, uint8_t *out1, int *len1)
{
    int half = len / 2;

    out0[0] = 2;
    out0[1] = 0x80 | fid;
    memcpy(out0 + 2, pic, half);
    *len0 = 2 + half;
    out1[0] = 2;
    out1[1] = 0x80 | fid | (eof ? 0x02 : 0);
    memcpy(out1 + 2, pic + half, len - half);
    *len1 = 2 + len - half;
}

#define NFRAMES 5
#define FRAME_LEN 1400

static uint8_t pictures[2][NFRAMES][FRAME_LEN];
static int picture_len[2][NFRAMES];

/*
 * What the stream of each format carries. MJPEG: five frames, the first
 * is skipped. H.264: a left-over MJPEG frame, a P picture before any IDR,
 * then an IDR and a P picture.
 */
static void make_pictures(void)
{
    static const uint8_t nal_p[] = { 0, 0, 0, 1, 0x41 };
    static const uint8_t nal_idr[] = { 0, 0, 0, 1, 0x67, 0, 0, 0, 1, 0x65 };
    int i, j;

    for (i = 0; i < NFRAMES; i++) {
        picture_len[0][i] = FRAME_LEN - 100 * i;
        picture_len[1][i] = FRAME_LEN - 40 * i;
        for (j = 0; j < picture_len[0][i]; j++) {
            pictures[0][i][j] = (j * 7 + i * 31) & 0x7f;
        }
        pictures[0][i][0] = 0xff;
        pictures[0][i][1] = 0xd8;
        pictures[0][i][picture_len[0][i] - 2] = 0xff;
        pictures[0][i][picture_len[0][i] - 1] = 0xd9;
        for (j = 0; j < picture_len[1][i]; j++) {
            pictures[1][i][j] = (j * 5 + i * 17) & 0x3f;
        }
    }
    /* H.264 sequence: [0] stale JPEG, [1] P, [2] IDR, [3] P, [4] P */
    memcpy(pictures[1][0], pictures[0][0], picture_len[0][0]);
    picture_len[1][0] = picture_len[0][0];
    memcpy(pictures[1][1], nal_p, sizeof(nal_p));
    memcpy(pictures[1][2], nal_idr, sizeof(nal_idr));
    memcpy(pictures[1][3], nal_p, sizeof(nal_p));
    memcpy(pictures[1][4], nal_p, sizeof(nal_p));
}

/* False once the host has stopped reading, as when it moves on. */
static bool bulk_wait_done(Gadget *g)
{
    int i;

    for (i = 0; i < 500; i++) {
        if (reg_r(g, DIEPINT(1)) & DXEPINT_XFERCOMPL) {
            reg_w(g, DIEPINT(1), DXEPINT_XFERCOMPL);
            return true;
        }
        qtest_clock_step(g->qts, 100000);
    }
    return false;
}

/* Send the committed format's frames, then go quiet. */
static void run_stream(Gadget *g, int fmt)
{
    uint8_t p0[FRAME_LEN + 8], p1[FRAME_LEN + 8];
    int l0, l1, i;

    for (i = 0; i < NFRAMES; i++) {
        frame_payloads(pictures[fmt - 1][i], picture_len[fmt - 1][i], i & 1,
                       fmt == 1, p0, &l0, p1, &l1);
        send_payload(g, p0, l0);
        if (!bulk_wait_done(g)) {
            return;
        }
        send_payload(g, p1, l1);
        if (!bulk_wait_done(g)) {
            return;
        }
    }
}

static void control_setup(Gadget *g, const uint8_t *s)
{
    uint8_t type = s[0], req = s[1];
    uint16_t value = lduw_le_p(s + 2), len = lduw_le_p(s + 6);
    uint8_t dev[18] = { 18, 1, 0, 2, 0xef, 2, 1, 64, 0x6b, 0x1d, 0x02, 0x01,
                        0x10, 0, 0, 0, 0, 1 };

    /* A control write's data must not land on the SETUP packet. */
    g->arm ^= 0x100;
    arm_ep0_out(g);
    g->pending_set = 0;
    if (type == 0x80 && req == 6 && (value >> 8) == 1) {
        arm_ep0_in(g, dev, MIN(len, 18));
    } else if (type == 0x80 && req == 6 && (value >> 8) == 2) {
        arm_ep0_in(g, g->cfg, MIN(len, g->cfg_len));
    } else if (type == 0x21 && req == 1) {
        g_assert_cmpint(len, ==, PROBE_LEN);
        g->pending_set = value >> 8;
    } else if (type == 0xa1 && req == 0x81) {
        uint8_t reply[PROBE_LEN];

        memcpy(reply, g->probe, PROBE_LEN);
        stl_le_p(reply + 22, 16384);
        arm_ep0_in(g, reply, MIN(len, PROBE_LEN));
    } else {
        /* SET_ADDRESS, SET_CONFIGURATION: the status stage only */
        arm_ep0_in(g, NULL, 0);
    }
}

static void control_data(Gadget *g)
{
    uint8_t data[PROBE_LEN];

    qtest_memread(g->qts, g->arm, data, PROBE_LEN);
    memcpy(g->probe, data, PROBE_LEN);
    if (g->pending_set == 2) {
        g->committed_format = data[2];
    }
    g->arm ^= 0x100;
    arm_ep0_out(g);
    arm_ep0_in(g, NULL, 0);
    g->pending_set = 0;
}

static void test_uvc_host(void)
{
    g_autofree char *dir = g_dir_make_tmp("uvc-test-XXXXXX", NULL);
    g_autofree char *mjpg = g_strdup_printf("%s/uvc-0.mjpg", dir);
    g_autofree char *h264 = g_strdup_printf("%s/uvc-1.h264", dir);
    Gadget g = { 0 };
    QDict *rsp;
    g_autofree char *state = NULL;
    char *got = NULL;
    gsize got_len;
    GString *exp = g_string_new(NULL);
    int i, streamed = 0;

    make_pictures();
    build_config(&g);
    g.arm = SETUP_BUF;
    g.qts = qtest_initf("-machine ingenic-t31 -device "
                        "dwc2-uvc-host,id=h,x-out=%s,x-format=mjpeg+h264,"
                        "x-frames=2", dir);

    /* Device mode, pull-up on, ready for the first SETUP. */
    reg_w(&g, GOTGCTL, GOTGCTL_CONID_B);
    arm_ep0_out(&g);
    reg_w(&g, DCTL, 0);

    for (i = 0; i < 20000; i++) {
        uint32_t oint = reg_r(&g, DOEPINT(0));

        if (oint & DXEPINT_SETUP) {
            uint8_t s[8];

            reg_w(&g, DOEPINT(0), DXEPINT_SETUP | DXEPINT_SETUP_RCVD |
                  DXEPINT_XFERCOMPL);
            qtest_memread(g.qts, g.arm, s, 8);
            control_setup(&g, s);
        } else if ((oint & DXEPINT_XFERCOMPL) && g.pending_set) {
            reg_w(&g, DOEPINT(0), DXEPINT_XFERCOMPL);
            control_data(&g);
        } else if (oint & DXEPINT_XFERCOMPL) {
            reg_w(&g, DOEPINT(0), DXEPINT_XFERCOMPL);
        }
        if (g.committed_format && streamed < g.committed_format) {
            /* The commit's status stage must finish before the stream. */
            while (!(reg_r(&g, DIEPINT(0)) & DXEPINT_XFERCOMPL)) {
                qtest_clock_step(g.qts, 100000);
            }
            reg_w(&g, DIEPINT(0), DXEPINT_XFERCOMPL);
            streamed = g.committed_format;
            run_stream(&g, streamed);
            g.committed_format = 0;
        }
        qtest_clock_step(g.qts, 100000);
        if (streamed == 2) {
            break;
        }
    }
    g_assert_cmpint(streamed, ==, 2);

    for (i = 0; i < 200; i++) {
        rsp = qtest_qmp(g.qts, "{'execute': 'qom-get', 'arguments': "
                        "{'path': '/machine/peripheral/h', "
                        "'property': 'state'}}");
        state = g_strdup(qdict_get_str(rsp, "return"));
        qobject_unref(rsp);
        if (!strcmp(state, "done")) {
            break;
        }
        g_free(state);
        state = NULL;
        qtest_clock_step(g.qts, 1000000);
    }
    g_assert_cmpstr(state, ==, "done");

    /* MJPEG: the first frame was the boundary; the next two are kept. */
    for (i = 1; i <= 2; i++) {
        g_string_append_len(exp, (char *)pictures[0][i], picture_len[0][i]);
    }
    g_assert_true(g_file_get_contents(mjpg, &got, &got_len, NULL));
    g_assert_cmpmem(got, got_len, exp->str, exp->len);
    g_free(got);

    /* H.264: the left-over JPEG and the P picture before the IDR are out. */
    g_string_truncate(exp, 0);
    for (i = 2; i <= 3; i++) {
        g_string_append_len(exp, (char *)pictures[1][i], picture_len[1][i]);
    }
    g_assert_true(g_file_get_contents(h264, &got, &got_len, NULL));
    g_assert_cmpmem(got, got_len, exp->str, exp->len);
    g_free(got);

    g_string_free(exp, true);
    qtest_quit(g.qts);
    unlink(mjpg);
    unlink(h264);
    rmdir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/dwc2-uvc-host", test_uvc_host);
    return g_test_run();
}
