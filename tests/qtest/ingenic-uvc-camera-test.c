/*
 * QTest testcase for the USB UVC camera
 *
 * The test is the A1's DWC2 host driver: it resets the port, addresses and
 * configures the camera, reads its descriptors, negotiates a stream (probe,
 * commit) and reads bulk payloads. MJPEG pictures must be whole JPEGs; a
 * YUYV picture must carry the frame number the payload headers' PTS gives
 * in its binary cells, so that a moving picture is told from a stuck one.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "hw/usb/dwc2-regs.h"

#define USB_DIR_OUT                 0x00
#define USB_DIR_IN                  0x80
#define USB_TYPE_CLASS              0x20
#define USB_RECIP_INTERFACE         0x01
#define USB_ENDPOINT_XFER_CONTROL   0
#define USB_ENDPOINT_XFER_BULK      2
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_CONFIGURATION   0x09

#define OTG             0x13640000      /* the A1's OTG1 */
#define SETUP_BUF       0x01000000
#define DATA_BUF        0x01001000
#define BULK_BUF        0x01100000
#define NOTE_BUF        0x01200000
#define HDR_LEN         12
#define PAYLOAD         16384
#define INTERVAL        333333u

typedef struct {
    QTestState *qts;
} Host;

static uint32_t reg_r(Host *h, uint32_t off)
{
    return qtest_readl(h->qts, OTG + off);
}

static void reg_w(Host *h, uint32_t off, uint32_t val)
{
    qtest_writel(h->qts, OTG + off, val);
}

/* One transfer on channel 0; returns the bytes moved */
static unsigned xfer(Host *h, unsigned addr, unsigned ep, unsigned type,
                     bool in, unsigned pid, uint32_t dma, unsigned len)
{
    unsigned mps = type == USB_ENDPOINT_XFER_BULK ? 512 : 64;
    unsigned pkts = len ? DIV_ROUND_UP(len, mps) : 1;
    uint32_t hcint = 0;

    reg_w(h, HCINT(0), 0xffffffff);
    reg_w(h, HCTSIZ(0), len | (pkts << TSIZ_PKTCNT_SHIFT) |
                        (pid << TSIZ_SC_MC_PID_SHIFT));
    reg_w(h, HCDMA(0), dma);
    reg_w(h, HCCHAR(0), mps | (ep << HCCHAR_EPNUM_SHIFT) |
                        (in ? HCCHAR_EPDIR : 0) |
                        (type << HCCHAR_EPTYPE_SHIFT) |
                        (addr << HCCHAR_DEVADDR_SHIFT) | HCCHAR_CHENA);
    for (int i = 0; i < 5000; i++) {
        hcint = reg_r(h, HCINT(0));
        if (hcint & (HCINTMSK_XFERCOMPL | HCINTMSK_STALL | HCINTMSK_XACTERR)) {
            break;
        }
        qtest_clock_step(h->qts, 1000000);
    }
    g_assert_cmphex(hcint & (HCINTMSK_STALL | HCINTMSK_XACTERR), ==, 0);
    g_assert_cmphex(hcint & HCINTMSK_XFERCOMPL, ==, HCINTMSK_XFERCOMPL);
    return len - (reg_r(h, HCTSIZ(0)) & TSIZ_XFERSIZE_MASK);
}

/* A control transfer; returns the data stage's bytes */
static int ctrl(Host *h, unsigned addr, uint8_t rt, uint8_t req, uint16_t val,
                uint16_t idx, uint16_t len, uint8_t *data)
{
    uint8_t setup[8] = { rt, req };
    bool in = rt & USB_DIR_IN;
    unsigned n = 0;

    stw_le_p(setup + 2, val);
    stw_le_p(setup + 4, idx);
    stw_le_p(setup + 6, len);
    qtest_memwrite(h->qts, SETUP_BUF, setup, 8);
    xfer(h, addr, 0, USB_ENDPOINT_XFER_CONTROL, false, TSIZ_SC_MC_PID_SETUP,
         SETUP_BUF, 8);
    if (len) {
        if (!in) {
            qtest_memwrite(h->qts, DATA_BUF, data, len);
        }
        n = xfer(h, addr, 0, USB_ENDPOINT_XFER_CONTROL, in,
                 TSIZ_SC_MC_PID_DATA1, DATA_BUF, len);
        if (in) {
            qtest_memread(h->qts, DATA_BUF, data, n);
        }
    }
    xfer(h, addr, 0, USB_ENDPOINT_XFER_CONTROL, !in || !len,
         TSIZ_SC_MC_PID_DATA1, DATA_BUF, 0);
    return in ? n : 0;
}

#define CLASS_IF_OUT    (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define CLASS_IF_IN     (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE)

static void stream_set(Host *h, unsigned cs, unsigned fmt, unsigned frame)
{
    uint8_t blk[34] = { 0 };

    blk[2] = fmt;
    blk[3] = frame;
    stl_le_p(blk + 4, INTERVAL);
    g_assert_cmpint(ctrl(h, 1, CLASS_IF_OUT, 0x01, cs << 8, 1, 34, blk), ==, 0);
}

/* The next picture from the bulk endpoint; its PTS in @pts, size returned */
static size_t picture(Host *h, uint8_t *buf, size_t max, uint32_t *pts,
                      bool *fid)
{
    size_t total = 0;
    bool first = true;

    for (;;) {
        uint8_t pl[PAYLOAD];
        unsigned n = xfer(h, 1, 1, USB_ENDPOINT_XFER_BULK, true,
                          TSIZ_SC_MC_PID_DATA0, BULK_BUF, PAYLOAD);

        qtest_memread(h->qts, BULK_BUF, pl, n);
        g_assert_cmpuint(n, >=, HDR_LEN);
        g_assert_cmpuint(pl[0], ==, HDR_LEN);
        g_assert_cmphex(pl[1] & 0xfc, ==, 0x8c);    /* EOH, SCR, PTS */
        if (first) {
            *pts = ldl_le_p(pl + 2);
            *fid = pl[1] & 1;
            first = false;
        } else {
            g_assert_cmpuint(ldl_le_p(pl + 2), ==, *pts);
            g_assert_cmpint(pl[1] & 1, ==, *fid);
        }
        g_assert_cmpuint(total + n - HDR_LEN, <=, max);
        memcpy(buf + total, pl + HDR_LEN, n - HDR_LEN);
        total += n - HDR_LEN;
        if (pl[1] & 2) {
            return total;
        }
        g_assert_cmpuint(n, ==, PAYLOAD);
    }
}

static void run_camera(const char *extra, const char *camera)
{
    Host h = { 0 };
    uint8_t d[512], *cfg = g_malloc(8192), *pic = g_malloc(1 << 20);
    unsigned len, nfmt = 0, nframe = 0, niad = 0;
    bool vs = false;
    uint32_t pts0, pts1;
    bool fid0, fid1;
    size_t n;

    h.qts = qtest_initf("-machine ingenic-a1 %s "
                        "-device usb-uvc-camera,bus=usb-bus.1,port=1%s",
                        extra, camera);

    /* the port: power, reset, enabled with a device connected */
    g_assert(reg_r(&h, HPRT0) & HPRT0_CONNSTS);
    reg_w(&h, HPRT0, HPRT0_PWR | HPRT0_RST);
    reg_w(&h, HPRT0, HPRT0_PWR);
    g_assert(reg_r(&h, HPRT0) & HPRT0_ENA);

    g_assert_cmpint(ctrl(&h, 0, USB_DIR_OUT, USB_REQ_SET_ADDRESS, 1, 0, 0, d),
                    ==, 0);
    g_assert_cmpint(ctrl(&h, 1, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, 0x0100, 0,
                         18, d), ==, 18);
    g_assert_cmpuint(d[4], ==, 0xef);               /* misc, IAD */
    g_assert_cmpuint(d[5], ==, 2);
    g_assert_cmpuint(d[6], ==, 1);
    g_assert_cmpuint(lduw_le_p(d + 8), ==, 0x46f4);

    g_assert_cmpint(ctrl(&h, 1, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, 0x0200, 0,
                         9, d), ==, 9);
    len = lduw_le_p(d + 2);
    g_assert_cmpuint(len, <, 8192);
    g_assert_cmpint(ctrl(&h, 1, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, 0x0200, 0,
                         len, cfg), ==, len);
    for (unsigned i = 0; i < len; i += cfg[i]) {
        if (cfg[i + 1] == 0x0b) {
            niad++;
            g_assert_cmpuint(cfg[i + 4], ==, 0x0e);
        } else if (cfg[i + 1] == 4) {
            vs = cfg[i + 2] == 1;
        } else if (vs && cfg[i + 1] == 0x24 && cfg[i + 2] >= 4 &&
                   cfg[i + 2] <= 7) {
            (cfg[i + 2] == 4 || cfg[i + 2] == 6) ? nfmt++ : nframe++;
        } else if (cfg[i + 1] == 5) {
            g_assert_cmphex(cfg[i + 2], ==, 0x81);
            g_assert_cmpuint(cfg[i + 3], ==, USB_ENDPOINT_XFER_BULK);
        }
    }
    g_assert_cmpuint(niad, ==, 1);
    g_assert_cmpuint(nfmt, ==, 3);
    g_assert_cmpuint(nframe, ==, 11);

    g_assert_cmpint(ctrl(&h, 1, USB_DIR_OUT, USB_REQ_SET_CONFIGURATION, 1, 0,
                         0, d), ==, 0);

    /* the processing unit's brightness takes a value and keeps it */
    stw_le_p(d, 20);
    g_assert_cmpint(ctrl(&h, 1, CLASS_IF_OUT, 0x01, 2 << 8, 2 << 8, 2, d),
                    ==, 0);
    g_assert_cmpint(ctrl(&h, 1, CLASS_IF_IN, 0x81, 2 << 8, 2 << 8, 2, d),
                    ==, 2);
    g_assert_cmpint((int16_t)lduw_le_p(d), ==, 20);

    /* MJPEG 640x480: the negotiated block, then two whole JPEGs */
    stream_set(&h, 1, 1, 3);
    g_assert_cmpint(ctrl(&h, 1, CLASS_IF_IN, 0x81, 1 << 8, 1, 34, d), ==, 34);
    g_assert_cmpuint(d[2], ==, 1);
    g_assert_cmpuint(d[3], ==, 3);
    g_assert_cmpuint(ldl_le_p(d + 4), ==, INTERVAL);
    g_assert_cmpuint(ldl_le_p(d + 22), ==, PAYLOAD);
    stream_set(&h, 2, 1, 3);
    n = picture(&h, pic, 1 << 20, &pts0, &fid0);
    g_assert_cmpuint(n, >, 1000);
    g_assert_cmphex(pic[0] << 8 | pic[1], ==, 0xffd8);
    g_assert_cmphex(pic[n - 2] << 8 | pic[n - 1], ==, 0xffd9);
    n = picture(&h, pic, 1 << 20, &pts1, &fid1);
    g_assert_cmphex(pic[0] << 8 | pic[1], ==, 0xffd8);
    g_assert_cmphex(pic[n - 2] << 8 | pic[n - 1], ==, 0xffd9);
    g_assert_cmpuint(pts1 - pts0, ==, INTERVAL * 3);
    g_assert_cmpint(fid0, !=, fid1);

    /* YUYV 640x480: the binary cells give the frame number the PTS does */
    stream_set(&h, 1, 2, 1);
    stream_set(&h, 2, 2, 1);
    for (int k = 0; k < 2; k++) {
        unsigned cell = 12, got = 0;

        n = picture(&h, pic, 1 << 20, &pts1, &fid1);
        g_assert_cmpuint(n, ==, 640 * 480 * 2);
        for (unsigned i = 0; i < 16; i++) {
            unsigned x = cell + i * cell + cell / 2, y = cell + cell / 2;

            got = (got << 1) | (pic[(y * 640 + x) * 2] > 128);
        }
        g_assert_cmpuint(got, ==, pts1 / (INTERVAL * 3));
        g_assert_cmpuint(got, >, 1);
    }

    qtest_quit(h.qts);
    g_free(cfg);
    g_free(pic);
}

static void test_camera(void)
{
    run_camera("", "");
}

/* the scene of an image sensor in place of the pattern */
static void test_camera_sensor(void)
{
    run_camera("-device image-sensor,model=sc2336p,bus=i2c0,id=sen",
               ",sensor=sen");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/uvc-camera", test_camera);
    qtest_add_func("ingenic/uvc-camera-sensor", test_camera_sensor);
    return g_test_run();
}
