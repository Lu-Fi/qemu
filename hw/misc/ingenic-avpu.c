/*
 * Ingenic AVPU: the T31's Allegro video encoder
 *
 * libimp runs Allegro's encoder control software in user space and reaches
 * the AVPU through the avpu driver's register ioctls. The registers sit at
 * 0x8000: the IP's identity, the interrupt mask (0x8014) and the pending
 * interrupts (0x8018, write 1 to clear), and per core (video 0, JPEG 1)
 * at 0x8200 + core * 0x200 a register zone, the command list address
 * (+0x1e0), the start (+0x1e4: 2 runs the command list, 8 its entropy pass,
 * 1 the JPEG zone), a reset (+0x1f0), a clock gate (+0x1f4) and the busy
 * flags (+0x1f8).
 *
 * A command list is a run of 512-byte entries, one per slice, the last
 * flagged in bit 31 of its first word. The T31's libimp asks for the whole
 * encode in the first pass: the entry's first 0x100 bytes say what to
 * encode, and the AVPU writes the slice data into the stream buffer at the
 * entry's offset, right after the slice header libimp has put there, then
 * fills the entry's status words (0x40-0x5f) and raises interrupt
 * core * 4. The layout is libimp's (its CmdRegsEnc1ToSliceParam and
 * EncodingStatusRegsToSliceStatus) checked against a real T31.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "block/thread-pool.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-avpu.h"

#define AVPU_UID            0x8000
#define AVPU_HW_ID          0x8004
#define AVPU_DATE           0x8008
#define AVPU_REV            0x800c
#define AVPU_AXI            0x8010
#define AVPU_INT_MASK       0x8014
#define AVPU_INT            0x8018
#define AVPU_TOP_CTRL       0x8054
#define AVPU_CORE(c)        (0x8200 + (c) * 0x200)
#define CORE_CL_ADDR        0x1e0
#define CORE_START          0x1e4
#define CORE_RESET          0x1f0
#define CORE_CLOCK          0x1f4
#define CORE_BUSY           0x1f8

#define START_ENC1          2
#define START_ENC2          8
#define START_JPEG          1

#define CL_ENTRY            0x200
#define CL_LAST             BIT(31)
#define CL_STATUS           0x40    /* word index of the status block */
#define CL_STATUS_WORDS     0x20

#define R(off)              (s->regs[((off) - INGENIC_AVPU_REGS) / 4])

/* A video core's encoder state, by the channel buffer in entry word 0x23 */
typedef struct AvpuChan {
    IngenicAvpuH264 *h264;
} AvpuChan;

struct IngenicAvpuJob {
    IngenicAvpuState *s;        /* NULL once the device has reset */
    int core;
    uint32_t cl;
    uint32_t w[CL_ENTRY / 4];
    IngenicAvpuPic pic;
    uint32_t chan;
    IngenicAvpuH264 *h264;      /* the channel's, while this encodes */
    uint8_t *y, *uv;
    uint32_t stride;
    GByteArray *data;
    IngenicAvpuStats st;
    bool ok;
};

static void avpu_update(IngenicAvpuState *s)
{
    qemu_set_irq(s->irq, !!(R(AVPU_INT) & R(AVPU_INT_MASK)));
}

static void avpu_chan_free(gpointer p)
{
    AvpuChan *ch = p;

    ingenic_avpu_h264_free(ch->h264);
    g_free(ch);
}

static void avpu_job_free(IngenicAvpuJob *j)
{
    ingenic_avpu_h264_free(j->h264);
    g_free(j->y);
    g_free(j->uv);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

static int32_t sext(uint32_t v, int bits)
{
    return (int32_t)(v << (32 - bits)) >> (32 - bits);
}

/* The settings of an encode, from an entry's command words */
static void avpu_decode(const uint32_t *w, IngenicAvpuPic *pic)
{
    pic->width = (extract32(w[1], 0, 11) + 1) * 8;
    pic->height = (extract32(w[1], 12, 11) + 1) * 8;
    pic->transform_8x8 = extract32(w[0], 10, 1);
    pic->cabac = extract32(w[2], 10, 1);
    pic->intra = extract32(w[3], 28, 2) == 2;
    pic->qp = extract32(w[3], 16, 8);
    pic->cabac_init_idc = extract32(w[3], 24, 1);
    pic->chroma_qp_offset = sext(extract32(w[3], 0, 5), 5);
    pic->beta_div2 = sext(extract32(w[4], 0, 5), 5);
    pic->alpha_c0_div2 = sext(extract32(w[4], 8, 6), 6);
}

static int avpu_work(void *opaque)
{
    IngenicAvpuJob *j = opaque;

    j->ok = ingenic_avpu_h264_encode(j->h264, &j->pic, j->y, j->uv,
                                     j->stride, j->data, &j->st);
    return 0;
}

/* The status words of an entry, as the T31 AVPU leaves them */
static void avpu_status(IngenicAvpuJob *j, uint32_t *st)
{
    uint32_t mbs = DIV_ROUND_UP(j->pic.width, 16) *
                   DIV_ROUND_UP(j->pic.height, 16);
    uint32_t len = j->data->len;
    uint32_t avail = j->w[0x33];

    memset(st, 0, CL_STATUS_WORDS * 4);
    if (j->pic.intra) {
        j->st.intra_mbs = mbs;
        j->st.inter_mbs = j->st.skip_mbs = 0;
    }
    st[0x41 - CL_STATUS] = MIN(len, avail) | (len > avail ? BIT(31) : 0);
    st[0x42 - CL_STATUS] = len * 8;                 /* CABAC bins */
    st[0x45 - CL_STATUS] = j->st.intra_mbs * 4;     /* intra 8x8 blocks */
    st[0x47 - CL_STATUS] = j->st.skip_mbs * 4;      /* skipped 8x8 blocks */
    st[0x49 - CL_STATUS] = mbs;                     /* 16x16 blocks */
    st[0x4d - CL_STATUS] = mbs * 4 * j->pic.qp;     /* QP over 8x8 blocks */
    st[0x4e - CL_STATUS] = j->pic.qp << 8 | j->pic.qp;  /* max, min QP */
}

static void avpu_done(void *opaque, int ret)
{
    IngenicAvpuJob *j = opaque;
    IngenicAvpuState *s = j->s;
    uint32_t st[CL_STATUS_WORDS];
    AvpuChan *ch;

    if (!s) {
        avpu_job_free(j);
        return;
    }
    ch = g_hash_table_lookup(s->chans, GUINT_TO_POINTER(j->chan));
    if (ch && !ch->h264) {
        ch->h264 = g_steal_pointer(&j->h264);
    }
    if (!j->ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: encode failed\n");
        g_byte_array_set_size(j->data, 0);
    }
    address_space_write(&address_space_memory, j->w[0x30] + j->w[0x32],
                        MEMTXATTRS_UNSPECIFIED, j->data->data,
                        MIN(j->data->len, j->w[0x33]));
    avpu_status(j, st);
    for (int i = 0; i < CL_STATUS_WORDS; i++) {
        st[i] = cpu_to_le32(st[i]);
    }
    address_space_write(&address_space_memory, j->cl + CL_STATUS * 4,
                        MEMTXATTRS_UNSPECIFIED, st, sizeof(st));
    s->job[j->core] = NULL;
    R(AVPU_INT) |= BIT(j->core * 4);
    avpu_update(s);
    avpu_job_free(j);
}

static void avpu_enc1(IngenicAvpuState *s, int core)
{
    IngenicAvpuJob *j = g_new0(IngenicAvpuJob, 1);
    AvpuChan *ch;
    size_t ylen, uvlen;

    j->s = s;
    j->core = core;
    j->cl = R(AVPU_CORE(core) + CORE_CL_ADDR);
    j->data = g_byte_array_new();
    address_space_read(&address_space_memory, j->cl, MEMTXATTRS_UNSPECIFIED,
                       j->w, sizeof(j->w));
    for (int i = 0; i < ARRAY_SIZE(j->w); i++) {
        j->w[i] = le32_to_cpu(j->w[i]);
    }
    avpu_decode(j->w, &j->pic);
    s->job[core] = j;
    if (!(j->w[0] & CL_LAST) || extract32(j->w[0], 28, 2) != 0) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: only single-slice H.264 "
                      "command lists are modelled (word 0 0x%08x)\n", j->w[0]);
        avpu_done(j, 0);
        return;
    }

    j->chan = j->w[0x23];
    ch = g_hash_table_lookup(s->chans, GUINT_TO_POINTER(j->chan));
    if (!ch) {
        ch = g_new0(AvpuChan, 1);
        g_hash_table_insert(s->chans, GUINT_TO_POINTER(j->chan), ch);
    }
    j->h264 = g_steal_pointer(&ch->h264);
    if (j->h264 && !ingenic_avpu_h264_matches(j->h264, &j->pic)) {
        ingenic_avpu_h264_free(j->h264);
        j->h264 = NULL;
    }
    if (!j->h264) {
        j->h264 = ingenic_avpu_h264_new(&j->pic);
    }
    if (!j->h264) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: no H.264 encoder (x264)\n");
        avpu_done(j, 0);
        return;
    }

    j->stride = j->w[0x22];
    ylen = (size_t)j->stride * j->pic.height;
    uvlen = (size_t)j->stride * j->pic.height / 2;
    j->y = g_malloc(ylen);
    j->uv = g_malloc(uvlen);
    address_space_read(&address_space_memory, j->w[0x20],
                       MEMTXATTRS_UNSPECIFIED, j->y, ylen);
    address_space_read(&address_space_memory, j->w[0x21],
                       MEMTXATTRS_UNSPECIFIED, j->uv, uvlen);
    thread_pool_submit_aio(avpu_work, j, avpu_done, j);
}

static void avpu_start(IngenicAvpuState *s, int core, uint32_t v)
{
    if (s->job[core]) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: core %d started while "
                      "busy\n", core);
        return;
    }
    if (core == 0 && v == START_ENC1) {
        avpu_enc1(s, core);
        return;
    }
    qemu_log_mask(LOG_UNIMP, "ingenic-avpu: core %d start 0x%x not modelled\n",
                  core, v);
}

static uint64_t ingenic_avpu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicAvpuState *s = INGENIC_AVPU(opaque);
    int core = (off - AVPU_CORE(0)) / 0x200;

    if (off < INGENIC_AVPU_REGS || off >= INGENIC_AVPU_REGS + 0x1000) {
        qemu_log_mask(LOG_UNIMP, "%s: 0x%05" HWADDR_PRIx "\n", __func__, off);
        return 0;
    }
    switch (off) {
    case AVPU_UID:
        return 0x30ab6e51;
    case AVPU_HW_ID:
        return 0x72000460;
    case AVPU_DATE:
        return 0x20190326;
    case AVPU_REV:
        return 0x5de5a975;
    case AVPU_AXI:
    case AVPU_TOP_CTRL:
        return 0;
    }
    if (off >= AVPU_CORE(0) && core < INGENIC_AVPU_CORES) {
        switch (off - AVPU_CORE(core)) {
        case CORE_START:
        case CORE_RESET:
            return 0;
        case CORE_CLOCK:
            return s->clock[core] ? 0x00010001 : 0;
        case CORE_BUSY:
            return s->job[core] ? 3 : 0;
        }
    }
    return R(off);
}

static void ingenic_avpu_write(void *opaque, hwaddr off, uint64_t value,
                               unsigned size)
{
    IngenicAvpuState *s = INGENIC_AVPU(opaque);
    int core = (off - AVPU_CORE(0)) / 0x200;
    uint32_t v = value;

    if (off < INGENIC_AVPU_REGS || off >= INGENIC_AVPU_REGS + 0x1000) {
        qemu_log_mask(LOG_UNIMP, "%s: 0x%05" HWADDR_PRIx " = 0x%08x\n",
                      __func__, off, v);
        return;
    }
    switch (off) {
    case AVPU_UID:
    case AVPU_HW_ID:
    case AVPU_DATE:
    case AVPU_REV:
        return;
    case AVPU_INT:
        R(AVPU_INT) &= ~v;
        avpu_update(s);
        return;
    case AVPU_INT_MASK:
        R(AVPU_INT_MASK) = v;
        avpu_update(s);
        return;
    }
    if (off >= AVPU_CORE(0) && core < INGENIC_AVPU_CORES) {
        switch (off - AVPU_CORE(core)) {
        case CORE_START:
            avpu_start(s, core, v);
            return;
        case CORE_RESET:
        case CORE_BUSY:
            return;
        case CORE_CLOCK:
            s->clock[core] = v & 1;
            return;
        }
    }
    R(off) = v;
}

static const MemoryRegionOps ingenic_avpu_ops = {
    .read = ingenic_avpu_read,
    .write = ingenic_avpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_avpu_reset_hold(Object *obj, ResetType type)
{
    IngenicAvpuState *s = INGENIC_AVPU(obj);

    for (int c = 0; c < INGENIC_AVPU_CORES; c++) {
        if (s->job[c]) {
            s->job[c]->s = NULL;    /* its completion just frees it */
            s->job[c] = NULL;
        }
        s->clock[c] = false;
    }
    memset(s->regs, 0, sizeof(s->regs));
    g_hash_table_remove_all(s->chans);
}

static void ingenic_avpu_reset_exit(Object *obj, ResetType type)
{
    avpu_update(INGENIC_AVPU(obj));
}

static void ingenic_avpu_init(Object *obj)
{
    IngenicAvpuState *s = INGENIC_AVPU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_avpu_ops, s,
                          TYPE_INGENIC_AVPU, INGENIC_AVPU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->chans = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL,
                                     avpu_chan_free);
}

static void ingenic_avpu_finalize(Object *obj)
{
    g_hash_table_destroy(INGENIC_AVPU(obj)->chans);
}

/* An encode in flight and the encoders' reference frames are host state */
static const VMStateDescription vmstate_ingenic_avpu = {
    .name = TYPE_INGENIC_AVPU,
    .unmigratable = 1,
};

static void ingenic_avpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic AVPU video encoder";
    dc->vmsd = &vmstate_ingenic_avpu;
    rc->phases.hold = ingenic_avpu_reset_hold;
    rc->phases.exit = ingenic_avpu_reset_exit;
}

static const TypeInfo ingenic_avpu_type_info = {
    .name = TYPE_INGENIC_AVPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAvpuState),
    .instance_init = ingenic_avpu_init,
    .instance_finalize = ingenic_avpu_finalize,
    .class_init = ingenic_avpu_class_init,
};

static void ingenic_avpu_register_types(void)
{
    type_register_static(&ingenic_avpu_type_info);
}

type_init(ingenic_avpu_register_types)
