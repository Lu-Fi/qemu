/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * FPU NaN handling of the Ingenic XBurst cores, as measured on a T41
 * (run with -cpu XBurst2) and a T31 (run with -cpu XBurstR2). XBurst2 uses
 * the IEEE 754-2008 rules with Status.FR set and the legacy ones, with
 * quirks, with FR clear; XBurst1 is legacy whatever its FCSR says.
 *
 * Built for FR=0, which XBurst1 is limited to. XBurst2 then switches to
 * FR=1 only: after a switch back its FCSR NaN bits keep reading 1.
 */
#include <stdint.h>
#include <stdio.h>
#include <sys/prctl.h>

#ifndef PR_SET_FP_MODE
#define PR_SET_FP_MODE 45
#define PR_FP_MODE_FR 1
#endif

#define FCSR_NAN_BITS 0x000c0000
/* Cause set beforehand, so that instructions which do not write it show */
#define CAUSE_PRESET  0x0001f000

static uint64_t op_a, op_b, op_c, op_r;
static uint32_t fcsr_in, fcsr_out;

/* Run insn with $f2 = a, $f4 = b, $f6 = c and FCSR = fcsr_in; $f0 = r */
#define DEF(fn, ld, st, insn)                                           \
static void fn(void)                                                    \
{                                                                       \
    asm volatile(".set push\n\t.set noreorder\n\t"                      \
                 ld " $f2, 0(%0)\n\t"                                   \
                 ld " $f4, 0(%1)\n\t"                                   \
                 ld " $f6, 0(%2)\n\t"                                   \
                 "ctc1 %4, $31\n\t"                                     \
                 insn "\n\t"                                            \
                 "cfc1 $8, $31\n\t"                                     \
                 st " $f0, 0(%3)\n\t"                                   \
                 "sw $8, 0(%5)\n\t"                                     \
                 ".set pop"                                             \
                 : : "r"(&op_a), "r"(&op_b), "r"(&op_c), "r"(&op_r),    \
                     "r"(fcsr_in), "r"(&fcsr_out)                       \
                 : "$8", "$f0", "$f1", "$f2", "$f3", "$f4", "$f5", "$f6", \
                   "$f7", "memory");                                    \
}

DEF(div_s, "lwc1", "swc1", "div.s $f0, $f2, $f4")
DEF(add_s, "lwc1", "swc1", "add.s $f0, $f2, $f4")
DEF(neg_s, "lwc1", "swc1", "neg.s $f0, $f2")
DEF(abs_s, "lwc1", "swc1", "abs.s $f0, $f2")
DEF(mov_s, "lwc1", "swc1", "mov.s $f0, $f2")
DEF(movz_s, "lwc1", "swc1", "movz.s $f0, $f2, $0")
DEF(rsqrt_s, "lwc1", "swc1", "rsqrt.s $f0, $f2")
DEF(nmadd_s, "lwc1", "swc1", "nmadd.s $f0, $f6, $f2, $f4")
DEF(cvt_s_d, "ldc1", "swc1", "cvt.s.d $f0, $f2")
DEF(cvt_w_s, "lwc1", "swc1", "cvt.w.s $f0, $f2")
DEF(cvt_l_s, "lwc1", "sdc1", "cvt.l.s $f0, $f2")

enum { XB2_FR1, XB2_FR0, XB1, MODES };

static const char *const mode_name[MODES] = {
    "XBurst2 FR=1", "XBurst2 FR=0", "XBurst1"
};

struct check {
    const char *name;
    void (*fn)(void);
    uint32_t fs;
    uint64_t a, b, c;
    uint64_t r[MODES];      /* result */
    uint32_t f[MODES];      /* FCSR afterwards, without the NaN bits */
};

static const struct check checks[] = {
    { "div.s 0/0", div_s, 0, 0, 0, 0,
      { 0x7fc00000, 0x7fbfffff, 0x7fbfffff },
      { 0x00010040, 0x00010040, 0x00010040 } },
    { "add.s 7fa00000+1", add_s, 0, 0x7fa00000, 0x3f800000, 0,
      { 0x7fe00000, 0x7fa00000, 0x7fa00000 },
      { 0x00010040, 0x00000000, 0x00000000 } },
    { "add.s 7fc00000+1", add_s, 0, 0x7fc00000, 0x3f800000, 0,
      { 0x7fc00000, 0x7fbfffff, 0x7fbfffff },
      { 0x00000000, 0x00010040, 0x00010040 } },
    { "neg.s 7fa00000", neg_s, 0, 0x7fa00000, 0, 0,
      { 0xffa00000, 0x7fa00000, 0x7fbfffff },
      { 0x0001f000, 0x00000000, 0x00010040 } },
    { "abs.s ffc00000", abs_s, 0, 0xffc00000, 0, 0,
      { 0x7fc00000, 0x7fbfffff, 0x7fbfffff },
      { 0x0001f000, 0x00010040, 0x00010040 } },
    { "abs.s 7fa00000", abs_s, 0, 0x7fa00000, 0, 0,
      { 0x7fa00000, 0x7fa00000, 0x7fbfffff },
      { 0x0001f000, 0x00000000, 0x00010040 } },
    { "abs.s FS denormal", abs_s, 1, 0x00000001, 0, 0,
      { 0x00000001, 0x00000000, 0x00000000 },
      { 0x0101f000, 0x01001004, 0x01001004 } },
    { "mov.s", mov_s, 0, 0x3f800000, 0, 0,
      { 0x3f800000, 0x3f800000, 0x3f800000 },
      { 0x0001f000, 0x0001f07c, 0x0001f000 } },
    { "movz.s", movz_s, 0, 0x3f800000, 0, 0,
      { 0x3f800000, 0x3f800000, 0x3f800000 },
      { 0x0001f000, 0x00000000, 0x0001f000 } },
    { "rsqrt.s 807fffff", rsqrt_s, 0, 0x807fffff, 0, 0,
      { 0x7fc00000, 0x7fbfffff, 0x7fbfffff },
      { 0x00010040, 0x00000040, 0x00010040 } },
    { "nmadd.s 0*0+7fa00000", nmadd_s, 0, 0, 0, 0x7fa00000,
      { 0xffe00000, 0x7fa00000, 0x7fbfffff },
      { 0x00010040, 0x00000000, 0x00010040 } },
    { "cvt.s.d 7ff0000012345678", cvt_s_d, 0, 0x7ff0000012345678ULL, 0, 0,
      { 0x7fc00000, 0x7fbfffff, 0x7f800000 },
      { 0x00010040, 0x00000000, 0x00000000 } },
    { "cvt.w.s 7fc00000", cvt_w_s, 0, 0x7fc00000, 0, 0,
      { 0x00000000, 0x7fffffff, 0x7fffffff },
      { 0x00010040, 0x00010040, 0x00010040 } },
    { "cvt.l.s 1.5", cvt_l_s, 0, 0x3fc00000, 0, 0,
      { 2, 2, 2 },
      { 0x00001004, 0x00001004, 0x00001004 } },
};

static uint32_t read_fcsr(void)
{
    uint32_t v;

    asm volatile("cfc1 %0, $31" : "=r"(v));
    return v;
}

static int run(int mode, uint32_t nan_bits)
{
    int fails = 0;
    uint32_t bits = read_fcsr() & FCSR_NAN_BITS;

    if (bits != nan_bits) {
        printf("%s: FCSR NaN bits %08x, expected %08x\n",
               mode_name[mode], bits, nan_bits);
        fails++;
    }
    for (unsigned i = 0; i < sizeof(checks) / sizeof(checks[0]); i++) {
        const struct check *c = &checks[i];
        uint64_t r;
        uint32_t f;

        op_a = c->a;
        op_b = c->b;
        op_c = c->c;
        op_r = 0;
        fcsr_in = CAUSE_PRESET | (c->fs << 24);
        c->fn();
        r = c->fn == cvt_l_s ? op_r : (uint32_t)op_r;
        f = fcsr_out & ~FCSR_NAN_BITS;
        if (r != c->r[mode] || f != c->f[mode]) {
            printf("%s: %s gave %llx fcsr %08x, expected %llx fcsr %08x\n",
                   mode_name[mode], c->name, (unsigned long long)r, f,
                   (unsigned long long)c->r[mode], c->f[mode]);
            fails++;
        }
    }
    return fails;
}

int main(void)
{
    uint32_t fir;
    int fails;

    asm volatile("cfc1 %0, $0" : "=r"(fir));
    if (fir == 0x00f32100) {
        fails = run(XB2_FR0, 0);
        if (prctl(PR_SET_FP_MODE, PR_FP_MODE_FR)) {
            printf("cannot set FR=1\n");
            return 1;
        }
        fails += run(XB2_FR1, FCSR_NAN_BITS);
    } else if ((fir & 0x0080ff00) == 0x00800000) {
        /* XBurst1: Has2008 and FPU ID 0, but legacy NaNs */
        fails = run(XB1, FCSR_NAN_BITS);
    } else {
        printf("FIR %08x is not an XBurst FPU\n", fir);
        return 1;
    }
    return fails ? 1 : 0;
}
