/*
 * mmiotrace - ordered MMIO access trace for Ingenic bootrom verification
 *
 * Logs every guest memory access whose *physical* address falls in a
 * configurable window (default 0x10000000..0x14000000, the XBurst
 * peripheral region = KSEG1 0xB0000000..0xB3FFFFFF) as an ordered trace
 * of (op, size, phys_addr, value). Consecutive identical accesses (same
 * pc/op/size/addr/value) are collapsed into a single line with a repeat
 * count, so poll loops do not flood the trace.
 *
 * The intent is differential verification: run the real mask ROM and a
 * reconstructed bootrom image through the same machine, then diff the two
 * traces. The diff key (op/size/addr/value) is independent of the code's
 * link address, so reconstructions with different PCs compare cleanly.
 * PC is emitted as a trailing field for debugging only.
 *
 * Args:
 *   out=FILE      output file (default: stderr)
 *   lo=0xADDR     low  phys bound (inclusive, default 0x10000000)
 *   hi=0xADDR     high phys bound (exclusive, default 0x14000000)
 *   collapse=on   collapse consecutive identical accesses (default on)
 *   pc=on         include pc field in the diff key (default off)
 *
 * License: GNU GPL, version 2 or later.
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static FILE *out;
static uint64_t lo = 0x10000000;
static uint64_t hi = 0x14000000;
static bool collapse = true;
static bool key_pc;
static bool io_only = true;    /* only IO regions (excludes TCSM/SRAM/SDRAM) */
static uint64_t max_records;   /* 0 = unlimited */

#define MAX_EXCL 8
static uint64_t ex_lo[MAX_EXCL], ex_hi[MAX_EXCL];
static int n_excl;

static GMutex lock;
static uint64_t seq;
static uint64_t emitted;
static bool capped;

/* last emitted record, for consecutive-duplicate collapse */
static bool have_last;
static uint64_t last_pc, last_addr, last_val;
static unsigned last_op, last_sz;
static const char *last_dev;
static uint64_t last_count;
static uint64_t last_seq;

static void flush_last(void)
{
    if (have_last) {
        fprintf(out, "%-8"PRIu64" %s %u 0x%08"PRIx64" 0x%08"PRIx64" %-14s",
                last_seq, last_op ? "W" : "R", last_sz << 3,
                last_addr, last_val, last_dev ? last_dev : "?");
        if (last_count > 1) {
            fprintf(out, " x%"PRIu64, last_count);
        }
        if (key_pc) {
            fprintf(out, " @0x%08"PRIx64, last_pc);
        }
        fputc('\n', out);
        have_last = false;
        if (max_records && ++emitted >= max_records && !capped) {
            capped = true;
            fprintf(out, "# ... capped at %"PRIu64" records\n", max_records);
            fflush(out);
        }
    }
}

static void vcpu_mem(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                     uint64_t vaddr, void *udata)
{
    if (capped) {
        return;
    }
    struct qemu_plugin_hwaddr *h = qemu_plugin_get_hwaddr(info, vaddr);
    if (!h) {
        return;
    }
    if (io_only && !qemu_plugin_hwaddr_is_io(h)) {
        return;
    }
    uint64_t pa = qemu_plugin_hwaddr_phys_addr(h);
    if (pa < lo || pa >= hi) {
        return;
    }
    for (int i = 0; i < n_excl; i++) {
        if (pa >= ex_lo[i] && pa < ex_hi[i]) {
            return;
        }
    }

    unsigned op = qemu_plugin_mem_is_store(info) ? 1 : 0;
    unsigned sz = 1u << qemu_plugin_mem_size_shift(info);   /* bytes */
    qemu_plugin_mem_value mv = qemu_plugin_mem_get_value(info);
    uint64_t val;
    switch (mv.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:  val = mv.data.u8;  break;
    case QEMU_PLUGIN_MEM_VALUE_U16: val = mv.data.u16; break;
    case QEMU_PLUGIN_MEM_VALUE_U32: val = mv.data.u32; break;
    case QEMU_PLUGIN_MEM_VALUE_U64: val = mv.data.u64; break;
    default:                        val = mv.data.u128.low; break;
    }
    const char *dev = qemu_plugin_hwaddr_device_name(h);
    uint64_t pc = (uint64_t)(uintptr_t)udata;

    g_mutex_lock(&lock);
    uint64_t s = seq++;
    if (collapse && have_last &&
        last_op == op && last_sz == sz && last_addr == pa &&
        last_val == val && (!key_pc || last_pc == pc)) {
        last_count++;
        g_mutex_unlock(&lock);
        return;
    }
    flush_last();
    have_last = true;
    last_seq = s; last_pc = pc; last_addr = pa; last_val = val;
    last_op = op; last_sz = sz; last_dev = dev; last_count = 1;
    g_mutex_unlock(&lock);
}

static void vcpu_tb_trans(struct qemu_plugin_tb *tb, void *userdata)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t vaddr = qemu_plugin_insn_vaddr(insn);
        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_RW,
                                         (void *)(uintptr_t)vaddr);
    }
}

static void plugin_exit(void *p)
{
    g_mutex_lock(&lock);
    flush_last();
    fflush(out);
    if (out != stderr) {
        fclose(out);
    }
    g_mutex_unlock(&lock);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info, int argc,
                                           char **argv)
{
    out = stderr;
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) t = g_strsplit(argv[i], "=", 2);
        if (g_strcmp0(t[0], "out") == 0) {
            out = fopen(t[1], "w");
            if (!out) {
                fprintf(stderr, "mmiotrace: cannot open %s\n", t[1]);
                return -1;
            }
        } else if (g_strcmp0(t[0], "lo") == 0) {
            lo = g_ascii_strtoull(t[1], NULL, 0);
        } else if (g_strcmp0(t[0], "hi") == 0) {
            hi = g_ascii_strtoull(t[1], NULL, 0);
        } else if (g_strcmp0(t[0], "collapse") == 0) {
            qemu_plugin_bool_parse(t[0], t[1], &collapse);
        } else if (g_strcmp0(t[0], "pc") == 0) {
            qemu_plugin_bool_parse(t[0], t[1], &key_pc);
        } else if (g_strcmp0(t[0], "max") == 0) {
            max_records = g_ascii_strtoull(t[1], NULL, 0);
        } else if (g_strcmp0(t[0], "io") == 0) {
            qemu_plugin_bool_parse(t[0], t[1], &io_only);
        } else if (g_strcmp0(t[0], "excl") == 0) {
            g_auto(GStrv) r = g_strsplit(t[1], ":", 2);
            if (r[0] && r[1] && n_excl < MAX_EXCL) {
                ex_lo[n_excl] = g_ascii_strtoull(r[0], NULL, 0);
                ex_hi[n_excl] = g_ascii_strtoull(r[1], NULL, 0);
                n_excl++;
            }
        } else {
            fprintf(stderr, "mmiotrace: bad option %s\n", argv[i]);
            return -1;
        }
    }
    fprintf(out, "# seq op bits addr value dev [xcount] [@pc]\n");
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
