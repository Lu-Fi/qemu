/*
 * coherency - check the cache maintenance a non-coherent MIPS core needs
 *
 * The Ingenic XBurst L1 caches are write-back and see neither device DMA
 * nor, for the I-cache, stores; the L2 sits on the DMA side of them. This
 * plugin follows what each L1 line may hold and reports where the guest
 * relies on coherence the silicon does not have:
 *
 *   dma-read-dirty   a device reads memory the D-cache holds dirty
 *   dma-write-dirty  a device writes memory the D-cache holds dirty, whose
 *                    writeback will clobber the device's data
 *   uc-read-dirty    an uncached load of memory the D-cache holds dirty
 *   uc-write-dirty   an uncached store to memory the D-cache holds dirty
 *   stale-read       a cached load from a line the D-cache may hold after
 *                    its memory changed behind it (DMA or uncached store)
 *   stale-write      a cached store to such a line
 *   stale-code       code runs on a vCPU whose I-cache may hold an older
 *                    copy: it changed after that vCPU fetched it, with no
 *                    I-cache invalidate since
 *   dirty-code       code runs while its new bytes are still dirty in the
 *                    D-cache (instruction fetches read memory)
 *
 * A line counts as cached from its first cached access until a cache
 * instruction invalidates it, so a report means the maintenance is missing.
 * Each vCPU also runs an LRU model of its L1s; a report whose line that
 * model has already evicted is marked "latent": the silicon most likely
 * got away with it, until the code or its timing changes. Mapped addresses
 * count as cached. The D-cache is taken as shared by the vCPUs and the
 * I-caches as private (up to two vCPUs).
 *
 * Needs the cache op and DMA callbacks of the thingino QEMU fork.
 *
 * Options:
 *   ram=BYTES   guest RAM to follow, from physical 0 (default 256M)
 *   l1=BYTES    size of each L1 (default 32K; 16K on the T21 and T23)
 *   ways=N      L1 ways (default 8)
 *   line=N      L1 line size (default 32)
 *   max=N       reports printed per kind, real and latent apart
 *               (default 20)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

enum kind {
    DMA_READ_DIRTY,
    DMA_WRITE_DIRTY,
    UC_READ_DIRTY,
    UC_WRITE_DIRTY,
    STALE_READ,
    STALE_WRITE,
    STALE_CODE,
    DIRTY_CODE,
    N_KINDS,
};

static const char *const kind_name[N_KINDS] = {
    "dma-read-dirty", "dma-write-dirty", "uc-read-dirty", "uc-write-dirty",
    "stale-read", "stale-write", "stale-code", "dirty-code",
};

#define MAX_CPUS    2
#define D_RES       0x01        /* the D-cache may hold the line */
#define D_DIRTY     0x02        /* ... dirty */
#define D_STALE     0x04        /* its memory changed behind the D-cache */
#define I_RES(c)    (0x10 << (c))   /* vCPU c's I-cache may hold it */
#define I_STALE(c)  (0x40 << (c))   /* ... and the code changed since */
#define D_BITS      (D_RES | D_DIRTY | D_STALE)

#define DEVICE      0xffffffffu     /* last_pc of a change made by DMA */

/* MIPS CACHE op field: cache in bits 1:0, operation in bits 4:2 */
#define OP_CACHE(op)    ((op) & 3)
#define OP_KIND(op)     ((op) >> 2)

static uint64_t ram_size = 256 << 20;
static uint64_t l1_size = 32 << 10;
static unsigned line_shift = 5;
static uint32_t n_sets = 128, n_ways = 8;
static unsigned max_reports = 20;
static unsigned n_cpus = 1;

/* Per line of guest RAM */
static uint8_t *state;
static uint32_t *last_pc;   /* the last cached store, or the stale change */
static uint32_t *d_epoch;   /* d_ops[set] when the D-cache took the line */
static uint32_t *i_epoch[MAX_CPUS];
/* Index cache ops done per set: a set that saw n_ways of them is clean */
static uint32_t *d_ops;
static uint32_t *i_ops[MAX_CPUS];

/*
 * The LRU model of one vCPU's L1: per set, the line + 1 held by each way
 * (0: empty), when it was last used, which ways are dirty, and the index
 * ops done on it since it was last emptied.
 */
#define MAX_WAYS    16
typedef struct {
    uint32_t tag[MAX_WAYS];
    uint32_t used[MAX_WAYS];
    uint32_t clock;
    uint16_t dirty;
    uint16_t index_ops;
} LruSet;

static LruSet *lru_d[MAX_CPUS], *lru_i[MAX_CPUS];

static GMutex lock;
static GHashTable *seen;    /* (kind << 33 | latent << 32 | pc) reported */
static uint64_t counts[N_KINDS][2];     /* [latent] */
static unsigned printed[N_KINDS][2];

static uint64_t n_lines(void)
{
    return ram_size >> line_shift;
}

static void report(enum kind k, uint64_t ln, uint32_t pc, uint32_t from,
                   int cpu, bool latent)
{
    gpointer key = GSIZE_TO_POINTER(((gsize)k << 33) | ((gsize)latent << 32) |
                                    pc);
    g_autofree char *who = NULL;
    g_autoptr(GString) out = NULL;

    g_mutex_lock(&lock);
    counts[k][latent]++;
    if (printed[k][latent] >= max_reports ||
        g_hash_table_contains(seen, key)) {
        g_mutex_unlock(&lock);
        return;
    }
    printed[k][latent]++;
    g_hash_table_add(seen, key);
    g_mutex_unlock(&lock);

    who = from == DEVICE ? g_strdup("a device") :
          from ? g_strdup_printf("pc 0x%08" PRIx32, from) : g_strdup("?");
    out = g_string_new(NULL);
    g_string_printf(out, "coherency: %s pa 0x%08" PRIx64, kind_name[k],
                    ln << line_shift);
    if (cpu >= 0) {
        g_string_append_printf(out, " cpu%d pc 0x%08" PRIx32, cpu, pc);
    }
    g_string_append_printf(out, " (last change: %s)%s\n", who,
                           latent ? " latent" : "");
    qemu_plugin_outs(out->str);
}

static LruSet *lru_set(LruSet *cache, uint64_t ln)
{
    return &cache[ln & (n_sets - 1)];
}

/* The way of @s that holds line @ln, or -1 */
static int lru_find(LruSet *s, uint64_t ln)
{
    for (unsigned w = 0; w < n_ways; w++) {
        if (s->tag[w] == ln + 1) {
            return w;
        }
    }
    return -1;
}

/* A cached access: the line moves in, over an empty or the oldest way */
static void lru_use(LruSet *cache, uint64_t ln, bool store)
{
    LruSet *s = lru_set(cache, ln);
    int w = lru_find(s, ln);

    if (w < 0) {
        uint32_t oldest = 0;

        w = 0;
        for (unsigned i = 0; i < n_ways; i++) {
            if (!s->tag[i]) {
                w = i;
                break;
            }
            if (s->clock - s->used[i] > oldest) {
                oldest = s->clock - s->used[i];
                w = i;
            }
        }
        s->tag[w] = ln + 1;
        s->dirty &= ~(1u << w);
    }
    s->used[w] = ++s->clock;
    if (store) {
        s->dirty |= 1u << w;
    }
}

/* A hit cache op: write the line back, and unless @keep invalidate it */
static void lru_hit(LruSet *cache, uint64_t ln, bool keep)
{
    LruSet *s = lru_set(cache, ln);
    int w = lru_find(s, ln);

    if (w >= 0) {
        s->dirty &= ~(1u << w);
        if (!keep) {
            s->tag[w] = 0;
        }
    }
}

/*
 * An index cache op. The model's ways are not the silicon's, so as in the
 * line states a set counts as emptied once it saw an op per way.
 */
static void lru_index(LruSet *cache, uint64_t vaddr)
{
    LruSet *s = &cache[(vaddr >> line_shift) & (n_sets - 1)];

    if (++s->index_ops >= n_ways) {
        memset(s->tag, 0, sizeof(s->tag));
        s->dirty = 0;
        s->index_ops = 0;
    }
}

static bool lru_holds(LruSet *cache, uint64_t ln, bool dirty)
{
    LruSet *s = lru_set(cache, ln);
    int w = lru_find(s, ln);

    return w >= 0 && (!dirty || (s->dirty >> w) & 1);
}

/* Whether some vCPU's model still holds @ln dirty */
static bool held_dirty(uint64_t ln)
{
    for (unsigned c = 0; c < n_cpus; c++) {
        if (lru_holds(lru_d[c], ln, true)) {
            return true;
        }
    }
    return false;
}

/* The line's state after the index ops done since the caches took it */
static uint8_t settle(uint64_t ln)
{
    uint32_t set = ln & (n_sets - 1);
    uint8_t st = __atomic_load_n(&state[ln], __ATOMIC_RELAXED);
    uint8_t clear = 0;

    if ((st & D_BITS) && d_ops[set] - d_epoch[ln] >= n_ways) {
        clear |= D_BITS;
    }
    for (unsigned c = 0; c < n_cpus; c++) {
        if ((st & (I_RES(c) | I_STALE(c))) &&
            i_ops[c][set] - i_epoch[c][ln] >= n_ways) {
            clear |= I_RES(c) | I_STALE(c);
        }
    }
    if (clear) {
        st = __atomic_and_fetch(&state[ln], ~clear, __ATOMIC_RELAXED);
    }
    return st;
}

static void set_bits(uint64_t ln, uint8_t bits)
{
    __atomic_or_fetch(&state[ln], bits, __ATOMIC_RELAXED);
}

static void clear_bits(uint64_t ln, uint8_t bits)
{
    __atomic_and_fetch(&state[ln], ~bits, __ATOMIC_RELAXED);
}

static void d_take(uint64_t ln)
{
    d_epoch[ln] = d_ops[ln & (n_sets - 1)];
    set_bits(ln, D_RES);
}

/* The bytes of a line changed: any I-cache holding it is now stale */
static void code_changed(uint64_t ln, uint8_t st, uint32_t from)
{
    uint8_t stale = 0;

    for (unsigned c = 0; c < n_cpus; c++) {
        if (st & I_RES(c)) {
            stale |= I_STALE(c);
        }
    }
    if (stale) {
        last_pc[ln] = from;
        set_bits(ln, stale);
    }
}

static void data_access(uint64_t ln, bool store, bool cached, uint32_t pc,
                        int cpu)
{
    uint8_t st = settle(ln);

    if (cached) {
        if (st & D_STALE) {
            report(store ? STALE_WRITE : STALE_READ, ln, pc, last_pc[ln],
                   cpu, !lru_holds(lru_d[cpu], ln, false));
            clear_bits(ln, D_STALE);
        }
        lru_use(lru_d[cpu], ln, store);
        d_take(ln);
        if (store) {
            set_bits(ln, D_DIRTY);
            code_changed(ln, st, pc);
            last_pc[ln] = pc;
        }
        return;
    }
    if (st & D_DIRTY) {
        report(store ? UC_WRITE_DIRTY : UC_READ_DIRTY, ln, pc, last_pc[ln],
               cpu, !held_dirty(ln));
    }
    if (store) {
        if (st & D_RES) {
            last_pc[ln] = pc;
            set_bits(ln, D_STALE);
        }
        code_changed(ln, st, pc);
    }
}

/* Guest physical address of @vaddr, from a vCPU callback */
static bool phys(uint64_t vaddr, uint64_t *pa)
{
    uint32_t va = vaddr;

    if (va >= 0x80000000u && va < 0xc0000000u) {
        *pa = va & 0x1fffffffu;
        return true;
    }
    return qemu_plugin_translate_vaddr(vaddr, pa);
}

static bool cached_va(uint64_t vaddr)
{
    uint32_t va = vaddr;

    return va < 0xa0000000u || va >= 0xc0000000u;
}

static void vcpu_mem(unsigned int cpu, qemu_plugin_meminfo_t info,
                     uint64_t vaddr, void *udata)
{
    struct qemu_plugin_hwaddr *hw = qemu_plugin_get_hwaddr(info, vaddr);
    uint64_t pa, size, ln, end;
    bool store, cached;

    if (cpu >= n_cpus || !hw || qemu_plugin_hwaddr_is_io(hw)) {
        return;
    }
    pa = qemu_plugin_hwaddr_phys_addr(hw);
    size = 1 << qemu_plugin_mem_size_shift(info);
    if (pa + size > ram_size) {
        return;
    }
    store = qemu_plugin_mem_is_store(info);
    cached = cached_va(vaddr);
    end = (pa + size - 1) >> line_shift;
    for (ln = pa >> line_shift; ln <= end; ln++) {
        data_access(ln, store, cached, (uintptr_t)udata, cpu);
    }
}

static void vcpu_tb_exec(unsigned int cpu, void *udata)
{
    uint64_t packed = (uintptr_t)udata;
    uint32_t va = packed, len = packed >> 32;
    bool cached = cached_va(va);
    uint32_t first = va >> line_shift;
    uint32_t last = (va + len - 1) >> line_shift;

    if (cpu >= n_cpus) {
        return;
    }
    for (uint32_t l = first; l <= last; l++) {
        uint32_t lva = l << line_shift;
        uint64_t pa, ln;
        uint8_t st;

        if (!phys(lva, &pa) || pa >= ram_size) {
            continue;
        }
        ln = pa >> line_shift;
        st = settle(ln);
        if (st & D_DIRTY) {
            report(DIRTY_CODE, ln, MAX(va, lva), last_pc[ln], cpu,
                   !held_dirty(ln));
        }
        if (!cached) {
            continue;
        }
        if (st & I_STALE(cpu)) {
            report(STALE_CODE, ln, MAX(va, lva), last_pc[ln], cpu,
                   !lru_holds(lru_i[cpu], ln, false));
            clear_bits(ln, I_STALE(cpu));
        }
        lru_use(lru_i[cpu], ln, false);
        i_epoch[cpu][ln] = i_ops[cpu][ln & (n_sets - 1)];
        set_bits(ln, I_RES(cpu));
    }
}

static void vcpu_tb_trans(struct qemu_plugin_tb *tb, void *udata)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    uint64_t va = qemu_plugin_tb_vaddr(tb);
    uint64_t len = 0;

    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t iva = qemu_plugin_insn_vaddr(insn);

        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_RW,
                                         (void *)(uintptr_t)(uint32_t)iva);
        len = iva + qemu_plugin_insn_size(insn) - va;
    }
    qemu_plugin_register_vcpu_tb_exec_cb(tb, vcpu_tb_exec,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         (void *)(uintptr_t)
                                         ((uint32_t)va | (len << 32)));
}

static void dma(uint64_t addr, uint64_t len, bool is_write, void *udata)
{
    uint64_t ln, end;

    if (!len || addr >= ram_size) {
        return;
    }
    end = (MIN(addr + len, ram_size) - 1) >> line_shift;
    for (ln = addr >> line_shift; ln <= end; ln++) {
        uint8_t st = settle(ln);

        if (st & D_DIRTY) {
            report(is_write ? DMA_WRITE_DIRTY : DMA_READ_DIRTY, ln, 0,
                   last_pc[ln], -1, !held_dirty(ln));
        }
        if (is_write) {
            if (st & D_RES) {
                last_pc[ln] = DEVICE;
                set_bits(ln, D_STALE);
            }
            code_changed(ln, st, DEVICE);
        }
    }
}

static void cache_op(unsigned int cpu, uint64_t vaddr, uint32_t op,
                     void *udata)
{
    uint32_t set = (vaddr >> line_shift) & (n_sets - 1);
    uint64_t pa, ln;

    if (cpu >= n_cpus) {
        return;
    }
    if (op == QEMU_PLUGIN_CACHE_OP_SYNCI) {
        /* Writes the D line back and invalidates it in every I-cache */
        if (phys(vaddr, &pa) && pa < ram_size) {
            uint8_t bits = D_DIRTY;

            ln = pa >> line_shift;
            for (unsigned c = 0; c < n_cpus; c++) {
                bits |= I_RES(c) | I_STALE(c);
            }
            clear_bits(ln, bits);
            lru_hit(lru_d[cpu], ln, true);
            lru_hit(lru_i[cpu], ln, false);
        }
        return;
    }
    switch (OP_CACHE(op)) {
    case 0: /* I-cache */
        switch (OP_KIND(op)) {
        case 0: /* Index Invalidate */
        case 2: /* Index Store Tag */
            __atomic_add_fetch(&i_ops[cpu][set], 1, __ATOMIC_RELAXED);
            lru_index(lru_i[cpu], vaddr);
            break;
        case 4: /* Hit Invalidate */
            if (phys(vaddr, &pa) && pa < ram_size) {
                clear_bits(pa >> line_shift, I_RES(cpu) | I_STALE(cpu));
                lru_hit(lru_i[cpu], pa >> line_shift, false);
            }
            break;
        case 5: /* Fill */
            if (phys(vaddr, &pa) && pa < ram_size) {
                ln = pa >> line_shift;
                clear_bits(ln, I_STALE(cpu));
                i_epoch[cpu][ln] = i_ops[cpu][set];
                set_bits(ln, I_RES(cpu));
                lru_use(lru_i[cpu], ln, false);
            }
            break;
        }
        break;
    case 1: /* D-cache */
        switch (OP_KIND(op)) {
        case 0: /* Index Writeback Invalidate */
        case 2: /* Index Store Tag */
            __atomic_add_fetch(&d_ops[set], 1, __ATOMIC_RELAXED);
            lru_index(lru_d[cpu], vaddr);
            break;
        case 4: /* Hit Invalidate */
        case 5: /* Hit Writeback Invalidate */
            if (phys(vaddr, &pa) && pa < ram_size) {
                clear_bits(pa >> line_shift, D_BITS);
                lru_hit(lru_d[cpu], pa >> line_shift, false);
            }
            break;
        case 6: /* Hit Writeback */
            if (phys(vaddr, &pa) && pa < ram_size) {
                clear_bits(pa >> line_shift, D_DIRTY);
                lru_hit(lru_d[cpu], pa >> line_shift, true);
            }
            break;
        }
        break;
    }
}

static void plugin_exit(void *udata)
{
    g_autoptr(GString) out = g_string_new("coherency: summary:");

    for (int k = 0; k < N_KINDS; k++) {
        g_string_append_printf(out, " %s %" PRIu64 " (latent %" PRIu64 ")",
                               kind_name[k], counts[k][0], counts[k][1]);
    }
    g_string_append_c(out, '\n');
    qemu_plugin_outs(out->str);
}

static bool parse_uint(const char *s, uint64_t *v)
{
    char *end;
    uint64_t n = g_ascii_strtoull(s, &end, 0);

    switch (*end) {
    case 'k':
    case 'K':
        n <<= 10, end++;
        break;
    case 'm':
    case 'M':
        n <<= 20, end++;
        break;
    case 'g':
    case 'G':
        n <<= 30, end++;
        break;
    }
    *v = n;
    return end != s && !*end;
}

QEMU_PLUGIN_EXPORT
int qemu_plugin_install(qemu_plugin_id_t id, const qemu_info_t *info,
                        int argc, char **argv)
{
    uint64_t v;

    if (!info->system_emulation) {
        fprintf(stderr, "coherency: system emulation only\n");
        return -1;
    }
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) tok = g_strsplit(argv[i], "=", 2);

        if (!tok[1] || !parse_uint(tok[1], &v) || !v) {
            fprintf(stderr, "coherency: bad option %s\n", argv[i]);
            return -1;
        }
        if (g_strcmp0(tok[0], "ram") == 0) {
            ram_size = v;
        } else if (g_strcmp0(tok[0], "l1") == 0) {
            l1_size = v;
        } else if (g_strcmp0(tok[0], "ways") == 0 && v <= MAX_WAYS) {
            n_ways = v;
        } else if (g_strcmp0(tok[0], "line") == 0 && !(v & (v - 1))) {
            line_shift = __builtin_ctzll(v);
        } else if (g_strcmp0(tok[0], "max") == 0) {
            max_reports = v;
        } else {
            fprintf(stderr, "coherency: bad option %s\n", argv[i]);
            return -1;
        }
    }
    n_sets = l1_size / ((uint64_t)n_ways << line_shift);
    if (!n_sets || (n_sets & (n_sets - 1)) ||
        (uint64_t)n_sets * n_ways << line_shift != l1_size) {
        fprintf(stderr, "coherency: l1, ways and line do not make a cache\n");
        return -1;
    }
    n_cpus = MIN(info->system.max_vcpus, MAX_CPUS);

    state = g_new0(uint8_t, n_lines());
    last_pc = g_new0(uint32_t, n_lines());
    d_epoch = g_new0(uint32_t, n_lines());
    d_ops = g_new0(uint32_t, n_sets);
    for (unsigned c = 0; c < n_cpus; c++) {
        i_epoch[c] = g_new0(uint32_t, n_lines());
        i_ops[c] = g_new0(uint32_t, n_sets);
        lru_d[c] = g_new0(LruSet, n_sets);
        lru_i[c] = g_new0(LruSet, n_sets);
    }
    seen = g_hash_table_new(g_direct_hash, g_direct_equal);

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans, NULL);
    qemu_plugin_register_vcpu_cache_op_cb(id, cache_op, NULL);
    qemu_plugin_register_dma_cb(id, dma, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
