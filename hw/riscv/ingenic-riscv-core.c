/*
 * Ingenic RISC-V co-processors: the T31's, T23's, T32's, T40's, T41's and
 * A1's
 *
 * The T31, T23, T32, T33, T40, T41 and A1 have a small RISC-V core beside
 * their MIPS one, which Ingenic's fast-boot SDKs (Zeratul, Tassadar) use to
 * run the sensor and the ISP's auto-exposure while U-Boot loads Linux (the
 * T40's Zeratul leaves it alone); the A1's SDK has a driver and a sample
 * that rotate pictures on it. They run in
 * machine mode only and start at +0x80 from where the SoC boots them (the
 * T31's from its SRAM, 0x12210000, by default; the others where their SPL
 * or driver says, in DDR). They are three designs, with the ISA, IDs and
 * CSRs read on cameras: the T31's (CPU model ingenic-t31); Victory0, the
 * T32's and T33's (ingenic-victory0) and, its IDs reading 0, the T23's
 * (ingenic-t23); and the T40's, T41's and A1's (ingenic-t41-little). The
 * T33's is the T32's: ingenic-t32-riscv serves both. Their TNPUs have a
 * core of their own (machine ingenic-tnpu), the T32Pro's two
 * (ingenic-t32pro-tnpu, core=0 or 1): CSR 0xcc1 reads which, core 1 starts
 * where the SoC says rather than at +0x80, DDR shows at 0x80000000 too,
 * and a core tells the SoC as it waits in wfi. A T32's or T33's TNPU core
 * drives a TNNA of its own here (the T32Pro's two cores share the SoC's).
 *
 * - The T31's vectors are laid out as PULP's zero-riscy lays them out,
 *   where it boots: interrupt id N at +4N. Its firmware never writes mtvec
 *   or mie; the interrupt controller on the SoC's side masks. As on
 *   zero-riscy, wfi waits on while mstatus.MIE is clear, which is how the
 *   firmware parks the core for good, in its mailbox handler.
 *
 * - The others' firmware sets mtvec and mie itself. The T23's, T40's, T41's
 *   and A1's mailbox is the machine software interrupt, the T32's local
 *   interrupt 16; the T32's, T40's, T41's and A1's count reaching its compare
 *   is the timer interrupt, and the SoC interrupts the T32's block lets
 *   through (the ISP's, the VIC's) its external one.
 *
 * Its clock stopped, a core keeps its state and takes no interrupt. DDR
 * shows again at 0x20000000: the firmware clears bit 31 of the SPL's
 * pointers, uncached (KSEG1) ones among them.
 *
 * These machines are the core alone. The rest of the SoC is the matching
 * machine in qemu-system-mipsel (ingenic-t31 and so on), reached through
 * the UNIX socket "socket" names (see
 * include/hw/misc/ingenic-riscv-link.h): DDR and the T31's SRAM are mapped
 * from the SoC's memory, and every other access goes across but the T31's
 * OST count, which the core counts on from what the SoC last sent. The
 * SoC holds the core in reset until its SPL releases it.
 *
 *   qemu-system-mipsel -M ingenic-t31 ... \
 *       -object memory-backend-memfd,id=ddr,size=64M,share=on \
 *       -machine memory-backend=ddr \
 *       -chardev socket,id=rv,path=rv.sock,server=on,wait=on \
 *       -global ingenic-riscv.chardev=rv
 *   qemu-system-riscv32 -M ingenic-t31-riscv,socket=rv.sock -display none
 *
 * Unlike zero-riscy, an exception goes to the vector base, not to +0x84
 * (illegal instruction) or +0x88 (ecall).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/boards.h"
#include "hw/misc/ingenic-riscv-link.h"
#include "hw/misc/ingenic-tnna.h"
#include "io/channel-socket.h"
#include "exec/tb-flush.h"
#include "system/address-spaces.h"
#include "system/qtest.h"
#include "system/runstate.h"
#include "target/riscv/cpu.h"

#define TYPE_INGENIC_RISCV_CORE MACHINE_TYPE_NAME("ingenic-riscv-core")
OBJECT_DECLARE_TYPE(IngenicRiscvCoreState, IngenicRiscvCoreClass,
                    INGENIC_RISCV_CORE)

struct IngenicRiscvCoreClass {
    MachineClass parent_class;

    bool zero_riscy;            /* vectors and enables as the T31's */
    const char *cpu_type;
    uint32_t mstatus;           /* as the core comes out of reset */
    uint32_t mcountinhibit;
    bool tnpu_pro;              /* one of the T32Pro's TNPU's two cores */
    bool tnna;                  /* drives its own TNNA (the T32's, T33's) */
};

#define RV_MEMS     2           /* DDR and the SRAM */
#define RV_DDR_ALIAS 0x20000000
#define RV_DDR_KSEG  0x80000000

struct IngenicRiscvCoreState {
    MachineState parent_obj;

    RISCVCPU cpu;
    MemoryRegion soc;           /* the SoC's bus, across the link */
    IngenicTnnaState tnna;
    MemoryRegion mem[RV_MEMS];
    MemoryRegion ddr_alias;
    MemoryRegion ddr_kseg;      /* the T32Pro's cores: DDR at 0x80000000 too */
    bool ddr_alias_mapped;
    hwaddr mem_addr[RV_MEMS];
    bool mem_mapped[RV_MEMS];
    uint32_t irqs;              /* the lines the SoC holds up */
    uint32_t boot;              /* where the SoC boots the core */
    uint32_t core;              /* which of the T32Pro's TNPU cores */
    uint32_t wakes;             /* IRQ and RUN messages had, for WFI */
    bool clock;                 /* the core's clock runs */
    bool frozen;                /* the core's clock stopped it */
    bool frozen_halted;         /* in wfi when it stopped */
    uint64_t frozen_mie;
    bool count_known;           /* the T31's OST count, as the SoC sent */
    uint64_t count;
    uint32_t count_hz;          /* 0 while it is stopped */
    int64_t count_at;
    uint32_t count_hi;          /* the upper half a read latched */

    char *socket;
    QIOChannel *ioc;
    QemuThread reader;
    QemuMutex lock;             /* what follows, and sending */
    QemuCond answer_cond;
    bool connected;
    bool answered;
    uint32_t answer;
};

static void link_send(IngenicRiscvCoreState *s, uint32_t type, uint32_t addr,
                      uint32_t size, uint32_t data)
{
    IngenicRiscvMsg m = {
        .type = cpu_to_le32(type),
        .addr = cpu_to_le32(addr),
        .size = cpu_to_le32(size),
        .data = cpu_to_le32(data),
    };

    qio_channel_write_all(s->ioc, (const char *)&m, sizeof(m), NULL);
}

static void core_freeze(IngenicRiscvCoreState *s, CPUState *cs)
{
    if (!s->frozen) {
        s->frozen = true;
        s->frozen_halted = cs->halted;
        s->frozen_mie = s->cpu.env.mie;
        s->cpu.env.mie = 0;
        cs->halted = 1;
    }
}

static void core_clock(CPUState *cs, run_on_cpu_data data)
{
    IngenicRiscvCoreState *s = data.host_ptr;

    if (!s->clock) {
        core_freeze(s, cs);
    } else if (s->frozen) {
        s->frozen = false;
        s->cpu.env.mie = s->frozen_mie;
        cs->halted = s->frozen_halted;
    }
}

static void core_start(CPUState *cs, run_on_cpu_data data)
{
    IngenicRiscvCoreState *s = data.host_ptr;
    CPURISCVState *env = &s->cpu.env;

    cpu_reset(cs);
    s->frozen = false;
    /* the T32Pro's core 1 starts where the SoC says, its boot code's +0x80 */
    env->pc = s->boot + (s->core ? 0 : 0x80);
    env->mstatus = INGENIC_RISCV_CORE_GET_CLASS(s)->mstatus;
    env->mcountinhibit = INGENIC_RISCV_CORE_GET_CLASS(s)->mcountinhibit;
    if (INGENIC_RISCV_CORE_GET_CLASS(s)->zero_riscy) {
        env->mtvec = s->boot | 1;                   /* vectored */
        env->mie = UINT32_MAX;
    } else {
        /*
         * Released, the others read mie 0, which QEMU's CPU reset leaves
         * as it was. Tassadar's park firmware, which Linux restarts the
         * T32's core into, sets mstatus.MIE before its stack pointer.
         */
        env->mie = 0;
    }
    riscv_cpu_update_mip(env, UINT32_MAX, s->irqs);
    cs->halted = 0;
    if (!s->clock) {
        core_freeze(s, cs);
    }
}

static void core_hold(CPUState *cs, run_on_cpu_data data)
{
    IngenicRiscvCoreState *s = data.host_ptr;

    cpu_reset(cs);              /* halted, as it starts powered off */
    s->frozen = false;
}

static void link_mem(IngenicRiscvCoreState *s, hwaddr addr, uint64_t size,
                     int fd)
{
    Error *err = NULL;
    g_autofree char *name = NULL;
    int i;

    for (i = 0; i < RV_MEMS; i++) {
        if (!s->mem_mapped[i] || s->mem_addr[i] == addr) {
            break;
        }
    }
    if (i == RV_MEMS) {
        error_report("ingenic-riscv-core: more memories than %d", RV_MEMS);
        close(fd);
        return;
    }
    if (s->mem_mapped[i]) {
        if (addr == 0 && s->ddr_alias_mapped) {
            memory_region_del_subregion(get_system_memory(), &s->ddr_alias);
            object_unparent(OBJECT(&s->ddr_alias));
            if (INGENIC_RISCV_CORE_GET_CLASS(s)->tnpu_pro) {
                memory_region_del_subregion(get_system_memory(),
                                            &s->ddr_kseg);
                object_unparent(OBJECT(&s->ddr_kseg));
            }
            s->ddr_alias_mapped = false;
        }
        memory_region_del_subregion(get_system_memory(), &s->mem[i]);
        object_unparent(OBJECT(&s->mem[i]));
        s->mem_mapped[i] = false;
    }
    name = g_strdup_printf("ingenic-riscv-core.mem@0x%" HWADDR_PRIx, addr);
    if (!memory_region_init_ram_from_fd(&s->mem[i], OBJECT(s), name, size,
                                        RAM_SHARED, fd, 0, &err)) {
        error_report_err(err);
        close(fd);
        return;
    }
    memory_region_add_subregion(get_system_memory(), addr, &s->mem[i]);
    s->mem_addr[i] = addr;
    s->mem_mapped[i] = true;
    if (addr == 0) {
        memory_region_init_alias(&s->ddr_alias, OBJECT(s),
                                 "ingenic-riscv-core.ddr-alias", &s->mem[i], 0,
                                 size);
        memory_region_add_subregion(get_system_memory(), RV_DDR_ALIAS,
                                    &s->ddr_alias);
        if (INGENIC_RISCV_CORE_GET_CLASS(s)->tnpu_pro) {
            /* the driver gives core 1 a stack at a KSEG0 address */
            memory_region_init_alias(&s->ddr_kseg, OBJECT(s),
                                     "ingenic-riscv-core.ddr-kseg",
                                     &s->mem[i], 0, size);
            memory_region_add_subregion(get_system_memory(), RV_DDR_KSEG,
                                        &s->ddr_kseg);
        }
        s->ddr_alias_mapped = true;
    }
}

/* A message the SoC sent of itself, with the BQL held; fd is taken */
static void link_dispatch(IngenicRiscvCoreState *s, const IngenicRiscvMsg *m,
                          int fd)
{
    CPUState *cs = CPU(&s->cpu);
    uint32_t addr = le32_to_cpu(m->addr);
    uint32_t data = le32_to_cpu(m->data);

    switch (le32_to_cpu(m->type)) {
    case IRL_MEM:
        if (fd < 0) {
            error_report("ingenic-riscv-core: the SoC sent memory at 0x%x "
                         "without its file descriptor", addr);
        } else {
            link_mem(s, addr, le32_to_cpu(m->size), fd);
        }
        return;
    case IRL_RUN:
        qatomic_inc(&s->wakes);
        if (data) {
            s->boot = addr;
            queue_tb_flush(cs);     /* the SoC has loaded the core's code */
            async_run_on_cpu(cs, core_start, RUN_ON_CPU_HOST_PTR(s));
        } else {
            async_run_on_cpu(cs, core_hold, RUN_ON_CPU_HOST_PTR(s));
        }
        break;
    case IRL_CLOCK:
        s->clock = data;
        async_run_on_cpu(cs, core_clock, RUN_ON_CPU_HOST_PTR(s));
        break;
    case IRL_VARIANT:
        if (INGENIC_RISCV_CORE_GET_CLASS(s)->tnna) {
            ingenic_tnna_set_t33(&s->tnna, data);
        }
        break;
    case IRL_COUNT:
        s->count_known = true;
        s->count = (uint64_t)le32_to_cpu(m->size) << 32 | data;
        s->count_hz = addr;
        s->count_at = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    case IRL_IRQ:
        qatomic_inc(&s->wakes);
        if (addr == IRQ_M_SOFT && data) {
            /*
             * The SoC rings before handing over a task, whose code it may
             * have just put in DDR: writes from its process do not reach
             * this one's translations.
             */
            queue_tb_flush(cs);
        }
        if (addr < 32) {
            s->irqs = deposit32(s->irqs, addr, 1, !!data);
            riscv_cpu_update_mip(&s->cpu.env, BIT_ULL(addr),
                                 data ? BIT_ULL(addr) : 0);
        }
        break;
    default:
        error_report("ingenic-riscv-core: unexpected message %u from the SoC",
                     le32_to_cpu(m->type));
        break;
    }
    if (fd >= 0) {
        close(fd);
    }
}

/*
 * The link's input, read in a thread of its own rather than the main
 * loop, which a core stopped for gdb or for shutdown may be waiting on.
 */
static void *link_reader(void *opaque)
{
    IngenicRiscvCoreState *s = opaque;

    for (;;) {
        IngenicRiscvMsg m;
        struct iovec iov = { .iov_base = &m, .iov_len = sizeof(m) };
        g_autofree int *fds = NULL;
        size_t nfds = 0;
        int fd = -1;

        if (qio_channel_readv_full_all_eof(s->ioc, &iov, 1, &fds, &nfds, 0,
                                           NULL) <= 0) {
            break;
        }
        for (size_t i = 0; i < nfds; i++) {
            if (fd < 0) {
                fd = fds[i];
            } else {
                close(fds[i]);
            }
        }
        if (le32_to_cpu(m.type) == IRL_DATA) {
            qemu_mutex_lock(&s->lock);
            s->answer = le32_to_cpu(m.data);
            s->answered = true;
            qemu_cond_signal(&s->answer_cond);
            qemu_mutex_unlock(&s->lock);
            if (fd >= 0) {
                close(fd);
            }
            continue;
        }
        bql_lock();
        link_dispatch(s, &m, fd);
        bql_unlock();
    }

    qemu_mutex_lock(&s->lock);
    s->connected = false;
    qemu_cond_broadcast(&s->answer_cond);
    qemu_mutex_unlock(&s->lock);
    info_report("ingenic-riscv-core: the SoC has gone");
    bql_lock();
    qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_ERROR);
    bql_unlock();
    return NULL;
}

/*
 * The core's access, made on the SoC's bus. It waits without the BQL, as
 * QEMU's multi-process proxy does: the reader may need it to deliver what
 * comes first, and a core polling the SoC would starve the main loop.
 */
static uint32_t link_transact(IngenicRiscvCoreState *s, uint32_t type,
                              hwaddr addr, unsigned size, uint32_t data)
{
    bool bql = bql_locked();
    uint32_t answer = 0;

    if (bql) {
        bql_unlock();
    }
    qemu_mutex_lock(&s->lock);
    if (s->connected) {
        s->answered = false;
        link_send(s, type, addr, size, data);
        while (!s->answered && s->connected) {
            qemu_cond_wait(&s->answer_cond, &s->lock);
        }
        answer = s->answer;
    }
    qemu_mutex_unlock(&s->lock);
    if (bql) {
        bql_lock();
    }
    return answer;
}

static uint64_t core_count(IngenicRiscvCoreState *s)
{
    return s->count +
        muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->count_at,
                 s->count_hz, NANOSECONDS_PER_SECOND);
}

static uint64_t soc_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicRiscvCoreState *s = opaque;
    uint64_t count;

    if (s->count_known && addr == INGENIC_RISCV_T31_COUNT && size == 4) {
        count = core_count(s);
        s->count_hi = count >> 32;
        return (uint32_t)count;
    }
    if (s->count_known && addr == INGENIC_RISCV_T31_COUNT_HI && size == 4) {
        return s->count_hi;
    }
    return link_transact(s, IRL_READ, addr, size, 0);
}

static void soc_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    link_transact(opaque, IRL_WRITE, addr, size, value);
}

static const MemoryRegionOps soc_ops = {
    .read = soc_read,
    .write = soc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * A T32Pro TNPU core waits in wfi: the SoC reads it asleep. Nothing else
 * the core does before its wfi crosses the link.
 */
static void core_wfi(RISCVCPU *cpu)
{
    IngenicRiscvCoreState *s = INGENIC_RISCV_CORE(qdev_get_machine());

    if (cpu->env.mip & cpu->env.mie) {
        return;                 /* it wakes at once */
    }
    qemu_mutex_lock(&s->lock);
    if (s->connected) {
        link_send(s, IRL_WFI, 0, 0, qatomic_read(&s->wakes));
    }
    qemu_mutex_unlock(&s->lock);
}

static void ingenic_riscv_core_init(MachineState *machine)
{
    IngenicRiscvCoreState *s = INGENIC_RISCV_CORE(machine);
    Object *cpu = OBJECT(&s->cpu);
    SocketAddress addr = { .type = SOCKET_ADDRESS_TYPE_UNIX };
    QIOChannelSocket *sioc;

    if (!s->socket && !qtest_enabled()) {
        error_report("ingenic-riscv-core: socket=<path> names the SoC's link");
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "cpu", &s->cpu,
                            INGENIC_RISCV_CORE_GET_CLASS(s)->cpu_type);
    object_property_set_uint(cpu, "resetvec", INGENIC_RISCV_SRAM_BASE + 0x80,
                             &error_abort);
    object_property_set_bool(cpu, "start-powered-off", true, &error_abort);
    s->cpu.env.mhartid = s->core;
    if (INGENIC_RISCV_CORE_GET_CLASS(s)->tnpu_pro) {
        s->cpu.wfi_notify = core_wfi;
    }
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);

    memory_region_init_io(&s->soc, OBJECT(s), &soc_ops, s,
                          "ingenic-riscv-core.soc", 4 * GiB);
    memory_region_add_subregion_overlap(get_system_memory(), 0, &s->soc, -1);
    if (INGENIC_RISCV_CORE_GET_CLASS(s)->tnna) {
        /* its NNA and colour converter, the core's view, over the link */
        object_initialize_child(OBJECT(s), "tnna", &s->tnna,
                                TYPE_INGENIC_TNNA);
        qdev_prop_set_bit(DEVICE(&s->tnna), "core-only", true);
        sysbus_realize(SYS_BUS_DEVICE(&s->tnna), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->tnna), 0, 0x12500000);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->tnna), 1, 0x12b00000);
    }
    qemu_mutex_init(&s->lock);
    qemu_cond_init(&s->answer_cond);

    if (!s->socket) {
        /* qtest only inspects the machine: no SoC, so the core stays off */
        return;
    }

    sioc = qio_channel_socket_new();
    addr.u.q_unix.path = s->socket;
    if (qio_channel_socket_connect_sync(sioc, &addr, &error_fatal) < 0) {
        exit(1);
    }
    s->ioc = QIO_CHANNEL(sioc);
    s->connected = true;
    link_send(s, IRL_HELLO, 0, 0, INGENIC_RISCV_LINK_VERSION);
    qemu_thread_create(&s->reader, "ingenic-rv-link", link_reader, s,
                       QEMU_THREAD_DETACHED);
}

static char *ingenic_riscv_core_get_socket(Object *obj, Error **errp)
{
    return g_strdup(INGENIC_RISCV_CORE(obj)->socket);
}

static void ingenic_riscv_core_set_socket(Object *obj, const char *value,
                                          Error **errp)
{
    IngenicRiscvCoreState *s = INGENIC_RISCV_CORE(obj);

    g_free(s->socket);
    s->socket = g_strdup(value);
}

static void ingenic_riscv_core_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = ingenic_riscv_core_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 0;
    object_class_property_add_str(oc, "socket",
                                  ingenic_riscv_core_get_socket,
                                  ingenic_riscv_core_set_socket);
    object_class_property_set_description(oc, "socket",
        "The UNIX socket the SoC's QEMU listens on (ingenic-riscv.chardev)");
}

static void ingenic_little_core(IngenicRiscvCoreClass *rc)
{
    rc->cpu_type = TYPE_RISCV_CPU_INGENIC_LITTLE;
    rc->mstatus = MSTATUS_MPP;
}

static void ingenic_t31_riscv_class_init(ObjectClass *oc, const void *data)
{
    IngenicRiscvCoreClass *rc = INGENIC_RISCV_CORE_CLASS(oc);

    MACHINE_CLASS(oc)->desc = "Ingenic T31 RISC-V co-processor, linked to an "
                              "ingenic-t31";
    rc->zero_riscy = true;
    rc->cpu_type = TYPE_RISCV_CPU_INGENIC_T31;
    rc->mstatus = MSTATUS_MPP;
}

static void ingenic_t23_riscv_class_init(ObjectClass *oc, const void *data)
{
    IngenicRiscvCoreClass *rc = INGENIC_RISCV_CORE_CLASS(oc);

    MACHINE_CLASS(oc)->desc = "Ingenic T23 RISC-V co-processor, linked to an "
                              "ingenic-t23";
    rc->cpu_type = TYPE_RISCV_CPU_INGENIC_T23;
    rc->mstatus = MSTATUS_MPIE;
    rc->mcountinhibit = 0xfffffff8;
}

/*
 * The T32's, T33's and T32Pro's TNPU core is the T23's design (misa and
 * IDs as a T32LQ's reads them), linked to the SoC's ingenic-tnpu
 */
static void ingenic_tnpu_class_init(ObjectClass *oc, const void *data)
{
    IngenicRiscvCoreClass *rc = INGENIC_RISCV_CORE_CLASS(oc);

    MACHINE_CLASS(oc)->desc = "Ingenic TNPU RISC-V core, linked to the "
                              "ingenic-tnpu of an ingenic-t32 or ingenic-t33";
    rc->cpu_type = TYPE_RISCV_CPU_INGENIC_T23;
    rc->tnna = true;
    rc->mstatus = MSTATUS_MPIE;
    rc->mcountinhibit = 0xfffffff8;
}

static void ingenic_riscv_core_get_core(Object *obj, Visitor *v,
                                        const char *name, void *opaque,
                                        Error **errp)
{
    visit_type_uint32(v, name, &INGENIC_RISCV_CORE(obj)->core, errp);
}

static void ingenic_riscv_core_set_core(Object *obj, Visitor *v,
                                        const char *name, void *opaque,
                                        Error **errp)
{
    uint32_t core;

    if (!visit_type_uint32(v, name, &core, errp)) {
        return;
    }
    if (core > 1) {
        error_setg(errp, "the T32Pro's TNPU has cores 0 and 1");
        return;
    }
    INGENIC_RISCV_CORE(obj)->core = core;
}

/*
 * The T32Pro's TNPU has two cores, each in a QEMU of its own linked to the
 * SoC's ingenic-tnpu (core 0 through its chardev, core 1 through chardev1)
 */
static void ingenic_t32pro_tnpu_class_init(ObjectClass *oc, const void *data)
{
    IngenicRiscvCoreClass *rc = INGENIC_RISCV_CORE_CLASS(oc);

    MACHINE_CLASS(oc)->desc = "Ingenic T32Pro TNPU RISC-V core (core=0 or 1), "
                              "linked to the ingenic-tnpu of an ingenic-t32";
    rc->cpu_type = TYPE_RISCV_CPU_INGENIC_T32PRO;
    rc->mstatus = MSTATUS_MPIE;
    rc->mcountinhibit = 0xfffffff8;
    rc->tnpu_pro = true;
    object_class_property_add(oc, "core", "uint32",
                              ingenic_riscv_core_get_core,
                              ingenic_riscv_core_set_core, NULL, NULL);
    object_class_property_set_description(oc, "core",
        "Which of the TNPU's cores this is: 0 (v0) or 1 (v1)");
}

static void ingenic_t32_riscv_class_init(ObjectClass *oc, const void *data)
{
    IngenicRiscvCoreClass *rc = INGENIC_RISCV_CORE_CLASS(oc);

    MACHINE_CLASS(oc)->desc = "Ingenic T32/T33 RISC-V co-processor, linked "
                              "to an ingenic-t32 or ingenic-t33";
    rc->cpu_type = TYPE_RISCV_CPU_INGENIC_VICTORY0;
    rc->mstatus = MSTATUS_MPIE;
    rc->mcountinhibit = 0xfffffff8;
}

static void ingenic_t40_riscv_class_init(ObjectClass *oc, const void *data)
{
    MACHINE_CLASS(oc)->desc = "Ingenic T40 RISC-V co-processor, linked to an "
                              "ingenic-t40";
    ingenic_little_core(INGENIC_RISCV_CORE_CLASS(oc));
}

static void ingenic_t41_riscv_class_init(ObjectClass *oc, const void *data)
{
    MACHINE_CLASS(oc)->desc = "Ingenic T41 RISC-V co-processor, linked to an "
                              "ingenic-t41";
    ingenic_little_core(INGENIC_RISCV_CORE_CLASS(oc));
}

static void ingenic_a1_riscv_class_init(ObjectClass *oc, const void *data)
{
    MACHINE_CLASS(oc)->desc = "Ingenic A1 RISC-V co-processor, linked to an "
                              "ingenic-a1";
    ingenic_little_core(INGENIC_RISCV_CORE_CLASS(oc));
}

static const TypeInfo ingenic_riscv_core_types[] = {
    {
        .name = TYPE_INGENIC_RISCV_CORE,
        .parent = TYPE_MACHINE,
        .abstract = true,
        .instance_size = sizeof(IngenicRiscvCoreState),
        .class_size = sizeof(IngenicRiscvCoreClass),
        .class_init = ingenic_riscv_core_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-t31-riscv"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_t31_riscv_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-t23-riscv"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_t23_riscv_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-t32-riscv"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_t32_riscv_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-t40-riscv"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_t40_riscv_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-t41-riscv"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_t41_riscv_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-a1-riscv"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_a1_riscv_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-tnpu"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_tnpu_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ingenic-t32pro-tnpu"),
        .parent = TYPE_INGENIC_RISCV_CORE,
        .class_init = ingenic_t32pro_tnpu_class_init,
    },
};

DEFINE_TYPES(ingenic_riscv_core_types)
