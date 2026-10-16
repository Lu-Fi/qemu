/*
 * MIPS watch registers: WatchLo/WatchHi pairs and the watch exception
 *
 * As measured on the Ingenic XBurst1 and XBurst2 cores: a pair matches a
 * virtual address on a doubleword, widened by the WatchHi mask, for any
 * ASID when WatchHi.G is set and else for EntryHi's. R matches loads and
 * LL, W stores and SC, I instruction fetches; CACHE, PREF and SYNCI match
 * nothing. A match sets the pair's WatchHi status bit and raises the
 * exception before the access, or with Status.EXL or ERL set only sets
 * Cause.WP, and the exception follows once both clear. This model takes
 * it at the next instruction, as XBurst2 does and as both cores do after
 * an eret; after an mtc0 that clears them, XBurst1 takes it four
 * instructions on (a taken branch's redirect counting as one). Cause.WP
 * reads back set at once here; on XBurst2 an mfc0 right after the access
 * still sees it clear, and one an instruction later may too.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "internal.h"
#include "exec/watchpoint.h"
#include "exec/helper-proto.h"
#include "../tcg-internal.h"

#define WL_I        4
#define WL_R        2
#define WL_W        1
#define WH_G        (1u << 30)
#define WH_MASK     0xff8u

static uint32_t watch_ignored(CPUMIPSState *env, int n)
{
    return (env->CP0_WatchHi[n] & WH_MASK) | 7;
}

static bool watch_matches(CPUMIPSState *env, int n, vaddr va)
{
    uint64_t hi = env->CP0_WatchHi[n];
    uint32_t asid = env->CP0_EntryHi_ASID_mask;

    if (((uint32_t)va ^ (uint32_t)env->CP0_WatchLo[n]) &
        ~watch_ignored(env, n)) {
        return false;
    }
    return (hi & WH_G) ||
           ((hi >> CP0WH_ASID) & asid) == (env->CP0_EntryHi & asid);
}

/* A match of pair @n: whether to raise the exception now */
static bool watch_hit(CPUMIPSState *env, int n, uint32_t status)
{
    if (env->hflags & MIPS_HFLAG_DM) {
        return false;
    }
    env->CP0_WatchHi[n] |= status;
    if (env->CP0_Status & ((1 << CP0St_EXL) | (1 << CP0St_ERL))) {
        BQL_LOCK_GUARD();

        env->CP0_Cause |= 1 << CP0Ca_WP;
        cpu_interrupt(env_cpu(env), CPU_INTERRUPT_WATCH);
        return false;
    }
    return true;
}

void mips_watch_update(CPUMIPSState *env, int n)
{
    CPUState *cs = env_cpu(env);
    uint32_t lo = env->CP0_WatchLo[n];
    int flags = BP_CPU | BP_STOP_BEFORE_ACCESS;
    bool fetch = false;

    if (env->watch_wp[n]) {
        cpu_watchpoint_remove_by_ref(cs, env->watch_wp[n]);
        env->watch_wp[n] = NULL;
    }
    if (lo & WL_R) {
        flags |= BP_MEM_READ;
    }
    if (lo & WL_W) {
        flags |= BP_MEM_WRITE;
    }
    if (flags & BP_MEM_ACCESS) {
        /* The aligned block holding every address the mask lets match */
        vaddr len = pow2ceil(watch_ignored(env, n) + 1);

        cpu_watchpoint_insert(cs, lo & ~(len - 1), len, flags,
                              &env->watch_wp[n]);
    }
    for (int i = 0; i < env->cpu_model->watch_pairs; i++) {
        fetch |= env->CP0_WatchLo[i] & WL_I;
    }
    env->watch_fetch = fetch;
}

void mips_watch_update_all(CPUMIPSState *env)
{
    for (int n = 0; n < env->cpu_model->watch_pairs; n++) {
        mips_watch_update(env, n);
    }
    if (env->CP0_Cause & (1 << CP0Ca_WP)) {
        BQL_LOCK_GUARD();

        cpu_interrupt(env_cpu(env), CPU_INTERRUPT_WATCH);
    }
}

bool mips_cpu_debug_check_watchpoint(CPUState *cs, CPUWatchpoint *wp)
{
    CPUMIPSState *env = cpu_env(cs);
    int n;

    for (n = 0; n < env->cpu_model->watch_pairs; n++) {
        if (env->watch_wp[n] == wp) {
            break;
        }
    }
    if (n == env->cpu_model->watch_pairs || !watch_matches(env, n, wp->hitaddr)) {
        return false;
    }
    if (env->watch_sc) {
        /* An SC: a store, though QEMU checks its compare-and-swap as both */
        return (wp->flags & BP_MEM_WRITE) && watch_hit(env, n, WL_W);
    }
    return watch_hit(env, n, wp->flags & BP_WATCHPOINT_HIT_WRITE ? WL_W : WL_R);
}

void mips_cpu_debug_excp_handler(CPUState *cs)
{
    CPUMIPSState *env = cpu_env(cs);
    CPUWatchpoint *wp = cs->watchpoint_hit;

    if (wp && (wp->flags & BP_CPU)) {
        cs->watchpoint_hit = NULL;
        env->watch_sc = 0;
        do_raise_exception(env, EXCP_DWATCH, 0);
    }
}

void HELPER(iwatch)(CPUMIPSState *env, target_ulong pc)
{
    for (int n = 0; n < env->cpu_model->watch_pairs; n++) {
        if ((env->CP0_WatchLo[n] & WL_I) && watch_matches(env, n, pc) &&
            watch_hit(env, n, WL_I)) {
            do_raise_exception(env, EXCP_IWATCH, GETPC());
        }
    }
}
