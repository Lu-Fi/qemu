/*
 * QEMU MIPS timer support
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "internal.h"

/* MIPS R4K timer */
static uint32_t cpu_mips_get_count_val(CPUMIPSState *env)
{
    MIPSCPU *cpu = env_archcpu(env);
    int64_t now_ns;
    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    return env->CP0_Count +
            (uint32_t)clock_ns_to_ticks(cpu->count_clock, now_ns);
}

static void cpu_mips_timer_update(CPUMIPSState *env)
{
    MIPSCPU *cpu = env_archcpu(env);
    uint64_t now_ns, next_ns;
    uint32_t wait;

    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    wait = env->CP0_Compare - cpu_mips_get_count_val(env);
    /* Clamp interval to overflow if virtual time had not progressed */
    if (!wait) {
        wait = UINT32_MAX;
    }
    next_ns = now_ns + clock_ticks_to_ns(cpu->count_clock, wait);
    timer_mod(env->timer, next_ns);
}

/* Expire the timer.  */
static void cpu_mips_timer_expire(CPUMIPSState *env)
{
    cpu_mips_timer_update(env);
    if (env->insn_flags & ISA_MIPS_R2) {
        env->CP0_Cause |= 1 << CP0Ca_TI;
    }
    qemu_irq_raise(env->irq[(env->CP0_IntCtl >> CP0IntCtl_IPTI) & 0x7]);
}

/*
 * Count runs unless Cause.DC stops it, WAIT has stopped the core clock, or
 * the CPU model's Count never runs at all.
 */
bool cpu_mips_count_running(CPUMIPSState *env)
{
    return !(env->CP0_Cause & (1 << CP0Ca_DC)) && !env->core_clock_stopped &&
           !(env->cpu_model->cp0_flags & MIPS_CP0_COUNT_FROZEN);
}

/* A Count that never runs matches Compare while the two are equal */
static void cpu_mips_count_match(CPUMIPSState *env)
{
    if (!(env->cpu_model->cp0_flags & MIPS_CP0_COUNT_FROZEN) ||
        env->CP0_Count != env->CP0_Compare) {
        return;
    }
    if (env->insn_flags & ISA_MIPS_R2) {
        env->CP0_Cause |= 1 << CP0Ca_TI;
    }
    qemu_irq_raise(env->irq[(env->CP0_IntCtl >> CP0IntCtl_IPTI) & 0x7]);
}

uint32_t cpu_mips_get_count(CPUMIPSState *env)
{
    if (!cpu_mips_count_running(env)) {
        return env->CP0_Count;
    } else {
        uint64_t now_ns;

        now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (timer_pending(env->timer)
            && timer_expired(env->timer, now_ns)) {
            /* The timer has already expired.  */
            cpu_mips_timer_expire(env);
        }

        return cpu_mips_get_count_val(env);
    }
}

void cpu_mips_store_count(CPUMIPSState *env, uint32_t count)
{
    /*
     * This gets called from cpu_state_reset(), potentially before timer init.
     * So env->timer may be NULL, so treat timer as disabled in that case.
     */
    MIPSCPU *cpu = env_archcpu(env);
    if (!cpu_mips_count_running(env) || !env->timer) {
        env->CP0_Count = count;
        cpu_mips_count_match(env);
    } else {
        /* Store new count register */
        env->CP0_Count = count - (uint32_t)clock_ns_to_ticks(cpu->count_clock,
                        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        /* Update timer timer */
        cpu_mips_timer_update(env);
    }
}

void cpu_mips_store_compare(CPUMIPSState *env, uint32_t value)
{
    env->CP0_Compare = value;
    if (cpu_mips_count_running(env)) {
        cpu_mips_timer_update(env);
    }
    if (env->insn_flags & ISA_MIPS_R2) {
        env->CP0_Cause &= ~(1 << CP0Ca_TI);
    }
    qemu_irq_lower(env->irq[(env->CP0_IntCtl >> CP0IntCtl_IPTI) & 0x7]);
    cpu_mips_count_match(env);
}

void cpu_mips_start_count(CPUMIPSState *env)
{
    cpu_mips_store_count(env, env->CP0_Count);
}

void cpu_mips_stop_count(CPUMIPSState *env)
{
    /* Store the current value */
    MIPSCPU *cpu = env_archcpu(env);
    env->CP0_Count += (uint32_t)clock_ns_to_ticks(cpu->count_clock,
                        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* WAIT stops the core clock on some cores, and Count with it */
void cpu_mips_core_clock_stop(CPUMIPSState *env)
{
    if (cpu_mips_count_running(env)) {
        cpu_mips_stop_count(env);
    }
    env->core_clock_stopped = true;
    cpu_mips_pmu_sync(env);
}

void cpu_mips_core_clock_start(CPUMIPSState *env)
{
    env->core_clock_stopped = false;
    if (cpu_mips_count_running(env)) {
        cpu_mips_start_count(env);
    }
    cpu_mips_pmu_sync(env);
}

/*
 * The cycles event of the performance counters counts the CPU input clock
 * in the modes PerfCtl enables, and stops with the core clock. A counter's
 * CP0_PerfCnt lacks the cycles since pmu_clock_ns while it counts them.
 */
static uint32_t cpu_mips_pmu_cycling(CPUMIPSState *env)
{
    uint32_t cycling = 0;

    if (env->core_clock_stopped || env_archcpu(env)->cycles_per_read) {
        return 0;
    }
    for (int i = 0; i < env->cpu_model->pmu_counters; i++) {
        if (mips_pmu_counts(env, i, env->cpu_model->pmu_ev_cycles)) {
            cycling |= 1 << i;
        }
    }
    return cycling;
}

static uint32_t cpu_mips_pmu_value(CPUMIPSState *env, int i, int64_t now)
{
    Clock *clk = env_archcpu(env)->clock;
    uint32_t value = env->CP0_PerfCnt[i];

    if (env->pmu_cycling & (1 << i)) {
        /* From absolute tick counts, so that no rounding loses a cycle */
        value += clock_ns_to_ticks(clk, now) -
                 clock_ns_to_ticks(clk, env->pmu_clock_ns);
    }
    return value;
}

static void cpu_mips_pmu_fold(CPUMIPSState *env, int64_t now)
{
    for (int i = 0; i < env->cpu_model->pmu_counters; i++) {
        env->CP0_PerfCnt[i] = cpu_mips_pmu_value(env, i, now);
    }
    env->pmu_clock_ns = now;
}

/*
 * Cause.PCI shows any counter with bit 31 set, and those whose IE is set
 * also raise the interrupt that IntCtl.IPPCI names. This reads the counters
 * as last folded, which the overflow timer does at each bit 31 flip.
 */
void cpu_mips_pmu_update_irq(CPUMIPSState *env)
{
    const mips_def_t *def = env->cpu_model;
    bool overflow = false;
    bool irq = false;

    if (!def->pmu_irq) {
        return;
    }
    for (int i = 0; i < def->pmu_counters; i++) {
        if (env->CP0_PerfCnt[i] >> 31) {
            overflow = true;
            irq |= extract32(env->CP0_PerfCtl[i], CP0PC_IE, 1);
        }
    }
    if (overflow == extract32(env->CP0_Cause, CP0Ca_PCI, 1) &&
        irq == env->pmu_irq_level) {
        return;
    }
    /* Devices update Cause under the BQL too */
    BQL_LOCK_GUARD();
    env->CP0_Cause = deposit32(env->CP0_Cause, CP0Ca_PCI, 1, overflow);
    if (irq != env->pmu_irq_level) {
        env->pmu_irq_level = irq;
        qemu_set_irq(env->irq[def->pmu_ip], irq);
    }
}

/* Set the overflow timer to the next bit 31 flip of a cycles counter */
void cpu_mips_pmu_arm(CPUMIPSState *env)
{
    Clock *clk = env_archcpu(env)->clock;
    uint32_t ticks = 0x80000000u;
    int64_t now;

    if (!env->pmu_timer) {
        return;
    }
    if (!env->pmu_cycling) {
        timer_del(env->pmu_timer);
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (int i = 0; i < env->cpu_model->pmu_counters; i++) {
        if (env->pmu_cycling & (1 << i)) {
            ticks = MIN(ticks, 0x80000000u -
                        (cpu_mips_pmu_value(env, i, now) & 0x7fffffffu));
        }
    }
    /* clock_ticks_to_ns() rounds down */
    timer_mod(env->pmu_timer, now + clock_ticks_to_ns(clk, ticks) + 1);
}

/* The timer fires on the main loop: fold on the vCPU thread, between TBs */
static void cpu_mips_pmu_flip(CPUState *cs, run_on_cpu_data data)
{
    CPUMIPSState *env = cpu_env(cs);

    cpu_mips_pmu_fold(env, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    cpu_mips_pmu_update_irq(env);
    cpu_mips_pmu_arm(env);
}

static void cpu_mips_pmu_timer_cb(void *opaque)
{
    CPUMIPSState *env = opaque;

    async_run_on_cpu(env_cpu(env), cpu_mips_pmu_flip, RUN_ON_CPU_NULL);
}

/* After a change of the mode, PerfCtl or the core clock */
void cpu_mips_pmu_sync(CPUMIPSState *env)
{
    uint32_t cycling = cpu_mips_pmu_cycling(env);

    if (cycling == env->pmu_cycling) {
        return;
    }
    cpu_mips_pmu_fold(env, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    env->pmu_cycling = cycling;
    cpu_mips_pmu_arm(env);
}

uint32_t cpu_mips_pmu_read(CPUMIPSState *env, int i)
{
    uint32_t step = env_archcpu(env)->cycles_per_read;

    if (step && mips_pmu_counts(env, i, env->cpu_model->pmu_ev_cycles)) {
        env->CP0_PerfCnt[i] += step;
        cpu_mips_pmu_update_irq(env);
    }
    if (!(env->pmu_cycling & (1 << i))) {
        return env->CP0_PerfCnt[i];
    }
    return cpu_mips_pmu_value(env, i, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

void cpu_mips_pmu_write(CPUMIPSState *env, int i, uint32_t value)
{
    if (env->pmu_cycling) {
        cpu_mips_pmu_fold(env, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    env->CP0_PerfCnt[i] = value;
    cpu_mips_pmu_update_irq(env);
    cpu_mips_pmu_arm(env);
}

static void mips_timer_cb(void *opaque)
{
    CPUMIPSState *env;

    env = opaque;

    if (!cpu_mips_count_running(env)) {
        return;
    }

    cpu_mips_timer_expire(env);
}

void cpu_mips_clock_init(MIPSCPU *cpu)
{
    CPUMIPSState *env = &cpu->env;

    env->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &mips_timer_cb, env);
    if (env->cpu_model->pmu_irq) {
        env->pmu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      &cpu_mips_pmu_timer_cb, env);
    }
}
