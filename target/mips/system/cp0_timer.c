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
}

void cpu_mips_core_clock_start(CPUMIPSState *env)
{
    env->core_clock_stopped = false;
    if (cpu_mips_count_running(env)) {
        cpu_mips_start_count(env);
    }
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
}
