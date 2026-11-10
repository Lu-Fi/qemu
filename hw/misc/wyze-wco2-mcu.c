/*
 * The Wyze Cam Outdoor v2's power-management MCU
 *
 * Zeratul battery cameras put a part of the board vendor's choosing beside
 * the SoC to power it (an STM32, an ATBM6441 or ATBM6431, an XR871...).
 * This is the Wyze Cam Outdoor v2's: an STM32L0 running Wyze's firmware
 * 02.01.09, behind I2C address 0x11, which Ingenic's zrt_cam_daemon drives.
 * A command is an 8-byte frame
 * (C3 09, index, its complement, 55 AA) taken only after a rising edge on
 * its wake input, which lets it out of Stop mode; the part does not answer
 * its address otherwise, nor outside the one command the edge started
 * (the real part stretches the clock there, and a bus that never ends
 * would only hang the model). The reply is a status, 8 bytes with a CRC-8
 * after them, then the data of a get, and a set takes an 8-byte payload
 * after its status. Each step waits one second of virtual time, as the
 * firmware's do, and a transfer that is not what the step wants ends the
 * command.
 *
 * It runs the board: Wi-Fi, PIR and red LED power, the interrupt pulse to
 * the SoC (10 ms), and the SoC's own power. Its RTC (two daily alarms), a
 * set of timers on its 32.768 kHz low-power timer (the watchdog among
 * them, which cuts the SoC's power when it is not fed) and the inputs
 * (button, PIR, charger, power switch, fuel gauge alert, Wi-Fi wake) set
 * the event flags the SoC reads, as 02.01.09 does. The firmware is
 * modelled from its disassembly, the timers' periods included: a "slow
 * tick" is 1030 timer ticks of 34 / 32768 s.
 *
 * The SoC's power is the PB6 output: when it falls the SoC's CPUs stop
 * (the board and the MCU run on), and when it rises again the machine is
 * reset (the SoC lost its power and boots);
 * with quit-on-power-off a fall shuts the machine down instead. The RTC
 * starts from the host's clock and runs on QEMU's RTC clock (-rtc clock=).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/ingenic-gpio.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/runstate.h"
#include "system/system.h"
#include "trace.h"

#define TYPE_WYZE_WCO2_MCU "wyze-wco2-mcu"
OBJECT_DECLARE_SIMPLE_TYPE(Wco2McuState, WYZE_WCO2_MCU)

#define HEAD            0xc309
#define TAIL            0x55aa
#define ST_OK           0x5aa5a55aU
#define ST_UNKNOWN      0x80030002U
#define ST_BAD          0x80000002U
#define LR_NORMAL       0x08002c23U
#define LR_BAD_FRAME    0x080028dfU
#define BAD_FRAMES_MAX  10000

#define STEP_NS         (1000 * SCALE_MS)
#define INT_PULSE_NS    (10 * SCALE_MS)
#define OFF_NS          (1500 * SCALE_MS)
#define WIFI_RESET_NS   (100 * SCALE_MS)
/* LPTIM1 on the 32768 Hz LSE with ARR 33 */
#define TICK_NS         1037597
#define SLOW_TICKS      1030

/* the flags word the SoC reads */
#define EV_KEY          0x1
#define EV_WIFI         0x2
#define EV_PIR          0x4
#define EV_BAT          0x8
#define EV_USB          0x20
#define EV_ALARM        0x40
#define EV_WDT          0x200
#define EV_CHG_DONE     0x400
#define EV_CHG_ERR      0x800
#define EV_DKEY         0x1000
#define EV_TIMER5       0x2000

/* EXTI lines, as the firmware's masks */
#define L0              BIT(0)
#define L1              BIT(1)
#define L2              BIT(2)
#define L3              BIT(3)
#define L4              BIT(4)
#define L7              BIT(7)
#define L13             BIT(13)
#define L14             BIT(14)
#define L15             BIT(15)
#define L_ALL           (L0 | L1 | L2 | L3 | L4 | L7 | L13 | L14 | L15)
#define L_PIR           (L0 | L7)
#define L_SENSE         (L2 | L3 | L4 | L13 | L14 | L15)

/* pins */
#define PA_PIR          0
#define PA_WLWAKE       2
#define PA_WAKE         3
#define PA_SWITCH       13
#define PA_CHG_STAT     14
#define PA_BUTTON       15
#define PB_ALERT        1
#define PB_USB          4
#define PB_PIR2         7

enum {
    O_INT, O_WIFI, O_CHG_LED, O_LED, O_PIR, O_MASTER, O_NUM
};

static const struct {
    const char *gpio;
    const char *prop;
} outs[O_NUM] = {
    [O_INT] = { "int", "out-int" },
    [O_WIFI] = { "wifi-power", "out-wifi" },
    [O_CHG_LED] = { "charge-led", "out-charge-led" },
    [O_LED] = { "red-led", "out-red-led" },
    [O_PIR] = { "pir-power", "out-pir" },
    [O_MASTER] = { "master-power", "out-master" },
};

typedef struct {
    const char *name;
    bool port_b;
    uint8_t bit;
    bool invert;                /* the property is the pin's level inverted */
    bool dflt;                  /* the pin's level at power on */
} InPin;

static const InPin ins[] = {
    { "pir", false, PA_PIR, false, false },
    { "pir-b", true, PB_PIR2, false, false },
    { "wifi-wake", false, PA_WLWAKE, false, false },
    { "battery-alert", true, PB_ALERT, true, true },
    { "usb-power", true, PB_USB, false, false },
    { "charge-stat", false, PA_CHG_STAT, false, false },
    { "power-switch", false, PA_SWITCH, false, true },
    { "button", false, PA_BUTTON, true, true },
};

/* the lines with an edge detector: pin, port B, rising, falling */
static const struct {
    uint8_t line;
    bool port_b;
    bool rise, fall;
} lines[] = {
    { 0, false, true, false },
    { 1, true, false, true },
    { 2, false, true, false },
    { 3, false, true, false },
    { 4, true, true, true },
    { 7, true, true, false },
    { 13, false, true, true },
    { 14, false, true, true },
    { 15, false, false, true },
};

typedef enum {
    T_STEP, T_INT, T_TICK, T_OFF, T_WIFI, T_L1, T_L4, T_L13, T_L13B, T_L14,
    T_L15, T_ALARM_A, T_ALARM_B, T_NUM
} TimerId;

typedef enum {
    S_IDLE, S_FRAME, S_STATUS, S_DATA, S_PAYLOAD
} Stage;

typedef enum {
    PK_NONE, PK_TIME, PK_ALARM_A, PK_ALARM_B, PK_RAW, PK_WDT_PERIOD,
    PK_PIR_TIME, PK_PIR_THR, PK_SAMP, PK_UPPER, PK_LOWER, PK_CLEAR
} Payload;

typedef struct Wco2McuTimer {
    Wco2McuState *s;
    TimerId id;
} Wco2McuTimer;

struct Wco2McuState {
    I2CSlave parent_obj;

    /* properties */
    DeviceState *gpio;
    DeviceState *wifi;
    int32_t wake_pin;
    int32_t int_pin;
    int32_t wifi_pin;
    char *version;
    bool quit_on_power_off;
    uint32_t pir_adc;
    Notifier done;

    qemu_irq irq[O_NUM];
    QEMUTimer *timer[T_NUM];
    Wco2McuTimer tid[T_NUM];

    /* pins and outputs */
    uint16_t pa_in, pb_in;
    bool out[O_NUM];
    bool pa5;
    bool i2c_pads;
    uint32_t imr, pr;

    /* firmware state */
    uint32_t flags;
    uint8_t mask;               /* timers running, bit 1..7 */
    uint32_t ticks;
    uint64_t ctr[8];
    uint64_t wdt_period, pir_time, samp, upper, lower;
    uint32_t wdt_strikes;
    uint32_t last_adc, first_adc;
    bool first_hit;
    bool wdt_en;
    bool cw_disabled;           /* charge wakeup disabled */
    bool shutoff;
    bool pir_on;
    uint8_t presses;
    uint8_t c1a, c1b, c1c;
    uint16_t c22;
    uint32_t bad_frames;
    uint32_t bat_low;
    uint32_t pir_thr;
    uint8_t pir_time_buf[8], pir_thr_buf[8];
    bool busy;                  /* inside a blocking delay */
    uint8_t off_stage;
    bool recover;               /* command 2: power back on after the off */

    /* RTC */
    int64_t rtc_ref_s;
    int64_t rtc_ref_ns;
    uint32_t alarm_reg[2];
    bool alarm_en[2];

    /* the command in hand */
    Stage stage;
    bool wake_pending;
    bool xfer;
    bool xfer_read;
    bool bad_dir;
    uint8_t rx[8];
    uint8_t rx_len;
    uint8_t tx[9];
    uint8_t tx_pos;
    bool want_status, want_data;
    Payload payload;
    uint8_t data[8];
    uint32_t status;
    uint32_t status_lr;
    bool servicing;
};

static void master_on(Wco2McuState *s);
static void exti_service(Wco2McuState *s);
static void session_begin(Wco2McuState *s);

static int64_t vnow(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* the calendar runs on QEMU's RTC clock, so -rtc clock= applies to it */
static int64_t rtc_ns(void)
{
    return qemu_clock_get_ns(rtc_clock);
}

static void arm(Wco2McuState *s, TimerId id, int64_t ns)
{
    timer_mod(s->timer[id], vnow() + ns);
}

/* ---- outputs ---------------------------------------------------------- */

static void out_set(Wco2McuState *s, int o, bool level)
{
    if (s->out[o] == level) {
        return;
    }
    s->out[o] = level;
    trace_wyze_wco2_mcu_out(vnow() / SCALE_MS, o, level);
    qemu_set_irq(s->irq[o], level);
    /*
     * Without power the SoC's CPUs stop while the board and this MCU run
     * on (a suspend would stop virtual time, and the MCU's timers with
     * it); with power back the machine resets, which resumes them, as
     * from a cold boot
     */
    if (o == O_MASTER) {
        if (level) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        } else if (s->quit_on_power_off) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        } else {
            CPUState *cs;

            CPU_FOREACH(cs) {
                cpu_pause(cs);
            }
        }
    }
}

static void int_lower(Wco2McuState *s)
{
    timer_del(s->timer[T_INT]);
    out_set(s, O_INT, false);
}

static void int_pulse(Wco2McuState *s)
{
    out_set(s, O_INT, true);
    arm(s, T_INT, INT_PULSE_NS);
}

static void wifi_reset(Wco2McuState *s)
{
    out_set(s, O_WIFI, false);
    arm(s, T_WIFI, WIFI_RESET_NS);
}

/* ---- pins ------------------------------------------------------------- */

static bool pin_pa(Wco2McuState *s, int bit)
{
    return extract32(s->pa_in, bit, 1);
}

static bool pin_pb(Wco2McuState *s, int bit)
{
    return extract32(s->pb_in, bit, 1);
}

static uint32_t gpioa_idr(Wco2McuState *s)
{
    uint32_t v = s->pa_in & (BIT(PA_PIR) | BIT(PA_WLWAKE) | BIT(PA_WAKE) |
                             BIT(PA_SWITCH) | BIT(PA_CHG_STAT) |
                             BIT(PA_BUTTON));

    v |= s->pa5 << 5 | s->out[O_INT] << 6 | s->out[O_WIFI] << 7;
    if (s->i2c_pads) {
        v |= BIT(9) | BIT(10);
    }
    return v;
}

static uint32_t gpiob_idr(Wco2McuState *s)
{
    uint32_t v = s->pb_in & (BIT(PB_ALERT) | BIT(PB_USB) | BIT(PB_PIR2));

    v |= s->out[O_CHG_LED] << 0 | s->out[O_LED] << 3 | s->out[O_PIR] << 5 |
         s->out[O_MASTER] << 6;
    return v;
}

static void pin_set(Wco2McuState *s, bool port_b, int bit, bool level)
{
    uint16_t *r = port_b ? &s->pb_in : &s->pa_in;

    if (extract32(*r, bit, 1) == level) {
        return;
    }
    *r = deposit32(*r, bit, 1, level);
    for (int i = 0; i < ARRAY_SIZE(lines); i++) {
        if (lines[i].line == bit && lines[i].port_b == port_b &&
            (level ? lines[i].rise : lines[i].fall)) {
            s->pr |= BIT(bit);
            exti_service(s);
        }
    }
}

static void wco2_mcu_wake_in(void *opaque, int n, int level)
{
    pin_set(opaque, false, PA_WAKE, level);
}

/* ---- EXTI mask -------------------------------------------------------- */

static void exti_enable(Wco2McuState *s, uint32_t m)
{
    s->imr |= m;
    exti_service(s);
}

static void exti_disable(Wco2McuState *s, uint32_t m)
{
    s->imr &= ~m;
}

/* ---- the scheduler (FUN_08001b30) ------------------------------------- */

static void tick_arm(Wco2McuState *s)
{
    uint32_t n;

    if (!s->mask) {
        timer_del(s->timer[T_TICK]);
        return;
    }
    if ((s->mask & (BIT(6) | BIT(7))) || s->c1b) {
        n = 1;
    } else {
        n = SLOW_TICKS - s->ticks % SLOW_TICKS;
    }
    arm(s, T_TICK, n * TICK_NS);
}

static void sched(Wco2McuState *s, unsigned n)
{
    if (n == 0) {
        s->mask = 0;
        s->ticks = 0;
        memset(s->ctr, 0, sizeof(s->ctr));
    } else if (n == 20) {
        s->mask = 1;
        s->ticks = 0;
        memset(s->ctr, 0, sizeof(s->ctr));
        tick_arm(s);
        return;
    } else if (n >= 21 && n <= 27) {
        s->mask |= BIT(n - 20);
    } else if (n >= 1 && n <= 7) {
        s->mask &= ~BIT(n);
        s->ctr[n] = 0;
    }
    if (!s->mask) {
        s->ticks = 0;
    }
    tick_arm(s);
}

static void sched_stop_all_pir(Wco2McuState *s)
{
    sched(s, 6);
    exti_enable(s, L7);
    exti_enable(s, L0);
}

static uint32_t adc_sample(Wco2McuState *s)
{
    return s->pir_adc & 0xfff;
}

static void tick_sampler(Wco2McuState *s)
{
    bool run = s->mask & BIT(6);
    uint64_t c = 0;
    bool check = false;

    if (!run || s->pir_time) {
        s->ctr[6] = 0;
        check = run;
    } else {
        c = ++s->ctr[6];
        check = true;
    }
    if (!check) {
        return;
    }
    if (c >= s->samp || s->pir_time) {
        if (!s->pir_time) {
            sched_stop_all_pir(s);
        }
        return;
    }
    s->last_adc = adc_sample(s);
    if (s->last_adc >= s->upper || s->last_adc <= s->lower) {
        if (!s->first_hit) {
            s->first_adc = s->last_adc;
            s->first_hit = true;
        }
        s->flags |= EV_PIR;
        master_on(s);
        int_pulse(s);
        sched_stop_all_pir(s);
    }
}

static void led_blink_end(Wco2McuState *s)
{
    out_set(s, O_LED, false);
}

static void tick_button(Wco2McuState *s)
{
    if (!(s->mask & BIT(7)) || ++s->ctr[7] < 501) {
        return;
    }
    if (pin_pb(s, PB_USB) || !s->shutoff) {
        if (pin_pb(s, PB_USB)) {
            s->shutoff = false;
        }
        s->flags |= s->presses < 2 ? EV_KEY : EV_DKEY;
        int_pulse(s);
        master_on(s);
        s->ctr[7] = 0;
    } else {
        sched(s, 3);
        led_blink_end(s);
        exti_enable(s, L15);
        s->pr &= ~L15;
    }
    sched(s, 7);
    s->presses = 0;
}

static void tick_slow(Wco2McuState *s)
{
    if (s->mask & BIT(2)) {
        uint64_t c = ++s->ctr[2];

        if (c >= s->wdt_period) {
            s->wdt_strikes++;
            out_set(s, O_MASTER, false);
            if (c - s->wdt_period > 2) {
                s->ctr[2] = 0;
                s->wdt_period = 30;
                s->flags |= EV_WDT;
                master_on(s);
            }
        }
    }
    if (s->mask & BIT(3)) {
        out_set(s, O_LED, s->c1b ? false : !s->out[O_LED]);
    }
    if ((s->mask & BIT(4)) && ++s->ctr[4] > 5 && pin_pa(s, PA_CHG_STAT) &&
        pin_pb(s, PB_USB)) {
        s->flags |= EV_CHG_DONE;
        sched(s, 4);
        sched(s, 3);
        int_pulse(s);
        out_set(s, O_LED, true);
        s->c1a = 0;
        if (pin_pa(s, PA_SWITCH) && !s->cw_disabled) {
            master_on(s);
        }
    }
    if ((s->mask & BIT(1)) && ++s->ctr[1] >= s->pir_time) {
        s->pir_time = 0;
        s->ctr[1] = 0;
        sched(s, 1);
        exti_enable(s, L0);
        exti_enable(s, L7);
    }
    if ((s->mask & BIT(5)) && ++s->ctr[5] > 60) {
        if (pin_pa(s, PA_SWITCH)) {
            s->flags |= EV_TIMER5;
            out_set(s, O_MASTER, true);
            out_set(s, O_WIFI, true);
            if (s->wdt_en) {
                sched(s, 22);
            }
            s->shutoff = false;
            exti_enable(s, L2 | L3 | L4 | L13 | L14 | L15);
            sched(s, 5);
        } else {
            sched(s, 5);
            sched(s, 25);
        }
    }
}

static void tick_fire(Wco2McuState *s)
{
    uint32_t n = 1;

    if (!(s->mask & (BIT(6) | BIT(7))) && !s->c1b) {
        n = SLOW_TICKS - s->ticks % SLOW_TICKS;
    }
    for (uint32_t i = 0; i < n; i++) {
        if (s->c1b && ++s->c22 >= 200) {
            s->c22 = 0;
            s->c1b = s->c1c == s->c1b ? 0 : 1;
            s->c1c = 1;
        }
        s->ticks++;
        tick_sampler(s);
        tick_button(s);
        if (s->ticks % SLOW_TICKS == 0) {
            tick_slow(s);
        }
    }
    tick_arm(s);
}

/* ---- the SoC's power -------------------------------------------------- */

static void master_on(Wco2McuState *s)
{
    exti_enable(s, L3);
    if (s->out[O_MASTER] || !pin_pa(s, PA_SWITCH)) {
        return;
    }
    if (s->wdt_en) {
        sched(s, 22);
    }
    out_set(s, O_MASTER, true);
    s->pa5 = true;
    out_set(s, O_WIFI, true);
    exti_enable(s, L2 | L3 | L4 | L13 | L14 | L15);
    s->i2c_pads = true;
}

static bool events_pending(Wco2McuState *s)
{
    return s->flags & ~(s->cw_disabled ? EV_CHG_DONE : 0);
}

static void master_off(Wco2McuState *s, bool recover)
{
    sched(s, 2);
    s->first_hit = false;
    int_lower(s);
    s->i2c_pads = false;
    s->pa5 = false;
    out_set(s, O_MASTER, false);
    s->busy = true;
    s->off_stage = 1;
    s->recover = recover;
    arm(s, T_OFF, OFF_NS);
}

static void off_fire(Wco2McuState *s)
{
    if (s->off_stage == 1) {
        if (events_pending(s)) {
            master_on(s);
        }
        if (s->recover) {
            s->off_stage = 2;
            arm(s, T_OFF, OFF_NS);
            return;
        }
    } else if (s->off_stage == 2) {
        master_on(s);
    }
    s->off_stage = 0;
    s->busy = false;
    if (s->wake_pending && s->stage == S_IDLE) {
        s->wake_pending = false;
        session_begin(s);
    }
}

static void shut_off(Wco2McuState *s)
{
    sched(s, 2);
    s->shutoff = true;
    s->imr = 0;
    s->pa5 = false;
    out_set(s, O_MASTER, false);
    out_set(s, O_LED, false);
    out_set(s, O_PIR, false);
    s->pr = 0;
    exti_enable(s, L1 | L2 | L4 | L13 | L14 | L15);
    if (pin_pb(s, PB_USB)) {
        sched(s, 1);
        sched(s, 6);
        sched(s, 7);
        sched(s, 5);
        sched(s, 25);
    }
}

/* ---- RTC -------------------------------------------------------------- */

static int64_t rtc_now(Wco2McuState *s)
{
    return s->rtc_ref_s + (rtc_ns() - s->rtc_ref_ns) / NANOSECONDS_PER_SECOND;
}

static void alarm_arm(Wco2McuState *s, int i)
{
    uint32_t h = s->alarm_reg[i] >> 16 & 0xff;
    uint32_t m = s->alarm_reg[i] >> 8 & 0xff;
    uint32_t sec = s->alarm_reg[i] & 0xff;
    int64_t now, target;

    timer_del(s->timer[T_ALARM_A + i]);
    if (!s->alarm_en[i] || h > 23 || m > 59 || sec > 59) {
        return;
    }
    now = rtc_now(s);
    target = now - now % 86400 + h * 3600 + m * 60 + sec;
    if (target <= now) {
        target += 86400;
    }
    timer_mod(s->timer[T_ALARM_A + i],
              s->rtc_ref_ns + (target - s->rtc_ref_s) * NANOSECONDS_PER_SECOND);
}

static void alarm_fire(Wco2McuState *s, int i)
{
    if (i == 0 || pin_pa(s, PA_PIR)) {
        int_pulse(s);
        s->flags |= EV_ALARM;
        master_on(s);
    }
    alarm_arm(s, i);
}

static void rtc_set_epoch(Wco2McuState *s, int64_t epoch)
{
    s->rtc_ref_s = epoch;
    s->rtc_ref_ns = rtc_ns();
    alarm_arm(s, 0);
    alarm_arm(s, 1);
}

/* ---- the command ------------------------------------------------------ */

static uint8_t crc8(const uint8_t *p, int n)
{
    uint8_t crc = 0;

    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = crc & 1 ? crc >> 1 ^ 0x8c : crc >> 1;
        }
    }
    return crc;
}

/* the 8 wire bytes of a payload, each halfword swapped back */
static uint64_t payload_swapped(const uint8_t *w)
{
    uint8_t b[8];

    for (int i = 0; i < 8; i += 2) {
        b[i] = w[i + 1];
        b[i + 1] = w[i];
    }
    return ldq_le_p(b);
}

static void session_end(Wco2McuState *s)
{
    timer_del(s->timer[T_STEP]);
    s->stage = S_IDLE;
    s->xfer = false;
    if (s->wake_pending && !s->busy) {
        s->wake_pending = false;
        session_begin(s);
    }
}

static void stage_enter(Wco2McuState *s, Stage next)
{
    if (next == S_STATUS && !s->want_status) {
        next = S_DATA;
    }
    if (next == S_DATA && !s->want_data) {
        next = S_PAYLOAD;
    }
    if (next == S_PAYLOAD && s->payload == PK_NONE) {
        next = S_IDLE;
    }
    if (next == S_IDLE) {
        session_end(s);
        return;
    }
    s->stage = next;
    s->rx_len = 0;
    s->tx_pos = 0;
    if (next == S_STATUS) {
        stq_le_p(s->tx, s->status | (uint64_t)s->status_lr << 32);
        s->tx[8] = crc8(s->tx, 8);
    } else if (next == S_DATA) {
        memcpy(s->tx, s->data, 8);
        s->tx[8] = crc8(s->tx, 8);
    }
    arm(s, T_STEP, STEP_NS);
}

static void set_time(Wco2McuState *s, const uint8_t *w)
{
    struct tm tm = {
        .tm_hour = w[3], .tm_min = w[0], .tm_sec = w[1],
        .tm_mday = w[2], .tm_mon = w[5] - 1, .tm_year = w[4] + 100,
    };

    rtc_set_epoch(s, mktimegm(&tm));
}

static void set_alarm(Wco2McuState *s, int i, const uint8_t *w)
{
    uint32_t lo = ldl_le_p(w), hi = ldl_le_p(w + 4);

    if (lo == (i ? 0x00192d23U : 0x1900232dU) && !hi) {
        s->alarm_en[i] = false;
    } else {
        s->alarm_reg[i] = w[3] << 16 | w[0] << 8 | w[1];
        s->alarm_en[i] = true;
    }
    alarm_arm(s, i);
}

static void payload_done(Wco2McuState *s)
{
    const uint8_t *w = s->rx;
    uint64_t v = payload_swapped(w);

    switch (s->payload) {
    case PK_TIME:
        set_time(s, w);
        break;
    case PK_ALARM_A:
        set_alarm(s, 0, w);
        break;
    case PK_ALARM_B:
        set_alarm(s, 1, w);
        break;
    case PK_RAW:
        s->bat_low = ldl_le_p(w);
        break;
    case PK_WDT_PERIOD:
        s->wdt_period = v;
        break;
    case PK_PIR_TIME:
        stq_le_p(s->pir_time_buf, v);
        s->pir_time = v;
        if (!v) {
            exti_enable(s, L_PIR);
            sched(s, 1);
            s->pr &= ~L_PIR;
        } else {
            exti_disable(s, L_PIR);
            sched(s, 1);
            sched(s, 21);
            s->pr &= ~L_PIR;
            s->flags &= ~EV_PIR;
        }
        break;
    case PK_PIR_THR:
        stq_le_p(s->pir_thr_buf, v);
        s->pir_thr = v;
        break;
    case PK_SAMP:
        s->samp = v;
        break;
    case PK_UPPER:
        s->upper = v;
        break;
    case PK_LOWER:
        s->lower = v;
        break;
    case PK_CLEAR:
        s->flags &= ~(uint32_t)v;
        break;
    default:
        break;
    }
}

static void get_time(Wco2McuState *s)
{
    time_t t = rtc_now(s);
    struct tm tm;

    gmtime_r(&t, &tm);
    s->data[0] = tm.tm_sec;
    s->data[1] = tm.tm_min;
    s->data[2] = tm.tm_hour;
    s->data[3] = tm.tm_mday;
    s->data[4] = tm.tm_mon + 1;
    s->data[5] = tm.tm_year - 100;
}

static uint32_t pir_status(Wco2McuState *s)
{
    bool pb5 = s->out[O_PIR];
    bool m7 = s->imr & L7;
    bool m0 = s->imr & L0;

    return !!s->pir_time << 1 | pb5 | m7 << 2 | m0 << 3 |
           (pb5 && !s->pir_time && m7 && m0) << 4;
}

static void dispatch(Wco2McuState *s)
{
    const uint8_t *f = s->rx;
    uint16_t head = f[0] << 8 | f[1];
    uint16_t idx = f[2] << 8 | f[3];
    uint16_t idxn = f[4] << 8 | f[5];
    uint16_t end = f[6] << 8 | f[7];
    int bad = head != HEAD ? 1 : end != TAIL ? 2 :
              idx != (uint16_t)~idxn ? 4 : 0;
    uint8_t *d = s->data;

    trace_wyze_wco2_mcu_cmd(vnow() / SCALE_MS, idx, bad, s->flags);
    memset(d, 0, 8);
    s->want_status = s->want_data = false;
    s->payload = PK_NONE;
    s->status = ST_OK;
    s->status_lr = LR_NORMAL;
    if (bad && ++s->bad_frames >= BAD_FRAMES_MAX) {
        s->status = bad << 16 | ST_BAD;
        s->status_lr = LR_BAD_FRAME;
        s->want_status = true;
        return;
    }
    s->want_status = idx & 0x4000;
    s->want_data = idx & 0x8000;

    switch (idx) {
    case 0x0001:
        master_off(s, false);
        break;
    case 0x0002:
        master_off(s, true);
        break;
    case 0x0003:
        shut_off(s);
        break;
    case 0x4001:
    case 0x400a ... 0x400c:
        break;
    case 0x4002:
        s->flags = 0;
        int_lower(s);
        break;
    case 0x4003:
        out_set(s, O_WIFI, true);
        break;
    case 0x4004:
        out_set(s, O_WIFI, false);
        break;
    case 0x4005:
        wifi_reset(s);
        break;
    case 0x4006:
        s->pir_on = true;
        s->pir_time = 0;
        exti_enable(s, L_PIR);
        sched(s, 1);
        out_set(s, O_PIR, true);
        break;
    case 0x4007:
        s->pir_on = false;
        exti_disable(s, L_PIR);
        sched(s, 1);
        out_set(s, O_PIR, false);
        break;
    case 0x4008:
        s->payload = PK_TIME;
        break;
    case 0x4009:
        s->payload = PK_ALARM_A;
        break;
    case 0x4010:
        s->payload = PK_ALARM_B;
        break;
    case 0x400d:
        out_set(s, O_LED, true);
        break;
    case 0x400e:
        out_set(s, O_LED, false);
        break;
    case 0x400f:
        s->payload = PK_RAW;
        break;
    case 0x4011:
        s->wdt_en = true;
        sched(s, 22);
        break;
    case 0x4012:
        s->wdt_en = false;
        sched(s, 2);
        break;
    case 0x4013:
        s->payload = PK_WDT_PERIOD;
        break;
    case 0x4014:
        s->ctr[2] = 0;
        s->wdt_strikes = 0;
        break;
    case 0x4015:
        s->payload = PK_PIR_TIME;
        break;
    case 0x4016:
        s->payload = PK_PIR_THR;
        break;
    case 0x4017:
        s->payload = PK_SAMP;
        break;
    case 0x4018:
        s->payload = PK_UPPER;
        break;
    case 0x4019:
        s->payload = PK_LOWER;
        break;
    case 0x401a:
        s->payload = PK_CLEAR;
        break;
    case 0x401b:
        s->cw_disabled = false;
        break;
    case 0x401c:
        s->cw_disabled = true;
        break;
    case 0xc001:
        memcpy(d, s->version, MIN(strlen(s->version), 8));
        break;
    case 0xc002:
        stl_le_p(d, s->flags);
        break;
    case 0xc003:
        stl_le_p(d, (!pin_pa(s, PA_BUTTON)) | pin_pa(s, PA_WLWAKE) << 1 |
                    pin_pa(s, PA_PIR) << 2 | (!pin_pb(s, PB_ALERT)) << 3 |
                    pin_pb(s, PB_USB) << 5);
        break;
    case 0xc009:
        s->last_adc = adc_sample(s);
        /* fall through */
    case 0xc004:
        stl_le_p(d, s->last_adc);
        stl_le_p(d + 4, s->first_adc);
        break;
    case 0xc006:
        get_time(s);
        break;
    case 0xc007:
    case 0xc008:
        stl_le_p(d, s->alarm_reg[idx - 0xc007]);
        break;
    case 0xc00a:
        stl_le_p(d, gpiob_idr(s) | gpioa_idr(s) << 16);
        break;
    case 0xc00c:
        memcpy(d, s->pir_time_buf, 8);
        break;
    case 0xc00d:
        memcpy(d, s->pir_thr_buf, 8);
        break;
    case 0xc00e:
        stq_le_p(d, s->samp);
        break;
    case 0xc00f:
        stq_le_p(d, s->upper);
        break;
    case 0xc010:
        stq_le_p(d, s->lower);
        break;
    case 0xc011:
        stl_le_p(d, pir_status(s));
        break;
    default:
        s->status = ST_UNKNOWN;
        s->want_data = false;
        break;
    }
}

static void session_begin(Wco2McuState *s)
{
    if (s->busy || s->stage != S_IDLE) {
        s->wake_pending = true;
        return;
    }
    s->stage = S_FRAME;
    s->rx_len = 0;
    s->xfer = false;
    arm(s, T_STEP, STEP_NS);
}

static void xfer_finish(Wco2McuState *s)
{
    bool ok;

    if (!s->xfer) {
        return;
    }
    s->xfer = false;
    switch (s->stage) {
    case S_FRAME:
    case S_PAYLOAD:
        ok = !s->xfer_read && s->rx_len >= 8;
        break;
    case S_STATUS:
    case S_DATA:
        ok = s->xfer_read ? s->tx_pos >= 9 : true;
        break;
    default:
        return;
    }
    if (s->bad_dir || !ok) {
        s->bad_dir = false;
        session_end(s);
        return;
    }
    if (s->stage == S_FRAME) {
        dispatch(s);
        if (s->busy) {
            s->want_status = s->want_data = false;
            s->payload = PK_NONE;
        }
        stage_enter(s, S_STATUS);
    } else if (s->stage == S_PAYLOAD) {
        payload_done(s);
        session_end(s);
    } else {
        stage_enter(s, s->stage == S_STATUS ? S_DATA : S_PAYLOAD);
    }
}

static int wco2_mcu_event(I2CSlave *i2c, enum i2c_event event)
{
    Wco2McuState *s = WYZE_WCO2_MCU(i2c);

    switch (event) {
    case I2C_START_RECV:
    case I2C_START_SEND:
        if (s->stage == S_IDLE) {
            trace_wyze_wco2_mcu_nak(0);
            return 1;
        }
        xfer_finish(s);
        if (s->stage == S_IDLE) {
            trace_wyze_wco2_mcu_nak(1);
            return 1;
        }
        s->xfer = true;
        s->xfer_read = event == I2C_START_RECV;
        s->rx_len = 0;
        s->tx_pos = 0;
        s->bad_dir = s->xfer_read != (s->stage == S_STATUS ||
                                      s->stage == S_DATA);
        if (s->bad_dir && s->xfer_read) {
            return 0;
        }
        return 0;
    case I2C_FINISH:
        xfer_finish(s);
        return 0;
    case I2C_START_SEND_ASYNC:
        return 1;
    default:
        return 0;
    }
}

static int wco2_mcu_send(I2CSlave *i2c, uint8_t data)
{
    Wco2McuState *s = WYZE_WCO2_MCU(i2c);

    if (!s->xfer) {
        return 1;
    }
    if (s->rx_len < 8) {
        s->rx[s->rx_len] = data;
    } else if (s->rx_len > 8) {
        return 1;
    }
    s->rx_len++;
    return 0;
}

static uint8_t wco2_mcu_recv(I2CSlave *i2c)
{
    Wco2McuState *s = WYZE_WCO2_MCU(i2c);

    if (!s->xfer || s->bad_dir || s->tx_pos >= 9) {
        return 0xff;
    }
    return s->tx[s->tx_pos++];
}

/* ---- EXTI handlers ---------------------------------------------------- */

static void h_pir(Wco2McuState *s)
{
    exti_disable(s, L_PIR);
    s->pr &= ~L_PIR;
    sched(s, 26);
}

static void h_line1_body(Wco2McuState *s)
{
    if (!pin_pb(s, PB_ALERT)) {
        int_pulse(s);
        s->flags |= EV_BAT;
        if (pin_pa(s, PA_SWITCH)) {
            master_on(s);
        }
    }
}

static void h_line4_body(Wco2McuState *s)
{
    s->shutoff = false;
    if (pin_pb(s, PB_USB)) {
        s->c1a = s->c1b = 0;
        out_set(s, O_CHG_LED, false);
        s->flags |= EV_USB;
        out_set(s, O_LED, false);
        sched(s, 23);
        int_pulse(s);
        if (!s->cw_disabled) {
            out_set(s, O_LED, true);
        }
        if (pin_pa(s, PA_SWITCH) && !s->cw_disabled) {
            master_on(s);
        }
    }
    if (!pin_pb(s, PB_USB)) {
        s->c1b = 0;
        out_set(s, O_CHG_LED, true);
        int_pulse(s);
        s->flags |= EV_USB;
        sched(s, 3);
        out_set(s, O_LED, false);
        if (pin_pa(s, PA_SWITCH)) {
            if (!s->cw_disabled) {
                master_on(s);
            }
        } else {
            out_set(s, O_PIR, false);
            exti_disable(s, L_PIR | L2 | L3 | L13 | L14 | L15);
            exti_enable(s, L13);
            sched(s, 0);
            master_off(s, false);
        }
    }
    s->pr &= ~L4;
    exti_enable(s, L4);
}

static void h_line13_body(Wco2McuState *s)
{
    exti_enable(s, L13);
    if (pin_pb(s, PB_USB)) {
        s->shutoff = false;
        if (pin_pa(s, PA_CHG_STAT)) {
            sched(s, 3);
            out_set(s, O_LED, true);
        } else {
            sched(s, 23);
        }
        sched(s, 25);
    }
    if (pin_pa(s, PA_SWITCH)) {
        if (s->shutoff) {
            sched(s, 3);
            led_blink_end(s);
            exti_enable(s, L4 | L15);
        } else {
            master_on(s);
            wifi_reset(s);
            exti_enable(s, L0 | L2 | L3 | L13 | L14 | L15);
            exti_disable(s, L7);
        }
    } else {
        s->flags = 0;
        sched(s, 2);
        out_set(s, O_MASTER, false);
        out_set(s, O_WIFI, false);
        out_set(s, O_PIR, false);
        arm(s, T_L13B, 1000 * SCALE_MS);
        return;
    }
    s->pr &= ~L13;
}

static void h_line13b_body(Wco2McuState *s)
{
    if (!pin_pb(s, PB_USB)) {
        exti_disable(s, L_PIR | L2 | L3 | L13 | L14 | L15);
        exti_enable(s, L13);
        sched(s, 0);
        master_off(s, false);
    }
    exti_enable(s, L4 | L14);
    s->pr &= ~L13;
}

static void h_line14_body(Wco2McuState *s)
{
    if (pin_pb(s, PB_USB)) {
        if (pin_pa(s, PA_CHG_STAT)) {
            sched(s, 24);
            s->c1a = 0;
        } else {
            s->c1b++;
            sched(s, 4);
            if (++s->c1a >= 11) {
                if (!s->cw_disabled) {
                    int_pulse(s);
                }
                s->flags |= EV_CHG_ERR;
                s->c1a = 0;
            }
            tick_arm(s);
        }
    } else {
        sched(s, 4);
    }
    s->pr &= ~L14;
    exti_enable(s, L14);
}

static void h_line15_body(Wco2McuState *s)
{
    if (!pin_pa(s, PA_BUTTON)) {
        s->presses++;
        sched(s, 27);
    }
    s->pr &= ~L15;
    exti_enable(s, L15);
}

static void line_delayed(Wco2McuState *s, uint32_t line, TimerId t,
                         int64_t ms, uint32_t disable)
{
    s->pr &= ~line;
    exti_disable(s, disable);
    arm(s, t, ms * SCALE_MS);
}

static void exti_service(Wco2McuState *s)
{
    uint32_t act;

    if (s->servicing) {
        return;
    }
    s->servicing = true;
    while ((act = s->pr & s->imr)) {
        if (act & L0 || act & L7) {
            h_pir(s);
        } else if (act & L1) {
            if (timer_pending(s->timer[T_L1])) {
                break;
            }
            line_delayed(s, L1, T_L1, 100, 0);
        } else if (act & L2) {
            s->pr &= ~L2;
            int_pulse(s);
            s->flags |= EV_WIFI;
            master_on(s);
        } else if (act & L3) {
            s->pr &= ~L3;
            session_begin(s);
        } else if (act & L4) {
            if (timer_pending(s->timer[T_L4])) {
                break;
            }
            line_delayed(s, L4, T_L4, 20, L4);
        } else if (act & L13) {
            if (timer_pending(s->timer[T_L13]) ||
                timer_pending(s->timer[T_L13B])) {
                break;
            }
            line_delayed(s, L13, T_L13, 100, L_ALL & ~L4 & ~L3);
            s->imr &= ~(L0 | L7 | L2 | L3 | L13 | L14 | L15);
        } else if (act & L14) {
            line_delayed(s, L14, T_L14, 5, L14);
        } else if (act & L15) {
            line_delayed(s, L15, T_L15, 5, L15);
        } else {
            break;
        }
    }
    s->servicing = false;
}

static void wco2_mcu_timer(void *opaque)
{
    Wco2McuTimer *t = opaque;
    Wco2McuState *s = t->s;

    switch (t->id) {
    case T_STEP:
        session_end(s);
        break;
    case T_INT:
        out_set(s, O_INT, false);
        break;
    case T_TICK:
        tick_fire(s);
        break;
    case T_OFF:
        off_fire(s);
        break;
    case T_WIFI:
        out_set(s, O_WIFI, true);
        break;
    case T_L1:
        h_line1_body(s);
        break;
    case T_L4:
        h_line4_body(s);
        break;
    case T_L13:
        h_line13_body(s);
        break;
    case T_L13B:
        h_line13b_body(s);
        break;
    case T_L14:
        h_line14_body(s);
        break;
    case T_L15:
        h_line15_body(s);
        break;
    case T_ALARM_A:
        alarm_fire(s, 0);
        break;
    case T_ALARM_B:
        alarm_fire(s, 1);
        break;
    default:
        break;
    }
    exti_service(s);
}

/* ---- properties and wiring -------------------------------------------- */

static void in_get(Object *obj, Visitor *v, const char *name, void *opaque,
                   Error **errp)
{
    Wco2McuState *s = WYZE_WCO2_MCU(obj);
    const InPin *p = opaque;
    bool level = p->port_b ? pin_pb(s, p->bit) : pin_pa(s, p->bit);
    bool val = level != p->invert;

    visit_type_bool(v, name, &val, errp);
}

static void in_set(Object *obj, Visitor *v, const char *name, void *opaque,
                   Error **errp)
{
    Wco2McuState *s = WYZE_WCO2_MCU(obj);
    const InPin *p = opaque;
    bool val;

    if (visit_type_bool(v, name, &val, errp)) {
        pin_set(s, p->port_b, p->bit, val != p->invert);
    }
}

static void out_get(Object *obj, Visitor *v, const char *name, void *opaque,
                    Error **errp)
{
    bool val = WYZE_WCO2_MCU(obj)->out[(uintptr_t)opaque];

    visit_type_bool(v, name, &val, errp);
}

static void wco2_mcu_done(Notifier *n, void *data)
{
    Wco2McuState *s = container_of(n, Wco2McuState, done);
    DeviceState *dev = DEVICE(s);
    DeviceState *wifi = s->wifi;
    bool ambiguous;

    if (!s->gpio) {
        Object *o = object_resolve_path_type("", TYPE_INGENIC_GPIO,
                                             &ambiguous);

        s->gpio = o && !ambiguous ? DEVICE(o) : NULL;
    }
    if (!wifi) {
        Object *o = object_resolve_path_type("", "bcm-sdio", &ambiguous);

        wifi = o && !ambiguous ? DEVICE(o) : NULL;
    }
    if (wifi) {
        qdev_connect_gpio_out_named(dev, "wifi-power", 0,
                                    qdev_get_gpio_in_named(wifi, "reg-on", 0));
    }
    if (s->gpio) {
        if (s->wake_pin >= 0) {
            qdev_connect_gpio_out_named(s->gpio, "pad", s->wake_pin,
                                        qdev_get_gpio_in_named(dev, "wake",
                                                               0));
        }
        if (s->int_pin >= 0) {
            qdev_connect_gpio_out_named(dev, "int", 0,
                                        qdev_get_gpio_in_named(s->gpio, "pull",
                                                               s->int_pin));
            qemu_set_irq(s->irq[O_INT], s->out[O_INT]);
        }
        if (s->wifi_pin >= 0 && !wifi) {
            qdev_connect_gpio_out_named(dev, "wifi-power", 0,
                                        qdev_get_gpio_in_named(s->gpio, "pull",
                                                               s->wifi_pin));
            qemu_set_irq(s->irq[O_WIFI], s->out[O_WIFI]);
        }
    }
    if (wifi) {
        qemu_set_irq(s->irq[O_WIFI], s->out[O_WIFI]);
    }
}

static void wco2_mcu_realize(DeviceState *dev, Error **errp)
{
    Wco2McuState *s = WYZE_WCO2_MCU(dev);
    I2CSlave *i2c = I2C_SLAVE(dev);

    if (!i2c->address) {
        i2c->address = 0x11;
    }
    if (strlen(s->version) > 8) {
        error_setg(errp, "%s: version is at most 8 characters",
                   TYPE_WYZE_WCO2_MCU);
        return;
    }
    s->rtc_ref_s = qemu_clock_get_ns(QEMU_CLOCK_HOST) / NANOSECONDS_PER_SECOND;
    s->rtc_ref_ns = rtc_ns();
    s->done.notify = wco2_mcu_done;
    qemu_add_machine_init_done_notifier(&s->done);
}

static void wco2_mcu_unrealize(DeviceState *dev)
{
    notifier_remove(&WYZE_WCO2_MCU(dev)->done);
}

static void wco2_mcu_init(Object *obj)
{
    Wco2McuState *s = WYZE_WCO2_MCU(obj);
    DeviceState *dev = DEVICE(obj);

    for (int i = 0; i < T_NUM; i++) {
        s->tid[i] = (Wco2McuTimer){ s, i };
        s->timer[i] = timer_new_ns(i == T_ALARM_A || i == T_ALARM_B ?
                                   rtc_clock : QEMU_CLOCK_VIRTUAL,
                                   wco2_mcu_timer,
                                   &s->tid[i]);
    }
    for (int i = 0; i < O_NUM; i++) {
        qdev_init_gpio_out_named(dev, &s->irq[i], outs[i].gpio, 1);
        object_property_add(obj, outs[i].prop, "bool", out_get, NULL, NULL,
                            (void *)(uintptr_t)i);
    }
    qdev_init_gpio_in_named(dev, wco2_mcu_wake_in, "wake", 1);
    for (int i = 0; i < ARRAY_SIZE(ins); i++) {
        object_property_add(obj, ins[i].name, "bool", in_get, in_set, NULL,
                            (void *)&ins[i]);
        if (ins[i].dflt) {
            if (ins[i].port_b) {
                s->pb_in |= BIT(ins[i].bit);
            } else {
                s->pa_in |= BIT(ins[i].bit);
            }
        }
    }
    object_property_add_uint32_ptr(obj, "pir-adc", &s->pir_adc,
                                   OBJ_PROP_FLAG_READWRITE);
    s->pir_adc = 2048;

    s->version = g_strdup("02.01.09");
    s->wdt_period = 30;
    s->pir_time = 180;
    s->samp = 2000;
    s->upper = 3000;
    s->lower = 1000;
    s->bat_low = 2160;
    s->pir_thr = 50;
    s->c1c = 1;
    s->i2c_pads = true;
    s->pa5 = true;
    s->imr = L1 | L2 | L3 | L4 | L13 | L14 | L15;
    s->out[O_WIFI] = true;
    s->out[O_MASTER] = true;
    s->out[O_CHG_LED] = true;
}

static const VMStateDescription vmstate_wco2_mcu = {
    .name = TYPE_WYZE_WCO2_MCU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Wco2McuState),
        VMSTATE_TIMER_PTR_ARRAY(timer, Wco2McuState, T_NUM),
        VMSTATE_UINT32(pir_adc, Wco2McuState),
        VMSTATE_UINT16(pa_in, Wco2McuState),
        VMSTATE_UINT16(pb_in, Wco2McuState),
        VMSTATE_BOOL_ARRAY(out, Wco2McuState, O_NUM),
        VMSTATE_BOOL(pa5, Wco2McuState),
        VMSTATE_BOOL(i2c_pads, Wco2McuState),
        VMSTATE_UINT32(imr, Wco2McuState),
        VMSTATE_UINT32(pr, Wco2McuState),
        VMSTATE_UINT32(flags, Wco2McuState),
        VMSTATE_UINT8(mask, Wco2McuState),
        VMSTATE_UINT32(ticks, Wco2McuState),
        VMSTATE_UINT64_ARRAY(ctr, Wco2McuState, 8),
        VMSTATE_UINT64(wdt_period, Wco2McuState),
        VMSTATE_UINT64(pir_time, Wco2McuState),
        VMSTATE_UINT64(samp, Wco2McuState),
        VMSTATE_UINT64(upper, Wco2McuState),
        VMSTATE_UINT64(lower, Wco2McuState),
        VMSTATE_UINT32(wdt_strikes, Wco2McuState),
        VMSTATE_UINT32(last_adc, Wco2McuState),
        VMSTATE_UINT32(first_adc, Wco2McuState),
        VMSTATE_BOOL(first_hit, Wco2McuState),
        VMSTATE_BOOL(wdt_en, Wco2McuState),
        VMSTATE_BOOL(cw_disabled, Wco2McuState),
        VMSTATE_BOOL(shutoff, Wco2McuState),
        VMSTATE_BOOL(pir_on, Wco2McuState),
        VMSTATE_UINT8(presses, Wco2McuState),
        VMSTATE_UINT8(c1a, Wco2McuState),
        VMSTATE_UINT8(c1b, Wco2McuState),
        VMSTATE_UINT8(c1c, Wco2McuState),
        VMSTATE_UINT16(c22, Wco2McuState),
        VMSTATE_UINT32(bad_frames, Wco2McuState),
        VMSTATE_UINT32(bat_low, Wco2McuState),
        VMSTATE_UINT32(pir_thr, Wco2McuState),
        VMSTATE_UINT8_ARRAY(pir_time_buf, Wco2McuState, 8),
        VMSTATE_UINT8_ARRAY(pir_thr_buf, Wco2McuState, 8),
        VMSTATE_BOOL(busy, Wco2McuState),
        VMSTATE_UINT8(off_stage, Wco2McuState),
        VMSTATE_BOOL(recover, Wco2McuState),
        VMSTATE_INT64(rtc_ref_s, Wco2McuState),
        VMSTATE_INT64(rtc_ref_ns, Wco2McuState),
        VMSTATE_UINT32_ARRAY(alarm_reg, Wco2McuState, 2),
        VMSTATE_BOOL_ARRAY(alarm_en, Wco2McuState, 2),
        VMSTATE_UINT32(stage, Wco2McuState),
        VMSTATE_BOOL(wake_pending, Wco2McuState),
        VMSTATE_BOOL(xfer, Wco2McuState),
        VMSTATE_BOOL(xfer_read, Wco2McuState),
        VMSTATE_BOOL(bad_dir, Wco2McuState),
        VMSTATE_UINT8_ARRAY(rx, Wco2McuState, 8),
        VMSTATE_UINT8(rx_len, Wco2McuState),
        VMSTATE_UINT8_ARRAY(tx, Wco2McuState, 9),
        VMSTATE_UINT8(tx_pos, Wco2McuState),
        VMSTATE_BOOL(want_status, Wco2McuState),
        VMSTATE_BOOL(want_data, Wco2McuState),
        VMSTATE_UINT32(payload, Wco2McuState),
        VMSTATE_UINT8_ARRAY(data, Wco2McuState, 8),
        VMSTATE_UINT32(status, Wco2McuState),
        VMSTATE_UINT32(status_lr, Wco2McuState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property wco2_mcu_props[] = {
    DEFINE_PROP_LINK("gpio", Wco2McuState, gpio, TYPE_DEVICE, DeviceState *),
    DEFINE_PROP_LINK("wifi", Wco2McuState, wifi, TYPE_DEVICE, DeviceState *),
    DEFINE_PROP_INT32("wake-pin", Wco2McuState, wake_pin, 53),
    DEFINE_PROP_INT32("int-pin", Wco2McuState, int_pin, 10),
    DEFINE_PROP_INT32("wifi-pin", Wco2McuState, wifi_pin, -1),
    DEFINE_PROP_STRING("version", Wco2McuState, version),
    DEFINE_PROP_BOOL("quit-on-power-off", Wco2McuState, quit_on_power_off,
                     false),
};

static void wco2_mcu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->desc = "Zeratul board power-management MCU";
    dc->realize = wco2_mcu_realize;
    dc->unrealize = wco2_mcu_unrealize;
    dc->vmsd = &vmstate_wco2_mcu;
    device_class_set_props(dc, wco2_mcu_props);
    k->event = wco2_mcu_event;
    k->recv = wco2_mcu_recv;
    k->send = wco2_mcu_send;
}

static const TypeInfo wco2_mcu_info = {
    .name = TYPE_WYZE_WCO2_MCU,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Wco2McuState),
    .instance_init = wco2_mcu_init,
    .class_init = wco2_mcu_class_init,
};

static void wco2_mcu_register_types(void)
{
    type_register_static(&wco2_mcu_info);
}

type_init(wco2_mcu_register_types)
