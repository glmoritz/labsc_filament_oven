/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bench diagnostics thread — status LED and per-cycle PID trace.
 *
 * BENCH ONLY. Compiled only when CONFIG_OVEN_DEBUG is set, which only debug.conf
 * does. Nothing in here has a safety role and nothing in here is allowed to
 * acquire one.
 *
 * --- How it stays out of the safety path ---------------------------------
 *
 *   Priority. This thread is PREEMPTIBLE (OVEN_DEBUG_PRIO). The PID and Output
 *   threads are cooperative and both outrank it, so either of them preempts it
 *   at any instruction and it can never delay a control cycle or a keep-alive
 *   pulse. It sits numerically above the Watchdog only because the Watchdog must
 *   remain the system's starvation canary; it spends effectively all of its time
 *   blocked (on the cycle semaphore, then on timed sleeps) so it has no way to
 *   starve anything.
 *
 *   Data. It never touches controller state. It reads one seqlock-protected
 *   snapshot per cycle through pid_pub_snapshot() and works entirely from that
 *   copy. A torn read is dropped, not retried forever.
 *
 *   Output. Logging is DEFERRED (CONFIG_LOG_MODE_DEFERRED, set in prj.conf), so
 *   LOG_INF here only packs arguments into the log ring buffer; the formatting
 *   and the UART writes happen later on the logging thread, off this path.
 *
 *   Faults. It raises none and clears none. If the LED device is missing it says
 *   so once and carries on; a dead indicator is not a safety event.
 *
 * --- The LED ---------------------------------------------------------------
 *
 * One WS2812 on the module, driven over I2S (see debug/<board>.overlay). Each
 * control cycle is drawn as:
 *
 *     white flash            marks the start of the cycle
 *     red, proportional      commanded heater power as a fraction of the
 *                            remainder of the cycle
 *     dark                   the rest
 *
 *     power = 100%  |WWWWW|RRRRRRRRRRRRRRRRRRRR|
 *     power =  50%  |WWWWW|RRRRRRRRRR..........|
 *     power =   0%  |WWWWW|....................|
 *                   0   100ms              500ms
 *
 * So the LED shows commanded power, not delivered power: it follows the PID's
 * output even while DISARMED, when the Output thread is (correctly) refusing to
 * energize anything. That is intentional — it is how you watch the loop settle
 * on the bench without a live heater.
 */

#include "shared.h"
#include "oven_threads.h"
#include "oven_debug.h"
#include "pid.h"
#include "pid_pub.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_OVEN_DEBUG_LED)
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#endif

LOG_MODULE_REGISTER(oven_dbg, LOG_LEVEL_INF);

/*
 * How long to wait for a control cycle before deciding the loop is not running.
 * Four control periods: long enough that a single late cycle is not reported,
 * short enough to notice a stalled or never-started loop.
 */
#define DBG_CYCLE_TIMEOUT_MS   (OVEN_PID_PERIOD_MS * 4U)

/* ---- Status LED --------------------------------------------------------- */

#if defined(CONFIG_OVEN_DEBUG_LED)

BUILD_ASSERT(CONFIG_OVEN_DEBUG_LED_FLASH_MS < (int)OVEN_PID_PERIOD_MS,
	     "cycle-start flash must be shorter than the control period");

/* Milliseconds left for the red power bar after the white flash. */
#define DBG_RED_WINDOW_MS  ((uint32_t)OVEN_PID_PERIOD_MS - \
			    (uint32_t)CONFIG_OVEN_DEBUG_LED_FLASH_MS)

#define DBG_LVL            ((uint8_t)CONFIG_OVEN_DEBUG_LED_LEVEL)

static const struct device *const s_led = DEVICE_DT_GET(DT_ALIAS(debug_led));
static bool s_led_ok;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
	struct led_rgb px;

	if (!s_led_ok) {
		return;
	}

	px.r = r;
	px.g = g;
	px.b = b;

	/* A failed LED update is not worth a log line every 500 ms. */
	(void)led_strip_update_rgb(s_led, &px, 1);
}

/*
 * Draw one control cycle. Total elapsed time is the white flash plus the red
 * bar, which is at most exactly one control period, so this returns in time to
 * wait for the next cycle.
 */
static void led_draw_cycle(uint32_t power)
{
	uint32_t red_ms;

	led_set(DBG_LVL, DBG_LVL, DBG_LVL);          /* white: cycle start */
	k_sleep(K_MSEC(CONFIG_OVEN_DEBUG_LED_FLASH_MS));

	power = MIN(power, (uint32_t)OVEN_HEATER_MAX_POWER);
	red_ms = (DBG_RED_WINDOW_MS * power) / (uint32_t)OVEN_HEATER_MAX_POWER;

	if (red_ms > 0U) {
		led_set(DBG_LVL, 0, 0);              /* red: commanded power */
		k_sleep(K_MSEC(red_ms));
	}

	led_set(0, 0, 0);
}

static void led_init(void)
{
	s_led_ok = device_is_ready(s_led);
	if (!s_led_ok) {
		LOG_WRN("status LED not ready — trace continues without it");
		return;
	}
	led_set(0, 0, 0);
}

#else  /* !CONFIG_OVEN_DEBUG_LED */

static inline void led_draw_cycle(uint32_t power)
{
	ARG_UNUSED(power);
	k_sleep(K_MSEC(OVEN_PID_PERIOD_MS));
}

static inline void led_init(void)
{
}

#endif /* CONFIG_OVEN_DEBUG_LED */

/* ---- Per-cycle trace ---------------------------------------------------- */

#if defined(CONFIG_OVEN_DEBUG_PID_TRACE)

/*
 * ---- Trace format ----
 *
 * ONE ROW PER CONTROL CYCLE, comma-separated, so a capture is a CSV you can
 * plot without parsing prose. Every row is self-contained (the gains ride along
 * rather than being printed only when they change), so a capture that starts
 * mid-run is still complete.
 *
 * Two prefixes, both greppable, deliberately distinct:
 *
 *   pidcsv#   header / units — comment rows, never data
 *   pidcsv,   one control cycle
 *
 * To capture:  ... | grep -o 'pidcsv,.*' > run.csv
 *
 * Only integers, in the units the firmware already holds — no decimal point, no
 * sign character, no locale, nothing for a plotting script to undo. Scale on the
 * way in to the plot, not on the way out of the firmware.
 */
#define TRACE_COLUMNS \
	"cycle,dt_ms,act,sp_cc,pv_cc,e_cc,dpv_cc,kp_m,ki_m,kd_m," \
	"p_mhc,i_mhc,d_mhc,sum_mhc,out_hc,sat,retune,armedge"

/*
 * Reprint the header periodically as well as at boot. A capture almost never
 * starts at boot, and a CSV whose columns are only described in a line you
 * missed is a CSV you have to guess at. The '#' prefix keeps these rows out of
 * the data, so re-emitting them costs nothing downstream.
 */
#define TRACE_HEADER_EVERY_CYCLES   (240U)     /* ~2 min at a 500 ms period */

static uint32_t s_rows_since_header;

/*
 * Signals come out of the control law in its internal scale
 * (OVEN_PID_TRACE_SCALE per half-cycle). Publish them as MILLI half-cycles:
 * three decimal digits of resolution with no fractional formatting.
 *
 * int64 on purpose. The proportional term is bounded by the gain contract at
 * kp_m(1e6) * e(3.2e4) * 1e3 = 3.2e16 internal units, which is 3.2e13 milli
 * half-cycles — far outside int32. The actuator command is clamped to 120, but
 * the UNSATURATED sum deliberately is not, and seeing how far past the rail the
 * law actually asked for is most of the point of tracing it.
 */
static inline int64_t u_milli(int64_t u)
{
	return u / (OVEN_PID_TRACE_SCALE / 1000);
}

static void trace_header(void)
{
	LOG_INF("pidcsv# labsc_filament_oven PID trace — one row per control cycle");
	LOG_INF("pidcsv# units: _cc=centi-degC  _m=gain x1000  "
		"_mhc=milli half-cycles  out_hc=half-cycles of %u max",
		(uint32_t)OVEN_HEATER_MAX_POWER);
	LOG_INF("pidcsv# sat: -1=floor(0) 0=linear +1=ceiling; non-zero means the "
		"integrator was pinned by anti-windup");
	LOG_INF("pidcsv#" TRACE_COLUMNS);

	s_rows_since_header = 0U;
}

static void trace_cycle(const struct pid_pub *p)
{
	const struct pid_diag *d = &p->diag;

	if (s_rows_since_header >= TRACE_HEADER_EVERY_CYCLES) {
		trace_header();
	}
	s_rows_since_header++;

	LOG_INF("pidcsv,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,"
		"%lld,%lld,%lld,%lld,%u,%d,%d,%d",
		p->cycle, p->delta_ms, (int)d->active,
		d->sp_cc, d->pv_cc, d->e_cc, d->dpv_cc,
		d->g.kp_m, d->g.ki_m, d->g.kd_m,
		u_milli(d->yp_u), u_milli(d->yi_u), u_milli(d->yd_u),
		u_milli(d->y_unsat_u),
		d->out, (int)d->sat, (int)d->retuned, (int)d->arm_edge);
}

#else  /* !CONFIG_OVEN_DEBUG_PID_TRACE */

static inline void trace_header(void)
{
}

static inline void trace_cycle(const struct pid_pub *p)
{
	ARG_UNUSED(p);
}

#endif /* CONFIG_OVEN_DEBUG_PID_TRACE */

/* ---- Bench preset ------------------------------------------------------- */

#if defined(CONFIG_OVEN_DEBUG_PRESET)

BUILD_ASSERT(CONFIG_OVEN_DEBUG_SETPOINT_CC >= OVEN_TEMP_VALID_MIN_CC &&
	     CONFIG_OVEN_DEBUG_SETPOINT_CC <= OVEN_TEMP_VALID_MAX_CC,
	     "bench setpoint is outside the plausibility window");
BUILD_ASSERT(CONFIG_OVEN_DEBUG_SETPOINT_CC < OVEN_SENSOR_B_ABS_LIMIT_CC,
	     "bench setpoint is at or above the protection-zone absolute limit");
BUILD_ASSERT(CONFIG_OVEN_DEBUG_KP_M >= OVEN_PID_GAIN_M_MIN &&
	     CONFIG_OVEN_DEBUG_KP_M <= OVEN_PID_GAIN_M_MAX,
	     "bench kp is outside the gain contract");
BUILD_ASSERT(CONFIG_OVEN_DEBUG_KI_M >= OVEN_PID_GAIN_M_MIN &&
	     CONFIG_OVEN_DEBUG_KI_M <= OVEN_PID_GAIN_M_MAX,
	     "bench ki is outside the gain contract");
BUILD_ASSERT(CONFIG_OVEN_DEBUG_KD_M >= OVEN_PID_GAIN_M_MIN &&
	     CONFIG_OVEN_DEBUG_KD_M <= OVEN_PID_GAIN_M_MAX,
	     "bench kd is outside the gain contract");

static void apply_preset(void)
{
	(void)atomic_set(&g_setpoint_cc, (atomic_val_t)CONFIG_OVEN_DEBUG_SETPOINT_CC);
	(void)atomic_set(&g_kp_m, (atomic_val_t)CONFIG_OVEN_DEBUG_KP_M);
	(void)atomic_set(&g_ki_m, (atomic_val_t)CONFIG_OVEN_DEBUG_KI_M);
	(void)atomic_set(&g_kd_m, (atomic_val_t)CONFIG_OVEN_DEBUG_KD_M);

	/* Same units as the trace columns, so this line reads like a data row. */
	LOG_WRN("BENCH PRESET: sp_cc=%d kp_m=%d ki_m=%d kd_m=%d "
		"— this image is NOT a field build",
		(int)CONFIG_OVEN_DEBUG_SETPOINT_CC, (int)CONFIG_OVEN_DEBUG_KP_M,
		(int)CONFIG_OVEN_DEBUG_KI_M, (int)CONFIG_OVEN_DEBUG_KD_M);
}

#else

static inline void apply_preset(void)
{
}

#endif /* CONFIG_OVEN_DEBUG_PRESET */

/* ---- Thread ------------------------------------------------------------- */

static void debug_thread_entry(void *p1, void *p2, void *p3)
{
	bool stalled = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	__ASSERT(k_thread_priority_get(k_current_get()) == OVEN_DEBUG_PRIO,
		 "Debug thread priority mismatch");

	while (atomic_get(&g_init_done) == 0) {
		k_sleep(K_MSEC(1));
	}

	led_init();
	trace_header();

	for (;;) {
		struct pid_pub pub;

		if (!pid_pub_wait(K_MSEC(DBG_CYCLE_TIMEOUT_MS))) {
			/*
			 * No control cycle for four periods. Report it once and
			 * keep waiting — detecting this is the Watchdog's job,
			 * not ours; we only make it visible.
			 */
			if (!stalled) {
				LOG_WRN("no PID cycle in %u ms — loop stalled or "
					"not started", (uint32_t)DBG_CYCLE_TIMEOUT_MS);
				stalled = true;
			}
			continue;
		}
		stalled = false;

		if (!pid_pub_snapshot(&pub)) {
			/* Writer kept winning; drop this cycle, not the thread. */
			continue;
		}

		trace_cycle(&pub);
		led_draw_cycle(pub.diag.out);
	}
}

K_THREAD_DEFINE(oven_debug_tid, OVEN_DEBUG_STACK_SIZE, debug_thread_entry,
		NULL, NULL, NULL, OVEN_DEBUG_PRIO, 0, 0);

void oven_debug_init(void)
{
	apply_preset();
}
