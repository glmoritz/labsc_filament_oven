/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Watchdog thread (architecture, "Watchdog Thread"). Runs at the LOWEST
 * priority in the system, so it starves first: if higher-priority threads
 * monopolise the CPU it stops running and the external hardware watchdog
 * expires. That starvation is the detector.
 *
 * It produces the diversified second pulse train (architecture, "Diversified
 * pulse trains"): the hw-wdt-out health pulse feeds BOTH the external hardware
 * watchdog (layer 3) and the contactor-enable missing-pulse detector (layer 5).
 * That pulse is gated on OVERALL system health AND a clear fault word, so
 * "Output thread alive but everything else dead" still drops the contactor.
 *
 * On-chip watchdog (optional, compiled in only when a `watchdog0` alias exists)
 * is fed on LIVENESS ONLY, not gated on the fault word — so a latched fault
 * reboots into FAULTED once and then STAYS there with telemetry alive, instead
 * of reboot-looping. It resets the chip only on a genuine firmware hang.
 */

#include "shared.h"
#include "oven_threads.h"
#include "oven_state.h"
#include "contactor.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#if DT_NODE_EXISTS(DT_ALIAS(watchdog0))
#include <zephyr/drivers/watchdog.h>
#define OVEN_HAS_ONCHIP_WDT 1
#else
#define OVEN_HAS_ONCHIP_WDT 0
#endif

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

#define WD_PULSE_WIDTH_US        (100U)

/*
 * Interlock quiet window: how long this thread stays SILENT at startup before it
 * is willing to enable the contactor at all.
 *
 * The interlock cannot hold the contactor closed without this pulse train, so a
 * contactor still reading closed after a full second of silence has not been
 * held there by us — it is welded, stuck, or the feedback is lying. This is the
 * weld test, and it runs at every single boot.
 */
#define WD_QUIET_WINDOW_MS       (1000U)

/*
 * Drop-out deadline. In normal operation the contactor STAYS closed; the pulse
 * train is a health signal, not the arm actuator. But when health is lost and we
 * stop pulsing, the contactor must be observed OPEN within this deadline. Still
 * closed after it = welded.
 */
#define WD_DROPOUT_CYCLES \
	(OVEN_CONTACTOR_OPEN_GRACE_MS / OVEN_WATCHDOG_PERIOD_MS)
#define WD_OUTPUT_STALE_CYCLES   (2U)   /* 200 ms — Output runs at 120 Hz    */
#define WD_PID_STALE_CYCLES      (8U)   /* 800 ms — PID runs at 2 Hz          */
#define WD_TIMER_STALE_CYCLES    (8U)   /* 800 ms — release timer at 2 Hz     */

static const struct gpio_dt_spec s_hw_wdt =
	GPIO_DT_SPEC_GET(DT_ALIAS(hw_wdt_out), gpios);

#if OVEN_HAS_ONCHIP_WDT
static const struct device *const s_wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int s_wdt_channel = -1;

static void onchip_wdt_setup(void)
{
	struct wdt_timeout_cfg cfg = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = 0U,
		.window.max = 2000U,   /* generous; fed every 100 ms when alive */
		.callback = NULL,
	};

	if (device_is_ready(s_wdt)) {
		s_wdt_channel = wdt_install_timeout(s_wdt, &cfg);
		if (s_wdt_channel >= 0) {
			(void)wdt_setup(s_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
		}
	}
}

static void onchip_wdt_feed(void)
{
	if (s_wdt_channel >= 0) {
		(void)wdt_feed(s_wdt, s_wdt_channel);
	}
}
#else
static void onchip_wdt_setup(void) { }
static void onchip_wdt_feed(void) { }
#endif /* OVEN_HAS_ONCHIP_WDT */

static void emit_hw_wdt_pulse(void)
{
	(void)gpio_pin_set_dt(&s_hw_wdt, 1);
	k_busy_wait(WD_PULSE_WIDTH_US);
	(void)gpio_pin_set_dt(&s_hw_wdt, 0);
}

/*
 * Phase 1 of the interlock sequence: prove the contactor is open while we are
 * deliberately not enabling it. Returns false (and has already raised the weld
 * fault) if the contactor is closed at any point during the window.
 */
static bool interlock_quiet_window(void)
{
	uint32_t elapsed = 0U;

	(void)gpio_pin_set_dt(&s_hw_wdt, 0);

	while (elapsed < WD_QUIET_WINDOW_MS) {
		if (contactor_is_closed()) {
			LOG_ERR("contactor CLOSED with the health pulse silent "
				"(%u ms in) — welded, stuck, or feedback lost",
				elapsed);
			fault_raise((uint32_t)OVEN_FAULT_CONTACTOR_WELD);
			return false;
		}
		k_sleep(K_MSEC(OVEN_WATCHDOG_PERIOD_MS));
		elapsed += OVEN_WATCHDOG_PERIOD_MS;
	}

	LOG_INF("contactor open through a %u ms silent window — interlock may "
		"now be closed by hand", WD_QUIET_WINDOW_MS);
	return true;
}

/*
 * Phases 2-3, evaluated once per cycle.
 *
 *   pulsing == true  : we are enabling the interlock. Watch for the operator
 *                      closing it (AWAIT_CONTACTOR -> DISARMED), and for it
 *                      dropping out from under us (-> AWAIT_CONTACTOR).
 *   pulsing == false : we have withdrawn enable. The contactor must open within
 *                      the drop-out deadline or it is welded.
 */
static void interlock_monitor(bool pulsing, uint32_t *cycles_not_pulsing)
{
	bool closed = contactor_is_closed();
	enum oven_state st = oven_state_get();

	if (pulsing) {
		*cycles_not_pulsing = 0U;

		if (st == OVEN_STATE_AWAIT_CONTACTOR) {
			if (closed && oven_state_contactor_confirmed()) {
				LOG_INF("contactor closed and observed — power "
					"path proven, oven is now armable");
			}
		} else if (!closed && ((st == OVEN_STATE_DISARMED) ||
				       (st == OVEN_STATE_ARMED))) {
			LOG_WRN("contactor dropped while enabled — returning to "
				"AWAIT_CONTACTOR; close the interlock by hand");
			oven_state_contactor_lost();
		} else {
			/* Closed and confirmed: the normal running condition. */
		}

		return;
	}

	/* Not pulsing: the interlock must let go, and we must see it happen. */
	if (*cycles_not_pulsing <= WD_DROPOUT_CYCLES) {
		(*cycles_not_pulsing)++;
	} else if (closed) {
		LOG_ERR("contactor STILL CLOSED %u ms after the health pulse "
			"stopped — welded", (uint32_t)OVEN_CONTACTOR_OPEN_GRACE_MS);
		fault_raise((uint32_t)OVEN_FAULT_CONTACTOR_WELD);
	} else {
		/* Opened as required. */
	}
}

static void watchdog_thread_entry(void *p1, void *p2, void *p3)
{
	uint32_t last_pid = 0U;
	uint32_t last_output = 0U;
	uint32_t last_timer = 0U;
	uint32_t pid_stale = 0U;
	uint32_t output_stale = 0U;
	uint32_t timer_stale = 0U;
	uint32_t cycles_not_pulsing = 0U;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	__ASSERT(k_thread_priority_get(k_current_get()) == OVEN_WATCHDOG_PRIO,
		 "Watchdog thread priority mismatch");

	while (atomic_get(&g_init_done) == 0) {
		k_sleep(K_MSEC(1));
	}

	/* Own the health-pulse pin: configured inactive before the first pulse. */
	(void)gpio_pin_configure_dt(&s_hw_wdt, GPIO_OUTPUT_INACTIVE);
	onchip_wdt_setup();

	/*
	 * Phase 1 of the interlock sequence, before this thread is willing to
	 * enable anything. On failure the weld fault is already latched and the
	 * state stays INIT, so arming is impossible; fall through into the normal
	 * loop so telemetry and diagnostics keep running (stop driving, keep
	 * monitoring).
	 */
	if (interlock_quiet_window()) {
		(void)oven_state_interlock_ready();
	}

	last_pid = (uint32_t)atomic_get(&g_pid_seq);
	last_output = (uint32_t)atomic_get(&g_output_seq);
	last_timer = (uint32_t)atomic_get(&g_timer_ticks);

	for (;;) {
		uint32_t pid;
		uint32_t output;
		uint32_t timer;
		bool alive;
		bool healthy;

		pid = (uint32_t)atomic_get(&g_pid_seq);
		output = (uint32_t)atomic_get(&g_output_seq);
		timer = (uint32_t)atomic_get(&g_timer_ticks);

		pid_stale = seq_advanced(pid, last_pid) ? 0U : (pid_stale + 1U);
		output_stale = seq_advanced(output, last_output) ? 0U : (output_stale + 1U);
		timer_stale = seq_advanced(timer, last_timer) ? 0U : (timer_stale + 1U);
		last_pid = pid;
		last_output = output;
		last_timer = timer;

		/* Raise faults for any starved liveness source. */
		if (output_stale > WD_OUTPUT_STALE_CYCLES) {
			fault_raise((uint32_t)OVEN_FAULT_OUTPUT_STALE);
		}
		if (pid_stale > WD_PID_STALE_CYCLES) {
			fault_raise((uint32_t)OVEN_FAULT_PID_STALE);
		}
		if (timer_stale > WD_TIMER_STALE_CYCLES) {
			fault_raise((uint32_t)OVEN_FAULT_TIMER_STALL);
		}

		alive = (output_stale <= WD_OUTPUT_STALE_CYCLES) &&
			(pid_stale <= WD_PID_STALE_CYCLES) &&
			(timer_stale <= WD_TIMER_STALE_CYCLES);

		healthy = alive && (fault_word_get() == 0U);

		/*
		 * Feed the on-chip watchdog on liveness alone (no reboot loop in
		 * FAULTED); emit the external health pulse only when fully healthy
		 * AND fault-free (drops the contactor + external watchdog on fault).
		 */
		if (alive) {
			onchip_wdt_feed();
		}

		if (healthy) {
			emit_hw_wdt_pulse();
		} else {
			(void)gpio_pin_set_dt(&s_hw_wdt, 0);
		}

		/*
		 * Watch the contactor against what we just asked for. This is the
		 * only place that owns the enable, so it is the only place that
		 * can judge the answer.
		 */
		interlock_monitor(healthy, &cycles_not_pulsing);

		k_sleep(K_MSEC(OVEN_WATCHDOG_PERIOD_MS));
	}
}

K_THREAD_DEFINE(watchdog_tid, OVEN_WATCHDOG_STACK_SIZE, watchdog_thread_entry,
		NULL, NULL, NULL, OVEN_WATCHDOG_PRIO, 0, 0);

bool watchdog_devices_ready(void)
{
	return gpio_is_ready_dt(&s_hw_wdt);
}
