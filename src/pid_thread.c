/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * PID thread (architecture, "PID Thread"). Released every 500 ms by a kernel
 * timer (hardware-clock backed). Reads both thermocouples, validates them,
 * checks the protection-zone absolute limit, computes the control output, and
 * publishes actuator + telemetry + a liveness counter.
 *
 * BOOTSTRAP SCOPE: the acquisition, validation, safety-limit and liveness paths
 * are real. The PID math itself is a STUB (heater power held at 0 = safe). No
 * process diagnostics yet. No dynamic allocation; no unbounded blocking (the
 * only wait is the bounded, driver-internal SPI transfer inside the MAX6675
 * sensor read).
 */

#include "shared.h"
#include "oven_threads.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pid, LOG_LEVEL_INF);

static const struct device *const s_tc_a = DEVICE_DT_GET(DT_ALIAS(tc0));
static const struct device *const s_tc_b = DEVICE_DT_GET(DT_ALIAS(tc1));

static struct k_sem s_release;
static struct k_timer s_release_timer;

/* Timer ISR context: release the PID cycle and beat the timer-progress heart. */
static void release_timer_cb(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	(void)atomic_inc(&g_timer_ticks);
	k_sem_give(&s_release);
}

/*
 * Read one MAX6675. Returns 0 and writes *out_cc on success; a negative errno
 * on a communication or open-thermocouple fault. Bounded blocking only.
 */
static int read_thermocouple(const struct device *dev, oven_temp_cc_t *out_cc)
{
	struct sensor_value val;
	int ret;

	ret = sensor_sample_fetch(dev);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
	if (ret != 0) {
		return ret;
	}

	/* sensor_value is deg + micro-deg; scale to centi-deg. */
	*out_cc = (oven_temp_cc_t)((val.val1 * OVEN_CENTI) + (val.val2 / 10000));

	return 0;
}

/* One acquisition of a sensor with range plausibility. Raises fault on error. */
static bool acquire_sensor(const struct device *dev, atomic_t *telemetry,
			   uint32_t comm_fault_bit)
{
	oven_temp_cc_t temp_cc = 0;
	bool ok = false;
	int ret;

	ret = read_thermocouple(dev, &temp_cc);
	if (ret != 0) {
		fault_raise(comm_fault_bit);
	} else if ((temp_cc < OVEN_TEMP_VALID_MIN_CC) ||
		   (temp_cc > OVEN_TEMP_VALID_MAX_CC)) {
		fault_raise(comm_fault_bit);
	} else {
		(void)atomic_set(telemetry, (atomic_val_t)temp_cc);
		ok = true;
	}

	return ok;
}

static void pid_cycle(void)
{
	bool a_ok;
	bool b_ok;
	oven_temp_cc_t temp_b;

	/* 1-2. Read control (A) then protection (B) zone. */
	a_ok = acquire_sensor(s_tc_a, &g_temp_a_cc, (uint32_t)OVEN_FAULT_SENSOR_A);
	b_ok = acquire_sensor(s_tc_b, &g_temp_b_cc, (uint32_t)OVEN_FAULT_SENSOR_B);

	/* 3-4. Absolute safety limit on B — independent of the setpoint path. */
	if (b_ok) {
		temp_b = (oven_temp_cc_t)atomic_get(&g_temp_b_cc);
		if (temp_b > OVEN_SENSOR_B_ABS_LIMIT_CC) {
			fault_raise((uint32_t)OVEN_FAULT_SENSOR_B_LIMIT);
		}
	}

	/* 5. Process diagnostics: TODO (ON-but-flat, OFF-but-rising, rate). */

	/*
	 * 6-7. Compute PID and publish actuator command.
	 * STUB: no control law yet. Hold the heater at 0 so the skeleton is
	 * inert and safe. When implemented, respect anti-windup, derivative-on-
	 * measurement and bumpless retune (architecture, "PID hygiene"), and use
	 * only sensor A. Never command power while any fault is latched.
	 */
	if (a_ok && (fault_word_get() == 0U)) {
		(void)atomic_set(&g_heater_power, 0);
	} else {
		(void)atomic_set(&g_heater_power, 0);
	}

	/* 8-9. Telemetry already published above; advance the liveness counter. */
	(void)atomic_inc(&g_pid_seq);
}

static void pid_thread_entry(void *p1, void *p2, void *p3)
{
	uint32_t last_start_ms = 0U;
	bool started = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	__ASSERT(k_thread_priority_get(k_current_get()) == OVEN_PID_PRIO,
		 "PID thread priority mismatch");

	/* Wait until main() has configured the hardware. */
	while (atomic_get(&g_init_done) == 0) {
		k_sleep(K_MSEC(1));
	}

	for (;;) {
		uint32_t now_ms;
		uint32_t delta_ms;

		k_sem_take(&s_release, K_FOREVER);

		now_ms = k_uptime_get_32();
		delta_ms = now_ms - last_start_ms;
		last_start_ms = now_ms;

		/* Single missed control cycle is a fault. */
		if (started &&
		    (delta_ms > (OVEN_PID_PERIOD_MS + OVEN_PID_DEADLINE_MARGIN_MS))) {
			fault_raise((uint32_t)OVEN_FAULT_PID_DEADLINE);
		}
		started = true;

		pid_cycle();
	}
}

K_THREAD_DEFINE(pid_tid, OVEN_PID_STACK_SIZE, pid_thread_entry,
		NULL, NULL, NULL, OVEN_PID_PRIO, 0, 0);

void pid_thread_start(void)
{
	k_sem_init(&s_release, 0, 1);
	k_timer_init(&s_release_timer, release_timer_cb, NULL);
	k_timer_start(&s_release_timer,
		      K_MSEC(OVEN_PID_PERIOD_MS), K_MSEC(OVEN_PID_PERIOD_MS));
}

bool pid_devices_ready(void)
{
	return (device_is_ready(s_tc_a) && device_is_ready(s_tc_b));
}
