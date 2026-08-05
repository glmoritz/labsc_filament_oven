/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * PID thread (architecture, "PID Thread"). Released every 500 ms by a kernel
 * timer (hardware-clock backed). Reads both thermocouples, validates them,
 * checks the protection-zone absolute limit, computes the control output, and
 * publishes actuator + telemetry + a liveness counter.
 *
 * The acquisition, validation, safety-limit, control-law and liveness paths are
 * real. The control law is the fixed-point PID in pid.c (default gains 0 -> inert
 * and safe until tuned). Process diagnostics are not implemented yet. No dynamic
 * allocation; no unbounded blocking (the only wait is the bounded, driver-internal
 * SPI transfer inside the MAX6675 sensor read).
 */

#include "shared.h"
#include "oven_threads.h"
#include "oven_state.h"
#include "pid.h"
#include "pid_pub.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pid, LOG_LEVEL_INF);

static const struct device *const s_tc_a = DEVICE_DT_GET(DT_ALIAS(tc0));
static const struct device *const s_tc_b = DEVICE_DT_GET(DT_ALIAS(tc1));

static struct k_sem s_release;
static struct k_timer s_release_timer;
static struct pid_state s_pid;

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

static void pid_cycle(uint32_t delta_ms)
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
	 * 6-7. Compute the control law (sensor A only) and publish the actuator
	 * command. The controller runs only when ARMED and fault-free and the
	 * control reading is valid; otherwise it is held inactive (output 0,
	 * integrator frozen, derivative still tracking). The Output thread applies
	 * SSR enable on top of this; a non-zero command never energizes anything
	 * while disarmed or faulted.
	 */
	{
		struct pid_gains gains;
		bool active;
		uint32_t power;
		oven_temp_cc_t setpoint;
		oven_temp_cc_t pv_a;

		gains.kp_m = (int32_t)atomic_get(&g_kp_m);
		gains.ki_m = (int32_t)atomic_get(&g_ki_m);
		gains.kd_m = (int32_t)atomic_get(&g_kd_m);
		setpoint = (oven_temp_cc_t)atomic_get(&g_setpoint_cc);
		pv_a = (oven_temp_cc_t)atomic_get(&g_temp_a_cc);

		active = a_ok &&
			 (oven_state_get() == OVEN_STATE_ARMED) &&
			 (fault_word_get() == 0U);

		power = pid_update(&s_pid, &gains, setpoint, pv_a, active);
		(void)atomic_set(&g_heater_power, (atomic_val_t)power);
	}

	/* 8-9. Telemetry already published above; advance the liveness counter. */
	(void)atomic_inc(&g_pid_seq);

	/*
	 * 10. Publish this cycle's internals for any diagnostics consumer.
	 *
	 * UNCONDITIONAL, in every build — see pid_pub.h. A field image compiles
	 * no consumer and the data goes nowhere, which is the point: the control
	 * loop must not have one instruction path on the bench and another in the
	 * field. Wait-free; it cannot block or fail.
	 */
	pid_pub_publish((uint32_t)atomic_get(&g_pid_seq), delta_ms, &s_pid.diag);
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

		pid_cycle(delta_ms);
	}
}

K_THREAD_DEFINE(pid_tid, OVEN_PID_STACK_SIZE, pid_thread_entry,
		NULL, NULL, NULL, OVEN_PID_PRIO, 0, 0);

void pid_thread_start(void)
{
	pid_reset(&s_pid);
	k_sem_init(&s_release, 0, 1);
	k_timer_init(&s_release_timer, release_timer_cb, NULL);
	k_timer_start(&s_release_timer,
		      K_MSEC(OVEN_PID_PERIOD_MS), K_MSEC(OVEN_PID_PERIOD_MS));
}

bool pid_devices_ready(void)
{
	return (device_is_ready(s_tc_a) && device_is_ready(s_tc_b));
}
