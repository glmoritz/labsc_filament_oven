/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * labsc_filament_oven — application entry.
 *
 * Responsibilities (architecture, "Boot, Reset, Flashing" + "Safe State"):
 *   - Always boot DISARMED (or FAULTED); the heater can never self-arm.
 *   - Restore any latched fault so a safety trip survives the reboot.
 *   - Configure the field I/O, verify every required device, then release the
 *     PID / Output / Watchdog threads.
 *
 * After init, main() drops to the lowest useful role: a slow status heartbeat.
 * The real work lives in the three threads; main holds no safety responsibility.
 */

#include "shared.h"
#include "fault.h"
#include "oven_state.h"
#include "oven_threads.h"
#include "oven_debug.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define HEARTBEAT_PERIOD_MS   (1000U)

/* Field I/O owned by main (the threads own the two safety pulse pins). */
static const struct gpio_dt_spec s_opto_out2 =
	GPIO_DT_SPEC_GET(DT_ALIAS(opto_out2), gpios);
static const struct gpio_dt_spec s_opto_in1 =
	GPIO_DT_SPEC_GET(DT_ALIAS(opto_in1), gpios);
static const struct gpio_dt_spec s_opto_in2 =
	GPIO_DT_SPEC_GET(DT_ALIAS(opto_in2), gpios);

/* Status LEDs are BlackPill-only; the C6 has none (uses its RGB, not driven). */
#if DT_NODE_EXISTS(DT_ALIAS(led0))
static const struct gpio_dt_spec s_led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define OVEN_HAS_LED0 1
#else
#define OVEN_HAS_LED0 0
#endif

static bool configure_field_io(void)
{
	bool ok = true;

	if (gpio_is_ready_dt(&s_opto_out2)) {
		(void)gpio_pin_configure_dt(&s_opto_out2, GPIO_OUTPUT_INACTIVE);
	} else {
		ok = false;
	}

	if (gpio_is_ready_dt(&s_opto_in1)) {
		(void)gpio_pin_configure_dt(&s_opto_in1, GPIO_INPUT);
	} else {
		ok = false;
	}

	if (gpio_is_ready_dt(&s_opto_in2)) {
		(void)gpio_pin_configure_dt(&s_opto_in2, GPIO_INPUT);
	} else {
		ok = false;
	}

#if OVEN_HAS_LED0
	if (gpio_is_ready_dt(&s_led0)) {
		(void)gpio_pin_configure_dt(&s_led0, GPIO_OUTPUT_INACTIVE);
	}
#endif

	return ok;
}

static void restore_latched_fault(void)
{
	uint32_t cause = 0U;

	if (fault_record_load(&cause)) {
		LOG_ERR("Latched fault restored: cause=0x%08x -> booting FAULTED",
			cause);
		fault_raise(cause | (uint32_t)OVEN_FAULT_RESTORED);
	}
}

static void verify_devices(void)
{
	if (!pid_devices_ready()) {
		LOG_ERR("thermocouple device(s) not ready");
		fault_raise((uint32_t)OVEN_FAULT_DEVICE_INIT);
	}
	if (!output_devices_ready()) {
		LOG_ERR("keep-alive output not ready");
		fault_raise((uint32_t)OVEN_FAULT_DEVICE_INIT);
	}
	if (!watchdog_devices_ready()) {
		LOG_ERR("hw-wdt output not ready");
		fault_raise((uint32_t)OVEN_FAULT_DEVICE_INIT);
	}
}

static void heartbeat(void)
{
	enum oven_state st = oven_state_get();

#if OVEN_HAS_LED0
	if (gpio_is_ready_dt(&s_led0)) {
		(void)gpio_pin_toggle_dt(&s_led0);
	}
#endif

	LOG_INF("state=%s fault=0x%08x filament=%d.%02d C heat=%d.%02d C hp=%ld pid=%ld out=%ld",
		oven_state_name(st), fault_word_get(),
		(int)(atomic_get(&g_temp_a_cc) / OVEN_CENTI),
		(int)(atomic_get(&g_temp_a_cc) % OVEN_CENTI),
		(int)(atomic_get(&g_temp_b_cc) / OVEN_CENTI),
		(int)(atomic_get(&g_temp_b_cc) % OVEN_CENTI),
		(long)atomic_get(&g_heater_power),
		(long)atomic_get(&g_pid_seq),
		(long)atomic_get(&g_output_seq));
}

int main(void)
{
	LOG_INF("labsc_filament_oven — control/safety skeleton");

	/* 1. Restore a latched fault before anything can arm. */
	restore_latched_fault();

	/* 2. Configure field I/O; a missing field pin is a device-init fault. */
	if (!configure_field_io()) {
		LOG_ERR("field I/O configuration failed");
		fault_raise((uint32_t)OVEN_FAULT_DEVICE_INIT);
	}

	/* 3. Verify the thread-owned devices. */
	verify_devices();


	/* 4. Start the 500 ms PID release timer. */
	pid_thread_start();

	/* 5. Leave INIT: DISARMED unless a fault already latched FAULTED. */
	oven_state_boot_ok();

	/*
	 * 5b. Bench instrumentation, if this image has any. Empty inline in a
	 * field build. Runs before the threads are released so the first control
	 * cycle already sees any preset tuning. Arms nothing.
	 */
	oven_debug_init();

	/* 6. Release the threads. */
	(void)atomic_set(&g_init_done, 1);

	LOG_INF("init complete: state=%s fault=0x%08x",
		oven_state_name(oven_state_get()), fault_word_get());

	/* 7. Slow status heartbeat; no safety role. */
	for (;;) {
		heartbeat();
		k_sleep(K_MSEC(HEARTBEAT_PERIOD_MS));
	}

	return 0;
}
