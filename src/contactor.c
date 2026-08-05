/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Contactor feedback. See contactor.h for the fail-safe sense.
 */

#include "contactor.h"
#include "shared.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(contactor, LOG_LEVEL_INF);

/*
 * Boot check sampling. Spread over enough time to outlast contact bounce and
 * opto settling, with every sample required to agree.
 */
#define BOOT_SAMPLES        (16U)
#define BOOT_SAMPLE_GAP_MS  (5U)      /* 16 x 5 ms = 80 ms of agreement */

static const struct gpio_dt_spec s_feedback =
	GPIO_DT_SPEC_GET(DT_ALIAS(opto_in1), gpios);

bool contactor_devices_ready(void)
{
	return gpio_is_ready_dt(&s_feedback);
}

bool contactor_is_closed(void)
{
	int val;

	/*
	 * A read error is treated as CLOSED, for the same reason the electrical
	 * sense is inverted: not knowing is not permission to heat.
	 */
	val = gpio_pin_get_dt(&s_feedback);
	if (val < 0) {
		return true;
	}

	return (val != 0);
}

bool contactor_boot_check(void)
{
	if (!contactor_devices_ready()) {
		LOG_ERR("contactor feedback not ready — cannot prove the power "
			"path is open");
		fault_raise((uint32_t)OVEN_FAULT_CONTACTOR_WELD |
			    (uint32_t)OVEN_FAULT_DEVICE_INIT);
		return false;
	}

	for (uint32_t i = 0U; i < BOOT_SAMPLES; i++) {
		if (contactor_is_closed()) {
			LOG_ERR("contactor CLOSED at boot (sample %u/%u) — welded, "
				"stuck, or feedback lost; refusing to become armable",
				i + 1U, BOOT_SAMPLES);
			fault_raise((uint32_t)OVEN_FAULT_CONTACTOR_WELD);
			return false;
		}
		k_sleep(K_MSEC(BOOT_SAMPLE_GAP_MS));
	}

	LOG_INF("contactor open at boot — power path proven clear");
	return true;
}
