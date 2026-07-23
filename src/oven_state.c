/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 */

#include "oven_state.h"
#include "shared.h"

#include <zephyr/sys/atomic.h>

static atomic_t s_state = ATOMIC_INIT((atomic_val_t)OVEN_STATE_INIT);

void oven_state_boot_ok(void)
{
	/* INIT -> DISARMED only if still in INIT (a restored fault wins). */
	(void)atomic_cas(&s_state,
			 (atomic_val_t)OVEN_STATE_INIT,
			 (atomic_val_t)OVEN_STATE_DISARMED);
}

void oven_state_enter_faulted(void)
{
	(void)atomic_set(&s_state, (atomic_val_t)OVEN_STATE_FAULTED);
}

bool oven_state_request_arm(void)
{
	bool armed = false;

	/* Refuse to arm while any fault is latched. */
	if (fault_word_get() == 0U) {
		armed = atomic_cas(&s_state,
				   (atomic_val_t)OVEN_STATE_DISARMED,
				   (atomic_val_t)OVEN_STATE_ARMED);
	}

	return armed;
}

void oven_state_request_disarm(void)
{
	(void)atomic_cas(&s_state,
			 (atomic_val_t)OVEN_STATE_ARMED,
			 (atomic_val_t)OVEN_STATE_DISARMED);
}

enum oven_state oven_state_get(void)
{
	return (enum oven_state)atomic_get(&s_state);
}

const char *oven_state_name(enum oven_state st)
{
	const char *name;

	switch (st) {
	case OVEN_STATE_INIT:
		name = "INIT";
		break;
	case OVEN_STATE_DISARMED:
		name = "DISARMED";
		break;
	case OVEN_STATE_ARMED:
		name = "ARMED";
		break;
	case OVEN_STATE_FAULTED:
		name = "FAULTED";
		break;
	default:
		name = "?";
		break;
	}

	return name;
}
