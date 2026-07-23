/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Definitions of the shared atomics and the central fault-raise path.
 */

#include "shared.h"
#include "oven_state.h"
#include "fault.h"

atomic_t g_setpoint_cc  = ATOMIC_INIT(0);
atomic_t g_kp_m         = ATOMIC_INIT(0);
atomic_t g_ki_m         = ATOMIC_INIT(0);
atomic_t g_kd_m         = ATOMIC_INIT(0);
atomic_t g_heater_power = ATOMIC_INIT(0);
atomic_t g_temp_a_cc    = ATOMIC_INIT(0);
atomic_t g_temp_b_cc    = ATOMIC_INIT(0);
atomic_t g_pid_seq      = ATOMIC_INIT(0);
atomic_t g_output_seq   = ATOMIC_INIT(0);
atomic_t g_timer_ticks  = ATOMIC_INIT(0);
atomic_t g_fault_word   = ATOMIC_INIT(0);
atomic_t g_init_done    = ATOMIC_INIT(0);

void fault_raise(uint32_t bits)
{
	atomic_val_t prev;

	if (bits == 0U) {
		return;
	}

	prev = atomic_or(&g_fault_word, (atomic_val_t)bits);

	/* Latch the system safe and persist the cause on the first fault only. */
	oven_state_enter_faulted();

	if (((uint32_t)prev & (uint32_t)bits) != (uint32_t)bits) {
		fault_record_store((uint32_t)(prev | (atomic_val_t)bits));
	}
}
