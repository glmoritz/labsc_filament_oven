/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Definitions of the shared atomics and the central fault-raise path.
 */

#include "shared.h"
#include "oven_state.h"
#include "fault.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oven_fault, LOG_LEVEL_INF);

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

/*
 * ---- Soft assert ---------------------------------------------------------
 *
 * Overrides Zephyr's __weak assert_post_action(), which panics. Panicking is
 * the WRONG failure mode for this machine: it stops the watchdog thread, so the
 * external watchdog expires and everything does de-energize — but it also stops
 * the sensor reads, the fault word, the telemetry and the trace. You end up in a
 * safe oven you cannot diagnose, and with no evidence of what tripped.
 *
 * A failed assert here does the same thing every other detector does: raise a
 * fault. That latches FAULTED, the Output thread stops the SSR keep-alive within
 * one 8.33 ms cycle, the Watchdog thread stops the health pulse and drops the
 * contactor, and the loop keeps running — de-energized, still reading sensors,
 * still publishing why. Stop driving, keep monitoring.
 *
 * REQUIRES CONFIG_ASSERT_TEST=y (set in prj.conf). Without it Zephyr expands
 * __ASSERT to `assert_post_action(); CODE_UNREACHABLE;` and returning from this
 * function is undefined behaviour. With it, the trailing unreachable marker is
 * omitted precisely so a custom hook may return. That Kconfig is named for its
 * origin in Zephyr's own assert tests; the mechanism is the supported one.
 *
 * Residual risk, accepted knowingly: execution CONTINUES past a violated
 * invariant, into code that assumed it held. What makes that acceptable here is
 * ordering — by the time control returns, the fault is already latched and the
 * actuator is already commanded off, so whatever the broken code does next it
 * cannot energize the heater. It can produce wrong telemetry; it cannot produce
 * heat.
 *
 * ISR-safe: fault_raise() is atomics plus a plain memory write guarded by a CRC.
 *
 * Log rate limit: because we now RETURN, an assert on a per-cycle path fires
 * again every cycle, forever. The fault word does not care — it is OR-only and
 * latches once — but the log would flood and push out the diagnostics that
 * explain the trip. Report the first few in full and then go quiet; the latched
 * fault bit remains the durable evidence.
 */
#ifdef CONFIG_ASSERT

#define ASSERT_LOG_BUDGET   (3)

static atomic_t s_assert_logs = ATOMIC_INIT(0);

#ifdef CONFIG_ASSERT_NO_FILE_INFO
void assert_post_action(void)
{
	fault_raise((uint32_t)OVEN_FAULT_ASSERT);
}
#else
void assert_post_action(const char *file, unsigned int line)
{
	atomic_val_t n;

	fault_raise((uint32_t)OVEN_FAULT_ASSERT);

	n = atomic_inc(&s_assert_logs);
	if (n < ASSERT_LOG_BUDGET) {
		LOG_ERR("ASSERT FAILED at %s:%u — latched FAULTED, de-energized, "
			"still monitoring", file, line);
	} else if (n == ASSERT_LOG_BUDGET) {
		LOG_ERR("ASSERT at %s:%u repeating — further assert logging "
			"suppressed; fault 0x%08x stays latched",
			file, line, fault_word_get());
	} else {
		/* silent: the fault word already carries the evidence */
	}
}
#endif
#endif /* CONFIG_ASSERT */
