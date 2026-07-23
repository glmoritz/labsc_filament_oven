/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Thread priority + stack configuration, single source of truth
 * (architecture, "Thread priority audit"). Priorities are pinned here and each
 * thread asserts its own actual priority at startup.
 *
 *   PID       : highest real-time  -> K_PRIO_COOP(1)
 *   Output     : just below PID     -> K_PRIO_COOP(2)
 *   Watchdog   : lowest in system   -> lowest preemptible (starvation detector)
 *
 * The two real-time threads are COOPERATIVE: they cannot be preempted by
 * background work (net/log/shell run at preemptible priorities) and yield only
 * when they block (PID on its release semaphore, Output on its period sleep).
 * The Watchdog is the lowest preemptible priority, so it starves first — that
 * starvation is what expires the hardware watchdog, by design.
 */

#ifndef OVEN_THREADS_H_
#define OVEN_THREADS_H_

#include <zephyr/kernel.h>

#define OVEN_PID_PRIO        K_PRIO_COOP(1)
#define OVEN_OUTPUT_PRIO     K_PRIO_COOP(2)
#define OVEN_WATCHDOG_PRIO   K_PRIO_PREEMPT(CONFIG_NUM_PREEMPT_PRIORITIES - 1)

#define OVEN_PID_STACK_SIZE       (2048)
#define OVEN_OUTPUT_STACK_SIZE    (1536)
#define OVEN_WATCHDOG_STACK_SIZE  (1536)

/* Started by main() once the hardware is configured. */
void pid_thread_start(void);

/* Per-module device-readiness checks, called by main() before arming init. */
bool pid_devices_ready(void);
bool output_devices_ready(void);
bool watchdog_devices_ready(void);

#endif /* OVEN_THREADS_H_ */
