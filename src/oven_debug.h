/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bench diagnostics consumer (CONFIG_OVEN_DEBUG). This is the ONLY part of the
 * diagnostics path that is configuration-dependent: the control loop always
 * computes and publishes (see pid_pub.h), and this module is what may or may not
 * be compiled in to read the publication.
 *
 * main() calls oven_debug_init() unconditionally; with CONFIG_OVEN_DEBUG off the
 * call is an empty inline and nothing is linked in. Callers therefore need no
 * #ifdef of their own.
 */

#ifndef OVEN_DEBUG_H_
#define OVEN_DEBUG_H_

#include <zephyr/kernel.h>

#if defined(CONFIG_OVEN_DEBUG)

/*
 * Seed the bench setpoint/gains (CONFIG_OVEN_DEBUG_PRESET) and hand the
 * diagnostics thread its go-ahead. Must be called before g_init_done is set, so
 * the very first control cycle already sees the preset tuning.
 *
 * This does NOT arm the oven. The system still boots DISARMED and the Output
 * thread still refuses to pulse the SSR keep-alive until armed and fault-free.
 */
void oven_debug_init(void);

#else

static inline void oven_debug_init(void)
{
}

#endif /* CONFIG_OVEN_DEBUG */

#endif /* OVEN_DEBUG_H_ */
