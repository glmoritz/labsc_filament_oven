/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Process-response diagnostics (architecture, "Process diagnostics"; the TODO at
 * step 5 of the PID cycle).
 *
 * These are the checks that do NOT depend on a sensor being honest about its own
 * health. A thermocouple converter reporting a plausible, in-range, wrong number
 * passes every validity test in the acquisition path — the MAX6675 cannot even
 * detect a short. What it cannot do is make the OVEN behave like the reading
 * says it should. These checks compare the readings against physics and against
 * each other, and fault on the disagreement.
 *
 * All three raise OVEN_FAULT_PROCESS_DIAG, which latches FAULTED like any other
 * fault: stop driving, keep monitoring.
 *
 * DESIGN BIAS: every check here is windowed and generously toleranced. A process
 * diagnostic that cries wolf gets switched off by the people it protects, so
 * these are tuned to catch gross, sustained, unambiguous misbehaviour and to say
 * nothing about anything marginal. Missing a subtle fault is acceptable here;
 * these are a backstop behind the absolute limit, not a replacement for it.
 */

#ifndef OVEN_DIAGNOSTICS_H_
#define OVEN_DIAGNOSTICS_H_

#include "shared.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Advance the diagnostics by one control cycle. Called unconditionally from the
 * PID thread; like the rest of that path it has no debug/release variant.
 *
 *   filament_cc / filament_ok : SENSOR A (control zone) and whether this cycle's
 *                               read was valid.
 *   heat_cc / heat_ok         : SENSOR B (protection zone), likewise.
 *   power                     : commanded heater power this cycle, half-cycles.
 *   heating                   : true only when the controller is actually
 *                               allowed to act (ARMED, fault-free, valid input).
 *
 * A sensor whose read failed this cycle is simply not evaluated; the acquisition
 * path already raised its own fault for that.
 */
void diag_update(oven_temp_cc_t filament_cc, bool filament_ok,
		 oven_temp_cc_t heat_cc, bool heat_ok,
		 uint32_t power, bool heating);

/* Reset all diagnostic history. Called once from main() before the loop runs. */
void diag_reset(void);

#endif /* OVEN_DIAGNOSTICS_H_ */
