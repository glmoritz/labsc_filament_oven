/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Discrete PID control law (fixed-point, allocation-free, RTOS-independent).
 *
 * Structure follows PID.pdf (parallel form, trapezoidal/Tustin integrator,
 * first-order-filtered derivative), with three deliberate deviations agreed
 * during design review:
 *
 *   1. DERIVATIVE ON MEASUREMENT, not on error. The PDF differentiates e[k];
 *      we differentiate the measurement pv[k] so a setpoint step produces no
 *      derivative kick (architecture, "PID hygiene").
 *   2. POLE-MATCHED DERIVATIVE FILTER. The PDF's forward-Euler pole (1 - N*Ts)
 *      is only stable for N*Ts < 2; we use the exact pole a = e^(-N*Ts), which
 *      is unconditionally stable for any N > 0, with the numerator scaled to
 *      preserve the ramp gain Kd. (N is a compile-time filter parameter; the
 *      pole a is stored directly as a fixed-point constant so no runtime
 *      transcendental is needed.)
 *   3. CONDITIONAL-INTEGRATION ANTI-WINDUP. The actuator is one-directional
 *      (0..OVEN_HEATER_MAX_POWER, it cannot cool), so the output saturates at
 *      BOTH rails; the integral increment is skipped whenever it would drive
 *      the output further into the active rail. Plus bumpless retune (the
 *      integrator is reconciled on a gain change so the output stays
 *      continuous) and a clean restart on arming.
 *
 * All state is caller-owned; nothing is allocated. All math is integer
 * (int32/int64); no floating point anywhere, matching the C6's FPU-less core.
 */

#ifndef OVEN_PID_H_
#define OVEN_PID_H_

#include <stdint.h>
#include <stdbool.h>

/* Tuning parameters, fixed-point x1000 (see OVEN_GAIN_SCALE). */
struct pid_gains {
	int32_t kp_m;   /* proportional gain x1000 */
	int32_t ki_m;   /* integral gain     x1000 */
	int32_t kd_m;   /* derivative gain   x1000 (0 = PI) */
};

/* Controller state — caller-owned, zeroed by pid_reset(). */
struct pid_state {
	int64_t yi_u;        /* integral accumulator, internal fixed-point   */
	int64_t yd_u;        /* derivative-filter state, internal fixed-point */
	int64_t y_prev_u;    /* last output, internal fixed-point (bumpless)  */
	int32_t e_prev;      /* previous error,       centi-degC             */
	int32_t pv_prev;     /* previous measurement, centi-degC             */
	struct pid_gains g_prev;
	bool primed;         /* histories valid                              */
	bool active_prev;    /* previous active flag (arm-edge detection)    */
};

/* Reset all state to a safe, un-wound-up zero. */
void pid_reset(struct pid_state *st);

/*
 * Advance the control law by one 500 ms step and return the actuator command
 * clamped to [0, OVEN_HEATER_MAX_POWER] half-cycles.
 *
 *   g         : current gains (read once per cycle by the caller).
 *   setpoint_cc, pv_cc : setpoint and control-zone measurement, centi-degC.
 *   active    : true only when ARMED and the fault word is clear. When false,
 *               the output is forced to 0, the integrator is frozen (no
 *               windup while disarmed), and the derivative filter keeps
 *               tracking the measurement so arming produces no kick.
 */
uint32_t pid_update(struct pid_state *st, const struct pid_gains *g,
		    int32_t setpoint_cc, int32_t pv_cc, bool active);

#endif /* OVEN_PID_H_ */
