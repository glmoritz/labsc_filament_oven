/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fixed-point PID control law — see pid.h for the design rationale.
 *
 * Internal fixed-point: all term accumulators (yp, yi, yd, y) are held in
 * "micro half-cycles" (actual half-cycles x PID_FP), int64. Inputs are gains
 * x1000 (OVEN_GAIN_SCALE) and temperatures in centi-degC. Output is rounded
 * back to whole half-cycles in [0, OVEN_HEATER_MAX_POWER].
 *
 * Coefficient derivation (baked for OVEN_PID_PERIOD_MS = 500 ms, so Ts = 0.5 s):
 *
 *   P : yp_u = Kp * e * PID_FP
 *             = (kp_m/1000) * e * 1e6 = kp_m * e * PID_P_SCALE,  PID_P_SCALE=1000
 *
 *   I : Tustin, yi_u += Ki*(Ts/2)*(e[k]+e[k-1]) * PID_FP
 *             Ki*Ts/2 = (ki_m/1000)*(0.5/2) = ki_m/4000
 *             increment_u = ki_m*(e+e_prev) * (PID_FP/4000) = ... * PID_I_SCALE
 *             PID_I_SCALE = PID_FP*Ts/(2*1000) = 250
 *
 *   D : on measurement, pole-matched. a = e^(-N*Ts) stored as PID_D_POLE_Q16.
 *             yd_u[k] = a*yd_u[k-1] - Kd*(1-a)/Ts * (pv[k]-pv[k-1]) * PID_FP
 *             per-cycle gain g_d_u = kd_m*(Q16 - a)*PID_D_NUM/PID_D_DEN
 *             so that kd_term_u = g_d_u * dpv.
 *             With Ts=0.5, PID_FP=1e6, GAIN_SCALE=1000, Q16=65536 this reduces
 *             to g_d_u = kd_m*(65536 - a)*125 / 4096.
 */

#include "pid.h"
#include "shared.h"

#include <zephyr/sys/util.h>   /* BUILD_ASSERT, CLAMP */

#define PID_FP          (1000000LL)
#define PID_Q16         (65536LL)
#define PID_MAX_U       ((int64_t)OVEN_HEATER_MAX_POWER * PID_FP)

/* P: yp_u = kp_m * e * PID_P_SCALE. */
#define PID_P_SCALE     (PID_FP / OVEN_GAIN_SCALE)
BUILD_ASSERT((PID_FP % OVEN_GAIN_SCALE) == 0, "PID_FP must be a multiple of the gain scale");

/* I: increment_u = ki_m * (e + e_prev) * PID_I_SCALE. Exact integer for Ts=0.5s. */
#define PID_I_SCALE     ((PID_FP * (int64_t)OVEN_PID_PERIOD_MS) / (2000 * OVEN_GAIN_SCALE))
BUILD_ASSERT(((PID_FP * (int64_t)OVEN_PID_PERIOD_MS) % (2000 * OVEN_GAIN_SCALE)) == 0,
	     "PID integral scale is not an exact integer for this period");

/*
 * D filter pole a = e^(-N*Ts) in Q16. Default N = 0.5 (Ts = 0.5 s -> N*Ts=0.25,
 * a = e^-0.25 = 0.7788 -> 0.7788*65536 = 51045), i.e. a ~2 s derivative filter.
 * Only relevant when kd_m != 0; the default gains are 0 (inert).
 */
#define PID_D_POLE_Q16  (51045LL)
#define PID_D_NUM       (125LL)
#define PID_D_DEN       (4096LL)

void pid_reset(struct pid_state *st)
{
	st->yi_u = 0;
	st->yd_u = 0;
	st->y_prev_u = 0;
	st->e_prev = 0;
	st->pv_prev = 0;
	st->g_prev.kp_m = 0;
	st->g_prev.ki_m = 0;
	st->g_prev.kd_m = 0;
	st->primed = false;
	st->active_prev = false;
}

static bool gains_changed(const struct pid_gains *a, const struct pid_gains *b)
{
	return ((a->kp_m != b->kp_m) || (a->ki_m != b->ki_m) || (a->kd_m != b->kd_m));
}

uint32_t pid_update(struct pid_state *st, const struct pid_gains *g,
		    int32_t setpoint_cc, int32_t pv_cc, bool active)
{
	int32_t e;
	int32_t dpv;
	int64_t yp_u;
	int64_t g_d_u;
	int64_t out_u;
	uint32_t out;

	e = setpoint_cc - pv_cc;

	/* First call: seed histories so the first derivative/step is zero. */
	if (!st->primed) {
		st->e_prev = e;
		st->pv_prev = pv_cc;
		st->primed = true;
	}

	dpv = pv_cc - st->pv_prev;

	/* Proportional. */
	yp_u = (int64_t)g->kp_m * e * PID_P_SCALE;

	/*
	 * Derivative on measurement, pole-matched filter. Updated every cycle
	 * (even when inactive) so it tracks the plant and arming produces no kick.
	 */
	g_d_u = ((int64_t)g->kd_m * (PID_Q16 - PID_D_POLE_Q16) * PID_D_NUM) / PID_D_DEN;
	st->yd_u = ((PID_D_POLE_Q16 * st->yd_u) / PID_Q16) - (g_d_u * (int64_t)dpv);

	if (!active) {
		/* Disarmed / faulted: output off, integrator frozen (no windup). */
		out_u = 0;
		st->y_prev_u = 0;
	} else {
		int64_t inc_u;
		int64_t y_unsat_u;

		if (!st->active_prev) {
			/* Clean restart on the DISARMED->ARMED edge. */
			st->yi_u = 0;
		} else if (gains_changed(g, &st->g_prev)) {
			/* Bumpless retune: keep the output continuous. */
			st->yi_u = st->y_prev_u - yp_u - st->yd_u;
		} else {
			/* no integrator adjustment */
		}

		/* Trapezoidal integral increment. */
		inc_u = (int64_t)g->ki_m * ((int64_t)e + (int64_t)st->e_prev) * PID_I_SCALE;
		st->yi_u += inc_u;

		/*
		 * Anti-windup by integrator clamping. Let the output reach the rail,
		 * then pin the integrator to exactly the value that holds it there, so
		 * it cannot wind up past the limit and de-saturates without lag when
		 * the error reverses. Applied at BOTH rails (the actuator is
		 * one-directional, so it saturates at 0 too).
		 */
		y_unsat_u = yp_u + st->yi_u + st->yd_u;
		if (y_unsat_u > PID_MAX_U) {
			st->yi_u = PID_MAX_U - yp_u - st->yd_u;
			out_u = PID_MAX_U;
		} else if (y_unsat_u < 0) {
			st->yi_u = -yp_u - st->yd_u;
			out_u = 0;
		} else {
			out_u = y_unsat_u;
		}

		/* Hard clamp the accumulator as a second-line bound. */
		st->yi_u = CLAMP(st->yi_u, -PID_MAX_U, PID_MAX_U);
		st->y_prev_u = out_u;
	}

	/* Round to whole half-cycles. */
	out = (uint32_t)((out_u + (PID_FP / 2)) / PID_FP);

	/* Update histories. */
	st->e_prev = e;
	st->pv_prev = pv_cc;
	st->g_prev = *g;
	st->active_prev = active;

	return out;
}
