/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fixed-point PID control law — see pid.h for the design rationale and the
 * input contract. This is safety code: it validates every input, asserts on
 * anything outside contract, clamps as a production net, and favours clear
 * checked arithmetic over speed.
 */

#include "pid.h"
#include "shared.h"

#include <stddef.h>                /* NULL */
#include <zephyr/sys/util.h>       /* BUILD_ASSERT, CLAMP, ARG_UNUSED */
#include <zephyr/sys/__assert.h>   /* __ASSERT */

/*
 * ---- Two fixed-point scales, for two different kinds of quantity ----
 *
 * PID_FP (decimal, 1e6) scales the SIGNAL accumulators yp/yi/yd/y — they are
 * held in "micro half-cycles" (actual half-cycles x PID_FP). A decimal,
 * high-resolution scale is used so a tiny integral increment (well below one
 * half-cycle) still accumulates instead of quantizing to zero.
 *
 * PID_Q16 (binary, 2^16) scales the derivative filter POLE 'a', a unitless
 * fraction in [0,1). A power-of-two scale is the natural format for a
 * coefficient < 1: the "divide by 65536" after multiplying by 'a' is exact and
 * cheap. The two scales are intentionally different — one is a signal
 * resolution, the other a fractional-coefficient resolution.
 */
#define PID_FP          (1000000LL)
#define PID_Q16         (65536LL)

/* Output ceiling in the internal signal scale. */
#define PID_MAX_U       ((int64_t)OVEN_HEATER_MAX_POWER * PID_FP)

/*
 * P conversion constant. yp_u = Kp * e * PID_FP, and Kp = kp_m / OVEN_GAIN_SCALE,
 * so yp_u = kp_m * e * (PID_FP / OVEN_GAIN_SCALE) = kp_m * e * PID_P_SCALE.
 * The BUILD_ASSERT guarantees the scale is an EXACT integer: if PID_FP or the
 * gain scale ever change to a non-dividing pair, the build FAILS rather than
 * silently truncating the constant and detuning the controller.
 */
#define PID_P_SCALE     (PID_FP / OVEN_GAIN_SCALE)
BUILD_ASSERT((PID_FP % OVEN_GAIN_SCALE) == 0,
	     "PID_FP must be an exact multiple of the gain scale");

/*
 * I conversion constant (trapezoidal / Tustin). Per step:
 *   yi_u += Ki*(Ts/2)*(e[k]+e[k-1]) * PID_FP
 * With Ki = ki_m/OVEN_GAIN_SCALE and Ts = OVEN_PID_PERIOD_MS/1000, the whole
 * (Ki, Ts/2, PID_FP) conversion folds into one integer multiply:
 *   increment_u = ki_m * (e+e_prev) * PID_I_SCALE
 *   PID_I_SCALE = PID_FP * Ts / (2 * OVEN_GAIN_SCALE)   (= 250 at Ts=0.5s)
 * The BUILD_ASSERT again forces this to be an exact integer for the configured
 * period; a period that makes it fractional is a compile error, not a silent
 * mis-tune.
 */
#define PID_I_SCALE     ((PID_FP * (int64_t)OVEN_PID_PERIOD_MS) / (2 * 1000 * OVEN_GAIN_SCALE))
BUILD_ASSERT(((PID_FP * (int64_t)OVEN_PID_PERIOD_MS) % (2 * 1000 * OVEN_GAIN_SCALE)) == 0,
	     "PID integral scale is not an exact integer for this sample period");

/*
 * D filter pole a = e^(-N*Ts) in Q16, plus the numerator scaling for the
 * pole-matched derivative. Default N = 0.5 (Ts = 0.5 s -> N*Ts = 0.25,
 * a = e^-0.25 = 0.7788 -> 0.7788 * 65536 = 51045), a ~2 s derivative filter.
 * PID_D_NUM/PID_D_DEN carry the (1-a)/Ts * PID_FP scaling; with Ts=0.5,
 * PID_FP=1e6, gain scale 1000 and Q16=65536 the per-cycle gain reduces to
 *   g_d_u = kd_m * (65536 - a) * 125 / 4096.
 * Only relevant when kd_m != 0; default gains are 0 (inert).
 */
#define PID_D_POLE_Q16  (51045LL)
#define PID_D_NUM       (125LL)
#define PID_D_DEN       (4096LL)

/* Maximum |error| the physical window allows; used as a tripwire assert. */
#define PID_E_MAX       ((int32_t)(OVEN_TEMP_VALID_MAX_CC - OVEN_TEMP_VALID_MIN_CC))

/* ---- Input validators: assert (catch caller bugs) AND clamp (stay safe). ---- */

static int32_t validate_gain(int32_t v, const char *name)
{
	ARG_UNUSED(name);
	__ASSERT((v >= OVEN_PID_GAIN_M_MIN) && (v <= OVEN_PID_GAIN_M_MAX),
		 "pid: gain '%s'=%d out of [%d, %d]", name, v,
		 OVEN_PID_GAIN_M_MIN, OVEN_PID_GAIN_M_MAX);
	return CLAMP(v, OVEN_PID_GAIN_M_MIN, OVEN_PID_GAIN_M_MAX);
}

static int32_t validate_temp(int32_t v, const char *name)
{
	ARG_UNUSED(name);
	__ASSERT((v >= OVEN_TEMP_VALID_MIN_CC) && (v <= OVEN_TEMP_VALID_MAX_CC),
		 "pid: temperature '%s'=%d out of [%d, %d]", name, v,
		 OVEN_TEMP_VALID_MIN_CC, OVEN_TEMP_VALID_MAX_CC);
	return CLAMP(v, OVEN_TEMP_VALID_MIN_CC, OVEN_TEMP_VALID_MAX_CC);
}

void pid_reset(struct pid_state *st)
{
	__ASSERT(st != NULL, "pid_reset: NULL state");

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
	struct pid_gains gv;   /* validated, clamped gains */
	int32_t sp;
	int32_t pv;
	int32_t e;
	int32_t dpv;
	int64_t yp_u;
	int64_t g_d_u;
	int64_t out_u;
	uint32_t out;

	/* ---- 0. Trust nothing. Validate the caller's pointers and data. ---- */
	__ASSERT(st != NULL, "pid_update: NULL state");
	__ASSERT(g != NULL, "pid_update: NULL gains");

	/*
	 * Gains: asserted against the contract (a violation is a write-boundary
	 * bug) and clamped so an out-of-range value can never reach the math.
	 */
	gv.kp_m = validate_gain(g->kp_m, "kp_m");
	gv.ki_m = validate_gain(g->ki_m, "ki_m");
	gv.kd_m = validate_gain(g->kd_m, "kd_m");

	/*
	 * Setpoint and measurement: re-checked here even though the sensor path
	 * already rejects bad reads. A bad setpoint (or a caller bug) must not be
	 * able to inject an implausible error into the loop.
	 */
	sp = validate_temp(setpoint_cc, "setpoint");
	pv = validate_temp(pv_cc, "pv");

	e = sp - pv;
	/* With both operands clamped to the window, |e| cannot exceed its span. */
	__ASSERT((e >= -PID_E_MAX) && (e <= PID_E_MAX), "pid: error span violated");

	/*
	 * `primed` == "do we have a valid previous sample yet?". The integral
	 * uses e[k-1] and the derivative uses pv[k-1]; on the first call after
	 * reset there is none, so seed both histories with the current values.
	 * That makes the first step's differences exactly zero (dpv = 0) instead
	 * of differencing against stale/zero history and spiking the derivative.
	 * Stays true until pid_reset().
	 */
	if (!st->primed) {
		st->e_prev = e;
		st->pv_prev = pv;
		st->primed = true;
	}

	dpv = pv - st->pv_prev;

	/*
	 * ---- Proportional ----
	 * yp_u = kp_m * e * PID_P_SCALE. The leading (int64_t) cast promotes the
	 * ENTIRE product chain to 64-bit, so no 32-bit intermediate is formed.
	 * Bound (worst case, gain at contract max, |e| at window span):
	 *   1e6 * 32000 * 1000 = 3.2e16  <<  int64 max 9.2e18  (margin ~290x).
	 */
	yp_u = (int64_t)gv.kp_m * e * PID_P_SCALE;

	/*
	 * ---- Derivative, on MEASUREMENT, pole-matched filter ----
	 * yd_u[k] = a*yd_u[k-1] - g_d_u*dpv, a = PID_D_POLE_Q16 / 2^16.
	 * Differentiating pv (not e) means a setpoint change adds nothing here.
	 * Updated every cycle, including while inactive, so the filter tracks the
	 * plant and arming produces no derivative kick.
	 */
	g_d_u = ((int64_t)gv.kd_m * (PID_Q16 - PID_D_POLE_Q16) * PID_D_NUM) / PID_D_DEN;
	st->yd_u = ((PID_D_POLE_Q16 * st->yd_u) / PID_Q16) - (g_d_u * (int64_t)dpv);

	/*
	 * Bound the derivative accumulator to +/- full-scale output. A derivative
	 * contribution larger than the whole actuator range is meaningless, and
	 * this is the only accumulator not otherwise clamped: without this bound a
	 * pathological Kd + sustained dpv could grow yd_u until the pole multiply
	 * (PID_D_POLE_Q16 * yd_u) overflows int64. With the clamp that product is
	 * at most 51045 * 1.2e8 = 6.1e12 -- overflow-proof for any int32 gain.
	 */
	st->yd_u = CLAMP(st->yd_u, -PID_MAX_U, PID_MAX_U);

	if (!active) {
		/*
		 * Disarmed / faulted: force the output off and FREEZE the
		 * integrator (holding it here is what prevents windup during long
		 * disarmed periods). The derivative state above kept tracking, so
		 * re-arming will not kick.
		 */
		out_u = 0;
		st->y_prev_u = 0;
	} else {
		int64_t inc_u;
		int64_t y_unsat_u;

		if (!st->active_prev) {
			/* DISARMED->ARMED edge: start the integrator clean. */
			st->yi_u = 0;
		} else if (gains_changed(&gv, &st->g_prev)) {
			/*
			 * Bumpless retune: reconcile the integrator so the output
			 * is continuous across the gain change (the change itself
			 * introduces no jump; normal control resumes from there).
			 */
			st->yi_u = st->y_prev_u - yp_u - st->yd_u;
		} else {
			/* no integrator adjustment this cycle */
		}

		/*
		 * ---- Integral, trapezoidal ----
		 * inc_u = ki_m * (e + e_prev) * PID_I_SCALE. The (e+e_prev) sum is
		 * formed in int64 to avoid any 32-bit overflow of the sum itself.
		 * Bound (gain at contract max, sum at 2x window span):
		 *   1e6 * 64000 * 250 = 1.6e13  <<  int64 max  (margin ~5e5x).
		 */
		inc_u = (int64_t)gv.ki_m *
			((int64_t)e + (int64_t)st->e_prev) * PID_I_SCALE;
		st->yi_u += inc_u;

		/*
		 * ---- Anti-windup by integrator clamping (both rails) ----
		 * Let the output reach a rail, then pin the integrator to exactly
		 * the value that holds it there, so it cannot wind up past the
		 * limit and de-saturates without lag when the error reverses. The
		 * actuator is one-directional, so 0 is a real rail too.
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

		/* Second-line bound on the accumulator itself. */
		st->yi_u = CLAMP(st->yi_u, -PID_MAX_U, PID_MAX_U);
		st->y_prev_u = out_u;
	}

	/*
	 * ---- Output: check our own range before we hand it out ----
	 * out_u is guaranteed within [0, PID_MAX_U] by the branches above; assert
	 * it (tripwire for any future edit that breaks the invariant), then round
	 * to whole half-cycles and re-check the final value.
	 */
	__ASSERT((out_u >= 0) && (out_u <= PID_MAX_U), "pid: out_u out of range");
	out = (uint32_t)((out_u + (PID_FP / 2)) / PID_FP);
	__ASSERT(out <= OVEN_HEATER_MAX_POWER, "pid: output exceeds heater max");

	/* ---- Commit histories (store the VALIDATED gains for next compare). ---- */
	st->e_prev = e;
	st->pv_prev = pv;
	st->g_prev = gv;
	st->active_prev = active;

	return out;
}
