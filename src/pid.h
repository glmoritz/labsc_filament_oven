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
 *      preserve the ramp gain Kd.
 *   3. CONDITIONAL-INTEGRATION ANTI-WINDUP. The actuator is one-directional
 *      (0..OVEN_HEATER_MAX_POWER, it cannot cool), so the output saturates at
 *      BOTH rails; the integrator is clamped so the output can reach a rail but
 *      never winds up past it, plus bumpless retune and a clean restart on arm.
 *
 * SAFETY POSTURE: this module trusts NOTHING it is handed. Every input to
 * pid_update() is range-checked against the contract below, asserted (to catch
 * a caller/boundary bug during test), AND clamped (so a bad value can never
 * propagate into the math even if asserts are compiled out). Slow, checked code
 * is preferred over speed everywhere here.
 *
 * All state is caller-owned; nothing is allocated. All math is integer
 * (int32/int64); no floating point anywhere, matching the C6's FPU-less core.
 */

#ifndef OVEN_PID_H_
#define OVEN_PID_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Input validity contract for pid_update().
 *
 * Gains are fixed-point x1000 and must be NON-NEGATIVE (a negative gain would
 * invert the sign of control for a heat-only actuator) and bounded well below
 * the point where the fixed-point products could stress int64. The real range
 * check belongs at the write boundary (OPC UA / shell); these bounds are this
 * module's own last-resort net, not a substitute for that.
 *
 * Setpoint and measurement are validated against the physical temperature
 * window (OVEN_TEMP_VALID_MIN_CC / _MAX_CC in shared.h).
 */
#define OVEN_PID_GAIN_M_MIN   (0)          /* gains are >= 0                    */
#define OVEN_PID_GAIN_M_MAX   (1000000)    /* gain magnitude <= 1000.000 (x1000) */

/* Tuning parameters, fixed-point x1000 (see OVEN_GAIN_SCALE). */
struct pid_gains {
	int32_t kp_m;   /* proportional gain x1000, in [MIN,MAX] */
	int32_t ki_m;   /* integral gain     x1000, in [MIN,MAX] */
	int32_t kd_m;   /* derivative gain   x1000, in [MIN,MAX] (0 = PI) */
};

/*
 * ---- Per-cycle diagnostics ------------------------------------------------
 *
 * A snapshot of the control law's internals, so a diagnostics consumer can show
 * WHY the controller asked for a given power instead of just the number. Filled
 * by pid_update() as a straight sequence of assignments into the caller's own
 * state: no locking, no allocation, nothing that can block or fail.
 *
 * DELIBERATELY UNCONDITIONAL. There is no debug/release variant of this file:
 * the capture is compiled and executed identically in every build, so the
 * instruction path, the timing and the stack usage you validate on the bench
 * are the ones that ship. A field image simply has no consumer for the data and
 * publishes it into a buffer nobody reads. Do not wrap any of this in a Kconfig
 * guard — that would make the control law's behaviour configuration-dependent,
 * which is exactly what this project refuses to do.
 *
 * The four signal fields are in the control law's INTERNAL scale, where one
 * actuator half-cycle is OVEN_PID_TRACE_SCALE. Anything that formats them must
 * scale down; nothing here is a float.
 */
#define OVEN_PID_TRACE_SCALE  (1000000LL)

/* Which rail the unsaturated sum was pinned to — this is the anti-windup event. */
enum pid_sat {
	PID_SAT_LOW  = -1,   /* pinned at 0; the actuator cannot cool         */
	PID_SAT_NONE = 0,    /* inside the linear range                       */
	PID_SAT_HIGH = 1,    /* pinned at OVEN_HEATER_MAX_POWER               */
};

struct pid_diag {
	int32_t sp_cc;          /* validated setpoint,     centi-degC         */
	int32_t pv_cc;          /* validated measurement,  centi-degC         */
	int32_t e_cc;           /* error = sp - pv,        centi-degC         */
	int32_t dpv_cc;         /* pv[k] - pv[k-1],        centi-degC         */
	struct pid_gains g;     /* gains actually used (validated + clamped)  */
	int64_t yp_u;           /* P contribution,         internal scale     */
	int64_t yi_u;           /* I accumulator AFTER this step              */
	int64_t yd_u;           /* D contribution,         internal scale     */
	int64_t y_unsat_u;      /* yp+yi+yd BEFORE clamping (0 when inactive) */
	uint32_t out;           /* final command, half-cycles                 */
	int8_t sat;             /* enum pid_sat — the windup indicator        */
	bool retuned;           /* bumpless retune ran this step              */
	bool arm_edge;          /* DISARMED->ARMED: integrator restarted      */
	bool active;            /* controller was allowed to act              */
};

/* Controller state — caller-owned, zeroed by pid_reset(). */
struct pid_state {
	int64_t yi_u;        /* integral accumulator, internal fixed-point   */
	int64_t yd_u;        /* derivative-filter state, internal fixed-point */
	int64_t y_prev_u;    /* last output, internal fixed-point (bumpless)  */
	int32_t e_prev;      /* previous error,       centi-degC             */
	int32_t pv_prev;     /* previous measurement, centi-degC             */
	struct pid_gains g_prev;  /* previously-applied (validated) gains    */
	bool primed;         /* histories valid                              */
	bool active_prev;    /* previous active flag (arm-edge detection)    */
	struct pid_diag diag;     /* last cycle's internals (always filled)  */
};

/* Reset all state to a safe, un-wound-up zero. */
void pid_reset(struct pid_state *st);

/*
 * Advance the control law by one 500 ms step and return the actuator command,
 * always within [0, OVEN_HEATER_MAX_POWER] half-cycles.
 *
 *   st        : caller-owned state (must be non-NULL; asserted).
 *   g         : current gains (must be non-NULL; each field validated+clamped).
 *   setpoint_cc, pv_cc : setpoint and control-zone measurement, centi-degC,
 *               validated+clamped to the physical window.
 *   active    : true only when ARMED and the fault word is clear. When false,
 *               the output is forced to 0, the integrator is frozen (no windup
 *               while disarmed), and the derivative filter keeps tracking the
 *               measurement so arming produces no kick.
 */
uint32_t pid_update(struct pid_state *st, const struct pid_gains *g,
		    int32_t setpoint_cc, int32_t pv_cc, bool active);

#endif /* OVEN_PID_H_ */
