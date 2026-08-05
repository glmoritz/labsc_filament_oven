/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared inter-thread state (architecture, "Shared Variables").
 *
 * ALL cross-thread communication is through the atomics declared here. Rules:
 *   - atomic_t is a 32-bit integer; floats are never stored. Everything is
 *     fixed-point (temperature in centi-degC, gains x1000).
 *   - Liveness uses monotonic SEQUENCE COUNTERS, never booleans; readers use
 *     unsigned "advanced since last sample" comparisons (seq_advanced()).
 *   - The fault word is an OR-only bitmask. Bits are set with fault_raise();
 *     they are NEVER cleared at runtime, only by the reboot-into-FAULTED path.
 *
 * No dynamic allocation anywhere in this project.
 */

#ifndef OVEN_SHARED_H_
#define OVEN_SHARED_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>

/* --- Fixed-point scaling ------------------------------------------------- */
#define OVEN_CENTI              (100)      /* 1 degC = 100 centi-degC          */
#define OVEN_GAIN_SCALE         (1000)     /* PID gains stored x1000           */

typedef int32_t oven_temp_cc_t;            /* temperature, centi-degC          */

/* --- Compile-time safety configuration ----------------------------------- *
 * These are NOT in the OPC UA writable address space (architecture, "Boundary
 * rules"): a compromised client must not be able to move the safety limit.
 * Values are filament-dryer placeholders; retune per the physical oven.
 *
 * The absolute limit applies to SENSOR B = the HEAT CHAMBER (architecture,
 * "Role separation"), which always runs hotter than the filament chamber the
 * loop controls.
 *
 * It is the FIRST rung of a three-rung thermal ladder, and its value only makes
 * sense read against the other two:
 *
 *   110 degC  this limit          firmware, recoverable, latches FAULTED
 *   150 degC  bimetallic thermostat  drops the contactor COIL
 *   300 degC  thermal fuse         in series with the heater element, one-shot
 *
 * Each rung must trip well before the next, so the cheap recoverable one acts
 * first and the destructive one is never reached in normal fault handling. The
 * 40 degC gap to the bimetallic is what keeps a firmware trip from racing the
 * mechanical one.
 *
 * Note which rung is actually the last line: the bimetallic acts through the
 * contactor coil, so a welded contactor defeats it (architecture, layer 6 —
 * "it must NOT act via the contactor coil"). The 300 degC fuse, in series with
 * the element, is the one that survives a weld. Do not treat the bimetallic as
 * the backstop.
 *
 * [VERIFY] the lower bound: 110 degC must stay above the heat chamber's
 * worst-case steady-state temperature with the filament chamber at its maximum
 * setpoint, or the oven trips during a normal heat-up. That figure comes from
 * the physical oven and has not been measured yet.
 */
#define OVEN_SENSOR_B_ABS_LIMIT_CC   (11000)   /* 110.00 degC protection limit */
#define OVEN_TEMP_VALID_MIN_CC       (-2000)   /* -20.00 degC plausibility     */
#define OVEN_TEMP_VALID_MAX_CC       (30000)   /* 300.00 degC plausibility     */

/* --- Timing (architecture, "Timing & Deadline Detection") ----------------- */
#define OVEN_PID_PERIOD_MS           (500U)
#define OVEN_OUTPUT_PERIOD_US        (8333U)   /* 120 Hz, coupled to 60 Hz mains */
#define OVEN_WATCHDOG_PERIOD_MS      (100U)
#define OVEN_PID_DEADLINE_MARGIN_MS  (50U)     /* single-miss detection margin */

/* PID staleness window seen by the Output thread: <= 2 control periods (1 s). */
#define OVEN_PID_STALE_LIMIT_CYCLES  (120U)    /* 1 s / 8.333 ms               */

/* Liveness window seen by the Watchdog thread, in watchdog cycles. */
#define OVEN_LIVENESS_LIMIT_CYCLES   (5U)      /* 500 ms / 100 ms              */

#define OVEN_HEATER_MAX_POWER        (120U)    /* half-cycles per second       */

/* --- Fault word bits (OR-only) ------------------------------------------- */
enum oven_fault_bit {
	OVEN_FAULT_PID_DEADLINE   = (1 << 0),  /* control loop deadline exceeded  */
	OVEN_FAULT_PID_STALE      = (1 << 1),  /* PID sequence counter stale      */
	OVEN_FAULT_OUTPUT_STALE   = (1 << 2),  /* Output sequence counter stale   */
	OVEN_FAULT_SENSOR_A       = (1 << 3),  /* control TC open / comm fault    */
	OVEN_FAULT_SENSOR_B       = (1 << 4),  /* protection TC open / comm fault */
	OVEN_FAULT_SENSOR_B_LIMIT = (1 << 5),  /* protection absolute limit hit   */
	OVEN_FAULT_PROCESS_DIAG   = (1 << 6),  /* process diagnostics inconsistent*/
	OVEN_FAULT_TIMER_STALL    = (1 << 7),  /* release timer not progressing   */
	OVEN_FAULT_SPI_COMM       = (1 << 8),  /* SPI transfer error / timeout    */
	OVEN_FAULT_DEVICE_INIT    = (1 << 9),  /* a required device is not ready  */
	OVEN_FAULT_BROWNOUT       = (1 << 10), /* brownout reset reason           */
	OVEN_FAULT_RESTORED       = (1 << 11), /* a latched fault survived reboot */
	OVEN_FAULT_ASSERT         = (1 << 13), /* firmware invariant failed (soft assert) */
};


/* --- Shared atomics (defined in shared.c) -------------------------------- */
extern atomic_t g_setpoint_cc;      /* control setpoint, centi-degC           */
extern atomic_t g_kp_m;             /* PID gains x1000                        */
extern atomic_t g_ki_m;
extern atomic_t g_kd_m;
extern atomic_t g_heater_power;     /* 0..OVEN_HEATER_MAX_POWER               */
extern atomic_t g_temp_a_cc;        /* SENSOR A = FILAMENT chamber (control)  */
extern atomic_t g_temp_b_cc;        /* SENSOR B = HEAT chamber (protection)   */
extern atomic_t g_pid_seq;          /* monotonic liveness counter             */
extern atomic_t g_output_seq;       /* monotonic liveness counter             */
extern atomic_t g_timer_ticks;      /* 500 ms release-timer heartbeat         */
extern atomic_t g_fault_word;       /* OR-only bitmask of oven_fault_bit      */
extern atomic_t g_init_done;        /* 0 until main() has configured the HW   */

/*
 * Raise one or more fault bits and latch the system into FAULTED. Callable from
 * any thread. OR-only: never clears. Defined in shared.c.
 */
void fault_raise(uint32_t bits);

/* Convenience reader. */
static inline uint32_t fault_word_get(void)
{
	return (uint32_t)atomic_get(&g_fault_word);
}

/*
 * Liveness comparison. Returns true if the monotonic counter advanced from
 * prev to now. Uses unsigned subtraction so it is wraparound-safe; never
 * compare a sequence counter for equality-to-expected.
 */
static inline bool seq_advanced(uint32_t now, uint32_t prev)
{
	return ((uint32_t)(now - prev) != 0U);
}

#endif /* OVEN_SHARED_H_ */
