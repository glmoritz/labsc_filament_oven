/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Process-response diagnostics. See diagnostics.h for what these are for and
 * why they are deliberately blunt.
 */

#include "diagnostics.h"
#include "shared.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oven_diag, LOG_LEVEL_INF);

/* Cycles per second of wall time, from the control period. */
#define CYCLES_PER_S            (1000U / OVEN_PID_PERIOD_MS)

/*
 * ---- 1. Frozen reading ----------------------------------------------------
 *
 * A reading that is BIT-IDENTICAL for this long is not a temperature, it is a
 * dead converter, a stuck bus, or a driver returning its last buffer. The
 * MAX6675 quantises to 0.25 degC, and no real thermocouple in a real enclosure
 * holds one quantisation step for minutes — thermal and electrical noise dither
 * the least significant bit continuously. (Watch the bench trace: the filament
 * chamber wanders across 24.00/24.25/24.50 within seconds.)
 *
 * This is the check that catches the failure the MAX6675 cannot report: it has
 * open-thermocouple detection only, so a converter that answers with a constant
 * — including a constant 0.00 degC from an absent or unselected device — passes
 * every range and validity test in the acquisition path.
 *
 * [VERIFY] the window against a real oven at steady state. Five minutes is
 * chosen to be far longer than any plausible quiet period; shorten it only with
 * evidence, because a false trip here stops a healthy oven mid-batch.
 */
#define DIAG_FROZEN_WINDOW_S    (300U)                      /* 5 minutes */
#define DIAG_FROZEN_CYCLES      (DIAG_FROZEN_WINDOW_S * CYCLES_PER_S)

/*
 * ---- Evaluation window for the two windowed checks ------------------------
 * Long enough that thermal lag and sensor noise cannot fake a trend, short
 * enough to catch a runaway well before the absolute limit does.
 */
#define DIAG_WINDOW_S           (60U)                       /* 1 minute */
#define DIAG_WINDOW_CYCLES      (DIAG_WINDOW_S * CYCLES_PER_S)

/*
 * ---- 2. Zone inversion ----------------------------------------------------
 *
 * Heat only reaches the filament chamber from the heat chamber, carried by the
 * fan. So WHENEVER THE FILAMENT CHAMBER IS RISING, the heat chamber must be
 * hotter than it — that is not a heuristic, it is the direction heat flows.
 *
 * Gating the check on "filament rising" is what makes it usable. A blanket
 * "heat >= filament" would fire on every cold start (both at ambient, noise
 * decides the order) and on every cool-down (the heat chamber sheds heat first).
 * Restricted to the moment the physics actually guarantees the ordering, it has
 * no legitimate counterexample.
 *
 * This is also the only software check that can catch the A<->B connector swap,
 * which the architecture otherwise lists as "mechanical prevention only": with
 * the sensors exchanged, the "heat chamber" input reports the cooler zone and
 * the inversion is permanent once heating starts.
 *
 * TWO GATES, both learned the hard way on the bench:
 *
 *   1. The rise must be REAL, not noise. An early version used one MAX6675 step
 *      (0.25 degC) as "rising" and faulted an idle oven inside two minutes: the
 *      filament reading dithers by a step continuously, so the threshold was
 *      measuring quantisation. 2 degC over the window is a trend.
 *
 *   2. We must have been HEATING. The ordering is guaranteed by heat flowing
 *      from the heat chamber through the fan — nothing else. A room warming up
 *      raises both zones with no such flow, and about that case this check has
 *      no opinion and must stay silent.
 */
#define DIAG_RISE_CC            (200)    /* 2.00 degC — a trend, not a step */
#define DIAG_INVERSION_CC       (500)    /* 5.00 degC of margin             */

/*
 * ---- 3. Powered but flat --------------------------------------------------
 *
 * Sustained near-full power that produces no rise in the heat chamber means the
 * commanded energy is not arriving where the sensor is: heater open, SSR not
 * conducting, contactor open, or the protection thermocouple detached from the
 * zone it is supposed to be measuring.
 *
 * Deliberately requires NEAR-FULL power for the whole window. At moderate duty
 * near setpoint a flat temperature is exactly correct behaviour, and must not
 * fault.
 */
#define DIAG_ONFLAT_POWER_MIN   ((OVEN_HEATER_MAX_POWER * 9U) / 10U)   /* 90% */
#define DIAG_ONFLAT_RISE_CC     (100)    /* 1.00 degC over the window      */

struct frozen_track {
	oven_temp_cc_t last;
	uint32_t cycles;
	bool seeded;
	bool reported;
};

static struct frozen_track s_frozen_filament;
static struct frozen_track s_frozen_heat;

static uint32_t s_window_cycles;
static oven_temp_cc_t s_filament_at_start;
static oven_temp_cc_t s_heat_at_start;
static bool s_window_seeded;
static bool s_powered_all_window;
static bool s_heated_any_window;

void diag_reset(void)
{
	s_frozen_filament = (struct frozen_track){0};
	s_frozen_heat = (struct frozen_track){0};
	s_window_cycles = 0U;
	s_filament_at_start = 0;
	s_heat_at_start = 0;
	s_window_seeded = false;
	s_powered_all_window = true;
	s_heated_any_window = false;
}

/*
 * One sensor's frozen-reading check. `name` is a literal, so logging it is safe
 * under deferred logging.
 */
static void check_frozen(struct frozen_track *t, oven_temp_cc_t value,
			 bool valid, const char *name)
{
	if (!valid) {
		/* Acquisition already faulted; don't also age the history. */
		return;
	}

	if (!t->seeded) {
		t->last = value;
		t->cycles = 0U;
		t->seeded = true;
		return;
	}

	if (value != t->last) {
		t->last = value;
		t->cycles = 0U;
		t->reported = false;
		return;
	}

	t->cycles++;
	if ((t->cycles >= DIAG_FROZEN_CYCLES) && !t->reported) {
		t->reported = true;
		LOG_ERR("%s reading frozen at %d cC for %u s — dead converter, "
			"stuck bus, or absent sensor", name, (int)value,
			DIAG_FROZEN_WINDOW_S);
		fault_raise((uint32_t)OVEN_FAULT_PROCESS_DIAG);
	}
}

void diag_update(oven_temp_cc_t filament_cc, bool filament_ok,
		 oven_temp_cc_t heat_cc, bool heat_ok,
		 uint32_t power, bool heating)
{
	/* ---- 1. Frozen readings: always checked, in every state. ---- */
	check_frozen(&s_frozen_filament, filament_cc, filament_ok, "filament chamber");
	check_frozen(&s_frozen_heat, heat_cc, heat_ok, "heat chamber");

	/*
	 * The windowed checks compare both zones, so a cycle in which either read
	 * failed cannot contribute. Restart the window rather than carry a gap.
	 */
	if (!filament_ok || !heat_ok) {
		s_window_seeded = false;
		return;
	}

	if (!s_window_seeded) {
		s_filament_at_start = filament_cc;
		s_heat_at_start = heat_cc;
		s_window_cycles = 0U;
		s_powered_all_window = true;
		s_heated_any_window = false;
		s_window_seeded = true;
		return;
	}

	/* Full power for the WHOLE window (check 3) vs any heating at all (check 2). */
	if (!heating || (power < DIAG_ONFLAT_POWER_MIN)) {
		s_powered_all_window = false;
	}
	if (heating && (power > 0U)) {
		s_heated_any_window = true;
	}

	s_window_cycles++;
	if (s_window_cycles < DIAG_WINDOW_CYCLES) {
		return;
	}

	/* ---- 2. Zone inversion: only while heating AND the filament rises. ---- */
	if (s_heated_any_window &&
	    ((filament_cc - s_filament_at_start) >= DIAG_RISE_CC)) {
		if (heat_cc < (filament_cc + DIAG_INVERSION_CC)) {
			LOG_ERR("filament chamber rising (%d -> %d cC) but heat "
				"chamber only %d cC — zones inverted, sensors "
				"swapped, or heat-chamber sensor detached",
				(int)s_filament_at_start, (int)filament_cc,
				(int)heat_cc);
			fault_raise((uint32_t)OVEN_FAULT_PROCESS_DIAG);
		}
	}

	/* ---- 3. Powered but flat. ---- */
	if (s_powered_all_window &&
	    ((heat_cc - s_heat_at_start) < DIAG_ONFLAT_RISE_CC)) {
		LOG_ERR("heater commanded >= %u/%u for %u s but heat chamber "
			"moved only %d cC (%d -> %d) — no energy arriving",
			DIAG_ONFLAT_POWER_MIN, (uint32_t)OVEN_HEATER_MAX_POWER,
			DIAG_WINDOW_S, (int)(heat_cc - s_heat_at_start),
			(int)s_heat_at_start, (int)heat_cc);
		fault_raise((uint32_t)OVEN_FAULT_PROCESS_DIAG);
	}

	/* Roll the window forward. */
	s_window_seeded = false;
}
