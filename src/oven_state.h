/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * System state machine (architecture, "Safe State & Fault Latching").
 *
 *      INIT --(1)--> AWAIT_CONTACTOR --(2)--> DISARMED <====> ARMED
 *        |                  ^                     |             |
 *        |                  +-------(3)-----------+-------------+
 *        |  (restored              contactor dropped
 *        |   fault)        |             |             |
 *        +-----------------+-------------+-------------+--> FAULTED
 *                                                 (terminal until reboot)
 *
 * (1) The Watchdog thread holds its health pulse SILENT for a full quiet window
 *     and observes the contactor OPEN throughout. The interlock cannot hold the
 *     contactor closed without that pulse, so a contactor still closed here is
 *     welded — that is the weld test, and it runs at every boot.
 *
 * (2) The Watchdog starts pulsing, which merely ENABLES the interlock. Closing
 *     the contactor is a manual, hardware act: the operator presses the physical
 *     button. The firmware then has to SEE the open->closed transition. Nothing
 *     is taken on trust: this proves the contactor moves, the interlock latches,
 *     and the feedback path works — all before the oven can be armed.
 *
 * (3) Losing the contactor afterwards returns the system to AWAIT_CONTACTOR, so
 *     the manual step must be repeated. Not a fault: no power path means no
 *     hazard, only a machine that refuses to heat.
 *
 * Invariants:
 *   - The heater can NEVER self-arm. Cold boot always lands in AWAIT_CONTACTOR
 *     (or FAULTED); reaching ARMED requires the physical interlock to have been
 *     closed by hand AND an explicit operator arm request.
 *   - The firmware cannot close the contactor by itself, and the hardware cannot
 *     close it without the firmware: the interlock loop needs a healthy watchdog
 *     pulse. Neither side can energize the machine alone.
 *   - DISARMED implies the contactor is closed and proven. It is NOT a state in
 *     which the power path is open.
 *   - FAULTED is entered only via fault_raise() and is terminal at runtime.
 *   - The state is an atomic so any thread may read it lock-free.
 */

#ifndef OVEN_STATE_H_
#define OVEN_STATE_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Renumbered when AWAIT_CONTACTOR was introduced, so the values follow the flow.
 * Nothing persists a state value (only the fault word survives a reboot), so the
 * renumber is safe; keep it that way if you add another.
 */
enum oven_state {
	OVEN_STATE_INIT            = 0,
	OVEN_STATE_AWAIT_CONTACTOR = 1,
	OVEN_STATE_DISARMED        = 2,
	OVEN_STATE_ARMED           = 3,
	OVEN_STATE_FAULTED         = 4,
};

/*
 * INIT -> AWAIT_CONTACTOR. Called by the Watchdog thread once it has held its
 * health pulse silent for the quiet window and seen the contactor open the whole
 * time. Refused if a fault is already latched. Returns true on the transition.
 */
bool oven_state_interlock_ready(void);

/*
 * AWAIT_CONTACTOR -> DISARMED. Called by the Watchdog thread when it observes
 * the contactor CLOSED while it is pulsing — i.e. the operator has closed the
 * physical interlock and the whole chain demonstrably works. Returns true on the
 * transition.
 */
bool oven_state_contactor_confirmed(void);

/*
 * DISARMED or ARMED -> AWAIT_CONTACTOR. The contactor dropped while we were
 * still asking for it. Not a fault — there is no power path, so no hazard — but
 * the manual interlock step has to be repeated before the oven can arm again.
 */
void oven_state_contactor_lost(void);

/* Force FAULTED. Idempotent, terminal. Called from fault_raise(). */
void oven_state_enter_faulted(void);

/*
 * Operator arm request (future: physical button edge). Refused unless the
 * current state is DISARMED and the fault word is clear. Returns true on the
 * DISARMED -> ARMED transition, false otherwise.
 */
bool oven_state_request_arm(void);

/* Operator disarm request. ARMED -> DISARMED. No effect once FAULTED. */
void oven_state_request_disarm(void);

/* Lock-free current state. */
enum oven_state oven_state_get(void);

/* Human-readable name for logging. */
const char *oven_state_name(enum oven_state st);

#endif /* OVEN_STATE_H_ */
