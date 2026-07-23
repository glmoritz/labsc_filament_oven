/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * System state machine (architecture, "Safe State & Fault Latching").
 *
 *      INIT ---------> DISARMED <====> ARMED
 *        |                 |             |
 *        |  (restored      |  fault_word != 0 (any state)
 *        |   fault)        v             v
 *        +-------------> FAULTED (terminal until reboot + operator action)
 *
 * Invariants:
 *   - The heater can NEVER self-arm. Cold boot always lands in DISARMED (or
 *     FAULTED); reaching ARMED requires an explicit operator arm request.
 *   - FAULTED is entered only via fault_raise() and is terminal at runtime.
 *   - The state is an atomic so any thread may read it lock-free.
 */

#ifndef OVEN_STATE_H_
#define OVEN_STATE_H_

#include <stdint.h>
#include <stdbool.h>

enum oven_state {
	OVEN_STATE_INIT     = 0,
	OVEN_STATE_DISARMED = 1,
	OVEN_STATE_ARMED    = 2,
	OVEN_STATE_FAULTED  = 3,
};

/* Leave INIT for DISARMED once hardware is up (no fault) — called by main(). */
void oven_state_boot_ok(void);

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
