/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Contactor feedback — the dry contact on opto_in1, read as proof of what the
 * power path is actually doing rather than what the firmware believes it
 * commanded.
 *
 * TRUST NO LEVEL. The sense is a phototransistor grounding an external pull-up,
 * read ACTIVE LOW: contact closed -> LED lit -> transistor conducts -> line low.
 * It can fail EITHER WAY —
 *
 *   transistor shorted, or the line shorted to ground -> reads always CLOSED
 *   transistor open, LED dead, or a broken wire       -> reads always OPEN
 *
 * — so there is no "safe" direction to assume, and this file deliberately
 * assumes neither. An earlier version of this comment (and of the board overlay)
 * claimed a failed opto reads as CLOSED. That claim is unfounded; do not
 * reintroduce it, and do not build anything on top of one.
 *
 * What makes the feedback trustworthy is not its resting level but the fact that
 * the interlock sequence REQUIRES IT TO CHANGE, on command, before the oven may
 * arm: silent watchdog must read open, then a hand-closed contactor must be
 * observed going closed. A sensor stuck at either level fails one of those two
 * phases and the oven never becomes armable. See oven_state.h for the sequence
 * and ARCHITECTURE.md for the per-failure analysis.
 *
 * The one thing this file does assume is that a READ ERROR is not permission to
 * heat — see contactor_is_closed().
 */

#ifndef OVEN_CONTACTOR_H_
#define OVEN_CONTACTOR_H_

#include <stdbool.h>

/*
 * True when the contactor reads CLOSED — or when the feedback is unavailable,
 * which is deliberately indistinguishable. Lock-free; a single GPIO read.
 */
bool contactor_is_closed(void);

/* Feedback input present and usable. */
bool contactor_devices_ready(void);

/*
 * Power-on proof that the power path starts open.
 *
 * The firmware must only come up ready to arm if it observed the contactor OPEN
 * at boot. A contactor already closed before any pulse train has been produced
 * cannot have been closed by this firmware — it is welded, stuck, or the
 * feedback is dead, and none of those may be allowed to reach ARMED.
 *
 * Returns true if the contactor was observed open. On false it has already
 * raised OVEN_FAULT_CONTACTOR_WELD, which latches FAULTED and makes arming
 * impossible until the fault is cleared by a reboot with the contactor open.
 *
 * Called by main() before leaving INIT. Samples repeatedly and requires EVERY
 * sample to read open, so a bouncing or intermittent contact fails the check.
 */
bool contactor_boot_check(void);

#endif /* OVEN_CONTACTOR_H_ */
