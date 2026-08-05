/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * PID cycle publication — the one-way boundary between the control loop and any
 * diagnostics consumer.
 *
 * DESIGN RULE THIS FILE EXISTS TO ENFORCE: the control path has no debug and no
 * release variant. pid.c and pid_thread.c contain not one conditional; every
 * build computes the same values, publishes them the same way, and takes the
 * same time doing it. What changes between a bench image and a field image is
 * only whether anything is compiled in to READ the publication. In a field image
 * the data is written into a buffer nobody looks at — deliberately, because a
 * loop whose timing depends on the build configuration is a loop you have not
 * actually validated.
 *
 * Concurrency: the writer is the PID thread (cooperative, K_PRIO_COOP(1)); the
 * reader is a preemptible diagnostics thread that the writer can preempt at any
 * instruction. A plain struct copy could therefore be torn. The publication is a
 * SEQLOCK: the writer brackets its update with an odd/even counter and never
 * blocks or spins; the reader retries a bounded number of times and reports
 * failure rather than looping forever (architecture, "no unbounded blocking").
 * A torn read costs one skipped trace line and nothing else.
 */

#ifndef OVEN_PID_PUB_H_
#define OVEN_PID_PUB_H_

#include "pid.h"

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

/* One control cycle, as published. */
struct pid_pub {
	uint32_t cycle;      /* g_pid_seq value for this cycle                */
	uint32_t delta_ms;   /* measured release-to-release interval, ms      */
	struct pid_diag diag;
};

/*
 * Publish one cycle. Called unconditionally by the PID thread at the end of
 * every cycle. Wait-free: a bounded sequence of stores, no locking, no
 * allocation, no possibility of blocking the control loop.
 *
 * Also signals the cycle semaphore below. With no consumer compiled in, the
 * semaphore simply saturates at 1 and nothing ever takes it.
 */
void pid_pub_publish(uint32_t cycle, uint32_t delta_ms, const struct pid_diag *diag);

/*
 * Copy the most recent stable publication into *out. Returns false if the
 * writer kept winning the race for the whole retry budget, in which case *out
 * is untouched and the caller should just skip this cycle.
 */
bool pid_pub_snapshot(struct pid_pub *out);

/*
 * Wait for the next published cycle. Returns false on timeout. Only a
 * diagnostics consumer should call this; the control path never waits on it.
 */
bool pid_pub_wait(k_timeout_t timeout);

#endif /* OVEN_PID_PUB_H_ */
