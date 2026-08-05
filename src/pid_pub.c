/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * PID cycle publication — seqlock implementation. See pid_pub.h for why this
 * module exists and why it is compiled into every image.
 */

#include "pid_pub.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>

/*
 * Sequence counter. Even = the buffer is stable, odd = a write is in progress.
 * The writer increments it twice per publication; a reader that sees the same
 * even value before and after its copy knows no write overlapped.
 */
static atomic_t s_seq = ATOMIC_INIT(0);
static struct pid_pub s_pub;

/*
 * Reader retry budget. The writer's critical section is a handful of stores, so
 * one retry is already generous; three is a bounded, obviously-terminating net.
 * The consumer runs at a preemptible priority below the writer, so this can
 * never delay the control loop no matter how it ends.
 */
#define PID_PUB_READ_TRIES   (3)

/* Signals a fresh publication. Max count 1: a slow consumer coalesces cycles
 * instead of building a backlog it would then chase.
 */
static K_SEM_DEFINE(s_cycle, 0, 1);

void pid_pub_publish(uint32_t cycle, uint32_t delta_ms, const struct pid_diag *diag)
{
	if (diag == NULL) {
		return;
	}

	(void)atomic_inc(&s_seq);          /* -> odd: write in progress */
	barrier_dmem_fence_full();

	s_pub.cycle = cycle;
	s_pub.delta_ms = delta_ms;
	s_pub.diag = *diag;

	barrier_dmem_fence_full();
	(void)atomic_inc(&s_seq);          /* -> even: stable again */

	k_sem_give(&s_cycle);
}

bool pid_pub_snapshot(struct pid_pub *out)
{
	if (out == NULL) {
		return false;
	}

	for (int try = 0; try < PID_PUB_READ_TRIES; try++) {
		uint32_t before;
		struct pid_pub tmp;

		before = (uint32_t)atomic_get(&s_seq);
		if ((before & 1U) != 0U) {
			/* Writer is mid-update; don't even copy. */
			continue;
		}

		barrier_dmem_fence_full();
		tmp = s_pub;
		barrier_dmem_fence_full();

		if ((uint32_t)atomic_get(&s_seq) == before) {
			*out = tmp;
			return true;
		}
	}

	return false;
}

bool pid_pub_wait(k_timeout_t timeout)
{
	return (k_sem_take(&s_cycle, timeout) == 0);
}
