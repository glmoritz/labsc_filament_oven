/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Defensive-clamp test for src/pid.c. Compiled with -DNDEBUG so the __ASSERT
 * contract checks become no-ops and the production CLAMP net is what runs. It
 * feeds deliberately garbage inputs (out-of-range and extreme gains, absurd
 * temperatures) and verifies the controller never produces an out-of-range
 * output and never trusts the bad values -- i.e. that "assume nothing" holds
 * even when asserts are disabled.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

#define OK(cond) do{ int _r=(cond); printf("  %-56s %s\n", #cond, _r?"PASS":"*** FAIL ***"); if(!_r) fails++; }while(0)
int fails=0;

int main(void)
{
	struct pid_state st;
	uint32_t o;
	int bad_out = 0;

	printf("A) garbage gains + extreme temps -> output always in [0,120]\n");
	pid_reset(&st);
	/* kp far above max, ki negative, kd far above max; INT32 extreme temps. */
	struct pid_gains junk = { 50000000, -1000, 2000000000 };
	for (int i = 0; i < 1000; i++) {
		int32_t sp = (i & 1) ? 2147483647 : -2147483647;
		int32_t pv = (i & 2) ? -2147483647 : 2147483647;
		o = pid_update(&st, &junk, sp, pv, true);
		if (o > 120u) { bad_out = 1; }
	}
	OK(bad_out == 0);

	printf("B) out-of-range gains behave EXACTLY like their clamped values\n");
	struct pid_state s_bad, s_ref;
	pid_reset(&s_bad); pid_reset(&s_ref);
	struct pid_gains over  = { 50000000, 9000000, -5 };   /* kp,ki over max; kd<min */
	struct pid_gains clamp = { 1000000, 1000000, 0 };     /* the expected clamps    */
	int mismatch = 0;
	for (int i = 0; i < 50; i++) {
		uint32_t a = pid_update(&s_bad, &over,  2200, 2000, true);
		uint32_t b = pid_update(&s_ref, &clamp, 2200, 2000, true);
		if (a != b) { mismatch = 1; }
	}
	OK(mismatch == 0);

	printf("C) absurd setpoint is clamped to the window (no implausible error)\n");
	pid_reset(&st);
	struct pid_gains p = { 1000, 0, 0 };  /* Kp=1.0 */
	/* setpoint 9,999,999 cC clamps to 30000; pv=2000 -> e=28000 -> P saturates. */
	o = pid_update(&st, &p, 9999999, 2000, true);
	OK(o == 120);
	/* setpoint -9,999,999 clamps to -2000; pv=2000 -> e=-4000 -> output 0. */
	o = pid_update(&st, &p, -9999999, 2000, true);
	OK(o == 0);

	printf("\n%s (%d failures)\n", fails ? "FAILURES" : "ALL PASS", fails);
	return fails ? 1 : 0;
}
