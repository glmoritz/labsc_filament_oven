/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bench / test shell commands — the seed of the fault-injection harness
 * (architecture, "Test & Validation Plan"). Compiled to nothing unless a shell
 * is enabled (debug.conf), so it never reaches a field image.
 *
 *   oven status            show state, fault word, liveness, temperatures
 *   oven arm               operator arm request (DISARMED -> ARMED)
 *   oven disarm            operator disarm request
 *   oven fault <hexbits>   inject fault bits (latches FAULTED)
 *   oven sp <centi-degC>   set control setpoint
 *   oven gains <kp> <ki> <kd>   set PID gains (each x1000)
 */

#ifdef CONFIG_SHELL

#include "shared.h"
#include "oven_state.h"

#include <zephyr/shell/shell.h>
#include <stdlib.h>

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "state   : %s", oven_state_name(oven_state_get()));
	shell_print(sh, "fault   : 0x%08x", fault_word_get());
	shell_print(sh, "temp A  : %ld cC", (long)atomic_get(&g_temp_a_cc));
	shell_print(sh, "temp B  : %ld cC", (long)atomic_get(&g_temp_b_cc));
	shell_print(sh, "heater  : %ld", (long)atomic_get(&g_heater_power));
	shell_print(sh, "pid_seq : %ld", (long)atomic_get(&g_pid_seq));
	shell_print(sh, "out_seq : %ld", (long)atomic_get(&g_output_seq));
	shell_print(sh, "tmr_tick: %ld", (long)atomic_get(&g_timer_ticks));

	return 0;
}

static int cmd_arm(const struct shell *sh, size_t argc, char **argv)
{
	bool armed;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	armed = oven_state_request_arm();
	shell_print(sh, "arm request: %s (state=%s)",
		    armed ? "accepted" : "refused",
		    oven_state_name(oven_state_get()));

	return 0;
}

static int cmd_disarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	oven_state_request_disarm();
	shell_print(sh, "disarmed (state=%s)", oven_state_name(oven_state_get()));

	return 0;
}

static int cmd_fault(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long bits;

	if (argc != 2) {
		shell_error(sh, "usage: oven fault <hexbits>");
		return -EINVAL;
	}

	bits = strtoul(argv[1], NULL, 16);
	fault_raise((uint32_t)bits);
	shell_print(sh, "injected 0x%08lx -> fault=0x%08x state=%s",
		    bits, fault_word_get(), oven_state_name(oven_state_get()));

	return 0;
}

static int cmd_sp(const struct shell *sh, size_t argc, char **argv)
{
	long sp;

	if (argc != 2) {
		shell_error(sh, "usage: oven sp <centi-degC>");
		return -EINVAL;
	}

	sp = strtol(argv[1], NULL, 10);
	(void)atomic_set(&g_setpoint_cc, (atomic_val_t)sp);
	shell_print(sh, "setpoint = %ld cC", sp);

	return 0;
}

static int cmd_gains(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "usage: oven gains <kp_m> <ki_m> <kd_m>  (each x1000)");
		return -EINVAL;
	}

	(void)atomic_set(&g_kp_m, (atomic_val_t)strtol(argv[1], NULL, 10));
	(void)atomic_set(&g_ki_m, (atomic_val_t)strtol(argv[2], NULL, 10));
	(void)atomic_set(&g_kd_m, (atomic_val_t)strtol(argv[3], NULL, 10));
	shell_print(sh, "gains x1000: kp=%ld ki=%ld kd=%ld",
		    (long)atomic_get(&g_kp_m), (long)atomic_get(&g_ki_m),
		    (long)atomic_get(&g_kd_m));

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(oven_sub,
	SHELL_CMD(status, NULL, "Show controller status", cmd_status),
	SHELL_CMD(arm,    NULL, "Operator arm request",   cmd_arm),
	SHELL_CMD(disarm, NULL, "Operator disarm request", cmd_disarm),
	SHELL_CMD_ARG(fault, NULL, "Inject fault bits: oven fault <hexbits>",
		      cmd_fault, 2, 0),
	SHELL_CMD_ARG(sp, NULL, "Set setpoint: oven sp <centi-degC>", cmd_sp, 2, 0),
	SHELL_CMD_ARG(gains, NULL, "Set gains: oven gains <kp> <ki> <kd> (x1000)",
		      cmd_gains, 4, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(oven, &oven_sub, "labsc_filament_oven test harness", NULL);

#endif /* CONFIG_SHELL */
