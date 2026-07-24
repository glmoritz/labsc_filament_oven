#!/usr/bin/env bash
#
# Host unit tests for the fixed-point PID control law (src/pid.c).
#
# pid.c is deliberately RTOS-independent, so it compiles and runs on the host
# with tiny stubs for the Zephyr headers it pulls in. Two builds:
#
#   test_pid.c        asserts ENABLED  -> exercises the control-law behaviour
#                     (scaling, trapezoidal integral, clamping anti-windup,
#                     derivative-on-measurement, pole-matched filter, bumpless).
#   test_pid_clamp.c  asserts DISABLED (-DNDEBUG) -> exercises the defensive
#                     input-clamp net: garbage inputs must never produce an
#                     out-of-range output.
#
#   ./run.sh
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$here/../../src"
cc="${CC:-gcc}"
cflags=(-std=c11 -Wall -Wextra -Werror -O2 -I"$here/host_stubs" -I"$src")

echo "== control-law tests (asserts on) =="
"$cc" "${cflags[@]}" "$here/test_pid.c" "$src/pid.c" -o "$here/pid_test"
"$here/pid_test"

echo
echo "== defensive-clamp tests (asserts off, -DNDEBUG) =="
"$cc" "${cflags[@]}" -DNDEBUG "$here/test_pid_clamp.c" "$src/pid.c" -o "$here/pid_clamp_test"
"$here/pid_clamp_test"
