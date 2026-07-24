#!/usr/bin/env bash
#
# Host unit test for the fixed-point PID control law (src/pid.c).
#
# pid.c is deliberately RTOS-independent, so it compiles and runs on the host
# with tiny stubs for the two Zephyr headers it pulls in via shared.h. This
# exercises the scaling, trapezoidal integral, integrator-clamping anti-windup
# (both rails), derivative-on-measurement (no setpoint kick), the pole-matched
# derivative filter, and bumpless retune — no hardware required.
#
#   ./run.sh
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$here/../../src"
cc="${CC:-gcc}"
"$cc" -std=c11 -Wall -Wextra -Werror -O2 \
    -I"$here/host_stubs" -I"$src" \
    "$here/test_pid.c" "$src/pid.c" -o "$here/pid_test"
"$here/pid_test"
