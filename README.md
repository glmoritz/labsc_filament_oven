# labsc_filament_oven — safety-oriented oven controller (Zephyr + ESP32-C6)

Firmware for a 3D-print **filament dryer**, built as if it were a
**safety-critical industrial oven** (Industry-4.0 style, Zephyr RTOS, OPC UA on
the roadmap). It runs on the LabSC dual-socket carrier (ESP32-C6 primary, WeAct
BlackPill F411 secondary), the same board documented in
`labsc_zephyr_lorawan/PINMAP.md`.

The authoritative design is **[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)** —
six protection layers, a 500 ms PID thread, a 120 Hz Output thread, a
lowest-priority Watchdog thread, atomics-only inter-thread communication, an
OR-only fault word, sequence-counter liveness, and boot-into-FAULTED latching.

> **This is the control/safety skeleton milestone.** The thread architecture,
> shared state, liveness, fault latching, real MAX6675 reads and the two
> software-generated safety pulses are implemented. The PID control law, process
> diagnostics, contactor sequencing and OPC UA are **not** yet — see
> [Implemented vs stubbed](#implemented-vs-stubbed) and
> [`SAFETY-STATUS.md`](SAFETY-STATUS.md).

## Engineering constraints

- **No dynamic memory allocation** anywhere. All storage is static or on-stack.
- **State machines** for system arming, the Output safety latch, and the
  Watchdog health gate.
- **MISRA-C leaning** C: fixed types, braces everywhere, single-exit helpers,
  no recursion, bounded loops.
- All thread priorities pinned explicitly (`src/oven_threads.h`); each thread
  `__ASSERT`s its own priority at startup.

## Targets

| Board | Role | Build |
|---|---|---|
| `esp32c6_devkitc/esp32c6/hpcore` | primary (networking / OPC UA later) | default (`.board`) |
| `blackpill_f411ce` | secondary, **control-only** (no Wi-Fi) | `west build -b blackpill_f411ce` |

## Build

```bash
# inside the dev container, from this directory
west build -b esp32c6_devkitc/esp32c6/hpcore .
west build -b blackpill_f411ce .

# bench build with the test shell / fault-injection harness (NEVER ship this):
west build -b esp32c6_devkitc/esp32c6/hpcore . -- -DEXTRA_CONF_FILE=debug.conf
```

No submodules; nothing to fetch. The `hal_espressif` C6-SPI link workaround in
`CMakeLists.txt` is required by this container's Zephyr revision (the two MAX6675
readers ride SPI).

## Thread model

| Thread | Priority | Period | Job |
|---|---|---|---|
| PID | `K_PRIO_COOP(1)` (highest RT) | 500 ms (kernel timer) | read both TCs, validate, protection-limit check, compute control (stub), publish + liveness |
| Output | `K_PRIO_COOP(2)` | 8.333 ms (120 Hz) | fault-word + PID-freshness gate; software SSR-enable keep-alive pulse; liveness |
| Watchdog | lowest preemptible | 100 ms | health gate; external HW-watchdog / contactor pulse; on-chip WDT feed (optional) |
| main | background | 1 s | boot sequencing, device checks, status heartbeat — no safety role |

The two RT threads are **cooperative** so background work (log/shell/net) cannot
preempt them; they yield only by blocking. The Watchdog is the lowest priority
so it starves first — that starvation is the hardware-watchdog detector.

## Pulse train → hardware mapping

The carrier has two software-driven pulse outputs; the architecture describes
three conceptual sinks, mapped as:

| Alias | Net | Driven by | Role |
|---|---|---|---|
| `keepalive-out` | `LM555_ON_PULSE` (555) | **Output thread**, gated on ARMED + PID fresh + fault==0 | SSR-enable missing-pulse detector (layer 4) |
| `hw-wdt-out` | `OUT1_3V3` | **Watchdog thread**, gated on liveness + fault==0 | external HW watchdog (layer 3) **and** contactor-enable pulse (layer 5) |

Both edges are produced by an **in-thread GPIO write every cycle** — never
hardware PWM/LEDC/RMT (architecture "HARD DESIGN RULE"). A hung CPU therefore
stops both trains.

> **Confirm on the physical board:** contactor topology (an open item in the
> architecture), which input is the physical **arm button** (candidate:
> `opto-in2`, currently RFU), and that `opto-in1` is the contactor aux /
> dry-contact for welded-contactor detection.

## Bench operation

The heater can never self-arm: cold boot lands in **DISARMED** and the Output
thread emits no SSR keep-alive until an explicit arm request. On a devkit with
no arm button wired, use the test shell (debug build):

```
oven status                 # state, fault word, liveness, temperatures
oven arm                    # DISARMED -> ARMED (refused if any fault latched)
oven disarm
oven fault 0x20             # inject OVEN_FAULT_SENSOR_B_LIMIT -> latches FAULTED
```

The always-on, demonstrable software pulse is **`hw-wdt-out`**: it runs from boot
while healthy and stops the instant a fault latches (put a scope / logic analyzer
on `OUT1_3V3`).

## PID control law

`src/pid.c` implements the discrete PID from `PID.pdf` (parallel form,
trapezoidal integrator, filtered derivative) in **fixed-point integer** math
(no FPU dependence), with three review-agreed changes:

- **Derivative on measurement**, not error — no setpoint kick.
- **Pole-matched derivative filter** (`a = e^{-N·Ts}`), unconditionally stable
  for any `N`, replacing the PDF's forward-Euler pole which needs `N·Ts < 2`.
  Default **`Kd = 0`** (runs as PI); the plant is slow and the MAX6675 quantizes
  at 0.25 °C, so derivative earns little — enable it only if the bench shows a
  benefit.
- **Clamping anti-windup** at both rails (the heater is one-directional, so it
  saturates at 0 too), plus bumpless retune and a clean restart on arming.

Gains are `x1000` fixed-point in the `g_kp_m`/`g_ki_m`/`g_kd_m` atomics; setpoint
and temperatures are centi-°C; output is `0..120` half-cycles. Default gains are
**0**, so the controller is inert and safe until tuned (via `oven gains` /
`oven sp` on the bench, or OPC UA later). The law is host-unit-tested:

```bash
./tests/pid/run.sh      # compiles src/pid.c on the host and runs 15 checks
```

## Implemented vs stubbed

| Area | State |
|---|---|
| Three-thread architecture, pinned priorities | **implemented** |
| Atomics IPC, OR-only fault word, sequence-counter liveness | **implemented** |
| Real MAX6675 reads (A control / B protection), range + open-TC validation | **implemented** |
| Sensor-B absolute safety limit | **implemented** |
| Software SSR keep-alive + health/contactor pulses (no HW PWM) | **implemented** |
| System state machine, boot-DISARMED, boot-into-FAULTED latching (noinit + CRC) | **implemented** |
| PID deadline + PID/Output/timer staleness detection | **implemented** |
| Test shell / fault-injection seed | **implemented (debug build)** |
| PID control law — fixed-point, D-on-measurement, pole-matched filter, clamping anti-windup (both rails), bumpless retune | **implemented** (`src/pid.c`; default gains 0 = inert) |
| Process diagnostics (ON-but-flat, OFF-but-rising, rate) | **TODO** |
| Contactor arm/disarm load-free sequencing + aux proof-test | **TODO** |
| On-chip watchdog reset path (needs a `watchdog0` alias) | **compiled out** |
| OPC UA (S2OPC) / networking | **TODO** (footprint-probe milestone) |
| NVS mirror of the latched fault (survives power loss) | **TODO** (partition reserved) |

## Layout

```
CMakeLists.txt          app + the hal_espressif C6-SPI workaround
prj.conf / debug.conf   base config / bench-shell overlay
.board                  esp32c6_devkitc/esp32c6/hpcore
boards/                 per-board overlays (.overlay) + Kconfig (.conf)
docs/ARCHITECTURE.md    authoritative design & safety analysis
SAFETY-STATUS.md        per-layer implementation tracker
src/
  shared.[ch]           atomics, fault word, fixed-point, seq helper, fault_raise
  oven_state.[ch]       system state machine (INIT/DISARMED/ARMED/FAULTED)
  fault.[ch]            latched-fault record (noinit RAM, CRC)
  oven_threads.h        thread priorities + stacks (single source of truth)
  pid_thread.c          500 ms control loop
  output_thread.c       120 Hz safety actuator
  watchdog_thread.c     health gate + pulse + on-chip WDT
  shell_cmds.c          test harness (CONFIG_SHELL only)
  main.c                boot sequencing + heartbeat
```
