# Safety layer status

Tracks each protection layer from `docs/ARCHITECTURE.md` against what the
firmware/hardware actually does today. **This is the skeleton milestone: the
final electrical layers are not yet wired, so the system is NOT yet safe to run
a real heater unattended.**

## Protection layers (outermost = last resort)

| # | Layer | Status | Notes |
|---|---|---|---|
| 1 | Software diagnostics (liveness, sensor validation, process response) | **partial** | liveness + sensor validation + Sensor-B absolute limit done; process-response diagnostics TODO |
| 2 | Output-thread autonomous cutoff (≤ 8.33 ms on fault, ≤ 1 s on PID staleness) | **implemented** | `output_thread.c`; fault word checked every cycle, PID freshness over ≤ 120 cycles |
| 3 | Hardware watchdog (external) | **firmware side done** | `hw-wdt-out` health pulse; the external watchdog module itself is off-board hardware |
| 4 | Missing-pulse detector #1 — SSR enable | **firmware side done** | `keepalive-out` from the Output thread; the 555 + detector are off-board |
| 5 | Missing-pulse detector #2 — contactor enable | **firmware side partial** | shares `hw-wdt-out` health gate; contactor arm/disarm **sequencing + aux proof-test are TODO** |
| 6 | Independent thermal cutoff (in the heater power line) | **hardware-only, external** | no firmware involvement by design; MUST be fitted before unattended use |

## Firmware safety mechanisms

| Mechanism | Status | Where |
|---|---|---|
| OR-only fault word, never cleared at runtime | done | `shared.c` `fault_raise()` |
| Sequence-counter liveness (wraparound-safe) | done | `shared.h` `seq_advanced()` |
| Boot always DISARMED; heater cannot self-arm | done | `main.c`, `oven_state.c` |
| Boot into FAULTED after a latched trip | done | `fault.c` (noinit + CRC), restored in `main.c` |
| Single missed PID cycle = fault | done | `pid_thread.c` deadline check |
| Sensor-B absolute limit (independent of setpoint) | done | `pid_thread.c` |
| Software-only pulse generation (no HW PWM) | done | `output_thread.c`, `watchdog_thread.c` |
| Pinned thread priorities, self-asserted | done | `oven_threads.h`, each thread |
| PID control law (fixed-point; D-on-measurement; clamping anti-windup; bumpless) | **done** | `pid.c`, host-tested (`tests/pid/run.sh`); default gains 0 = inert |
| Process-response diagnostics | **DONE** | frozen reading (5 min), zone inversion while filament rises, powered-but-flat (60 s). See src/diagnostics.c; thresholds still [VERIFY] |
| Contactor load-free sequencing + aux proof-test | **TODO** | needs aux-contact input mapping |
| On-chip watchdog reset → FAULTED | **compiled out** | add a `watchdog0` alias to enable |
| NVS mirror of fault record (survives power loss) | **TODO** | `storage_partition` reserved |

## Hardware items to confirm before trusting the board

- Contactor topology (series-in-power-path vs relay-drives-coil) — architecture
  open item; decides whether a single weld defeats layer 5.
- Physical **arm button** input pin (candidate `opto-in2`).
- `opto-in1` = contactor aux / dry contact for welded-contactor detection.
- MAX6675 detects **open TC only**. The protection sensor (tc1) ideally becomes
  a MAX31855/31856 for short-to-GND/VCC detection — not in this Zephyr tree
  (MAX31855 driver is; MAX31856 would be out-of-tree).
- Per-pin reset/boot behaviour of the two pulse GPIOs (must be benign at reset
  and not strapping — the overlay keeps them off strap pins).
