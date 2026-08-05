# Oven Controller — Software & Safety Architecture

## Status

**Design draft, revision 2** — safety-oriented oven controller on Zephyr RTOS + ESP32-C6.

This document captures the current architecture and design decisions. It is **not** intended to represent a certified or standardized architecture. Items marked **[VERIFY]** are working assumptions that must be checked against datasheets or current source code before implementation.

---

# Objectives

The controller shall:

- Execute a deterministic PID control loop every 500 ms.
- Generate a 120 Hz keep-alive pulse required by the heater power stage.
- Enter a safe state if the control loop fails, autonomously and quickly.
- Never self-arm the heater after any reset, fault, or power cycle.
- Execute non-critical software only when processor time is available.
- Avoid unbounded blocking operations within the real-time control path.

**Design goal in one sentence:** no single software failure, sensor failure, or power-stage failure may leave the heater energized; the final layers of protection must not depend on programmed silicon.

---

# Protection Layers (outermost = last resort)

1. **Software diagnostics** — liveness counters, sensor validation, process-response checks. Fastest, smartest, least trustworthy.
2. **Output-thread autonomous cutoff** — Output thread stops pulsing within ≤ 2 control periods (≤ 1 s) of PID staleness, and within ≤ 8.33 ms of any latched fault (fault word checked every Output cycle).
3. **Hardware watchdog** — expires if the watchdog thread (lowest priority) is starved or refuses to pet.
4. **Missing-pulse detector #1 (SSR enable)** — SSR gate power removed if the 120 Hz software-generated pulse stops.
5. **Missing-pulse detector #2 (contactor enable)** — interlocked relay + contactor drop out on missing pulse; re-arm **only** via physical button.
6. **Independent thermal cutoff** — mechanical capillary safety thermostat and/or one-shot thermal fuse with its switching element **in series with the heater power line**, breaking load current directly. It must NOT act via the contactor coil: a coil-series cutoff depends on the contactor opening and is defeated by welded contacts — exactly the failure this layer exists to survive. No firmware involved. Non-negotiable for unattended operation.

## Contactor feedback failure modes

The feedback is a phototransistor grounding an external pull-up, read ACTIVE LOW.
**It can fail in both directions**, and there is no safe assumption to make about
its resting level:

- transistor shorted, or the line shorted to ground → reads always **CLOSED**
- transistor open, LED burnt out, or broken wire → reads always **OPEN**

An earlier version of this document and of the board overlay claimed that a
failed or unpowered optocoupler reads as ASSERTED, i.e. that the sense fails
toward alarm. **That claim was wrong** and has been removed. Nothing may be built
on a fail direction for this part.

What makes the feedback trustworthy is not its level but the requirement that it
**CHANGE, on command, in a specific order**, before the oven may arm. The
interlock sequence demands both logical states from the sensor at every single
boot, so a sensor stuck at either one fails one of the phases:

| Failure | Reads | Caught by | Result |
|---|---|---|---|
| Transistor shorted / line to GND | always CLOSED | Phase 1, quiet window: contactor must read open while the health pulse is silent | weld fault, latched FAULTED, never armable |
| Transistor open / LED dead / broken wire | always OPEN | Phase 2: the open→closed transition is never observed | stays in AWAIT_CONTACTOR, never armable |
| Dies stuck CLOSED mid-run | frozen closed | Drop-out deadline: at the next loss of health the contactor "fails to open" within 1 s | weld fault. May misdiagnose a healthy contactor — fails safe, in the right direction |
| Dies stuck OPEN mid-run | frozen open | Read as the contactor having dropped | returns to AWAIT_CONTACTOR, disarmed; re-arming needs a fresh observed closure |

The property that matters: **no failure of this sensor can cause the oven to
heat when it should not.** Every one of them either latches a fault or prevents
arming. The worst outcome is a false weld report, which stops a healthy machine —
the acceptable direction to be wrong in.

Residual, and accepted: between boot and the next loss of health, the sensor is
never exercised, so a mid-run death is only caught when the state is next
required to change. None of the intermediate windows is hazardous — a stuck
reading cannot energize anything by itself, and the contactor is only one of six
protection layers.

## Thermal ladder — as built

Three rungs, each of which must trip well before the next, so the cheap
recoverable one acts first and the destructive one is never reached in normal
fault handling:

| Trip | Device | Acts on | Recoverable? |
|---|---|---|---|
| **110 °C** | firmware, sensor B in the heat chamber (`OVEN_SENSOR_B_ABS_LIMIT_CC`) | latches FAULTED, stops the keep-alive | yes |
| **150 °C** | bimetallic thermostat | drops the contactor **coil** | yes, self-resetting or manual per part |
| **300 °C** | thermal fuse | **in series with the heater element** | no — one-shot, must be replaced |

Which rung is the real backstop matters. The 150 °C bimetallic acts through the
contactor coil, so a **welded contactor defeats it** — it is an extra rung, not
the last line, and layer 6 above is explicit about why. Only the 300 °C fuse,
switching load current directly, survives a weld. Read "we have a bimetallic" as
defence in depth, never as compliance with layer 6.

The 40 °C gap between the firmware limit and the bimetallic is deliberate: it
keeps a firmware trip from racing the mechanical one, so ordinary faults are
handled in software and leave a diagnosable latched fault word instead of a
silently opened contactor. **[VERIFY]** the lower bound on 110 °C — it must stay
above the heat chamber's worst-case steady-state temperature with the filament
chamber at its maximum setpoint, a figure that has not been measured yet.

The SSR's dominant failure mode is **failed short** (it switches constantly under burst fire, dissipates real power, and dies closed). The contactor + missing-pulse interlock is the mitigation for that specific failure — it is doing real work in this design, not decoration. The thermal cutoff covers simultaneous failure of everything electronic, including design errors in this document.

---

# Execution Architecture

```
                500 ms Hardware Timer
                         │
                         ▼
               +---------------------+
               | PID Thread          |
               | Highest RT priority |
               +---------------------+
                         │
             publishes atomics + seq counter
                         │
                         ▼
               +---------------------+
               | Output Thread       |
               | 120 Hz              |
               | gates pulses on     |
               | PID freshness +     |
               | fault word          |
               +---------------------+

--------------------------------------------
 Background (only in spare CPU time):
   S2OPC (OPC UA)
   Logging
   Shell
   Diagnostics / Statistics
--------------------------------------------

               +---------------------+
               | Watchdog Thread     |
               | Lowest priority     |
               | pets HW WDT only if |
               | all health OK and   |
               | fault word == 0     |
               +---------------------+
```

## Thread priority audit

All system thread priorities are pinned explicitly — no trusting defaults. Maintain a table in `prj.conf` comments covering at minimum:

| Thread | Priority | Notes |
|---|---|---|
| PID thread | highest cooperative/RT | above Wi-Fi driver threads |
| Output thread | just below PID | |
| Wi-Fi driver / net stack workqueues | below Output | `CONFIG_*_PRIORITY` audit required |
| sysworkq, shell, logging | background | |
| S2OPC threads | background | |
| Watchdog thread | **lowest in the system** | starvation detector |

**Caveat:** thread priority does not shield against Wi-Fi **interrupt** load — radio ISRs preempt everything on the C6's single HP core. At 200 ms this is noise; for the 120 Hz thread, the missing-pulse detector window must tolerate ± ~2 ms jitter so radio bursts cannot cause nuisance disarms. Measure Output-thread jitter under Wi-Fi traffic load before fixing the detector window. Nuisance trips are the enemy: they train the operator to disable protections.

A Wi-Fi driver failure (disconnect storm, supplicant timeout from starvation) must not consume CPU at a priority that delays the Output thread. Network is non-critical by design; verify reconnect logic runs at background priority.

---

# PID Thread

Released every 500 ms by a hardware timer.

Execution sequence:

1. Read sensor A (control zone)
2. Read sensor B (protection zone)
3. Validate sensor status (fault bits, communication result)
4. Validate safety limits (sensor B absolute limits — independent of setpoint path)
5. Execute process diagnostics
6. Compute PID (control zone, sensor A)
7. Publish actuator command
8. Publish telemetry
9. Increment PID sequence counter (liveness)

## Blocking policy (revised)

The original rule "no semaphores" is unimplementable with stock Zephyr SPI drivers, which block on an internal completion during transfers. Revised rule:

> The PID thread shall not perform **unbounded** blocking. Driver-internal synchronization with bounded WCET **and a timeout** is permitted. The thread shall not access the filesystem, perform network communication, or allocate dynamic memory.

Only the two MAX66xx devices share the SPI bus, so contention is bounded and small. **[VERIFY]** the espressif SPI driver behavior on C6 (timeout support, worst-case transfer latency) against current Zephyr source.

## PID hygiene

- **Integrator anti-windup** (clamping or back-calculation). The heater saturates for long periods during heat-up; without anti-windup, overshoot will nuisance-trip the overtemp diagnostics.
- **Derivative on measurement**, not on error (no setpoint kick), with a low-pass filter on the derivative term.
- **Bumpless parameter updates**: when OPC UA changes gains mid-run, reinitialize the integrator state so output does not jump.

---

# Output Thread

Period: 8.333 ms (120 Hz).

Responsibilities, every cycle:

1. Read fault word. If nonzero → stop pulsing, force output off. (Gives every fault detector a ≤ 8.33 ms path to de-energization.)
2. Check PID sequence counter advanced within the last ≤ 2 control periods (≤ 1 s). If stale → stop pulsing, force output off, latch fault. **The Output thread is the fast safety actuator; the watchdog chain is only the backstop.**
3. Read latest actuator command (atomic).
4. Generate heater keep-alive pulse(s) — see hard rule below.
5. Apply requested output power (burst-fire enable decision).
6. Increment Output sequence counter (liveness).

The thread performs no control calculations. Communication with the PID thread is exclusively through atomics.

## HARD DESIGN RULE — keep-alive pulses must require ongoing software execution

The pulses **must not** be generated by free-running hardware (LEDC/PWM, RMT). A hung CPU keeps hardware PWM running forever, silently defeating both missing-pulse detectors. Each pulse edge must result from a software action performed in that cycle (GPIO toggle in-thread, or a one-shot timer re-armed by software every cycle).

This rule is recorded here explicitly because a future "the pulse has jitter, let me move it to hardware PWM" refactor would look innocent and would destroy the safety architecture.

## Diversified pulse trains

The two missing-pulse detectors must not share a single failure mode:

- **SSR-enable pulse** — generated by the Output thread (fast path, gated on PID freshness + fault word).
- **Contactor-enable pulse** — gated on **overall system health**: generated by the same decision logic that pets the hardware watchdog (or by the PID thread). Consequence: "Output thread alive but everything else dead" drops the contactor.

If both pulses came from the Output thread alone, the second detector would add almost nothing.

---

# Power Modulation

Heater is driven by a **zero-crossing SSR** with half-cycle burst fire on 60 Hz mains (120 half-cycles/s → the 120 Hz Output period).

- With a plain zero-cross SSR, no phase alignment is required in software: the SSR only switches at zero crossings, so the 8.33 ms thread merely updates the enable decision and the SSR quantizes it. Jitter tolerance is therefore genuinely relaxed.
- The Output thread period is coupled to mains frequency; "one missed cycle" means one missing/extra half-cycle of power — thermally irrelevant, but relevant to liveness accounting.
- Future option (not now): zero-cross detection input for proper burst-fire with evenly distributed half-cycles (less flicker/ripple).

---

# Contactor & Interlock Failure Analysis

## Welded contacts are a latent fault

A welded contactor does not announce itself. The SSR still modulates power, the system behaves normally, and layer 5 is silently gone — undiscovered until the day the SSR *also* fails short, at which point only the thermal cutoff remains. The design goal is therefore not only "survive a weld" (the power-line thermal cutoff does) but "never operate for long with a dead protection layer without knowing it."

## Prevention: never switch the contactor under load

Welds occur at break under load current or at make into inrush. Software controls both ends of the power chain, so **normal** arm/disarm is sequenced:

- **Disarm:** SSR output off → wait a few mains cycles → drop the contactor-enable pulse. The contactor breaks ~zero current.
- **Arm:** contactor closes first → only then is the SSR permitted to conduct.

The **fault** path is exempt: if the pulse train dies (CPU hang, watchdog trip), the contactor drops whatever the SSR is doing, possibly under full load — that is its job. The sequencing rule protects the contacts during the overwhelming majority of switching cycles (normal disarms), preserving them for the rare emergency break. Size the contactor with real margin over heater inrush.

## Detection: exercise-and-verify at every disarm

- **Mirror/auxiliary contact (preferred):** use a contactor with a mechanically linked NC auxiliary contact — guaranteed open whenever any main contact is closed (mirror-contact construction; **[VERIFY]** the exact IEC 60947-4-1 clause if a citation is needed). Wire it to a GPIO. At every disarm, command drop → confirm aux indicates the mains contacts opened; mismatch → latch fault, refuse the next arm. At startup, aux must indicate open before arm is permitted. This converts the latent fault into one detected at the very next operating cycle (proof-test-per-cycle logic).
- **Downstream AC-presence sensing (alternative/additional):** an opto/detection circuit downstream of the contactor. Contactor commanded off + voltage present = welded contacts. Note: **load current sensing does not catch this case** — a welded contactor with the SSR off draws no current. Voltage presence or the aux contact are the correct observables.

## Topology — OPEN ITEM

The design uses "an interlocked relay **and** contactor." Confirm the wiring topology:

- **Contacts in series in the heater power path:** a single weld does not defeat layer 5 (both devices must weld). Monitor both with aux contacts if available — this also identifies *which* device welded.
- **Relay drives contactor coil (series in the control path only):** no weld redundancy in the power path; the small relay welding is, if anything, more likely than the contactor welding.

Document the actual topology and update this analysis accordingly.



Two thermocouples via MAX66xx SPI converters, **dual zone**, shared SPI bus, separate CS lines.

## Physical zones (decided)

The oven is **two chambers with forced convection between them**: the heater sits
in the **heat chamber**; a fan blows that air into the **filament chamber**, which
is what the process actually cares about.

- **Heat chamber** — heater, hottest point in the system, hosts the failure that
  starts a fire.
- **Filament chamber** — downstream of the fan, always cooler than the heat
  chamber, holds the filament. This is the controlled variable.

## Role separation (decided)

- **Sensor A** = **filament chamber**. Control input to the PID. Nothing else.
- **Sensor B** = **heat chamber**. Protection only: absolute safety-limit
  comparison + process diagnostics input. Never used by the PID.

Rationale: the classic runaway is the control sensor detaching or reading low →
PID sees "cold" → commands 100%. Sensor B's absolute limit catches this even
though the two zones never numerically "agree."

**Why the protection sensor goes in the HEAT chamber**, and the control sensor in
the cooler one — the assignment is not arbitrary and reversing it is a safety
regression:

- The absolute limit must sit at the **hottest point**. Guarding the filament
  chamber leaves the chamber that contains the heater with no absolute limit at
  all.
- It makes **fan failure** a covered failure rather than the worst one. Fan
  stops → heat chamber climbs, filament chamber cools → the PID reads "cold" and
  commands 100% → B trips on the heat chamber. With the sensors reversed this
  same sequence is an undetected runaway in which the control sensor keeps
  reporting that everything is fine.
- Controlling on the filament chamber means the loop regulates the variable the
  process cares about, instead of regulating the heat chamber and leaving the
  filament in open loop.

**Rejected: cascade control** (filament-chamber outer loop producing the heat
chamber's setpoint). It performs better on paper and it is how you would build
this with three sensors, but with two thermocouples it makes **both** sensors
control inputs, which deletes the independent protection channel this section
exists to define. It also inverts the fan-failure response: the outer loop, seeing
the filament chamber cold, raises the heat-chamber setpoint and drives *into* the
fault. Revisit only alongside a third, protection-dedicated thermocouple in the
heat chamber.

**Wiring is load-bearing.** The A↔B swap is already listed in the failure table as
"mechanical prevention only" — software cannot tell the two apart at a cold start.
With this assignment a swap puts the absolute limit on the cool chamber and the
control loop on the hot one, i.e. exactly the reversed arrangement rejected above.
Key and label the connectors.

## Part selection **[VERIFY against datasheets]**

- **MAX6675**: conversion ≈ 0.17–0.22 s; a read aborts the conversion in progress. At the 500 ms PID period this is comfortably outside the conversion window, so timing is no longer a constraint — but the part detects **only** open thermocouple (no short-to-GND/VCC detection), which remains the argument against it for the protection sensor.
- **MAX31855**: ~100 ms-class conversion; detects open, short-to-GND, short-to-VCC.
- **MAX31856**: continuous conversion, configurable 50/60 Hz mains rejection filter, open/short fault detection. Preferred if available.

Short-to-GND/VCC detection matters for the protection path: a thermocouple lead pinched against grounded chassis produces a **plausible but wrong** reading. Because the zones legitimately differ, the disagreement threshold must be loose, and this failure may pass plausibility checks — the chip-level short detection is the mitigation.

## Shared-bus common-cause — accepted risk (documented)

Both converters share MISO/SCK and one driver instance. A stuck MISO line, solder bridge, or driver bug blinds both sensors identically. **Accepted for this project**, mitigated by:

- Process-response diagnostics (heater ON but temperature flat), which do not depend on sensor agreement.
- The independent thermal cutoff.

Hardware notes:

- Separate CS lines with independent pull-ups, so a floating CS during reset cannot leave a converter driving MISO.
- **[VERIFY]** the chosen C6 pins' behavior during ROM bootloader execution.
- Ideally the two converters are different part numbers or at least different interfaces, so a single driver bug cannot blind both plausibility inputs (Future Work if the same part is already purchased).

## Thermocouple failure modes covered

| Failure | Symptom | Detected by |
|---|---|---|
| Open circuit | Fault bit | Chip open-TC detection |
| Short to GND / VCC | Plausible wrong reading | Chip short detection (31855/31856) |
| Detached from workpiece | Reads low, PID drives 100% | Sensor B absolute limit; process-response (ON, no rise where expected) |
| Reversed polarity | Reading moves **down** when heating | Plausibility: temperature must rise when heater duty is high |
| Pinched/damaged lead | Erratic or offset reading | Sudden-jump check; disagreement (loose threshold) |
| Cold-junction error (converter chip heated by enclosure) | Slow offset drift | Mount converters away from oven heat; monitor internal CJ temperature register and alarm if > design limit **[VERIFY register availability per part]** |
| Aging / decalibration drift | Slow accuracy loss | Not detectable online; periodic manual verification against a reference thermometer (maintenance procedure) |
| Sensors swapped at connectors (A↔B) | Control loop driven by wrong zone | Keyed/labeled connectors; startup plausibility (both near ambient at cold start makes this undetectable in software — mechanical prevention only) |

---

# Shared Variables

Thread communication uses Zephyr atomics with the following conventions:

- **`atomic_t` is a 32-bit integer** — floats are not stored directly. All parameters use **fixed-point scaling** (e.g., temperature in centi-°C, gains × 1000). Fixed-point also makes OPC UA range validation and logging unambiguous.
- **Sequence counters, not booleans, for liveness.** Booleans have a classic failure mode (producer sets, monitor clears, producer hangs → ordering-dependent missed detection). Monotonic counters with "advanced since last check" semantics are strictly better. Wraparound: compare with unsigned subtraction, never equality-to-expected.
- **Central fault word**: one atomic bitmask of fault causes. Any thread ORs bits in (`atomic_or`); bits are never cleared at runtime — only by the reboot-into-faulted-state path after operator action.

```c
atomic_t setpoint_cC;        /* centi-degC */
atomic_t kp_m, ki_m, kd_m;   /* gains x1000 */
atomic_t heater_power;       /* 0..120 half-cycles per second */
atomic_t pid_seq;            /* monotonic */
atomic_t output_seq;         /* monotonic */
atomic_t fault_word;         /* bitmask, OR-only */
```

Torn parameter **sets** (new kp with old ki for one 500 ms cycle) are accepted **because** the plant is slow and all parameters are individually range-clamped before publication — not merely "considered acceptable."

---

# OPC UA (S2OPC)

Stack: **S2OPC** (Systerel) — chosen over open62541 for footprint and safety/security pedigree. **[VERIFY]** current maintenance status of the S2OPC Zephyr port against the Zephyr release in use before committing.

Responsibilities: telemetry out, parameter/setpoint writes in, diagnostics out. Executes only as background work; never blocks real-time threads.

## Boundary rules (safety-relevant)

- All writes (setpoint, gains) are **range-checked and clamped in the address-space write callbacks** before publishing to atomics.
- **Safety temperature limits are not in the writable address space at all.** They are compile-time or physically-jumpered configuration, independent of the setpoint path. A compromised or buggy OPC UA client must not be able to move both the setpoint and the limit.

## Architecture option if RAM is tight

Wi-Fi + mbedTLS + OPC UA server on ~512 KB SRAM will be tight. Escape hatch: S2OPC **PubSub over UDP** for telemetry (much lighter, fits the "background, never blocks" model naturally) + a minimal separate mechanism for parameter writes. Prototype memory footprint **early**; do not assume it fits.

---

# Watchdog Thread

Runs at the **lowest priority** in the system.

Pets the hardware watchdog only if **all** of:

- PID sequence counter advanced since last check.
- Output sequence counter advanced since last check.
- Hardware timer is progressing (catches a stalled/misconfigured/gated timer).
- Fault word == 0.
- Additional health indicators OK.

If higher-priority threads monopolize the CPU, this thread starves and the hardware watchdog expires — starvation detection is the point of the lowest priority.

The contactor-enable pulse is gated on this same health decision (see Diversified pulse trains).

---

# Timing & Deadline Detection

- PID loop: 500 ms (comfortably clears all candidate sensor conversion windows — see sensor section).
- Output thread: 8.333 ms (120 Hz, coupled to 60 Hz mains).
- Background: spare CPU only; may be delayed indefinitely.

Fault-detection latency consequence of the 500 ms period: faults detected **by the PID thread** (sensor faults, safety limits, process diagnostics) take up to 500 ms to reach the fault word; the Output thread then reacts within 8.33 ms. Worst-case ~510 ms sensor-to-de-energization is thermally negligible for this plant and is accepted. Liveness faults (PID staleness) are bounded by the ≤ 1 s window above.

Deadline measurement mechanics: timestamp each PID cycle start (`k_uptime_ticks()` or cycle counter), compare against previous release, count overruns. A single missed control cycle is a fault. Log worst-case observed jitter and execution time continuously (cheap running max), not only in a lab test.

---

# Process Diagnostics

The controller verifies that the physical process behaves consistently with commands.

## Heater commanded ON

If heater duty remains high for a configurable time and temperature does not rise sufficiently → fault. Possible causes: heater open circuit, SSR failed open, wiring failure, detached control sensor.

## Heater commanded OFF

If output is disabled and temperature keeps rising beyond expected limits → fault, and (because software output is already off) rely on contactor drop + thermal cutoff. Possible causes: **SSR welded closed** (dominant), external heat source.

## Rate plausibility

- Temperature rise rate above physical maximum (with heater at 100% from cold) → sensor or wiring fault.
- Sudden step change larger than physically possible in one sample → sensor fault.

---

# Safety Conditions

Enter the safe state when any of:

- Control loop deadline exceeded (single miss).
- PID or Output sequence counter stale.
- Open / shorted thermocouple detected.
- Sensor communication failure (SPI timeout, invalid status bits).
- Sensor B exceeds absolute safety limit.
- Process diagnostics inconsistent.
- Brownout detected (see below).
- Hardware watchdog expires.

---

# Safe State & Fault Latching

**Normal disarm** (operator-commanded, no fault) follows the load-free sequencing in the Contactor & Interlock section: SSR off → delay → contactor pulse dropped → aux-contact verification.

**Fault-driven safe state** skips sequencing — immediate de-energization takes priority over contact wear:

- Fault word bit set (OR-only).
- Output thread stops pulses and forces output off within ≤ 8.33 ms of the fault word becoming nonzero.
- Watchdog thread stops petting → hardware watchdog reset (if configured for reset).
- Missing-pulse detectors drop SSR enable and contactor.
- Contactor requires **physical button** re-arm.

## Latched fault across reboot (promoted from Future Work — now required)

After a safety trip, software must **not** resume control after the watchdog reboot. Sequence:

1. Before/at fault: store fault cause in retained/noinit RAM (magic number + cause + counter) and, when feasible, NVS.
2. After any reset: boot into **FAULTED** state — heater command off, no pulses, telemetry and shell available, fault cause displayed/exported.
3. Exit FAULTED only by explicit operator action (physical input or authenticated command), mirroring the electrical manual re-arm.
4. **Heater cannot self-arm after any reset**, faulted or not — cold boot always starts disarmed; the button is always required.

NVS/settings integrity: CRC-protect stored parameters; on corruption, load safe defaults and set a (non-tripping) warning flag.

---

# Boot, Reset, Flashing

- Keep-alive pins: chosen to be high-Z/benign at reset, **not strapping pins**; ROM bootloader and MCUboot must not be able to produce anything resembling a pulse train on them. **[VERIFY per-pin on C6.]**
- Flashing/JTAG: heater must not be armable during programming. The manual arm button largely enforces this; document it as an explicit requirement anyway.
- **Brownout**: enable the ESP32-C6 brownout detector; treat brownout reset like any other reset (boot disarmed). Mains dips that chatter the contactor are handled electrically by the interlock (drops out, requires button).

---

# Electrical / Physical Notes (interfaces to software)

- **Thermal cutoff** (layer 6): switching element **in the heater power line**, breaking full load current — never via the contactor coil (defeated by welded contacts). Capillary safety thermostats (16 A / 250 VAC class, standard oven parts) separate the sensing bulb (hot zone) from the contact block (kept cool outside); one-shot thermal fuses (10–15 A class) mount thermally coupled to the heater body — **crimp, do not solder** leads near the body, as install-time solder heat can degrade the fusible alloy. Select trip point above max process temperature but below damage temperature of enclosure/insulation, and verify the current rating against the actual heater load. A coil-series cutoff may be added as a cheap *extra* trip, but never as the final layer. If the supply is phase–phase, a single-pole cutoff still stops current (the fire-safety function), but the heater remains referenced to a live phase — a double-pole device or the upstream breaker covers de-energization for service.
- SSR heatsinking sized for worst-case duty; an overheated SSR fails **short** — undersized heatsink converts a thermal problem into the exact failure the contactor exists for. Consider monitoring SSR heatsink temperature with a cheap sensor (warning only, Future Work).
- Pulse lines to the missing-pulse detectors: keep short, consider RC filtering/opto isolation so mains EMI cannot fake pulses into the detectors. A detector that can be triggered by coupled noise from the very load it controls is a real design bug class.
- Grounding: single-point ground for thermocouple shields; avoid ground loops through the chassis that turn a pinched lead into an undetected offset.
- Supply: properly fused, and the controller's own PSU failure must de-energize (it does: no PSU → no pulses → detectors drop everything).
- Arm button circuit: a **stuck-closed button must not hold the contactor armed** — the interlock must be edge/latch based (button initiates latch; missing pulse breaks it regardless of button state). Verify the relay interlock circuit against this case.

---

# Assumptions (revised)

- Missing a single control cycle is a fault.
- One-cycle-old parameters are acceptable (slow plant, clamped ranges).
- Background software may be delayed indefinitely.
- Real-time communication uses only Zephyr atomics (fixed-point, sequence counters, OR-only fault word).
- The control path contains no **unbounded** blocking; bounded driver-internal waits with timeouts are permitted.
- Control and protection sensors share an SPI bus — accepted common-cause risk, mitigated by process diagnostics and the thermal cutoff.
- Safety decisions combine software liveness and physical process diagnostics.
- The independent thermal cutoff exists and is wired **in the heater power line**; software safety analysis assumes it as the final layer.
- Normal disarms are sequenced load-free; the contactor breaks load current only in fault conditions.

---

# Test & Validation Plan (new)

Every safety path must be **seen firing** before the system is trusted unattended:

- **Fault-injection harness** (test build only): shell commands to kill/suspend each thread, freeze the hardware timer, force sensor fault bits, inject stuck readings, stop each pulse train individually.
- Verify Output-thread autonomous cutoff timing (scope on SSR-enable line: fault injected → pulses stop ≤ 8.33 ms).
- Verify both missing-pulse detectors independently (stop each pulse, confirm the correct stage drops).
- Verify contactor manual re-arm and stuck-button case.
- Welded-contactor detection: jumper the aux-contact input (or physically block the armature in a bench test) → confirm fault latches at disarm and arm is refused at next startup.
- Verify arm/disarm sequencing on a scope: SSR conduction never overlaps contactor make/break during normal operation.
- Verify FAULTED boot after watchdog reset, with correct cause reported.
- Thermal tests: detach control thermocouple with heater running → confirm sensor B limit trips; block/disconnect heater → confirm "ON but no rise" trips.
- Wi-Fi stress: measure Output-thread jitter and PID deadline misses under heavy radio traffic; confirm no nuisance disarms.
- Thermal cutoff functional test (controlled overtemp or manufacturer test procedure).

---

# Future Work

- Windowed hardware watchdog (detects too-early petting).
- Zero-cross detection input for evenly distributed burst-fire half-cycles.
- Event log in non-volatile memory with pre-fault ring buffer (flight-recorder style).
- Startup self-tests: sensor read + fault-bit exercise; optional contactor test (command off, verify no temperature rise before allowing arm).
- Power-stage diagnostics: load current sensing to confirm commanded heater operation directly (detects SSR welded/open within seconds instead of thermal time constants; does **not** detect a welded contactor with SSR off — that requires the aux contact or downstream voltage presence).
- Downstream AC-presence sensing as a second welded-contactor observable (complementary to the aux contact).
- SSR heatsink temperature monitoring (warning).
- Continuous WCET/jitter logging exported via OPC UA.
- Different sensor part numbers / interfaces for A and B to remove driver common-cause.
- Periodic thermocouple calibration check procedure (maintenance schedule).
