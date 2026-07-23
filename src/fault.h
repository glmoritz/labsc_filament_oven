/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 *
 * Latched-fault record across reboot (architecture, "Latched fault across
 * reboot"). Stored in a CRC-guarded noinit RAM section that survives a warm
 * reset (watchdog / software reboot). A full power cycle clears it, which is
 * the desired default: cold boot always starts DISARMED and un-faulted.
 *
 * NVS mirroring (survives power loss) is future work; the storage_partition is
 * already reserved in the board overlays for it.
 */

#ifndef OVEN_FAULT_H_
#define OVEN_FAULT_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Load a previously latched fault cause. Returns true and writes *cause_out
 * when a valid record with a non-zero cause survived the reset; false
 * otherwise. Safe to call once, early in main().
 */
bool fault_record_load(uint32_t *cause_out);

/* Persist the current fault cause. Called from fault_raise() on first trip. */
void fault_record_store(uint32_t cause);

/*
 * Clear the record (operator acknowledgement path — mirrors the electrical
 * manual re-arm). Not wired to an input in this bootstrap.
 */
void fault_record_clear(void);

#endif /* OVEN_FAULT_H_ */
