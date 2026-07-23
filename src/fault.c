/*
 * Copyright (c) 2026 Guilherme Luiz Moritz
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fault.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

#define FAULT_RECORD_MAGIC   (0x0F3EA17DU)   /* "safe fault" marker */

struct fault_record {
	uint32_t magic;
	uint32_t cause;       /* fault-word bits latched at trip time */
	uint32_t trip_count;  /* cumulative trips since last power-on  */
	uint32_t crc;         /* crc32 over magic|cause|trip_count     */
};

/* noinit: preserved across a warm reset, uninitialised at power-on. */
static struct fault_record s_rec __noinit;

static uint32_t record_crc(const struct fault_record *rec)
{
	/* CRC covers everything except the trailing crc field itself. */
	return crc32_ieee((const uint8_t *)rec,
			  sizeof(*rec) - sizeof(rec->crc));
}

static bool record_valid(const struct fault_record *rec)
{
	return ((rec->magic == FAULT_RECORD_MAGIC) &&
		(rec->crc == record_crc(rec)));
}

bool fault_record_load(uint32_t *cause_out)
{
	bool present = false;

	if ((cause_out != NULL) && record_valid(&s_rec) && (s_rec.cause != 0U)) {
		*cause_out = s_rec.cause;
		present = true;
	}

	return present;
}

void fault_record_store(uint32_t cause)
{
	uint32_t count = 0U;

	/* Preserve the running trip count if the existing record is intact. */
	if (record_valid(&s_rec)) {
		count = s_rec.trip_count;
	}

	s_rec.magic = FAULT_RECORD_MAGIC;
	s_rec.cause = cause;
	s_rec.trip_count = count + 1U;
	s_rec.crc = record_crc(&s_rec);
}

void fault_record_clear(void)
{
	s_rec.magic = 0U;
	s_rec.cause = 0U;
	s_rec.trip_count = 0U;
	s_rec.crc = 0U;
}
