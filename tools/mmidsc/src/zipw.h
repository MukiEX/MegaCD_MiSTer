/* Minimal ZIP64 writer for stored (and raw-copied) entries with aligned
 * data. SPDX-License-Identifier: MIT */
#ifndef ZIPW_H
#define ZIPW_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	char name[512];
	uint16_t method, flags, mtime, mdate;
	uint32_t crc;
	uint64_t comp, uncomp, local_off;
} zw_entry;

typedef struct {
	int fd;
	uint64_t pos;
	zw_entry *ents;
	int n, cap;
} zip_writer;

int zw_open(zip_writer *w, const char *path);
/* Start an entry whose sizes are known. Returns the absolute offset at which
 * the caller must write comp_size bytes of entry data (with write_full_at),
 * aligned to `align` bytes. */
uint64_t zw_begin(zip_writer *w, const char *name, uint16_t method, uint16_t flags, uint16_t mtime, uint16_t mdate,
                  uint64_t comp_size, uint64_t uncomp_size, uint32_t align);
/* Finish the entry started last; patches its CRC into the local header. */
int zw_end(zip_writer *w, uint32_t crc);
/* Write the central directory and close. */
int zw_finish(zip_writer *w);

#endif
