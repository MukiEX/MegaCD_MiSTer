/* Reading LaserActive .mmi images (and .dscmmi files): ZIP/ZIP64 central
 * directory, stored QON video streams, QOI2 frame decoding.
 * SPDX-License-Identifier: MIT */
#ifndef MMI_H
#define MMI_H

#include <stddef.h>
#include <stdint.h>

#include "qon.h"
#include "qoi2.h"

typedef struct {
	char name[512];
	uint16_t method;
	uint16_t flags;
	uint16_t mtime, mdate;
	uint32_t crc;
	uint64_t comp_size;
	uint64_t uncomp_size;
	uint64_t local_off;
	uint64_t data_off;
} zip_entry;

typedef struct {
	int fd;
	uint64_t size;
	zip_entry *entries;
	int count;
} zip_file;

int zip_open(zip_file *z, const char *path, char *err, size_t errlen);
void zip_close(zip_file *z);
const zip_entry *zip_find(const zip_file *z, const char *name);

/* A stored QON stream inside a zip (or a bare .qon file). */
typedef struct {
	int fd;
	uint64_t base;      /* absolute offset of the QON header */
	uint64_t end;       /* absolute end of the entry */
	qon_desc hdr;
	uint64_t *off;      /* absolute offset of each frame's size prefix, frame_count + 1 entries */
} qon_stream;

int qon_open(qon_stream *q, int fd, uint64_t base, uint64_t size, char *err, size_t errlen);
void qon_close(qon_stream *q);
/* Compressed span of frame i in bytes (size prefix included). */
uint64_t qon_frame_span(const qon_stream *q, uint32_t i);
/* Read and decode frame i into rgb (hdr.width * hdr.height * 3 bytes).
 * scratch/cap is a reusable read buffer. Returns 0 or -1. */
int qon_read_frame(const qon_stream *q, uint32_t i, uint8_t *rgb, uint8_t **scratch, size_t *cap);

/* Is this entry a stored QON video stream? */
int zip_entry_is_qon(const zip_file *z, const zip_entry *e);

#endif
