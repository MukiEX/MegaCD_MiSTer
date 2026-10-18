/* SPDX-License-Identifier: MIT */
#define QOI2_IMPLEMENTATION
#define QON_IMPLEMENTATION
#include "mmi.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "util.h"

#ifdef _WIN32
#include <io.h>
#define O_BINARY_FLAG O_BINARY
#else
#include <unistd.h>
#define O_BINARY_FLAG 0
#endif

static void set_err(char *err, size_t len, const char *msg)
{
	if (err && len) snprintf(err, len, "%s", msg);
}

int zip_open(zip_file *z, const char *path, char *err, size_t errlen)
{
	memset(z, 0, sizeof *z);
	z->fd = open(path, O_RDONLY | O_BINARY_FLAG);
	if (z->fd < 0) { set_err(err, errlen, "cannot open file"); return -1; }
#ifdef _WIN32
	z->size = (uint64_t)_lseeki64(z->fd, 0, SEEK_END);
#else
	z->size = (uint64_t)lseek(z->fd, 0, SEEK_END);
#endif
	size_t tail = z->size < 65557 ? (size_t)z->size : 65557;
	uint8_t *buf = malloc(tail);
	if (!buf || read_full_at(z->fd, buf, tail, z->size - tail)) { free(buf); set_err(err, errlen, "read error"); return -1; }
	long eocd = -1;
	for (long i = (long)tail - 22; i >= 0; i--)
		if (rd32le(buf + i) == 0x06054b50) { eocd = i; break; }
	if (eocd < 0) { free(buf); set_err(err, errlen, "not a ZIP file (no end of central directory)"); return -1; }
	uint64_t entries = rd16le(buf + eocd + 10);
	uint64_t cd_size = rd32le(buf + eocd + 12);
	uint64_t cd_off = rd32le(buf + eocd + 16);
	uint64_t eocd_abs = z->size - tail + (uint64_t)eocd;
	free(buf);
	if (eocd_abs >= 20) {
		uint8_t loc[20];
		if (!read_full_at(z->fd, loc, 20, eocd_abs - 20) && rd32le(loc) == 0x07064b50) {
			uint8_t rec[56];
			if (!read_full_at(z->fd, rec, 56, rd64le(loc + 8)) && rd32le(rec) == 0x06064b50) {
				entries = rd64le(rec + 32);
				cd_size = rd64le(rec + 40);
				cd_off = rd64le(rec + 48);
			}
		}
	}
	if (cd_off + cd_size > z->size || entries > 65536) { set_err(err, errlen, "bad central directory"); return -1; }
	uint8_t *cd = malloc(cd_size ? cd_size : 1);
	if (!cd || read_full_at(z->fd, cd, cd_size, cd_off)) { free(cd); set_err(err, errlen, "read error"); return -1; }
	z->entries = calloc(entries ? entries : 1, sizeof(zip_entry));
	size_t p = 0;
	for (uint64_t k = 0; k < entries; k++) {
		if (p + 46 > cd_size || rd32le(cd + p) != 0x02014b50) break;
		zip_entry *e = &z->entries[z->count];
		uint16_t nlen = rd16le(cd + p + 28), xlen = rd16le(cd + p + 30), clen = rd16le(cd + p + 32);
		if (p + 46 + nlen + xlen + clen > cd_size) break;
		e->flags = rd16le(cd + p + 8);
		e->method = rd16le(cd + p + 10);
		e->mtime = rd16le(cd + p + 12);
		e->mdate = rd16le(cd + p + 14);
		e->crc = rd32le(cd + p + 16);
		e->comp_size = rd32le(cd + p + 20);
		e->uncomp_size = rd32le(cd + p + 24);
		e->local_off = rd32le(cd + p + 42);
		size_t copy = nlen < sizeof e->name - 1 ? nlen : sizeof e->name - 1;
		memcpy(e->name, cd + p + 46, copy);
		e->name[copy] = 0;
		const uint8_t *x = cd + p + 46 + nlen, *xe = x + xlen;
		while (x + 4 <= xe) {
			uint16_t id = rd16le(x), sz = rd16le(x + 2);
			const uint8_t *d = x + 4;
			if (d + sz > xe) break;
			if (id == 0x0001) {
				const uint8_t *q = d;
				if (e->uncomp_size == 0xffffffffu && q + 8 <= d + sz) { e->uncomp_size = rd64le(q); q += 8; }
				if (e->comp_size == 0xffffffffu && q + 8 <= d + sz) { e->comp_size = rd64le(q); q += 8; }
				if (e->local_off == 0xffffffffu && q + 8 <= d + sz) { e->local_off = rd64le(q); q += 8; }
			}
			x = d + sz;
		}
		uint8_t lh[30];
		if (!read_full_at(z->fd, lh, 30, e->local_off) && rd32le(lh) == 0x04034b50) {
			e->data_off = e->local_off + 30 + rd16le(lh + 26) + rd16le(lh + 28);
			z->count++;
		}
		p += 46 + nlen + xlen + clen;
	}
	free(cd);
	return 0;
}

void zip_close(zip_file *z)
{
	if (z->fd >= 0) close(z->fd);
	free(z->entries);
	memset(z, 0, sizeof *z);
	z->fd = -1;
}

const zip_entry *zip_find(const zip_file *z, const char *name)
{
	for (int i = 0; i < z->count; i++)
		if (!strcmp(z->entries[i].name, name)) return &z->entries[i];
	return NULL;
}

int zip_entry_is_qon(const zip_file *z, const zip_entry *e)
{
	uint8_t m[4];
	return e->method == 0 && e->comp_size >= QON_BARE_HEADER_SIZE && !read_full_at(z->fd, m, 4, e->data_off) &&
	       !memcmp(m, "qon1", 4);
}

int qon_open(qon_stream *q, int fd, uint64_t base, uint64_t size, char *err, size_t errlen)
{
	memset(q, 0, sizeof *q);
	q->fd = fd;
	q->base = base;
	q->end = base + size;
	uint8_t h[QON_BARE_HEADER_SIZE];
	if (read_full_at(fd, h, sizeof h, base) || !qon_decode_header(h, sizeof h, &q->hdr)) {
		set_err(err, errlen, "bad QON header");
		return -1;
	}
	if (q->hdr.flags & QON_FLAGS_USES_INTERFRAME_COMPRESSION) {
		set_err(err, errlen, "QON stream uses inter-frame compression (ares does not support this either)");
		return -1;
	}
	uint32_t fc = q->hdr.frame_count;
	size_t isz = (size_t)fc * QON_INDEX_SIZE_PER_ENTRY;
	uint8_t *idx = malloc(isz);
	q->off = malloc(((size_t)fc + 1) * sizeof(uint64_t));
	if (!idx || !q->off || read_full_at(fd, idx, isz, base + QON_BARE_HEADER_SIZE)) {
		free(idx);
		set_err(err, errlen, "cannot read QON index");
		return -1;
	}
	uint64_t data_base = base + QON_BARE_HEADER_SIZE + isz;
	for (uint32_t i = 0; i < fc; i++) q->off[i] = data_base + (rd64le(idx + (size_t)i * 8) & 0x0000FFFFFFFFFFFFull);
	free(idx);
	uint8_t sz[4];
	if (read_full_at(fd, sz, 4, q->off[fc - 1])) { set_err(err, errlen, "cannot read last frame"); return -1; }
	q->off[fc] = q->off[fc - 1] + 4 + rd32le(sz);
	return 0;
}

void qon_close(qon_stream *q)
{
	free(q->off);
	q->off = NULL;
}

uint64_t qon_frame_span(const qon_stream *q, uint32_t i)
{
	return q->off[i + 1] - q->off[i];
}

int qon_read_frame(const qon_stream *q, uint32_t i, uint8_t *rgb, uint8_t **scratch, size_t *cap)
{
	uint8_t szb[4];
	if (read_full_at(q->fd, szb, 4, q->off[i])) return -1;
	uint32_t len = rd32le(szb);
	if (q->off[i] + 4 + len > q->end) return -1;
	if (len > *cap) {
		free(*scratch);
		*cap = len;
		*scratch = malloc(len);
		if (!*scratch) { *cap = 0; return -1; }
	}
	if (read_full_at(q->fd, *scratch, len, q->off[i] + 4)) return -1;
	qoi2_desc d = { q->hdr.width, q->hdr.height, q->hdr.channels, q->hdr.colorspace };
	return qoi2_decode_data(*scratch, len, &d, NULL, rgb, 3) ? 0 : -1;
}
