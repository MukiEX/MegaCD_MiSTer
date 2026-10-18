/* SPDX-License-Identifier: MIT */
#include "zipw.h"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"

#ifdef _WIN32
#include <io.h>
#define OFLAGS (O_RDWR | O_CREAT | O_TRUNC | O_BINARY)
#else
#include <unistd.h>
#define OFLAGS (O_RDWR | O_CREAT | O_TRUNC)
#endif

#define ALIGN_EXTRA_ID 0xD935 /* same ID zipalign uses; readers skip unknown extra fields */

int zw_open(zip_writer *w, const char *path)
{
	memset(w, 0, sizeof *w);
	w->fd = open(path, OFLAGS, 0644);
	return w->fd < 0 ? -1 : 0;
}

uint64_t zw_begin(zip_writer *w, const char *name, uint16_t method, uint16_t flags, uint16_t mtime, uint16_t mdate,
                  uint64_t comp_size, uint64_t uncomp_size, uint32_t align)
{
	if (w->n == w->cap) {
		w->cap = w->cap ? w->cap * 2 : 16;
		w->ents = realloc(w->ents, sizeof(zw_entry) * (size_t)w->cap);
	}
	zw_entry *e = &w->ents[w->n++];
	memset(e, 0, sizeof *e);
	strncpy(e->name, name, sizeof e->name - 1);
	e->method = method;
	e->flags = flags & ~0x0008; /* sizes are in the local header, no data descriptor */
	e->mtime = mtime;
	e->mdate = mdate;
	e->comp = comp_size;
	e->uncomp = uncomp_size;
	e->local_off = w->pos;

	size_t nlen = strlen(e->name);
	uint64_t base = w->pos + 30 + nlen + 20;
	size_t pad_extra = 0;
	if (align > 1 && base % align) {
		uint64_t pad = (align - (base + 4) % align) % align;
		pad_extra = 4 + (size_t)pad;
	}
	size_t xlen = 20 + pad_extra;
	size_t hlen = 30 + nlen + xlen;
	uint8_t *h = calloc(1, hlen);
	wr32le(h, 0x04034b50);
	wr16le(h + 4, 45);
	wr16le(h + 6, e->flags);
	wr16le(h + 8, method);
	wr16le(h + 10, mtime);
	wr16le(h + 12, mdate);
	wr32le(h + 14, 0); /* CRC patched by zw_end */
	wr32le(h + 18, 0xffffffffu);
	wr32le(h + 22, 0xffffffffu);
	wr16le(h + 26, (uint16_t)nlen);
	wr16le(h + 28, (uint16_t)xlen);
	memcpy(h + 30, e->name, nlen);
	uint8_t *x = h + 30 + nlen;
	wr16le(x, 0x0001);
	wr16le(x + 2, 16);
	wr64le(x + 4, uncomp_size);
	wr64le(x + 12, comp_size);
	if (pad_extra) {
		wr16le(x + 20, ALIGN_EXTRA_ID);
		wr16le(x + 22, (uint16_t)(pad_extra - 4));
	}
	write_full_at(w->fd, h, hlen, w->pos);
	free(h);
	uint64_t data = w->pos + hlen;
	w->pos = data + comp_size;
	return data;
}

int zw_end(zip_writer *w, uint32_t crc)
{
	zw_entry *e = &w->ents[w->n - 1];
	e->crc = crc;
	uint8_t c[4];
	wr32le(c, crc);
	return write_full_at(w->fd, c, 4, e->local_off + 14);
}

int zw_finish(zip_writer *w)
{
	uint64_t cd_off = w->pos;
	int err = 0;
	for (int i = 0; i < w->n; i++) {
		zw_entry *e = &w->ents[i];
		size_t nlen = strlen(e->name);
		uint8_t h[46 + 512 + 28];
		memset(h, 0, sizeof h);
		wr32le(h, 0x02014b50);
		wr16le(h + 4, 45);
		wr16le(h + 6, 45);
		wr16le(h + 8, e->flags);
		wr16le(h + 10, e->method);
		wr16le(h + 12, e->mtime);
		wr16le(h + 14, e->mdate);
		wr32le(h + 16, e->crc);
		wr32le(h + 20, 0xffffffffu);
		wr32le(h + 24, 0xffffffffu);
		wr16le(h + 28, (uint16_t)nlen);
		wr16le(h + 30, 28);
		wr32le(h + 42, 0xffffffffu);
		memcpy(h + 46, e->name, nlen);
		uint8_t *x = h + 46 + nlen;
		wr16le(x, 0x0001);
		wr16le(x + 2, 24);
		wr64le(x + 4, e->uncomp);
		wr64le(x + 12, e->comp);
		wr64le(x + 20, e->local_off);
		size_t len = 46 + nlen + 28;
		err |= write_full_at(w->fd, h, len, w->pos);
		w->pos += len;
	}
	uint64_t cd_size = w->pos - cd_off;
	uint64_t eocd64 = w->pos;
	uint8_t r[56 + 20 + 22];
	memset(r, 0, sizeof r);
	wr32le(r, 0x06064b50);
	wr64le(r + 4, 44);
	wr16le(r + 12, 45);
	wr16le(r + 14, 45);
	wr64le(r + 24, (uint64_t)w->n);
	wr64le(r + 32, (uint64_t)w->n);
	wr64le(r + 40, cd_size);
	wr64le(r + 48, cd_off);
	uint8_t *l = r + 56;
	wr32le(l, 0x07064b50);
	wr64le(l + 8, eocd64);
	wr32le(l + 16, 1);
	uint8_t *e = l + 20;
	wr32le(e, 0x06054b50);
	wr16le(e + 8, 0xffff);
	wr16le(e + 10, 0xffff);
	wr32le(e + 12, 0xffffffffu);
	wr32le(e + 16, 0xffffffffu);
	err |= write_full_at(w->fd, r, sizeof r, w->pos);
	w->pos += sizeof r;
	err |= close(w->fd) != 0;
	free(w->ents);
	w->ents = NULL;
	return err ? -1 : 0;
}
