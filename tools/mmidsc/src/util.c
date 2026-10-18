/* SPDX-License-Identifier: MIT */
#include "util.h"

#include <stdlib.h>
#include <string.h>

/* MMIDSC_WASM: the browser converter (web/) builds only the CRC, byte-order
 * and VBI parts; it has no files or clock. */
#ifndef MMIDSC_WASM
#include <errno.h>
#include <stdio.h>
#include <time.h>

#ifdef _WIN32
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#endif
#endif

/* ---------------------------------------------------------------- CRC32 */

static uint32_t crc_table[256];
static int crc_ready;

static void crc_init(void)
{
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;
		for (int k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		crc_table[i] = c;
	}
	crc_ready = 1;
}

uint32_t crc32_update(uint32_t crc, const void *buf, size_t len)
{
	if (!crc_ready) crc_init();
	const uint8_t *p = buf;
	crc = ~crc;
	while (len--) crc = crc_table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
	return ~crc;
}

/* CRC of A||B from CRC(A), CRC(B) and len(B): multiply CRC(A) by x^(8*len2)
 * modulo the CRC polynomial (bit-reflected arithmetic), then xor CRC(B). */
static uint32_t multmodp(uint32_t a, uint32_t b)
{
	uint32_t m = 1u << 31, p = 0;
	for (;;) {
		if (a & m) {
			p ^= b;
			if ((a & (m - 1)) == 0) break;
		}
		m >>= 1;
		b = b & 1 ? (b >> 1) ^ 0xEDB88320u : b >> 1;
	}
	return p;
}

uint32_t crc32_combine64(uint32_t crc1, uint32_t crc2, uint64_t len2)
{
	static uint32_t x2n[64];
	static int ready;
	if (!ready) {
		uint32_t p = 1u << 30; /* x^1 */
		x2n[0] = p;
		for (int n = 1; n < 64; n++) x2n[n] = p = multmodp(p, p);
		ready = 1;
	}
	/* x^(8*len2) = x^(len2 << 3) */
	uint32_t p = 1u << 31; /* x^0 */
	uint64_t n = len2;
	int k = 3;
	while (n) {
		if (n & 1) p = multmodp(x2n[k & 63], p);
		n >>= 1;
		k++;
	}
	return multmodp(p, crc1) ^ crc2;
}

/* ---------------------------------------------------------- endianness */

uint16_t rd16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
uint64_t rd64le(const uint8_t *p) { return (uint64_t)rd32le(p) | ((uint64_t)rd32le(p + 4) << 32); }
void wr16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void wr32le(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
void wr64le(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }

#ifndef MMIDSC_WASM
/* ------------------------------------------------------- positional I/O */

#ifdef _WIN32
int read_full_at(int fd, void *buf, size_t len, uint64_t off)
{
	HANDLE h = (HANDLE)_get_osfhandle(fd);
	uint8_t *b = buf;
	while (len) {
		OVERLAPPED ov;
		memset(&ov, 0, sizeof ov);
		ov.Offset = (DWORD)off;
		ov.OffsetHigh = (DWORD)(off >> 32);
		DWORD chunk = len > (1u << 30) ? (1u << 30) : (DWORD)len, got = 0;
		if (!ReadFile(h, b, chunk, &got, &ov) || got == 0) return -1;
		b += got; len -= got; off += got;
	}
	return 0;
}

int write_full_at(int fd, const void *buf, size_t len, uint64_t off)
{
	HANDLE h = (HANDLE)_get_osfhandle(fd);
	const uint8_t *b = buf;
	while (len) {
		OVERLAPPED ov;
		memset(&ov, 0, sizeof ov);
		ov.Offset = (DWORD)off;
		ov.OffsetHigh = (DWORD)(off >> 32);
		DWORD chunk = len > (1u << 30) ? (1u << 30) : (DWORD)len, put = 0;
		if (!WriteFile(h, b, chunk, &put, &ov) || put == 0) return -1;
		b += put; len -= put; off += put;
	}
	return 0;
}
#else
int read_full_at(int fd, void *buf, size_t len, uint64_t off)
{
	uint8_t *b = buf;
	while (len) {
		ssize_t r = pread(fd, b, len, (off_t)off);
		if (r < 0) { if (errno == EINTR) continue; return -1; }
		if (r == 0) return -1;
		b += r; len -= (size_t)r; off += (uint64_t)r;
	}
	return 0;
}

int write_full_at(int fd, const void *buf, size_t len, uint64_t off)
{
	const uint8_t *b = buf;
	while (len) {
		ssize_t r = pwrite(fd, b, len, (off_t)off);
		if (r < 0) { if (errno == EINTR) continue; return -1; }
		if (r == 0) return -1;
		b += r; len -= (size_t)r; off += (uint64_t)r;
	}
	return 0;
}
#endif

double now_seconds(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ------------------------------------------------------------------ PNG */

static void be32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }

static int png_chunk(FILE *f, const char *type, const uint8_t *data, uint32_t len)
{
	uint8_t hdr[8];
	be32(hdr, len);
	memcpy(hdr + 4, type, 4);
	uint32_t crc = crc32_update(0, type, 4);
	if (len) crc = crc32_update(crc, data, len);
	uint8_t c[4];
	be32(c, crc);
	return fwrite(hdr, 1, 8, f) != 8 || (len && fwrite(data, 1, len, f) != len) || fwrite(c, 1, 4, f) != 4;
}

int write_png_rgb(const char *path, const uint8_t *rgb, int w, int h, size_t stride)
{
	FILE *f = fopen(path, "wb");
	if (!f) return -1;
	static const uint8_t sig[8] = { 137, 80, 78, 71, 13, 10, 26, 10 };
	fwrite(sig, 1, 8, f);
	uint8_t ihdr[13];
	be32(ihdr, (uint32_t)w);
	be32(ihdr + 4, (uint32_t)h);
	ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
	png_chunk(f, "IHDR", ihdr, 13);

	size_t row = (size_t)w * 3 + 1;
	size_t raw_len = row * (size_t)h;
	size_t blocks = (raw_len + 65534) / 65535;
	size_t z_len = 2 + raw_len + blocks * 5 + 4;
	uint8_t *z = malloc(z_len);
	if (!z) { fclose(f); return -1; }
	size_t zp = 0;
	z[zp++] = 0x78; z[zp++] = 0x01;
	uint32_t a = 1, b = 0;
	size_t remaining = raw_len, block_left = 0;
	size_t len_pos = 0;
	for (int y = 0; y < h; y++) {
		for (size_t i = 0; i < row; i++) {
			if (block_left == 0) {
				size_t n = remaining > 65535 ? 65535 : remaining;
				z[zp++] = remaining == n ? 1 : 0;
				len_pos = zp;
				z[zp++] = (uint8_t)n; z[zp++] = (uint8_t)(n >> 8);
				z[zp++] = (uint8_t)~n; z[zp++] = (uint8_t)(~n >> 8);
				block_left = n;
			}
			uint8_t v = i == 0 ? 0 : rgb[(size_t)y * stride + i - 1];
			z[zp++] = v;
			a = (a + v) % 65521;
			b = (b + a) % 65521;
			block_left--;
			remaining--;
		}
	}
	(void)len_pos;
	be32(z + zp, (b << 16) | a);
	zp += 4;
	int err = png_chunk(f, "IDAT", z, (uint32_t)zp);
	free(z);
	err |= png_chunk(f, "IEND", NULL, 0);
	err |= fclose(f) != 0;
	return err ? -1 : 0;
}

#endif /* MMIDSC_WASM */

/* -------------------------------------------------------------- VBI */

uint32_t vbi_decode_line(const uint8_t *frame, int w, int src_width, int x_offset, int line)
{
	/* Constants as in ares: 29.97 fps, 525-line frame buffer, 2 us bit cells */
	const double fps = 30.0 / 1.001;
	double lines_per_second = fps * 525.0;
	double us_per_line = 1e6 / lines_per_second;
	double cell = (double)src_width * (2.0 / us_per_line);
	double quarter = cell / 4.0;
	size_t search_start = (size_t)((float)src_width * 0.15f);
	size_t search_end = (size_t)((float)src_width * 0.25f);
	size_t search_len = (size_t)(cell / 4.0);

	const uint8_t *row = frame + (size_t)line * w * 3;
#define PIX(x) ((int)(x) - x_offset >= 0 && (int)(x) - x_offset < w ? row + ((size_t)((int)(x) - x_offset)) * 3 : NULL)
	size_t start = search_start;
	int found = 0;
	while (!found && start < search_end) {
		int fail = 0;
		for (size_t i = 0; i < search_len; i++) {
			const uint8_t *p = PIX(start + i);
			if (!p || p[0] < 0x80 || p[1] < 0x80 || p[2] < 0x80) { fail = 1; break; }
		}
		if (!fail) { found = 1; break; }
		start++;
	}
	if (!found) return 0;
	start -= (size_t)(cell / 2.0);
	uint32_t code = 0;
	for (int bit = 0; bit < 24; bit++) {
		size_t lead = (size_t)(cell * bit + quarter);
		size_t trail = (size_t)(cell * bit + quarter * 3.0);
		const uint8_t *pl = PIX(start + lead), *pt = PIX(start + trail);
		if (!pl || !pt) return 0;
		int lv = pl[0] >= 0xC0 && pl[1] >= 0xC0 && pl[2] >= 0xC0;
		int tv = pt[0] >= 0xC0 && pt[1] >= 0xC0 && pt[2] >= 0xC0;
		if (lv == tv) return 0;
		code = (code << 1) | (uint32_t)tv;
	}
#undef PIX
	return code;
}
