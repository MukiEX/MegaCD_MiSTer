/* Small helpers shared by the tools. SPDX-License-Identifier: MIT */
#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>
#include <stdint.h>

/* Build with -D_FILE_OFFSET_BITS=64 so off_t is 64-bit on 32-bit hosts. */

uint32_t crc32_update(uint32_t crc, const void *buf, size_t len);
uint32_t crc32_combine64(uint32_t crc1, uint32_t crc2, uint64_t len2);

uint16_t rd16le(const uint8_t *p);
uint32_t rd32le(const uint8_t *p);
uint64_t rd64le(const uint8_t *p);
void wr16le(uint8_t *p, uint16_t v);
void wr32le(uint8_t *p, uint32_t v);
void wr64le(uint8_t *p, uint64_t v);

int read_full_at(int fd, void *buf, size_t len, uint64_t off);
int write_full_at(int fd, const void *buf, size_t len, uint64_t off);

double now_seconds(void);

/* Uncompressed (stored-deflate) RGB PNG writer. */
int write_png_rgb(const char *path, const uint8_t *rgb, int w, int h, size_t stride);

/* LaserDisc VBI biphase code reader, a port of ares' MCD::LD::
 * decodeBiphaseCodeFromScanline (ares/md/mcd/megald.cpp). frame is packed
 * RGB of width w (the original capture width, normally 910); x_offset is
 * subtracted from sample positions (0 for an uncropped frame). Returns the
 * 24-bit code or 0. */
uint32_t vbi_decode_line(const uint8_t *frame, int w, int src_width, int x_offset, int line);

#endif
