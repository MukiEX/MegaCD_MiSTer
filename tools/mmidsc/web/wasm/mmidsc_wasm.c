/* The per-frame work of mmidsc's convert command, built for WebAssembly
 * (the browser converter in web/). JS reads the files and writes the
 * output; this decodes a QON frame, reads its VBI codes and encodes it with
 * DSC, exactly as src/main.c (convert_worker) does.
 * SPDX-License-Identifier: MIT */
#define MMIDSC_WASM 1
#define QOI2_IMPLEMENTATION
#include "../../src/qoi2.h"
#define DSC_NO_ENC_CACHE /* heap_reset() after each frame frees everything */
#include "../../src/dsc.c"
#include "../../src/util.c"

#define EXPORT(n) __attribute__((export_name(n)))

void heap_mark(void);
void heap_reset(void);

/* ------------------------------------------- same as src/main.c */
typedef struct {
	int src_w, src_h;
	int crop_x, width, height, slice_h;
	int f0, f0n, f1, f1n;
	int bpp16;
} geom;

static void pick_geometry(int src_w, int src_h, int bpp16, geom *g)
{
	memset(g, 0, sizeof *g);
	g->src_w = src_w;
	g->src_h = src_h;
	g->bpp16 = bpp16;
	if (src_w == 910 && src_h == 525) {
		g->crop_x = 118;
		g->width = 792;
		g->height = 525;
		g->slice_h = 263;
		g->f0 = 0; g->f0n = 263;
		g->f1 = 263; g->f1n = 262;
	} else {
		g->crop_x = 0;
		g->width = src_w - src_w % 3;
		g->height = src_h;
		g->slice_h = (src_h + 1) / 2;
		g->f0 = 0; g->f0n = g->slice_h;
		g->f1 = g->slice_h; g->f1n = src_h - g->slice_h;
	}
	while (g->width > 9 && (g->width * bpp16) % 128) g->width -= 3;
}

static int encode_with_fallback(const dsc_pps *p, const geom *g, const dsc_enc_opts *opts, const uint8_t *pic, size_t stride,
                                uint8_t *out, int *fallback)
{
	*fallback = 0;
	if (!dsc_encode_picture(p, opts, pic, stride, out, NULL, NULL)) return 0;
	dsc_enc_opts o2 = *opts;
	o2.disable_ich = !o2.disable_ich;
	*fallback = 1;
	if (!dsc_encode_picture(p, &o2, pic, stride, out, NULL, NULL)) return 0;
	size_t row = (size_t)g->width * 3;
	uint8_t *soft = malloc(row * g->height);
	for (int y = 0; y < g->height; y++)
		for (int x = 0; x < g->width; x++)
			for (int c = 0; c < 3; c++) {
				int l = pic[y * stride + (size_t)(x > 0 ? x - 1 : x) * 3 + c];
				int m = pic[y * stride + (size_t)x * 3 + c];
				int r = pic[y * stride + (size_t)(x < g->width - 1 ? x + 1 : x) * 3 + c];
				soft[y * row + (size_t)x * 3 + c] = (uint8_t)((l + 2 * m + r + 2) / 4);
			}
	int ret = dsc_encode_picture(p, opts, soft, row, out, NULL, NULL);
	free(soft);
	return ret;
}

static const int VBI_ROWS[4] = { 15, 16, 278, 279 };

/* ------------------------------------------------------------ state */
/* info[] layout (32-bit words), read by JS:
 *   0 crop_x  1 width  2 height  3 f0  4 f0n  5 f1  6 f1n
 *   7 frame_bytes  8 stride  9 slices_per_picture
 *   10..13 VBI codes of the last frame  14 CRC32 of the last frame (stride bytes)
 *   32.. 128 bytes: the packed PPS */
static uint32_t info[64];
static geom G;
static dsc_pps P;
static dsc_enc_opts O;
static uint8_t *src, *in, *out;
static int in_cap, channels, colorspace;
static char err[256];

EXPORT("info_ptr") uint32_t *info_ptr(void) { return info; }
EXPORT("in_ptr") uint8_t *in_ptr(void) { return in; }
EXPORT("out_ptr") uint8_t *out_ptr(void) { return out; }
EXPORT("err_ptr") char *err_ptr(void) { return err; }

/* Returns 0, or -1 with a message at err_ptr(). Call once per stream. */
EXPORT("setup") int setup(int src_w, int src_h, int ch, int cs, int bpp16, int no_ich, int max_in)
{
	heap_reset();
	pick_geometry(src_w, src_h, bpp16, &G);
	if (dsc_pps_init(&P, G.width, G.height, G.slice_h, bpp16, err, sizeof err)) return -1;
	memset(&O, 0, sizeof O);
	O.disable_ich = no_ich;
	channels = ch;
	colorspace = cs;
	size_t fb = dsc_picture_bytes(&P);
	uint32_t stride = (uint32_t)((fb + 511) & ~(size_t)511);
	memset(info, 0, sizeof info);
	info[0] = (uint32_t)G.crop_x; info[1] = (uint32_t)G.width; info[2] = (uint32_t)G.height;
	info[3] = (uint32_t)G.f0; info[4] = (uint32_t)G.f0n; info[5] = (uint32_t)G.f1; info[6] = (uint32_t)G.f1n;
	info[7] = (uint32_t)fb; info[8] = stride; info[9] = (uint32_t)P.slices_per_picture;
	dsc_pps_pack(&P, (uint8_t *)&info[32]);
	src = malloc((size_t)src_w * src_h * 3);
	in = malloc((size_t)max_in);
	out = malloc(stride);
	in_cap = max_in;
	if (!src || !in || !out) { snprintf(err, sizeof err, "%s", "out of memory"); return -1; }
	heap_mark();
	return 0;
}

/* The compressed frame (without its 4-byte size) is at in_ptr(); in_len < 0
 * means it couldn't be read (stored black, as the native tool does).
 * Leaves stride bytes at out_ptr(), the VBI codes and CRC in info[].
 * Returns bit 0: didn't fit the DSC budget, bit 1: needed a fallback,
 * bit 2: source frame unreadable. */
EXPORT("encode_frame") int encode_frame(int in_len)
{
	int flags = 0;
	size_t src_bytes = (size_t)G.src_w * G.src_h * 3;
	qoi2_desc d = { (unsigned)G.src_w, (unsigned)G.src_h, (unsigned char)channels, (unsigned char)colorspace };
	if (in_len < 0 || in_len > in_cap || !qoi2_decode_data(in, (size_t)in_len, &d, NULL, src, 3)) {
		flags |= 4;
		memset(src, 0, src_bytes);
	}
	for (int k = 0; k < 4; k++) info[10 + k] = vbi_decode_line(src, G.src_w, G.src_w, 0, VBI_ROWS[k]);
	int fallback;
	size_t fb = info[7], stride = info[8];
	int fail = encode_with_fallback(&P, &G, &O, src + (size_t)G.crop_x * 3, (size_t)G.src_w * 3, out, &fallback);
	if (fail) flags |= 1;
	else if (fallback) flags |= 2;
	memset(out + fb, 0, stride - fb);
	info[14] = crc32_update(0, out, stride);
	heap_reset();
	return flags;
}

/* CRC32 helpers for JS (header, copied data) */
EXPORT("crc32") uint32_t wcrc32(uint32_t crc, const uint8_t *p, uint32_t n) { return crc32_update(crc, p, n); }
EXPORT("crc32_combine") uint32_t wcrc32_combine(uint32_t a, uint32_t b, double len2) { return crc32_combine64(a, b, (uint64_t)len2); }
EXPORT("scratch") uint8_t *scratch(uint32_t n) { return malloc(n); }
