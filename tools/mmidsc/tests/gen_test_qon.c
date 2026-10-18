/* Generates a synthetic LaserActive-like QON stream for testing: 910x525
 * frames with two stacked fields, a moving scene with analog-style noise,
 * black horizontal blanking, and real biphase VBI codes on rows 15/16 and
 * 278/279 (lines 16/17/279/280) in the format ares decodes.
 * usage: gen_test_qon out.qon [frames] [noise] */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define QOI2_IMPLEMENTATION
#define QON_IMPLEMENTATION
#include "../src/qon.h"
#include "../src/qoi2.h"

static unsigned seed = 12345;
static int noise(int amp)
{
	if (!amp) return 0;
	seed = seed * 1103515245u + 12345u;
	return (int)((seed >> 16) % (unsigned)(2 * amp + 1)) - amp;
}

static void put(unsigned char *p, int r, int g, int b)
{
	p[0] = (unsigned char)(r < 0 ? 0 : r > 255 ? 255 : r);
	p[1] = (unsigned char)(g < 0 ? 0 : g > 255 ? 255 : g);
	p[2] = (unsigned char)(b < 0 ? 0 : b > 255 ? 255 : b);
}

static void biphase(unsigned char *row, unsigned code, int amp)
{
	double us_per_line = 1e6 / (30.0 / 1.001 * 525.0);
	double cell = 910.0 * 2.0 / us_per_line;
	double x0 = 150.0;
	for (int bit = 0; bit < 24; bit++) {
		int v = (code >> (23 - bit)) & 1;
		for (int half = 0; half < 2; half++) {
			int high = half == 0 ? !v : v;
			int xs = (int)(x0 + cell * bit + half * cell / 2), xe = (int)(x0 + cell * bit + (half + 1) * cell / 2);
			for (int x = xs; x < xe && x < 910; x++) {
				int l = high ? 230 : 20;
				put(row + x * 3, l + noise(amp), l + noise(amp), l + noise(amp));
			}
		}
	}
}

int main(int argc, char **argv)
{
	if (argc < 2) { fprintf(stderr, "usage: %s out.qon [frames] [noise]\n", argv[0]); return 2; }
	unsigned frames = argc > 2 ? (unsigned)atoi(argv[2]) : 120;
	int amp = argc > 3 ? atoi(argv[3]) : 3;
	const unsigned w = 910, h = 525;
	qon_desc qd = { w, h, 3, QON_SRGB, 0, frames, 33367 };
	qoi2_desc fd = { w, h, 3, QOI2_SRGB };
	size_t maxenc = qoi2_max_encoded_data_size_without_header(&fd);
	unsigned char *px = malloc((size_t)w * h * 3), *enc = malloc(maxenc);
	unsigned char *index = calloc(frames, QON_INDEX_SIZE_PER_ENTRY);
	FILE *f = fopen(argv[1], "wb");
	if (!f || !px || !enc || !index) return 1;
	unsigned char hdr[QON_BARE_HEADER_SIZE];
	qon_encode_header(&qd, hdr);
	fwrite(hdr, 1, sizeof hdr, f);
	fwrite(index, QON_INDEX_SIZE_PER_ENTRY, frames, f);
	unsigned long long off = 0;
	for (unsigned n = 0; n < frames; n++) {
		for (unsigned y = 0; y < h; y++) {
			unsigned field_line = y < 263 ? y : y - 263;
			for (unsigned x = 0; x < w; x++) {
				unsigned char *p = px + ((size_t)y * w + x) * 3;
				if (x < 118 || field_line < 21) { /* blanking and VBI: black with noise */
					put(p, 16 + noise(amp), 16 + noise(amp), 16 + noise(amp));
					continue;
				}
				double t = n * 0.05 + (y >= 263 ? 0.025 : 0);
				double fx = (x - 118) / 792.0, fy = field_line / 263.0;
				/* sky gradient, rolling hills, a moving sun and a striped building */
				int r = (int)(60 + 120 * fy), g = (int)(90 + 100 * fy), b = (int)(200 - 80 * fy);
				double hill = 0.55 + 0.08 * sin(fx * 9 + t) + 0.04 * sin(fx * 23 - t * 2);
				if (fy > hill) { r = (int)(40 + 30 * sin(fx * 40)); g = (int)(110 + 40 * fy); b = 50; }
				double sx = 0.2 + 0.6 * fmod(t * 0.3, 1.0), sy = 0.3;
				if ((fx - sx) * (fx - sx) * 4 + (fy - sy) * (fy - sy) < 0.004) { r = 250; g = 220; b = 90; }
				if (fx > 0.7 && fx < 0.85 && fy > 0.25) { r = g = b = ((int)(fx * 400) % 6 < 3) ? 200 : 90; }
				put(p, r + noise(amp), g + noise(amp), b + noise(amp));
			}
		}
		biphase(px + (size_t)15 * w * 3, 0x88FFFF, amp);
		biphase(px + (size_t)16 * w * 3, 0xF80000 | (n & 0x7FFFF), amp);
		biphase(px + (size_t)278 * w * 3, 0x88FFFF, amp);
		biphase(px + (size_t)279 * w * 3, 0xF80000 | (n & 0x7FFFF), amp);
		size_t len = maxenc;
		if (!qoi2_encode_data(px, 3, &fd, NULL, enc, &len)) { fprintf(stderr, "encode failed\n"); return 1; }
		unsigned char sz[4];
		qon_encode_frame_size(sz, len);
		qon_encode_index_entry(index, n, (size_t)off, 0);
		fwrite(sz, 1, 4, f);
		fwrite(enc, 1, len, f);
		off += 4 + len;
	}
	fseek(f, QON_BARE_HEADER_SIZE, SEEK_SET);
	fwrite(index, QON_INDEX_SIZE_PER_ENTRY, frames, f);
	fclose(f);
	fprintf(stderr, "wrote %u frames, %.1f MB\n", frames, (QON_BARE_HEADER_SIZE + frames * 8.0 + off) / 1e6);
	return 0;
}
