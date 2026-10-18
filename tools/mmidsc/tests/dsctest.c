/* Round-trip tests for dsc.c: the decoder must reproduce the encoder's
 * reconstruction bit for bit, every slice must fit its budget, and the PPS
 * must survive pack/unpack. */
#include "../src/dsc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned rng = 1;
static unsigned rnd(void) { rng = rng * 1103515245u + 12345u; return rng >> 8; }

static void make_image(int kind, uint8_t *img, int w, int h, int seed)
{
	rng = (unsigned)seed * 2654435761u + 7;
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			uint8_t *p = img + ((size_t)y * w + x) * 3;
			int r, g, b;
			switch (kind) {
			case 0: /* white noise: worst case for rate control */
				r = rnd() & 255; g = rnd() & 255; b = rnd() & 255;
				break;
			case 1: /* smooth gradients */
				r = (x * 255) / w; g = (y * 255) / h; b = ((x + y) * 127) / (w + h) + 64;
				break;
			case 2: /* flat with a few blocks: exercises flatness and ICH */
				r = g = b = 40;
				if ((x / 24 + y / 16) % 5 == 0) { r = 200; g = 30; b = 90; }
				if (x % 50 < 2) { r = g = b = 250; }
				break;
			case 3: { /* video-like: gradients + moving circle + low noise */
				int n = (int)(rnd() % 9) - 4;
				int dx = x - w / 3, dy = y - h / 2;
				r = (x + seed * 3) % 256 + n;
				g = (y * 2 + seed) % 256 + n;
				b = 128 + (x - y) / 8 + n;
				if (dx * dx + dy * dy < 60 * 60) { r = 220 + n; g = 170 - dy / 3; b = 40; }
				break;
			}
			case 4: /* text-like: sharp 2-colour glyph pattern with repeats (ICH food) */
				r = g = b = 16;
				if (((x / 3) ^ (y / 2)) % 7 == 0 && (y % 12) < 9) { r = 235; g = 235; b = 180; }
				break;
			case 5: /* flat, then noise in the last eighth: stresses the end-of-slice budget */
				r = g = b = 90;
				if (y >= h - h / 8) { r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; }
				break;
			case 6: /* 1-pixel checkerboard of saturated colours */
				if ((x + y) & 1) { r = 255; g = 0; b = 255; } else { r = 0; g = 255; b = 0; }
				break;
			case 7: /* random vertical stripes: defeats MMAP's horizontal prediction */
				r = (int)((x * 2654435761u) >> 24) & 255; g = (int)((x * 40503u) >> 8) & 255; b = (y & 1) ? 255 : 0;
				break;
			default: /* mixed: noise band in the middle of smooth content */
				r = (x * 3) & 255; g = (y * 5) & 255; b = 100;
				if (y > h / 3 && y < h / 2) { r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; }
			}
			p[0] = (uint8_t)(r < 0 ? 0 : r > 255 ? 255 : r);
			p[1] = (uint8_t)(g < 0 ? 0 : g > 255 ? 255 : g);
			p[2] = (uint8_t)(b < 0 ? 0 : b > 255 ? 255 : b);
		}
	}
}

static double psnr(const uint8_t *a, const uint8_t *b, size_t n)
{
	double se = 0;
	for (size_t i = 0; i < n; i++) { double d = (double)a[i] - b[i]; se += d * d; }
	if (se == 0) return 99.0;
	return 10 * log10(255.0 * 255.0 / (se / n));
}

/* Random collage of rectangles with different content types, to hunt for
 * rate-control budget overflows at the .dscmmi geometry. */
static void make_collage(uint8_t *img, int w, int h, unsigned seed)
{
	rng = seed * 2654435761u + 99;
	memset(img, (int)(rnd() & 255), (size_t)w * h * 3);
	int rects = 1 + (int)(rnd() % 40);
	for (int r = 0; r < rects; r++) {
		int x0 = (int)(rnd() % (unsigned)w), y0 = (int)(rnd() % (unsigned)h);
		int x1 = x0 + 1 + (int)(rnd() % (unsigned)(w - x0)), y1 = y0 + 1 + (int)(rnd() % (unsigned)(h - y0));
		int kind = (int)(rnd() % 5);
		int base[3] = { (int)(rnd() & 255), (int)(rnd() & 255), (int)(rnd() & 255) };
		int amp = (int)(rnd() % 64);
		for (int y = y0; y < y1; y++)
			for (int x = x0; x < x1; x++)
				for (int c = 0; c < 3; c++) {
					int v;
					switch (kind) {
					case 0: v = base[c]; break;
					case 1: v = (int)(rnd() & 255); break;
					case 2: v = base[c] + (int)(rnd() % (unsigned)(2 * amp + 1)) - amp; break;
					case 3: v = base[c] + ((x - x0) * 255) / (x1 - x0 + 1) - 128; break;
					default: v = ((x ^ y) & 1) ? base[c] : 255 - base[c];
					}
					img[((size_t)y * w + x) * 3 + c] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
				}
	}
}

static int stress(int count)
{
	dsc_pps p;
	char err[200];
	if (dsc_pps_init(&p, 792, 525, 263, 128, err, sizeof err)) { printf("%s\n", err); return 1; }
	size_t n = (size_t)792 * 525 * 3, bytes = dsc_picture_bytes(&p);
	uint8_t *img = malloc(n), *rec = malloc(n), *dec = malloc(n), *bs = malloc(bytes);
	int fails = 0;
	double worst = 0;
	for (int i = 0; i < count; i++) {
		make_collage(img, 792, 525, (unsigned)i + 1);
		dsc_stats st;
		memset(&st, 0, sizeof st);
		dsc_enc_opts o = { .disable_ich = i & 1 };
		int er = dsc_encode_picture(&p, &o, img, 792 * 3, bs, rec, &st);
		int dr = dsc_decode_picture(&p, bs, bytes, dec);
		double used = (double)st.slice_bits_used_max / p.slice_bits;
		if (used > worst) worst = used;
		if (er || dr || memcmp(rec, dec, n)) { fails++; printf("FAIL collage %d enc=%d dec=%d used=%.4f\n", i, er, dr, used); }
	}
	printf("stress: %d collages at 792x525, %d failures, fullest slice used %.3f%% of its budget\n", count, fails, 100 * worst);
	return fails != 0;
}

int main(int argc, char **argv)
{
	if (argc > 2 && !strcmp(argv[1], "--stress")) return stress(atoi(argv[2]));
	const char *names[] = { "noise", "gradient", "flat+blocks", "video-like", "text-like", "flat>noise", "checker", "stripes", "mixed" };
	int sizes[][3] = { { 792, 526, 263 }, { 792, 526, 526 }, { 300, 100, 100 }, { 99, 37, 8 }, { 792, 263, 32 } };
	int bpps[] = { 128, 96, 144, 160 };
	int fails = 0, runs = 0;
	for (size_t si = 0; si < sizeof sizes / sizeof sizes[0]; si++) {
		int w = sizes[si][0], h = sizes[si][1], sh = sizes[si][2];
		for (size_t bi = 0; bi < sizeof bpps / sizeof bpps[0]; bi++) {
			dsc_pps p, q;
			char err[200];
			if (dsc_pps_init(&p, w, h, sh, bpps[bi], err, sizeof err)) {
				printf("skip %dx%d sh=%d bpp=%d: %s\n", w, h, sh, bpps[bi], err);
				continue;
			}
			uint8_t pps[DSC_PPS_SIZE];
			dsc_pps_pack(&p, pps);
			if (dsc_pps_unpack(&q, pps, err, sizeof err) || memcmp(&p, &q, sizeof p)) {
				printf("FAIL PPS round trip %dx%d: %s\n", w, h, err);
				fails++;
			}
			size_t n = (size_t)w * h * 3;
			uint8_t *img = malloc(n), *rec = malloc(n), *dec = malloc(n);
			size_t bytes = dsc_picture_bytes(&p);
			uint8_t *bs = malloc(bytes);
			for (int kind = 0; kind < 9; kind++) {
				for (int ich = 0; ich < 2; ich++) {
					make_image(kind, img, w, h, kind * 31 + (int)si);
					dsc_enc_opts o = { .disable_ich = ich };
					dsc_stats st;
					memset(&st, 0, sizeof st);
					int er = dsc_encode_picture(&q, &o, img, (size_t)w * 3, bs, rec, &st);
					int dr = dsc_decode_picture(&q, bs, bytes, dec);
					int same = memcmp(rec, dec, n) == 0;
					runs++;
					int ok = er == 0 && dr == 0 && same;
					if (!ok) fails++;
					printf("%s %4dx%-3d sh=%-3d %5.2fbpp %-11s ich=%s  PSNR %5.2f dB  used %6.2f%% of slice  ich %5d mpp %6d forced %4d flat %4d\n",
					       ok ? "ok  " : "FAIL", w, h, sh, bpps[bi] / 16.0, names[kind], ich ? "off" : "on ",
					       psnr(img, dec, n), 100.0 * st.slice_bits_used_max / p.slice_bits, st.ich_groups, st.mpp_units,
					       st.force_mpp, st.flat_signals);
					if (!ok) printf("     enc=%d dec=%d identical=%d\n", er, dr, same);
				}
			}
			free(img); free(rec); free(dec); free(bs);
		}
	}
	printf("\n%d runs, %d failures\n", runs, fails);
	return fails != 0;
}
