/* Bit-exactness + speed harness for dsc.c (make bench-check).
 * Prints one FNV-1a hash per (image, rate, opts) case over the encoded bytes,
 * the reconstruction, the return value and the stats, then the mean encode
 * time. tests/dscbench.golden holds the hashes from the original encoder;
 * any change to dsc.c must reproduce them exactly. "quick" runs 8 video-like
 * frames at 8/9/10 bpp, for timing. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "dsc.h"

static unsigned rng = 1;
static unsigned rnd(void) { rng = rng * 1103515245u + 12345u; return rng >> 8; }
static int clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }

static void make_image(int kind, uint8_t *img, int w, int h, int seed)
{
	rng = (unsigned)seed * 2654435761u + 7;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			uint8_t *p = img + ((size_t)y * w + x) * 3;
			int r, g, b;
			switch (kind) {
			case 0: r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; break;
			case 1: r = (x * 255) / w; g = (y * 255) / h; b = ((x + y) * 127) / (w + h) + 64; break;
			case 2: r = g = b = 40; if ((x / 24 + y / 16) % 5 == 0) { r = 200; g = 30; b = 90; } if (x % 50 < 2) { r = g = b = 250; } break;
			case 3: { int n = (int)(rnd() % 9) - 4; int dx = x - w / 3, dy = y - h / 2;
				r = (x + seed * 3) % 256 + n; g = (y * 2 + seed) % 256 + n; b = 128 + (x - y) / 8 + n;
				if (dx * dx + dy * dy < 60 * 60) { r = 220 + n; g = 170 - dy / 3; b = 40; } break; }
			case 4: r = g = b = 16; if (((x / 3) ^ (y / 2)) % 7 == 0 && (y % 12) < 9) { r = 235; g = 235; b = 180; } break;
			case 5: r = g = b = 90; if (y >= h - h / 8) { r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; } break;
			case 6: if ((x + y) & 1) { r = 255; g = 0; b = 255; } else { r = 0; g = 255; b = 0; } break;
			case 7: r = (int)((x * 2654435761u) >> 24) & 255; g = (int)((x * 40503u) >> 8) & 255; b = (y & 1) ? 255 : 0; break;
			default: r = (x * 3) & 255; g = (y * 5) & 255; b = 100;
				if (y > h / 3 && y < h / 2) { r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; }
			}
			p[0] = (uint8_t)clamp8(r); p[1] = (uint8_t)clamp8(g); p[2] = (uint8_t)clamp8(b);
		}
}

/* LaserActive-like scene (from gen_test_qon), already cropped to 792 wide */
static void make_scene(uint8_t *img, int w, int h, int n, int amp)
{
	rng = 12345u + (unsigned)n * 77u;
	for (int y = 0; y < h; y++) {
		int fl = y < 263 ? y : y - 263;
		for (int x = 0; x < w; x++) {
			uint8_t *p = img + ((size_t)y * w + x) * 3;
			int nz[3];
			for (int c = 0; c < 3; c++) nz[c] = amp ? (int)(rnd() % (unsigned)(2 * amp + 1)) - amp : 0;
			if (fl < 21) { for (int c = 0; c < 3; c++) p[c] = (uint8_t)clamp8(16 + nz[c]); continue; }
			double t = n * 0.05 + (y >= 263 ? 0.025 : 0);
			double fx = x / 792.0, fy = fl / 263.0;
			int r = (int)(60 + 120 * fy), g = (int)(90 + 100 * fy), b = (int)(200 - 80 * fy);
			double hill = 0.55 + 0.08 * sin(fx * 9 + t) + 0.04 * sin(fx * 23 - t * 2);
			if (fy > hill) { r = (int)(40 + 30 * sin(fx * 40)); g = (int)(110 + 40 * fy); b = 50; }
			double sx = 0.2 + 0.6 * fmod(t * 0.3, 1.0), sy = 0.3;
			if ((fx - sx) * (fx - sx) * 4 + (fy - sy) * (fy - sy) < 0.004) { r = 250; g = 220; b = 90; }
			if (fx > 0.7 && fx < 0.85 && fy > 0.25) { r = g = b = ((int)(fx * 400) % 6 < 3) ? 200 : 90; }
			p[0] = (uint8_t)clamp8(r + nz[0]); p[1] = (uint8_t)clamp8(g + nz[1]); p[2] = (uint8_t)clamp8(b + nz[2]);
		}
	}
}

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
					img[((size_t)y * w + x) * 3 + c] = (uint8_t)clamp8(v);
				}
	}
}

static uint64_t fnv(uint64_t h, const void *d, size_t n)
{
	const uint8_t *p = d;
	for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 1099511628211ull; }
	return h;
}

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

int main(int argc, char **argv)
{
	int quick = argc > 1 && !strcmp(argv[1], "quick");
	const int W = 792, H = 525;
	uint8_t *img = malloc((size_t)W * H * 3), *rec = malloc((size_t)W * H * 3), *dec = malloc((size_t)W * H * 3);
	uint8_t *out = malloc(1 << 21);
	double tenc = 0; int nenc = 0, bad = 0;
	int nimg = quick ? 8 : 9 + 6 + 30;
	int rates[] = { 128, 144, 160, 96, 192 };
	for (int im = 0; im < nimg; im++) {
		if (quick) make_scene(img, W, H, im * 300, (int[]){ 0, 2, 3, 5, 8, 3, 2, 4 }[im]);
		else if (im < 9) make_image(im, img, W, H, im + 1);
		else if (im < 15) make_scene(img, W, H, (im - 9) * 1000, (int[]){ 0, 1, 2, 3, 5, 8 }[im - 9]);
		else make_collage(img, W, H, (unsigned)(im * 31 + 5));
		for (int ri = 0; ri < (quick ? 3 : 5); ri++)
			for (int ich = 0; ich < (quick ? 1 : 2); ich++) {
				dsc_pps p; char err[200];
				if (dsc_pps_init(&p, W, H, 263, rates[ri], err, sizeof err)) { printf("pps: %s\n", err); return 1; }
				dsc_enc_opts o = { ich };
				dsc_stats st; memset(&st, 0, sizeof st);
				size_t ob = dsc_picture_bytes(&p);
				memset(out, 0xAA, ob); memset(rec, 0x55, (size_t)W * H * 3);
				double t0 = now();
				int r = dsc_encode_picture(&p, &o, img, (size_t)W * 3, out, rec, &st);
				tenc += now() - t0; nenc++;
				int d = dsc_decode_picture(&p, out, ob, dec);
				if (d || memcmp(dec, rec, (size_t)W * H * 3)) bad++;
				uint64_t h = 1469598103934665603ull;
				h = fnv(h, out, ob); h = fnv(h, rec, (size_t)W * H * 3); h = fnv(h, &r, sizeof r);
				h = fnv(h, &st, sizeof st);
				printf("img %2d rate %3d ich_off %d ret %d hash %016llx\n", im, rates[ri], ich, r, (unsigned long long)h);
			}
	}
	fprintf(stderr, "%d encodes, %.2f ms/frame, decode mismatches %d\n", nenc, 1000 * tenc / nenc, bad);
	return 0;
}
