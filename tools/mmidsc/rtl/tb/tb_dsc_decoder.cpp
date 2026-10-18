// Verilator testbench: dsc_decoder.sv against the C reference (src/dsc.c).
//
//   tb_dsc_decoder                 synthetic pictures at 6, 8, 9, 10 and 12 bpp
//   tb_dsc_decoder FILE.dscmmi N   N frames spread across a .dscmmi video stream
//   tb_dsc_decoder --many N        N varied 792x525 frames at each of 8, 9 and 10 bpp
//
// Every frame is decoded by the RTL with random input stalls and random
// output back-pressure, and every pixel is compared with dsc_decode_picture().
// Frames are fed back to back: the next frame's words are offered as soon as
// the previous frame's last word is taken.
#include "Vdsc_decoder.h"
#include "verilated.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

extern "C" {
#include "../../src/dsc.h"
#include "../../src/dscv.h"
#include "../../src/mmi.h"
#include "../../src/util.h"
}

static Vdsc_decoder *top;
static uint64_t cycles;
static unsigned rng = 12345;
static unsigned rnd() { rng = rng * 1103515245u + 12345u; return rng >> 8; }

static void tick()
{
	top->clk = 0; top->eval();
	top->clk = 1; top->eval();
	cycles++;
}

struct Frame {
	uint8_t pps[DSC_PPS_SIZE];
	std::vector<uint8_t> bytes;
	std::vector<uint8_t> ref;  // expected RGB
	int w, h;
	std::string name;
};

// feeds words of a queue of frames; checks output of the current one
static int run_frames(std::vector<Frame> &frames, int stall_in, int stall_out)
{
	// all frames in a run must share a PPS (as within one .dscv stream)
	for (int i = 0; i < DSC_PPS_SIZE; i++) {
		top->pps_we = 1; top->pps_addr = i; top->pps_data = frames[0].pps[i];
		tick();
	}
	top->pps_we = 0;

	std::deque<uint64_t> words;
	for (auto &f : frames) {
		size_t n = (f.bytes.size() + 7) / 8;
		for (size_t k = 0; k < n; k++) {
			uint64_t w = 0;
			for (int b = 0; b < 8; b++) {
				size_t o = k * 8 + b;
				if (o < f.bytes.size()) w |= (uint64_t)f.bytes[o] << (8 * b);
			}
			words.push_back(w);
		}
	}

	int fails = 0;
	for (size_t fi = 0; fi < frames.size(); fi++) {
		Frame &f = frames[fi];
		std::vector<uint8_t> got((size_t)f.w * f.h * 3, 0);
		std::vector<uint8_t> seen((size_t)f.w * f.h, 0);
		top->start = 1;
		tick();
		top->start = 0;
		uint64_t c0 = cycles;
		long beats = 0;
		bool fin = false;
		while (!fin) {
			top->in_valid = !words.empty() && (stall_in == 0 || (rnd() % stall_in) != 0);
			top->in_data = words.empty() ? 0 : words.front();
			top->out_ready = stall_out == 0 || (rnd() % stall_out) != 0;
			top->clk = 0;
			top->eval();
			bool take = top->in_valid && top->in_ready;
			bool out = top->out_valid && top->out_ready;
			int ox = top->out_x, oy = top->out_y;
			uint32_t px[3] = { top->out_rgb0, top->out_rgb1, top->out_rgb2 };
			bool d = top->done;
			top->clk = 1;
			top->eval();
			cycles++;
			if (take) words.pop_front();
			if (out) {
				beats++;
				if (oy >= f.h || ox + 2 >= f.w) { printf("  bad output coordinate %d,%d\n", ox, oy); fails++; }
				else
					for (int i = 0; i < 3; i++) {
						size_t o = (size_t)oy * f.w + ox + i;
						seen[o]++;
						got[o * 3 + 0] = px[i] >> 16;
						got[o * 3 + 1] = px[i] >> 8;
						got[o * 3 + 2] = px[i];
					}
			}
			if (d) fin = true;
			if (cycles - c0 > 400000000ull) { printf("  TIMEOUT\n"); return fails + 1; }
		}
		// drain a last output beat
		for (int k = 0; k < 4 && top->out_valid; k++) {
			top->out_ready = 1;
			top->clk = 0; top->eval();
			if (top->out_valid) {
				int ox = top->out_x, oy = top->out_y;
				uint32_t px[3] = { top->out_rgb0, top->out_rgb1, top->out_rgb2 };
				for (int i = 0; i < 3; i++) {
					size_t o = (size_t)oy * f.w + ox + i;
					seen[o]++;
					got[o * 3 + 0] = px[i] >> 16; got[o * 3 + 1] = px[i] >> 8; got[o * 3 + 2] = px[i];
				}
				beats++;
			}
			top->clk = 1; top->eval(); cycles++;
		}
		long bad = 0, missing = 0, first = -1;
		for (size_t o = 0; o < seen.size(); o++) {
			if (seen[o] != 1) missing++;
			if (memcmp(&got[o * 3], &f.ref[o * 3], 3)) { if (first < 0) first = (long)o; bad++; }
		}
		bool ok = !bad && !missing && !top->error;
		if (!ok) fails++;
		double cpg = (double)(cycles - c0) / ((double)f.w * f.h / 3);
		printf("%s %-34s %ld groups, %.2f clocks/group%s", ok ? "ok  " : "FAIL", f.name.c_str(), beats, cpg,
		       stall_in || stall_out ? " (with stalls)" : "");
		if (!ok)
			printf("  error=%d mismatched px=%ld (first at x=%ld y=%ld) not-output-once=%ld", top->error, bad,
			       first < 0 ? -1 : first % f.w, first < 0 ? -1 : first / f.w, missing);
		printf("\n");
	}
	if (!words.empty()) { printf("  FAIL: %zu input words left over\n", words.size()); fails++; }
	return fails;
}

// ------------------------------------------------------------ pictures

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
			case 2:
				r = g = b = 40;
				if ((x / 24 + y / 16) % 5 == 0) { r = 200; g = 30; b = 90; }
				if (x % 50 < 2) { r = g = b = 250; }
				break;
			case 3: {
				int n = (int)(rnd() % 9) - 4, dx = x - w / 3, dy = y - h / 2;
				r = (x + seed * 3) % 256 + n; g = (y * 2 + seed) % 256 + n; b = 128 + (x - y) / 8 + n;
				if (dx * dx + dy * dy < 60 * 60) { r = 220 + n; g = 170 - dy / 3; b = 40; }
				break;
			}
			case 4:
				r = g = b = 16;
				if (((x / 3) ^ (y / 2)) % 7 == 0 && (y % 12) < 9) { r = 235; g = 235; b = 180; }
				break;
			case 5:
				r = g = b = 90;
				if (y >= h - h / 8) { r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; }
				break;
			case 6:
				if ((x + y) & 1) { r = 255; g = 0; b = 255; } else { r = 0; g = 255; b = 0; }
				break;
			default: {
				// collage of rectangles
				r = (x * 7 + y * 3) & 255; g = (x ^ y) & 255; b = (y * 5) & 255;
				int cx = (x / 37 + y / 23 + seed) % 4;
				if (cx == 0) { r = g = b = 60; }
				else if (cx == 1) { r = rnd() & 255; g = rnd() & 255; b = rnd() & 255; }
				else if (cx == 2) { r = 200; g = 200 - (x % 40); b = 10; }
			}
			}
			p[0] = (uint8_t)(r < 0 ? 0 : r > 255 ? 255 : r);
			p[1] = (uint8_t)(g < 0 ? 0 : g > 255 ? 255 : g);
			p[2] = (uint8_t)(b < 0 ? 0 : b > 255 ? 255 : b);
		}
}

static bool synth_frame(Frame &f, int w, int h, int sh, int bpp16, int kind, int seed, int no_ich)
{
	dsc_pps p;
	char err[200];
	if (dsc_pps_init(&p, w, h, sh, bpp16, err, sizeof err)) return false;
	dsc_pps_pack(&p, f.pps);
	std::vector<uint8_t> img((size_t)w * h * 3);
	make_image(kind, img.data(), w, h, seed);
	f.bytes.resize(dsc_picture_bytes(&p));
	f.ref.resize(img.size());
	dsc_enc_opts o;
	memset(&o, 0, sizeof o);
	o.disable_ich = no_ich;
	std::vector<uint8_t> rec(img.size());
	if (dsc_encode_picture(&p, &o, img.data(), (size_t)w * 3, f.bytes.data(), rec.data(), nullptr)) return false;
	if (dsc_decode_picture(&p, f.bytes.data(), f.bytes.size(), f.ref.data())) return false;
	f.w = w; f.h = h;
	char name[100];
	snprintf(name, sizeof name, "%dx%d/%d %gbpp kind%d%s", w, h, sh, bpp16 / 16.0, kind, no_ich ? " noich" : "");
	f.name = name;
	return true;
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	top = new Vdsc_decoder;
	top->reset = 1;
	tick(); tick();
	top->reset = 0;
	int fails = 0, runs = 0;

	if (argc >= 3 && !strcmp(argv[1], "--many")) {
		// N frames per rate (8, 9, 10 bpp) at 792x525, varied content, ICH on and off
		int n = atoi(argv[2]);
		for (int bpp16 : { 128, 144, 160 }) {
			std::vector<Frame> fr;
			for (int k = 0; k < n; k++) {
				Frame f;
				if (!synth_frame(f, 792, 525, 263, bpp16, k % 8, 1000 + k * 13 + bpp16, (k / 8) & 1)) {
					printf("encode failed\n");
					return 1;
				}
				fr.push_back(f);
			}
			fails += run_frames(fr, 4, 5);
			runs += n;
		}
	} else if (argc >= 3) {
		// frames from a .dscmmi
		zip_file z;
		char err[256];
		if (zip_open(&z, argv[1], err, sizeof err)) { printf("%s\n", err); return 1; }
		const zip_entry *e = nullptr;
		for (int i = 0; i < z.count; i++) {
			size_t l = strlen(z.entries[i].name);
			if (l > 5 && !strcmp(z.entries[i].name + l - 5, ".dscv")) { e = &z.entries[i]; break; }
		}
		if (!e) { printf("no .dscv entry\n"); return 1; }
		uint8_t hb[DSCV_HEADER_SIZE];
		read_full_at(z.fd, hb, sizeof hb, e->data_off);
		dscv_header h;
		if (dscv_unpack(&h, hb)) { printf("bad DSCV header\n"); return 1; }
		dsc_pps p;
		if (dsc_pps_unpack(&p, h.pps, err, sizeof err)) { printf("%s\n", err); return 1; }
		int n = atoi(argv[2]);
		std::vector<Frame> fr;
		for (int k = 0; k < n; k++) {
			uint32_t idx = (uint32_t)(((double)k + 0.5) * h.frame_count / n);
			Frame f;
			memcpy(f.pps, h.pps, DSC_PPS_SIZE);
			f.bytes.resize(h.frame_bytes);
			read_full_at(z.fd, f.bytes.data(), h.frame_bytes, e->data_off + h.frames_offset + (uint64_t)idx * h.frame_stride);
			f.w = p.pic_width; f.h = p.pic_height;
			f.ref.resize((size_t)f.w * f.h * 3);
			if (dsc_decode_picture(&p, f.bytes.data(), f.bytes.size(), f.ref.data())) { printf("ref decode failed\n"); return 1; }
			char name[64];
			snprintf(name, sizeof name, "frame %u (%gbpp)", idx, p.bits_per_pixel / 16.0);
			f.name = name;
			fr.push_back(f);
		}
		fails += run_frames(fr, 5, 7);
		runs += n;
	} else {
		int bpps[] = { 128, 144, 160, 96, 192 };
		for (int bpp16 : bpps) {
			// the .dscmmi geometry, three frames back to back, different content
			std::vector<Frame> fr;
			for (int k = 0; k < 3; k++) {
				Frame f;
				int kind = (bpp16 / 16 * 3 + k) % 8;
				if (!synth_frame(f, 792, 525, 263, bpp16, kind, k + bpp16, 0)) { printf("encode failed\n"); return 1; }
				fr.push_back(f);
			}
			fails += run_frames(fr, bpp16 == 128 ? 0 : 4, bpp16 == 128 ? 0 : 6);
			runs += 3;
		}
		// every content type at 8, 9 and 10 bpp on a smaller picture with 3 slices, ICH on and off
		for (int bpp16 : { 128, 144, 160 })
			for (int kind = 0; kind < 8; kind++)
				for (int ni = 0; ni < 2; ni++) {
					std::vector<Frame> fr(1);
					if (!synth_frame(fr[0], 288, 160, 56, bpp16, kind, kind * 7 + ni, ni)) {
						printf("encode failed 288x160 bpp %d kind %d\n", bpp16, kind);
						return 1;
					}
					fails += run_frames(fr, 3, 3);
					runs++;
				}
	}
	printf("\n%d frames, %d failures, %llu clocks\n", runs, fails, (unsigned long long)cycles);
	delete top;
	return fails != 0;
}
