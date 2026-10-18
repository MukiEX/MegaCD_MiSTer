// Verilator testbench for ld_video.sv + ld_mix.sv (the LaserActive picture
// mixed over a model of the VDP's output).
//
//   tb_ld_mix FILE.dscmmi [h32] [border] [hoff] [voff] [vframes]
//
// A core reset hits in the middle of a decode at VDP frame 30.
// The "HPS" commits consecutive disc frames at 29.97 Hz with jitter (and a
// still in the middle); the DDR3 model answers with random wait states. The
// VDP model produces the core's timing (H40: 420 slots, 3420 clocks a line;
// H32: 342 slots of 10 clocks; 262 lines with V counter 0-0xEA, 0x1E5-0x1FF)
// and pseudo-random pixels. Every output pixel is checked against a model
// of the mixer working from dsc.c's decode of the frame on screen, and the
// field logic, the 480i signals and the picture cadence are checked too.
#include "Vld_tb_top.h"
#include "verilated.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <vector>

extern "C" {
#include "../../src/dsc.h"
#include "../../src/dscv.h"
#include "../../src/mmi.h"
#include "../../src/util.h"
}

static Vld_tb_top *top;
static unsigned rng = 4242;
static unsigned rnd() { rng = rng * 1103515245u + 12345u; return rng >> 8; }

static const uint64_t MEM_BASE = 0x30000000ull;
static std::vector<uint64_t> mem(0x1000000 / 8);
static uint64_t &word(uint64_t byte_addr)
{
	uint64_t i = (byte_addr - MEM_BASE) / 8;
	if (byte_addr < MEM_BASE || i >= mem.size()) { printf("DDR access out of range: %llx\n", (unsigned long long)byte_addr); exit(1); }
	return mem[i];
}
struct ReadReq { uint64_t addr; int n; uint64_t due; };
static std::deque<ReadReq> rq;
static uint64_t cyc;
static int wr_left = 0;
static uint64_t wr_addr = 0;

static uint32_t hash3(uint32_t a, uint32_t b, uint32_t c)
{
	uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA77u ^ (c + 0x165667B1u) * 0xC2B2AE3Du;
	h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
	return h;
}

// --------------------------------------------------------- mixer model
static uint8_t lerp(uint8_t a, uint8_t b, int t)
{
	int d = ((int)b - (int)a) * t;          // fits 16 bits signed
	int v = a + ((d >> 6) & 0xFF) + ((d >> 5) & 1);
	return (uint8_t)v;
}
// the line buffer keeps 7-7-6 bits (top bits repeated below)
static uint8_t q20(uint8_t v, int c)
{
	if (c < 2) return (v & 0xFE) | (v >> 7);
	return (v & 0xFC) | (v >> 6);
}
static int weight(int f) { return (f * 65 + 8) >> 4; }
static uint8_t blend(uint8_t v, uint8_t l, int w) { return (uint8_t)((v * w + l * (256 - w) + 128) >> 8); }

struct Expect { bool blank; uint8_t r, g, b; };

int main(int argc, char **argv)
{
	if (argc < 2) { printf("usage: tb_ld_mix FILE.dscmmi [h32] [border] [hoff] [voff] [vframes]\n"); return 2; }
	bool h32 = argc > 2 && atoi(argv[2]);
	bool border = argc > 3 && atoi(argv[3]);
	int hoff = argc > 4 ? atoi(argv[4]) : 0;
	int voff = argc > 5 ? atoi(argv[5]) : 0;
	int vframes = argc > 6 ? atoi(argv[6]) : 210;

	zip_file z;
	char err[256];
	if (zip_open(&z, argv[1], err, sizeof err)) { printf("%s\n", err); return 1; }
	const zip_entry *e = nullptr;
	for (int i = 0; i < z.count; i++) {
		size_t l = strlen(z.entries[i].name);
		if (l > 5 && !strcmp(z.entries[i].name + l - 5, ".dscv")) { e = &z.entries[i]; break; }
	}
	if (!e) { printf("no .dscv\n"); return 1; }
	uint8_t hb[DSCV_HEADER_SIZE];
	read_full_at(z.fd, hb, sizeof hb, e->data_off);
	dscv_header h;
	dscv_unpack(&h, hb);
	dsc_pps p;
	if (dsc_pps_unpack(&p, h.pps, err, sizeof err)) { printf("%s\n", err); return 1; }
	int W = p.pic_width, H = p.pic_height;
	uint32_t words = (h.frame_bytes + 7) / 8;
	printf("%s: %dx%d, field 1 at row %u; %s%s, hoff %d/256, voff %d\n", argv[1], W, H, h.field1_row,
	       h32 ? "H32" : "H40", border ? " with border" : "", hoff, voff);

	std::map<uint32_t, std::vector<uint8_t>> ref;     // disc frame -> decoded picture
	auto frame_data = [&](uint32_t f, std::vector<uint8_t> &buf) {
		buf.assign(words * 8, 0);
		read_full_at(z.fd, buf.data(), h.frame_bytes, e->data_off + h.frames_offset + (uint64_t)f * h.frame_stride);
		if (!ref.count(f)) {
			ref[f].resize((size_t)W * H * 3);
			if (dsc_decode_picture(&p, buf.data(), h.frame_bytes, ref[f].data())) { printf("ref decode failed\n"); exit(1); }
		}
	};

	Verilated::commandArgs(argc, argv);
	top = new Vld_tb_top;

	// ------------------------------------------------------- VDP model
	// slot lengths of one line
	std::vector<int> slot_len;
	int h_disp_start = h32 ? 466 : 458, hblank_end = h32 ? 4 : 11, hint_pos = h32 ? 265 : 329;
	int disp_w = h32 ? 256 : 320, nslots = h32 ? 342 : 420;
	for (int i = 0; i < nslots; i++) {
		int hc = (h_disp_start + i) & 511, len = h32 ? 10 : 8;
		if (!h32 && hc >= 460 && hc <= 493) {
			int k = hc - 460;                    // 8aaaaaaa99aaaaaaa8aaaaaaa99aaaaaaa
			static const char *pat = "8aaaaaaa99aaaaaaa8aaaaaaa99aaaaaaa";
			len = pat[k] == 'a' ? 10 : pat[k] - '0';
		}
		slot_len.push_back(len);
	}
	std::vector<int> vseq;
	for (int v = 0; v <= 0xEA; v++) vseq.push_back(v);
	for (int v = 0x1E5; v <= 0x1FF; v++) vseq.push_back(v);
	auto tsl = [](int v) { return v >= 0x1E5 ? v - 0x1E5 + 3 : v >= 0xE8 ? v - 0xE8 : v + 30; };

	int slot = 0, slot_left = 0, line = 0, vframe = 0;
	int vcnt = vseq[0];

	uint32_t regs = 0;
	uint32_t regs_next = 0;
	auto set_regs = [&](int r01, int r0C, int r1A, int r1B) { regs_next = (uint32_t)r01 << 24 | (uint32_t)r0C << 16 | (uint32_t)r1A << 8 | (uint32_t)r1B; };
	set_regs(0x41, 0x8A, 0x80, 0x00);
	regs = regs_next;

	// --------------------------------------------------- DDR + clock
	auto tick = [&]() {
		top->clk = 0;
		top->eval();
		bool busy = (rnd() % 5) == 0;
		top->DDRAM_BUSY = busy;
		top->eval();
		bool rd = top->DDRAM_RD && !busy, we = top->DDRAM_WE && !busy;
		uint64_t addr = (uint64_t)top->DDRAM_ADDR * 8;
		int bc = top->DDRAM_BURSTCNT;
		if (rd) {
			if (!bc) { printf("read with burst 0\n"); exit(1); }
			rq.push_back({ addr, bc, cyc + 8 + rnd() % 40 });
		}
		if (we) {
			if (wr_left == 0) { wr_addr = addr; wr_left = bc; }
			word(wr_addr) = top->DDRAM_DIN;
			wr_addr += 8;
			wr_left--;
		}
		top->DDRAM_DOUT_READY = 0;
		if (!rq.empty() && rq.front().due <= cyc && (rnd() % 4) != 0) {
			ReadReq &r = rq.front();
			top->DDRAM_DOUT = word(r.addr);
			top->DDRAM_DOUT_READY = 1;
			r.addr += 8;
			if (--r.n == 0) rq.pop_front();
		}
		top->clk = 1;
		top->eval();
		cyc++;
	};

	top->reset = 1;
	for (int i = 0; i < 4; i++) tick();
	top->reset = 0;
	for (int i = 0; i < DSC_PPS_SIZE; i++) { top->pps_we = 1; top->pps_addr = i; top->pps_data = h.pps[i]; tick(); }
	top->pps_we = 0;
	top->frame_words = words;
	top->field1_row = h.field1_row;
	top->ctl_restart = 1; tick(); top->ctl_restart = 0;
	top->ctl_enable = 1;
	top->ctl_visible = 1;
	top->show = 1;
	top->force_il = 0;
	top->border = border;
	top->hoff = (int16_t)hoff;
	top->voff = (int8_t)voff;

	// ---------------------------------------------------------- state
	const double LD_PERIOD = 1791563.0;                 // 29.97 Hz
	uint64_t next_commit = 200000;
	uint32_t next_frame = 0;
	int committed = 0;
	uint32_t slot_frame[16];
	uint64_t commit_time[100000];
	int buf_frame[3] = { -1, -1, -1 };
	int disp_frame = -1;
	bool tb_tog = false, tb_even = false;
	int tb_il = 0;

	struct LineState { int frame; bool even; bool ok; };
	LineState fetched[512];
	for (auto &l : fetched) l = { -1, false, false };
	LineState cur_line = { -1, false, false };
	int cur_tsl = 0;
	int sub = 0;                                        // output pixel within the line

	std::deque<Expect> expq;
	long checked = 0, bad = 0, mixed_px = 0;
	int prev_hbl = 1, prev_vbl = 1;
	int vf_since_new = 0;
	std::map<int, int> hold_hist;
	long lat_sum = 0, lat_n = 0, lat_max = 0;
	int il_bad = 0, field_bad = 0, field_checks = 0;
	int first_new_vf = -1;
	std::deque<int> latched_frames;
	int still_from = 120, still_len = 12;               // LD frames: a still in the middle

	int fails = 0;
	uint64_t latch_at = 0;
	bool did_reset = false;
	int reset_clks = 0;
	bool latch_seen_new = false;
	int latch_r01 = 0, latch_r0C = 0;
	bool latch_force = false;
	int forced_new = 0;
	while (vframe < vframes) {
		// HPS: commit consecutive frames on its own clock (jittered)
		top->commit_we = 0;
		if (cyc >= next_commit && top->queue_level < 4) {
			uint32_t f = next_frame % h.frame_count;
			std::vector<uint8_t> buf;
			frame_data(f, buf);
			int s = committed % 16;
			for (uint32_t i = 0; i < words; i++) {
				uint64_t v = 0;
				for (int b = 0; b < 8; b++) v |= (uint64_t)buf[i * 8 + b] << (8 * b);
				word(0x30100000ull + (uint64_t)s * 0x80000 + i * 8) = v;
			}
			slot_frame[s] = f;
			commit_time[f] = cyc;
			top->commit_we = 1;
			top->commit_slot = s;
			committed++;
			next_frame++;
			int jitter = (int)(rnd() % 540000) - 270000;    // +-5 ms
			double t = 200000 + next_frame * LD_PERIOD;
			if ((int)next_frame >= still_from) t += still_len * LD_PERIOD;
			next_commit = (uint64_t)(t + jitter);
		}

		// VDP: next slot
		top->ce_pix = 0;
		if (slot_left == 0) {
			int hc = (h_disp_start + slot) & 511;
			if (hc == hint_pos) {
				line = (line + 1) % (int)vseq.size();
				vcnt = vseq[line];
				if (vcnt == 0x1F0) {
					regs = regs_next;                           // (the game writes them in the blanking)
					// OSD "both fields": over the first-field-only program, and over field select
					top->force_il = (vframe >= 20 && vframe < 40) || (vframe >= 110 && vframe < 118);
				}
				if (line == 0) {
					vframe++;
					// register program, by VDP frame
					if (vframe == 50) set_regs(0x41, 0x80, 0x40, 0x10);       // interlaced
					if (vframe == 100) set_regs(0x81, 0x8B, 0x80, 0x00);      // field select: second field
					if (vframe == 140) set_regs(0xC1, 0x8A, 0xC0, 0x30);      // field select: first field
					if (vframe == 170) set_regs(0x01, 0x80, 0x00, 0x00);      // mixing off
					if (vframe == 190) set_regs(0x41, 0x8A, 0xFC, 0x00);      // VDP only over LD backdrop
				}
			}
			int x = hc - hblank_end - 13;                   // pixel x (negative in the left border)
			if (hc >= 458) x = -1000;
			bool hbl = border ? !(hc >= hblank_end && hc < hblank_end + 13 + disp_w + 14 - 3) : !(x >= 0 && x < disp_w);
			bool vbl = border ? (vcnt >= 232 && vcnt < 0x1F5) : !(vcnt < 224);
			uint32_t hv = hash3(vframe, vcnt, (uint32_t)(x + 2000));
			bool blank = hbl || vbl;
			top->ce_pix = 1;
			top->hbl = hbl;
			top->vbl = vbl;
			top->vdp_vcnt = vcnt;
			top->vdp_r = blank ? 0 : (hv & 0xFF);
			top->vdp_g = blank ? 0 : ((hv >> 8) & 0xFF);
			top->vdp_b = blank ? 0 : ((hv >> 16) & 0xFF);
			top->vdp_backdrop = blank ? 1 : ((hv >> 24) & 1);
			top->hs = (hc >= h_disp_start + 7 && hc < h_disp_start + 28);
			top->vs = (vcnt >= 0x1E5 && vcnt < 0x1E9);

			// model: line starts and ends, frame latches
			if (prev_hbl && !hbl) {
				cur_tsl = tsl(vcnt);
				cur_line = fetched[cur_tsl];
				sub = 0;
			}
			if (!prev_hbl && hbl) {
				int t = cur_tsl >= 260 ? cur_tsl - 260 : cur_tsl + 2;
				int fl = t + voff;
				int row = fl * 2 + (tb_even ? 1 : 0);
				fetched[t] = { disp_frame, tb_even, disp_frame >= 0 && fl >= 0 && row < H };
				if (getenv("TB_TRACE") && vframe == 2) printf("tb fetch t %d row %d\n", t, row);
			}
			bool latch = !prev_vbl && vbl;
			prev_hbl = hbl; prev_vbl = vbl;

			// expected output pixels for this slot
			int r01 = regs >> 24, r0C = (regs >> 16) & 0xFF, f1A = ((regs >> 8) & 0xFF) >> 2, f1B = (regs & 0xFF) >> 2;
			bool mixing = (r01 >> 6) != 0;
			for (int s2 = 0; s2 < 2; s2++) {
				Expect ex = { blank, (uint8_t)top->vdp_r, (uint8_t)top->vdp_g, (uint8_t)top->vdp_b };
				if (!blank && mixing) {
					int period = slot_len[(slot + nslots - 1) % nslots];   // the previous slot's length
					int start = period == 10 ? (border ? 349845 : 914024) : (border ? 590706 : 1042050);
					int step = period / 2 == 5 ? 21699 : 17359;
					int64_t pos = start + (int64_t)hoff * 64 + (int64_t)sub * step;
					pos &= (1 << 25) - 1;
					if (pos & (1 << 24)) pos -= (1 << 25);
					int col_raw = (int)((pos >> 14) & 0x3FF);
					int col = pos < 0 ? 0 : (col_raw > 790 ? 790 : col_raw);
					int frac = (pos < 0 || col_raw > 790) ? 0 : (int)((pos >> 8) & 63);
					uint8_t ld[3] = { 0, 0, 0 };
					if (cur_line.ok) {
						int fl = cur_tsl + voff;
						int src = cur_line.even ? (int)h.field1_row + fl : fl;
						const uint8_t *img = ref[cur_line.frame].data();
						for (int c = 0; c < 3; c++)
							ld[c] = lerp(q20(img[((size_t)src * W + col) * 3 + c], c), q20(img[((size_t)src * W + col + 1) * 3 + c], c), frac);
					}
					int w = weight((top->vdp_backdrop) ? f1B : f1A);
					ex.r = blend(ex.r, ld[0], w);
					ex.g = blend(ex.g, ld[1], w);
					ex.b = blend(ex.b, ld[2], w);
					mixed_px++;
				}
				if (!blank) sub++;
				expq.push_back(ex);
			}

			if (latch) { latch_at = cyc; latch_seen_new = false; latch_r01 = r01; latch_r0C = r0C; latch_force = top->force_il; }
			slot_left = slot_len[slot];
			slot = (slot + 1) % nslots;
		}
		slot_left--;
		top->regs = regs;
		// a core reset while a frame is being decoded: it is decoded again
		top->reset = 0;
		if (vframe == 30 && line == 100 && !did_reset) {
			if (top->queue_level != 0) { top->reset = 1; if (++reset_clks == 3) { did_reset = true; printf("     (core reset at VDP frame 30, queue %d)\n", top->queue_level); } }
		}
		tick();
		if (top->dbg_setup) { buf_frame[top->dbg_back] = slot_frame[top->dbg_slot]; if (getenv("TB_TRACE")) printf("vf %d line %d: decode frame %d into buf %d\n", vframe, line, slot_frame[top->dbg_slot], top->dbg_back); }
		if (top->dbg_lf_req && getenv("TB_TRACE") && vframe == 2) printf("lf_req row %d (cur tsl %d)\n", top->dbg_lf_row, cur_tsl);

		// frame latch: the DUT says within a few clocks whether a new picture went up
		if (latch_at) {
			if (top->dbg_latch_new) latch_seen_new = true;
			if (cyc == latch_at + 6) {
				bool isnew = latch_seen_new;
				int r01 = latch_r01, r0C = latch_r0C;
				bool mixing = (r01 >> 6) != 0;
				vf_since_new++;
				if (isnew) {
					int f = buf_frame[top->dbg_disp];
					if (getenv("TB_TRACE")) printf("vf %d: buf %d frame %d\n", vframe, top->dbg_disp, f);
					if (first_new_vf >= 0) hold_hist[vf_since_new]++;
					if (first_new_vf < 0) first_new_vf = vframe;
					vf_since_new = 0;
					if (f >= 0) {
						long lat = (long)(cyc - commit_time[f]);
						lat_sum += lat; lat_n++;
						if (lat > lat_max) lat_max = lat;
						if (!latched_frames.empty() && f != latched_frames.back() + 1) printf("     frame %d not shown (now %d)\n", latched_frames.back() + 1, f);
						latched_frames.push_back(f);
					}
					disp_frame = f;
				}
				bool force = latch_force;
				bool il = force || (!(r0C & 0x20) && !(r0C & 0x08) && !(r0C & 0x02));
				tb_tog = force ? !tb_tog : isnew ? false : (il ? !tb_tog : tb_tog);
				bool fs = !force && (r01 >> 6) >= 2 && ((r0C & 0x02) || (r0C & 0x08));
				tb_even = fs ? (r0C & 1) : tb_tog;
				if (force && isnew && tb_even) { field_bad++; if (field_bad < 5) printf("FAIL: both fields forced, but a new frame started on its second field (vframe %d)\n", vframe); }
				if (force && isnew) forced_new++;
				tb_il = il && mixing && disp_frame >= 0;
			}
			if (cyc == latch_at + 10) {
				latch_at = 0;
				field_checks++;
				if (top->f1 != tb_even) { field_bad++; if (field_bad < 5) printf("FAIL: field %d, expected %d (vframe %d)\n", top->f1, tb_even, vframe); }
				if (top->il != tb_il || top->vs_delay != (tb_il && !tb_even)) { il_bad++; if (il_bad < 5) printf("FAIL: il %d vs_delay %d, expected %d %d\n", top->il, top->vs_delay, tb_il, tb_il && !tb_even); }
			}
		}
		// output pixels
		if (top->ce_out) {
			if (expq.empty()) { printf("FAIL: output pixel with nothing expected\n"); fails++; break; }
			Expect ex = expq.front();
			expq.pop_front();
			if ((bool)(top->hbl_out || top->vbl_out) != ex.blank) {
				bad++;
				if (bad < 10) printf("FAIL: blank %d, expected %d (vframe %d)\n", top->hbl_out || top->vbl_out, ex.blank, vframe);
			} else if (!ex.blank) {
				checked++;
				if (top->r_out != ex.r || top->g_out != ex.g || top->b_out != ex.b) {
					bad++;
					if (bad < 10) printf("FAIL: vframe %d line %d px %d: %02x%02x%02x, expected %02x%02x%02x\n", vframe, line, sub,
					                     top->r_out, top->g_out, top->b_out, ex.r, ex.g, ex.b);
				}
			}
		}
	}

	printf("\nchecked %ld pixels (%ld mixed), %ld wrong; field checks %d (%d wrong), 480i signal errors %d\n",
	       checked, mixed_px, bad, field_checks, field_bad, il_bad);
	printf("new frames with both fields forced: %d (all on their first field unless reported)\n", forced_new);
	printf("pictures shown: %zu; VDP frames each:", latched_frames.size());
	for (auto &kv : hold_hist) printf(" %dx%d", kv.first, kv.second);
	printf("\ncommit to screen: avg %.1f ms, max %.1f ms; late %u, dropped %u, decode errors %u, decode %u x256 clocks\n",
	       lat_n ? lat_sum / (double)lat_n / 53693.175 : 0.0, lat_max / 53693.175, top->late_count, top->drop_count, top->error_count, top->decode_kclks);
	bool ok = !fails && bad == 0 && field_bad == 0 && il_bad == 0 && checked > 100000 && top->error_count == 0 && latched_frames.size() > 50;
	printf("%s\n", ok ? "PASS" : "FAIL");
	delete top;
	return ok ? 0 : 1;
}
