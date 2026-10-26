// Host test harness for the Main_MiSTer LaserActive player (megald*.cpp and
// megacdd.cpp's LaserDisc mode), against a C++ model of the core side:
// the PD6103A block and hps_ext commands (as rtl/MCD/PD6103A.vhd and
// rtl/hps_ext.v), the ld_video commit queue (rtl/LD/ld_video.sv), and the
// CD audio FIFO that paces the drive. Time is simulated in 1 ms steps.
//
//   harness DISC.dscmmi
//
// The scripted sub-CPU does what games do through the PD6103A: spin up,
// seek to a frame and play, set a stop point, step frames, freeze/unfreeze
// the input block, and so on, and checks the output registers, the video
// frames that reach the screen, and the audio sectors.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/stat.h>
#include <math.h>
#include <deque>
#include <vector>
#include <map>
#include <functional>
#include <string>

#include "file_io.h"
#include "support/megacd/megacd.h"
#include "support/megacd/megald.h"
#include "support/megacd/megald_mmi.h"
#include "support/megacd/megald_video.h"

// ------------------------------------------------------------- sim clock
static unsigned long now_ms = 1000;
unsigned long GetTimer(unsigned long offset) { return now_ms + offset; }
unsigned long CheckTimer(unsigned long t) { return (long)(now_ms - t) >= 0; }

// ------------------------------------------------------------- misc stubs
const char *getRootDir() { return "harness_root"; }
const char *getFullPath(const char *name) { static char p[1100]; snprintf(p, sizeof p, "%s/%s", getRootDir(), name); return p; }
void Info(const char *message, int, int, int, int) { printf("INFO: %s\n", message); }
static std::vector<uint8_t> ddr(16 * 0x80000);
void *shmem_map(uint32_t address, uint32_t size) { if (address != 0x30100000 || size > ddr.size()) abort(); return ddr.data(); }
chd_error mister_chd_read_sector(chd_file *, int, uint32_t, uint32_t, int, uint8_t *, uint8_t *, int *) { return CHDERR_NONE; }
chd_error mister_load_chd(const char *, toc_t *) { return CHDERR_FILE_NOT_FOUND; }
void chd_close(chd_file *) {}
const char *chd_error_string(chd_error) { return "chd"; }

// --- file_io subset on stdio
fileTYPE::fileTYPE() { filp = 0; mode = 0; type = 0; zip = 0; size = 0; offset = 0; base = 0; }
fileTYPE::~fileTYPE() { FileClose(this); }
int fileTYPE::opened() { return filp != 0; }
int FileClose(fileTYPE *f) { if (f->filp) fclose(f->filp); f->filp = 0; return 1; }
int FileOpenEx(fileTYPE *f, const char *name, int, char, int)
{
	FileClose(f);
	f->filp = fopen(name, "rb");
	if (!f->filp) return 0;
	fseeko(f->filp, 0, SEEK_END);
	f->size = ftello(f->filp);
	fseeko(f->filp, 0, SEEK_SET);
	f->base = 0;
	return 1;
}
int FileOpen(fileTYPE *f, const char *name, char mute) { return FileOpenEx(f, name, 0, mute, 0); }
int FileOpenSub(fileTYPE *f, const char *name, __off64_t base, __off64_t size, char)
{
	if (!FileOpenEx(f, name, 0, 0, 0)) return 0;
	if (base + size > f->size) { FileClose(f); return 0; }
	f->base = base; f->size = size;
	return FileSeek(f, 0, SEEK_SET);
}
int FileSeek(fileTYPE *f, __off64_t off, int origin)
{
	if (!f->filp) return 0;
	if (origin == SEEK_SET) off += f->base;
	if (fseeko(f->filp, off, origin) < 0) return 0;
	f->offset = ftello(f->filp) - f->base;
	return 1;
}
int FileReadAdv(fileTYPE *f, void *buf, int len, int failres)
{
	if (!f->filp) return failres;
	int r = (int)fread(buf, 1, len, f->filp);
	f->offset += r;
	return r;
}
int FileLoad(const char *name, void *buf, int size)
{
	FILE *fp = fopen(name, "rb");
	if (!fp) return 0;
	int r = (int)fread(buf, 1, size, fp);
	fclose(fp);
	return r;
}

// ------------------------------------------------------------ core model
static int fails = 0;
static void FAIL(const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	printf("FAIL @%lu: ", now_ms); vprintf(fmt, ap); printf("\n");
	va_end(ap);
	fails++;
}

// PD6103A (rtl/MCD/PD6103A.vhd)
struct pd6103a
{
	uint8_t in_regs[32] = {}, in_applied[32] = {}, out_live[32] = {}, out_snap[32] = {}, hold_data[32] = {};
	int hold_cnt[32] = {};
	bool in_frozen = false, frozen = false, pend = false;
	bool log_reads = false, log_cpu = false;
	uint32_t toc[256] = {};
	std::deque<uint16_t> log;
	bool ovf = false;

	pd6103a() { reset(); }
	void reset()
	{
		*this = pd6103a(0);
		for (int r = 0x1A; r <= 0x1D; r++) in_regs[r] = in_applied[r] = 0xFF;
		out_live[1] = 0x80; out_live[5] = 0x7F;
	}
	explicit pd6103a(int) {}
	uint8_t drv(int r)
	{
		if (r == 0) return 0x80 | (out_live[0] & 0x3E) | (in_applied[0] & 1);
		if (r == 6) return out_live[6] | (pend ? 0x80 : 0);
		if (r >= 0x11 && r <= 0x14 && in_applied[5] != 0x00 && in_applied[5] != 0xFF) return (uint8_t)(toc[in_applied[5]] >> (8 * (0x14 - r)));
		if (r >= 0x0A && r <= 0x0F) return in_applied[r - 4];
		if (r == 0x10) return in_applied[5];
		return out_live[r];
	}
	void push(uint16_t e) { if (log.size() >= 512) ovf = true; else log.push_back(e); }
	void write_in(int r, uint8_t d)
	{
		if (!(r == 0 && (d & 0x81) == (in_regs[0] & 0x81))) push((r << 9) | d);
		in_regs[r] = d;
		bool chg = false;
		if (r == 0)
		{
			if (d & 0x80)
			{
				if ((d & 1) != (in_applied[0] & 1)) chg = true;
				if (in_frozen)
				{
					if ((in_regs[2] & 0x3F) != (in_applied[2] & 0x3F)) chg = true;
					for (int i = 6; i <= 11; i++) if (in_regs[i] != in_applied[i]) chg = true;
				}
			}
		}
		else if (!in_frozen)
		{
			if (r == 2 && (d & 0x3F) != (in_applied[2] & 0x3F)) chg = true;
			if (r >= 6 && r <= 11 && d != in_applied[r]) chg = true;
		}
		if (chg) pend = true;
		if (r == 0)
		{
			if ((d & 0x40) && !frozen) for (int i = 0; i < 32; i++) out_snap[i] = drv(i);
			frozen = d & 0x40;
			bool was = in_frozen;
			in_frozen = !(d & 0x80);
			if (d & 0x80)
			{
				if (was) for (int i = 1; i < 32; i++) in_applied[i] = in_regs[i];
				in_applied[0] = d;
			}
		}
		else if (!in_frozen || r >= 0x19) in_applied[r] = d;
	}
	uint8_t read_out(int r)
	{
		uint8_t v;
		if (hold_cnt[r]) { hold_cnt[r]--; v = hold_data[r]; }
		else if (frozen) v = out_snap[r];
		else v = drv(r);
		// reads are logged only while the log is under half full
		if (log_reads && log.size() < 256) log.push_back(0xC000 | (r << 9) | v);
		return v;
	}
	// the CPU polls all the time; on the real chip reads land between Main's
	// drain and its ack
	void poll_during_spi() { if (log_reads) read_out(6); }
	void ga_reset()
	{
		for (int i = 0; i < 12; i++) in_regs[i] = in_applied[i] = 0;
		memset(out_live, 0, 32); memset(out_snap, 0, 32); memset(hold_cnt, 0, sizeof hold_cnt);
		out_live[1] = 0x80; out_live[5] = 0x7F;
		in_frozen = frozen = pend = false;
		push(0x7F5A);
	}
} pd;

// ld_video (rtl/LD/ld_video.sv): decodes each commit at once (27 ms) into a
// spare buffer; at each VDP frame (59.92 Hz) a finished picture goes up if
// the last one has been up for two VDP frames (or at once when late)
struct video_model
{
	uint8_t pps[128];
	bool enable = false, visible = false;
	int pos_x = 0, pos_y = 0;
	std::deque<int> q;
	double next_vframe = 0;
	bool decoding = false, fresh = false;
	int ready_frame = -1;
	double decode_end = 0;
	int since = 7;
	uint16_t rd = 0, late = 0, drops = 0;
	bool shown_one = false;
	int shown_frame = -1;            // stream frame on screen
	std::vector<std::string> heads;  // first 64 bytes of each stream frame, to identify slots
	std::vector<int> history;        // frames shown, in order
	std::vector<unsigned long> history_t;
	void restart() { q.clear(); decoding = fresh = false; since = 7; rd = late = drops = 0; shown_one = false; shown_frame = -1; }
	int identify(int slot)
	{
		std::string h((const char *)&ddr[(size_t)slot * 0x80000], 64);
		for (size_t i = 0; i < heads.size(); i++) if (heads[i] == h) return (int)i;
		return -2;
	}
	void step(double t)
	{
		if (decoding && t >= decode_end)
		{
			decoding = false;
			if (fresh) drops++;
			ready_frame = identify(q.front());
			fresh = true;
			q.pop_front();
			rd++;
		}
		while (!decoding && enable && q.size() >= 3) { q.pop_front(); rd++; drops++; }
		if (!decoding && enable && !q.empty())
		{
			decoding = true;
			decode_end = t + 27.2;
		}
		if (t >= next_vframe)
		{
			next_vframe += 1000.0 / 59.92;
			if (fresh && since >= 2)
			{
				if (since == 3 && shown_one) late++;
				fresh = false;
				since = 1;
				shown_one = true;
				shown_frame = ready_frame;
				history.push_back(shown_frame);
				history_t.push_back(now_ms);
			}
			else if (since < 7) since++;
		}
	}
} vid;

// hps_ext transaction state
static int cur_cmd = -1, byte_cnt = 0, op = 0, toc_w = 0, toc_i = 0;
static uint32_t toc_d = 0;
uint16_t spi_uio_cmd_cont(uint16_t cmd)
{
	cur_cmd = cmd; byte_cnt = 0;
	switch (cmd)
	{
	case 0x50: return (pd.ovf ? 0x8000 : 0) | (pd.pend ? 0x4000 : 0) | (uint16_t)pd.log.size();
	case 0x53: return 0x4C44;
	default: return 0;
	}
}
void DisableIO() { cur_cmd = -1; }
uint16_t fpga_spi(uint16_t w)
{
	byte_cnt++;
	switch (cur_cmd)
	{
	case 0x50:
	{
		if (pd.log.empty()) return 0;
		uint16_t e = pd.log.front(); pd.log.pop_front(); return e;
	}
	case 0x51:
		if (byte_cnt == 1) { op = w & 3; toc_w = 0; return 0; }
		if (op == 3)
		{
			if (toc_w == 0) { toc_i = w >> 8; toc_d = (uint32_t)(w & 0xFF) << 24; }
			else if (toc_w == 1) toc_d |= (uint32_t)w << 8;
			else { toc_d |= w >> 8; pd.toc[toc_i] = toc_d; }
			toc_w = (toc_w + 1) % 3;
			return 0;
		}
		if (op == 0) { if (w & 2) pd.ovf = false; pd.log_reads = w & 1; pd.log_cpu = w & 4; }
		else if (op == 1) { int r = (w >> 8) & 31; pd.out_live[r] = w & 0xFF; }
		else if (op == 2) { pd.poll_during_spi(); if (pd.log.empty()) pd.pend = false; }
		return 0;
	case 0x53:
		switch (byte_cnt)
		{
		case 1: return vid.rd;
		case 2: return vid.late;
		case 3: return 0;
		case 4: return 5700;
		case 5: return (uint16_t)((std::min<size_t>(vid.q.size(), 7) << 1) | (vid.shown_one ? 1 : 0));
		case 6: return vid.drops;
		case 7: return 4;
		}
		return 0;
	case 0x52:
		if (byte_cnt == 1) { op = w & 7; return 0; }
		if (op == 4) { if (byte_cnt == 2) vid.pos_x = (int16_t)w; else if (byte_cnt == 3) vid.pos_y = (int8_t)w; return 0; }
		if (op == 0) { vid.enable = w & 1; if (w & 2) vid.restart(); vid.visible = (w >> 2) & 1; }
		else if (op == 1) vid.pps[(w >> 8) & 127] = w & 0xFF;
		else if (op == 3) { if (vid.q.size() < 4) vid.q.push_back(w & 15); else FAIL("commit into a full queue"); }
		return 0;
	}
	return 0;
}

// CD audio FIFO (rtl/MCD/CDDA_FIFO.v: 1280 samples, ready with room for a sector)
static double fifo = 0;
static std::vector<int> audio_marks;     // first left sample of each sector sent
static std::vector<int> audio_right;
static std::vector<int> data_lbas;
static std::map<int, uint32_t> data_crcs;   // what the core was given, by LBA
static uint32_t crc2048(const uint8_t *p)
{
	uint32_t c = 0xFFFFFFFFu;
	for (int i = 0; i < 2048; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1))); }
	return ~c;
}
static int can_send(uint8_t type) { return type ? 1 : fifo <= 1280 - 588; }
static int send_data(uint8_t *buf, int len, uint8_t index)
{
	if (index == MCD_CDDA_IO_INDEX)
	{
		fifo += len / 4;
		if (fifo > 1280) FAIL("CDDA FIFO overflow");
		int16_t *s = (int16_t *)buf;
		audio_marks.push_back(s[0]);
		audio_right.push_back(s[1]);
	}
	else if (index == MCD_DATA_IO_INDEX)
	{
		int m = (buf[12] >> 4) * 10 + (buf[12] & 15), s = (buf[13] >> 4) * 10 + (buf[13] & 15), f = (buf[14] >> 4) * 10 + (buf[14] & 15);
		data_lbas.push_back((m * 60 + s) * 75 + f - 150);
		data_crcs[(m * 60 + s) * 75 + f - 150] = crc2048(buf + 16);
	}
	return 1;
}

// mcd_poll's drive timing
static void step_ms()
{
	static unsigned long cdd_timer = 0;
	static int adj = 0;
	now_ms++;
	fifo -= 44.1;
	if (fifo < 0) fifo = 0;
	vid.step((double)now_ms);
	if (!cdd_timer || CheckTimer(cdd_timer))
	{
		if ((!cdd.isData || cdd.ld_mode) && cdd.status == CD_STAT_PLAY && cdd.latency == 0) { cdd_timer = GetTimer(10); adj = 0; }
		else { cdd_timer = GetTimer(13 + (!adj ? 1 : 0)); if (++adj >= 3) adj = 0; }
		cdd.Update();
	}
	static unsigned long topup_timer = 0;
	if (cdd.ld_mode && (!topup_timer || CheckTimer(topup_timer))) { topup_timer = GetTimer(2); cdd.LDTopUp(); }
	ld_poll();
	ld_video_poll();
	usleep(30);   // let the reader thread keep up with simulated time
}
static void run(int ms) { for (int i = 0; i < ms; i++) step_ms(); }

// ------------------------------------------------------------- the "CPU"
static void w(int r, uint8_t d) { pd.write_in(r, d); }
static uint8_t r(int reg) { return pd.read_out(reg); }
static int bcd(uint8_t v) { return (v >> 4) * 10 + (v & 15); }
static int frame_no() { return bcd(r(0x16)) * 10000 + bcd(r(0x17)) * 100 + bcd(r(0x18)); }

static bool wait_until(const char *what, std::function<bool()> cond, int max_ms)
{
	for (int i = 0; i < max_ms; i++)
	{
		if (cond()) return true;
		step_ms();
	}
	FAIL("timeout waiting for %s", what);
	return false;
}

static void check(const char *what, int got, int expect)
{
	if (got != expect) FAIL("%s: got %02X, expected %02X", what, got, expect);
	else printf("ok   %s = %02X\n", what, got);
}

// stream index of player frame number n (CAV, 1-based): lead-in frames come first
static int stream_of(int n) { return 20 + n - 1; }

int main(int argc, char **argv)
{
	if (argc < 2) { printf("usage: harness DISC.dscmmi\n"); return 2; }
	system("mkdir -p harness_root");  // laseractive.log goes here
	{
		// log every data sector until the picture-position test rewrites the file
		FILE *fp = fopen("harness_root/laseractive.cfg", "w");
		fprintf(fp, "log_sectors 1\n");
		fclose(fp);
	}

	// reference frame heads
	{
		ld_disc d;
		char err[128];
		if (!mmi_open(&d, argv[1], 0, err, sizeof err)) { printf("%s\n", err); return 1; }
		for (uint32_t i = 0; i < d.frame_count; i++)
		{
			char h[64];
			mmi_read(&d, h, 64, d.frames_off + (uint64_t)i * d.frame_stride);
			vid.heads.push_back(std::string(h, 64));
		}
		mmi_close(&d);
	}

	cdd.SendData = send_data;
	cdd.CanSendData = can_send;
	ld_reset();
	run(20);

	// ---- mount (mcd_set_image)
	cdd.Unload();
	if (cdd.Load(argv[1]) <= 0) { printf("load failed\n"); return 1; }
	cdd.status = cdd.loaded ? CD_STAT_STOP : CD_STAT_NO_DISC;
	cdd.latency = 10;
	printf("mounted: ld_mode %d, tracks %d, end %d\n", cdd.ld_mode, cdd.TrackCount(), cdd.DiscEnd());
	if (!cdd.ld_mode || cdd.TrackCount() != 2) FAIL("mount");
	run(50);

	// ---- power-on state as the BIOS sees it
	check("out 01", r(0x01), 0x80);
	check("out 05", r(0x05), 0x7F);
	check("out 02 (LD loaded, not spinning)", r(0x02), 0x80);
	wait_until("drive state 2", [] { return (r(0x06) & 0x8F) == 0x02; }, 100);
	check("out 15 (track, spun down)", r(0x15), 0x01);

	// ---- TOC reads answer at once, as the BIOS expects (reads within microseconds)
	w(0x00, 0x80);
	w(0x05, 0xA0);
	check("TOC A0: first track", r(0x12), 0x01);
	check("TOC A0: last track", r(0x13), 0x02);
	w(0x05, 0xA1);
	check("TOC A1: lead-out minutes", r(0x12), 0x00);
	check("TOC A1: lead-out seconds", r(0x13), 0x10);
	w(0x05, 0x02);
	check("TOC track 2: control/valid", r(0x11), 0x01);
	check("TOC track 2: seconds", r(0x13), 0x04);
	w(0x05, 0x01);
	check("TOC track 1: data track", r(0x11), 0x41);
	w(0x05, 0x00);
	run(5);

	// ---- spin up: drive state 4
	w(0x00, 0x80);
	w(0x02, 0x04);
	check("out 06 busy right after the command", r(0x06) & 0x80, 0x80);
	wait_until("drive state 4, not busy", [] { return (r(0x06) & 0x8F) == 0x04; }, 200);
	check("out 06 pause bit after load", r(0x06) & 0x10, 0x10);
	check("out 02 (CAV LD spun up)", r(0x02), 0xC5);
	check("out 03 (side A, disc present)", r(0x03), 0x48);
	check("out 08", r(0x08), 0x41);

	// ---- mixing on (analog video + audio), digital memory
	w(0x01, 0xC0);
	w(0x0C, 0x02);
	w(0x0F, 0x00);            // digital audio off, so the analog track can be checked

	// ---- seek to video frame 50 and play, set up in a frozen block
	w(0x00, 0x00);            // freeze inputs
	w(0x06, 0x06);            // absolute, video frame
	w(0x07, 0x00);
	w(0x08, 0x00);
	w(0x09, 0x00);
	w(0x0A, 0x50);
	w(0x00, 0x80);            // unfreeze
	check("out 0E echoes applied input 0A", r(0x0E), 0x50);
	w(0x02, 0x25);            // play, seek enabled
	check("out 06 busy after play+seek", r(0x06) & 0x80, 0x80);
	size_t hist0 = vid.history.size();
	unsigned long t0 = now_ms;
	wait_until("seek finished", [] { return (r(0x06) & 0xBF) == 0x05; }, 2000);
	printf("     seek reported done after %lu ms; frame number %d\n", now_ms - t0, frame_no());
	// "done" means the core has decoded the frame; it goes up at the next VDP frame
	for (int i = 0; i < 40 && vid.history.size() <= hist0; i++) step_ms();
	if (vid.history.size() <= hist0) FAIL("seek done, but no frame shown within 40 ms");
	else
	{
		int f = vid.history[hist0];
		printf("     first frame on screen after the seek: stream %d (frame no. %d)\n", f, f - 19);
		if (abs(f - stream_of(50)) > 1) FAIL("first frame after the seek is %d, expected ~%d", f, stream_of(50));
	}
	if (!vid.visible) FAIL("LD picture not visible while playing with mixing on");

	// ---- play for 2 s: frame number advances ~60, frames on screen follow, audio is the analog track
	int fn0 = frame_no();
	size_t h0 = vid.history.size(), a0 = audio_marks.size();
	run(2000);
	int fn1 = frame_no();
	printf("     2 s of play: frame %d -> %d; %zu frames shown, %u late, %u skipped\n", fn0, fn1, vid.history.size() - h0, vid.late, vid.drops);
	if (abs((fn1 - fn0) - 60) > 2) FAIL("frame number moved %d in 2 s", fn1 - fn0);
	if (vid.history.size() - h0 < 57) FAIL("only %zu frames shown in 2 s", vid.history.size() - h0);
	int seq_bad = 0;
	for (size_t i = h0 + 1; i < vid.history.size(); i++) if (vid.history[i] != vid.history[i - 1] + 1) seq_bad++;
	if (seq_bad > 1) FAIL("%d jumps in the frame sequence during play", seq_bad);
	int lag = (stream_of(frame_no())) - vid.shown_frame;
	printf("     on screen: stream %d; drive at frame no. %d (video lags by %d frames)\n", vid.shown_frame, frame_no(), lag);
	if (lag < 0 || lag > 4) FAIL("video %d frames behind the drive", lag);
	int abad = 0, sectors = (int)(audio_marks.size() - a0);
	for (size_t i = a0 + 1; i < audio_marks.size(); i++)
		if (audio_marks[i] != ((audio_marks[i - 1] + 8) & 0x1FF8) && audio_marks[i] != 0) abad++;
	printf("     %d audio sectors in 2 s (%d out of sequence), right channel %d\n", sectors, abad, audio_right.back());
	if (abs(sectors - 150) > 3) FAIL("audio sectors per 2 s: %d", sectors);
	if (abad > 0) FAIL("analog audio out of sequence");
	if (audio_right.back() != 0x2000 * 0 + 0x2000) FAIL("analog right channel %d", audio_right.back());

	// ---- stop point at frame 120
	w(0x06, 0x03);
	w(0x07, 0x00); w(0x08, 0x00); w(0x09, 0x01); w(0x0A, 0x20);
	run(5);
	check("out 1F stop point latched", r(0x1F), 0x01);
	check("out 1C stop frame low", r(0x1C), 0x20);
	wait_until("stop point", [] { return r(0x1F) == 0x00; }, 5000);
	run(100);
	check("out 09 after the stop point", r(0x09), 0x03);
	check("out 07 at a stop point", r(0x07), 0x20);
	printf("     stopped at frame no. %d, screen stream %d\n", frame_no(), vid.shown_frame);
	if (frame_no() != 120) FAIL("stopped at frame %d", frame_no());
	if (vid.shown_frame != stream_of(120)) FAIL("screen shows %d, expected %d", vid.shown_frame, stream_of(120));
	int fs = frame_no();
	run(500);
	if (frame_no() != fs) FAIL("moved while stopped");

	// ---- toggle input 0 bit 0 with seeking off: clears the stop, plays on (Pretty Illusion)
	w(0x02, 0x05);            // seek disabled, play
	w(0x00, 0x81);
	run(300);
	check("out 09 cleared by the toggle", r(0x09), 0x00);
	if (frame_no() <= fs) FAIL("did not resume after the toggle (frame %d)", frame_no());

	// ---- still frame stepping: mode 2 speed 1 (one frame per bit-0 toggle)
	w(0x03, 0x21);
	run(200);
	int st0 = frame_no();
	check("out 07 after the step", r(0x07), 0x20);
	w(0x00, 0x80);            // toggle bit 0: one more step
	run(300);
	int st1 = frame_no();
	printf("     step: frame %d -> %d, screen %d\n", st0, st1, vid.shown_frame);
	if (st1 != st0 + 1) FAIL("single step moved %d frames", st1 - st0);
	if (vid.shown_frame != stream_of(st1)) FAIL("screen %d after the step, expected %d", vid.shown_frame, stream_of(st1));
	// reverse step
	w(0x03, 0x29);
	run(300);
	if (frame_no() != st1 - 1) FAIL("reverse step: frame %d", frame_no());

	// ---- frame step at 1/4 speed backwards for 1 s: ~7.5 frames
	w(0x03, 0x2B);
	int rs0 = frame_no();
	run(1000);
	printf("     1/4 speed reverse for 1 s: %d -> %d\n", rs0, frame_no());
	if (abs((rs0 - frame_no()) - 7) > 1) FAIL("reverse 1/4 speed moved %d", rs0 - frame_no());

	// ---- normal play into the picture stop code at frame 151
	w(0x03, 0x00);
	wait_until("picture stop", [] { return r(0x09) == 0x01; }, 5000);
	run(100);
	printf("     picture stop: frame %d, screen %d, out 07 %02X\n", frame_no(), vid.shown_frame, r(0x07));
	if (frame_no() < 150 || frame_no() > 152) FAIL("picture stop at frame %d", frame_no());

	// ---- fast forward 3x through to near the end
	w(0x03, 0x32);            // resumes (mode change clears the stop)
	int ff0 = frame_no();
	run(500);
	printf("     3x for 0.5 s: %d -> %d\n", ff0, frame_no());
	if (abs((frame_no() - ff0) - 45) > 4) FAIL("3x moved %d frames in 0.5 s", frame_no() - ff0);

	// ---- pause blanks the picture (no image hold)
	w(0x03, 0x00);
	w(0x02, 0x15);
	run(100);
	if (vid.visible) FAIL("picture still visible while paused");
	w(0x0C, 0x22);            // image hold: picture back
	run(50);
	if (!vid.visible) FAIL("picture not back with image hold");

	// ---- output freeze holds values while the drive moves
	w(0x0C, 0x02);
	w(0x02, 0x05);
	run(200);
	w(0x00, 0xC0);
	int frz = frame_no();
	run(300);
	if (frame_no() != frz) FAIL("frozen output block changed");
	w(0x00, 0x80);
	if (frame_no() == frz) FAIL("output block did not update after unfreeze");

	// ---- gate-array reset
	pd.ga_reset();
	run(50);
	wait_until("drive state 2 after the gate-array reset", [] { return (r(0x06) & 0x8F) == 0x02; }, 200);
	check("in 0C kept (only 00-0B are reset)", pd.in_regs[0x0C], 0x02);
	check("in 01 reset", pd.in_regs[0x01], 0x00);

	// ---- picture position from laseractive.cfg
	{
		const char *cfg_path = "harness_root/laseractive.cfg";
		FILE *fp = fopen(cfg_path, "w");
		fprintf(fp, "video_x -1.5\nvideo_y 3\n");
		fclose(fp);
		struct timespec ts[2] = { { 0, UTIME_OMIT }, { time(NULL) + 5, 0 } };
		utimensat(AT_FDCWD, cfg_path, ts, 0);
		run(1500);
		printf("     video_x -1.5, video_y 3 -> core offsets %d/256, %d\n", vid.pos_x, vid.pos_y);
		if (vid.pos_x != -384 || vid.pos_y != 3) FAIL("picture position from laseractive.cfg");
	}

	// ---- log_reads: a command's busy still clears while the CPU polls (it
	// stuck on in 2.0: the log was never empty when the ack came)
	{
		const char *cfg_path = "harness_root/laseractive.cfg";
		FILE *fp = fopen(cfg_path, "w");
		fprintf(fp, "video_x -1.5\nvideo_y 3\nlog_regs 1\nlog_reads 1\n");
		fclose(fp);
		struct timespec ts[2] = { { 0, UTIME_OMIT }, { time(NULL) + 10, 0 } };
		utimensat(AT_FDCWD, cfg_path, ts, 0);
		run(1500);
		if (!pd.log_reads) FAIL("log_reads 1 didn't reach the core");
		w(0x02, 0x04);            // pause
		check("out 06 busy after pause (log_reads on)", r(0x06) & 0x80, 0x80);
		unsigned long t1 = now_ms;
		if (wait_until("busy cleared with log_reads on", [] { return !(r(0x06) & 0x80); }, 500))
			printf("ok   busy cleared after %lu ms with log_reads on\n", now_ms - t1);
		w(0x02, 0x05);            // play again
		wait_until("busy cleared again", [] { return !(r(0x06) & 0x80); }, 500);
		for (int i = 0; i < 1200; i++) { r(0x06); r(0x06); step_ms(); }   // a BIOS-style poll loop
		fp = fopen(cfg_path, "w");
		fprintf(fp, "video_x -1.5\nvideo_y 3\n");
		fclose(fp);
		struct timespec ts2[2] = { { 0, UTIME_OMIT }, { time(NULL) + 15, 0 } };
		utimensat(AT_FDCWD, cfg_path, ts2, 0);
		run(1500);
		if (pd.log_reads) FAIL("log_reads stayed on after it was removed");
		FILE *rf = fopen("harness_root/laseractive_pd6103a.log", "r");
		char line[512];
		int reads = 0, summaries = 0;
		ld_log_if_open("harness: regs check");
		while (rf && fgets(line, sizeof line, rf)) { if (strstr(line, " R out ")) reads++; if (strstr(line, "repeated reads")) summaries++; }
		if (rf) fclose(rf);
		printf("     register log: %d reads logged, %d repeat summaries\n", reads, summaries);
		if (!reads || !summaries) FAIL("register log has no reads or no repeat summaries");
	}

	// ---- log_cpu: sub-CPU probe samples (rtl/MCD/PD6103A.vhd) are summed up
	// in laseractive.log and never taken for writes
	{
		const char *cfg_path = "harness_root/laseractive.cfg";
		FILE *fp = fopen(cfg_path, "w");
		fprintf(fp, "video_x -1.5\nvideo_y 3\nlog_cpu 1\n");
		fclose(fp);
		struct timespec ts[2] = { { 0, UTIME_OMIT }, { time(NULL) + 20, 0 } };
		utimensat(AT_FDCWD, cfg_path, ts, 0);
		run(1500);
		if (!pd.log_cpu) FAIL("log_cpu 1 didn't reach the core");
		uint8_t in10 = pd.in_applied[0x10], in17 = pd.in_applied[0x17];
		for (int k = 0; k < 3; k++)
		{
			// PC 01234A, data read at FF8004, status: read, IPL_N 5, not halted, not in reset, fetched
			// main CPU: PC FF0124, data read at A1200E, IPL_N 3
			const uint8_t b[14] = { 0x01, 0x23, 0x4A, 0xFF, 0x80, 0x04, 0x6F, 0xFF, 0x01, 0x24, 0xA1, 0x20, 0x0E, 0x5F };
			for (int i = 0; i < 14; i++) pd.push((uint16_t)(((0x10 + i) << 9) | 0x100 | b[i]));
			run(5);
		}
		run(1100);
		if (pd.in_applied[0x10] != in10 || pd.in_applied[0x17] != in17) FAIL("a probe entry was taken for a write");
		ld_log_if_open("harness: probe check");
		FILE *lf = fopen("harness_root/laseractive.log", "r");
		char line[1200];
		int found = 0;
		while (lf && fgets(line, sizeof line, lf))
		{
			if (strstr(line, "sub CPU, 3 samples: fetching at 01234A x3; data R FF8004 x3; IPL 2")) found |= 1;
			if (strstr(line, "main CPU, 3 samples: fetching at FF0124 x3; data R A1200E x3; IPL 4")) found |= 2;
		}
		if (found != 3) found = 0;
		if (lf) fclose(lf);
		if (!found) FAIL("log_cpu: no sub and main CPU lines with the right PCs and data addresses");
		else printf("ok   log_cpu probe samples summed up in laseractive.log\n");
		fp = fopen(cfg_path, "w");
		fprintf(fp, "video_x -1.5\nvideo_y 3\n");
		fclose(fp);
		struct timespec ts2[2] = { { 0, UTIME_OMIT }, { time(NULL) + 25, 0 } };
		utimensat(AT_FDCWD, cfg_path, ts2, 0);
		run(1500);
		if (pd.log_cpu) FAIL("log_cpu stayed on after it was removed");
	}

	// ---- audio read-ahead: the sources it hands out, from memory and read directly
	{
		ld_audio_stats_t a0;
		ld_audio_stats(&a0);
		printf("     audio read-ahead so far: %u sectors, %u read directly, %u chunk loads, %u read errors, %u late, longest gap %u ms\n",
			a0.sectors, a0.misses, a0.chunk_loads, a0.read_errors, a0.late, a0.worst_gap);
		if (a0.read_errors) FAIL("audio read errors");
		if (a0.misses * 10 > a0.sectors) FAIL("audio read-ahead mostly missing: %u of %u", a0.misses, a0.sectors);
		int t2 = cdd.TrackStart(1), bad = 0;
		int16_t dg[1176], an[1176];
		int hd, ha;
		int lbas[] = { -150, -1, 0, 149, t2, t2 + 1, t2 + 200, t2 + 449, t2 + 450, 520, 300, 301, 302 };
		for (int pass = 0; pass < 2; pass++)
			for (int l : lbas)
			{
				ld_audio_get(l, 1, dg, &hd, an, &ha);
				int aba = l + 150;
				bool exp_d = l >= t2 && l < t2 + 450, exp_a = aba < 850;
				if (hd != exp_d || ha != exp_a) { bad++; printf("     lba %d: digital %d analog %d\n", l, hd, ha); continue; }
				if (hd && (dg[0] != 1000 || dg[1] != -1000 || dg[1174] != 1000 || dg[1175] != -1000)) bad++;
				if (ha && (an[0] != (aba & 0x3FF) * 8 || an[1] != 0x2000 || an[1174] != (aba & 0x3FF) * 8)) bad++;
				run(30);   // let the reader fill in, so pass 2 comes from memory
			}
		// data track (2352-byte sectors, user data from byte 16): sector 0 says SEGADISCSYSTEM
		uint8_t db[2048];
		for (int pass = 0; pass < 2; pass++)
		{
			if (!ld_audio_get_data(0, 1, db) || memcmp(db, "SEGADISCSYSTEM", 14)) { bad++; printf("     data sector 0 wrong (pass %d)\n", pass); }
			if (!ld_audio_get_data(149, 1, db)) bad++;
			if (ld_audio_get_data(150, 1, db) || ld_audio_get_data(-1, 1, db)) bad++;   // not data
			run(30);
		}
		if (bad) FAIL("audio read-ahead: %d wrong sectors", bad);
		else printf("ok   audio read-ahead hands out the right sectors (memory and direct)\n");
	}

	// ---- log_sectors: the log (written by its own thread) names the sectors
	// the core was given, with the right CRCs
	{
		ld_log_if_open("harness: log check");   // writes out everything queued
		FILE *fp = fopen("harness_root/laseractive.log", "r");
		char line[1200];
		int n = 0, bad = 0, ended = 0;
		while (fp && fgets(line, sizeof line, fp))
		{
			if (strstr(line, "harness: log check")) ended = 1;
			const char *p = strstr(line, "sector: lba ");
			int lba; unsigned crc;
			if (!p || sscanf(p, "sector: lba %d", &lba) != 1) continue;
			const char *c = strstr(p, "crc ");
			if (!c || sscanf(c, "crc %x", &crc) != 1) { bad++; continue; }
			n++;
			if (lba >= 0 && data_crcs.count(lba) && data_crcs[lba] != crc) bad++;
		}
		if (fp) fclose(fp);
		printf("     log_sectors: %d sector lines\n", n);
		if (!ended) FAIL("log: the last line wasn't written out by ld_log_if_open");
		if (!n) FAIL("log_sectors logged no sectors");
		else if (bad) FAIL("log_sectors: %d lines with a wrong CRC", bad);
		else printf("ok   log_sectors lines match the sectors sent\n");
	}

	// ---- eject: the video player stops
	ld_disc_unmount();
	cdd.Unload();
	run(50);
	if (vid.enable) FAIL("video player still enabled after eject");

	printf("\nvideo: %u commits decoded, %u late, %u skipped; audio: %zu sectors; data: %zu sectors\n", vid.rd, vid.late, vid.drops,
		audio_marks.size(), data_lbas.size());
	printf("%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails, fails == 1 ? "" : "s");
	fflush(stdout);
	_exit(fails ? 1 : 0);
}
