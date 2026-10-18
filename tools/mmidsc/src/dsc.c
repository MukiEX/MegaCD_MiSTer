/*
 * dsc.c - VESA DSC encoder/decoder, 8bpc RGB 4:4:4, one slice per line.
 * See dsc.h for scope. Section numbers refer to the DSC 1.2a standard;
 * behaviour follows dsc_version_minor = 1 (DSC 1.1 semantics).
 *
 * Interpretation notes (places where the standard's text is informal and
 * the VESA C model is the normative reference):
 *
 *  RC1  Rate-control timing. Figure 6-13 shows the sizes of group n
 *       producing the QP used for group n + 2 (one extra group of latency
 *       for hardware decoders). That is what this code does: after group n,
 *       short-term RC computes stQp[n + 2] with prevQp = masterQp[n + 1]
 *       and prev2Qp = masterQp[n]. Groups 0 and 1 of a slice use QP 0.
 *  RC2  The flatness override (Section 6.8.5.2) is applied to stQp[t] just
 *       before group t is coded, once the signalling groups (t's supergroup
 *       start - 2 and - 1) have been parsed. "current masterQp equal to
 *       range_max_qp[14]" is tested on stQp[t].
 *  RC3  "Previous group has zero residuals for all three components" is
 *       rcSizeGroup == 3 (the minimum, Section 6.8.4).
 *  RC4  Range selection: the first range k with rcModelFullness <=
 *       rc_buf_thresh[k] - rc_model_size, else range 14.
 *  RC5  Linear transformation update order follows Section 6.8.2's list:
 *       scale decrement while scale > 8 at the start of the slice, scale set
 *       to 9 once the initial delay has passed on a non-first line with
 *       rcXformOffset > -rc_model_size (if scale_increment_interval != 0),
 *       then incremented every scale_increment_interval groups.
 *  ENC  Encoder-only decisions (flatness search, ICH choice, forced MPP)
 *       follow Sections 6.4.4.2, 6.5.3 and 6.8.5.1; they do not affect what
 *       a decoder must do.
 *
 * SPDX-License-Identifier: MIT
 */
#include "dsc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CLAMP(x, lo, hi) ((x) > (hi) ? (hi) : ((x) < (lo) ? (lo) : (x)))
#define ABS(x) ((x) < 0 ? -(x) : (x))

/* Table 6-2, 8bpc */
static const int QLEVEL_Y[16] = { 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 7 };
static const int QLEVEL_C[16] = { 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 8, 8, 8 };

#define MUX_WORD 48
#define MAX_SE 36             /* 4 * bpc + 4 for luma, 4 * (bpc + 1) for chroma at 8bpc */
#define SOMEWHAT_FLAT_QP 7    /* 7 + 2 * (bpc - 8) */
#define VERY_FLAT_QP 1        /* 1 + 2 * (bpc - 8) */

static int ceil_log2(int v)
{
	int r = 0;
	while (v) { r++; v >>= 1; }
	return r;
}

/* Minimum two's complement size of a residual (Table 3-1) */
static int res_size(int v)
{
	if (v == 0) return 0;
	return v > 0 ? ceil_log2(v) + 1 : ceil_log2(-v - 1) + 1;
}

static int floor_shift(long long v, int n)
{
	/* arithmetic shift right, rounding toward minus infinity */
	return (int)(v >= 0 ? v >> n : -((-v + (1LL << n) - 1) >> n));
}

static void set_err(char *err, size_t len, const char *msg)
{
	if (err && len) snprintf(err, len, "%s", msg);
}

/* ------------------------------------------------------------------ PPS */

static const int THRESH[14] = { 896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064 };
static const int MINQP_8BPP[15] = { 0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 9, 12 };
static const int MAXQP_8BPP[15] = { 4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13 };
static const int OFS_8BPP[15] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12 };

static int derive_basic(dsc_pps *p, char *err, size_t errlen)
{
	if (p->slice_width <= 0 || p->slice_height <= 0 || p->pic_width <= 0 || p->pic_height <= 0) {
		set_err(err, errlen, "bad picture or slice size");
		return -1;
	}
	p->groups_per_line = (p->slice_width + 2) / 3;
	p->groups_total = p->groups_per_line * p->slice_height;
	p->mux_word_size = MUX_WORD;
	int chunk = (p->slice_width * p->bits_per_pixel + 127) / 128;
	if (p->chunk_size && p->chunk_size != chunk) {
		set_err(err, errlen, "chunk_size does not match bits_per_pixel * slice_width");
		return -1;
	}
	p->chunk_size = chunk;
	p->slice_bits = 8 * p->chunk_size * p->slice_height;
	int extra = 3 * (MUX_WORD + MAX_SE - 2);
	while (extra > 0 && (p->slice_bits - extra) % MUX_WORD) extra--;
	p->num_extra_mux_bits = extra;
	p->slices_per_picture = (p->pic_height + p->slice_height - 1) / p->slice_height;
	p->rc_buffer_size = p->rc_model_size - p->initial_offset + (p->initial_xmit_delay * p->bits_per_pixel + 15) / 16 +
	                    p->groups_per_line * p->first_line_bpg_offset;
	return 0;
}

int dsc_pps_derive(dsc_pps *p, char *err, size_t errlen)
{
	if (p->dsc_version_major != 1 || (p->dsc_version_minor != 1 && p->dsc_version_minor != 2)) {
		set_err(err, errlen, "unsupported DSC version");
		return -1;
	}
	if (p->dsc_version_minor != 1) {
		set_err(err, errlen, "only dsc_version_minor = 1 bitstreams are implemented");
		return -1;
	}
	if (p->bits_per_component != 8 || !p->convert_rgb || p->simple_422 || p->native_420 || p->native_422 || p->vbr_enable) {
		set_err(err, errlen, "only 8bpc RGB 4:4:4 CBR is implemented");
		return -1;
	}
	if (p->block_pred_enable) {
		set_err(err, errlen, "block prediction is not implemented (block_pred_enable must be 0)");
		return -1;
	}
	if (p->linebuf_depth < 8 || p->linebuf_depth > 13) {
		set_err(err, errlen, "bad linebuf_depth");
		return -1;
	}
	if (p->slice_width != p->pic_width || p->slice_width % 3 || p->slice_width < 9) {
		set_err(err, errlen, "need one slice per line and a slice width that is a multiple of 3 (>= 9)");
		return -1;
	}
	if (p->bits_per_pixel < 6 * 16 || p->bits_per_pixel > 24 * 16) {
		set_err(err, errlen, "bits_per_pixel out of range");
		return -1;
	}
	if ((p->slice_width * p->bits_per_pixel) % 128) {
		set_err(err, errlen, "slice_width * bpp must be a whole number of bytes (no chunk padding is implemented)");
		return -1;
	}
	return derive_basic(p, err, errlen);
}

int dsc_pps_init(dsc_pps *p, int width, int height, int slice_height, int bpp16, char *err, size_t errlen)
{
	memset(p, 0, sizeof *p);
	p->dsc_version_major = 1;
	p->dsc_version_minor = 1;
	p->bits_per_component = 8;
	p->linebuf_depth = 9;
	p->convert_rgb = 1;
	p->bits_per_pixel = bpp16;
	p->pic_width = width;
	p->pic_height = height;
	p->slice_width = width;
	p->slice_height = slice_height;

	/* Table E-4 */
	p->rc_model_size = 8192;
	p->rc_edge_factor = 6;
	p->rc_tgt_offset_hi = 3;
	p->rc_tgt_offset_lo = 3;
	memcpy(p->rc_buf_thresh, THRESH, sizeof THRESH);
	/* Table E-5, 8bpp / 8bpc */
	p->flatness_min_qp = 3;
	p->flatness_max_qp = 12;
	p->rc_quant_incr_limit0 = 11;
	p->rc_quant_incr_limit1 = 11;
	for (int i = 0; i < 15; i++) {
		p->range_min_qp[i] = MINQP_8BPP[i];
		p->range_max_qp[i] = MAXQP_8BPP[i];
		p->range_bpg_offset[i] = OFS_8BPP[i];
	}
	p->initial_offset = 6144;
	/* initial_xmit_delay * bpp ~= rc_model_size / 2 (Table E-2) */
	p->initial_xmit_delay = (4096 * 16 + bpp16 / 2) / bpp16;
	if (p->initial_xmit_delay > 1023) p->initial_xmit_delay = 1023;
	/* Table E-2 */
	if (slice_height >= 8)
		p->first_line_bpg_offset = 12 + (int)(0.09 * MIN(34, slice_height - 8));
	else
		p->first_line_bpg_offset = 2 * (slice_height - 1);

	if (dsc_pps_derive(p, err, errlen)) return -1;
	if ((long)p->slice_width * p->slice_height < 15000 || p->slice_bits < 2 * p->rc_model_size) {
		/* Table E-2: rc_model_size = 8192 is meant for slices of 15000+ pixels;
		 * smaller slices can overflow their budget with these parameters. */
		set_err(err, errlen, "slice too small for the recommended rate-control parameters (need >= 15000 pixels)");
		return -1;
	}

	int gpl = p->groups_per_line;
	p->initial_scale_value = (p->rc_model_size * 8) / (p->rc_model_size - p->initial_offset);
	if (gpl < p->initial_scale_value - 8) p->initial_scale_value = gpl + 8;
	if (p->initial_scale_value > 8)
		p->scale_decrement_interval = gpl / (p->initial_scale_value - 8);
	else
		p->scale_decrement_interval = 4095;
	p->final_offset = p->rc_model_size - (p->initial_xmit_delay * p->bits_per_pixel + 8) / 16 + p->num_extra_mux_bits;
	if (p->final_offset >= p->rc_model_size) {
		set_err(err, errlen, "final_offset >= rc_model_size for this initial_xmit_delay");
		return -1;
	}
	int final_scale = (p->rc_model_size * 8) / (p->rc_model_size - p->final_offset);
	p->nfl_bpg_offset = slice_height > 1 ? ((p->first_line_bpg_offset << 11) + slice_height - 2) / (slice_height - 1) : 0;
	long long sbo = ((long long)(p->rc_model_size - p->initial_offset + p->num_extra_mux_bits) << 11);
	p->slice_bpg_offset = (int)((sbo + p->groups_total - 1) / p->groups_total);
	if (final_scale > 9)
		p->scale_increment_interval = (int)(((long long)p->final_offset << 11) /
		                                     ((long long)(p->nfl_bpg_offset + p->slice_bpg_offset) * (final_scale - 9)));
	else
		p->scale_increment_interval = 0;
	if (p->scale_increment_interval > 65535 || p->scale_decrement_interval > 4095 || p->nfl_bpg_offset > 65535 ||
	    p->slice_bpg_offset > 65535) {
		set_err(err, errlen, "rate-control parameters out of range; use a smaller slice height");
		return -1;
	}
	derive_basic(p, NULL, 0);
	int hrd = (p->rc_buffer_size * 16 + p->bits_per_pixel - 1) / p->bits_per_pixel;
	p->initial_dec_delay = hrd - p->initial_xmit_delay;
	return 0;
}

static void put16(uint8_t *b, int v) { b[0] = (uint8_t)(v >> 8); b[1] = (uint8_t)v; }
static int get16(const uint8_t *b) { return (b[0] << 8) | b[1]; }

void dsc_pps_pack(const dsc_pps *p, uint8_t o[DSC_PPS_SIZE])
{
	memset(o, 0, DSC_PPS_SIZE);
	o[0] = (uint8_t)((p->dsc_version_major << 4) | p->dsc_version_minor);
	o[1] = (uint8_t)p->pps_identifier;
	o[3] = (uint8_t)(((p->bits_per_component & 0xF) << 4) | (p->linebuf_depth & 0xF));
	o[4] = (uint8_t)((p->block_pred_enable << 5) | (p->convert_rgb << 4) | (p->simple_422 << 3) | (p->vbr_enable << 2) |
	                 ((p->bits_per_pixel >> 8) & 3));
	o[5] = (uint8_t)p->bits_per_pixel;
	put16(o + 6, p->pic_height);
	put16(o + 8, p->pic_width);
	put16(o + 10, p->slice_height);
	put16(o + 12, p->slice_width);
	put16(o + 14, p->chunk_size);
	o[16] = (uint8_t)((p->initial_xmit_delay >> 8) & 3);
	o[17] = (uint8_t)p->initial_xmit_delay;
	put16(o + 18, p->initial_dec_delay);
	o[21] = (uint8_t)(p->initial_scale_value & 0x3F);
	put16(o + 22, p->scale_increment_interval);
	o[24] = (uint8_t)((p->scale_decrement_interval >> 8) & 0xF);
	o[25] = (uint8_t)p->scale_decrement_interval;
	o[27] = (uint8_t)(p->first_line_bpg_offset & 0x1F);
	put16(o + 28, p->nfl_bpg_offset);
	put16(o + 30, p->slice_bpg_offset);
	put16(o + 32, p->initial_offset);
	put16(o + 34, p->final_offset);
	o[36] = (uint8_t)(p->flatness_min_qp & 0x1F);
	o[37] = (uint8_t)(p->flatness_max_qp & 0x1F);
	put16(o + 38, p->rc_model_size);
	o[40] = (uint8_t)(p->rc_edge_factor & 0xF);
	o[41] = (uint8_t)(p->rc_quant_incr_limit0 & 0x1F);
	o[42] = (uint8_t)(p->rc_quant_incr_limit1 & 0x1F);
	o[43] = (uint8_t)(((p->rc_tgt_offset_hi & 0xF) << 4) | (p->rc_tgt_offset_lo & 0xF));
	for (int i = 0; i < 14; i++) o[44 + i] = (uint8_t)(p->rc_buf_thresh[i] >> 6);
	for (int i = 0; i < 15; i++)
		put16(o + 58 + 2 * i, ((p->range_min_qp[i] & 0x1F) << 11) | ((p->range_max_qp[i] & 0x1F) << 6) |
		                          (p->range_bpg_offset[i] & 0x3F));
	o[88] = (uint8_t)((p->native_420 << 1) | p->native_422);
	o[89] = (uint8_t)(p->second_line_bpg_offset & 0x1F);
	put16(o + 90, p->nsl_bpg_offset);
	put16(o + 92, p->second_line_offset_adj);
}

int dsc_pps_unpack(dsc_pps *p, const uint8_t o[DSC_PPS_SIZE], char *err, size_t errlen)
{
	memset(p, 0, sizeof *p);
	p->dsc_version_major = o[0] >> 4;
	p->dsc_version_minor = o[0] & 0xF;
	p->pps_identifier = o[1];
	p->bits_per_component = o[3] >> 4;
	if (p->bits_per_component == 0) p->bits_per_component = 16;
	p->linebuf_depth = o[3] & 0xF;
	if (p->linebuf_depth == 0) p->linebuf_depth = 16;
	p->block_pred_enable = (o[4] >> 5) & 1;
	p->convert_rgb = (o[4] >> 4) & 1;
	p->simple_422 = (o[4] >> 3) & 1;
	p->vbr_enable = (o[4] >> 2) & 1;
	p->bits_per_pixel = ((o[4] & 3) << 8) | o[5];
	p->pic_height = get16(o + 6);
	p->pic_width = get16(o + 8);
	p->slice_height = get16(o + 10);
	p->slice_width = get16(o + 12);
	p->chunk_size = get16(o + 14);
	p->initial_xmit_delay = ((o[16] & 3) << 8) | o[17];
	p->initial_dec_delay = get16(o + 18);
	p->initial_scale_value = o[21] & 0x3F;
	p->scale_increment_interval = get16(o + 22);
	p->scale_decrement_interval = ((o[24] & 0xF) << 8) | o[25];
	p->first_line_bpg_offset = o[27] & 0x1F;
	p->nfl_bpg_offset = get16(o + 28);
	p->slice_bpg_offset = get16(o + 30);
	p->initial_offset = get16(o + 32);
	p->final_offset = get16(o + 34);
	p->flatness_min_qp = o[36] & 0x1F;
	p->flatness_max_qp = o[37] & 0x1F;
	p->rc_model_size = get16(o + 38);
	p->rc_edge_factor = o[40] & 0xF;
	p->rc_quant_incr_limit0 = o[41] & 0x1F;
	p->rc_quant_incr_limit1 = o[42] & 0x1F;
	p->rc_tgt_offset_hi = o[43] >> 4;
	p->rc_tgt_offset_lo = o[43] & 0xF;
	for (int i = 0; i < 14; i++) p->rc_buf_thresh[i] = o[44 + i] << 6;
	for (int i = 0; i < 15; i++) {
		int v = get16(o + 58 + 2 * i);
		p->range_min_qp[i] = (v >> 11) & 0x1F;
		p->range_max_qp[i] = (v >> 6) & 0x1F;
		int ofs = v & 0x3F;
		p->range_bpg_offset[i] = ofs >= 32 ? ofs - 64 : ofs;
	}
	p->native_420 = (o[88] >> 1) & 1;
	p->native_422 = o[88] & 1;
	p->second_line_bpg_offset = o[89] & 0x1F;
	p->nsl_bpg_offset = get16(o + 90);
	p->second_line_offset_adj = get16(o + 92);
	return dsc_pps_derive(p, err, errlen);
}

size_t dsc_picture_bytes(const dsc_pps *p)
{
	return (size_t)p->chunk_size * p->slice_height * p->slices_per_picture;
}

/* ------------------------------------------------------------ bit I/O */

typedef struct {
	uint8_t *buf;
	size_t cap;   /* bytes */
	size_t bits;  /* bits written */
	size_t rpos;  /* read position (decoder) */
} bitbuf;

static void bb_reserve(bitbuf *b, size_t bits)
{
	size_t need = (bits + 7) / 8 + 16;
	if (need <= b->cap) return;
	size_t nc = b->cap ? b->cap * 2 : 4096;
	while (nc < need) nc *= 2;
	uint8_t *nb = realloc(b->buf, nc);
	if (!nb) { fprintf(stderr, "dsc: out of memory\n"); exit(1); }
	memset(nb + b->cap, 0, nc - b->cap);
	b->buf = nb;
	b->cap = nc;
}

static void bb_put(bitbuf *b, uint32_t v, int n)
{
	if (n <= 0) return;
	bb_reserve(b, b->bits + (size_t)n);
	while (n > 0) {
		int room = 8 - (int)(b->bits & 7);
		int take = n < room ? n : room;
		uint32_t chunk = (v >> (n - take)) & ((1u << take) - 1);
		b->buf[b->bits >> 3] |= (uint8_t)(chunk << (room - take));
		b->bits += (size_t)take;
		n -= take;
	}
}

static void bb_zeros(bitbuf *b, int n)
{
	if (n <= 0) return;
	bb_reserve(b, b->bits + (size_t)n);
	b->bits += (size_t)n;
}

static int bb_get_bit(const uint8_t *buf, size_t pos)
{
	return (buf[pos >> 3] >> (7 - (pos & 7))) & 1;
}

/* Read n <= 32 bits at bit position pos; bits past srcbits read as 0. */
static uint32_t bb_peek(const uint8_t *src, size_t srcbits, size_t pos, int n)
{
	uint32_t v = 0;
	while (n > 0) {
		if (pos >= srcbits) { v <<= n; break; }
		int avail = 8 - (int)(pos & 7);
		int take = n < avail ? n : avail;
		if ((size_t)take > srcbits - pos) take = (int)(srcbits - pos);
		uint32_t byte = src[pos >> 3];
		v = (v << take) | ((byte >> (avail - take)) & ((1u << take) - 1));
		pos += (size_t)take;
		n -= take;
	}
	return v;
}

/* Copy n bits from src at position spos into dst (appending). Bits past
 * srcbits read as 0. */
static void bb_copy(bitbuf *dst, const uint8_t *src, size_t srcbits, size_t spos, int n)
{
	while (n > 0) {
		int k = n > 24 ? 24 : n;
		bb_put(dst, bb_peek(src, srcbits, spos, k), k);
		spos += (size_t)k;
		n -= k;
	}
}

/* ------------------------------------------------------- slice coder */

typedef struct {
	const dsc_pps *p;
	const dsc_enc_opts *opts;
	int encoding;
	int w, h, gpl, total;
	int depth[3], maxval[3], mid[3];
	int lb_shift[3];

	const int *orig[3]; /* encoder: original YCoCg samples, w*h each */
	int *recon[3];      /* reconstructed samples, w*h each */

	int ich[32][3];
	int ich_valid[32];

	int req_size[3][3];
	int prev_qlevel[3];
	int prev_ich;
	int prev_recon[3];

	/* rate control */
	int buffer_fullness;
	int pixel_count;
	int bpg_frac_accum;
	long long xform_offset; /* rcXformOffset, 11 fractional bits */
	int offset_clamp;
	int scale, scale_counter, scale_inc_start;
	int *st_qp, *master_qp; /* per group, total + 2 entries */
	int rc_size_prev;
	int force_mpp;

	/* flatness */
	int pending_flag, pending_grp, pending_type;
	int flat_target, flat_type;
	int prev_is_flat; /* encoder */

	/* substreams */
	bitbuf ss[3];
	int *se_size[3]; /* encoder: per group syntax element sizes */
	int ss_fullness[3];
	size_t ss_read[3];

	dsc_stats *stats;
} slice_t;

static int qlevel(int c, int qp)
{
	qp = CLAMP(qp, 0, 15);
	return c == 0 ? QLEVEL_Y[qp] : QLEVEL_C[qp];
}

/* Line storage read (Section 6.3) for the previous line */
static int prev_sample(const slice_t *S, int c, int line, int x)
{
	x = CLAMP(x, 0, S->w - 1);
	int v = S->recon[c][(line - 1) * S->w + x];
	int sh = S->lb_shift[c];
	if (sh) {
		int st = MIN((v + (1 << (sh - 1))) >> sh, (1 << S->p->linebuf_depth) - 1);
		v = st << sh;
	}
	return v;
}

static int ich_prevline_start(const slice_t *S, int hpos)
{
	int start = hpos == 0 ? 0 : hpos - 2;
	if (start + 7 > S->w) start = S->w - 7;
	return start;
}

/* Value of ICH entry idx for a group at (line, hpos) */
static void ich_value(const slice_t *S, int line, int hpos, int idx, int out[3])
{
	if (line > 0 && idx >= 25) {
		int x = ich_prevline_start(S, hpos) + (idx - 25);
		if (!S->lb_shift[1]) {
			size_t o = (size_t)(line - 1) * S->w + x;
			out[0] = S->recon[0][o]; out[1] = S->recon[1][o]; out[2] = S->recon[2][o];
		} else {
			for (int c = 0; c < 3; c++) out[c] = prev_sample(S, c, line, x);
		}
	} else {
		for (int c = 0; c < 3; c++) out[c] = S->ich[idx][c];
	}
}

static int ich_entry_valid(const slice_t *S, int line, int idx)
{
	if (line > 0 && idx >= 25) return 1;
	return S->ich_valid[idx];
}

static void ich_update_p(slice_t *S, int line, const int px[3][3])
{
	int n = line > 0 ? 25 : 32;
	for (int i = n - 1; i >= 3; i--) {
		memcpy(S->ich[i], S->ich[i - 3], sizeof S->ich[i]);
		S->ich_valid[i] = S->ich_valid[i - 3];
	}
	for (int k = 0; k < 3; k++) {
		/* rightmost pixel becomes entry 0 */
		for (int c = 0; c < 3; c++) S->ich[k][c] = px[2 - k][c];
		S->ich_valid[k] = 1;
	}
}

static void ich_update_ich(slice_t *S, int line, int hpos, const int idx[3])
{
	int n = line > 0 ? 25 : 32;
	int sel[3], nsel = 0;
	/* ignore the first occurrence of a repeated index */
	for (int k = 0; k < 3; k++) {
		int dup = 0;
		for (int j = k + 1; j < 3; j++) if (idx[j] == idx[k]) dup = 1;
		if (!dup) sel[nsel++] = idx[k];
	}
	int newv[32][3], newvalid[32], cnt = 0;
	for (int k = nsel - 1; k >= 0; k--) {
		ich_value(S, line, hpos, sel[k], newv[cnt]);
		newvalid[cnt++] = 1;
	}
	for (int i = 0; i < n && cnt < n; i++) {
		int used = 0;
		for (int k = 0; k < nsel; k++) if (sel[k] == i) used = 1;
		if (used) continue;
		memcpy(newv[cnt], S->ich[i], sizeof newv[cnt]);
		newvalid[cnt++] = S->ich_valid[i];
	}
	for (int i = 0; i < n; i++) {
		memcpy(S->ich[i], newv[i], sizeof S->ich[i]);
		S->ich_valid[i] = newvalid[i];
	}
}

static int adj_pred_size(const slice_t *S, int c, int g, int q)
{
	if (g == 0) return 0;
	int ps = (S->req_size[c][0] + S->req_size[c][1] + 2 * S->req_size[c][2] + 2) >> 2;
	int change = q - S->prev_qlevel[c];
	int max_size = S->depth[c] - q;
	return CLAMP(ps - change, 0, max_size - 1);
}

/* MMAP prediction for the three samples of component c (Section 6.4.1).
 * R[] are the inverse-quantized residuals of earlier samples in the group;
 * returns the prediction for sample i. */
typedef struct {
	int first_line;
	int a, blend_b, blend_c, blend_d, blend_e;
} mmap_ctx;

static void mmap_setup(const slice_t *S, int c, int line, int hpos, int q, mmap_ctx *m)
{
	m->first_line = line == 0;
	if (hpos == 0) m->a = S->mid[c];
	else m->a = S->recon[c][line * S->w + hpos - 1];
	if (m->first_line) return;
	int cc2 = prev_sample(S, c, line, hpos - 2);
	int cc = prev_sample(S, c, line, hpos - 1);
	int b = prev_sample(S, c, line, hpos);
	int d = prev_sample(S, c, line, hpos + 1);
	int e = prev_sample(S, c, line, hpos + 2);
	int f = prev_sample(S, c, line, hpos + 3);
	if (hpos == 0) { cc2 = b; cc = b; } /* replication left of the slice */
	int filt_c = (cc2 + 2 * cc + b + 2) >> 2;
	int filt_b = (cc + 2 * b + d + 2) >> 2;
	int filt_d = (b + 2 * d + e + 2) >> 2;
	int filt_e = (d + 2 * e + f + 2) >> 2;
	int lim = (1 << q) / 2;
	m->blend_c = cc + CLAMP(filt_c - cc, -lim, lim);
	m->blend_b = b + CLAMP(filt_b - b, -lim, lim);
	m->blend_d = d + CLAMP(filt_d - d, -lim, lim);
	m->blend_e = e + CLAMP(filt_e - e, -lim, lim);
	if (hpos == 0) m->blend_c = S->mid[c];
}

static int mmap_pred(const slice_t *S, int c, const mmap_ctx *m, int i, const int R[3])
{
	int a = m->a;
	if (m->first_line) {
		if (i == 0) return a;
		if (i == 1) return CLAMP(a + R[0], 0, S->maxval[c]);
		return CLAMP(a + R[0] + R[1], 0, S->maxval[c]);
	}
	int b = m->blend_b, cc = m->blend_c, d = m->blend_d, e = m->blend_e;
	if (i == 0) return CLAMP(a + b - cc, MIN(a, b), MAX(a, b));
	if (i == 1) return CLAMP(a + d - cc + R[0], MIN(a, MIN(b, d)), MAX(a, MAX(b, d)));
	return CLAMP(a + e - cc + R[0] + R[1], MIN(MIN(a, b), MIN(d, e)), MAX(MAX(a, b), MAX(d, e)));
}

static int quantize(int e, int q)
{
	int round = q > 0 ? (1 << q) / 2 - 1 : 0;
	return e < 0 ? -((round - e) >> q) : ((e + round) >> q);
}

static int mpp_pred(const slice_t *S, int c, int q)
{
	return (1 << (S->depth[c] - 1)) + (S->prev_recon[c] & ((1 << q) - 1));
}

/* ------------------------------------------------------- rate control */

static int flat_override(slice_t *S, int t, int st)
{
	int qp = st;
	if (S->flat_target == t) {
		if (st != S->p->range_max_qp[14]) {
			if (S->flat_type) qp = VERY_FLAT_QP;
			else qp = MAX(st - 4, 0);
		}
		S->flat_target = -1;
		if (S->stats) S->stats->flat_signals++;
	}
	return CLAMP(qp, 0, 15);
}

static void rc_after_group(slice_t *S, int g, int coded, int rc_size)
{
	const dsc_pps *p = S->p;
	int line = g / S->gpl;
	int first_line = line == 0;
	int bpp = p->bits_per_pixel;
	int ixd = p->initial_xmit_delay;

	/* Buffer level tracker (Section 6.8.1) */
	int prev_pixel_count = S->pixel_count;
	int removed = 0;
	for (int i = 0; i < 3; i++) {
		S->pixel_count++;
		if (S->pixel_count >= ixd) {
			S->bpg_frac_accum += bpp & 0xF;
			removed += (bpp >> 4) + (S->bpg_frac_accum >> 4);
			S->bpg_frac_accum &= 0xF;
		}
		if (((S->pixel_count - ixd) % p->slice_width) == 0) S->bpg_frac_accum = 0;
	}
	S->buffer_fullness += coded - removed;

	/* Linear transformation (Section 6.8.2) */
	if (g == 0) {
		S->scale = p->initial_scale_value;
		S->scale_counter = 0;
	} else if (!S->scale_inc_start && S->scale > 8) {
		if (++S->scale_counter >= p->scale_decrement_interval) {
			S->scale_counter = 0;
			S->scale--;
		}
	} else if (S->scale_inc_start) {
		if (++S->scale_counter >= p->scale_increment_interval) {
			S->scale_counter = 0;
			S->scale++;
		}
	}
	long long inc = first_line ? -((long long)p->first_line_bpg_offset << 11) : p->nfl_bpg_offset;
	if (prev_pixel_count < ixd) {
		int n = MIN(3, ixd - prev_pixel_count);
		inc -= ((long long)bpp * n) << 7;
	} else if (p->scale_increment_interval && !S->scale_inc_start && !first_line &&
	           S->xform_offset > -((long long)p->rc_model_size << 11)) {
		S->scale = 9;
		S->scale_counter = 0;
		S->scale_inc_start = 1;
	}
	inc += p->slice_bpg_offset;
	S->xform_offset += inc;
	long long final_lim = (long long)(p->final_offset - p->rc_model_size) << 11;
	if (S->xform_offset < final_lim) S->offset_clamp = 1;
	if (S->offset_clamp && S->xform_offset > final_lim) S->xform_offset = final_lim;

	int ofs = floor_shift(S->xform_offset, 11);
	int model = floor_shift((long long)S->scale * (S->buffer_fullness + ofs), 3);

	/* Long-term parameter selection (Section 6.8.3) */
	int r = 14;
	for (int k = 0; k < 14; k++) {
		if (model <= p->rc_buf_thresh[k] - p->rc_model_size) { r = k; break; }
	}
	int min_qp = p->range_min_qp[r], max_qp = p->range_max_qp[r];

	/* masterQp for the next group (RC2) */
	if (g + 1 < S->total) S->master_qp[g + 1] = flat_override(S, g + 1, S->st_qp[g + 1]);

	/* Short-term QP adjustment (Section 6.8.4, DSC 1.1 flowchart) for group g + 2 (RC1) */
	if (g + 2 < S->total + 2) {
		int bpg_ofs = first_line ? p->first_line_bpg_offset : -(p->nfl_bpg_offset >> 11);
		bpg_ofs -= p->slice_bpg_offset >> 11;
		int tgt = ((3 * bpp + 8) >> 4) + p->range_bpg_offset[r] + bpg_ofs;
		int tgt_minus = tgt - p->rc_tgt_offset_lo;
		int tgt_plus = tgt + p->rc_tgt_offset_hi;
		int prev_qp = g + 1 < S->total ? S->master_qp[g + 1] : S->master_qp[g];
		int prev2_qp = S->master_qp[g];
		int st;
		if (S->buffer_fullness + ofs > -172) {
			st = p->range_max_qp[14];
		} else if (rc_size == 3) {
			st = MAX(prev_qp - 1, min_qp / 2);
		} else if (coded < tgt_minus && rc_size < tgt_minus) {
			st = MAX(prev_qp - 1, min_qp);
		} else if (coded > tgt_plus && S->buffer_fullness >= 64) {
			int cur = MAX(min_qp, prev_qp);
			int incr = (coded - tgt) >> 1;
			int edge = rc_size * 2 < S->rc_size_prev * p->rc_edge_factor;
			if (cur == prev2_qp)
				st = edge ? MIN(max_qp, cur + incr) : cur;
			else if (cur < prev2_qp)
				st = (edge && cur < p->rc_quant_incr_limit0) ? MIN(max_qp, cur + incr) : cur;
			else if (cur < p->rc_quant_incr_limit1)
				st = MIN(max_qp, cur + incr);
			else
				st = cur;
		} else {
			st = prev_qp;
		}
		S->st_qp[g + 2] = CLAMP(st, 0, 15);
	}
	S->rc_size_prev = rc_size;

	/* forceMpp for the next group (encoder only, Section 6.8.1) */
	int max_bpg = (bpp * 3 + 15) >> 4;
	S->force_mpp = (S->pixel_count > ixd) && (S->buffer_fullness < max_bpg - 3);
}

/* ------------------------------------------------- flatness (encoder) */

static int group_flatness(const slice_t *S, int t, int qp)
{
	int line = t / S->gpl;
	int hpos = (t % S->gpl) * 3;
	int fq = MAX(0, qp - 4);
	for (int pass = 0; pass < 2; pass++) {
		int x0 = pass == 0 ? hpos - 1 : hpos;
		int x1 = pass == 0 ? hpos + 2 : hpos + 5;
		int very = 1, some = 1;
		for (int c = 0; c < 3; c++) {
			int mn = 1 << 30, mx = -1;
			for (int x = x0; x <= x1; x++) {
				int v = S->orig[c][line * S->w + CLAMP(x, 0, S->w - 1)];
				mn = MIN(mn, v);
				mx = MAX(mx, v);
			}
			int d = mx - mn;
			if (d > 2) very = 0;
			if (d > (1 << qlevel(c, fq))) some = 0;
		}
		if (very) return 2;
		if (some) return 1;
	}
	return 0;
}

static void enc_flatness_search(slice_t *S, int g, int qp, int signal)
{
	int s = g + 2;
	int line = g / S->gpl;
	S->pending_flag = 0;
	for (int t = s; t < s + 4; t++) {
		int fl = 0;
		if (t < S->total && t / S->gpl == line) fl = group_flatness(S, t, qp);
		int is_flat = fl > 0;
		if (signal && !S->pending_flag && !S->prev_is_flat && is_flat) {
			S->pending_flag = 1;
			S->pending_grp = t - s;
			S->pending_type = fl == 2;
		}
		S->prev_is_flat = is_flat;
	}
}

/* ------------------------------------------------------- encoder */

typedef struct {
	int q[3], max_size[3], adj[3];
	int qe[3][3];
	int recon[3][3]; /* [pixel][comp] */
	int mpp[3];
	int unit_res_size[3];
	int coded_size[3];
} pgroup;

static void enc_pmode(slice_t *S, int g, int line, int hpos, int qp, pgroup *P)
{
	for (int c = 0; c < 3; c++) {
		int q = qlevel(c, qp);
		P->q[c] = q;
		P->max_size[c] = S->depth[c] - q;
		P->adj[c] = adj_pred_size(S, c, g, q);
		mmap_ctx m;
		mmap_setup(S, c, line, hpos, q, &m);
		int R[3] = { 0, 0, 0 };
		int maxrs = 0;
		for (int i = 0; i < 3; i++) {
			int x = S->orig[c][line * S->w + hpos + i];
			int pr = mmap_pred(S, c, &m, i, R);
			int qe = quantize(x - pr, q);
			R[i] = qe << q;
			P->qe[i][c] = qe;
			P->recon[i][c] = CLAMP(pr + R[i], 0, S->maxval[c]);
			maxrs = MAX(maxrs, res_size(qe));
		}
		if (S->force_mpp || maxrs >= P->max_size[c]) {
			int pr = mpp_pred(S, c, q);
			int lim = 1 << (P->max_size[c] - 1);
			for (int i = 0; i < 3; i++) {
				int x = S->orig[c][line * S->w + hpos + i];
				int qe = CLAMP(quantize(x - pr, q), -lim, lim - 1);
				P->qe[i][c] = qe;
				P->recon[i][c] = CLAMP(pr + (qe << q), 0, S->maxval[c]);
			}
			P->mpp[c] = 1;
			P->unit_res_size[c] = P->max_size[c];
			P->coded_size[c] = P->max_size[c];
		} else {
			P->mpp[c] = 0;
			P->unit_res_size[c] = maxrs;
			P->coded_size[c] = MAX(maxrs, P->adj[c]);
		}
	}
}

static int pmode_bits(const slice_t *S, const pgroup *P)
{
	int bits = 0;
	for (int c = 0; c < 3; c++) {
		int delta = MAX(0, P->coded_size[c] - P->adj[c]);
		int prefix;
		if (c == 0) {
			if (S->prev_ich) prefix = 1 + delta + (P->coded_size[0] < P->max_size[0] ? 1 : 0);
			else prefix = delta + 1;
		} else {
			prefix = delta + (P->coded_size[c] < P->max_size[c] ? 1 : 0);
		}
		bits += prefix + 3 * P->coded_size[c];
	}
	return bits;
}

static int enc_ich_decide(slice_t *S, int g, int line, int hpos, int qp, const pgroup *P, int idx_out[3])
{
	if ((S->opts && S->opts->disable_ich) || S->force_mpp || g == 0) return 0;
	int n = line > 0 ? 32 : 32; /* indices 0..31; on non-first lines 25..31 are previous-line pixels */
	int mod_qp = MIN(qp + 2, 15);
	int max_err_y = (1 << qlevel(0, mod_qp)) / 2;
	int max_err_c = (1 << qlevel(1, mod_qp)) / 2;
	int vals[32][3], valid[32];
	int nhist = line > 0 ? 25 : 32;
	memcpy(vals, S->ich, sizeof(int) * 3 * (size_t)nhist);
	memcpy(valid, S->ich_valid, sizeof(int) * (size_t)nhist);
	for (int i = nhist; i < n; i++) {
		valid[i] = 1;
		ich_value(S, line, hpos, i, vals[i]);
	}
	/* First condition (Section 6.5.3.2): every pixel needs some entry within
	 * the error threshold. Checked first because it usually fails. */
	int o[3][3];
	for (int px = 0; px < 3; px++)
		for (int c = 0; c < 3; c++) o[px][c] = S->orig[c][line * S->w + hpos + px];
	for (int px = 0; px < 3; px++) {
		int ok = 0;
		for (int i = 0; i < n && !ok; i++)
			ok = valid[i] && ABS(o[px][0] - vals[i][0]) <= max_err_y && ABS(o[px][1] - vals[i][1]) <= max_err_c &&
			     ABS(o[px][2] - vals[i][2]) <= max_err_c;
		if (!ok) return 0;
	}
	/* Best entry per pixel (Section 6.5.3.1) */
	int max_err_ich[3] = { 0, 0, 0 };
	for (int px = 0; px < 3; px++) {
		int best = -1, best_sad = 1 << 30;
		for (int i = 0; i < n; i++) {
			if (!valid[i]) continue;
			int sad = 2 * ABS(o[px][0] - vals[i][0]) + ABS(o[px][1] - vals[i][1]) + ABS(o[px][2] - vals[i][2]);
			if (sad < best_sad) { best_sad = sad; best = i; }
		}
		idx_out[px] = best;
		for (int c = 0; c < 3; c++) max_err_ich[c] = MAX(max_err_ich[c], ABS(o[px][c] - vals[best][c]));
	}
	int max_err_p[3] = { 0, 0, 0 };
	for (int px = 0; px < 3; px++)
		for (int c = 0; c < 3; c++)
			max_err_p[c] = MAX(max_err_p[c], ABS(S->orig[c][line * S->w + hpos + px] - P->recon[px][c]));
	int bits_ich = (S->prev_ich ? 1 : P->max_size[0] + 1 - P->adj[0]) + 15;
	int bits_p = pmode_bits(S, P);
	int log_ich = 2 * ceil_log2(max_err_ich[0]) + ceil_log2(max_err_ich[1]) + ceil_log2(max_err_ich[2]);
	int log_p = 2 * ceil_log2(max_err_p[0]) + ceil_log2(max_err_p[1]) + ceil_log2(max_err_p[2]);
	return (log_ich <= log_p) && (bits_ich + 4 * log_ich < bits_p + 4 * log_p);
}

static void write_twos(bitbuf *b, int v, int n)
{
	if (n > 0) bb_put(b, (uint32_t)v & ((1u << n) - 1), n);
}

static void enc_group(slice_t *S, int g)
{
	int line = g / S->gpl;
	int gx = g % S->gpl;
	int hpos = gx * 3;
	int qp = S->master_qp[g];
	size_t start[3];
	for (int c = 0; c < 3; c++) start[c] = S->ss[c].bits;

	/* Flatness syntax (Table 4-9) */
	if (g % 4 == 3) {
		int in_range = qp >= S->p->flatness_min_qp && qp <= S->p->flatness_max_qp;
		enc_flatness_search(S, g, qp, in_range);
		if (in_range) bb_put(&S->ss[0], (uint32_t)S->pending_flag, 1);
		else S->pending_flag = 0;
	} else if (g % 4 == 0 && S->pending_flag) {
		int type = 0;
		if (qp >= SOMEWHAT_FLAT_QP) {
			type = S->pending_type;
			bb_put(&S->ss[0], (uint32_t)type, 1);
		}
		bb_put(&S->ss[0], (uint32_t)S->pending_grp, 2);
		S->flat_target = g + 1 + S->pending_grp;
		S->flat_type = type;
		S->pending_flag = 0;
	}

	pgroup P;
	enc_pmode(S, g, line, hpos, qp, &P);
	int idx[3];
	int use_ich = enc_ich_decide(S, g, line, hpos, qp, &P, idx);

	int px[3][3];
	int rc_size;
	if (use_ich) {
		if (S->prev_ich) bb_put(&S->ss[0], 1, 1);
		else bb_zeros(&S->ss[0], P.max_size[0] + 1 - P.adj[0]);
		for (int k = 0; k < 3; k++) {
			bb_put(&S->ss[k], (uint32_t)idx[k], 5);
			ich_value(S, line, hpos, idx[k], px[k]);
		}
		rc_size = 16;
		if (S->stats) S->stats->ich_groups++;
	} else {
		rc_size = 0;
		for (int c = 0; c < 3; c++) {
			int delta = MAX(0, P.coded_size[c] - P.adj[c]);
			if (c == 0 && S->prev_ich) {
				bb_zeros(&S->ss[0], 1 + delta);
				if (P.coded_size[0] < P.max_size[0]) bb_put(&S->ss[0], 1, 1);
			} else if (c == 0) {
				bb_zeros(&S->ss[0], delta);
				bb_put(&S->ss[0], 1, 1);
			} else {
				bb_zeros(&S->ss[c], delta);
				if (P.coded_size[c] < P.max_size[c]) bb_put(&S->ss[c], 1, 1);
			}
			for (int i = 0; i < 3; i++) write_twos(&S->ss[c], P.qe[i][c], P.coded_size[c]);
			rc_size += 3 * P.unit_res_size[c] + 1;
			if (P.mpp[c] && S->stats) S->stats->mpp_units++;
		}
		memcpy(px, P.recon, sizeof px);
	}

	/* State updates shared with the decoder path */
	for (int i = 0; i < 3; i++)
		for (int c = 0; c < 3; c++) S->recon[c][line * S->w + hpos + i] = px[i][c];
	if (!use_ich) {
		for (int c = 0; c < 3; c++)
			for (int i = 0; i < 3; i++) S->req_size[c][i] = P.mpp[c] ? P.max_size[c] : res_size(P.qe[i][c]);
	}
	if (gx != S->gpl - 1) {
		if (use_ich) ich_update_ich(S, line, hpos, idx);
		else ich_update_p(S, line, (const int (*)[3])px);
	}
	for (int c = 0; c < 3; c++) {
		S->prev_qlevel[c] = qlevel(c, qp);
		S->prev_recon[c] = px[2][c];
	}
	S->prev_ich = use_ich;
	if (S->stats) {
		S->stats->qp_hist[CLAMP(qp, 0, 15)]++;
		if (S->force_mpp) S->stats->force_mpp++;
	}

	int coded = 0;
	for (int c = 0; c < 3; c++) {
		S->se_size[c][g] = (int)(S->ss[c].bits - start[c]);
		coded += S->se_size[c][g];
	}
	rc_after_group(S, g, coded, rc_size);
}

/* ------------------------------------------------------- decoder */

static int dec_bit(slice_t *S, int c, int *err)
{
	bitbuf *b = &S->ss[c];
	if (b->rpos >= b->bits) { *err = 1; return 0; }
	return bb_get_bit(b->buf, b->rpos++);
}

static int dec_bits(slice_t *S, int c, int n, int *err)
{
	int v = 0;
	for (int i = 0; i < n; i++) v = (v << 1) | dec_bit(S, c, err);
	return v;
}

static int dec_twos(slice_t *S, int c, int n, int *err)
{
	if (n == 0) return 0;
	int v = dec_bits(S, c, n, err);
	if (v & (1 << (n - 1))) v -= 1 << n;
	return v;
}

static int dec_group(slice_t *S, int g, const uint8_t *slice, size_t slice_bits, size_t *mux_pos)
{
	int err = 0;
	int line = g / S->gpl;
	int gx = g % S->gpl;
	int hpos = gx * 3;
	int qp = S->master_qp[g];

	/* Substream demultiplexer (Table 4-6) */
	for (int c = 0; c < 3; c++) {
		if (S->ss_fullness[c] < MAX_SE) {
			if (*mux_pos + MUX_WORD > slice_bits) return -1;
			bb_copy(&S->ss[c], slice, slice_bits, *mux_pos, MUX_WORD);
			*mux_pos += MUX_WORD;
			S->ss_fullness[c] += MUX_WORD;
		}
	}
	size_t start[3];
	for (int c = 0; c < 3; c++) start[c] = S->ss[c].rpos;

	if (g % 4 == 3) {
		S->pending_flag = 0;
		if (qp >= S->p->flatness_min_qp && qp <= S->p->flatness_max_qp) S->pending_flag = dec_bit(S, 0, &err);
	} else if (g % 4 == 0 && S->pending_flag) {
		int type = qp >= SOMEWHAT_FLAT_QP ? dec_bit(S, 0, &err) : 0;
		int grp = dec_bits(S, 0, 2, &err);
		S->flat_target = g + 1 + grp;
		S->flat_type = type;
		S->pending_flag = 0;
	}

	int q[3], max_size[3], adj[3], size[3];
	for (int c = 0; c < 3; c++) {
		q[c] = qlevel(c, qp);
		max_size[c] = S->depth[c] - q[c];
		adj[c] = adj_pred_size(S, c, g, q[c]);
	}

	/* Luma prefix */
	int use_ich = 0;
	if (S->prev_ich) {
		if (dec_bit(S, 0, &err)) {
			use_ich = 1;
		} else {
			int limit = 1 + max_size[0] - adj[0];
			int z = 1;
			while (z < limit && !err) {
				if (dec_bit(S, 0, &err)) break;
				z++;
			}
			size[0] = adj[0] + z - 1;
		}
	} else {
		int limit = max_size[0] + 1 - adj[0];
		int z = 0;
		while (z < limit && !err) {
			if (dec_bit(S, 0, &err)) break;
			z++;
		}
		if (z == limit) use_ich = 1;
		else size[0] = adj[0] + z;
	}

	int px[3][3];
	int idx[3];
	int qe[3][3];
	int rc_size = 0;
	if (use_ich) {
		for (int k = 0; k < 3; k++) {
			idx[k] = dec_bits(S, k, 5, &err);
			if (!ich_entry_valid(S, line, idx[k])) err = 1;
			ich_value(S, line, hpos, idx[k], px[k]);
		}
		rc_size = 16;
	} else {
		for (int i = 0; i < 3; i++) qe[i][0] = dec_twos(S, 0, size[0], &err);
		for (int c = 1; c < 3; c++) {
			int limit = max_size[c] - adj[c];
			int z = 0;
			while (z < limit && !err) {
				if (dec_bit(S, c, &err)) break;
				z++;
			}
			size[c] = adj[c] + z;
			for (int i = 0; i < 3; i++) qe[i][c] = dec_twos(S, c, size[c], &err);
		}
		for (int c = 0; c < 3; c++) {
			int mpp = size[c] == max_size[c];
			if (mpp) {
				int pr = mpp_pred(S, c, q[c]);
				for (int i = 0; i < 3; i++) px[i][c] = CLAMP(pr + (qe[i][c] << q[c]), 0, S->maxval[c]);
				rc_size += 3 * max_size[c] + 1;
			} else {
				mmap_ctx m;
				mmap_setup(S, c, line, hpos, q[c], &m);
				int R[3] = { 0, 0, 0 };
				int maxrs = 0;
				for (int i = 0; i < 3; i++) {
					int pr = mmap_pred(S, c, &m, i, R);
					R[i] = qe[i][c] << q[c];
					px[i][c] = CLAMP(pr + R[i], 0, S->maxval[c]);
					maxrs = MAX(maxrs, res_size(qe[i][c]));
				}
				rc_size += 3 * maxrs + 1;
			}
		}
	}
	if (err) return -1;

	for (int i = 0; i < 3; i++)
		for (int c = 0; c < 3; c++) S->recon[c][line * S->w + hpos + i] = px[i][c];
	if (!use_ich) {
		for (int c = 0; c < 3; c++)
			for (int i = 0; i < 3; i++) S->req_size[c][i] = size[c] == max_size[c] ? max_size[c] : res_size(qe[i][c]);
	}
	if (gx != S->gpl - 1) {
		if (use_ich) ich_update_ich(S, line, hpos, idx);
		else ich_update_p(S, line, (const int (*)[3])px);
	}
	for (int c = 0; c < 3; c++) {
		S->prev_qlevel[c] = q[c];
		S->prev_recon[c] = px[2][c];
	}
	S->prev_ich = use_ich;

	int coded = 0;
	for (int c = 0; c < 3; c++) {
		int used = (int)(S->ss[c].rpos - start[c]);
		S->ss_fullness[c] -= used;
		coded += used;
	}
	rc_after_group(S, g, coded, rc_size);
	return 0;
}

/* ------------------------------------------------------- slices */

static int slice_alloc(slice_t *S, const dsc_pps *p, int encoding)
{
	memset(S, 0, sizeof *S);
	S->p = p;
	S->encoding = encoding;
	S->w = p->slice_width;
	S->h = p->slice_height;
	S->gpl = p->groups_per_line;
	S->total = p->groups_total;
	S->depth[0] = 8;
	S->depth[1] = S->depth[2] = 9;
	for (int c = 0; c < 3; c++) {
		S->maxval[c] = (1 << S->depth[c]) - 1;
		S->mid[c] = 1 << (S->depth[c] - 1);
		S->lb_shift[c] = MAX(0, S->depth[c] - p->linebuf_depth);
		S->recon[c] = malloc(sizeof(int) * (size_t)S->w * (size_t)S->h);
		if (!S->recon[c]) return -1;
		if (encoding) {
			S->se_size[c] = malloc(sizeof(int) * (size_t)S->total);
			if (!S->se_size[c]) return -1;
		}
	}
	S->st_qp = calloc((size_t)S->total + 2, sizeof(int));
	S->master_qp = calloc((size_t)S->total + 2, sizeof(int));
	if (!S->st_qp || !S->master_qp) return -1;
	return 0;
}

static void slice_reset(slice_t *S)
{
	memset(S->ich, 0, sizeof S->ich);
	memset(S->ich_valid, 0, sizeof S->ich_valid);
	memset(S->req_size, 0, sizeof S->req_size);
	memset(S->prev_qlevel, 0, sizeof S->prev_qlevel);
	S->prev_ich = 0;
	memset(S->prev_recon, 0, sizeof S->prev_recon);
	S->buffer_fullness = 0;
	S->pixel_count = 0;
	S->bpg_frac_accum = 0;
	S->xform_offset = (long long)(S->p->initial_offset - S->p->rc_model_size) << 11;
	S->offset_clamp = 0;
	S->scale = S->p->initial_scale_value;
	S->scale_counter = 0;
	S->scale_inc_start = 0;
	memset(S->st_qp, 0, sizeof(int) * ((size_t)S->total + 2));
	memset(S->master_qp, 0, sizeof(int) * ((size_t)S->total + 2));
	S->rc_size_prev = 0;
	S->force_mpp = 0;
	S->pending_flag = 0;
	S->flat_target = -1;
	S->flat_type = 0;
	S->prev_is_flat = 0;
	for (int c = 0; c < 3; c++) {
		S->ss[c].bits = 0;
		S->ss[c].rpos = 0;
		if (S->ss[c].buf) memset(S->ss[c].buf, 0, S->ss[c].cap);
		S->ss_fullness[c] = 0;
	}
}

static void slice_free(slice_t *S)
{
	for (int c = 0; c < 3; c++) {
		free(S->recon[c]);
		free(S->se_size[c]);
		free(S->ss[c].buf);
	}
	free(S->st_qp);
	free(S->master_qp);
}

static void rgb_to_ycocg(int r, int g, int b, int *y, int *co, int *cg)
{
	int csc_co = r - b;
	int t = b + (csc_co >> 1);
	int csc_cg = g - t;
	*y = t + (csc_cg >> 1);
	*co = csc_co + 256;
	*cg = csc_cg + 256;
}

static void ycocg_to_rgb(int y, int co, int cg, uint8_t *out)
{
	int csc_co = co - 256, csc_cg = cg - 256;
	int t = y - (csc_cg >> 1);
	int g = csc_cg + t;
	int b = t - (csc_co >> 1);
	int r = csc_co + b;
	out[0] = (uint8_t)CLAMP(r, 0, 255);
	out[1] = (uint8_t)CLAMP(g, 0, 255);
	out[2] = (uint8_t)CLAMP(b, 0, 255);
}

int dsc_encode_picture(const dsc_pps *p, const dsc_enc_opts *opts, const uint8_t *rgb, size_t stride, uint8_t *out,
                       uint8_t *recon_rgb, dsc_stats *stats)
{
	slice_t S;
	if (slice_alloc(&S, p, 1)) { slice_free(&S); return -1; }
	S.opts = opts;
	S.stats = stats;
	int w = p->slice_width, h = p->slice_height;
	int *orig[3];
	for (int c = 0; c < 3; c++) {
		orig[c] = malloc(sizeof(int) * (size_t)w * (size_t)h);
		S.orig[c] = orig[c];
	}
	size_t chunk_total = (size_t)p->chunk_size * h;
	int ret = 0;
	bitbuf mux = { 0 };

	for (int sl = 0; sl < p->slices_per_picture; sl++) {
		int y0 = sl * h;
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				int Y, Co, Cg;
				if (y0 + y < p->pic_height) {
					const uint8_t *s = rgb + (size_t)(y0 + y) * stride + (size_t)x * 3;
					rgb_to_ycocg(s[0], s[1], s[2], &Y, &Co, &Cg);
				} else {
					Y = 128; Co = 256; Cg = 256; /* Section 6.2 */
				}
				orig[0][y * w + x] = Y;
				orig[1][y * w + x] = Co;
				orig[2][y * w + x] = Cg;
			}
		}
		slice_reset(&S);
		for (int g = 0; g < S.total; g++) {
			if (g == 0) S.master_qp[0] = flat_override(&S, 0, S.st_qp[0]);
			enc_group(&S, g);
		}

		/* Substream multiplexing (Section 6.7 / Table 4-6) */
		mux.bits = 0;
		if (mux.buf) memset(mux.buf, 0, mux.cap);
		size_t pos[3] = { 0, 0, 0 };
		int full[3] = { 0, 0, 0 };
		for (int g = 0; g < S.total; g++) {
			for (int c = 0; c < 3; c++) {
				if (full[c] < MAX_SE) {
					bb_copy(&mux, S.ss[c].buf, S.ss[c].bits, pos[c], MUX_WORD);
					pos[c] += MUX_WORD;
					full[c] += MUX_WORD;
				}
			}
			for (int c = 0; c < 3; c++) full[c] -= S.se_size[c][g];
		}
		for (int c = 0; c < 3; c++)
			if (pos[c] < S.ss[c].bits) ret = -1; /* cannot happen if the model is right */
		if (stats && (long)mux.bits > stats->slice_bits_used_max) stats->slice_bits_used_max = (long)mux.bits;
		if (mux.bits > (size_t)p->slice_bits) {
			ret = -1;
			mux.bits = (size_t)p->slice_bits; /* truncated: picture will not decode correctly */
		}
		bb_reserve(&mux, (size_t)p->slice_bits);
		memcpy(out + (size_t)sl * chunk_total, mux.buf, chunk_total);
		if (mux.bits < (size_t)p->slice_bits) {
			/* zero padding up to sliceBits: buffer already zeroed past mux.bits except partial byte */
			size_t first = (mux.bits + 7) / 8;
			if (first < chunk_total) memset(out + (size_t)sl * chunk_total + first, 0, chunk_total - first);
		}

		if (recon_rgb) {
			for (int y = 0; y < h && y0 + y < p->pic_height; y++)
				for (int x = 0; x < w; x++)
					ycocg_to_rgb(S.recon[0][y * w + x], S.recon[1][y * w + x], S.recon[2][y * w + x],
					             recon_rgb + ((size_t)(y0 + y) * w + x) * 3);
		}
	}
	free(mux.buf);
	for (int c = 0; c < 3; c++) free(orig[c]);
	slice_free(&S);
	return ret;
}

int dsc_decode_picture(const dsc_pps *p, const uint8_t *in, size_t inlen, uint8_t *rgb)
{
	if (inlen < dsc_picture_bytes(p)) return -1;
	slice_t S;
	if (slice_alloc(&S, p, 0)) { slice_free(&S); return -1; }
	int w = p->slice_width, h = p->slice_height;
	size_t chunk_total = (size_t)p->chunk_size * h;
	int ret = 0;
	for (int sl = 0; sl < p->slices_per_picture && !ret; sl++) {
		slice_reset(&S);
		const uint8_t *sd = in + (size_t)sl * chunk_total;
		size_t mux_pos = 0;
		for (int g = 0; g < S.total; g++) {
			if (g == 0) S.master_qp[0] = flat_override(&S, 0, S.st_qp[0]);
			if (dec_group(&S, g, sd, (size_t)p->slice_bits, &mux_pos)) { ret = -1; break; }
		}
		int y0 = sl * h;
		for (int y = 0; y < h && y0 + y < p->pic_height; y++)
			for (int x = 0; x < w; x++)
				ycocg_to_rgb(S.recon[0][y * w + x], S.recon[1][y * w + x], S.recon[2][y * w + x],
				             rgb + ((size_t)(y0 + y) * w + x) * 3);
	}
	slice_free(&S);
	return ret;
}
