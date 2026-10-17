// dsc_decoder.sv - VESA DSC decoder for .dscmmi video, one core for 8, 9
// and 10 bits per pixel (any rate the PPS carries, 6 to 12 bpp tested).
//
// Scope matches src/dsc.c, which is the bit-exact reference model:
//   - DSC 1.1 bitstream semantics, 8 bpc RGB (YCoCg-R), 4:4:4, CBR
//   - one slice per line (slice_width == pic_width), slice_width % 3 == 0
//   - block prediction off, linebuf_depth >= 9 (no line-storage rounding)
//
// The rate is not a build option: bits_per_pixel and every rate-control
// parameter are read from the 128-byte PPS that the host writes through
// pps_we/pps_addr/pps_data before pulsing start.
//
// Input: the frame's bytes in order, packed little-endian into 64-bit words
// (byte 0 of the frame in in_data[7:0]); exactly ceil(frame_bytes / 8) words
// per frame. The decoder stops accepting words at the end of the frame, so
// the next frame's words can already be waiting.
//
// Output: one beat per group of three pixels, (out_x, out_y) = the leftmost
// pixel, out_rgb0..2 = {R, G, B} left to right. Rows past pic_height (the
// last slice's padding) are not output. Hold-off with out_ready.
//
// Throughput: 9 clocks per group, plus 1 per 48-bit mux word and 4 per line.
// Measured: 9.53 (8 bpp), 9.61 (9 bpp), 9.67 (10 bpp) clocks per group. A
// 792x525 frame is 138,600 groups, about 1.34 M clocks, so 29.97 frames/s
// needs about 40 MHz.
//
// SPDX-License-Identifier: MIT
`default_nettype none

module dsc_decoder #(
	parameter LB_AW = 10                // line buffer holds 2^LB_AW groups (3072 px)
) (
	input  wire        clk,
	input  wire        reset,

	input  wire        pps_we,
	input  wire  [6:0] pps_addr,
	input  wire  [7:0] pps_data,

	input  wire        start,            // begin a frame (uses the PPS as written)
	output reg         busy,
	output reg         done,             // one-clock pulse at the end of a frame
	output reg         error,            // sticky until the next start: bad PPS or bad stream

	input  wire [63:0] in_data,
	input  wire        in_valid,
	output wire        in_ready,

	output reg         out_valid,
	input  wire        out_ready,
	output reg  [15:0] out_x,
	output reg  [15:0] out_y,
	output reg  [23:0] out_rgb0,
	output reg  [23:0] out_rgb1,
	output reg  [23:0] out_rgb2
);

// ------------------------------------------------------------------ PPS
reg [7:0] pps [0:127];
always @(posedge clk) if (pps_we) pps[pps_addr] <= pps_data;

wire  [9:0] bpp        = {pps[4][1:0], pps[5]};
wire [15:0] pic_h      = {pps[6], pps[7]};
wire [15:0] slice_h    = {pps[10], pps[11]};
wire [15:0] slice_w    = {pps[12], pps[13]};
wire [15:0] chunk      = {pps[14], pps[15]};
wire  [9:0] ixd        = {pps[16][1:0], pps[17]};
wire  [5:0] init_scale = pps[21][5:0];
wire [15:0] sii        = {pps[22], pps[23]};
wire [11:0] sdi        = {pps[24][3:0], pps[25]};
wire  [4:0] flbo       = pps[27][4:0];
wire [15:0] nfl        = {pps[28], pps[29]};
wire [15:0] sbo        = {pps[30], pps[31]};
wire [15:0] init_off   = {pps[32], pps[33]};
wire [15:0] final_off  = {pps[34], pps[35]};
wire  [4:0] fminqp     = pps[36][4:0];
wire  [4:0] fmaxqp     = pps[37][4:0];
wire [15:0] rc_model   = {pps[38], pps[39]};
wire  [3:0] edge_f     = pps[40][3:0];
wire  [4:0] qil0       = pps[41][4:0];
wire  [4:0] qil1       = pps[42][4:0];
wire  [3:0] tgt_hi     = pps[43][7:4];
wire  [3:0] tgt_lo     = pps[43][3:0];
wire  [4:0] maxqp14    = {pps[86][2:0], pps[87][7:6]};  // range_max_qp[14]

function automatic integer thresh(input integer k);   // rc_buf_thresh[k]
	thresh = {pps[44 + k], 6'd0};
endfunction
function automatic integer rminqp(input integer k);
	rminqp = pps[58 + 2 * k][7:3];
endfunction
function automatic integer rmaxqp(input integer k);
	rmaxqp = {pps[58 + 2 * k][2:0], pps[59 + 2 * k][7:6]};
endfunction
function automatic integer rofs(input integer k);      // signed 6 bits
	rofs = $signed(pps[59 + 2 * k][5:0]);
endfunction

wire pps_ok = pps[0] == 8'h11 &&                       // DSC 1.1
              pps[3][7:4] == 4'd8 &&                   // 8 bpc
              (pps[3][3:0] >= 4'd9 || pps[3][3:0] == 4'd0) &&
              pps[4][5:2] == 4'b0100 &&                // no block pred, RGB, no 4:2:2, CBR
              slice_w == {pps[8], pps[9]} &&           // one slice per line
              slice_w >= 16'd9 && slice_h != 16'd0 &&
              slice_w <= 16'd3 << LB_AW;

// ------------------------------------------------------------ tables
function automatic integer qlevel(input integer c, input integer qp);
	// Table 6-2, 8 bpc
	reg [3:0] q;
	begin
		q = qp;
		if (c == 0)
			case (q)
				4'd0, 4'd1, 4'd2: qlevel = 0;
				4'd3, 4'd4:       qlevel = 1;
				4'd5, 4'd6:       qlevel = 2;
				4'd7, 4'd8:       qlevel = 3;
				4'd9, 4'd10:      qlevel = 4;
				4'd11, 4'd12, 4'd13: qlevel = 5;
				4'd14:            qlevel = 6;
				default:          qlevel = 7;
			endcase
		else
			case (q)
				4'd0:             qlevel = 0;
				4'd1:             qlevel = 1;
				4'd2, 4'd3:       qlevel = 2;
				4'd4, 4'd5:       qlevel = 3;
				4'd6, 4'd7:       qlevel = 4;
				4'd8, 4'd9:       qlevel = 5;
				4'd10, 4'd11:     qlevel = 6;
				4'd12:            qlevel = 7;
				default:          qlevel = 8;
			endcase
	end
endfunction

function automatic integer lzc11(input [10:0] v);    // leading zeros, 11 if none
	integer i;
	begin
		lzc11 = 11;
		for (i = 0; i <= 10; i = i + 1) if (v[i]) lzc11 = 10 - i;
	end
endfunction

function automatic integer res_size(input integer v); // Table 3-1
	integer a, n, i;
	begin
		a = v < 0 ? -v - 1 : v;
		n = 0;
		for (i = 0; i < 10; i = i + 1) if (a >= (1 << i)) n = i + 1;
		res_size = v == 0 ? 0 : n + 1;
	end
endfunction

// field k (0..2) of size s (0..10) from the top of a substream buffer, signed
function automatic integer field(input [95:0] s, input integer k, input integer sz);
	reg [95:0] t;
	integer v;
	begin
		t = s << (k * sz);
		v = t[95:86] >> (10 - sz);
		if (sz != 0 && v[sz - 1]) v = v - (1 << sz);
		field = sz == 0 ? 0 : v;
	end
endfunction

function automatic integer clampi(input integer v, input integer lo, input integer hi);
	clampi = v < lo ? lo : v > hi ? hi : v;
endfunction
function automatic integer mini(input integer a, input integer b);
	mini = a < b ? a : b;
endfunction
function automatic integer maxi(input integer a, input integer b);
	maxi = a > b ? a : b;
endfunction

// pixel = {Y[7:0], Co[8:0], Cg[8:0]}
function automatic integer comp(input [25:0] p, input integer c);
	comp = c == 0 ? p[25:18] : c == 1 ? p[17:9] : p[8:0];
endfunction

function automatic [23:0] to_rgb(input [25:0] p);
	integer y, co, cg, t, r, g, b;
	reg [7:0] r8, g8, b8;
	begin
		y = p[25:18]; co = p[17:9] - 256; cg = p[8:0] - 256;
		t = y - (cg >>> 1);
		g = cg + t;
		b = t - (co >>> 1);
		r = co + b;
		r8 = clampi(r, 0, 255);
		g8 = clampi(g, 0, 255);
		b8 = clampi(b, 0, 255);
		to_rgb = {r8, g8, b8};
	end
endfunction

// ------------------------------------------------------------ state
localparam S_IDLE = 4'd0, S_SLICE = 4'd1, S_LP = 4'd2, S_MUX = 4'd3, S_FLAT = 4'd4, S_PFX = 4'd5,
           S_RES = 4'd6, S_CRES = 4'd7, S_REC1 = 4'd8, S_REC2 = 4'd9, S_RC1 = 4'd10, S_RC2 = 4'd11,
           S_SKIP = 4'd12;
reg  [3:0] state;

reg [15:0] gpl;           // groups per line
reg [31:0] slice_bits;
reg [15:0] y0;            // first row of this slice
reg [15:0] line;          // row within slice
reg [15:0] gx;            // group within row
reg  [1:0] g4;            // group index within slice, mod 4
reg        g0;            // first group of the slice
reg  [1:0] lp_cnt;

wire [15:0] hpos       = gx * 16'd3;
wire        first_line = line == 16'd0;
wire        last_gx    = gx == gpl - 16'd1;

// --------------------------------------------------- input bit buffer
reg [127:0] bb;           // MSB is the next bit
reg   [7:0] bb_cnt;
reg  [31:0] fpos;         // bits fetched this frame
reg  [31:0] fetch_lim;    // stop fetching here (end of the last slice)
reg  [31:0] mux_pos;      // bits demultiplexed in this slice
reg  [31:0] slice_start;  // frame bit offset of this slice
reg  [31:0] skip_rem;
reg   [7:0] bb_take;      // comb: bits consumed this clock

wire [127:0] bb_c  = bb << bb_take;
wire   [7:0] cnt_c = bb_cnt - bb_take;
wire         active = state != S_IDLE;
assign in_ready = active && cnt_c <= 8'd64 && fpos < fetch_lim;
wire  [63:0] w_be = {in_data[7:0], in_data[15:8], in_data[23:16], in_data[31:24],
                     in_data[39:32], in_data[47:40], in_data[55:48], in_data[63:56]};

// ---------------------------------------------------------- substreams
reg [95:0] ss0, ss1, ss2;  // MSB first
reg  [6:0] sf0, sf1, sf2;  // bits held
reg  [6:0] coded;          // bits consumed by this group

// ------------------------------------------------------- group state
reg  [3:0] mq;             // masterQp of this group
reg  [3:0] st1;            // stQp of the next group
reg  [3:0] prev_q [0:2];
reg  [3:0] rs [0:2][0:2];  // req_size[c][i]
reg        prev_ich;
reg  [8:0] prev_recon [0:2];
reg [25:0] ich [0:31];
reg [31:0] ich_v;
reg        fl_pending, fl_tvalid, fl_type;
reg  [1:0] fl_cnt;

reg        use_ich;
reg  [3:0] size [0:2];
reg  [4:0] idx [0:2];
reg signed [9:0] qe [0:2][0:2];  // [c][i]

reg  [9:0] bl_a [0:2], bl_b [0:2], bl_c [0:2], bl_d [0:2], bl_e [0:2];
reg [25:0] ichpx [0:2];
reg [25:0] px [0:2];       // reconstructed pixels of this group
reg  [6:0] rc_size;

// rate control
reg signed [19:0] buf_full;
reg        [31:0] pix_cnt;
reg         [4:0] frac;
reg               mod_act;
reg        [15:0] mod_cnt;
reg signed [31:0] xoff;
reg               off_clamp;
reg         [6:0] scale;
reg        [15:0] scnt;
reg               sis;
reg         [6:0] rc_size_prev;
reg signed [19:0] model_r;
reg signed [19:0] ofs_r;

// ---------------------------------------------------------- line buffer
reg [77:0] lb [0:(1 << LB_AW) - 1];
reg [77:0] lb_rd, lb_next;
reg        lb_we;
reg [LB_AW-1:0] lb_wa;
reg [77:0] lb_wd;
wire [LB_AW-1:0] lb_ra = state == S_LP ? {{(LB_AW-2){1'b0}}, lp_cnt} : gx[LB_AW-1:0] + 3'd3;
always @(posedge clk) begin
	if (lb_we) lb[lb_wa] <= lb_wd;
	lb_rd <= lb[lb_ra];
end
reg [25:0] win [0:14];     // previous line, x = hpos - 6 .. hpos + 8

// ======================================================= combinational
// quantisation for this group
integer q [0:2], ms [0:2], adj [0:2];
always @* begin : qcalc
	integer c, ps;
	for (c = 0; c < 3; c = c + 1) begin
		q[c] = qlevel(c, mq);
		ms[c] = (c == 0 ? 8 : 9) - q[c];
		ps = (rs[c][0] + rs[c][1] + 2 * rs[c][2] + 2) >> 2;
		adj[c] = g0 ? 0 : clampi(ps - (q[c] - prev_q[c]), 0, ms[c] - 1);
	end
end

// integer copies, so mixed expressions stay signed
integer I_rcm, I_tlo, I_thi, BF, XO, MR, OF, I_coded, RCS;
always @* begin
	I_rcm = rc_model; I_tlo = tgt_lo; I_thi = tgt_hi;
	BF = buf_full; XO = xoff; MR = model_r; OF = ofs_r;
	I_coded = coded; RCS = rc_size;
end

// MUX: substream needing a word (in order 0, 1, 2)
wire need0 = sf0 < 7'd36, need1 = sf1 < 7'd36, need2 = sf2 < 7'd36;

// FLAT: flatness bits on substream 0
integer fl_n, fl_grp, fl_t;
reg fl_set, fl_pend_n;
always @* begin
	fl_n = 0; fl_grp = 0; fl_t = 0; fl_set = 0; fl_pend_n = fl_pending;
	if (g4 == 2'd3) begin
		fl_pend_n = 0;
		if (mq >= fminqp && mq <= fmaxqp) begin fl_pend_n = ss0[95]; fl_n = 1; end
	end else if (g4 == 2'd0 && fl_pending) begin
		fl_set = 1;
		fl_pend_n = 0;
		if (mq >= 4'd7) begin fl_t = ss0[95]; fl_grp = ss0[94:93]; fl_n = 3; end
		else begin fl_grp = ss0[95:94]; fl_n = 2; end
	end
end

// PFX: luma prefix
integer px_n, px_size;
reg     px_ich;
always @* begin : pfx
	integer L, lim;
	L = 0; lim = 0;
	px_ich = 0; px_size = 0; px_n = 0;
	if (prev_ich) begin
		if (ss0[95]) begin px_ich = 1; px_n = 1; end
		else begin
			lim = ms[0] - adj[0];
			L = lzc11({ss0[94:85], 1'b1});
			if (L < lim) begin px_size = adj[0] + L; px_n = L + 2; end
			else begin px_size = adj[0] + lim; px_n = 1 + lim; end
		end
	end else begin
		lim = ms[0] + 1 - adj[0];
		L = lzc11(ss0[95:85]);
		if (L < lim) begin px_size = adj[0] + L; px_n = L + 1; end
		else begin px_ich = 1; px_n = lim; end
	end
end

// RES: chroma prefixes (P mode)
integer cp_size [1:2], cp_n [1:2];
always @* begin : cpfx
	integer c, L, lim;
	reg [95:0] s;
	for (c = 1; c <= 2; c = c + 1) begin
		s = c == 1 ? ss1 : ss2;
		lim = ms[c] - adj[c];
		L = lzc11(s[95:85]);
		if (L < lim) begin cp_size[c] = adj[c] + L; cp_n[c] = L + 1; end
		else begin cp_size[c] = ms[c]; cp_n[c] = lim; end
	end
end

// REC1: blends and ICH values
integer srel;
always @* begin
	if (gx == 16'd0) srel = 0;
	else if (last_gx) srel = -4;
	else srel = -2;
end

// REC2: prediction and reconstruction
integer rec [0:2][0:2];   // [i][c]
integer rcs_c;
always @* begin : recon
	integer c, i, a, b, cc, d, e, R0, R1, pr, mpp_pr, mx, maxrs;
	a = 0; b = 0; cc = 0; d = 0; e = 0; R0 = 0; R1 = 0; pr = 0; mpp_pr = 0; mx = 0; maxrs = 0;
	rcs_c = use_ich ? 16 : 0;
	for (c = 0; c < 3; c = c + 1) begin
		mx = c == 0 ? 255 : 511;
		a = bl_a[c]; b = bl_b[c]; cc = bl_c[c]; d = bl_d[c]; e = bl_e[c];
		R0 = qe[c][0] * (1 << q[c]);
		R1 = qe[c][1] * (1 << q[c]);
		if (use_ich) begin
			for (i = 0; i < 3; i = i + 1) rec[i][c] = comp(ichpx[i], c);
		end else if (size[c] == ms[c]) begin
			mpp_pr = (c == 0 ? 128 : 256) + (prev_recon[c] & ((1 << q[c]) - 1));
			for (i = 0; i < 3; i = i + 1) rec[i][c] = clampi(mpp_pr + qe[c][i] * (1 << q[c]), 0, mx);
			rcs_c = rcs_c + 3 * ms[c] + 1;
		end else begin
			if (first_line) begin
				rec[0][c] = clampi(a + R0, 0, mx);
				pr = clampi(a + R0, 0, mx);
				rec[1][c] = clampi(pr + R1, 0, mx);
				pr = clampi(a + R0 + R1, 0, mx);
				rec[2][c] = clampi(pr + qe[c][2] * (1 << q[c]), 0, mx);
			end else begin
				pr = clampi(a + b - cc, mini(a, b), maxi(a, b));
				rec[0][c] = clampi(pr + R0, 0, mx);
				pr = clampi(a + d - cc + R0, mini(a, mini(b, d)), maxi(a, maxi(b, d)));
				rec[1][c] = clampi(pr + R1, 0, mx);
				pr = clampi(a + e - cc + R0 + R1, mini(mini(a, b), mini(d, e)), maxi(maxi(a, b), maxi(d, e)));
				rec[2][c] = clampi(pr + qe[c][2] * (1 << q[c]), 0, mx);
			end
			maxrs = 0;
			for (i = 0; i < 3; i = i + 1) maxrs = maxi(maxrs, res_size(qe[c][i]));
			rcs_c = rcs_c + 3 * maxrs + 1;
		end
	end
end

// RC1 (ICH history update + buffer model)
reg [25:0] ich_n [0:31];
reg [31:0] ich_vn;
always @* begin : ichupd
	integer i, j, t, nsel, s0, s1, s2, tmp, h0, h1, h2;
	reg keep0, keep1;
	reg [25:0] f [0:2];
	t = 0; j = 0; tmp = 0;
	for (i = 0; i < 32; i = i + 1) begin ich_n[i] = ich[i]; ich_vn[i] = ich_v[i]; end
	keep0 = use_ich ? (idx[0] != idx[1] && idx[0] != idx[2]) : 1'b1;
	keep1 = use_ich ? (idx[1] != idx[2]) : 1'b1;
	// front of the history: rightmost pixel first, duplicates dropped
	f[0] = px[2];
	f[1] = keep1 ? px[1] : px[0];
	f[2] = px[0];
	nsel = 1 + (keep1 ? 1 : 0) + (keep0 ? 1 : 0);
	// history entries the group used (they move to the front)
	h0 = (use_ich && keep0 && (first_line || idx[0] < 25)) ? idx[0] : 63;
	h1 = (use_ich && keep1 && (first_line || idx[1] < 25)) ? idx[1] : 63;
	h2 = (use_ich && (first_line || idx[2] < 25)) ? idx[2] : 63;
	s0 = h0; s1 = h1; s2 = h2;
	if (s0 > s1) begin tmp = s0; s0 = s1; s1 = tmp; end
	if (s1 > s2) begin tmp = s1; s1 = s2; s2 = tmp; end
	if (s0 > s1) begin tmp = s0; s0 = s1; s1 = tmp; end
	for (i = 0; i < 32; i = i + 1) begin
		if (i < nsel) begin
			// (constant indices only: Quartus checks every unrolled index)
			ich_n[i] = i == 0 ? f[0] : i == 1 ? f[1] : f[2];
			ich_vn[i] = 1'b1;
		end else begin
			t = i - nsel;
			j = t;
			if (j >= s0) j = j + 1;
			if (j >= s1) j = j + 1;
			if (j >= s2) j = j + 1;
			// j is i-3 .. i: a 4-way choice, not a 32-way one
			case (i - j)
			// "& 31" keeps the unrolled index in range; the i >= n guards keep the meaning
			0: begin ich_n[i] = ich[i]; ich_vn[i] = ich_v[i]; end
			1: if (i >= 1) begin ich_n[i] = ich[(i - 1) & 31]; ich_vn[i] = ich_v[(i - 1) & 31]; end
			2: if (i >= 2) begin ich_n[i] = ich[(i - 2) & 31]; ich_vn[i] = ich_v[(i - 2) & 31]; end
			default: if (i >= 3) begin ich_n[i] = ich[(i - 3) & 31]; ich_vn[i] = ich_v[(i - 3) & 31]; end
			endcase
		end
	end
end

integer bf_n, rem_n, pc_n, acc_n, mc_n, sc_n, scn_n, xo_n, ofs_n, model_n;
reg ma_n, sis_n, cl_n;
always @* begin : rc1
	integer i, inc, n, fl;
	n = 0; fl = 0; inc = 0;
	pc_n = pix_cnt; acc_n = frac; rem_n = 0; ma_n = mod_act; mc_n = mod_cnt;
	for (i = 0; i < 3; i = i + 1) begin
		pc_n = pc_n + 1;
		if (pc_n >= ixd) begin
			acc_n = acc_n + bpp[3:0];
			rem_n = rem_n + bpp[9:4] + (acc_n >> 4);
			acc_n = acc_n & 15;
		end
		if (pc_n == ixd) begin ma_n = 1; mc_n = 0; end
		else if (ma_n) mc_n = (mc_n == slice_w - 1) ? 0 : mc_n + 1;
		if (ma_n && mc_n == 0) acc_n = 0;
	end
	bf_n = BF + I_coded - rem_n;

	sc_n = scale; scn_n = scnt; sis_n = sis;
	if (g0) begin sc_n = init_scale; scn_n = 0; end
	else if (!sis && scale > 8) begin
		scn_n = scn_n + 1;
		if (scn_n >= sdi) begin scn_n = 0; sc_n = sc_n - 1; end
	end else if (sis) begin
		scn_n = scn_n + 1;
		if (scn_n >= sii) begin scn_n = 0; sc_n = sc_n + 1; end
	end
	inc = first_line ? -(flbo * 2048) : nfl;
	if (pix_cnt < ixd) begin
		n = mini(3, ixd - pix_cnt);
		inc = inc - bpp * n * 128;
	end else if (sii != 0 && !sis && !first_line && XO > -(I_rcm * 2048)) begin
		sc_n = 9; scn_n = 0; sis_n = 1;
	end
	inc = inc + sbo;
	xo_n = xoff + inc;
	fl = (final_off - rc_model) * 2048;
	cl_n = off_clamp;
	if (xo_n < fl) cl_n = 1;
	if (cl_n && xo_n > fl) xo_n = fl;
	ofs_n = xo_n >>> 11;
	model_n = (sc_n * (bf_n + ofs_n)) >>> 3;
end

// RC2: range, flatness override, short-term QP
integer mq_n, st_n;
reg tv_n;
reg [1:0] fc_n;
always @* begin : rc2
	integer k, r, minq, maxq, ro, bpgo, tgt, prev_qp, cur, incr;
	reg edge_c;
	cur = 0; incr = 0;
	r = 14;
	for (k = 13; k >= 0; k = k - 1) if (MR <= thresh(k) - I_rcm) r = k;
	minq = rminqp(r); maxq = rmaxqp(r); ro = rofs(r);

	mq_n = st1; tv_n = fl_tvalid; fc_n = fl_cnt;
	if (fl_tvalid) begin
		if (fl_cnt == 2'd0) begin
			if (st1 != maxqp14) mq_n = fl_type ? 1 : maxi(st1 - 4, 0);
			tv_n = 0;
		end else fc_n = fl_cnt - 2'd1;
	end
	mq_n = clampi(mq_n, 0, 15);

	bpgo = first_line ? flbo : -(nfl >> 11);
	bpgo = bpgo - (sbo >> 11);
	tgt = ((3 * bpp + 8) >> 4) + ro + bpgo;
	prev_qp = mq_n;
	edge_c = rc_size * 2 < rc_size_prev * edge_f;
	if (BF + OF > -172) st_n = maxqp14;
	else if (RCS == 3) st_n = maxi(prev_qp - 1, minq >> 1);
	else if (I_coded < tgt - I_tlo && RCS < tgt - I_tlo) st_n = maxi(prev_qp - 1, minq);
	else if (I_coded > tgt + I_thi && BF >= 64) begin
		cur = maxi(minq, prev_qp);
		incr = (I_coded - tgt) >> 1;
		if (cur == mq) st_n = edge_c ? mini(maxq, cur + incr) : cur;
		else if (cur < mq) st_n = (edge_c && cur < qil0) ? mini(maxq, cur + incr) : cur;
		else if (cur < qil1) st_n = mini(maxq, cur + incr);
		else st_n = cur;
	end else st_n = prev_qp;
	st_n = clampi(st_n, 0, 15);
end

// bits consumed from the input buffer this clock
always @* begin
	bb_take = 8'd0;
	if (state == S_MUX && ((need0 || need1 || need2) && bb_cnt >= 8'd48)) bb_take = 8'd48;
	else if (state == S_SKIP) bb_take = skip_rem < {24'd0, bb_cnt} ? skip_rem[7:0] : bb_cnt;
end

// ============================================================ sequential
integer c, i, k;
always @(posedge clk) begin
	done <= 1'b0;
	lb_we <= 1'b0;
	if (out_valid && out_ready) out_valid <= 1'b0;

	// input buffer (consume, then refill)
	if (in_valid && in_ready) begin
		bb <= bb_c | ({64'd0, w_be} << (8'd64 - cnt_c));
		bb_cnt <= cnt_c + 8'd64;
		fpos <= fpos + 32'd64;
	end else begin
		bb <= bb_c;
		bb_cnt <= cnt_c;
	end

	case (state)
	S_IDLE: ;

	S_SLICE: begin
		// slice_reset()
		for (k = 0; k < 32; k = k + 1) ich[k] <= 26'd0;
		ich_v <= 32'd0;
		for (c = 0; c < 3; c = c + 1) begin
			prev_q[c] <= 4'd0;
			prev_recon[c] <= 9'd0;
			for (i = 0; i < 3; i = i + 1) rs[c][i] <= 4'd0;
		end
		prev_ich <= 1'b0;
		buf_full <= 20'sd0;
		pix_cnt <= 32'd0;
		frac <= 5'd0;
		mod_act <= ixd == 10'd0;
		mod_cnt <= 16'd0;
		xoff <= ($signed({16'd0, init_off}) - $signed({16'd0, rc_model})) * 2048;
		off_clamp <= 1'b0;
		scale <= {1'b0, init_scale};
		scnt <= 16'd0;
		sis <= 1'b0;
		mq <= 4'd0;
		st1 <= 4'd0;
		rc_size_prev <= 7'd0;
		fl_pending <= 1'b0;
		fl_tvalid <= 1'b0;
		fl_type <= 1'b0;
		fl_cnt <= 2'd0;
		ss0 <= 96'd0; ss1 <= 96'd0; ss2 <= 96'd0;
		sf0 <= 7'd0; sf1 <= 7'd0; sf2 <= 7'd0;
		mux_pos <= 32'd0;
		line <= 16'd0;
		gx <= 16'd0;
		g4 <= 2'd0;
		g0 <= 1'b1;
		lp_cnt <= 2'd0;
		fetch_lim <= y0 + slice_h >= pic_h ? slice_start + slice_bits : 32'hFFFFFFFF;
		state <= S_LP;
	end

	// load the previous line's groups 0..2 into the window
	S_LP: begin
		lp_cnt <= lp_cnt + 2'd1;
		if (lp_cnt == 2'd1) begin
			for (k = 0; k < 6; k = k + 1) win[k] <= lb_rd[77:52];
			win[6] <= lb_rd[77:52]; win[7] <= lb_rd[51:26]; win[8] <= lb_rd[25:0];
		end else if (lp_cnt == 2'd2) begin
			win[9] <= lb_rd[77:52]; win[10] <= lb_rd[51:26]; win[11] <= lb_rd[25:0];
		end else if (lp_cnt == 2'd3) begin
			win[12] <= lb_rd[77:52]; win[13] <= lb_rd[51:26]; win[14] <= lb_rd[25:0];
			state <= S_MUX;
		end
	end

	// substream demultiplexer (Table 4-6)
	S_MUX: begin
		coded <= 7'd0;
		if (need0 || need1 || need2) begin
			if (bb_cnt >= 8'd48) begin
				if (mux_pos + 32'd48 > slice_bits) error <= 1'b1;
				mux_pos <= mux_pos + 32'd48;
				if (need0) begin ss0 <= ss0 | ({bb[127:80], 48'd0} >> sf0); sf0 <= sf0 + 7'd48; end
				else if (need1) begin ss1 <= ss1 | ({bb[127:80], 48'd0} >> sf1); sf1 <= sf1 + 7'd48; end
				else begin ss2 <= ss2 | ({bb[127:80], 48'd0} >> sf2); sf2 <= sf2 + 7'd48; end
			end
		end else state <= S_FLAT;
	end

	S_FLAT: begin
		lb_next <= lb_rd;  // previous line, group gx + 3
		fl_pending <= fl_pend_n;
		if (fl_set) begin
			fl_tvalid <= 1'b1;
			fl_type <= fl_t[0];
			fl_cnt <= fl_grp[1:0];
		end
		ss0 <= ss0 << fl_n;
		sf0 <= sf0 - fl_n[6:0];
		coded <= fl_n[6:0];
		state <= S_PFX;
	end

	S_PFX: begin
		use_ich <= px_ich;
		size[0] <= px_size[3:0];
		ss0 <= ss0 << px_n;
		sf0 <= sf0 - px_n[6:0];
		if (sf0 < px_n) error <= 1'b1;
		coded <= coded + px_n[6:0];
		state <= S_RES;
	end

	S_RES: begin
		if (use_ich) begin
			idx[0] <= ss0[95:91]; idx[1] <= ss1[95:91]; idx[2] <= ss2[95:91];
			ss0 <= ss0 << 5; ss1 <= ss1 << 5; ss2 <= ss2 << 5;
			sf0 <= sf0 - 7'd5; sf1 <= sf1 - 7'd5; sf2 <= sf2 - 7'd5;
			if (sf0 < 7'd5 || sf1 < 7'd5 || sf2 < 7'd5) error <= 1'b1;
			coded <= coded + 7'd15;
		end else begin
			for (i = 0; i < 3; i = i + 1) qe[0][i] <= field(ss0, i, size[0]);
			ss0 <= ss0 << (3 * size[0]);
			sf0 <= sf0 - 7'd3 * size[0];
			if (sf0 < 7'd3 * size[0]) error <= 1'b1;
			size[1] <= cp_size[1][3:0];
			size[2] <= cp_size[2][3:0];
			ss1 <= ss1 << cp_n[1];
			ss2 <= ss2 << cp_n[2];
			sf1 <= sf1 - cp_n[1][6:0];
			sf2 <= sf2 - cp_n[2][6:0];
			coded <= coded + 7'd3 * size[0] + cp_n[1][6:0] + cp_n[2][6:0];
		end
		state <= S_CRES;
	end

	S_CRES: begin
		if (!use_ich) begin
			for (i = 0; i < 3; i = i + 1) begin
				qe[1][i] <= field(ss1, i, size[1]);
				qe[2][i] <= field(ss2, i, size[2]);
			end
			ss1 <= ss1 << (3 * size[1]);
			ss2 <= ss2 << (3 * size[2]);
			sf1 <= sf1 - 7'd3 * size[1];
			sf2 <= sf2 - 7'd3 * size[2];
			if (sf1 < 7'd3 * size[1] || sf2 < 7'd3 * size[2]) error <= 1'b1;
			coded <= coded + 7'd3 * size[1] + 7'd3 * size[2];
		end
		state <= S_REC1;
	end

	S_REC1: begin : rec1
		integer cc2, ccv, b, d, e, f, lim, mid, fc, fb, fd, fe, wi;
		for (c = 0; c < 3; c = c + 1) begin
			mid = c == 0 ? 128 : 256;
			cc2 = comp(win[4], c); ccv = comp(win[5], c); b = comp(win[6], c);
			d = comp(win[7], c); e = comp(win[8], c); f = comp(win[9], c);
			fc = (cc2 + 2 * ccv + b + 2) >> 2;
			fb = (ccv + 2 * b + d + 2) >> 2;
			fd = (b + 2 * d + e + 2) >> 2;
			fe = (d + 2 * e + f + 2) >> 2;
			lim = (1 << q[c]) >> 1;
			bl_a[c] <= gx == 16'd0 ? mid[9:0] : {1'b0, prev_recon[c]};
			bl_b[c] <= b + clampi(fb - b, -lim, lim);
			bl_c[c] <= gx == 16'd0 ? mid : ccv + clampi(fc - ccv, -lim, lim);
			bl_d[c] <= d + clampi(fd - d, -lim, lim);
			bl_e[c] <= e + clampi(fe - e, -lim, lim);
		end
		for (k = 0; k < 3; k = k + 1) begin
			if (!first_line && idx[k] >= 5'd25) begin
				wi = idx[k] - 19 + srel;
				ichpx[k] <= win[wi];
			end else begin
				ichpx[k] <= ich[idx[k]];
				if (use_ich && !ich_v[idx[k]]) error <= 1'b1;
			end
		end
		state <= S_REC2;
	end

	S_REC2: begin
		if (!(out_valid && !out_ready)) begin
			for (i = 0; i < 3; i = i + 1)
				px[i] <= {rec[i][0][7:0], rec[i][1][8:0], rec[i][2][8:0]};
			rc_size <= rcs_c[6:0];
			lb_we <= 1'b1;
			lb_wa <= gx[LB_AW-1:0];
			lb_wd <= {rec[0][0][7:0], rec[0][1][8:0], rec[0][2][8:0],
			          rec[1][0][7:0], rec[1][1][8:0], rec[1][2][8:0],
			          rec[2][0][7:0], rec[2][1][8:0], rec[2][2][8:0]};
			if (y0 + line < pic_h) begin
				out_valid <= 1'b1;
				out_x <= hpos;
				out_y <= y0 + line;
				out_rgb0 <= to_rgb({rec[0][0][7:0], rec[0][1][8:0], rec[0][2][8:0]});
				out_rgb1 <= to_rgb({rec[1][0][7:0], rec[1][1][8:0], rec[1][2][8:0]});
				out_rgb2 <= to_rgb({rec[2][0][7:0], rec[2][1][8:0], rec[2][2][8:0]});
			end
			for (c = 0; c < 3; c = c + 1) begin
				if (!use_ich)
					for (i = 0; i < 3; i = i + 1)
						rs[c][i] <= size[c] == ms[c] ? ms[c][3:0] : res_size(qe[c][i]);
				prev_q[c] <= q[c][3:0];
				prev_recon[c] <= rec[2][c][8:0];
			end
			prev_ich <= use_ich;
			state <= S_RC1;
		end
	end

	S_RC1: begin
		if (!last_gx) begin
			for (k = 0; k < 32; k = k + 1) ich[k] <= ich_n[k];
			ich_v <= ich_vn;
		end
		pix_cnt <= pc_n;
		frac <= acc_n[4:0];
		mod_act <= ma_n;
		mod_cnt <= mc_n[15:0];
		buf_full <= bf_n[19:0];
		scale <= sc_n[6:0];
		scnt <= scn_n[15:0];
		sis <= sis_n;
		xoff <= xo_n;
		off_clamp <= cl_n;
		ofs_r <= ofs_n[19:0];
		model_r <= model_n[19:0];
		state <= S_RC2;
	end

	S_RC2: begin
		mq <= mq_n[3:0];
		st1 <= st_n[3:0];
		fl_tvalid <= tv_n;
		fl_cnt <= fc_n;
		rc_size_prev <= rc_size;
		g4 <= g4 + 2'd1;
		g0 <= 1'b0;
		if (last_gx) begin
			gx <= 16'd0;
			lp_cnt <= 2'd0;
			if (line == slice_h - 16'd1) begin
				skip_rem <= slice_bits - mux_pos;
				state <= S_SKIP;
			end else begin
				line <= line + 16'd1;
				state <= S_LP;
			end
		end else begin
			gx <= gx + 16'd1;
			for (k = 0; k < 12; k = k + 1) win[k] <= win[k + 3];
			if (gx + 16'd3 < gpl) begin
				win[12] <= lb_next[77:52]; win[13] <= lb_next[51:26]; win[14] <= lb_next[25:0];
			end else begin
				win[12] <= win[14]; win[13] <= win[14]; win[14] <= win[14];
			end
			state <= S_MUX;
		end
	end

	// drop the rest of the slice's chunks
	S_SKIP: begin
		skip_rem <= skip_rem - {24'd0, bb_take};
		if (skip_rem == 32'd0) begin
			slice_start <= slice_start + slice_bits;
			if (y0 + slice_h >= pic_h) begin
				done <= 1'b1;
				busy <= 1'b0;
				state <= S_IDLE;
			end else begin
				y0 <= y0 + slice_h;
				state <= S_SLICE;
			end
		end
	end

	default: state <= S_IDLE;
	endcase

	if (start) begin
		bb <= 128'd0;
		bb_cnt <= 8'd0;
		fpos <= 32'd0;
		fetch_lim <= 32'd0;
		slice_start <= 32'd0;
		y0 <= 16'd0;
		error <= !pps_ok;
		gpl <= ({17'd0, slice_w} * 33'd43691) >> 17;
		slice_bits <= {chunk, 3'd0} * slice_h;
		out_valid <= 1'b0;
		if (pps_ok) begin
			busy <= 1'b1;
			state <= S_SLICE;
		end else begin
			busy <= 1'b0;
			done <= 1'b1;
			state <= S_IDLE;
		end
	end

	if (reset) begin
		state <= S_IDLE;
		busy <= 1'b0;
		error <= 1'b0;
		out_valid <= 1'b0;
		bb_cnt <= 8'd0;
		fetch_lim <= 32'd0;
	end
end

endmodule

`default_nettype wire
