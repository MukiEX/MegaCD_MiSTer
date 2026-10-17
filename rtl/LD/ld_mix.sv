// ld_mix.sv - LaserActive (Sega PAC) video mixer for the MegaCD core.
//
// Mixes the Mega Drive VDP picture over the LaserDisc picture the way ares
// does (MCD::LD::scanline), line by line and in step with the VDP:
//
//   * Every VDP pixel becomes two output pixels, so the LD picture keeps
//     about its full resolution (640 pixels across the H40 active area).
//   * Horizontal mapping follows ares: a line of 1495 VDP dots (MCLK/2
//     units) spans the 792 kept columns of the LD picture. In H40 the active
//     area starts at dot 120 (column 63.6), in H32 at dot 105 (column 55.6).
//     Each output pixel takes the LD picture at its centre, interpolating
//     linearly between the two nearest columns.
//   * Vertical mapping: the LD line for a VDP line is its "target scan line"
//     (0 = V counter 0xE8, the first line of the bottom blanking; active
//     line 0 is 30), in the field chosen for the frame.
//   * Mixing: out = vdp * f + ld * (1 - f), with f = in 0x1A (or in 0x1B for
//     backdrop, border and blank pixels) >> 2 as a 6-bit fraction. Mixing
//     mode (in 0x01 bits 7-6) 0 passes the VDP picture through untouched.
//   * Fields, chosen at the start of each VDP frame (VBlank):
//       interlaced     = !0C[5] && !0C[3] && !0C[1]
//       field select   = mixing mode >= 2 && (0C[1] || 0C[3]): field = 0C[0]
//       otherwise a new LD frame starts on its first (odd) field, and an
//       interlaced picture toggles fields on every VDP frame.
//     The OSD can force both fields (force_il): they then alternate on every
//     VDP frame whatever the game set.
//     When the LD picture is interlaced, field and vsync timing are driven
//     for real 480i (il / f1 / vs_delay, the VDP delays VSync half a line
//     on the first field), as a LaserActive locks the Mega Drive to the disc.
//
// The LD lines come from ld_video (the frame buffer it decoded into), one
// line ahead: when a line's active area ends, the line after next is
// fetched into the other half of a two-line buffer.
//
// Output timing: everything (pixels, CE, syncs, blanks) is the VDP's, delayed
// by LAT clocks, with two CE pulses per VDP pixel.
//
// SPDX-License-Identifier: MIT
`default_nettype none

module ld_mix (
	input  wire        clk,

	// VDP output (all change together, on ce_pix)
	input  wire        ce_pix,
	input  wire  [7:0] vdp_r, vdp_g, vdp_b,
	input  wire        vdp_backdrop,
	input  wire  [8:0] vdp_vcnt,
	input  wire        hbl, vbl, hs, vs,
	input  wire        border,          // VDP border enabled (the first pixel is x = -13)

	// PD6103A registers in effect: {in 0x01, in 0x0C, in 0x1A, in 0x1B}
	input  wire [31:0] regs,
	input  wire        show,            // OSD: LD picture allowed
	input  wire        ld_on,           // the player shows its picture (else it reads as black)
	input  wire        force_il,        // OSD: always both fields, alternating (480i)
	input  wire signed [15:0] hoff,     // horizontal fine adjust, 1/256 LD column
	input  wire signed [7:0]  voff,     // vertical adjust, LD lines (per field)

	// ld_video: frame latch and line fetch
	output reg         disp_latch,      // pulse at the start of a VDP frame
	input  wire        latch_new,       // (the clock after) a new LD frame was latched
	input  wire [10:0] pic_rows,        // rows in the frame buffer (both fields)
	output reg         lf_req,
	output reg   [9:0] lf_row,
	input  wire        lf_we,
	input  wire  [8:0] lf_addr,         // word in the line (2 pixels each)
	input  wire [39:0] lf_data,         // {pixel 1, pixel 0}: {B[7:2], G[7:1], R[7:1]} each

	// mixed video
	output reg         ce_out,
	output reg   [7:0] r_out, g_out, b_out,
	output wire        hbl_out, vbl_out, hs_out, vs_out,
	output reg         il,              // the LD picture is interlaced (480i out)
	output reg         f1,              // current field: 0 = first (upper), 1 = second
	output reg         vs_delay         // to the VDP: delay the coming VSync half a line
);

localparam LAT = 7;   // clocks from a VDP pixel (ce_pix) to its first output pixel

// --------------------------------------------------------------- registers
wire [1:0] mixmode = regs[31:30];
wire [7:0] r0C     = regs[23:16];
wire [5:0] fade_a  = regs[15:10];        // in 0x1A >> 2
wire [5:0] fade_b  = regs[7:2];          // in 0x1B >> 2
wire       mixing  = show && mixmode != 2'd0;

// -------------------------------------------------------------- line buffer
// two lines of 792 pixels, 20 bits each (7-7-6, so a line pair fits in four
// M10Ks: the analog picture doesn't need more)
reg  [39:0] lbuf [0:1023];
reg   [9:0] lb_raddr;
reg  [39:0] lb_q;
reg         lb_wbank;
always @(posedge clk) begin
	if (lf_we) lbuf[{lb_wbank, lf_addr}] <= lf_data;
	lb_q <= lbuf[lb_raddr];
end

// ------------------------------------------------------------ VDP timing
reg  [3:0] ce_cnt;          // clocks since the last ce_pix
reg  [3:0] period = 4'd8;   // clocks per VDP pixel
reg  [3:0] phase = 4'd15;
reg        hbl_l = 1'b1, vbl_l = 1'b1;
reg  [7:0] cr, cg, cb;      // VDP pixel being output
reg        cbd, cblank;
wire [3:0] half = {1'b0, period[3:1]};

// target scan line of a V counter value (NTSC V28)
function automatic [8:0] tsl(input [8:0] v);
	if (v >= 9'h1E5)      tsl = v - 9'h1E5 + 9'd3;
	else if (v >= 9'h0E8) tsl = v - 9'h0E8;
	else                  tsl = v + 9'd30;
endfunction

wire [8:0] vcnt_tsl = tsl(vdp_vcnt);
reg  [8:0] cur_tsl;
reg  [1:0] bank_ok;         // the line in each half of the buffer is a picture line
reg        cur_bank;
reg        tog, even;
reg        fetch_go;
reg  [8:0] fetch_tsl;
reg  [2:0] latch_wait;

reg  signed [24:0] pos;     // LD column, 14 fraction bits
reg         go;
wire signed [9:0] fetch_line = $signed({1'b0, fetch_tsl}) + voff;
wire        fetch_ok = !fetch_line[9] && {1'b0, fetch_line[8:0], even} < pic_rows;
wire [15:0] step = (half == 4'd5) ? 16'd21699 : 16'd17359;   // LD columns per output pixel

// start column of the first output pixel of a line
wire [23:0] start_pos = (period == 4'd10) ? (border ? 24'd349845 : 24'd914024)
                                          : (border ? 24'd590706 : 24'd1042050);

always @(posedge clk) begin
	go <= 1'b0;
	lf_req <= 1'b0;
	disp_latch <= 1'b0;
	fetch_go <= 1'b0;

	if (ce_pix) begin
		ce_cnt <= 4'd1;
		if (ce_cnt >= 4'd6 && ce_cnt <= 4'd12) period <= ce_cnt;
		phase <= 4'd1;
		go <= 1'b1;
		cr <= vdp_r; cg <= vdp_g; cb <= vdp_b;
		cbd <= vdp_backdrop;
		cblank <= hbl || vbl;
		hbl_l <= hbl;
		vbl_l <= vbl;

		// a line's active area starts: its LD line is in the buffer
		if (hbl_l && !hbl) begin
			cur_tsl <= vcnt_tsl;
			cur_bank <= vcnt_tsl[0];
			pos <= $signed({1'b0, start_pos}) + ($signed(hoff) <<< 6);
		end
		// ... and ends: fetch the line after next into the half just used
		if (!hbl_l && hbl) begin
			fetch_go <= 1'b1;
			fetch_tsl <= (cur_tsl >= 9'd260) ? cur_tsl - 9'd260 : cur_tsl + 9'd2;
		end
		// a frame's active area ends: pick the frame and field for the next one
		if (!vbl_l && vbl) begin
			disp_latch <= 1'b1;
			latch_wait <= 3'd1;
		end
	end else begin
		if (ce_cnt != 4'd15) ce_cnt <= ce_cnt + 4'd1;
		if (phase != 4'd15) phase <= phase + 4'd1;
		if (phase == half) go <= 1'b1;
	end

	// output pixels walk across the LD line
	if (go && !cblank && !(ce_pix && hbl_l && !hbl)) pos <= pos + $signed({9'd0, step});

	// frame and field selection (ares MCD::LD::scanline, target line 0);
	// latch_new is valid two clocks after the latch request
	if (latch_wait != 0) latch_wait <= latch_wait + 3'd1;
	if (latch_wait == 3'd2) begin
		latch_wait <= 3'd0;
		tog <= tog_n;
		even <= even_n;
		il <= il_n && mixing && ld_on;
		f1 <= even_n;
		vs_delay <= il_n && mixing && ld_on && !even_n;
	end

	if (fetch_go) begin
		// field line + vertical adjust, woven row = 2 * line + field
		lf_req <= fetch_ok;
		lf_row <= {fetch_line[8:0], even};
		lb_wbank <= fetch_tsl[0];
		bank_ok[fetch_tsl[0]] <= ld_on && fetch_ok;
	end
end

// force_il: both fields whatever the game set, strictly alternating (no
// restart on a new frame, so a CRT always gets odd/even/odd...)
wire il_n   = force_il || (!r0C[5] && !r0C[3] && !r0C[1]);
wire tog_n  = force_il ? ~tog : latch_new ? 1'b0 : (il_n ? ~tog : tog);
wire even_n = (!force_il && mixmode >= 2'd2 && (r0C[1] || r0C[3])) ? r0C[0] : tog_n;

// ----------------------------------------------------------- sample stage
// A: column and first read; A+1: second read; A+2: first word; A+3: pick the
// two pixels; A+4: interpolate; A+5: mix; A+6 (LAT): output.
wire [9:0] col_raw = pos[23:14];
wire [9:0] col = pos[24] ? 10'd0 : (col_raw > 10'd790 ? 10'd790 : col_raw);
wire [5:0] frac = (pos[24] || col_raw > 10'd790) ? 6'd0 : pos[13:8];

reg  [5:0] s_go;
reg        s_odd [1:3];
reg  [5:0] s_frac [1:4];
reg  [8:0] s_word;
reg [39:0] w0;
reg  [7:0] s_vr [1:5], s_vg [1:5], s_vb [1:5];
reg        s_bd [1:5], s_blank [1:5], s_ok [1:4];
reg  [23:0] pa, pb;
reg  [7:0] ld_r, ld_g, ld_b;

always @(posedge clk) begin
	s_go <= {s_go[4:0], go};

	// A
	if (go) begin
		lb_raddr <= {cur_bank, col[9:1]};
		s_word <= col[9:1];
		s_odd[1] <= col[0];
		s_frac[1] <= frac;
		s_vr[1] <= cr; s_vg[1] <= cg; s_vb[1] <= cb;
		s_bd[1] <= cbd;
		s_blank[1] <= cblank;
		s_ok[1] <= bank_ok[cur_bank];
	end
	// A+1
	if (s_go[0]) begin
		lb_raddr <= {cur_bank, s_word + 9'd1};
		s_odd[2] <= s_odd[1]; s_frac[2] <= s_frac[1];
		s_vr[2] <= s_vr[1]; s_vg[2] <= s_vg[1]; s_vb[2] <= s_vb[1];
		s_bd[2] <= s_bd[1]; s_blank[2] <= s_blank[1]; s_ok[2] <= s_ok[1];
	end
	// A+2
	if (s_go[1]) begin
		w0 <= lb_q;
		s_odd[3] <= s_odd[2]; s_frac[3] <= s_frac[2];
		s_vr[3] <= s_vr[2]; s_vg[3] <= s_vg[2]; s_vb[3] <= s_vb[2];
		s_bd[3] <= s_bd[2]; s_blank[3] <= s_blank[2]; s_ok[3] <= s_ok[2];
	end
	// A+3
	if (s_go[2]) begin
		pa <= px24(s_odd[3] ? w0[39:20] : w0[19:0]);
		pb <= px24(s_odd[3] ? lb_q[19:0] : w0[39:20]);
		s_frac[4] <= s_frac[3];
		s_vr[4] <= s_vr[3]; s_vg[4] <= s_vg[3]; s_vb[4] <= s_vb[3];
		s_bd[4] <= s_bd[3]; s_blank[4] <= s_blank[3]; s_ok[4] <= s_ok[3];
	end
	// A+4
	if (s_go[3]) begin
		ld_r <= s_ok[4] ? lerp(pa[7:0],   pb[7:0],   s_frac[4]) : 8'd0;
		ld_g <= s_ok[4] ? lerp(pa[15:8],  pb[15:8],  s_frac[4]) : 8'd0;
		ld_b <= s_ok[4] ? lerp(pa[23:16], pb[23:16], s_frac[4]) : 8'd0;
		s_vr[5] <= s_vr[4]; s_vg[5] <= s_vg[4]; s_vb[5] <= s_vb[4];
		s_bd[5] <= s_bd[4]; s_blank[5] <= s_blank[4];
	end
end

// 20-bit line buffer pixel to {B, G, R}
function automatic [23:0] px24(input [19:0] p);
	px24 = {p[19:14], p[19:18], p[13:7], p[13], p[6:0], p[6]};
endfunction

function automatic [7:0] lerp(input [7:0] a, input [7:0] b, input [5:0] t);
	reg signed [15:0] d;
	begin
		d = ($signed({1'b0, b}) - $signed({1'b0, a})) * $signed({1'b0, t});
		lerp = a + d[13:6] + {7'd0, d[5]};  // rounded; stays within a..b
	end
endfunction

// weight of the VDP pixel out of 256, from a 6-bit fader (63 -> 256)
function automatic [8:0] weight(input [5:0] f);
	reg [12:0] p;
	begin
		p = f * 7'd65 + 13'd8;
		weight = p[12:4];
	end
endfunction

function automatic [7:0] blend(input [7:0] v, input [7:0] l, input [8:0] w);
	reg [16:0] s;
	begin
		s = v * w + l * (9'd256 - w) + 17'd128;
		blend = s[15:8];
	end
endfunction

// A+5: mix, A+6: out
reg  [7:0] m_r, m_g, m_b;
always @(posedge clk) begin
	ce_out <= s_go[5];
	if (s_go[4]) begin
		if (!mixing || s_blank[5]) begin
			m_r <= s_vr[5]; m_g <= s_vg[5]; m_b <= s_vb[5];
		end else begin
			m_r <= blend(s_vr[5], ld_r, weight(s_bd[5] ? fade_b : fade_a));
			m_g <= blend(s_vg[5], ld_g, weight(s_bd[5] ? fade_b : fade_a));
			m_b <= blend(s_vb[5], ld_b, weight(s_bd[5] ? fade_b : fade_a));
		end
	end
	if (s_go[5]) begin
		r_out <= m_r; g_out <= m_g; b_out <= m_b;
	end
end

// syncs and blanks: the VDP's, delayed to line up with ce_out
// (d_x[n] holds x from n + 1 clocks ago, ce_out comes LAT + 1 clocks after ce_pix)
reg [LAT:0] d_hbl, d_vbl, d_hs, d_vs;
always @(posedge clk) begin
	d_hbl <= {d_hbl[LAT-1:0], hbl};
	d_vbl <= {d_vbl[LAT-1:0], vbl};
	d_hs  <= {d_hs[LAT-1:0], hs};
	d_vs  <= {d_vs[LAT-1:0], vs};
end
assign hbl_out = d_hbl[LAT];
assign vbl_out = d_vbl[LAT];
assign hs_out  = d_hs[LAT];
assign vs_out  = d_vs[LAT];

endmodule

`default_nettype wire
