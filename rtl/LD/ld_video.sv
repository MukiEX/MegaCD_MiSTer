// ld_video.sv - LaserActive video player for the MegaCD core.
//
// The HPS (Main_MiSTer) keeps .dscmmi frames in 16 slots in DDR3 and
// commits the slot to show next (commit_we/commit_slot) whenever the disc
// reaches a new frame. Commits wait in a 4-entry queue. This block reads the
// oldest, decodes it with dsc_decoder and writes the picture into a frame
// buffer, woven (the two fields stored in the .dscmmi picture, rows
// 0..field1_row-1 and field1_row.., go to alternate lines).
//
// Three frame buffers: the one on screen ("disp"), the newest finished one
// ("ready") and the one being decoded into, which is never either of the
// other two. A decode starts as soon as a commit is waiting. If three are
// waiting (scanning, or after a hiccup), the oldest is skipped. With nothing
// committed the last picture stays up (still frames).
//
// ld_mix asks for a picture at the start of every VDP frame (disp_latch). A
// finished picture not shown yet ("fresh") goes up when the last one has
// been up for two VDP frames (the disc's 29.97 Hz against the VDP's 59.92),
// or at once if it is late. Commits come from the HPS with some jitter (the
// disc's sectors don't line up with its frames, and the HPS polls), so the
// time each picture waits between being finished and going up (its margin)
// is averaged, and the cadence is moved by one VDP frame when the average
// gets too small to absorb the jitter (one picture is held for three VDP
// frames) or bigger than it needs to be (one is shown for just one): the
// disc runs a hair faster than the VDP, so this happens every minute or two
// during long playback, and the delay stays short.
//
// ld_mix reads the picture line by line (lf_req/lf_row); those reads come
// first on the DDR3 port.
//
// DDR3 map (physical byte addresses):
//   0x30100000 + k * 0x80000            slot k (0-15), 512 KB each
//   0x30A00000, 0x30C00000, 0x30E00000  frame buffers, 32 bits per pixel (R in byte 0)
//
// One frame takes about 1.34 M clocks to decode at 53.69 MHz (a 29.97 Hz
// frame is 1.79 M), so decoding keeps up with a margin.
//
// SPDX-License-Identifier: MIT
`default_nettype none

module ld_video #(
	parameter [31:0] RING_BASE = 32'h3010_0000,
	parameter [31:0] FB0_BASE  = 32'h30A0_0000,
	parameter [31:0] FB1_BASE  = 32'h30C0_0000,
	parameter [31:0] FB2_BASE  = 32'h30E0_0000
) (
	input  wire        clk,
	input  wire        reset,

	// from the HPS (hps_ext)
	input  wire        ctl_enable,      // a stream is loaded and running
	input  wire        ctl_restart,     // pulse: forget everything, counters to 0
	input  wire        ctl_visible,     // the player wants the picture shown
	input  wire        pps_we,
	input  wire  [6:0] pps_addr,
	input  wire  [7:0] pps_data,
	input  wire [17:0] frame_words,     // 64-bit words per frame = ceil(frame_bytes / 8)
	input  wire [10:0] field1_row,      // first row of the second field (0 = progressive)
	input  wire        commit_we,       // pulse: queue commit_slot
	input  wire  [3:0] commit_slot,

	// to the HPS
	output reg  [15:0] rd_count,        // commits finished (decoded and flipped)
	output reg  [15:0] late_count,      // VDP frame pairs that wanted a new picture while one was decoding
	output wire  [2:0] queue_level,     // commits waiting, including one being decoded
	output reg  [15:0] drop_count,      // commits skipped to catch up (also counted in rd_count)
	output reg  [15:0] error_count,     // frames the decoder flagged
	output reg  [15:0] decode_kclks,    // clocks the last frame took, / 256
	output wire        active,          // at least one frame decoded

	// display (ld_mix)
	output wire        ld_on,           // a picture is up and the HPS wants it shown
	output wire [10:0] pic_rows,
	input  wire        disp_latch,      // start of a VDP frame
	output reg         latch_new,       // the clock after: a new picture was latched
	input  wire        lf_req,          // read a line of the picture on screen
	input  wire  [9:0] lf_row,
	output reg         lf_we,
	output reg   [8:0] lf_addr,
	output reg  [39:0] lf_data,         // 2 pixels of 20 bits: {B[7:2], G[7:1], R[7:1]} each

	// DDR3 (MiSTer DDRAM port, same clock)
	input  wire        DDRAM_BUSY,
	output reg   [7:0] DDRAM_BURSTCNT,
	output reg  [28:0] DDRAM_ADDR,
	input  wire [63:0] DDRAM_DOUT,
	input  wire        DDRAM_DOUT_READY,
	output reg         DDRAM_RD,
	output wire [63:0] DDRAM_DIN,
	output wire  [7:0] DDRAM_BE,
	output reg         DDRAM_WE
);

// ----------------------------------------------------------- picture size
reg [15:0] pic_w, pic_h;
always @(posedge clk) if (pps_we) begin
	case (pps_addr)
		7'd6: pic_h[15:8] <= pps_data;
		7'd7: pic_h[7:0]  <= pps_data;
		7'd8: pic_w[15:8] <= pps_data;
		7'd9: pic_w[7:0]  <= pps_data;
		default: ;
	endcase
end
wire [13:0] stride_b = {pic_w[11:0], 2'b00};       // bytes per framebuffer line
wire [11:0] line_words = pic_w[12:1];               // 2 pixels per 64-bit word

reg  [1:0] disp_buf, ready_buf, back_buf;          // frame buffers 0-2
reg        fresh;                                   // ready_buf holds a picture not shown yet
reg        shown_one;                               // disp_buf holds a picture
assign active   = shown_one;
assign ld_on    = ctl_visible && shown_one;
assign pic_rows = pic_h[10:0];

function automatic [28:0] fb_word(input [1:0] n);
	fb_word = n == 2'd0 ? FB0_BASE[31:3] : n == 2'd1 ? FB1_BASE[31:3] : FB2_BASE[31:3];
endfunction

// --------------------------------------------------------------- decoder
wire        dec_busy, dec_done, dec_error;
wire [63:0] in_q;
wire        in_valid;
wire        in_ready;
wire        dec_out_valid;
wire        dec_out_ready;
wire [15:0] dec_x, dec_y;
wire [23:0] rgb0, rgb1, rgb2;
reg         dec_start;

dsc_decoder dec (
	.clk(clk), .reset(reset),
	.pps_we(pps_we), .pps_addr(pps_addr), .pps_data(pps_data),
	.start(dec_start), .busy(dec_busy), .done(dec_done), .error(dec_error),
	.in_data(in_q), .in_valid(in_valid), .in_ready(in_ready),
	.out_valid(dec_out_valid), .out_ready(dec_out_ready),
	.out_x(dec_x), .out_y(dec_y), .out_rgb0(rgb0), .out_rgb1(rgb1), .out_rgb2(rgb2)
);

// ------------------------------------------------- frame read: DDR3 -> FIFO
reg         flush;
reg  [17:0] rd_words_left;    // words not yet requested
reg   [8:0] rd_outstanding;   // requested, not yet returned
reg  [28:0] rd_addr;          // next word address
wire  [8:0] infifo_count;

// Reads come back in order: a small FIFO of the bursts in flight says whose
// each returning word is (decoder or line fetch).
reg   [7:0] tag_len [0:15];
reg  [15:0] tag_lf;
reg   [3:0] tag_rd, tag_wr;
reg   [4:0] tag_n;
reg   [7:0] tag_left;         // words left of the burst at the head
wire        rd_ret  = DDRAM_DOUT_READY && tag_n != 0;
wire        ret_lf  = tag_lf[tag_rd];

ld_fifo #(.W(64), .AW(8)) infifo (
	.clk(clk), .flush(flush),
	.push(rd_ret && !ret_lf), .din(DDRAM_DOUT),
	.pop(in_valid && in_ready), .q(in_q), .valid(in_valid), .count(infifo_count)
);

// request another burst when the FIFO has room for it on top of what's in flight
wire  [7:0] rd_len  = rd_words_left > 18'd64 ? 8'd64 : rd_words_left[7:0];
wire        rd_want = rd_words_left != 0 && ({1'b0, infifo_count} + {1'b0, rd_outstanding} + rd_len) <= 10'd256 && tag_n < 5'd14;

// ------------------------------------------------- line fetch for ld_mix
reg   [9:0] lf_words_left;
reg  [28:0] lf_raddr;
reg         lf_pend;
reg   [9:0] lf_pend_row;
reg   [9:0] lf_outstanding;
reg   [8:0] lf_wcnt;
wire  [7:0] lf_len  = lf_words_left > 10'd128 ? 8'd128 : lf_words_left[7:0];
wire        lf_want = lf_words_left != 0 && tag_n < 5'd14;

// --------------------------------------- pixels: decoder -> FIFO of pairs
// 32-bit pixel: byte 0 = R, 1 = G, 2 = B
function automatic [31:0] px32(input [23:0] rgb);
	px32 = {8'h00, rgb[7:0], rgb[15:8], rgb[23:16]};
endfunction

reg   [1:0] pk_state;         // 0 idle, 1 second push pending
reg  [31:0] pk_hold;          // odd pixel waiting for its partner
reg         pk_hold_v;
reg  [63:0] pk_second;
reg         pk_push;
reg  [63:0] pk_din;
wire [63:0] px_q;
wire        px_valid;
wire  [8:0] px_count;
reg         px_pop;
reg         frame_setup;

// ------------------------------------------------------------ commit queue
reg   [3:0] q_mem [0:3];
reg   [1:0] q_rd, q_wr;
reg   [2:0] q_level;
reg         q_pop;
wire  [3:0] cur_slot = q_mem[q_rd];
assign queue_level = q_level;

ld_fifo #(.W(64), .AW(8)) pxfifo (
	.clk(clk), .flush(flush),
	.push(pk_push), .din(pk_din),
	.pop(px_pop), .q(px_q), .valid(px_valid), .count(px_count)
);

assign dec_out_ready = pk_state == 2'd0 && px_count <= 9'd244;

always @(posedge clk) begin
	pk_push <= 1'b0;
	if (flush) begin
		pk_state <= 2'd0;
		pk_hold_v <= 1'b0;
	end else if (pk_state == 2'd1) begin
		pk_push <= 1'b1;
		pk_din <= pk_second;
		pk_state <= 2'd0;
	end else if (dec_out_valid && dec_out_ready) begin
		pk_push <= 1'b1;
		if (!pk_hold_v) begin
			pk_din <= {px32(rgb1), px32(rgb0)};
			pk_hold <= px32(rgb2);
			pk_hold_v <= 1'b1;
		end else begin
			pk_din <= {px32(rgb0), pk_hold};
			pk_second <= {px32(rgb2), px32(rgb1)};
			pk_hold_v <= 1'b0;
			pk_state <= 2'd1;
		end
	end
end

// -------------------------------------------- line writes: FIFO -> DDR3
reg  [10:0] wr_row;           // decoded row being written
reg  [11:0] wr_line_left;     // words of this line not yet started
reg  [28:0] wr_addr;          // next word address in the framebuffer
reg         wr_line_go;       // address for this row is set up
reg  [10:0] rows_done;
wire  [7:0] wr_len  = wr_line_left > 12'd128 ? 8'd128 : wr_line_left[7:0];
wire        wr_want = wr_line_go && wr_line_left != 0 && px_count >= {1'b0, wr_len};
reg   [7:0] wr_beats;         // beats left in the current burst
reg         wr_burst;

// row in the framebuffer: fields woven into alternate lines (registered;
// a new row waits one clock for it, see row_wait)
wire [10:0] fb_row = field1_row == 11'd0 ? wr_row :
                     wr_row < field1_row ? {wr_row[9:0], 1'b0} : {wr_row[9:0] - field1_row[9:0], 1'b1};
reg  [10:0] fb_row_r;
reg         row_wait;
always @(posedge clk) fb_row_r <= fb_row;

assign DDRAM_DIN = px_q;
assign DDRAM_BE  = 8'hFF;

// ------------------------------------------------- DDR3 port arbitration
// A command (RD, or WE with the first beat) is held until BUSY is low.
// Line fetches first, then decoder reads and writes in turn.
reg  prefer_rd;
wire port_free = !wr_burst && (!DDRAM_BUSY || (!DDRAM_RD && !DDRAM_WE));
wire issue_lf  = port_free && lf_want;
wire issue_wr  = port_free && !issue_lf && wr_want && (!rd_want || !prefer_rd);
wire issue_rd  = port_free && !issue_lf && !issue_wr && rd_want;

always @(posedge clk) begin
	if (!DDRAM_BUSY) DDRAM_RD <= 1'b0;

	if (wr_burst && DDRAM_WE && !DDRAM_BUSY) begin
		if (wr_beats == 8'd1) begin
			wr_burst <= 1'b0;
			DDRAM_WE <= 1'b0;
		end
		wr_beats <= wr_beats - 8'd1;
	end

	if (issue_wr) begin
		DDRAM_ADDR <= wr_addr;
		DDRAM_BURSTCNT <= wr_len;
		DDRAM_WE <= 1'b1;
		wr_burst <= 1'b1;
		wr_beats <= wr_len;
		wr_addr <= wr_addr + wr_len;
		wr_line_left <= wr_line_left - wr_len;
		prefer_rd <= 1'b1;
	end
	if (issue_rd) begin
		DDRAM_ADDR <= rd_addr;
		DDRAM_BURSTCNT <= rd_len;
		DDRAM_RD <= 1'b1;
		rd_addr <= rd_addr + rd_len;
		rd_words_left <= rd_words_left - rd_len;
		prefer_rd <= 1'b0;
	end
	if (issue_lf) begin
		DDRAM_ADDR <= lf_raddr;
		DDRAM_BURSTCNT <= lf_len;
		DDRAM_RD <= 1'b1;
		lf_raddr <= lf_raddr + lf_len;
		lf_words_left <= lf_words_left - lf_len;
	end
	rd_outstanding <= rd_outstanding + (issue_rd ? {1'b0, rd_len} : 9'd0) - (rd_ret && !ret_lf ? 9'd1 : 9'd0);

	// bursts in flight
	if (issue_rd || issue_lf) begin
		tag_len[tag_wr] <= issue_lf ? lf_len : rd_len;
		tag_lf[tag_wr] <= issue_lf;
		tag_wr <= tag_wr + 4'd1;
	end
	if (rd_ret) begin
		if ((tag_left == 8'd0 ? tag_len[tag_rd] : tag_left) == 8'd1) begin
			tag_rd <= tag_rd + 4'd1;
			tag_left <= 8'd0;
		end else tag_left <= (tag_left == 8'd0 ? tag_len[tag_rd] : tag_left) - 8'd1;
	end
	tag_n <= tag_n + ((issue_rd || issue_lf) ? 5'd1 : 5'd0)
	               - ((rd_ret && (tag_left == 8'd0 ? tag_len[tag_rd] : tag_left) == 8'd1) ? 5'd1 : 5'd0);

	// line fetch: returning words go to ld_mix's line buffer
	lf_we <= rd_ret && ret_lf;
	lf_addr <= lf_wcnt;
	lf_data <= {DDRAM_DOUT[55:50], DDRAM_DOUT[47:41], DDRAM_DOUT[39:33], DDRAM_DOUT[23:18], DDRAM_DOUT[15:9], DDRAM_DOUT[7:1]};
	if (rd_ret && ret_lf) lf_wcnt <= lf_wcnt + 9'd1;
	lf_outstanding <= lf_outstanding + (issue_lf ? {2'b00, lf_len} : 10'd0) - (rd_ret && ret_lf ? 10'd1 : 10'd0);
	if ((lf_req || lf_pend) && lf_words_left == 0 && lf_outstanding == 0 && !issue_lf) begin
		// a new line once every word of the last one is in
		lf_raddr <= fb_word(disp_buf) + (lf_req ? lf_row : lf_pend_row) * stride_b[13:3];
		lf_words_left <= {1'b0, line_words[8:0]};
		lf_wcnt <= 9'd0;
		lf_pend <= 1'b0;
	end else if (lf_req) begin
		lf_pend <= 1'b1;
		lf_pend_row <= lf_row;
	end

	// set up the next row once every burst of this one has been sent
	row_wait <= 1'b0;
	if (wr_line_go && wr_line_left == 0 && !wr_burst && !issue_wr) begin
		wr_line_go <= 1'b0;
		wr_row <= wr_row + 11'd1;
		rows_done <= rows_done + 11'd1;
		row_wait <= 1'b1;
	end else if (!wr_line_go && !row_wait && wr_row < pic_h[10:0] && !frame_setup) begin
		wr_addr <= fb_word(back_buf) + fb_row_r * stride_b[13:3];
		wr_line_left <= line_words;
		wr_line_go <= 1'b1;
	end

	if (frame_setup) begin
		rd_addr <= RING_BASE[31:3] + {cur_slot, 16'd0};
		rd_words_left <= frame_words;
		wr_row <= 11'd0;
		rows_done <= 11'd0;
		wr_line_go <= 1'b0;
		row_wait <= 1'b1;
	end

	// On reset stop issuing, but never withdraw a command or cut a write
	// burst short (the DDR3 bridge needs every beat), and keep counting the
	// reads already in flight so they drain before the next frame starts.
	if (reset) begin
		rd_words_left <= 18'd0;
		wr_line_go <= 1'b0;
		wr_row <= 11'h7FF;
	end
end

// px_pop must be combinational to follow the FWFT FIFO
always @* px_pop = wr_burst && DDRAM_WE && !DDRAM_BUSY;

// -------------------------------------------------------- frame scheduler
localparam F_IDLE = 2'd0, F_SETUP = 2'd1, F_RUN = 2'd2;
reg  [1:0] fstate;
reg [23:0] clocks;
reg        dec_finished;
reg  [2:0] since_new;         // VDP frames since a new picture went up (saturates at 7)
reg [21:0] fresh_clks;        // clocks since the fresh picture was finished
reg [11:0] margin_avg;        // average margin, 1024 clocks
reg        margin_ok;         // margin_avg holds something
reg  [1:0] adj;               // 1: hold the next picture one VDP frame longer, 2: one shorter
localparam [11:0] MARGIN_LO = 12'd682;     // 13 ms
localparam [11:0] MARGIN_HI = 12'd1678;    // 32 ms
localparam [11:0] VFRAME    = 12'd873;     // one VDP frame (893,780 clocks)
wire [11:0] margin_now  = fresh_clks[21] ? 12'hFFF : {1'b0, fresh_clks[20:10]};
wire [11:0] margin_next = margin_avg - {3'd0, margin_avg[11:3]} + {3'd0, margin_now[11:3]};

// Power-up state (the MiSTer projects allow power-up don't-care otherwise):
// nothing shown and the DDR3 port idle until the HPS sets the player up.
initial begin
	fstate = F_IDLE;
	shown_one = 1'b0;
	fresh = 1'b0;
	disp_buf = 2'd0;
	ready_buf = 2'd0;
	back_buf = 2'd1;
	since_new = 3'd7;
	margin_ok = 1'b0;
	adj = 2'd0;
	q_rd = 2'd0;
	q_wr = 2'd0;
	q_level = 3'd0;
	rd_count = 16'd0;
	drop_count = 16'd0;
	late_count = 16'd0;
	error_count = 16'd0;
	decode_kclks = 16'd0;
	DDRAM_RD = 1'b0;
	DDRAM_WE = 1'b0;
	wr_burst = 1'b0;
	rd_words_left = 18'd0;
	rd_outstanding = 9'd0;
	tag_rd = 4'd0;
	tag_wr = 4'd0;
	tag_n = 5'd0;
	tag_left = 8'd0;
	lf_words_left = 10'd0;
	lf_outstanding = 10'd0;
	lf_pend = 1'b0;
	wr_line_go = 1'b0;
	wr_row = 11'h7FF;
	pk_state = 2'd0;
	pk_hold_v = 1'b0;
end

// The head entry stays queued while it is decoded and is removed when the
// picture is complete, so a core reset in mid-frame decodes it again.
wire q_full = q_level == 3'd4;

// the buffer to decode into: neither the one on screen nor the newest
wire [1:0] free_buf = (disp_buf == ready_buf) ? (disp_buf == 2'd2 ? 2'd0 : disp_buf + 2'd1)
                                              : 2'd3 - disp_buf - ready_buf;

always @(posedge clk) begin
	q_pop = 1'b0;
	dec_start <= 1'b0;
	flush <= 1'b0;
	frame_setup <= 1'b0;
	latch_new <= 1'b0;
	clocks <= clocks + 24'd1;

	if (!fresh_clks[21]) fresh_clks <= fresh_clks + 22'd1;

	// start of a VDP frame: put up the next picture when it's due
	if (disp_latch) begin
		if (fresh && (since_new >= 3'd3 || (since_new == 3'd2 && adj != 2'd1) || (since_new == 3'd1 && adj == 2'd2))) begin
			disp_buf <= ready_buf;
			fresh <= 1'b0;
			shown_one <= 1'b1;
			latch_new <= 1'b1;
			since_new <= 3'd1;
			if (since_new == 3'd3 && shown_one && adj != 2'd1) late_count <= late_count + 16'd1;
			// margin bookkeeping, while pictures keep coming
			if (since_new <= 3'd3) begin
				margin_ok <= 1'b1;
				if (adj == 2'd1 && since_new == 3'd3) begin
					adj <= 2'd0;                       // held one VDP frame longer
					margin_avg <= margin_avg + VFRAME;
				end else if (adj == 2'd2 && since_new == 3'd1) begin
					adj <= 2'd0;                       // shown one VDP frame shorter
					margin_avg <= margin_avg - VFRAME;
				end else if (!margin_ok) begin
					margin_avg <= margin_now;
				end else begin
					margin_avg <= margin_next;
					if (adj == 2'd0 && margin_next < MARGIN_LO) adj <= 2'd1;
					if (adj == 2'd0 && margin_next > MARGIN_HI) adj <= 2'd2;
				end
			end else margin_ok <= 1'b0;
		end else if (since_new != 3'd7) since_new <= since_new + 3'd1;
	end

	case (fstate)
	F_IDLE:
		if (ctl_enable && q_level >= 3'd3) begin
			// too far behind: drop the oldest commit, decide again next clock
			q_pop = 1'b1;
			rd_count <= rd_count + 16'd1;
			drop_count <= drop_count + 16'd1;
		end else if (ctl_enable && q_level != 3'd0 && !reset) begin
			// start once traffic from an abandoned frame has drained
			if (rd_outstanding == 0 && !wr_burst) begin
				back_buf <= free_buf;
				flush <= 1'b1;
				frame_setup <= 1'b1;
				fstate <= F_SETUP;
			end
		end

	F_SETUP: begin
		dec_start <= 1'b1;
		dec_finished <= 1'b0;
		clocks <= 24'd0;
		fstate <= F_RUN;
	end

	F_RUN: begin
		if (dec_done) begin
			dec_finished <= 1'b1;
			if (dec_error) error_count <= error_count + 16'd1;
		end
		// done when the decoder has finished and every row is in the frame buffer
		if ((dec_finished || dec_done) && rows_done == pic_h[10:0] && !wr_burst && !disp_latch) begin
			// (a fresh picture not shown yet is replaced: it was too late anyway)
			if (fresh) drop_count <= drop_count + 16'd1;
			ready_buf <= back_buf;
			fresh <= 1'b1;
			fresh_clks <= 22'd0;
			rd_count <= rd_count + 16'd1;
			decode_kclks <= clocks[23:8];
			q_pop = 1'b1;
			fstate <= F_IDLE;
		end
		if (!ctl_enable) fstate <= F_IDLE;
	end

	default: fstate <= F_IDLE;
	endcase

	// A core reset abandons the frame in progress (it is decoded again from
	// its slot, which the HPS has not reused); only the HPS restarts the
	// frame counters, so it never loses track of the ring.
	if (reset) begin
		fstate <= F_IDLE;
		flush <= 1'b1;
	end
	// queue bookkeeping
	if (q_pop) q_rd <= q_rd + 2'd1;
	if (commit_we && !q_full) begin
		q_mem[q_wr] <= commit_slot;
		q_wr <= q_wr + 2'd1;
	end
	q_level <= q_level + ((commit_we && !q_full) ? 3'd1 : 3'd0) - (q_pop ? 3'd1 : 3'd0);

	if (ctl_restart) begin
		q_rd <= 2'd0;
		q_wr <= 2'd0;
		q_level <= 3'd0;
		fstate <= F_IDLE;
		rd_count <= 16'd0;
		drop_count <= 16'd0;
		late_count <= 16'd0;
		error_count <= 16'd0;
		decode_kclks <= 16'd0;
		shown_one <= 1'b0;
		fresh <= 1'b0;
		since_new <= 3'd7;
		margin_ok <= 1'b0;
		adj <= 2'd0;
		flush <= 1'b1;
	end
end

endmodule

`default_nettype wire
