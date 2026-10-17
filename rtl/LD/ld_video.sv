// ld_video.sv - LaserActive video test player for the MegaCD core.
//
// The HPS (Main_MiSTer) streams .dscmmi frames into a ring of 8 slots in
// DDR3 and tells this block how many it has written (wr_count). At 29.97 Hz
// this block takes the next frame, reads it from its slot, decodes it with
// dsc_decoder, and writes the picture into one of two framebuffers in DDR3,
// which MiSTer's scaler displays (FB_EN/FB_BASE). The two fields stored in
// the .dscmmi picture (rows 0..field1_row-1 and field1_row..) are woven back
// into alternate lines.
//
// DDR3 map (physical byte addresses):
//   0x30100000 + k * 0x100000   ring slot k (k = frame number mod 8)
//   0x30A00000, 0x30C00000      framebuffers, 32 bits per pixel (R in byte 0)
//
// One frame takes about 1.34 M clocks to decode at 53.69 MHz (the 29.97 Hz
// period is 1.79 M), so decoding and line writes keep up with a margin.
//
// SPDX-License-Identifier: MIT
`default_nettype none

module ld_video #(
	parameter TICK_CLKS = 1791563,         // clk / 29.97 Hz (53.693175 MHz)
	parameter [31:0] RING_BASE = 32'h3010_0000,
	parameter [31:0] FB0_BASE  = 32'h30A0_0000,
	parameter [31:0] FB1_BASE  = 32'h30C0_0000
) (
	input  wire        clk,
	input  wire        reset,

	// from the HPS (hps_ext)
	input  wire        ctl_enable,      // a stream is loaded and running
	input  wire        ctl_restart,     // pulse: forget everything, frame counters to 0
	input  wire        pps_we,
	input  wire  [6:0] pps_addr,
	input  wire  [7:0] pps_data,
	input  wire [17:0] frame_words,     // 64-bit words per frame = ceil(frame_bytes / 8)
	input  wire [10:0] field1_row,      // first row of the second field (0 = progressive)
	input  wire [15:0] wr_count,        // frames the HPS has put in the ring

	// to the HPS
	output reg  [15:0] rd_count,        // frames shown (slot rd_count % 8 is in use while decoding)
	output reg  [15:0] late_count,      // ticks with no new frame ready, after the first frame
	output reg  [15:0] error_count,     // frames the decoder flagged
	output reg  [15:0] decode_kclks,    // clocks the last frame took, / 256
	output wire        active,          // at least one frame decoded

	// display
	input  wire        show,            // OSD: show the LD picture
	output wire        fb_en,
	output wire  [4:0] fb_format,
	output wire [11:0] fb_width,
	output wire [11:0] fb_height,
	output wire [31:0] fb_base,
	output wire [13:0] fb_stride,

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

reg front;                                          // framebuffer being shown
reg shown_one;
assign active    = shown_one;
assign fb_en     = show && shown_one;
assign fb_format = 5'b00110;                        // 32 bpp, RGB (R in the low byte)
assign fb_width  = pic_w[11:0];
assign fb_height = pic_h[11:0];
assign fb_base   = front ? FB1_BASE : FB0_BASE;
assign fb_stride = stride_b;
wire [31:0] back_base = front ? FB0_BASE : FB1_BASE;

// ------------------------------------------------------------- 29.97 Hz
reg [20:0] tick_cnt;
reg        tick;
always @(posedge clk) begin
	tick <= 1'b0;
	if (tick_cnt == TICK_CLKS - 1) begin
		tick_cnt <= 21'd0;
		tick <= 1'b1;
	end else tick_cnt <= tick_cnt + 21'd1;
end

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
reg   [9:0] rd_outstanding;   // requested, not yet returned
reg  [28:0] rd_addr;          // next word address
wire  [9:0] infifo_count;

ld_fifo #(.W(64), .AW(9)) infifo (
	.clk(clk), .flush(flush),
	.push(DDRAM_DOUT_READY && rd_outstanding != 0), .din(DDRAM_DOUT),
	.pop(in_valid && in_ready), .q(in_q), .valid(in_valid), .count(infifo_count)
);

// request another burst when the FIFO has room for it on top of what's in flight
wire  [7:0] rd_len  = rd_words_left > 18'd64 ? 8'd64 : rd_words_left[7:0];
wire        rd_want = rd_words_left != 0 && ({1'b0, infifo_count} + rd_outstanding + rd_len) <= 11'd512;

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
wire  [9:0] px_count;
reg         px_pop;
reg         frame_setup;

ld_fifo #(.W(64), .AW(9)) pxfifo (
	.clk(clk), .flush(flush),
	.push(pk_push), .din(pk_din),
	.pop(px_pop), .q(px_q), .valid(px_valid), .count(px_count)
);

assign dec_out_ready = pk_state == 2'd0 && px_count <= 10'd500;

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
wire        wr_want = wr_line_go && wr_line_left != 0 && px_count >= {2'b00, wr_len};
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
reg  prefer_rd;
wire port_free = !wr_burst && (!DDRAM_BUSY || (!DDRAM_RD && !DDRAM_WE));
wire issue_wr  = port_free && wr_want && (!rd_want || !prefer_rd);
wire issue_rd  = port_free && !issue_wr && rd_want;
wire rd_ret    = DDRAM_DOUT_READY && rd_outstanding != 0;

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
	rd_outstanding <= rd_outstanding + (issue_rd ? {2'b00, rd_len} : 10'd0) - (rd_ret ? 10'd1 : 10'd0);

	// set up the next row once every burst of this one has been sent
	row_wait <= 1'b0;
	if (wr_line_go && wr_line_left == 0 && !wr_burst && !issue_wr) begin
		wr_line_go <= 1'b0;
		wr_row <= wr_row + 11'd1;
		rows_done <= rows_done + 11'd1;
		row_wait <= 1'b1;
	end else if (!wr_line_go && !row_wait && wr_row < pic_h[10:0] && !frame_setup) begin
		wr_addr <= back_base[31:3] + fb_row_r * stride_b[13:3];
		wr_line_left <= line_words;
		wr_line_go <= 1'b1;
	end

	if (frame_setup) begin
		rd_addr <= RING_BASE[31:3] + {rd_count[2:0], 17'd0};
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
reg        tick_pend;
reg [23:0] clocks;
reg        dec_finished;

// Power-up state (the MiSTer projects allow power-up don't-care otherwise):
// nothing shown and the DDR3 port idle until the HPS sets the player up.
initial begin
	fstate = F_IDLE;
	tick_pend = 1'b0;
	shown_one = 1'b0;
	front = 1'b0;
	rd_count = 16'd0;
	late_count = 16'd0;
	error_count = 16'd0;
	decode_kclks = 16'd0;
	tick_cnt = 21'd0;
	DDRAM_RD = 1'b0;
	DDRAM_WE = 1'b0;
	wr_burst = 1'b0;
	rd_words_left = 18'd0;
	rd_outstanding = 10'd0;
	wr_line_go = 1'b0;
	wr_row = 11'h7FF;
	pk_state = 2'd0;
	pk_hold_v = 1'b0;
end

always @(posedge clk) begin
	dec_start <= 1'b0;
	flush <= 1'b0;
	frame_setup <= 1'b0;
	if (tick) tick_pend <= 1'b1;
	clocks <= clocks + 24'd1;

	case (fstate)
	F_IDLE:
		if (tick_pend || tick) begin
			if (ctl_enable && wr_count != rd_count) begin
				// start once traffic from an abandoned frame has drained
				if (rd_outstanding == 0 && !wr_burst) begin
					tick_pend <= 1'b0;
					flush <= 1'b1;
					frame_setup <= 1'b1;
					fstate <= F_SETUP;
				end else tick_pend <= 1'b1;
			end else begin
				tick_pend <= 1'b0;
				if (shown_one) late_count <= late_count + 16'd1;
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
		// done when the decoder has finished and every row is in the framebuffer
		if ((dec_finished || dec_done) && rows_done == pic_h[10:0] && !wr_burst) begin
			front <= ~front;
			shown_one <= 1'b1;
			rd_count <= rd_count + 16'd1;
			decode_kclks <= clocks[23:8];
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
		tick_pend <= 1'b0;
		flush <= 1'b1;
	end
	if (ctl_restart) begin
		fstate <= F_IDLE;
		rd_count <= 16'd0;
		late_count <= 16'd0;
		error_count <= 16'd0;
		decode_kclks <= 16'd0;
		shown_one <= 1'b0;
		front <= 1'b0;
		tick_pend <= 1'b0;
		flush <= 1'b1;
	end
end

endmodule

`default_nettype wire
