// Test top for tb_ld_mix: ld_video and ld_mix wired as in MegaCD.sv.
`default_nettype none
module ld_tb_top (
	input  wire        clk,
	input  wire        reset,
	input  wire        ctl_enable, ctl_restart, ctl_visible,
	input  wire        pps_we,
	input  wire  [6:0] pps_addr,
	input  wire  [7:0] pps_data,
	input  wire [17:0] frame_words,
	input  wire [10:0] field1_row,
	input  wire        commit_we,
	input  wire  [3:0] commit_slot,
	output wire [15:0] rd_count, late_count, drop_count, error_count, decode_kclks,
	output wire  [2:0] queue_level,
	output wire        active,

	input  wire        ce_pix,
	input  wire  [7:0] vdp_r, vdp_g, vdp_b,
	input  wire        vdp_backdrop,
	input  wire  [8:0] vdp_vcnt,
	input  wire        hbl, vbl, hs, vs, border,
	input  wire [31:0] regs,
	input  wire        show,
	input  wire        force_il,
	input  wire signed [15:0] hoff,
	input  wire signed [7:0]  voff,
	output wire        ce_out,
	output wire  [7:0] r_out, g_out, b_out,
	output wire        hbl_out, vbl_out, hs_out, vs_out,
	output wire        il, f1, vs_delay,

	output wire  [1:0] dbg_disp, dbg_back,
	output wire        dbg_setup, dbg_latch_new,
	output wire  [3:0] dbg_slot,
	output wire        dbg_lf_req,
	output wire  [9:0] dbg_lf_row,

	input  wire        DDRAM_BUSY,
	output wire  [7:0] DDRAM_BURSTCNT,
	output wire [28:0] DDRAM_ADDR,
	input  wire [63:0] DDRAM_DOUT,
	input  wire        DDRAM_DOUT_READY,
	output wire        DDRAM_RD,
	output wire [63:0] DDRAM_DIN,
	output wire  [7:0] DDRAM_BE,
	output wire        DDRAM_WE
);
wire        ld_on, disp_latch, latch_new, lf_req, lf_we, latch_ok, fixed_cadence;
wire [10:0] pic_rows;
wire  [9:0] lf_row;
wire  [8:0] lf_addr;
wire [39:0] lf_data;

ld_video vid (
	.clk(clk), .reset(reset),
	.ctl_enable(ctl_enable), .ctl_restart(ctl_restart), .ctl_visible(ctl_visible),
	.pps_we(pps_we), .pps_addr(pps_addr), .pps_data(pps_data),
	.frame_words(frame_words), .field1_row(field1_row),
	.commit_we(commit_we), .commit_slot(commit_slot),
	.rd_count(rd_count), .late_count(late_count), .queue_level(queue_level), .drop_count(drop_count),
	.error_count(error_count), .decode_kclks(decode_kclks), .active(active),
	.ld_on(ld_on), .pic_rows(pic_rows), .disp_latch(disp_latch), .latch_new(latch_new), .latch_ok(latch_ok), .fixed_cadence(fixed_cadence),
	.lf_req(lf_req), .lf_row(lf_row), .lf_we(lf_we), .lf_addr(lf_addr), .lf_data(lf_data),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

ld_mix mix (
	.clk(clk),
	.ce_pix(ce_pix), .vdp_r(vdp_r), .vdp_g(vdp_g), .vdp_b(vdp_b), .vdp_backdrop(vdp_backdrop),
	.vdp_vcnt(vdp_vcnt), .hbl(hbl), .vbl(vbl), .hs(hs), .vs(vs), .border(border),
	.regs(regs), .show(show), .ld_on(ld_on), .force_il(force_il), .swap_order(1'b0), .hoff(hoff), .voff(voff),
	.disp_latch(disp_latch), .latch_ok(latch_ok), .fixed_cadence(fixed_cadence), .latch_new(latch_new), .pic_rows(pic_rows),
	.lf_req(lf_req), .lf_row(lf_row), .lf_we(lf_we), .lf_addr(lf_addr), .lf_data(lf_data),
	.ce_out(ce_out), .r_out(r_out), .g_out(g_out), .b_out(b_out),
	.hbl_out(hbl_out), .vbl_out(vbl_out), .hs_out(hs_out), .vs_out(vs_out),
	.il(il), .f1(f1), .vs_delay(vs_delay)
);

assign dbg_disp = vid.disp_buf;
assign dbg_back = vid.back_buf;
assign dbg_setup = vid.frame_setup;
assign dbg_latch_new = latch_new;
assign dbg_slot = vid.cur_slot;
assign dbg_lf_req = lf_req;
assign dbg_lf_row = lf_row;
endmodule
`default_nettype wire
