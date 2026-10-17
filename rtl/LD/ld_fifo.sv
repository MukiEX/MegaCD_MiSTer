// ld_fifo.sv - first-word-fall-through FIFO in block RAM.
// q/valid show the oldest word; pop takes it. One push and one pop per clock.
// SPDX-License-Identifier: MIT
`default_nettype none

module ld_fifo #(
	parameter W  = 64,
	parameter AW = 9
) (
	input  wire          clk,
	input  wire          flush,
	input  wire          push,
	input  wire  [W-1:0] din,
	input  wire          pop,
	output wire  [W-1:0] q,
	output wire          valid,
	output reg  [AW:0]   count
);

reg [W-1:0] mem [0:(1 << AW) - 1];
reg [AW-1:0] wr_ptr, rd_ptr;
reg [W-1:0] q_mem, byp_d;
reg byp;

wire do_pop  = pop && count != 0;
wire do_push = push && (count != (1 << AW) || do_pop);
wire [AW-1:0] raddr = do_pop ? rd_ptr + 1'd1 : rd_ptr;

always @(posedge clk) begin
	if (do_push) mem[wr_ptr] <= din;
	q_mem <= mem[raddr];
	// the RAM returns old data when the address being read is written in the same clock
	byp <= do_push && wr_ptr == raddr;
	byp_d <= din;
end

always @(posedge clk) begin
	if (flush) begin
		wr_ptr <= 0;
		rd_ptr <= 0;
		count <= 0;
	end else begin
		if (do_push) wr_ptr <= wr_ptr + 1'd1;
		if (do_pop) rd_ptr <= rd_ptr + 1'd1;
		count <= count + (do_push ? 1'd1 : 1'd0) - (do_pop ? 1'd1 : 1'd0);
	end
end

assign q = byp ? byp_d : q_mem;
assign valid = count != 0;

endmodule

`default_nettype wire
