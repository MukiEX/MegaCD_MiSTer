//
// hps_ext for Mega CD
//
// Copyright (c) 2020 Alexey Melnikov
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////

module hps_ext
(
	input             clk_sys,
	inout      [35:0] EXT_BUS,

	// CD interface
	input      [48:0] cd_in,
	output reg [48:0] cd_out,

	input             cdda_ready,
	input             cd_data_ready,

	// LaserActive PD6103A interface
	input       [9:0] ld_log_count,
	input             ld_log_ovf,
	input      [15:0] ld_log_data,
	output reg        ld_log_pop,
	output reg        ld_log_reads,
	output reg        ld_log_ovf_clr,
	output reg        ld_arm_we,
	output reg  [4:0] ld_arm_reg,
	output reg  [7:0] ld_arm_data,
	output reg        ld_arm_ack,
	output reg        ld_toc_we,
	output reg  [7:0] ld_toc_addr,
	output reg [31:0] ld_toc_data,
	input             ld_pending,

	// LaserActive video (rtl/LD/ld_video.sv)
	output reg        ldv_enable,
	output reg        ldv_restart,
	output reg        ldv_visible,
	output reg        ldv_pps_we,
	output reg  [6:0] ldv_pps_addr,
	output reg  [7:0] ldv_pps_data,
	output reg [17:0] ldv_frame_words,
	output reg [10:0] ldv_field1_row,
	output reg        ldv_commit_we,
	output reg  [3:0] ldv_commit_slot,
	input      [15:0] ldv_rd_count,
	input      [15:0] ldv_late,
	input      [15:0] ldv_errors,
	input      [15:0] ldv_kclks,
	input             ldv_active,
	input       [2:0] ldv_level,
	input      [15:0] ldv_drops
);

assign EXT_BUS[15:0] = io_dout;
wire [15:0] io_din = EXT_BUS[31:16];
assign EXT_BUS[32] = dout_en;
wire io_strobe = EXT_BUS[33];
wire io_enable = EXT_BUS[34];

localparam EXT_CMD_MIN = CD_GET;
localparam EXT_CMD_MAX = CD_SET;

localparam CD_GET = 'h34;
localparam CD_SET = 'h35;

// LaserActive PD6103A. Not in the UIO_* list in Main_MiSTer user_io.h, so
// only this core's hps_ext answers them.
//  LD_GET: reply to the command word = {ovf, pending, 4'b0, count[9:0]};
//          each further word returns and removes one log entry. Read at most
//          'count' words.
//  LD_SET: word 1 = op. op 0: next word = control {.., clear_ovf, log_reads}.
//          op 1: each further word = {3'b0, reg[4:0], data[7:0]}, sets the
//          driven value of one output register.
//          op 2: acknowledge: clears the command-pending busy bit if the
//          log is empty.
//          op 3: TOC table, 3 words per entry: {index, out 11}, {out 12,
//          out 13}, {out 14, 8'h00}.
localparam LD_GET = 'h50;
localparam LD_SET = 'h51;

// LaserActive video.
//  LD_VGET: reply to the command word = 'LD' (16'h4C44); then commits done,
//           repeated ticks, decode errors, last decode time (clocks / 256),
//           {queue level[3:1], active[0]}, skipped commits, protocol version (3).
//  LD_VSET: word 1 = op. op 0: control {.., visible, restart, enable}. op 1:
//           each further word = {1'b0, addr[6:0], data[7:0]} of the DSC PPS.
//           op 2: frame_words[15:0], frame_words[17:16], field1_row.
//           op 3: each further word = slot[3:0] to commit.
localparam LD_VSET = 'h52;
localparam LD_VGET = 'h53;

initial begin
	ld_log_pop = 0;
	ld_log_reads = 0;
	ld_log_ovf_clr = 0;
	ld_arm_we = 0;
	ld_arm_reg = 0;
	ld_arm_data = 0;
	ld_arm_ack = 0;
	ld_toc_we = 0;
	ld_toc_addr = 0;
	ld_toc_data = 0;
	ldv_enable = 0;
	ldv_restart = 0;
	ldv_visible = 0;
	ldv_commit_we = 0;
	ldv_commit_slot = 0;
	ldv_pps_we = 0;
	ldv_frame_words = 0;
	ldv_field1_row = 0;
end

reg [15:0] io_dout;
reg        dout_en = 0;
reg  [9:0] byte_cnt;

always@(posedge clk_sys) begin
	reg [15:0] cmd;
	reg  [7:0] cd_req = 0;
	reg        old_cd = 0;
	reg  [1:0] get_cmd;
	reg        send_data_type;
	reg  [1:0] ld_op;
	reg  [1:0] toc_w;
	reg  [1:0] ldv_op;

	ld_log_pop <= 0;
	ld_log_ovf_clr <= 0;
	ld_arm_we <= 0;
	ld_arm_ack <= 0;
	ld_toc_we <= 0;
	ldv_restart <= 0;
	ldv_commit_we <= 0;
	ldv_pps_we <= 0;

	old_cd <= cd_in[48];
	if(old_cd ^ cd_in[48]) cd_req <= cd_req + 1'd1; 

	if(~io_enable) begin
		dout_en <= 0;
		io_dout <= 0;
		byte_cnt <= 0;
		if(cmd == 'h35) cd_out[48] <= ~cd_out[48]; 
	end
	else if(io_strobe) begin

		io_dout <= 0;
		if(~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;

		if(byte_cnt == 0) begin
			cmd <= io_din;
			dout_en <= (io_din >= EXT_CMD_MIN && io_din <= EXT_CMD_MAX) || io_din == LD_GET || io_din == LD_SET || io_din == LD_VSET || io_din == LD_VGET;
			if(io_din == CD_GET) io_dout <= cd_req; 
			if(io_din == LD_GET) io_dout <= {ld_log_ovf, ld_pending, 4'd0, ld_log_count};
			if(io_din == LD_VGET) io_dout <= 16'h4C44;
		end else begin
			case(cmd)
				CD_GET:
					if(!byte_cnt[9:3]) begin
						if (byte_cnt[2:0] == 1) begin
							get_cmd <= io_din[1:0];
							send_data_type <= io_din[2];
						end else begin
							if (get_cmd == 0) begin // Request Command data
								case(byte_cnt[2:0])
									2: io_dout <= cd_in[15:0];
									3: io_dout <= cd_in[31:16];
									4: io_dout <= cd_in[47:32];
								endcase
							end else if (get_cmd == 1) begin // Request ready for data
								if (byte_cnt[2:0] == 2) begin
									io_dout <= {15'd0, send_data_type ? cd_data_ready : cdda_ready};
								end
							end
						end
					end

				CD_SET:
					if(!byte_cnt[9:3]) begin
						case(byte_cnt[2:0])
							1: cd_out[15:0]  <= io_din;
							2: cd_out[31:16] <= io_din;
							3: cd_out[47:32] <= io_din;
						endcase
					end

				LD_GET:
					begin
						io_dout <= ld_log_data;
						if(ld_log_count != 0) ld_log_pop <= 1;
					end

				LD_SET:
					if(byte_cnt == 1) begin
						ld_op <= io_din[1:0];
						toc_w <= 0;
					end
					else if(ld_op == 0) begin
						ld_log_reads <= io_din[0];
						ld_log_ovf_clr <= io_din[1];
					end
					else if(ld_op == 1) begin
						ld_arm_reg <= io_din[12:8];
						ld_arm_data <= io_din[7:0];
						ld_arm_we <= 1;
					end
					else if(ld_op == 2) ld_arm_ack <= 1;
					else begin
						case(toc_w)
							0: {ld_toc_addr, ld_toc_data[31:24]} <= io_din;
							1: ld_toc_data[23:8] <= io_din;
							2: begin
									ld_toc_data[7:0] <= io_din[15:8];
									ld_toc_we <= 1;
								end
						endcase
						toc_w <= (toc_w == 2) ? 2'd0 : toc_w + 2'd1;
					end

				LD_VGET:
					case(byte_cnt)
						1: io_dout <= ldv_rd_count;
						2: io_dout <= ldv_late;
						3: io_dout <= ldv_errors;
						4: io_dout <= ldv_kclks;
						5: io_dout <= {12'd0, ldv_level, ldv_active};
						6: io_dout <= ldv_drops;
						7: io_dout <= 16'd3;
						default: ;
					endcase

				LD_VSET:
					if(byte_cnt == 1) ldv_op <= io_din[1:0];
					else case(ldv_op)
						0: begin
								ldv_enable <= io_din[0];
								ldv_restart <= io_din[1];
								ldv_visible <= io_din[2];
							end
						1: begin
								ldv_pps_addr <= io_din[14:8];
								ldv_pps_data <= io_din[7:0];
								ldv_pps_we <= 1;
							end
						2: case(byte_cnt)
								2: ldv_frame_words[15:0] <= io_din;
								3: ldv_frame_words[17:16] <= io_din[1:0];
								4: ldv_field1_row <= io_din[10:0];
								default: ;
							endcase
						3: begin
								ldv_commit_slot <= io_din[3:0];
								ldv_commit_we <= 1;
							end
					endcase
			endcase
		end
	end
end

endmodule
