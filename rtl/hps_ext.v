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
	output reg  [7:0] ld_arm_data
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
//  LD_GET: reply to the command word = {ovf, 5'b0, count[9:0]}; each further
//          word returns and removes one log entry. Read at most 'count' words.
//  LD_SET: word 1 = op. op 0: next word = control {.., clear_ovf, log_reads}.
//          op 1: each further word = {3'b0, reg[4:0], data[7:0]}, sets the
//          driven value of one output register.
localparam LD_GET = 'h50;
localparam LD_SET = 'h51;

initial begin
	ld_log_pop = 0;
	ld_log_reads = 0;
	ld_log_ovf_clr = 0;
	ld_arm_we = 0;
	ld_arm_reg = 0;
	ld_arm_data = 0;
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

	ld_log_pop <= 0;
	ld_log_ovf_clr <= 0;
	ld_arm_we <= 0;

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
			dout_en <= (io_din >= EXT_CMD_MIN && io_din <= EXT_CMD_MAX) || io_din == LD_GET || io_din == LD_SET;
			if(io_din == CD_GET) io_dout <= cd_req; 
			if(io_din == LD_GET) io_dout <= {ld_log_ovf, 5'd0, ld_log_count};
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
					if(byte_cnt == 1) ld_op <= io_din[1:0];
					else if(ld_op == 0) begin
						ld_log_reads <= io_din[0];
						ld_log_ovf_clr <= io_din[1];
					end
					else if(ld_op == 1) begin
						ld_arm_reg <= io_din[12:8];
						ld_arm_data <= io_din[7:0];
						ld_arm_we <= 1;
					end
			endcase
		end
	end
end

endmodule
