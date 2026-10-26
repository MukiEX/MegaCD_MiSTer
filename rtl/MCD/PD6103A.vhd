-- Pioneer PD6103A register block (LaserActive Sega PAC), step 1 skeleton.
--
-- The PD6103A bridges the Mega CD sub-CPU to the LaserActive player. It sits
-- on the sub-CPU bus at 0xFD0000-0xFDFFFF, odd bytes only:
--   address bit 7    : 0 = input block (CPU -> player), 1 = output block
--   address bits 5-1 : register number 0x00-0x1F
--
-- Behaviour follows Nemesis's reverse engineering in ares
-- (ares/md/mcd/megald.cpp, MCD::LD::read / MCD::LD::write):
--   * Input registers read back the last value written.
--   * Input reg 0x00 bit 7 = 0 freezes the input block: writes are held and
--     only take effect (are "applied") when bit 7 is set again, except regs
--     0x19-0x1F which always apply.
--   * Output registers are driven by the player. Most of their "live" values
--     come from the ARM side (Main_MiSTer) through ARM_WE/ARM_REG/ARM_DATA.
--     A few are pure echoes of applied input registers and are generated here,
--     so handshakes do not wait on the ARM:
--       out 0x00: bit 7 = 1, bit 6 = 0, bit 0 = applied in 0x00 bit 0
--       out 0x0A-0x0F = applied in 0x06-0x0B, out 0x10 = applied in 0x05
--     ARM writes to these are ignored (bits 1-5 of out 0x00 still come from
--     CPU writes, as below).
--   * Input reg 0x00 bit 6 freezes the output block: a snapshot is taken on the
--     rising edge and reads return the snapshot while frozen.
--   * Output reg 0x00 bits 1-5 and output regs 0x1A-0x1F are not driven by the
--     player, so CPU writes to them are retained.
--   * A CPU write to an unfrozen output register is read back for the next 5
--     reads before the driven value reappears (the NEC PAC BIOS relies on this).
--
--   * Command pending: when an applied input write changes something the
--     player has to act on (input 0x00 bit 0, input 0x02 bits 0-5, or the
--     seek registers 0x06-0x0B), output 0x06 reads with bit 7 (busy) set
--     until the ARM has drained the log and acknowledges (ARM_ACK while the
--     log is empty). Games and the BIOS poll that bit after a command, and
--     must not see "done" before the ARM has seen the command.
--
-- Not handled here (by design, the ARM side owns it): everything the player
-- does in response to input register writes. Every CPU write (and optionally
-- every read) is pushed into a 512-entry log FIFO for the ARM to drain.
--
-- TOC table: outputs 0x11-0x14 report the disc's TOC entry picked by
-- input 0x05 (track number, A0/B0 first and last track, A1/B1 lead-out).
-- The BIOS writes 0x05 and reads the answer within microseconds, so the ARM
-- uploads all 256 possible answers (TOC_WE/TOC_ADDR/TOC_DATA = {11,12,13,14})
-- and the chip looks them up by the applied value of input 0x05. Input 0x05
-- values 0x00 and 0xFF mean "current position", which the ARM drives live.
--
-- Gate-array (peripheral) reset: while GA_RST is high, input registers
-- 0x00-0x0B, both freeze states and every output register go back to their
-- reset values (ares MCD::LD::power(reset, gateArrayReset = true); the
-- mixing registers 0x0C-0x1F keep their values). The LaserActive BIOS does
-- this while booting. The rising edge also puts a marker in the log
-- (0x7F5A, a value no access produces) so the ARM resets its player at the
-- same point in the command stream.
--
-- Log entry format (16 bits):
--   [15]   1 = read, 0 = write
--   [14]   1 = output block, 0 = input block
--   [13:9] register number
--   [8]    for reads: 1 = value came from the frozen snapshot or the
--          post-write hold; always 0 for writes
--   [7:0]  data written, or data returned to the CPU
--
-- Sub-CPU probe (debugging, PROBE_EN from the ARM's log_cpu): about 100
-- times a second, 7 entries in the write format with bit 8 set (no access
-- produces that): registers 0x10-0x12 the address of the last instruction
-- fetch (bits 23-16, 15-8, 7-0), 0x13-0x15 the last data access's address,
-- 0x16 status {0, data access was a read, IPL_N[2:0], HALT_N, RESET_N,
-- an instruction was fetched since the last sample}. Like reads, only
-- while the log is under half full.
--
-- SPDX-License-Identifier: GPL-3.0-or-later (same as MegaCD_MiSTer)

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity PD6103A is
	port(
		CLK			: in std_logic;
		RST_N			: in std_logic;

		-- Sub-CPU side. SEL is high for the whole access (decoded by the ASIC,
		-- including the data strobes), A(7) selects the block, A(5:1) the reg.
		SEL			: in std_logic;
		A				: in std_logic_vector(7 downto 1);
		RNW			: in std_logic;
		LDS_N			: in std_logic;
		DI				: in std_logic_vector(7 downto 0);
		DO				: out std_logic_vector(7 downto 0);

		-- ARM side: set driven output register values
		ARM_WE		: in std_logic;
		ARM_REG		: in std_logic_vector(4 downto 0);
		ARM_DATA		: in std_logic_vector(7 downto 0);
		ARM_ACK		: in std_logic;	-- clear the command-pending busy if the log is empty
		GA_RST		: in std_logic;	-- gate-array reset (Mega CD peripheral reset)
		TOC_WE		: in std_logic;
		TOC_ADDR		: in std_logic_vector(7 downto 0);
		TOC_DATA		: in std_logic_vector(31 downto 0);
		PENDING		: out std_logic;
		-- video mixing registers in effect: in 0x01, 0x0C, 0x1A, 0x1B
		MIX			: out std_logic_vector(31 downto 0);

		-- ARM side: access log
		LOG_READS	: in std_logic;
		PROBE_EN		: in std_logic;
		CPU_A			: in std_logic_vector(23 downto 1);
		CPU_AS_N		: in std_logic;
		CPU_FC		: in std_logic_vector(2 downto 0);
		CPU_RNW		: in std_logic;
		CPU_IPL_N	: in std_logic_vector(2 downto 0);
		CPU_HALT_N	: in std_logic;
		CPU_RESET_N	: in std_logic;
		LOG_POP		: in std_logic;
		LOG_DATA		: out std_logic_vector(15 downto 0);
		LOG_COUNT	: out std_logic_vector(9 downto 0);
		LOG_OVF		: out std_logic;
		LOG_OVF_CLR	: in std_logic;

		-- Debug
		OUT_FROZEN	: out std_logic
	);
end PD6103A;

architecture rtl of PD6103A is

	type reg_bank_t is array (0 to 31) of std_logic_vector(7 downto 0);
	type hold_bank_t is array (0 to 31) of unsigned(2 downto 0);

	signal IN_REGS		: reg_bank_t;	-- read-back value (last written)
	signal IN_APPLIED	: reg_bank_t;	-- values in effect (differs while frozen)
	signal IN_FROZEN	: std_logic;
	signal OUT_LIVE	: reg_bank_t;
	signal OUT_SNAP	: reg_bank_t;
	signal HOLD_DATA	: reg_bank_t;
	signal HOLD_CNT	: hold_bank_t;
	signal FROZEN		: std_logic;

	signal SEL_D		: std_logic;
	signal PEND			: std_logic;

	type toc_mem_t is array (0 to 255) of std_logic_vector(31 downto 0);
	signal TOC_MEM		: toc_mem_t;
	attribute ramstyle : string;
	attribute ramstyle of TOC_MEM : signal is "MLAB";
	signal TOC_Q		: std_logic_vector(31 downto 0);
	signal TOC_LIVE	: std_logic;
	signal GA_RST_D	: std_logic;

	-- Output value as driven by the chip: echoes, else the ARM/CPU value
	function drv(r : integer; live : reg_bank_t; app : reg_bank_t; pend : std_logic;
	             toc : std_logic_vector(31 downto 0); toc_live : std_logic) return std_logic_vector is
	begin
		case r is
			when 16#11# => if toc_live = '1' then return live(r); else return toc(31 downto 24); end if;
			when 16#12# => if toc_live = '1' then return live(r); else return toc(23 downto 16); end if;
			when 16#13# => if toc_live = '1' then return live(r); else return toc(15 downto 8); end if;
			when 16#14# => if toc_live = '1' then return live(r); else return toc(7 downto 0); end if;
			when 0 => return '1' & '0' & live(0)(5 downto 1) & app(0)(0);
			when 6 => return (live(6)(7) or pend) & live(6)(6 downto 0);
			when 16#0A# to 16#0F# => return app(r - 4);
			when 16#10# => return app(5);
			when others => return live(r);
		end case;
	end function;

	-- Log FIFO
	constant LOG_DEPTH_LOG2 : integer := 9;
	type log_mem_t is array (0 to 2**LOG_DEPTH_LOG2 - 1) of std_logic_vector(15 downto 0);
	signal LOG_MEM		: log_mem_t;
	signal WR_PTR		: unsigned(LOG_DEPTH_LOG2 downto 0);
	signal RD_PTR		: unsigned(LOG_DEPTH_LOG2 downto 0);
	signal LOG_WE		: std_logic;
	signal LOG_WDATA	: std_logic_vector(15 downto 0);
	signal OVF			: std_logic;
	signal COUNT		: unsigned(LOG_DEPTH_LOG2 downto 0);

	-- Sub-CPU probe
	signal PRB_AS_D	: std_logic;
	signal PRB_PC		: std_logic_vector(23 downto 0);
	signal PRB_DA		: std_logic_vector(23 downto 0);
	signal PRB_DRNW	: std_logic;
	signal PRB_GOT		: std_logic;
	signal PRB_TMR		: unsigned(18 downto 0);
	signal PRB_IDX		: unsigned(2 downto 0);
	signal PRB_S_PC	: std_logic_vector(23 downto 0);
	signal PRB_S_DA	: std_logic_vector(23 downto 0);
	signal PRB_S_ST	: std_logic_vector(7 downto 0);

begin

	OUT_FROZEN <= FROZEN;

	-- TOC lookup (registered read; input 0x05 settles long before the next CPU access)
	process( CLK )
	begin
		if rising_edge(CLK) then
			if TOC_WE = '1' then
				TOC_MEM(to_integer(unsigned(TOC_ADDR))) <= TOC_DATA;
			end if;
			TOC_Q <= TOC_MEM(to_integer(unsigned(IN_APPLIED(5))));
		end if;
	end process;
	TOC_LIVE <= '1' when IN_APPLIED(5) = x"00" or IN_APPLIED(5) = x"FF" else '0';
	PENDING <= PEND;
	MIX <= IN_APPLIED(16#01#) & IN_APPLIED(16#0C#) & IN_APPLIED(16#1A#) & IN_APPLIED(16#1B#);
	COUNT <= WR_PTR - RD_PTR;
	LOG_COUNT <= std_logic_vector(COUNT);
	LOG_OVF <= OVF;

	process( RST_N, CLK )
		variable r		: integer range 0 to 31;
		variable d		: std_logic_vector(7 downto 0);
		variable alt	: std_logic;
		variable chg	: std_logic;
	begin
		if RST_N = '0' then
			for i in 0 to 31 loop
				IN_REGS(i) <= (others => '0');
				IN_APPLIED(i) <= (others => '0');
				OUT_LIVE(i) <= (others => '0');
				OUT_SNAP(i) <= (others => '0');
				HOLD_DATA(i) <= (others => '0');
				HOLD_CNT(i) <= (others => '0');
			end loop;
			-- Power-on values from MCD::LD::power(reset, gateArrayReset = false)
			IN_REGS(16#1A#) <= x"FF";
			IN_REGS(16#1B#) <= x"FF";
			IN_REGS(16#1C#) <= x"FF";
			IN_REGS(16#1D#) <= x"FF";
			IN_APPLIED(16#1A#) <= x"FF";
			IN_APPLIED(16#1B#) <= x"FF";
			IN_APPLIED(16#1C#) <= x"FF";
			IN_APPLIED(16#1D#) <= x"FF";
			-- Constant outputs (MCD::LD::getOutputRegisterValue): 0x01 = 0x80,
			-- 0x05 = 0x7F (no remote buttons pressed)
			OUT_LIVE(16#01#) <= x"80";
			OUT_LIVE(16#05#) <= x"7F";
			IN_FROZEN <= '0';
			FROZEN <= '0';
			PEND <= '0';
			GA_RST_D <= '0';
			SEL_D <= '0';
			DO <= (others => '0');
			LOG_WE <= '0';
			LOG_WDATA <= (others => '0');
			PRB_AS_D <= '1';
			PRB_PC <= (others => '0');
			PRB_DA <= (others => '0');
			PRB_DRNW <= '0';
			PRB_GOT <= '0';
			PRB_TMR <= (others => '0');
			PRB_IDX <= (others => '0');
			PRB_S_PC <= (others => '0');
			PRB_S_DA <= (others => '0');
			PRB_S_ST <= (others => '0');
		elsif rising_edge(CLK) then
			LOG_WE <= '0';
			SEL_D <= SEL;

			-- Sub-CPU probe: the last instruction fetch and data access, sampled
			-- every 2^19 clocks
			PRB_AS_D <= CPU_AS_N;
			if PRB_AS_D = '1' and CPU_AS_N = '0' then
				if CPU_FC(1 downto 0) = "10" then
					PRB_PC <= CPU_A & '0';
					PRB_GOT <= '1';
				elsif CPU_FC /= "111" then
					PRB_DA <= CPU_A & '0';
					PRB_DRNW <= CPU_RNW;
				end if;
			end if;
			PRB_TMR <= PRB_TMR + 1;
			if PRB_TMR = 0 and PROBE_EN = '1' and PRB_IDX = 0 and COUNT(LOG_DEPTH_LOG2 downto LOG_DEPTH_LOG2 - 1) = "00" then
				PRB_S_PC <= PRB_PC;
				PRB_S_DA <= PRB_DA;
				PRB_S_ST <= '0' & PRB_DRNW & CPU_IPL_N & CPU_HALT_N & CPU_RESET_N & PRB_GOT;
				PRB_GOT <= '0';
				PRB_IDX <= "111";
			end if;

			-- The ARM has handled every logged write: drop the pending busy. A
			-- write in this same clock (below) sets it again.
			if ARM_ACK = '1' and COUNT = 0 and LOG_WE = '0' then
				PEND <= '0';
			end if;

			-- ARM updates to the driven output values. A CPU write in the same
			-- cycle (below) takes priority.
			if ARM_WE = '1' then
				OUT_LIVE(to_integer(unsigned(ARM_REG))) <= ARM_DATA;
			end if;

			-- One action per bus access, on the first cycle SEL is seen. Only the
			-- lower byte lane (odd address) reaches the chip; even-byte accesses
			-- are acknowledged by the ASIC but have no effect here.
			GA_RST_D <= GA_RST;
			if GA_RST = '1' then
				for i in 0 to 11 loop
					IN_REGS(i) <= (others => '0');
					IN_APPLIED(i) <= (others => '0');
				end loop;
				for i in 0 to 31 loop
					OUT_LIVE(i) <= (others => '0');
					OUT_SNAP(i) <= (others => '0');
					HOLD_CNT(i) <= (others => '0');
				end loop;
				OUT_LIVE(16#01#) <= x"80";
				OUT_LIVE(16#05#) <= x"7F";
				IN_FROZEN <= '0';
				FROZEN <= '0';
				PEND <= '0';
				if GA_RST_D = '0' then
					LOG_WE <= '1';
					LOG_WDATA <= x"7F5A";
				end if;
			elsif SEL = '1' and SEL_D = '0' and LDS_N = '0' then
				r := to_integer(unsigned(A(5 downto 1)));

				if RNW = '1' then
					alt := '0';
					if A(7) = '0' then
						d := IN_REGS(r);
					elsif HOLD_CNT(r) /= 0 then
						d := HOLD_DATA(r);
						HOLD_CNT(r) <= HOLD_CNT(r) - 1;
						alt := '1';
					elsif FROZEN = '1' then
						d := OUT_SNAP(r);
						alt := '1';
					else
						d := drv(r, OUT_LIVE, IN_APPLIED, PEND, TOC_Q, TOC_LIVE);
					end if;
					DO <= d;
					-- reads only while the log is under half full, so polling can
					-- never push out the writes the player depends on
					if LOG_READS = '1' and COUNT(LOG_DEPTH_LOG2 downto LOG_DEPTH_LOG2 - 1) = "00" then
						LOG_WE <= '1';
						LOG_WDATA <= '1' & A(7) & A(5 downto 1) & alt & d;
					end if;

				else
					d := DI;
					-- Input 0x00 writes that only toggle the output freeze (bit 6) are
					-- handled here and not logged: games poll by freezing and
					-- unfreezing the outputs thousands of times a second, which would
					-- flood the log. Bits 7 (input freeze) and 0 still go to the ARM.
					if not (A(7) = '0' and A(5 downto 1) = "00000" and
					        d(7) = IN_REGS(0)(7) and d(0) = IN_REGS(0)(0)) then
						LOG_WE <= '1';
					end if;
					LOG_WDATA <= '0' & A(7) & A(5 downto 1) & '0' & d;

					if A(7) = '0' then
						-- Input block
						IN_REGS(r) <= d;
						-- does this write change an applied command register?
						chg := '0';
						if r = 0 then
							if d(7) = '1' then
								if d(0) /= IN_APPLIED(0)(0) then chg := '1'; end if;
								if IN_FROZEN = '1' then
									if IN_REGS(2)(5 downto 0) /= IN_APPLIED(2)(5 downto 0) then chg := '1'; end if;
									for i in 6 to 11 loop
										if IN_REGS(i) /= IN_APPLIED(i) then chg := '1'; end if;
									end loop;
								end if;
							end if;
						elsif IN_FROZEN = '0' then
							if r = 2 and d(5 downto 0) /= IN_APPLIED(2)(5 downto 0) then chg := '1'; end if;
							if r >= 6 and r <= 11 and d /= IN_APPLIED(r) then chg := '1'; end if;
						end if;
						if chg = '1' then
							PEND <= '1';
						end if;
						if r = 0 then
							-- Output block freeze (bit 6)
							if d(6) = '1' and FROZEN = '0' then
								for i in 0 to 31 loop
									OUT_SNAP(i) <= drv(i, OUT_LIVE, IN_APPLIED, PEND, TOC_Q, TOC_LIVE);
								end loop;
							end if;
							FROZEN <= d(6);
							-- Input block freeze (bit 7 = 0). Unfreezing applies every
							-- held register, then this write itself.
							IN_FROZEN <= not d(7);
							if d(7) = '1' then
								if IN_FROZEN = '1' then
									for i in 1 to 31 loop
										IN_APPLIED(i) <= IN_REGS(i);
									end loop;
								end if;
								IN_APPLIED(0) <= d;
							end if;
						elsif IN_FROZEN = '0' or r >= 16#19# then
							IN_APPLIED(r) <= d;
						end if;

					elsif FROZEN = '1' then
						-- Output block, frozen: most locations are freely writable
						if r = 0 then
							OUT_SNAP(0) <= (d and x"7F") or (drv(0, OUT_LIVE, IN_APPLIED, PEND, TOC_Q, TOC_LIVE) and x"80");
							OUT_LIVE(0) <= (d and x"3E") or (OUT_LIVE(0) and x"C1");
						else
							OUT_SNAP(r) <= d;
							if r >= 16#1A# then
								OUT_LIVE(r) <= d;
							end if;
						end if;

					else
						-- Output block, not frozen: value is held for a few reads
						HOLD_DATA(r) <= d;
						HOLD_CNT(r) <= to_unsigned(5, 3);
						if r = 0 then
							OUT_LIVE(0) <= (d and x"3E") or (OUT_LIVE(0) and x"C1");
						elsif r >= 16#1A# then
							OUT_LIVE(r) <= d;
						end if;
					end if;
				end if;
			end if;

			-- Probe samples go into the log between the CPU's accesses to the chip
			if SEL = '0' and GA_RST = '0' and PRB_IDX /= 0 then
				LOG_WE <= '1';
				case PRB_IDX is
					when "111" => LOG_WDATA <= "0010000" & '1' & PRB_S_PC(23 downto 16);
					when "110" => LOG_WDATA <= "0010001" & '1' & PRB_S_PC(15 downto 8);
					when "101" => LOG_WDATA <= "0010010" & '1' & PRB_S_PC(7 downto 0);
					when "100" => LOG_WDATA <= "0010011" & '1' & PRB_S_DA(23 downto 16);
					when "011" => LOG_WDATA <= "0010100" & '1' & PRB_S_DA(15 downto 8);
					when "010" => LOG_WDATA <= "0010101" & '1' & PRB_S_DA(7 downto 0);
					when others => LOG_WDATA <= "0010110" & '1' & PRB_S_ST;
				end case;
				PRB_IDX <= PRB_IDX - 1;
			end if;
		end if;
	end process;

	-- Log FIFO: write side
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			WR_PTR <= (others => '0');
			OVF <= '0';
		elsif rising_edge(CLK) then
			if LOG_OVF_CLR = '1' then
				OVF <= '0';
			end if;
			if LOG_WE = '1' then
				if COUNT(LOG_DEPTH_LOG2) = '1' then
					OVF <= '1';
				else
					WR_PTR <= WR_PTR + 1;
				end if;
			end if;
		end if;
	end process;

	-- Log memory (inferred block RAM, registered read)
	process( CLK )
	begin
		if rising_edge(CLK) then
			if LOG_WE = '1' and COUNT(LOG_DEPTH_LOG2) = '0' then
				LOG_MEM(to_integer(WR_PTR(LOG_DEPTH_LOG2 - 1 downto 0))) <= LOG_WDATA;
			end if;
			LOG_DATA <= LOG_MEM(to_integer(RD_PTR(LOG_DEPTH_LOG2 - 1 downto 0)));
		end if;
	end process;

	-- Log FIFO: read side
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			RD_PTR <= (others => '0');
		elsif rising_edge(CLK) then
			if LOG_POP = '1' and COUNT /= 0 then
				RD_PTR <= RD_PTR + 1;
			end if;
		end if;
	end process;

end rtl;
