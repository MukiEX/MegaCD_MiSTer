# LaserActive (Mega-LD) support: step 1, PD6103A register logger

This branch starts LaserActive Sega PAC support in the MegaCD core, following
Nemesis's implementation in [ares](https://github.com/ares-emulator/ares)
(`ares/md/mcd/megald.cpp`). A LaserActive with a Sega PAC is a Mega CD plus one
extra chip, the Pioneer **PD6103A**, which connects the Mega CD sub-CPU to the
LaserDisc player.

Step 1 adds that chip's register block to the FPGA and logs everything the
BIOS does with it. The LaserDisc player itself, LaserDisc video, MMI loading
and analog audio come in later steps.

## What changed

| File | Change |
|---|---|
| `rtl/MCD/PD6103A.vhd` | New. The register block: 32 input and 32 output registers, freeze and hold behaviour, echo registers, a 512-entry access log |
| `rtl/MCD/ASIC.vhd` | Decodes `0xFD0000–0xFDFFFF` on the sub-CPU bus using the full address (A23–A16), with its own DTACK. That window is removed from the word-RAM decode, which only looks at A19–A1 and would otherwise alias it to `0x0D0000` |
| `rtl/MCD/MCD.vhd` | Instantiates the PD6103A and adds it to the sub-CPU read mux |
| `rtl/MCD/MCD.qip` | Adds `PD6103A.vhd` |
| `rtl/hps_ext.v` | Two new HPS commands: `0x50` LD_GET (drain the log) and `0x51` LD_SET (control, set output registers) |
| `MegaCD.sv` | Wiring, plus OSD option **LaserActive PD6103A: Off/On** (status bit 28) |

With the OSD option **Off** (the default), the core behaves exactly as before.

The Main_MiSTer side is a separate patch: `support/megacd/megald.cpp` and
`megald.h` (new), plus two hook lines in `support/megacd/megacd.cpp`.

## Register map (sub-CPU)

The chip only uses the lower byte lane (odd addresses).

- Input register *n* (written by the BIOS, read back as last written): `0xFD0001 + 2n`
- Output register *n* (driven by the player): `0xFD0081 + 2n`

What the FPGA implements itself, from ares' `MCD::LD::read`/`write`:

- **Input block freeze:** input reg 0x00 bit 7 = 0 holds input writes until bit 7 goes back to 1. Regs 0x19–0x1F always apply.
- **Output block freeze:** input reg 0x00 bit 6 = 1 snapshots the output block, and reads return the snapshot while it's set.
- **Retained CPU writes:** output reg 0x00 bits 1–5 and output regs 0x1A–0x1F aren't driven by the player, so CPU writes to them stick.
- **Hold after write:** a CPU write to an unfrozen output register reads back for the next 5 reads, then the driven value returns.
- **Hardware echoes,** so handshakes don't wait for the ARM:
  - output 0x00 = `1, 0, (retained bits 5–1), applied input 0x00 bit 0`
  - output 0x0A–0x0F = applied input 0x06–0x0B
  - output 0x10 = applied input 0x05
- **Constants:** output 0x01 = `0x80`, output 0x05 = `0x7F` (no remote buttons pressed).

Every other output register is set from the ARM side.

## Running it

1. Build the core and Main_MiSTer with both patches, and install both.
2. BIOS: name the LaserActive **Sega PAC** BIOS `cd_bios.rom` and put it in the
   same folder as the disc image, or name it `boot.rom` in the core's home
   folder. It's loaded in place of the Mega CD BIOS; ares loads it the same way.
3. In the OSD, set **LaserActive PD6103A** to **On**, then reset.
4. Disc (optional for this step): an `.mmi` is an uncompressed ZIP, so any
   unzip tool can extract `DigitalAudio.cue` and `DigitalAudio.img` from it.
   Load that `.cue` with *Insert Disk*. This is only the digital track, so
   don't expect a game to run yet.
5. The log is written to `/media/fat/laseractive_pd6103a.log`.

### Log format

```
--- t=48211 ms, 3 entries ---
W in  00 = 81  10000001
W in  06 = 12  00010010
R out 02 = 80  10000000  (snapshot/held)
```

- `W` lines are CPU writes, and `R` lines are CPU reads (only if read logging is on).
- `in` and `out` give the block, followed by the register number and the value.
- `(snapshot/held)` means the read returned a frozen snapshot or a post-write hold, not the live value.
- An overflow line means the 512-entry buffer filled before the ARM drained it (it polls every 5 ms).

### Setting output registers without rebuilding

Create `/media/fat/laseractive_out.txt`. It's re-read within a second of
changing and re-applied after every reset:

```
# Values ares reports for a LaserDisc in the tray, door closed, not spinning
# (MCD::LD::getOutputRegisterValue with drive state 0x02)
out 02 80
out 04 20
out 06 02
out 15 01

# Uncomment to log reads too (busy: the BIOS polls constantly)
# log_reads 1
```

Writes to the echo registers (0x00, 0x0A–0x10) are ignored, since the
hardware generates those.

## What to expect

The BIOS should boot, and the log shows its conversation with the chip. Where
the BIOS stops is the useful result: the last writes before it starts polling
say which output register it's waiting for. Compare that against
`getOutputRegisterValue` in ares and try values in `laseractive_out.txt`.
That's the input for step 2 (player logic in Main_MiSTer).

Known gaps for this step:

- ares also resets the PD6103A when the main CPU performs the gate-array reset
  sequence that hands over to a Mega CD game (`io-external.cpp`). This isn't
  done yet, so booting a regular Mega CD game through the LaserActive BIOS may
  stall at that handshake.
- When the output block is frozen, the snapshot uses the ARM's latest values
  rather than values computed at that instant.

## Verification done

- **GHDL testbench** of the real `ASIC.vhd` decode and DTACK together with
  `PD6103A.vhd`, driving sub-CPU bus cycles (52 checks):
  - reset values and constants
  - input read-back
  - ARM-driven outputs
  - 5-read hold
  - output freeze snapshot
  - retained registers
  - output 0x00 bit merging
  - echoes and input-block freeze/unfreeze
  - upper-byte-only accesses
  - no word-RAM writes during PD6103A accesses
  - log order and content
  - read logging
  - 512-entry overflow and clear
  - no decode with the option off
- **Verilator testbench** of `hps_ext.v` driven with the same word sequences
  Main_MiSTer sends, against a model of the log FIFO.
- All changed VHDL also analyses in VHDL-93 mode.
- `megald.cpp` and the modified `megacd.cpp` compile warning-free with
  `-Wall -Wextra` and the target's unsigned `char`.
- **Not done:** a Quartus build, or anything on real hardware.
