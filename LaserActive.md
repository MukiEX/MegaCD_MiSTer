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

# Step 2: LaserDisc video test (.dscmmi playback)

This test plays the analog video of a `.dscmmi` file (made by
`mmidsc convert`) full screen, decoded by the FPGA. It doesn't touch the
game side: the Mega CD keeps running underneath, and the OSD switches the
picture between the two.

## How it works

| Part | What it does |
|---|---|
| Main_MiSTer `support/megacd/megald_video.cpp` | Opens the `.dscmmi`, finds the `.dscv` stream, sends the core its DSC PPS and frame size, then a reader thread copies frames into an 8-slot ring in DDR3 (`0x30100000`, 1 MB per slot). It stays at most 7 frames ahead of the core and logs progress every 5 s. |
| `rtl/LD/ld_video.sv` | Every 1/29.97 s takes the next frame from the ring, decodes it, and writes the picture to one of two framebuffers in DDR3 (`0x30A00000`, `0x30C00000`). The two fields stored in the picture are woven back into alternate lines. |
| `rtl/LD/dsc_decoder.sv` | The DSC decoder. Any rate (8, 9 or 10 bpp) comes from the file's PPS. |
| `rtl/LD/ld_fifo.sv` | FIFOs between DDR3 and the decoder. |
| `rtl/hps_ext.v` | Commands `0x52` LD_VSET (enable/restart, PPS bytes, frame size, frames written) and `0x53` LD_VGET (frames shown, late ticks, decode errors, decode time). |
| `MegaCD.sv` | Wiring; the DDR3 port (unused before, since PRG-RAM lives in SDRAM); the core framebuffer (`MISTER_FB`); OSD entries. |
| `MegaCD.qsf`, `MegaCD_Q13.qsf` | `VERILOG_MACRO "MISTER_FB=1"`. |

**Simulation:** the whole path (ring in DDR3, decoder, line writes,
weaving, double buffering) was run against a DDR3 model with random wait
states and read latency, using frames from 8, 9 and 10 bpp files. Every
pixel of every framebuffer matched the C reference. The runs also covered a
stalled ARM side (counted as late ticks) and a core reset in the middle of a
frame.

Each frame takes about 1.32 M clocks of the 1.79 M available at 53.69 MHz.

## Running it

1. Build the core (Quartus 17.0 Lite, `MegaCD.qpf`) and Main_MiSTer with both
   patches, and install both.
2. Load the core. In the OSD, choose **Load LD Video Test** and pick a
   `.dscmmi`. The file picker shows names whose extension starts with `ds`.
3. The picture replaces the Mega CD's output as soon as the first frame
   decodes. **LD Video Test: Show/Hide** switches back to the Mega CD
   picture without stopping playback.
4. Playback starts at frame 0 and loops at the end of the disc side. To stop
   it, load the core again.

Main_MiSTer prints a line every 5 seconds. To see it, run it from a console
(stop the running copy, then start `/media/fat/MiSTer` over SSH):

```
LDV: shown 1500 (disc frame 1500), ring 7, late ticks 0, decode errors 0, decode 23.8 ms, read errors 0
```

- **ring:** frames waiting in DDR3. It should sit at 6–7; if it drops to 0,
  storage can't keep up.
- **late ticks:** frame periods with no new frame ready. They should stay at
  0 once playback is running.
- **decode errors:** frames the decoder rejected. They should be 0.
- **decode:** the time one frame takes in the FPGA, under the 33.4 ms frame
  period.

## What to look for

- **No picture, Mega CD output only:** check the log line. If `shown` stays
  at 0, the core isn't decoding. If Main says the core has no LD video
  player, the core and Main_MiSTer builds don't match.
- **Red and blue swapped:** the framebuffer byte order is set in
  `ld_video.sv` (`fb_format`). It's an easy fix if so.
- **Combing on motion:** expected. The two fields are woven into one frame,
  as they would be on a progressive display; there's no deinterlacing yet.
- **Picture size:** 792×525 including the VBI lines. The white dashes at the
  top are the biphase frame codes, which shows they survived compression.

Loading a `.dscmmi` is remembered like a disc image, so the core starts
playing it again the next time it loads.
