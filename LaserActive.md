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

# Step 3: the game drives the disc

A `.dscmmi` now goes in the CD drive, and the LaserActive BIOS and games
play it through the PD6103A: spin-up, seeks, play, stop points, still
frames, frame stepping, scanning and picture stop codes all work the way
ares does them. The analog audio plays too. The Mega Drive graphics aren't
drawn over the video yet (step 4): while the player shows the disc picture,
it fills the screen.

## Running it

1. Install the new core and `MiSTer_LD` (both changed; an older pair
   won't talk to each other).
2. Put the **Sega PAC** BIOS in the same folder as the `.dscmmi`, named
   `cd_bios.rom`.
3. In the OSD, set **LaserActive PD6103A** to **On**, then use
   **Insert Disk** and pick the `.dscmmi`. The game boots like a Mega CD
   game.
4. **LaserDisc Video: Auto/Hide.** Auto shows the disc picture whenever the
   player would show it (mixing on, not blanked, a frame latched). Hide
   keeps the Mega Drive picture up.

The player writes `/media/fat/laseractive.log`: the disc it mounted, every
drive command, seek, stop point and picture stop, and a video line every
10 s. For the raw register traffic as well, create
`/media/fat/laseractive.cfg` containing `log_regs 1` (and `log_reads 1`
for reads too); that goes to `/media/fat/laseractive_pd6103a.log`.

**Load LD Video Test** still plays a file's video on its own, from frame 0,
without the BIOS.

## How it works

| Part | What it does |
|---|---|
| Main_MiSTer `megald.cpp` | The player: a port of ares' `MCD::LD`. It drains the core's PD6103A log every millisecond, applies each input write the way ares does (freeze and unfreeze order included), and sends back every output register that changed. |
| Main_MiSTer `megacdd.cpp` | The CD drive in LaserDisc mode. The digital track is read from inside the archive. Each sector tick sends the sector's data and one sector of audio (digital plus the analog track, mixed with the player's faders), then asks the player how far to move: 1 for play, 0 for a still, 2 to 20 for scanning, negative for reverse. The ticks are paced by the core's CD audio FIFO, so the disc runs at exactly 75 sectors per second on the core's clock. |
| Main_MiSTer `megald_video.cpp` | Video frames: a 16-slot cache in DDR3 (512 KB each), filled by a reader thread with the frames the player expects next, and a commit to the core for each new frame the disc reaches. |
| Main_MiSTer `megald_mmi.cpp` | Reads the archive's directory, `MediaInfo.json`, `DSCMMI.json` and the `.dscv` header and VBI table. |
| `rtl/LD/ld_video.sv` | Commits wait in a 4-entry queue. Each 29.97 Hz tick shows the oldest commit if two are waiting or it waited a whole tick (that absorbs the ARM's timing jitter); otherwise the last picture stays up. If three commits have waited a tick, the oldest is skipped. `visible` (from the player) and the OSD gate the framebuffer. |
| `rtl/MCD/PD6103A.vhd` | **Command busy:** an input write that changes a command (input 0x00 bit 0, 0x02 bits 0–5, the seek registers 0x06–0x0B) sets bit 7 of output 0x06 until Main has processed it, so code polling for "done" never sees the old state as finished. **Gate-array reset:** clears inputs 0x00–0x0B and the output block as ares does, and puts a marker in the log so the player resets at the same point. |
| `rtl/hps_ext.v` | `0x51` op 2 acknowledges the busy bit; `0x52` op 3 commits a slot, op 0 bit 2 is `visible`; `0x53` adds the queue level, skipped commits and a protocol version (3). |
| `MegaCD.sv` | "Original" aspect is 4:3 while the disc picture is up, and the 216p crop is off. |

Differences from ares, all in `megald.cpp`'s header comment: output
registers are computed after each change rather than per read; the
drive-state delay on output 0x06 is 20 ms instead of 10 reads; a seek
counts as finished when its first frame is on screen; picture stop codes
come from the `.dscv` VBI table.

## Testing done

- **Main_MiSTer side:** a host test harness (`ldtest/harness.cpp` in the
  package) runs the player, the CD drive's LaserDisc mode, the archive
  reader and the frame cache against a model of the core (PD6103A, hps_ext,
  the commit queue, the CD audio FIFO), on a test disc with a data track, an
  audio track, analog audio carrying a per-sector marker, and a picture stop
  code. It checks spin-up, a frozen-block seek to a frame, 2 s of play (60
  frames, each in order on screen, 150 audio sectors with the analog audio
  in sequence), a stop point, resuming by toggling input 0x00 bit 0, single
  steps forward and back, quarter-speed reverse, the picture stop code, 3×
  scan, pause blanking and image hold, the output freeze, and a gate-array
  reset.
- **Core:** the Verilator test of `ld_video.sv` now commits frames with
  jitter, a jump back, a repeated frame, a still and a core reset mid-decode;
  every picture matches the C decoder. The GHDL test of `PD6103A.vhd` adds
  the busy bit and the gate-array reset.
- **Not done:** a run with the real BIOS and a real game. The video lags the
  drive by about two frames (the commit queue plus decoding).

# Step 4: the Mega Drive picture over the disc

The VDP's graphics are now mixed over the LaserDisc picture inside the core,
the way the Sega PAC does it (ares `MCD::LD::scanline`), and the result goes
out as the core's normal video: HDMI and the analog output alike, so it
works on a CRT.

## What you see

- **Full LD resolution.** With the PD6103A on, every VDP pixel goes out as
  two pixels (640 across the H40 active area, 512 in H32), so the disc
  picture keeps about its full horizontal resolution under the 320-pixel
  graphics.
- **Placement as in ares.** A line of 1495 VDP dots covers the 792 kept
  columns of the disc picture; the H40 active area starts at column 63.6,
  H32 at column 55.6. Active line 0 is line 30 of the field. Each output
  pixel interpolates the two nearest disc columns.
- **Mixing:** `out = VDP × f + LD × (1 − f)`, where `f` is input 0x1A
  (0x1B for backdrop, border and blanking pixels) divided by 4, as a
  fraction of 63. Mixing mode 0 (input 0x01 bits 7–6) shows the VDP picture
  alone. While the player blanks the disc picture (paused without image
  hold, or input 0x0C bit 2) the disc side mixes in as black.
- **Fields.** At the start of each VDP frame: field select (mixing mode 2 or
  3 with input 0x0C bit 1 or 3 set) shows the field in bit 0; otherwise a new
  disc frame starts on its first field, and if the picture is interlaced
  (input 0x0C bits 5, 3 and 1 all clear) the fields alternate every VDP
  frame. Time Gal (0x01 = 0x41, 0x0C = 0x8A) gets the first field only,
  i.e. 240p.
- **480i.** When the disc picture is interlaced, the core outputs real 480i:
  the VDP delays VSync by half a line on the first field and the field flag
  follows the disc, as a LaserActive locks the Mega Drive to the player. A
  CRT interlaces it; the HDMI scaler weaves it.
- **Less delay.** A committed frame is decoded at once into a spare buffer
  (three frame buffers now) and put up at the start of a VDP frame. A
  picture stays up for two VDP frames (59.92 Hz against the disc's 29.97).
  The core averages how long finished pictures wait, and moves the cadence
  by one VDP frame when that wait gets too short for the HPS's timing
  jitter, or longer than needed. On average a frame reaches the screen
  about 45 ms after the HPS commits it (it was up to 100 ms before).

## Changes

| Part | What changed |
|---|---|
| `rtl/LD/ld_mix.sv` (new) | The mixer. Runs on the VDP's pixel clock: two output pixels per VDP pixel, 8 clocks of delay, syncs and blanks delayed to match. Keeps two disc lines in a line buffer (four M10Ks; 7-7-6 bits per pixel) and asks `ld_video` for the line after next as each line ends. Picks the disc frame and field at the start of each VDP frame (VBlank). |
| `rtl/LD/ld_video.sv` | Three frame buffers (`0x30A00000`, `0x30C00000`, `0x30E00000`); the decode starts as soon as a commit is waiting; pictures go up at VDP frame starts with the cadence control above. Line reads for the mixer come first on the DDR3 port (bursts in flight are tagged, since the decoder reads at the same time). The decoder's FIFOs are halved (256 words). The 29.97 Hz tick is gone. |
| `rtl/GEN/vdp.vhd`, `gen.sv` | New outputs: `BACKDROP` (the pixel shows the backdrop colour, border or blank) and the V counter of the line being output; new input `LD_VS_DELAY` (half-line VSync delay for the first field of an interlaced disc picture). |
| `rtl/MCD/PD6103A.vhd`, `MCD.vhd` | `MIX` output: inputs 0x01, 0x0C, 0x1A and 0x1B as they are in effect. |
| `rtl/hps_ext.v` | `0x52` op 4: picture position (horizontal offset in 1/256 disc columns, vertical in lines). Protocol version 4. |
| `MegaCD.sv` | The mixer sits between the VDP and `video_mixer` while the PD6103A is on and **LaserDisc Video** is **Auto**; cofi (composite blending) and the scandoubler are bypassed then (the scandoubler's line buffers hold 320 pixels). `VGA_F1` follows the disc's fields in 480i. The framebuffer (`MISTER_FB`), the 4:3 override and **Load LD Video Test** are gone. |
| Main_MiSTer | Sends the picture position from `laseractive.cfg`, protocol version 4, and the video test is removed. |

## Running it

As in step 3: install the new core and `MiSTer_LD` together, launch with the
MGL, PD6103A **On**, insert the `.dscmmi`. **LaserDisc Video: Hide** now
switches the mixer off altogether (plain Mega Drive output).

If the disc picture sits off from the graphics on some game, nudge it in
`/media/fat/laseractive.cfg` (re-read every second, no restart needed):

```
# move the disc picture right by 1.5 disc columns (negative: left)
video_x 1.5
# ... and down by 2 lines (negative: up)
video_y 2
```

**On a CRT:** use the analog output at 15 kHz (direct video, or VGA with
the scandoubler off). With **Forced scandoubler** on, a 31 kHz monitor gets
15 kHz while the PD6103A is on, since the mixer bypasses the scandoubler.

## Testing done

- **Core:** a new Verilator test (`dscmmi/rtl/tb/tb_ld_mix.cpp`) runs
  `ld_video` and `ld_mix` together against a model of the VDP's timing
  (H40 with its 9- and 10-clock hsync slots, and H32; 262 lines with the
  real V counter sequence) with random pixels and backdrop flags. The "HPS"
  commits frames at 29.97 Hz with ±5 ms jitter and a still, the DDR3 model
  adds random wait states, and a core reset hits mid-decode. Every output
  pixel is checked against a model working from the C decoder's picture,
  through a register program covering field 1 only, interlaced, field
  select (both fields), mixing off and full VDP fader. Also checked: the
  field flag, 480i flag and VSync delay at every VDP frame, and the picture
  cadence. Runs: H40, H32 with border and offsets (+300/256 column,
  −3 lines), H40 with border and offsets (−200/256, +5), H32.
- **VDP:** `vdp.vhd` analyses and elaborates in GHDL; the PD6103A GHDL test
  still passes.
- **Main_MiSTer:** the harness's video model follows the new scheduler and
  it checks the position from `laseractive.cfg`; all its tests pass.
- **Not done:** a Quartus build (RAM is tight: the line buffer takes four
  M10Ks, the halved FIFOs give four back) and a run on hardware.

## Fields and deinterlacing options

Two OSD entries, under **LaserDisc Video**:

- **LaserDisc Fields: Auto / Both (480i).** Auto does what the game asks
  (Time Gal's field memory shows the first field only, as on real
  hardware). Both ignores that and alternates both fields on every VDP
  frame, strictly odd/even, so the disc picture keeps all 480 lines: true
  480i on a CRT. It also overrides a game's explicit field choice, so keep
  Auto if something looks wrong.
  Each disc frame starts on its first field, then shows its second: a new
  frame only goes up on a VDP frame that shows a first field. (The first
  version alternated fields without regard to frame starts, so half the
  time a frame's fields went out second-then-first, backwards in time,
  and motion juddered.) Late frames now wait two VDP frames instead of
  one, so the order never breaks.
- **LaserDisc Field Order: Normal / Reversed.** With Both, which of a
  frame's two fields goes out first. Normal follows the order stored in the
  disc image (the first field is rows 0-262 of the capture), as ares does.
  If motion still judders on some disc, try Reversed.
- **Deinterlace (HDMI): Weave / Bob.** How the HDMI scaler shows 480i
  (MiSTer's own deinterlacer, `HDMI_BOB_DEINT`). Weave shows both fields
  together: sharpest, but telecined film shows comb lines on motion. Bob
  shows each field line-doubled: no combing, a little softer. It applies to
  the Mega Drive's own interlace modes too. No effect on the analog output.

Neither uses any RAM.
