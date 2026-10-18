Note: in this repository the RTL itself is in `rtl/LD/` of the core
(`dsc_decoder.sv`, `ld_fifo.sv`, `ld_video.sv`, `ld_mix.sv`); this folder
keeps its documentation and Verilator testbenches (`tb/`).

# dsc_decoder: FPGA DSC decoder for .dscmmi video

`dsc_decoder.sv` decodes the DSC pictures that `mmidsc convert` writes. It is
a single core for every rate: 8, 9 and 10 bits per pixel all use the same
build. The rate, and every rate-control parameter, comes from the 128-byte
PPS stored in each `.dscv` header. The testbench also covers 6 and 12 bpp.

It is written to match `src/dsc.c` bit for bit. `dsc.c` is the reference
model (see the caveat in the main README about VESA's C model).

## Interface

| Signal | Dir | Meaning |
|---|---|---|
| `clk`, `reset` | in | one clock domain, synchronous reset |
| `pps_we`, `pps_addr[6:0]`, `pps_data[7:0]` | in | write the PPS (bytes 128–255 of the `.dscv` header) before `start` |
| `start` | in | one-clock pulse: decode one frame |
| `busy`, `done` | out | `done` pulses once the frame is finished |
| `error` | out | sticky until the next `start`: unsupported PPS, or a stream that runs past its slice budget or references an empty colour-history entry |
| `in_data[63:0]`, `in_valid`, `in_ready` | in/out | the frame's bytes in order, little-endian in each word (frame byte 0 in `in_data[7:0]`) |
| `out_valid`, `out_ready` | out/in | one beat per group of 3 pixels |
| `out_x`, `out_y` | out | position of the group's leftmost pixel |
| `out_rgb0..2[23:0]` | out | `{R, G, B}` of the three pixels, left to right |

### Feeding frames

Send exactly `ceil(frame_bytes / 8)` words per frame, starting at the
frame's 512-aligned offset (`frames_offset + n × frame_stride`). Each
frame starts on a fresh word.

`in_ready` drops after the frame's last word. That means a reader can
queue the next frame's words straight away, and the decoder picks them up
after the next `start`.

### Output

Groups come out in raster order: slice 0 (rows 0–262), then slice 1 (rows
263–524). The last slice's padding row is decoded but not output.

`out_ready` back-pressures the decoder, so it can write straight into a
framebuffer port that sometimes stalls.

## Supported streams

These are checked from the PPS when `start` is pulsed. If a check fails,
`error` and `done` are raised.

- DSC 1.1, 8 bits per component, RGB, 4:4:4, constant bit rate.
- No block prediction.
- `linebuf_depth` ≥ 9.
- One slice per line, with a slice width that is a multiple of 3, at least
  9 and at most 3 × 2^`LB_AW` (3072 with the default `LB_AW = 10`).

`mmidsc` only writes streams like this.

## How it works

The core is a multi-cycle state machine that decodes one group (3 pixels)
per pass:

| State | Work |
|---|---|
| `MUX` | refill any substream holding fewer than 36 bits with a 48-bit mux word (Table 4-6), one word per clock |
| `FLAT` | flatness bits; captures the previous line's group `gx + 3` from the line buffer |
| `PFX` | luma prefix: size or ICH escape |
| `RES` | luma residuals plus chroma prefixes, or the three ICH indices |
| `CRES` | chroma residuals |
| `REC1` | MMAP blend filters and ICH entry lookup |
| `REC2` | prediction and reconstruction (MMAP, MPP or ICH); output beat; line buffer write |
| `RC1` | ICH history update; buffer model; linear transform |
| `RC2` | range selection, flatness override, short-term QP |

Storage:

- The previous line is a 2^`LB_AW` × 78-bit block RAM, one group per word.
  The same RAM is read and written, because a group's slot is only
  overwritten after the window has read it.
- A 15-pixel window of the previous line (groups `gx - 2` … `gx + 2`) feeds
  both MMAP prediction and the 7 previous-line ICH entries.
- The 32-entry colour history is held in registers.

**Speed** (measured): 10.5–10.7 clocks per group. A 792×525 frame
(138,600 groups) takes about 1.46 M clocks, so 29.97 frames/s needs about
44 MHz. The Mega CD core's 53.69 MHz system clock leaves about 18%
headroom. The states were split so that each does at most one clamp chain,
add-and-compare or multiply; the first Quartus build, before that split,
missed 53.69 MHz by 9.5 ns.

## Testing

```
cd tb && ./build.sh                          # needs Verilator 5 and gcc
./obj/Vdsc_decoder                           # synthetic pictures
./obj/Vdsc_decoder FILE.dscmmi 20            # 20 frames spread across a converted disc
./obj/Vdsc_decoder --many 32                 # 32 varied 792x525 frames at each of 8, 9, 10 bpp
```

The testbench links `src/dsc.c` and compares every pixel with
`dsc_decode_picture()`. It feeds the input with random stalls and the output
with random back-pressure, and it queues frames back to back.

Results here: 63 synthetic frames passed with 0 mismatches:

- 792×525 at 6, 8, 9, 10 and 12 bpp.
- 288×160 with three slices and a partial last slice, at 8, 9 and 10 bpp,
  8 content types, colour history on and off.

Also matched, with 0 mismatches:

- 96 more varied 792×525 frames (`--many 32`), 32 each at 8, 9 and 10 bpp;
- 18 frames from converted 8, 9 and 10 bpp `.dscmmi` files.

That is 177 frames in all.

## Size estimate

Yosys 0.70 (`synth_intel_alm -family cyclonev`, no Quartus run yet) gives:

| Resource | Used | DE10-Nano total |
|---|---|---|
| Logic | about 21,000 ALUTs (12,900 logic + 8,300 arithmetic) | 83,820 ALUTs |
| Flip-flops | 3,642 | — |
| M10K block RAMs | 8 | 553 |
| DSP blocks | 23 | 112 |

21,000 ALUTs is roughly 11,000–13,000 ALMs, a quarter to a third of the
chip. Treat it as an upper bound:

- Much of the arithmetic is written with 32-bit `integer`s, and yosys trims
  those widths less well than Quartus does.
- The `qe × 2^q` products should become shifts.

Both are the first things to tighten if the Mega CD core plus this decoder
runs short of room.

## The LaserActive player around it

- `ld_video.sv`: commit queue, DDR3 reads of the committed slot, the decoder,
  and writes into three woven frame buffers; serves line reads for the mixer.
- `ld_mix.sv`: mixes the VDP picture over the frame on screen, line by line,
  as the Sega PAC does (see its header and `LaserActive.md` in the core).
- `tb/tb_ld_mix.cpp` (built by `tb/build.sh`): both together against a model
  of the VDP's timing, the HPS and DDR3; every output pixel is checked
  against the C decoder's picture:

```
./tb/obj_mix/Vld_tb_top FILE.dscmmi [h32] [border] [hoff] [voff] [vframes]
```
