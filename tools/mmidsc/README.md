# mmidsc

`mmidsc` converts LaserActive `.mmi` images (the format ares uses) into
`.dscmmi` files. In a `.dscmmi`, the analog video is compressed with VESA
Display Stream Compression (DSC) at a **constant bit rate**, and everything
else in the image is copied unchanged.

The point is storage bandwidth. On a MiSTer, SD and USB both read at about
22 MB/s. Time Gal's lossless QON video needs 18.6 MB/s on average and
25.6 MB/s in its busiest second. With DSC at 8 bits per pixel, every frame
is the same size:

| | Rate | Notes |
|---|---|---|
| Original QON (Time Gal) | 18.6 MB/s average, 25.6 MB/s worst second | varies with content |
| `.dscmmi`, 8 bpp | **12.49 MB/s, every second** | 416,768 bytes per frame |
| `.dscmmi`, 9 bpp | **14.06 MB/s** | 468,992 bytes per frame |
| `.dscmmi`, 10 bpp | **15.62 MB/s** | 521,216 bytes per frame |

All three rates fit under the 22 MB/s the MiSTer reads, and one FPGA
decoder (`rtl/`) plays all of them.

Because frames are fixed size, frame *n* is at a fixed offset, so seeking
needs no index. The original `.mmi` stays the archival master; a `.dscmmi`
is a playback copy for hardware that can't stream the original.

## Quick start

Build it with `make` (Linux, macOS; `make windows` cross-compiles a Windows
`.exe`), or download a binary from the repository's releases.

Or skip the command line: open `web/mmidsc.html` in Chrome or Edge, pick the
`.mmi`, and press **Convert and save** (see `web/README.md`). It writes the
same files.

For a folder of discs on a NAS or another machine, run the server there and
convert from any browser. It's one Python 3.8+ file, standard library only (no
venv or pip), and it drives this native `mmidsc`, so it runs at full speed and
nothing goes through the browser:

    python3 server/mmidsc_server.py /path/to/discs --host 0.0.0.0

Open `http://that-machine:8642`, tick the discs, pick a quality and press
**Convert**. Jobs queue and run one at a time; each file is written next to its
source as `NAME.8bpp.dscmmi` (or `--out DIR`). It looks for `mmidsc` next to the
script, then on `PATH` (or use `--mmidsc PATH`). There is no login, so it only
listens on the machine itself unless you pass `--host`.

### 1. Measure quality and bandwidth on a real disc

```
mmidsc analyze "Time Gal (J) - PEASJ5039 [2025-08-19].mmi" --png timegal_png
```

This encodes 300 frames spread across the disc side, decodes them again, and
reports:

- **Quality:** PSNR against the lossless source (mean, median, 5th
  percentile, and the worst frame).
- **Bandwidth:** DSC's fixed rate, next to lossless QOI2 on the same cropped
  picture.
- **VBI codes:** whether the biphase codes on lines 16/17/279/280 still decode
  from the compressed picture. `.dscmmi` files also store them separately, so
  this is informational.
- **Self-checks:** every frame must fit its bit budget and decode bit-exactly.
- **Speed:** how long a full conversion would take.

`--png DIR` writes the source, the DSC result and a difference image
(amplified 8x) for the worst and the median frame, so you can look at them
yourself.

Other options:

- `-n N` tests N frames.
- `--csv frames.csv` saves per-frame results.
- `--bpp 9` (or any value from 6 to 12) tries another rate. 8, 9 and 10 are
  the rates the MiSTer decoder is tested at.

### 2. Compare rates (optional)

```
mmidsc compare "Time Gal (J).mmi" --bpp 8,9,10 --xor all -n 10 --png timegal_cmp
```

This encodes 10 frames spread across the side at each rate and prints each
rate's bandwidth and PSNR. In the folder it writes, for each frame:

- the decoded picture at each rate (`frame012345_8bpp.png`, ...);
- for each pair of rates, the XOR of the two decoded pictures
  (`frame012345_xor_8bpp_vs_10bpp.png`). Identical pixels are black, and
  brighter means more bits differ;
- a mask of the same pair (`frame012345_xormask_8bpp_vs_10bpp.png`). Each
  channel is fully on wherever it differs at all. The raw XOR of small
  differences is nearly black, so the mask shows where they are.

`--xor 8:10` picks one pair and can be repeated. `--xor all` gives every
pair of rates. Use `src` for the lossless original: `--xor src:10` shows
exactly what 10 bpp loses, and adds `frame012345_src.png`.

### 3. Convert

```
mmidsc convert "Time Gal (J).mmi" "Time Gal (J).dscmmi"
```

The converter uses every CPU core. Encoding takes about 145 ms per frame per
core, so a 61,000-frame side takes roughly 18 minutes on 8 cores.

**Several rates in one pass:** `--bpp 8,9,10` writes one file per rate, and
reads and decodes each source frame only once:

```
mmidsc convert "Time Gal (J).mmi" "Time Gal (J).dscmmi" --bpp 8,9,10 --xor all
  -> Time Gal (J).8bpp.dscmmi, Time Gal (J).9bpp.dscmmi, Time Gal (J).10bpp.dscmmi
```

Any combination works: `--bpp 8,10`, `--bpp 9`, and so on. A single rate
keeps the output name exactly as given. Encoding time grows with the number
of rates (about 145 ms per frame per core for each rate).

**XOR images after converting:** `--xor A:B` (repeatable, `src` = the
original, e.g. `--xor src:10`) or `--xor all`
decodes sample frames back out of the finished files and writes the same
XOR and mask PNGs as `compare`, plus a summary of how much the rates differ.

- `--xor-frames N` sets how many frames to sample (default 10).
- `--xor-dir DIR` sets the folder (default: the output name with `_xor`).

These images come from the actual output files, so they show exactly what
the core will display.

### 4. Check the result

```
mmidsc verify "Time Gal (J).dscmmi" --source "Time Gal (J).mmi"
mmidsc frame  "Time Gal (J).dscmmi" 12345 frame.png
mmidsc info   "Time Gal (J).dscmmi"
```

`verify` decodes every frame (or `-n N` frames spread across the side),
compares each one to the source, and checks the stored VBI codes.
`unzip -t` also works on `.dscmmi` files and checks every CRC.

## What is kept from each frame

ares (`MCD::LD::scanline`) reads columns 118–909 of each 910×525 capture. It
stores the two fields as rows 0–262 and 263–524. `mmidsc` keeps exactly those
columns and all 525 rows, VBI lines included. That gives a 792×525 picture,
and each field becomes its own DSC slice of 792×263 (the second slice gets
one padding row).

## .dscmmi format

A `.dscmmi` is a ZIP64 archive of stored entries, like an `.mmi`, with every
entry's data aligned to 4096 bytes.

- Each stored QON video entry `X.qon` is replaced by `X.dscv`.
- Every other entry is copied byte for byte, `MediaInfo.json` included.
- A `DSCMMI.json` entry is added. It maps each original video entry to its
  replacement and records the DSC settings:

```json
{
  "format": "DSCMMI",
  "version": 1,
  "video": [
    { "source_entry": "Disc1Side1/AnalogVideo.qon",
      "entry": "Disc1Side1/AnalogVideo.dscv",
      "bits_per_pixel": 8.0, "width": 792, "height": 525, "crop_x": 118,
      "frame_count": 61081, "frame_bytes": 416592, "frame_stride": 416768 } ]
}
```

A loader resolves streams through `MediaInfo.json` exactly as ares does. For
each video file it then looks up the replacement entry in `DSCMMI.json`.

### DSCV stream

All fields are little-endian, except the embedded DSC PPS, which uses the
standard's big-endian layout. `src/dscv.h` has the authoritative table.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | `"DSCV"` |
| 4 | 2 | version (1) |
| 6 | 2 | header size (512) |
| 8 | 4 | source width and height (910, 525) |
| 12 | 4 | crop x and y (118, 0) |
| 16 | 4 | kept width and height (792, 525) |
| 20 | 8 | field rows: first row and row count of each field (0/263, 263/262) |
| 28 | 4 | frame count, using the same numbering as the source QON |
| 32 | 4 | frame duration in µs |
| 36 | 4 | source QON flags |
| 40 | 4 | DSC bytes per frame |
| 44 | 4 | frame stride (512-aligned) |
| 48 | 8 | offset of frame 0 (4096-aligned) |
| 56 | 8 | offset of the VBI table |
| 64 | 4 | VBI entry size (16) |
| 68 | 4 | DSC slices per frame |
| 128 | 128 | DSC picture parameter set |

- **Frames:** frame *n* starts at `frames_offset + n × frame_stride`. A
  frame's slices are stored back to back, each `chunk_size × slice_height`
  bytes. Slice 0 is the first field and slice 1 the second.
- **VBI table:** four 32-bit codes per frame, for source rows 15, 16, 278 and
  279 (lines 16, 17, 279 and 280). They are decoded from the lossless source
  with the same algorithm as ares' `decodeBiphaseCodeFromScanline`, and 0
  means no code. A player can use this table instead of decoding codes from
  pixels.

## FPGA decoder

`rtl/dsc_decoder.sv` is a synthesizable DSC decoder for the MiSTer core. One
build plays 8, 9 and 10 bpp files: the rate is read from the PPS in each
file, so the core needs no per-rate variant. It matches `src/dsc.c` bit
for bit in a Verilator testbench that covers:

- synthetic pictures at 6–12 bpp;
- frames from converted 8, 9 and 10 bpp files.

It needs about 40 MHz for 29.97 frames/s. `rtl/README.md` has the
interface, the design and the test details.

## About the DSC implementation

`src/dsc.c` is a DSC encoder and decoder written from the text of the VESA
DSC 1.2a standard. Its scope is chosen to keep an FPGA decoder small:

- 8 bits per component, RGB input converted to YCoCg-R.
- Bitstream semantics of DSC 1.1 (`dsc_version_minor = 1`).
- Constant bit rate, one slice per line, and slice width a multiple of 3.
- **Block prediction is disabled** in the PPS. It is optional for DSC
  decoders, and dropping it removes the decoder's block-prediction search
  logic.
- Everything else is implemented as specified:
  - modified median-adaptive prediction and midpoint prediction;
  - indexed colour history;
  - DSU-VLC entropy coding;
  - flatness signalling;
  - rate control with the recommended 8 bpp parameters (Tables E-4/E-5);
  - substream multiplexing.

**Caveat:** the standard names VESA's C model as normative where the two
differ, and that C model wasn't available here. In a few places the text is
informal; the main one is rate-control pipeline timing. The choices made there
are listed at the top of `dsc.c`. So:

- The encoder and decoder in this repository match each other exactly (tested
  below).
- Bit-compatibility with third-party DSC hardware or IP has **not** been
  verified.
- For the MiSTer core that doesn't matter, because its decoder will be written
  to match `dsc.c`, which serves as the reference model.

## Testing done

- **Round trips** (`make test`): 270 encode/decode runs across 4 picture and slice sizes (792×526 with 263- and 526-row slices, 792×263 with 32-row slices, 300×100),
  4 bit rates (6, 8, 9 and 10 bpp), 9 content types and ICH on/off. The decoder
  reproduced the encoder's reconstruction bit for bit in every run, and every
  slice fit its budget. The content types include worst cases for rate
  control: white noise, a 1-pixel checkerboard, random vertical stripes, and
  flat content turning into noise at the bottom of the slice.
- **Stress** (`./dsctest --stress 400`): 400 random collages at the real
  792×525 geometry. There were no failures; the fullest slice used 99.986% of
  its budget.
- **End to end:** synthetic LaserActive-like discs with real biphase VBI
  codes went through analyze, convert, verify and frame extraction:
  - all frames decoded;
  - the VBI table matched the source;
  - `unzip -t` passed;
  - a source over 4 GB, with the video stored past the 4 GB mark, converted
    correctly into a valid ZIP64 output;
  - the static Linux build produced byte-identical output.
- **Multi-rate conversion:**
  - `--bpp 8,9,10` on a synthetic disc gave three files that each passed
    `verify` (all frames, VBI table) and `unzip -t`.
  - The 8 bpp file was byte-identical to a single-rate conversion.
  - The static Linux build produced identical streams.
- **FPGA decoder:** bit-exact against `dsc.c` on 177 frames (see
  `rtl/README.md`).
- **Fallback path:** if a frame ever overflowed its budget, `convert`
  re-encodes it with the other ICH setting, then with a light horizontal blur.
  This path was exercised by forcing failures. It never triggered on test
  content.
- **Not done:** the Windows build compiles but was not run here, and nothing
  has been run on a real LaserActive disc image yet.

## Building

```
make            # native
make test       # codec round trips
make windows    # needs an x86_64-w64-mingw32 toolchain (e.g. llvm-mingw)
```

Release binaries (a static musl Linux x86-64 build and an llvm-mingw Windows
x86-64 build) go in the repository's releases rather than in the tree.

## Licences

- `mmidsc` source: MIT.
- `src/qon.h` and `src/qoi2.h`: MIT, Copyright (c) 2025 Roger Sanders and
  (c) 2021 Dominic Szablewski, taken unmodified from ares.
- DSC itself: VESA distributes the standard free of charge, and patent
  licences are available from VESA member contributors under RAND terms.
  Check VESA's terms before distributing hardware or software that
  implements it.
