/* DSCV: the DSC-compressed analog video stream stored in a .dscmmi file in
 * place of an MMI's AnalogVideo.qon. All integers little-endian except the
 * embedded DSC PPS (big-endian, as defined by the DSC standard).
 *
 *   offset  size  field
 *   0       4     magic "DSCV"
 *   4       2     version (1)
 *   6       2     header_size (512)
 *   8       2     src_width        width of the original capture (910)
 *   10      2     src_height       height of the original capture (525)
 *   12      2     crop_x           first source column kept (118)
 *   14      2     crop_y           first source row kept (0)
 *   16      2     width            columns kept (= DSC pic_width)
 *   18      2     height           rows kept (= DSC pic_height)
 *   20      2     field0_row, 22 field0_rows   rows of the first field (0, 263)
 *   24      2     field1_row, 26 field1_rows   rows of the second field (263, 262)
 *   28      4     frame_count      same frame numbering as the source QON
 *   32      4     frame_duration_us
 *   36      4     source_qon_flags
 *   40      4     frame_bytes      DSC bytes per frame (all slices)
 *   44      4     frame_stride     bytes from one frame to the next (>= frame_bytes, 512-aligned)
 *   48      8     frames_offset    from the start of the stream, 4096-aligned
 *   56      8     vbi_offset       from the start of the stream (0 = none)
 *   64      4     vbi_entry_size   16: four 32-bit biphase codes per frame, for source rows
 *                                  15, 16, 278, 279 (lines 16, 17, 279, 280), decoded from the
 *                                  lossless source exactly as ares does; 0 = no code
 *   68      4     dsc_slices_per_frame
 *   72      56    reserved (0)
 *   128     128   DSC picture parameter set (DSC 1.2a Section 4.1)
 *   256     256   reserved (0)
 *
 * Frame n's DSC picture starts at frames_offset + n * frame_stride. Slices
 * are stored back to back, each chunk_size * slice_height bytes; with the
 * default settings slice 0 is the first field and slice 1 the second.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef DSCV_H
#define DSCV_H

#include <stdint.h>

#include "dsc.h"

#define DSCV_HEADER_SIZE 512
#define DSCV_VBI_ENTRY 16

typedef struct {
	int version;
	int src_width, src_height, crop_x, crop_y, width, height;
	int field0_row, field0_rows, field1_row, field1_rows;
	uint32_t frame_count, frame_duration_us, source_qon_flags;
	uint32_t frame_bytes, frame_stride;
	uint64_t frames_offset, vbi_offset;
	uint32_t vbi_entry_size;
	uint32_t slices_per_frame;
	uint8_t pps[DSC_PPS_SIZE];
} dscv_header;

void dscv_pack(const dscv_header *h, uint8_t out[DSCV_HEADER_SIZE]);
int dscv_unpack(dscv_header *h, const uint8_t in[DSCV_HEADER_SIZE]);

#endif
