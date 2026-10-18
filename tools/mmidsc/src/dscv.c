/* SPDX-License-Identifier: MIT */
#include "dscv.h"

#include <string.h>

#include "util.h"

void dscv_pack(const dscv_header *h, uint8_t o[DSCV_HEADER_SIZE])
{
	memset(o, 0, DSCV_HEADER_SIZE);
	memcpy(o, "DSCV", 4);
	wr16le(o + 4, (uint16_t)h->version);
	wr16le(o + 6, DSCV_HEADER_SIZE);
	wr16le(o + 8, (uint16_t)h->src_width);
	wr16le(o + 10, (uint16_t)h->src_height);
	wr16le(o + 12, (uint16_t)h->crop_x);
	wr16le(o + 14, (uint16_t)h->crop_y);
	wr16le(o + 16, (uint16_t)h->width);
	wr16le(o + 18, (uint16_t)h->height);
	wr16le(o + 20, (uint16_t)h->field0_row);
	wr16le(o + 22, (uint16_t)h->field0_rows);
	wr16le(o + 24, (uint16_t)h->field1_row);
	wr16le(o + 26, (uint16_t)h->field1_rows);
	wr32le(o + 28, h->frame_count);
	wr32le(o + 32, h->frame_duration_us);
	wr32le(o + 36, h->source_qon_flags);
	wr32le(o + 40, h->frame_bytes);
	wr32le(o + 44, h->frame_stride);
	wr64le(o + 48, h->frames_offset);
	wr64le(o + 56, h->vbi_offset);
	wr32le(o + 64, h->vbi_entry_size);
	wr32le(o + 68, h->slices_per_frame);
	memcpy(o + 128, h->pps, DSC_PPS_SIZE);
}

int dscv_unpack(dscv_header *h, const uint8_t o[DSCV_HEADER_SIZE])
{
	memset(h, 0, sizeof *h);
	if (memcmp(o, "DSCV", 4) || rd16le(o + 6) != DSCV_HEADER_SIZE) return -1;
	h->version = rd16le(o + 4);
	if (h->version != 1) return -1;
	h->src_width = rd16le(o + 8);
	h->src_height = rd16le(o + 10);
	h->crop_x = rd16le(o + 12);
	h->crop_y = rd16le(o + 14);
	h->width = rd16le(o + 16);
	h->height = rd16le(o + 18);
	h->field0_row = rd16le(o + 20);
	h->field0_rows = rd16le(o + 22);
	h->field1_row = rd16le(o + 24);
	h->field1_rows = rd16le(o + 26);
	h->frame_count = rd32le(o + 28);
	h->frame_duration_us = rd32le(o + 32);
	h->source_qon_flags = rd32le(o + 36);
	h->frame_bytes = rd32le(o + 40);
	h->frame_stride = rd32le(o + 44);
	h->frames_offset = rd64le(o + 48);
	h->vbi_offset = rd64le(o + 56);
	h->vbi_entry_size = rd32le(o + 64);
	h->slices_per_frame = rd32le(o + 68);
	memcpy(h->pps, o + 128, DSC_PPS_SIZE);
	return 0;
}
