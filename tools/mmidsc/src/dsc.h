/*
 * dsc - a VESA Display Stream Compression (DSC) encoder and decoder for
 * 8-bit RGB 4:4:4 pictures, written from the DSC 1.2a standard text
 * (bitstream compatible with DSC 1.1: dsc_version_minor = 1).
 *
 * Scope and choices (see README for why):
 *   - 8 bits per component, RGB input converted to YCoCg-R (convert_rgb = 1)
 *   - CBR mode, one slice per line (slice_width == pic_width), slice width
 *     a multiple of 3 (no partial groups), any slice height
 *   - block prediction disabled in the PPS (block_pred_enable = 0); it is
 *     optional for DSC decoders and keeps a hardware decoder smaller
 *   - MMAP, MPP, ICH, DSU-VLC, flatness signalling, rate control and
 *     substream multiplexing as specified
 *
 * This was written from the standard's text, not from VESA's C model, which
 * the standard names as normative where the two differ. Where the text is
 * informal (mainly rate-control pipeline timing), the choices made are
 * documented in dsc.c. Encoder and decoder here are self-consistent and
 * tested against each other; bit-exactness with third-party DSC decoders
 * has not been verified.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef DSC_H
#define DSC_H

#include <stddef.h>
#include <stdint.h>

#define DSC_PPS_SIZE 128

typedef struct {
	/* Syntax elements (names as in DSC 1.2a Table 4-1 / 4-2) */
	int dsc_version_major;
	int dsc_version_minor;
	int pps_identifier;
	int bits_per_component;
	int linebuf_depth;
	int block_pred_enable;
	int convert_rgb;
	int simple_422;
	int vbr_enable;
	int bits_per_pixel; /* units of 1/16 bpp */
	int pic_height, pic_width;
	int slice_height, slice_width;
	int chunk_size;
	int initial_xmit_delay;
	int initial_dec_delay;
	int initial_scale_value;
	int scale_increment_interval;
	int scale_decrement_interval;
	int first_line_bpg_offset;
	int nfl_bpg_offset;   /* 11 fractional bits */
	int slice_bpg_offset; /* 11 fractional bits */
	int initial_offset;
	int final_offset;
	int flatness_min_qp;
	int flatness_max_qp;
	int rc_model_size;
	int rc_edge_factor;
	int rc_quant_incr_limit0;
	int rc_quant_incr_limit1;
	int rc_tgt_offset_hi;
	int rc_tgt_offset_lo;
	int rc_buf_thresh[14];
	int range_min_qp[15];
	int range_max_qp[15];
	int range_bpg_offset[15];
	int native_420, native_422;
	int second_line_bpg_offset, nsl_bpg_offset, second_line_offset_adj;

	/* Derived values (Table E-1), filled by dsc_pps_derive() */
	int groups_per_line;
	int groups_total;
	int slice_bits;
	int num_extra_mux_bits;
	int mux_word_size;
	int rc_buffer_size; /* minRateBufferSize */
	int slices_per_picture;
} dsc_pps;

/* Build a PPS for an 8bpc RGB picture with the recommended rate-control
 * parameters (DSC 1.2a Table E-4/E-5, 8bpp column). bpp16 is the target rate
 * in 1/16 bpp (128 = 8 bpp). Returns 0, or -1 with a message in err. */
int dsc_pps_init(dsc_pps *p, int width, int height, int slice_height, int bpp16, char *err, size_t errlen);

/* Compute derived values and check constraints this implementation needs. */
int dsc_pps_derive(dsc_pps *p, char *err, size_t errlen);

/* 128-byte PPS serialisation (DSC 1.2a Section 4.1). */
void dsc_pps_pack(const dsc_pps *p, uint8_t out[DSC_PPS_SIZE]);
int dsc_pps_unpack(dsc_pps *p, const uint8_t in[DSC_PPS_SIZE], char *err, size_t errlen);

/* Compressed size of one picture in bytes: chunk_size * pic_height rounded up
 * to whole slices. */
size_t dsc_picture_bytes(const dsc_pps *p);

typedef struct {
	int ich_groups;     /* groups coded in ICH mode */
	int mpp_units;      /* units coded with midpoint prediction */
	int force_mpp;      /* groups forced to MPP by the buffer model */
	int flat_signals;   /* flatness signals sent */
	long slice_bits_used_max; /* largest number of bits used by a slice, before padding */
	int qp_hist[16];    /* masterQp histogram, in groups */
} dsc_stats;

typedef struct {
	int disable_ich; /* encoder never selects ICH mode (for experiments) */
} dsc_enc_opts;

/* Encode an 8-bit RGB picture (rgb, row stride in bytes) into out, which must
 * hold dsc_picture_bytes() bytes. If recon is non-NULL it receives the
 * encoder's reconstructed RGB picture (packed, width*3 per row), which a
 * decoder must reproduce exactly. stats may be NULL.
 * Returns 0 on success, -1 if a slice did not fit its bit budget (rate
 * control failure; should not happen). */
int dsc_encode_picture(const dsc_pps *p, const dsc_enc_opts *opts, const uint8_t *rgb, size_t stride,
                       uint8_t *out, uint8_t *recon, dsc_stats *stats);

/* Decode a picture produced with the same PPS into packed RGB (width*3 per
 * row). Returns 0, or -1 if the bitstream is malformed. */
int dsc_decode_picture(const dsc_pps *p, const uint8_t *in, size_t inlen, uint8_t *rgb);

#endif
