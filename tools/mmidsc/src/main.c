/*
 * mmidsc - convert LaserActive .mmi images (as used by ares) into .dscmmi
 * files whose analog video is VESA DSC compressed at a constant bit rate,
 * for playback hardware with limited storage bandwidth (MiSTer).
 *
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <windows.h>
#define mkdir_p(p) _mkdir(p)
#else
#include <unistd.h>
#define mkdir_p(p) mkdir(p, 0755)
#endif

#include "dsc.h"
#include "dscv.h"
#include "mmi.h"
#include "util.h"
#include "zipw.h"

#define TOOL_VERSION "1.1"

static int g_threads = 0;

static int cpu_count(void)
{
#ifdef _WIN32
	SYSTEM_INFO si;
	GetSystemInfo(&si);
	return (int)si.dwNumberOfProcessors;
#else
	long n = sysconf(_SC_NPROCESSORS_ONLN);
	return n > 0 ? (int)n : 1;
#endif
}

static void die(const char *fmt, const char *arg)
{
	fprintf(stderr, "error: ");
	fprintf(stderr, fmt, arg ? arg : "");
	fprintf(stderr, "\n");
	exit(1);
}

/* ------------------------------------------------------------ geometry */

typedef struct {
	int src_w, src_h;
	int crop_x, width, height, slice_h;
	int f0, f0n, f1, f1n;
	int bpp16;
} geom;

static void pick_geometry(int src_w, int src_h, int bpp16, geom *g)
{
	memset(g, 0, sizeof *g);
	g->src_w = src_w;
	g->src_h = src_h;
	g->bpp16 = bpp16;
	if (src_w == 910 && src_h == 525) {
		/* ares (MCD::LD::scanline) reads columns 118..909 and stores the two
		 * fields as rows 0..262 and 263..524 of each frame. */
		g->crop_x = 118;
		g->width = 792;
		g->height = 525;
		g->slice_h = 263;
		g->f0 = 0; g->f0n = 263;
		g->f1 = 263; g->f1n = 262;
	} else {
		g->crop_x = 0;
		g->width = src_w - src_w % 3;
		g->height = src_h;
		g->slice_h = (src_h + 1) / 2;
		g->f0 = 0; g->f0n = g->slice_h;
		g->f1 = g->slice_h; g->f1n = src_h - g->slice_h;
	}
	while (g->width > 9 && (g->width * bpp16) % 128) g->width -= 3;
}

static void make_pps(const geom *g, dsc_pps *p)
{
	char err[256];
	if (dsc_pps_init(p, g->width, g->height, g->slice_h, g->bpp16, err, sizeof err)) die("DSC setup: %s", err);
}

/* -------------------------------------------------------- video entries */

typedef struct {
	zip_file zip;
	int qon_index[64];
	int qon_count;
} mmi_file;

static void open_mmi(mmi_file *m, const char *path)
{
	char err[256];
	memset(m, 0, sizeof *m);
	if (zip_open(&m->zip, path, err, sizeof err)) die("%s", err);
	for (int i = 0; i < m->zip.count && m->qon_count < 64; i++)
		if (zip_entry_is_qon(&m->zip, &m->zip.entries[i])) m->qon_index[m->qon_count++] = i;
}

static const zip_entry *pick_qon(mmi_file *m, const char *want)
{
	if (!m->qon_count) die("no stored QON video stream found%s", "");
	if (!want) return &m->zip.entries[m->qon_index[0]];
	for (int k = 0; k < m->qon_count; k++)
		if (!strcmp(m->zip.entries[m->qon_index[k]].name, want)) return &m->zip.entries[m->qon_index[k]];
	die("no QON video entry named %s", want);
	return NULL;
}

static void open_qon(qon_stream *q, mmi_file *m, const zip_entry *e)
{
	char err[256];
	if (qon_open(q, m->zip.fd, e->data_off, e->comp_size, err, sizeof err)) die("%s", err);
	if (q->hdr.channels != 3 && q->hdr.channels != 4) die("unexpected channel count%s", "");
}

static double fps_of(const qon_stream *q)
{
	return q->hdr.frame_duration_in_microseconds ? 1e6 / q->hdr.frame_duration_in_microseconds : 30000.0 / 1001.0;
}

/* --------------------------------------------------------------- info */

static void cmd_info(const char *path)
{
	mmi_file m;
	open_mmi(&m, path);
	printf("%s: %d ZIP entries\n", path, m.zip.count);
	for (int i = 0; i < m.zip.count; i++) {
		const zip_entry *e = &m.zip.entries[i];
		printf("  %-44s %11.1f MB  %s\n", e->name, e->comp_size / 1e6, e->method == 0 ? "stored" : "compressed");
	}
	for (int k = 0; k < m.qon_count; k++) {
		const zip_entry *e = &m.zip.entries[m.qon_index[k]];
		qon_stream q;
		open_qon(&q, &m, e);
		double fps = fps_of(&q), total = 0, worst = 0, acc = 0;
		uint32_t fc = q.hdr.frame_count, win = (uint32_t)(fps + 0.5);
		for (uint32_t i = 0; i < fc; i++) {
			double s = (double)qon_frame_span(&q, i);
			total += s;
			acc += s;
			if (i >= win) acc -= (double)qon_frame_span(&q, i - win);
			if (acc > worst) worst = acc;
		}
		printf("\nQON video %s\n  %ux%u, %u frames, %.3f fps\n", e->name, q.hdr.width, q.hdr.height, fc, fps);
		printf("  compressed: avg %.0f KB/frame = %.1f MB/s, worst 1 s %.1f MB/s\n", total / fc / 1024, total / fc * fps / 1e6,
		       worst / 1e6);
		qon_close(&q);
	}
	for (int i = 0; i < m.zip.count; i++) {
		const zip_entry *e = &m.zip.entries[i];
		uint8_t hb[DSCV_HEADER_SIZE];
		dscv_header h;
		if (e->method != 0 || e->comp_size < DSCV_HEADER_SIZE || read_full_at(m.zip.fd, hb, sizeof hb, e->data_off) ||
		    dscv_unpack(&h, hb))
			continue;
		dsc_pps p;
		char err[256];
		int ok = !dsc_pps_unpack(&p, h.pps, err, sizeof err);
		double fps = h.frame_duration_us ? 1e6 / h.frame_duration_us : 30000.0 / 1001.0;
		printf("\nDSCV video %s\n", e->name);
		printf("  source %dx%d, kept %dx%d from column %d row %d; fields rows %d+%d and %d+%d\n", h.src_width, h.src_height,
		       h.width, h.height, h.crop_x, h.crop_y, h.field0_row, h.field0_rows, h.field1_row, h.field1_rows);
		printf("  %u frames, %u bytes/frame (stride %u) = %.2f MB/s constant, data at +%llu, VBI table %s\n", h.frame_count,
		       h.frame_bytes, h.frame_stride, h.frame_stride * fps / 1e6, (unsigned long long)h.frames_offset,
		       h.vbi_offset ? "present" : "absent");
		if (ok)
			printf("  DSC %d.%d, %.2f bpp, %d slices of %dx%d, chunk %d bytes, block prediction %s\n", p.dsc_version_major,
			       p.dsc_version_minor, p.bits_per_pixel / 16.0, p.slices_per_picture, p.slice_width, p.slice_height,
			       p.chunk_size, p.block_pred_enable ? "on" : "off");
		else
			printf("  PPS not decodable by this tool: %s\n", err);
	}
	zip_close(&m.zip);
}

/* ------------------------------------------------------------ analyze */

typedef struct {
	uint32_t frame;
	double psnr;
	int max_err;
	uint32_t qoi_bytes;
	double enc_ms;
	int bitexact, enc_ok;
	int vbi_src, vbi_same;
	dsc_stats st;
} frame_result;

typedef struct {
	mmi_file *m;
	qon_stream *q;
	const geom *g;
	const dsc_pps *p;
	dsc_enc_opts opts;
	frame_result *res;
	int count;
	int next;
	pthread_mutex_t lock;
	int lossless_compare;
} analyze_job;

static const int VBI_ROWS[4] = { 15, 16, 278, 279 };

static double rgb_psnr(const uint8_t *a, size_t astride, const uint8_t *b, size_t bstride, int w, int h, int *max_err)
{
	double se = 0;
	int me = 0;
	for (int y = 0; y < h; y++) {
		const uint8_t *pa = a + (size_t)y * astride, *pb = b + (size_t)y * bstride;
		for (int i = 0; i < w * 3; i++) {
			int d = (int)pa[i] - (int)pb[i];
			se += (double)d * d;
			if (d < 0) d = -d;
			if (d > me) me = d;
		}
	}
	if (max_err) *max_err = me;
	if (se == 0) return 99.0;
	return 10 * log10(255.0 * 255.0 / (se / ((double)w * h * 3)));
}

static void *analyze_worker(void *arg)
{
	analyze_job *J = arg;
	const geom *g = J->g;
	size_t src_bytes = (size_t)g->src_w * g->src_h * 3;
	size_t pic_bytes = (size_t)g->width * g->height * 3;
	uint8_t *src = malloc(src_bytes), *rec = malloc(pic_bytes), *dec = malloc(pic_bytes);
	size_t out_bytes = dsc_picture_bytes(J->p);
	uint8_t *out = malloc(out_bytes);
	qoi2_desc qd = { (unsigned)g->width, (unsigned)g->height, 3, 0 };
	size_t qmax = qoi2_max_encoded_data_size_without_header(&qd);
	uint8_t *qbuf = J->lossless_compare ? malloc(qmax) : NULL;
	uint8_t *crop = J->lossless_compare ? malloc(pic_bytes) : NULL;
	uint8_t *scratch = NULL;
	size_t cap = 0;
	for (;;) {
		pthread_mutex_lock(&J->lock);
		int k = J->next++;
		pthread_mutex_unlock(&J->lock);
		if (k >= J->count) break;
		frame_result *r = &J->res[k];
		if (qon_read_frame(J->q, r->frame, src, &scratch, &cap)) {
			r->enc_ok = 0;
			continue;
		}
		const uint8_t *pic = src + (size_t)g->crop_x * 3;
		size_t stride = (size_t)g->src_w * 3;
		memset(&r->st, 0, sizeof r->st);
		double t0 = now_seconds();
		r->enc_ok = dsc_encode_picture(J->p, &J->opts, pic, stride, out, rec, &r->st) == 0;
		r->enc_ms = (now_seconds() - t0) * 1e3;
		r->bitexact = dsc_decode_picture(J->p, out, out_bytes, dec) == 0 && !memcmp(dec, rec, pic_bytes);
		r->psnr = rgb_psnr(pic, stride, dec, (size_t)g->width * 3, g->width, g->height, &r->max_err);
		r->vbi_src = r->vbi_same = 0;
		for (int i = 0; i < 4; i++) {
			uint32_t a = vbi_decode_line(src, g->src_w, g->src_w, 0, VBI_ROWS[i]);
			if (!a) continue;
			r->vbi_src++;
			uint32_t b = vbi_decode_line(dec, g->width, g->src_w, g->crop_x, VBI_ROWS[i]);
			if (a == b) r->vbi_same++;
		}
		if (J->lossless_compare) {
			for (int y = 0; y < g->height; y++) memcpy(crop + (size_t)y * g->width * 3, pic + (size_t)y * stride, (size_t)g->width * 3);
			size_t len = qmax;
			r->qoi_bytes = qoi2_encode_data(crop, 3, &qd, NULL, qbuf, &len) ? (uint32_t)len + 4 : 0;
		}
	}
	free(src); free(rec); free(dec); free(out); free(qbuf); free(crop); free(scratch);
	return NULL;
}

static int cmp_psnr(const void *a, const void *b)
{
	double x = ((const frame_result *)a)->psnr, y = ((const frame_result *)b)->psnr;
	return (x > y) - (x < y);
}

static void write_frame_pngs(const char *dir, const char *tag, mmi_file *m, qon_stream *q, const geom *g, const dsc_pps *p,
                             const dsc_enc_opts *o, uint32_t frame)
{
	(void)m;
	size_t src_bytes = (size_t)g->src_w * g->src_h * 3, pic_bytes = (size_t)g->width * g->height * 3;
	uint8_t *src = malloc(src_bytes), *dec = malloc(pic_bytes), *diff = malloc(pic_bytes), *out = malloc(dsc_picture_bytes(p));
	uint8_t *scratch = NULL;
	size_t cap = 0;
	if (qon_read_frame(q, frame, src, &scratch, &cap)) goto done;
	const uint8_t *pic = src + (size_t)g->crop_x * 3;
	size_t stride = (size_t)g->src_w * 3;
	dsc_encode_picture(p, o, pic, stride, out, NULL, NULL);
	dsc_decode_picture(p, out, dsc_picture_bytes(p), dec);
	for (int y = 0; y < g->height; y++)
		for (int i = 0; i < g->width * 3; i++) {
			int d = abs((int)pic[(size_t)y * stride + i] - (int)dec[(size_t)y * g->width * 3 + i]) * 8;
			diff[(size_t)y * g->width * 3 + i] = (uint8_t)(d > 255 ? 255 : d);
		}
	char path[1024];
	snprintf(path, sizeof path, "%s/%s_frame%06u_source.png", dir, tag, frame);
	write_png_rgb(path, pic, g->width, g->height, stride);
	snprintf(path, sizeof path, "%s/%s_frame%06u_dsc.png", dir, tag, frame);
	write_png_rgb(path, dec, g->width, g->height, (size_t)g->width * 3);
	snprintf(path, sizeof path, "%s/%s_frame%06u_diff_x8.png", dir, tag, frame);
	write_png_rgb(path, diff, g->width, g->height, (size_t)g->width * 3);
	printf("  wrote %s/%s_frame%06u_{source,dsc,diff_x8}.png\n", dir, tag, frame);
done:
	free(src); free(dec); free(diff); free(out); free(scratch);
}

static void cmd_analyze(const char *path, int n, long start, int step, int bpp16, int no_ich, const char *png_dir,
                        const char *csv, const char *entry)
{
	mmi_file m;
	open_mmi(&m, path);
	const zip_entry *e = pick_qon(&m, entry);
	qon_stream q;
	open_qon(&q, &m, e);
	geom g;
	pick_geometry((int)q.hdr.width, (int)q.hdr.height, bpp16, &g);
	dsc_pps p;
	make_pps(&g, &p);
	uint32_t fc = q.hdr.frame_count;
	double fps = fps_of(&q);
	if (n > (int)fc) n = (int)fc;

	frame_result *res = calloc((size_t)n, sizeof *res);
	for (int k = 0; k < n; k++) {
		long f;
		if (step > 0) f = (start < 0 ? 0 : start) + (long)k * step;
		else if (start >= 0) f = start + k;
		else f = (long)(((double)k + 0.5) * fc / n);
		if (f >= (long)fc) f = (long)fc - 1;
		res[k].frame = (uint32_t)f;
	}

	size_t frame_bytes = dsc_picture_bytes(&p);
	uint32_t stride = (uint32_t)((frame_bytes + 511) & ~(size_t)511);
	printf("video entry %s: %ux%u, %u frames, %.3f fps\n", e->name, q.hdr.width, q.hdr.height, fc, fps);
	printf("kept picture: columns %d..%d (%d wide), %d rows; DSC slices: %d x %d rows (one per field)\n", g.crop_x,
	       g.crop_x + g.width - 1, g.width, g.height, p.slices_per_picture, p.slice_height);
	printf("DSC: %.2f bpp, %zu bytes/frame (%u with 512-byte alignment) = %.2f MB/s, constant\n", bpp16 / 16.0, frame_bytes,
	       stride, stride * fps / 1e6);
	{
		double total = 0, worst = 0, acc = 0;
		uint32_t win = (uint32_t)(fps + 0.5);
		for (uint32_t i = 0; i < fc; i++) {
			double s = (double)qon_frame_span(&q, i);
			total += s;
			acc += s;
			if (i >= win) acc -= (double)qon_frame_span(&q, i - win);
			if (acc > worst) worst = acc;
		}
		printf("source QON (all frames, from the index): avg %.1f MB/s, worst 1 s %.1f MB/s\n", total / fc * fps / 1e6, worst / 1e6);
	}

	int threads = g_threads > 0 ? g_threads : cpu_count();
	analyze_job J;
	memset(&J, 0, sizeof J);
	J.m = &m; J.q = &q; J.g = &g; J.p = &p; J.res = res; J.count = n; J.lossless_compare = 1;
	J.opts.disable_ich = no_ich;
	pthread_mutex_init(&J.lock, NULL);
	printf("\nencoding %d frames on %d threads...\n", n, threads);
	double t0 = now_seconds();
	pthread_t *th = malloc(sizeof(pthread_t) * (size_t)threads);
	for (int t = 0; t < threads; t++) pthread_create(&th[t], NULL, analyze_worker, &J);
	for (int t = 0; t < threads; t++) pthread_join(th[t], NULL);
	double dt = now_seconds() - t0;
	free(th);

	int bad_enc = 0, bad_exact = 0, vbi_src = 0, vbi_same = 0;
	double qsum = 0, qmax = 0, psum = 0, enc_ms = 0;
	dsc_stats tot;
	memset(&tot, 0, sizeof tot);
	for (int k = 0; k < n; k++) {
		frame_result *r = &res[k];
		bad_enc += !r->enc_ok;
		bad_exact += !r->bitexact;
		vbi_src += r->vbi_src;
		vbi_same += r->vbi_same;
		qsum += r->qoi_bytes;
		if (r->qoi_bytes > qmax) qmax = r->qoi_bytes;
		psum += r->psnr;
		enc_ms += r->enc_ms;
		tot.ich_groups += r->st.ich_groups;
		tot.mpp_units += r->st.mpp_units;
		tot.force_mpp += r->st.force_mpp;
		for (int i = 0; i < 16; i++) tot.qp_hist[i] += r->st.qp_hist[i];
	}
	if (csv) {
		FILE *f = fopen(csv, "w");
		if (f) {
			fprintf(f, "frame,psnr_db,max_abs_err,qoi2_cropped_bytes,encode_ms,bitexact,fits,vbi_codes,vbi_survived\n");
			for (int k = 0; k < n; k++)
				fprintf(f, "%u,%.3f,%d,%u,%.2f,%d,%d,%d,%d\n", res[k].frame, res[k].psnr, res[k].max_err, res[k].qoi_bytes,
				        res[k].enc_ms, res[k].bitexact, res[k].enc_ok, res[k].vbi_src, res[k].vbi_same);
			fclose(f);
			printf("per-frame results written to %s\n", csv);
		}
	}
	frame_result *sorted = malloc(sizeof *sorted * (size_t)n);
	memcpy(sorted, res, sizeof *sorted * (size_t)n);
	qsort(sorted, (size_t)n, sizeof *sorted, cmp_psnr);

	printf("\nquality (RGB PSNR over the kept picture, %d frames):\n", n);
	printf("  mean %.2f dB, median %.2f dB, 5th percentile %.2f dB, worst %.2f dB (frame %u)\n", psum / n,
	       sorted[n / 2].psnr, sorted[(n * 5) / 100].psnr, sorted[0].psnr, sorted[0].frame);
	long groups = 0;
	for (int i = 0; i < 16; i++) groups += tot.qp_hist[i];
	printf("  QP use:");
	for (int i = 0; i < 16; i++)
		if (tot.qp_hist[i]) printf(" %d:%.1f%%", i, 100.0 * tot.qp_hist[i] / groups);
	printf("\n  ICH groups %.2f%%, forced MPP groups %.2f%%\n", 100.0 * tot.ich_groups / groups, 100.0 * tot.force_mpp / groups);
	printf("\nbandwidth at %.3f fps:\n", fps);
	printf("  DSC %.2f bpp:                    %.2f MB/s every second (%.0f KB/frame)\n", bpp16 / 16.0, stride * fps / 1e6,
	       stride / 1024.0);
	printf("  lossless QOI2, same kept picture: avg %.2f MB/s, largest sampled frame %.0f KB (= %.2f MB/s if sustained)\n",
	       qsum / n * fps / 1e6, qmax / 1024, qmax * fps / 1e6);
	if (vbi_src)
		printf("\nVBI biphase codes: %d of %d found in the source decode identically from the DSC frame\n", vbi_same, vbi_src);
	else
		printf("\nVBI biphase codes: none found in the sampled source frames\n");
	printf("  (.dscmmi files carry the codes decoded from the lossless source in a side table either way)\n");
	printf("\nchecks: %d of %d frames fit the slice budget, %d of %d decoded bit-exactly to the encoder's reconstruction\n",
	       n - bad_enc, n, n - bad_exact, n);
	double per_frame = dt / n;
	printf("speed: %.1f frames/s on %d threads (%.1f ms per frame per thread); converting all %u frames would take about %.0f min\n",
	       n / dt, threads, enc_ms / n, fc, fc * per_frame / 60.0);

	if (png_dir) {
		mkdir_p(png_dir);
		printf("\n");
		write_frame_pngs(png_dir, "worst", &m, &q, &g, &p, &J.opts, sorted[0].frame);
		write_frame_pngs(png_dir, "median", &m, &q, &g, &p, &J.opts, sorted[n / 2].frame);
	}
	free(sorted);
	free(res);
	qon_close(&q);
	zip_close(&m.zip);
	if (bad_enc || bad_exact) exit(2);
}

/* ------------------------------------------------------------ convert */

#define MAX_RATES 6

typedef struct {
	int bpp16;
	geom g;
	dsc_pps p;
	size_t frame_bytes;
	uint32_t stride;
	uint64_t frames_off, total;
	zip_writer w;
	char path[1100];
	char entry[600];
	uint64_t at;
	uint32_t *crc;
	int failures, fallbacks;
	uint32_t first_failure;
	char json[16384];
	size_t jl;
	int nvid;
} rate_out;

typedef struct {
	qon_stream *q;
	rate_out *r;
	int nr;
	dsc_enc_opts opts;
	uint8_t *vbi;
	uint32_t count;
	uint32_t next;
	uint32_t done;
	int read_failures;
	pthread_mutex_t lock;
} convert_job;

/* Encode one cropped picture at one rate, with the overflow fallbacks. */
static int encode_with_fallback(const dsc_pps *p, const geom *g, const dsc_enc_opts *opts, const uint8_t *pic, size_t stride,
                                uint8_t *out, int *fallback)
{
	*fallback = 0;
	if (!dsc_encode_picture(p, opts, pic, stride, out, NULL, NULL)) return 0;
	/* Rate control overflowed the slice budget (not seen in testing). Retry
	 * with the other ICH setting, then with a light horizontal blur; both
	 * change the encoder's choices and the stream stays valid DSC. */
	dsc_enc_opts o2 = *opts;
	o2.disable_ich = !o2.disable_ich;
	*fallback = 1;
	if (!dsc_encode_picture(p, &o2, pic, stride, out, NULL, NULL)) return 0;
	size_t row = (size_t)g->width * 3;
	uint8_t *soft = malloc(row * g->height);
	for (int y = 0; y < g->height; y++)
		for (int x = 0; x < g->width; x++)
			for (int c = 0; c < 3; c++) {
				int l = pic[y * stride + (size_t)(x > 0 ? x - 1 : x) * 3 + c];
				int m = pic[y * stride + (size_t)x * 3 + c];
				int r = pic[y * stride + (size_t)(x < g->width - 1 ? x + 1 : x) * 3 + c];
				soft[y * row + (size_t)x * 3 + c] = (uint8_t)((l + 2 * m + r + 2) / 4);
			}
	int ret = dsc_encode_picture(p, opts, soft, row, out, NULL, NULL);
	free(soft);
	return ret;
}

static void *convert_worker(void *arg)
{
	convert_job *J = arg;
	const geom *g0 = &J->r[0].g;
	size_t src_bytes = (size_t)g0->src_w * g0->src_h * 3;
	uint8_t *src = malloc(src_bytes);
	uint32_t max_stride = 0;
	for (int k = 0; k < J->nr; k++) if (J->r[k].stride > max_stride) max_stride = J->r[k].stride;
	uint8_t *out = calloc(1, max_stride);
	uint8_t *scratch = NULL;
	size_t cap = 0;
	for (;;) {
		pthread_mutex_lock(&J->lock);
		uint32_t i = J->next++;
		pthread_mutex_unlock(&J->lock);
		if (i >= J->count) break;
		int read_fail = 0;
		if (qon_read_frame(J->q, i, src, &scratch, &cap)) {
			read_fail = 1;
			memset(src, 0, src_bytes);
		}
		for (int k = 0; k < 4; k++) wr32le(J->vbi + (size_t)i * 16 + k * 4, vbi_decode_line(src, g0->src_w, g0->src_w, 0, VBI_ROWS[k]));
		for (int k = 0; k < J->nr; k++) {
			rate_out *R = &J->r[k];
			const uint8_t *pic = src + (size_t)R->g.crop_x * 3;
			int fallback;
			int fail = encode_with_fallback(&R->p, &R->g, &J->opts, pic, (size_t)R->g.src_w * 3, out, &fallback) || read_fail;
			memset(out + R->frame_bytes, 0, R->stride - R->frame_bytes);
			R->crc[i] = crc32_update(0, out, R->stride);
			if (write_full_at(R->w.fd, out, R->stride, R->at + R->frames_off + (uint64_t)i * R->stride)) {
				fprintf(stderr, "\nerror: write failed: %s\n", strerror(errno));
				exit(1);
			}
			pthread_mutex_lock(&J->lock);
			if (fail) {
				if (!R->failures || i < R->first_failure) R->first_failure = i;
				R->failures++;
			} else if (fallback) {
				R->fallbacks++;
			}
			pthread_mutex_unlock(&J->lock);
		}
		pthread_mutex_lock(&J->lock);
		J->done++;
		J->read_failures += read_fail;
		pthread_mutex_unlock(&J->lock);
	}
	free(src); free(out); free(scratch);
	return NULL;
}

static void copy_entry(zip_writer *w, const zip_file *z, const zip_entry *e)
{
	uint64_t at = zw_begin(w, e->name, e->method, e->flags, e->mtime, e->mdate, e->comp_size, e->uncomp_size, 4096);
	size_t chunk = 8u << 20;
	uint8_t *buf = malloc(chunk);
	for (uint64_t done = 0; done < e->comp_size;) {
		size_t n = e->comp_size - done < chunk ? (size_t)(e->comp_size - done) : chunk;
		if (read_full_at(z->fd, buf, n, e->data_off + done)) die("read failed while copying %s", e->name);
		if (write_full_at(w->fd, buf, n, at + done)) die("write failed while copying %s", e->name);
		done += n;
	}
	free(buf);
	zw_end(w, e->crc);
}

static const char *base_name(const char *p)
{
	const char *s = strrchr(p, '/');
	const char *b = strrchr(p, '\\');
	if (b && (!s || b > s)) s = b;
	return s ? s + 1 : p;
}

static void rate_label(int bpp16, char *buf, size_t len)
{
	if (bpp16 % 16 == 0) snprintf(buf, len, "%dbpp", bpp16 / 16);
	else snprintf(buf, len, "%.4gbpp", bpp16 / 16.0);
}

/* "Game.dscmmi" + 9 bpp -> "Game.9bpp.dscmmi" */
static void rate_path(const char *outp, int bpp16, char *buf, size_t len)
{
	char lab[32];
	rate_label(bpp16, lab, sizeof lab);
	const char *dot = strrchr(outp, '.');
	const char *base = base_name(outp);
	if (dot && dot > base) snprintf(buf, len, "%.*s.%s%s", (int)(dot - outp), outp, lab, dot);
	else snprintf(buf, len, "%s.%s.dscmmi", outp, lab);
}

/* ------------------------------------------------------- XOR images */

typedef struct {
	int a, b; /* indices into the rate list */
} rate_pair;

typedef struct {
	long long bytes, differing;
	double abs_sum;
	int frames;
} xor_stats;

static void write_xor(const char *dir, const char *label, uint32_t frame, const uint8_t *A, const uint8_t *B, int w, int h,
                      xor_stats *st)
{
	size_t n = (size_t)w * h * 3;
	uint8_t *x = malloc(n);
	for (size_t i = 0; i < n; i++) {
		x[i] = A[i] ^ B[i];
		st->differing += x[i] != 0;
		st->abs_sum += abs((int)A[i] - (int)B[i]);
	}
	st->bytes += (long long)n;
	st->frames++;
	char path[1300];
	snprintf(path, sizeof path, "%s/frame%06u_xor_%s.png", dir, frame, label);
	write_png_rgb(path, x, w, h, (size_t)w * 3);
	/* The raw XOR is mostly near-black (differences of 1-3 levels), so also
	 * write a mask: a channel is full on wherever its XOR is non-zero. */
	for (size_t i = 0; i < n; i++) x[i] = x[i] ? 255 : 0;
	snprintf(path, sizeof path, "%s/frame%06u_xormask_%s.png", dir, frame, label);
	write_png_rgb(path, x, w, h, (size_t)w * 3);
	free(x);
}

static void report_xor(const char *label, const xor_stats *st)
{
	if (!st->frames) return;
	printf("  %-16s %d frames: %.2f%% of bytes differ, mean |difference| %.3f\n", label, st->frames,
	       100.0 * st->differing / st->bytes, st->abs_sum / st->bytes);
}

/* rate index used for the lossless source picture in --xor src:N */
#define SRC_IDX MAX_RATES

static void pair_label(const int *bpp16, rate_pair pr, char *buf, size_t len)
{
	char a[32], b[32];
	if (pr.a == SRC_IDX) snprintf(a, sizeof a, "src");
	else rate_label(bpp16[pr.a], a, sizeof a);
	if (pr.b == SRC_IDX) snprintf(b, sizeof b, "src");
	else rate_label(bpp16[pr.b], b, sizeof b);
	snprintf(buf, len, "%s_vs_%s", a, b);
}

/* Decode sampled frames from finished .dscmmi outputs and write XOR images. */
static void crop_source(const qon_stream *q, uint32_t frame, const geom *g, uint8_t *src, uint8_t *out, uint8_t **scratch,
                        size_t *cap)
{
	if (qon_read_frame(q, frame, src, scratch, cap)) die("could not read source frame%s", "");
	for (int y = 0; y < g->height; y++)
		memcpy(out + (size_t)y * g->width * 3, src + ((size_t)y * g->src_w + g->crop_x) * 3, (size_t)g->width * 3);
}

static void xor_from_outputs(rate_out *r, int nr, const char *entry, uint32_t fc, const rate_pair *pairs, int np, int nframes,
                             const char *dir, const qon_stream *q)
{
	if (!np || nframes <= 0) return;
	mkdir_p(dir);
	int bpps[MAX_RATES];
	for (int k = 0; k < nr; k++) bpps[k] = r[k].bpp16;
	zip_file z[MAX_RATES];
	const zip_entry *e[MAX_RATES];
	uint8_t *dec[MAX_RATES + 1], *in = NULL;
	uint8_t *srcbuf = malloc((size_t)r[0].g.src_w * r[0].g.src_h * 3), *scratch = NULL;
	size_t cap = 0;
	dec[SRC_IDX] = malloc((size_t)r[0].g.width * r[0].g.height * 3);
	char err[256];
	size_t pic = 0;
	for (int k = 0; k < nr; k++) {
		if (zip_open(&z[k], r[k].path, err, sizeof err)) die("%s", err);
		e[k] = zip_find(&z[k], entry);
		if (!e[k]) die("missing %s in output", entry);
		pic = (size_t)r[k].g.width * r[k].g.height * 3;
		dec[k] = malloc(pic);
	}
	uint32_t max_bytes = 0;
	for (int k = 0; k < nr; k++) if (r[k].frame_bytes > max_bytes) max_bytes = (uint32_t)r[k].frame_bytes;
	in = malloc(max_bytes);
	xor_stats st[16];
	memset(st, 0, sizeof st);
	if (nframes > (int)fc) nframes = (int)fc;
	printf("XOR images (%d frames) in %s:\n", nframes, dir);
	for (int f = 0; f < nframes; f++) {
		uint32_t frame = (uint32_t)(((double)f + 0.5) * fc / nframes);
		int need[MAX_RATES + 1] = { 0 };
		for (int p = 0; p < np; p++) need[pairs[p].a] = need[pairs[p].b] = 1;
		if (need[SRC_IDX]) crop_source(q, frame, &r[0].g, srcbuf, dec[SRC_IDX], &scratch, &cap);
		for (int k = 0; k < nr; k++) {
			if (!need[k]) continue;
			uint64_t off = e[k]->data_off + r[k].frames_off + (uint64_t)frame * r[k].stride;
			if (read_full_at(z[k].fd, in, r[k].frame_bytes, off) || dsc_decode_picture(&r[k].p, in, r[k].frame_bytes, dec[k]))
				die("could not decode a frame back from %s", r[k].path);
		}
		for (int p = 0; p < np; p++) {
			char lab[64];
			pair_label(bpps, pairs[p], lab, sizeof lab);
			write_xor(dir, lab, frame, dec[pairs[p].a], dec[pairs[p].b], r[0].g.width, r[0].g.height, &st[p]);
		}
	}
	for (int p = 0; p < np; p++) {
		char lab[64];
		pair_label(bpps, pairs[p], lab, sizeof lab);
		report_xor(lab, &st[p]);
	}
	for (int k = 0; k < nr; k++) { zip_close(&z[k]); free(dec[k]); }
	free(dec[SRC_IDX]); free(srcbuf); free(scratch);
	free(in);
}

static void cmd_convert(const char *in, const char *outp, const int *bpps, int nr, int no_ich, const rate_pair *pairs, int np,
                        int xor_frames, const char *xor_dir)
{
	mmi_file m;
	open_mmi(&m, in);
	if (!m.qon_count) die("no stored QON video stream found in %s", in);
	int threads = g_threads > 0 ? g_threads : cpu_count();
	rate_out *r = calloc((size_t)nr, sizeof *r);
	for (int k = 0; k < nr; k++) {
		r[k].bpp16 = bpps[k];
		if (nr == 1) snprintf(r[k].path, sizeof r[k].path, "%s", outp);
		else rate_path(outp, bpps[k], r[k].path, sizeof r[k].path);
		if (zw_open(&r[k].w, r[k].path)) die("cannot create %s", r[k].path);
		r[k].jl = (size_t)snprintf(r[k].json, sizeof r[k].json,
		                           "{\n  \"format\": \"DSCMMI\",\n  \"version\": 1,\n  \"tool\": \"mmidsc " TOOL_VERSION "\",\n"
		                           "  \"source\": \"%s\",\n  \"video\": [", base_name(in));
		printf("output %s (%.4g bpp)\n", r[k].path, bpps[k] / 16.0);
	}
	char xdir[1100];
	if (xor_dir) snprintf(xdir, sizeof xdir, "%s", xor_dir);
	else {
		const char *dot = strrchr(outp, '.');
		if (dot && dot > base_name(outp)) snprintf(xdir, sizeof xdir, "%.*s_xor", (int)(dot - outp), outp);
		else snprintf(xdir, sizeof xdir, "%s_xor", outp);
	}

	for (int i = 0; i < m.zip.count; i++) {
		const zip_entry *e = &m.zip.entries[i];
		if (!zip_entry_is_qon(&m.zip, e)) {
			printf("copying %s (%.1f MB)%s\n", e->name, e->comp_size / 1e6, nr > 1 ? " into each output" : "");
			for (int k = 0; k < nr; k++) copy_entry(&r[k].w, &m.zip, e);
			continue;
		}
		qon_stream q;
		open_qon(&q, &m, e);
		uint32_t fc = q.hdr.frame_count;
		double fps = fps_of(&q);
		char name[600];
		snprintf(name, sizeof name, "%s", e->name);
		size_t nl = strlen(name);
		if (nl > 4 && !strcmp(name + nl - 4, ".qon")) name[nl - 4] = 0;
		strncat(name, ".dscv", sizeof name - strlen(name) - 1);
		printf("converting %s -> %s, %u frames\n", e->name, name, fc);

		dscv_header h[MAX_RATES];
		for (int k = 0; k < nr; k++) {
			rate_out *R = &r[k];
			pick_geometry((int)q.hdr.width, (int)q.hdr.height, R->bpp16, &R->g);
			make_pps(&R->g, &R->p);
			R->frame_bytes = dsc_picture_bytes(&R->p);
			R->stride = (uint32_t)((R->frame_bytes + 511) & ~(size_t)511);
			R->frames_off = (DSCV_HEADER_SIZE + (uint64_t)fc * DSCV_VBI_ENTRY + 4095) & ~(uint64_t)4095;
			R->total = R->frames_off + (uint64_t)fc * R->stride;
			snprintf(R->entry, sizeof R->entry, "%s", name);
			R->at = zw_begin(&R->w, name, 0, 0, e->mtime, e->mdate, R->total, R->total, 4096);
			R->crc = calloc(fc, sizeof(uint32_t));
			R->failures = R->fallbacks = 0;
			printf("  %.4g bpp: %dx%d kept, %zu bytes/frame = %.2f MB/s, %.2f GB\n", R->bpp16 / 16.0, R->g.width, R->g.height,
			       R->frame_bytes, R->stride * fps / 1e6, R->total / 1e9);
			dscv_header *H = &h[k];
			memset(H, 0, sizeof *H);
			H->version = 1;
			H->src_width = R->g.src_w; H->src_height = R->g.src_h;
			H->crop_x = R->g.crop_x; H->crop_y = 0;
			H->width = R->g.width; H->height = R->g.height;
			H->field0_row = R->g.f0; H->field0_rows = R->g.f0n; H->field1_row = R->g.f1; H->field1_rows = R->g.f1n;
			H->frame_count = fc;
			H->frame_duration_us = q.hdr.frame_duration_in_microseconds;
			H->source_qon_flags = q.hdr.flags;
			H->frame_bytes = (uint32_t)R->frame_bytes;
			H->frame_stride = R->stride;
			H->frames_offset = R->frames_off;
			H->vbi_offset = DSCV_HEADER_SIZE;
			H->vbi_entry_size = DSCV_VBI_ENTRY;
			H->slices_per_frame = (uint32_t)R->p.slices_per_picture;
			dsc_pps_pack(&R->p, H->pps);
		}

		convert_job J;
		memset(&J, 0, sizeof J);
		J.q = &q; J.r = r; J.nr = nr;
		J.opts.disable_ich = no_ich;
		J.count = fc;
		J.vbi = calloc(fc, DSCV_VBI_ENTRY);
		pthread_mutex_init(&J.lock, NULL);
		double t0 = now_seconds();
		pthread_t *th = malloc(sizeof(pthread_t) * (size_t)threads);
		for (int t = 0; t < threads; t++) pthread_create(&th[t], NULL, convert_worker, &J);
		for (;;) {
#ifdef _WIN32
			Sleep(1000);
#else
			sleep(1);
#endif
			pthread_mutex_lock(&J.lock);
			uint32_t done = J.done;
			pthread_mutex_unlock(&J.lock);
			double el = now_seconds() - t0;
			double rate = done / (el > 0 ? el : 1);
			printf("\r  %u / %u frames, %.1f frames/s, %.0f s left   ", done, fc, rate, rate > 0 ? (fc - done) / rate : 0.0);
			fflush(stdout);
			if (done >= fc) break;
		}
		for (int t = 0; t < threads; t++) pthread_join(th[t], NULL);
		free(th);
		printf("\n");
		if (J.read_failures) printf("  WARNING: %d source frame(s) could not be read; stored as black\n", J.read_failures);

		for (int k = 0; k < nr; k++) {
			rate_out *R = &r[k];
			/* header, VBI table and padding, then the frames' CRCs in order */
			size_t head_len = (size_t)R->frames_off;
			uint8_t *head = calloc(1, head_len);
			dscv_pack(&h[k], head);
			memcpy(head + DSCV_HEADER_SIZE, J.vbi, (size_t)fc * DSCV_VBI_ENTRY);
			if (write_full_at(R->w.fd, head, head_len, R->at)) die("write failed%s", "");
			uint32_t crc = crc32_update(0, head, head_len);
			for (uint32_t f = 0; f < fc; f++) crc = crc32_combine64(crc, R->crc[f], R->stride);
			zw_end(&R->w, crc);
			free(head);
			if (R->fallbacks)
				printf("  %.4g bpp: %d frame(s) needed a fallback encode to fit the DSC budget\n", R->bpp16 / 16.0, R->fallbacks);
			if (R->failures)
				printf("  WARNING %.4g bpp: %d frame(s) did not fit the DSC budget (first: frame %u)\n", R->bpp16 / 16.0,
				       R->failures, R->first_failure);
			free(R->crc);
			R->jl += (size_t)snprintf(R->json + R->jl, sizeof R->json - R->jl,
			                          "%s\n    {\n      \"source_entry\": \"%s\",\n      \"entry\": \"%s\",\n"
			                          "      \"codec\": \"VESA DSC 1.1, RGB 4:4:4, 8 bpc\",\n      \"bits_per_pixel\": %.4f,\n"
			                          "      \"width\": %d,\n      \"height\": %d,\n      \"crop_x\": %d,\n      \"frame_count\": %u,\n"
			                          "      \"frame_bytes\": %zu,\n      \"frame_stride\": %u,\n      \"bandwidth_bytes_per_second\": %.0f\n    }",
			                          R->nvid ? "," : "", e->name, name, R->bpp16 / 16.0, R->g.width, R->g.height, R->g.crop_x, fc,
			                          R->frame_bytes, R->stride, R->stride * fps);
			R->nvid++;
		}
		free(J.vbi);
		qon_close(&q);
	}
	for (int k = 0; k < nr; k++) {
		rate_out *R = &r[k];
		R->jl += (size_t)snprintf(R->json + R->jl, sizeof R->json - R->jl, "\n  ]\n}\n");
		uint64_t at = zw_begin(&R->w, "DSCMMI.json", 0, 0, 0, 0x21, R->jl, R->jl, 1);
		write_full_at(R->w.fd, R->json, R->jl, at);
		zw_end(&R->w, crc32_update(0, R->json, R->jl));
		if (zw_finish(&R->w)) die("error finishing %s", R->path);
		printf("wrote %s\n", R->path);
	}
	/* XOR images for each video entry, decoded back from the finished files */
	if (np) {
		for (int k = 0; k < m.qon_count; k++) {
			const zip_entry *e = &m.zip.entries[m.qon_index[k]];
			char name[600], sub[1300];
			snprintf(name, sizeof name, "%s", e->name);
			size_t nl = strlen(name);
			if (nl > 4 && !strcmp(name + nl - 4, ".qon")) name[nl - 4] = 0;
			strncat(name, ".dscv", sizeof name - strlen(name) - 1);
			qon_stream q;
			open_qon(&q, &m, e);
			if (m.qon_count > 1) {
				char clean[600];
				snprintf(clean, sizeof clean, "%s", e->name);
				for (char *c = clean; *c; c++) if (*c == '/' || *c == '\\') *c = '_';
				snprintf(sub, sizeof sub, "%s/%s", xdir, clean);
				mkdir_p(xdir);
			} else {
				snprintf(sub, sizeof sub, "%s", xdir);
			}
			for (int j = 0; j < nr; j++) {
				rate_out *R = &r[j];
				pick_geometry((int)q.hdr.width, (int)q.hdr.height, R->bpp16, &R->g);
				make_pps(&R->g, &R->p);
				R->frame_bytes = dsc_picture_bytes(&R->p);
				R->stride = (uint32_t)((R->frame_bytes + 511) & ~(size_t)511);
				R->frames_off = (DSCV_HEADER_SIZE + (uint64_t)q.hdr.frame_count * DSCV_VBI_ENTRY + 4095) & ~(uint64_t)4095;
			}
			xor_from_outputs(r, nr, name, q.hdr.frame_count, pairs, np, xor_frames, sub, &q);
			qon_close(&q);
		}
	}
	free(r);
	zip_close(&m.zip);
}

/* ------------------------------------------------------------ compare */

typedef struct {
	qon_stream *q;
	const geom *g;
	const dsc_pps *p;
	const int *bpps;
	int nr;
	const rate_pair *pairs;
	int np;
	const uint32_t *frames;
	int count, next;
	const char *dir;
	dsc_enc_opts opts;
	double psnr_sum[MAX_RATES], psnr_min[MAX_RATES];
	xor_stats xs[16];
	pthread_mutex_t lock;
} compare_job;

static void *compare_worker(void *arg)
{
	compare_job *J = arg;
	const geom *g0 = &J->g[0];
	uint8_t *src = malloc((size_t)g0->src_w * g0->src_h * 3);
	uint8_t *dec[MAX_RATES + 1];
	dec[SRC_IDX] = malloc((size_t)g0->width * g0->height * 3);
	size_t maxout = 0;
	for (int k = 0; k < J->nr; k++) {
		dec[k] = malloc((size_t)J->g[k].width * J->g[k].height * 3);
		if (dsc_picture_bytes(&J->p[k]) > maxout) maxout = dsc_picture_bytes(&J->p[k]);
	}
	uint8_t *out = malloc(maxout), *scratch = NULL;
	size_t cap = 0;
	for (;;) {
		pthread_mutex_lock(&J->lock);
		int idx = J->next++;
		pthread_mutex_unlock(&J->lock);
		if (idx >= J->count) break;
		uint32_t f = J->frames[idx];
		if (qon_read_frame(J->q, f, src, &scratch, &cap)) continue;
		for (int y = 0; y < g0->height; y++)
			memcpy(dec[SRC_IDX] + (size_t)y * g0->width * 3, src + ((size_t)y * g0->src_w + g0->crop_x) * 3, (size_t)g0->width * 3);
		for (int p = 0; p < J->np; p++)
			if (J->pairs[p].a == SRC_IDX || J->pairs[p].b == SRC_IDX) {
				char path[1300];
				snprintf(path, sizeof path, "%s/frame%06u_src.png", J->dir, f);
				write_png_rgb(path, dec[SRC_IDX], g0->width, g0->height, (size_t)g0->width * 3);
				break;
			}
		double ps[MAX_RATES];
		for (int k = 0; k < J->nr; k++) {
			const geom *g = &J->g[k];
			const uint8_t *pic = src + (size_t)g->crop_x * 3;
			size_t stride = (size_t)g->src_w * 3;
			int fb;
			encode_with_fallback(&J->p[k], g, &J->opts, pic, stride, out, &fb);
			dsc_decode_picture(&J->p[k], out, dsc_picture_bytes(&J->p[k]), dec[k]);
			ps[k] = rgb_psnr(pic, stride, dec[k], (size_t)g->width * 3, g->width, g->height, NULL);
			if (J->dir) {
				char lab[32], path[1300];
				rate_label(J->bpps[k], lab, sizeof lab);
				snprintf(path, sizeof path, "%s/frame%06u_%s.png", J->dir, f, lab);
				write_png_rgb(path, dec[k], g->width, g->height, (size_t)g->width * 3);
			}
		}
		xor_stats local[16];
		memset(local, 0, sizeof local);
		for (int p = 0; p < J->np; p++) {
			rate_pair pr = J->pairs[p];
			char lab[64];
			pair_label(J->bpps, pr, lab, sizeof lab);
			write_xor(J->dir, lab, f, dec[pr.a], dec[pr.b], g0->width, g0->height, &local[p]);
		}
		pthread_mutex_lock(&J->lock);
		for (int k = 0; k < J->nr; k++) {
			J->psnr_sum[k] += ps[k];
			if (ps[k] < J->psnr_min[k]) J->psnr_min[k] = ps[k];
		}
		for (int p = 0; p < J->np; p++) {
			J->xs[p].bytes += local[p].bytes;
			J->xs[p].differing += local[p].differing;
			J->xs[p].abs_sum += local[p].abs_sum;
			J->xs[p].frames += local[p].frames;
		}
		pthread_mutex_unlock(&J->lock);
	}
	for (int k = 0; k < J->nr; k++) free(dec[k]);
	free(dec[SRC_IDX]);
	free(src); free(out); free(scratch);
	return NULL;
}

static void cmd_compare(const char *path, const int *bpps, int nr, const rate_pair *pairs, int np, int n, const char *dir,
                        int no_ich, const char *entry)
{
	mmi_file m;
	open_mmi(&m, path);
	const zip_entry *e = pick_qon(&m, entry);
	qon_stream q;
	open_qon(&q, &m, e);
	geom g[MAX_RATES];
	dsc_pps p[MAX_RATES];
	double fps = fps_of(&q);
	for (int k = 0; k < nr; k++) {
		pick_geometry((int)q.hdr.width, (int)q.hdr.height, bpps[k], &g[k]);
		make_pps(&g[k], &p[k]);
	}
	uint32_t fc = q.hdr.frame_count;
	if (n > (int)fc) n = (int)fc;
	uint32_t *frames = malloc(sizeof(uint32_t) * (size_t)n);
	for (int k = 0; k < n; k++) frames[k] = (uint32_t)(((double)k + 0.5) * fc / n);
	mkdir_p(dir);
	compare_job J;
	memset(&J, 0, sizeof J);
	J.q = &q; J.g = g; J.p = p; J.bpps = bpps; J.nr = nr; J.pairs = pairs; J.np = np;
	J.frames = frames; J.count = n; J.dir = dir;
	J.opts.disable_ich = no_ich;
	for (int k = 0; k < nr; k++) J.psnr_min[k] = 1e9;
	pthread_mutex_init(&J.lock, NULL);
	int threads = g_threads > 0 ? g_threads : cpu_count();
	pthread_t *th = malloc(sizeof(pthread_t) * (size_t)threads);
	for (int t = 0; t < threads; t++) pthread_create(&th[t], NULL, compare_worker, &J);
	for (int t = 0; t < threads; t++) pthread_join(th[t], NULL);
	free(th);
	printf("%s: %d frames compared, images in %s\n", e->name, n, dir);
	for (int k = 0; k < nr; k++) {
		size_t fb = dsc_picture_bytes(&p[k]);
		uint32_t stride = (uint32_t)((fb + 511) & ~(size_t)511);
		printf("  %5.4g bpp: %.2f MB/s, mean PSNR %.2f dB, worst %.2f dB\n", bpps[k] / 16.0, stride * fps / 1e6,
		       J.psnr_sum[k] / n, J.psnr_min[k]);
	}
	if (np) printf("XOR between pictures:\n");
	for (int pi = 0; pi < np; pi++) {
		char lab[64];
		pair_label(bpps, pairs[pi], lab, sizeof lab);
		report_xor(lab, &J.xs[pi]);
	}
	free(frames);
	qon_close(&q);
	zip_close(&m.zip);
}

/* ------------------------------------------------------- verify / frame */

typedef struct {
	int fd;
	uint64_t data_off;
	dscv_header h;
	dsc_pps p;
} dscv_stream;

static int open_dscv(dscv_stream *s, const zip_file *z, const zip_entry *e)
{
	uint8_t hb[DSCV_HEADER_SIZE];
	char err[256];
	if (e->method != 0 || e->comp_size < DSCV_HEADER_SIZE || read_full_at(z->fd, hb, sizeof hb, e->data_off) ||
	    dscv_unpack(&s->h, hb))
		return -1;
	if (dsc_pps_unpack(&s->p, s->h.pps, err, sizeof err)) {
		fprintf(stderr, "error: %s: %s\n", e->name, err);
		return -1;
	}
	if (dsc_picture_bytes(&s->p) != s->h.frame_bytes || s->h.frames_offset + (uint64_t)s->h.frame_count * s->h.frame_stride > e->comp_size) {
		fprintf(stderr, "error: %s: inconsistent header\n", e->name);
		return -1;
	}
	s->fd = z->fd;
	s->data_off = e->data_off;
	return 0;
}

typedef struct {
	dscv_stream *s;
	qon_stream *src;
	uint32_t *frames;
	int count, next;
	int errors, vbi_mismatch;
	double psnr_sum, psnr_min;
	uint32_t psnr_min_frame;
	pthread_mutex_t lock;
} verify_job;

static void *verify_worker(void *arg)
{
	verify_job *J = arg;
	dscv_stream *s = J->s;
	size_t pic = (size_t)s->h.width * s->h.height * 3;
	uint8_t *in = malloc(s->h.frame_stride), *dec = malloc(pic);
	uint8_t *src = J->src ? malloc((size_t)s->h.src_width * s->h.src_height * 3) : NULL;
	uint8_t *scratch = NULL;
	size_t cap = 0;
	for (;;) {
		pthread_mutex_lock(&J->lock);
		int k = J->next++;
		pthread_mutex_unlock(&J->lock);
		if (k >= J->count) break;
		uint32_t f = J->frames[k];
		int err = read_full_at(s->fd, in, s->h.frame_bytes, s->data_off + s->h.frames_offset + (uint64_t)f * s->h.frame_stride) ||
		          dsc_decode_picture(&s->p, in, s->h.frame_bytes, dec);
		double ps = 0;
		int vbi_bad = 0;
		if (!err && src) {
			if (qon_read_frame(J->src, f, src, &scratch, &cap)) err = 1;
			else {
				ps = rgb_psnr(src + (size_t)s->h.crop_x * 3, (size_t)s->h.src_width * 3, dec, (size_t)s->h.width * 3, s->h.width,
				              s->h.height, NULL);
				if (s->h.vbi_offset) {
					uint8_t vb[16];
					read_full_at(s->fd, vb, 16, s->data_off + s->h.vbi_offset + (uint64_t)f * 16);
					for (int i = 0; i < 4; i++)
						if (rd32le(vb + i * 4) != vbi_decode_line(src, s->h.src_width, s->h.src_width, 0, VBI_ROWS[i])) vbi_bad = 1;
				}
			}
		}
		pthread_mutex_lock(&J->lock);
		if (err) J->errors++;
		J->vbi_mismatch += vbi_bad;
		if (src && !err) {
			J->psnr_sum += ps;
			if (ps < J->psnr_min) { J->psnr_min = ps; J->psnr_min_frame = f; }
		}
		pthread_mutex_unlock(&J->lock);
	}
	free(in); free(dec); free(src); free(scratch);
	return NULL;
}

static void cmd_verify(const char *path, const char *source, int n)
{
	zip_file z;
	char err[256];
	if (zip_open(&z, path, err, sizeof err)) die("%s", err);
	mmi_file sm;
	int have_src = 0;
	if (source) { open_mmi(&sm, source); have_src = 1; }
	int streams = 0, failed = 0;
	for (int i = 0; i < z.count; i++) {
		dscv_stream s;
		if (open_dscv(&s, &z, &z.entries[i])) continue;
		streams++;
		uint32_t fc = s.h.frame_count;
		int count = n > 0 && (uint32_t)n < fc ? n : (int)fc;
		verify_job J;
		memset(&J, 0, sizeof J);
		J.s = &s;
		J.count = count;
		J.psnr_min = 1e9;
		J.frames = malloc(sizeof(uint32_t) * (size_t)count);
		for (int k = 0; k < count; k++) J.frames[k] = count == (int)fc ? (uint32_t)k : (uint32_t)(((double)k + 0.5) * fc / count);
		qon_stream q;
		if (have_src) {
			char qn[600];
			snprintf(qn, sizeof qn, "%s", z.entries[i].name);
			size_t l = strlen(qn);
			if (l > 5 && !strcmp(qn + l - 5, ".dscv")) qn[l - 5] = 0;
			strncat(qn, ".qon", sizeof qn - strlen(qn) - 1);
			const zip_entry *se = zip_find(&sm.zip, qn);
			if (!se) die("source has no entry %s", qn);
			open_qon(&q, &sm, se);
			J.src = &q;
		}
		pthread_mutex_init(&J.lock, NULL);
		int threads = g_threads > 0 ? g_threads : cpu_count();
		double t0 = now_seconds();
		pthread_t *th = malloc(sizeof(pthread_t) * (size_t)threads);
		for (int t = 0; t < threads; t++) pthread_create(&th[t], NULL, verify_worker, &J);
		for (int t = 0; t < threads; t++) pthread_join(th[t], NULL);
		free(th);
		printf("%s: decoded %d of %u frames in %.1f s, %d error(s)\n", z.entries[i].name, count, fc, now_seconds() - t0, J.errors);
		if (have_src && count > J.errors)
			printf("  vs source: mean PSNR %.2f dB, worst %.2f dB (frame %u), VBI table mismatches %d\n",
			       J.psnr_sum / (count - J.errors), J.psnr_min, J.psnr_min_frame, J.vbi_mismatch);
		failed += J.errors + J.vbi_mismatch;
		if (have_src) qon_close(&q);
		free(J.frames);
	}
	if (!streams) die("no DSCV video streams in %s", path);
	zip_close(&z);
	if (have_src) zip_close(&sm.zip);
	if (failed) exit(2);
}

static void cmd_frame(const char *path, uint32_t f, const char *png, const char *entry)
{
	mmi_file m;
	open_mmi(&m, path);
	for (int i = 0; i < m.zip.count; i++) {
		const zip_entry *e = &m.zip.entries[i];
		if (entry && strcmp(e->name, entry)) continue;
		dscv_stream s;
		if (!open_dscv(&s, &m.zip, e)) {
			if (f >= s.h.frame_count) die("frame out of range%s", "");
			uint8_t *in = malloc(s.h.frame_bytes), *dec = malloc((size_t)s.h.width * s.h.height * 3);
			if (read_full_at(s.fd, in, s.h.frame_bytes, s.data_off + s.h.frames_offset + (uint64_t)f * s.h.frame_stride) ||
			    dsc_decode_picture(&s.p, in, s.h.frame_bytes, dec))
				die("decode failed%s", "");
			write_png_rgb(png, dec, s.h.width, s.h.height, (size_t)s.h.width * 3);
			printf("wrote %s (%dx%d, DSC frame %u of %s)\n", png, s.h.width, s.h.height, f, e->name);
			return;
		}
		if (zip_entry_is_qon(&m.zip, e)) {
			qon_stream q;
			open_qon(&q, &m, e);
			if (f >= q.hdr.frame_count) die("frame out of range%s", "");
			uint8_t *rgb = malloc((size_t)q.hdr.width * q.hdr.height * 3), *scratch = NULL;
			size_t cap = 0;
			if (qon_read_frame(&q, f, rgb, &scratch, &cap)) die("decode failed%s", "");
			write_png_rgb(png, rgb, (int)q.hdr.width, (int)q.hdr.height, (size_t)q.hdr.width * 3);
			printf("wrote %s (%ux%u, QON frame %u of %s)\n", png, q.hdr.width, q.hdr.height, f, e->name);
			return;
		}
	}
	die("no video stream found%s", "");
}

/* --------------------------------------------------------------- main */

static void usage(void)
{
	fprintf(stderr,
	        "mmidsc " TOOL_VERSION " - LaserActive MMI to constant-bit-rate DSC (.dscmmi)\n\n"
	        "usage:\n"
	        "  mmidsc info FILE                     list entries and video streams (.mmi or .dscmmi)\n"
	        "  mmidsc analyze IN.mmi [options]      test DSC quality and bandwidth on sample frames\n"
	        "      -n N          number of frames (default 300, spread across the disc side)\n"
	        "      -s FRAME      start at FRAME and test consecutive frames (or every --step)\n"
	        "      --step K      test every K-th frame\n"
	        "      --png DIR     write source / DSC / difference PNGs of the worst and median frames\n"
	        "      --csv FILE    per-frame results\n"
	        "  mmidsc convert IN.mmi OUT.dscmmi    convert all analog video streams, copy everything else\n"
	        "      --bpp 8,9,10  several rates in one pass: one file per rate (OUT.8bpp.dscmmi, ...)\n"
	        "      --xor A:B     after converting, write XOR images of decoded rate A vs rate B\n"
	        "                    (repeatable, or --xor all for every pair of rates); use src for\n"
	        "                    the lossless original, e.g. --xor src:10\n"
	        "      --xor-frames N  frames to sample for XOR images (default 10)\n"
	        "      --xor-dir DIR   where to put them (default OUT_xor)\n"
	        "  mmidsc compare IN.mmi --bpp 8,9,10 [--xor A:B|all] [-n N] [--png DIR]\n"
	        "                                       encode sample frames at each rate, no output files;\n"
	        "                                       writes each rate's decoded PNG and the XOR images\n"
	        "  mmidsc verify FILE.dscmmi [--source IN.mmi] [-n N]\n"
	        "                                       decode every frame (or N spread frames), optionally compare to the source\n"
	        "  mmidsc frame FILE N OUT.png          decode frame N of an .mmi or .dscmmi to PNG\n\n"
	        "common options:\n"
	        "  --bpp X       DSC bits per pixel (default 8; 6 to 12 work; 8, 9 and 10 are the supported\n"
	        "                rates for the MiSTer decoder)\n"
	        "  --no-ich      never use indexed colour history (experiment)\n"
	        "  -t N          threads (default: all CPUs)\n"
	        "  -v ENTRY      video entry for analyze/compare/frame (default: first)\n");
	exit(1);
}

static int parse_rate(const char *s)
{
	char *end;
	double v = strtod(s, &end);
	while (*end == ' ') end++;
	if (end == s || (*end && strncmp(end, "bpp", 3))) die("bad rate '%s'", s);
	int b = (int)(v * 16 + 0.5);
	if (b < 96 || b > 192) die("--bpp must be between 6 and 12 (got %s)", s);
	return b;
}

static int find_rate(const int *bpps, int nr, const char *s)
{
	if (!strcmp(s, "src") || !strcmp(s, "source") || !strcmp(s, "orig") || !strcmp(s, "original")) return SRC_IDX;
	int b = parse_rate(s);
	for (int k = 0; k < nr; k++) if (bpps[k] == b) return k;
	die("--xor rate %s is not one of the --bpp rates", s);
	return -1;
}

int main(int argc, char **argv)
{
	if (argc < 3) usage();
	const char *cmd = argv[1];
	int n = -1, step = 0, no_ich = 0, xor_frames = 10, xor_all = 0;
	long start = -1;
	const char *png = NULL, *csv = NULL, *source = NULL, *entry = NULL, *xor_dir = NULL, *bpp_arg = "8";
	const char *xor_args[32];
	int nxor = 0;
	const char *pos[3] = { NULL, NULL, NULL };
	int npos = 0;
	for (int i = 2; i < argc; i++) {
		const char *a = argv[i];
#define NEXT() (i + 1 < argc ? argv[++i] : (usage(), (char *)NULL))
		if (!strcmp(a, "-n")) n = atoi(NEXT());
		else if (!strcmp(a, "-s")) start = atol(NEXT());
		else if (!strcmp(a, "--step")) step = atoi(NEXT());
		else if (!strcmp(a, "--bpp")) bpp_arg = NEXT();
		else if (!strcmp(a, "--no-ich")) no_ich = 1;
		else if (!strcmp(a, "--png")) png = NEXT();
		else if (!strcmp(a, "--csv")) csv = NEXT();
		else if (!strcmp(a, "--source")) source = NEXT();
		else if (!strcmp(a, "-t")) g_threads = atoi(NEXT());
		else if (!strcmp(a, "-v")) entry = NEXT();
		else if (!strcmp(a, "--xor")) {
			const char *x = NEXT();
			if (!strcmp(x, "all")) xor_all = 1;
			else if (nxor < 32) xor_args[nxor++] = x;
		}
		else if (!strcmp(a, "--xor-frames")) xor_frames = atoi(NEXT());
		else if (!strcmp(a, "--xor-dir")) xor_dir = NEXT();
		else if (a[0] == '-' && a[1]) usage();
		else if (npos < 3) pos[npos++] = a;
		else usage();
	}
	/* rates: "8", "8,9,10", "8 9 10" */
	int bpps[MAX_RATES], nr = 0;
	{
		char buf[256];
		snprintf(buf, sizeof buf, "%s", bpp_arg);
		for (char *t = strtok(buf, ", "); t; t = strtok(NULL, ", ")) {
			if (nr == MAX_RATES) die("at most 6 rates%s", "");
			int b = parse_rate(t);
			for (int k = 0; k < nr; k++) if (bpps[k] == b) die("rate %s given twice", t);
			bpps[nr++] = b;
		}
		if (!nr) die("--bpp needs a value%s", "");
	}
	rate_pair pairs[16];
	int np = 0;
	if (xor_all) {
		for (int a = 0; a < nr; a++)
			for (int b = a + 1; b < nr; b++)
				if (np < 16) pairs[np++] = (rate_pair){ a, b };
	}
	for (int k = 0; k < nxor; k++) {
		char buf[64];
		snprintf(buf, sizeof buf, "%s", xor_args[k]);
		char *c = strchr(buf, ':');
		if (!c) die("--xor wants A:B, e.g. --xor 8:10 (got %s)", xor_args[k]);
		*c = 0;
		rate_pair pr = { find_rate(bpps, nr, buf), find_rate(bpps, nr, c + 1) };
		if (pr.a == pr.b) die("--xor %s compares a rate with itself", xor_args[k]);
		int dup = 0;
		for (int j = 0; j < np; j++)
			if ((pairs[j].a == pr.a && pairs[j].b == pr.b) || (pairs[j].a == pr.b && pairs[j].b == pr.a)) dup = 1;
		if (!dup && np < 16) pairs[np++] = pr;
	}
	if (xor_all && nr < 2) die("--xor all needs at least two rates in --bpp, e.g. --bpp 8,10 --xor all%s", "");
	int single = strcmp(cmd, "convert") && strcmp(cmd, "compare");
	if (single && nr > 1) die("%s takes a single --bpp rate", cmd);
	setvbuf(stdout, NULL, _IONBF, 0);
	if (!strcmp(cmd, "info") && npos == 1) cmd_info(pos[0]);
	else if (!strcmp(cmd, "analyze") && npos == 1) cmd_analyze(pos[0], n > 0 ? n : 300, start, step, bpps[0], no_ich, png, csv, entry);
	else if (!strcmp(cmd, "convert") && npos == 2)
		cmd_convert(pos[0], pos[1], bpps, nr, no_ich, pairs, np, xor_frames, xor_dir);
	else if (!strcmp(cmd, "compare") && npos == 1) {
		char dir[1100];
		if (png) snprintf(dir, sizeof dir, "%s", png);
		else if (xor_dir) snprintf(dir, sizeof dir, "%s", xor_dir);
		else snprintf(dir, sizeof dir, "compare_png");
		cmd_compare(pos[0], bpps, nr, pairs, np, n > 0 ? n : 10, dir, no_ich, entry);
	}
	else if (!strcmp(cmd, "verify") && npos == 1) cmd_verify(pos[0], source, n);
	else if (!strcmp(cmd, "frame") && npos == 3) cmd_frame(pos[0], (uint32_t)atol(pos[1]), pos[2], entry);
	else usage();
	return 0;
}
