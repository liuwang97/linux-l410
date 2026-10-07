// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video decoder: HEVC
 *
 * Frame-based decoding: a request carries all slice segments of a picture
 * (Annex B) and the array of their slice parameters; without the array
 * (Chromium) the driver parses the slice segment headers itself. The slices are copied
 * as RBSP into a stream buffer (the VDH does not skip emulation prevention
 * bytes) and decoded in one VDH run. Message layout of the HiSilicon VFMW
 * HEVC HAL (VDH V5R6C1/V500R003).
 */

#include <linux/string.h>
#include <media/v4l2-mem2mem.h>

#include "hisi_vdec.h"
#include "hisi_vdec_tables.h"

#define HEVC_MAX_SLICES		(HIVDEC_MAX_SLICES - 2)

#define HEVC_APC_SLOTS		16
#define HEVC_STREAM_SIZE_MIN	SZ_8M

/* line buffers, vendor sizes for 4096 x 4096 */
#define HEVC_TOP_LEN		(64 * 4 * 4096)
#define HEVC_LEFT_LEN		(64 * 4 * 4096)
#define HEVC_TILE_INFO_LEN	2048
#define HEVC_HEAD_LEN		SZ_256K

enum { HEVC_B = 0, HEVC_P = 1, HEVC_I = 2 };

struct hivdec_hevc_slice {
	u32 offset;		/* NAL header byte, in the stream buffer */
	u32 data_byte;		/* slice_segment_data() start in the RBSP */
	u32 bits;		/* valid bits from there */
};

struct hivdec_hevc_buf_meta {
	s8 apc;
};

struct hivdec_hevc_ctx {
	struct hivdec_aux_buf stream;
	struct hivdec_aux_buf work;
	u32 cabac, tile_info, sed_top, pmv_top, pmv_left, rcn_top;
	u32 dblk_left, dblk_top, sao_left, sao_top, head;	/* IOVAs */
	u32 *tile_cpu;
	struct hivdec_hevc_slice slices[HEVC_MAX_SLICES];
	u32 sc[HEVC_MAX_SLICES + 1];		/* start code positions in the OUTPUT buffer */
	struct v4l2_ctrl_hevc_slice_params *fsp;	/* parsed when userspace gives none */
	struct hivdec_hevc_buf_meta meta[VB2_MAX_FRAME];
	s8 apc[HEVC_APC_SLOTS];
	u16 *rs2ts, *ts2rs;
	u32 nctbs;
	u32 col_bd[22], row_bd[22];
};

/* ---------------------------------------------------------------- setup */

static int hivdec_hevc_start(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_hevc_ctx *h;
	u32 base, off;
	int i, ret;

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->fsp = kvcalloc(HEVC_MAX_SLICES, sizeof(*h->fsp), GFP_KERNEL);
	if (!h->fsp) {
		kfree(h);
		return -ENOMEM;
	}
	ret = hivdec_aux_alloc(vdec, &h->stream,
			       max_t(u32, HEVC_STREAM_SIZE_MIN,
				     ctx->coded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage + SZ_1M));
	if (ret)
		goto err_free;
	ret = hivdec_aux_alloc(vdec, &h->work, SZ_8K + 4 * HEVC_TOP_LEN +
			       4 * HEVC_LEFT_LEN + HEVC_HEAD_LEN);
	if (ret)
		goto err_stream;

	base = h->work.map.iova;
	memcpy(h->work.cpu, hivdec_hevc_cabac_mn, sizeof(hivdec_hevc_cabac_mn));
	h->cabac = base;
	h->tile_info = base + SZ_4K;
	h->tile_cpu = h->work.cpu + SZ_4K;
	off = SZ_8K;
	h->sed_top = base + off;
	off += HEVC_TOP_LEN;
	h->pmv_top = base + off;
	off += HEVC_TOP_LEN;
	h->rcn_top = base + off;
	off += HEVC_TOP_LEN;
	h->dblk_top = base + off;
	off += HEVC_TOP_LEN;
	h->pmv_left = base + off;
	off += HEVC_LEFT_LEN;
	h->dblk_left = base + off;
	off += HEVC_LEFT_LEN;
	h->sao_left = base + off;
	off += HEVC_LEFT_LEN;
	h->sao_top = base + off;
	off += HEVC_LEFT_LEN;
	h->head = base + off;
	hivdec_aux_sync_for_device(vdec, &h->work);

	for (i = 0; i < HEVC_APC_SLOTS; i++)
		h->apc[i] = -1;
	for (i = 0; i < VB2_MAX_FRAME; i++)
		h->meta[i].apc = -1;
	ctx->priv = h;
	return 0;

err_stream:
	hivdec_aux_free(vdec, &h->stream);
err_free:
	kvfree(h->fsp);
	kfree(h);
	return ret;
}

static void hivdec_hevc_stop(struct hivdec_ctx *ctx)
{
	struct hivdec_hevc_ctx *h = ctx->priv;

	if (!h)
		return;
	hivdec_aux_free(ctx->dev, &h->work);
	hivdec_aux_free(ctx->dev, &h->stream);
	kvfree(h->rs2ts);
	kvfree(h->ts2rs);
	kvfree(h->fsp);
	kfree(h);
	ctx->priv = NULL;
}

static int hivdec_hevc_adjust_fmt(struct hivdec_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 3 / 2;
	fmt->plane_fmt[0].sizeimage = max_t(u32, fmt->plane_fmt[0].sizeimage, SZ_1M);
	return 0;
}

static int hivdec_hevc_try_ctrl(struct hivdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_HEVC_SPS) {
		const struct v4l2_ctrl_hevc_sps *sps = ctrl->p_new.p_hevc_sps;

		if (sps->chroma_format_idc != 1)
			return -EINVAL;
		if (sps->bit_depth_luma_minus8 > 2 ||
		    sps->bit_depth_chroma_minus8 != sps->bit_depth_luma_minus8)
			return -EINVAL;
		if (sps->pic_width_in_luma_samples > 4096 ||
		    sps->pic_height_in_luma_samples > 4096)
			return -EINVAL;
	}
	return 0;
}

static unsigned int hivdec_hevc_bit_depth(const struct v4l2_ctrl *ctrl)
{
	if (ctrl->id != V4L2_CID_STATELESS_HEVC_SPS)
		return 0;
	return ctrl->p_new.p_hevc_sps->bit_depth_luma_minus8 + 8;
}

/* ---------------------------------------------------------------- helpers */

static u32 hevc_nal_to_rbsp(u8 *dst, const u8 *src, u32 len, u32 raw_limit, u32 *rbsp_at_limit)
{
	u32 i, n = 0, zeros = 0;

	for (i = 0; i < len; i++) {
		if (i == raw_limit)
			*rbsp_at_limit = n;
		if (zeros >= 2 && src[i] == 0x03) {
			zeros = 0;
			continue;
		}
		zeros = src[i] ? 0 : zeros + 1;
		dst[n++] = src[i];
	}
	if (raw_limit >= len)
		*rbsp_at_limit = n;
	return n;
}

/*
 * bits of slice_segment_data() including rbsp_stop_one_bit: the VDH does not
 * read past this length, and its CABAC engine needs the stop bit to decode
 * the final bins of the last CTB.
 */
static u32 hevc_payload_bits(const u8 *p, u32 len)
{
	while (len && !p[len - 1])
		len--;
	if (!len)
		return 0;
	return len * 8 - __builtin_ctz(p[len - 1]);
}

static int hevc_find_buf(struct hivdec_ctx *ctx, u64 ts)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = vb2_find_buffer(q, ts);

	return vb ? vb->index : -1;
}

static struct hivdec_decoded_buffer *hevc_buf(struct hivdec_ctx *ctx, int idx)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = idx >= 0 ? vb2_get_buffer(q, idx) : NULL;

	return vb ? vb2_to_hivdec_buf(vb) : NULL;
}

/* stable slot per reference frame (co-located MV data refers to slots) */
static void hevc_update_apc(struct hivdec_hevc_ctx *h, const int *dpb_idx, int n)
{
	bool used[VB2_MAX_FRAME] = { };
	int i, s;

	for (i = 0; i < n; i++)
		if (dpb_idx[i] >= 0)
			used[dpb_idx[i]] = true;
	for (s = 0; s < HEVC_APC_SLOTS; s++)
		if (h->apc[s] >= 0 && !used[h->apc[s]]) {
			h->meta[h->apc[s]].apc = -1;
			h->apc[s] = -1;
		}
	for (i = 0; i < n; i++) {
		int b = dpb_idx[i];

		if (b < 0 || h->meta[b].apc >= 0)
			continue;
		for (s = 0; s < HEVC_APC_SLOTS && h->apc[s] >= 0; s++)
			;
		if (s == HEVC_APC_SLOTS)
			break;
		h->apc[s] = b;
		h->meta[b].apc = s;
	}
}

/* scaling lists: raster order in V4L2; the VDH wants column-interleaved words */
static void hevc_qm_4x4(const u8 *q, u32 *d)
{
	int j;

	for (j = 0; j < 4; j++)
		d[j] = q[j] | q[8 + j] << 8 | q[4 + j] << 16 | (u32)q[12 + j] << 24;
}

static void hevc_qm_8x8(const u8 *q, u32 *d)
{
	int j;

	for (j = 0; j < 8; j++) {
		d[2 * j] = q[j] | q[16 + j] << 8 | q[32 + j] << 16 | (u32)q[48 + j] << 24;
		d[2 * j + 1] = q[8 + j] | q[24 + j] << 8 | q[40 + j] << 16 | (u32)q[56 + j] << 24;
	}
}

static void hevc_qm_16_32(const u8 *q, u32 *d)
{
	int j;

	for (j = 0; j < 8; j++) {
		d[2 * j] = q[j] | q[8 + j] << 8 | q[16 + j] << 16 | (u32)q[24 + j] << 24;
		d[2 * j + 1] = q[32 + j] | q[40 + j] << 8 | q[48 + j] << 16 | (u32)q[56 + j] << 24;
	}
}

static void hevc_build_qmatrix(const struct v4l2_ctrl_hevc_scaling_matrix *sm, u32 *q)
{
	int m;

	for (m = 0; m < 6; m++)
		hevc_qm_8x8(sm->scaling_list_8x8[m], &q[16 * m]);
	for (m = 0; m < 6; m++)
		hevc_qm_16_32(sm->scaling_list_16x16[m], &q[96 + 16 * m]);
	for (m = 0; m < 2; m++)
		hevc_qm_16_32(sm->scaling_list_32x32[m], &q[192 + 16 * m]);
	for (m = 0; m < 6; m++)
		hevc_qm_4x4(sm->scaling_list_4x4[m], &q[224 + 4 * m]);
	q[248] = sm->scaling_list_dc_coef_16x16[0] | sm->scaling_list_dc_coef_16x16[1] << 8 |
		 sm->scaling_list_dc_coef_16x16[2] << 16 | (u32)sm->scaling_list_dc_coef_16x16[3] << 24;
	q[249] = sm->scaling_list_dc_coef_16x16[4] | sm->scaling_list_dc_coef_16x16[5] << 8 |
		 sm->scaling_list_dc_coef_32x32[0] << 16 | (u32)sm->scaling_list_dc_coef_32x32[1] << 24;
}

/* tile boundaries (6.5.1), tile info buffer, CTB raster <-> tile scan maps */
static int hevc_setup_tiles(struct hivdec_hevc_ctx *h, const struct v4l2_ctrl_hevc_pps *pps,
			    u32 wctb, u32 hctb, u32 log2_ctb)
{
	bool tiles = pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
	u32 ncols = tiles ? pps->num_tile_columns_minus1 + 1 : 1;
	u32 nrows = tiles ? pps->num_tile_rows_minus1 + 1 : 1;
	u32 i, j, k, n = wctb * hctb, *t = h->tile_cpu;

	if (ncols > 10 || nrows > 11)
		return -EINVAL;
	h->col_bd[0] = 0;
	for (i = 0; i < ncols; i++) {
		u32 w;

		if (!tiles)
			w = wctb;
		else if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
			w = ((i + 1) * wctb) / ncols - (i * wctb) / ncols;
		else if (i < ncols - 1)
			w = pps->column_width_minus1[i] + 1;
		else
			w = wctb - h->col_bd[i];
		h->col_bd[i + 1] = h->col_bd[i] + w;
	}
	h->row_bd[0] = 0;
	for (j = 0; j < nrows; j++) {
		u32 r;

		if (!tiles)
			r = hctb;
		else if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
			r = ((j + 1) * hctb) / nrows - (j * hctb) / nrows;
		else if (j < nrows - 1)
			r = pps->row_height_minus1[j] + 1;
		else
			r = hctb - h->row_bd[j];
		h->row_bd[j + 1] = h->row_bd[j] + r;
	}
	if (h->col_bd[ncols] != wctb || h->row_bd[nrows] != hctb)
		return -EINVAL;

	/* tile info: tile index per 16-pixel column/row, then tile CTB ranges */
	memset(t, 0, HEVC_TILE_INFO_LEN);
	for (i = 0; i < ncols; i++) {
		for (k = h->col_bd[i] << (log2_ctb - 4); k < h->col_bd[i + 1] << (log2_ctb - 4); k++)
			if (k < 256)
				t[k / 4] |= i << (8 * (k % 4));
		t[256 + i] = h->col_bd[i] | (h->col_bd[i + 1] - 1) << 16;
	}
	for (j = 0; j < nrows; j++) {
		for (k = h->row_bd[j] << (log2_ctb - 4); k < h->row_bd[j + 1] << (log2_ctb - 4); k++)
			if (k < 256)
				t[128 + k / 4] |= j << (8 * (k % 4));
		t[276 + j] = h->row_bd[j] | (h->row_bd[j + 1] - 1) << 16;
	}

	if (n != h->nctbs) {
		kvfree(h->rs2ts);
		kvfree(h->ts2rs);
		h->rs2ts = kvcalloc(n, sizeof(u16), GFP_KERNEL);
		h->ts2rs = kvcalloc(n, sizeof(u16), GFP_KERNEL);
		if (!h->rs2ts || !h->ts2rs) {
			h->nctbs = 0;
			return -ENOMEM;
		}
		h->nctbs = n;
	}
	for (k = 0; k < n; k++) {
		u32 tbx = k % wctb, tby = k / wctb, tx = 0, ty = 0, ts = 0;

		for (i = 0; i < ncols; i++)
			if (tbx >= h->col_bd[i])
				tx = i;
		for (j = 0; j < nrows; j++)
			if (tby >= h->row_bd[j])
				ty = j;
		for (i = 0; i < tx; i++)
			ts += (h->row_bd[ty + 1] - h->row_bd[ty]) * (h->col_bd[i + 1] - h->col_bd[i]);
		for (j = 0; j < ty; j++)
			ts += wctb * (h->row_bd[j + 1] - h->row_bd[j]);
		ts += (tby - h->row_bd[ty]) * (h->col_bd[tx + 1] - h->col_bd[tx]) + tbx - h->col_bd[tx];
		h->rs2ts[k] = ts;
		h->ts2rs[ts] = k;
	}
	return 0;
}

/* ---------------------------------------------------------------- slice headers */

/* bit reader over the RBSP */
struct hevc_bits {
	const u8 *p;
	u32 len, pos;
	bool error;
};

static u32 hevc_u(struct hevc_bits *b, unsigned int n)
{
	u32 v = 0;

	while (n--) {
		if ((b->pos >> 3) >= b->len) {
			b->error = true;
			return 0;
		}
		v = v << 1 | ((b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1);
		b->pos++;
	}
	return v;
}

static u32 hevc_ue(struct hevc_bits *b)
{
	unsigned int lz = 0;

	while (!hevc_u(b, 1) && !b->error)
		if (++lz > 31) {
			b->error = true;
			return 0;
		}
	return (1U << lz) - 1 + hevc_u(b, lz);
}

static s32 hevc_se(struct hevc_bits *b)
{
	u32 v = hevc_ue(b);

	return v & 1 ? (s32)((v + 1) >> 1) : -(s32)(v >> 1);
}

static unsigned int hevc_ceil_log2(u32 v)
{
	return v > 1 ? fls(v - 1) : 0;
}

/* st_ref_pic_set(num_short_term_ref_pic_sets) of a slice header (7.3.7) */
static void hevc_skip_st_rps(struct hevc_bits *b, const struct v4l2_ctrl_hevc_sps *sps,
			     const struct v4l2_ctrl_hevc_decode_params *dp)
{
	u32 i, n;

	if (sps->num_short_term_ref_pic_sets && hevc_u(b, 1)) {
		/* inter_ref_pic_set_prediction_flag */
		hevc_ue(b);			/* delta_idx_minus1 */
		hevc_u(b, 1);			/* delta_rps_sign */
		hevc_ue(b);			/* abs_delta_rps_minus1 */
		for (i = 0; i <= dp->num_delta_pocs_of_ref_rps_idx && !b->error; i++)
			if (!hevc_u(b, 1))	/* used_by_curr_pic_flag */
				hevc_u(b, 1);	/* use_delta_flag */
		return;
	}
	n = hevc_ue(b);				/* num_negative_pics */
	n += hevc_ue(b);			/* num_positive_pics */
	for (i = 0; i < n && !b->error; i++) {
		hevc_ue(b);			/* delta_poc_sX_minus1 */
		hevc_u(b, 1);			/* used_by_curr_pic_sX_flag */
	}
}

/* pred_weight_table() (7.3.6.3) into the V4L2 layout (final chroma offsets) */
static void hevc_parse_pwt(struct hevc_bits *b, const struct v4l2_ctrl_hevc_sps *sps,
			   struct v4l2_ctrl_hevc_slice_params *sp)
{
	struct v4l2_hevc_pred_weight_table *w = &sp->pred_weight_table;
	bool chroma = sps->chroma_format_idc != 0;
	unsigned int l, i, j, n[2];
	u32 lf, cf;

	n[0] = sp->num_ref_idx_l0_active_minus1 + 1;
	n[1] = sp->slice_type == 0 ? sp->num_ref_idx_l1_active_minus1 + 1 : 0;
	w->luma_log2_weight_denom = hevc_ue(b);
	if (chroma)
		w->delta_chroma_log2_weight_denom = hevc_se(b);
	for (l = 0; l < 2; l++) {
		s8 *dl = l ? w->delta_luma_weight_l1 : w->delta_luma_weight_l0;
		s8 *lo = l ? w->luma_offset_l1 : w->luma_offset_l0;
		s8 (*dc)[2] = l ? w->delta_chroma_weight_l1 : w->delta_chroma_weight_l0;
		s8 (*co)[2] = l ? w->chroma_offset_l1 : w->chroma_offset_l0;
		u32 cd = w->luma_log2_weight_denom + w->delta_chroma_log2_weight_denom;

		if (!n[l])
			break;
		lf = cf = 0;
		for (i = 0; i < n[l]; i++)
			lf |= hevc_u(b, 1) << i;	/* luma_weight_lX_flag */
		if (chroma)
			for (i = 0; i < n[l]; i++)
				cf |= hevc_u(b, 1) << i;	/* chroma_weight_lX_flag */
		for (i = 0; i < n[l] && !b->error; i++) {
			if (lf & BIT(i)) {
				dl[i] = hevc_se(b);
				lo[i] = hevc_se(b);
			}
			if (!(cf & BIT(i)))
				continue;
			for (j = 0; j < 2; j++) {
				s32 cw, o;

				dc[i][j] = hevc_se(b);
				cw = (1 << cd) + dc[i][j];
				o = 128 + hevc_se(b) - ((128 * cw) >> cd);
				co[i][j] = clamp(o, -128, 127);
			}
		}
	}
}

/*
 * slice_segment_header() (7.3.6.1) of the NAL in @b (RBSP, from the NAL
 * header) into @sp; dependent segments inherit from @indep. Returns the
 * RBSP byte offset of slice_segment_data().
 */
static int hevc_parse_slice(struct hevc_bits *b, const struct v4l2_ctrl_hevc_sps *sps,
			    const struct v4l2_ctrl_hevc_pps *pps,
			    const struct v4l2_ctrl_hevc_decode_params *dp, u32 nctbs,
			    const struct v4l2_ctrl_hevc_slice_params *indep,
			    struct v4l2_ctrl_hevc_slice_params *sp)
{
	u32 nal_type = (b->p[0] >> 1) & 0x3f, i, n, total, first, dep = 0, addr = 0;
	u32 list_entry[2][V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = { };
	bool sao = false, dbk_disabled, mod[2] = { };
	u8 temp[2][V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	unsigned int l, k;

	hevc_u(b, 16);					/* NAL header */
	first = hevc_u(b, 1);				/* first_slice_segment_in_pic_flag */
	if (nal_type >= 16 && nal_type <= 23)
		hevc_u(b, 1);				/* no_output_of_prior_pics_flag */
	hevc_ue(b);					/* slice_pic_parameter_set_id */
	if (!first) {
		if (pps->flags & V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED)
			dep = hevc_u(b, 1);
		addr = hevc_u(b, hevc_ceil_log2(nctbs));
	}
	if (dep) {
		if (!indep)
			return -EINVAL;
		*sp = *indep;
		sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
		sp->slice_segment_addr = addr;
		goto entry_points;
	}

	memset(sp, 0, sizeof(*sp));
	sp->nal_unit_type = nal_type;
	sp->nuh_temporal_id_plus1 = b->p[1] & 7;
	sp->slice_segment_addr = addr;
	hevc_u(b, pps->num_extra_slice_header_bits);	/* slice_reserved_flag */
	sp->slice_type = hevc_ue(b);
	if (sp->slice_type > 2)
		return -EINVAL;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)
		hevc_u(b, 1);				/* pic_output_flag */
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		sp->colour_plane_id = hevc_u(b, 2);
	sp->slice_pic_order_cnt = dp->pic_order_cnt_val;
	if (nal_type != 19 && nal_type != 20) {		/* not IDR */
		hevc_u(b, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (!hevc_u(b, 1))			/* short_term_ref_pic_set_sps_flag */
			hevc_skip_st_rps(b, sps, dp);
		else if (sps->num_short_term_ref_pic_sets > 1)
			hevc_u(b, hevc_ceil_log2(sps->num_short_term_ref_pic_sets));
		if (sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) {
			u32 nsps = 0, npics;

			if (sps->num_long_term_ref_pics_sps)
				nsps = hevc_ue(b);
			npics = hevc_ue(b);
			for (i = 0; i < nsps + npics && !b->error; i++) {
				if (i < nsps) {
					if (sps->num_long_term_ref_pics_sps > 1)
						hevc_u(b, hevc_ceil_log2(sps->num_long_term_ref_pics_sps));
				} else {
					hevc_u(b, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
					hevc_u(b, 1);	/* used_by_curr_pic_lt_flag */
				}
				if (hevc_u(b, 1))	/* delta_poc_msb_present_flag */
					hevc_ue(b);
			}
		}
		if ((sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) && hevc_u(b, 1))
			sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
	}
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) {
		if (hevc_u(b, 1))
			sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA;
		if (sps->chroma_format_idc && hevc_u(b, 1))
			sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA;
		sao = sp->flags & (V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA |
				   V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA);
	}
	sp->collocated_ref_idx = 0;
	if (sp->slice_type != 2) {			/* P or B */
		sp->num_ref_idx_l0_active_minus1 = pps->num_ref_idx_l0_default_active_minus1;
		sp->num_ref_idx_l1_active_minus1 = pps->num_ref_idx_l1_default_active_minus1;
		if (hevc_u(b, 1)) {			/* num_ref_idx_active_override_flag */
			sp->num_ref_idx_l0_active_minus1 = hevc_ue(b);
			if (sp->slice_type == 0)
				sp->num_ref_idx_l1_active_minus1 = hevc_ue(b);
		}
		if (sp->num_ref_idx_l0_active_minus1 >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX ||
		    sp->num_ref_idx_l1_active_minus1 >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
			return -EINVAL;
		total = dp->num_poc_st_curr_before + dp->num_poc_st_curr_after + dp->num_poc_lt_curr;
		if ((pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT) && total > 1) {
			for (l = 0; l < (sp->slice_type == 0 ? 2U : 1U); l++) {
				n = (l ? sp->num_ref_idx_l1_active_minus1 :
				     sp->num_ref_idx_l0_active_minus1) + 1;
				mod[l] = hevc_u(b, 1);
				for (i = 0; mod[l] && i < n; i++)
					list_entry[l][i] = hevc_u(b, hevc_ceil_log2(total));
			}
		}
		if (sp->slice_type == 0 && hevc_u(b, 1))
			sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO;
		if ((pps->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT) && hevc_u(b, 1))
			sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT;
		if (sp->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED) {
			bool from_l0 = sp->slice_type != 0 || hevc_u(b, 1);

			if (from_l0)
				sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;
			if ((from_l0 && sp->num_ref_idx_l0_active_minus1) ||
			    (!from_l0 && sp->num_ref_idx_l1_active_minus1))
				sp->collocated_ref_idx = hevc_ue(b);
		}
		if (((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) && sp->slice_type == 1) ||
		    ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) && sp->slice_type == 0))
			hevc_parse_pwt(b, sps, sp);
		sp->five_minus_max_num_merge_cand = hevc_ue(b);

		/* reference picture lists (8.3.4), as DPB indices */
		for (l = 0; l < (sp->slice_type == 0 ? 2U : 1U) && total; l++) {
			u32 rps_n = max_t(u32, total, (l ? sp->num_ref_idx_l1_active_minus1 :
						      sp->num_ref_idx_l0_active_minus1) + 1);
			const u8 *first_set = l ? dp->poc_st_curr_after : dp->poc_st_curr_before;
			const u8 *second_set = l ? dp->poc_st_curr_before : dp->poc_st_curr_after;
			u32 nfirst = l ? dp->num_poc_st_curr_after : dp->num_poc_st_curr_before;
			u32 nsecond = l ? dp->num_poc_st_curr_before : dp->num_poc_st_curr_after;
			u8 *out = l ? sp->ref_idx_l1 : sp->ref_idx_l0;

			rps_n = min_t(u32, rps_n, V4L2_HEVC_DPB_ENTRIES_NUM_MAX);
			for (k = 0; k < rps_n;) {
				for (i = 0; i < nfirst && k < rps_n; i++)
					temp[l][k++] = first_set[i];
				for (i = 0; i < nsecond && k < rps_n; i++)
					temp[l][k++] = second_set[i];
				for (i = 0; i < dp->num_poc_lt_curr && k < rps_n; i++)
					temp[l][k++] = dp->poc_lt_curr[i];
			}
			n = (l ? sp->num_ref_idx_l1_active_minus1 : sp->num_ref_idx_l0_active_minus1) + 1;
			for (i = 0; i < n; i++)
				out[i] = temp[l][mod[l] ? min_t(u32, list_entry[l][i], rps_n - 1) : i];
		}
	}
	sp->slice_qp_delta = hevc_se(b);
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) {
		sp->slice_cb_qp_offset = hevc_se(b);
		sp->slice_cr_qp_offset = hevc_se(b);
	}
	dbk_disabled = pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;
	sp->slice_beta_offset_div2 = pps->pps_beta_offset_div2;
	sp->slice_tc_offset_div2 = pps->pps_tc_offset_div2;
	if ((pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) &&
	    hevc_u(b, 1)) {				/* deblocking_filter_override_flag */
		dbk_disabled = hevc_u(b, 1);
		if (!dbk_disabled) {
			sp->slice_beta_offset_div2 = hevc_se(b);
			sp->slice_tc_offset_div2 = hevc_se(b);
		}
	}
	if (dbk_disabled)
		sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) {
		if (!(sao || !dbk_disabled) || hevc_u(b, 1))
			sp->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;
	}

entry_points:
	sp->num_entry_point_offsets = 0;
	if (pps->flags & (V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
			  V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) {
		n = hevc_ue(b);
		sp->num_entry_point_offsets = n;
		if (n) {
			u32 len = hevc_ue(b) + 1;	/* offset_len_minus1 */

			if (len > 32)
				return -EINVAL;
			for (i = 0; i < n && !b->error; i++)
				hevc_u(b, len);
		}
	}
	if (pps->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT) {
		n = hevc_ue(b);
		for (i = 0; i < n && !b->error; i++)
			hevc_u(b, 8);
	}
	/* byte_alignment(): alignment_bit_equal_to_one, then zeros */
	hevc_u(b, 1);
	if (b->pos & 7)
		hevc_u(b, 8 - (b->pos & 7));
	if (b->error)
		return -EINVAL;
	return b->pos >> 3;
}

/*
 * Frame-based request without slice parameters: copy every VCL NAL of the
 * OUTPUT buffer as RBSP and parse its slice segment header into h->fsp.
 */
static int hevc_frame_slices(struct hivdec_ctx *ctx, const struct v4l2_ctrl_hevc_sps *sps,
			     const struct v4l2_ctrl_hevc_pps *pps,
			     const struct v4l2_ctrl_hevc_decode_params *dp, u32 nctbs,
			     const u8 *data, u32 size, u32 *count, u32 *used)
{
	struct hivdec_hevc_ctx *h = ctx->priv;
	const struct v4l2_ctrl_hevc_slice_params *indep = NULL;
	u32 i, start, end, n = 0, dummy;
	int ret;

	for (i = 0; i + 2 < size; i++) {
		struct hevc_bits b = { };
		u32 rbsp;

		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		start = i + 3;
		for (end = start; end + 2 < size; end++)
			if (!data[end] && !data[end + 1] && (data[end + 2] == 1 || !data[end + 2]))
				break;
		if (end + 2 >= size)
			end = size;
		i = end - 1;
		if (end - start < 3 || ((data[start] >> 1) & 0x3f) > 31)
			continue;			/* not a VCL NAL */
		if (n == HEVC_MAX_SLICES || *used + (end - start) + 64 > h->stream.size)
			return -ENOSPC;
		h->slices[n].offset = *used;
		rbsp = hevc_nal_to_rbsp(h->stream.cpu + *used, data + start, end - start, 0, &dummy);
		b.p = h->stream.cpu + *used;
		b.len = rbsp;
		ret = hevc_parse_slice(&b, sps, pps, dp, nctbs, indep, &h->fsp[n]);
		if (ret < 0 || ret >= rbsp) {
			if (ctx->dev->debug)
				dev_info(ctx->dev->dev, "hevc: slice %u header parse %d\n", n, ret);
			return -EINVAL;
		}
		if (!(h->fsp[n].flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
			indep = &h->fsp[n];
		h->fsp[n].data_byte_offset = ret;
		h->fsp[n].bit_size = rbsp * 8;
		h->slices[n].data_byte = ret;
		h->slices[n].bits = hevc_payload_bits(h->stream.cpu + *used + ret, rbsp - ret);
		if (ctx->dev->debug > 1)
			dev_info(ctx->dev->dev, "  slice %u: nal %u..%u rbsp %u data %u bits %u addr %u type %u (parsed)\n",
				 n, start, end, rbsp, ret, h->slices[n].bits,
				 h->fsp[n].slice_segment_addr, h->fsp[n].slice_type);
		memset(h->stream.cpu + *used + rbsp, 0, 32);
		*used = ALIGN(*used + rbsp + 32, 64);
		n++;
	}
	*count = n;
	return n ? 0 : -EINVAL;
}

/* ---------------------------------------------------------------- decode */

struct hevc_pic {
	u32 wctb, hctb, nctbs, log2_ctb;
	u32 w64, ah, luma, mv;
	int dpb_idx[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
};

static u32 hevc_slot_of(struct hivdec_hevc_ctx *h, const struct hevc_pic *pic, u8 dpb_i)
{
	int b = dpb_i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX ? pic->dpb_idx[dpb_i] : -1;

	return b >= 0 && h->meta[b].apc >= 0 ? h->meta[b].apc : 0;
}

static void hevc_slice_msg(struct hivdec_ctx *ctx, const struct hevc_pic *pic,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_ctrl_hevc_pps *pps,
			   const struct v4l2_ctrl_hevc_decode_params *dp,
			   const struct v4l2_ctrl_hevc_slice_params *sp,
			   const struct v4l2_ctrl_hevc_slice_params *indep,
			   const struct hivdec_hevc_slice *s, u32 *msg, u32 next,
			   u32 end_rs, u32 end_ts, u32 stream_base)
{
	struct hivdec_hevc_ctx *h = ctx->priv;
	u8 t = indep->slice_type;
	u32 nl0 = t != HEVC_I ? indep->num_ref_idx_l0_active_minus1 + 1 : 0;
	u32 nl1 = t == HEVC_B ? indep->num_ref_idx_l1_active_minus1 + 1 : 0;
	u32 addr, i, v, col = 0, low_delay = 1, mv = 0;
	bool tmvp = indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
	bool col_l0 = indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;

	memset(msg, 0, HIVDEC_MSG_SLOT_BYTES);

	addr = h->stream.map.iova + s->offset + s->data_byte;
	msg[0] = ((addr & ~15) - (stream_base & ~15)) >> 4;
	msg[1] = (addr & 15) * 8;
	msg[2] = s->bits;

	for (i = 0; i < nl0; i++)
		if (indep->ref_idx_l0[i] < 16 &&
		    dp->dpb[indep->ref_idx_l0[i]].pic_order_cnt_val > dp->pic_order_cnt_val)
			low_delay = 0;
	for (i = 0; i < nl1; i++)
		if (indep->ref_idx_l1[i] < 16 &&
		    dp->dpb[indep->ref_idx_l1[i]].pic_order_cnt_val > dp->pic_order_cnt_val)
			low_delay = 0;
	if (t == HEVC_B && !col_l0)
		col = hevc_slot_of(h, pic, indep->ref_idx_l1[indep->collocated_ref_idx]);
	else if (t != HEVC_I)
		col = hevc_slot_of(h, pic, indep->ref_idx_l0[indep->collocated_ref_idx]);

	msg[6] = indep->slice_segment_addr |
		 !!(indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT) << 18 |
		 ((26 + pps->init_qp_minus26 + indep->slice_qp_delta) & 0x7f) << 19 |
		 !!(sp->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT) << 26 |
		 low_delay << 27;
	msg[7] = (t == HEVC_B ? 2 : t == HEVC_P ? 1 : 0) | nl0 << 2 | nl1 << 8 |
		 !!(indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO) << 14 |
		 tmvp << 15 | (nl0 ? nl0 - 1 : 0) << 16 | (nl1 ? nl1 - 1 : 0) << 20 |
		 (5 - indep->five_minus_max_num_merge_cand) << 24 |
		 (u32)(t != HEVC_I && (col_l0 || t == HEVC_P)) << 27 | col << 28;
	msg[8] = sp->slice_segment_addr;
	msg[9] = (sp->slice_segment_addr % pic->wctb) | (sp->slice_segment_addr / pic->wctb) << 16;
	msg[10] = h->rs2ts[sp->slice_segment_addr];
	msg[11] = (indep->slice_cr_qp_offset & 31) | (indep->slice_cb_qp_offset & 31) << 8;
	msg[12] = (u32)!!(indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED) |
		  !!(indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED) << 1 |
		  !!(indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA) << 2 |
		  !!(indep->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA) << 3 |
		  (indep->slice_beta_offset_div2 & 15) << 8 |
		  (indep->slice_tc_offset_div2 & 15) << 16;
	for (i = 0, v = 0; i < nl0; i++)
		if (indep->ref_idx_l0[i] < 16 &&
		    dp->dpb[indep->ref_idx_l0[i]].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			v |= BIT(i);
	msg[13] = v;
	for (i = 0, v = 0; i < nl1; i++)
		if (indep->ref_idx_l1[i] < 16 &&
		    dp->dpb[indep->ref_idx_l1[i]].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			v |= BIT(i);
	msg[14] = v;
	for (i = 0; i < nl0; i++)
		msg[15 + i / 8] |= hevc_slot_of(h, pic, indep->ref_idx_l0[i]) << (4 * (i % 8));
	for (i = 0; i < nl1; i++)
		msg[17 + i / 8] |= hevc_slot_of(h, pic, indep->ref_idx_l1[i]) << (4 * (i % 8));

	/* co-located motion vectors */
	if (tmvp && t != HEVC_I) {
		struct hivdec_decoded_buffer *cb = hevc_buf(ctx, h->apc[col]);

		if (cb && cb->mv.priv)
			mv = cb->mv.map.iova;
	}
	msg[23] = (mv ? mv : pic->mv) >> 4;
	/* Kirin: words 24..42, 45..49 (STB multi-layer extension) are zero */
	msg[43] = end_rs;
	msg[44] = end_ts;
	msg[63] = next;

	/* weighted prediction */
	if ((t == HEVC_P && (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED)) ||
	    (t == HEVC_B && (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED))) {
		const struct v4l2_hevc_pred_weight_table *w = &indep->pred_weight_table;
		u32 ld = w->luma_log2_weight_denom;
		u32 cd = ld + w->delta_chroma_log2_weight_denom;
		int L, c;

		for (L = 0; L < (t == HEVC_B ? 2 : 1); L++) {
			const s8 *dl = L ? w->delta_luma_weight_l1 : w->delta_luma_weight_l0;
			const s8 *lo = L ? w->luma_offset_l1 : w->luma_offset_l0;
			const s8 (*dc)[2] = L ? w->delta_chroma_weight_l1 : w->delta_chroma_weight_l0;
			const s8 (*co)[2] = L ? w->chroma_offset_l1 : w->chroma_offset_l0;
			u32 n = L ? nl1 : nl0;

			for (i = 0; i < n; i++) {
				s32 cw[2], cof[2];

				/* chroma_offset_lX is the final ChromaOffsetLX (7-56) */
				for (c = 0; c < 2; c++) {
					cw[c] = (1 << cd) + dc[i][c];
					cof[c] = co[i][c];
				}
				msg[64 + 16 * L + i] = (ld & 7) |
					(((1 << ld) + dl[i]) & 0x1ff) << 3 | (u32)(u8)lo[i] << 12;
				msg[128 + 16 * L + i] = (cd & 7) | (cw[0] & 0x1ff) << 3 |
					(u32)(u8)cof[0] << 12;
				msg[160 + 16 * L + i] = (cw[1] & 0x1ff) | (u32)(u8)cof[1] << 9;
			}
		}
	}
}

static int hivdec_hevc_run(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_hevc_ctx *h = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct hivdec_decoded_buffer *dbuf = vb2_to_hivdec_buf(&dst->vb2_buf);
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	const struct v4l2_ctrl_hevc_decode_params *dp;
	const struct v4l2_ctrl_hevc_scaling_matrix *sm;
	const struct v4l2_ctrl_hevc_slice_params *sps_arr, *indep = NULL;
	struct v4l2_ctrl *sctrl;
	struct hevc_pic pic = { };
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0);
	u32 *msg, *head, i, k, n, used = 0, ystride, cfg0, cfg1, mvsize, pitch, fmt_h;
	u32 log2_min_cb, log2_min_tb, max_cu_depth, slot, nsc;
	bool abs_off;
	int ret = 0;

	if (req)
		v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	sps = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_HEVC_SPS);
	pps = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_HEVC_PPS);
	dp = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	sm = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX);
	sctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_SLICE_PARAMS);
	sps_arr = sctrl->p_cur.p;
	n = sctrl->elems;
	if (!data || !n || n > HEVC_MAX_SLICES) {
		if (vdec->debug)
			dev_info(vdec->dev, "hevc: %u slice params, %u bytes\n", n, size);
		ret = -EINVAL;
		goto out;
	}

	log2_min_cb = sps->log2_min_luma_coding_block_size_minus3 + 3;
	pic.log2_ctb = log2_min_cb + sps->log2_diff_max_min_luma_coding_block_size;
	pic.wctb = DIV_ROUND_UP(sps->pic_width_in_luma_samples, 1 << pic.log2_ctb);
	pic.hctb = DIV_ROUND_UP(sps->pic_height_in_luma_samples, 1 << pic.log2_ctb);
	pic.nctbs = pic.wctb * pic.hctb;
	pic.w64 = ALIGN(sps->pic_width_in_luma_samples, 64);
	pic.ah = ALIGN(sps->pic_height_in_luma_samples, 64);
	/* 8-bit: NV12; 10-bit: 16-bit samples (P010) */
	pitch = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	fmt_h = ctx->decoded_fmt.fmt.pix_mp.height;
	ystride = pitch * 8;
	if ((sps->bit_depth_luma_minus8 ? 2 : 1) != hivdec_decoded_bps(ctx) ||
	    pic.w64 * hivdec_decoded_bps(ctx) > pitch || pic.ah > fmt_h ||
	    pitch * fmt_h * 3 / 2 > vb2_plane_size(&dst->vb2_buf, 0)) {
		dev_err(vdec->dev, "capture buffer too small for %ux%u\n",
			sps->pic_width_in_luma_samples, sps->pic_height_in_luma_samples);
		ret = -EINVAL;
		goto out;
	}
	ret = hevc_setup_tiles(h, pps, pic.wctb, pic.hctb, pic.log2_ctb);
	if (ret)
		goto out;

	/* co-located MV buffer: 16 bytes per 16x16 block */
	mvsize = ALIGN(ALIGN(DIV_ROUND_UP(sps->pic_height_in_luma_samples, 16), 4) *
		       DIV_ROUND_UP(sps->pic_width_in_luma_samples, 16) * 16, 128);
	if (!dbuf->mv.priv || dbuf->mv.size < mvsize) {
		hivdec_aux_free(vdec, &dbuf->mv);
		ret = hivdec_aux_alloc(vdec, &dbuf->mv, mvsize);
		if (ret)
			goto out;
	}
	pic.luma = dbuf->map.iova;
	pic.mv = dbuf->mv.map.iova;

	/* Chromium sends no slice parameters: parse the slice headers here */
	if (n == 1 && !sps_arr[0].bit_size && !sps_arr[0].data_byte_offset) {
		ret = hevc_frame_slices(ctx, sps, pps, dp, pic.nctbs, data, size, &n, &used);
		if (ret)
			goto out;
		sps_arr = h->fsp;
	} else {
		/*
		 * slices: data_byte_offset locates slice_segment_data() in RBSP terms
		 * (emulation prevention bytes of the header not counted). GStreamer
		 * counts it from the slice's own start code, libva-v4l2-request from
		 * the start of the OUTPUT buffer; the latter is recognised by every
		 * offset falling inside its own slice. Each slice NAL is copied as
		 * RBSP (the VDH does not drop emulation prevention bytes).
		 */
		nsc = 0;
		for (i = 0; i + 2 < size && nsc < ARRAY_SIZE(h->sc); i++) {
			if (!data[i] && !data[i + 1] && data[i + 2] == 1) {
				h->sc[nsc++] = i;
				i += 2;
			}
		}
		if (nsc < n) {
			ret = -EINVAL;
			goto out;
		}
		abs_off = true;
		for (k = 0; k < n && abs_off; k++) {
			u32 hi = k + 1 < nsc ? h->sc[k + 1] : size;

			if (sps_arr[k].data_byte_offset <= h->sc[k] + 3 ||
			    sps_arr[k].data_byte_offset >= hi)
				abs_off = false;
		}
		for (k = 0; k < n; k++) {
			const struct v4l2_ctrl_hevc_slice_params *sp = &sps_arr[k];
			u32 nal = h->sc[k] + 3, end = k + 1 < nsc ? h->sc[k + 1] : size;
			u32 roff = abs_off ? sp->data_byte_offset - nal : sp->data_byte_offset - 3;
			u32 rbsp, dummy;

			if (!abs_off && sp->data_byte_offset < 3) {
				ret = -EINVAL;
				goto out;
			}
			if (used + (end - nal) + 64 > h->stream.size) {
				ret = -ENOSPC;
				goto out;
			}
			h->slices[k].offset = used;
			rbsp = hevc_nal_to_rbsp(h->stream.cpu + used, data + nal, end - nal, 0, &dummy);
			if (roff >= rbsp) {
				ret = -EINVAL;
				goto out;
			}
			h->slices[k].data_byte = roff;
			h->slices[k].bits = hevc_payload_bits(h->stream.cpu + used + roff, rbsp - roff);
			if (vdec->debug > 1)
				dev_info(vdec->dev, "  slice %u: nal %u..%u rbsp %u data %u bits %u addr %u%s\n",
					 k, nal, end, rbsp, roff, h->slices[k].bits,
					 sp->slice_segment_addr, abs_off ? "" : " (rel)");
			memset(h->stream.cpu + used + rbsp, 0, 32);
			used = ALIGN(used + rbsp + 32, 64);
		}
	}

	/* DPB -> capture buffers -> stable slots */
	for (i = 0; i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++)
		pic.dpb_idx[i] = i < dp->num_active_dpb_entries ?
				 hevc_find_buf(ctx, dp->dpb[i].timestamp) : -1;
	hevc_update_apc(h, pic.dpb_idx, V4L2_HEVC_DPB_ENTRIES_NUM_MAX);

	/* picture message */
	log2_min_tb = sps->log2_min_luma_transform_block_size_minus2 + 2;
	max_cu_depth = sps->log2_diff_max_min_luma_coding_block_size +
		       (log2_min_cb > log2_min_tb ? log2_min_cb - log2_min_tb : 0);
	msg = hivdec_msg_slot(ctx, HIVDEC_SLOT_PIC);
	memset(msg, 0, HIVDEC_MSG_SLOT_BYTES);
	msg[0] = (pic.wctb - 1) | (pic.hctb - 1) << 9 |
		 !!(sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) << 18 |
		 sps->chroma_format_idc << 19 |
		 !!(sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED) << 21 |
		 !!(sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) << 22 |
		 !!(sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED) << 23 |
		 !!(sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) << 24 |
		 !!(sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) << 25;
	msg[1] = dp->pic_order_cnt_val;
	msg[2] = pic.log2_ctb | log2_min_cb << 3 | max_cu_depth << 6 |
		 (log2_min_tb + sps->log2_diff_max_min_luma_transform_block_size) << 9 |
		 log2_min_tb << 12;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		u32 pmin = sps->log2_min_pcm_luma_coding_block_size_minus3 + 3;

		msg[2] |= (pmin + sps->log2_diff_max_min_pcm_luma_coding_block_size) << 15 |
			  pmin << 18;
	}
	msg[2] |= sps->max_transform_hierarchy_depth_intra << 21 |
		  sps->max_transform_hierarchy_depth_inter << 24 |
		  !!(pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) << 27 |
		  !!(pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) << 28 |
		  (u32)pps->log2_parallel_merge_level_minus2 << 29;
	msg[3] = (sps->bit_depth_luma_minus8 + 8) | (sps->bit_depth_chroma_minus8 + 8) << 4 |
		 (sps->pcm_sample_bit_depth_luma_minus1 + 1) << 8 |
		 (sps->pcm_sample_bit_depth_chroma_minus1 + 1) << 12 |
		 (6 * sps->bit_depth_luma_minus8) << 16 | (6 * sps->bit_depth_chroma_minus8) << 22;
	msg[4] = h->sed_top >> 4;
	msg[5] = h->pmv_top >> 4;
	msg[6] = pic.mv >> 4;
	msg[7] = h->rcn_top >> 4;
	for (i = 0; i < HEVC_APC_SLOTS; i++) {
		struct hivdec_decoded_buffer *r = hevc_buf(ctx, h->apc[i]);

		msg[8 + i] = (r ? r->map.iova : pic.luma) >> 4;
	}
	msg[25] = h->cabac >> 4;
	for (i = 0; i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++) {
		int b = pic.dpb_idx[i];

		if (b >= 0 && h->meta[b].apc >= 0)
			msg[26 + h->meta[b].apc] = dp->dpb[i].pic_order_cnt_val;
	}
	msg[43] = h->pmv_left >> 4;
	msg[54] = sps->pic_width_in_luma_samples | sps->pic_height_in_luma_samples << 16;
	msg[55] = h->tile_info >> 4;
	msg[56] = h->sao_left >> 4;
	msg[57] = h->dblk_left >> 4;
	msg[58] = h->sao_top >> 4;
	msg[59] = h->dblk_top >> 4;
	{
		u32 log2_minqg = pic.log2_ctb - ((pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) ?
					       pps->diff_cu_qp_delta_depth : 0);
		bool tiles = pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED;

		msg[60] = (pps->pps_cr_qp_offset & 31) | (pps->pps_cb_qp_offset & 31) << 5 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 10 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED) << 11 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) << 12 |
			  ((pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) ?
			   pps->diff_cu_qp_delta_depth : 0) << 13 |
			  log2_minqg << 16 | tiles << 19 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING) << 20 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) << 21 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED) << 22 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED) << 23 |
			  !!(pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED) << 24;
		msg[61] = tiles ? pps->num_tile_columns_minus1 | pps->num_tile_rows_minus1 << 16 : 0;
	}
	msg[63] = hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_SLICE0);
	if (sm && (sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED))
		hevc_build_qmatrix(sm, &msg[64]);

	/* head message: dummy compression header buffers */
	head = hivdec_msg_slot(ctx, HIVDEC_SLOT_HEAD);
	memset(head, 0, HIVDEC_MSG_SLOT_BYTES);
	head[0] = h->head >> 4;
	head[1] = (h->head + HEVC_HEAD_LEN / 2) >> 4;
	head[2] = ALIGN(pic.w64 / 16, 32);
	for (i = 0; i < 16; i++) {
		head[3 + 2 * i] = head[0];
		head[4 + 2 * i] = head[1];
	}

	/* slice messages */
	slot = HIVDEC_SLOT_SLICE0;
	for (k = 0; k < n; k++) {
		const struct v4l2_ctrl_hevc_slice_params *sp = &sps_arr[k];
		u32 start_ts = h->rs2ts[min(sp->slice_segment_addr, pic.nctbs - 1)];
		u32 end_ts, end_rs, next;

		if (!(sp->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
			indep = sp;
		if (!indep || (k == 0 && start_ts)) {
			ret = -EINVAL;	/* missing first slice */
			goto out;
		}
		if (k + 1 < n) {
			end_ts = h->rs2ts[sps_arr[k + 1].slice_segment_addr] - 1;
			end_rs = (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) ?
				 h->ts2rs[end_ts] : end_ts;
			next = hivdec_msg_slot_addr(ctx, slot + 1);
		} else {
			end_ts = end_rs = pic.nctbs - 1;
			next = 0;
		}
		hevc_slice_msg(ctx, &pic, sps, pps, dp, sp, indep, &h->slices[k],
			       hivdec_msg_slot(ctx, slot), next, end_rs, end_ts,
			       h->stream.map.iova);
		slot++;
	}

	/* registers */
	cfg0 = CFG0_MBAMT_TO_DEC(pic.nctbs - 1) | CFG0_COEF_IDX_DETECT |
	       ((sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) ? CFG0_LOAD_QMATRIX : 0);
	cfg1 = CFG1_VIDEO_STANDARD(HIVDEC_STD_HEVC) | CFG1_MFD_MMU_EN | CFG1_UV_ORDER_EN |
	       CFG1_FST_SLC_GRP | CFG1_MV_OUTPUT_EN | CFG1_MAX_SLCGRP_NUM(3) | CFG1_VDH_2D_EN;
	vdh_write(vdec, VDH_BASIC_CFG0, cfg0);
	vdh_write(vdec, VDH_BASIC_CFG1, cfg1);
	vdh_write(vdec, VDH_AVM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_PIC));
	vdh_write(vdec, VDH_VAM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_UP));
	vdh_write(vdec, VDH_STREAM_BASE_ADDR, (h->stream.map.iova & ~15) >> 4);
	vdh_write(vdec, VDH_SED_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_ITRANS_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PMV_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PRC_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_RCN_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_DBLK_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PPFD_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_YSTADDR_1D, (pic.luma & ~15) >> 4);
	vdh_write(vdec, VDH_YSTRIDE_1D, ystride);
	vdh_write(vdec, VDH_UVOFFSET_1D, pitch * fmt_h);
	vdh_write(vdec, VDH_HEAD_INF_OFFSET, 0);
	vdh_write(vdec, VDH_YSTRIDE_2BIT, 0);
	vdh_write(vdec, VDH_YOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_UVOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_REF_PIC_TYPE, 0);
	vdh_write(vdec, VDH_FF_APT_EN, 2);
	vdh_write(vdec, VDH_UVSTRIDE_1D, (ystride / 2) & 0x3ffff);
	vdh_write(vdec, VDH_CFGINFO_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_HEAD));
	vdh_write(vdec, VDH_DDR_INTERLEAVE_MODE, 3);
	vdh_write(vdec, SCD_EMAR_ID, 0x101);
	vdh_write(vdec, SCD_AVS_FLAG, 0);
	vdh_write(vdec, SCD_VDH_SELRST, 1);

	if (vdec->debug)
		dev_info(vdec->dev, "hevc %ux%u ctb%u poc %d slices %u dpb %u\n",
			 sps->pic_width_in_luma_samples, sps->pic_height_in_luma_samples,
			 1 << pic.log2_ctb, dp->pic_order_cnt_val, n, dp->num_active_dpb_entries);

	hivdec_aux_sync_for_device(vdec, &h->work);
	hivdec_aux_sync_for_device(vdec, &h->stream);
out:
	if (req)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	if (ret)
		return ret;
	hivdec_hw_run(ctx);
	return 0;
}

const struct hivdec_coded_fmt_ops hivdec_hevc_fmt_ops = {
	.adjust_fmt = hivdec_hevc_adjust_fmt,
	.start = hivdec_hevc_start,
	.stop = hivdec_hevc_stop,
	.run = hivdec_hevc_run,
	.try_ctrl = hivdec_hevc_try_ctrl,
	.bit_depth = hivdec_hevc_bit_depth,
};
