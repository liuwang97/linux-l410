// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video decoder: VP9 (profile 0)
 *
 * One request = one frame. Software keeps the four probability contexts and
 * applies the forward updates of the compressed header (v4l2-vp9 helpers);
 * the VDH decodes the tiles straight from the OUTPUT buffer and performs the
 * backward adaptation itself: it reads the pre-update context from the "count"
 * buffer and writes the adapted probabilities back into it. Message layout of
 * the HiSilicon VFMW VP9 HAL as used on Kirin (picture message in slot 5, tile
 * messages 256 bytes apart from slot 6, compression head message in slot 4).
 */

#include <linux/string.h>
#include <linux/unaligned.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-vp9.h>
#include <media/videobuf2-dma-sg.h>

#include "hisi_vdec.h"

#define VP9_TILE_MSG_BYTES	256
#define VP9_MAX_TILES		((HIVDEC_MSG_SLOTS - HIVDEC_SLOT_SLICE0) * \
				 HIVDEC_MSG_SLOT_BYTES / VP9_TILE_MSG_BYTES)
#define VP9_NO_REF		15

/*
 * Motion compensation of the VDH reads up to ~6 pixels past the right and
 * bottom edges of a reference frame instead of clamping, so every decoded
 * frame gets its edges replicated into this much padding.
 */
#define VP9_PAD			16

/* line buffers, vendor sizes for 4096 x 2304 */
#define VP9_TOP_LEN		SZ_1M
#define VP9_DBLK_LEFT_LEN	(130 * SZ_1K)
#define VP9_HEAD_LEN		SZ_256K
#define VP9_MV_LEN		0x240000	/* 64 bytes per 16x16 block */
#define VP9_SEGMAP_LEN		SZ_256K
#define VP9_SEGMAP_STRIDE	2048

/* probability table: forward-updated context in, pre-update context in / adapted out */
#define VP9_PROB_LEN		4608
#define VP9_PROB_TAB		0
#define VP9_PROB_CNT		SZ_8K

/* probability table layout (bytes) */
#define HWP_COEF		0x0000	/* [tx4][plane2][ref2][band8][ctx8][node4] */
#define HWP_YMODE		0x1000	/* [4][16] */
#define HWP_UVMODE		0x1040	/* [10][16] */
#define HWP_PART		0x1100	/* [16][3], key frame probs for intra frames */
#define HWP_SKIP		0x1130
#define HWP_TX8			0x1133
#define HWP_TX16		0x1135
#define HWP_TX32		0x1139
#define HWP_ISINTER		0x113f
#define HWP_COMPMODE		0x1143
#define HWP_COMPREF		0x1148
#define HWP_SINGLEREF		0x114d
#define HWP_INTERMODE		0x1157
#define HWP_INTERP		0x116c
#define HWP_MV_SIGN		0x1174
#define HWP_MV_JOINT		0x1176
#define HWP_MV_CLASSES		0x1179
#define HWP_MV_CLASS0		0x118d
#define HWP_MV_BITS		0x118f
#define HWP_MV_C0FR		0x11a3
#define HWP_MV_FR		0x11af
#define HWP_MV_C0HP		0x11b5
#define HWP_MV_HP		0x11b7
#define HWP_SEG_TREE		0x11c0
#define HWP_SEG_PRED		0x11c8

struct hivdec_vp9_buf_meta {
	u16 w, h;		/* size the buffer was decoded at */
	s8 id;			/* frame id given to the VDH */
};

struct hivdec_vp9_ctx {
	struct hivdec_aux_buf work;
	struct hivdec_aux_buf prob;
	struct hivdec_aux_buf segmap;
	u32 sed_top, pmv_top, rcn_top, dblk_top, dblk_left, mv, head;	/* IOVAs */

	struct v4l2_vp9_frame_context fc[4];
	struct v4l2_vp9_frame_context cur;	/* context of the frame being decoded */
	u8 fc_idx;
	u32 flags;				/* V4L2_VP9_FRAME_FLAG_* of that frame */
	int dst_idx;
	u16 w, h;

	/* previous decoded frame */
	u8 last_frame_type;
	bool last_show;
	u16 last_w, last_h;

	struct hivdec_vp9_buf_meta meta[VB2_MAX_FRAME];
};

/* ---------------------------------------------------------------- setup */

static int hivdec_vp9_start(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_vp9_ctx *v;
	u32 base, off;
	int i, ret;

	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	ret = hivdec_aux_alloc(vdec, &v->work, 4 * VP9_TOP_LEN + VP9_DBLK_LEFT_LEN +
			       VP9_HEAD_LEN + VP9_MV_LEN);
	if (ret)
		goto err_free;
	ret = hivdec_aux_alloc(vdec, &v->prob, 2 * SZ_8K);
	if (ret)
		goto err_work;
	ret = hivdec_aux_alloc(vdec, &v->segmap, VP9_SEGMAP_LEN);
	if (ret)
		goto err_prob;

	base = v->work.map.iova;
	off = 0;
	v->sed_top = base + off;
	off += VP9_TOP_LEN;
	v->pmv_top = base + off;
	off += VP9_TOP_LEN;
	v->rcn_top = base + off;
	off += VP9_TOP_LEN;
	v->dblk_top = base + off;
	off += VP9_TOP_LEN;
	v->mv = base + off;
	off += VP9_MV_LEN;
	v->head = base + off;
	off += VP9_HEAD_LEN;
	v->dblk_left = base + off;
	hivdec_aux_sync_for_device(vdec, &v->work);
	hivdec_aux_sync_for_device(vdec, &v->segmap);

	for (i = 0; i < 4; i++)
		v->fc[i] = v4l2_vp9_default_probs;
	for (i = 0; i < VB2_MAX_FRAME; i++)
		v->meta[i].id = -1;
	v->last_frame_type = 3;
	v->dst_idx = -1;
	ctx->priv = v;
	return 0;

err_prob:
	hivdec_aux_free(vdec, &v->prob);
err_work:
	hivdec_aux_free(vdec, &v->work);
err_free:
	kfree(v);
	return ret;
}

static void hivdec_vp9_stop(struct hivdec_ctx *ctx)
{
	struct hivdec_vp9_ctx *v = ctx->priv;

	if (!v)
		return;
	hivdec_aux_free(ctx->dev, &v->segmap);
	hivdec_aux_free(ctx->dev, &v->prob);
	hivdec_aux_free(ctx->dev, &v->work);
	kfree(v);
	ctx->priv = NULL;
}

static int hivdec_vp9_adjust_fmt(struct hivdec_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 3 / 2;
	fmt->plane_fmt[0].sizeimage = max_t(u32, fmt->plane_fmt[0].sizeimage, SZ_1M);
	return 0;
}

static int hivdec_vp9_try_ctrl(struct hivdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_VP9_FRAME) {
		const struct v4l2_ctrl_vp9_frame *f = ctrl->p_new.p_vp9_frame;

		if (f->profile != 0 || f->bit_depth != 8)
			return -EINVAL;
		if (f->frame_width_minus_1 >= 4096 || f->frame_height_minus_1 >= 2304)
			return -EINVAL;
	}
	return 0;
}

/* ---------------------------------------------------------------- probabilities */

static void vp9_probs_to_hw(u8 *hw, const struct v4l2_vp9_frame_context *fc,
			    const struct v4l2_ctrl_vp9_frame *f)
{
	bool intra = f->flags & (V4L2_VP9_FRAME_FLAG_KEY_FRAME | V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	int t, i, j, b, c;

	memset(hw, 0, VP9_PROB_LEN);
	for (t = 0; t < 4; t++)
		for (i = 0; i < 2; i++)
			for (j = 0; j < 2; j++)
				for (b = 0; b < 6; b++)
					for (c = 0; c < (b ? 6 : 3); c++)
						memcpy(hw + HWP_COEF +
						       ((((t * 2 + i) * 2 + j) * 8 + b) * 8 + c) * 4,
						       fc->coef[t][i][j][b][c], 3);
	for (i = 0; i < 4; i++)
		memcpy(hw + HWP_YMODE + 16 * i, fc->y_mode[i], 9);
	for (i = 0; i < 10; i++)
		memcpy(hw + HWP_UVMODE + 16 * i, fc->uv_mode[i], 9);
	memcpy(hw + HWP_PART, intra ? v4l2_vp9_kf_partition_probs : fc->partition, 48);
	memcpy(hw + HWP_SKIP, fc->skip, 3);
	memcpy(hw + HWP_TX8, fc->tx8, 2);
	memcpy(hw + HWP_TX16, fc->tx16, 4);
	memcpy(hw + HWP_TX32, fc->tx32, 6);
	memcpy(hw + HWP_ISINTER, fc->is_inter, 4);
	memcpy(hw + HWP_COMPMODE, fc->comp_mode, 5);
	memcpy(hw + HWP_COMPREF, fc->comp_ref, 5);
	memcpy(hw + HWP_SINGLEREF, fc->single_ref, 10);
	memcpy(hw + HWP_INTERMODE, fc->inter_mode, 21);
	memcpy(hw + HWP_INTERP, fc->interp_filter, 8);
	memcpy(hw + HWP_MV_SIGN, fc->mv.sign, 2);
	memcpy(hw + HWP_MV_JOINT, fc->mv.joint, 3);
	memcpy(hw + HWP_MV_CLASSES, fc->mv.classes, 20);
	memcpy(hw + HWP_MV_CLASS0, fc->mv.class0_bit, 2);
	memcpy(hw + HWP_MV_BITS, fc->mv.bits, 20);
	memcpy(hw + HWP_MV_C0FR, fc->mv.class0_fr, 12);
	memcpy(hw + HWP_MV_FR, fc->mv.fr, 6);
	memcpy(hw + HWP_MV_C0HP, fc->mv.class0_hp, 2);
	memcpy(hw + HWP_MV_HP, fc->mv.hp, 2);
	memcpy(hw + HWP_SEG_TREE, f->seg.tree_probs, 7);
	memcpy(hw + HWP_SEG_PRED, f->seg.pred_probs, 3);
}

/*
 * Adapted probabilities. As in libvpx only the coefficient probabilities
 * adapt on intra frames; the partition probabilities of an intra frame are
 * the fixed key frame ones and never go back into the context.
 */
static void vp9_probs_from_hw(struct v4l2_vp9_frame_context *fc, const u8 *hw, bool intra)
{
	int t, i, j, b, c;

	for (t = 0; t < 4; t++)
		for (i = 0; i < 2; i++)
			for (j = 0; j < 2; j++)
				for (b = 0; b < 6; b++)
					for (c = 0; c < (b ? 6 : 3); c++)
						memcpy(fc->coef[t][i][j][b][c], hw + HWP_COEF +
						       ((((t * 2 + i) * 2 + j) * 8 + b) * 8 + c) * 4, 3);
	if (intra)
		return;
	for (i = 0; i < 4; i++)
		memcpy(fc->y_mode[i], hw + HWP_YMODE + 16 * i, 9);
	for (i = 0; i < 10; i++)
		memcpy(fc->uv_mode[i], hw + HWP_UVMODE + 16 * i, 9);
	memcpy(fc->partition, hw + HWP_PART, 48);
	memcpy(fc->skip, hw + HWP_SKIP, 3);
	memcpy(fc->tx8, hw + HWP_TX8, 2);
	memcpy(fc->tx16, hw + HWP_TX16, 4);
	memcpy(fc->tx32, hw + HWP_TX32, 6);
	memcpy(fc->is_inter, hw + HWP_ISINTER, 4);
	memcpy(fc->comp_mode, hw + HWP_COMPMODE, 5);
	memcpy(fc->comp_ref, hw + HWP_COMPREF, 5);
	memcpy(fc->single_ref, hw + HWP_SINGLEREF, 10);
	memcpy(fc->inter_mode, hw + HWP_INTERMODE, 21);
	memcpy(fc->interp_filter, hw + HWP_INTERP, 8);
	memcpy(fc->mv.sign, hw + HWP_MV_SIGN, 2);
	memcpy(fc->mv.joint, hw + HWP_MV_JOINT, 3);
	memcpy(fc->mv.classes, hw + HWP_MV_CLASSES, 20);
	memcpy(fc->mv.class0_bit, hw + HWP_MV_CLASS0, 2);
	memcpy(fc->mv.bits, hw + HWP_MV_BITS, 20);
	memcpy(fc->mv.class0_fr, hw + HWP_MV_C0FR, 12);
	memcpy(fc->mv.fr, hw + HWP_MV_FR, 6);
	memcpy(fc->mv.class0_hp, hw + HWP_MV_C0HP, 2);
	memcpy(fc->mv.hp, hw + HWP_MV_HP, 2);
}

/* ---------------------------------------------------------------- helpers */

static int vp9_find_buf(struct hivdec_ctx *ctx, u64 ts)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = vb2_find_buffer(q, ts);

	return vb ? vb->index : -1;
}

static struct hivdec_decoded_buffer *vp9_buf(struct hivdec_ctx *ctx, int idx)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = idx >= 0 ? vb2_get_buffer(q, idx) : NULL;

	return vb ? vb2_to_hivdec_buf(vb) : NULL;
}

/* loop filter level per segment, reference and mode (libvpx vp9_loop_filter_frame_init) */
static void vp9_lf_levels(const struct v4l2_ctrl_vp9_frame *f, u8 lvl[8][4][2])
{
	const struct v4l2_vp9_loop_filter *lf = &f->lf;
	const struct v4l2_vp9_segmentation *seg = &f->seg;
	int shift = lf->level >> 5, s, r, m;

	for (s = 0; s < 8; s++) {
		int l = lf->level;

		if ((seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED) &&
		    (seg->feature_enabled[s] & V4L2_VP9_SEGMENT_FEATURE_ENABLED(V4L2_VP9_SEG_LVL_ALT_L))) {
			int d = seg->feature_data[s][V4L2_VP9_SEG_LVL_ALT_L];

			l = (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE) ? d : l + d;
			l = clamp(l, 0, 63);
		}
		if (!(lf->flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED)) {
			memset(lvl[s], l, sizeof(lvl[s]));
			continue;
		}
		lvl[s][0][0] = clamp(l + lf->ref_deltas[0] * (1 << shift), 0, 63);
		lvl[s][0][1] = lvl[s][0][0];
		for (r = 1; r < 4; r++)
			for (m = 0; m < 2; m++)
				lvl[s][r][m] = clamp(l + lf->ref_deltas[r] * (1 << shift) +
						     lf->mode_deltas[m] * (1 << shift), 0, 63);
	}
}

static u32 vp9_abs_sign(s8 v, int shift)
{
	return (u32)(abs(v) & 15) << shift | (u32)(v < 0) << (shift + 4);
}

/* ---------------------------------------------------------------- decode */

static int hivdec_vp9_run(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_vp9_ctx *v = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct hivdec_decoded_buffer *dbuf = vb2_to_hivdec_buf(&dst->vb2_buf);
	struct hivdec_src_buffer *sbuf = vb2_to_hivdec_src(&src->vb2_buf);
	const struct v4l2_ctrl_vp9_frame *f;
	const struct v4l2_ctrl_vp9_compressed_hdr *ch;
	const struct v4l2_pix_format_mplane *dfmt = &ctx->decoded_fmt.fmt.pix_mp;
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0);
	u32 w, h, mi_cols, mi_rows, sb_cols, sb_rows, stride, uvoff;
	u32 *msg, *head, i, s, r, pos, ntiles = 0, ctb = 0, base, prev_row_sb = 0, row_acc = 0;
	u32 tile_rows, tile_cols, tr, tc;
	int refs[3], ids[3], cur = dst->vb2_buf.index, ret = 0;
	bool intra, key, used_id[VP9_NO_REF] = { };
	u8 lvl[8][4][2] = { };
	u8 *prob = v->prob.cpu;

	if (req)
		v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	f = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_VP9_FRAME);
	ch = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_VP9_COMPRESSED_HDR);
	if (!data || !f || !ch) {
		ret = -EINVAL;
		goto out;
	}
	key = f->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME;
	intra = key || (f->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	w = f->frame_width_minus_1 + 1;
	h = f->frame_height_minus_1 + 1;
	mi_cols = (w + 7) >> 3;
	mi_rows = (h + 7) >> 3;
	sb_cols = (mi_cols + 7) >> 3;
	sb_rows = (mi_rows + 7) >> 3;
	stride = dfmt->plane_fmt[0].bytesperline;
	uvoff = stride * dfmt->height;
	if (w + VP9_PAD > stride || h + VP9_PAD > dfmt->height ||
	    uvoff * 3 / 2 > vb2_plane_size(&dst->vb2_buf, 0)) {
		dev_err(vdec->dev, "capture buffer too small for %ux%u\n", w, h);
		ret = -EINVAL;
		goto out;
	}

	/* references */
	refs[0] = vp9_find_buf(ctx, f->last_frame_ts);
	refs[1] = vp9_find_buf(ctx, f->golden_frame_ts);
	refs[2] = vp9_find_buf(ctx, f->alt_frame_ts);
	for (r = 0; r < 3; r++) {
		if (intra || refs[r] < 0 || refs[r] == cur || v->meta[refs[r]].id < 0) {
			if (!intra)
				dev_dbg(vdec->dev, "vp9: missing reference %u\n", r);
			refs[r] = cur;
		}
	}
	for (r = 0; r < 3; r++) {
		ids[r] = refs[r] == cur ? -1 : v->meta[refs[r]].id;
		if (ids[r] >= 0)
			used_id[ids[r]] = true;
	}
	for (i = 0; i < VP9_NO_REF - 1 && used_id[i]; i++)
		;
	v->meta[cur].id = i;
	v->meta[cur].w = w;
	v->meta[cur].h = h;
	for (r = 0; r < 3; r++)
		if (ids[r] < 0)
			ids[r] = key ? v->meta[cur].id : VP9_NO_REF;

	/* probabilities: forward update in software, backward adaptation by the VDH */
	v->fc_idx = v4l2_vp9_reset_frame_ctx(f, v->fc);
	v->cur = v->fc[v->fc_idx];
	v4l2_vp9_fw_update_probs(&v->cur, ch, f);
	vp9_probs_to_hw(prob + VP9_PROB_TAB, &v->cur, f);
	vp9_probs_to_hw(prob + VP9_PROB_CNT, &v->fc[v->fc_idx], f);
	v->flags = f->flags;
	v->dst_idx = cur;
	v->w = w;
	v->h = h;

	/* segment map and MVs of the previous frame are read and written in place */
	if (intra || (f->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) ||
	    w != v->last_w || h != v->last_h) {
		memset(v->segmap.cpu, 0, VP9_SEGMAP_LEN);
		hivdec_aux_sync_for_device(vdec, &v->segmap);
	}

	if (f->lf.level)
		vp9_lf_levels(f, lvl);

	/* picture message */
	msg = hivdec_msg_slot(ctx, HIVDEC_SLOT_PIC);
	memset(msg, 0, HIVDEC_MSG_SLOT_BYTES);
	msg[0] = (u32)!key | (v->last_frame_type & 3) << 1 | v->last_show << 3 |
		 !!(f->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) << 4 |
		 !!(f->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY) << 7 |
		 !!(f->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX) << 8 |
		 !!(f->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE) << 9 |
		 (ch->tx_mode & 7) << 10 | (f->reference_mode & 3) << 13 |
		 (f->bit_depth & 15) << 15;
	msg[0] |= 3 << 5;	/* profile 0 is 4:2:0, also for intra-only frames */
	msg[1] = (f->interpolation_filter & 7) |
		 !!(f->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV) << 3 |
		 (intra ? 0 : (f->ref_frame_sign_bias & 7) << 5) |
		 ids[0] << 8 | ids[1] << 12 | ids[2] << 16 |
		 (w != v->last_w || h != v->last_h) << 20;
	msg[2] = (mi_cols - 1) | (mi_rows - 1) << 16;
	msg[3] = (f->lf.sharpness & 7) | (f->lf.level & 63) << 8;
	for (s = 0; s < 8; s++) {
		msg[4 + 2 * s] = lvl[s][0][0] | lvl[s][0][1] << 8 |
				 lvl[s][1][0] << 16 | (u32)lvl[s][1][1] << 24;
		msg[5 + 2 * s] = lvl[s][2][0] | lvl[s][2][1] << 8 |
				 lvl[s][3][0] << 16 | (u32)lvl[s][3][1] << 24;
	}
	msg[20] = f->quant.base_q_idx | vp9_abs_sign(f->quant.delta_q_y_dc, 8) |
		  vp9_abs_sign(f->quant.delta_q_uv_dc, 16) |
		  vp9_abs_sign(f->quant.delta_q_uv_ac, 24);
	msg[21] = (u32)!!(f->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED) |
		  !!(f->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP) << 1 |
		  !!(f->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE) << 2 |
		  !!(f->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE) << 3;
	for (s = 0; s < 8; s++) {
		const s16 *fd = f->seg.feature_data[s];

		msg[22] |= (f->seg.feature_enabled[s] & 15) << (4 * s);
		msg[23 + s / 3] |= (fd[V4L2_VP9_SEG_LVL_ALT_Q] & 511) << (9 * (s % 3));
		msg[26 + s / 4] |= (u32)(u8)fd[V4L2_VP9_SEG_LVL_ALT_L] << (8 * (s % 4));
		msg[28] |= (fd[V4L2_VP9_SEG_LVL_REF_FRAME] & 3) << (2 * s) |
			   (fd[V4L2_VP9_SEG_LVL_SKIP] & 1) << (16 + s);
	}
	msg[29] = HIVDEC_ADDR(v->segmap.map.iova);
	msg[30] = VP9_SEGMAP_STRIDE;
	for (r = 0; r < 3; r++)
		msg[35 - r] = HIVDEC_ADDR(vp9_buf(ctx, refs[r])->map.iova);	/* last, golden, alt */
	msg[36] = HIVDEC_ADDR(v->sed_top);
	msg[37] = HIVDEC_ADDR(v->pmv_top);
	msg[38] = HIVDEC_ADDR(v->rcn_top);
	msg[39] = HIVDEC_ADDR(v->prob.map.iova + VP9_PROB_TAB);
	msg[40] = HIVDEC_ADDR(v->dblk_top);
	msg[41] = HIVDEC_ADDR(v->prob.map.iova + VP9_PROB_CNT);
	msg[42] = HIVDEC_ADDR(v->mv);
	msg[43] = HIVDEC_ADDR(v->dblk_left);
	msg[44] = 0x001e140a;
	for (r = 0; r < 3; r++) {
		u32 rw = intra ? w : v->meta[refs[r]].w, rh = intra ? h : v->meta[refs[r]].h;
		u32 xs = (rw << 14) / w, ys = (rh << 14) / h;

		if (rw > 2 * w || rh > 2 * h || w > 16 * rw || h > 16 * rh)
			xs = ys = 0xffff;
		msg[45 + r] = ys | xs << 16;
		msg[48] |= ((ys >> 10) & 63) << (8 * r);
		msg[49] |= ((xs >> 10) & 63) << (8 * r);
		msg[50 + r] = intra ? 0 : rw | rh << 16;
		msg[54 + 3 * r] = stride * 8;
		msg[55 + 3 * r] = uvoff;
		msg[56 + 3 * r] = stride * 4;
	}
	msg[53] = v->last_w | v->last_h << 16;
	msg[63] = hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_SLICE0);

	/* head message: dummy compression head buffers */
	head = hivdec_msg_slot(ctx, HIVDEC_SLOT_HEAD);
	memset(head, 0, VP9_TILE_MSG_BYTES);
	head[0] = HIVDEC_ADDR(v->head);
	head[1] = HIVDEC_ADDR(v->head + VP9_HEAD_LEN / 2);
	head[2] = ALIGN(stride / 16, 32);
	for (r = 0; r < 3; r++) {
		head[3 + 2 * r] = head[0];
		head[4 + 2 * r] = head[1];
		head[35 + r] = head[2];
	}

	/* tiles: all but the last one carry a 4-byte big endian size */
	pos = f->uncompressed_header_size + f->compressed_header_size;
	base = sbuf->map.iova & ~15;
	tile_rows = 1 << f->tile_rows_log2;
	tile_cols = 1 << f->tile_cols_log2;
	for (tr = 0; tr < tile_rows; tr++) {
		u32 mi_r0, mi_r1, row_sb, col_acc = 0, col_sb = 0;

		row_acc += sb_rows;
		row_sb = row_acc >> f->tile_rows_log2;
		mi_r0 = min(prev_row_sb << 3, mi_rows);
		mi_r1 = min(row_sb << 3, mi_rows);
		prev_row_sb = row_sb;
		for (tc = 0; tc < tile_cols; tc++) {
			u32 mi_c0 = min(col_sb << 3, mi_cols), mi_c1, tsize, n_sb, addr;
			u32 sbr0 = (mi_r0 + 7) >> 3, sbr1 = (mi_r1 + 7) >> 3;
			u32 sbc0, sbc1;

			col_acc += sb_cols;
			col_sb = col_acc >> f->tile_cols_log2;
			mi_c1 = min(col_sb << 3, mi_cols);
			sbc0 = (mi_c0 + 7) >> 3;
			sbc1 = (mi_c1 + 7) >> 3;

			if (tr == tile_rows - 1 && tc == tile_cols - 1) {
				tsize = size > pos ? size - pos : 0;
			} else {
				if (pos + 4 > size) {
					ret = -EINVAL;
					goto out;
				}
				tsize = get_unaligned_be32(data + pos);
				pos += 4;
			}
			if (tsize > size - pos) {
				ret = -EINVAL;
				goto out;
			}
			n_sb = (sbc1 - sbc0) * (sbr1 - sbr0);
			if (n_sb) {
				if (ntiles == VP9_MAX_TILES) {
					ret = -EINVAL;
					goto out;
				}
				msg = (u32 *)((u8 *)hivdec_msg_slot(ctx, HIVDEC_SLOT_SLICE0) +
					      ntiles * VP9_TILE_MSG_BYTES);
				memset(msg, 0, VP9_TILE_MSG_BYTES);
				addr = sbuf->map.iova + pos;
				msg[0] = ((addr & ~15) - base) >> 4;
				msg[1] = (addr & 15) * 8;
				msg[2] = tsize * 8 + 128;
				msg[6] = sbr0 << 16 | (sbr1 - 1);
				msg[7] = sbc0 << 16 | (sbc1 - 1);
				msg[8] = (sbr1 - 1) * sb_cols + sbc1 - 1;
				msg[9] = ctb + n_sb - 1;
				msg[10] = sbr0 * sb_cols + sbc0;
				msg[11] = ctb;
				if (ntiles)
					((u32 *)((u8 *)msg - VP9_TILE_MSG_BYTES))[63] =
						HIVDEC_ADDR(ctx->msg.map.iova +
							    HIVDEC_SLOT_SLICE0 * HIVDEC_MSG_SLOT_BYTES +
							    ntiles * VP9_TILE_MSG_BYTES);
				ntiles++;
				ctb += n_sb;
			}
			pos += tsize;
		}
	}
	if (!ntiles) {
		ret = -EINVAL;
		goto out;
	}

	/* registers */
	vdh_write(vdec, VDH_BASIC_CFG0, CFG0_MBAMT_TO_DEC(sb_cols * sb_rows - 1));
	vdh_write(vdec, VDH_BASIC_CFG1, CFG1_VIDEO_STANDARD(HIVDEC_STD_VP9) | CFG1_MFD_MMU_EN |
		  CFG1_UV_ORDER_EN | CFG1_FST_SLC_GRP | CFG1_MV_OUTPUT_EN |
		  CFG1_MAX_SLCGRP_NUM(3) | CFG1_VDH_2D_EN);
	vdh_write(vdec, VDH_AVM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_PIC));
	vdh_write(vdec, VDH_VAM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_UP));
	vdh_write(vdec, VDH_STREAM_BASE_ADDR, HIVDEC_ADDR(base));
	vdh_write(vdec, VDH_SED_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_ITRANS_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PMV_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PRC_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_RCN_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_DBLK_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PPFD_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PART_DEC_OVER_INT_LEVEL, 60);
	vdh_write(vdec, VDH_YSTADDR_1D, HIVDEC_ADDR(dbuf->map.iova));
	vdh_write(vdec, VDH_YSTRIDE_1D, stride * 8);
	vdh_write(vdec, VDH_UVOFFSET_1D, uvoff);
	vdh_write(vdec, VDH_HEAD_INF_OFFSET, 0);
	vdh_write(vdec, VDH_YSTRIDE_2BIT, 0);
	vdh_write(vdec, VDH_YOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_UVOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_REF_PIC_TYPE, 0);
	vdh_write(vdec, VDH_FF_APT_EN, 0);
	vdh_write(vdec, VDH_UVSTRIDE_1D, (stride * 4) & 0x3ffff);
	vdh_write(vdec, VDH_CFGINFO_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_HEAD));
	vdh_write(vdec, VDH_DDR_INTERLEAVE_MODE, 3);
	vdh_write(vdec, SCD_EMAR_ID, 0x101);
	vdh_write(vdec, SCD_AVS_FLAG, 0);
	vdh_write(vdec, SCD_VDH_SELRST, 1);

	if (vdec->debug)
		dev_info(vdec->dev, "vp9 %ux%u %s%s show %u q %u lf %u tiles %ux%u (%u) ctx %u refs %d %d %d ids %d %d %d->%d\n",
			 w, h, key ? "key" : "inter", (f->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY) ? " intra" : "",
			 !!(f->flags & V4L2_VP9_FRAME_FLAG_SHOW_FRAME), f->quant.base_q_idx, f->lf.level,
			 tile_cols, tile_rows, ntiles, v->fc_idx, refs[0], refs[1], refs[2],
			 ids[0], ids[1], ids[2], v->meta[cur].id);

	hivdec_aux_sync_for_device(vdec, &v->prob);
out:
	if (req)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	if (ret)
		return ret;
	hivdec_hw_run(ctx);
	return 0;
}

/* walks the DMA segments of a CAPTURE buffer in increasing offset order */
struct vp9_sg_cursor {
	struct device *dev;
	struct scatterlist *sg;
	u32 base;		/* buffer offset of the current segment */
};

static void vp9_sync(struct vp9_sg_cursor *c, u32 off, u32 len, bool for_cpu)
{
	while (len && c->sg) {
		u32 seg = sg_dma_len(c->sg), n;

		if (off >= c->base + seg) {
			c->base += seg;
			c->sg = sg_next(c->sg);
			continue;
		}
		n = min(len, c->base + seg - off);
		if (for_cpu)
			dma_sync_single_for_cpu(c->dev, sg_dma_address(c->sg) + off - c->base, n,
						DMA_FROM_DEVICE);
		else
			dma_sync_single_for_device(c->dev, sg_dma_address(c->sg) + off - c->base, n,
						   DMA_TO_DEVICE);
		off += n;
		len -= n;
	}
}

static void vp9_cursor_init(struct vp9_sg_cursor *c, struct device *dev, struct sg_table *sgt)
{
	c->dev = dev;
	c->sg = sgt->sgl;
	c->base = 0;
}

/* replicate the right and bottom edges of a w x h NV12 frame into the padding */
static void vp9_pad_frame(struct hivdec_ctx *ctx, struct vb2_v4l2_buffer *dst, u32 w, u32 h)
{
	const struct v4l2_pix_format_mplane *fmt = &ctx->decoded_fmt.fmt.pix_mp;
	struct sg_table *sgt = vb2_dma_sg_plane_desc(&dst->vb2_buf, 0);
	u8 *p = vb2_plane_vaddr(&dst->vb2_buf, 0);
	u32 stride = fmt->plane_fmt[0].bytesperline, uv = stride * fmt->height;
	u32 cw = 2 * DIV_ROUND_UP(w, 2), ch = DIV_ROUND_UP(h, 2), y;
	struct vp9_sg_cursor c;

	if (!p || !sgt || w + VP9_PAD > stride || h + VP9_PAD > fmt->height)
		return;

	/* luma: right edge, then the bottom rows */
	vp9_cursor_init(&c, ctx->dev->dev, sgt);
	for (y = 0; y < h; y++) {
		u32 off = y * stride + w - 1;

		vp9_sync(&c, off, 1, true);
		memset(p + off + 1, p[off], VP9_PAD);
		vp9_sync(&c, off + 1, VP9_PAD, false);
	}
	vp9_cursor_init(&c, ctx->dev->dev, sgt);
	vp9_sync(&c, (h - 1) * stride, w, true);
	for (y = h; y < h + VP9_PAD; y++) {
		memcpy(p + y * stride, p + (h - 1) * stride, w + VP9_PAD);
		vp9_sync(&c, y * stride, w + VP9_PAD, false);
	}

	/* chroma: interleaved U/V pairs */
	vp9_cursor_init(&c, ctx->dev->dev, sgt);
	for (y = 0; y < ch; y++) {
		u32 off = uv + y * stride + cw - 2, x;

		vp9_sync(&c, off, 2, true);
		for (x = 2; x <= VP9_PAD; x += 2)
			memcpy(p + off + x, p + off, 2);
		vp9_sync(&c, off + 2, VP9_PAD, false);
	}
	vp9_cursor_init(&c, ctx->dev->dev, sgt);
	vp9_sync(&c, uv + (ch - 1) * stride, cw, true);
	for (y = ch; y < ch + VP9_PAD / 2; y++) {
		memcpy(p + uv + y * stride, p + uv + (ch - 1) * stride, cw + VP9_PAD);
		vp9_sync(&c, uv + y * stride, cw + VP9_PAD, false);
	}
}

static int hivdec_vp9_done(struct hivdec_ctx *ctx, enum vb2_buffer_state state)
{
	struct hivdec_vp9_ctx *v = ctx->priv;
	bool intra = v->flags & (V4L2_VP9_FRAME_FLAG_KEY_FRAME | V4L2_VP9_FRAME_FLAG_INTRA_ONLY);

	if (state == VB2_BUF_STATE_DONE &&
	    !(v->flags & (V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT | V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE))) {
		hivdec_aux_sync_for_cpu(ctx->dev, &v->prob);
		vp9_probs_from_hw(&v->cur, (u8 *)v->prob.cpu + VP9_PROB_CNT, intra);
	}
	if (v->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX)
		v->fc[v->fc_idx] = v->cur;

	v->last_frame_type = !(v->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME);
	v->last_show = !!(v->flags & V4L2_VP9_FRAME_FLAG_SHOW_FRAME);
	v->last_w = v->w;
	v->last_h = v->h;
	if (state != VB2_BUF_STATE_DONE && v->dst_idx >= 0)
		v->meta[v->dst_idx].id = -1;

	if (state == VB2_BUF_STATE_DONE)
		vp9_pad_frame(ctx, v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx), v->w, v->h);
	return 0;
}

const struct hivdec_coded_fmt_ops hivdec_vp9_fmt_ops = {
	.adjust_fmt = hivdec_vp9_adjust_fmt,
	.start = hivdec_vp9_start,
	.stop = hivdec_vp9_stop,
	.run = hivdec_vp9_run,
	.done = hivdec_vp9_done,
	.try_ctrl = hivdec_vp9_try_ctrl,
};
