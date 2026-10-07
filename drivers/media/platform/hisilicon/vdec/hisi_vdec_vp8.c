// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video decoder: VP8
 *
 * Userspace parses the frame header (the first-partition header up to the
 * macroblock data) and passes the final probabilities and the bool decoder
 * state; the VDH decodes the macroblock headers and the DCT partitions
 * straight from the OUTPUT buffer. Message layout of the HiSilicon VFMW VP8
 * HAL as used on Kirin: picture message in slot 5 with the DCT partition
 * table 256 bytes into it, compression head message in slot 4.
 */

#include <linux/string.h>
#include <media/v4l2-mem2mem.h>

#include "hisi_vdec.h"

#define VP8_TOP_LEN		SZ_1M
#define VP8_PROB_LEN		2752
#define VP8_SEGMAP_LEN		SZ_32K
#define VP8_HEAD_LEN		SZ_64K

struct hivdec_vp8_ctx {
	struct hivdec_aux_buf work;	/* line buffers, segment map, head */
	struct hivdec_aux_buf prob;
	u32 sed_top, pmv_top, rcn_top, dblk_top, segmap, head;	/* IOVAs */

	/* previous frame */
	u8 last_frame_type;
	u8 last_filter_type;
	u8 last_sharpness;
};

/* ---------------------------------------------------------------- setup */

static int hivdec_vp8_start(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_vp8_ctx *v;
	u32 base;
	int ret;

	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	ret = hivdec_aux_alloc(vdec, &v->work, 4 * VP8_TOP_LEN + VP8_SEGMAP_LEN + VP8_HEAD_LEN);
	if (ret)
		goto err_free;
	ret = hivdec_aux_alloc(vdec, &v->prob, SZ_4K);
	if (ret)
		goto err_work;

	base = v->work.map.iova;
	v->sed_top = base;
	v->pmv_top = base + VP8_TOP_LEN;
	v->rcn_top = base + 2 * VP8_TOP_LEN;
	v->dblk_top = base + 3 * VP8_TOP_LEN;
	v->segmap = base + 4 * VP8_TOP_LEN;
	v->head = v->segmap + VP8_SEGMAP_LEN;
	hivdec_aux_sync_for_device(vdec, &v->work);

	v->last_frame_type = 3;
	v->last_filter_type = 3;
	ctx->priv = v;
	return 0;

err_work:
	hivdec_aux_free(vdec, &v->work);
err_free:
	kfree(v);
	return ret;
}

static void hivdec_vp8_stop(struct hivdec_ctx *ctx)
{
	struct hivdec_vp8_ctx *v = ctx->priv;

	if (!v)
		return;
	hivdec_aux_free(ctx->dev, &v->prob);
	hivdec_aux_free(ctx->dev, &v->work);
	kfree(v);
	ctx->priv = NULL;
}

static int hivdec_vp8_adjust_fmt(struct hivdec_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 3 / 2;
	fmt->plane_fmt[0].sizeimage = max_t(u32, fmt->plane_fmt[0].sizeimage, SZ_512K);
	return 0;
}

/* ---------------------------------------------------------------- tables */

static void vp8_build_prob_table(u8 *tab, const struct v4l2_ctrl_vp8_frame *f)
{
	const struct v4l2_vp8_entropy *e = &f->entropy;
	int i, j, k, c;

	memset(tab, 0, VP8_PROB_LEN);
	tab[0] = f->prob_skip_false;
	tab[1] = f->segment.segment_probs[0];
	tab[2] = f->segment.segment_probs[1];
	tab[3] = f->segment.segment_probs[2];
	tab[4] = f->prob_intra;
	tab[5] = f->prob_last;
	tab[6] = f->prob_gf;
	memcpy(tab + 16, e->y_mode_probs, 4);
	memcpy(tab + 20, e->uv_mode_probs, 3);
	for (c = 0; c < 2; c++) {
		u8 *t = tab + 704 + 32 * c;
		const u8 *m = e->mv_probs[c];

		t[0] = m[1];			/* sign */
		t[1] = m[0];			/* is_short */
		memcpy(t + 2, m + 2, 7);	/* short tree */
		t[16] = m[1];
		memcpy(t + 17, m + 9, 10);	/* long bits */
	}
	for (i = 0; i < 4; i++)
		for (j = 0; j < 8; j++)
			for (k = 0; k < 3; k++)
				memcpy(tab + 768 + i * 384 + j * 48 + k * 16,
				       e->coeff_probs[i][j][k], 11);
}

/* ---------------------------------------------------------------- decode */

static struct hivdec_decoded_buffer *vp8_ref(struct hivdec_ctx *ctx, u64 ts,
					     struct hivdec_decoded_buffer *cur)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = vb2_find_buffer(q, ts);

	return vb ? vb2_to_hivdec_buf(vb) : cur;
}

static int hivdec_vp8_run(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_vp8_ctx *v = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct hivdec_decoded_buffer *dbuf = vb2_to_hivdec_buf(&dst->vb2_buf);
	struct hivdec_src_buffer *sbuf = vb2_to_hivdec_src(&src->vb2_buf);
	const struct v4l2_pix_format_mplane *dfmt = &ctx->decoded_fmt.fmt.pix_mp;
	const struct v4l2_ctrl_vp8_frame *f;
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0);
	u32 wmb, hmb, stride, uvoff, hdr, base, start, len, cnt, la = 0, off, i;
	u32 *msg, *head;
	bool key;
	int ret = 0;

	if (req)
		v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	f = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_VP8_FRAME);
	if (!data || !f || !f->num_dct_parts || f->num_dct_parts > 8) {
		ret = -EINVAL;
		goto out;
	}
	key = f->flags & V4L2_VP8_FRAME_FLAG_KEY_FRAME;
	hdr = key ? 10 : 3;
	wmb = DIV_ROUND_UP(f->width, 16);
	hmb = DIV_ROUND_UP(f->height, 16);
	stride = dfmt->plane_fmt[0].bytesperline;
	uvoff = stride * dfmt->height;
	if (wmb * 16 > stride || hmb * 16 > dfmt->height ||
	    uvoff * 3 / 2 > vb2_plane_size(&dst->vb2_buf, 0)) {
		dev_err(vdec->dev, "capture buffer too small for %ux%u\n", f->width, f->height);
		ret = -EINVAL;
		goto out;
	}
	off = hdr + f->first_part_size + 3 * (f->num_dct_parts - 1);
	for (i = 0; i < f->num_dct_parts; i++)
		off += f->dct_part_sizes[i];
	if (off > size || f->first_part_header_bits + 8 > f->first_part_size * 8) {
		ret = -EINVAL;
		goto out;
	}

	vp8_build_prob_table(v->prob.cpu, f);

	/*
	 * Bool decoder: the VDH continues right after the 8-bit value window,
	 * first_part_header_bits into partition 1; value carries the window
	 * plus the look-ahead bits up to the next byte boundary.
	 */
	base = sbuf->map.iova & ~15;
	start = (sbuf->map.iova & 15) * 8 + hdr * 8 + f->first_part_header_bits + 8;
	len = f->first_part_size * 8 - f->first_part_header_bits - 8;
	cnt = (-f->first_part_header_bits) & 7;
	if (cnt) {
		u32 pos = hdr * 8 + f->first_part_header_bits + 8;

		la = data[pos / 8] & ((1 << cnt) - 1);
	}

	msg = hivdec_msg_slot(ctx, HIVDEC_SLOT_PIC);
	memset(msg, 0, HIVDEC_MSG_SLOT_BYTES);
	msg[0] = (u32)!key | (v->last_frame_type & 3) << 1;
	/* bit 0: 6-tap filter (versions 0 and >3), bit 1: full pixel (version 3) */
	msg[1] = (f->version == 3) << 1 | (f->version == 0 || f->version > 3);
	msg[2] = (wmb - 1) | (hmb - 1) << 16;
	msg[3] = f->prob_skip_false | !!(f->flags & V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF) << 8 |
		 ilog2(f->num_dct_parts) << 9;
	msg[4] = (u32)!!(f->segment.flags & V4L2_VP8_SEGMENT_FLAG_ENABLED) |
		 !!(f->segment.flags & V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP) << 1 |
		 !(f->segment.flags & V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE) << 2 |
		 !!(f->lf.flags & V4L2_VP8_LF_ADJ_ENABLE) << 3;
	msg[5] = (u32)!!(f->lf.flags & V4L2_VP8_LF_FILTER_TYPE_SIMPLE) |
		 (v->last_filter_type & 3) << 1 | (f->lf.level & 63) << 3 |
		 (f->lf.sharpness_level & 7) << 9 | (v->last_sharpness & 7) << 12;
	msg[6] = (f->quant.y_dc_delta < 0) | (abs(f->quant.y_dc_delta) & 15) << 1 |
		 (f->quant.y2_dc_delta < 0) << 5 | (abs(f->quant.y2_dc_delta) & 15) << 6 |
		 (f->quant.y2_ac_delta < 0) << 10 | (abs(f->quant.y2_ac_delta) & 15) << 11 |
		 (f->quant.uv_dc_delta < 0) << 15 | (abs(f->quant.uv_dc_delta) & 15) << 16 |
		 (f->quant.uv_ac_delta < 0) << 20 | (abs(f->quant.uv_ac_delta) & 15) << 21;
	msg[7] = f->quant.y_ac_qi & 0x7f;
	msg[8] = (u32)f->coder_state.value << 16 | la << (16 - cnt);
	msg[9] = f->coder_state.range | cnt << 16;
	msg[16] = (len & 0x1ffffff) | (start & 127) << 25;
	msg[17] = (start >> 7) & 0xfffff;
	for (i = 0; i < 4; i++) {
		msg[20] |= (u32)(u8)f->segment.quant_update[i] << (8 * i);
		msg[21] |= (u32)(u8)f->segment.lf_update[i] << (8 * i);
		msg[22] |= (u32)(u8)f->lf.ref_frm_delta[i] << (8 * i);
		msg[23] |= (u32)(u8)f->lf.mb_mode_delta[i] << (8 * i);
	}
	msg[24] = !!(f->flags & V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN) << 16 |
		  !!(f->flags & V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT) << 24;
	msg[25] = HIVDEC_ADDR(v->segmap);
	msg[32] = (dbuf->map.iova + 15) >> 4;
	if (key) {
		msg[33] = msg[34] = msg[35] = msg[32];
	} else {
		msg[33] = (vp8_ref(ctx, f->alt_frame_ts, dbuf)->map.iova + 15) >> 4;
		msg[34] = (vp8_ref(ctx, f->golden_frame_ts, dbuf)->map.iova + 15) >> 4;
		msg[35] = (vp8_ref(ctx, f->last_frame_ts, dbuf)->map.iova + 15) >> 4;
	}
	msg[36] = HIVDEC_ADDR(v->sed_top);
	msg[37] = HIVDEC_ADDR(v->pmv_top);
	msg[38] = HIVDEC_ADDR(v->rcn_top);
	msg[39] = HIVDEC_ADDR(v->prob.map.iova);
	msg[40] = HIVDEC_ADDR(v->dblk_top);
	msg[63] = HIVDEC_ADDR(ctx->msg.map.iova + HIVDEC_SLOT_PIC * HIVDEC_MSG_SLOT_BYTES + 256);

	/* DCT partitions follow the 3-byte size table after partition 1 */
	off = (sbuf->map.iova & 15) + hdr + f->first_part_size + 3 * (f->num_dct_parts - 1);
	for (i = 0; i < f->num_dct_parts; i++) {
		msg[64 + 4 * i] = ((f->dct_part_sizes[i] * 8) & 0x1ffffff) | (off & 15) << 28;
		msg[65 + 4 * i] = (off >> 4) & 0xffffff;
		off += f->dct_part_sizes[i];
	}

	/* head message: dummy compression head buffers */
	head = hivdec_msg_slot(ctx, HIVDEC_SLOT_HEAD);
	memset(head, 0, HIVDEC_MSG_SLOT_BYTES);
	head[0] = HIVDEC_ADDR(v->head);
	head[1] = HIVDEC_ADDR(v->head + VP8_HEAD_LEN / 2);
	head[2] = ALIGN(stride / 16, 32);
	for (i = 0; i < 3; i++) {
		head[3 + 2 * i] = head[0];
		head[4 + 2 * i] = head[1];
	}

	/* registers */
	vdh_write(vdec, VDH_BASIC_CFG0, CFG0_MBAMT_TO_DEC(wmb * hmb - 1));
	vdh_write(vdec, VDH_BASIC_CFG1, CFG1_VIDEO_STANDARD(HIVDEC_STD_VP8) | CFG1_MFD_MMU_EN |
		  CFG1_UV_ORDER_EN | CFG1_FST_SLC_GRP | CFG1_MV_OUTPUT_EN | CFG1_VDH_2D_EN);
	vdh_write(vdec, VDH_AVM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_PIC));
	vdh_write(vdec, VDH_VAM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_UP));
	vdh_write(vdec, VDH_STREAM_BASE_ADDR, HIVDEC_ADDR(base));
	vdh_write(vdec, VDH_YSTADDR_1D, HIVDEC_ADDR(dbuf->map.iova));
	vdh_write(vdec, VDH_YSTRIDE_1D, stride * 8);
	vdh_write(vdec, VDH_UVOFFSET_1D, uvoff);
	vdh_write(vdec, VDH_UVSTRIDE_1D, (stride * 4) & 0x3ffff);
	vdh_write(vdec, VDH_HEAD_INF_OFFSET, 0);
	vdh_write(vdec, VDH_YSTRIDE_2BIT, 0);
	vdh_write(vdec, VDH_YOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_UVOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_REF_PIC_TYPE, 0);
	vdh_write(vdec, VDH_FF_APT_EN, 0);
	vdh_write(vdec, VDH_CFGINFO_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_HEAD));
	vdh_write(vdec, VDH_DDR_INTERLEAVE_MODE, 3);
	vdh_write(vdec, SCD_EMAR_ID, 0x101);
	vdh_write(vdec, SCD_AVS_FLAG, 0);
	vdh_write(vdec, SCD_VDH_SELRST, 1);

	if (vdec->debug)
		dev_info(vdec->dev, "vp8 %ux%u %s v%u q %u lf %u parts %u hdr bits %u\n",
			 f->width, f->height, key ? "key" : "inter", f->version,
			 f->quant.y_ac_qi, f->lf.level, f->num_dct_parts,
			 f->first_part_header_bits);

	v->last_frame_type = !key;
	v->last_filter_type = !!(f->lf.flags & V4L2_VP8_LF_FILTER_TYPE_SIMPLE);
	v->last_sharpness = f->lf.sharpness_level;
	hivdec_aux_sync_for_device(vdec, &v->prob);
out:
	if (req)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	if (ret)
		return ret;
	hivdec_hw_run(ctx);
	return 0;
}

const struct hivdec_coded_fmt_ops hivdec_vp8_fmt_ops = {
	.adjust_fmt = hivdec_vp8_adjust_fmt,
	.start = hivdec_vp8_start,
	.stop = hivdec_vp8_stop,
	.run = hivdec_vp8_run,
};
