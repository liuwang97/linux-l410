// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video decoder: MPEG-2
 *
 * One request carries all slices of a picture (frame or field) with their
 * start codes. The VDH does not parse slice headers: the driver finds the
 * slices, reads quantiser_scale_code / intra_slice and the first macroblock
 * address increment, and hands every slice over as a bit range starting
 * at the first macroblock. Message layout of the HiSilicon VFMW MPEG-2 HAL
 * as used on Kirin: picture message in slot 5, 32-byte slice messages
 * following it.
 */

#include <linux/string.h>
#include <media/v4l2-mem2mem.h>

#include "hisi_vdec.h"

#define MPEG2_MAX_SLICES	((HIVDEC_MSG_SLOTS - HIVDEC_SLOT_PIC) * HIVDEC_MSG_SLOT_BYTES / 32 - 9)
#define MPEG2_SLICE_MSG_WORDS	8
#define MPEG2_TOP_LEN		SZ_1M

/* MPEG-2 picture_structure */
#define MPEG2_TOP_FIELD		1
#define MPEG2_BOTTOM_FIELD	2
#define MPEG2_FRAME		3

struct hivdec_mpeg2_slice {
	u32 bitpos;		/* first macroblock, from the start of the OUTPUT buffer */
	u32 bits;
	u32 start_mb;
	u8 qscale;
	u8 intra;
};

struct hivdec_mpeg2_ctx {
	struct hivdec_aux_buf work;	/* pmv_top line buffer + MV output */
	u32 pmv_top, mv;		/* IOVAs */
	struct hivdec_mpeg2_slice *slices;	/* MPEG2_MAX_SLICES */
	bool field_coded[VB2_MAX_FRAME];	/* per CAPTURE buffer: decoded as two fields */
	/* previous picture, to pair field pictures (top_field_first is 0 for them) */
	int last_dst;
	u8 last_structure;
	bool last_first_field;
};

/* scan index -> raster index */
static const u8 mpeg2_zigzag[64] = {
	0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* ---------------------------------------------------------------- setup */

static int hivdec_mpeg2_start(struct hivdec_ctx *ctx)
{
	struct hivdec_mpeg2_ctx *m;
	int ret;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->slices = kvcalloc(MPEG2_MAX_SLICES, sizeof(*m->slices), GFP_KERNEL);
	if (!m->slices) {
		kfree(m);
		return -ENOMEM;
	}
	/* MV output: up to 64 bytes per macroblock of a 4096 x 2304 picture */
	ret = hivdec_aux_alloc(ctx->dev, &m->work, MPEG2_TOP_LEN + 256 * 144 * 64);
	if (ret) {
		kvfree(m->slices);
		kfree(m);
		return ret;
	}
	m->last_dst = -1;
	m->pmv_top = m->work.map.iova;
	m->mv = m->work.map.iova + MPEG2_TOP_LEN;
	hivdec_aux_sync_for_device(ctx->dev, &m->work);
	ctx->priv = m;
	return 0;
}

static void hivdec_mpeg2_stop(struct hivdec_ctx *ctx)
{
	struct hivdec_mpeg2_ctx *m = ctx->priv;

	if (!m)
		return;
	hivdec_aux_free(ctx->dev, &m->work);
	kvfree(m->slices);
	kfree(m);
	ctx->priv = NULL;
}

static int hivdec_mpeg2_adjust_fmt(struct hivdec_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 3 / 2;
	fmt->plane_fmt[0].sizeimage = max_t(u32, fmt->plane_fmt[0].sizeimage, SZ_512K);
	return 0;
}

static int hivdec_mpeg2_try_ctrl(struct hivdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_MPEG2_SEQUENCE) {
		const struct v4l2_ctrl_mpeg2_sequence *seq = ctrl->p_new.p_mpeg2_sequence;

		if (seq->chroma_format != 1)	/* 4:2:0 */
			return -EINVAL;
		if (seq->horizontal_size > 4096 || seq->vertical_size > 2304)
			return -EINVAL;
	}
	return 0;
}

/* ---------------------------------------------------------------- slice headers */

struct mpeg2_bits {
	const u8 *p;
	u32 size;	/* bytes */
	u32 pos;	/* bits */
};

static u32 mpeg2_show(struct mpeg2_bits *b, u32 n)
{
	u32 v = 0, i;

	for (i = 0; i < n; i++) {
		u32 bit = b->pos + i, byte = bit >> 3;

		v = v << 1 | (byte < b->size ? (b->p[byte] >> (7 - (bit & 7))) & 1 : 0);
	}
	return v;
}

static u32 mpeg2_read(struct mpeg2_bits *b, u32 n)
{
	u32 v = mpeg2_show(b, n);

	b->pos += n;
	return v;
}

/* macroblock_address_increment, table B-1 (escape/stuffing handled by the caller) */
static int mpeg2_mba_inc(struct mpeg2_bits *b)
{
	u32 c = mpeg2_show(b, 11), v;

	if (c >= 1024) {
		b->pos += 1;
		return 1;
	}
	if (c >= 512) {
		b->pos += 3;
		return 2 + ((c >> 8) == 2);
	}
	if (c >= 256) {
		b->pos += 4;
		return 4 + ((c >> 7) == 2);
	}
	if (c >= 128) {
		b->pos += 5;
		return 6 + ((c >> 6) == 2);
	}
	if (c >= 64) {
		v = c >> 4;
		if (v >= 6) {
			b->pos += 7;
			return 8 + (7 - v);
		}
		b->pos += 8;
		return 10 + (11 - (c >> 3));
	}
	if (c >= 32) {
		v = c >> 3;
		if (v >= 6) {
			b->pos += 8;
			return 14 + (7 - v);
		}
		v = c >> 1;
		if (v >= 18) {
			b->pos += 10;
			return 16 + (23 - v);
		}
		b->pos += 11;
		return 22 + (35 - c);
	}
	if (c >= 24) {
		b->pos += 11;
		return 26 + (31 - c);
	}
	return -EINVAL;
}

/*
 * slice(): start code at @sc (the 00 00 01 prefix), data up to @end. Fills
 * the slice up to the first macroblock's address increment.
 */
static int mpeg2_parse_slice(const u8 *data, u32 sc, u32 end, u32 vsize, u32 wmb,
			     struct hivdec_mpeg2_slice *s)
{
	struct mpeg2_bits b = { .p = data, .size = end, .pos = (sc + 4) * 8 };
	u32 row = data[sc + 3] - 1, inc = 0;
	int v;

	if (vsize > 2800)
		row += mpeg2_read(&b, 3) << 7;
	s->qscale = mpeg2_read(&b, 5);
	s->intra = 0;
	/* intra_slice_flag, else the closing extra_bit_slice */
	if (mpeg2_read(&b, 1)) {
		s->intra = mpeg2_read(&b, 1);
		mpeg2_read(&b, 7);		/* reserved_bits */
		while (mpeg2_read(&b, 1))	/* extra_bit_slice */
			mpeg2_read(&b, 8);	/* extra_information_slice */
	}
	for (;;) {
		u32 c = mpeg2_show(&b, 11);

		if (c == 8)		/* macroblock_escape */
			inc += 33;
		else if (c != 15)	/* macroblock_stuffing (MPEG-1) */
			break;
		b.pos += 11;
	}
	v = mpeg2_mba_inc(&b);
	if (v < 0 || b.pos >= end * 8)
		return -EINVAL;
	inc += v;
	s->start_mb = row * wmb + inc - 1;
	s->bitpos = b.pos;
	s->bits = end * 8 - b.pos;
	return 0;
}

/* ---------------------------------------------------------------- decode */

static int mpeg2_find_buf(struct hivdec_ctx *ctx, u64 ts)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = vb2_find_buffer(q, ts);

	return vb ? vb->index : -1;
}

static int hivdec_mpeg2_run(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_mpeg2_ctx *m = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct hivdec_decoded_buffer *dbuf = vb2_to_hivdec_buf(&dst->vb2_buf);
	struct hivdec_src_buffer *sbuf = vb2_to_hivdec_src(&src->vb2_buf);
	const struct v4l2_pix_format_mplane *dfmt = &ctx->decoded_fmt.fmt.pix_mp;
	const struct v4l2_ctrl_mpeg2_sequence *seq;
	const struct v4l2_ctrl_mpeg2_picture *pic;
	const struct v4l2_ctrl_mpeg2_quantisation *q;
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0);
	u32 wmb, hmb, frame_hmb, stride, uvoff, base, slot_iova, i, j, n = 0, pre = 0;
	u32 *msg, *slc, cur, fwd, bwd, fwd_fld = 0, bwd_fld = 0;
	bool field, second;
	int fi, bi, ret = 0;

	if (req)
		v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	seq = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_MPEG2_SEQUENCE);
	pic = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_MPEG2_PICTURE);
	q = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_MPEG2_QUANTISATION);
	if (!data || !seq || !pic || !q) {
		ret = -EINVAL;
		goto out;
	}

	field = pic->picture_structure != MPEG2_FRAME;
	/* the second field goes into the CAPTURE buffer held for the first one */
	second = field && m->last_first_field && m->last_dst == dst->vb2_buf.index &&
		 m->last_structure != pic->picture_structure;
	m->last_dst = dst->vb2_buf.index;
	m->last_structure = pic->picture_structure;
	m->last_first_field = field && !second;
	wmb = DIV_ROUND_UP(seq->horizontal_size, 16);
	frame_hmb = (seq->flags & V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE) ?
		    DIV_ROUND_UP(seq->vertical_size, 16) : 2 * DIV_ROUND_UP(seq->vertical_size, 32);
	hmb = field ? frame_hmb / 2 : DIV_ROUND_UP(seq->vertical_size, 16);
	stride = dfmt->plane_fmt[0].bytesperline;
	uvoff = stride * dfmt->height;
	if (wmb * 16 > stride || frame_hmb * 16 > dfmt->height ||
	    uvoff * 3 / 2 > vb2_plane_size(&dst->vb2_buf, 0)) {
		dev_err(vdec->dev, "capture buffer too small for %ux%u\n",
			seq->horizontal_size, seq->vertical_size);
		ret = -EINVAL;
		goto out;
	}

	/* slices: start codes 00 00 01 01..af, each up to the next start code */
	for (i = 0; i + 3 < size && n < MPEG2_MAX_SLICES; i++) {
		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		if (data[i + 3] >= 1 && data[i + 3] <= 0xaf) {
			u32 end;

			for (end = i + 4; end + 2 < size; end++)
				if (!data[end] && !data[end + 1] && data[end + 2] == 1)
					break;
			if (end + 2 >= size)
				end = size;
			if (!mpeg2_parse_slice(data, i, end, seq->vertical_size, wmb,
					       &m->slices[n]) &&
			    m->slices[n].start_mb < wmb * hmb &&
			    (!n || m->slices[n].start_mb > m->slices[n - 1].start_mb))
				n++;
			i = end - 1;
			continue;
		}
		i += 2;
	}
	if (!n) {
		ret = -EINVAL;
		goto out;
	}

	/* references: the current picture stands in for missing ones */
	cur = dbuf->map.iova;
	fwd = bwd = cur;
	m->field_coded[dst->vb2_buf.index] = field;
	fwd_fld = bwd_fld = field;
	if (pic->picture_coding_type != V4L2_MPEG2_PIC_CODING_TYPE_I) {
		fi = mpeg2_find_buf(ctx, pic->forward_ref_ts);
		if (fi >= 0) {
			fwd = vb2_to_hivdec_buf(vb2_get_buffer(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx),
							       fi))->map.iova;
			fwd_fld = m->field_coded[fi];
		}
	}
	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		bi = mpeg2_find_buf(ctx, pic->backward_ref_ts);
		if (bi >= 0) {
			bwd = vb2_to_hivdec_buf(vb2_get_buffer(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx),
							       bi))->map.iova;
			bwd_fld = m->field_coded[bi];
		}
	}

	/* picture message */
	msg = hivdec_msg_slot(ctx, HIVDEC_SLOT_PIC);
	memset(msg, 0, 256 + (n + 1) * MPEG2_SLICE_MSG_WORDS * 4);
	msg[0] = (wmb - 1) | (hmb - 1) << 16;
	msg[1] = (u32)!!(pic->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT) |
		 (pic->picture_structure & 3) << 8 | second << 10 |
		 !!(pic->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV) << 16 |
		 (pic->picture_coding_type & 7) << 24;
	msg[2] = (pic->f_code[1][1] & 15) | (pic->f_code[1][0] & 15) << 8 |
		 (pic->f_code[0][1] & 15) << 16 | (pic->f_code[0][0] & 15) << 24 |
		 (u32)!!(pic->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST) << 31;
	msg[3] = (pic->intra_dc_precision & 3) |
		 !!(pic->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE) << 8 |
		 !!(pic->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC) << 16 |
		 !!(pic->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN) << 24;
	msg[4] = (bwd + 15) >> 4;
	msg[5] = (fwd + 15) >> 4;
	msg[6] = (cur + 15) >> 4;
	msg[7] = (m->mv + 15) >> 4;
	/* picture-level stream: the first slice */
	msg[9] = ((m->slices[0].bits + 24) & 0xffffff) |
		 (((sbuf->map.iova + m->slices[0].bitpos / 8) & 15) * 8 +
		  m->slices[0].bitpos % 8) << 24;
	{
		u8 iq[64], nq[64];

		for (i = 0; i < 64; i++) {
			iq[mpeg2_zigzag[i]] = q->intra_quantiser_matrix[i];
			nq[mpeg2_zigzag[i]] = q->non_intra_quantiser_matrix[i];
		}
		for (i = 0; i < 8; i++)
			for (j = 0; j < 2; j++) {
				u32 k = i + 8 * j;

				msg[16 + 2 * i + j] = iq[k] | iq[k + 16] << 8 | iq[k + 32] << 16 |
						      (u32)iq[k + 48] << 24;
				msg[32 + 2 * i + j] = nq[k] | nq[k + 16] << 8 | nq[k + 32] << 16 |
						      (u32)nq[k + 48] << 24;
			}
	}
	msg[48] = HIVDEC_ADDR(m->pmv_top);
	slot_iova = ctx->msg.map.iova + HIVDEC_SLOT_PIC * HIVDEC_MSG_SLOT_BYTES + 256;
	msg[63] = HIVDEC_ADDR(slot_iova);

	/* slice messages; a dummy one conceals macroblocks before the first slice */
	base = sbuf->map.iova & ~15;
	slc = msg + 64;
	if (m->slices[0].start_mb) {
		u32 a = sbuf->map.iova + m->slices[0].bitpos / 8;

		slc[0] = 1 | ((a & 15) * 8 + m->slices[0].bitpos % 8) << 24;
		slc[1] = ((a & ~15) - base) >> 4;
		slc[4] = (m->slices[0].qscale & 0x3f) | m->slices[0].intra << 6;
		slc[5] = 0;
		slc[6] = m->slices[0].start_mb - 1;
		slc[7] = HIVDEC_ADDR(slot_iova + 32);
		pre = 1;
	}
	for (i = 0; i < n; i++) {
		const struct hivdec_mpeg2_slice *s = &m->slices[i];
		u32 *d = slc + MPEG2_SLICE_MSG_WORDS * (i + pre);
		u32 a = sbuf->map.iova + s->bitpos / 8;

		d[0] = (s->bits & 0xffffff) | ((a & 15) * 8 + s->bitpos % 8) << 24;
		d[1] = (((a & ~15) - base) >> 4) & 0xffffff;
		d[4] = (s->qscale & 0x3f) | s->intra << 6;
		d[5] = s->start_mb;
		if (i + 1 < n) {
			d[6] = m->slices[i + 1].start_mb - 1;
			d[7] = HIVDEC_ADDR(slot_iova + 32 * (i + 1 + pre));
		} else {
			d[6] = wmb * hmb - 1;
			d[7] = 0;
		}
	}

	/* registers */
	vdh_write(vdec, VDH_BASIC_CFG0, CFG0_MBAMT_TO_DEC(wmb * hmb - 1) | CFG0_MARKER_BIT_DETECT |
		  CFG0_COEF_IDX_DETECT | CFG0_LOAD_QMATRIX);
	vdh_write(vdec, VDH_BASIC_CFG1, CFG1_VIDEO_STANDARD(HIVDEC_STD_MPEG2) | CFG1_MFD_MMU_EN |
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
	vdh_write(vdec, VDH_YSTADDR_1D, HIVDEC_ADDR(cur));
	vdh_write(vdec, VDH_YSTRIDE_1D, stride * 8);
	vdh_write(vdec, VDH_UVOFFSET_1D, uvoff);
	vdh_write(vdec, VDH_UVSTRIDE_1D, (stride * 4) & 0x3ffff);
	vdh_write(vdec, VDH_HEAD_INF_OFFSET, 0);
	vdh_write(vdec, VDH_YSTRIDE_2BIT, 0);
	vdh_write(vdec, VDH_YOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_UVOFFSET_2BIT, 0);
	vdh_write(vdec, VDH_REF_PIC_TYPE, fwd_fld | bwd_fld << 2);
	vdh_write(vdec, VDH_FF_APT_EN, 0);
	memset(hivdec_msg_slot(ctx, HIVDEC_SLOT_HEAD), 0, HIVDEC_MSG_SLOT_BYTES);
	vdh_write(vdec, VDH_CFGINFO_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_HEAD));
	vdh_write(vdec, VDH_DDR_INTERLEAVE_MODE, 3);
	vdh_write(vdec, SCD_EMAR_ID, 0x101);
	vdh_write(vdec, SCD_AVS_FLAG, 0);
	vdh_write(vdec, SCD_VDH_SELRST, 1);

	if (vdec->debug)
		dev_info(vdec->dev, "mpeg2 %ux%u type %u struct %u%s slices %u first mb %u\n",
			 seq->horizontal_size, seq->vertical_size, pic->picture_coding_type,
			 pic->picture_structure, second ? " (2nd)" : "", n, m->slices[0].start_mb);
out:
	if (req)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	if (ret)
		return ret;
	hivdec_hw_run(ctx);
	return 0;
}

const struct hivdec_coded_fmt_ops hivdec_mpeg2_fmt_ops = {
	.adjust_fmt = hivdec_mpeg2_adjust_fmt,
	.start = hivdec_mpeg2_start,
	.stop = hivdec_mpeg2_stop,
	.run = hivdec_mpeg2_run,
	.try_ctrl = hivdec_mpeg2_try_ctrl,
};
