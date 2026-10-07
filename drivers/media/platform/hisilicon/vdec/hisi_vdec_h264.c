// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video decoder: H.264
 *
 * Slice-based decoding: each request carries one slice. The slices of a
 * picture are gathered into a stream buffer and the VDH decodes the whole
 * picture in one run, when the slice without V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF
 * arrives. Message layout of the HiSilicon VFMW H.264 HAL (VDH V500R003).
 *
 * The VDH does not skip emulation prevention bytes (the VFMW start code
 * detector strips them while copying the stream), so slices are copied as
 * RBSP; the driver parses the slice header itself to find where slice_data()
 * starts instead of relying on header_bit_size conventions.
 */

#include <linux/bits.h>
#include <linux/string.h>
#include <media/v4l2-h264.h>
#include <media/v4l2-mem2mem.h>

#include "hisi_vdec.h"
#include "hisi_vdec_tables.h"

#define H264_MAX_SLICES		(HIVDEC_MAX_SLICES - 2)	/* + dummy first slice */
#define H264_APC_SLOTS		16
#define H264_STREAM_SIZE_MIN	SZ_4M

/* hardware line buffers, sized as the vendor HAL does for 8192 wide VP9 */
#define H264_CABAC_LEN		(64 * 4 * 20)
#define H264_LINE_BUF_LEN	(64 * 4 * 8192)
#define H264_HEAD_LEN		SZ_256K

static const u8 h264_struct_trans[8] = { 0, 3, 1, 1, 2, 2, 3, 3 };
static const u8 h264_list_struct_frame[8] = { 0, 1, 3, 3, 3, 3, 2, 2 };
static const u8 h264_list_struct_field[8] = { 0, 1, 2, 2, 2, 2, 2, 2 };

enum { H264_P = 0, H264_B = 1, H264_I = 2 };	/* VFMW slice types */

struct hivdec_h264_slice {
	u32 offset;		/* NAL header byte, in the stream buffer */
	u32 size;		/* NAL size in bytes */
	u32 data_bits;		/* raw bit position of slice_data() from the NAL header */
	struct v4l2_ctrl_h264_slice_params sp;
	struct v4l2_ctrl_h264_pred_weights pw;
};

/* decoder state of each CAPTURE buffer, indexed by vb2 buffer index */
struct hivdec_h264_buf_meta {
	s8 apc;			/* APC slot while the frame is in the DPB, else -1 */
	u8 decoded;		/* fields decoded: 1 top, 2 bottom, 3 both / frame */
	u8 field_coded;		/* decoded as field picture(s) */
	u64 ts;			/* timestamp of the frame the state belongs to */
};

struct hivdec_h264_ctx {
	struct hivdec_aux_buf stream;
	u32 stream_used;
	unsigned int nslices;
	struct hivdec_h264_slice *slices;
	struct hivdec_aux_buf work;
	u32 cabac, sed_top, pmv_top, rcn_top, dblk_top, head;	/* IOVAs */
	struct hivdec_h264_buf_meta meta[VB2_MAX_FRAME];
	s8 apc[H264_APC_SLOTS];		/* vb2 index in each APC slot, -1 = free */
	/* picture level controls of the slices being gathered */
	struct v4l2_ctrl_h264_sps sps;
	struct v4l2_ctrl_h264_pps pps;
	struct v4l2_ctrl_h264_scaling_matrix sm;
	struct v4l2_ctrl_h264_decode_params dp;
	bool sm_valid;
	bool carry;	/* current slice is taken after the running decode */
	bool error;	/* a decode into the current CAPTURE buffer failed */
};


/* ---------------------------------------------------------------- bit reader */

/* bit reader over the RBSP (emulation prevention bytes already removed) */
struct h264_bits {
	const u8 *p;
	u32 len;	/* bytes */
	u32 pos;	/* bit position */
	bool error;
};

static void h264_bits_init(struct h264_bits *b, const u8 *p, u32 len)
{
	b->p = p;
	b->len = len;
	b->pos = 0;
	b->error = false;
}

static u32 h264_bit(struct h264_bits *b)
{
	u32 byte = b->pos >> 3;
	u32 v;

	if (byte >= b->len) {
		b->error = true;
		return 0;
	}
	v = (b->p[byte] >> (7 - (b->pos & 7))) & 1;
	b->pos++;
	return v;
}

static u32 h264_bits_u(struct h264_bits *b, unsigned int n)
{
	u32 v = 0;

	while (n--)
		v = v << 1 | h264_bit(b);
	return v;
}

static u32 h264_bits_ue(struct h264_bits *b)
{
	unsigned int lz = 0;

	while (!h264_bit(b) && !b->error)
		if (++lz > 31) {
			b->error = true;
			return 0;
		}
	return (1U << lz) - 1 + h264_bits_u(b, lz);
}

static s32 h264_bits_se(struct h264_bits *b)
{
	u32 v = h264_bits_ue(b);

	return v & 1 ? (s32)((v + 1) >> 1) : -(s32)(v >> 1);
}

/*
 * Walk the slice header (7.3.3) and return the bit position of slice_data()
 * in the RBSP, counted from the NAL header byte.
 */
static int h264_slice_data_bits(const struct hivdec_h264_ctx *h, const u8 *nal, u32 len,
				const struct v4l2_ctrl_h264_slice_params *sp, u32 *bits)
{
	const struct v4l2_ctrl_h264_sps *sps = &h->sps;
	const struct v4l2_ctrl_h264_pps *pps = &h->pps;
	struct h264_bits b;
	u32 nal_ref_idc, nal_type, type, i, l, n[2];
	bool field = false, chroma = sps->chroma_format_idc != 0;

	if (len < 2)
		return -EINVAL;
	nal_ref_idc = (nal[0] >> 5) & 3;
	nal_type = nal[0] & 0x1f;
	if (nal_type != 1 && nal_type != 5)
		return -EINVAL;
	h264_bits_init(&b, nal, len);
	h264_bits_u(&b, 8);				/* NAL header */
	h264_bits_ue(&b);				/* first_mb_in_slice */
	type = h264_bits_ue(&b) % 5;			/* slice_type */
	h264_bits_ue(&b);				/* pic_parameter_set_id */
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		h264_bits_u(&b, 2);
	h264_bits_u(&b, sps->log2_max_frame_num_minus4 + 4);	/* frame_num */
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		field = h264_bits_u(&b, 1);
		if (field)
			h264_bits_u(&b, 1);		/* bottom_field_flag */
	}
	if (nal_type == 5)
		h264_bits_ue(&b);			/* idr_pic_id */
	if (sps->pic_order_cnt_type == 0) {
		h264_bits_u(&b, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) && !field)
			h264_bits_se(&b);
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		h264_bits_se(&b);
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) && !field)
			h264_bits_se(&b);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		h264_bits_ue(&b);
	if (type == V4L2_H264_SLICE_TYPE_B)
		h264_bits_u(&b, 1);			/* direct_spatial_mv_pred_flag */
	n[0] = pps->num_ref_idx_l0_default_active_minus1 + 1;
	n[1] = pps->num_ref_idx_l1_default_active_minus1 + 1;
	if (type == V4L2_H264_SLICE_TYPE_P || type == V4L2_H264_SLICE_TYPE_SP ||
	    type == V4L2_H264_SLICE_TYPE_B) {
		if (h264_bits_u(&b, 1)) {		/* num_ref_idx_active_override_flag */
			n[0] = h264_bits_ue(&b) + 1;
			if (type == V4L2_H264_SLICE_TYPE_B)
				n[1] = h264_bits_ue(&b) + 1;
		}
	}
	/* ref_pic_list_modification() */
	for (l = 0; l < 2; l++) {
		if (l == 0 && (type == V4L2_H264_SLICE_TYPE_I || type == V4L2_H264_SLICE_TYPE_SI))
			break;
		if (l == 1 && type != V4L2_H264_SLICE_TYPE_B)
			break;
		if (h264_bits_u(&b, 1)) {
			u32 idc;

			do {
				idc = h264_bits_ue(&b);
				if (idc <= 2)
					h264_bits_ue(&b);
			} while (idc != 3 && !b.error);
		}
	}
	/* pred_weight_table() */
	if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) &&
	     (type == V4L2_H264_SLICE_TYPE_P || type == V4L2_H264_SLICE_TYPE_SP)) ||
	    (pps->weighted_bipred_idc == 1 && type == V4L2_H264_SLICE_TYPE_B)) {
		h264_bits_ue(&b);
		if (chroma)
			h264_bits_ue(&b);
		for (l = 0; l < (type == V4L2_H264_SLICE_TYPE_B ? 2U : 1U); l++)
			for (i = 0; i < n[l] && !b.error; i++) {
				if (h264_bits_u(&b, 1)) {
					h264_bits_se(&b);
					h264_bits_se(&b);
				}
				if (chroma && h264_bits_u(&b, 1)) {
					h264_bits_se(&b);
					h264_bits_se(&b);
					h264_bits_se(&b);
					h264_bits_se(&b);
				}
			}
	}
	/* dec_ref_pic_marking() */
	if (nal_ref_idc) {
		if (nal_type == 5) {
			h264_bits_u(&b, 2);
		} else if (h264_bits_u(&b, 1)) {
			u32 op;

			do {
				op = h264_bits_ue(&b);
				if (op == 1 || op == 3)
					h264_bits_ue(&b);
				if (op == 2 || op == 3 || op == 4 || op == 6)
					h264_bits_ue(&b);
			} while (op && !b.error);
		}
	}
	if ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
	    type != V4L2_H264_SLICE_TYPE_I && type != V4L2_H264_SLICE_TYPE_SI)
		h264_bits_ue(&b);			/* cabac_init_idc */
	h264_bits_se(&b);				/* slice_qp_delta */
	if (type == V4L2_H264_SLICE_TYPE_SP || type == V4L2_H264_SLICE_TYPE_SI) {
		if (type == V4L2_H264_SLICE_TYPE_SP)
			h264_bits_u(&b, 1);
		h264_bits_se(&b);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		if (h264_bits_ue(&b) != 1) {
			h264_bits_se(&b);
			h264_bits_se(&b);
		}
	}
	if (pps->num_slice_groups_minus1)
		return -EOPNOTSUPP;			/* FMO */
	if (b.error)
		return -EINVAL;
	*bits = b.pos;
	/* CABAC: slice_data() starts byte aligned (cabac_alignment_one_bit) */
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		*bits = ALIGN(*bits, 8);
	return 0;
}

/* bits up to and including the rbsp_stop_one_bit, trailing zero bytes excluded */
static u32 h264_payload_bits(const u8 *p, u32 len, bool cabac)
{
	while (len && !p[len - 1])
		len--;
	if (!len)
		return 0;
	/* CAVLC: drop the stop bit and the alignment zeros; CABAC keeps the stop bit */
	return len * 8 - __builtin_ctz(p[len - 1]) - (cabac ? 0 : 1);
}

/* ---------------------------------------------------------------- setup */

static int hivdec_h264_start(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_h264_ctx *h;
	u32 base, off;
	int i, ret;

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->slices = kvcalloc(H264_MAX_SLICES, sizeof(*h->slices), GFP_KERNEL);
	if (!h->slices) {
		ret = -ENOMEM;
		goto err_free;
	}
	ret = hivdec_aux_alloc(vdec, &h->stream,
			       max_t(u32, H264_STREAM_SIZE_MIN,
				     ctx->coded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage * 2));
	if (ret)
		goto err_slices;
	ret = hivdec_aux_alloc(vdec, &h->work, SZ_8K + 4 * H264_LINE_BUF_LEN + H264_HEAD_LEN);
	if (ret)
		goto err_stream;

	base = h->work.map.iova;
	memcpy(h->work.cpu, hivdec_h264_cabac_mn, H264_CABAC_LEN);
	h->cabac = base;
	off = SZ_8K;
	h->sed_top = base + off;
	off += H264_LINE_BUF_LEN;
	h->pmv_top = base + off;
	off += H264_LINE_BUF_LEN;
	h->rcn_top = base + off;
	off += H264_LINE_BUF_LEN;
	h->dblk_top = base + off;
	off += H264_LINE_BUF_LEN;
	h->head = base + off;		/* dummy compression headers */
	hivdec_aux_sync_for_device(vdec, &h->work);

	for (i = 0; i < H264_APC_SLOTS; i++)
		h->apc[i] = -1;
	for (i = 0; i < VB2_MAX_FRAME; i++)
		h->meta[i].apc = -1;

	ctx->priv = h;
	return 0;

err_stream:
	hivdec_aux_free(vdec, &h->stream);
err_slices:
	kvfree(h->slices);
err_free:
	kfree(h);
	return ret;
}

static void hivdec_h264_stop(struct hivdec_ctx *ctx)
{
	struct hivdec_h264_ctx *h = ctx->priv;

	if (!h)
		return;
	hivdec_aux_free(ctx->dev, &h->work);
	hivdec_aux_free(ctx->dev, &h->stream);
	kvfree(h->slices);
	kfree(h);
	ctx->priv = NULL;
}

static int hivdec_h264_adjust_fmt(struct hivdec_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 3 / 2;
	fmt->plane_fmt[0].sizeimage = max_t(u32, fmt->plane_fmt[0].sizeimage, SZ_512K);
	return 0;
}

static int hivdec_h264_try_ctrl(struct hivdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_H264_SPS) {
		const struct v4l2_ctrl_h264_sps *sps = ctrl->p_new.p_h264_sps;

		if (sps->chroma_format_idc > 1)
			return -EINVAL;	/* 4:2:0 and monochrome only */
		if (sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8)
			return -EINVAL;
		if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
			return -EINVAL;
		if (sps->pic_width_in_mbs_minus1 >= 256 ||
		    sps->pic_height_in_map_units_minus1 >= 256)
			return -EINVAL;
	}
	if (ctrl->id == V4L2_CID_STATELESS_H264_PPS) {
		if (ctrl->p_new.p_h264_pps->num_slice_groups_minus1)
			return -EINVAL;	/* no FMO/ASO */
	}
	return 0;
}

/* ---------------------------------------------------------------- slices */

static const u8 *hivdec_h264_skip_start_code(const u8 *p, u32 *len)
{
	u32 i = 0;

	while (i + 2 < *len && !p[i] && !p[i + 1]) {
		if (p[i + 2] == 1) {
			*len -= i + 3;
			return p + i + 3;
		}
		if (p[i + 2])
			break;
		i++;
	}
	return p;
}

/* copy a NAL unit dropping emulation_prevention_three_byte (7.4.1) */
static u32 hivdec_nal_to_rbsp(u8 *dst, const u8 *src, u32 len)
{
	u32 i, n = 0, zeros = 0;

	for (i = 0; i < len; i++) {
		if (zeros >= 2 && src[i] == 0x03) {
			zeros = 0;
			continue;
		}
		zeros = src[i] ? 0 : zeros + 1;
		dst[n++] = src[i];
	}
	return n;
}

static int hivdec_h264_gather(struct hivdec_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	struct hivdec_h264_slice *s;
	const struct v4l2_ctrl_h264_pred_weights *pw;
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 len = vb2_get_plane_payload(&src->vb2_buf, 0);
	int ret;

	if (!data)
		return -EFAULT;
	if (h->nslices >= H264_MAX_SLICES)
		return -ENOSPC;

	if (!h->nslices) {
		const void *sm = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX);

		h->sps = *(struct v4l2_ctrl_h264_sps *)
			 hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_SPS);
		h->pps = *(struct v4l2_ctrl_h264_pps *)
			 hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_PPS);
		h->dp = *(struct v4l2_ctrl_h264_decode_params *)
			hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
		h->sm_valid = sm != NULL;
		if (sm)
			h->sm = *(struct v4l2_ctrl_h264_scaling_matrix *)sm;
		h->stream_used = 0;
	}

	data = hivdec_h264_skip_start_code(data, &len);
	if (h->stream_used + len + 64 > h->stream.size)
		return -ENOSPC;

	s = &h->slices[h->nslices];
	s->sp = *(struct v4l2_ctrl_h264_slice_params *)
		hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	pw = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_PRED_WEIGHTS);
	if (pw)
		s->pw = *pw;
	/* the VDH reads RBSP: drop emulation prevention bytes while copying */
	s->offset = h->stream_used;
	s->size = hivdec_nal_to_rbsp(h->stream.cpu + s->offset, data, len);
	len = s->size;
	ret = h264_slice_data_bits(h, h->stream.cpu + s->offset, len, &s->sp,
				   &s->data_bits);
	if (ret) {
		if (ctx->dev->debug)
			dev_info(ctx->dev->dev, "slice %u: header parse %d\n", h->nslices, ret);
		return ret;
	}
	/* zero padding keeps the entropy decoder's prefetch inside the buffer */
	memset(h->stream.cpu + s->offset + len, 0, 32);
	h->stream_used = ALIGN(s->offset + len + 32, 64);
	h->nslices++;
	return 0;
}

/* ---------------------------------------------------------------- references */

static int h264_find_buf(struct hivdec_ctx *ctx, u64 ts)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = vb2_find_buffer(q, ts);

	return vb ? vb->index : -1;
}

static struct hivdec_decoded_buffer *h264_buf(struct hivdec_ctx *ctx, int idx)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct vb2_buffer *vb = idx >= 0 ? vb2_get_buffer(q, idx) : NULL;

	return vb ? vb2_to_hivdec_buf(vb) : NULL;
}

/*
 * Keep a stable APC slot per DPB frame: the VDH records slot numbers in the
 * co-located motion vector data, so slots must not move while a frame is a
 * reference (VFMW GetAPC / RemoveFrameStoreOutDPB).
 */
static void h264_update_apc(struct hivdec_h264_ctx *h, const int dpb_idx[16])
{
	bool used[VB2_MAX_FRAME] = { };
	int i, s;

	for (i = 0; i < 16; i++)
		if (dpb_idx[i] >= 0)
			used[dpb_idx[i]] = true;
	for (s = 0; s < H264_APC_SLOTS; s++)
		if (h->apc[s] >= 0 && !used[h->apc[s]]) {
			h->meta[h->apc[s]].apc = -1;
			h->apc[s] = -1;
		}
	for (i = 0; i < 16; i++) {
		int b = dpb_idx[i];

		if (b < 0 || h->meta[b].apc >= 0)
			continue;
		for (s = 0; s < H264_APC_SLOTS && h->apc[s] >= 0; s++)
			;
		if (s == H264_APC_SLOTS)
			break;
		h->apc[s] = b;
		h->meta[b].apc = s;
	}
}

/* ---------------------------------------------------------------- messages */

static void h264_build_qmatrix(u32 *q, const struct hivdec_h264_ctx *h)
{
	const struct v4l2_ctrl_h264_scaling_matrix *sm = &h->sm;
	int l, r;

	if (!h->sm_valid || !(h->pps.flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT)) {
		for (l = 0; l < 56; l++)
			q[l] = 0x10101010;
		return;
	}
	/* per row: even columns first, then odd columns */
	for (l = 0; l < 6; l++)
		for (r = 0; r < 4; r++) {
			const u8 *m = &sm->scaling_list_4x4[l][4 * r];

			q[4 * l + r] = m[0] | m[2] << 8 | m[1] << 16 | (u32)m[3] << 24;
		}
	for (l = 0; l < 2; l++)
		for (r = 0; r < 8; r++) {
			const u8 *m = &sm->scaling_list_8x8[l][8 * r];

			q[24 + 16 * l + 2 * r] = m[0] | m[2] << 8 | m[4] << 16 | (u32)m[6] << 24;
			q[24 + 16 * l + 2 * r + 1] = m[1] | m[3] << 8 | m[5] << 16 | (u32)m[7] << 24;
		}
}

struct h264_pic {
	u32 width_mb, height_mb, frame_height_mb, mbtodec;
	u8 structure;		/* 0 frame, 1 top, 2 bottom */
	bool mbaff_sps, mbaff, field;
	u32 luma, mv, mv_half;
	int dpb_idx[16];
	u32 frame_bytes, w64, ah;
};

static u32 h264_mv_size(const struct h264_pic *pic)
{
	return ALIGN(pic->width_mb * pic->frame_height_mb * 64, 128);
}

static void h264_ref_entry(struct hivdec_ctx *ctx, const struct h264_pic *pic,
			   const struct v4l2_h264_reference *r, int i, u32 *msg, int L)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	const struct hivdec_h264_buf_meta *m = NULL;
	u32 st = 0, idc = 0;
	bool lt = false;
	int b = -1;

	if (r->index < 16) {
		b = pic->dpb_idx[r->index];
		lt = h->dp.dpb[r->index].flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
	}
	if (b >= 0 && h->meta[b].apc >= 0)
		m = &h->meta[b];
	if (m) {
		/* frame_structure: 0 coded as frame, 3 coded as field pair */
		u8 fs = m->field_coded ? 3 : 0;

		idc = (m->apc & 15) << 1;
		if (!pic->field) {
			st = (h264_list_struct_frame[pic->mbaff_sps + fs * 2] & 3) | (u32)lt << 3;
		} else if (r->fields == V4L2_H264_BOTTOM_FIELD_REF) {
			u8 s = m->decoded == 3 ? fs : 2;

			st = (h264_list_struct_field[pic->mbaff_sps + s * 2] & 3) | (u32)lt << 3;
			idc |= 1;
		} else {
			u8 s = m->decoded == 3 ? fs : 1;

			st = (h264_list_struct_field[pic->mbaff_sps + s * 2] & 3) | BIT(2) | (u32)lt << 3;
		}
	}
	msg[12 + 4 * L + i / 8] |= st << (4 * (i % 8));
	msg[20 + 8 * L + i / 4] |= idc << (5 * (i % 4));
}

static u32 h264_mv_addr_of(struct hivdec_ctx *ctx, int vb_idx)
{
	struct hivdec_decoded_buffer *d = h264_buf(ctx, vb_idx);

	return d && d->mv.priv ? d->mv.map.iova : 0;
}

static void h264_slice_msg(struct hivdec_ctx *ctx, const struct h264_pic *pic,
			   const struct hivdec_h264_slice *s, u32 *msg, u32 next_slot,
			   u32 end_mb, u32 stream_base, bool dummy)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	const struct v4l2_ctrl_h264_slice_params *sp = &s->sp;
	const struct v4l2_ctrl_h264_pps *pps = &h->pps;
	static const u8 type_map[5] = { H264_P, H264_B, H264_I, H264_P, H264_I };
	u8 t = type_map[sp->slice_type % 5];
	u32 pmv = t == H264_P ? 1 : t == H264_B ? 2 : 0;
	u32 nl0 = t != H264_I ? sp->num_ref_idx_l0_active_minus1 + 1 : 0;
	u32 nl1 = t == H264_B ? sp->num_ref_idx_l1_active_minus1 + 1 : 0;
	bool cabac = pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
	u32 weight_flag, i;

	memset(msg, 0, 256 * 4);

	/* stream position of slice_data() */
	if (dummy) {
		msg[0] = 0;
		msg[1] = 0;
		msg[2] = 1;
	} else {
		u32 addr = h->stream.map.iova + s->offset + s->data_bits / 8;
		u32 bits = s->size * 8 > s->data_bits ?
			   h264_payload_bits(h->stream.cpu + s->offset, s->size, cabac) - s->data_bits : 0;

		msg[0] = ((addr & ~15) - (stream_base & ~15)) >> 4;
		msg[1] = ((addr & 15) * 8 + (s->data_bits & 7)) & 127;
		msg[2] = bits;
	}

	msg[6] = (sp->first_mb_in_slice & 0xfffff) | (sp->cabac_init_idc & 3) << 24 |
		 (u32)(26 + pps->pic_init_qp_minus26 + sp->slice_qp_delta) << 26;
	msg[7] = pmv | nl0 << 2 | nl1 << 8 |
		 !!(h->sps.flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE) << 14 |
		 !!(sp->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED) << 15 |
		 ((nl0 ? nl0 - 1 : 0) & 31) << 16 | ((nl1 ? nl1 - 1 : 0) & 31) << 21;

	/* co-located motion vectors: list1[0] */
	if (t == H264_B && sp->ref_pic_list1[0].index < 16) {
		int b = pic->dpb_idx[sp->ref_pic_list1[0].index];
		u32 a = h264_mv_addr_of(ctx, b);

		if (a && b >= 0) {
			const struct hivdec_h264_buf_meta *m = &h->meta[b];
			u32 half = pic->mv_half;

			if (!pic->field) {
				msg[8] = (a & ~15) >> 4;
				if (m->field_coded)
					msg[9] = ((a + half) & ~15) >> 4;
			} else if ((m->decoded == 3 && !m->field_coded) ||
				   sp->ref_pic_list1[0].fields != V4L2_H264_BOTTOM_FIELD_REF) {
				msg[8] = (a & ~15) >> 4;
			} else {
				msg[8] = ((a + half) & ~15) >> 4;
			}
		}
	}

	weight_flag = t == H264_P ? !!(pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) :
		      t == H264_B ? pps->weighted_bipred_idc : 0;
	msg[10] = (pps->second_chroma_qp_index_offset & 31) |
		  (pps->chroma_qp_index_offset & 31) << 5 | weight_flag << 16;
	msg[11] = sp->disable_deblocking_filter_idc | (sp->slice_beta_offset_div2 & 15) << 8 |
		  (sp->slice_alpha_c0_offset_div2 & 15) << 16;
	msg[44] = end_mb;
	msg[63] = next_slot;

	if (t == H264_I)
		return;

	for (i = 0; i < nl0; i++)
		h264_ref_entry(ctx, pic, &sp->ref_pic_list0[i], i, msg, 0);
	for (i = 0; i < nl1; i++)
		h264_ref_entry(ctx, pic, &sp->ref_pic_list1[i], i, msg, 1);

	/* Apc2RefIdx: lowest list0 index referencing each APC slot (field) */
	{
		u32 map[32] = { };
		int k;

		for (k = nl0 - 1; k >= 0; k--) {
			const struct v4l2_h264_reference *r = &sp->ref_pic_list0[k];
			int b = r->index < 16 ? pic->dpb_idx[r->index] : -1;
			int slot = b >= 0 ? h->meta[b].apc : -1;

			if (slot < 0)
				continue;
			if (pic->field) {
				map[2 * slot + (r->fields == V4L2_H264_BOTTOM_FIELD_REF)] = k;
			} else {
				map[2 * slot] = k;
				map[2 * slot + 1] = k;
			}
		}
		for (k = 0; k < 32; k++)
			msg[36 + k / 4] |= (map[k] & 31) << (5 * (k % 4));
	}

	/* explicit weighted prediction */
	if ((t == H264_P && (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED)) ||
	    (t == H264_B && pps->weighted_bipred_idc == 1)) {
		const struct v4l2_ctrl_h264_pred_weights *pw = &s->pw;
		u32 ld = pw->luma_log2_weight_denom & 7, cd = pw->chroma_log2_weight_denom & 7;
		int L;

		for (L = 0; L < (t == H264_B ? 2 : 1); L++) {
			const struct v4l2_h264_weight_factors *f = &pw->weight_factors[L];
			u32 n = L ? nl1 : nl0;

			for (i = 0; i < n; i++) {
				msg[64 + 32 * L + i] = ld | (f->luma_weight[i] & 511) << 3 |
						       (u32)(u8)f->luma_offset[i] << 12;
				msg[128 + 32 * L + i] = cd | (f->chroma_weight[i][0] & 511) << 3 |
							(u32)(u8)f->chroma_offset[i][0] << 12;
				msg[192 + 32 * L + i] = (f->chroma_weight[i][1] & 511) |
							(u32)(u8)f->chroma_offset[i][1] << 9;
			}
		}
	}
}

static int hivdec_h264_decode(struct hivdec_ctx *ctx, struct vb2_v4l2_buffer *dst)
{
	struct hivdec_dev *vdec = ctx->dev;
	struct hivdec_h264_ctx *h = ctx->priv;
	struct hivdec_decoded_buffer *dbuf = vb2_to_hivdec_buf(&dst->vb2_buf);
	const struct v4l2_ctrl_h264_sps *sps = &h->sps;
	const struct v4l2_ctrl_h264_pps *pps = &h->pps;
	const struct v4l2_ctrl_h264_decode_params *dp = &h->dp;
	struct hivdec_h264_buf_meta *cur;
	struct h264_pic pic = { };
	u32 *msg, *head, stream_base, ystride, cfg0, cfg1, emar;
	unsigned int i, k, n, slot, first;
	bool fmo = sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;
	bool dummy;
	int ret;

	pic.width_mb = sps->pic_width_in_mbs_minus1 + 1;
	pic.frame_height_mb = (sps->pic_height_in_map_units_minus1 + 1) * (fmo ? 1 : 2);
	pic.field = dp->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
	pic.structure = pic.field ? (dp->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD ? 2 : 1) : 0;
	pic.height_mb = pic.frame_height_mb >> pic.field;
	pic.mbaff_sps = sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
	pic.mbaff = pic.mbaff_sps && !pic.field;
	pic.mbtodec = pic.width_mb * pic.height_mb;
	pic.w64 = ALIGN(pic.width_mb * 16, 64);
	pic.ah = ALIGN(pic.frame_height_mb * 16, 32);
	ystride = pic.w64 * 8;

	if (pic.w64 > ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline ||
	    pic.w64 * pic.ah * 3 / 2 > vb2_plane_size(&dst->vb2_buf, 0)) {
		dev_err(vdec->dev, "capture buffer too small for %ux%u MBs\n",
			pic.width_mb, pic.frame_height_mb);
		return -EINVAL;
	}

	/* co-located MV buffer of the target frame */
	if (!dbuf->mv.priv || dbuf->mv.size < h264_mv_size(&pic)) {
		hivdec_aux_free(vdec, &dbuf->mv);
		ret = hivdec_aux_alloc(vdec, &dbuf->mv, h264_mv_size(&pic));
		if (ret)
			return ret;
	}
	pic.luma = dbuf->map.iova;
	pic.mv = dbuf->mv.map.iova;
	pic.mv_half = h264_mv_size(&pic) / 2;

	/* DPB -> capture buffers -> stable APC slots */
	for (i = 0; i < 16; i++) {
		pic.dpb_idx[i] = -1;
		if (dp->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)
			pic.dpb_idx[i] = h264_find_buf(ctx, dp->dpb[i].reference_ts);
	}
	cur = &h->meta[dst->vb2_buf.index];
	if (cur->ts != dst->vb2_buf.timestamp || !pic.field) {
		/* new frame in this buffer: forget what it held before */
		cur->ts = dst->vb2_buf.timestamp;
		cur->decoded = 0;
		cur->field_coded = pic.field;
	}
	h264_update_apc(h, pic.dpb_idx);

	/* picture message */
	msg = hivdec_msg_slot(ctx, HIVDEC_SLOT_PIC);
	memset(msg, 0, HIVDEC_MSG_SLOT_BYTES);
	msg[0] = (pic.width_mb - 1) |
		 (u32)h264_struct_trans[pic.structure * 2 + pic.mbaff_sps] << 14 |
		 (pic.height_mb - 1) << 16 |
		 (sps->chroma_format_idc == 1) << 25 |
		 !!(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 26 |
		 !!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) << 27 |
		 !!(pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) << 28 |
		 (u32)(dp->nal_ref_idc != 0) << 31;
	msg[1] = (pic.luma + 15) >> 4;
	if (pic.field)
		msg[2] = pic.structure == 2 ? dp->bottom_field_order_cnt : dp->top_field_order_cnt;
	else
		msg[2] = min(dp->top_field_order_cnt, dp->bottom_field_order_cnt);
	if (pic.mbaff_sps) {
		msg[3] = dp->top_field_order_cnt;
		msg[4] = dp->bottom_field_order_cnt;
	}
	msg[5] = h->sed_top >> 4;
	msg[6] = h->pmv_top >> 4;
	msg[7] = ((pic.mv + (pic.structure == 2 ? pic.mv_half : 0)) & ~15) >> 4;
	msg[8] = h->rcn_top >> 4;
	msg[9] = hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_SLICE0);
	for (i = 0; i < H264_APC_SLOTS; i++) {
		struct hivdec_decoded_buffer *r = h264_buf(ctx, h->apc[i]);

		msg[10 + i] = (r ? r->map.iova : pic.luma) >> 4;
	}
	msg[26] = h->cabac >> 4;
	for (i = 0; i < 16; i++) {
		int b = pic.dpb_idx[i];
		s32 top = dp->dpb[i].top_field_order_cnt;
		s32 bot = dp->dpb[i].bottom_field_order_cnt;
		int s;

		if (b < 0 || (s = h->meta[b].apc) < 0)
			continue;
		if (dp->dpb[i].fields == V4L2_H264_TOP_FIELD_REF)
			bot = top;
		else if (dp->dpb[i].fields == V4L2_H264_BOTTOM_FIELD_REF)
			top = bot;
		msg[27 + 2 * s] = top;
		msg[28 + 2 * s] = bot;
	}
	msg[59] = h->dblk_top >> 4;
	h264_build_qmatrix(&msg[64], h);

	/* head (compression header) message: dummy buffers, compression is off */
	head = hivdec_msg_slot(ctx, HIVDEC_SLOT_HEAD);
	memset(head, 0, HIVDEC_MSG_SLOT_BYTES);
	head[0] = h->head >> 4;
	head[1] = (h->head + H264_HEAD_LEN / 2) >> 4;
	head[2] = ALIGN(pic.w64 / 16, 32);
	for (i = 0; i < 16; i++) {
		head[3 + 2 * i] = head[0];
		head[4 + 2 * i] = head[1];
	}

	/* slice messages, chained from slot SLICE0 */
	stream_base = h->stream.map.iova;
	first = h->slices[0].sp.first_mb_in_slice;
	dummy = first != 0;
	slot = HIVDEC_SLOT_SLICE0;
	if (dummy) {
		h264_slice_msg(ctx, &pic, &h->slices[0], hivdec_msg_slot(ctx, slot),
			       hivdec_msg_slot_addr(ctx, slot + 1), first - 1, stream_base, true);
		slot++;
	}
	for (k = 0, n = 0; k < h->nslices; k++) {
		u32 end_mb, next;
		unsigned int j;

		/* next slice with a larger first_mb (out of order slices are dropped) */
		for (j = k + 1; j < h->nslices &&
		     h->slices[j].sp.first_mb_in_slice <= h->slices[k].sp.first_mb_in_slice; j++)
			;
		if (j < h->nslices) {
			end_mb = h->slices[j].sp.first_mb_in_slice - 1;
			next = hivdec_msg_slot_addr(ctx, slot + 1);
		} else {
			end_mb = (min(pic.mbtodec, 0x40000U) - 1) / (pic.mbaff + 1);
			next = 0;
		}
		h264_slice_msg(ctx, &pic, &h->slices[k], hivdec_msg_slot(ctx, slot), next,
			       end_mb, stream_base, false);
		slot++;
		n++;
		k = j - 1;
	}

	/* registers */
	cfg0 = CFG0_MBAMT_TO_DEC(min(pic.mbtodec, 0x40000U) - 1) | CFG0_COEF_IDX_DETECT |
	       CFG0_LOAD_QMATRIX;
	cfg1 = CFG1_VIDEO_STANDARD(HIVDEC_STD_H264) | CFG1_MFD_MMU_EN | CFG1_FST_SLC_GRP |
	       (dp->nal_ref_idc ? CFG1_MV_OUTPUT_EN : 0) | CFG1_MAX_SLCGRP_NUM(2) |
	       CFG1_VDH_2D_EN | CFG1_UV_ORDER_EN;	/* NV12 (default order is CrCb) */
	emar = pic.width_mb <= 119 || (pic.width_mb <= 256 && !pic.mbaff);

	vdh_write(vdec, VDH_BASIC_CFG0, cfg0);
	vdh_write(vdec, VDH_BASIC_CFG1, cfg1);
	vdh_write(vdec, VDH_AVM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_PIC));
	vdh_write(vdec, VDH_VAM_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_UP));
	vdh_write(vdec, VDH_STREAM_BASE_ADDR, (stream_base & ~15) >> 4);
	vdh_write(vdec, SCD_EMAR_ID, (vdh_read(vdec, SCD_EMAR_ID) & ~SCD_EMAR_ID_ONCHIP) |
		  (emar ? SCD_EMAR_ID_ONCHIP : 0));
	vdh_write(vdec, VDH_SED_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_ITRANS_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PMV_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PRC_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_RCN_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_DBLK_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_PPFD_TO, VDH_TIMEOUT_DEFAULT);
	vdh_write(vdec, VDH_YSTADDR_1D, (pic.luma & ~15) >> 4);
	vdh_write(vdec, VDH_YSTRIDE_1D, ystride);
	vdh_write(vdec, VDH_UVOFFSET_1D, pic.w64 * pic.ah);
	vdh_write(vdec, VDH_HEAD_INF_OFFSET, 0);
	{
		u32 ref_type = 0;

		for (i = 0; i < H264_APC_SLOTS; i++)
			if (h->apc[i] >= 0 && h->meta[h->apc[i]].field_coded)
				ref_type |= 1 << (2 * i);
		vdh_write(vdec, VDH_REF_PIC_TYPE, ref_type);
	}
	vdh_write(vdec, VDH_FF_APT_EN,
		  h264_struct_trans[pic.structure * 2 + pic.mbaff_sps] == 0 ? 2 : 0);
	vdh_write(vdec, VDH_UVSTRIDE_1D, (ystride / 2) & 0x3ffff);
	vdh_write(vdec, VDH_CFGINFO_ADDR, hivdec_msg_slot_addr(ctx, HIVDEC_SLOT_HEAD));
	vdh_write(vdec, VDH_DDR_INTERLEAVE_MODE, 3);
	vdh_write(vdec, SCD_AVS_FLAG, 0);
	vdh_write(vdec, SCD_VDH_SELRST, 1);

	/* the frame this picture belongs to */
	cur->decoded |= pic.field ? pic.structure : 3;

	if (vdec->debug > 2) {
		for (i = 0; i < 16; i++)
			if (dp->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)
				dev_info(vdec->dev, "  dpb[%u] buf %d apc %d fields %u flags %#x poc %d/%d fn %u\n",
					 i, pic.dpb_idx[i],
					 pic.dpb_idx[i] >= 0 ? h->meta[pic.dpb_idx[i]].apc : -9,
					 dp->dpb[i].fields, dp->dpb[i].flags,
					 dp->dpb[i].top_field_order_cnt,
					 dp->dpb[i].bottom_field_order_cnt, dp->dpb[i].frame_num);
		for (k = 0; k < h->nslices; k++) {
			const struct v4l2_ctrl_h264_slice_params *sp = &h->slices[k].sp;

			for (i = 0; i <= sp->num_ref_idx_l0_active_minus1 && sp->slice_type != 2; i++)
				dev_info(vdec->dev, "  L0[%u] = dpb %u fields %u\n", i,
					 sp->ref_pic_list0[i].index, sp->ref_pic_list0[i].fields);
			for (i = 0; sp->slice_type == 1 && i <= sp->num_ref_idx_l1_active_minus1; i++)
				dev_info(vdec->dev, "  L1[%u] = dpb %u fields %u\n", i,
					 sp->ref_pic_list1[i].index, sp->ref_pic_list1[i].fields);
		}
	}
	if (vdec->debug > 1)
		for (k = 0; k < h->nslices; k++)
			dev_info(vdec->dev, "  slice %u: first_mb %u type %u qp %d bits %u/%u\n", k,
				 h->slices[k].sp.first_mb_in_slice, h->slices[k].sp.slice_type,
				 h->slices[k].sp.slice_qp_delta, h->slices[k].data_bits,
				 h->slices[k].size * 8);
	if (vdec->debug)
		dev_info(vdec->dev, "h264 %ux%u MB %s poc %d slices %u%s\n",
			 pic.width_mb, pic.height_mb,
			 pic.field ? (pic.structure == 1 ? "top" : "bot") : "frame",
			 (s32)msg[2], n, dummy ? " +dummy" : "");

	hivdec_aux_sync_for_device(vdec, &h->stream);
	hivdec_hw_run(ctx);
	return 0;
}

/*
 * Gather the current slice and decode the picture if the slice is the last
 * one for the CAPTURE buffer: 1 = job done without hardware (CAPTURE buffer
 * held), 0 = VDH started, <0 = error.
 */
static int hivdec_h264_take_slice(struct hivdec_ctx *ctx)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	int ret;

	ret = hivdec_h264_gather(ctx, src);
	if (req)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	if (ret) {
		h->nslices = 0;
		h->error = true;
		return ret;
	}
	if (ctx->dev->debug > 1)
		dev_info(ctx->dev->dev, "  take slice hold=%d\n",
			 !!(src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF));
	if (src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF)
		return 1;
	ret = hivdec_h264_decode(ctx, dst);
	h->nslices = 0;
	if (ret)
		h->error = true;
	return ret;
}

static int hivdec_h264_run(struct hivdec_ctx *ctx)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	const struct v4l2_ctrl_h264_decode_params *dp;
	int ret;

	if (req)
		v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	/*
	 * Both fields of a frame share the CAPTURE buffer, so HOLD_CAPTURE_BUF
	 * stays set across the field boundary: a change of the decode
	 * parameters starts a new picture. Decode the gathered one first and
	 * take the current slice when the hardware is done.
	 */
	dp = hivdec_find_control_data(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	if (h->nslices && memcmp(dp, &h->dp, sizeof(*dp))) {
		ret = hivdec_h264_decode(ctx, dst);
		h->nslices = 0;
		if (!ret) {
			h->carry = true;
			return 0;
		}
		h->error = true;
	}
	return hivdec_h264_take_slice(ctx);
}

static int hivdec_h264_done(struct hivdec_ctx *ctx, enum vb2_buffer_state state)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	bool err;
	int ret;

	if (state != VB2_BUF_STATE_DONE)
		h->error = true;
	if (h->carry) {
		h->carry = false;
		ret = hivdec_h264_take_slice(ctx);
		if (!ret)
			return 1;	/* VDH running again for this job */
		if (ret > 0)
			return 0;	/* CAPTURE buffer stays held */
		return ret;
	}
	/* the CAPTURE buffer is complete */
	err = h->error;
	h->error = false;
	return err ? -EIO : 0;
}

static int hivdec_h264_flush(struct hivdec_ctx *ctx)
{
	struct hivdec_h264_ctx *h = ctx->priv;
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	int ret;

	if (!h || !h->nslices || !dst)
		return 1;
	ret = hivdec_h264_decode(ctx, dst);
	h->nslices = 0;
	h->error = false;
	return ret ? 1 : 0;
}

const struct hivdec_coded_fmt_ops hivdec_h264_fmt_ops = {
	.adjust_fmt = hivdec_h264_adjust_fmt,
	.start = hivdec_h264_start,
	.stop = hivdec_h264_stop,
	.run = hivdec_h264_run,
	.done = hivdec_h264_done,
	.flush = hivdec_h264_flush,
	.try_ctrl = hivdec_h264_try_ctrl,
};
