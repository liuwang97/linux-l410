// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 stateless video decoder (VCodec V500 VDH)
 *
 * V4L2 plumbing follows the Rockchip rkvdec driver (Collabora).
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>

#include "hisi_vdec.h"

static unsigned int clk_level = 1;
module_param(clk_level, uint, 0644);
MODULE_PARM_DESC(clk_level, "decoder clock: 0 = 480 MHz, 1 = 332 MHz, 2 = 277 MHz, 3 = 185 MHz");

/* ------------------------------------------------------------------ formats */

static const u32 hivdec_yuv420_fmts[] = { V4L2_PIX_FMT_NV12 };

static const struct hivdec_ctrl_desc hivdec_h264_ctrls[] = {
	{ .cfg.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS },
	{ .cfg.id = V4L2_CID_STATELESS_H264_SPS },
	{ .cfg.id = V4L2_CID_STATELESS_H264_PPS },
	{ .cfg.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX },
	{ .cfg.id = V4L2_CID_STATELESS_H264_SLICE_PARAMS },
	{ .cfg.id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS },
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.cfg.max = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.cfg.def = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_START_CODE,
		.cfg.min = V4L2_STATELESS_H264_START_CODE_NONE,
		.cfg.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.cfg.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.cfg.menu_skip_mask = BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),
		.cfg.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.cfg.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
	},
};

static const struct hivdec_coded_fmt_desc hivdec_coded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 16,
			.min_height = 64,
			.max_height = 2304,
			.step_height = 16,
		},
		.ctrls = hivdec_h264_ctrls,
		.num_ctrls = ARRAY_SIZE(hivdec_h264_ctrls),
		.ops = &hivdec_h264_fmt_ops,
		.decoded_fmts = hivdec_yuv420_fmts,
		.num_decoded_fmts = ARRAY_SIZE(hivdec_yuv420_fmts),
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
	},
};

static const struct hivdec_coded_fmt_desc *hivdec_find_coded_fmt_desc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hivdec_coded_fmts); i++)
		if (hivdec_coded_fmts[i].fourcc == fourcc)
			return &hivdec_coded_fmts[i];
	return NULL;
}

/*
 * Decoded frames: NV12 in one plane. The VDH writes the luma at YSTADDR_1D
 * with stride YSTRIDE_1D and the interleaved chroma UVOFFSET_1D bytes further.
 * Keep the stride 64-byte aligned and the height a multiple of 32 (field and
 * MBAFF pictures are written in 32-line macroblock pairs).
 */
static void hivdec_fill_decoded_pixfmt(struct hivdec_ctx *ctx,
				       struct v4l2_pix_format_mplane *pix_mp)
{
	u32 w = ALIGN(pix_mp->width, 16);
	u32 h = ALIGN(pix_mp->height, 32);
	u32 stride = ALIGN(w, 64);

	pix_mp->width = w;
	pix_mp->height = h;
	pix_mp->num_planes = 1;
	pix_mp->plane_fmt[0].bytesperline = stride;
	pix_mp->plane_fmt[0].sizeimage = stride * h * 3 / 2;
	pix_mp->field = V4L2_FIELD_NONE;
}

static void hivdec_reset_decoded_fmt(struct hivdec_ctx *ctx)
{
	struct v4l2_format *f = &ctx->decoded_fmt;

	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->fmt.pix_mp.pixelformat = ctx->coded_fmt_desc->decoded_fmts[0];
	f->fmt.pix_mp.width = ctx->coded_fmt.fmt.pix_mp.width;
	f->fmt.pix_mp.height = ctx->coded_fmt.fmt.pix_mp.height;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	hivdec_fill_decoded_pixfmt(ctx, &f->fmt.pix_mp);
}

static void hivdec_reset_coded_fmt(struct hivdec_ctx *ctx)
{
	struct v4l2_format *f = &ctx->coded_fmt;

	ctx->coded_fmt_desc = &hivdec_coded_fmts[0];
	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f->fmt.pix_mp.pixelformat = ctx->coded_fmt_desc->fourcc;
	f->fmt.pix_mp.width = ctx->coded_fmt_desc->frmsize.min_width;
	f->fmt.pix_mp.height = ctx->coded_fmt_desc->frmsize.min_height;
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	if (ctx->coded_fmt_desc->ops->adjust_fmt)
		ctx->coded_fmt_desc->ops->adjust_fmt(ctx, f);
}

void *hivdec_find_control_data(struct hivdec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	return ctrl ? ctrl->p_cur.p : NULL;
}

/* ------------------------------------------------------------------ ioctls */

static int hivdec_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hivdec_ctx *ctx = container_of(ctrl->handler, struct hivdec_ctx, ctrl_hdl);
	const struct hivdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;

	if (desc->ops->try_ctrl)
		return desc->ops->try_ctrl(ctx, ctrl);
	return 0;
}

static const struct v4l2_ctrl_ops hivdec_ctrl_ops = {
	.try_ctrl = hivdec_try_ctrl,
};

static int hivdec_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	const struct hivdec_coded_fmt_desc *fmt;

	if (fsize->index != 0)
		return -EINVAL;
	fmt = hivdec_find_coded_fmt_desc(fsize->pixel_format);
	if (!fmt)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 1;
	fsize->stepwise.max_width = fmt->frmsize.max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = fmt->frmsize.max_height;
	fsize->stepwise.step_height = 1;
	return 0;
}

static int hivdec_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct hivdec_dev *vdec = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);

	strscpy(cap->driver, vdec->dev->driver->name, sizeof(cap->driver));
	strscpy(cap->card, vdev->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 vdec->dev->driver->name);
	return 0;
}

static int hivdec_try_capture_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(file);
	const struct hivdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	unsigned int i;

	for (i = 0; i < desc->num_decoded_fmts; i++)
		if (desc->decoded_fmts[i] == pix_mp->pixelformat)
			break;
	if (i == desc->num_decoded_fmts)
		pix_mp->pixelformat = desc->decoded_fmts[0];

	/* the decoded size always follows the coded size */
	pix_mp->width = ctx->coded_fmt.fmt.pix_mp.width;
	pix_mp->height = ctx->coded_fmt.fmt.pix_mp.height;
	hivdec_fill_decoded_pixfmt(ctx, pix_mp);
	return 0;
}

static int hivdec_try_output_fmt(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(file);
	const struct hivdec_coded_fmt_desc *desc;

	desc = hivdec_find_coded_fmt_desc(pix_mp->pixelformat);
	if (!desc) {
		pix_mp->pixelformat = hivdec_coded_fmts[0].fourcc;
		desc = &hivdec_coded_fmts[0];
	}
	v4l2_apply_frmsize_constraints(&pix_mp->width, &pix_mp->height,
				       &desc->frmsize);
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->num_planes = 1;
	if (desc->ops->adjust_fmt)
		return desc->ops->adjust_fmt(ctx, f);
	return 0;
}

static int hivdec_s_capture_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(file);
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(vq))
		return -EBUSY;
	ret = hivdec_try_capture_fmt(file, priv, f);
	if (ret)
		return ret;
	ctx->decoded_fmt = *f;
	return 0;
}

static int hivdec_s_output_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	const struct hivdec_coded_fmt_desc *desc;
	struct v4l2_format *cap_fmt;
	struct vb2_queue *peer_vq, *vq;
	int ret;

	vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (vb2_is_streaming(vq) ||
	    (vb2_is_busy(vq) &&
	     f->fmt.pix_mp.pixelformat != ctx->coded_fmt.fmt.pix_mp.pixelformat))
		return -EBUSY;
	peer_vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(peer_vq))
		return -EBUSY;

	ret = hivdec_try_output_fmt(file, priv, f);
	if (ret)
		return ret;
	desc = hivdec_find_coded_fmt_desc(f->fmt.pix_mp.pixelformat);
	if (!desc)
		return -EINVAL;
	ctx->coded_fmt_desc = desc;
	ctx->coded_fmt = *f;

	hivdec_reset_decoded_fmt(ctx);
	cap_fmt = &ctx->decoded_fmt;
	cap_fmt->fmt.pix_mp.colorspace = f->fmt.pix_mp.colorspace;
	cap_fmt->fmt.pix_mp.xfer_func = f->fmt.pix_mp.xfer_func;
	cap_fmt->fmt.pix_mp.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	cap_fmt->fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;

	vq->subsystem_flags |= desc->subsystem_flags;
	return 0;
}

static int hivdec_g_output_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	*f = file_to_hivdec_ctx(file)->coded_fmt;
	return 0;
}

static int hivdec_g_capture_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	*f = file_to_hivdec_ctx(file)->decoded_fmt;
	return 0;
}

static int hivdec_enum_output_fmt(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(hivdec_coded_fmts))
		return -EINVAL;
	f->pixelformat = hivdec_coded_fmts[f->index].fourcc;
	return 0;
}

static int hivdec_enum_capture_fmt(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(file);

	if (f->index >= ctx->coded_fmt_desc->num_decoded_fmts)
		return -EINVAL;
	f->pixelformat = ctx->coded_fmt_desc->decoded_fmts[f->index];
	return 0;
}

static int hivdec_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->coded_fmt.fmt.pix_mp.width;
		s->r.height = ctx->coded_fmt.fmt.pix_mp.height;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops hivdec_ioctl_ops = {
	.vidioc_querycap = hivdec_querycap,
	.vidioc_enum_framesizes = hivdec_enum_framesizes,
	.vidioc_try_fmt_vid_cap_mplane = hivdec_try_capture_fmt,
	.vidioc_try_fmt_vid_out_mplane = hivdec_try_output_fmt,
	.vidioc_s_fmt_vid_out_mplane = hivdec_s_output_fmt,
	.vidioc_s_fmt_vid_cap_mplane = hivdec_s_capture_fmt,
	.vidioc_g_fmt_vid_out_mplane = hivdec_g_output_fmt,
	.vidioc_g_fmt_vid_cap_mplane = hivdec_g_capture_fmt,
	.vidioc_enum_fmt_vid_out = hivdec_enum_output_fmt,
	.vidioc_enum_fmt_vid_cap = hivdec_enum_capture_fmt,
	.vidioc_g_selection = hivdec_g_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
};

/* ------------------------------------------------------------------ vb2 */

static int hivdec_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f = V4L2_TYPE_IS_OUTPUT(vq->type) ?
				&ctx->coded_fmt : &ctx->decoded_fmt;
	unsigned int i;

	if (*num_planes) {
		if (*num_planes != f->fmt.pix_mp.num_planes)
			return -EINVAL;
		for (i = 0; i < f->fmt.pix_mp.num_planes; i++)
			if (sizes[i] < f->fmt.pix_mp.plane_fmt[i].sizeimage)
				return -EINVAL;
	} else {
		*num_planes = f->fmt.pix_mp.num_planes;
		for (i = 0; i < f->fmt.pix_mp.num_planes; i++)
			sizes[i] = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}
	return 0;
}

/* map every buffer into the VDH address space once, when vb2 first sees it */
static int hivdec_buf_init(struct vb2_buffer *vb)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, 0);
	struct hivdec_mapping *map;
	bool *mapped;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		map = &vb2_to_hivdec_src(vb)->map;
		mapped = &vb2_to_hivdec_src(vb)->mapped;
	} else {
		map = &vb2_to_hivdec_buf(vb)->map;
		mapped = &vb2_to_hivdec_buf(vb)->mapped;
	}
	ret = hivdec_mmu_map_sgt(ctx->dev, sgt, vb2_plane_size(vb, 0), map);
	if (ret)
		return ret;
	*mapped = true;
	return 0;
}

static void hivdec_buf_cleanup(struct vb2_buffer *vb)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		struct hivdec_src_buffer *b = vb2_to_hivdec_src(vb);

		if (b->mapped)
			hivdec_mmu_unmap(ctx->dev, &b->map);
		b->mapped = false;
	} else {
		struct hivdec_decoded_buffer *b = vb2_to_hivdec_buf(vb);

		if (b->mapped)
			hivdec_mmu_unmap(ctx->dev, &b->map);
		b->mapped = false;
		hivdec_aux_free(ctx->dev, &b->mv);
	}
}

static int hivdec_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f = V4L2_TYPE_IS_OUTPUT(vq->type) ?
				&ctx->coded_fmt : &ctx->decoded_fmt;

	if (vb2_plane_size(vb, 0) < f->fmt.pix_mp.plane_fmt[0].sizeimage)
		return -EINVAL;
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		vb2_set_plane_payload(vb, 0, f->fmt.pix_mp.plane_fmt[0].sizeimage);
	return 0;
}

static void hivdec_buf_queue(struct vb2_buffer *vb)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int hivdec_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static void hivdec_buf_request_complete(struct vb2_buffer *vb)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static int hivdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(q);
	const struct hivdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	int ret;

	if (V4L2_TYPE_IS_CAPTURE(q->type))
		return 0;

	if (!ctx->msg.priv) {
		ret = hivdec_aux_alloc(ctx->dev, &ctx->msg,
				       HIVDEC_MSG_SLOTS * HIVDEC_MSG_SLOT_BYTES);
		if (ret)
			return ret;
	}
	if (desc->ops->start)
		return desc->ops->start(ctx);
	return 0;
}

static void hivdec_queue_cleanup(struct vb2_queue *vq, u32 state)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(vq);

	while (true) {
		struct vb2_v4l2_buffer *vbuf;

		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req,
					   &ctx->ctrl_hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static void hivdec_stop_streaming(struct vb2_queue *q)
{
	struct hivdec_ctx *ctx = vb2_get_drv_priv(q);

	if (V4L2_TYPE_IS_OUTPUT(q->type) && ctx->coded_fmt_desc->ops->stop)
		ctx->coded_fmt_desc->ops->stop(ctx);
	hivdec_queue_cleanup(q, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops hivdec_queue_ops = {
	.queue_setup = hivdec_queue_setup,
	.buf_init = hivdec_buf_init,
	.buf_cleanup = hivdec_buf_cleanup,
	.buf_prepare = hivdec_buf_prepare,
	.buf_queue = hivdec_buf_queue,
	.buf_out_validate = hivdec_buf_out_validate,
	.buf_request_complete = hivdec_buf_request_complete,
	.start_streaming = hivdec_start_streaming,
	.stop_streaming = hivdec_stop_streaming,
};

static int hivdec_request_validate(struct media_request *req)
{
	unsigned int count = vb2_request_buffer_cnt(req);

	if (!count)
		return -ENOENT;
	if (count > 1)
		return -EINVAL;
	return vb2_request_validate(req);
}

static const struct media_device_ops hivdec_media_ops = {
	.req_validate = hivdec_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

/* ------------------------------------------------------------------ hardware */

static int hivdec_soft_reset(struct hivdec_dev *vdec, u32 mask)
{
	u32 req = vdh_read(vdec, SOFTRST_REQ), ok;
	int ret;

	vdh_write(vdec, SOFTRST_REQ, req | mask);
	ret = readl_poll_timeout_atomic(vdec->regs + SOFTRST_OK, ok, ok & mask,
					10, 10000);
	vdh_write(vdec, SOFTRST_REQ, req & ~mask);
	if (ret)
		dev_warn(vdec->dev, "reset %#x timed out (ok=%#x)\n", mask, ok);
	return ret;
}

static int hivdec_power_on(struct hivdec_dev *vdec)
{
	unsigned int level = min(clk_level, 3U);
	int ret;

	ret = regulator_enable(vdec->media_supply);
	if (ret)
		return ret;
	ret = clk_prepare_enable(vdec->clk);
	if (ret)
		goto err_media;
	ret = clk_set_rate(vdec->clk, vdec->clk_rate[3]);
	if (ret)
		goto err_clk;
	ret = regulator_enable(vdec->vdec_supply);
	if (ret)
		goto err_clk;
	if (level != 3) {
		ret = clk_set_rate(vdec->clk, vdec->clk_rate[level]);
		if (ret)
			dev_warn(vdec->dev, "clock rate %u: %d\n",
				 vdec->clk_rate[level], ret);
	}

	vdec->powered = true;
	hivdec_soft_reset(vdec, RST_ALL);
	hivdec_mmu_hw_setup(vdec);
	vdh_write(vdec, VDH_INT_MASK, ~0U);
	vdh_write(vdec, VDH_INT_STATE, ~0U);
	return 0;

err_clk:
	clk_disable_unprepare(vdec->clk);
err_media:
	regulator_disable(vdec->media_supply);
	return ret;
}

static void hivdec_power_off(struct hivdec_dev *vdec)
{
	vdec->powered = false;
	regulator_disable(vdec->vdec_supply);
	clk_set_rate(vdec->clk, vdec->clk_rate[3]);
	clk_disable_unprepare(vdec->clk);
	regulator_disable(vdec->media_supply);
	hivdec_mmu_powered_off(vdec);
}

/* start the VDH on the messages the codec code prepared */
void hivdec_hw_run(struct hivdec_ctx *ctx)
{
	struct hivdec_dev *vdec = ctx->dev;

	vdec->run_ctx = ctx;
	hivdec_aux_sync_for_device(vdec, &ctx->msg);
	schedule_delayed_work(&vdec->watchdog_work, msecs_to_jiffies(2000));

	vdh_write(vdec, VDH_INT_STATE, ~0U);
	vdh_write(vdec, VDH_INT_MASK, ~1U);	/* dec_over */
	wmb();
	vdh_write(vdec, VDH_START, 0);
	vdh_write(vdec, VDH_START, 1);
	vdh_write(vdec, VDH_START, 0);
}

static void hivdec_job_finish_no_pm(struct hivdec_ctx *ctx,
				    enum vb2_buffer_state state)
{
	v4l2_m2m_buf_done_and_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx, state);
}

void hivdec_job_finish(struct hivdec_ctx *ctx, enum vb2_buffer_state state)
{
	pm_runtime_put_autosuspend(ctx->dev->dev);
	hivdec_job_finish_no_pm(ctx, state);
}

/* process context: called from the threaded IRQ handler or the watchdog */
static void hivdec_hw_done(struct hivdec_dev *vdec, bool timeout)
{
	struct hivdec_ctx *ctx = vdec->run_ctx;
	enum vb2_buffer_state state;
	int ret = 0;

	vdec->run_ctx = NULL;
	if (!ctx)
		return;
	hivdec_aux_sync_for_cpu(vdec, &ctx->msg);
	state = (!timeout && (vdec->irq_status & 1) &&
		 !(vdec->vdh_state & STATE_DEC_ERR)) ?
		VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;
	if (vdec->debug > 1) {
		const u32 *up = hivdec_msg_slot(ctx, HIVDEC_SLOT_UP);
		unsigned int i, n = min_t(u32, STATE_DECODED_SLICES(vdec->vdh_state), 16);

		for (i = 0; i < n; i++)
			dev_info(vdec->dev, "  up[%u]: %08x %08x %08x %08x\n", i,
				 up[4 * i], up[4 * i + 1], up[4 * i + 2], up[4 * i + 3]);
	}
	if (vdec->debug || state != VB2_BUF_STATE_DONE)
		dev_info(vdec->dev, "%s int %#x state %#x (%u slices) cycles %u\n",
			 timeout ? "timeout" : "done", vdec->irq_status,
			 vdec->vdh_state, STATE_DECODED_SLICES(vdec->vdh_state),
			 vdec->dec_cycles);
	/* the codec may chain another hardware run into the same job */
	if (ctx->coded_fmt_desc->ops->done)
		ret = ctx->coded_fmt_desc->ops->done(ctx, state);
	if (ret > 0)
		return;
	hivdec_job_finish(ctx, ret < 0 ? VB2_BUF_STATE_ERROR : state);
}

static irqreturn_t hivdec_irq(int irq, void *priv)
{
	struct hivdec_dev *vdec = priv;
	u32 st = vdh_read(vdec, VDH_INT_STATE);

	if (!(st & 3))
		return IRQ_NONE;
	vdec->irq_status = st;
	vdec->vdh_state = vdh_read(vdec, VDH_STATE);
	vdec->dec_cycles = vdh_read(vdec, VDH_DEC_CYCLEPERPIC);
	vdh_write(vdec, VDH_INT_STATE, ~0U);
	vdh_write(vdec, VDH_INT_MASK, ~0U);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t hivdec_irq_thread(int irq, void *priv)
{
	struct hivdec_dev *vdec = priv;

	if (cancel_delayed_work(&vdec->watchdog_work))
		hivdec_hw_done(vdec, false);
	return IRQ_HANDLED;
}

static void hivdec_watchdog_func(struct work_struct *work)
{
	struct hivdec_dev *vdec = container_of(to_delayed_work(work),
					       struct hivdec_dev, watchdog_work);

	vdec->irq_status = vdh_read(vdec, VDH_INT_STATE);
	vdec->vdh_state = vdh_read(vdec, VDH_STATE);
	vdh_write(vdec, VDH_INT_MASK, ~0U);
	dev_err(vdec->dev, "decode timeout, VCTRL_STATE %#x SED_STA %#x SMMU %#x dbg %#x %#x\n",
		vdh_read(vdec, VDH_VCTRL_STATE), vdh_read(vdec, VDH_SED_STA),
		vdh_read(vdec, SMMU_INTSTAT_NS), vdh_read(vdec, SMMU_MSTR_DBG(0)),
		vdh_read(vdec, SMMU_MSTR_DBG(1)));
	hivdec_soft_reset(vdec, RST_MFDE);
	hivdec_hw_done(vdec, true);
}

/* ------------------------------------------------------------------ m2m */

static void hivdec_device_run(void *priv)
{
	struct hivdec_ctx *ctx = priv;
	struct hivdec_dev *vdec = ctx->dev;
	int ret;

	ret = pm_runtime_resume_and_get(vdec->dev);
	if (ret < 0) {
		hivdec_job_finish_no_pm(ctx, VB2_BUF_STATE_ERROR);
		return;
	}
	ret = ctx->coded_fmt_desc->ops->run(ctx);
	if (ret)
		hivdec_job_finish(ctx, ret > 0 ? VB2_BUF_STATE_DONE :
						 VB2_BUF_STATE_ERROR);
}

static const struct v4l2_m2m_ops hivdec_m2m_ops = {
	.device_run = hivdec_device_run,
};

static int hivdec_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct hivdec_ctx *ctx = priv;
	struct hivdec_dev *vdec = ctx->dev;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &hivdec_queue_ops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->buf_struct_size = sizeof(struct hivdec_src_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &vdec->vdev_lock;
	src_vq->dev = vdec->dev;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->bidirectional = true;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &hivdec_queue_ops;
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->buf_struct_size = sizeof(struct hivdec_decoded_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &vdec->vdev_lock;
	dst_vq->dev = vdec->dev;
	return vb2_queue_init(dst_vq);
}

static int hivdec_init_ctrls(struct hivdec_ctx *ctx)
{
	unsigned int i, j, nctrls = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(hivdec_coded_fmts); i++)
		nctrls += hivdec_coded_fmts[i].num_ctrls;
	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, nctrls);

	for (i = 0; i < ARRAY_SIZE(hivdec_coded_fmts); i++) {
		for (j = 0; j < hivdec_coded_fmts[i].num_ctrls; j++) {
			struct v4l2_ctrl_config cfg = hivdec_coded_fmts[i].ctrls[j].cfg;

			if (!cfg.ops)
				cfg.ops = &hivdec_ctrl_ops;
			v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &cfg, ctx);
			if (ctx->ctrl_hdl.error) {
				ret = ctx->ctrl_hdl.error;
				goto err_free;
			}
		}
	}
	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);
	if (ret)
		goto err_free;
	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	return 0;

err_free:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	return ret;
}

static int hivdec_open(struct file *filp)
{
	struct hivdec_dev *vdec = video_drvdata(filp);
	struct hivdec_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = vdec;
	hivdec_reset_coded_fmt(ctx);
	hivdec_reset_decoded_fmt(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vdec->m2m_dev, ctx, hivdec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free;
	}
	ret = hivdec_init_ctrls(ctx);
	if (ret)
		goto err_m2m;
	v4l2_fh_add(&ctx->fh, filp);
	return 0;

err_m2m:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_free:
	kfree(ctx);
	return ret;
}

static int hivdec_release(struct file *filp)
{
	struct hivdec_ctx *ctx = file_to_hivdec_ctx(filp);

	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	hivdec_aux_free(ctx->dev, &ctx->msg);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations hivdec_fops = {
	.owner = THIS_MODULE,
	.open = hivdec_open,
	.release = hivdec_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int hivdec_v4l2_init(struct hivdec_dev *vdec)
{
	int ret;

	ret = v4l2_device_register(vdec->dev, &vdec->v4l2_dev);
	if (ret)
		return ret;
	vdec->m2m_dev = v4l2_m2m_init(&hivdec_m2m_ops);
	if (IS_ERR(vdec->m2m_dev)) {
		ret = PTR_ERR(vdec->m2m_dev);
		goto err_v4l2;
	}

	vdec->mdev.dev = vdec->dev;
	strscpy(vdec->mdev.model, "hisi-vdec", sizeof(vdec->mdev.model));
	strscpy(vdec->mdev.bus_info, "platform:hisi-vdec", sizeof(vdec->mdev.bus_info));
	media_device_init(&vdec->mdev);
	vdec->mdev.ops = &hivdec_media_ops;
	vdec->v4l2_dev.mdev = &vdec->mdev;

	vdec->vdev.lock = &vdec->vdev_lock;
	vdec->vdev.v4l2_dev = &vdec->v4l2_dev;
	vdec->vdev.fops = &hivdec_fops;
	vdec->vdev.release = video_device_release_empty;
	vdec->vdev.vfl_dir = VFL_DIR_M2M;
	vdec->vdev.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	vdec->vdev.ioctl_ops = &hivdec_ioctl_ops;
	strscpy(vdec->vdev.name, "hisi-vdec", sizeof(vdec->vdev.name));
	video_set_drvdata(&vdec->vdev, vdec);

	ret = video_register_device(&vdec->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;
	ret = v4l2_m2m_register_media_controller(vdec->m2m_dev, &vdec->vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret)
		goto err_vdev;
	ret = media_device_register(&vdec->mdev);
	if (ret)
		goto err_mc;
	return 0;

err_mc:
	v4l2_m2m_unregister_media_controller(vdec->m2m_dev);
err_vdev:
	video_unregister_device(&vdec->vdev);
err_m2m:
	media_device_cleanup(&vdec->mdev);
	v4l2_m2m_release(vdec->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&vdec->v4l2_dev);
	return ret;
}

static void hivdec_v4l2_cleanup(struct hivdec_dev *vdec)
{
	media_device_unregister(&vdec->mdev);
	v4l2_m2m_unregister_media_controller(vdec->m2m_dev);
	video_unregister_device(&vdec->vdev);
	media_device_cleanup(&vdec->mdev);
	v4l2_m2m_release(vdec->m2m_dev);
	v4l2_device_unregister(&vdec->v4l2_dev);
}

/* ------------------------------------------------------------------ probe */

static int hivdec_runtime_resume(struct device *dev)
{
	return hivdec_power_on(dev_get_drvdata(dev));
}

static int hivdec_runtime_suspend(struct device *dev)
{
	hivdec_power_off(dev_get_drvdata(dev));
	return 0;
}

static int hivdec_regs_show(struct seq_file *s, void *unused)
{
	struct hivdec_dev *vdec = s->private;
	static const u32 regs[] = {
		VDH_BASIC_CFG0, VDH_BASIC_CFG1, VDH_AVM_ADDR, VDH_VAM_ADDR,
		VDH_STREAM_BASE_ADDR, VDH_STATE, VDH_INT_STATE, VDH_INT_MASK,
		VDH_VCTRL_STATE, VDH_YSTADDR_1D, VDH_YSTRIDE_1D, VDH_UVOFFSET_1D,
		VDH_UVSTRIDE_1D, VDH_CFGINFO_ADDR, VDH_DEC_CYCLEPERPIC,
		VDH_SED_STA, VDH_SED_END0, SCD_EMAR_ID, SOFTRST_REQ, SOFTRST_OK,
		SMMU_SCR, SMMU_INTSTAT_NS, SMMU_CB_TTBR0, SMMU_CB_TTBR_MSB,
		SMMU_CB_TTBCR, SMMU_MSTR_GLB_BYPASS, SMMU_MSTR_DBG(0),
		SMMU_MSTR_DBG(1), SMMU_MSTR_DBG(2),
	};
	unsigned int i;
	int ret;

	ret = pm_runtime_resume_and_get(vdec->dev);
	if (ret < 0)
		return ret;
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		seq_printf(s, "%05x: %08x\n", regs[i], vdh_read(vdec, regs[i]));
	pm_runtime_put_autosuspend(vdec->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(hivdec_regs);

static int hivdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hivdec_dev *vdec;
	int ret;

	vdec = devm_kzalloc(dev, sizeof(*vdec), GFP_KERNEL);
	if (!vdec)
		return -ENOMEM;
	platform_set_drvdata(pdev, vdec);
	vdec->dev = dev;
	mutex_init(&vdec->vdev_lock);
	INIT_DELAYED_WORK(&vdec->watchdog_work, hivdec_watchdog_func);

	vdec->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdec->regs))
		return PTR_ERR(vdec->regs);

	/* vendor binding: clock "clk_vdec", supplies "ldo_media" and "ldo_vdec" */
	vdec->clk = devm_clk_get(dev, "clk_vdec");
	if (IS_ERR(vdec->clk))
		return dev_err_probe(dev, PTR_ERR(vdec->clk), "clock\n");
	vdec->media_supply = devm_regulator_get(dev, "ldo_media");
	if (IS_ERR(vdec->media_supply))
		return dev_err_probe(dev, PTR_ERR(vdec->media_supply), "ldo_media\n");
	vdec->vdec_supply = devm_regulator_get(dev, "ldo_vdec");
	if (IS_ERR(vdec->vdec_supply))
		return dev_err_probe(dev, PTR_ERR(vdec->vdec_supply), "ldo_vdec\n");
	if (of_property_read_u32_array(dev->of_node, "dec_clk_rate",
				       vdec->clk_rate, 4)) {
		vdec->clk_rate[0] = 480000000;
		vdec->clk_rate[1] = 332000000;
		vdec->clk_rate[2] = 277000000;
		vdec->clk_rate[3] = 185000000;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	vdec->irq = platform_get_irq(pdev, 0);
	if (vdec->irq < 0)
		return vdec->irq;
	ret = devm_request_threaded_irq(dev, vdec->irq, hivdec_irq, hivdec_irq_thread,
					IRQF_ONESHOT, dev_name(dev), vdec);
	if (ret)
		return ret;

	ret = hivdec_mmu_init(vdec);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	/* smoke test: the block answers and reports its version */
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto err_pm;
	vdec->version = STATE_VERSION(vdh_read(vdec, VDH_STATE));
	dev_info(dev, "VDH version %#x, clock %lu Hz, SMMU TTBR %#llx\n",
		 vdec->version, clk_get_rate(vdec->clk),
		 (u64)vdec->mmu.cfg.arm_lpae_s1_cfg.ttbr);
	pm_runtime_put_autosuspend(dev);

	ret = hivdec_v4l2_init(vdec);
	if (ret)
		goto err_pm;

	vdec->debugfs = debugfs_create_dir(KBUILD_MODNAME, NULL);
	debugfs_create_file("regs", 0400, vdec->debugfs, vdec, &hivdec_regs_fops);
	debugfs_create_u32("debug", 0600, vdec->debugfs, &vdec->debug);
	return 0;

err_pm:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	hivdec_mmu_fini(vdec);
	return ret;
}

static void hivdec_remove(struct platform_device *pdev)
{
	struct hivdec_dev *vdec = platform_get_drvdata(pdev);

	debugfs_remove_recursive(vdec->debugfs);
	cancel_delayed_work_sync(&vdec->watchdog_work);
	/* before the V4L2 teardown: v4l2_device_unregister() clears drvdata */
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (vdec->powered)
		hivdec_power_off(vdec);
	hivdec_v4l2_cleanup(vdec);
	hivdec_mmu_fini(vdec);
}

static const struct dev_pm_ops hivdec_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(hivdec_runtime_suspend, hivdec_runtime_resume, NULL)
};

static const struct of_device_id hivdec_of_match[] = {
	{ .compatible = "hisilicon,kirin990-vdec" },
	{ .compatible = "hisilicon,HiVCodecV500-vdec" },	/* firmware DT */
	{ }
};
MODULE_DEVICE_TABLE(of, hivdec_of_match);

static struct platform_driver hivdec_driver = {
	.probe = hivdec_probe,
	.remove = hivdec_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = hivdec_of_match,
		.pm = &hivdec_pm_ops,
	},
};
module_platform_driver(hivdec_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 stateless video decoder");
MODULE_LICENSE("GPL");
