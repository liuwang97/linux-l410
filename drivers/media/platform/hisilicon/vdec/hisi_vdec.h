/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon Kirin 990 (VCodec V500, VDH V500R003) stateless video decoder
 *
 * The VDH reads a per-picture "message pool" from memory: one picture message,
 * a chain of per-slice messages and returns a status ("up") message. The
 * message layout follows the HiSilicon VFMW HAL; every bus address written into
 * a message or an address register is the IOVA shifted right by 4.
 */
#ifndef HISI_VDEC_H_
#define HISI_VDEC_H_

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/genalloc.h>
#include <linux/io-pgtable.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>

/* VDH (MFDE) registers, offsets from the vdec base */
#define VDH_START		0x000
#define VDH_REPAIR		0x004
#define VDH_BASIC_CFG0		0x008
#define VDH_BASIC_CFG1		0x00c
#define VDH_AVM_ADDR		0x010	/* down message (picture message) */
#define VDH_VAM_ADDR		0x014	/* up message (decode report) */
#define VDH_STREAM_BASE_ADDR	0x018
#define VDH_STATE		0x01c
#define VDH_INT_STATE		0x020
#define VDH_INT_MASK		0x024
#define VDH_VCTRL_STATE		0x028
#define VDH_SED_TO		0x03c
#define VDH_ITRANS_TO		0x040
#define VDH_PMV_TO		0x044
#define VDH_PRC_TO		0x048
#define VDH_RCN_TO		0x04c
#define VDH_DBLK_TO		0x050
#define VDH_PPFD_TO		0x054
#define VDH_PART_DEC_OVER_INT_LEVEL 0x05c
#define VDH_YSTADDR_1D		0x060
#define VDH_YSTRIDE_1D		0x064
#define VDH_UVOFFSET_1D		0x068
#define VDH_HEAD_INF_OFFSET	0x06c
#define VDH_YSTRIDE_2BIT	0x074
#define VDH_YOFFSET_2BIT	0x078
#define VDH_UVOFFSET_2BIT	0x07c
#define VDH_REF_PIC_TYPE	0x094
#define VDH_FF_APT_EN		0x098
#define VDH_DEC_CYCLEPERPIC	0x0b0
#define VDH_RD_BDWIDTH_PERPIC	0x0b4
#define VDH_WR_BDWIDTH_PERPIC	0x0b8
#define VDH_UVSTRIDE_1D		0x0c4
#define VDH_CFGINFO_ADDR	0x0c8
#define VDH_DDR_INTERLEAVE_MODE	0x0f4
#define VDH_SED_STA		0x1000
#define VDH_SED_END0		0x1014

#define VDH_TIMEOUT_DEFAULT	0x00300c03

/* BASIC_CFG0 */
#define CFG0_MBAMT_TO_DEC(n)	((n) & 0xfffff)
#define CFG0_MARKER_BIT_DETECT	BIT(22)
#define CFG0_AC_LAST_DETECT	BIT(23)
#define CFG0_COEF_IDX_DETECT	BIT(24)
#define CFG0_VOP_TYPE_DETECT	BIT(25)
#define CFG0_LOAD_QMATRIX	BIT(30)
#define CFG0_SEC_MODE		BIT(31)
/* BASIC_CFG1 */
#define CFG1_VIDEO_STANDARD(s)	((s) & 0xf)
#define CFG1_MFD_MMU_EN		BIT(12)
#define CFG1_UV_ORDER_EN	BIT(13)
#define CFG1_FST_SLC_GRP	BIT(14)
#define CFG1_MV_OUTPUT_EN	BIT(15)
#define CFG1_MAX_SLCGRP_NUM(n)	(((n) & 0xfff) << 16)
#define CFG1_LINE_NUM_OUTPUT_EN	BIT(28)
#define CFG1_VDH_2D_EN		BIT(29)
#define CFG1_COMPRESS_EN	BIT(30)
#define CFG1_PPFD_EN		BIT(31)

/* VDH_STATE */
#define STATE_DECODED_SLICES(v)	((v) & 0x1ffff)
#define STATE_DEC_OVER		BIT(17)
#define STATE_DEC_ERR		BIT(18)
#define STATE_VERSION(v)	(((v) >> 19) & 0xff)

/* BASIC_CFG1 video_standard */
enum hivdec_std {
	HIVDEC_STD_H264 = 0,
	HIVDEC_STD_MPEG2 = 3,
	HIVDEC_STD_VP8 = 12,
	HIVDEC_STD_HEVC = 13,
	HIVDEC_STD_VP9 = 14,
};

/* SCD block (start code detector, only a few shared registers are used) */
#define SCD_BASE		0xc000
#define SCD_AVS_FLAG		(SCD_BASE + 0x000)
#define SCD_EMAR_ID		(SCD_BASE + 0x004)
#define SCD_VDH_SELRST		(SCD_BASE + 0x008)
#define SCD_EMAR_ID_ONCHIP	BIT(8)

/* soft reset, inside the vdec CRG */
#define SOFTRST_REQ		0xcc0c
#define SOFTRST_OK		0xcc10
#define RST_ALL			BIT(0)
#define RST_MFDE		BIT(1)
#define RST_SCD			BIT(2)

/* SMMU: master and common blocks inside the vdec */
#define SMMU_MSTR_BASE		0xf000
#define SMMU_MSTR_GLB_BYPASS	(SMMU_MSTR_BASE + 0x000)
#define SMMU_MSTR_DBG(n)	(SMMU_MSTR_BASE + 0x010 + 4 * (n))
#define SMMU_COMMON_BASE	0x20000
#define SMMU_SCR		(SMMU_COMMON_BASE + 0x000)
#define SMMU_INTSTAT_NS		(SMMU_COMMON_BASE + 0x018)
#define SMMU_SMRX_NS(n)		(SMMU_COMMON_BASE + 0x020 + 4 * (n))
#define SMMU_CB_TTBR0		(SMMU_COMMON_BASE + 0x204)
#define SMMU_CB_TTBCR		(SMMU_COMMON_BASE + 0x20c)
#define SMMU_CB_TTBR_MSB	(SMMU_COMMON_BASE + 0x224)
#define SMMU_ERR_ADDR_MSB_NS	(SMMU_COMMON_BASE + 0x300)
#define SMMU_ERR_RDADDR_NS	(SMMU_COMMON_BASE + 0x304)
#define SMMU_ERR_WRADDR_NS	(SMMU_COMMON_BASE + 0x308)

/* message pool: slots of 4 x 320 bytes, as laid out by the vendor HAL */
#define HIVDEC_MSG_SLOT_BYTES	(4 * 320)
#define HIVDEC_SLOT_UP		0	/* decode report */
#define HIVDEC_SLOT_HEAD	4	/* compressed-frame head info (CFGINFO) */
#define HIVDEC_SLOT_PIC		5	/* picture message */
#define HIVDEC_SLOT_SLICE0	6	/* first slice message */
#define HIVDEC_MAX_SLICES	200
#define HIVDEC_MSG_SLOTS	(HIVDEC_SLOT_SLICE0 + HIVDEC_MAX_SLICES + 1)

/* bus address as stored by the VDH */
#define HIVDEC_ADDR(a)		((u32)((a) >> 4))

struct hivdec_ctx;
struct hivdec_dev;

/* a buffer mapped into the decoder's private IOMMU address space */
struct hivdec_mapping {
	u64 iova;
	size_t size;
};

struct hivdec_aux_buf {
	void *cpu;
	phys_addr_t dma;	/* physical address of the first page */
	size_t size;
	struct hivdec_mapping map;
	void *priv;
};

struct hivdec_mmu {
	struct io_pgtable_cfg cfg;
	struct io_pgtable_ops *ops;
	struct gen_pool *iova_pool;
	struct mutex lock;
	/* ranges freed while the TLB may still hold them, reused after power off */
	struct list_head deferred;
	struct hivdec_aux_buf err_rd, err_wr;
};

struct hivdec_ctrl_desc {
	struct v4l2_ctrl_config cfg;
};

struct hivdec_coded_fmt_ops {
	int (*adjust_fmt)(struct hivdec_ctx *ctx, struct v4l2_format *f);
	int (*start)(struct hivdec_ctx *ctx);
	void (*stop)(struct hivdec_ctx *ctx);
	/* returns >0 if the job finished without hardware (e.g. slice gathered) */
	int (*run)(struct hivdec_ctx *ctx);
	/*
	 * hardware run finished (process context); return >0 if another run
	 * was started for the same job, <0 to fail the job
	 */
	int (*done)(struct hivdec_ctx *ctx, enum vb2_buffer_state state);
	int (*try_ctrl)(struct hivdec_ctx *ctx, struct v4l2_ctrl *ctrl);
	/* bit depth a control asks for (decides NV12 / P010), 0 if unrelated */
	unsigned int (*bit_depth)(const struct v4l2_ctrl *ctrl);
	/*
	 * decode what was gathered for a held CAPTURE buffer before a drain
	 * releases it: 0 = VDH started, >0 = nothing to do
	 */
	int (*flush)(struct hivdec_ctx *ctx);
};

struct hivdec_coded_fmt_desc {
	u32 fourcc;
	struct v4l2_frmsize_stepwise frmsize;
	const struct hivdec_ctrl_desc *ctrls;
	unsigned int num_ctrls;
	const struct hivdec_coded_fmt_ops *ops;
	const u32 *decoded_fmts;
	unsigned int num_decoded_fmts;
	u32 subsystem_flags;
	u32 height_align;	/* of decoded frames (macroblock pairs / CTBs) */
	u32 pad;		/* extra columns and rows kept around decoded frames */
};

/* per capture buffer: decoder-private data kept with the decoded frame */
struct hivdec_decoded_buffer {
	struct v4l2_m2m_buffer base;	/* must be first */
	struct hivdec_mapping map;	/* whole capture plane in VDH space */
	struct hivdec_aux_buf mv;	/* co-located motion vectors (PMV) */
	bool mapped;
};

static inline struct hivdec_decoded_buffer *
vb2_to_hivdec_buf(struct vb2_buffer *vb)
{
	return container_of(vb, struct hivdec_decoded_buffer, base.vb.vb2_buf);
}

struct hivdec_src_buffer {
	struct v4l2_m2m_buffer base;	/* must be first */
	struct hivdec_mapping map;
	bool mapped;
};

static inline struct hivdec_src_buffer *vb2_to_hivdec_src(struct vb2_buffer *vb)
{
	return container_of(vb, struct hivdec_src_buffer, base.vb.vb2_buf);
}

struct hivdec_dev {
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct regulator *media_supply;
	struct regulator *vdec_supply;
	u32 clk_rate[4];		/* high, normal, low, lowest */
	int irq;
	struct mutex vdev_lock;		/* serializes ioctls */
	struct delayed_work watchdog_work;
	struct hivdec_mmu mmu;
	u32 version;
	bool powered;

	/* hardware job state */
	struct hivdec_ctx *run_ctx;
	bool flush_run;			/* VDH run outside an m2m job (drain) */
	struct completion flush_done;
	wait_queue_head_t job_wq;
	u32 irq_status;
	u32 vdh_state;
	u32 dec_cycles;
	struct dentry *debugfs;
	unsigned int debug;
};

struct hivdec_ctx {
	struct v4l2_fh fh;
	struct v4l2_format coded_fmt;
	struct v4l2_format decoded_fmt;
	const struct hivdec_coded_fmt_desc *coded_fmt_desc;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct hivdec_dev *dev;
	struct hivdec_aux_buf msg;	/* message pool */
	unsigned int bit_depth;		/* of the stream, from SPS / frame header */
	bool job_active;
	void *priv;			/* codec state */
};

static inline struct hivdec_ctx *file_to_hivdec_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct hivdec_ctx, fh);
}

/* bytes per sample of the decoded frames: 2 for P010 (10-bit) */
static inline unsigned int hivdec_decoded_bps(const struct hivdec_ctx *ctx)
{
	return ctx->decoded_fmt.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_P010 ? 2 : 1;
}

static inline u32 vdh_read(struct hivdec_dev *vdec, u32 reg)
{
	return readl(vdec->regs + reg);
}

static inline void vdh_write(struct hivdec_dev *vdec, u32 reg, u32 val)
{
	writel(val, vdec->regs + reg);
}

static inline u32 *hivdec_msg_slot(struct hivdec_ctx *ctx, unsigned int slot)
{
	return ctx->msg.cpu + slot * HIVDEC_MSG_SLOT_BYTES;
}

static inline u32 hivdec_msg_slot_addr(struct hivdec_ctx *ctx, unsigned int slot)
{
	return HIVDEC_ADDR(ctx->msg.map.iova + slot * HIVDEC_MSG_SLOT_BYTES);
}

/* hisi_vdec_mmu.c */
int hivdec_mmu_init(struct hivdec_dev *vdec);
void hivdec_mmu_fini(struct hivdec_dev *vdec);
void hivdec_mmu_hw_setup(struct hivdec_dev *vdec);
void hivdec_mmu_powered_off(struct hivdec_dev *vdec);
int hivdec_mmu_map_sgt(struct hivdec_dev *vdec, struct sg_table *sgt,
		       size_t size, struct hivdec_mapping *map);
void hivdec_mmu_unmap(struct hivdec_dev *vdec, struct hivdec_mapping *map);
int hivdec_aux_alloc(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf,
		     size_t size);
void hivdec_aux_free(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf);
void hivdec_aux_sync_for_device(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf);
void hivdec_aux_sync_for_cpu(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf);

/* hisi_vdec.c */
void hivdec_hw_run(struct hivdec_ctx *ctx);
void hivdec_job_finish(struct hivdec_ctx *ctx, enum vb2_buffer_state state);
struct media_request;
void *hivdec_find_control_data(struct hivdec_ctx *ctx, u32 id);

extern const struct hivdec_coded_fmt_ops hivdec_h264_fmt_ops;
extern const struct hivdec_coded_fmt_ops hivdec_hevc_fmt_ops;
extern const struct hivdec_coded_fmt_ops hivdec_vp9_fmt_ops;

#endif /* HISI_VDEC_H_ */
