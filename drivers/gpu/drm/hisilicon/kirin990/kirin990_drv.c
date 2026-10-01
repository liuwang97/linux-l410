// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Kirin 990 display subsystem (DSS v510) DRM driver.
 *
 * The DSS feeds the internal eDP panel through MIPI DSI0 and a TI SN65DSI86
 * DSI-to-eDP bridge. UEFI brings the whole pipeline up (power, clocks, DSI
 * link, bridge, panel, backlight) and scans out its GOP framebuffer through
 * one read channel of overlay OV0. This driver takes that running pipeline
 * over: it reads the mode back from the DSI/LDI registers and re-points the
 * read channel at DRM framebuffers. A full modeset (DSI host + bridge chain)
 * is not implemented yet.
 *
 * The DRM driver name is "kirin" so that Mesa's kmsro pairs it with Panfrost.
 */

#include <linux/aperture.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_crtc.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "kirin990_dss_regs.h"

#define KIRIN_NUM_RCH		8

/*
 * Read channels, indexed by the MCTL channel number. Only channels without a
 * scaler are listed as candidates for the primary plane.
 */
struct kirin_rch {
	u32 dma;	/* DMA block; DFC, DMA_BUF and REG_DEFAULT hang off it */
	u8 smr_first;	/* first SMMU stream (SMR) index */
	u8 smr_num;
};

static const struct kirin_rch kirin_rch[KIRIN_NUM_RCH] = {
	[0] = { 0x52000, 0, 4 },
	[1] = { 0x53000, 4, 1 },
	[2] = { 0x20000, 5, 4 },	/* VG0: scaler + ARSR */
	[3] = { 0x38000, 9, 4 },	/* G0: scaler */
	[4] = { 0x28000, 13, 4 },	/* VG1: scaler */
	[5] = { 0x40000, 17, 4 },	/* G1: scaler */
	[6] = { 0x50000, 21, 1 },
	[7] = { 0x51000, 22, 1 },
};

struct kirin_format {
	u32 fourcc;
	u8 dma_fmt;
	u8 dfc_fmt;
	bool alpha;
};

static const struct kirin_format kirin_formats[] = {
	{ DRM_FORMAT_XRGB8888, DMA_FMT_XRGB8888, DFC_FMT_XRGB8888, false },
	{ DRM_FORMAT_ARGB8888, DMA_FMT_ARGB8888, DFC_FMT_ARGB8888, true },
	{ DRM_FORMAT_XBGR8888, DMA_FMT_XRGB8888, DFC_FMT_XBGR8888, false },
	{ DRM_FORMAT_ABGR8888, DMA_FMT_ARGB8888, DFC_FMT_ABGR8888, true },
	/*
	 * 16 bpp needs the DMA window (16-byte units) and DFC_PIX_IN_NUM
	 * reprogrammed as well; not done by the firmware takeover path.
	 */
};

static const u64 kirin_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

struct kirin_dss {
	struct drm_device drm;
	void __iomem *base;
	int irq;

	struct drm_plane plane;
	struct drm_plane cursor;
	unsigned int cursor_ch;		/* read channel of the cursor plane */
	unsigned int cursor_layer;	/* OV0 layer of the cursor plane */
	u32 rch_ctl, rch_buf_ctrl, rch_bitext;	/* channel defaults, from UEFI */
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	struct drm_display_mode fw_mode;
	unsigned int ch;		/* read channel feeding OV0 */
	unsigned int layer;		/* OV0 layer used by that channel */
	bool smmu_translate;		/* firmware used the DSS SMMU for this channel */
	/* firmware scanout buffer (reserved by UEFI), shown while we're off */
	u32 fw_addr, fw_stride, fw_ctrl, fw_fmt, fw_mif1;
	u32 fw_smr[4];
	bool fw_translate;

	spinlock_t irq_lock;
	u32 underflows;
	struct work_struct recover_work;
	struct delayed_work report_work;
};

#define to_kirin(x) container_of(x, struct kirin_dss, drm)

static bool dump_state = true;
module_param(dump_state, bool, 0444);
MODULE_PARM_DESC(dump_state, "Dump the firmware DSS state at probe");

static bool selftest = true;
module_param(selftest, bool, 0444);
MODULE_PARM_DESC(selftest, "Scan out test patterns at probe and log the output CRCs");

static bool cursor_plane;
module_param(cursor_plane, bool, 0444);
MODULE_PARM_DESC(cursor_plane, "Expose a hardware cursor plane (experimental)");

static inline u32 dss_rd(struct kirin_dss *k, u32 off)
{
	return readl(k->base + off);
}

static inline void dss_wr(struct kirin_dss *k, u32 off, u32 val)
{
	writel(val, k->base + off);
}

static inline void dss_rmw(struct kirin_dss *k, u32 off, u32 mask, u32 val)
{
	dss_wr(k, off, (dss_rd(k, off) & ~mask) | (val & mask));
}

static inline u32 dsi_rd(struct kirin_dss *k, u32 off)
{
	return readl(k->base + DSS_DSI0 + off);
}

static inline void dsi_wr(struct kirin_dss *k, u32 off, u32 val)
{
	writel(val, k->base + DSS_DSI0 + off);
}

/* ------------------------------------------------------------------------ */
/* bring-up diagnostics */

struct dump_range {
	const char *name;
	u32 off;
	u32 len;
};

static const struct dump_range dss_dump_ranges[] = {
	{ "DSI0", DSS_DSI0, 0x280 },
	{ "AIF0", DSS_AIF0, 0x180 },
	{ "MIF", DSS_MIF, 0x1a0 },
	{ "MCTL_SYS", DSS_MCTL_SYS, 0x300 },
	{ "MCTL_CTL0", DSS_MCTL_CTL0, 0x100 },
	{ "DBG", 0x11000, 0x30 },
	{ "GLB", DSS_GLB, 0x80 },
	{ "GLB_INT", DSS_GLB + 0x200, 0x100 },
	{ "GLB_CLK", DSS_GLB + 0x300, 0x20 },
	{ "OVL0", DSS_OVL0, 0x360 },
	{ "DISP_CH0", DSS_DISP_CH0, 0x40 },
	{ "DBUF0", 0x6e000, 0xc0 },
	{ "SMMU", DSS_SMMU, 0xc0 },
	{ "SMMU_CB", DSS_SMMU + 0x200, 0x30 },
	{ "DISP_GLB", 0xa1000, 0x40 },
	{ "PIPE_SW", 0xbe000, 0x40 },
};

static void kirin_dump_block(struct kirin_dss *k, const char *name, u32 off, u32 len)
{
	char line[128];
	int n = 0, cnt = 0;
	u32 a, v;

	drm_info(&k->drm, "KDUMP ## %s %05x+%x\n", name, off, len);
	for (a = 0; a < len; a += 4) {
		v = dss_rd(k, off + a);
		if (!v)
			continue;
		n += scnprintf(line + n, sizeof(line) - n, " %05x=%08x", off + a, v);
		if (++cnt == 6) {
			drm_info(&k->drm, "KDUMP%s\n", line);
			n = cnt = 0;
		}
	}
	if (cnt)
		drm_info(&k->drm, "KDUMP%s\n", line);
}

static void kirin_dump_rch(struct kirin_dss *k, unsigned int ch)
{
	char name[16];
	u32 dma = kirin_rch[ch].dma;

	snprintf(name, sizeof(name), "RCH%u_DMA", ch);
	kirin_dump_block(k, name, dma, 0xe0);
	snprintf(name, sizeof(name), "RCH%u_DFC", ch);
	kirin_dump_block(k, name, dma + DFC_BASE, 0x50);
	snprintf(name, sizeof(name), "RCH%u_BUF", ch);
	kirin_dump_block(k, name, dma + DMA_BUF_BASE, 0x60);
}

static void kirin_dump_state(struct kirin_dss *k)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dss_dump_ranges); i++)
		kirin_dump_block(k, dss_dump_ranges[i].name,
				 dss_dump_ranges[i].off, dss_dump_ranges[i].len);
	for (i = 0; i < KIRIN_NUM_RCH; i++)
		if (dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(i)) ||
		    (dss_rd(k, kirin_rch[i].dma + DMA_CH_CTL) & CH_CTL_EN))
			kirin_dump_rch(k, i);
}

/* ------------------------------------------------------------------------ */
/* firmware state */

/*
 * Rebuild the mode that UEFI programmed. The DSI host timing registers count
 * horizontal periods in lane byte clocks; HRZ_CTRL3 holds the active width in
 * the same unit, which gives the byte clock / pixel clock ratio.
 */
static int kirin_read_fw_mode(struct kirin_dss *k)
{
	struct drm_display_mode *m = &k->fw_mode;
	u32 hact, vact, hact_lbc, hsa, hbp, hline, vsa, vbp, vfp;
	u32 hsw, hbpx, htotal, vtotal, pol;

	hact = (dsi_rd(k, MIPI_LDI_DPI0_HRZ_CTRL2) & 0xfff) + 1;
	vact = (dsi_rd(k, MIPI_LDI_VRT_CTRL2) & 0xfff) + 1;
	hact_lbc = (dsi_rd(k, MIPI_LDI_DPI0_HRZ_CTRL3) & 0xfff) + 1;
	hsa = dsi_rd(k, DSI_VID_HSA_TIME) & 0xfff;
	hbp = dsi_rd(k, DSI_VID_HBP_TIME) & 0xfff;
	hline = dsi_rd(k, DSI_VID_HLINE_TIME) & 0x7fff;
	vsa = dsi_rd(k, DSI_VID_VSA_LINES) & 0x3ff;
	vbp = dsi_rd(k, DSI_VID_VBP_LINES) & 0x3ff;
	vfp = dsi_rd(k, DSI_VID_VFP_LINES) & 0x3ff;
	pol = dsi_rd(k, DSI_DPI_CFG_POL);

	drm_info(&k->drm, "firmware DSI timing: hact %u (%u lbc) hsa %u hbp %u hline %u, vact %u vsa %u vbp %u vfp %u pol %#x\n",
		 hact, hact_lbc, hsa, hbp, hline, vact, vsa, vbp, vfp, pol);

	if (hact < 64 || vact < 64 || !hact_lbc || hline <= hact_lbc)
		return -ENODEV;

	/* convert byte-clock periods to pixels */
	hsw = DIV_ROUND_CLOSEST(hsa * hact, hact_lbc);
	hbpx = DIV_ROUND_CLOSEST(hbp * hact, hact_lbc);
	htotal = DIV_ROUND_CLOSEST(hline * hact, hact_lbc);
	if (htotal < hact + hsw + hbpx + 1)
		htotal = hact + hsw + hbpx + 1;
	vtotal = vact + vsa + vbp + vfp;

	m->hdisplay = hact;
	m->hsync_start = htotal - hsw - hbpx;
	m->hsync_end = htotal - hbpx;
	m->htotal = htotal;
	m->vdisplay = vact;
	m->vsync_start = vact + vfp;
	m->vsync_end = vact + vfp + vsa;
	m->vtotal = vtotal;
	/* the refresh rate is not recoverable from these registers: 60 Hz */
	m->clock = DIV_ROUND_CLOSEST(htotal * vtotal * 60, 1000);
	m->flags = (pol & BIT(2) ? DRM_MODE_FLAG_NHSYNC : DRM_MODE_FLAG_PHSYNC) |
		   (pol & BIT(1) ? DRM_MODE_FLAG_NVSYNC : DRM_MODE_FLAG_PVSYNC);
	m->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(m);

	drm_info(&k->drm, "firmware mode " DRM_MODE_FMT "\n", DRM_MODE_ARG(m));
	return 0;
}

/* Find the read channel and OV0 layer UEFI is scanning out from. */
static int kirin_find_fw_layer(struct kirin_dss *k)
{
	u32 sel = dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL);
	unsigned int layer, ch, i;
	u32 scr, smr;

	for (layer = 0; layer < 7; layer++) {
		ch = (sel >> (4 * (layer + 1))) & 0xf;
		if (ch >= KIRIN_NUM_RCH)
			continue;
		if (!(dss_rd(k, DSS_OVL0 + OV_LAYER(layer) + OV_LAYER_CFG) & BIT(0)))
			continue;
		if (!(dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(ch)) & MCTL_OV_OEN_OV0))
			continue;
		goto found;
	}
	drm_err(&k->drm, "no active OV0 layer (RCH_OV0_SEL %#x)\n", sel);
	return -ENODEV;

found:
	k->ch = ch;
	k->layer = layer;
	k->fw_addr = dss_rd(k, kirin_rch[ch].dma + DMA_DATA_ADDR0);
	k->fw_stride = dss_rd(k, kirin_rch[ch].dma + DMA_STRIDE0);
	k->fw_ctrl = dss_rd(k, kirin_rch[ch].dma + DMA_CTRL);
	k->fw_fmt = dss_rd(k, kirin_rch[ch].dma + DFC_BASE + DFC_DISP_FMT);

	scr = dss_rd(k, DSS_SMMU + SMMU_SCR);
	smr = dss_rd(k, DSS_SMMU + SMMU_SMRX_NS(kirin_rch[ch].smr_first));
	k->smmu_translate = !(scr & SMMU_SCR_GLB_BYPASS) && !(smr & SMMU_SMR_BYPASS);
	k->fw_translate = k->smmu_translate;
	k->fw_mif1 = dss_rd(k, DSS_MIF + MIF_CH(ch) + MIF_CTRL1);
	for (i = 0; i < kirin_rch[ch].smr_num; i++)
		k->fw_smr[i] = dss_rd(k, DSS_SMMU + SMMU_SMRX_NS(kirin_rch[ch].smr_first + i));

	drm_info(&k->drm, "firmware scans out RCH%u on OV0 layer %u: addr %#x stride %u ctrl %#x fmt %#x, SMMU scr %#x smr %#x%s\n",
		 ch, layer, dss_rd(k, kirin_rch[ch].dma + DMA_DATA_ADDR0),
		 dss_rd(k, kirin_rch[ch].dma + DMA_STRIDE0) * 16,
		 dss_rd(k, kirin_rch[ch].dma + DMA_CTRL),
		 dss_rd(k, kirin_rch[ch].dma + DFC_BASE + DFC_DISP_FMT),
		 scr, smr, k->smmu_translate ? " (translating)" : "");

	for (i = 0; i < kirin_rch[ch].smr_num; i++)
		if (dss_rd(k, DSS_SMMU + SMMU_SMRX_NS(kirin_rch[ch].smr_first + i)) != smr)
			drm_warn(&k->drm, "RCH%u SMR%u differs\n", ch, i);

	/* channel settings UEFI left, reused for the other channels we program */
	k->rch_ctl = dss_rd(k, kirin_rch[ch].dma + DMA_CH_CTL) & ~CH_CTL_EN;
	k->rch_buf_ctrl = dss_rd(k, kirin_rch[ch].dma + DMA_BUF_BASE + DMA_BUF_CTRL);
	k->rch_bitext = dss_rd(k, kirin_rch[ch].dma + DFC_BASE + DFC_BITEXT_CTL);

	/* cursor: a scaler-less channel on the layer above the firmware's */
	k->cursor_ch = ch == 6 ? 7 : 6;
	k->cursor_layer = layer + 1;
	if (cursor_plane && k->cursor_layer >= OV_NUM_LAYERS - 1)
		cursor_plane = false;
	return 0;
}

/* ------------------------------------------------------------------------ */
/* scanout programming */

static const struct kirin_format *kirin_find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(kirin_formats); i++)
		if (kirin_formats[i].fourcc == fourcc)
			return &kirin_formats[i];
	return NULL;
}

static void kirin_mutex_lock(struct kirin_dss *k)
{
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX, 1);
}

static void kirin_mutex_unlock(struct kirin_dss *k)
{
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX, 0);
}

/*
 * Point the firmware's read channel at a new buffer. Everything else about the
 * channel (size, AIF/MIF, DMA buffer, OV layer) stays as UEFI set it up, as the
 * framebuffer has the same size as the mode. Must be called with the MCTL
 * mutex held; the hardware latches the channel at the next frame start after
 * the mutex is released.
 */
static void kirin_rch_update(struct kirin_dss *k, dma_addr_t addr, u32 pitch,
			     const struct kirin_format *fmt)
{
	const struct kirin_rch *r = &kirin_rch[k->ch];
	unsigned int i;

	if (k->smmu_translate) {
		/* feed physical addresses: bypass the SMMU for this channel */
		for (i = 0; i < r->smr_num; i++)
			dss_rmw(k, DSS_SMMU + SMMU_SMRX_NS(r->smr_first + i),
				SMMU_SMR_BYPASS, SMMU_SMR_BYPASS);
		dss_rmw(k, r->dma + DMA_CTRL, DMA_CTRL_MMU_EN, 0);
		dss_rmw(k, DSS_MIF + MIF_CH(k->ch) + MIF_CTRL1, ~0u, MIF_CTRL1_BYPASS);
		k->smmu_translate = false;
	}

	dss_rmw(k, r->dma + DMA_CTRL, GENMASK(7, 3), DMA_CTRL_FMT(fmt->dma_fmt));
	dss_wr(k, r->dma + DMA_DATA_ADDR0, lower_32_bits(addr));
	dss_wr(k, r->dma + DMA_STRIDE0, pitch / 16);
	dss_rmw(k, r->dma + DFC_BASE + DFC_DISP_FMT, GENMASK(5, 1), fmt->dfc_fmt << 1);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_FLUSH_EN(k->ch), 1);
}

/*
 * Point the channel back at the firmware framebuffer (UEFI keeps its memory
 * reserved), so that nothing fetches from buffers DRM may free.
 */
static void kirin_rch_restore_fw(struct kirin_dss *k)
{
	const struct kirin_rch *r = &kirin_rch[k->ch];
	unsigned int i;

	kirin_mutex_lock(k);
	for (i = 0; i < r->smr_num; i++)
		dss_wr(k, DSS_SMMU + SMMU_SMRX_NS(r->smr_first + i), k->fw_smr[i]);
	dss_wr(k, DSS_MIF + MIF_CH(k->ch) + MIF_CTRL1, k->fw_mif1);
	dss_wr(k, r->dma + DMA_CTRL, k->fw_ctrl);
	dss_wr(k, r->dma + DMA_DATA_ADDR0, k->fw_addr);
	dss_wr(k, r->dma + DMA_STRIDE0, k->fw_stride);
	dss_wr(k, r->dma + DFC_BASE + DFC_DISP_FMT, k->fw_fmt);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_FLUSH_EN(k->ch), 1);
	kirin_mutex_unlock(k);
	k->smmu_translate = k->fw_translate;
}

/* ------------------------------------------------------------------------ */
/* plane */

static int kirin_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = new->fb;
	int ret;

	if (!new->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new->crtc);
	ret = drm_atomic_helper_check_plane_state(new, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret || !new->visible)
		return ret;

	/* the firmware layer is programmed for a full-screen source */
	if (new->src_x || new->src_y ||
	    (new->src_w >> 16) != crtc_state->mode.hdisplay ||
	    (new->src_h >> 16) != crtc_state->mode.vdisplay)
		return -EINVAL;
	if (fb->pitches[0] % 16 || fb->offsets[0] % 16)
		return -EINVAL;
	if (!kirin_find_format(fb->format->format))
		return -EINVAL;
	return 0;
}

static void kirin_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(plane->dev);
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new->fb;
	dma_addr_t addr;

	if (!new->visible || !fb)
		return;

	addr = drm_fb_dma_get_gem_addr(fb, new, 0);
	kirin_rch_update(k, addr, fb->pitches[0],
			 kirin_find_format(fb->format->format));
}

static const struct drm_plane_helper_funcs kirin_plane_helper_funcs = {
	.atomic_check = kirin_plane_atomic_check,
	.atomic_update = kirin_plane_atomic_update,
};

static bool kirin_plane_format_mod_supported(struct drm_plane *plane,
					     u32 format, u64 modifier)
{
	return modifier == DRM_FORMAT_MOD_LINEAR;
}

static const struct drm_plane_funcs kirin_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.format_mod_supported = kirin_plane_format_mod_supported,
};

/* ------------------------------------------------------------------------ */
/* cursor plane: a scaler-less read channel on an upper OV0 layer */

#define KIRIN_CURSOR_MAX	256

/* OV layer blending (vendor g_ovl_alpha): opaque source, premultiplied over */
#define OV_ALPHA_OPAQUE		0x01004000
#define OV_ALPHA_PREMULT_OVER	0xc2004000
#define OV_ALPHA_A_OPAQUE	0x03ff03ff

static int kirin_cursor_atomic_check(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	int ret;

	if (!new->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new->crtc);
	ret = drm_atomic_helper_check_plane_state(new, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  true, true);
	if (ret || !new->visible)
		return ret;

	if (new->fb->width > KIRIN_CURSOR_MAX ||
	    new->fb->height > KIRIN_CURSOR_MAX ||
	    new->fb->pitches[0] % 16 || new->fb->offsets[0] % 16 ||
	    new->fb->format->cpp[0] != 4)
		return -EINVAL;
	return 0;
}

static void kirin_cursor_hide(struct kirin_dss *k)
{
	const struct kirin_rch *r = &kirin_rch[k->cursor_ch];
	u32 sel;

	dss_rmw(k, DSS_OVL0 + OV_LAYER(k->cursor_layer) + OV_LAYER_CFG, BIT(0), 0);
	sel = dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL);
	sel |= 0xf << (4 * (k->cursor_layer + 1));
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL, sel);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(k->cursor_ch), 0);
	dss_rmw(k, r->dma + DMA_CH_CTL, CH_CTL_EN, 0);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_FLUSH_EN(k->cursor_ch), 1);
	dss_wr(k, DSS_MCTL_SYS + MCTL_OV0_FLUSH_EN, 0xd);
}

/*
 * Program a whole read channel + OV layer (called with the MCTL mutex held).
 * The DMA fetches whole 16-byte units, so the source window is widened to a
 * multiple of 4 pixels and the DFC clips the extra pixels off again. The DMA
 * starts at the first fetched pixel, with zero DMA window offsets.
 */
static void kirin_cursor_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(plane->dev);
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	const struct kirin_rch *r = &kirin_rch[k->cursor_ch];
	struct drm_framebuffer *fb = new->fb;
	const struct kirin_format *fmt;
	u32 sx, sy, w, h, dx, dy, ax0, ax1, clip_l, clip_r, sel, alpha;
	dma_addr_t addr;
	unsigned int i;

	if (!new->visible || !fb) {
		kirin_cursor_hide(k);
		return;
	}

	fmt = kirin_find_format(fb->format->format);
	sx = new->src.x1 >> 16;
	sy = new->src.y1 >> 16;
	w = drm_rect_width(&new->dst);
	h = drm_rect_height(&new->dst);
	dx = new->dst.x1;
	dy = new->dst.y1;
	ax0 = round_down(sx, 4);
	ax1 = round_up(sx + w, 4);
	clip_l = sx - ax0;
	clip_r = ax1 - (sx + w);
	/* new->src is the source rectangle after clipping to the CRTC */
	addr = drm_fb_dma_get_gem_obj(fb, 0)->dma_addr + fb->offsets[0] +
	       sy * fb->pitches[0] + ax0 * 4;

	for (i = 0; i < r->smr_num; i++)
		dss_wr(k, DSS_SMMU + SMMU_SMRX_NS(r->smr_first + i), SMMU_SMR_BYPASS);
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX_RCH(k->cursor_ch), 1);

	dss_wr(k, r->dma + DMA_OFT_X0, 0);
	dss_wr(k, r->dma + DMA_OFT_Y0, 0);
	dss_wr(k, r->dma + DMA_OFT_X1, (ax1 - ax0) / 4 - 1);
	dss_wr(k, r->dma + DMA_OFT_Y1, h - 1);
	dss_wr(k, r->dma + DMA_MASK0, 0);
	dss_wr(k, r->dma + DMA_MASK1, 0);
	dss_wr(k, r->dma + DMA_STRETCH_SIZE_VRT, h - 1);
	dss_wr(k, r->dma + DMA_CTRL, DMA_CTRL_FMT(fmt->dma_fmt));
	dss_wr(k, r->dma + DMA_TILE_SCRAM, 0);
	dss_wr(k, r->dma + DMA_DATA_ADDR0, lower_32_bits(addr));
	dss_wr(k, r->dma + DMA_STRIDE0, fb->pitches[0] / 16);
	dss_wr(k, r->dma + DMA_STRETCH_STRIDE0, 0);
	dss_wr(k, r->dma + DMA_BUF_BASE + DMA_BUF_CTRL, k->rch_buf_ctrl);
	dss_wr(k, r->dma + DMA_CH_CTL, k->rch_ctl | CH_CTL_EN);

	dss_wr(k, r->dma + DFC_BASE + DFC_DISP_SIZE, ((ax1 - ax0 - 1) << 16) | (h - 1));
	dss_wr(k, r->dma + DFC_BASE + DFC_PIX_IN_NUM, 0);
	dss_wr(k, r->dma + DFC_BASE + DFC_DISP_FMT, fmt->dfc_fmt << 1);
	dss_wr(k, r->dma + DFC_BASE + DFC_CLIP_CTL_HRZ, (clip_l << 16) | clip_r);
	dss_wr(k, r->dma + DFC_BASE + DFC_CLIP_CTL_VRZ, 0);
	dss_wr(k, r->dma + DFC_BASE + DFC_CTL_CLIP_EN, 1);
	dss_wr(k, r->dma + DFC_BASE + DFC_ICG_MODULE, 1);
	dss_wr(k, r->dma + DFC_BASE + DFC_DITHER_ENABLE, 0);
	dss_wr(k, r->dma + DFC_BASE + DFC_PADDING_CTL, 0);
	dss_wr(k, r->dma + DFC_BASE + DFC_BITEXT_CTL, k->rch_bitext);

	alpha = fmt->alpha && new->pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE ?
		OV_ALPHA_PREMULT_OVER : OV_ALPHA_OPAQUE;
	i = DSS_OVL0 + OV_LAYER(k->cursor_layer);
	dss_wr(k, i + OV_LAYER_POS, (dy << 16) | dx);
	dss_wr(k, i + OV_LAYER_SIZE, ((dy + h - 1) << 16) | (dx + w - 1));
	dss_wr(k, i + OV_LAYER_PATTERN_RGB, 0);
	dss_wr(k, i + OV_LAYER_ALPHA_MODE, alpha);
	dss_wr(k, i + OV_LAYER_ALPHA_A, OV_ALPHA_A_OPAQUE);
	dss_wr(k, i + OV_LAYER_PSPOS, (dy << 16) | dx);
	dss_wr(k, i + OV_LAYER_PEPOS, ((dy + h - 1) << 16) | (dx + w - 1));
	dss_wr(k, i + OV_LAYER_CFG, 1);

	sel = dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL);
	sel &= ~(0xf << (4 * (k->cursor_layer + 1)));
	sel |= k->cursor_ch << (4 * (k->cursor_layer + 1));
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL, sel);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_STARTY(k->cursor_ch), dy | (8 << 16));
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(k->cursor_ch), MCTL_OV_OEN_OV0);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_FLUSH_EN(k->cursor_ch), 1);
	dss_wr(k, DSS_MCTL_SYS + MCTL_OV0_FLUSH_EN, 0xd);
}

static void kirin_cursor_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	kirin_cursor_hide(to_kirin(plane->dev));
}

static const struct drm_plane_helper_funcs kirin_cursor_helper_funcs = {
	.atomic_check = kirin_cursor_atomic_check,
	.atomic_update = kirin_cursor_atomic_update,
	.atomic_disable = kirin_cursor_atomic_disable,
};

/* ------------------------------------------------------------------------ */
/* CRTC */

static void kirin_irq_mask(struct kirin_dss *k, u32 bits, bool mask)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&k->irq_lock, flags);
	v = dsi_rd(k, MIPI_LDI_CPU_ITF_INT_MSK);
	v = mask ? v | bits : v & ~bits;
	dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK, v);
	spin_unlock_irqrestore(&k->irq_lock, flags);
}

static int kirin_crtc_enable_vblank(struct drm_crtc *crtc)
{
	kirin_irq_mask(to_kirin(crtc->dev), LDI_INT_VSYNC, false);
	return 0;
}

static void kirin_crtc_disable_vblank(struct drm_crtc *crtc)
{
	kirin_irq_mask(to_kirin(crtc->dev), LDI_INT_VSYNC, true);
}

static int kirin_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct kirin_dss *k = to_kirin(crtc->dev);

	if (!cs->enable)
		return 0;
	/* no modeset support yet: only the firmware mode */
	if (!drm_mode_equal(&cs->mode, &k->fw_mode))
		return -EINVAL;
	/* the firmware layer can't be turned off without a modeset path */
	if (!(cs->plane_mask & drm_plane_mask(crtc->primary)))
		return -EINVAL;
	return 0;
}

static void kirin_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	drm_crtc_vblank_on(crtc);
}

static void kirin_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(crtc->dev);

	/*
	 * Without a modeset path the pipeline keeps running: stop fetching
	 * from buffers that may be freed after this commit. (Hiding the OV
	 * layer instead led to LDI underflows.)
	 */
	if (cursor_plane) {
		kirin_mutex_lock(k);
		kirin_cursor_hide(k);
		kirin_mutex_unlock(k);
	}
	kirin_rch_restore_fw(k);
	drm_crtc_wait_one_vblank(crtc);
	drm_crtc_vblank_off(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void kirin_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	kirin_mutex_lock(to_kirin(crtc->dev));
}

static void kirin_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;

	kirin_mutex_unlock(k);

	if (event) {
		crtc->state->event = NULL;
		spin_lock_irq(&crtc->dev->event_lock);
		if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs kirin_crtc_helper_funcs = {
	.atomic_check = kirin_crtc_atomic_check,
	.atomic_begin = kirin_crtc_atomic_begin,
	.atomic_flush = kirin_crtc_atomic_flush,
	.atomic_enable = kirin_crtc_atomic_enable,
	.atomic_disable = kirin_crtc_atomic_disable,
};

static const struct drm_crtc_funcs kirin_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = kirin_crtc_enable_vblank,
	.disable_vblank = kirin_crtc_disable_vblank,
};

static irqreturn_t kirin_dss_irq(int irq, void *data)
{
	struct kirin_dss *k = data;
	u32 ints, msk;

	ints = dsi_rd(k, MIPI_LDI_CPU_ITF_INTS);
	dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, ints);
	msk = dsi_rd(k, MIPI_LDI_CPU_ITF_INT_MSK);
	ints &= ~msk;
	if (!ints)
		return IRQ_NONE;

	if (ints & LDI_INT_VSYNC)
		drm_crtc_handle_vblank(&k->crtc);
	if (ints & LDI_INT_UNFLOW) {
		/*
		 * The underflow status stays set until the pipeline is reset:
		 * mask it, stop the LDI and recover from process context.
		 */
		spin_lock(&k->irq_lock);
		dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK,
		       dsi_rd(k, MIPI_LDI_CPU_ITF_INT_MSK) | LDI_INT_UNFLOW);
		spin_unlock(&k->irq_lock);
		k->underflows++;
		schedule_work(&k->recover_work);
	}
	return IRQ_HANDLED;
}

/*
 * LDI underflow recovery, as the vendor driver does it: clear the MCTL,
 * wait for the D-PHY lanes to reach stop state, cycle the DSI host and
 * restart the LDI.
 */
static void kirin_recover_work(struct work_struct *work)
{
	struct kirin_dss *k = container_of(work, struct kirin_dss, recover_work);
	u32 v;
	int ret;

	drm_err(&k->drm, "LDI underflow #%u, recovering\n", k->underflows);
	kirin_dump_block(k, "RCH_DMA", kirin_rch[k->ch].dma, 0xe0);
	kirin_dump_block(k, "OVL0", DSS_OVL0, 0x60);
	kirin_dump_block(k, "MCTL_CTL0", DSS_MCTL_CTL0, 0x70);

	dsi_wr(k, MIPI_LDI_CTRL, dsi_rd(k, MIPI_LDI_CTRL) & ~LDI_EN);

	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_CLEAR, 1);
	ret = readl_poll_timeout(k->base + DSS_MCTL_CTL0 + MCTL_CTL_STATUS, v,
				 !(v & BIT(4)), 1, 1000);
	if (ret)
		drm_err(&k->drm, "MCTL clear timeout, status %#x\n", v);

	/* request stop state on all four lanes, wait for it */
	dsi_wr(k, DSI_DPHYTX_CTRL, BIT(0) | (BIT(0) << 3));
	ret = readl_poll_timeout(k->base + DSS_DSI0 + DSI_DPHYTX_TRSTOP_FLAG, v,
				 v & BIT(0), 2, 1000);
	dsi_wr(k, DSI_DPHYTX_CTRL, 0);
	if (ret)
		drm_err(&k->drm, "D-PHY stop state timeout\n");

	dsi_wr(k, DSI_PWR_UP, 0);
	udelay(5);
	dsi_wr(k, DSI_PWR_UP, 1);

	dsi_wr(k, MIPI_LDI_CTRL, dsi_rd(k, MIPI_LDI_CTRL) | LDI_EN);

	msleep(50);
	dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, LDI_INT_UNFLOW);
	if (k->underflows < 50)
		kirin_irq_mask(k, LDI_INT_UNFLOW, false);
	else
		drm_err(&k->drm, "too many underflows, leaving the interrupt masked\n");
}

/* ------------------------------------------------------------------------ */
/* connector: the panel as UEFI left it */

static int kirin_connector_get_modes(struct drm_connector *connector)
{
	struct kirin_dss *k = to_kirin(connector->dev);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &k->fw_mode);
	if (!mode)
		return 0;
	drm_mode_probed_add(connector, mode);
	return 1;
}

static const struct drm_connector_helper_funcs kirin_connector_helper_funcs = {
	.get_modes = kirin_connector_get_modes,
};

static const struct drm_connector_funcs kirin_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_encoder_funcs kirin_encoder_funcs = {
};

/* ------------------------------------------------------------------------ */
/* self test: prove that the scanout follows our buffers */

/* CRC units: GLB LDI0, DBG OV0, DISP_CH0 and the DSI0 debug CRC */
static void kirin_crc_enable(struct kirin_dss *k)
{
	/* frame counts first (0 may mean "never"), then the enables */
	dss_wr(k, DSS_GLB + 0x410, 1);
	dss_wr(k, 0x11018, 1);
	dss_wr(k, DSS_DISP_CH0 + 0x34, 1);
	dss_wr(k, DSS_GLB + 0x40c, 1);
	dss_wr(k, 0x1100c, 1);
	dss_wr(k, DSS_DISP_CH0 + 0x30, 1);
	dsi_wr(k, 0x1a4, 1);
}

static void kirin_crc_snapshot(struct kirin_dss *k, const char *what)
{
	drm_info(&k->drm, "selftest %s: crc glb %08x/%u dbg %08x/%u dispch %08x/%u dsi %08x ctl %#x, dma addr %08x, ldi ints %#x, rch dbg ints %#x\n",
		 what,
		 dss_rd(k, DSS_GLB + 0x404), dss_rd(k, DSS_GLB + 0x410),
		 dss_rd(k, 0x11000), dss_rd(k, 0x11018),
		 dss_rd(k, DSS_DISP_CH0 + 0x38), dss_rd(k, DSS_DISP_CH0 + 0x34),
		 dsi_rd(k, 0x1a8), dsi_rd(k, 0x1a4),
		 dss_rd(k, kirin_rch[k->ch].dma + DMA_DATA_ADDR0),
		 dsi_rd(k, MIPI_LDI_CPU_ITF_INTS),
		 dss_rd(k, 0x11254 + 8 * k->ch));
}

static void kirin_selftest(struct kirin_dss *k)
{
	static const u32 colors[] = { 0x00ff0000, 0x000000ff, 0x00ff0000 };
	const struct drm_display_mode *m = &k->fw_mode;
	u32 pitch = ALIGN(m->hdisplay * 4, 64);
	size_t size = (size_t)pitch * m->vdisplay;
	dma_addr_t dma;
	u32 *buf;
	unsigned int i, x, y;

	buf = dma_alloc_wc(k->drm.dev, size, &dma, GFP_KERNEL);
	if (!buf) {
		drm_warn(&k->drm, "selftest: no memory\n");
		return;
	}

	kirin_crc_enable(k);
	msleep(50);
	kirin_crc_snapshot(k, "firmware fb");

	for (i = 0; i < ARRAY_SIZE(colors); i++) {
		char what[32];

		for (y = 0; y < m->vdisplay; y++)
			for (x = 0; x < m->hdisplay; x++)
				buf[y * pitch / 4 + x] =
					(x < 64 && y < 64) ? 0x00ffffff : colors[i];
		wmb();
		kirin_mutex_lock(k);
		kirin_rch_update(k, dma, pitch, kirin_find_format(DRM_FORMAT_XRGB8888));
		kirin_mutex_unlock(k);
		msleep(100);
		snprintf(what, sizeof(what), "color %06x", colors[i]);
		kirin_crc_snapshot(k, what);
		msleep(50);
		kirin_crc_snapshot(k, what);
	}

	/* back to the firmware framebuffer until DRM clients take over */
	kirin_rch_restore_fw(k);
	msleep(50);
	kirin_crc_snapshot(k, "firmware fb again");

	dma_free_wc(k->drm.dev, size, buf, dma);
}

static int kirin_state_show(struct seq_file *m, void *arg)
{
	struct drm_debugfs_entry *entry = m->private;
	struct kirin_dss *k = to_kirin(entry->dev);
	const struct kirin_rch *r = &kirin_rch[k->ch];
	u32 l = DSS_OVL0 + OV_LAYER(k->layer);

	seq_printf(m, "rch %u layer %u underflows %u vblanks %llu\n", k->ch,
		   k->layer, k->underflows, drm_crtc_vblank_count(&k->crtc));
	seq_printf(m, "dma addr %08x stride %u ctrl %#x ch_ctl %#x oft %u,%u-%u,%u\n",
		   dss_rd(k, r->dma + DMA_DATA_ADDR0), dss_rd(k, r->dma + DMA_STRIDE0) * 16,
		   dss_rd(k, r->dma + DMA_CTRL), dss_rd(k, r->dma + DMA_CH_CTL),
		   dss_rd(k, r->dma + DMA_OFT_X0), dss_rd(k, r->dma + DMA_OFT_Y0),
		   dss_rd(k, r->dma + DMA_OFT_X1), dss_rd(k, r->dma + DMA_OFT_Y1));
	seq_printf(m, "dfc size %#x fmt %#x clip %#x/%#x\n",
		   dss_rd(k, r->dma + DFC_BASE + DFC_DISP_SIZE),
		   dss_rd(k, r->dma + DFC_BASE + DFC_DISP_FMT),
		   dss_rd(k, r->dma + DFC_BASE + DFC_CLIP_CTL_HRZ),
		   dss_rd(k, r->dma + DFC_BASE + DFC_CTL_CLIP_EN));
	seq_printf(m, "ov size %#x gcfg %#x layer pos %#x size %#x alpha %#x cfg %#x\n",
		   dss_rd(k, DSS_OVL0 + OV_SIZE), dss_rd(k, DSS_OVL0 + OV_GCFG),
		   dss_rd(k, l + OV_LAYER_POS), dss_rd(k, l + OV_LAYER_SIZE),
		   dss_rd(k, l + OV_LAYER_ALPHA_MODE), dss_rd(k, l + OV_LAYER_CFG));
	seq_printf(m, "mctl sel %#x oen %#x mutex %#x flush_status %#x status %#x\n",
		   dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL),
		   dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(k->ch)),
		   dss_rd(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX),
		   dss_rd(k, DSS_MCTL_CTL0 + MCTL_CTL_FLUSH_STATUS),
		   dss_rd(k, DSS_MCTL_CTL0 + MCTL_CTL_STATUS));
	seq_printf(m, "ldi ctrl %#x ints %#x msk %#x smmu scr %#x smr %#x\n",
		   dsi_rd(k, MIPI_LDI_CTRL), dsi_rd(k, MIPI_LDI_CPU_ITF_INTS),
		   dsi_rd(k, MIPI_LDI_CPU_ITF_INT_MSK), dss_rd(k, DSS_SMMU + SMMU_SCR),
		   dss_rd(k, DSS_SMMU + SMMU_SMRX_NS(r->smr_first)));
	seq_printf(m, "crc glb %08x dbg %08x dispch %08x dsi %08x\n",
		   dss_rd(k, DSS_GLB + 0x404), dss_rd(k, 0x11000),
		   dss_rd(k, DSS_DISP_CH0 + 0x38), dsi_rd(k, 0x1a8));
	return 0;
}

static void kirin_report_work(struct work_struct *work)
{
	struct kirin_dss *k = container_of(work, struct kirin_dss,
					   report_work.work);
	static int runs;

	kirin_crc_snapshot(k, "report");
	drm_info(&k->drm, "report: vblanks %llu underflows %u\n",
		 drm_crtc_vblank_count(&k->crtc), k->underflows);
	if (++runs < 4)
		schedule_delayed_work(&k->report_work, 3 * HZ);
}

/* ------------------------------------------------------------------------ */
/* device */

static const struct drm_mode_config_funcs kirin_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(kirin_fops);

static const struct drm_driver kirin_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &kirin_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "kirin",
	.desc = "HiSilicon Kirin 990 DSS",
	.major = 1,
	.minor = 0,
};

static int kirin_modeset_init(struct kirin_dss *k)
{
	struct drm_device *drm = &k->drm;
	u32 formats[ARRAY_SIZE(kirin_formats)];
	unsigned int i;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = 16;
	drm->mode_config.min_height = 16;
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &kirin_mode_config_funcs;

	for (i = 0; i < ARRAY_SIZE(kirin_formats); i++)
		formats[i] = kirin_formats[i].fourcc;

	ret = drm_universal_plane_init(drm, &k->plane, 0, &kirin_plane_funcs,
				       formats, ARRAY_SIZE(formats),
				       kirin_modifiers, DRM_PLANE_TYPE_PRIMARY,
				       NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&k->plane, &kirin_plane_helper_funcs);

	if (cursor_plane) {
		ret = drm_universal_plane_init(drm, &k->cursor, 0, &kirin_plane_funcs,
					       formats, ARRAY_SIZE(formats),
					       kirin_modifiers, DRM_PLANE_TYPE_CURSOR,
					       NULL);
		if (ret)
			return ret;
		drm_plane_helper_add(&k->cursor, &kirin_cursor_helper_funcs);
		drm_plane_create_blend_mode_property(&k->cursor,
				BIT(DRM_MODE_BLEND_PIXEL_NONE) |
				BIT(DRM_MODE_BLEND_PREMULTI));
		drm->mode_config.cursor_width = KIRIN_CURSOR_MAX;
		drm->mode_config.cursor_height = KIRIN_CURSOR_MAX;
	}

	ret = drmm_crtc_init_with_planes(drm, &k->crtc, &k->plane,
					 cursor_plane ? &k->cursor : NULL,
					 &kirin_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&k->crtc, &kirin_crtc_helper_funcs);

	ret = drmm_encoder_init(drm, &k->encoder, &kirin_encoder_funcs,
				DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;
	k->encoder.possible_crtcs = drm_crtc_mask(&k->crtc);

	ret = drmm_connector_init(drm, &k->connector, &kirin_connector_funcs,
				  DRM_MODE_CONNECTOR_eDP, NULL);
	if (ret)
		return ret;
	drm_connector_helper_add(&k->connector, &kirin_connector_helper_funcs);
	k->connector.status = connector_status_connected;
	ret = drm_connector_attach_encoder(&k->connector, &k->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	return 0;
}

static void kirin_regulator_disable(void *data)
{
	regulator_disable(data);
}

/*
 * Keep the power domains and clocks UEFI turned on for the display enabled, so
 * that they are not switched off as unused once their drivers are present.
 * Both are optional: without the clock/regulator drivers the hardware simply
 * stays as the firmware left it.
 */
static int kirin_dss_claim_resources(struct kirin_dss *k)
{
	static const char * const supplies[] = {
		"regulator_media_subsys", "regulator_dsssubsys",
	};
	/* what the vendor driver keeps enabled for the DSI0 panel path */
	static const char * const clocks[] = {
		"aclk_dss", "pclk_dss", "clk_edc0", "clk_dss_axi_mm",
		"clk_txdphy0_ref", "clk_txdphy0_cfg", "pclk_dsi0",
	};
	struct device *dev = k->drm.dev;
	struct regulator *reg;
	struct clk *clk;
	unsigned int i, held = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(supplies); i++) {
		reg = devm_regulator_get_optional(dev, supplies[i]);
		if (IS_ERR(reg)) {
			if (PTR_ERR(reg) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(reg), "%s\n", supplies[i]);
		}
		ret = regulator_enable(reg);
		if (ret)
			return dev_err_probe(dev, ret, "enabling %s\n", supplies[i]);
		ret = devm_add_action_or_reset(dev, kirin_regulator_disable, reg);
		if (ret)
			return ret;
	}

	/*
	 * Clocks left running by UEFI stay on with clk_ignore_unused anyway, so
	 * a clock that can't be had is not fatal.
	 */
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		clk = devm_clk_get_optional_enabled(dev, clocks[i]);
		if (IS_ERR(clk))
			drm_warn(&k->drm, "clock %s: %pe\n", clocks[i], clk);
		else if (clk)
			held++;
	}
	if (held)
		drm_info(&k->drm, "holding %u DSS clocks\n", held);
	return 0;
}

static int kirin_dss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct kirin_dss *k;
	int ret;

	k = devm_drm_dev_alloc(dev, &kirin_drm_driver, struct kirin_dss, drm);
	if (IS_ERR(k))
		return PTR_ERR(k);
	platform_set_drvdata(pdev, k);
	spin_lock_init(&k->irq_lock);

	k->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(k->base))
		return PTR_ERR(k->base);

	/* the DSI0/LDI interrupt carries the primary panel's vsync */
	k->irq = platform_get_irq(pdev, 3);
	if (k->irq < 0)
		return k->irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = kirin_dss_claim_resources(k);
	if (ret)
		return ret;

	if (dump_state)
		kirin_dump_state(k);

	if (!(dsi_rd(k, MIPI_LDI_CTRL) & LDI_EN)) {
		dev_err(dev, "display not running (LDI off); cold start not supported yet\n");
		return -ENODEV;
	}
	ret = kirin_read_fw_mode(k);
	if (ret)
		return dev_err_probe(dev, ret, "can't read the firmware mode\n");
	ret = kirin_find_fw_layer(k);
	if (ret)
		return ret;

	if (selftest)
		kirin_selftest(k);

	ret = kirin_modeset_init(k);
	if (ret)
		return ret;

	INIT_WORK(&k->recover_work, kirin_recover_work);

	/* mask everything but vsync/underflow before hooking the interrupt */
	dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK, ~(u32)(LDI_INT_UNFLOW | LDI_INT_VSYNC));
	dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, ~0u);
	ret = devm_request_irq(dev, k->irq, kirin_dss_irq, 0, "kirin-dss", k);
	if (ret)
		return ret;

	ret = aperture_remove_all_conflicting_devices(kirin_drm_driver.name);
	if (ret)
		return ret;

	drm_debugfs_add_file(&k->drm, "kirin_state", kirin_state_show, NULL);

	ret = drm_dev_register(&k->drm, 0);
	if (ret)
		return ret;

	drm_client_setup(&k->drm, NULL);

	if (selftest) {
		INIT_DELAYED_WORK(&k->report_work, kirin_report_work);
		schedule_delayed_work(&k->report_work, 2 * HZ);
	}
	return 0;
}

static void kirin_dss_remove(struct platform_device *pdev)
{
	struct kirin_dss *k = platform_get_drvdata(pdev);

	if (selftest)
		cancel_delayed_work_sync(&k->report_work);
	drm_dev_unplug(&k->drm);
	drm_atomic_helper_shutdown(&k->drm);
	cancel_work_sync(&k->recover_work);
}

static void kirin_dss_shutdown(struct platform_device *pdev)
{
	struct kirin_dss *k = platform_get_drvdata(pdev);

	if (selftest)
		cancel_delayed_work_sync(&k->report_work);
	drm_atomic_helper_shutdown(&k->drm);
}

static const struct of_device_id kirin_dss_of_match[] = {
	{ .compatible = "hisilicon,kunpeng902-dpe" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin_dss_of_match);

static struct platform_driver kirin_dss_driver = {
	.probe = kirin_dss_probe,
	.remove = kirin_dss_remove,
	.shutdown = kirin_dss_shutdown,
	.driver = {
		.name = "kirin990-dss",
		.of_match_table = kirin_dss_of_match,
	},
};
module_platform_driver(kirin_dss_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 display subsystem DRM driver");
MODULE_LICENSE("GPL");
