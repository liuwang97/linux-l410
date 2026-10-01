// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Kirin 990 display subsystem (DSS v510) DRM driver.
 *
 * The DSS feeds the internal eDP panel through MIPI DSI0 and a TI SN65DSI86
 * DSI-to-eDP bridge. UEFI brings the whole pipeline up (power, clocks, DSI
 * link, bridge, panel, backlight) and scans out its GOP framebuffer through
 * one read channel of overlay OV0. This driver takes that running pipeline
 * over at probe: it reads the mode back from the DSI/LDI registers and
 * re-points the read channel at DRM framebuffers.
 *
 * Disabling the CRTC (DPMS off, suspend) powers the whole chain down the way
 * the vendor kernel does: backlight, eDP stream, DSI link and D-PHY, bridge
 * and panel supplies, then the DSS clocks and power domains. Enabling it
 * again sets everything up from reset (kirin990_power.c, kirin990_dsi.c,
 * kirin990_edp.c).
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
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/soc/hisilicon/l410-perf.h>

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
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_prime.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "kirin990_drv.h"

/*
 * Read channels, indexed by the MCTL channel number. Only channels without a
 * scaler are listed as candidates for the primary plane.
 */
const struct kirin_rch kirin_rch[KIRIN_NUM_RCH] = {
	[0] = { 0x52000, 0, 4 },
	[1] = { 0x53000, 4, 1 },
	[2] = { 0x20000, 5, 4 },	/* VG0: scaler + ARSR */
	[3] = { 0x38000, 9, 4 },	/* G0: scaler */
	[4] = { 0x28000, 13, 4 },	/* VG1: scaler */
	[5] = { 0x40000, 17, 4 },	/* G1: scaler */
	[6] = { 0x50000, 21, 1 },
	[7] = { 0x51000, 22, 1 },
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

/* nonblocking commits are handed to the SCHED_FIFO worker in this state */
struct kirin_atomic_state {
	struct drm_atomic_state base;
	struct kthread_work hw_work;
};

#define to_kirin_state(s) container_of(s, struct kirin_atomic_state, base)

static bool dump_state = true;
module_param(dump_state, bool, 0444);
MODULE_PARM_DESC(dump_state, "Dump the firmware DSS state at probe");

static bool selftest = true;
module_param(selftest, bool, 0444);
MODULE_PARM_DESC(selftest, "Scan out test patterns at probe and log the output CRCs");

/*
 * Verified on the L410 with Plasma (KWin uses it): no LDI underflow through
 * cursor moves, shape changes, hiding, window drags and maximize cycles.
 */
static bool cursor_plane = true;
module_param(cursor_plane, bool, 0444);
MODULE_PARM_DESC(cursor_plane, "Expose a hardware cursor plane");

static bool rt_commit = true;
module_param(rt_commit, bool, 0444);
MODULE_PARM_DESC(rt_commit, "Run nonblocking commits from a SCHED_FIFO thread");

static bool vblank_filter = true;
module_param(vblank_filter, bool, 0644);
MODULE_PARM_DESC(vblank_filter, "Filter the interrupt latency out of vblank timestamps");

/*
 * The DSS scans out through a 32-bit DMA window without an IOMMU; buffers of
 * other drivers (e.g. Panfrost's) are neither contiguous nor below 4 GiB, and
 * mapping them bounces through swiotlb for milliseconds before failing.
 */
static bool foreign_import;
module_param(foreign_import, bool, 0644);
MODULE_PARM_DESC(foreign_import, "Try to import dma-bufs of other drivers");

/*
 * How far a disabled CRTC (DPMS off, suspend) powers down:
 *  0: nothing; the scanout goes back to the UEFI framebuffer, as before
 *     full modesets were supported (only at boot: the UEFI pipeline must
 *     not have been power cycled yet)
 *  1: backlight, eDP panel and bridge, DSI link; the DSS stays powered and
 *     clocked with the LDI stopped
 *  2: and the DSS clocks
 *  3: and the DSS power domain (default)
 *  4: and the vivobus and media1 domains behind it, as the vendor kernel
 *     does. On the L410 the machine hangs hard shortly after (without the
 *     DSS touching anything): something else still depends on them.
 *
 * Measured on battery at backlight 50/100: screen
 * off 3.1-3.6 W -> 1.46-1.48 W at levels 1 and 2 (the DSS clocks have other
 * users and stay on), 1.31 W at level 3.
 */
static int power_off = KIRIN_OFF_DSS;
module_param(power_off, int, 0644);
MODULE_PARM_DESC(power_off, "Power-down depth when the display is off (0-4, default 3; 4 hangs the L410)");

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
 * The refresh rate is not recoverable from the timing registers (the pixel
 * clock comes from the D-PHY PLL), and it is not 60 Hz: the L410 panel runs
 * at 60.51 Hz. A mode that claims 60 Hz makes compositors schedule frames
 * against the wrong period (KWin extrapolates the next vblank from it, and
 * after an idle period its target drifts by milliseconds). Measure it: time
 * the raw VSYNC status over a number of frames before the interrupt is hooked.
 */
#define KIRIN_MEASURE_FRAMES	16

static u64 kirin_measure_frame_ns(struct kirin_dss *k)
{
	ktime_t t0 = 0, t = 0;
	unsigned int i;
	u32 v;

	for (i = 0; i <= KIRIN_MEASURE_FRAMES; i++) {
		dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, LDI_INT_VSYNC);
		if (readl_poll_timeout(k->base + DSS_DSI0 + MIPI_LDI_CPU_ITF_INTS, v,
				       v & LDI_INT_VSYNC, 20, 50 * USEC_PER_MSEC))
			return 0;
		t = ktime_get();
		if (!i)
			t0 = t;
	}
	return div_u64(ktime_to_ns(ktime_sub(t, t0)), KIRIN_MEASURE_FRAMES);
}

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
	u64 frame_ns;

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
	frame_ns = kirin_measure_frame_ns(k);
	if (frame_ns > 8 * NSEC_PER_MSEC && frame_ns < 34 * NSEC_PER_MSEC) {
		m->clock = DIV_ROUND_CLOSEST_ULL((u64)htotal * vtotal * USEC_PER_SEC, frame_ns);
		drm_info(&k->drm, "measured frame period %llu ns (%llu.%03llu Hz)\n", frame_ns,
			 div_u64(NSEC_PER_SEC, frame_ns),
			 div_u64(NSEC_PER_SEC * 1000ULL, frame_ns) % 1000);
	} else {
		drm_warn(&k->drm, "can't measure the refresh rate, assuming 60 Hz\n");
		m->clock = DIV_ROUND_CLOSEST(htotal * vtotal * 60, 1000);
	}
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

const struct kirin_format *kirin_find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(kirin_formats); i++)
		if (kirin_formats[i].fourcc == fourcc)
			return &kirin_formats[i];
	return NULL;
}

void kirin_mutex_lock(struct kirin_dss *k)
{
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX, 1);
}

void kirin_mutex_unlock(struct kirin_dss *k)
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
	const struct kirin_format *fmt;
	dma_addr_t addr;

	if (!new->visible || !fb || !k->hw_on)
		return;

	addr = drm_fb_dma_get_gem_addr(fb, new, 0);
	fmt = kirin_find_format(fb->format->format);
	if (k->primary_setup) {
		kirin_rch_update(k, addr, fb->pitches[0], fmt);
	} else {
		/* first frame after a power-up: the whole channel and layer */
		kirin_primary_program(k, addr, fb->pitches[0], fmt);
		k->primary_setup = true;
	}
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

/*
 * Program a whole scaler-less read channel + OV layer (called with the MCTL
 * mutex held). The DMA fetches whole 16-byte units, so the source window is
 * widened to a multiple of 4 pixels and the DFC clips the extra pixels off
 * again. The DMA starts at the first fetched pixel, with zero DMA window
 * offsets. Used for the cursor, and for the primary plane after a power-up.
 *
 * The layer is never switched off again once used: turning an OV layer off
 * and flushing OV0 is what left the LDI in permanent underflow during
 * bring-up. Hiding the cursor shows a transparent buffer instead.
 */
void kirin_layer_program(struct kirin_dss *k, unsigned int ch, unsigned int layer,
			 dma_addr_t addr, u32 pitch, const struct kirin_format *fmt,
			 u32 sx, u32 w, u32 h, u32 dx, u32 dy, u32 alpha)
{
	const struct kirin_rch *r = &kirin_rch[ch];
	u32 ax0, ax1, clip_l, clip_r, sel;
	unsigned int i;

	ax0 = round_down(sx, 4);
	ax1 = round_up(sx + w, 4);
	clip_l = sx - ax0;
	clip_r = ax1 - (sx + w);
	addr += ax0 * 4;

	for (i = 0; i < r->smr_num; i++)
		dss_wr(k, DSS_SMMU + SMMU_SMRX_NS(r->smr_first + i), SMMU_SMR_BYPASS);
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX_RCH(ch), 1);

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
	dss_wr(k, r->dma + DMA_STRIDE0, pitch / 16);
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

	i = DSS_OVL0 + OV_LAYER(layer);
	dss_wr(k, i + OV_LAYER_POS, (dy << 16) | dx);
	dss_wr(k, i + OV_LAYER_SIZE, ((dy + h - 1) << 16) | (dx + w - 1));
	dss_wr(k, i + OV_LAYER_PATTERN_RGB, 0);
	dss_wr(k, i + OV_LAYER_ALPHA_MODE, alpha);
	dss_wr(k, i + OV_LAYER_ALPHA_A, OV_ALPHA_A_OPAQUE);
	dss_wr(k, i + OV_LAYER_PSPOS, (dy << 16) | dx);
	dss_wr(k, i + OV_LAYER_PEPOS, ((dy + h - 1) << 16) | (dx + w - 1));
	dss_wr(k, i + OV_LAYER_CFG, 1);

	sel = dss_rd(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL);
	sel &= ~(0xf << (4 * (layer + 1)));
	sel |= ch << (4 * (layer + 1));
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL, sel);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_STARTY(ch), dy | (8 << 16));
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(ch), MCTL_OV_OEN_OV0);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_FLUSH_EN(ch), 1);
	dss_wr(k, DSS_MCTL_SYS + MCTL_OV0_FLUSH_EN, 0xd);
}

static void kirin_cursor_program(struct kirin_dss *k, dma_addr_t addr, u32 pitch,
				 const struct kirin_format *fmt, u32 sx, u32 w,
				 u32 h, u32 dx, u32 dy, u32 alpha)
{
	kirin_layer_program(k, k->cursor_ch, k->cursor_layer, addr, pitch, fmt,
			    sx, w, h, dx, dy, alpha);
}

#define KIRIN_CURSOR_BLANK	16	/* 16x16 transparent ARGB8888 */

static void kirin_cursor_hide(struct kirin_dss *k)
{
	kirin_cursor_program(k, k->cursor_blank_dma, KIRIN_CURSOR_BLANK * 4,
			     kirin_find_format(DRM_FORMAT_ARGB8888), 0,
			     KIRIN_CURSOR_BLANK, KIRIN_CURSOR_BLANK, 0, 0,
			     OV_ALPHA_PREMULT_OVER);
}

static void kirin_cursor_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(plane->dev);
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new->fb;
	const struct kirin_format *fmt;
	u32 sx, sy, alpha;
	dma_addr_t addr;

	if (!k->hw_on)
		return;
	if (!new->visible || !fb) {
		kirin_cursor_hide(k);
		return;
	}

	fmt = kirin_find_format(fb->format->format);
	sx = new->src.x1 >> 16;
	sy = new->src.y1 >> 16;
	/* new->src is the source rectangle after clipping to the CRTC */
	addr = drm_fb_dma_get_gem_obj(fb, 0)->dma_addr + fb->offsets[0] +
	       sy * fb->pitches[0];
	alpha = fmt->alpha && new->pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE ?
		OV_ALPHA_PREMULT_OVER : OV_ALPHA_OPAQUE;

	kirin_cursor_program(k, addr, fb->pitches[0], fmt, sx,
			     drm_rect_width(&new->dst), drm_rect_height(&new->dst),
			     new->dst.x1, new->dst.y1, alpha);
}

static void kirin_cursor_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(plane->dev);

	if (k->hw_on)
		kirin_cursor_hide(k);
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
	if (k->hw_on) {
		v = dsi_rd(k, MIPI_LDI_CPU_ITF_INT_MSK);
		v = mask ? v | bits : v & ~bits;
		dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK, v);
	}
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
	/* one mode: the panel's */
	if (!drm_mode_equal(&cs->mode, &k->fw_mode))
		return -EINVAL;
	/* the primary layer is never switched off while scanning out */
	if (!(cs->plane_mask & drm_plane_mask(crtc->primary)))
		return -EINVAL;
	return 0;
}

/*
 * Power the display chain up from reset, in the vendor's order: DSS (CRTC
 * enable), bridge and panel supplies (bridge pre-enable), DSI link (encoder
 * enable), eDP link training and stream (bridge enable). The LDI starts in
 * the flush that programs the first frame, the backlight a few frames later.
 */
static void kirin_pipe_on(struct kirin_dss *k, const struct drm_display_mode *m)
{
	ktime_t t0 = ktime_get();
	int ret;

	mutex_lock(&k->hw_lock);
	drm_info(&k->drm, "display on (from off level %d)\n", k->off_level);
	ret = kirin_dss_power_on(k, k->off_level);
	if (ret)
		goto out;
	kirin_dss_hw_init(k, m);
	kirin_edp_power_on(k);
	ret = kirin_dsi_on(k, m);
	if (ret) {
		kirin_edp_power_off(k);
		kirin_dss_power_off(k, k->off_level);
		goto out;
	}
	kirin_edp_enable(k);

	dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, ~0u);
	dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK, ~(u32)LDI_INT_UNFLOW);
	k->hw_on = true;
	k->fw_pipeline = false;
	k->primary_setup = false;
	k->ldi_pending = true;
	k->power_cycles++;
	enable_irq(k->irq);
out:
	mutex_unlock(&k->hw_lock);
	if (ret)
		drm_err(&k->drm, "display power-up failed: %d\n", ret);
	else
		drm_info(&k->drm, "display on after %lld ms\n", ktime_ms_delta(ktime_get(), t0));
}

/*
 * The reverse, as the vendor's bridge disable, encoder disable, bridge
 * post-disable and CRTC disable: backlight, eDP stream, LDI and D-PHY, panel
 * and bridge supplies, DSS clocks and power domains.
 */
static void kirin_pipe_off(struct kirin_dss *k, int level)
{
	kirin_backlight_off(k);
	msleep(200);	/* vendor: backlight off to video off */

	mutex_lock(&k->hw_lock);
	drm_info(&k->drm, "display off (level %d)\n", level);
	disable_irq(k->irq);
	cancel_work_sync(&k->recover_work);
	kirin_edp_disable(k);
	kirin_dsi_off(k);
	kirin_edp_power_off(k);
	spin_lock_irq(&k->irq_lock);
	k->hw_on = false;
	spin_unlock_irq(&k->irq_lock);
	kirin_dss_power_off(k, level);
	k->off_level = level;
	k->ldi_pending = false;
	k->primary_setup = false;
	mutex_unlock(&k->hw_lock);
	drm_info(&k->drm, "display off done\n");
}

static void kirin_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(crtc->dev);
	const struct drm_display_mode *m = &crtc->state->adjusted_mode;
	unsigned long flags;

	if (!k->hw_on)
		kirin_pipe_on(k, m);

	spin_lock_irqsave(&k->irq_lock, flags);
	k->frame_ns = m->clock ? div_u64((u64)m->htotal * m->vtotal * USEC_PER_SEC,
					 m->clock) : 0;
	k->vbl_est = 0;
	spin_unlock_irqrestore(&k->irq_lock, flags);

	drm_crtc_vblank_on(crtc);
}

static void kirin_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(crtc->dev);
	int level = clamp(READ_ONCE(power_off), KIRIN_OFF_NONE, KIRIN_OFF_ALL);

	/* the UEFI framebuffer is only there to go back to until the first power cycle */
	if (level == KIRIN_OFF_NONE && !k->fw_pipeline)
		level = KIRIN_OFF_PANEL;

	if (level != KIRIN_OFF_NONE) {
		drm_crtc_vblank_off(crtc);
		if (k->hw_on)
			kirin_pipe_off(k, level);
	} else if (k->hw_on) {
		/*
		 * Leave the pipeline running, but stop fetching from buffers
		 * that may be freed after this commit. (Hiding the OV layer
		 * instead led to LDI underflows.)
		 */
		if (cursor_plane) {
			kirin_mutex_lock(k);
			kirin_cursor_hide(k);
			kirin_mutex_unlock(k);
		}
		kirin_rch_restore_fw(k);
		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_vblank_off(crtc);
	} else {
		drm_crtc_vblank_off(crtc);
	}

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
	struct kirin_dss *k = to_kirin(crtc->dev);

	if (k->hw_on)
		kirin_mutex_lock(k);
}

/* frame statistics: how many vblanks between consecutive new primary frames */
static void kirin_count_flip(struct kirin_dss *k, struct drm_crtc *crtc,
			     struct drm_atomic_state *state)
{
	struct drm_plane_state *old, *new;
	u32 vstate;
	u64 vbl;

	new = drm_atomic_get_new_plane_state(state, crtc->primary);
	old = drm_atomic_get_old_plane_state(state, crtc->primary);
	if (!new || !new->fb || (old && old->fb == new->fb))
		return;

	vstate = dsi_rd(k, MIPI_LDI_VSTATE) & LDI_VSTATE_MASK;
	if (vstate & (LDI_VSTATE_VFP | LDI_VSTATE_VSW | LDI_VSTATE_VBP))
		k->flush_in_blank++;

	vbl = drm_crtc_vblank_count(crtc);
	if (k->flips && vbl > k->last_flip_vbl)
		k->flip_gap[min_t(u64, vbl - k->last_flip_vbl, 4) - 1]++;
	k->last_flip_vbl = vbl;
	k->flips++;

	l410_perf_frame();
}

static void kirin_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct kirin_dss *k = to_kirin(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;
	bool hw_on = k->hw_on;

	if (hw_on) {
		kirin_mutex_unlock(k);
		if (k->ldi_pending) {
			/* first frame after power-up is programmed: start scanning out */
			k->ldi_pending = false;
			kirin_ldi_enable(k);
			kirin_backlight_on_later(k, 50);
		} else {
			kirin_count_flip(k, crtc, state);
		}
	}

	if (event) {
		crtc->state->event = NULL;
		spin_lock_irq(&crtc->dev->event_lock);
		if (hw_on && crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
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

/*
 * vblank timestamps
 *
 * The DSS has no readable scanline counter, so the timestamps are taken in
 * the vsync interrupt, which runs on a little core that may be in a cluster
 * power-down state. The latency only ever adds to the real time, so track
 * the vblank phase as the earliest arrival: extrapolate by one frame and pull
 * towards later arrivals only slowly (drift); an earlier arrival or a jump
 * resynchronises. Compositors schedule their frames from these timestamps.
 */
static void kirin_vblank_filter(struct kirin_dss *k, ktime_t raw)
{
	s64 err;
	ktime_t pred;

	spin_lock(&k->irq_lock);
	pred = ktime_add_ns(k->vbl_est, k->frame_ns);
	err = ktime_to_ns(ktime_sub(raw, pred));
	if (!k->frame_ns || !vblank_filter || !k->vbl_est ||
	    err < 0 || err > NSEC_PER_MSEC)
		k->vbl_est = raw;
	else
		k->vbl_est = ktime_add_ns(pred, err / 16);
	k->vbl_raw = raw;
	spin_unlock(&k->irq_lock);
}

static bool kirin_crtc_get_vblank_timestamp(struct drm_crtc *crtc, int *max_error,
					    ktime_t *vblank_time, bool in_vblank_irq)
{
	struct kirin_dss *k = to_kirin(crtc->dev);
	unsigned long flags;
	ktime_t est, now;
	u64 frame_ns;

	spin_lock_irqsave(&k->irq_lock, flags);
	est = k->vbl_est;
	frame_ns = k->frame_ns;
	spin_unlock_irqrestore(&k->irq_lock, flags);

	if (!est || !frame_ns)
		return false;

	if (!in_vblank_irq) {
		/* the last vblank before now, extrapolated from the phase */
		now = ktime_get();
		if (ktime_after(now, est))
			est = ktime_add_ns(est, div64_u64(ktime_to_ns(ktime_sub(now, est)),
							  frame_ns) * frame_ns);
	}
	*vblank_time = est;
	return true;
}

static const struct drm_crtc_funcs kirin_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = kirin_crtc_enable_vblank,
	.disable_vblank = kirin_crtc_disable_vblank,
	.get_vblank_timestamp = kirin_crtc_get_vblank_timestamp,
};

static irqreturn_t kirin_dss_irq(int irq, void *data)
{
	struct kirin_dss *k = data;
	ktime_t now = ktime_get();
	u32 ints, msk;

	ints = dsi_rd(k, MIPI_LDI_CPU_ITF_INTS);
	dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, ints);
	msk = dsi_rd(k, MIPI_LDI_CPU_ITF_INT_MSK);
	ints &= ~msk;
	if (!ints)
		return IRQ_NONE;

	if (ints & LDI_INT_VSYNC) {
		kirin_vblank_filter(k, now);
		drm_crtc_handle_vblank(&k->crtc);
	}
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

	mutex_lock(&k->hw_lock);
	seq_printf(m, "power %s%s, power-ups %lu, eDP link failures %lu\n",
		   k->hw_on ? "on" : "off", k->fw_pipeline ? " (UEFI pipeline)" : "",
		   k->power_cycles, k->link_failures);
	seq_printf(m, "rch %u layer %u underflows %u vblanks %llu\n", k->ch,
		   k->layer, k->underflows, drm_crtc_vblank_count(&k->crtc));
	if (!k->hw_on)
		goto out;
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
out:
	mutex_unlock(&k->hw_lock);
	return 0;
}

static void kirin_report_work(struct work_struct *work)
{
	struct kirin_dss *k = container_of(work, struct kirin_dss,
					   report_work.work);
	static int runs;

	mutex_lock(&k->hw_lock);
	if (k->hw_on)
		kirin_crc_snapshot(k, "report");
	mutex_unlock(&k->hw_lock);
	drm_info(&k->drm, "report: vblanks %llu underflows %u\n",
		 drm_crtc_vblank_count(&k->crtc), k->underflows);
	if (++runs < 4)
		schedule_delayed_work(&k->report_work, 3 * HZ);
}

/* debugfs "kirin_dump": the DSS register ranges dumped at probe, now */
static int kirin_dump_show(struct seq_file *m, void *arg)
{
	struct drm_debugfs_entry *entry = m->private;
	struct kirin_dss *k = to_kirin(entry->dev);

	mutex_lock(&k->hw_lock);
	if (k->hw_on)
		kirin_dump_state(k);
	seq_printf(m, "%s (see the kernel log)\n", k->hw_on ? "dumped" : "powered off");
	mutex_unlock(&k->hw_lock);
	return 0;
}

/* debugfs "kirin_edp": the bridge's registers, as far as it is powered */
static int kirin_edp_show(struct seq_file *m, void *arg)
{
	struct drm_debugfs_entry *entry = m->private;
	struct kirin_dss *k = to_kirin(entry->dev);

	mutex_lock(&k->hw_lock);
	if (k->hw_on)
		kirin_edp_dump(k);
	seq_printf(m, "%s (see the kernel log)\n", k->hw_on ? "dumped" : "powered off");
	mutex_unlock(&k->hw_lock);
	return 0;
}

static int kirin_frames_show(struct seq_file *m, void *arg)
{
	struct drm_debugfs_entry *entry = m->private;
	struct kirin_dss *k = to_kirin(entry->dev);
	unsigned long flags;
	ktime_t raw, est;

	spin_lock_irqsave(&k->irq_lock, flags);
	raw = k->vbl_raw;
	est = k->vbl_est;
	spin_unlock_irqrestore(&k->irq_lock, flags);

	seq_printf(m, "vblanks %llu flips %lu\n", drm_crtc_vblank_count(&k->crtc), k->flips);
	seq_printf(m, "flip gap 1:%lu 2:%lu 3:%lu 4+:%lu vblanks\n", k->flip_gap[0],
		   k->flip_gap[1], k->flip_gap[2], k->flip_gap[3]);
	seq_printf(m, "flushes in blanking %lu, foreign imports refused %lu, underflows %u\n",
		   k->flush_in_blank, k->foreign_imports, k->underflows);
	seq_printf(m, "frame %llu ns, last vblank irq latency over estimate %lld ns\n",
		   k->frame_ns, ktime_to_ns(ktime_sub(raw, est)));
	return 0;
}

/* ------------------------------------------------------------------------ */
/* commits */

static struct drm_atomic_state *kirin_atomic_state_alloc(struct drm_device *dev)
{
	struct kirin_atomic_state *ks = kzalloc(sizeof(*ks), GFP_KERNEL);

	if (!ks)
		return NULL;
	if (drm_atomic_state_init(dev, &ks->base) < 0) {
		kfree(ks);
		return NULL;
	}
	return &ks->base;
}

static void kirin_atomic_state_free(struct drm_atomic_state *state)
{
	drm_atomic_state_default_release(state);
	kfree(to_kirin_state(state));
}

/*
 * Up to the hardware latching the new state: the time critical part. As
 * drm_atomic_helper_commit_tail_rpm(): the CRTC is powered up before its
 * planes are programmed, and planes of a switched-off CRTC are left alone.
 */
static void kirin_commit_hw(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_wait_for_fences(dev, state, false);
	drm_atomic_helper_wait_for_dependencies(state);
	drm_atomic_helper_commit_modeset_disables(dev, state);
	drm_atomic_helper_commit_modeset_enables(dev, state);
	drm_atomic_helper_commit_planes(dev, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);
	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
}

static void kirin_commit_cleanup(struct drm_atomic_state *state)
{
	drm_atomic_helper_wait_for_vblanks(state->dev, state);
	drm_atomic_helper_cleanup_planes(state->dev, state);
	drm_atomic_helper_commit_cleanup_done(state);
	drm_atomic_state_put(state);
}

static void kirin_commit_cleanup_work(struct work_struct *work)
{
	kirin_commit_cleanup(container_of(work, struct drm_atomic_state, commit_work));
}

static void kirin_commit_hw_work(struct kthread_work *work)
{
	struct kirin_atomic_state *ks = container_of(work, struct kirin_atomic_state, hw_work);

	kirin_commit_hw(&ks->base);
	/* waiting for the vblank to clean up must not hold up the next commit */
	queue_work(system_unbound_wq, &ks->base.commit_work);
}

/*
 * drm_atomic_helper_commit() with the hardware part of nonblocking commits
 * on a SCHED_FIFO thread instead of an ordinary kworker: a compositor
 * commits about 2 ms before the vblank from a real-time thread and must not
 * wait behind busy CPUs to get the registers written.
 */
static int kirin_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state,
			       bool nonblock)
{
	struct kirin_dss *k = to_kirin(dev);
	struct kirin_atomic_state *ks = to_kirin_state(state);
	int ret;

	if (state->async_update || !k->commit_worker)
		return drm_atomic_helper_commit(dev, state, nonblock);

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		return ret;

	INIT_WORK(&state->commit_work, kirin_commit_cleanup_work);
	kthread_init_work(&ks->hw_work, kirin_commit_hw_work);

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		return ret;

	if (!nonblock) {
		ret = drm_atomic_helper_wait_for_fences(dev, state, true);
		if (ret)
			goto err;
	}

	ret = drm_atomic_helper_swap_state(state, true);
	if (ret)
		goto err;

	drm_atomic_state_get(state);
	if (nonblock) {
		kthread_queue_work(k->commit_worker, &ks->hw_work);
	} else {
		kirin_commit_hw(state);
		kirin_commit_cleanup(state);
	}
	return 0;

err:
	drm_atomic_helper_unprepare_planes(dev, state);
	return ret;
}

/* ------------------------------------------------------------------------ */
/* device */

static struct drm_gem_object *kirin_gem_prime_import(struct drm_device *dev,
						     struct dma_buf *dma_buf)
{
	struct kirin_dss *k = to_kirin(dev);

	if (!drm_gem_is_prime_exported_dma_buf(dev, dma_buf) && !foreign_import) {
		k->foreign_imports++;
		return ERR_PTR(-EINVAL);
	}
	return drm_gem_prime_import(dev, dma_buf);
}

static const struct drm_mode_config_funcs kirin_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = kirin_atomic_commit,
	.atomic_state_alloc = kirin_atomic_state_alloc,
	.atomic_state_clear = drm_atomic_state_default_clear,
	.atomic_state_free = kirin_atomic_state_free,
};

DEFINE_DRM_GEM_DMA_FOPS(kirin_fops);

static const struct drm_driver kirin_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &kirin_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	.gem_prime_import = kirin_gem_prime_import,
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
		k->cursor_blank = dmam_alloc_coherent(drm->dev,
						      KIRIN_CURSOR_BLANK * KIRIN_CURSOR_BLANK * 4,
						      &k->cursor_blank_dma, GFP_KERNEL);
		if (!k->cursor_blank)
			cursor_plane = false;
	}

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

/*
 * Take over the pipeline UEFI lit: mode, scanout channel and layer, and a
 * check that the DSI and bridge set-up this driver would program matches.
 */
static int kirin_takeover(struct kirin_dss *k)
{
	int ret;

	if (dump_state)
		kirin_dump_state(k);
	ret = kirin_read_fw_mode(k);
	if (ret)
		return dev_err_probe(k->drm.dev, ret, "can't read the firmware mode\n");
	ret = kirin_find_fw_layer(k);
	if (ret)
		return ret;
	kirin_dsi_check_fw(k);
	kirin_edp_dump(k);

	k->smmu_scr = dss_rd(k, DSS_SMMU + SMMU_SCR) | SMMU_SCR_GLB_BYPASS;
	k->hw_on = true;
	k->fw_pipeline = true;
	k->primary_setup = true;
	return 0;
}

/* no picture from UEFI: power down again and set up at the first enable */
static void kirin_cold(struct kirin_dss *k)
{
	drm_info(&k->drm, "display not running (LDI off), will power it up from reset\n");
	kirin_dsi_default_mode(k, &k->fw_mode);
	k->ch = KIRIN_PRIMARY_RCH;
	k->layer = 0;
	k->cursor_ch = KIRIN_CURSOR_RCH;
	k->cursor_layer = 1;
	k->rch_ctl = KIRIN_RCH_CTL_DEFAULT;
	k->rch_buf_ctrl = KIRIN_RCH_BUF_CTRL_DEFAULT;
	k->rch_bitext = KIRIN_RCH_BITEXT_DEFAULT;
	k->smmu_scr = KIRIN_SMMU_SCR_DEFAULT;
	selftest = false;

	clk_bulk_disable_unprepare(k->num_dsi_clks, k->dsi_clks);
	kirin_dss_power_off(k, KIRIN_OFF_DSS);
	k->off_level = KIRIN_OFF_DSS;
}

static int kirin_dss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct kirin_dss *k;
	bool running;
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

	/* powers and clocks the DSS as UEFI has it, so its registers can be read */
	ret = kirin_power_init(k);
	if (ret)
		return ret;
	running = dsi_rd(k, MIPI_LDI_CTRL) & LDI_EN;

	ret = kirin_dsi_init(k);
	if (ret)
		return ret;
	ret = kirin_edp_init(k, running);
	if (ret)
		return ret;

	if (running) {
		ret = kirin_takeover(k);
		if (ret)
			return ret;
	} else {
		kirin_cold(k);
	}

	if (selftest)
		kirin_selftest(k);

	ret = kirin_modeset_init(k);
	if (ret)
		return ret;

	INIT_WORK(&k->recover_work, kirin_recover_work);

	/* mask everything but vsync/underflow before hooking the interrupt */
	if (running) {
		dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK, ~(u32)(LDI_INT_UNFLOW | LDI_INT_VSYNC));
		dsi_wr(k, MIPI_LDI_CPU_ITF_INTS, ~0u);
	}
	ret = devm_request_irq(dev, k->irq, kirin_dss_irq, running ? 0 : IRQF_NO_AUTOEN,
			       "kirin-dss", k);
	if (ret)
		return ret;

	ret = aperture_remove_all_conflicting_devices(kirin_drm_driver.name);
	if (ret)
		return ret;

	drm_debugfs_add_file(&k->drm, "kirin_state", kirin_state_show, NULL);
	drm_debugfs_add_file(&k->drm, "kirin_frames", kirin_frames_show, NULL);
	drm_debugfs_add_file(&k->drm, "kirin_edp", kirin_edp_show, NULL);
	drm_debugfs_add_file(&k->drm, "kirin_dump", kirin_dump_show, NULL);

	if (rt_commit) {
		struct kthread_worker *worker = kthread_run_worker(0, "kirin-commit");

		if (IS_ERR(worker)) {
			drm_warn(&k->drm, "no commit thread: %pe\n", worker);
		} else {
			sched_set_fifo(worker->task);
			k->commit_worker = worker;
		}
	}

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
	if (k->commit_worker)
		kthread_destroy_worker(k->commit_worker);
}

static void kirin_dss_shutdown(struct platform_device *pdev)
{
	struct kirin_dss *k = platform_get_drvdata(pdev);

	if (selftest)
		cancel_delayed_work_sync(&k->report_work);
	drm_atomic_helper_shutdown(&k->drm);
}

static int kirin_dss_suspend(struct device *dev)
{
	struct kirin_dss *k = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&k->drm);
}

static int kirin_dss_resume(struct device *dev)
{
	struct kirin_dss *k = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&k->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(kirin_dss_pm_ops, kirin_dss_suspend, kirin_dss_resume);

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
		.pm = pm_sleep_ptr(&kirin_dss_pm_ops),
	},
};
module_platform_driver(kirin_dss_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 display subsystem DRM driver");
MODULE_LICENSE("GPL");
