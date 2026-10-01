/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * HiSilicon Kirin 990 display subsystem (DSS v510) DRM driver: shared state.
 */
#ifndef __KIRIN990_DRV_H__
#define __KIRIN990_DRV_H__

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dp_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane.h>

#include "kirin990_dss_regs.h"

struct backlight_device;
struct gpio_desc;
struct i2c_client;
struct regulator;

#define KIRIN_NUM_RCH		8

/*
 * Read channels, indexed by the MCTL channel number (vendor DSS_RCHN_*:
 * D2, D3, V0, G0, V1, G1, D0, D1).
 */
struct kirin_rch {
	u32 dma;	/* DMA block; DFC, DMA_BUF and REG_DEFAULT hang off it */
	u8 smr_first;	/* first SMMU stream (SMR) index */
	u8 smr_num;
};

extern const struct kirin_rch kirin_rch[KIRIN_NUM_RCH];

/*
 * Channels used after a power-up from reset (scaler-less, so there is no
 * scaler or sharpening stage to set up), and the channel defaults UEFI uses.
 */
#define KIRIN_PRIMARY_RCH		1
#define KIRIN_CURSOR_RCH		6
#define KIRIN_RCH_CTL_DEFAULT		0xf000
#define KIRIN_RCH_BUF_CTRL_DEFAULT	0xc
#define KIRIN_RCH_BITEXT_DEFAULT	0x2
#define KIRIN_SMMU_SCR_DEFAULT		0x000f8001	/* UEFI: global bypass */

/* how far the display is powered down while off (module parameter power_off) */
enum {
	KIRIN_OFF_NONE,		/* scanout back to the UEFI framebuffer only */
	KIRIN_OFF_PANEL,	/* backlight, panel, bridge, DSI */
	KIRIN_OFF_CLOCKS,	/* + DSS clocks */
	KIRIN_OFF_DSS,		/* + DSS power domain */
	KIRIN_OFF_ALL,		/* + vivobus and media1 domains */
};

struct kirin_format {
	u32 fourcc;
	u8 dma_fmt;
	u8 dfc_fmt;
	bool alpha;
};

/*
 * D-PHY and DSI host timing for the panel link, computed the way the vendor
 * driver does (hisi_mipi_dsi.c). Delays are in lane byte clock cycles.
 */
struct kirin_dphy {
	u64 lane_clock;			/* Hz, after PLL rounding */
	u64 lane_byte_clk;		/* Hz */
	u32 pll_posdiv, pll_fbkdiv;
	u32 clk_pre_delay, clk_post_delay, clk_t_lpx, clk_t_hs_prepare;
	u32 clk_t_hs_zero, clk_t_hs_trail;
	u32 data_pre_delay, data_post_delay, data_t_lpx, data_t_hs_prepare;
	u32 data_t_hs_zero, data_t_hs_trail;
	u32 clk_lane_lp2hs, clk_lane_hs2lp, data_lane_lp2hs, data_lane_hs2lp;
	u32 phy_stop_wait;
	u32 clk_division;		/* TX escape clock divider */
};

struct kirin_dss {
	struct drm_device drm;
	void __iomem *base;
	void __iomem *peri_crg;
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
	/* firmware scanout buffer (reserved by UEFI), for the probe self test */
	u32 fw_addr, fw_stride, fw_ctrl, fw_fmt, fw_mif1;
	u32 fw_smr[4];
	bool fw_translate;
	bool primary_setup;		/* primary channel fully programmed since power-up */
	u32 smmu_scr;			/* DSS SMMU global control (UEFI's, in bypass) */

	spinlock_t irq_lock;
	u32 underflows;
	struct work_struct recover_work;
	struct delayed_work report_work;

	/* transparent buffer the cursor layer shows while "hidden" */
	void *cursor_blank;
	dma_addr_t cursor_blank_dma;

	/* nonblocking commits run here: SCHED_FIFO, as msm's commit threads */
	struct kthread_worker *commit_worker;

	/* filtered vblank timestamps (irq_lock) */
	ktime_t vbl_raw, vbl_est;
	u64 frame_ns;

	/* primary plane flips: vblanks between consecutive new frames */
	u64 last_flip_vbl;
	unsigned long flips, flip_gap[4];	/* 1, 2, 3, 4+ vblanks */
	unsigned long flush_in_blank;		/* flushed in VFP/VSW/VBP */
	unsigned long foreign_imports;		/* dma-bufs refused */

	/* ---- power (kirin990_power.c) ---- */
	struct mutex hw_lock;		/* power transitions vs. debugfs readers */
	bool hw_on;			/* DSS registers are clocked and powered */
	bool fw_pipeline;		/* still the pipeline UEFI set up */
	bool ldi_pending;		/* start the LDI after the next plane flush */
	int off_level;			/* KIRIN_OFF_* the display is powered down to */
	struct regulator *media_supply, *vivobus_supply, *dss_supply;
	struct clk_bulk_data *core_clks;
	int num_core_clks;
	struct clk *pri_clk, *mmbuf_clk;
	unsigned long pri_rate, mmbuf_rate;
	unsigned long power_cycles, link_failures;

	/* ---- DSI0 host, D-PHY, LDI (kirin990_dsi.c) ---- */
	struct kirin_dphy dphy;
	u32 dsi_bit_clk_mhz;
	struct clk_bulk_data *dsi_clks;
	int num_dsi_clks;

	/* ---- SN65DSI86 bridge, eDP panel, backlight (kirin990_edp.c) ---- */
	struct i2c_client *bridge;
	struct clk *bridge_refclk;
	struct gpio_desc *gpio_1v2, *gpio_1v8, *gpio_bridge_en, *gpio_panel_vdd;
	struct backlight_device *backlight;
	struct delayed_work backlight_work;
	ktime_t panel_off_time;
	struct drm_dp_aux aux;		/* DDC over the bridge's AUX channel */
	const struct drm_edid *edid;	/* the panel's, applied in get_modes() */
	bool edid_tried;
};

#define to_kirin(x) container_of(x, struct kirin_dss, drm)

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

static inline void dsi_rmw(struct kirin_dss *k, u32 off, u32 mask, u32 val)
{
	dsi_wr(k, off, (dsi_rd(k, off) & ~mask) | (val & mask));
}

/* kirin990_drv.c */
void kirin_mutex_lock(struct kirin_dss *k);
void kirin_mutex_unlock(struct kirin_dss *k);
const struct kirin_format *kirin_find_format(u32 fourcc);
void kirin_layer_program(struct kirin_dss *k, unsigned int ch, unsigned int layer,
			 dma_addr_t addr, u32 pitch, const struct kirin_format *fmt,
			 u32 sx, u32 w, u32 h, u32 dx, u32 dy, u32 alpha);

/* kirin990_dsi.c */
int kirin_dsi_init(struct kirin_dss *k);
void kirin_dsi_check_fw(struct kirin_dss *k);
int kirin_dsi_on(struct kirin_dss *k, const struct drm_display_mode *mode);
void kirin_dsi_off(struct kirin_dss *k);
void kirin_ldi_enable(struct kirin_dss *k);
void kirin_ldi_disable(struct kirin_dss *k);
u64 kirin_dsi_frame_ns(struct kirin_dss *k, const struct drm_display_mode *mode);
void kirin_dsi_default_mode(struct kirin_dss *k, struct drm_display_mode *mode);

/* kirin990_edp.c */
int kirin_edp_init(struct kirin_dss *k, bool running);
void kirin_edp_dump(struct kirin_dss *k);
void kirin_edp_read_edid(struct kirin_dss *k);
void kirin_edp_power_on(struct kirin_dss *k);
int kirin_edp_enable(struct kirin_dss *k);
void kirin_edp_disable(struct kirin_dss *k);
void kirin_edp_power_off(struct kirin_dss *k);
void kirin_backlight_on_later(struct kirin_dss *k, unsigned int ms);
void kirin_backlight_off(struct kirin_dss *k);

/* kirin990_power.c */
int kirin_power_init(struct kirin_dss *k);
int kirin_dss_power_on(struct kirin_dss *k, int level);
void kirin_dss_power_off(struct kirin_dss *k, int level);
void kirin_dss_hw_init(struct kirin_dss *k, const struct drm_display_mode *mode);
void kirin_primary_program(struct kirin_dss *k, dma_addr_t addr, u32 pitch,
			   const struct kirin_format *fmt);

#endif /* __KIRIN990_DRV_H__ */
