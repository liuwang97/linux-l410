// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kirin 990 DSS: MIPI DSI0 host (Synopsys DW MIPI DSI), its D-PHY and the
 * LDI (display timing generator) that feeds it.
 *
 * The panel link is the vendor's for the L410 (laptop_bridge.c "UA"): four
 * lanes at 1248 Mbps (dsi_bit_clk 624 MHz) into the SN65DSI86, video burst
 * mode, host timing given in lane byte clocks (dsi_timing_support). The
 * D-PHY PLL and lane timings are computed with the vendor's formulas
 * (hisi_mipi_dsi.c get_dsi_dphy_ctrl()); on the L410 they come out exactly
 * as UEFI programs them, which kirin_dsi_check_fw() confirms at probe.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/of.h>

#include <drm/drm_print.h>

#include "kirin990_drv.h"

#define DSI_LANES		4
#define DPHY_REF_HZ		19200000ULL	/* clk_txdphy0_ref */
#define DSI_MAX_TX_ESC_HZ	10000000ULL
#define PCTRL_HZ		83000000ULL	/* DEFAULT_PCLK_PCTRL_RATE */
#define DSI_DEFAULT_BIT_CLK	624		/* MHz, SN65DSI86 node "dsi_bit_clk" */

/* PERI_CRG resets of the DSI hosts */
#define PERRSTEN3		0x084
#define PERRSTDIS3		0x088
#define PERRSTSTAT3		0x08c
#define   RST_DSI0		BIT(28)
#define   RST_DSI1		BIT(29)

/* D-PHY test codes (written through PHY_TST_CTRL0/1) */
#define TST_PLL_CP		0x10042
#define TST_PLL_DIV		0x10049
#define TST_PLL_FBK		0x1004a
#define TST_PLL_UPDATE		0x1004b
#define TST_LANE(n)		((n) << 5)	/* lanes 0,1 data, 2 clock, 3,4 data */
#define TST_DATA_PRE_DELAY	0x10070
#define TST_DATA_POST_DELAY	0x10071
#define TST_DATA_TLPX		0x10072
#define TST_DATA_PREPARE	0x10073
#define TST_DATA_ZERO		0x10074
#define TST_DATA_TRAIL		0x10075
#define TST_LANE_PROPERTY	0x10077
#define TST_CLK_PRE_DELAY	0x100b0
#define TST_CLK_POST_DELAY	0x100b1
#define TST_CLK_TLPX		0x100b2
#define TST_CLK_PREPARE		0x100b3
#define TST_CLK_ZERO		0x100b4
#define TST_CLK_TRAIL		0x100b5

/* PHY_STATUS: PLL lock, stop state of the clock lane and data lanes 0-3 */
#define PHY_STATUS_LOCK		BIT(0)
#define PHY_STATUS_STOP		(BIT(4) | BIT(7) | BIT(9) | BIT(11))

/*
 * Host timing in lane byte clocks, from the vendor panel table: HSA, HBP, line
 * time and active width (hsw 32, hbp 80, htotal 2320, hact 2160 pixels at the
 * panel's 207.76 MHz). Replaced by what UEFI programmed when it lit the panel.
 */
static u32 dsi_hsa = 24, dsi_hbp = 60, dsi_hline = 1742, dsi_hact = 1622;
static u32 dsi_vsa = 10, dsi_vbp = 27, dsi_vfp = 3, dsi_pol;

static u32 ceil_div(u64 x, u32 y)
{
	return div_u64(x + y - 1, y);
}

static u32 dss_reduce(u32 x)
{
	return x ? x - 1 : 0;
}

/* vendor get_dsi_dphy_ctrl() for a D-PHY, normal LP-11, no timing adjusts */
static void kirin_dphy_compute(struct kirin_dphy *p, u32 bit_clk_mhz)
{
	static const u32 post_div[] = { 1, 2, 4, 8, 16, 32 };
	const u32 acc = 10;	/* timings below are in 1/100 ns */
	u32 clk_post, clk_pre, clk_t_hs_exit, clk_t_hs_trail, clk_t_hs_prepare;
	u32 data_t_hs_trail, data_t_hs_prepare, clk_t_lpx, clk_t_hs_zero;
	u32 data_t_lpx, data_t_hs_zero, ui, byte, idx = 0;
	u64 lane_mhz = 2ULL * bit_clk_mhz, vco;

	/* PLL: VCO above 2 GHz, then divided down to the lane rate */
	vco = lane_mhz;
	while (vco <= 2000 && idx < ARRAY_SIZE(post_div) - 1)
		vco = lane_mhz * post_div[++idx];
	p->pll_posdiv = idx;
	p->pll_fbkdiv = div64_u64(vco * 1000000, DPHY_REF_HZ);
	p->lane_clock = div_u64(p->pll_fbkdiv * DPHY_REF_HZ, post_div[idx]);

	ui = div64_u64(10ULL * NSEC_PER_SEC * acc, p->lane_clock);
	byte = 8 * ui;

	clk_post = 600 * acc + 52 * ui + byte;
	clk_pre = 8 * ui + byte;
	clk_t_hs_exit = 1000 * acc + 100 * acc;
	clk_t_hs_trail = 600 * acc + 3 * byte;
	clk_t_hs_prepare = 660 * acc;
	data_t_hs_trail = max(600 * acc + 4 * ui, 8 * ui) + 8 * ui + 3 * byte;
	data_t_hs_prepare = min(400 * acc + 4 * ui + 35 * ui, 850 * acc + 6 * ui - 8 * ui);
	clk_t_lpx = 2000 * acc + 10 * acc - clk_t_hs_prepare;
	clk_t_hs_zero = 3000 * acc + 3 * byte - clk_t_hs_prepare;
	data_t_lpx = 2000 * acc + 10 * acc - data_t_hs_prepare;
	data_t_hs_zero = 1450 * acc + 10 * ui + 3 * byte - data_t_hs_prepare;

	p->clk_pre_delay = 0;
	p->clk_t_hs_prepare = ceil_div(clk_t_hs_prepare, byte);
	p->clk_t_lpx = ceil_div(clk_t_lpx, byte);
	p->clk_t_hs_zero = ceil_div(clk_t_hs_zero, byte);
	p->clk_t_hs_trail = ceil_div(clk_t_hs_trail, byte);
	p->data_post_delay = 0;
	p->data_t_hs_prepare = ceil_div(data_t_hs_prepare, byte);
	p->data_t_lpx = ceil_div(data_t_lpx, byte);
	p->data_t_hs_zero = ceil_div(data_t_hs_zero, byte);
	p->data_t_hs_trail = ceil_div(data_t_hs_trail, byte);

	p->clk_post_delay = p->data_t_hs_trail + ceil_div(clk_post, byte);
	p->data_pre_delay = p->clk_pre_delay + 2 + p->clk_t_lpx + p->clk_t_hs_prepare +
			    p->clk_t_hs_zero + 8 + ceil_div(clk_pre, byte);

	p->clk_lane_lp2hs = p->clk_pre_delay + p->clk_t_lpx + p->clk_t_hs_prepare +
			    p->clk_t_hs_zero + 5 + 7;
	p->clk_lane_hs2lp = p->clk_t_hs_trail + p->clk_post_delay + 8 + 4;
	p->data_lane_lp2hs = p->data_pre_delay + 5 + p->data_t_lpx +
			     p->data_t_hs_prepare + p->data_t_hs_zero + 7;
	p->data_lane_hs2lp = p->data_t_hs_trail + 8 + 5;
	p->phy_stop_wait = p->clk_post_delay + 4 + p->clk_t_hs_trail +
			   ceil_div(clk_t_hs_exit, byte) -
			   (p->data_post_delay + 4 + p->data_t_hs_trail) + 3;

	p->lane_byte_clk = div_u64(p->lane_clock, 8);
	p->clk_division = ceil_div(p->lane_byte_clk / 2, DSI_MAX_TX_ESC_HZ);
}

static void dphy_write(struct kirin_dss *k, u32 code, u32 val)
{
	dsi_wr(k, DSI_PHY_TST_CTRL1, code);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0x2);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0x0);
	dsi_wr(k, DSI_PHY_TST_CTRL1, val);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0x2);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0x0);
}

/* latch the test code address only; TESTDOUT shows its current value */
static u32 dphy_read(struct kirin_dss *k, u32 code)
{
	u32 v;

	dsi_wr(k, DSI_PHY_TST_CTRL1, code);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0x2);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0x0);
	v = (dsi_rd(k, DSI_PHY_TST_CTRL1) >> 8) & 0xff;
	dsi_wr(k, DSI_PHY_TST_CTRL1, 0);
	return v;
}

struct dphy_code {
	u32 code;
	u32 val;
};

/* everything mipi_cdphy_init_config() writes, in its order */
static unsigned int kirin_dphy_codes(const struct kirin_dphy *p, struct dphy_code *c)
{
	static const u8 data_lanes[] = { 0, 1, 3, 4 };
	unsigned int n = 0, i, l;

	c[n++] = (struct dphy_code){ TST_PLL_CP, 0x21 };
	c[n++] = (struct dphy_code){ TST_PLL_DIV, p->pll_posdiv << 4 };
	c[n++] = (struct dphy_code){ TST_PLL_FBK, p->pll_fbkdiv };
	c[n++] = (struct dphy_code){ TST_PLL_UPDATE, 1 };
	for (i = 0; i <= DSI_LANES; i++)
		c[n++] = (struct dphy_code){ TST_LANE_PROPERTY + TST_LANE(i), 0x43 };
	c[n++] = (struct dphy_code){ TST_CLK_PRE_DELAY, dss_reduce(p->clk_pre_delay) };
	c[n++] = (struct dphy_code){ TST_CLK_POST_DELAY, dss_reduce(p->clk_post_delay) };
	c[n++] = (struct dphy_code){ TST_CLK_TLPX, dss_reduce(p->clk_t_lpx) };
	c[n++] = (struct dphy_code){ TST_CLK_PREPARE, dss_reduce(p->clk_t_hs_prepare) };
	c[n++] = (struct dphy_code){ TST_CLK_ZERO, dss_reduce(p->clk_t_hs_zero) };
	c[n++] = (struct dphy_code){ TST_CLK_TRAIL, dss_reduce(p->clk_t_hs_trail) };
	for (i = 0; i < ARRAY_SIZE(data_lanes); i++) {
		l = TST_LANE(data_lanes[i]);
		c[n++] = (struct dphy_code){ TST_DATA_PRE_DELAY + l, dss_reduce(p->data_pre_delay) };
		c[n++] = (struct dphy_code){ TST_DATA_POST_DELAY + l, dss_reduce(p->data_post_delay) };
		c[n++] = (struct dphy_code){ TST_DATA_TLPX + l, dss_reduce(p->data_t_lpx) };
		c[n++] = (struct dphy_code){ TST_DATA_PREPARE + l, dss_reduce(p->data_t_hs_prepare) };
		c[n++] = (struct dphy_code){ TST_DATA_ZERO + l, dss_reduce(p->data_t_hs_zero) };
		c[n++] = (struct dphy_code){ TST_DATA_TRAIL + l, dss_reduce(p->data_t_hs_trail) };
	}
	return n;
}

#define DPHY_MAX_CODES	48

int kirin_dsi_init(struct kirin_dss *k)
{
	struct device_node *np;
	u32 mhz = DSI_DEFAULT_BIT_CLK;

	np = of_parse_phandle(k->drm.dev->of_node, "hisilicon,edp-bridge", 0);
	if (np) {
		of_property_read_u32(np, "dsi_bit_clk", &mhz);
		of_node_put(np);
	}
	k->dsi_bit_clk_mhz = mhz;
	kirin_dphy_compute(&k->dphy, mhz);
	drm_dbg_driver(&k->drm, "D-PHY %u MHz: lane %llu Hz posdiv %u fbkdiv %u\n",
		       mhz, k->dphy.lane_clock, k->dphy.pll_posdiv, k->dphy.pll_fbkdiv);
	return 0;
}

/* the frame period follows from the host timing: vtotal lines of HLINE byte clocks */
u64 kirin_dsi_frame_ns(struct kirin_dss *k, const struct drm_display_mode *mode)
{
	return div64_u64((u64)mode->vtotal * dsi_hline * NSEC_PER_SEC, k->dphy.lane_byte_clk);
}

/* the panel mode for a cold start (UEFI did not light the panel) */
void kirin_dsi_default_mode(struct kirin_dss *k, struct drm_display_mode *m)
{
	u32 hsw = DIV_ROUND_CLOSEST(dsi_hsa * 2160, dsi_hact);
	u32 hbp = DIV_ROUND_CLOSEST(dsi_hbp * 2160, dsi_hact);
	u32 htotal = DIV_ROUND_CLOSEST(dsi_hline * 2160, dsi_hact);

	memset(m, 0, sizeof(*m));
	m->hdisplay = 2160;
	m->hsync_start = htotal - hsw - hbp;
	m->hsync_end = htotal - hbp;
	m->htotal = htotal;
	m->vdisplay = 1440;
	m->vsync_start = 1440 + dsi_vfp;
	m->vsync_end = 1440 + dsi_vfp + dsi_vsa;
	m->vtotal = 1440 + dsi_vfp + dsi_vsa + dsi_vbp;
	m->clock = DIV_ROUND_CLOSEST_ULL((u64)htotal * m->vtotal * USEC_PER_SEC,
					 kirin_dsi_frame_ns(k, m));
	m->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;
	m->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(m);
}

struct dsi_expect {
	const char *name;
	u32 off, mask, val;
};

/*
 * Compare what kirin_dsi_on() would program with what UEFI left running, so a
 * mismatch shows up in the log before the first power cycle depends on it.
 * Adopts UEFI's host timing (it is what the panel was lit with).
 */
void kirin_dsi_check_fw(struct kirin_dss *k)
{
	const struct kirin_dphy *p = &k->dphy;
	const struct dsi_expect exp[] = {
		{ "CLKMGR_CFG", DSI_CLKMGR_CFG, 0xfffff,
		  (5 << 16) | (p->clk_division << 8) | p->clk_division },
		{ "PHY_IF_CFG", DSI_PHY_IF_CFG, 0xff03, (p->phy_stop_wait << 8) | (DSI_LANES - 1) },
		{ "PHY_TMR_LPCLK_CFG", DSI_PHY_TMR_LPCLK_CFG, 0x3ff03ff,
		  (p->clk_lane_hs2lp << 16) | p->clk_lane_lp2hs },
		{ "PHY_TMR_CFG", DSI_PHY_TMR_CFG, 0x3ff03ff,
		  (p->data_lane_hs2lp << 16) | p->data_lane_lp2hs },
		{ "DPI_COLOR_CODING", DSI_DPI_COLOR_CODING, 0xf, 5 },
		{ "VID_MODE_CFG", DSI_VID_MODE_CFG, 0xbf03, 0xbf02 },
		{ "PCKHDL_CFG", DSI_PCKHDL_CFG, 0x5, 0x5 },
		{ "LPCLK_CTRL", DSI_LPCLK_CTRL, 0x3, 0x1 },
		{ "PHY_RSTZ", DSI_PHY_RSTZ, 0xf, 0xf },
		{ "DSI_MEM_CTRL", DSI_MEM_CTRL, ~0u, 0x02600008 },
	};
	struct dphy_code codes[DPHY_MAX_CODES];
	unsigned int i, n, bad = 0;
	u32 v;

	dsi_hsa = dsi_rd(k, DSI_VID_HSA_TIME) & 0xfff;
	dsi_hbp = dsi_rd(k, DSI_VID_HBP_TIME) & 0xfff;
	dsi_hline = dsi_rd(k, DSI_VID_HLINE_TIME) & 0x7fff;
	dsi_hact = (dsi_rd(k, MIPI_LDI_DPI0_HRZ_CTRL3) & 0xfff) + 1;
	dsi_vsa = dsi_rd(k, DSI_VID_VSA_LINES) & 0x3ff;
	dsi_vbp = dsi_rd(k, DSI_VID_VBP_LINES) & 0x3ff;
	dsi_vfp = dsi_rd(k, DSI_VID_VFP_LINES) & 0x3ff;
	dsi_pol = dsi_rd(k, DSI_DPI_CFG_POL) & 0x1f;

	for (i = 0; i < ARRAY_SIZE(exp); i++) {
		v = dsi_rd(k, exp[i].off);
		if ((v & exp[i].mask) != exp[i].val) {
			drm_warn(&k->drm, "DSI %s: firmware %#x, computed %#x\n",
				 exp[i].name, v & exp[i].mask, exp[i].val);
			bad++;
		}
	}

	n = kirin_dphy_codes(p, codes);
	for (i = 0; i < n; i++) {
		if (codes[i].code == TST_PLL_UPDATE)
			continue;	/* self-clearing strobe */
		v = dphy_read(k, codes[i].code);
		if (v != codes[i].val) {
			drm_warn(&k->drm, "D-PHY code %#x: firmware %#x, computed %#x\n",
				 codes[i].code, v, codes[i].val);
			bad++;
		}
	}

	drm_info(&k->drm, "DSI %u Mbps x%u: %s firmware (reset stat %#x)\n",
		 (u32)div_u64(p->lane_clock, 1000000), DSI_LANES,
		 bad ? "computed setup DIFFERS from" : "computed setup matches",
		 k->peri_crg ? readl(k->peri_crg + PERRSTSTAT3) : 0);
}

void kirin_ldi_enable(struct kirin_dss *k)
{
	dsi_rmw(k, MIPI_LDI_CTRL, LDI_EN, LDI_EN);
}

void kirin_ldi_disable(struct kirin_dss *k)
{
	dsi_rmw(k, MIPI_LDI_CTRL, LDI_EN, 0);
}

static void kirin_dsi_cmd_mode_lp(struct kirin_dss *k)
{
	dsi_rmw(k, DSI_MODE_CFG, BIT(0), BIT(0));
	dsi_rmw(k, DSI_CMD_MODE_CFG, GENMASK(14, 8), 0x7f << 8);
	dsi_rmw(k, DSI_CMD_MODE_CFG, GENMASK(19, 16), 0xf << 16);
	dsi_rmw(k, DSI_CMD_MODE_CFG, BIT(24), BIT(24));
	dsi_rmw(k, DSI_LPCLK_CTRL, BIT(0), 0);
}

/*
 * vendor mipi_dsi_on(): out of reset, clocks, then mipi_init() (D-PHY, DPI,
 * video mode, timing, LDI geometry) in command mode, then video mode with the
 * HS clock running. The LDI stays off until the first frame is programmed.
 */
int kirin_dsi_on(struct kirin_dss *k, const struct drm_display_mode *mode)
{
	const struct kirin_dphy *p = &k->dphy;
	struct dphy_code codes[DPHY_MAX_CODES];
	unsigned int i, n;
	u32 v;
	int ret;

	if (k->peri_crg) {
		writel(RST_DSI0, k->peri_crg + PERRSTDIS3);
		writel(RST_DSI1, k->peri_crg + PERRSTDIS3);
	}

	ret = clk_bulk_prepare_enable(k->num_dsi_clks, k->dsi_clks);
	if (ret) {
		drm_err(&k->drm, "DSI clocks: %d\n", ret);
		return ret;
	}

	/* PHY start */
	dsi_rmw(k, DSI_PHY_IF_CFG, 0x3, DSI_LANES - 1);
	dsi_rmw(k, DSI_CLKMGR_CFG, 0xffff, (p->clk_division << 8) | p->clk_division);
	dsi_wr(k, DSI_PHY_RSTZ, 0);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0);
	dsi_wr(k, DSI_PHY_TST_CTRL0, 1);	/* testclr */
	dsi_wr(k, DSI_PHY_TST_CTRL0, 0);
	n = kirin_dphy_codes(p, codes);
	for (i = 0; i < n; i++)
		dphy_write(k, codes[i].code, codes[i].val);
	dsi_wr(k, DSI_PHY_RSTZ, 0xf);
	if (readl_poll_timeout(k->base + DSS_DSI0 + DSI_PHY_STATUS, v,
			       v & PHY_STATUS_LOCK, 10, 500 * USEC_PER_MSEC))
		drm_err(&k->drm, "D-PHY PLL not locked, status %#x\n", v);
	if (readl_poll_timeout(k->base + DSS_DSI0 + DSI_PHY_STATUS, v,
			       (v & PHY_STATUS_STOP) == PHY_STATUS_STOP, 10,
			       500 * USEC_PER_MSEC))
		drm_err(&k->drm, "D-PHY lanes not in stop state, status %#x\n", v);

	/* PHY end */
	dsi_rmw(k, DSI_MODE_CFG, BIT(1), BIT(1));
	dsi_rmw(k, DSI_PHY_IF_CFG, 0xff00, p->phy_stop_wait << 8);

	/* DPI interface: virtual channel 0, 24 bpp, firmware polarities */
	dsi_rmw(k, DSI_DPI_VCID, 0x3, 0);
	dsi_rmw(k, DSI_DPI_COLOR_CODING, 0xf, 5);
	dsi_rmw(k, DSI_DPI_CFG_POL, 0x1f, dsi_pol);

	/* video transmission: LP in all blanking periods, burst mode */
	dsi_rmw(k, DSI_VID_MODE_CFG, GENMASK(13, 8), 0x3f << 8);
	dsi_rmw(k, DSI_DPI_LP_CMD_TIM, GENMASK(23, 16), 4 << 16);
	dsi_rmw(k, DSI_VID_MODE_CFG, BIT(15), BIT(15));
	dsi_rmw(k, DSI_VID_PKT_SIZE, GENMASK(13, 0), mode->hdisplay);
	dsi_rmw(k, DSI_VID_MODE_CFG, 0x3, 2);
	dsi_rmw(k, DSI_PCKHDL_CFG, BIT(2), BIT(2));

	/* timing */
	dsi_rmw(k, DSI_VID_HSA_TIME, 0xfff, dsi_hsa);
	dsi_rmw(k, DSI_VID_HBP_TIME, 0xfff, dsi_hbp);
	dsi_rmw(k, DSI_VID_HLINE_TIME, 0x7fff, dsi_hline);
	dsi_rmw(k, DSI_VID_VSA_LINES, 0x3ff, mode->vsync_end - mode->vsync_start);
	dsi_rmw(k, DSI_VID_VBP_LINES, 0x3ff, mode->vtotal - mode->vsync_end);
	dsi_rmw(k, DSI_VID_VFP_LINES, 0x3ff, mode->vsync_start - mode->vdisplay);
	dsi_rmw(k, DSI_VID_VACTIVE_LINES, 0x3fff, mode->vdisplay);
	dsi_rmw(k, DSI_TO_CNT_CFG, 0xffff, 0x7ff);

	/* core PHY parameters */
	dsi_rmw(k, DSI_PHY_TMR_LPCLK_CFG, 0x3ff03ff,
		(p->clk_lane_hs2lp << 16) | p->clk_lane_lp2hs);
	dsi_rmw(k, DSI_PHY_TMR_RD_CFG, 0x7fff, 0x7fff);
	dsi_rmw(k, DSI_PHY_TMR_CFG, 0x3ff03ff,
		(p->data_lane_hs2lp << 16) | p->data_lane_lp2hs);
	dsi_rmw(k, DSI_CLKMGR_CFG, GENMASK(19, 16), 5 << 16);
	dsi_rmw(k, DSI_PHY_MODE, BIT(0), 0);

	/* LDI geometry (the LDI itself stays off) */
	dsi_rmw(k, MIPI_LDI_DPI0_HRZ_CTRL3, 0xfff, dsi_hact - 1);
	dsi_rmw(k, MIPI_LDI_DPI0_HRZ_CTRL2, 0xfff, mode->hdisplay - 1);
	dsi_rmw(k, MIPI_LDI_VRT_CTRL2, 0xfff, mode->vdisplay - 1);
	kirin_ldi_disable(k);
	dsi_rmw(k, MIPI_LDI_FRM_MSK, BIT(0), 0);
	dsi_rmw(k, MIPI_DSI_CMD_MOD_CTRL, BIT(1), BIT(1));

	dsi_rmw(k, DSI_PWR_UP, BIT(0), BIT(0));

	/* mipi_dsi_on_sub1(): command mode, LP, no HS clock */
	dsi_wr(k, DSI_MEM_CTRL, 0x02600008);
	kirin_dsi_cmd_mode_lp(k);

	/* mipi_dsi_on_sub2(): video mode, EoTp, continuous HS clock */
	dsi_rmw(k, DSI_MODE_CFG, BIT(0), 0);
	dsi_rmw(k, DSI_PCKHDL_CFG, BIT(0), BIT(0));
	dsi_rmw(k, DSI_LPCLK_CTRL, 0x3, 0x1);
	dsi_wr(k, DSI_DPHYTX_STOPSNT, ceil_div((u64)dsi_hline * PCTRL_HZ, p->lane_byte_clk));
	return 0;
}

/* vendor dsi_encoder_disable(): LDI off, idle, D-PHY shut down, reset */
void kirin_dsi_off(struct kirin_dss *k)
{
	u32 v;

	dsi_wr(k, MIPI_LDI_CPU_ITF_INT_MSK, ~0u);
	kirin_ldi_disable(k);
	if (readl_poll_timeout(k->base + DSS_MCTL_SYS + MCTL_MOD17_STATUS, v,
			       v & BIT(4), 1000, 100 * USEC_PER_MSEC))
		drm_warn(&k->drm, "MCTL interface not idle, status %#x\n", v);

	kirin_dsi_cmd_mode_lp(k);
	udelay(10);
	dsi_rmw(k, DSI_PHY_RSTZ, 0x7, 0);

	clk_bulk_disable_unprepare(k->num_dsi_clks, k->dsi_clks);
	if (k->peri_crg)
		writel(RST_DSI0, k->peri_crg + PERRSTEN3);
}
