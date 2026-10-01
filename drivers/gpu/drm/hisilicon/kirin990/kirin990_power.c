// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kirin 990 DSS: power domains, clocks and the display pipeline set-up from
 * reset, as the vendor kernel does on every CRTC enable (dpe_on(),
 * dpe_init(), hisi_overlay_on()).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "kirin990_drv.h"

/* vendor DSS_CORE_CLK_RATE_L1 / DSS_MMBUF_CLK_RATE_L1, and the *_OFF rates */
#define DSS_CORE_RATE		208000000UL
#define DSS_MMBUF_RATE		208000000UL
#define DSS_OFF_RATE		104000000UL
#define DPHY_REF_RATE		19200000UL

/* enabled in this order (vendor dpe_common_clk_enable, dpe_inner_clk_enable) */
static const char * const kirin_core_clk_names[] = {
	"clk_dss_axi_mm", "aclk_dss", "pclk_dss", "clk_edc0",
};

static const char * const kirin_dsi_clk_names[] = {
	"clk_txdphy0_ref", "clk_txdphy0_cfg", "pclk_dsi0",
};

static struct clk_bulk_data *kirin_clk_bulk(struct device *dev,
					    const char * const *names, int n)
{
	struct clk_bulk_data *c;
	int i, ret;

	c = devm_kcalloc(dev, n, sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);
	for (i = 0; i < n; i++)
		c[i].id = names[i];
	ret = devm_clk_bulk_get_optional(dev, n, c);
	return ret ? ERR_PTR(ret) : c;
}

static struct regulator *kirin_supply(struct device *dev, const char *name)
{
	struct regulator *reg = devm_regulator_get_optional(dev, name);

	if (IS_ERR(reg) && PTR_ERR(reg) == -ENODEV)
		return NULL;
	return reg;
}

static void kirin_supply_disable(struct kirin_dss *k, struct regulator *reg)
{
	if (reg && regulator_disable(reg))
		drm_warn(&k->drm, "regulator won't switch off\n");
}

static int kirin_supply_enable(struct kirin_dss *k, struct regulator *reg, const char *name)
{
	int ret = reg ? regulator_enable(reg) : 0;

	if (ret)
		drm_err(&k->drm, "%s supply: %d\n", name, ret);
	return ret;
}

/*
 * Claim the power domains and clocks and switch them on, at the rates UEFI
 * left them: if it lit the display, this takes a reference on everything it
 * had on (so nothing is switched off as unused and the first CRTC disable
 * balances it); if not, the caller powers down again after a look at the LDI.
 */
int kirin_power_init(struct kirin_dss *k)
{
	struct device *dev = k->drm.dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	int ret;

	ret = drmm_mutex_init(&k->drm, &k->hw_lock);
	if (ret)
		return ret;

	/* PERI_CRG (the DSI resets); the IP power domain driver owns the region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res)
		k->peri_crg = devm_ioremap(dev, res->start, resource_size(res));

	k->media_supply = kirin_supply(dev, "regulator_media_subsys");
	if (IS_ERR(k->media_supply))
		return dev_err_probe(dev, PTR_ERR(k->media_supply), "media supply\n");
	/* the DSS domain's own supply; held so that power_off=3 leaves it on */
	k->vivobus_supply = kirin_supply(dev, "regulator_vivobus");
	if (IS_ERR(k->vivobus_supply))
		return dev_err_probe(dev, PTR_ERR(k->vivobus_supply), "vivobus supply\n");
	k->dss_supply = kirin_supply(dev, "regulator_dsssubsys");
	if (IS_ERR(k->dss_supply))
		return dev_err_probe(dev, PTR_ERR(k->dss_supply), "DSS supply\n");

	k->num_core_clks = ARRAY_SIZE(kirin_core_clk_names);
	k->core_clks = kirin_clk_bulk(dev, kirin_core_clk_names, k->num_core_clks);
	if (IS_ERR(k->core_clks))
		return dev_err_probe(dev, PTR_ERR(k->core_clks), "DSS clocks\n");
	k->mmbuf_clk = k->core_clks[0].clk;
	k->pri_clk = k->core_clks[3].clk;

	k->num_dsi_clks = ARRAY_SIZE(kirin_dsi_clk_names);
	k->dsi_clks = kirin_clk_bulk(dev, kirin_dsi_clk_names, k->num_dsi_clks);
	if (IS_ERR(k->dsi_clks))
		return dev_err_probe(dev, PTR_ERR(k->dsi_clks), "DSI clocks\n");

	/* rates UEFI runs the pipeline at (at least the vendor's), for every power-up */
	k->pri_rate = k->pri_clk ? clk_get_rate(k->pri_clk) : 0;
	k->mmbuf_rate = k->mmbuf_clk ? clk_get_rate(k->mmbuf_clk) : 0;
	k->pri_rate = max(k->pri_rate, DSS_CORE_RATE);
	k->mmbuf_rate = max(k->mmbuf_rate, DSS_MMBUF_RATE);
	drm_info(&k->drm, "DSS core clock %lu Hz, mmbuf %lu Hz\n", k->pri_rate, k->mmbuf_rate);

	/* in dpe_on() order: media domain, clocks, DSS domain */
	ret = kirin_supply_enable(k, k->media_supply, "media");
	if (ret)
		return ret;
	ret = kirin_supply_enable(k, k->vivobus_supply, "vivobus");
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(k->num_core_clks, k->core_clks);
	if (ret)
		return dev_err_probe(dev, ret, "DSS clocks\n");
	if (k->dss_supply) {
		ret = regulator_enable(k->dss_supply);
		if (ret)
			return dev_err_probe(dev, ret, "DSS supply\n");
	}
	ret = clk_bulk_prepare_enable(k->num_dsi_clks, k->dsi_clks);
	if (ret)
		return dev_err_probe(dev, ret, "DSI clocks\n");
	return 0;
}

/*
 * vendor dpe_on(): media domain, clocks, DSS domain -- undoing what
 * kirin_dss_power_off() did at the given level (KIRIN_OFF_*).
 */
int kirin_dss_power_on(struct kirin_dss *k, int level)
{
	int ret;

	if (level >= KIRIN_OFF_ALL) {
		ret = kirin_supply_enable(k, k->media_supply, "media");
		if (ret)
			return ret;
		ret = kirin_supply_enable(k, k->vivobus_supply, "vivobus");
		if (ret)
			goto err_media;
	}
	if (level >= KIRIN_OFF_CLOCKS) {
		if (level >= KIRIN_OFF_DSS && k->pri_clk)
			clk_set_rate(k->pri_clk, k->pri_rate);
		ret = clk_bulk_prepare_enable(k->num_core_clks, k->core_clks);
		if (ret) {
			drm_err(&k->drm, "DSS clocks: %d\n", ret);
			goto err_vivobus;
		}
	}
	if (level >= KIRIN_OFF_DSS) {
		ret = kirin_supply_enable(k, k->dss_supply, "DSS");
		if (ret)
			goto err_clk;
		if (k->mmbuf_clk)
			clk_set_rate(k->mmbuf_clk, k->mmbuf_rate);
	}
	if (k->dsi_clks[0].clk)
		clk_set_rate(k->dsi_clks[0].clk, DPHY_REF_RATE);
	if (k->dsi_clks[1].clk)
		clk_set_rate(k->dsi_clks[1].clk, DPHY_REF_RATE);
	return 0;

err_clk:
	clk_bulk_disable_unprepare(k->num_core_clks, k->core_clks);
err_vivobus:
	if (level >= KIRIN_OFF_ALL)
		kirin_supply_disable(k, k->vivobus_supply);
err_media:
	if (level >= KIRIN_OFF_ALL)
		kirin_supply_disable(k, k->media_supply);
	return ret;
}

/*
 * vendor dpe_off(): DBUF out of the DDR frequency handshake (deinit_dbuf),
 * then the clocks parked at the low rate, DSS domain, clocks, and the
 * domains behind it -- as far as the level (KIRIN_OFF_*) says. The LDI is
 * already off (kirin_dsi_off()).
 */
void kirin_dss_power_off(struct kirin_dss *k, int level)
{
	dss_rmw(k, DSS_DISP_GLB + DSS_DFS_OK_MASK, BIT(0), BIT(0));

	if (level >= KIRIN_OFF_DSS) {
		if (k->pri_clk)
			clk_set_rate(k->pri_clk, DSS_OFF_RATE);
		if (k->mmbuf_clk)
			clk_set_rate(k->mmbuf_clk, DSS_OFF_RATE);
		kirin_supply_disable(k, k->dss_supply);
	}
	if (level >= KIRIN_OFF_CLOCKS)
		clk_bulk_disable_unprepare(k->num_core_clks, k->core_clks);
	if (level >= KIRIN_OFF_ALL) {
		kirin_supply_disable(k, k->vivobus_supply);
		kirin_supply_disable(k, k->media_supply);
	}
}

struct dss_reg_val {
	u32 off;
	u32 val;
};

/*
 * vendor dss_inner_clk_common_enable(), low-power variant (CONFIG_DSS_LP_USED
 * is always defined): automatic clock gating in every module.
 */
static const struct dss_reg_val kirin_clk_gating[] = {
	{ DSS_GLB + GLB_MODULE_CLK_SEL, 0x01800000 },
	{ DSS_DISP_GLB + MODULE_CORE_CLK_SEL, 0x00030000 },
	{ DSS_AIF0 + AIF_MODULE_CLK_SEL, 0 },
	{ DSS_AIF1 + AIF_MODULE_CLK_SEL, 0 },
	{ DSS_CMDLIST_CLK_SEL, 0 },
	{ DSS_AIF0 + AIF_CLK_SEL0, 0 },
	{ DSS_AIF0 + AIF_CLK_SEL1, 0 },
	{ DSS_SMMU + SMMU_LP_CTRL, 1 },
	{ DSS_AIF1 + AIF_CLK_SEL0, 0 },
	{ DSS_AIF1 + AIF_CLK_SEL1, 0 },
	{ DSS_DISP_CH0 + DISP_CH_CLK_SEL, 0 },
	{ DSS_HI_ACE_RAMCLK_FUNC, 0 },
	{ DSS_DPP + DPP_CLK_SEL, 0 },
	{ DSS_DSC_CLK_SEL, 0 },
	{ DSS_DBUF0 + DBUF_CLK_SEL, 0 },
	{ DSS_DISP_CH1 + DISP_CH_CLK_SEL, 0 },
	{ DSS_DPP1 + DPP_CLK_SEL, 0 },
	{ DSS_WB_CLK_SEL, 0 },
	{ DSS_MIF + MIF_CLK_CTL, 1 },
	{ DSS_MCTL_CTL(0) + MCTL_CTL_CLK_SEL, 0 },
	{ DSS_MCTL_CTL(1) + MCTL_CTL_CLK_SEL, 0 },
	{ DSS_MCTL_CTL(2) + MCTL_CTL_CLK_SEL, 0 },
	{ DSS_MCTL_CTL(3) + MCTL_CTL_CLK_SEL, 0 },
	{ DSS_MCTL_CTL(4) + MCTL_CTL_CLK_SEL, 0 },
	{ DSS_MCTL_CTL(5) + MCTL_CTL_CLK_SEL, 0 },
	{ DSS_MCTL_SYS + MCTL_MCTL_CLK_SEL, 0 },
	{ DSS_MCTL_SYS + MCTL_MOD_CLK_SEL, 0 },
	/* read channels VG0, VG1, VG2, G0, G1, D2, D3, D0, D1 */
	{ 0x20000 + DMA_CH_CLK_SEL, 0 },
	{ 0x20000 + DMA_FBCD_CTRL_GATE, 0xc },
	{ 0x28000 + DMA_CH_CLK_SEL, 0 },
	{ 0x28000 + DMA_FBCD_CTRL_GATE, 0xc },
	{ 0x30000 + DMA_CH_CLK_SEL, 0 },
	{ 0x38000 + DMA_CH_CLK_SEL, 0 },
	{ 0x38000 + DMA_FBCD_CTRL_GATE, 0xc },
	{ 0x40000 + DMA_CH_CLK_SEL, 0 },
	{ 0x40000 + DMA_FBCD_CTRL_GATE, 0xc },
	{ 0x50000 + DMA_CH_CLK_SEL, 0 },
	{ 0x51000 + DMA_CH_CLK_SEL, 0 },
	{ 0x52000 + DMA_CH_CLK_SEL, 0 },
	{ 0x52000 + DMA_FBCD_CTRL_GATE, 0xc },
	{ 0x53000 + DMA_CH_CLK_SEL, 0 },
	/* write channels */
	{ 0x5a000 + DMA_CH_CLK_SEL, 0 },
	{ 0x5a964, 0xc },
	{ 0x5c000 + DMA_CH_CLK_SEL, 0 },
	{ 0x5c964, 0xc },
	{ DSS_OVL0 + OV8_CLK_SEL, 0 },
	{ DSS_OVL2 + OV8_CLK_SEL, 0 },
	{ DSS_OVL3 + OV2_CLK_SEL, 0 },
	{ DSS_PIPE_SW_DSI0 + PIPE_SW_CLK_SEL, 0 },
	{ DSS_PIPE_SW_DSI0 + 0x40 + PIPE_SW_CLK_SEL, 0 },	/* DSI1 */
	{ DSS_PIPE_SW_DSI0 + 0x100 + PIPE_SW_CLK_SEL, 0 },	/* WB */
};

/* vendor init_dbuf(): one SRAM, thresholds as fractions of its depth */
static void kirin_init_dbuf(struct kirin_dss *k, const struct drm_display_mode *m)
{
	const u32 depth = DBUF_DEPTH, cg_in = depth - 1;
	const u32 bef_in = depth * 70 / 100, bef_out = depth * 80 / 100;
	const u32 b = DSS_DBUF0;

	dss_wr(k, b + DBUF_FRM_SIZE, m->hdisplay * m->vdisplay);
	dss_wr(k, b + DBUF_FRM_HSIZE, m->hdisplay - 1);
	dss_wr(k, b + DBUF_SRAM_VALID_NUM, 0);
	dss_wr(k, b + DBUF_THD_RQOS, ((depth * 90 / 100) << 16) | (depth * 80 / 100));
	dss_wr(k, b + DBUF_THD_WQOS, 0);
	dss_wr(k, b + DBUF_THD_CG, ((cg_in * 95 / 100) << 16) | cg_in);
	dss_wr(k, b + DBUF_THD_OTHER, 0);
	dss_wr(k, b + DBUF_THD_FLUX_REQ_BEF, (bef_out << 16) | bef_in);
	dss_wr(k, b + DBUF_THD_FLUX_REQ_AFT, (bef_out << 16) | bef_in);
	dss_wr(k, b + DBUF_THD_DFS_OK, bef_in);
	dss_wr(k, b + DBUF_FLUX_REQ_CTRL, 1);
	dss_wr(k, b + DBUF_DFS_LP_CTRL, 1);
	dss_wr(k, b + DBUF_DFS_RAM_MANAGE, 0xfff0);
	dss_wr(k, b + DBUF_THD_RQOS_IDLE, depth);
	dss_rmw(k, DSS_DISP_GLB + DSS_DFS_OK_MASK, BIT(0), 0);
}

/* vendor init_acm() + init_igm_gmp_xcc_gm(): colour processing of both DPPs bypassed */
static void kirin_init_dpp_bypass(struct kirin_dss *k)
{
	static const u32 dpp[] = { DSS_DPP, DSS_DPP1 };
	unsigned int i;

	dss_wr(k, DSS_DPP + DPP_ACM_MEM_CTRL_ES, 0x4);
	for (i = 0; i < ARRAY_SIZE(dpp); i++) {
		dss_wr(k, dpp[i] + DPP_DEGAMA_MEM_CTRL, 0x2);
		dss_wr(k, dpp[i] + DPP_GMP_MEM_CTRL, 0x2);
		dss_wr(k, dpp[i] + DPP_GAMA_MEM_CTRL, 0x2);
	}
	for (i = 0; i < ARRAY_SIZE(dpp); i++) {
		dss_rmw(k, dpp[i] + DPP_GMP_EN, BIT(0), 0);
		dss_rmw(k, dpp[i] + DPP_DEGAMA_EN, BIT(0), 0);
		dss_rmw(k, dpp[i] + DPP_GAMA_EN, BIT(0), 0);
		dss_rmw(k, dpp[i] + DPP_XCC_EN, BIT(0), 0);
	}
}

/* vendor dpe_interrupt_mask() + dpe_interrupt_clear(), without its stray 0x7000c write */
static void kirin_init_irqs(struct kirin_dss *k)
{
	unsigned int i;

	dss_wr(k, DSS_GLB + GLB_CPU_PDP_INT_MSK, ~0u);
	dss_wr(k, DSS_DISP_CH0 + DISP_CH_DPP_INT_MSK, ~0u);
	dss_wr(k, DSS_DBG + DBG_DSS_GLB_INTS + 4, ~0u);
	dss_wr(k, DSS_DBG + DBG_MCTL_INTS + 4, ~0u);
	dss_wr(k, DSS_DBG + DBG_WCH0_INTS + 4, ~0u);
	dss_wr(k, DSS_DBG + DBG_WCH1_INTS + 4, ~0u);
	for (i = 0; i < KIRIN_NUM_RCH; i++)
		dss_wr(k, DSS_DBG + DBG_RCH_INTS(i) + 4, ~0u);

	dss_wr(k, DSS_GLB + GLB_CPU_PDP_INTS, ~0u);
	dss_wr(k, DSS_DISP_CH0 + DISP_CH_DPP_INTS, ~0u);
	dss_wr(k, DSS_DBG + DBG_MCTL_INTS, ~0u);
	dss_wr(k, DSS_DBG + DBG_WCH0_INTS, ~0u);
	dss_wr(k, DSS_DBG + DBG_WCH1_INTS, ~0u);
	for (i = 0; i < KIRIN_NUM_RCH; i++)
		dss_wr(k, DSS_DBG + DBG_RCH_INTS(i), ~0u);
	dss_wr(k, DSS_DBG + DBG_DSS_GLB_INTS, ~0u);
}

/*
 * Everything but the planes, from reset, as the vendor's CRTC enable does it
 * (dpe_on() with dss_inner_clk_common_enable() and dpe_init(), then
 * hisi_overlay_on(), then the overlay base of the first frame). The planes
 * are programmed by the first commit (kirin_primary_program()), the DSI
 * block by kirin_dsi_on(), and the LDI starts after the first frame.
 *
 * Where UEFI and the vendor differ, this follows UEFI, which is what the
 * takeover path has been running with: the DSS SMMU stays in global bypass
 * (the vendor kernel used it as an IOMMU), the AXI/MIF channel settings keep
 * their reset values, and the GLB/DPP interrupts stay masked (vsync and
 * underflow come from the LDI).
 */
void kirin_dss_hw_init(struct kirin_dss *k, const struct drm_display_mode *m)
{
	u32 size = ((m->vdisplay - 1) << 16) | (m->hdisplay - 1);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(kirin_clk_gating); i++)
		dss_wr(k, kirin_clk_gating[i].off, kirin_clk_gating[i].val);

	/* dpe_init() for the primary display */
	kirin_init_dbuf(k, m);
	dss_wr(k, DSS_DISP_CH0 + DISP_CH_IMG_SIZE_BEF_SR, size);
	dss_wr(k, DSS_DISP_CH0 + DISP_CH_IMG_SIZE_AFT_SR, size);
	dss_wr(k, DSS_DISP_CH0 + DISP_CH_IMG_SIZE_AFT_IFBCSW, size);
	kirin_init_dpp_bypass(k);
	dss_rmw(k, DSS_PIPE_SW_DSI0 + PIPE_SW_SIG_CTRL, 0xff, 1);
	dss_rmw(k, DSS_PIPE_SW_DSI0 + SW_POS_CTRL_SIG_EN, BIT(0), 1);
	dss_rmw(k, DSS_PIPE_SW_DSI0 + PIPE_SW_DAT_CTRL, 0xff, 1);
	dss_rmw(k, DSS_PIPE_SW_DSI0 + SW_POS_CTRL_DAT_EN, BIT(0), 1);
	dss_rmw(k, DSS_DISP_GLB + DPPSW_SIG_CTRL, GENMASK(23, 0), 0x20101);
	dss_rmw(k, DSS_DISP_GLB + DPPSW_DAT_CTRL, GENMASK(23, 0), 0x20101);
	dss_rmw(k, DSS_DISP_GLB + IFBCSW_SIG_CTRL, GENMASK(15, 0), 0x403);
	dss_rmw(k, DSS_DISP_GLB + IFBCSW_DAT_CTRL, GENMASK(15, 0), 0x403);
	dss_rmw(k, DSS_DISP_GLB + DYN_SW_DEFAULT, BIT(0), 0);
	kirin_init_irqs(k);

	/* hisi_dss_on(): memory interface on, DSS SMMU in bypass */
	dss_rmw(k, DSS_MIF + MIF_ENABLE, BIT(0), BIT(0));
	for (i = 0; i < 12; i++)
		dss_rmw(k, DSS_MIF + MIF_CH(i) + MIF_CTRL0, BIT(0), BIT(0));
	dss_rmw(k, DSS_MIF + MIF_CMD_RELOAD, BIT(0), BIT(0));
	dss_wr(k, DSS_SMMU + SMMU_SCR, k->smmu_scr);
	dss_rmw(k, DSS_SMMU + SMMU_LP_CTRL, BIT(0), BIT(0));
	dss_rmw(k, DSS_SMMU + SMMU_CB_TTBCR, BIT(0), BIT(0));

	/* hisi_dss_mctl_on(): MCTL0 drives OV0 into interface 0 (DSI0), video mode */
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_EN, 1);
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX_ITF, 1);
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_DBG, 0x00b13a00);
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_TOP, 2);

	/*
	 * When the DSS kept its registers (power_off < 3), UEFI's channel is
	 * still set up: take every channel but ours off OV0.
	 */
	for (i = 0; i < KIRIN_NUM_RCH; i++) {
		if (i == KIRIN_PRIMARY_RCH || i == k->cursor_ch)
			continue;
		dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV_OEN(i), 0);
		dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX_RCH(i), 0);
		dss_rmw(k, kirin_rch[i].dma + DMA_CH_CTL, CH_CTL_EN, 0);
	}

	/* hisi_ov_base_config(): OV0 background, no layers yet */
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX_DBUF, 1);
	dss_wr(k, DSS_MCTL_CTL0 + MCTL_CTL_MUTEX_OV, 1);
	dss_wr(k, DSS_OVL0 + OV8_REG_DEFAULT, 1);
	dss_wr(k, DSS_OVL0 + OV8_REG_DEFAULT, 0);
	dss_wr(k, DSS_OVL0 + OV8_BLOCK_DBG, 0x4);
	dss_wr(k, DSS_OVL0 + OV_SIZE, size);
	dss_wr(k, DSS_OVL0 + OV_BG_COLOR_RGB, 0);
	dss_wr(k, DSS_OVL0 + OV_BG_COLOR_A, 0x3ff);
	dss_wr(k, DSS_OVL0 + OV_DST_STARTPOS, 0);
	dss_wr(k, DSS_OVL0 + OV_DST_ENDPOS, size);
	dss_rmw(k, DSS_OVL0 + OV_GCFG, BIT(0) | BIT(16), BIT(0) | BIT(16));
	dss_rmw(k, DSS_OVL0 + OV8_BLOCK_SIZE, GENMASK(30, 16), 0x7fff << 16);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL, 0xfffffffe);
	dss_wr(k, DSS_MCTL_SYS + MCTL_RCH_OV0_SEL1, 0xf);
	dss_wr(k, DSS_MCTL_SYS + MCTL_OV0_FLUSH_EN, 0xd);
}

/*
 * The primary plane after a power-up: a scaler-less channel (the vendor's
 * V0 needs its scaler and sharpener set up even at 1:1), full screen,
 * opaque, on OV0 layer 0 -- programmed like the cursor.
 */
void kirin_primary_program(struct kirin_dss *k, dma_addr_t addr, u32 pitch,
			   const struct kirin_format *fmt)
{
	const struct drm_display_mode *m = &k->fw_mode;

	k->ch = KIRIN_PRIMARY_RCH;
	k->layer = 0;
	k->smmu_translate = false;
	kirin_layer_program(k, k->ch, k->layer, addr, pitch, fmt, 0, m->hdisplay,
			    m->vdisplay, 0, 0, OV_ALPHA_OPAQUE);
}

