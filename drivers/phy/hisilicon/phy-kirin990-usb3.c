// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB 3.1 PHY for the HiSilicon Kirin 990 (vendor codename "apr").
 *
 * The block consists of:
 *  - the USB "misc ctrl" (usb3otg_bc) that holds the controller/PHY resets,
 *    the combo PHY power controls and the Type-C Assist (TCA) mux at +0x200,
 *  - a Synopsys USB 3.1 / DP combo PHY behind USB_DP_CTRL, whose SRAM is
 *    loaded with a firmware image through the PHY control register (CR) port,
 *  - the USB 2.0 PHY controls in the HSDT system controller.
 *
 * Only host operation is supported: the PHY brings up the combo PHY in
 * "USB 3.1 + DP 2 lanes" mode (the other two lanes stay available for DP),
 * releases the USB 2.0 PHY and takes the DWC3 controller out of reset, so
 * that the controller glue can populate the DWC3 core afterwards.
 *
 * Based on the vendor dwc3-apr.c/combophy.c/hisi_usb3_31phy_v2.c drivers.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

/* USB misc ctrl (usb3otg_bc) */
#define MISC_CFG00			0x00
#define  MISC_CFG00_VBUSVLD		GENMASK(7, 6)
#define MISC_CFG50			0x50
#define  MISC_CFG50_PHY_TESTPOWERDOWN	BIT(23)
#define MISC_CFG54			0x54
#define  MISC_CFG54_PHY_POWER_STABLE	(BIT(2) | BIT(5) | BIT(7))
#define MISC_CFGA0			0xa0
#define  MISC_CFGA0_PHY_RESET_N		BIT(1)
#define  MISC_CFGA0_CTRL_RESET_N	BIT(8)

/* Type-C Assist, inside the misc ctrl */
#define TCA_BASE			0x200
#define TCA_INTR_STS			(TCA_BASE + 0x08)
#define  TCA_INTR_XA_ACK		BIT(0)
#define  TCA_INTR_XA_TIMEOUT		BIT(1)
#define TCA_TCPC			(TCA_BASE + 0x14)
#define  TCA_TCPC_MUX			GENMASK(1, 0)
#define  TCA_TCPC_ORIENT_FLIP		BIT(2)
#define  TCA_TCPC_LOW_POWER_EN		BIT(3)
#define  TCA_TCPC_VALID			BIT(4)
#define  TCA_TCPC_MASK			GENMASK(4, 0)
#define TCA_CTRLSYNCMODE_CFG1		(TCA_BASE + 0x24)
#define  TCA_XA_TIMEOUT_DEFAULT		0x927c
#define TCA_CTRLSYNCMODE_DBG0		(TCA_BASE + 0x28)
#define TCA_PSTATE			(TCA_BASE + 0x30)
#define  TCA_PSTATE_ALL_P3		0x333333

enum tca_mux {
	TCA_MUX_NC = 0,
	TCA_MUX_USB = 1,
	TCA_MUX_DP4 = 2,
	TCA_MUX_USB_DP2 = 3,
};

/* USB_DP_CTRL */
#define DP_CTRL_CR_ADDR			0x08
#define  DP_CTRL_CR_ADDR_EN		BIT(16)
#define DP_CTRL_CR_DATA			0x0c
#define DP_CTRL_USB_CFG0		0x10
#define  USB_CFG0_HOST_FORCE_GEN1	BIT(19)
#define DP_CTRL_PHY_CFG0		0x20
#define  PHY_CFG0_SSC_EN		BIT(1)
#define  PHY_CFG0_CR_CLK_EN		BIT(3)
#define  PHY_CFG0_CR_SEL		BIT(4)
#define  PHY_CFG0_RX_CDR_LEGACY_EN	BIT(11)
#define DP_CTRL_PHY_CFG6		0x48
#define  PHY_CFG6_SRAM_EXT_LD_DONE	BIT(23)
#define  PHY_CFG6_SRAM_BYPASS		BIT(24)
#define DP_CTRL_PHY_STAT0		0x74
#define  PHY_STAT0_SRAM_INIT_DONE	BIT(2)

/* combo PHY CR registers */
#define CR_TX_VBOOST_LVL		0x21
#define  CR_TX_VBOOST_LVL_VAL		GENMASK(6, 4)
#define  CR_TX_VBOOST_LVL_EN		BIT(7)
#define CR_LANE_RX_SIGDET(n)		(0x4135 + (n) * 0x100)
#define CR_LANE_TERM(n)			(0x301a + (n) * 0x100)
#define CR_SRAM_BASE			0xc000

/* HSDT system controller: USB 2.0 PHY controls */
#define USB2_MISC_CTRL0			0x600
#define  USB2_CTRL0_VBUSVLD		GENMASK(6, 5)
#define  USB2_CTRL0_UTMI_16BIT		GENMASK(9, 8)
#define USB2_MISC_CTRL1			0x604	/* eye diagram parameters */
#define USB2_MISC_CTRL2			0x608
#define  USB2_CTRL2_SIDDQ		BIT(0)
#define USB2_MISC_CTRL3			0x60c
#define  USB2_CTRL3_REFCLKSEL		BIT(4)
#define  USB2_CTRL3_VREGBYPASS		BIT(12)
#define USB2_MISC_CTRL6			0x618
#define  USB2_CTRL6_RESET_N		BIT(0)
#define  USB2_CTRL6_SEL_ASP		BIT(5)

/* system controller */
#define SCTRL_SCDEEPSLEEPED		0x08
#define  SCDEEPSLEEPED_USB2_VREGBYPASS	BIT(7)
#define  SCDEEPSLEEPED_USB_CLK_PAD	BIT(20)

/* peripheral CRG */
#define PERI_CRG_ISODIS			0x148
#define  USB_REFCLK_ISO_EN		BIT(11)
#define PERI_CRG_CLKDIV9		0xcc
#define  CLKDIV9_SEL_ABB_BACKUP		BIT(4)
#define PERI_CRG_CLKDIV29		0x70c
#define  CLKDIV29_GT_CLK_ABB_SYS	BIT(1)
#define PERI_CRG_PEREN5			0x50
#define  GT_CLK_ABB_BACKUP		BIT(7)

/* PCTRL */
#define PCTRL_PERI_CTRL24		0x64
#define  PERI_CTRL24_USB3PHY_CLK_PAD	BIT(25)

/* PMCTRL: NoC power idle of the USB bus */
#define PMCTRL_NOC_POWER_IDLEREQ	0x380
#define PMCTRL_NOC_POWER_IDLEACK	0x384
#define PMCTRL_NOC_POWER_IDLE		0x388
#define  NOC_USB_IDLE			BIT(6)

/* MMC0 CRG: reset of the misc ctrl */
#define MMC0CRG_PERRSTEN0		0x20
#define MMC0CRG_PERRSTDIS0		0x24
#define  MMC0CRG_USB_MISC_RST		(BIT(5) | BIT(6))

/* DWC3 global registers (only touched while switching the TCA mux) */
#define DWC3_GUCTL			0xc12c
#define  DWC3_GUCTL_REFCLKPER_B26	BIT(26)
#define DWC3_GUSB3PIPECTL0		0xc2c0
#define  DWC3_GUSB3PIPECTL_SUSPHY	BIT(17)

#define KIRIN990_USB2PHY_REF_RATE	19200000
#define KIRIN990_DEFAULT_EYE_DIAGRAM	0x027cfee4
#define KIRIN990_DEFAULT_VBOOST		3
#define KIRIN990_FW_NAME		"hisilicon/kirin990-usb31phy.bin"

/* clocks, named as in the firmware device tree */
enum kirin990_usb_clk {
	CLK_USB3OTG_REF,	/* "clk_usb3phy_ref": controller/PHY ref */
	CLK_ACLK,		/* "aclk_usb3otg" */
	CLK_HCLK,		/* "hclk_usb3otg": misc ctrl bus clock */
	CLK_TCXO,		/* "clk_usb3_tcxo_en": combo PHY ref (ABB) */
	CLK_USB2PHY_REF,	/* "clk_usb2phy_ref" */
	CLK_NUM,
};

static const char * const kirin990_usb_clk_names[CLK_NUM] = {
	[CLK_USB3OTG_REF]	= "clk_usb3phy_ref",
	[CLK_ACLK]		= "aclk_usb3otg",
	[CLK_HCLK]		= "hclk_usb3otg",
	[CLK_TCXO]		= "clk_usb3_tcxo_en",
	[CLK_USB2PHY_REF]	= "clk_usb2phy_ref",
};

struct kirin990_usb_phy {
	struct device *dev;
	void __iomem *misc;
	void __iomem *dp_ctrl;
	void __iomem *dwc3;
	struct regmap *crg;
	struct regmap *mmc0crg;
	struct regmap *pctrl;
	struct regmap *sctrl;
	struct regmap *pmctrl;
	struct regmap *usb2_ctrl;
	struct clk *clks[CLK_NUM];
	struct regulator *vdd33;
	bool cr_started;

	enum tca_mux mux;
	bool flip;
	bool force_gen1;
	u32 eye_diagram;
	u32 tx_vboost_lvl;
	u32 term;
	bool term_valid;
	bool sram_loaded;
};

static inline void misc_rmw(struct kirin990_usb_phy *p, u32 off, u32 clr, u32 set)
{
	u32 v = readl(p->misc + off);

	writel((v & ~clr) | set, p->misc + off);
}

static inline void dp_rmw(struct kirin990_usb_phy *p, u32 off, u32 clr, u32 set)
{
	u32 v = readl(p->dp_ctrl + off);

	writel((v & ~clr) | set, p->dp_ctrl + off);
}

static inline void usb2_rmw(struct kirin990_usb_phy *p, u32 off, u32 clr, u32 set)
{
	regmap_update_bits(p->usb2_ctrl, off, clr | set, set);
}

/* ---- clocks ---- */

static int kirin990_clk_on(struct kirin990_usb_phy *p, enum kirin990_usb_clk id)
{
	return clk_prepare_enable(p->clks[id]);
}

static void kirin990_clk_off(struct kirin990_usb_phy *p, enum kirin990_usb_clk id)
{
	clk_disable_unprepare(p->clks[id]);
}

/* ---- combo PHY control register port ---- */

/*
 * USB_DP_CTRL is only reachable while the misc ctrl is out of reset and the
 * USB NoC is not idle, so track the CR port state and never touch the block
 * outside of that window (as the vendor driver does).
 */
static void kirin990_cr_start(struct kirin990_usb_phy *p)
{
	if (p->cr_started)
		return;
	dp_rmw(p, DP_CTRL_PHY_CFG0, PHY_CFG0_CR_CLK_EN, 0);
	dp_rmw(p, DP_CTRL_PHY_CFG0, 0, PHY_CFG0_CR_SEL);
	dp_rmw(p, DP_CTRL_PHY_CFG0, 0, PHY_CFG0_CR_CLK_EN);
	p->cr_started = true;
}

static void kirin990_cr_end(struct kirin990_usb_phy *p)
{
	if (!p->cr_started)
		return;
	dp_rmw(p, DP_CTRL_PHY_CFG0, PHY_CFG0_CR_CLK_EN, 0);
	dp_rmw(p, DP_CTRL_PHY_CFG0, PHY_CFG0_CR_SEL, 0);
	p->cr_started = false;
}

static u32 kirin990_cr_read(struct kirin990_usb_phy *p, u32 addr)
{
	writel((addr & 0xffff) | DP_CTRL_CR_ADDR_EN, p->dp_ctrl + DP_CTRL_CR_ADDR);
	return readl(p->dp_ctrl + DP_CTRL_CR_DATA);
}

static void kirin990_cr_write(struct kirin990_usb_phy *p, u32 addr, u32 val)
{
	writel((addr & 0xffff) | DP_CTRL_CR_ADDR_EN, p->dp_ctrl + DP_CTRL_CR_ADDR);
	writel(val, p->dp_ctrl + DP_CTRL_CR_DATA);
}

/* ---- misc ctrl ---- */

static void kirin990_noc_idle(struct kirin990_usb_phy *p, bool idle)
{
	u32 ack, st;
	int i;

	/* hiword-mask register */
	regmap_write(p->pmctrl, PMCTRL_NOC_POWER_IDLEREQ,
		     (NOC_USB_IDLE << 16) | (idle ? NOC_USB_IDLE : 0));

	for (i = 0; i < 10; i++) {
		regmap_read(p->pmctrl, PMCTRL_NOC_POWER_IDLEACK, &ack);
		regmap_read(p->pmctrl, PMCTRL_NOC_POWER_IDLE, &st);
		if (!!(ack & NOC_USB_IDLE) == idle && !!(st & NOC_USB_IDLE) == idle)
			return;
		udelay(10);
	}
	dev_warn(p->dev, "NoC power idle %s timeout (ack %#x state %#x)\n",
		 idle ? "enter" : "exit", ack, st);
}

static int kirin990_misc_ctrl_on(struct kirin990_usb_phy *p)
{
	int ret;

	ret = kirin990_clk_on(p, CLK_HCLK);
	if (ret)
		return ret;
	ret = kirin990_clk_on(p, CLK_ACLK);
	if (ret)
		goto err_hclk;

	/*
	 * Start from a reset misc ctrl. No USB_DP_CTRL access here: after
	 * phy_exit (or depending on the boot firmware) the USB NoC is idle and
	 * an access would take an external abort.
	 */
	kirin990_noc_idle(p, true);
	udelay(100);

	regmap_write(p->mmc0crg, MMC0CRG_PERRSTEN0, MMC0CRG_USB_MISC_RST);
	regmap_write(p->mmc0crg, MMC0CRG_PERRSTDIS0, MMC0CRG_USB_MISC_RST);
	udelay(100);

	kirin990_noc_idle(p, false);
	kirin990_cr_start(p);
	return 0;

err_hclk:
	kirin990_clk_off(p, CLK_HCLK);
	return ret;
}

static void kirin990_misc_ctrl_off(struct kirin990_usb_phy *p)
{
	kirin990_cr_end(p);
	kirin990_noc_idle(p, true);
	kirin990_clk_off(p, CLK_ACLK);
	kirin990_clk_off(p, CLK_HCLK);
}

/* ---- combo PHY ---- */

static bool kirin990_usb_clk_from_pad(struct kirin990_usb_phy *p)
{
	u32 v;

	regmap_read(p->sctrl, SCTRL_SCDEEPSLEEPED, &v);
	return v & SCDEEPSLEEPED_USB_CLK_PAD;
}

static int kirin990_combophy_clk_on(struct kirin990_usb_phy *p)
{
	int ret;

	if (!kirin990_usb_clk_from_pad(p)) {
		/* reference clock from the ABB (TCXO buffer) */
		regmap_write(p->crg, PERI_CRG_ISODIS, USB_REFCLK_ISO_EN);
		ret = kirin990_clk_on(p, CLK_TCXO);
		if (ret)
			return ret;
		mdelay(10);
		regmap_update_bits(p->pctrl, PCTRL_PERI_CTRL24,
				   PERI_CTRL24_USB3PHY_CLK_PAD, 0);
	} else {
		dev_info(p->dev, "combo PHY clock from pad\n");
		regmap_write(p->crg, PERI_CRG_CLKDIV9, CLKDIV9_SEL_ABB_BACKUP << 16);
		regmap_write(p->crg, PERI_CRG_CLKDIV29,
			     CLKDIV29_GT_CLK_ABB_SYS | (CLKDIV29_GT_CLK_ABB_SYS << 16));
		regmap_write(p->crg, PERI_CRG_PEREN5, GT_CLK_ABB_BACKUP);
		regmap_update_bits(p->pctrl, PCTRL_PERI_CTRL24,
				   PERI_CTRL24_USB3PHY_CLK_PAD,
				   PERI_CTRL24_USB3PHY_CLK_PAD);
	}
	return 0;
}

static void kirin990_combophy_clk_off(struct kirin990_usb_phy *p)
{
	if (!kirin990_usb_clk_from_pad(p))
		kirin990_clk_off(p, CLK_TCXO);
	else
		regmap_write(p->crg, PERI_CRG_PEREN5 + 4, GT_CLK_ABB_BACKUP);
}

static void kirin990_combophy_load_fw(struct kirin990_usb_phy *p,
				      const struct firmware *fw)
{
	const __le16 *words = (const __le16 *)fw->data;
	size_t i, n = fw->size / 2;
	u32 v;

	kirin990_cr_start(p);
	if (readl_poll_timeout_atomic(p->dp_ctrl + DP_CTRL_PHY_STAT0, v,
				      v & PHY_STAT0_SRAM_INIT_DONE, 10, 1000))
		dev_warn(p->dev, "combo PHY SRAM init timeout\n");

	for (i = 0; i < n; i++)
		kirin990_cr_write(p, CR_SRAM_BASE + i, le16_to_cpu(words[i]));

	dp_rmw(p, DP_CTRL_PHY_CFG0, 0, PHY_CFG0_RX_CDR_LEGACY_EN);
	dp_rmw(p, DP_CTRL_PHY_CFG6, 0, PHY_CFG6_SRAM_EXT_LD_DONE);
	udelay(100);
	p->sram_loaded = true;
}

static int kirin990_combophy_power_on(struct kirin990_usb_phy *p)
{
	const struct firmware *fw = NULL;
	u32 v;
	int ret;

	p->sram_loaded = false;
	ret = request_firmware(&fw, KIRIN990_FW_NAME, p->dev);
	if (ret)
		dev_warn(p->dev, "no %s (%d), combo PHY runs its ROM code\n",
			 KIRIN990_FW_NAME, ret);
	else if (fw->size & 1 || fw->size > 2 * (0x10000 - CR_SRAM_BASE)) {
		dev_warn(p->dev, "bad firmware size %zu, ignored\n", fw->size);
		release_firmware(fw);
		fw = NULL;
	}

	writel(TCA_XA_TIMEOUT_DEFAULT / 3, p->misc + TCA_CTRLSYNCMODE_CFG1);

	ret = kirin990_combophy_clk_on(p);
	if (ret)
		goto out;

	misc_rmw(p, MISC_CFG50, MISC_CFG50_PHY_TESTPOWERDOWN, 0);
	udelay(50);
	misc_rmw(p, MISC_CFG54, 0, MISC_CFG54_PHY_POWER_STABLE);

	/* with a firmware image the PHY waits for it in SRAM, else runs ROM code */
	dp_rmw(p, DP_CTRL_PHY_CFG6, PHY_CFG6_SRAM_BYPASS,
	       fw ? 0 : PHY_CFG6_SRAM_BYPASS);

	misc_rmw(p, MISC_CFGA0, 0, MISC_CFGA0_PHY_RESET_N);

	if (fw)
		kirin990_combophy_load_fw(p, fw);

	/* the TCA acknowledges the PHY coming up */
	if (readl_poll_timeout(p->misc + TCA_INTR_STS, v,
			       v & (TCA_INTR_XA_ACK | TCA_INTR_XA_TIMEOUT),
			       20000, 1000000))
		dev_warn(p->dev, "no TCA ack after PHY power up\n");
	writel(0xffff, p->misc + TCA_INTR_STS);

	misc_rmw(p, TCA_TCPC, TCA_TCPC_LOW_POWER_EN, 0);
	udelay(2);
	writel(2 * TCA_XA_TIMEOUT_DEFAULT, p->misc + TCA_CTRLSYNCMODE_CFG1);
out:
	release_firmware(fw);
	return ret;
}

static void kirin990_combophy_power_off(struct kirin990_usb_phy *p)
{
	int lane;

	/* disable rx signal detect, then enter test power down */
	for (lane = 0; lane < 2; lane++)
		kirin990_cr_write(p, CR_LANE_RX_SIGDET(lane), 0x2a);
	udelay(10);
	misc_rmw(p, MISC_CFG50, 0, MISC_CFG50_PHY_TESTPOWERDOWN);
	kirin990_combophy_clk_off(p);
}

/* ---- controller ---- */

static int kirin990_controller_clks_on(struct kirin990_usb_phy *p)
{
	int ret;

	ret = kirin990_clk_on(p, CLK_ACLK);
	if (ret)
		return ret;
	ret = kirin990_clk_on(p, CLK_USB3OTG_REF);
	if (ret) {
		kirin990_clk_off(p, CLK_ACLK);
		return ret;
	}

	/* the USB 2.0 PHY belongs to the USB 3.1 controller (not to the ASP) */
	usb2_rmw(p, USB2_MISC_CTRL6, USB2_CTRL6_SEL_ASP, 0);
	usb2_rmw(p, USB2_MISC_CTRL3, 0, USB2_CTRL3_REFCLKSEL);
	return 0;
}

static void kirin990_controller_clks_off(struct kirin990_usb_phy *p)
{
	kirin990_clk_off(p, CLK_USB3OTG_REF);
	kirin990_clk_off(p, CLK_ACLK);
}

static void kirin990_controller_reset(struct kirin990_usb_phy *p)
{
	misc_rmw(p, MISC_CFGA0, MISC_CFGA0_CTRL_RESET_N, 0);
	usb2_rmw(p, USB2_MISC_CTRL6, USB2_CTRL6_RESET_N, 0);
	usb2_rmw(p, USB2_MISC_CTRL2, 0, USB2_CTRL2_SIDDQ);
}

/*
 * Mux switches are synchronised with the controller: it has to be running
 * and allowed to put its PIPE interface into P3 while the TCA reconfigures
 * the lanes.
 */
static int kirin990_tca_switch(struct kirin990_usb_phy *p, enum tca_mux mux)
{
	u32 v, sts = 0;
	int ret, try;

	ret = kirin990_controller_clks_on(p);
	if (ret)
		return ret;
	usb2_rmw(p, USB2_MISC_CTRL2, USB2_CTRL2_SIDDQ, 0);
	usb2_rmw(p, USB2_MISC_CTRL6, 0, USB2_CTRL6_RESET_N);
	udelay(100);
	misc_rmw(p, MISC_CFGA0, 0, MISC_CFGA0_CTRL_RESET_N);
	udelay(100);

	v = readl(p->dwc3 + DWC3_GUCTL);
	writel(v | DWC3_GUCTL_REFCLKPER_B26, p->dwc3 + DWC3_GUCTL);
	v = readl(p->dwc3 + DWC3_GUSB3PIPECTL0);
	writel(v | DWC3_GUSB3PIPECTL_SUSPHY, p->dwc3 + DWC3_GUSB3PIPECTL0);

	if (mux == TCA_MUX_NC &&
	    readl_poll_timeout(p->misc + TCA_PSTATE, v, v == TCA_PSTATE_ALL_P3,
			       10000, 500000))
		dev_dbg(p->dev, "PSTATE %#x before disconnect\n", v);

	v = readl(p->misc + TCA_PSTATE);
	if (!v) {
		dev_err(p->dev, "TCA PSTATE is 0, PHY not running\n");
		ret = -EIO;
		goto out;
	}

	ret = -ETIMEDOUT;
	for (try = 0; try < 3; try++) {
		writel(0xffff, p->misc + TCA_INTR_STS);
		udelay(1);
		v = TCA_TCPC_VALID | FIELD_PREP(TCA_TCPC_MUX, mux);
		if (p->flip)
			v |= TCA_TCPC_ORIENT_FLIP;
		misc_rmw(p, TCA_TCPC, TCA_TCPC_MASK, v);

		readl_poll_timeout(p->misc + TCA_INTR_STS, sts,
				   sts & (TCA_INTR_XA_ACK | TCA_INTR_XA_TIMEOUT),
				   5000, 600000);
		if (sts & TCA_INTR_XA_ACK) {
			ret = 0;
			break;
		}
		dev_warn(p->dev, "TCA switch to %d: sts %#x dbg0 %#x pstate %#x\n",
			 mux, sts, readl(p->misc + TCA_CTRLSYNCMODE_DBG0),
			 readl(p->misc + TCA_PSTATE));
		msleep(50);
	}
	writel(TCA_INTR_XA_ACK | TCA_INTR_XA_TIMEOUT, p->misc + TCA_INTR_STS);
	dev_dbg(p->dev, "TCA mux %d: TCPC %#x PSTATE %#x\n", mux,
		readl(p->misc + TCA_TCPC), readl(p->misc + TCA_PSTATE));
out:
	/* back to reset; the host start below brings it up cleanly */
	kirin990_controller_reset(p);
	kirin990_controller_clks_off(p);
	return ret;
}

static int kirin990_controller_start(struct kirin990_usb_phy *p)
{
	int ret;

	ret = kirin990_controller_clks_on(p);
	if (ret)
		return ret;

	ret = clk_set_rate(p->clks[CLK_USB2PHY_REF], KIRIN990_USB2PHY_REF_RATE);
	if (ret)
		goto err_clks;
	ret = kirin990_clk_on(p, CLK_USB2PHY_REF);
	if (ret)
		goto err_clks;
	udelay(100);

	usb2_rmw(p, USB2_MISC_CTRL2, USB2_CTRL2_SIDDQ, 0);
	dp_rmw(p, DP_CTRL_PHY_CFG0, 0, PHY_CFG0_SSC_EN);

	/* 16 bit UTMI at 30 MHz */
	usb2_rmw(p, USB2_MISC_CTRL0, 0, USB2_CTRL0_UTMI_16BIT);
	udelay(100);
	usb2_rmw(p, USB2_MISC_CTRL6, 0, USB2_CTRL6_RESET_N);

	if (p->force_gen1)
		dp_rmw(p, DP_CTRL_USB_CFG0, 0, USB_CFG0_HOST_FORCE_GEN1);

	misc_rmw(p, MISC_CFGA0, 0, MISC_CFGA0_CTRL_RESET_N);
	udelay(100);

	/* no VBUS sensing on the host-only port: force VBUS valid */
	misc_rmw(p, MISC_CFG00, 0, MISC_CFG00_VBUSVLD);
	usb2_rmw(p, USB2_MISC_CTRL0, 0, USB2_CTRL0_VBUSVLD);
	mdelay(10);
	return 0;

err_clks:
	kirin990_controller_clks_off(p);
	return ret;
}

static void kirin990_phy_tune(struct kirin990_usb_phy *p)
{
	u32 v;
	int lane;

	regmap_write(p->usb2_ctrl, USB2_MISC_CTRL1, p->eye_diagram);

	if (p->tx_vboost_lvl <= 5) {
		kirin990_cr_read(p, CR_TX_VBOOST_LVL);
		v = kirin990_cr_read(p, CR_TX_VBOOST_LVL);
		v &= ~CR_TX_VBOOST_LVL_VAL;
		v |= CR_TX_VBOOST_LVL_EN |
		     FIELD_PREP(CR_TX_VBOOST_LVL_VAL, p->tx_vboost_lvl);
		kirin990_cr_write(p, CR_TX_VBOOST_LVL, v & 0xffff);
	}

	if (p->term_valid)
		for (lane = 0; lane < 4; lane++)
			kirin990_cr_write(p, CR_LANE_TERM(lane), p->term);
}

static void kirin990_usb2_vregbypass(struct kirin990_usb_phy *p)
{
	u32 v;

	regmap_read(p->sctrl, SCTRL_SCDEEPSLEEPED, &v);
	if (v & SCDEEPSLEEPED_USB2_VREGBYPASS)
		usb2_rmw(p, USB2_MISC_CTRL3, 0, USB2_CTRL3_VREGBYPASS);
}

static int kirin990_usb_phy_init(struct phy *phy)
{
	struct kirin990_usb_phy *p = phy_get_drvdata(phy);
	u32 clk, rst, noc;
	int ret;

	/* the USB blocks may be gated here: only look at the system controllers */
	regmap_read(p->mmc0crg, 0x0c, &clk);
	regmap_read(p->mmc0crg, 0x28, &rst);
	regmap_read(p->pmctrl, PMCTRL_NOC_POWER_IDLE, &noc);
	dev_dbg(p->dev, "before init: mmc0crg clk %#x rst %#x, noc idle %#x\n",
		clk, rst, noc);

	ret = regulator_enable(p->vdd33);
	if (ret)
		return ret;

	kirin990_usb2_vregbypass(p);

	ret = kirin990_misc_ctrl_on(p);
	if (ret)
		goto err_vdd;

	ret = kirin990_combophy_power_on(p);
	if (ret)
		goto err_misc;

	ret = kirin990_tca_switch(p, TCA_MUX_NC);
	if (!ret)
		ret = kirin990_tca_switch(p, p->mux);
	if (ret)
		goto err_combophy;

	ret = kirin990_controller_start(p);
	if (ret)
		goto err_combophy;

	kirin990_phy_tune(p);

	dev_info(p->dev, "up: fw %s, mux %d, TCPC %#x PSTATE %#x, misc a0 %#x\n",
		 p->sram_loaded ? "loaded" : "rom", p->mux,
		 readl(p->misc + TCA_TCPC), readl(p->misc + TCA_PSTATE),
		 readl(p->misc + MISC_CFGA0));
	return 0;

err_combophy:
	kirin990_combophy_power_off(p);
err_misc:
	kirin990_misc_ctrl_off(p);
err_vdd:
	regulator_disable(p->vdd33);
	return ret;
}

static int kirin990_usb_phy_exit(struct phy *phy)
{
	struct kirin990_usb_phy *p = phy_get_drvdata(phy);

	kirin990_controller_reset(p);
	kirin990_clk_off(p, CLK_USB2PHY_REF);
	kirin990_controller_clks_off(p);
	kirin990_combophy_power_off(p);
	kirin990_misc_ctrl_off(p);
	regulator_disable(p->vdd33);
	return 0;
}

static const struct phy_ops kirin990_usb_phy_ops = {
	.init		= kirin990_usb_phy_init,
	.exit		= kirin990_usb_phy_exit,
	.owner		= THIS_MODULE,
};

static struct regmap *kirin990_get_syscon(struct device *dev, const char *prop)
{
	struct device_node *np;
	struct regmap *map;

	np = of_parse_phandle(dev->of_node, prop, 0);
	if (!np)
		return dev_err_ptr_probe(dev, -ENODEV, "missing %s\n", prop);
	map = device_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(map))
		dev_err(dev, "%s: %pe\n", prop, map);
	return map;
}

static void __iomem *kirin990_iomap_phandle(struct device *dev, const char *prop,
					    resource_size_t offset,
					    resource_size_t size)
{
	struct device_node *np;
	struct resource res;
	int ret;

	np = of_parse_phandle(dev->of_node, prop, 0);
	if (!np)
		return IOMEM_ERR_PTR(dev_err_probe(dev, -ENODEV, "missing %s\n", prop));
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return IOMEM_ERR_PTR(ret);
	if (!size)
		size = resource_size(&res);
	/* shared with other drivers: map without claiming the region */
	return devm_ioremap(dev, res.start + offset, size) ?: IOMEM_ERR_PTR(-ENOMEM);
}

/*
 * Tuning values live in the controller ("hisilicon,dwc3-usb") and
 * "hisilicon,pd" nodes of the firmware device tree.
 */
static void kirin990_usb_phy_parse(struct kirin990_usb_phy *p)
{
	struct device_node *np;
	const char *s;

	p->eye_diagram = KIRIN990_DEFAULT_EYE_DIAGRAM;
	p->tx_vboost_lvl = KIRIN990_DEFAULT_VBOOST;
	p->mux = TCA_MUX_USB_DP2;
	p->force_gen1 = true;
	p->term_valid = !of_property_read_u32(p->dev->of_node, "usb3_phy_term",
					      &p->term);

	np = of_find_compatible_node(NULL, NULL, "hisilicon,dwc3-usb");
	if (np) {
		of_property_read_u32(np, "eye_diagram_host_param", &p->eye_diagram);
		of_property_read_u32(np, "usb3_phy_tx_vboost_lvl", &p->tx_vboost_lvl);
		if (!of_property_read_string(np, "host-maximum-speed", &s))
			p->force_gen1 = strcmp(s, "super-speed-plus") != 0;
		of_node_put(np);
	}

	np = of_find_compatible_node(NULL, NULL, "hisilicon,pd");
	if (np) {
		if (!of_property_read_string(np, "init-mode", &s) &&
		    strcmp(s, "usb_dp") != 0)
			p->mux = TCA_MUX_USB;
		of_node_put(np);
	}
}

static int kirin990_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct kirin990_usb_phy *p;
	struct phy *phy;
	int i;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;

	p->misc = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->misc))
		return PTR_ERR(p->misc);

	p->dp_ctrl = kirin990_iomap_phandle(dev, "hisilicon,usb-dp-ctrl", 0, 0);
	if (IS_ERR(p->dp_ctrl))
		return PTR_ERR(p->dp_ctrl);

	/* global registers of the DWC3 core, for the TCA handshake */
	p->dwc3 = kirin990_iomap_phandle(dev, "hisilicon,dwc3", 0, 0x10000);
	if (IS_ERR(p->dwc3))
		return PTR_ERR(p->dwc3);

	p->crg = kirin990_get_syscon(dev, "hisilicon,pericrg-syscon");
	p->mmc0crg = kirin990_get_syscon(dev, "hisilicon,mmc0crg-syscon");
	p->pctrl = kirin990_get_syscon(dev, "hisilicon,pctrl-syscon");
	p->sctrl = kirin990_get_syscon(dev, "hisilicon,sctrl-syscon");
	p->pmctrl = kirin990_get_syscon(dev, "hisilicon,pmctrl-syscon");
	p->usb2_ctrl = kirin990_get_syscon(dev, "hisilicon,hsdt-sctrl-syscon");
	if (IS_ERR(p->crg) || IS_ERR(p->mmc0crg) || IS_ERR(p->pctrl) ||
	    IS_ERR(p->sctrl) || IS_ERR(p->pmctrl) || IS_ERR(p->usb2_ctrl))
		return -ENODEV;

	for (i = 0; i < CLK_NUM; i++) {
		p->clks[i] = devm_clk_get(dev, kirin990_usb_clk_names[i]);
		if (IS_ERR(p->clks[i]))
			return dev_err_probe(dev, PTR_ERR(p->clks[i]), "clock %s\n",
					     kirin990_usb_clk_names[i]);
	}

	/* 3.3 V for the USB 2.0 PHY */
	p->vdd33 = devm_regulator_get(dev, "usb_phy_ldo_33v");
	if (IS_ERR(p->vdd33))
		return dev_err_probe(dev, PTR_ERR(p->vdd33), "usb_phy_ldo_33v\n");

	kirin990_usb_phy_parse(p);

	phy = devm_phy_create(dev, NULL, &kirin990_usb_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, p);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id kirin990_usb_phy_of_match[] = {
	{ .compatible = "hisilicon,apr-dwc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin990_usb_phy_of_match);

static struct platform_driver kirin990_usb_phy_driver = {
	.probe	= kirin990_usb_phy_probe,
	.driver = {
		.name		= "kirin990-usb-phy",
		.of_match_table	= kirin990_usb_phy_of_match,
	},
};
module_platform_driver(kirin990_usb_phy_driver);

MODULE_FIRMWARE(KIRIN990_FW_NAME);
MODULE_DESCRIPTION("HiSilicon Kirin 990 USB 3.1 combo PHY driver");
MODULE_LICENSE("GPL");
