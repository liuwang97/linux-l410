// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for the HiSilicon Kirin 990 ("kport") SoC
 *
 * The SoC has two DesignWare root complexes. Each has an ELBI/APB block,
 * a Synopsys C10 PHY (patched from SRAM at power up) and a DBI window that
 * is shared with the outbound AXI window: DBI accesses are selected with a
 * sideband bit in the ELBI block, everything else is translated by the
 * iATU. Both PHYs take their 100 MHz reference from one FNPLL in the HSDT
 * CRG. RC0 carries the on-board RTL8168, RC1 the Hi110x WiFi/BT chip.
 *
 * The firmware devicetree describes the RCs with the vendor "pcie-kport,rc"
 * binding, which this driver consumes as is.
 *
 * Ported from the vendor pcie-kport.c/pcie-apr.c drivers,
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/platform_drivers/pcie-kport-api.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "pcie-designware.h"
#include "pcie-kport-phy-fw.h"

#define KPORT_MAX_RC			2

/* ELBI ("apb") registers */
#define CTRL0				0x000
#define  CTRL0_DEV_TYPE			GENMASK(31, 28)
#define  CTRL0_DEV_TYPE_RC		4
#define  CTRLx_SLV_DBI_EN		BIT(21)	/* CTRL0: writes, CTRL1: reads */
#define CTRL1				0x004
#define CTRL7				0x01c
#define  CTRL7_LTSSM_EN			BIT(11)
#define  CTRL7_AUX_PWR_DET		BIT(10)
#define CTRL10				0x028
#define  CTRL10_AXI_TIMEOUT_MASK	BIT(22)
#define CTRL11				0x02c
#define  CTRL11_AXI_TIMEOUT_CLR		BIT(22)
#define CTRL12				0x030
#define  CTRL12_PERST_IN		GENMASK(3, 2)
#define  CTRL12_PERST_IN_RC		BIT(2)
#define  CTRL12_PERST_OE		BIT(1)
#define  CTRL12_PERST_OUT		BIT(0)
#define CTRL21				0x054
#define  CTRL21_IO_CLK_SEL		GENMASK(18, 17)	/* 0: from CIO */
#define CTRL22				0x058
#define  CTRL22_CLKREQ_OUT		BIT(0)
#define CTRL25				0x064
#define  CTRL25_AXI_TIMEOUT		GENMASK(6, 1)
#define  CTRL25_AXI_TIMEOUT_48MS	0x36
#define STATE0				0x400
#define  STATE0_LINK_UP			(BIT(15) | BIT(5))
#define STATE1				0x404
#define  STATE1_PME_ACK			BIT(16)
#define STATE4				0x410
#define  STATE4_LTSSM			GENMASK(5, 0)

/* PHY APB registers, "phy" + apb offset */
#define PHY_CTRL0			0x000
#define  PHY_CTRL0_REF_USE_CIO_PAD	BIT(14)
#define  PHY_CTRL0_TEST_POWERDOWN	BIT(22)
#define PHY_CTRL1			0x004
#define  PHY_CTRL1_REF_USE_PAD		BIT(8)
#define  PHY_CTRL1_PHY_RESET		BIT(16)
#define  PHY_CTRL1_PHY_RESET_SEL	BIT(17)	/* 1: reset driven by the controller */
#define  PHY_CTRL1_LANE0_RESET		BIT(19)
#define PHY_CTRL40			0x0a0
#define  PHY_CTRL40_SRAM_BYPASS		BIT(0)
#define  PHY_CTRL40_SRAM_EXT_LD_DONE	BIT(4)
#define PHY_CTRL150			0x258
#define  PHY_CTRL150_CDR_LEGACY_EN	BIT(0)
#define PHY_STATE0			0x400
#define  PHY_STATE0_PIPE_CLK_UNSTABLE	BIT(19)
#define PHY_STATE39			0x49c
#define  PHY_STATE39_SRAM_INIT_DONE	BIT(0)

/* PHY "natural" (CR) registers, 16 bit each at "phy" + natural offset + 4 * reg */
#define PHY_SUP_DIG_LVL_OVRD_IN		0x21
#define  PHY_SUP_DIG_LVL_VBOOST		0xb5
#define PHY_RAWAONLANEN_DIG_RX_OVRD_OUT_3 0x4035
#define  PHY_RX_SIGDET_OFF		0x2a

/* DBI port logic */
#define PORT_GEN3_RELATED_OFF		0x890
#define  GEN3_ZRXDC_NONCOMPL		BIT(0)

/*
 * HSDT CRG, per-RC gate bits are shifted by 12 for RC1. Bits 0 and 3-5
 * (aux, APB PHY/SYS, AXI) are the DT clocks and belong to the clock driver.
 */
#define HSDT_PEREN1			0x010
#define HSDT_PERDIS1			0x014
#define  HSDT_GT_HP			BIT(1)
#define  HSDT_GT_DEBOUNCE		BIT(2)
#define  HSDT_GT_PHYREF			BIT(6)
#define  HSDT_GT_IO			BIT(7)
#define HSDT_PCIEPLL_STATE		0x208
#define  FNPLL_LOCK			BIT(4)
#define HSDT_PLL_CFG(n)			(0x224 + 4 * (n))
#define  FNPLL_EN			BIT(0)
#define  FNPLL_BYPASS			BIT(1)
#define HSDT_PCIECTRL(rc)		(0x300 + 4 * (rc))
#define  PCIEIO_HW_BYPASS		BIT(0)
#define  PCIEPHY_REF_HW_BYPASS		BIT(1)
#define  PCIEIO_OE_EN_SOFT		BIT(6)
#define  PCIEIO_OE_POLAR		BIT(9)
#define  PCIEIO_OE_EN_HARD_BYPASS	BIT(11)
#define  PCIEIO_IE_EN_HARD_BYPASS	BIT(27)
#define  PCIEIO_IE_EN_SOFT		BIT(28)
#define  PCIEIO_IE_POLAR		BIT(29)

/* PMCTRL NoC power idle */
#define PMC_NOC_POWER_IDLEREQ		0x380
#define PMC_NOC_POWER_IDLE		0x388
#define  PMC_NOC_PCIE(rc)		BIT(10 + (rc))

#define CHIP_TYPE_CS2			3
#define EP_DEVICE_HI110X		2

enum kport_clk {
	KPORT_CLK_AUX,
	KPORT_CLK_APB_PHY,
	KPORT_CLK_APB_SYS,
	KPORT_CLK_ACLK,
	KPORT_NUM_CLKS,
};

static const char * const kport_clk_names[KPORT_NUM_CLKS] = {
	"pcie_aux", "pcie_apb_phy", "pcie_apb_sys", "pcie_aclk",
};

struct kport_pcie {
	struct dw_pcie		pci;
	u32			rc_id;
	void __iomem		*apb;
	void __iomem		*phy;
	u32			phy_natural_off;
	u32			phy_sram_off;
	u32			phy_apb_off;
	struct regmap		*hsdt;
	struct regmap		*sctrl;
	struct regmap		*pmctrl;
	struct clk		*clks[KPORT_NUM_CLKS];
	struct gpio_desc	*perst;
	struct regulator	*vpcie;

	u32			iso[2];		/* sysctrl ISOEN offset, bits */
	u32			rst[2];		/* HSDT PERRSTEN offset, bits */
	u32			t_ref2perst[2];
	u32			t_perst2access[2];
	u32			t_perst2rst[2];
	u32			*eye;		/* triplets: reg, mask, value */
	u32			eye_num;
	u32			chip_type;
	u32			eco;
	u32			ep_type;
	u32			aspm_state;	/* PCI_EXP_LNKCAP_ASPMS encoding */
	u16			device_id;

	/* DBI sideband select and DBI/child config accesses */
	raw_spinlock_t		dbi_lock;
	void __iomem		*(*child_map_bus)(struct pci_bus *bus,
						  unsigned int devfn, int where);
	struct pci_ops		root_ops;
	struct pci_ops		child_ops;

	/*
	 * Locking: enum_lock serializes enumerate/rescan/remove and is held
	 * across endpoint probe/remove. lock guards the power state and is
	 * never held while the PCI core probes or removes endpoints, so an
	 * endpoint driver may call pm_control/lp_ctrl/register_event from its
	 * probe. ep_lock guards the endpoint callbacks and event registration.
	 * Order: enum_lock -> lock -> ep_lock.
	 */
	struct mutex		enum_lock;
	struct mutex		lock;
	spinlock_t		ep_lock;
	bool			powered;
	bool			bridge_up;
	bool			enumerated;
	bool			usr_suspended;
	bool			pm_hidden;	/* RC off across a system sleep */
	u32			ep_link_status;
	struct pci_dev		*root_port;
	struct pci_saved_state	*rp_state;

	int			(*ep_poweron)(void *data);
	int			(*ep_poweroff)(void *data);
	void			*ep_data;
	struct pcie_kport_register_event *event_reg;
	struct work_struct	linkdown_work;
	struct work_struct	cpltimeout_work;
};

#define to_kport(x)	container_of((x), struct kport_pcie, pci)

static struct kport_pcie *kport_rcs[KPORT_MAX_RC];
static DEFINE_MUTEX(kport_rcs_lock);

/* The FNPLL feeds both RCs */
static DEFINE_MUTEX(kport_pll_lock);
static unsigned int kport_pll_users;

static unsigned int probe_enum;
module_param(probe_enum, uint, 0444);
MODULE_PARM_DESC(probe_enum, "Bitmask of RCs to enumerate at probe even when their endpoint driver is expected to do it");

static u32 kport_apb_readl(struct kport_pcie *k, u32 reg)
{
	return readl(k->apb + reg);
}

static void kport_apb_writel(struct kport_pcie *k, u32 val, u32 reg)
{
	writel(val, k->apb + reg);
}

static void kport_apb_rmw(struct kport_pcie *k, u32 reg, u32 clr, u32 set)
{
	kport_apb_writel(k, (kport_apb_readl(k, reg) & ~clr) | set, reg);
}

static u32 kport_phy_readl(struct kport_pcie *k, u32 reg)
{
	return readl(k->phy + k->phy_apb_off + reg);
}

static void kport_phy_writel(struct kport_pcie *k, u32 val, u32 reg)
{
	writel(val, k->phy + k->phy_apb_off + reg);
}

static void kport_phy_rmw(struct kport_pcie *k, u32 reg, u32 clr, u32 set)
{
	kport_phy_writel(k, (kport_phy_readl(k, reg) & ~clr) | set, reg);
}

static u32 kport_phy_cr_read(struct kport_pcie *k, u32 reg)
{
	return readl(k->phy + k->phy_natural_off + reg * 4);
}

static void kport_phy_cr_write(struct kport_pcie *k, u32 val, u32 reg)
{
	writel(val, k->phy + k->phy_natural_off + reg * 4);
}

static u32 kport_hsdt_bits(struct kport_pcie *k, u32 bits)
{
	return bits << (12 * k->rc_id);
}

/* ---- clocks ---- */

static int kport_clk_enable(struct kport_pcie *k, enum kport_clk a, enum kport_clk b)
{
	int ret;

	ret = clk_prepare_enable(k->clks[a]);
	if (ret)
		return ret;
	ret = clk_prepare_enable(k->clks[b]);
	if (ret)
		clk_disable_unprepare(k->clks[a]);
	return ret;
}

static void kport_clk_disable(struct kport_pcie *k, enum kport_clk a, enum kport_clk b)
{
	clk_disable_unprepare(k->clks[b]);
	clk_disable_unprepare(k->clks[a]);
}

/* ---- reference clock: FNPLL -> PHY ref and PCIe IO (refclk pad) ---- */

static int kport_pll_init(struct kport_pcie *k)
{
	struct regmap *h = k->hsdt;
	u32 val;
	int ret;

	/* disable and bypass while reprogramming */
	regmap_update_bits(h, HSDT_PLL_CFG(6), FNPLL_EN | FNPLL_BYPASS, FNPLL_BYPASS);

	if (k->chip_type == CHIP_TYPE_CS2) {
		regmap_write(h, HSDT_PLL_CFG(0), 0x00000000);
		regmap_write(h, HSDT_PLL_CFG(1), 0x00b50000);
		regmap_write(h, HSDT_PLL_CFG(2), 0x20101fa0);
		regmap_write(h, HSDT_PLL_CFG(3), 0x2404ff20);
		regmap_write(h, HSDT_PLL_CFG(4), 0x0034013f);
		regmap_write(h, HSDT_PLL_CFG(5), 0x00000046);
		/* 3.0 GHz VCO -> 100 MHz: dll_en, fout4phasepd, postdiv2 5, postdiv1 6, fbdiv 0x4e, refdiv 1 */
		regmap_write(h, HSDT_PLL_CFG(6), BIT(29) | BIT(28) | (5 << 23) | (6 << 20) |
			     (0x4e << 8) | (1 << 2) | FNPLL_BYPASS);
		regmap_update_bits(h, HSDT_PLL_CFG(7), GENMASK(23, 0), 0x200000);
	} else {
		regmap_write(h, HSDT_PLL_CFG(0), 0x00042000);
		regmap_write(h, HSDT_PLL_CFG(1), 0x000000b5);
		regmap_write(h, HSDT_PLL_CFG(2), 0x1fa02010);
		regmap_write(h, HSDT_PLL_CFG(3), 0xff202404);
		regmap_write(h, HSDT_PLL_CFG(4), 0x013f0004);
		/* 2.4 GHz VCO -> 100 MHz: dll_en, fout4phasepd, fout2xpd, postdiv2 4, postdiv1 6, fbdiv 0x3e, refdiv 1 */
		regmap_write(h, HSDT_PLL_CFG(6), BIT(29) | BIT(28) | BIT(27) | (4 << 23) |
			     (6 << 20) | (0x3e << 8) | (1 << 2) | FNPLL_BYPASS);
		regmap_update_bits(h, HSDT_PLL_CFG(7), GENMASK(23, 0), 0x800000);
	}
	udelay(5);

	regmap_update_bits(h, HSDT_PLL_CFG(6), FNPLL_EN, FNPLL_EN);
	ret = regmap_read_poll_timeout_atomic(h, HSDT_PCIEPLL_STATE, val,
					      val & FNPLL_LOCK, 1, 200);
	if (ret) {
		dev_err(k->pci.dev, "FNPLL does not lock (state %#x)\n", val);
		return ret;
	}
	regmap_update_bits(h, HSDT_PLL_CFG(6), FNPLL_BYPASS, 0);
	return 0;
}

static void kport_pll_shutdown(struct kport_pcie *k)
{
	regmap_update_bits(k->hsdt, HSDT_PLL_CFG(6), FNPLL_EN, 0);
	regmap_update_bits(k->hsdt, HSDT_PLL_CFG(6), FNPLL_BYPASS, FNPLL_BYPASS);
}

/*
 * PHY ref and IO (refclk output) clocks are left to hardware control by
 * CLKREQ#; their software gates stay off. RC mode: IO output enable follows
 * the controller, input disabled.
 */
static void kport_refclk_route(struct kport_pcie *k, bool enable)
{
	u32 clr, set;

	clr = PCIEIO_HW_BYPASS | PCIEIO_OE_POLAR | PCIEIO_OE_EN_SOFT |
	      PCIEIO_IE_POLAR | PCIEIO_IE_EN_SOFT;
	set = PCIEIO_IE_EN_HARD_BYPASS;
	if (enable)
		clr |= PCIEPHY_REF_HW_BYPASS | PCIEIO_OE_EN_HARD_BYPASS;
	else
		set |= PCIEPHY_REF_HW_BYPASS | PCIEIO_OE_EN_HARD_BYPASS;
	regmap_update_bits(k->hsdt, HSDT_PCIECTRL(k->rc_id), clr | set, set);
	regmap_write(k->hsdt, HSDT_PERDIS1, kport_hsdt_bits(k, HSDT_GT_PHYREF | HSDT_GT_IO));
}

static int kport_refclk_on(struct kport_pcie *k)
{
	int ret = 0;

	kport_phy_rmw(k, PHY_CTRL1, PHY_CTRL1_REF_USE_PAD, 0);
	kport_phy_rmw(k, PHY_CTRL0, PHY_CTRL0_REF_USE_CIO_PAD, 0);

	mutex_lock(&kport_pll_lock);
	if (!kport_pll_users)
		ret = kport_pll_init(k);
	if (!ret)
		kport_pll_users++;
	mutex_unlock(&kport_pll_lock);
	if (ret)
		return ret;

	regmap_write(k->hsdt, HSDT_PEREN1, kport_hsdt_bits(k, HSDT_GT_HP | HSDT_GT_DEBOUNCE));
	kport_apb_rmw(k, CTRL21, CTRL21_IO_CLK_SEL, 0);
	kport_refclk_route(k, true);
	return 0;
}

static void kport_refclk_off(struct kport_pcie *k)
{
	regmap_write(k->hsdt, HSDT_PERDIS1, kport_hsdt_bits(k, HSDT_GT_HP | HSDT_GT_DEBOUNCE));
	kport_refclk_route(k, false);

	mutex_lock(&kport_pll_lock);
	if (kport_pll_users && !--kport_pll_users)
		kport_pll_shutdown(k);
	mutex_unlock(&kport_pll_lock);
}

/* ---- PHY ---- */

static void kport_phy_eye_param(struct kport_pcie *k)
{
	u32 i, reg, mask, val;

	for (i = 0; i < k->eye_num; i++) {
		reg = k->eye[3 * i];
		mask = k->eye[3 * i + 1];
		val = k->eye[3 * i + 2];
		if (val == 0xffff)
			continue;
		kport_phy_cr_write(k, (kport_phy_cr_read(k, reg) & ~mask) | val, reg);
	}
}

static int kport_phy_init(struct kport_pcie *k)
{
	u32 val, i;
	int ret;

	/* SRAM bypass off when the PHY needs the ECO firmware patch */
	kport_phy_rmw(k, PHY_CTRL40, PHY_CTRL40_SRAM_BYPASS,
		      k->eco ? 0 : PHY_CTRL40_SRAM_BYPASS);
	kport_phy_rmw(k, PHY_CTRL0, PHY_CTRL0_TEST_POWERDOWN, 0);

	if (k->eco) {
		kport_phy_rmw(k, PHY_CTRL1, PHY_CTRL1_PHY_RESET_SEL, PHY_CTRL1_PHY_RESET);
		udelay(10);
		kport_phy_rmw(k, PHY_CTRL1, PHY_CTRL1_PHY_RESET, 0);
	}

	/* release the controller's perst_n */
	kport_apb_rmw(k, CTRL12, 0, CTRL12_PERST_IN_RC);
	udelay(10);

	if (!k->eco)
		return 0;

	ret = readl_poll_timeout_atomic(k->phy + k->phy_apb_off + PHY_STATE39, val,
					val & PHY_STATE39_SRAM_INIT_DONE, 100, 1000);
	if (ret) {
		dev_err(k->pci.dev, "PHY SRAM init not done\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(kport_phy_fw_apr); i++)
		writel(kport_phy_fw_apr[i], k->phy + k->phy_sram_off + i * 4);

	val = kport_phy_cr_read(k, PHY_SUP_DIG_LVL_OVRD_IN);
	kport_phy_cr_write(k, (val & ~0xffff) | PHY_SUP_DIG_LVL_VBOOST, PHY_SUP_DIG_LVL_OVRD_IN);
	kport_phy_rmw(k, PHY_CTRL150, 0, PHY_CTRL150_CDR_LEGACY_EN);

	kport_phy_rmw(k, PHY_CTRL40, 0, PHY_CTRL40_SRAM_EXT_LD_DONE);
	return 0;
}

/* ---- NoC power domain ---- */

static int kport_noc_power(struct kport_pcie *k, bool on)
{
	u32 bit = PMC_NOC_PCIE(k->rc_id), val;
	int ret;

	/* bit 16+n is the write mask for bit n; bit n set requests idle */
	regmap_write(k->pmctrl, PMC_NOC_POWER_IDLEREQ, (bit << 16) | (on ? 0 : bit));
	ret = regmap_read_poll_timeout_atomic(k->pmctrl, PMC_NOC_POWER_IDLE, val,
					      on ? !(val & bit) : (val & bit), 1, 1000);
	if (ret)
		dev_err(k->pci.dev, "NoC does not %s idle (%#x)\n", on ? "leave" : "enter", val);
	return ret;
}

/* ---- endpoint power and PERST# ---- */

static void kport_ep_callbacks(struct kport_pcie *k, int (**on)(void *data),
			       int (**off)(void *data), void **data)
{
	spin_lock(&k->ep_lock);
	*on = k->ep_poweron;
	*off = k->ep_poweroff;
	*data = k->ep_data;
	spin_unlock(&k->ep_lock);
}

static int kport_ep_power_on(struct kport_pcie *k)
{
	struct device *dev = k->pci.dev;
	int (*on)(void *data), (*off)(void *data);
	void *data;
	int ret;

	if (k->vpcie) {
		ret = regulator_enable(k->vpcie);
		if (ret) {
			dev_err(dev, "cannot enable vpcie3v3: %d\n", ret);
			return ret;
		}
	}

	kport_ep_callbacks(k, &on, &off, &data);
	if (on) {
		ret = on(data);
		if (ret) {
			dev_err(dev, "endpoint power-on callback failed: %d\n", ret);
			goto err_reg;
		}
	}

	/* PERST# high (released) */
	usleep_range(k->t_ref2perst[0], k->t_ref2perst[1]);
	gpiod_set_value_cansleep(k->perst, 1);
	usleep_range(k->t_perst2access[0], k->t_perst2access[1]);
	return 0;

err_reg:
	if (k->vpcie)
		regulator_disable(k->vpcie);
	return ret;
}

static void kport_ep_power_off(struct kport_pcie *k)
{
	int (*on)(void *data), (*off)(void *data);
	void *data;

	/* PERST# low (reset) */
	gpiod_set_value_cansleep(k->perst, 0);
	usleep_range(k->t_perst2rst[0], k->t_perst2rst[1]);

	kport_ep_callbacks(k, &on, &off, &data);
	if (off && off(data))
		dev_err(k->pci.dev, "endpoint power-off callback failed\n");
	if (k->vpcie)
		regulator_disable(k->vpcie);
}

/* ---- controller power ---- */

static void kport_iso(struct kport_pcie *k, bool isolate)
{
	/* ISOEN at iso[0], ISODIS at iso[0] + 4 */
	regmap_write(k->sctrl, k->iso[0] + (isolate ? 0 : 4), k->iso[1]);
}

static void kport_reset(struct kport_pcie *k, bool assert)
{
	/* PERRSTEN at rst[0], PERRSTDIS at rst[0] + 4 */
	regmap_write(k->hsdt, k->rst[0] + (assert ? 0 : 4), k->rst[1]);
}

/* sys_aux_pwr_det, RC mode, PERST# output driven low, PHY resets to hardware */
static void kport_natural_cfg(struct kport_pcie *k)
{
	kport_apb_rmw(k, CTRL22, CTRL22_CLKREQ_OUT, 0);
	kport_apb_rmw(k, CTRL7, 0, CTRL7_AUX_PWR_DET);
	kport_apb_rmw(k, CTRL0, CTRL0_DEV_TYPE,
		      FIELD_PREP(CTRL0_DEV_TYPE, CTRL0_DEV_TYPE_RC));
	kport_apb_rmw(k, CTRL12, CTRL12_PERST_IN | CTRL12_PERST_OUT, CTRL12_PERST_OE);
	kport_phy_rmw(k, PHY_CTRL1, PHY_CTRL1_LANE0_RESET, PHY_CTRL1_PHY_RESET_SEL);
}

static int kport_power_on(struct kport_pcie *k)
{
	struct dw_pcie *pci = &k->pci;
	u32 val;
	int ret;

	if (k->powered)
		return 0;

	kport_iso(k, false);
	kport_reset(k, true);
	ret = kport_clk_enable(k, KPORT_CLK_APB_PHY, KPORT_CLK_APB_SYS);
	if (ret) {
		dev_err(pci->dev, "cannot enable APB clocks: %d\n", ret);
		goto err_iso;
	}
	kport_reset(k, false);

	kport_apb_rmw(k, CTRL25, CTRL25_AXI_TIMEOUT,
		      FIELD_PREP(CTRL25_AXI_TIMEOUT, CTRL25_AXI_TIMEOUT_48MS));
	kport_natural_cfg(k);

	ret = kport_refclk_on(k);
	if (ret)
		goto err_rst;

	ret = kport_clk_enable(k, KPORT_CLK_ACLK, KPORT_CLK_AUX);
	if (ret) {
		dev_err(pci->dev, "cannot enable AXI/AUX clocks: %d\n", ret);
		goto err_refclk;
	}

	ret = kport_phy_init(k);
	if (ret)
		goto err_clk;

	ret = kport_ep_power_on(k);
	if (ret)
		goto err_clk;

	ret = readl_poll_timeout(k->phy + k->phy_apb_off + PHY_STATE0, val,
				 !(val & PHY_STATE0_PIPE_CLK_UNSTABLE), 1000, 100000);
	if (ret) {
		dev_err(pci->dev, "PIPE clock not stable (PHY state0 %#x)\n", val);
		goto err_ep;
	}

	kport_phy_eye_param(k);

	ret = kport_noc_power(k, true);
	if (ret)
		goto err_ep;

	WRITE_ONCE(k->powered, true);
	k->ep_link_status = PCIE_KPORT_DEVICE_LINK_UP;

	/* L1ss exit fix from the vendor code */
	val = dw_pcie_readl_dbi(pci, PORT_GEN3_RELATED_OFF);
	dw_pcie_writel_dbi(pci, PORT_GEN3_RELATED_OFF, val & ~GEN3_ZRXDC_NONCOMPL);
	return 0;

err_ep:
	kport_ep_power_off(k);
err_clk:
	kport_clk_disable(k, KPORT_CLK_ACLK, KPORT_CLK_AUX);
err_refclk:
	kport_refclk_off(k);
err_rst:
	kport_reset(k, true);
	kport_clk_disable(k, KPORT_CLK_APB_PHY, KPORT_CLK_APB_SYS);
err_iso:
	kport_iso(k, true);
	return ret;
}

static void kport_power_off(struct kport_pcie *k)
{
	if (!k->powered)
		return;

	WRITE_ONCE(k->powered, false);

	kport_apb_rmw(k, CTRL10, 0, CTRL10_AXI_TIMEOUT_MASK);
	kport_noc_power(k, false);
	kport_ep_power_off(k);

	kport_apb_rmw(k, CTRL12, CTRL12_PERST_IN_RC, 0);
	/* close the RX signal detectors */
	kport_phy_cr_write(k, (kport_phy_cr_read(k, PHY_RAWAONLANEN_DIG_RX_OVRD_OUT_3) & ~0x3f) |
			   PHY_RX_SIGDET_OFF, PHY_RAWAONLANEN_DIG_RX_OVRD_OUT_3);
	kport_phy_rmw(k, PHY_CTRL0, 0, PHY_CTRL0_TEST_POWERDOWN);

	kport_clk_disable(k, KPORT_CLK_ACLK, KPORT_CLK_AUX);
	kport_refclk_off(k);
	kport_apb_rmw(k, CTRL22, 0, CTRL22_CLKREQ_OUT);
	kport_clk_disable(k, KPORT_CLK_APB_PHY, KPORT_CLK_APB_SYS);
	kport_iso(k, true);
}

/* ---- DBI and config space ---- */

static u32 kport_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size)
{
	struct kport_pcie *k = to_kport(pci);
	unsigned long flags;
	u32 val;

	if (!READ_ONCE(k->powered))
		return ~0U;

	raw_spin_lock_irqsave(&k->dbi_lock, flags);
	kport_apb_rmw(k, CTRL1, 0, CTRLx_SLV_DBI_EN);
	dw_pcie_read(base + reg, size, &val);
	kport_apb_rmw(k, CTRL1, CTRLx_SLV_DBI_EN, 0);
	raw_spin_unlock_irqrestore(&k->dbi_lock, flags);
	return val;
}

static void kport_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg,
			    size_t size, u32 val)
{
	struct kport_pcie *k = to_kport(pci);
	unsigned long flags;

	if (!READ_ONCE(k->powered))
		return;

	raw_spin_lock_irqsave(&k->dbi_lock, flags);
	kport_apb_rmw(k, CTRL0, 0, CTRLx_SLV_DBI_EN);
	dw_pcie_write(base + reg, size, val);
	kport_apb_rmw(k, CTRL0, CTRLx_SLV_DBI_EN, 0);
	raw_spin_unlock_irqrestore(&k->dbi_lock, flags);
}

static int kport_root_read(struct pci_bus *bus, unsigned int devfn, int where,
			   int size, u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(bus->sysdata);

	if (PCI_SLOT(devfn) || !READ_ONCE(to_kport(pci)->powered)) {
		PCI_SET_ERROR_RESPONSE(val);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	*val = dw_pcie_read_dbi(pci, where, size);
	return PCIBIOS_SUCCESSFUL;
}

static int kport_root_write(struct pci_bus *bus, unsigned int devfn, int where,
			    int size, u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(bus->sysdata);

	if (PCI_SLOT(devfn) || !READ_ONCE(to_kport(pci)->powered))
		return PCIBIOS_DEVICE_NOT_FOUND;

	dw_pcie_write_dbi(pci, where, size, val);
	return PCIBIOS_SUCCESSFUL;
}

/*
 * Child config TLPs go through the same AXI window as DBI accesses, so they
 * must not overlap with a DBI access on another CPU that has the sideband
 * select set (e.g. the MSI handler). The DesignWare map_bus programs the
 * iATU through our DBI accessors; only the access itself is serialized.
 */
static int kport_child_read(struct pci_bus *bus, unsigned int devfn, int where,
			    int size, u32 *val)
{
	struct kport_pcie *k = to_kport(to_dw_pcie_from_pp(bus->sysdata));
	unsigned long flags;
	void __iomem *addr;

	addr = k->child_map_bus(bus, devfn, where);
	if (!addr) {
		PCI_SET_ERROR_RESPONSE(val);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	raw_spin_lock_irqsave(&k->dbi_lock, flags);
	if (size == 1)
		*val = readb(addr);
	else if (size == 2)
		*val = readw(addr);
	else
		*val = readl(addr);
	raw_spin_unlock_irqrestore(&k->dbi_lock, flags);
	return PCIBIOS_SUCCESSFUL;
}

static int kport_child_write(struct pci_bus *bus, unsigned int devfn, int where,
			     int size, u32 val)
{
	struct kport_pcie *k = to_kport(to_dw_pcie_from_pp(bus->sysdata));
	unsigned long flags;
	void __iomem *addr;

	addr = k->child_map_bus(bus, devfn, where);
	if (!addr)
		return PCIBIOS_DEVICE_NOT_FOUND;

	raw_spin_lock_irqsave(&k->dbi_lock, flags);
	if (size == 1)
		writeb(val, addr);
	else if (size == 2)
		writew(val, addr);
	else
		writel(val, addr);
	raw_spin_unlock_irqrestore(&k->dbi_lock, flags);
	return PCIBIOS_SUCCESSFUL;
}

static void kport_set_child_ops(struct kport_pcie *k, struct pci_bus *bus)
{
	struct pci_bus *child;

	list_for_each_entry(child, &bus->children, node) {
		if (child->ops != &k->child_ops)
			pci_bus_set_ops(child, &k->child_ops);
		kport_set_child_ops(k, child);
	}
}

/* ---- DesignWare glue ---- */

static bool kport_link_up(struct dw_pcie *pci)
{
	struct kport_pcie *k = to_kport(pci);

	if (!READ_ONCE(k->powered) || k->ep_link_status != PCIE_KPORT_DEVICE_LINK_UP)
		return false;

	return (kport_apb_readl(k, STATE0) & STATE0_LINK_UP) == STATE0_LINK_UP;
}

static enum dw_pcie_ltssm kport_get_ltssm(struct dw_pcie *pci)
{
	struct kport_pcie *k = to_kport(pci);

	if (!READ_ONCE(k->powered))
		return DW_PCIE_LTSSM_DETECT_QUIET;

	return FIELD_GET(STATE4_LTSSM, kport_apb_readl(k, STATE4));
}

static int kport_start_link(struct dw_pcie *pci)
{
	/* same value as the vendor driver, which also drops sys_aux_pwr_det here */
	kport_apb_writel(to_kport(pci), CTRL7_LTSSM_EN, CTRL7);
	return 0;
}

static void kport_stop_link(struct dw_pcie *pci)
{
	struct kport_pcie *k = to_kport(pci);

	if (READ_ONCE(k->powered))
		kport_apb_rmw(k, CTRL7, CTRL7_LTSSM_EN, 0);
}

/* RC-specific DBI setup after every power up */
static void kport_rc_fixup(struct kport_pcie *k)
{
	struct dw_pcie *pci = &k->pci;
	u8 cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 val;

	dw_pcie_dbi_ro_wr_en(pci);

	/* both RCs report 19e5:3690; the vendor driver makes RC1 3691 */
	if (!k->device_id)
		k->device_id = dw_pcie_readw_dbi(pci, PCI_DEVICE_ID) + k->rc_id;
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, k->device_id);

	/* advertise only the ASPM states the board supports (DT aspm_state) */
	if (cap) {
		val = dw_pcie_readl_dbi(pci, cap + PCI_EXP_LNKCAP);
		val = u32_replace_bits(val, FIELD_GET(PCI_EXP_LNKCAP_ASPMS, val) & k->aspm_state,
				       PCI_EXP_LNKCAP_ASPMS);
		dw_pcie_writel_dbi(pci, cap + PCI_EXP_LNKCAP, val);
	}

	dw_pcie_dbi_ro_wr_dis(pci);
}

/* power up and prepare the RC for dw_pcie_setup_rc(); called with k->lock held */
static int __kport_host_init(struct kport_pcie *k)
{
	struct dw_pcie_rp *pp = &k->pci.pp;
	int ret;

	/* root bus: DBI through the sideband; children: serialized with DBI */
	pp->bridge->ops = &k->root_ops;
	if (!k->child_map_bus) {
		k->child_map_bus = pp->bridge->child_ops->map_bus;
		k->child_ops = *pp->bridge->child_ops;
		k->child_ops.read = kport_child_read;
		k->child_ops.write = kport_child_write;
	}

	ret = kport_power_on(k);
	if (ret)
		return ret;

	kport_rc_fixup(k);
	return 0;
}

/* DesignWare host ops: dw_pcie_host_init() and dw_pcie_{suspend,resume}_noirq() */
static int kport_host_init(struct dw_pcie_rp *pp)
{
	struct kport_pcie *k = to_kport(to_dw_pcie_from_pp(pp));
	int ret;

	mutex_lock(&k->lock);
	ret = __kport_host_init(k);
	mutex_unlock(&k->lock);
	return ret;
}

static void kport_host_deinit(struct dw_pcie_rp *pp)
{
	struct kport_pcie *k = to_kport(to_dw_pcie_from_pp(pp));

	mutex_lock(&k->lock);
	kport_power_off(k);
	mutex_unlock(&k->lock);
}

/* Broadcast PME_Turn_Off through a MSG iATU window and wait for the ack */
static void kport_pme_turn_off(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct kport_pcie *k = to_kport(pci);
	struct dw_pcie_ob_atu_cfg atu = { 0 };
	void __iomem *mem;
	u32 val;

	if (!pp->msg_res || pci->num_ob_windows <= pp->msg_atu_index) {
		dev_warn(pci->dev, "no iATU window for PME_Turn_Off\n");
		return;
	}

	atu.code = PCIE_MSG_CODE_PME_TURN_OFF;
	atu.routing = PCIE_MSG_TYPE_R_BC;
	atu.type = PCIE_ATU_TYPE_MSG;
	atu.size = resource_size(pp->msg_res);
	atu.index = pp->msg_atu_index;
	atu.parent_bus_addr = pp->msg_res->start - pci->parent_bus_offset;
	if (dw_pcie_prog_outbound_atu(pci, &atu))
		return;

	mem = ioremap(pp->msg_res->start, pci->region_align);
	if (!mem)
		return;
	writel(0, mem);
	iounmap(mem);

	if (readl_poll_timeout(k->apb + STATE1, val, val & STATE1_PME_ACK, 10, 10000))
		dev_warn(pci->dev, "no PME_TO_Ack\n");
}

static const struct dw_pcie_ops kport_dw_ops = {
	.read_dbi	= kport_read_dbi,
	.write_dbi	= kport_write_dbi,
	.link_up	= kport_link_up,
	.get_ltssm	= kport_get_ltssm,
	.start_link	= kport_start_link,
	.stop_link	= kport_stop_link,
};

static const struct dw_pcie_host_ops kport_host_ops = {
	.init		= kport_host_init,
	.deinit		= kport_host_deinit,
	.pme_turn_off	= kport_pme_turn_off,
};

/* ---- link management shared by enumeration, the EP API and PM ---- */

/* called with k->lock held */
static int kport_link_resume(struct kport_pcie *k)
{
	struct dw_pcie *pci = &k->pci;
	int ret;

	ret = __kport_host_init(k);
	if (ret)
		return ret;

	ret = dw_pcie_setup_rc(&pci->pp);
	if (ret)
		goto err;
	kport_start_link(pci);
	ret = dw_pcie_wait_for_link(pci);
	if (ret)
		goto err;
	return 0;

err:
	kport_stop_link(pci);
	kport_power_off(k);
	return ret;
}

static struct pci_dev *kport_root_port(struct kport_pcie *k)
{
	struct pci_host_bridge *bridge = k->pci.pp.bridge;

	if (!k->root_port && bridge && bridge->bus)
		k->root_port = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	return k->root_port;
}

static struct pci_dev *kport_first_ep(struct kport_pcie *k)
{
	struct pci_dev *rp = kport_root_port(k);
	struct pci_bus *bus = rp ? rp->subordinate : NULL;

	if (!bus || list_empty(&bus->devices))
		return NULL;
	return list_first_entry(&bus->devices, struct pci_dev, bus_list);
}

static void kport_aspm(struct kport_pcie *k, bool enable)
{
	struct pci_dev *ep = kport_first_ep(k);
	u16 aspmc = enable ? k->aspm_state : 0;

	if (!ep)
		return;

	/* upstream first when enabling, downstream first when disabling */
	if (enable)
		pcie_capability_clear_and_set_word(k->root_port, PCI_EXP_LNKCTL,
						   PCI_EXP_LNKCTL_ASPMC, aspmc);
	pcie_capability_clear_and_set_word(ep, PCI_EXP_LNKCTL, PCI_EXP_LNKCTL_ASPMC, aspmc);
	if (!enable)
		pcie_capability_clear_and_set_word(k->root_port, PCI_EXP_LNKCTL,
						   PCI_EXP_LNKCTL_ASPMC, aspmc);
}

static void kport_save_rp(struct kport_pcie *k)
{
	if (!k->root_port)
		return;
	pci_save_state(k->root_port);
	kfree(k->rp_state);
	k->rp_state = pci_store_saved_state(k->root_port);
}

static void kport_restore_rp(struct kport_pcie *k)
{
	if (!k->root_port || !k->rp_state)
		return;
	pci_load_saved_state(k->root_port, k->rp_state);
	pci_restore_state(k->root_port);
}

/*
 * Called with enum_lock held. k->lock is only taken around power changes:
 * endpoint drivers probed from pci_host_probe()/pci_rescan_bus() may call
 * back into the API.
 */
static int kport_enumerate(struct kport_pcie *k)
{
	struct dw_pcie_rp *pp = &k->pci.pp;
	struct device *dev = k->pci.dev;
	struct pci_dev *ep;
	int ret;

	if (k->enumerated)
		return 0;

	if (!k->bridge_up) {
		ret = dw_pcie_host_init(pp);
		if (ret)
			return dev_err_probe(dev, ret, "host init failed\n");
		k->bridge_up = true;
		kport_root_port(k);
	} else {
		mutex_lock(&k->lock);
		ret = kport_link_resume(k);
		if (!ret)
			kport_restore_rp(k);
		mutex_unlock(&k->lock);
		if (ret)
			return ret;
		pci_lock_rescan_remove();
		pci_rescan_bus(pp->bridge->bus);
		pci_unlock_rescan_remove();
	}

	pci_lock_rescan_remove();
	kport_set_child_ops(k, pp->bridge->bus);
	pci_unlock_rescan_remove();

	mutex_lock(&k->lock);
	ep = kport_first_ep(k);
	if (!ep) {
		dev_err(dev, "no endpoint found (LTSSM %#x)\n",
			kport_get_ltssm(&k->pci));
		kport_save_rp(k);
		kport_stop_link(&k->pci);
		kport_power_off(k);
		mutex_unlock(&k->lock);
		return -ENODEV;
	}

	dev_info(dev, "RC%u: %s [%04x:%04x]\n", k->rc_id, pci_name(ep),
		 ep->vendor, ep->device);
	kport_save_rp(k);
	k->enumerated = true;
	k->usr_suspended = false;
	mutex_unlock(&k->lock);
	return 0;
}

/* ---- events for endpoint drivers ---- */

/*
 * Runs from the event works with no lock held, so the endpoint's callback
 * may call back into this API. pcie_kport_deregister_event() clears
 * event_reg before flushing the works.
 */
static void kport_notify(struct kport_pcie *k, enum pcie_kport_event event)
{
	struct pcie_kport_register_event *reg;

	spin_lock(&k->ep_lock);
	reg = k->event_reg;
	spin_unlock(&k->ep_lock);

	if (!reg || !(reg->events & event))
		return;

	reg->notify.event = event;
	reg->notify.user = reg->user;
	if (reg->mode == PCIE_KPORT_TRIGGER_CALLBACK && reg->callback)
		reg->callback(&reg->notify);
	else if (reg->mode == PCIE_KPORT_TRIGGER_COMPLETION && reg->completion)
		complete(reg->completion);
}

static void kport_linkdown_work(struct work_struct *work)
{
	struct kport_pcie *k = container_of(work, struct kport_pcie, linkdown_work);

	dev_err(k->pci.dev, "link down (LTSSM %#x)\n", kport_get_ltssm(&k->pci));
	kport_notify(k, PCIE_KPORT_EVENT_LINKDOWN);
}

static void kport_cpltimeout_work(struct work_struct *work)
{
	struct kport_pcie *k = container_of(work, struct kport_pcie, cpltimeout_work);

	dev_err(k->pci.dev, "completion timeout\n");
	kport_notify(k, PCIE_KPORT_EVENT_CPL_TIMEOUT);
}

static irqreturn_t kport_linkdown_irq(int irq, void *data)
{
	struct kport_pcie *k = data;

	if (READ_ONCE(k->powered))
		schedule_work(&k->linkdown_work);
	return IRQ_HANDLED;
}

static irqreturn_t kport_cpltimeout_irq(int irq, void *data)
{
	struct kport_pcie *k = data;

	if (READ_ONCE(k->powered)) {
		kport_apb_rmw(k, CTRL11, 0, CTRL11_AXI_TIMEOUT_CLR);
		schedule_work(&k->cpltimeout_work);
	}
	return IRQ_HANDLED;
}

/* ---- API for endpoint drivers (linux/platform_drivers/pcie-kport-api.h) ---- */

static struct kport_pcie *kport_get(u32 rc_idx)
{
	struct kport_pcie *k = NULL;

	mutex_lock(&kport_rcs_lock);
	if (rc_idx < KPORT_MAX_RC)
		k = kport_rcs[rc_idx];
	mutex_unlock(&kport_rcs_lock);
	if (!k)
		pr_err("pcie-kport: RC%u not available\n", rc_idx);
	return k;
}

static struct kport_pcie *kport_from_pdev(struct pci_dev *pdev)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);
	int i;

	for (i = 0; i < KPORT_MAX_RC; i++)
		if (kport_rcs[i] && kport_rcs[i]->pci.pp.bridge == bridge)
			return kport_rcs[i];
	return NULL;
}

int pcie_kport_enumerate(u32 rc_idx)
{
	struct kport_pcie *k = kport_get(rc_idx);
	int ret;

	if (!k)
		return -ENODEV;

	mutex_lock(&k->enum_lock);
	ret = kport_enumerate(k);
	mutex_unlock(&k->enum_lock);
	if (!ret) {
		mutex_lock(&k->lock);
		if (k->powered)
			kport_aspm(k, true);
		mutex_unlock(&k->lock);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(pcie_kport_enumerate);

int pcie_kport_remove_ep(u32 rc_idx)
{
	struct kport_pcie *k = kport_get(rc_idx);
	struct pci_dev *dev, *tmp;

	if (!k)
		return -ENODEV;

	mutex_lock(&k->enum_lock);
	if (!k->enumerated || !k->root_port->subordinate) {
		mutex_unlock(&k->enum_lock);
		return -EINVAL;
	}
	mutex_lock(&k->lock);
	if (k->powered)
		kport_aspm(k, false);
	mutex_unlock(&k->lock);
	list_for_each_entry_safe(dev, tmp, &k->root_port->subordinate->devices, bus_list)
		pci_stop_and_remove_bus_device_locked(dev);
	mutex_unlock(&k->enum_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(pcie_kport_remove_ep);

int pcie_kport_rescan_ep(u32 rc_idx)
{
	struct kport_pcie *k = kport_get(rc_idx);
	int ret = 0;

	if (!k)
		return -ENODEV;

	mutex_lock(&k->enum_lock);
	if (!k->enumerated || !READ_ONCE(k->powered)) {
		ret = -EINVAL;
		goto out;
	}
	pci_lock_rescan_remove();
	pci_rescan_bus(k->pci.pp.bridge->bus);
	kport_set_child_ops(k, k->pci.pp.bridge->bus);
	pci_unlock_rescan_remove();
	mutex_lock(&k->lock);
	if (!kport_first_ep(k))
		ret = -ENODEV;
	else if (k->powered)
		kport_aspm(k, true);
	mutex_unlock(&k->lock);
out:
	mutex_unlock(&k->enum_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pcie_kport_rescan_ep);

int pcie_kport_pm_control(int power_ops, u32 rc_idx)
{
	struct kport_pcie *k = kport_get(rc_idx);
	struct device *dev;
	int ret = 0;

	if (!k)
		return -ENODEV;
	dev = k->pci.dev;

	mutex_lock(&k->lock);
	switch (power_ops) {
	case PCIE_KPORT_POWERON:
		if (k->powered && !k->usr_suspended && kport_link_up(&k->pci))
			break;
		if (!k->bridge_up) {
			ret = -EINVAL;
			break;
		}
		kport_power_off(k);
		ret = kport_link_resume(k);
		if (ret) {
			k->usr_suspended = true;
			break;
		}
		kport_restore_rp(k);
		k->usr_suspended = false;
		kport_aspm(k, true);
		break;

	case PCIE_KPORT_POWEROFF_BUSON:
	case PCIE_KPORT_POWEROFF_BUSDOWN:
		if (k->usr_suspended || !k->powered) {
			/*
			 * hi110x powers the link down once more at the end of its boot
			 * calibration; skipping that call there breaks the boot, so
			 * only keep it out of the error log
			 */
			mutex_unlock(&k->lock);
			dev_dbg(dev, "already suspended by the endpoint\n");
			return -EINVAL;
		}
		kport_aspm(k, false);
		kport_save_rp(k);
		if (power_ops == PCIE_KPORT_POWEROFF_BUSON)
			kport_pme_turn_off(&k->pci.pp);
		/* PHY reset back to the controller */
		kport_phy_rmw(k, PHY_CTRL1, 0, PHY_CTRL1_PHY_RESET_SEL);
		kport_stop_link(&k->pci);
		kport_power_off(k);
		k->usr_suspended = true;
		break;

	case PCIE_KPORT_POWERON_CLK:
		ret = kport_power_on(k);
		if (!ret)
			kport_rc_fixup(k);
		break;

	default:
		ret = -EINVAL;
	}
	mutex_unlock(&k->lock);

	if (ret)
		dev_err(dev, "pm_control(%d) failed: %d\n", power_ops, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(pcie_kport_pm_control);

int pcie_kport_lp_ctrl(u32 rc_idx, u32 enable)
{
	struct kport_pcie *k = kport_get(rc_idx);
	int ret = 0;

	if (!k)
		return -ENODEV;

	/* also valid from the endpoint's probe, before enumeration completes */
	mutex_lock(&k->lock);
	if (!k->powered || !kport_root_port(k))
		ret = -EINVAL;
	else
		kport_aspm(k, !!enable);
	mutex_unlock(&k->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pcie_kport_lp_ctrl);

int pcie_kport_power_notifiy_register(u32 rc_id, int (*poweron)(void *data),
				      int (*poweroff)(void *data), void *data)
{
	struct kport_pcie *k = kport_get(rc_id);

	if (!k)
		return -ENODEV;

	spin_lock(&k->ep_lock);
	k->ep_poweron = poweron;
	k->ep_poweroff = poweroff;
	k->ep_data = data;
	spin_unlock(&k->ep_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(pcie_kport_power_notifiy_register);

int pcie_kport_register_event(struct pcie_kport_register_event *reg)
{
	struct kport_pcie *k;

	if (!reg || !reg->user)
		return -EINVAL;

	k = kport_from_pdev(reg->user);
	if (!k)
		return -ENODEV;

	spin_lock(&k->ep_lock);
	k->event_reg = reg;
	spin_unlock(&k->ep_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(pcie_kport_register_event);

int pcie_kport_deregister_event(struct pcie_kport_register_event *reg)
{
	struct kport_pcie *k;

	if (!reg || !reg->user)
		return -EINVAL;

	k = kport_from_pdev(reg->user);
	if (!k)
		return -ENODEV;

	spin_lock(&k->ep_lock);
	if (k->event_reg == reg)
		k->event_reg = NULL;
	spin_unlock(&k->ep_lock);
	flush_work(&k->linkdown_work);
	flush_work(&k->cpltimeout_work);
	return 0;
}
EXPORT_SYMBOL_GPL(pcie_kport_deregister_event);

int pcie_kport_ep_link_ltssm_notify(u32 rc_id, u32 link_status)
{
	struct kport_pcie *k = kport_get(rc_id);

	if (!k)
		return -ENODEV;
	if (link_status <= PCIE_KPORT_DEVICE_LINK_MIN ||
	    link_status >= PCIE_KPORT_DEVICE_LINK_MAX)
		return -EINVAL;

	WRITE_ONCE(k->ep_link_status, link_status);
	return 0;
}
EXPORT_SYMBOL_GPL(pcie_kport_ep_link_ltssm_notify);

void pcie_kport_refclk_device_vote(u32 ep_type, u32 rc_id, u32 vote)
{
	/* refclk gating by vote is not implemented; the refclk stays on while powered */
}
EXPORT_SYMBOL_GPL(pcie_kport_refclk_device_vote);

/* ---- probe ---- */

static void kport_read_pair(struct device_node *np, const char *prop, u32 *val,
			    u32 min, u32 max)
{
	if (of_property_read_u32_array(np, prop, val, 2)) {
		val[0] = min;
		val[1] = max;
	}
}

static int kport_parse_dt(struct kport_pcie *k)
{
	struct device *dev = k->pci.dev;
	struct device_node *np = dev->of_node;
	u32 layout[3];
	int ret, n;

	if (of_property_read_u32(np, "rc-id", &k->rc_id) || k->rc_id >= KPORT_MAX_RC)
		return dev_err_probe(dev, -EINVAL, "missing or bad rc-id\n");

	ret = of_property_read_u32_array(np, "phy_layout_info", layout, 3);
	if (ret)
		return dev_err_probe(dev, ret, "missing phy_layout_info\n");
	k->phy_natural_off = layout[0];
	k->phy_sram_off = layout[1];
	k->phy_apb_off = layout[2];

	ret = of_property_read_u32_array(np, "iso_info", k->iso, 2);
	if (ret)
		return dev_err_probe(dev, ret, "missing iso_info\n");
	ret = of_property_read_u32_array(np, "assert_info", k->rst, 2);
	if (ret)
		return dev_err_probe(dev, ret, "missing assert_info\n");

	kport_read_pair(np, "t_ref2perst", k->t_ref2perst, 20000, 21000);
	kport_read_pair(np, "t_perst2access", k->t_perst2access, 7000, 8000);
	kport_read_pair(np, "t_perst2rst", k->t_perst2rst, 10000, 11000);

	if (of_property_read_u32(np, "chip_type", &k->chip_type))
		k->chip_type = 2;
	of_property_read_u32(np, "eco", &k->eco);
	of_property_read_u32(np, "ep_device_type", &k->ep_type);
	if (of_property_read_u32(np, "aspm_state", &k->aspm_state))
		k->aspm_state = PCI_EXP_LNKCTL_ASPM_L1;
	k->aspm_state &= PCI_EXP_LNKCTL_ASPMC;

	if (!of_property_read_u32(np, "eye_param_nums", &k->eye_num) && k->eye_num) {
		n = of_property_count_u32_elems(np, "eye_param_details");
		if (n < 0 || (u32)n < 3 * k->eye_num)
			return dev_err_probe(dev, -EINVAL, "bad eye_param_details\n");
		k->eye = devm_kcalloc(dev, 3 * k->eye_num, sizeof(u32), GFP_KERNEL);
		if (!k->eye)
			return -ENOMEM;
		of_property_read_u32_array(np, "eye_param_details", k->eye, 3 * k->eye_num);
	}
	return 0;
}

static struct regmap *kport_syscon(struct device *dev, const char *compat)
{
	struct device_node *np;
	struct regmap *map;

	np = of_find_compatible_node(NULL, NULL, compat);
	if (!np) {
		dev_err(dev, "no %s node\n", compat);
		return ERR_PTR(-ENODEV);
	}
	map = device_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(map))
		dev_err(dev, "cannot map %s: %pe\n", compat, map);
	return map;
}

static int kport_get_resources(struct kport_pcie *k, struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = &k->pci;
	struct resource *res;
	int i;

	/*
	 * "dbi" (16 MiB) contains "config": map it without claiming the
	 * region so that the DesignWare core can claim "config".
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "missing dbi\n");
	pci->dbi_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!pci->dbi_base)
		return -ENOMEM;
	pci->dbi_phys_addr = res->start;
	/* CS2 shadow registers at +1 MiB, iATU (unrolled) at +3 MiB */
	pci->dbi_base2 = pci->dbi_base + SZ_1M;

	k->apb = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(k->apb))
		return PTR_ERR(k->apb);
	k->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(k->phy))
		return PTR_ERR(k->phy);

	k->hsdt = kport_syscon(dev, "hisilicon,hsdt-crg");
	if (IS_ERR(k->hsdt))
		return PTR_ERR(k->hsdt);
	k->sctrl = kport_syscon(dev, "hisilicon,sysctrl");
	if (IS_ERR(k->sctrl))
		return PTR_ERR(k->sctrl);
	k->pmctrl = kport_syscon(dev, "hisilicon,pmctrl");
	if (IS_ERR(k->pmctrl))
		return PTR_ERR(k->pmctrl);

	for (i = 0; i < KPORT_NUM_CLKS; i++) {
		k->clks[i] = devm_clk_get(dev, kport_clk_names[i]);
		if (IS_ERR(k->clks[i]))
			return dev_err_probe(dev, PTR_ERR(k->clks[i]),
					     "cannot get clock %s\n", kport_clk_names[i]);
	}

	/*
	 * The vendor "reset-gpio" is flagged active high but carries the
	 * PERST# line level: 0 holds the endpoint in reset, 1 releases it.
	 */
	k->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(k->perst))
		return dev_err_probe(dev, PTR_ERR(k->perst), "cannot get PERST# gpio\n");
	gpiod_set_consumer_name(k->perst, k->rc_id ? "pcie1-perst" : "pcie0-perst");

	k->vpcie = devm_regulator_get_optional(dev, "vpcie3v3");
	if (IS_ERR(k->vpcie)) {
		if (PTR_ERR(k->vpcie) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(k->vpcie), "cannot get vpcie3v3\n");
		k->vpcie = NULL;
	}
	return 0;
}

static int kport_request_irqs(struct kport_pcie *k, struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq, ret;

	/* the DesignWare MSI controller output is wired to the INTb line */
	irq = platform_get_irq_byname(pdev, "INTb");
	if (irq < 0)
		return irq;
	k->pci.pp.msi_irq[0] = irq;

	irq = platform_get_irq_byname(pdev, "link_down");
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, kport_linkdown_irq, IRQF_TRIGGER_RISING,
			       devm_kasprintf(dev, GFP_KERNEL, "pcie%u-linkdown", k->rc_id), k);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request link_down irq\n");

	irq = platform_get_irq_byname(pdev, "cpl_timeout");
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, kport_cpltimeout_irq, IRQF_TRIGGER_RISING,
			       devm_kasprintf(dev, GFP_KERNEL, "pcie%u-cpltimeout", k->rc_id), k);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request cpl_timeout irq\n");
	return 0;
}

static void kport_unregister(void *data)
{
	struct kport_pcie *k = data;

	mutex_lock(&kport_rcs_lock);
	kport_rcs[k->rc_id] = NULL;
	mutex_unlock(&kport_rcs_lock);
}

static int kport_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct kport_pcie *k;
	int ret;

	k = devm_kzalloc(dev, sizeof(*k), GFP_KERNEL);
	if (!k)
		return -ENOMEM;

	k->pci.dev = dev;
	k->pci.ops = &kport_dw_ops;
	k->pci.pp.ops = &kport_host_ops;
	k->pci.pp.use_atu_msg = true;
	/*
	 * After PME_Turn_Off the LTSSM stays in L1 (0x14) instead of reporting
	 * L2 idle; like the vendor driver, rely on PME_TO_Ack instead.
	 */
	k->pci.pp.skip_l23_ready = true;
	raw_spin_lock_init(&k->dbi_lock);
	mutex_init(&k->enum_lock);
	mutex_init(&k->lock);
	spin_lock_init(&k->ep_lock);
	INIT_WORK(&k->linkdown_work, kport_linkdown_work);
	INIT_WORK(&k->cpltimeout_work, kport_cpltimeout_work);
	k->root_ops.read = kport_root_read;
	k->root_ops.write = kport_root_write;
	platform_set_drvdata(pdev, k);

	ret = kport_parse_dt(k);
	if (ret)
		return ret;
	ret = kport_get_resources(k, pdev);
	if (ret)
		return ret;
	ret = kport_request_irqs(k, pdev);
	if (ret)
		return ret;

	mutex_lock(&kport_rcs_lock);
	if (kport_rcs[k->rc_id]) {
		mutex_unlock(&kport_rcs_lock);
		return dev_err_probe(dev, -EBUSY, "RC%u already registered\n", k->rc_id);
	}
	kport_rcs[k->rc_id] = k;
	mutex_unlock(&kport_rcs_lock);
	ret = devm_add_action_or_reset(dev, kport_unregister, k);
	if (ret)
		return ret;

	if (k->ep_type == EP_DEVICE_HI110X && !(probe_enum & BIT(k->rc_id))) {
		dev_info(dev, "RC%u: waiting for the endpoint driver to enumerate\n", k->rc_id);
		return 0;
	}

	mutex_lock(&k->enum_lock);
	ret = kport_enumerate(k);
	mutex_unlock(&k->enum_lock);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_warn(dev, "RC%u: enumeration failed (%d), powered down\n", k->rc_id, ret);
	return 0;
}

static void kport_pcie_shutdown(struct platform_device *pdev)
{
	struct kport_pcie *k = platform_get_drvdata(pdev);

	mutex_lock(&k->lock);
	if (k->powered) {
		kport_stop_link(&k->pci);
		kport_power_off(k);
	}
	mutex_unlock(&k->lock);
}

/*
 * An endpoint driver that powers its link down itself (hi110x does it from a
 * PM notifier, before the devices suspend) leaves the RC unpowered for the
 * whole system sleep.  The PCI core would then find the root port
 * inaccessible, give up waiting for its link on resume and mark every device
 * below it disconnected for good, so the endpoint's config reads return ~0
 * even after its driver powers the link up again.  Keep the PM core away from
 * that hierarchy while it is off; prepare runs parents first and complete
 * children first, so the flag spans the whole sleep.
 */
static int kport_set_syscore(struct pci_dev *pdev, void *on)
{
	dev_pm_syscore_device(&pdev->dev, *(bool *)on);
	return 0;
}

static void kport_pm_hide(struct kport_pcie *k, bool on)
{
	struct pci_dev *rp = kport_root_port(k);

	if (!rp)
		return;
	dev_pm_syscore_device(&rp->dev, on);
	if (rp->subordinate)
		pci_walk_bus(rp->subordinate, kport_set_syscore, &on);
}

static int kport_pcie_prepare(struct device *dev)
{
	struct kport_pcie *k = dev_get_drvdata(dev);

	mutex_lock(&k->enum_lock);
	mutex_lock(&k->lock);
	k->pm_hidden = k->bridge_up && !k->powered;
	if (k->pm_hidden)
		kport_pm_hide(k, true);
	mutex_unlock(&k->lock);
	mutex_unlock(&k->enum_lock);
	return 0;
}

static void kport_pcie_complete(struct device *dev)
{
	struct kport_pcie *k = dev_get_drvdata(dev);

	mutex_lock(&k->enum_lock);
	mutex_lock(&k->lock);
	if (k->pm_hidden)
		kport_pm_hide(k, false);
	k->pm_hidden = false;
	mutex_unlock(&k->lock);
	mutex_unlock(&k->enum_lock);
}

/* the DesignWare helpers power the RC through the host init/deinit ops */
static int kport_pcie_suspend_noirq(struct device *dev)
{
	struct kport_pcie *k = dev_get_drvdata(dev);
	bool active;

	mutex_lock(&k->lock);
	active = k->powered && k->enumerated && !k->usr_suspended;
	mutex_unlock(&k->lock);

	return active ? dw_pcie_suspend_noirq(&k->pci) : 0;
}

static int kport_pcie_resume_noirq(struct device *dev)
{
	struct kport_pcie *k = dev_get_drvdata(dev);

	return dw_pcie_resume_noirq(&k->pci);
}

static const struct dev_pm_ops kport_pcie_pm_ops = {
	.prepare = kport_pcie_prepare,
	.complete = kport_pcie_complete,
	NOIRQ_SYSTEM_SLEEP_PM_OPS(kport_pcie_suspend_noirq, kport_pcie_resume_noirq)
};

static const struct of_device_id kport_pcie_of_match[] = {
	{ .compatible = "pcie-kport,rc" },
	{ }
};

static struct platform_driver kport_pcie_driver = {
	.probe		= kport_pcie_probe,
	.shutdown	= kport_pcie_shutdown,
	.driver		= {
		.name			= "pcie-kport",
		.of_match_table		= kport_pcie_of_match,
		.pm			= &kport_pcie_pm_ops,
		.suppress_bind_attrs	= true,
	},
};
builtin_platform_driver(kport_pcie_driver);
