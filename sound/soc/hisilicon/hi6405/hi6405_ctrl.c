// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Hi6405 codec controller: register access over the SoC "codec SSI"
 * window, codec reset and the codec interrupt line.
 *
 * Based on the vendor hi_cdc_ctrl.c / hi_cdc_ssi.c (Huawei, GPL-2.0).
 *
 * The SSI block maps one 256-byte page of codec registers at a time into
 * the MMIO window (one codec byte per 32-bit word); the page is selected
 * through three page registers. Codec registers in [reg8_begin, reg8_end]
 * are 8 bit wide, everything else is 32 bit and goes through the codec's
 * RAM2AXI bridge.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "hi64xx/hi_cdc_ctrl.h"
#include "hi6405_drv.h"

#define SSI_PAGE_MASK		0xff
#define SSI_REG32_MASK		0xfc
#define SSI_PAGE_SEL0		0x1fd
#define SSI_PAGE_SEL1		0x1fe
#define SSI_PAGE_SEL2		0x1ff
#define HI64XX_CFG_BASE		0x20007000
#define RAM2AXI_RD_DATA0	(HI64XX_CFG_BASE + 0x23)
#define RAM2AXI_RD_DATA1	(HI64XX_CFG_BASE + 0x24)
#define RAM2AXI_RD_DATA2	(HI64XX_CFG_BASE + 0x25)
#define RAM2AXI_RD_DATA3	(HI64XX_CFG_BASE + 0x26)

#define SLIMBUS_CLK_DRV_DEFAULT	0x5
#define SLIMBUS_DATA_DRV_DEFAULT 0x5

struct hi6405_ctrl {
	/* must stay first: the child drivers use dev_get_drvdata(parent) */
	struct hi_cdc_ctrl cdc_ctrl;
	void __iomem *ssi;
	struct clk *ssi_clk;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	int irq;
	struct mutex io_lock;
	unsigned int page;
	u32 reg8_begin;
	u32 reg8_end;
};

static struct hi6405_ctrl *g_ctrl;

static inline u32 ssi_raw_read(struct hi6405_ctrl *c, unsigned int reg)
{
	return readl(c->ssi + ((reg & SSI_PAGE_MASK) << 2));
}

static inline void ssi_raw_write(struct hi6405_ctrl *c, unsigned int reg, u32 val)
{
	writel(val & SSI_PAGE_MASK, c->ssi + ((reg & SSI_PAGE_MASK) << 2));
}

static void ssi_select_page(struct hi6405_ctrl *c, unsigned int reg)
{
	unsigned int page = reg & ~SSI_PAGE_MASK;

	if (c->page == page)
		return;
	c->page = page;
	page >>= 8;
	writel(page & 0xff, c->ssi + (SSI_PAGE_SEL0 << 2));
	writel((page >> 8) & 0xff, c->ssi + (SSI_PAGE_SEL1 << 2));
	writel((page >> 16) & 0xff, c->ssi + (SSI_PAGE_SEL2 << 2));
}

/* an SSI read returns the result of the previous transfer: read twice */
static u32 ssi_read8(struct hi6405_ctrl *c, unsigned int reg)
{
	ssi_select_page(c, reg);
	ssi_raw_read(c, reg);
	return ssi_raw_read(c, reg) & 0xff;
}

static u32 ssi_read32(struct hi6405_ctrl *c, unsigned int reg)
{
	static const unsigned int data[] = {
		RAM2AXI_RD_DATA3, RAM2AXI_RD_DATA2,
		RAM2AXI_RD_DATA1, RAM2AXI_RD_DATA0,
	};
	u32 val = 0;
	int i;

	/* reading the aligned word kicks the RAM2AXI read */
	ssi_select_page(c, reg);
	ssi_raw_read(c, reg & SSI_REG32_MASK);
	ssi_raw_read(c, reg & SSI_REG32_MASK);

	ssi_select_page(c, RAM2AXI_RD_DATA0);
	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ssi_raw_read(c, data[i]);
		val = (val << 8) | (ssi_raw_read(c, data[i]) & 0xff);
	}
	return val;
}

static void ssi_write8(struct hi6405_ctrl *c, unsigned int reg, u32 val)
{
	ssi_select_page(c, reg);
	ssi_raw_write(c, reg, val);
}

static void ssi_write32(struct hi6405_ctrl *c, unsigned int reg, u32 val)
{
	if (reg & 0x3)
		return;
	ssi_select_page(c, reg);
	ssi_raw_write(c, reg, val);
	ssi_raw_write(c, reg + 1, val >> 8);
	ssi_raw_write(c, reg + 2, val >> 16);
	ssi_raw_write(c, reg + 3, val >> 24);
}

static bool is_reg8(struct hi6405_ctrl *c, unsigned int reg)
{
	return reg >= c->reg8_begin && reg <= c->reg8_end;
}

unsigned int hi_cdcctrl_reg_read(struct hi_cdc_ctrl *cdc_ctrl, unsigned int reg)
{
	struct hi6405_ctrl *c = container_of(cdc_ctrl, struct hi6405_ctrl, cdc_ctrl);
	u32 v1, v2;

	mutex_lock(&c->io_lock);
	/* the vendor driver reads twice to filter SSI frame noise */
	if (is_reg8(c, reg)) {
		v1 = ssi_read8(c, reg);
		v2 = ssi_read8(c, reg);
	} else {
		v1 = ssi_read32(c, reg);
		v2 = ssi_read32(c, reg);
	}
	mutex_unlock(&c->io_lock);

	if (v1 != v2 && cdc_ctrl->ssi_check_enable)
		dev_warn_ratelimited(cdc_ctrl->dev,
			"ssi read mismatch at %#x: %#x/%#x\n", reg, v1, v2);
	return v2;
}

int hi_cdcctrl_reg_write(struct hi_cdc_ctrl *cdc_ctrl, unsigned int reg,
	unsigned int val)
{
	struct hi6405_ctrl *c = container_of(cdc_ctrl, struct hi6405_ctrl, cdc_ctrl);

	mutex_lock(&c->io_lock);
	if (is_reg8(c, reg))
		ssi_write8(c, reg, val);
	else
		ssi_write32(c, reg, val);
	mutex_unlock(&c->io_lock);
	return 0;
}

void hi_cdcctrl_reg_update_bits(struct hi_cdc_ctrl *cdc_ctrl, unsigned int reg,
	unsigned int mask, unsigned int value)
{
	unsigned int old = hi_cdcctrl_reg_read(cdc_ctrl, reg);

	hi_cdcctrl_reg_write(cdc_ctrl, reg, (old & ~mask) | (value & mask));
}

int hi_cdcctrl_hw_reset(const struct hi_cdc_ctrl *cdc_ctrl)
{
	return 0;
}

int hi_cdcctrl_get_irq(const struct hi_cdc_ctrl *cdc_ctrl)
{
	const struct hi6405_ctrl *c =
		container_of(cdc_ctrl, struct hi6405_ctrl, cdc_ctrl);

	return c->irq;
}

int hi_cdcctrl_enable_supply(struct hi_cdc_ctrl *cdc_ctrl,
	enum hi_cdcctrl_supply sup_type, bool enable)
{
	/* codec-main / codec-anlg are not wired on the L410 (dummy upstream) */
	return 0;
}

int hi_cdcctrl_enable_clk(struct hi_cdc_ctrl *cdc_ctrl,
	enum hi_cdcctrl_clk clk_type, bool enable)
{
	struct hi6405_ctrl *c = container_of(cdc_ctrl, struct hi6405_ctrl, cdc_ctrl);

	if (clk_type != CDC_MCLK)
		return 0;
	if (enable)
		return clk_prepare_enable(c->mclk);
	clk_disable_unprepare(c->mclk);
	return 0;
}

void hi_cdcctrl_pm_get(void)
{
}

void hi_cdcctrl_pm_put(void)
{
}

void hi_cdcctrl_dump(struct hi_cdc_ctrl *cdc_ctrl)
{
}

static void hi6405_ctrl_reset(struct hi6405_ctrl *c)
{
	/*
	 * Same sequence as the vendor kernel: hold the interrupt line low while
	 * the reset is pulsed, then hand the line back to the codec.
	 */
	if (c->irq_gpio)
		gpiod_direction_output(c->irq_gpio, 0);
	gpiod_set_value_cansleep(c->reset_gpio, 1);
	mdelay(1);
	gpiod_set_value_cansleep(c->reset_gpio, 0);
	if (c->irq_gpio)
		gpiod_direction_input(c->irq_gpio);
	c->cdc_ctrl.need_reset_in_kernel = true;
}

static int hi6405_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct hi6405_ctrl *c;
	u32 val;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	mutex_init(&c->io_lock);
	c->page = 1;	/* never a page address: forces the first page select */
	c->cdc_ctrl.dev = dev;
	c->cdc_ctrl.bus_sel = BUSTYPE_SELECT_SSI;
	c->cdc_ctrl.pm_runtime_support = of_property_read_bool(np, "pm_runtime_support");
	c->cdc_ctrl.slimbusclk_cdc_drv = SLIMBUS_CLK_DRV_DEFAULT;
	c->cdc_ctrl.slimbusdata_cdc_drv = SLIMBUS_DATA_DRV_DEFAULT;
	if (!of_property_read_u32(np, "slimbusclk_io_driver", &val))
		c->cdc_ctrl.slimbusclk_cdc_drv = val;
	if (!of_property_read_u32(np, "slimbusdata_io_driver", &val))
		c->cdc_ctrl.slimbusdata_cdc_drv = val;

	ret = of_property_read_u32(np, "hisilicon,reg-8bit-begin-addr", &c->reg8_begin);
	if (ret)
		return dev_err_probe(dev, ret, "no reg-8bit-begin-addr\n");
	ret = of_property_read_u32(np, "hisilicon,reg-8bit-end-addr", &c->reg8_end);
	if (ret)
		return dev_err_probe(dev, ret, "no reg-8bit-end-addr\n");

	c->ssi = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(c->ssi))
		return PTR_ERR(c->ssi);

	c->ssi_clk = devm_clk_get_enabled(dev, "clk_codecssi");
	if (IS_ERR(c->ssi_clk))
		return dev_err_probe(dev, PTR_ERR(c->ssi_clk), "no SSI clock\n");

	/* 19.2 MHz codec master clock from the PMIC; kept on, refcounted by resmgr */
	c->mclk = devm_clk_get_enabled(dev, "clk_pmuaudioclk");
	if (IS_ERR(c->mclk))
		return dev_err_probe(dev, PTR_ERR(c->mclk), "no codec mclk\n");
	usleep_range(1000, 1100);

	/* "gpios" index 0: codec interrupt line (also a reset-time strap) */
	c->irq_gpio = devm_gpiod_get_index(dev, NULL, 0, GPIOD_IN);
	if (IS_ERR(c->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(c->irq_gpio), "no irq gpio\n");

	c->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(c->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(c->reset_gpio), "no reset gpio\n");
	if (c->reset_gpio && of_property_read_bool(np, "need_reset_in_kernel"))
		hi6405_ctrl_reset(c);

	c->irq = gpiod_to_irq(c->irq_gpio);
	if (c->irq < 0)
		dev_warn(dev, "codec irq unavailable: %d\n", c->irq);

	platform_set_drvdata(pdev, c);
	g_ctrl = c;

	val = hi_cdcctrl_reg_read(&c->cdc_ctrl, HI6405_VERSION_REG);
	dev_info(dev, "Hi6405 version %#x, chip id %02x %02x %02x %02x\n", val,
		hi_cdcctrl_reg_read(&c->cdc_ctrl, HI6405_CHIP_ID_REG0),
		hi_cdcctrl_reg_read(&c->cdc_ctrl, HI6405_CHIP_ID_REG0 + 1),
		hi_cdcctrl_reg_read(&c->cdc_ctrl, HI6405_CHIP_ID_REG0 + 2),
		hi_cdcctrl_reg_read(&c->cdc_ctrl, HI6405_CHIP_ID_REG0 + 3));
	/*
	 * No answer (0x00/0xff) usually means the codec master clock is not
	 * running yet: the PMIC that gates it may probe after us. Retry when
	 * other devices bind instead of giving up.
	 */
	if (val != HI6405_VERSION_CS)
		return dev_err_probe(dev, -EPROBE_DEFER,
			"codec version %#x, expected %#x (MCLK not running?)\n",
			val, HI6405_VERSION_CS);

	/* hi64xx_irq@0 -> hi6405_codec@0 */
	return devm_of_platform_populate(dev);
}

static void hi6405_ctrl_remove(struct platform_device *pdev)
{
	g_ctrl = NULL;
}

static const struct of_device_id hi6405_ctrl_match[] = {
	{ .compatible = "hisilicon,codec-controller", },
	{ }
};
MODULE_DEVICE_TABLE(of, hi6405_ctrl_match);

struct platform_driver hi6405_ctrl_driver = {
	.driver = {
		.name = "hi6405-ctrl",
		.of_match_table = hi6405_ctrl_match,
	},
	.probe = hi6405_ctrl_probe,
	.remove = hi6405_ctrl_remove,
};
