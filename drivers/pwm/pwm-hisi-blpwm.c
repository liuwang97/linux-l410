// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Kirin backlight PWM (BLPWM) output, as found on the Kirin 990.
 *
 * The block also has a PWM input (for content adaptive backlight on phones);
 * only the output is supported. The output counts in units of the functional
 * clock divided by (DIV + 1): CFG holds the high and low phase lengths.
 *
 * Limitations:
 * - Only normal polarity.
 * - A new setting takes effect at the end of the running period.
 * - When disabled the output is driven low.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define BLPWM_OUT_CTRL		0x100
#define  BLPWM_OUT_EN		BIT(0)
#define BLPWM_OUT_DIV		0x104
#define BLPWM_OUT_CFG		0x108
#define  BLPWM_CFG_HIGH		GENMASK(31, 16)
#define  BLPWM_CFG_LOW		GENMASK(15, 0)

#define BLPWM_MAX_DIV		0xff
#define BLPWM_MAX_COUNT		0xffff

/* clk_blpwm: clk_fll_src (180 MHz) / 2, used when the clock isn't available */
#define BLPWM_DEFAULT_RATE	90000000UL

struct hisi_blpwm {
	void __iomem *base;
	unsigned long rate;
};

static inline struct hisi_blpwm *to_hisi_blpwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int hisi_blpwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct hisi_blpwm *bl = to_hisi_blpwm(chip);
	u64 period, duty;
	u32 div, total, high;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	if (!state->enabled) {
		writel(0, bl->base + BLPWM_OUT_CTRL);
		return 0;
	}

	/* period and duty in input clock cycles */
	period = mul_u64_u64_div_u64(state->period, bl->rate, NSEC_PER_SEC);
	duty = mul_u64_u64_div_u64(state->duty_cycle, bl->rate, NSEC_PER_SEC);

	div = DIV64_U64_ROUND_UP(period, BLPWM_MAX_COUNT);
	if (div)
		div--;
	if (div > BLPWM_MAX_DIV)
		div = BLPWM_MAX_DIV;
	total = min_t(u64, div64_u64(period, div + 1), BLPWM_MAX_COUNT);
	if (total < 2)
		return -EINVAL;
	high = min_t(u64, div64_u64(duty, div + 1), total);

	writel(div, bl->base + BLPWM_OUT_DIV);
	writel(FIELD_PREP(BLPWM_CFG_HIGH, high) |
	       FIELD_PREP(BLPWM_CFG_LOW, total - high),
	       bl->base + BLPWM_OUT_CFG);
	writel(BLPWM_OUT_EN, bl->base + BLPWM_OUT_CTRL);
	return 0;
}

static int hisi_blpwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct hisi_blpwm *bl = to_hisi_blpwm(chip);
	u32 div = readl(bl->base + BLPWM_OUT_DIV) + 1;
	u32 cfg = readl(bl->base + BLPWM_OUT_CFG);
	u64 high = FIELD_GET(BLPWM_CFG_HIGH, cfg);
	u64 low = FIELD_GET(BLPWM_CFG_LOW, cfg);

	state->enabled = readl(bl->base + BLPWM_OUT_CTRL) & BLPWM_OUT_EN;
	state->polarity = PWM_POLARITY_NORMAL;
	state->period = DIV64_U64_ROUND_UP((high + low) * div * NSEC_PER_SEC, bl->rate);
	state->duty_cycle = DIV64_U64_ROUND_UP(high * div * NSEC_PER_SEC, bl->rate);
	return 0;
}

static const struct pwm_ops hisi_blpwm_ops = {
	.apply = hisi_blpwm_apply,
	.get_state = hisi_blpwm_get_state,
};

static int hisi_blpwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct hisi_blpwm *bl;
	struct clk *clk;
	int ret;

	chip = devm_pwmchip_alloc(dev, 1, sizeof(*bl));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	bl = to_hisi_blpwm(chip);

	bl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bl->base))
		return PTR_ERR(bl->base);

	clk = devm_clk_get_optional_enabled(dev, "clk_blpwm");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "clk_blpwm\n");
	bl->rate = clk ? clk_get_rate(clk) : 0;
	if (!bl->rate)
		bl->rate = BLPWM_DEFAULT_RATE;
	if (bl->rate > NSEC_PER_SEC)
		return dev_err_probe(dev, -EINVAL, "clock too fast\n");

	chip->ops = &hisi_blpwm_ops;
	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "can't add PWM chip\n");

	dev_info(dev, "clock %lu Hz, firmware state: ctrl %#x div %#x cfg %#x\n",
		 bl->rate, readl(bl->base + BLPWM_OUT_CTRL),
		 readl(bl->base + BLPWM_OUT_DIV), readl(bl->base + BLPWM_OUT_CFG));
	return 0;
}

static const struct of_device_id hisi_blpwm_of_match[] = {
	{ .compatible = "hisilicon,hisiblpwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_blpwm_of_match);

static struct platform_driver hisi_blpwm_driver = {
	.probe = hisi_blpwm_probe,
	.driver = {
		.name = "hisi-blpwm",
		.of_match_table = hisi_blpwm_of_match,
	},
};
module_platform_driver(hisi_blpwm_driver);

MODULE_DESCRIPTION("HiSilicon Kirin backlight PWM output driver");
MODULE_LICENSE("GPL");
