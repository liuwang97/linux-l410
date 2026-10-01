// SPDX-License-Identifier: GPL-2.0
/*
 * RTC of the HiSilicon SPMI PMIC (Kirin 990 generation)
 *
 * The PMIC keeps a battery backed 32-bit seconds counter with a PL031-like
 * register set, each register being four consecutive 8-bit PMIC registers
 * (least significant byte first) starting at "hisilicon,pmic-rtc-base":
 *   +0x0 DR (count), +0x4 MR (match), +0x8 LR (load), +0xc CR (bit0: enable)
 * The match interrupt is a PMIC interrupt.
 *
 * The vendor kernel additionally mirrors the counter into a SoC PL031 and
 * uses that one for alarms while running; the PMIC counter is the one that
 * survives power off, so it is the only one used here.
 *
 * Based on the Huawei vendor driver (drivers/rtc/rtc_hisi_pmic_spmi.c).
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#define HISI_RTC_DR	0x0
#define HISI_RTC_MR	0x4
#define HISI_RTC_LR	0x8
#define HISI_RTC_CR	0xc
#define HISI_RTC_CR_EN	BIT(0)

struct hisi_spmi_rtc {
	struct device *dev;
	struct regmap *regmap;
	struct rtc_device *rtc;
	u32 base;
	int irq;
	bool alarm_enabled;
};

static int hisi_spmi_rtc_read32(struct hisi_spmi_rtc *hrtc, u32 reg, u32 *val)
{
	unsigned int b;
	u32 v = 0;
	int i, ret;

	for (i = 0; i < 4; i++) {
		ret = regmap_read(hrtc->regmap, hrtc->base + reg + i, &b);
		if (ret)
			return ret;
		v |= (b & 0xff) << (8 * i);
	}
	*val = v;
	return 0;
}

static int hisi_spmi_rtc_write32(struct hisi_spmi_rtc *hrtc, u32 reg, u32 val)
{
	int i, ret;

	for (i = 0; i < 4; i++) {
		ret = regmap_write(hrtc->regmap, hrtc->base + reg + i,
				   (val >> (8 * i)) & 0xff);
		if (ret)
			return ret;
	}
	return 0;
}

static int hisi_spmi_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct hisi_spmi_rtc *hrtc = dev_get_drvdata(dev);
	u32 t1, t2;
	int ret, tries = 3;

	/* the four bytes are read one by one: retry if a carry sneaks in */
	ret = hisi_spmi_rtc_read32(hrtc, HISI_RTC_DR, &t1);
	while (!ret && tries--) {
		ret = hisi_spmi_rtc_read32(hrtc, HISI_RTC_DR, &t2);
		if (ret || t1 == t2)
			break;
		t1 = t2;
	}
	if (ret)
		return ret;

	rtc_time64_to_tm(t1, tm);
	return 0;
}

static int hisi_spmi_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct hisi_spmi_rtc *hrtc = dev_get_drvdata(dev);

	return hisi_spmi_rtc_write32(hrtc, HISI_RTC_LR, rtc_tm_to_time64(tm));
}

static int hisi_spmi_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct hisi_spmi_rtc *hrtc = dev_get_drvdata(dev);
	u32 t;
	int ret;

	ret = hisi_spmi_rtc_read32(hrtc, HISI_RTC_MR, &t);
	if (ret)
		return ret;

	rtc_time64_to_tm(t, &alrm->time);
	alrm->enabled = hrtc->alarm_enabled;
	return 0;
}

static int hisi_spmi_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct hisi_spmi_rtc *hrtc = dev_get_drvdata(dev);

	if (hrtc->alarm_enabled == !!enabled)
		return 0;

	if (enabled)
		enable_irq(hrtc->irq);
	else
		disable_irq(hrtc->irq);
	hrtc->alarm_enabled = enabled;
	return 0;
}

static int hisi_spmi_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct hisi_spmi_rtc *hrtc = dev_get_drvdata(dev);
	int ret;

	hisi_spmi_rtc_alarm_irq_enable(dev, 0);
	ret = hisi_spmi_rtc_write32(hrtc, HISI_RTC_MR, rtc_tm_to_time64(&alrm->time));
	if (ret)
		return ret;

	return hisi_spmi_rtc_alarm_irq_enable(dev, alrm->enabled);
}

static const struct rtc_class_ops hisi_spmi_rtc_ops = {
	.read_time		= hisi_spmi_rtc_read_time,
	.set_time		= hisi_spmi_rtc_set_time,
	.read_alarm		= hisi_spmi_rtc_read_alarm,
	.set_alarm		= hisi_spmi_rtc_set_alarm,
	.alarm_irq_enable	= hisi_spmi_rtc_alarm_irq_enable,
};

static const struct rtc_class_ops hisi_spmi_rtc_noalarm_ops = {
	.read_time		= hisi_spmi_rtc_read_time,
	.set_time		= hisi_spmi_rtc_set_time,
};

static irqreturn_t hisi_spmi_rtc_alarm_irq(int irq, void *data)
{
	struct hisi_spmi_rtc *hrtc = data;

	rtc_update_irq(hrtc->rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}

static int hisi_spmi_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hisi_spmi_rtc *hrtc;
	unsigned int cr;
	int ret;

	hrtc = devm_kzalloc(dev, sizeof(*hrtc), GFP_KERNEL);
	if (!hrtc)
		return -ENOMEM;

	hrtc->dev = dev;
	hrtc->regmap = dev_get_regmap(dev->parent, NULL);
	if (!hrtc->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");

	if (of_property_read_u32(dev->of_node, "hisilicon,pmic-rtc-base", &hrtc->base))
		return dev_err_probe(dev, -EINVAL, "no hisilicon,pmic-rtc-base\n");

	platform_set_drvdata(pdev, hrtc);

	ret = regmap_read(hrtc->regmap, hrtc->base + HISI_RTC_CR, &cr);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read control register\n");
	if (!(cr & HISI_RTC_CR_EN)) {
		dev_warn(dev, "counter was stopped, starting it\n");
		ret = regmap_write(hrtc->regmap, hrtc->base + HISI_RTC_CR,
				   cr | HISI_RTC_CR_EN);
		if (ret)
			return ret;
		msleep(200);	/* as the vendor driver does */
	}

	hrtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(hrtc->rtc))
		return PTR_ERR(hrtc->rtc);

	hrtc->rtc->range_max = U32_MAX;

	hrtc->irq = platform_get_irq_byname_optional(pdev, "hisi-pmic-rtc");
	if (hrtc->irq == -EPROBE_DEFER)
		return hrtc->irq;
	if (hrtc->irq > 0) {
		/* stays masked until an alarm is armed */
		ret = devm_request_threaded_irq(dev, hrtc->irq, NULL,
						hisi_spmi_rtc_alarm_irq,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						"hisi-pmic-rtc", hrtc);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request alarm irq\n");
		hrtc->rtc->ops = &hisi_spmi_rtc_ops;
		device_init_wakeup(dev, true);
		/* arm the alarm as a wakeup interrupt across system sleep */
		ret = devm_pm_set_wake_irq(dev, hrtc->irq);
		if (ret)
			dev_warn(dev, "alarm cannot wake the system: %d\n", ret);
	} else {
		dev_warn(dev, "no alarm interrupt, alarms disabled\n");
		hrtc->rtc->ops = &hisi_spmi_rtc_noalarm_ops;
	}

	return devm_rtc_register_device(hrtc->rtc);
}

static const struct of_device_id hisi_spmi_rtc_of_match[] = {
	{ .compatible = "hisilicon-hisi-rtc-spmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_spmi_rtc_of_match);

static struct platform_driver hisi_spmi_rtc_driver = {
	.driver = {
		.name		= "hisi-spmi-rtc",
		.of_match_table	= hisi_spmi_rtc_of_match,
	},
	.probe	= hisi_spmi_rtc_probe,
};
module_platform_driver(hisi_spmi_rtc_driver);

MODULE_DESCRIPTION("HiSilicon SPMI PMIC RTC (Kirin 990)");
MODULE_LICENSE("GPL");
