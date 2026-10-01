// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 hardware vote block (PMCTRL)
 *
 * Frequency / voltage requests to the LPM3 power controller are "votes" in
 * PMCTRL registers: each channel (little-freq, middle-freq, big-freq,
 * gpu-freq, l3-freq, peri-volt, ...) has a result register and one register
 * field per voter. The vendor firmware device tree describes them:
 *
 *   hw_vote {                               compatible = "hisi,freq-hw-vote"
 *       little-freq {
 *           result_reg = <offset rd_mask wr_mask>;
 *           ratio = <1>;                    value unit (MHz for frequencies)
 *           vote-src-1 { vote_reg = <offset rd_mask wr_mask>; };
 *       };
 *   };
 *
 * A vote is written as wr_mask | value << ffs(rd_mask) - 1 in one go (the
 * wr_mask bit tells the hardware which half of the register is updated).
 *
 * This driver maps the block and hands out voters to the cpufreq driver
 * (it creates the "hisi-hwvote-cpufreq" device) and to the peripheral
 * DVFS code.
 *
 * Based on the Huawei vendor driver (drivers/hisi/hw_vote/hw_vote.c).
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/hisilicon/kirin-hw-vote.h>

struct kirin_hv_reg {
	void __iomem *reg;
	u32 rd_mask;
	u32 wr_mask;
};

struct kirin_hv {
	struct kirin_hv_reg vote;
	struct kirin_hv_reg result;
	u32 ratio;
};

static DEFINE_MUTEX(kirin_hv_lock);
static struct device *kirin_hv_dev;
static void __iomem *kirin_hv_base;

static int kirin_hv_parse_reg(struct device_node *np, const char *prop,
			      struct kirin_hv_reg *r)
{
	u32 v[3];
	int ret;

	ret = of_property_read_u32_array(np, prop, v, 3);
	if (ret)
		return ret;
	if (!v[1] || v[0] > SZ_4K - 4)
		return -EINVAL;

	r->reg = kirin_hv_base + v[0];
	r->rd_mask = v[1];
	r->wr_mask = v[2];
	return 0;
}

static u32 kirin_hv_read(const struct kirin_hv_reg *r)
{
	return (readl(r->reg) & r->rd_mask) >> __ffs(r->rd_mask);
}

/**
 * kirin_hv_get() - claim a voter
 * @channel: channel node name, e.g. "little-freq", "peri-volt"
 * @src: voter node name, e.g. "vote-src-1"
 *
 * Return: the voter, ERR_PTR(-EPROBE_DEFER) while the hw_vote block has not
 * probed, ERR_PTR(-ENODEV/-EBUSY) otherwise.
 */
struct kirin_hv *kirin_hv_get(const char *channel, const char *src)
{
	struct device_node *ch, *vs;
	struct kirin_hv *hv;
	int ret;

	mutex_lock(&kirin_hv_lock);
	if (!kirin_hv_dev) {
		hv = ERR_PTR(-EPROBE_DEFER);
		goto out;
	}

	ch = of_get_child_by_name(kirin_hv_dev->of_node, channel);
	vs = ch ? of_get_child_by_name(ch, src) : NULL;
	if (!vs) {
		hv = ERR_PTR(-ENODEV);
		goto put;
	}

	hv = devm_kzalloc(kirin_hv_dev, sizeof(*hv), GFP_KERNEL);
	if (!hv) {
		hv = ERR_PTR(-ENOMEM);
		goto put;
	}

	ret = kirin_hv_parse_reg(ch, "result_reg", &hv->result);
	if (!ret)
		ret = kirin_hv_parse_reg(vs, "vote_reg", &hv->vote);
	if (!ret && (of_property_read_u32(ch, "ratio", &hv->ratio) || !hv->ratio))
		ret = -EINVAL;
	if (ret) {
		devm_kfree(kirin_hv_dev, hv);
		hv = ERR_PTR(ret);
	}
put:
	of_node_put(vs);
	of_node_put(ch);
out:
	mutex_unlock(&kirin_hv_lock);
	return hv;
}
EXPORT_SYMBOL_GPL(kirin_hv_get);

/**
 * kirin_hv_get_reg() - claim a voter the firmware DT does not describe
 * @offset: vote register offset in PMCTRL (e.g. 0x270, the DDR minimum)
 * @rd_mask: value field
 * @wr_mask: write enable bit(s) of that field
 *
 * The result reads back the vote itself (value unit 1, e.g. MHz).
 */
struct kirin_hv *kirin_hv_get_reg(u32 offset, u32 rd_mask, u32 wr_mask)
{
	struct kirin_hv *hv;

	if (!rd_mask || offset > SZ_4K - 4)
		return ERR_PTR(-EINVAL);

	mutex_lock(&kirin_hv_lock);
	if (!kirin_hv_dev) {
		hv = ERR_PTR(-EPROBE_DEFER);
		goto out;
	}
	hv = devm_kzalloc(kirin_hv_dev, sizeof(*hv), GFP_KERNEL);
	if (!hv) {
		hv = ERR_PTR(-ENOMEM);
		goto out;
	}
	hv->vote.reg = kirin_hv_base + offset;
	hv->vote.rd_mask = rd_mask;
	hv->vote.wr_mask = wr_mask;
	hv->result = hv->vote;
	hv->ratio = 1;
out:
	mutex_unlock(&kirin_hv_lock);
	return hv;
}
EXPORT_SYMBOL_GPL(kirin_hv_get_reg);

/* write @value (in channel units, e.g. MHz) as this voter's request */
void kirin_hv_set(struct kirin_hv *hv, u32 value)
{
	u32 shift = __ffs(hv->vote.rd_mask);
	u32 max = hv->vote.rd_mask >> shift;

	value /= hv->ratio;
	writel(hv->vote.wr_mask | (min(value, max) << shift), hv->vote.reg);
}
EXPORT_SYMBOL_GPL(kirin_hv_set);

/* this voter's current request */
u32 kirin_hv_get_vote(struct kirin_hv *hv)
{
	return kirin_hv_read(&hv->vote) * hv->ratio;
}
EXPORT_SYMBOL_GPL(kirin_hv_get_vote);

/* what LPM3 granted to the channel */
u32 kirin_hv_get_result(struct kirin_hv *hv)
{
	return kirin_hv_read(&hv->result) * hv->ratio;
}
EXPORT_SYMBOL_GPL(kirin_hv_get_result);

static void kirin_hv_unregister(void *data)
{
	platform_device_unregister(data);
}

static int kirin_hv_probe(struct platform_device *pdev)
{
	struct platform_device *cpufreq;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	/* PMCTRL is shared with the clock and DVFS code: map, do not claim */
	kirin_hv_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!kirin_hv_base)
		return -ENOMEM;

	mutex_lock(&kirin_hv_lock);
	kirin_hv_dev = &pdev->dev;
	mutex_unlock(&kirin_hv_lock);

	cpufreq = platform_device_register_data(&pdev->dev, "hisi-hwvote-cpufreq",
						PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(cpufreq))
		return PTR_ERR(cpufreq);

	return devm_add_action_or_reset(&pdev->dev, kirin_hv_unregister, cpufreq);
}

static const struct of_device_id kirin_hv_of_match[] = {
	{ .compatible = "hisi,freq-hw-vote" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin_hv_of_match);

static struct platform_driver kirin_hv_driver = {
	.driver = {
		.name		= "kirin-hw-vote",
		.of_match_table	= kirin_hv_of_match,
		.suppress_bind_attrs = true,
	},
	.probe	= kirin_hv_probe,
};

static int __init kirin_hv_init(void)
{
	return platform_driver_register(&kirin_hv_driver);
}
core_initcall(kirin_hv_init);

MODULE_DESCRIPTION("HiSilicon Kirin 990 hardware vote (PMCTRL)");
MODULE_LICENSE("GPL");
