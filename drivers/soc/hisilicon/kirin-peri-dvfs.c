// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 peripheral voltage voting ("peri DVFS")
 *
 * Several peripheral clocks may only run fast when the shared peripheral
 * rail is at a high enough level. Each voter owns a small field in PMCTRL
 * where it records the level it needs; LPM3 applies the maximum. On the
 * Kirin 990 every change is additionally signalled to LPM3 through the
 * "peri-volt" hardware vote channel; when a voter raises its level the
 * call waits until PMCTRL reports the new level as applied.
 *
 * Vendor firmware device tree:
 *   peri_dvfs@0xfff01000 {              compatible = "hisilicon,soc-peri-dvfs"
 *       peri_dvfs_hw_vote_flag;
 *       pvp_edc0 {                      compatible = "hisilicon,soc-peri-volt"
 *           perivolt-poll-reg = <0x354 0x03>;   PMCTRL field of this voter
 *           perivolt-poll-id = <0x14>;          voter id (clock-id)
 *           perivolt-avs-ip = <3>;              AVS id, optional
 *       };
 *   };
 *
 * The clock driver calls set_volt(id, level) through
 * kirin_clk_set_perivolt_ops().
 *
 * Based on the Huawei vendor driver
 * (drivers/clk/hisi_extreme/dvfs/hisi_peri_dvfs_volt.c).
 */

#include <linux/bitops.h>
#include <linux/clk/kirin.h>
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/hisilicon/kirin-hw-vote.h>

#define PERI_HWLOCK_ID			19
#define PERI_HWLOCK_TIMEOUT_MS		1000
#define PERI_VOLT_MAX			3

#define SCTRL_SCBAKDATA24		0x46c
#define SCTRL_AVS_DONE			BIT(28)	/* set by LPM3 when AVS is done */

/* PMCTRL "peri ctrl4" (perivolt-poll-lpmcu): level LPM3 applied */
#define PERI_STATUS_VDD			GENMASK(29, 28)
#define PERI_APPLY_POLLS		400	/* x 150..300 us */

/* peri-volt vote: level [3:0], avs id [7:4], voter id [14:8] */
#define PERI_HV_AVS_SHIFT		4
#define PERI_HV_ID_SHIFT		8
#define PERI_HV_MASK			0x7fff
#define PERI_HV_TOGGLE			BIT(3)

struct kirin_pvp {
	const char *name;
	u32 id;
	u32 avs;
	u32 offset;
	u32 mask;
	u32 status;
};

struct kirin_peri_dvfs {
	struct device *dev;
	void __iomem *pmctrl;
	void __iomem *sctrl;
	struct hwspinlock *hwlock;
	struct kirin_hv *hv;
	struct mutex lock;
	int npvp;
	struct kirin_pvp pvp[];
};

static struct kirin_peri_dvfs *kirin_peri;

static struct kirin_pvp *kirin_peri_find(struct kirin_peri_dvfs *pd, u32 id)
{
	int i;

	for (i = 0; i < pd->npvp; i++)
		if (pd->pvp[i].id == id)
			return &pd->pvp[i];
	return NULL;
}

static void kirin_peri_hw_vote(struct kirin_peri_dvfs *pd, struct kirin_pvp *pvp,
			       u32 level)
{
	u32 val;

	val = ((pvp->id << PERI_HV_ID_SHIFT) | (pvp->avs << PERI_HV_AVS_SHIFT) |
	       level) & PERI_HV_MASK;
	/* every vote has to differ from the previous one to interrupt LPM3 */
	if (val == kirin_hv_get_vote(pd->hv))
		val ^= PERI_HV_TOGGLE;
	kirin_hv_set(pd->hv, val);
}

static u32 kirin_peri_applied(struct kirin_peri_dvfs *pd, struct kirin_pvp *pvp)
{
	return (readl(pd->pmctrl + pvp->status) & PERI_STATUS_VDD) >>
	       __ffs(PERI_STATUS_VDD);
}

/* raising the level: wait until LPM3 has applied it */
static int kirin_peri_wait_applied(struct kirin_peri_dvfs *pd,
				   struct kirin_pvp *pvp, u32 level)
{
	int i;

	for (i = 0; i < PERI_APPLY_POLLS; i++) {
		if (kirin_peri_applied(pd, pvp) >= level)
			return 0;
		usleep_range(150, 300);
	}

	dev_err(pd->dev, "%s: level %u not applied (at %u)\n", pvp->name,
		level, kirin_peri_applied(pd, pvp));
	return -ETIMEDOUT;
}

static int kirin_peri_set_volt(u32 id, u32 level)
{
	struct kirin_peri_dvfs *pd = READ_ONCE(kirin_peri);
	struct kirin_pvp *pvp;
	u32 val;
	int ret;

	if (!pd)
		return -EPROBE_DEFER;
	if (level > PERI_VOLT_MAX)
		return -EINVAL;

	pvp = kirin_peri_find(pd, id);
	if (!pvp) {
		dev_dbg(pd->dev, "no voter %u\n", id);
		return -ENODEV;
	}

	mutex_lock(&pd->lock);
	ret = hwspin_lock_timeout(pd->hwlock, PERI_HWLOCK_TIMEOUT_MS);
	if (ret) {
		dev_err(pd->dev, "%s: hwspinlock timeout\n", pvp->name);
		goto unlock;
	}

	val = readl(pd->pmctrl + pvp->offset);
	val &= ~pvp->mask;
	val |= (level << __ffs(pvp->mask)) & pvp->mask;
	writel(val, pd->pmctrl + pvp->offset);

	if (pvp->avs) {
		val = readl(pd->sctrl + SCTRL_SCBAKDATA24);
		writel(val & ~SCTRL_AVS_DONE, pd->sctrl + SCTRL_SCBAKDATA24);
	}

	if (pd->hv)
		kirin_peri_hw_vote(pd, pvp, level);

	hwspin_unlock(pd->hwlock);

	ret = kirin_peri_wait_applied(pd, pvp, level);
unlock:
	mutex_unlock(&pd->lock);
	return ret;
}

static const struct kirin_clk_perivolt_ops kirin_peri_ops = {
	.set_volt = kirin_peri_set_volt,
};

static void kirin_peri_release(void *data)
{
	struct kirin_peri_dvfs *pd = data;

	kirin_clk_set_perivolt_ops(NULL);
	WRITE_ONCE(kirin_peri, NULL);
	if (pd->hwlock)
		hwspin_lock_free(pd->hwlock);
	if (pd->sctrl)
		iounmap(pd->sctrl);
}

static int kirin_peri_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child, *sctrl_np;
	struct kirin_peri_dvfs *pd;
	struct resource *res;
	u32 reg[2];
	int n, ret;

	n = of_get_available_child_count(np);
	pd = devm_kzalloc(dev, struct_size(pd, pvp, n), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;
	pd->dev = dev;
	mutex_init(&pd->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	/* PMCTRL is shared with the clock and hw vote code: map, do not claim */
	pd->pmctrl = devm_ioremap(dev, res->start, resource_size(res));
	if (!pd->pmctrl)
		return -ENOMEM;

	if (of_property_read_bool(np, "peri_dvfs_hw_vote_flag")) {
		pd->hv = kirin_hv_get("peri-volt", "vote-src-1");
		if (IS_ERR(pd->hv))
			return dev_err_probe(dev, PTR_ERR(pd->hv), "no peri-volt vote\n");
	}

	pd->hwlock = hwspin_lock_request_specific(PERI_HWLOCK_ID);
	if (IS_ERR_OR_NULL(pd->hwlock)) {
		pd->hwlock = NULL;
		return dev_err_probe(dev, -EPROBE_DEFER, "hwspinlock %d\n", PERI_HWLOCK_ID);
	}

	sctrl_np = of_find_compatible_node(NULL, NULL, "hisilicon,sysctrl");
	pd->sctrl = sctrl_np ? of_iomap(sctrl_np, 0) : NULL;
	of_node_put(sctrl_np);

	ret = devm_add_action_or_reset(dev, kirin_peri_release, pd);
	if (ret)
		return ret;
	if (!pd->sctrl)
		return dev_err_probe(dev, -ENODEV, "no SCTRL\n");

	for_each_available_child_of_node(np, child) {
		struct kirin_pvp *pvp = &pd->pvp[pd->npvp];

		if (of_property_read_u32_array(child, "perivolt-poll-reg", reg, 2) ||
		    !reg[1] || reg[0] > resource_size(res) - 4 ||
		    of_property_read_u32(child, "perivolt-poll-id", &pvp->id) ||
		    of_property_read_u32(child, "perivolt-poll-lpmcu", &pvp->status) ||
		    pvp->status > resource_size(res) - 4) {
			dev_warn(dev, "%pOFn: incomplete, skipped\n", child);
			continue;
		}
		pvp->name = child->name;
		pvp->offset = reg[0];
		pvp->mask = reg[1];
		of_property_read_u32(child, "perivolt-avs-ip", &pvp->avs);
		pd->npvp++;
	}

	WRITE_ONCE(kirin_peri, pd);
	kirin_clk_set_perivolt_ops(&kirin_peri_ops);

	dev_info(dev, "%d peripheral voltage voters%s\n", pd->npvp,
		 pd->hv ? ", hw vote" : "");
	return 0;
}

static const struct of_device_id kirin_peri_of_match[] = {
	{ .compatible = "hisilicon,soc-peri-dvfs" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin_peri_of_match);

static struct platform_driver kirin_peri_driver = {
	.driver = {
		.name		= "kirin-peri-dvfs",
		.of_match_table	= kirin_peri_of_match,
		.suppress_bind_attrs = true,
	},
	.probe	= kirin_peri_probe,
};
module_platform_driver(kirin_peri_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 peripheral voltage voting");
MODULE_LICENSE("GPL");
