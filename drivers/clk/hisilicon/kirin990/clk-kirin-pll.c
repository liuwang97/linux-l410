// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 PLL gates and DVFS front-end clocks.
 *
 *  - "hisilicon,kirin-ppll-ctrl" / "hisilicon,ppll-ctrl": enable/lock/bypass
 *    control of the peripheral PLLs. The PLL rate itself is a fixed-clock
 *    parent; the second cell of hisilicon,ipc-lpm3-cmd-en is the PLL id.
 *  - "hisilicon,clkdev-dvfs": media clocks whose rate changes need a
 *    peripheral voltage vote; they forward everything to the real clock named
 *    by clock-friend-names.
 *
 * Ported from the Huawei vendor driver (hisi-kirin-ppll.c, clk-dvfs.c;
 * GPL-2.0).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>

#include "clk-kirin.h"

enum {
	PPLL0 = 0, PPLL1, PPLL2, PPLL3, PPLL4, PPLL5, PPLL6, PPLL7,
	SCPLL = 0x8, PPLL2_B = 0x9, FNPLL1 = 0xa, FNPLL4 = 0xb, AUPLL = 0xc,
	PCIE0PLL = 0xd, PCIE1PLL = 0xe,
};

#define HIWORD(bit)		BIT((bit) + 16)
#define PPLLCTRL0_LOCK		BIT(26)
#define PLL_LOCK_TIMEOUT_US	1000

/* HSDT CRG: PCIEPLL used as SCPLL (kirin990) */
#define HSDT_PCIEPLL_CTRL0	0x200
#define HSDT_PCIEPLL_EN		BIT(0)
#define HSDT_PCIEPLL_BP		BIT(1)
#define HSDT_PCIEPLL_CTRL1	0x204
#define HSDT_PCIEPLL_GT		BIT(26)
#define HSDT_PCIEPLL_STAT	0x208
#define HSDT_PCIEPLL_LOCK	BIT(0)

struct kirin_pll {
	struct clk_hw hw;
	u32 id;
	void __iomem *base;		/* en/gt/bypass registers */
	void __iomem *lock_base;	/* PPLLxCTRL0 (lock status) */
	u32 en[2], gt[2], bp[2];	/* <offset bit> */
	u32 ctrl0;
	bool has_regs;
	bool hsdt;
};

#define to_kirin_pll(_hw) container_of(_hw, struct kirin_pll, hw)

static void kirin_pll_hiword(struct kirin_pll *p, const u32 *r, bool on)
{
	writel(HIWORD(r[1]) | (on ? BIT(r[1]) : 0), p->base + r[0]);
}

static int kirin_pll_is_enabled(struct clk_hw *hw)
{
	struct kirin_pll *p = to_kirin_pll(hw);

	if (p->hsdt)
		return (readl(p->base + HSDT_PCIEPLL_CTRL0) &
			(HSDT_PCIEPLL_EN | HSDT_PCIEPLL_BP)) == HSDT_PCIEPLL_EN;
	if (!p->has_regs)
		return 1;
	return (readl(p->base + p->en[0]) & BIT(p->en[1])) &&
	       (readl(p->base + p->gt[0]) & BIT(p->gt[1])) &&
	       !(readl(p->base + p->bp[0]) & BIT(p->bp[1]));
}

static int kirin_pll_wait_lock(struct clk_hw *hw, void __iomem *reg, u32 bit)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(reg, val, val & bit, 1,
					PLL_LOCK_TIMEOUT_US);
	if (ret)
		pr_err("kirin-clk: %s: PLL lock timeout\n", clk_hw_get_name(hw));
	return ret;
}

static void kirin_hsdt_setbits(void __iomem *reg, u32 set, u32 clr)
{
	writel((readl(reg) & ~clr) | set, reg);
}

static int kirin_pll_enable(struct clk_hw *hw)
{
	struct kirin_pll *p = to_kirin_pll(hw);
	int ret;

	if (kirin_pll_is_enabled(hw))
		return 0;

	if (p->hsdt) {
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL1, 0, HSDT_PCIEPLL_GT);
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL0, HSDT_PCIEPLL_EN, 0);
		udelay(20);
		ret = kirin_pll_wait_lock(hw, p->base + HSDT_PCIEPLL_STAT,
					  HSDT_PCIEPLL_LOCK);
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL0, 0, HSDT_PCIEPLL_BP);
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL1, HSDT_PCIEPLL_GT, 0);
		return ret;
	}

	/* output gated, enable, wait for lock, leave bypass, ungate */
	kirin_pll_hiword(p, p->gt, false);
	kirin_pll_hiword(p, p->en, true);
	udelay(20);
	ret = kirin_pll_wait_lock(hw, p->lock_base + p->ctrl0, PPLLCTRL0_LOCK);
	kirin_pll_hiword(p, p->bp, false);
	kirin_pll_hiword(p, p->gt, true);
	return ret;
}

static void kirin_pll_disable(struct clk_hw *hw)
{
	struct kirin_pll *p = to_kirin_pll(hw);

	if (kirin_clk_keep_on)
		return;

	if (p->hsdt) {
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL1, 0, HSDT_PCIEPLL_GT);
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL0, HSDT_PCIEPLL_BP, 0);
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL0, 0, HSDT_PCIEPLL_EN);
		kirin_hsdt_setbits(p->base + HSDT_PCIEPLL_CTRL1, HSDT_PCIEPLL_GT, 0);
		return;
	}
	kirin_pll_hiword(p, p->gt, false);
	kirin_pll_hiword(p, p->bp, true);
	kirin_pll_hiword(p, p->en, false);
	kirin_pll_hiword(p, p->gt, true);
	udelay(1);
}

static const struct clk_ops kirin_pll_ops = {
	.enable = kirin_pll_enable,
	.disable = kirin_pll_disable,
	.is_enabled = kirin_pll_is_enabled,
};

/* PPLL0 / PPLL1 have no AP controls: always running */
static const struct clk_ops kirin_pll_fixed_ops = {
};

static void __init kirin_pll_register(struct device_node *np,
				      struct kirin_pll *p)
{
	struct clk_init_data init = { };
	const char *parent;

	parent = of_clk_get_parent_name(np, 0);
	init.name = kirin_clk_name(np);
	if (!init.name || !parent)
		goto err;
	init.ops = (p->has_regs || p->hsdt) ? &kirin_pll_ops :
					      &kirin_pll_fixed_ops;
	init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED;
	init.parent_names = &parent;
	init.num_parents = 1;
	p->hw.init = &init;
	if (!kirin_clk_register(np, &p->hw))
		return;
err:
	kfree(p);
}

static int __init kirin_pll_id(struct device_node *np, u32 *id)
{
	u32 cmd[KIRIN_LPM3_CMD_LEN];

	if (of_property_read_u32_array(np, "hisilicon,ipc-lpm3-cmd-en", cmd,
				       KIRIN_LPM3_CMD_LEN)) {
		pr_err("kirin-clk: %pOF: no PLL id\n", np);
		return -EINVAL;
	}
	*id = cmd[1];
	return 0;
}

static void __init kirin_ppll_setup(struct device_node *np)
{
	struct kirin_pll *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return;
	p->base = kirin_clk_parent_base(np);
	p->lock_base = kirin_clk_base(KIRIN_PMCTRL);
	if (!p->base || !p->lock_base || kirin_pll_id(np, &p->id)) {
		kfree(p);
		return;
	}
	p->has_regs =
		!of_property_read_u32_array(np, "hisilicon,pll-en-reg", p->en, 2) &&
		!of_property_read_u32_array(np, "hisilicon,pll-gt-reg", p->gt, 2) &&
		!of_property_read_u32_array(np, "hisilicon,pll-bypass-reg", p->bp, 2) &&
		!of_property_read_u32(np, "hisilicon,pll-ctrl0-reg", &p->ctrl0) &&
		p->en[1] < 16 && p->gt[1] < 16 && p->bp[1] < 16;
	kirin_pll_register(np, p);
}
CLK_OF_DECLARE(kirin_ppll, "hisilicon,kirin-ppll-ctrl", kirin_ppll_setup);

static void __init kirin_hsdt_pll_setup(struct device_node *np)
{
	struct kirin_pll *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return;
	p->base = kirin_clk_parent_base(np);
	if (!p->base || kirin_pll_id(np, &p->id)) {
		kfree(p);
		return;
	}
	/* only the SCPLL (HSDT PCIEPLL) is described on kirin990 */
	p->hsdt = p->id == SCPLL;
	if (!p->hsdt)
		pr_warn("kirin-clk: %pOF: PLL %u not supported, treated as fixed\n",
			np, p->id);
	kirin_pll_register(np, p);
}
CLK_OF_DECLARE(kirin_hsdt_pll, "hisilicon,ppll-ctrl", kirin_hsdt_pll_setup);

/* ---- "hisilicon,clkdev-dvfs" ------------------------------------------ */

#define DVFS_MAX_LEVELS		4

struct kirin_dvfs_clk {
	struct clk_hw hw;
	const char *link;
	struct clk *link_clk;
	u32 id;
	u32 levels;
	u32 freq[DVFS_MAX_LEVELS];	/* kHz, upper bound of each level */
	u32 volt[DVFS_MAX_LEVELS + 1];
	u32 cur_volt;
	bool prepared;
};

#define to_kirin_dvfs(_hw) container_of(_hw, struct kirin_dvfs_clk, hw)

static struct clk *kirin_dvfs_link(struct kirin_dvfs_clk *d)
{
	return kirin_clk_friend(d->link, &d->link_clk);
}

static u32 kirin_dvfs_volt(struct kirin_dvfs_clk *d, unsigned long rate)
{
	unsigned int i;

	for (i = 0; i < d->levels; i++)
		if (rate <= (unsigned long)d->freq[i] * 1000)
			return d->volt[i];
	return d->volt[d->levels];
}

static int kirin_dvfs_prepare(struct clk_hw *hw)
{
	struct kirin_dvfs_clk *d = to_kirin_dvfs(hw);
	struct clk *link = kirin_dvfs_link(d);
	int ret;

	if (!link)
		return -EPROBE_DEFER;
	d->cur_volt = kirin_dvfs_volt(d, clk_get_rate(link));
	ret = kirin_clk_perivolt_set(d->id, d->cur_volt);
	if (ret)
		return ret;
	ret = clk_prepare(link);
	if (ret)
		kirin_clk_perivolt_set(d->id, 0);
	else
		d->prepared = true;
	return ret;
}

static void kirin_dvfs_unprepare(struct clk_hw *hw)
{
	struct kirin_dvfs_clk *d = to_kirin_dvfs(hw);

	clk_unprepare(d->link_clk);
	d->prepared = false;
	kirin_clk_perivolt_set(d->id, 0);
}

static int kirin_dvfs_enable(struct clk_hw *hw)
{
	return clk_enable(to_kirin_dvfs(hw)->link_clk);
}

static void kirin_dvfs_disable(struct clk_hw *hw)
{
	clk_disable(to_kirin_dvfs(hw)->link_clk);
}

static unsigned long kirin_dvfs_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk *link = kirin_dvfs_link(to_kirin_dvfs(hw));

	return link ? clk_get_rate(link) : 0;
}

static int kirin_dvfs_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	struct clk *link = kirin_dvfs_link(to_kirin_dvfs(hw));
	long rate;

	if (!link)
		return -EPROBE_DEFER;
	rate = clk_round_rate(link, req->rate);
	if (rate < 0)
		return rate;
	req->rate = rate;
	return 0;
}

static int kirin_dvfs_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct kirin_dvfs_clk *d = to_kirin_dvfs(hw);
	struct clk *link = kirin_dvfs_link(d);
	u32 volt;
	int ret;

	if (!link)
		return -EPROBE_DEFER;
	if (!d->prepared)
		return clk_set_rate(link, rate);

	/* raise the voltage before, lower it after the rate change */
	volt = kirin_dvfs_volt(d, rate);
	if (volt > d->cur_volt) {
		ret = kirin_clk_perivolt_set(d->id, volt);
		if (ret)
			return ret;
	}
	ret = clk_set_rate(link, rate);
	if (ret) {
		if (volt > d->cur_volt)
			kirin_clk_perivolt_set(d->id, d->cur_volt);
		return ret;
	}
	if (volt < d->cur_volt)
		kirin_clk_perivolt_set(d->id, volt);
	d->cur_volt = volt;
	return 0;
}

static const struct clk_ops kirin_dvfs_ops = {
	.prepare = kirin_dvfs_prepare,
	.unprepare = kirin_dvfs_unprepare,
	.enable = kirin_dvfs_enable,
	.disable = kirin_dvfs_disable,
	.recalc_rate = kirin_dvfs_recalc_rate,
	.determine_rate = kirin_dvfs_determine_rate,
	.set_rate = kirin_dvfs_set_rate,
};

static void __init kirin_dvfs_setup(struct device_node *np)
{
	struct clk_init_data init = { };
	struct kirin_dvfs_clk *d;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return;
	init.name = kirin_clk_name(np);
	if (!init.name ||
	    of_property_read_string(np, "clock-friend-names", &d->link) ||
	    of_property_read_u32(np, "hisilicon,clk-devfreq-id", &d->id) ||
	    of_property_read_u32(np, "hisilicon,clk-dvfs-level", &d->levels) ||
	    d->levels > DVFS_MAX_LEVELS ||
	    of_property_read_u32_array(np, "hisilicon,sensitive-freq", d->freq,
				       d->levels) ||
	    of_property_read_u32_array(np, "hisilicon,sensitive-volt", d->volt,
				       d->levels + 1)) {
		pr_err("kirin-clk: %pOF: incomplete clkdev-dvfs description\n",
		       np);
		kfree(d);
		return;
	}
	init.ops = &kirin_dvfs_ops;
	init.flags = CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED;
	d->hw.init = &init;
	if (kirin_clk_register(np, &d->hw))
		kfree(d);
}
CLK_OF_DECLARE(kirin_dvfs, "hisilicon,clkdev-dvfs", kirin_dvfs_setup);
