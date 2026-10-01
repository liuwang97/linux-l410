// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 clocks owned by the LPM3 power controller:
 *
 *  - "hisilicon,hi3xxx-xfreq-clk": CPU clusters, GPU and DDR. The rate is
 *    changed by an IPC request to LPM3 (or, for the DDR limits, a hardware
 *    vote register in PMCTRL); the current OPP index is published by LPM3
 *    in a SCTRL backup-data register.
 *  - "hisilicon,interactive-clk": bus clocks that LPM3 switches on and off
 *    on request.
 *
 * Ported from the Huawei vendor driver (clk-kirin-common.c, GPL-2.0).
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mailbox/kirin-ipc.h>
#include <linux/slab.h>
#include <linux/soc/hisilicon/kirin-hw-vote.h>

#include "clk-kirin.h"

#define XFREQ_MAX_OPPS		16

enum {
	XFREQ_CPU_CLUSTER0,
	XFREQ_CPU_CLUSTER1,
	XFREQ_GPU,
	XFREQ_DDR_FREQ,
	XFREQ_DDR_MAX,
	XFREQ_DDR_MIN,
	XFREQ_DMSS_MIN,
};

struct kirin_xfreq {
	struct clk_hw hw;
	void __iomem *scbak;		/* SCTRL: current OPP indexes */
	void __iomem *vote;		/* PMCTRL: DDR hardware vote */
	u32 id;
	bool hw_vote;
	u32 vote_mask;
	u32 set_cmd[KIRIN_LPM3_CMD_LEN];
	u32 get_cmd[KIRIN_LPM3_CMD_LEN];
	u32 freq[XFREQ_MAX_OPPS];	/* kHz */
	unsigned int nr_opps;
	unsigned long rate;		/* last requested rate */
	/* PMCTRL hardware vote channel ("hisilicon,hw-vote-channel") */
	const char *hv_channel, *hv_src;
	struct kirin_hv *hv;
	unsigned int nr_logged;		/* bring-up: first votes are logged */
};

#define to_kirin_xfreq(_hw) container_of(_hw, struct kirin_xfreq, hw)

/*
 * SCBAKDATA: bits 0-3 little cluster, 4-7 big cluster, 8-11 DDR,
 * 12-15 GPU OPP index.
 */
static int kirin_xfreq_index(struct kirin_xfreq *x)
{
	static const u8 shift[] = {
		[XFREQ_CPU_CLUSTER0] = 0, [XFREQ_CPU_CLUSTER1] = 4,
		[XFREQ_GPU] = 12, [XFREQ_DDR_FREQ] = 8,
	};
	u32 idx;

	if (x->id > XFREQ_DDR_FREQ || !x->nr_opps)
		return -EINVAL;
	idx = (readl(x->scbak) >> shift[x->id]) & 0xf;
	return idx < x->nr_opps ? idx : 0;
}

static unsigned long kirin_xfreq_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct kirin_xfreq *x = to_kirin_xfreq(hw);
	int idx;
	u32 mhz;

	/* voted domains: the frequency LPM3 granted, from the result register */
	if (!IS_ERR_OR_NULL(x->hv)) {
		mhz = kirin_hv_get_result(x->hv);
		if (mhz)
			return (unsigned long)mhz * 1000000;
	}

	idx = kirin_xfreq_index(x);
	if (idx < 0)
		return x->rate;
	return (unsigned long)x->freq[idx] * 1000;
}

static int kirin_xfreq_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	return 0;	/* LPM3 picks the closest OPP */
}

static int kirin_xfreq_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct kirin_xfreq *x = to_kirin_xfreq(hw);
	u32 mhz = rate / 1000000;
	u32 cmd[KIRIN_LPM3_CMD_LEN];
	int ret;

	/*
	 * LPM3 ignores the IPC rate request for some domains (the GPU): their
	 * DVFS is driven by the PMCTRL hardware vote, like the vendor kernel
	 * does. The vote block is a platform driver, so look it up lazily and
	 * fall back to IPC until it is there.
	 */
	if (x->hv_channel) {
		if (IS_ERR_OR_NULL(x->hv)) {
			x->hv = kirin_hv_get(x->hv_channel, x->hv_src);
			if (IS_ERR(x->hv))
				pr_warn_ratelimited("kirin-clk: %s: hw vote %s/%s: %pe, using IPC\n",
						    clk_hw_get_name(hw), x->hv_channel,
						    x->hv_src, x->hv);
			else
				pr_info("kirin-clk: %s: DVFS through hw vote %s/%s\n",
					clk_hw_get_name(hw), x->hv_channel, x->hv_src);
		}
		if (!IS_ERR(x->hv)) {
			kirin_hv_set(x->hv, mhz);
			x->rate = rate;
			if (x->nr_logged < 16) {
				x->nr_logged++;
				pr_info("kirin-clk: %s: vote %u MHz, vote reg %u, result %u\n",
					clk_hw_get_name(hw), mhz,
					kirin_hv_get_vote(x->hv),
					kirin_hv_get_result(x->hv));
			}
			return 0;
		}
	}

	if (x->hw_vote) {
		if (x->id < XFREQ_DDR_MAX || x->id > XFREQ_DMSS_MIN)
			return -EINVAL;
		writel(mhz | x->vote_mask, x->vote);
	} else {
		cmd[0] = x->set_cmd[0];
		cmd[1] = mhz;
		ret = kirin_ipc_send(KIRIN_LPM3_MBOX, cmd, KIRIN_LPM3_CMD_LEN,
				     NULL, 0);
		if (ret) {
			pr_err("kirin-clk: %s: LPM3 rate request failed: %d\n",
			       clk_hw_get_name(hw), ret);
			return ret;
		}
	}
	x->rate = rate;
	return 0;
}

static const struct clk_ops kirin_xfreq_ops = {
	.recalc_rate = kirin_xfreq_recalc_rate,
	.determine_rate = kirin_xfreq_determine_rate,
	.set_rate = kirin_xfreq_set_rate,
};

/* the first node clocked by @clk_np that carries an OPP table */
static struct device_node *kirin_xfreq_user(struct device_node *clk_np)
{
	struct device_node *np, *c;
	int i;

	for_each_node_with_property(np, "clocks") {
		if (!of_device_is_available(np) ||
		    (!of_property_present(np, "operating-points-v2") &&
		     !of_property_present(np, "operating-points")))
			continue;
		for (i = 0; (c = of_parse_phandle(np, "clocks", i)); i++) {
			of_node_put(c);
			if (c == clk_np)
				return np;
		}
	}
	return NULL;
}

/* OPP table of the device that uses the clock */
static struct device_node *kirin_xfreq_opp_owner(struct device_node *clk_np,
						 u32 id)
{
	struct device_node *np;

	np = kirin_xfreq_user(clk_np);
	if (np)
		return np;

	/* the DDR devfreq nodes do not reference their clock */
	switch (id) {
	case XFREQ_GPU:
		return of_find_compatible_node(NULL, NULL, "arm,mali-midgard");
	case XFREQ_DDR_FREQ:
	case XFREQ_DDR_MAX:
	case XFREQ_DDR_MIN:
		return of_find_compatible_node(NULL, NULL,
					       "hisilicon,ddr_devfreq");
	default:
		return NULL;
	}
}

static int kirin_xfreq_opps(struct kirin_xfreq *x, struct device_node *owner)
{
	struct device_node *opp_np, *child;
	const __be32 *val;
	unsigned int n = 0;
	int len;

	opp_np = of_parse_phandle(owner, "operating-points-v2", 0);
	if (opp_np) {
		for_each_available_child_of_node(opp_np, child) {
			u64 hz;

			if (of_property_read_u64(child, "opp-hz", &hz))
				continue;
			if (n < XFREQ_MAX_OPPS)
				x->freq[n++] = div_u64(hz, 1000);
		}
		of_node_put(opp_np);
	} else {
		/* operating-points = <kHz uV>... */
		val = of_get_property(owner, "operating-points", &len);
		if (!val)
			return -ENODEV;
		len /= 2 * sizeof(u32);
		for (; n < len && n < XFREQ_MAX_OPPS; n++, val += 2)
			x->freq[n] = be32_to_cpup(val);
	}
	x->nr_opps = n;
	return n ? 0 : -ENODEV;
}

static void __init kirin_xfreq_setup(struct device_node *np)
{
	struct clk_init_data init = { };
	struct device_node *owner;
	void __iomem *sctrl, *pmctrl;
	struct kirin_xfreq *x;
	const char *name;
	u32 scbak, ipchw;
	int idx;

	name = kirin_clk_name(np);
	sctrl = kirin_clk_base(KIRIN_SCTRL);
	pmctrl = kirin_clk_base(KIRIN_PMCTRL);
	if (!name || !sctrl || !pmctrl)
		return;

	x = kzalloc(sizeof(*x), GFP_KERNEL);
	if (!x)
		return;
	if (of_property_read_u32(np, "hisilicon,hi3xxx-xfreq-devid", &x->id) ||
	    of_property_read_u32(np, "hisilicon,hi3xxx-xfreq-scbakdata", &scbak) ||
	    of_property_read_u32_array(np, "hisilicon,get-rate-ipc-cmd",
				       x->get_cmd, KIRIN_LPM3_CMD_LEN) ||
	    of_property_read_u32_array(np, "hisilicon,set-rate-ipc-cmd",
				       x->set_cmd, KIRIN_LPM3_CMD_LEN)) {
		pr_err("kirin-clk: %s: incomplete xfreq description\n", name);
		goto err;
	}
	if (!of_property_read_u32(np, "hisilicon,hi3xxx-xfreq-ipchw", &ipchw))
		x->hw_vote = ipchw == 1;
	x->vote_mask = 0x00ff0000;
	of_property_read_u32(np, "hisilicon,hi3xxx-xfreq-mask", &x->vote_mask);
	x->scbak = sctrl + scbak;
	x->vote = pmctrl + scbak;
	if (of_property_read_string_index(np, "hisilicon,hw-vote-channel", 0,
					  &x->hv_channel) ||
	    of_property_read_string_index(np, "hisilicon,hw-vote-channel", 1,
					  &x->hv_src))
		x->hv_channel = NULL;

	owner = kirin_xfreq_opp_owner(np, x->id);
	if (owner) {
		if (kirin_xfreq_opps(x, owner))
			pr_warn("kirin-clk: %s: no OPP table in %pOF\n", name,
				owner);
		of_node_put(owner);
	}
	idx = kirin_xfreq_index(x);
	if (idx >= 0)
		x->rate = (unsigned long)x->freq[idx] * 1000;

	init.name = name;
	init.ops = &kirin_xfreq_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	x->hw.init = &init;
	if (!kirin_clk_register(np, &x->hw))
		return;
err:
	kfree(x);
}
CLK_OF_DECLARE(kirin_xfreq, "hisilicon,hi3xxx-xfreq-clk", kirin_xfreq_setup);

/* ---- "hisilicon,interactive-clk" -------------------------------------- */

struct kirin_mclk {
	struct clk_hw hw;
	u32 en_cmd[KIRIN_LPM3_CMD_LEN];
	u32 dis_cmd[KIRIN_LPM3_CMD_LEN];
	bool always_on;
};

#define to_kirin_mclk(_hw) container_of(_hw, struct kirin_mclk, hw)

static int kirin_mclk_prepare(struct clk_hw *hw)
{
	struct kirin_mclk *m = to_kirin_mclk(hw);
	int ret;

	ret = kirin_ipc_send(KIRIN_LPM3_MBOX, m->en_cmd, KIRIN_LPM3_CMD_LEN,
			     NULL, 0);
	if (ret)
		pr_err("kirin-clk: %s: LPM3 enable failed: %d\n",
		       clk_hw_get_name(hw), ret);
	return ret;
}

static void kirin_mclk_unprepare(struct clk_hw *hw)
{
	struct kirin_mclk *m = to_kirin_mclk(hw);
	int ret;

	if (m->always_on || kirin_clk_keep_on)
		return;
	ret = kirin_ipc_send(KIRIN_LPM3_MBOX, m->dis_cmd, KIRIN_LPM3_CMD_LEN,
			     NULL, 0);
	if (ret)
		pr_err("kirin-clk: %s: LPM3 disable failed: %d\n",
		       clk_hw_get_name(hw), ret);
}

static const struct clk_ops kirin_mclk_ops = {
	.prepare = kirin_mclk_prepare,
	.unprepare = kirin_mclk_unprepare,
};

static void __init kirin_mclk_setup(struct device_node *np)
{
	struct clk_init_data init = { };
	const char *name, *parent;
	struct kirin_mclk *m;

	name = kirin_clk_name(np);
	parent = of_clk_get_parent_name(np, 0);
	if (!name)
		return;
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return;
	if (of_property_read_u32_array(np, "hisilicon,ipc-lpm3-cmd-en",
				       m->en_cmd, KIRIN_LPM3_CMD_LEN) ||
	    of_property_read_u32_array(np, "hisilicon,ipc-lpm3-cmd-dis",
				       m->dis_cmd, KIRIN_LPM3_CMD_LEN)) {
		pr_err("kirin-clk: %s: no LPM3 commands\n", name);
		kfree(m);
		return;
	}
	m->always_on = of_property_read_bool(np, "always_on");

	init.name = name;
	init.ops = &kirin_mclk_ops;
	init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED;
	init.parent_names = parent ? &parent : NULL;
	init.num_parents = parent ? 1 : 0;
	m->hw.init = &init;
	if (kirin_clk_register(np, &m->hw))
		kfree(m);
}
CLK_OF_DECLARE(kirin_mclk, "hisilicon,interactive-clk", kirin_mclk_setup);
