// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 clock driver: register blocks, muxes, dividers and
 * gates described one per node in the firmware device tree.
 *
 * Ported from the Huawei vendor driver (drivers/clk/hisi/clk-kirin-common.c,
 * hisi-clkgate.c, 4.19; GPL-2.0).
 */

#include <linux/clk.h>
#include <linux/clk/kirin.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "clk-kirin.h"

DEFINE_SPINLOCK(kirin_clk_lock);

/*
 * Dropping the last Linux reference gates a clock (or PLL) in hardware; the
 * clocks the vendor kernel keeps running without a Linux user are pinned
 * below. Verified with the whole L410 (display takeover, GPU, audio, WiFi,
 * Bluetooth, USB, camera, UFS, I2C HID) on the integrated kernel. Boot with
 * "kirin_clk_keep_on" to never gate in hardware, as during bring-up (the
 * vendor CONFIG_HISI_CLK_ALWAYS_ON). "kirin_clk_gating" is accepted and is
 * the default now.
 */
bool kirin_clk_keep_on;
static int __init kirin_clk_gating_setup(char *s)
{
	kirin_clk_keep_on = false;
	return 1;
}
__setup("kirin_clk_gating", kirin_clk_gating_setup);

static int __init kirin_clk_keep_on_setup(char *s)
{
	kirin_clk_keep_on = true;
	return 1;
}
__setup("kirin_clk_keep_on", kirin_clk_keep_on_setup);

/*
 * With real gating on, these clocks are held enabled for good: they are the
 * clocks the vendor kernel keeps enabled in steady state (clk_summary of
 * 4.19.71-23 with the display, USB, PCIe, audio and I2C devices running).
 * Holding them (and so their parents) keeps shared bus/PLL/divider parents
 * running for hardware that the firmware set up and no Linux driver
 * accounts for yet. Drop an entry once the driver that owns the clock
 * holds it itself.
 */
static const char * const kirin_clk_pinned[] = {
	"clk_g3d",
	"clk_asp_pll_sel",
	"clk_asp_subsys",
	"aclk_pcie",
	"pclk_pcie_sys",
	"clk_gt_ioperi",
	"sc_div_ioperi",
	"clk_spi3",
	"clk_vivobus",
	"aclk_disp_noc_subsys",
	"aclk_dss",
	"clk_ap_ppll0",
	"sc_sel_320m_pll",
	"gt_clk_320m_pll",
	"sc_div_320m",
	"pclk_pcie_andgt",
	"pclk_pcie_div",
	"pclk_mmc1_pcie",
	"pclk_pcie_phy",
	"clkdiv_i2c",
	"clkmux_i2c",
	"clk_i2c7",
	"clk_i2c6_acpu",
	"clk_i2c4",
	"clk_i2c3",
	"clkgt_uartl",
	"clkdiv_uartl",
	"clkmux_uartl",
	"clk_uart2",
	"clkgt_uarth",
	"clkdiv_uarth",
	"clkmux_uarth",
	"pclk_uart4",
	"clk_mmc_usbdp_andgt",
	"div_mmc_usbdp",
	"clk_mmc_usbdp",
	"aclk_usb3otg",
	"hclk_usb3otg",
	"div_sysbus_pll",
	"pclk_disp_noc_subsys",
	"pclk_dss",
	"clk_dmac",
	"clk_ppll0_media",
	"aclk_mmbuf_sw",
	"clk_mmbuf_gt",
	"aclk_mmbuf_div",
	"clk_mmbuf",
	"clk_dss_axi_mm",
	"sel_edc0_pll",
	"clk_edc0_gt",
	"clk_edc0_div",
	"clk_edc0",
	"clk_blpwm",
	"clk_wd0_mux",
	"pclk_wd0",
	"clk_sys_ini",
	"clkmux_uart0",
	"clk_uart0",
	"pclk_uart0",
	"clk_pmuaudioclk",
	"clk_pcieaux",
	"clk_txdphy0_ref",
	"clk_txdphy0_cfg",
	"clk_usb3otg_ref",
	"sel_codeccssi",
	"clk_codecssi",
	"clkin_sys_div",
	"clk_usb2phy_ref_mux",
	"clk_usb2phy_ref",
	"clk_nfc",
	"clk_abb_192",
	"clk_usb_tcxo_en",
	"clk_abb_usb",
};

static const struct of_device_id kirin_crg_match[] = {
	{ .compatible = "hisilicon,clk-pmctrl", .data = (void *)KIRIN_PMCTRL },
	{ .compatible = "hisilicon,clk-sctrl", .data = (void *)KIRIN_SCTRL },
	{ .compatible = "hisilicon,clk-crgctrl", .data = (void *)KIRIN_CRGCTRL },
	{ .compatible = "hisilicon,hi6421pmic", .data = (void *)KIRIN_PMUCTRL },
	{ .compatible = "hisilicon,clk-pctrl", .data = (void *)KIRIN_PCTRL },
	{ .compatible = "hisilicon,media-crg", .data = (void *)KIRIN_MEDIACRG },
	{ .compatible = "hisilicon,iomcu-crg", .data = (void *)KIRIN_IOMCUCRG },
	{ .compatible = "hisilicon,media1-crg", .data = (void *)KIRIN_MEDIA1CRG },
	{ .compatible = "hisilicon,media2-crg", .data = (void *)KIRIN_MEDIA2CRG },
	{ .compatible = "hisilicon,mmc1-crg", .data = (void *)KIRIN_MMC1CRG },
	{ .compatible = "hisilicon,hsdt-crg", .data = (void *)KIRIN_HSDTCRG },
	{ .compatible = "hisilicon,mmc0-crg", .data = (void *)KIRIN_MMC0CRG },
	{ .compatible = "hisilicon,hsdt1-crg", .data = (void *)KIRIN_HSDT1CRG },
	{ }
};

/* the same register blocks as standalone system controller nodes */
static const char * const kirin_crg_syscon[KIRIN_CRG_MAX] = {
	[KIRIN_PMCTRL] = "hisilicon,pmctrl",
	[KIRIN_SCTRL] = "hisilicon,sysctrl",
	[KIRIN_CRGCTRL] = "hisilicon,crgctrl",
	[KIRIN_PCTRL] = "hisilicon,pctrl",
	[KIRIN_MEDIACRG] = "hisilicon,mediactrl",
	[KIRIN_IOMCUCRG] = "hisilicon,iomcuctrl",
	[KIRIN_MEDIA1CRG] = "hisilicon,media1ctrl",
	[KIRIN_MEDIA2CRG] = "hisilicon,media2ctrl",
	[KIRIN_MMC1CRG] = "hisilicon,mmc1_sysctrl",
	[KIRIN_HSDTCRG] = "hisilicon,hsdt_crg",
	[KIRIN_MMC0CRG] = "hisilicon,mmc0crg",
	[KIRIN_HSDT1CRG] = "hisilicon,hsdt1_crg",
};

static void __iomem *kirin_bases[KIRIN_CRG_MAX];

void __iomem *kirin_clk_base(enum kirin_crg type)
{
	struct device_node *np;

	if (type >= KIRIN_CRG_MAX)
		return NULL;
	if (kirin_bases[type])
		return kirin_bases[type];
	if (!kirin_crg_syscon[type])
		return NULL;
	np = of_find_compatible_node(NULL, NULL, kirin_crg_syscon[type]);
	if (!np) {
		pr_err("kirin-clk: no %s node\n", kirin_crg_syscon[type]);
		return NULL;
	}
	kirin_bases[type] = of_iomap(np, 0);
	of_node_put(np);
	return kirin_bases[type];
}

void __iomem *kirin_clk_parent_base(struct device_node *np)
{
	const struct of_device_id *match;
	struct device_node *parent;
	enum kirin_crg type;

	parent = of_get_parent(np);
	if (!parent)
		return NULL;
	match = of_match_node(kirin_crg_match, parent);
	if (!match) {
		pr_err("kirin-clk: %pOF: unknown register block\n", np);
		of_node_put(parent);
		return NULL;
	}
	type = (uintptr_t)match->data;
	if (!kirin_bases[type])
		kirin_bases[type] = of_iomap(parent, 0);
	of_node_put(parent);
	if (!kirin_bases[type])
		pr_err("kirin-clk: %pOF: cannot map registers\n", np);
	return kirin_bases[type];
}

const char *kirin_clk_name(struct device_node *np)
{
	const char *name;

	if (of_property_read_string(np, "clock-output-names", &name)) {
		pr_err("kirin-clk: %pOF: no clock-output-names\n", np);
		return NULL;
	}
	return name;
}

int kirin_clk_register(struct device_node *np, struct clk_hw *hw)
{
	const char *name = hw->init->name;
	int ret;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		pr_err("kirin-clk: %s: register failed: %d\n", name, ret);
		return ret;
	}
	ret = of_clk_add_hw_provider(np, of_clk_hw_simple_get, hw);
	if (ret)
		pr_err("kirin-clk: %s: add provider failed: %d\n", name, ret);
	/*
	 * Vendor drivers look clocks up by name (clk_get(NULL, "clk_xxx")).
	 * clkdev connection ids are limited to 15 characters.
	 */
	if (strlen(name) < 16)
		clk_hw_register_clkdev(hw, name, NULL);
	return 0;
}

struct clk *kirin_clk_friend(const char *name, struct clk **cache)
{
	if (!*cache && name)
		*cache = __clk_lookup(name);
	return *cache;
}

/* ---- peripheral voltage votes (provided by the power driver) ---------- */

static const struct kirin_clk_perivolt_ops *kirin_perivolt_ops;

void kirin_clk_set_perivolt_ops(const struct kirin_clk_perivolt_ops *ops)
{
	WRITE_ONCE(kirin_perivolt_ops, ops);
}
EXPORT_SYMBOL_GPL(kirin_clk_set_perivolt_ops);

int kirin_clk_perivolt_set(u32 id, u32 level)
{
	const struct kirin_clk_perivolt_ops *ops = READ_ONCE(kirin_perivolt_ops);

	int ret;

	if (!ops || !ops->set_volt)
		return 0;	/* not available yet: keep the firmware voltage */
	ret = ops->set_volt(id, level);
	return ret == -ENODEV ? 0 : ret;	/* no voter for this id */
}

/* ---- "hisilicon,hi3xxx-clk-mux" --------------------------------------- */

static void __init kirin_mux_setup(struct device_node *np)
{
	const char **parents;
	void __iomem *base;
	struct clk_hw *hw;
	const char *name;
	u32 rdata[2];
	int i, n;

	base = kirin_clk_parent_base(np);
	name = kirin_clk_name(np);
	if (!base || !name)
		return;
	if (of_property_read_u32_array(np, "hisilicon,clkmux-reg", rdata, 2) ||
	    !rdata[1]) {
		pr_err("kirin-clk: %s: bad hisilicon,clkmux-reg\n", name);
		return;
	}
	n = of_clk_get_parent_count(np);
	if (n <= 0)
		return;
	parents = kcalloc(n, sizeof(*parents), GFP_KERNEL);
	if (!parents)
		return;
	for (i = 0; i < n; i++) {
		parents[i] = of_clk_get_parent_name(np, i);
		if (!parents[i]) {
			pr_err("kirin-clk: %s: parent %d unknown\n", name, i);
			goto out;
		}
	}

	hw = clk_hw_register_mux_table(NULL, name, parents, n,
				       CLK_SET_RATE_PARENT,
				       base + rdata[0], ffs(rdata[1]) - 1,
				       rdata[1] >> (ffs(rdata[1]) - 1),
				       of_property_read_bool(np, "hiword") ?
				       CLK_MUX_HIWORD_MASK : 0,
				       NULL, &kirin_clk_lock);
	if (IS_ERR(hw)) {
		pr_err("kirin-clk: %s: register failed: %ld\n", name,
		       PTR_ERR(hw));
		goto out;
	}
	of_clk_add_hw_provider(np, of_clk_hw_simple_get, hw);
	if (strlen(name) < 16)
		clk_hw_register_clkdev(hw, name, NULL);
out:
	kfree(parents);
}
CLK_OF_DECLARE(kirin_mux, "hisilicon,hi3xxx-clk-mux", kirin_mux_setup);

/* ---- "hisilicon,hi3xxx-clk-div" --------------------------------------- */

static void __init kirin_div_setup(struct device_node *np)
{
	struct clk_div_table *table;
	unsigned int i, n, mult = 1;
	void __iomem *base;
	const char *name, *parent;
	struct clk_hw *hw;
	u32 reg[2], range[2];
	u8 shift, width;

	base = kirin_clk_parent_base(np);
	name = kirin_clk_name(np);
	parent = of_clk_get_parent_name(np, 0);
	if (!base || !name || !parent)
		return;
	if (of_property_read_u32_array(np, "hisilicon,clkdiv", reg, 2) ||
	    !reg[1] ||
	    of_property_read_u32_array(np, "hisilicon,clkdiv-table", range, 2) ||
	    (u8)range[1] > (u8)range[0]) {
		pr_err("kirin-clk: %s: bad divider description\n", name);
		return;
	}
	shift = ffs(reg[1]) - 1;
	width = fls(reg[1]) - shift;

	/* <max min>: register value i selects divider (min + i) */
	if (of_property_read_bool(np, "double_div"))
		mult = 2;
	n = (u8)range[0] - (u8)range[1] + 1;
	table = kcalloc(n + 1, sizeof(*table), GFP_KERNEL);
	if (!table)
		return;
	for (i = 0; i < n; i++) {
		table[i].val = i;
		table[i].div = ((u8)range[1] + i) * mult;
	}

	hw = clk_hw_register_divider_table(NULL, name, parent,
					   CLK_SET_RATE_PARENT,
					   base + reg[0], shift, width,
					   of_property_read_bool(np, "hiword") ?
					   CLK_DIVIDER_HIWORD_MASK : 0,
					   table, &kirin_clk_lock);
	if (IS_ERR(hw)) {
		pr_err("kirin-clk: %s: register failed: %ld\n", name,
		       PTR_ERR(hw));
		kfree(table);
		return;
	}
	of_clk_add_hw_provider(np, of_clk_hw_simple_get, hw);
	if (strlen(name) < 16)
		clk_hw_register_clkdev(hw, name, NULL);
}
CLK_OF_DECLARE(kirin_div, "hisilicon,hi3xxx-clk-div", kirin_div_setup);

/* ---- "hisilicon,clk-gate": plain (hiword) register bit ---------------- */

struct kirin_bitgate {
	struct clk_hw hw;
	void __iomem *reg;
	u8 bit;
	bool hiword;
	bool inverted;
	bool always_on;
};

#define to_kirin_bitgate(_hw) container_of(_hw, struct kirin_bitgate, hw)

static void kirin_bitgate_set(struct kirin_bitgate *g, bool on)
{
	unsigned long flags;
	u32 val;

	if (g->inverted)
		on = !on;
	if (g->hiword) {
		writel(BIT(g->bit + 16) | (on ? BIT(g->bit) : 0), g->reg);
		return;
	}
	spin_lock_irqsave(&kirin_clk_lock, flags);
	val = readl(g->reg);
	val = on ? val | BIT(g->bit) : val & ~BIT(g->bit);
	writel(val, g->reg);
	spin_unlock_irqrestore(&kirin_clk_lock, flags);
}

static int kirin_bitgate_enable(struct clk_hw *hw)
{
	kirin_bitgate_set(to_kirin_bitgate(hw), true);
	return 0;
}

static void kirin_bitgate_disable(struct clk_hw *hw)
{
	struct kirin_bitgate *g = to_kirin_bitgate(hw);

	if (!g->always_on && !kirin_clk_keep_on)
		kirin_bitgate_set(g, false);
}

static int kirin_bitgate_is_enabled(struct clk_hw *hw)
{
	struct kirin_bitgate *g = to_kirin_bitgate(hw);

	return !!(readl(g->reg) & BIT(g->bit)) ^ g->inverted;
}

static const struct clk_ops kirin_bitgate_ops = {
	.enable = kirin_bitgate_enable,
	.disable = kirin_bitgate_disable,
	.is_enabled = kirin_bitgate_is_enabled,
};

static void __init kirin_bitgate_setup(struct device_node *np)
{
	struct clk_init_data init = { };
	const char *name, *parent;
	struct kirin_bitgate *g;
	void __iomem *base;
	u32 gate[2];

	base = kirin_clk_parent_base(np);
	name = kirin_clk_name(np);
	parent = of_clk_get_parent_name(np, 0);
	if (!base || !name || !parent)
		return;
	if (of_property_read_u32_array(np, "hisilicon,clkgate", gate, 2) ||
	    gate[1] > 31) {
		pr_err("kirin-clk: %s: bad hisilicon,clkgate\n", name);
		return;
	}
	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return;
	if (of_property_read_bool(np, "pmu32khz"))
		gate[0] <<= 2;
	g->reg = base + gate[0];
	g->bit = gate[1];
	g->hiword = of_property_read_bool(np, "hiword");
	g->inverted = of_property_read_bool(np, "hisilicon,clkgate-inverted");
	g->always_on = of_property_read_bool(np, "always_on");
	if (g->hiword && g->bit > 15) {
		pr_err("kirin-clk: %s: hiword gate bit %u\n", name, g->bit);
		kfree(g);
		return;
	}

	init.name = name;
	init.ops = &kirin_bitgate_ops;
	init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED;
	init.parent_names = &parent;
	init.num_parents = 1;
	g->hw.init = &init;
	if (kirin_clk_register(np, &g->hw))
		kfree(g);
}
CLK_OF_DECLARE(kirin_bitgate, "hisilicon,clk-gate", kirin_bitgate_setup);

/* ---- "hisilicon,hi3xxx-clk-gate": PEREN/PERDIS register pairs ---------- */

#define PERDIS_OFFSET		0x4
#define GATE_SYNC_MAX_US	10
#define DVFS_MAX_FREQ_NUM	3

struct kirin_gate {
	struct clk_hw hw;
	void __iomem *enable;		/* NULL: no hardware gate */
	u32 ebits;
	u32 sync_time;
	bool always_on;
	const char *friend;
	struct clk *friend_clk;
	/* peripheral voltage vote while prepared */
	bool perivolt;
	u32 perivolt_id;
	u32 perivolt_level;
	u32 freq[DVFS_MAX_FREQ_NUM];	/* kHz */
	u32 volt[DVFS_MAX_FREQ_NUM + 1];
};

#define to_kirin_gate(_hw) container_of(_hw, struct kirin_gate, hw)

static u32 kirin_gate_volt(struct kirin_gate *g)
{
	unsigned long rate = clk_hw_get_rate(&g->hw);
	unsigned int i, level = g->perivolt_level;

	if (!g->freq[0])
		return g->volt[level];
	for (i = 0; i < level; i++)
		if (rate <= (unsigned long)g->freq[i] * 1000)
			return g->volt[i];
	return g->volt[level];
}

static int kirin_gate_prepare(struct clk_hw *hw)
{
	struct kirin_gate *g = to_kirin_gate(hw);
	struct clk *friend = kirin_clk_friend(g->friend, &g->friend_clk);
	int ret;

	if (g->friend) {
		if (!friend)
			return -EPROBE_DEFER;
		ret = clk_prepare(friend);
		if (ret)
			return ret;
	}
	if (g->perivolt) {
		ret = kirin_clk_perivolt_set(g->perivolt_id, kirin_gate_volt(g));
		if (ret)
			pr_err("kirin-clk: %s: voltage vote failed: %d\n",
			       clk_hw_get_name(hw), ret);
	}
	return 0;
}

static void kirin_gate_unprepare(struct clk_hw *hw)
{
	struct kirin_gate *g = to_kirin_gate(hw);

	if (g->perivolt)
		kirin_clk_perivolt_set(g->perivolt_id, 0);
	if (g->friend_clk)
		clk_unprepare(g->friend_clk);
}

static int kirin_gate_enable(struct clk_hw *hw)
{
	struct kirin_gate *g = to_kirin_gate(hw);
	int ret;

	if (g->sync_time)
		udelay(g->sync_time);
	if (g->enable)
		writel(g->ebits, g->enable);
	if (g->friend_clk) {
		ret = clk_enable(g->friend_clk);
		if (ret)
			return ret;
	}
	if (g->sync_time)
		udelay(g->sync_time);
	return 0;
}

static void kirin_gate_disable(struct clk_hw *hw)
{
	struct kirin_gate *g = to_kirin_gate(hw);

	if (g->enable && !g->always_on && !kirin_clk_keep_on)
		writel(g->ebits, g->enable + PERDIS_OFFSET);
	if (g->sync_time)
		udelay(g->sync_time);
	if (g->friend_clk)
		clk_disable(g->friend_clk);
}

static const struct clk_ops kirin_gate_ops = {
	.prepare = kirin_gate_prepare,
	.unprepare = kirin_gate_unprepare,
	.enable = kirin_gate_enable,
	.disable = kirin_gate_disable,
};

static void __init kirin_gate_setup(struct device_node *np)
{
	struct clk_init_data init = { };
	const char *name, *parent;
	void __iomem *base;
	struct kirin_gate *g;
	u32 gate[2], level = 0;

	base = kirin_clk_parent_base(np);
	name = kirin_clk_name(np);
	parent = of_clk_get_parent_name(np, 0);
	if (!base || !name || !parent)
		return;
	if (of_property_read_u32_array(np, "hisilicon,hi3xxx-clkgate", gate, 2)) {
		pr_err("kirin-clk: %s: no hisilicon,hi3xxx-clkgate\n", name);
		return;
	}

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return;
	g->enable = gate[1] ? base + gate[0] : NULL;
	g->ebits = gate[1];
	g->always_on = of_property_read_bool(np, "always_on");
	if (!of_property_read_u32(np, "gate_sync_time", &g->sync_time))
		g->sync_time = min_t(u32, g->sync_time, GATE_SYNC_MAX_US);
	if (of_property_read_string(np, "clock-friend-names", &g->friend))
		g->friend = NULL;

	if (of_property_read_bool(np, "peri_dvfs_sensitive")) {
		of_property_read_u32(np, "clock-id", &g->perivolt_id);
		of_property_read_u32(np, "hisilicon,clk-dvfs-level", &level);
		if (level <= DVFS_MAX_FREQ_NUM) {
			g->perivolt = true;
			g->perivolt_level = level;
			of_property_read_u32_array(np, "hisilicon,sensitive-freq",
						   g->freq, level);
			of_property_read_u32_array(np, "hisilicon,sensitive-volt",
						   g->volt, level + 1);
		}
	}

	init.name = name;
	init.ops = &kirin_gate_ops;
	init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED;
	init.parent_names = &parent;
	init.num_parents = 1;
	g->hw.init = &init;

	if (kirin_clk_register(np, &g->hw))
		kfree(g);
}
CLK_OF_DECLARE(kirin_gate, "hisilicon,hi3xxx-clk-gate", kirin_gate_setup);

/* ---- "hisilicon,clk-pmu-gate": clock outputs of the PMIC -------------- */

#define ABB_SCBAKDATA		0x43c	/* SCBAKDATA12: ABB clock votes */
#define ABB_VOTE_AP		BIT(0)
#define ABB_VOTE_LPM3		BIT(1)
#define ABB_HWLOCK_TIMEOUT_MS	1000
#define NO_HWLOCK		0xff

static struct regmap *kirin_pmic_map;
static LIST_HEAD(kirin_pmu_gates);
/* serialises the gate ops against the PMIC regmap hand-over */
static DEFINE_MUTEX(kirin_pmu_mutex);

struct kirin_pmu_gate {
	struct clk_hw hw;
	struct list_head node;
	u32 reg;
	u32 bit;
	u32 hwlock_id;
	bool always_on;
	bool abb;			/* shared with LPM3 through SCBAKDATA12 */
	bool on;			/* prepared by Linux */
	struct hwspinlock *hwlock;
	void __iomem *sctrl;
};

#define to_kirin_pmu_gate(_hw) container_of(_hw, struct kirin_pmu_gate, hw)

/* call with kirin_pmu_mutex held */
static int kirin_pmu_write(struct kirin_pmu_gate *g, bool on)
{
	if (!kirin_pmic_map)
		return 0;	/* applied in kirin_clk_set_pmic_regmap() */
	return regmap_update_bits(kirin_pmic_map, g->reg, BIT(g->bit),
				  on ? BIT(g->bit) : 0);
}

/* ABB clock votes are shared with LPM3 under hardware spinlock 9 */
static bool kirin_abb_lock(struct kirin_pmu_gate *g)
{
	if (!g->hwlock)
		g->hwlock = hwspin_lock_request_specific(g->hwlock_id);
	if (!g->hwlock) {
		pr_warn_once("kirin-clk: %s: hwspinlock %u unavailable, voting unlocked\n",
			     clk_hw_get_name(&g->hw), g->hwlock_id);
		return false;
	}
	if (hwspin_lock_timeout(g->hwlock, ABB_HWLOCK_TIMEOUT_MS)) {
		pr_warn("kirin-clk: %s: hwspinlock timeout\n",
			clk_hw_get_name(&g->hw));
		return false;
	}
	return true;
}

static void kirin_abb_vote(struct kirin_pmu_gate *g, bool on)
{
	bool locked = kirin_abb_lock(g);
	u32 val = readl(g->sctrl);

	if (on) {
		/* nobody else votes: switch the PMIC clock on */
		if (!(val & ABB_VOTE_LPM3))
			kirin_pmu_write(g, true);
		writel(val | ABB_VOTE_AP, g->sctrl);
	} else if (val & ABB_VOTE_AP) {
		if (!(val & ABB_VOTE_LPM3) && !g->always_on &&
		    !kirin_clk_keep_on)
			kirin_pmu_write(g, false);
		writel(val & ~ABB_VOTE_AP, g->sctrl);
	}
	if (locked)
		hwspin_unlock(g->hwlock);
}

static int kirin_pmu_gate_prepare(struct clk_hw *hw)
{
	struct kirin_pmu_gate *g = to_kirin_pmu_gate(hw);

	mutex_lock(&kirin_pmu_mutex);
	g->on = true;
	if (g->abb)
		kirin_abb_vote(g, true);
	else
		kirin_pmu_write(g, true);
	if (!kirin_pmic_map)
		pr_info("kirin-clk: %s: enabled once the PMIC driver is up\n",
			clk_hw_get_name(hw));
	mutex_unlock(&kirin_pmu_mutex);
	if (g->abb)
		mdelay(1);
	return 0;
}

static void kirin_pmu_gate_unprepare(struct clk_hw *hw)
{
	struct kirin_pmu_gate *g = to_kirin_pmu_gate(hw);

	mutex_lock(&kirin_pmu_mutex);
	g->on = false;
	if (g->abb)
		kirin_abb_vote(g, false);
	else if (!g->always_on && !kirin_clk_keep_on)
		kirin_pmu_write(g, false);
	mutex_unlock(&kirin_pmu_mutex);
}

/*
 * Called by the PMIC driver. Gates prepared before the PMIC was available
 * get their enable bit written now.
 */
void kirin_clk_set_pmic_regmap(struct regmap *map)
{
	struct kirin_pmu_gate *g;

	mutex_lock(&kirin_pmu_mutex);
	kirin_pmic_map = map;
	list_for_each_entry(g, &kirin_pmu_gates, node) {
		if (!g->on || !map)
			continue;
		if (g->abb)
			kirin_abb_vote(g, true);
		else
			kirin_pmu_write(g, true);
		pr_info("kirin-clk: %s: enabled in the PMIC\n",
			clk_hw_get_name(&g->hw));
	}
	mutex_unlock(&kirin_pmu_mutex);
}
EXPORT_SYMBOL_GPL(kirin_clk_set_pmic_regmap);

static const struct clk_ops kirin_pmu_gate_ops = {
	.prepare = kirin_pmu_gate_prepare,
	.unprepare = kirin_pmu_gate_unprepare,
};

static void __init kirin_pmu_gate_setup(struct device_node *np)
{
	struct clk_init_data init = { };
	const char *name, *parent;
	struct kirin_pmu_gate *g;
	u32 gate[2];

	name = kirin_clk_name(np);
	parent = of_clk_get_parent_name(np, 0);
	if (!name || !parent)
		return;
	if (of_property_read_u32_array(np, "hisilicon,clkgate", gate, 2) ||
	    gate[1] > 7) {
		pr_err("kirin-clk: %s: bad hisilicon,clkgate\n", name);
		return;
	}
	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return;
	g->reg = gate[0];
	g->bit = gate[1];
	g->always_on = of_property_read_bool(np, "always_on");
	if (of_property_read_u32(np, "hwspinlock-id", &g->hwlock_id))
		g->hwlock_id = NO_HWLOCK;
	g->abb = !strcmp(name, "clk_abb_192") && g->hwlock_id != NO_HWLOCK;
	if (g->abb) {
		g->sctrl = kirin_clk_base(KIRIN_SCTRL);
		if (!g->sctrl) {
			kfree(g);
			return;
		}
		g->sctrl += ABB_SCBAKDATA;
	}

	init.name = name;
	init.ops = &kirin_pmu_gate_ops;
	init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED;
	init.parent_names = &parent;
	init.num_parents = 1;
	g->hw.init = &init;

	if (kirin_clk_register(np, &g->hw)) {
		kfree(g);
		return;
	}
	mutex_lock(&kirin_pmu_mutex);
	list_add_tail(&g->node, &kirin_pmu_gates);
	mutex_unlock(&kirin_pmu_mutex);
}
CLK_OF_DECLARE(kirin_pmu_gate, "hisilicon,clk-pmu-gate", kirin_pmu_gate_setup);

/* ---- boot report ------------------------------------------------------- */

static bool kirin_clk_dump;
static int __init kirin_clk_dump_setup(char *s)
{
	kirin_clk_dump = true;
	return 1;
}
__setup("kirin_clk_dump", kirin_clk_dump_setup);

/*
 * Check that every clock node of the firmware DT resolves, and report
 * orphans (clocks whose parent never registered). With "kirin_clk_dump" on
 * the command line, also log the rate of every clock.
 */
static int __init kirin_clk_report(void)
{
	struct device_node *root, *np;
	unsigned int total = 0, missing = 0, orphans = 0;

	root = of_find_node_by_path("/clocks@0");
	if (!root)
		return 0;

	for_each_node_with_property(np, "#clock-cells") {
		struct of_phandle_args spec = { .np = np };
		struct device_node *p;
		struct clk_hw *hw;
		struct clk *clk;
		bool under = false;

		for (p = of_get_parent(np); p; p = of_get_next_parent(p))
			if (p == root)
				under = true;
		if (!under || !of_device_is_available(np))
			continue;
		total++;
		clk = of_clk_get_from_provider(&spec);
		if (IS_ERR(clk)) {
			missing++;
			pr_warn("kirin-clk: %pOF: no provider (%ld)\n", np,
				PTR_ERR(clk));
			continue;
		}
		hw = __clk_get_hw(clk);
		if (clk_hw_get_num_parents(hw) && !clk_hw_get_parent(hw)) {
			orphans++;
			pr_warn("kirin-clk: %s: orphan\n", clk_hw_get_name(hw));
		}
		if (kirin_clk_dump)
			pr_info("kirin-clk: %-24s %10lu %s\n", clk_hw_get_name(hw),
				clk_get_rate(clk),
				clk_hw_get_parent(hw) ?
				clk_hw_get_name(clk_hw_get_parent(hw)) : "-");
		clk_put(clk);
	}
	of_node_put(root);
	pr_info("kirin-clk: %u clock nodes, %u without provider, %u orphans\n",
		total, missing, orphans);
	return 0;
}
late_initcall_sync(kirin_clk_report);

/*
 * serial_core touches the registers of ports that are not open (uart_proc_show
 * -> get_mctrl for /proc/tty/driver/ttyAMA), and the PL011 driver has no pm
 * op to clock the port for it: with its functional clock gated the read is a
 * synchronous external abort on this SoC. hi110x also opens and closes the
 * BUART (uart4) on every Bluetooth sleep / wake. Keep the clocks of every
 * enabled PL011 running; they cost next to nothing.
 */
static unsigned int __init kirin_clk_pin_uarts(void)
{
	struct device_node *np;
	unsigned int n = 0;
	int i, nr;

	for_each_compatible_node(np, NULL, "arm,pl011") {
		if (!of_device_is_available(np))
			continue;
		nr = of_count_phandle_with_args(np, "clocks", "#clock-cells");
		for (i = 0; i < nr; i++) {
			struct clk *clk = of_clk_get(np, i);

			if (IS_ERR(clk))
				continue;
			if (clk_prepare_enable(clk)) {
				clk_put(clk);
				continue;
			}
			n++;	/* reference kept forever */
		}
	}
	return n;
}

static int __init kirin_clk_pin(void)
{
	struct device_node *root, *np;
	unsigned int i, pinned = 0;

	if (kirin_clk_keep_on)
		return 0;
	root = of_find_node_by_path("/clocks@0");
	if (!root)
		return 0;

	for_each_node_with_property(np, "#clock-cells") {
		struct of_phandle_args spec = { .np = np };
		struct device_node *p;
		const char *name;
		bool under = false;
		struct clk *clk;

		if (of_property_read_string(np, "clock-output-names", &name))
			continue;
		for (i = 0; i < ARRAY_SIZE(kirin_clk_pinned); i++)
			if (!strcmp(name, kirin_clk_pinned[i]))
				break;
		if (i == ARRAY_SIZE(kirin_clk_pinned))
			continue;
		for (p = of_get_parent(np); p; p = of_get_next_parent(p))
			if (p == root)
				under = true;
		if (!under)
			continue;
		clk = of_clk_get_from_provider(&spec);
		if (IS_ERR(clk) || clk_prepare_enable(clk)) {
			pr_warn("kirin-clk: %s: cannot pin\n", name);
			continue;
		}
		pinned++;	/* reference kept forever */
	}
	of_node_put(root);
	pr_info("kirin-clk: gating on, %u of %zu clocks pinned, %u UART clocks\n",
		pinned, ARRAY_SIZE(kirin_clk_pinned), kirin_clk_pin_uarts());
	return 0;
}
/* once IPC and hwspinlock are up, before the device drivers probe */
subsys_initcall_sync(kirin_clk_pin);
