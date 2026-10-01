// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 IP power domains ("IP regulators")
 *
 * The media, display, video codec, NPU, GPU and audio subsystems of the
 * Kirin 990 are powered by the trusted firmware or by the LPM3 power
 * controller. The vendor firmware device tree describes each of them as a
 * regulator below "hisilicon,hisi_regulator_ip_atf_core":
 *
 *   ip-regulator-atf  power on/off through a SiP call:
 *                     x0 = 0xc500fff0, x1 = hisilicon,hisi-regulator-id,
 *                     x2 = 1 (on) / 0 (off)
 *                     optional clocks enabled while the domain is on
 *                     ("hisilicon,hisi-need-to-enable-clock") or lowered to
 *                     "hisilicon,hisi-clock-rate-set" around the power on
 *                     ("hisilicon,hisi-clock-rate-set-flag")
 *   ip-regulator-lpm  power on/off through a two word IPC message to LPM3
 *                     (ip_to_lpm_enable_step / ip_to_lpm_disable_step)
 *
 * "hisilicon,hisi-regulator-is-fake" domains only track their state.
 * Domains shared with the sensor hub / TEE (media1, vivobus, dss) take a
 * vote in SCTRL ("hisilicon,hisi-need-to-hwlock" = <hwlock offset>) and are
 * only switched when no other master voted for them.
 *
 * The state of a domain is not readable, so like the vendor kernel the
 * driver reports every domain off until it is enabled, except that the DSS
 * domain counts as on when the boot firmware left it out of reset.
 *
 * Based on the Huawei vendor drivers (drivers/regulator/ip_regulator.c,
 * drivers/mfd/ip_core.c).
 */

#include <linux/arm-smccc.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#ifdef CONFIG_KIRIN_IPC_MBOX
#include <linux/mailbox/kirin-ipc.h>
#endif

#define IP_REGULATOR_SMC_FN		0xc500fff0
#define IP_REGULATOR_LPM_MBOX		"HISI_ACPU_LPM3_MBX_2"
#define IP_REGULATOR_LPM_WORDS		2
#define IP_REGULATOR_HWLOCK_TIMEOUT_MS	1000

/* Kirin 990 ("Phoenix") domain ids, hisilicon,hisi-regulator-id */
#define IP_ID_VIVOBUS			0
#define IP_ID_DSSSUBSYS			2
#define IP_ID_MEDIA1_SUBSYS		10

/*
 * SCTRL power vote register (offset from hisilicon,hisi-need-to-hwlock):
 *   bit 0/1   AP / sensor hub vote for media1
 *   bit 2/3   AP / sensor hub vote for vivobus
 *   bit 4/5   AP / sensor hub vote for dss
 *   bit 30/31 TEE vote for media1 / vivobus
 */
#define VOTE_AP_MEDIA1		BIT(0)
#define VOTE_IOMCU_MEDIA1	BIT(1)
#define VOTE_AP_VIVOBUS		BIT(2)
#define VOTE_IOMCU_VIVOBUS	BIT(3)
#define VOTE_AP_DSS		BIT(4)
#define VOTE_IOMCU_DSS		BIT(5)
#define VOTE_TEE_MEDIA1		BIT(30)
#define VOTE_TEE_VIVOBUS	BIT(31)

struct hisi_ip_core {
	struct device *dev;
	void __iomem *base;		/* media1 CRG, for the DSS boot check */
	void __iomem *sctrl;		/* power vote register */
	struct hwspinlock *hwlock;
	u32 hwlock_id;
	struct mutex vote_lock;		/* vote read-modify-write */
};

struct hisi_ip_regulator {
	struct regulator_desc desc;
	struct hisi_ip_core *core;
	struct device *dev;
	u32 id;
	bool fake;
	bool lpm;
	bool enabled;

	/* ATF domains */
	struct clk_bulk_data *clks;
	int nclks;
	bool enable_clocks;
	u32 *low_rates;			/* 0: leave that clock alone */
	unsigned long *saved_rates;
	bool vote;
	u32 vote_offset;
	bool dss_check;
	u32 dss_check_reg;
	u32 dss_check_mask;

	/* LPM domains */
	u32 lpm_on[IP_REGULATOR_LPM_WORDS];
	u32 lpm_off[IP_REGULATOR_LPM_WORDS];
};

static int hisi_ip_smc(u32 id, bool on)
{
	struct arm_smccc_res res;

	arm_smccc_smc(IP_REGULATOR_SMC_FN, id, on, 0, 0, 0, 0, 0, &res);
	return res.a0 ? -EIO : 0;
}

/* returns the (AP bit, other masters' bits) for a voting domain */
static void hisi_ip_vote_bits(u32 id, u32 *ap, u32 *others)
{
	switch (id) {
	case IP_ID_VIVOBUS:
		*ap = VOTE_AP_VIVOBUS;
		*others = VOTE_IOMCU_VIVOBUS | VOTE_TEE_VIVOBUS;
		break;
	case IP_ID_DSSSUBSYS:
		*ap = VOTE_AP_DSS;
		*others = VOTE_IOMCU_DSS;
		break;
	default:		/* media1 */
		*ap = VOTE_AP_MEDIA1;
		*others = VOTE_IOMCU_MEDIA1 | VOTE_TEE_MEDIA1;
		break;
	}
}

static int hisi_ip_hwlock(struct hisi_ip_core *core)
{
	if (!core->hwlock && core->hwlock_id) {
		core->hwlock = hwspin_lock_request_specific(core->hwlock_id);
		if (IS_ERR(core->hwlock))
			core->hwlock = NULL;
		if (!core->hwlock)
			dev_warn_once(core->dev,
				      "hwspinlock %u unavailable, voting unlocked\n",
				      core->hwlock_id);
	}

	if (core->hwlock)
		return hwspin_lock_timeout(core->hwlock, IP_REGULATOR_HWLOCK_TIMEOUT_MS);

	return 0;
}

static void hisi_ip_hwunlock(struct hisi_ip_core *core)
{
	if (core->hwlock)
		hwspin_unlock(core->hwlock);
}

/*
 * Power a voting domain: record the AP vote and only talk to the firmware
 * when no other master holds the domain.
 */
static int hisi_ip_vote_power(struct hisi_ip_regulator *ipr, bool on)
{
	struct hisi_ip_core *core = ipr->core;
	void __iomem *reg = core->sctrl + ipr->vote_offset;
	u32 ap, others, val;
	int ret;

	hisi_ip_vote_bits(ipr->id, &ap, &others);

	mutex_lock(&core->vote_lock);
	ret = hisi_ip_hwlock(core);
	if (ret) {
		dev_err(ipr->dev, "vote hwspinlock timeout\n");
		goto unlock;
	}

	val = readl(reg);
	if (!(val & others))
		ret = hisi_ip_smc(ipr->id, on);
	if (!ret)
		writel(on ? val | ap : val & ~ap, reg);

	hisi_ip_hwunlock(core);
unlock:
	mutex_unlock(&core->vote_lock);
	return ret;
}

static bool hisi_ip_dss_left_on(struct hisi_ip_regulator *ipr)
{
	return ipr->dss_check &&
	       !(readl(ipr->core->base + ipr->dss_check_reg) & ipr->dss_check_mask);
}

static int hisi_ip_set_low_rates(struct hisi_ip_regulator *ipr)
{
	int i, ret;

	for (i = 0; i < ipr->nclks; i++) {
		if (!ipr->low_rates[i])
			continue;
		ipr->saved_rates[i] = clk_get_rate(ipr->clks[i].clk);
		ret = clk_set_rate(ipr->clks[i].clk, ipr->low_rates[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static void hisi_ip_restore_rates(struct hisi_ip_regulator *ipr)
{
	int i;

	for (i = 0; i < ipr->nclks; i++)
		if (ipr->low_rates[i] && ipr->saved_rates[i])
			clk_set_rate(ipr->clks[i].clk, ipr->saved_rates[i]);
}

static int hisi_ip_atf_enable(struct regulator_dev *rdev)
{
	struct hisi_ip_regulator *ipr = rdev_get_drvdata(rdev);
	int ret;

	if (ipr->fake)
		goto done;

	/* the boot firmware drives the display: nothing to do */
	if (hisi_ip_dss_left_on(ipr))
		goto done;

	if (ipr->low_rates) {
		ret = hisi_ip_set_low_rates(ipr);
		if (ret)
			dev_warn(ipr->dev, "cannot lower clocks: %d\n", ret);
	}

	if (ipr->enable_clocks) {
		ret = clk_bulk_prepare_enable(ipr->nclks, ipr->clks);
		if (ret)
			goto restore;
	}

	ret = ipr->vote ? hisi_ip_vote_power(ipr, true) : hisi_ip_smc(ipr->id, true);
	if (ret) {
		dev_err(ipr->dev, "firmware power on failed: %d\n", ret);
		if (ipr->enable_clocks)
			clk_bulk_disable_unprepare(ipr->nclks, ipr->clks);
	}

restore:
	if (ipr->low_rates)
		hisi_ip_restore_rates(ipr);
	if (ret)
		return ret;
done:
	ipr->enabled = true;
	return 0;
}

static int hisi_ip_atf_disable(struct regulator_dev *rdev)
{
	struct hisi_ip_regulator *ipr = rdev_get_drvdata(rdev);
	int ret;

	if (!ipr->fake) {
		ret = ipr->vote ? hisi_ip_vote_power(ipr, false) :
				  hisi_ip_smc(ipr->id, false);
		if (ret) {
			dev_err(ipr->dev, "firmware power off failed: %d\n", ret);
			return ret;
		}
		if (ipr->enable_clocks)
			clk_bulk_disable_unprepare(ipr->nclks, ipr->clks);
	}

	ipr->enabled = false;
	return 0;
}

static int hisi_ip_is_enabled(struct regulator_dev *rdev)
{
	struct hisi_ip_regulator *ipr = rdev_get_drvdata(rdev);

	return ipr->enabled;
}

static int hisi_ip_lpm_send(struct hisi_ip_regulator *ipr, const u32 *msg)
{
#ifdef CONFIG_KIRIN_IPC_MBOX
	u32 ack[IP_REGULATOR_LPM_WORDS] = { };
	int ret;

	ret = kirin_ipc_send(IP_REGULATOR_LPM_MBOX, msg, IP_REGULATOR_LPM_WORDS,
			     ack, IP_REGULATOR_LPM_WORDS);
	if (ret)
		return ret;

	/* LPM3 echoes the command, bits 31:24 of the second word are its status */
	if (ack[0] != msg[0] || ack[1] >> 24) {
		dev_err(ipr->dev, "LPM3 refused 0x%08x: ack 0x%08x 0x%08x\n",
			msg[0], ack[0], ack[1]);
		return -EIO;
	}
	return 0;
#else
	return -ENODEV;
#endif
}

static int hisi_ip_lpm_enable(struct regulator_dev *rdev)
{
	struct hisi_ip_regulator *ipr = rdev_get_drvdata(rdev);
	int ret;

	if (!ipr->fake) {
		ret = hisi_ip_lpm_send(ipr, ipr->lpm_on);
		if (ret) {
			dev_err(ipr->dev, "LPM3 power on failed: %d\n", ret);
			return ret;
		}
	}

	ipr->enabled = true;
	return 0;
}

static int hisi_ip_lpm_disable(struct regulator_dev *rdev)
{
	struct hisi_ip_regulator *ipr = rdev_get_drvdata(rdev);
	int ret;

	if (!ipr->fake) {
		ret = hisi_ip_lpm_send(ipr, ipr->lpm_off);
		/* the vendor kernel treats a timeout as "off" as well */
		if (ret && ret != -ETIMEDOUT) {
			dev_err(ipr->dev, "LPM3 power off failed: %d\n", ret);
			return ret;
		}
	}

	ipr->enabled = false;
	return 0;
}

static const struct regulator_ops hisi_ip_atf_ops = {
	.enable		= hisi_ip_atf_enable,
	.disable	= hisi_ip_atf_disable,
	.is_enabled	= hisi_ip_is_enabled,
};

static const struct regulator_ops hisi_ip_lpm_ops = {
	.enable		= hisi_ip_lpm_enable,
	.disable	= hisi_ip_lpm_disable,
	.is_enabled	= hisi_ip_is_enabled,
};

static int hisi_ip_parse_atf(struct hisi_ip_regulator *ipr)
{
	struct device *dev = ipr->dev;
	struct device_node *np = dev->of_node;
	struct hisi_ip_core *core = ipr->core;
	u32 val, pair[2];
	int ret;

	if (ipr->fake)
		return 0;

	ret = devm_clk_bulk_get_all(dev, &ipr->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot get clocks\n");
	ipr->nclks = ret;

	ipr->enable_clocks = !of_property_read_u32(np, "hisilicon,hisi-need-to-enable-clock",
						  &val) && val && ipr->nclks;

	if (!of_property_read_u32(np, "hisilicon,hisi-clock-rate-set-flag", &val) &&
	    val && ipr->nclks) {
		ipr->low_rates = devm_kcalloc(dev, ipr->nclks, sizeof(u32), GFP_KERNEL);
		ipr->saved_rates = devm_kcalloc(dev, ipr->nclks, sizeof(unsigned long),
						GFP_KERNEL);
		if (!ipr->low_rates || !ipr->saved_rates)
			return -ENOMEM;
		of_property_read_u32_array(np, "hisilicon,hisi-clock-rate-set",
					   ipr->low_rates, ipr->nclks);
	}

	if (!of_property_read_u32_array(np, "hisilicon,hisi-need-to-hwlock", pair, 2) &&
	    pair[0]) {
		if (!core->sctrl)
			return dev_err_probe(dev, -ENODEV, "vote needs SCTRL\n");
		mutex_lock(&core->vote_lock);
		core->hwlock_id = pair[0];
		mutex_unlock(&core->vote_lock);
		ipr->vote = true;
		ipr->vote_offset = pair[1];
	}

	if (ipr->id == IP_ID_DSSSUBSYS &&
	    !of_property_read_u32_array(np, "hisilicon,hisi-regulator-dss-boot-check",
					pair, 2) && core->base) {
		ipr->dss_check = true;
		ipr->dss_check_reg = pair[0];
		ipr->dss_check_mask = pair[1];
	}

	return 0;
}

static int hisi_ip_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regulator_config config = { };
	struct hisi_ip_regulator *ipr;
	struct regulator_dev *rdev;
	u32 val;
	int ret;

	ipr = devm_kzalloc(dev, sizeof(*ipr), GFP_KERNEL);
	if (!ipr)
		return -ENOMEM;

	ipr->dev = dev;
	ipr->core = dev_get_drvdata(dev->parent);
	if (!ipr->core)
		return -EPROBE_DEFER;

	if (of_property_read_u32(np, "hisilicon,hisi-regulator-id", &ipr->id))
		return dev_err_probe(dev, -EINVAL, "no regulator id\n");
	ipr->fake = !of_property_read_u32(np, "hisilicon,hisi-regulator-is-fake", &val) && val;
	ipr->lpm = of_device_is_compatible(np, "ip-regulator-lpm");

	if (ipr->lpm) {
		if (of_property_read_u32_array(np, "ip_to_lpm_enable_step",
					       ipr->lpm_on, IP_REGULATOR_LPM_WORDS) ||
		    of_property_read_u32_array(np, "ip_to_lpm_disable_step",
					       ipr->lpm_off, IP_REGULATOR_LPM_WORDS))
			return dev_err_probe(dev, -EINVAL, "no LPM3 commands\n");
		ipr->desc.ops = &hisi_ip_lpm_ops;
	} else {
		ret = hisi_ip_parse_atf(ipr);
		if (ret)
			return ret;
		ipr->desc.ops = &hisi_ip_atf_ops;
	}

	ipr->desc.name = of_get_property(np, "regulator-name", NULL);
	if (!ipr->desc.name)
		ipr->desc.name = np->name;
	/* "<name>-supply" in the node names the parent domain */
	ipr->desc.supply_name = of_get_property(np, "hisilicon,supply_name", NULL);
	ipr->desc.type = REGULATOR_VOLTAGE;
	ipr->desc.owner = THIS_MODULE;

	config.dev = dev;
	config.of_node = np;
	config.driver_data = ipr;
	config.init_data = of_get_regulator_init_data(dev, np, &ipr->desc);
	if (!config.init_data)
		return -ENOMEM;

	rdev = devm_regulator_register(dev, &ipr->desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "cannot register %s\n",
				     ipr->desc.name);

	return 0;
}

static const struct of_device_id hisi_ip_regulator_of_match[] = {
	{ .compatible = "ip-regulator-atf" },
	{ .compatible = "ip-regulator-lpm" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_ip_regulator_of_match);

static struct platform_driver hisi_ip_regulator_driver = {
	.driver = {
		.name		= "hisi-ip-regulator",
		.of_match_table	= hisi_ip_regulator_of_match,
	},
	.probe	= hisi_ip_regulator_probe,
};

static void hisi_ip_core_release(void *data)
{
	struct hisi_ip_core *core = data;

	if (core->hwlock)
		hwspin_lock_free(core->hwlock);
	if (core->sctrl)
		iounmap(core->sctrl);
}

static int hisi_ip_core_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *sctrl_np;
	struct hisi_ip_core *core;
	struct resource *res;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	mutex_init(&core->vote_lock);

	/* shared with the clock driver: map, do not claim */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		core->base = devm_ioremap(dev, res->start, resource_size(res));

	sctrl_np = of_find_compatible_node(NULL, NULL, "hisilicon,sysctrl");
	if (sctrl_np) {
		core->sctrl = of_iomap(sctrl_np, 0);
		of_node_put(sctrl_np);
	}
	if (!core->sctrl)
		dev_warn(dev, "no SCTRL, voting domains unavailable\n");

	ret = devm_add_action_or_reset(dev, hisi_ip_core_release, core);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, core);

	return devm_of_platform_populate(dev);
}

static const struct of_device_id hisi_ip_core_of_match[] = {
	{ .compatible = "hisilicon,hisi_regulator_ip_atf_core" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_ip_core_of_match);

static struct platform_driver hisi_ip_core_driver = {
	.driver = {
		.name		= "hisi-ip-regulator-core",
		.of_match_table	= hisi_ip_core_of_match,
	},
	.probe	= hisi_ip_core_probe,
};

static struct platform_driver * const hisi_ip_drivers[] = {
	&hisi_ip_core_driver,
	&hisi_ip_regulator_driver,
};

static int __init hisi_ip_regulator_init(void)
{
	return platform_register_drivers(hisi_ip_drivers, ARRAY_SIZE(hisi_ip_drivers));
}
subsys_initcall(hisi_ip_regulator_init);

static void __exit hisi_ip_regulator_exit(void)
{
	platform_unregister_drivers(hisi_ip_drivers, ARRAY_SIZE(hisi_ip_drivers));
}
module_exit(hisi_ip_regulator_exit);

MODULE_DESCRIPTION("HiSilicon Kirin 990 IP power domains");
MODULE_LICENSE("GPL");
