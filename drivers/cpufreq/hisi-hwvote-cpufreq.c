// SPDX-License-Identifier: GPL-2.0
/*
 * CPU frequency scaling for the HiSilicon Kirin 990 (Huawei Qingyun L410)
 *
 * The LPM3 power controller performs the DVFS of the three CPU clusters
 * (clock, voltage and AVS). Linux only votes a target frequency in PMCTRL
 * (see drivers/soc/hisilicon/kirin-hw-vote.c) and reads back the frequency
 * LPM3 granted. The vendor cpu nodes name their vote channel and voter:
 *
 *   cpu@0 { freq-vote-channel = "little-freq", "vote-src-1";
 *           operating-points-v2 = <&opp_table0>; };
 *
 * Based on the Huawei vendor driver (drivers/cpufreq/hisi/hisi_cpufreq_dt.c).
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/soc/hisilicon/kirin-hw-vote.h>

#define HWVOTE_KHZ_PER_MHZ	1000

struct hwvote_policy {
	struct kirin_hv *hv;
	int opp_token;
};

/*
 * All operating points of the firmware tables carry opp-supported-hw = <3>;
 * the vendor kernel enables bit 0 for "HUAWEI Kirin 990".
 */
static const u32 hwvote_supported_hw = BIT(0);

static struct device *hwvote_dev;

static int hwvote_cpufreq_target_index(struct cpufreq_policy *policy,
				       unsigned int index)
{
	struct hwvote_policy *hp = policy->driver_data;

	kirin_hv_set(hp->hv, policy->freq_table[index].frequency / HWVOTE_KHZ_PER_MHZ);
	return 0;
}

static unsigned int hwvote_cpufreq_get(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct hwvote_policy *hp;

	if (!policy || !policy->driver_data)
		return 0;

	hp = policy->driver_data;
	return kirin_hv_get_result(hp->hv) * HWVOTE_KHZ_PER_MHZ;
}

static int hwvote_cpufreq_init(struct cpufreq_policy *policy)
{
	struct dev_pm_opp_config config = {
		.supported_hw = &hwvote_supported_hw,
		.supported_hw_count = 1,
	};
	struct cpufreq_frequency_table *table;
	const char *channel, *src;
	struct hwvote_policy *hp;
	struct device *cpu_dev;
	unsigned int latency;
	int ret, n;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev)
		return -ENODEV;

	if (of_property_read_string_index(cpu_dev->of_node, "freq-vote-channel", 0, &channel) ||
	    of_property_read_string_index(cpu_dev->of_node, "freq-vote-channel", 1, &src)) {
		dev_err(hwvote_dev, "cpu%u: no freq-vote-channel\n", policy->cpu);
		return -ENODEV;
	}

	hp = kzalloc(sizeof(*hp), GFP_KERNEL);
	if (!hp)
		return -ENOMEM;

	hp->hv = kirin_hv_get(channel, src);
	if (IS_ERR(hp->hv)) {
		ret = PTR_ERR(hp->hv);
		dev_err(hwvote_dev, "cpu%u: vote channel %s/%s: %d\n",
			policy->cpu, channel, src, ret);
		goto free_hp;
	}

	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, policy->cpus);
	if (ret) {
		dev_err(hwvote_dev, "cpu%u: no OPP sharing info: %d\n", policy->cpu, ret);
		goto free_hp;
	}

	hp->opp_token = dev_pm_opp_set_config(cpu_dev, &config);
	if (hp->opp_token < 0) {
		ret = hp->opp_token;
		goto free_hp;
	}

	ret = dev_pm_opp_of_cpumask_add_table(policy->cpus);
	if (ret)
		goto clear_config;

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &table);
	if (ret) {
		dev_err(hwvote_dev, "cpu%u: no frequency table: %d\n", policy->cpu, ret);
		goto remove_table;
	}

	latency = dev_pm_opp_get_max_transition_latency(cpu_dev);
	policy->cpuinfo.transition_latency = latency ?: CPUFREQ_DEFAULT_TRANSITION_LATENCY_NS;
	policy->freq_table = table;
	policy->driver_data = hp;
	policy->dvfs_possible_from_any_cpu = true;

	n = dev_pm_opp_get_opp_count(cpu_dev);
	dev_info(hwvote_dev, "cpus %*pbl (%s): %u..%u kHz, granted %u kHz\n",
		 cpumask_pr_args(policy->cpus), channel, table[0].frequency,
		 table[n - 1].frequency,
		 kirin_hv_get_result(hp->hv) * HWVOTE_KHZ_PER_MHZ);
	return 0;

remove_table:
	dev_pm_opp_of_cpumask_remove_table(policy->cpus);
clear_config:
	dev_pm_opp_clear_config(hp->opp_token);
free_hp:
	kfree(hp);
	return ret;
}

static void hwvote_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct hwvote_policy *hp = policy->driver_data;
	struct device *cpu_dev = get_cpu_device(policy->cpu);

	dev_pm_opp_free_cpufreq_table(cpu_dev, &policy->freq_table);
	dev_pm_opp_of_cpumask_remove_table(policy->related_cpus);
	dev_pm_opp_clear_config(hp->opp_token);
	kfree(hp);
	policy->driver_data = NULL;
}

static struct cpufreq_driver hwvote_cpufreq_driver = {
	.name		= "hisi-hwvote",
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_IS_COOLING_DEV,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= hwvote_cpufreq_target_index,
	.get		= hwvote_cpufreq_get,
	.init		= hwvote_cpufreq_init,
	.exit		= hwvote_cpufreq_exit,
};

static int hwvote_cpufreq_probe(struct platform_device *pdev)
{
	int ret;

	hwvote_dev = &pdev->dev;
	ret = cpufreq_register_driver(&hwvote_cpufreq_driver);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "cannot register cpufreq driver\n");

	return 0;
}

static void hwvote_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&hwvote_cpufreq_driver);
}

/* created by the hw_vote driver */
static struct platform_driver hwvote_cpufreq_platdrv = {
	.driver = {
		.name	= "hisi-hwvote-cpufreq",
	},
	.probe	= hwvote_cpufreq_probe,
	.remove	= hwvote_cpufreq_remove,
};
module_platform_driver(hwvote_cpufreq_platdrv);

MODULE_DESCRIPTION("HiSilicon Kirin 990 hardware vote cpufreq driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hisi-hwvote-cpufreq");
