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
 *           operating-points-v2 = <&opp_table0>;
 *           sched-energy-costs = <&core_cost0 &cluster_cost0>; };
 *
 * A vote is a single register write that neither sleeps nor locks, so the
 * driver supports fast switching from the scheduler (like qcom-cpufreq-hw).
 *
 * The energy model comes from the vendor's (pre-mainline EAS) energy tables:
 * busy-cost-data holds (capacity, power in mW) pairs per cluster. Each OPP's
 * capacity is interpolated into that curve, so EAS and the power allocator
 * thermal governor work without measuring the clusters again.
 *
 * Based on the Huawei vendor driver (drivers/cpufreq/hisi/hisi_cpufreq_dt.c).
 */

#include <linux/arch_topology.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/energy_model.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/soc/hisilicon/kirin-hw-vote.h>
#include <linux/workqueue.h>

#define HWVOTE_KHZ_PER_MHZ	1000
#define HWVOTE_MAX_COSTS	16
#define HWVOTE_PRESSURE_MS	200

struct hwvote_policy {
	struct kirin_hv *hv;
	int opp_token;
	struct cpufreq_policy *policy;

	/* vendor energy table: capacity -> mW, ascending */
	unsigned int nr_costs;
	u32 cost_cap[HWVOTE_MAX_COSTS];
	u32 cost_mw[HWVOTE_MAX_COSTS];

	/* LPM3 granting less than the vote (firmware capping) */
	struct delayed_work pressure_work;
	u32 last_vote;
	unsigned int low_samples;
	bool pressure;
};

/*
 * All operating points of the firmware tables carry opp-supported-hw = <3>;
 * the vendor kernel enables bit 0 for "HUAWEI Kirin 990".
 */
static const u32 hwvote_supported_hw = BIT(0);

static struct device *hwvote_dev;

/*
 * LPM3 completes a vote in 0.4-0.8 ms (measured), the firmware tables claim
 * 2 ms, which made schedutil rate limit itself to 3 ms.
 */
static unsigned int rate_limit_us = 500;
module_param(rate_limit_us, uint, 0444);
MODULE_PARM_DESC(rate_limit_us, "schedutil rate limit to start with (us)");

static bool fast_switch = true;
module_param(fast_switch, bool, 0444);
MODULE_PARM_DESC(fast_switch, "Vote from scheduler context (fast switching)");

static bool energy_model = true;
module_param(energy_model, bool, 0444);
MODULE_PARM_DESC(energy_model, "Register the vendor energy tables as energy model");

static bool report_pressure;
module_param_named(hw_pressure, report_pressure, bool, 0444);
MODULE_PARM_DESC(hw_pressure, "Report LPM3 frequency capping to the scheduler");

static int hwvote_cpufreq_target_index(struct cpufreq_policy *policy,
				       unsigned int index)
{
	struct hwvote_policy *hp = policy->driver_data;

	kirin_hv_set(hp->hv, policy->freq_table[index].frequency / HWVOTE_KHZ_PER_MHZ);
	return 0;
}

static unsigned int hwvote_cpufreq_fast_switch(struct cpufreq_policy *policy,
					       unsigned int target_freq)
{
	struct hwvote_policy *hp = policy->driver_data;

	kirin_hv_set(hp->hv, target_freq / HWVOTE_KHZ_PER_MHZ);
	return target_freq;
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

/* ------------------------------------------------------------------------ */
/* energy model */

static int hwvote_parse_costs(struct hwvote_policy *hp, struct device_node *cpu_np)
{
	struct device_node *np;
	u32 v[2 * HWVOTE_MAX_COSTS];
	int n, i;

	np = of_parse_phandle(cpu_np, "sched-energy-costs", 0);
	if (!np)
		return -ENOENT;

	n = of_property_count_u32_elems(np, "busy-cost-data");
	if (n < 2 || n % 2 || n > ARRAY_SIZE(v) ||
	    of_property_read_u32_array(np, "busy-cost-data", v, n)) {
		of_node_put(np);
		return -EINVAL;
	}
	of_node_put(np);

	for (i = 0; i < n / 2; i++) {
		hp->cost_cap[i] = v[2 * i];
		hp->cost_mw[i] = v[2 * i + 1];
		if (!hp->cost_cap[i] || !hp->cost_mw[i] ||
		    (i && (hp->cost_cap[i] <= hp->cost_cap[i - 1] ||
			   hp->cost_mw[i] <= hp->cost_mw[i - 1])))
			return -EINVAL;
	}
	hp->nr_costs = n / 2;
	return 0;
}

/* power in uW at @cap, from the (capacity, mW) curve */
static unsigned long hwvote_cost_uw(struct hwvote_policy *hp, unsigned long cap)
{
	unsigned int i, n = hp->nr_costs;
	u64 c0, c1, p0, p1;

	/* below the table: proportional to the first point */
	if (cap <= hp->cost_cap[0])
		return div_u64((u64)hp->cost_mw[0] * 1000 * cap, hp->cost_cap[0]);

	for (i = 1; i < n - 1 && cap > hp->cost_cap[i]; i++)
		;
	c0 = hp->cost_cap[i - 1];
	c1 = hp->cost_cap[i];
	p0 = hp->cost_mw[i - 1] * 1000ULL;
	p1 = hp->cost_mw[i] * 1000ULL;
	/* linear in between, extrapolated with the last slope above */
	return p0 + div_u64((p1 - p0) * (cap - c0), c1 - c0);
}

static int hwvote_active_power(struct device *cpu_dev, unsigned long *uw,
			       unsigned long *khz)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu_dev->id);
	struct cpufreq_frequency_table *pos;
	struct hwvote_policy *hp;
	unsigned long cap;

	if (!policy || !policy->driver_data)
		return -ENODEV;
	hp = policy->driver_data;

	/*
	 * The capacities of the table are the vendor's normalised ones (its
	 * last point is the cluster at its fastest OPP). Don't use
	 * arch_scale_cpu_capacity() here: it is only normalised once every
	 * cluster has a cpufreq policy, after this runs for the first ones.
	 */
	cpufreq_for_each_valid_entry(pos, policy->freq_table) {
		if (pos->frequency < *khz)
			continue;
		cap = DIV_ROUND_CLOSEST((u64)hp->cost_cap[hp->nr_costs - 1] *
					pos->frequency, policy->cpuinfo.max_freq);
		*khz = pos->frequency;
		*uw = hwvote_cost_uw(hp, cap);
		return 0;
	}
	return -EINVAL;
}

static void hwvote_cpufreq_register_em(struct cpufreq_policy *policy)
{
	struct em_data_callback cb = EM_DATA_CB(hwvote_active_power);
	struct hwvote_policy *hp = policy->driver_data;
	struct device *cpu_dev = get_cpu_device(policy->cpu);
	int ret, n;

	if (!energy_model || !hp->nr_costs)
		return;

	n = dev_pm_opp_get_opp_count(cpu_dev);
	if (n <= 0)
		return;

	ret = em_dev_register_perf_domain(cpu_dev, n, &cb, policy->related_cpus, true);
	if (ret)
		dev_warn(hwvote_dev, "cpus %*pbl: no energy model: %d\n",
			 cpumask_pr_args(policy->related_cpus), ret);
	else
		dev_info(hwvote_dev, "cpus %*pbl: energy model, %u states, %lu..%lu mW\n",
			 cpumask_pr_args(policy->related_cpus), n,
			 hwvote_cost_uw(hp, DIV_ROUND_CLOSEST((u64)hp->cost_cap[hp->nr_costs - 1] *
							       policy->cpuinfo.min_freq,
							       policy->cpuinfo.max_freq)) / 1000,
			 hwvote_cost_uw(hp, hp->cost_cap[hp->nr_costs - 1]) / 1000);
}

/* ------------------------------------------------------------------------ */
/* hardware pressure: LPM3 may grant less than the vote (thermal, AVS limits) */

static void hwvote_pressure_work(struct work_struct *work)
{
	struct hwvote_policy *hp = container_of(to_delayed_work(work),
						struct hwvote_policy, pressure_work);
	struct cpufreq_policy *policy = hp->policy;
	u32 vote = kirin_hv_get_vote(hp->hv);
	u32 granted = kirin_hv_get_result(hp->hv);

	/* two samples in a row with the same vote: not a transition */
	if (vote && granted + granted / 20 < vote && vote == hp->last_vote) {
		if (++hp->low_samples >= 2) {
			arch_update_hw_pressure(policy->related_cpus,
						granted * HWVOTE_KHZ_PER_MHZ);
			hp->pressure = true;
		}
	} else {
		hp->low_samples = 0;
		if (hp->pressure && granted >= vote) {
			arch_update_hw_pressure(policy->related_cpus,
						policy->cpuinfo.max_freq);
			hp->pressure = false;
		}
	}
	hp->last_vote = vote;

	queue_delayed_work(system_power_efficient_wq, &hp->pressure_work,
			   msecs_to_jiffies(HWVOTE_PRESSURE_MS));
}

/* ------------------------------------------------------------------------ */

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
	hp->policy = policy;

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

	if (hwvote_parse_costs(hp, cpu_dev->of_node))
		dev_warn(hwvote_dev, "cpu%u: no usable sched-energy-costs\n", policy->cpu);

	latency = dev_pm_opp_get_max_transition_latency(cpu_dev);
	policy->cpuinfo.transition_latency = latency ?: CPUFREQ_DEFAULT_TRANSITION_LATENCY_NS;
	if (rate_limit_us)
		policy->transition_delay_us = rate_limit_us;
	policy->freq_table = table;
	policy->driver_data = hp;
	policy->dvfs_possible_from_any_cpu = true;
	policy->fast_switch_possible = fast_switch;

	if (report_pressure) {
		INIT_DEFERRABLE_WORK(&hp->pressure_work, hwvote_pressure_work);
		queue_delayed_work(system_power_efficient_wq, &hp->pressure_work,
				   msecs_to_jiffies(HWVOTE_PRESSURE_MS));
	}

	n = dev_pm_opp_get_opp_count(cpu_dev);
	dev_info(hwvote_dev, "cpus %*pbl (%s): %u..%u kHz, granted %u kHz%s\n",
		 cpumask_pr_args(policy->cpus), channel, table[0].frequency,
		 table[n - 1].frequency,
		 kirin_hv_get_result(hp->hv) * HWVOTE_KHZ_PER_MHZ,
		 fast_switch ? ", fast switch" : "");
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

	if (report_pressure) {
		cancel_delayed_work_sync(&hp->pressure_work);
		if (hp->pressure)
			arch_update_hw_pressure(policy->related_cpus,
						policy->cpuinfo.max_freq);
	}
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
	.fast_switch	= hwvote_cpufreq_fast_switch,
	.get		= hwvote_cpufreq_get,
	.init		= hwvote_cpufreq_init,
	.exit		= hwvote_cpufreq_exit,
	.register_em	= hwvote_cpufreq_register_em,
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
