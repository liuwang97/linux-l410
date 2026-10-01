// SPDX-License-Identifier: GPL-2.0
/*
 * DDR frequency scaling for the HiSilicon Kirin 990 (Huawei Qingyun L410)
 *
 * LPM3 performs the DDR DVFS. Linux votes a minimum DDR frequency in PMCTRL
 * (the vendor's clk_ddrc_min hardware vote: offset 0x270, MHz in bits 14:0,
 * bit 15 write enable) and reads the frequency LPM3 set back from SCTRL
 * 0x41c bits 11:8, an index into the firmware operating-points table
 * (415, 900, 1106, 1370, 1660, 1800, 2133 MHz).
 *
 * UEFI leaves a 2133 MHz vote behind, so without this driver the memory runs
 * at its fastest OPP all the time. The vendor driver (ddr_devfreq with its
 * pm_qos governor) turned memory throughput requests into this vote; here the
 * devfreq core aggregates DEV_PM_QOS_MIN_FREQUENCY requests (l410-perf: mode,
 * boosts and a CPU/GPU load coupling) and the powersave governor picks the
 * lowest OPP that satisfies them.
 *
 * The display (DSS) needs about 750 MB/s for 2160x1440@60, far below the
 * lowest OPP; the vendor DSS driver never votes for memory bandwidth.
 */

#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/soc/hisilicon/kirin-hw-vote.h>
#include <linux/units.h>

/*
 * Start with the performance governor, i.e. the fastest OPP UEFI leaves
 * behind; the power profiles switch to powersave (lowest OPP satisfying
 * the PM QoS minimum requests).
 */
static char *governor = DEVFREQ_GOV_PERFORMANCE;
module_param(governor, charp, 0444);
MODULE_PARM_DESC(governor, "Initial devfreq governor");

#define DDR_VOTE_OFFSET		0x270	/* PMCTRL: clk_ddrc_min */
#define DDR_SCTRL_FREQ		0xfa89b41c
#define DDR_SCTRL_IDX(v)	(((v) >> 8) & 0xf)

struct kirin_ddr {
	struct devfreq_dev_profile profile;
	struct devfreq *devfreq;
	struct kirin_hv *hv;
	void __iomem *freq_reg;
	unsigned long *freqs;	/* Hz, ascending, the firmware table */
	unsigned int nr_freqs;
	unsigned long cur;
};

static unsigned long kirin_ddr_read(struct kirin_ddr *d)
{
	unsigned int idx = DDR_SCTRL_IDX(readl(d->freq_reg));

	return idx < d->nr_freqs ? d->freqs[idx] : d->cur;
}

static int kirin_ddr_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct kirin_ddr *d = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	kirin_hv_set(d->hv, *freq / HZ_PER_MHZ);
	d->cur = *freq;
	return 0;
}

static int kirin_ddr_get_cur_freq(struct device *dev, unsigned long *freq)
{
	*freq = kirin_ddr_read(dev_get_drvdata(dev));
	return 0;
}

static int kirin_ddr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_pm_opp *opp;
	struct kirin_ddr *d;
	unsigned long f;
	int ret, i;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	platform_set_drvdata(pdev, d);

	d->hv = kirin_hv_get_reg(DDR_VOTE_OFFSET, 0x7fff, 0x8000);
	if (IS_ERR(d->hv))
		return dev_err_probe(dev, PTR_ERR(d->hv), "no DDR vote\n");

	/* SCTRL is shared with the clock and reset drivers: map, do not claim */
	d->freq_reg = devm_ioremap(dev, DDR_SCTRL_FREQ, 4);
	if (!d->freq_reg)
		return -ENOMEM;

	/* operating-points (v1): kHz / uV pairs */
	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "no operating points\n");

	ret = dev_pm_opp_get_opp_count(dev);
	if (ret <= 0)
		return ret ?: -EINVAL;
	d->nr_freqs = ret;
	d->freqs = devm_kcalloc(dev, d->nr_freqs, sizeof(*d->freqs), GFP_KERNEL);
	if (!d->freqs)
		return -ENOMEM;
	for (i = 0, f = 0; i < d->nr_freqs; i++, f++) {
		opp = dev_pm_opp_find_freq_ceil(dev, &f);
		if (IS_ERR(opp))
			return PTR_ERR(opp);
		dev_pm_opp_put(opp);
		d->freqs[i] = f;
	}

	d->cur = kirin_ddr_read(d);
	d->profile.initial_freq = d->cur;
	d->profile.target = kirin_ddr_target;
	d->profile.get_cur_freq = kirin_ddr_get_cur_freq;

	d->devfreq = devm_devfreq_add_device(dev, &d->profile, governor, NULL);
	if (IS_ERR(d->devfreq))
		return dev_err_probe(dev, PTR_ERR(d->devfreq), "devfreq\n");

	dev_info(dev, "%u OPPs %lu..%lu MHz, now %lu MHz (vote %u MHz)\n",
		 d->nr_freqs, d->freqs[0] / HZ_PER_MHZ,
		 d->freqs[d->nr_freqs - 1] / HZ_PER_MHZ, kirin_ddr_read(d) / HZ_PER_MHZ,
		 kirin_hv_get_vote(d->hv));
	return 0;
}

static const struct of_device_id kirin_ddr_of_match[] = {
	{ .compatible = "hisilicon,kirin990-ddr" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin_ddr_of_match);

static struct platform_driver kirin_ddr_driver = {
	.probe = kirin_ddr_probe,
	.driver = {
		.name = "kirin990-ddr-devfreq",
		.of_match_table = kirin_ddr_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(kirin_ddr_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 DDR frequency scaling");
MODULE_LICENSE("GPL");
