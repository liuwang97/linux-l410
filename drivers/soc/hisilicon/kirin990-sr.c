// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kirin 990 system suspend ("deep"), the handshake of the vendor kernel's
 * drivers/hisi/pm/old/pm.c: with every other core off, flag the suspend in
 * SCTRL SCBAKDATA8 and enter the cluster sleep state through PSCI
 * CPU_SUSPEND.  BL31 sees the flag and hands the system to LPM3 (DDR self
 * refresh, clocks, IO) instead of only powering the cluster down.  PSCI
 * SYSTEM_SUSPEND, which the generic PSCI code offers as "deep" because the
 * DT claims PSCI 1.0, is a firmware path the vendor never used.
 *
 * The flag register is shared by all clusters (FCM), so it may only be set
 * here, once the other cores are off; cpuidle would otherwise take the
 * system sleep path from its cluster sleep state.
 *
 * It does not resume yet: the board resets about 15 s after the RTC alarm,
 * without the kernel running again.  So "mem" defaults to suspend-to-idle,
 * which works; "deep" stays selectable (through the handshake, never PSCI
 * SYSTEM_SUSPEND) and kirin990_sr.deep=1 makes it the default.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/psci.h>
#include <linux/suspend.h>
#include <asm/cputype.h>

#define SCTRL_SCBAKDATA8	0x42c
#define  AP_SUSPEND_FLAG	BIT(16)
#define CRG_PERPWRACK		0x15c
#define  CORE_PWRACK(core)	BIT(11 + (core))	/* FCM cores 0-7 */
#define  CORE_PWRACK_ALL	GENMASK(18, 11)
/* the cluster sleep state cpuidle uses; the flag makes it a system suspend */
#define SR_POWER_STATE		0x01010000

static bool deep;
module_param(deep, bool, 0444);
MODULE_PARM_DESC(deep, "Make \"deep\" (the vendor LPM3 handshake) the default instead of suspend-to-idle");

static void __iomem *sctrl;
static void __iomem *crg;

static void kirin_sr_flag(bool on)
{
	u32 val = readl(sctrl + SCTRL_SCBAKDATA8);

	writel(on ? val | AP_SUSPEND_FLAG : val & ~AP_SUSPEND_FLAG,
	       sctrl + SCTRL_SCBAKDATA8);
}

static int kirin_sr_enter(suspend_state_t state)
{
	unsigned int core = MPIDR_AFFINITY_LEVEL(read_cpuid_mpidr(), 1);
	u32 others = CORE_PWRACK_ALL & ~CORE_PWRACK(core);
	u32 ack;
	int ret;

	/* PSCI CPU_OFF returns before the other cores have actually powered down */
	if (readl_poll_timeout_atomic(crg + CRG_PERPWRACK, ack, !(ack & others),
				      10, 100 * USEC_PER_MSEC)) {
		pr_err("kirin990-sr: cores %#x still powered\n", ack & others);
		return -EBUSY;
	}

	kirin_sr_flag(true);
	ret = psci_cpu_suspend_enter(SR_POWER_STATE);
	kirin_sr_flag(false);

	return ret ? -EIO : 0;
}

static const struct platform_suspend_ops kirin_sr_ops = {
	.valid	= suspend_valid_only_mem,
	.enter	= kirin_sr_enter,
};

static void __iomem *kirin_sr_map(const char *compat)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, compat);
	void __iomem *base = np ? of_iomap(np, 0) : NULL;

	of_node_put(np);
	return base;
}

/* after PSCI, which registers SYSTEM_SUSPEND from setup_arch */
static int __init kirin_sr_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "hisilicon,lowpm_func");
	of_node_put(np);
	if (!np)
		return 0;

	sctrl = kirin_sr_map("hisilicon,sysctrl");
	crg = kirin_sr_map("hisilicon,crgctrl");
	if (!sctrl || !crg) {
		pr_err("kirin990-sr: no sysctrl / crgctrl\n");
		return -ENODEV;
	}

	/* a flag left over from an interrupted suspend would misroute cpuidle */
	kirin_sr_flag(false);
	suspend_set_ops(&kirin_sr_ops);
	/* unless mem_sleep_default= chose one */
	if (!deep && mem_sleep_default == PM_SUSPEND_MAX)
		mem_sleep_current = PM_SUSPEND_TO_IDLE;
	pr_info("kirin990-sr: deep through the LPM3 handshake, default %s\n",
		mem_sleep_current == PM_SUSPEND_TO_IDLE ? "s2idle" : "deep");
	return 0;
}
late_initcall(kirin_sr_init);
