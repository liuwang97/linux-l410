// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei L410 (Kirin 990) bring-up deadman.
 *
 * Arms the AP SP805 watchdog (WDT0) as early as possible and never pets it,
 * so a test kernel that hangs, or boots but is unreachable, resets the
 * machine back to the default GRUB entry after a bounded time.
 *
 *   l410_deadman=<seconds>          0 leaves the watchdog alone (default 600)
 *   /sys/kernel/l410_deadman/timeout  write N to re-arm for N seconds, 0 to stop
 *
 * WDT0 is always clocked (its gate is a dummy) and counts at 32.768 kHz.
 * The SP805 raises its interrupt on the first expiry and asserts reset on the
 * second one, so LOAD is half the requested period. The firmware routes that
 * interrupt as an FIQ, which the kernel treats as fatal: in practice the test
 * kernel panics (and reboots via PSCI) at half the period, and the SP805 reset
 * is the backstop if even that hangs.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/minmax.h>
#include <linux/printk.h>
#include <linux/sysfs.h>

#define WDT0_BASE	0xfe026000
#define WDT0_RATE	32768

#define WDTLOAD		0x000
#define WDTCONTROL	0x008
#define  WDT_INTEN	BIT(0)
#define  WDT_RESEN	BIT(1)
#define WDTINTCLR	0x00c
#define WDTLOCK		0xc00
#define  WDT_UNLOCK_KEY	0x1acce551

static unsigned int deadman_secs = 600;
static void __iomem *wdt0;

static int __init deadman_param(char *s)
{
	return kstrtouint(s, 0, &deadman_secs);
}
early_param("l410_deadman", deadman_param);

static void deadman_set(unsigned int secs)
{
	u64 load = min_t(u64, (u64)WDT0_RATE * secs / 2, U32_MAX);

	writel_relaxed(WDT_UNLOCK_KEY, wdt0 + WDTLOCK);
	if (secs) {
		writel_relaxed((u32)load, wdt0 + WDTLOAD);
		writel_relaxed(1, wdt0 + WDTINTCLR);
		writel_relaxed(WDT_INTEN | WDT_RESEN, wdt0 + WDTCONTROL);
	} else {
		writel_relaxed(0, wdt0 + WDTCONTROL);
	}
	writel_relaxed(0, wdt0 + WDTLOCK);
	readl_relaxed(wdt0 + WDTLOCK);
	deadman_secs = secs;
}

static ssize_t timeout_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", deadman_secs);
}

static ssize_t timeout_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int secs;
	int ret = kstrtouint(buf, 0, &secs);

	if (ret)
		return ret;
	deadman_set(secs);
	pr_info("l410-deadman: %s (%u s)\n", secs ? "re-armed" : "stopped", secs);
	return count;
}

static struct kobj_attribute timeout_attr = __ATTR_RW(timeout);

static struct attribute *deadman_attrs[] = {
	&timeout_attr.attr,
	NULL,
};

static const struct attribute_group deadman_group = {
	.attrs = deadman_attrs,
};

static int __init deadman_early(void)
{
	if (!deadman_secs)
		return 0;
	wdt0 = ioremap(WDT0_BASE, SZ_4K);
	if (!wdt0)
		return -ENOMEM;
	deadman_set(deadman_secs);
	pr_info("l410-deadman: SP805 armed, reset in %u s unless re-armed\n", deadman_secs);
	return 0;
}
early_initcall(deadman_early);

static int __init deadman_sysfs(void)
{
	struct kobject *kobj;

	if (!wdt0)
		return 0;
	kobj = kobject_create_and_add("l410_deadman", kernel_kobj);
	if (!kobj)
		return -ENOMEM;
	return sysfs_create_group(kobj, &deadman_group);
}
late_initcall(deadman_sysfs);
