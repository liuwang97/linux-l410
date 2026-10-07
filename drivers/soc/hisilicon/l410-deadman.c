// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei L410 (Kirin 990) AP watchdog takeover and bring-up deadman.
 *
 * The firmware starts the AP SP805 watchdog (WDT0) before it enters the
 * kernel and leaves it to the OS to feed: at kernel entry about 60 s are
 * left. BL31 delivers the SP805 interrupt to the kernel as an FIQ, so a
 * kernel that leaves WDT0 alone panics a minute after boot. This driver
 * takes WDT0 over in early_initcall:
 *
 *   l410_deadman=0 (default)        stop it
 *   l410_deadman=<seconds>          re-arm it and never pet it, so a test
 *                                   kernel that hangs, or boots but is
 *                                   unreachable, resets the machine after a
 *                                   bounded time
 *   /sys/kernel/l410_deadman/timeout  write N to re-arm for N seconds, 0 to stop
 *
 * WDT0 is always clocked (its gate is a dummy) and counts at 32.768 kHz.
 * The SP805 raises its interrupt on the first expiry and asserts reset on the
 * second one, so LOAD is half the requested period. The FIQ handler below
 * panics on that interrupt with the interrupted context, and the SP805 reset
 * is the backstop if even that hangs.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/minmax.h>
#include <linux/panic.h>
#include <linux/printk.h>
#include <linux/sched/debug.h>
#include <linux/sysfs.h>

#include <asm/irq.h>

#define WDT0_BASE	0xfe026000
#define WDT0_RATE	32768

#define WDTLOAD		0x000
#define WDTVALUE	0x004
#define WDTCONTROL	0x008
#define  WDT_INTEN	BIT(0)
#define  WDT_RESEN	BIT(1)
#define WDTINTCLR	0x00c
#define WDTRIS		0x010
#define WDTLOCK		0xc00
#define  WDT_UNLOCK_KEY	0x1acce551

/* BL31 stores the interrupt ID it forwards as an FIQ here (low 16 bits) */
#define BL31_FIQ_SOURCE	0x26d81000

static unsigned int deadman_secs;
static void __iomem *wdt0;
static u64 *fiq_source;

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
		writel_relaxed(1, wdt0 + WDTINTCLR);
	}
	writel_relaxed(0, wdt0 + WDTLOCK);
	readl_relaxed(wdt0 + WDTLOCK);
	deadman_secs = secs;
}

/*
 * Besides WDT0 (interrupt ID 76), the vendor kernel expects BL31 to forward
 * DDR security (DMSS) violations (ID 163) as FIQs. Say which one it was.
 */
static void deadman_fiq(struct pt_regs *regs)
{
	u32 ris = readl_relaxed(wdt0 + WDTRIS);

	pr_emerg("l410-deadman: FIQ, WDT0 control %#x raw status %#x, BL31 source %#llx\n",
		 readl_relaxed(wdt0 + WDTCONTROL), ris,
		 fiq_source ? READ_ONCE(*fiq_source) : 0);
	show_regs(regs);
	if (ris & 1)
		panic("l410-deadman: AP watchdog expired (armed for %u s)", deadman_secs);
	panic("l410-deadman: FIQ not raised by the AP watchdog");
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
	u32 ctl, left;

	wdt0 = ioremap(WDT0_BASE, SZ_4K);
	if (!wdt0)
		return -ENOMEM;

	ctl = readl_relaxed(wdt0 + WDTCONTROL);
	left = readl_relaxed(wdt0 + WDTVALUE) / WDT0_RATE;
	if (ctl & (WDT_INTEN | WDT_RESEN))
		pr_info("l410-deadman: firmware left WDT0 running, FIQ in %u s\n", left);

	fiq_source = memremap(BL31_FIQ_SOURCE, sizeof(*fiq_source), MEMREMAP_WB);
	if (set_handle_fiq(deadman_fiq))
		pr_warn("l410-deadman: FIQ handler already set\n");

	deadman_set(deadman_secs);
	if (deadman_secs)
		pr_info("l410-deadman: SP805 armed, reset in %u s unless re-armed\n", deadman_secs);
	else
		pr_info("l410-deadman: WDT0 stopped\n");
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
