// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin hardware spinlocks.
 *
 * The locks live in the PCTRL block (or a dedicated "peri_resource_lock"
 * block): groups of 32-bit registers, each holding 32 / bits-per-single
 * locks. A lock is taken by writing (master id << 1 | 1) into its field of
 * the LOCK register and reading it back from the STATUS register (+0x8); it
 * is released through the UNLOCK register (+0x4). Lock ids are shared with
 * the LPM3, sensor hub and modem firmware (e.g. lock 9 guards the ABB clock).
 *
 * Ported from the Huawei vendor driver (hisi_hwspinlock.c, GPL-2.0).
 */

#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "hwspinlock_internal.h"

#define KIRIN_HWLOCK_MASTER	0x01	/* ACPU */
#define KIRIN_HWLOCK_CMD	((KIRIN_HWLOCK_MASTER << 1) | 0x01)
#define KIRIN_HWLOCK_UNLOCK	0x4
#define KIRIN_HWLOCK_STATUS	0x8
#define KIRIN_HWLOCK_MAX_GROUPS	16

struct kirin_hwlock {
	void __iomem *reg;
	unsigned int shift;
	u32 mask;
};

static int kirin_hwspinlock_trylock(struct hwspinlock *lock)
{
	struct kirin_hwlock *hl = lock->priv;

	writel(KIRIN_HWLOCK_CMD << hl->shift, hl->reg);
	return ((readl(hl->reg + KIRIN_HWLOCK_STATUS) >> hl->shift) &
		hl->mask) == KIRIN_HWLOCK_CMD;
}

static void kirin_hwspinlock_unlock(struct hwspinlock *lock)
{
	struct kirin_hwlock *hl = lock->priv;

	writel(KIRIN_HWLOCK_CMD << hl->shift, hl->reg + KIRIN_HWLOCK_UNLOCK);
}

static void kirin_hwspinlock_relax(struct hwspinlock *lock)
{
	ndelay(50);
}

static const struct hwspinlock_ops kirin_hwspinlock_ops = {
	.trylock = kirin_hwspinlock_trylock,
	.unlock = kirin_hwspinlock_unlock,
	.relax = kirin_hwspinlock_relax,
};

static void __iomem *kirin_hwspinlock_base(struct device *dev)
{
	struct device_node *np;
	void __iomem *base;

	/* the node has no reg: the locks are in the PCTRL register block */
	np = of_find_compatible_node(NULL, NULL, "hisilicon,peri_resource_lock");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "hisilicon,pctrl");
	if (!np)
		return IOMEM_ERR_PTR(-ENODEV);
	base = of_iomap(np, 0);
	of_node_put(np);
	return base ?: IOMEM_ERR_PTR(-ENOMEM);
}

static int kirin_hwspinlock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 width, bits, groups, offset[KIRIN_HWLOCK_MAX_GROUPS];
	struct hwspinlock_device *bank;
	struct kirin_hwlock *hl;
	unsigned int per_reg, num, i;
	void __iomem *base;

	if (of_property_read_u32(np, "hwlock,register-width", &width) ||
	    of_property_read_u32(np, "hwlock,bits-per-single", &bits) ||
	    of_property_read_u32(np, "hwlock,groups", &groups) ||
	    !bits || bits > width || width > 32 || !groups ||
	    groups > KIRIN_HWLOCK_MAX_GROUPS ||
	    of_property_read_u32_array(np, "hwlock,offset", offset, groups))
		return dev_err_probe(dev, -EINVAL, "bad lock description\n");

	base = kirin_hwspinlock_base(dev);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base), "no register block\n");

	per_reg = width / bits;
	num = per_reg * groups;
	bank = devm_kzalloc(dev, struct_size(bank, lock, num), GFP_KERNEL);
	hl = devm_kcalloc(dev, num, sizeof(*hl), GFP_KERNEL);
	if (!bank || !hl) {
		iounmap(base);
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		hl[i].reg = base + offset[i / per_reg];
		hl[i].shift = (i % per_reg) * bits;
		hl[i].mask = GENMASK(bits - 1, 0);
		bank->lock[i].priv = &hl[i];
	}

	platform_set_drvdata(pdev, bank);
	return devm_hwspin_lock_register(dev, bank, &kirin_hwspinlock_ops, 0,
					 num);
}

static const struct of_device_id kirin_hwspinlock_of_match[] = {
	{ .compatible = "hisilicon,hwspinlock" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin_hwspinlock_of_match);

static struct platform_driver kirin_hwspinlock_driver = {
	.probe = kirin_hwspinlock_probe,
	.driver = {
		.name = "kirin-hwspinlock",
		.of_match_table = kirin_hwspinlock_of_match,
	},
};

static int __init kirin_hwspinlock_init(void)
{
	return platform_driver_register(&kirin_hwspinlock_driver);
}
/* clock and GPIO users need the locks early */
arch_initcall(kirin_hwspinlock_init);

MODULE_DESCRIPTION("HiSilicon Kirin hardware spinlock driver");
MODULE_LICENSE("GPL");
