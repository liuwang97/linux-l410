// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SPMI PMIC core (Kirin 990 generation)
 *
 * Unlike the Hi6421v600 (Kirin 970) the register layout of this PMIC
 * generation is described entirely by the firmware device tree: every
 * regulator, the RTC and the power key are child nodes with their own
 * compatible, and the interrupt banks are listed as register arrays.
 *
 * This driver provides the SPMI regmap for the children, creates platform
 * devices for the child nodes and registers the PMIC interrupt controller.
 * The interrupt controller is a separate cell so that the regulators do not
 * have to wait for the GPIO controller that carries the PMIC interrupt line.
 *
 * Based on the Huawei vendor driver (drivers/mfd/hisi_pmic_spmi.c) and on
 * drivers/misc/hi6421v600-irq.c.
 */

#include <linux/bitops.h>
#include <linux/clk/kirin.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/spmi.h>

#define HISI_PMIC_IRQ_BANK_BITS		8
#define HISI_PMIC_IRQ_BANK_MASK		0xff

/* bank 0: when a short press latches both, report down before up */
#define HISI_PMIC_IRQ_POWERKEY_UP	6
#define HISI_PMIC_IRQ_POWERKEY_DOWN	7

static const struct regmap_config hisi_spmi_pmic_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= true,
};

static const struct mfd_cell hisi_spmi_pmic_irq_cell = {
	.name = "hisi-spmi-pmic-irq",
};

/*
 * The boot firmware reads the reason of the last reset from a PMIC register
 * that survives resets (HRST_REG13 on the main PMIC). Anything the kernel
 * did not write there is logged as AP_S_ABNORMAL, so record an orderly
 * restart / power off like the vendor kernel does (values from its
 * mntn_common_interface.h: COLDBOOT = 0x10, AP_S_COLDBOOT = 0x00).
 */
struct hisi_spmi_pmic_data {
	u16 reboot_reason_reg;
	bool clk_provider;	/* registers of the SoC "clk-pmu-gate" clocks */
};

static const struct hisi_spmi_pmic_data hisi_spmi_pmic_main = {
	.reboot_reason_reg = 0x303,
	.clk_provider = true,
};

static void hisi_spmi_pmic_clk_unhook(void *data)
{
	kirin_clk_set_pmic_regmap(NULL);
}

#define HISI_PMIC_REASON_COLDBOOT	0x10	/* restart */
#define HISI_PMIC_REASON_AP_S_COLDBOOT	0x00	/* power off */

struct hisi_spmi_pmic {
	struct regmap *regmap;
	const struct hisi_spmi_pmic_data *data;
	struct notifier_block reboot_nb;
};

static int hisi_spmi_pmic_reboot_notify(struct notifier_block *nb,
					unsigned long action, void *cmd)
{
	struct hisi_spmi_pmic *pmic = container_of(nb, struct hisi_spmi_pmic,
						   reboot_nb);
	unsigned int reason = action == SYS_RESTART ? HISI_PMIC_REASON_COLDBOOT :
						      HISI_PMIC_REASON_AP_S_COLDBOOT;

	regmap_write(pmic->regmap, pmic->data->reboot_reason_reg, reason);
	return NOTIFY_DONE;
}

static int hisi_spmi_pmic_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct hisi_spmi_pmic *pmic;
	struct regmap *regmap;
	int ret;

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	regmap = devm_regmap_init_spmi_ext(sdev, &hisi_spmi_pmic_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "regmap init failed\n");

	pmic->regmap = regmap;
	pmic->data = device_get_match_data(dev);
	if (pmic->data && pmic->data->reboot_reason_reg) {
		unsigned int reason;

		if (!regmap_read(regmap, pmic->data->reboot_reason_reg, &reason))
			dev_info(dev, "last reset reason 0x%02x\n", reason);

		pmic->reboot_nb.notifier_call = hisi_spmi_pmic_reboot_notify;
		ret = devm_register_reboot_notifier(dev, &pmic->reboot_nb);
		if (ret)
			return dev_err_probe(dev, ret, "cannot register reboot notifier\n");
	}

	/* the "hisilicon,clk-pmu-gate" clocks (32 kHz, audio MCLK, ...) live here */
	if (pmic->data && pmic->data->clk_provider) {
		kirin_clk_set_pmic_regmap(regmap);
		ret = devm_add_action_or_reset(dev, hisi_spmi_pmic_clk_unhook, NULL);
		if (ret)
			return ret;
	}

	if (of_property_read_bool(dev->of_node, "interrupt-controller")) {
		ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
					   &hisi_spmi_pmic_irq_cell, 1,
					   NULL, 0, NULL);
		if (ret)
			return dev_err_probe(dev, ret, "failed to add irq cell\n");
	}

	/* regulators, rtc, power key ... are child nodes with a compatible */
	ret = devm_of_platform_populate(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to populate children\n");

	dev_info(dev, "PMIC at SPMI usid %u\n", sdev->usid);
	return 0;
}

static const struct of_device_id hisi_spmi_pmic_of_match[] = {
	{ .compatible = "hisilicon-hisi-pmic-spmi", .data = &hisi_spmi_pmic_main },
	{ .compatible = "hisilicon-hisi-sub-pmic-spmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_spmi_pmic_of_match);

static struct spmi_driver hisi_spmi_pmic_driver = {
	.driver = {
		.name		= "hisi-spmi-pmic",
		.of_match_table	= hisi_spmi_pmic_of_match,
	},
	.probe	= hisi_spmi_pmic_probe,
};

/*
 * Interrupt controller. Firmware properties on the PMIC node:
 *   hisilicon,hisi-pmic-irq-num        number of interrupts (8 per bank)
 *   hisilicon,hisi-pmic-irq-array      number of banks
 *   hisilicon,hisi-pmic-irq-mask-addr  mask register of each bank (1 = masked)
 *   hisilicon,hisi-pmic-irq-addr       status register of each bank (write 1 to clear)
 *   gpios                              GPIO the (active low) PMIC interrupt line is on
 * Consumers use the PMIC node as interrupt-parent, two cells <hwirq flags>.
 */
struct hisi_spmi_pmic_irq {
	struct device *dev;
	struct regmap *regmap;
	struct irq_domain *domain;
	spinlock_t lock;	/* mask register read-modify-write */
	u32 nirqs;
	u32 nbanks;
	u32 *mask_regs;
	u32 *status_regs;
	int parent_irq;
};

static void hisi_spmi_pmic_irq_set_mask(struct irq_data *d, bool masked)
{
	struct hisi_spmi_pmic_irq *pirq = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	u32 reg = pirq->mask_regs[hw / HISI_PMIC_IRQ_BANK_BITS];
	u32 bit = BIT(hw % HISI_PMIC_IRQ_BANK_BITS);
	unsigned long flags;

	spin_lock_irqsave(&pirq->lock, flags);
	regmap_update_bits(pirq->regmap, reg, bit, masked ? bit : 0);
	spin_unlock_irqrestore(&pirq->lock, flags);
}

static void hisi_spmi_pmic_irq_mask(struct irq_data *d)
{
	hisi_spmi_pmic_irq_set_mask(d, true);
}

static void hisi_spmi_pmic_irq_unmask(struct irq_data *d)
{
	hisi_spmi_pmic_irq_set_mask(d, false);
}

/* what wakes the SoC is the PMIC's own line to it (an AO GPIO) */
static int hisi_spmi_pmic_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct hisi_spmi_pmic_irq *pirq = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(pirq->parent_irq, on);
}

static struct irq_chip hisi_spmi_pmic_irq_chip = {
	.name		= "hisi-spmi-pmic",
	.irq_mask	= hisi_spmi_pmic_irq_mask,
	.irq_unmask	= hisi_spmi_pmic_irq_unmask,
	.irq_disable	= hisi_spmi_pmic_irq_mask,
	.irq_enable	= hisi_spmi_pmic_irq_unmask,
	.irq_set_wake	= hisi_spmi_pmic_irq_set_wake,
};

static int hisi_spmi_pmic_irq_map(struct irq_domain *d, unsigned int virq,
				  irq_hw_number_t hw)
{
	struct hisi_spmi_pmic_irq *pirq = d->host_data;

	irq_set_chip_data(virq, pirq);
	irq_set_chip_and_handler(virq, &hisi_spmi_pmic_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, true);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops hisi_spmi_pmic_irq_domain_ops = {
	.map	= hisi_spmi_pmic_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static void hisi_spmi_pmic_irq_dispatch(struct hisi_spmi_pmic_irq *pirq,
					u32 bank, u32 bit)
{
	irq_hw_number_t hw = bank * HISI_PMIC_IRQ_BANK_BITS + bit;
	unsigned int virq;

	if (hw >= pirq->nirqs)
		return;

	virq = irq_find_mapping(pirq->domain, hw);
	if (virq)
		handle_nested_irq(virq);
	else
		dev_dbg_ratelimited(pirq->dev, "unmapped interrupt %lu\n", hw);
}

static irqreturn_t hisi_spmi_pmic_irq_thread(int irq, void *data)
{
	struct hisi_spmi_pmic_irq *pirq = data;
	unsigned long pending;
	unsigned int status, mask;
	u32 bank, bit;
	bool handled = false;

	for (bank = 0; bank < pirq->nbanks; bank++) {
		if (regmap_read(pirq->regmap, pirq->status_regs[bank], &status) ||
		    regmap_read(pirq->regmap, pirq->mask_regs[bank], &mask))
			continue;
		/* masked sources still latch their status bit; leave them */
		status &= ~mask & HISI_PMIC_IRQ_BANK_MASK;
		if (!status)
			continue;

		/* write 1 to clear what we are about to handle */
		regmap_write(pirq->regmap, pirq->status_regs[bank], status);
		handled = true;

		pending = status;
		if (bank == 0 && (pending & BIT(HISI_PMIC_IRQ_POWERKEY_DOWN)) &&
		    (pending & BIT(HISI_PMIC_IRQ_POWERKEY_UP))) {
			hisi_spmi_pmic_irq_dispatch(pirq, 0, HISI_PMIC_IRQ_POWERKEY_DOWN);
			hisi_spmi_pmic_irq_dispatch(pirq, 0, HISI_PMIC_IRQ_POWERKEY_UP);
			pending &= ~(BIT(HISI_PMIC_IRQ_POWERKEY_DOWN) |
				     BIT(HISI_PMIC_IRQ_POWERKEY_UP));
		}

		for_each_set_bit(bit, &pending, HISI_PMIC_IRQ_BANK_BITS)
			hisi_spmi_pmic_irq_dispatch(pirq, bank, bit);
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static void hisi_spmi_pmic_irq_remove_domain(void *data)
{
	struct hisi_spmi_pmic_irq *pirq = data;
	irq_hw_number_t hw;

	for (hw = 0; hw < pirq->nirqs; hw++)
		irq_dispose_mapping(irq_find_mapping(pirq->domain, hw));
	irq_domain_remove(pirq->domain);
}

static int hisi_spmi_pmic_irq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *pmic_dev = dev->parent;
	struct device_node *np = pmic_dev->of_node;
	struct hisi_spmi_pmic_irq *pirq;
	struct gpio_desc *gpio;
	unsigned int dummy;
	int irq, ret;
	u32 i;

	pirq = devm_kzalloc(dev, sizeof(*pirq), GFP_KERNEL);
	if (!pirq)
		return -ENOMEM;

	pirq->dev = dev;
	spin_lock_init(&pirq->lock);
	pirq->regmap = dev_get_regmap(pmic_dev, NULL);
	if (!pirq->regmap)
		return -ENODEV;

	if (of_property_read_u32(np, "hisilicon,hisi-pmic-irq-num", &pirq->nirqs) ||
	    of_property_read_u32(np, "hisilicon,hisi-pmic-irq-array", &pirq->nbanks) ||
	    !pirq->nbanks || !pirq->nirqs ||
	    pirq->nirqs > pirq->nbanks * HISI_PMIC_IRQ_BANK_BITS)
		return dev_err_probe(dev, -EINVAL, "bad interrupt bank description\n");

	pirq->mask_regs = devm_kcalloc(dev, pirq->nbanks, sizeof(u32), GFP_KERNEL);
	pirq->status_regs = devm_kcalloc(dev, pirq->nbanks, sizeof(u32), GFP_KERNEL);
	if (!pirq->mask_regs || !pirq->status_regs)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "hisilicon,hisi-pmic-irq-mask-addr",
					 pirq->mask_regs, pirq->nbanks);
	if (!ret)
		ret = of_property_read_u32_array(np, "hisilicon,hisi-pmic-irq-addr",
						 pirq->status_regs, pirq->nbanks);
	if (ret)
		return dev_err_probe(dev, ret, "missing interrupt registers\n");

	/* the interrupt line is a plain "gpios" property on the PMIC node */
	gpio = devm_fwnode_gpiod_get_index(dev, dev_fwnode(pmic_dev), NULL, 0,
					   GPIOD_IN, "pmic-irq");
	if (IS_ERR(gpio))
		return dev_err_probe(dev, PTR_ERR(gpio), "no interrupt gpio\n");

	irq = gpiod_to_irq(gpio);
	if (irq < 0)
		return dev_err_probe(dev, irq, "interrupt gpio has no irq\n");

	/* mask everything and clear what is latched, like the vendor kernel */
	for (i = 0; i < pirq->nbanks; i++) {
		ret = regmap_write(pirq->regmap, pirq->mask_regs[i],
				   HISI_PMIC_IRQ_BANK_MASK);
		if (ret)
			return dev_err_probe(dev, ret, "cannot mask interrupts\n");
		regmap_read(pirq->regmap, pirq->status_regs[i], &dummy);
		regmap_write(pirq->regmap, pirq->status_regs[i],
			     HISI_PMIC_IRQ_BANK_MASK);
	}

	pirq->parent_irq = irq;

	/* consumers name the PMIC node as their interrupt-parent */
	pirq->domain = irq_domain_create_linear(dev_fwnode(pmic_dev), pirq->nirqs,
						&hisi_spmi_pmic_irq_domain_ops, pirq);
	if (!pirq->domain)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, hisi_spmi_pmic_irq_remove_domain, pirq);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, hisi_spmi_pmic_irq_thread,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					dev_name(pmic_dev), pirq);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq %d\n", irq);

	dev_info(dev, "%u interrupts in %u banks on irq %d\n",
		 pirq->nirqs, pirq->nbanks, irq);
	return 0;
}

static struct platform_driver hisi_spmi_pmic_irq_driver = {
	.driver = {
		.name = "hisi-spmi-pmic-irq",
	},
	.probe = hisi_spmi_pmic_irq_probe,
};

static int __init hisi_spmi_pmic_init(void)
{
	int ret;

	ret = platform_driver_register(&hisi_spmi_pmic_irq_driver);
	if (ret)
		return ret;

	ret = spmi_driver_register(&hisi_spmi_pmic_driver);
	if (ret)
		platform_driver_unregister(&hisi_spmi_pmic_irq_driver);

	return ret;
}
subsys_initcall(hisi_spmi_pmic_init);

static void __exit hisi_spmi_pmic_exit(void)
{
	spmi_driver_unregister(&hisi_spmi_pmic_driver);
	platform_driver_unregister(&hisi_spmi_pmic_irq_driver);
}
module_exit(hisi_spmi_pmic_exit);

MODULE_DESCRIPTION("HiSilicon SPMI PMIC core (Kirin 990)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hisi-spmi-pmic-irq");
