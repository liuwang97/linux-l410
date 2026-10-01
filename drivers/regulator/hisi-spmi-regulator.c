// SPDX-License-Identifier: GPL-2.0
/*
 * Regulators of the HiSilicon SPMI PMIC (Kirin 990 generation)
 *
 * Each LDO/BUCK is its own firmware device tree node below the PMIC:
 *   hisilicon,hisi-ctrl           <reg enable-mask [eco-mask]>
 *   hisilicon,hisi-vset           <reg mask>
 *   hisilicon,hisi-n-voltages     number of selectors
 *   hisilicon,hisi-vset-table     voltage of each selector (uV)
 *   hisilicon,hisi-off-on-delay-us, hisilicon,hisi-enable-time-us
 *   hisilicon,hisi-eco-microamp   load up to which the eco (idle) mode is fine
 *   hisilicon,valid-modes-mask    REGULATOR_MODE_* the regulator supports
 * plus the generic regulator-* constraints.
 *
 * Based on the Huawei vendor driver (drivers/regulator/hisi_regulator_spmi.c).
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

struct hisi_spmi_regulator {
	struct regulator_desc desc;
	u32 eco_mask;
	u32 eco_uA;
};

static unsigned int hisi_spmi_regulator_get_mode(struct regulator_dev *rdev)
{
	struct hisi_spmi_regulator *sreg = rdev_get_drvdata(rdev);
	unsigned int val;

	if (sreg->eco_mask &&
	    !regmap_read(rdev->regmap, rdev->desc->enable_reg, &val) &&
	    (val & sreg->eco_mask))
		return REGULATOR_MODE_IDLE;

	return REGULATOR_MODE_NORMAL;
}

static int hisi_spmi_regulator_set_mode(struct regulator_dev *rdev,
					unsigned int mode)
{
	struct hisi_spmi_regulator *sreg = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
					  sreg->eco_mask, 0);
	case REGULATOR_MODE_IDLE:
		if (!sreg->eco_mask)
			return -EINVAL;
		return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
					  sreg->eco_mask, sreg->eco_mask);
	default:
		return -EINVAL;
	}
}

static unsigned int hisi_spmi_regulator_get_optimum_mode(struct regulator_dev *rdev,
							 int input_uV, int output_uV,
							 int load_uA)
{
	struct hisi_spmi_regulator *sreg = rdev_get_drvdata(rdev);

	if (!sreg->eco_mask || load_uA <= 0 || load_uA > sreg->eco_uA)
		return REGULATOR_MODE_NORMAL;

	return REGULATOR_MODE_IDLE;
}

static const struct regulator_ops hisi_spmi_regulator_ops = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_iterate,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_mode		= hisi_spmi_regulator_get_mode,
	.set_mode		= hisi_spmi_regulator_set_mode,
	.get_optimum_mode	= hisi_spmi_regulator_get_optimum_mode,
};

static int hisi_spmi_regulator_parse_dt(struct device *dev,
					struct hisi_spmi_regulator *sreg)
{
	struct device_node *np = dev->of_node;
	struct regulator_desc *desc = &sreg->desc;
	u32 ctrl[3] = { }, vset[2], n;
	unsigned int *table;
	int count, ret;

	count = of_property_count_u32_elems(np, "hisilicon,hisi-ctrl");
	if (count < 2 || count > 3)
		return dev_err_probe(dev, -EINVAL, "bad hisilicon,hisi-ctrl\n");
	of_property_read_u32_array(np, "hisilicon,hisi-ctrl", ctrl, count);

	ret = of_property_read_u32_array(np, "hisilicon,hisi-vset", vset, 2);
	if (ret || !vset[1])
		return dev_err_probe(dev, -EINVAL, "bad hisilicon,hisi-vset\n");

	count = of_property_count_u32_elems(np, "hisilicon,hisi-vset-table");
	if (of_property_read_u32(np, "hisilicon,hisi-n-voltages", &n) || !n ||
	    count <= 0)
		return dev_err_probe(dev, -EINVAL, "no voltage table\n");
	/* the firmware tables are not always the advertised length */
	n = min_t(u32, n, count);
	n = min_t(u32, n, (vset[1] >> __ffs(vset[1])) + 1);

	table = devm_kcalloc(dev, n, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	of_property_read_u32_array(np, "hisilicon,hisi-vset-table", table, n);

	desc->name = of_get_property(np, "regulator-name", NULL);
	if (!desc->name)
		desc->name = np->name;
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &hisi_spmi_regulator_ops;
	desc->volt_table = table;
	desc->n_voltages = n;
	desc->vsel_reg = vset[0];
	desc->vsel_mask = vset[1];
	desc->enable_reg = ctrl[0];
	desc->enable_mask = ctrl[1];
	sreg->eco_mask = ctrl[2];

	of_property_read_u32(np, "hisilicon,hisi-enable-time-us", &desc->enable_time);
	of_property_read_u32(np, "hisilicon,hisi-off-on-delay-us", &desc->off_on_delay);
	of_property_read_u32(np, "hisilicon,hisi-eco-microamp", &sreg->eco_uA);

	return 0;
}

static int hisi_spmi_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	struct hisi_spmi_regulator *sreg;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	u32 modes;
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");

	sreg = devm_kzalloc(dev, sizeof(*sreg), GFP_KERNEL);
	if (!sreg)
		return -ENOMEM;

	ret = hisi_spmi_regulator_parse_dt(dev, sreg);
	if (ret)
		return ret;

	init_data = of_get_regulator_init_data(dev, dev->of_node, &sreg->desc);
	if (!init_data)
		return -ENOMEM;

	/* vendor property for the NORMAL/IDLE (eco) mode pair */
	if (!of_property_read_u32(dev->of_node, "hisilicon,valid-modes-mask", &modes)) {
		init_data->constraints.valid_modes_mask |= modes;
		if (sreg->eco_mask && hweight32(modes) > 1)
			init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_MODE;
	}

	config.dev = dev;
	config.init_data = init_data;
	config.driver_data = sreg;
	config.of_node = dev->of_node;
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, &sreg->desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "failed to register %s\n",
				     sreg->desc.name);

	return 0;
}

static const struct of_device_id hisi_spmi_regulator_of_match[] = {
	{ .compatible = "hisilicon-hisi-ldo" },
	{ .compatible = "hisilicon-hisi-sub-pmic-ldo" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_spmi_regulator_of_match);

static struct platform_driver hisi_spmi_regulator_driver = {
	.driver = {
		.name		= "hisi-spmi-regulator",
		.of_match_table	= hisi_spmi_regulator_of_match,
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe	= hisi_spmi_regulator_probe,
};
module_platform_driver(hisi_spmi_regulator_driver);

MODULE_DESCRIPTION("HiSilicon SPMI PMIC regulators (Kirin 990)");
MODULE_LICENSE("GPL");
