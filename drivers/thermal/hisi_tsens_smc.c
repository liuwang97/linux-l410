// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 on-die temperature sensors
 *
 * The sensors belong to the secure world; the trusted firmware returns the
 * raw ADC code of a sensor through a SiP call. Codes between
 * "hisi,tsensor_adc_start_value" and "hisi,tsensor_adc_end_value" map
 * linearly to -40..125 degC.
 *
 * Firmware device tree (vendor binding):
 *   hisi,tsensor_name = "cluster0", ...;   one name per sensor
 *   hisi,detect_<name>_regno = <n>;        firmware sensor number, 0xff: none
 * Each sensor is a thermal sensor with index = position in
 * hisi,tsensor_name (#thermal-sensor-cells = <1>). Sensors not used by any
 * thermal zone of the device tree get a trip-less zone of the same name, so
 * that every temperature shows up in /sys/class/thermal.
 *
 * Based on the Huawei vendor driver (drivers/thermal/hisi/hisi_tsens.c).
 */

#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#define TSENS_SMC_READTEMP	0xc5009900
#define TSENS_TEMP_MIN_MC	(-40000)
#define TSENS_TEMP_MAX_MC	125000
#define TSENS_REGNO_NONE	0xff
#define TSENS_MAX_SENSORS	16

struct hisi_tsens;

struct hisi_tsens_sensor {
	struct hisi_tsens *tsens;
	const char *name;
	u32 regno;
	struct thermal_zone_device *tzd;
};

struct hisi_tsens {
	struct device *dev;
	u32 adc_start;
	u32 adc_end;
	int nsensors;
	struct hisi_tsens_sensor sensor[] __counted_by(nsensors);
};

static int hisi_tsens_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct hisi_tsens_sensor *s = thermal_zone_device_priv(tz);
	struct hisi_tsens *tsens = s->tsens;
	struct arm_smccc_res res;
	long code;

	arm_smccc_smc(TSENS_SMC_READTEMP, s->regno, 0, 0, 0, 0, 0, 0, &res);
	code = (int)res.a0;

	if (code < tsens->adc_start || code > tsens->adc_end) {
		dev_dbg_ratelimited(tsens->dev, "%s: code %ld out of range\n",
				    s->name, code);
		return -EAGAIN;
	}

	*temp = TSENS_TEMP_MIN_MC +
		(int)div_s64((s64)(code - tsens->adc_start) *
			     (TSENS_TEMP_MAX_MC - TSENS_TEMP_MIN_MC),
			     tsens->adc_end - tsens->adc_start);
	return 0;
}

static const struct thermal_zone_device_ops hisi_tsens_ops = {
	.get_temp = hisi_tsens_get_temp,
};

static void hisi_tsens_unregister_tripless(void *data)
{
	thermal_zone_device_unregister(data);
}

static int hisi_tsens_register(struct hisi_tsens *tsens, int id)
{
	struct hisi_tsens_sensor *s = &tsens->sensor[id];
	struct device *dev = tsens->dev;
	int ret;

	s->tzd = devm_thermal_of_zone_register(dev, id, s, &hisi_tsens_ops);
	if (!IS_ERR(s->tzd))
		return 0;
	if (PTR_ERR(s->tzd) != -ENODEV)
		return dev_err_probe(dev, PTR_ERR(s->tzd), "%s: zone\n", s->name);

	/* no thermal zone in the device tree: just expose the temperature */
	s->tzd = thermal_tripless_zone_device_register(s->name, s,
						       &hisi_tsens_ops, NULL);
	if (IS_ERR(s->tzd))
		return dev_err_probe(dev, PTR_ERR(s->tzd), "%s: tripless zone\n",
				     s->name);

	ret = devm_add_action_or_reset(dev, hisi_tsens_unregister_tripless, s->tzd);
	if (ret)
		return ret;

	return thermal_zone_device_enable(s->tzd);
}

static int hisi_tsens_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct hisi_tsens *tsens;
	char prop[64];
	int n, i, ret;

	n = of_property_count_strings(np, "hisi,tsensor_name");
	if (n <= 0 || n > TSENS_MAX_SENSORS)
		return dev_err_probe(dev, -EINVAL, "bad hisi,tsensor_name\n");

	tsens = devm_kzalloc(dev, struct_size(tsens, sensor, n), GFP_KERNEL);
	if (!tsens)
		return -ENOMEM;
	tsens->nsensors = n;
	tsens->dev = dev;

	if (of_property_read_u32(np, "hisi,tsensor_adc_start_value", &tsens->adc_start) ||
	    of_property_read_u32(np, "hisi,tsensor_adc_end_value", &tsens->adc_end) ||
	    tsens->adc_end <= tsens->adc_start)
		return dev_err_probe(dev, -EINVAL, "bad ADC range\n");

	for (i = 0; i < n; i++) {
		struct hisi_tsens_sensor *s = &tsens->sensor[i];

		s->tsens = tsens;
		s->regno = TSENS_REGNO_NONE;
		of_property_read_string_index(np, "hisi,tsensor_name", i, &s->name);
		snprintf(prop, sizeof(prop), "hisi,detect_%s_regno", s->name);
		of_property_read_u32(np, prop, &s->regno);
		if (s->regno == TSENS_REGNO_NONE)
			continue;

		ret = hisi_tsens_register(tsens, i);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, tsens);
	return 0;
}

static const struct of_device_id hisi_tsens_of_match[] = {
	{ .compatible = "hisi,tsens" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_tsens_of_match);

static struct platform_driver hisi_tsens_driver = {
	.driver = {
		.name		= "hisi-tsens-smc",
		.of_match_table	= hisi_tsens_of_match,
	},
	.probe	= hisi_tsens_probe,
};
module_platform_driver(hisi_tsens_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 temperature sensors (firmware SMC)");
MODULE_LICENSE("GPL");
