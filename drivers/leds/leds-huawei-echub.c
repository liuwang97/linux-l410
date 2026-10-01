// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keyboard mute LED of the Huawei "echub" embedded controller
 *
 * The LED on the speaker mute key (F7) is switched by the EC. The register is
 * given by the "muteled_reg_addr" property of the EC node.
 *
 * Ported from the vendor 4.19 drivers/echub/keyboard/muteled.c.
 */

#include <linux/leds.h>
#include <linux/mfd/huawei-echub.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ECHUB_MUTELED_REG_DEFAULT	0x0277
#define ECHUB_MUTELED_ON		0x5a
#define ECHUB_MUTELED_OFF		0x55

struct echub_led {
	struct huawei_echub *ec;
	struct led_classdev cdev;
	u16 reg;
};

static int echub_led_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct echub_led *led = container_of(cdev, struct echub_led, cdev);

	return huawei_echub_write(led->ec, led->reg,
				  brightness ? ECHUB_MUTELED_ON : ECHUB_MUTELED_OFF);
}

static int echub_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct echub_led *led;
	u32 reg = ECHUB_MUTELED_REG_DEFAULT;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->ec = dev_get_drvdata(dev->parent);
	if (!led->ec)
		return -EPROBE_DEFER;

	of_property_read_u32(dev->parent->of_node, "muteled_reg_addr", &reg);
	if (reg > 0xffff)
		return dev_err_probe(dev, -EINVAL, "bad mute LED register %#x\n", reg);
	led->reg = reg;

	led->cdev.name = "platform::mute";
	led->cdev.max_brightness = 1;
	led->cdev.brightness_set_blocking = echub_led_set;
	led->cdev.default_trigger = "audio-mute";

	return devm_led_classdev_register(dev, &led->cdev);
}

static const struct platform_device_id echub_led_id[] = {
	{ "huawei-echub-led" },
	{ }
};
MODULE_DEVICE_TABLE(platform, echub_led_id);

static struct platform_driver echub_led_driver = {
	.driver = {
		.name = "huawei-echub-led",
	},
	.probe = echub_led_probe,
	.id_table = echub_led_id,
};
module_platform_driver(echub_led_driver);

MODULE_DESCRIPTION("Huawei echub EC keyboard mute LED");
MODULE_LICENSE("GPL");
