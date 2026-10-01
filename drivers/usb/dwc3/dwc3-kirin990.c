// SPDX-License-Identifier: GPL-2.0-only
/*
 * DWC3 glue for the HiSilicon Kirin 990 USB 3.1 controller.
 *
 * The controller's resets, clocks and the USB 2.0 / USB 3.1 combo PHY are
 * all handled by the PHY driver ("hisilicon,apr-dwc3"). The glue powers the
 * PHY up, which also takes the controller out of reset, and then creates
 * the DWC3 core from the child node. Host mode only.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

struct dwc3_kirin990 {
	struct phy *phy;
};

static int dwc3_kirin990_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc3_kirin990 *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	priv->phy = devm_phy_get(dev, "usb");
	if (IS_ERR(priv->phy))
		return dev_err_probe(dev, PTR_ERR(priv->phy), "no USB PHY\n");

	ret = phy_init(priv->phy);
	if (ret)
		return dev_err_probe(dev, ret, "PHY init failed\n");

	ret = phy_power_on(priv->phy);
	if (ret) {
		dev_err(dev, "PHY power on failed: %d\n", ret);
		goto err_exit;
	}

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to create dwc3 core: %d\n", ret);
		goto err_power_off;
	}
	return 0;

err_power_off:
	phy_power_off(priv->phy);
err_exit:
	phy_exit(priv->phy);
	return ret;
}

static void dwc3_kirin990_remove(struct platform_device *pdev)
{
	struct dwc3_kirin990 *priv = platform_get_drvdata(pdev);

	of_platform_depopulate(&pdev->dev);
	phy_power_off(priv->phy);
	phy_exit(priv->phy);
}

static const struct of_device_id dwc3_kirin990_of_match[] = {
	{ .compatible = "hisilicon,dwc3-usb" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwc3_kirin990_of_match);

static struct platform_driver dwc3_kirin990_driver = {
	.probe	= dwc3_kirin990_probe,
	.remove	= dwc3_kirin990_remove,
	.driver	= {
		.name		= "dwc3-kirin990",
		.of_match_table	= dwc3_kirin990_of_match,
	},
};
module_platform_driver(dwc3_kirin990_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 DWC3 glue");
MODULE_LICENSE("GPL");
