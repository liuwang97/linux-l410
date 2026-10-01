// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei L410 (Kirin 990) audio: Hi6405 codec on SSI + SLIMbus, ASP DMA,
 * TAS2562 smart PAs. Registers the platform drivers of this module in
 * dependency order.
 */
#include <linux/module.h>
#include <linux/platform_device.h>

#include "hi6405_drv.h"

static struct platform_driver * const hi6405_drivers[] = {
	&hisi_slimbus_driver,
	&hi6405_ctrl_driver,
	&hi64xx_irq_driver,
	&hi6405_codec_driver,
	&asp_pcm_driver,
	&hi6405_card_driver,
};

static int __init hi6405_audio_init(void)
{
	return platform_register_drivers(hi6405_drivers, ARRAY_SIZE(hi6405_drivers));
}
module_init(hi6405_audio_init);

static void __exit hi6405_audio_exit(void)
{
	platform_unregister_drivers(hi6405_drivers, ARRAY_SIZE(hi6405_drivers));
}
module_exit(hi6405_audio_exit);

MODULE_DESCRIPTION("Huawei L410 Hi6405/ASP/SLIMbus audio");
MODULE_LICENSE("GPL");
