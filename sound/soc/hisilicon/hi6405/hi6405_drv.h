/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Internal interfaces of the L410 Hi6405 / ASP / SLIMbus audio driver set.
 */
#ifndef __HI6405_DRV_H__
#define __HI6405_DRV_H__

#include <linux/platform_device.h>

#define HI6405_VERSION_REG	0x20007000
#define HI6405_VERSION_CS	0x11
#define HI6405_CHIP_ID_REG0	0x20007092

extern struct platform_driver hi6405_ctrl_driver;
extern struct platform_driver hi64xx_irq_driver;
extern struct platform_driver hi6405_codec_driver;
extern struct platform_driver hisi_slimbus_driver;
extern struct platform_driver asp_pcm_driver;
extern struct platform_driver hi6405_card_driver;

/* SLIMbus manager is up and the codec framer can be enumerated */
bool slimbus_is_ready(void);

#endif /* __HI6405_DRV_H__ */
