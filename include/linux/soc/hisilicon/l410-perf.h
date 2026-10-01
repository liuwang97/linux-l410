/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Huawei L410 performance modes and interaction boosts
 * (drivers/soc/hisilicon/l410-perf.c).
 */
#ifndef __LINUX_SOC_HISILICON_L410_PERF_H
#define __LINUX_SOC_HISILICON_L410_PERF_H

#if IS_ENABLED(CONFIG_L410_PERF)
/* a new frame went to the display: keeps an animation boost alive */
void l410_perf_frame(void);
#else
static inline void l410_perf_frame(void) { }
#endif

#endif /* __LINUX_SOC_HISILICON_L410_PERF_H */
