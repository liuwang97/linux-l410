/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hooks of the HiSilicon Kirin (990) clock driver for other SoC drivers.
 */
#ifndef __LINUX_CLK_KIRIN_H
#define __LINUX_CLK_KIRIN_H

#include <linux/errno.h>
#include <linux/types.h>

struct regmap;

/*
 * Peripheral voltage voting used by "peri_dvfs_sensitive" gates and the
 * "hisilicon,clkdev-dvfs" clocks: @set_volt asks the power controller for
 * voltage level @level (0 = lowest) on behalf of voter @id (the clock-id /
 * hisilicon,clk-devfreq-id of the clock) and returns once it is applied.
 */
struct kirin_clk_perivolt_ops {
	int (*set_volt)(u32 id, u32 level);
};

#if IS_ENABLED(CONFIG_COMMON_CLK_KIRIN990)
/*
 * The PMIC driver hands over its register map; until then the
 * "hisilicon,clk-pmu-gate" clocks (clk_abb_192, clk_pmu32k*, ...) assume
 * that the firmware left them running.
 */
void kirin_clk_set_pmic_regmap(struct regmap *map);
void kirin_clk_set_perivolt_ops(const struct kirin_clk_perivolt_ops *ops);
#else
static inline void kirin_clk_set_pmic_regmap(struct regmap *map) { }
static inline void
kirin_clk_set_perivolt_ops(const struct kirin_clk_perivolt_ops *ops) { }
#endif

#endif /* __LINUX_CLK_KIRIN_H */
