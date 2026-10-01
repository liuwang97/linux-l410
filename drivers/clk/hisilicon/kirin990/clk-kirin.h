/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon Kirin 990 clock driver: shared definitions.
 *
 * The firmware device tree describes every clock as its own node under
 * /clocks@0, grouped by register block (clk-crgctrl, clk-sctrl, media1-crg,
 * ...). The container nodes are disabled; the clocks are registered from
 * CLK_OF_DECLARE() callbacks at time_init.
 */
#ifndef __CLK_KIRIN_H
#define __CLK_KIRIN_H

#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/spinlock.h>

enum kirin_crg {
	KIRIN_PMCTRL,
	KIRIN_SCTRL,
	KIRIN_CRGCTRL,
	KIRIN_PMUCTRL,
	KIRIN_PCTRL,
	KIRIN_MEDIACRG,
	KIRIN_IOMCUCRG,
	KIRIN_MEDIA1CRG,
	KIRIN_MEDIA2CRG,
	KIRIN_MMC1CRG,
	KIRIN_HSDTCRG,
	KIRIN_MMC0CRG,
	KIRIN_HSDT1CRG,
	KIRIN_CRG_MAX,
};

#define KIRIN_LPM3_CMD_LEN	2
#define KIRIN_LPM3_MBOX		"HISI_ACPU_LPM3_MBX_1"

extern spinlock_t kirin_clk_lock;
extern bool kirin_clk_keep_on;

/* base of the register block a clock node lives in (its parent container) */
void __iomem *kirin_clk_parent_base(struct device_node *np);
/* base of a system controller block, by type ("hisilicon,sysctrl", ...) */
void __iomem *kirin_clk_base(enum kirin_crg type);

const char *kirin_clk_name(struct device_node *np);
int kirin_clk_register(struct device_node *np, struct clk_hw *hw);

/* resolve a "clock-friend-names" clock; call with the prepare lock held */
struct clk *kirin_clk_friend(const char *name, struct clk **cache);

int kirin_clk_perivolt_set(u32 id, u32 level);

#endif
