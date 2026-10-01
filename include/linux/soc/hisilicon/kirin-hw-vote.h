/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon Kirin 990 hardware vote block (PMCTRL): frequency/voltage
 * requests to the LPM3 power controller.
 */
#ifndef __LINUX_SOC_HISILICON_KIRIN_HW_VOTE_H
#define __LINUX_SOC_HISILICON_KIRIN_HW_VOTE_H

#include <linux/err.h>
#include <linux/types.h>

struct kirin_hv;

#if IS_ENABLED(CONFIG_KIRIN_HW_VOTE)
struct kirin_hv *kirin_hv_get(const char *channel, const char *src);
struct kirin_hv *kirin_hv_get_reg(u32 offset, u32 rd_mask, u32 wr_mask);
void kirin_hv_set(struct kirin_hv *hv, u32 value);
u32 kirin_hv_get_vote(struct kirin_hv *hv);
u32 kirin_hv_get_result(struct kirin_hv *hv);
#else
static inline struct kirin_hv *kirin_hv_get(const char *channel, const char *src)
{
	return ERR_PTR(-ENODEV);
}
static inline struct kirin_hv *kirin_hv_get_reg(u32 offset, u32 rd_mask, u32 wr_mask)
{
	return ERR_PTR(-ENODEV);
}
static inline void kirin_hv_set(struct kirin_hv *hv, u32 value) { }
static inline u32 kirin_hv_get_vote(struct kirin_hv *hv) { return 0; }
static inline u32 kirin_hv_get_result(struct kirin_hv *hv) { return 0; }
#endif

#endif /* __LINUX_SOC_HISILICON_KIRIN_HW_VOTE_H */
