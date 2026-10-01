/* SPDX-License-Identifier: GPL-2.0 */
/* Rate limited SLIMbus logging (replaces the vendor counter based macros). */
#ifndef __SLIMBUS_DEBUG_H__
#define __SLIMBUS_DEBUG_H__

#include <linux/printk.h>

extern volatile uint32_t lostms_times;
extern uint32_t slimbus_rdwrerr_times;

#define slimbus_limit_err(msg, ...)	pr_err_ratelimited("slimbus: %s: " msg, __func__, ##__VA_ARGS__)
#define slimbus_limit_info(msg, ...)	pr_info_ratelimited("slimbus: %s: " msg, __func__, ##__VA_ARGS__)
#define slimbus_recover_info(msg, ...)	do { } while (0)
#define slimbus_drv_limit_err(msg, ...)	pr_err_ratelimited("slimbus: %s: " msg, __func__, ##__VA_ARGS__)
#define slimbus_dev_limit_info(msg, ...) pr_info_ratelimited("slimbus: %s: " msg, __func__, ##__VA_ARGS__)
#define slimbus_dev_limit_err(msg, ...)	pr_err_ratelimited("slimbus: %s: " msg, __func__, ##__VA_ARGS__)
#define slimbus_dev_lostms_recover(msg, ...) \
	do { \
		if (lostms_times > 0) \
			pr_info("slimbus: " msg, ##__VA_ARGS__); \
		lostms_times = 0; \
	} while (0)
#define slimbus_core_limit_err(msg, ...) pr_err_ratelimited("slimbus: %s: " msg, __func__, ##__VA_ARGS__)

#endif
