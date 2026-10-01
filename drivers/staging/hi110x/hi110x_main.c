// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Hi110x connectivity driver: module entry.
 *
 * The vendor code consists of a "plat" image (board/power/PCIe/firmware download/BFGX)
 * and a "wifi" image (hmac/wal/cfg80211), which the vendor kernel initialised from
 * userspace through /sys/hisys/boot/{plat,wifi} once /vendor was mounted. Here both
 * live in one module; initialisation runs from a work item once the firmware files
 * are reachable, or on demand through the same sysfs files (autoboot=0).
 */
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/namei.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

/* the vendor code renames its global symbols per chip family ("oneimage") */
#include "platform_oneimage_define.h"
#include "wlan_oneimage_define.h"

int32_t plat_sysfs_init(void);
void plat_sysfs_exit(void);
int32_t wifi_sysfs_init(void);
void wifi_sysfs_exit(void);
int32_t hi110x_boot_plat(void);
int32_t hi110x_boot_wifi(void);
int32_t hi110x_bluez_register(void);

static bool bt = true;
module_param(bt, bool, 0444);
MODULE_PARM_DESC(bt, "Register the Bluetooth HCI device after autoboot");

static bool autoboot = true;
module_param(autoboot, bool, 0444);
MODULE_PARM_DESC(autoboot, "Bring up platform and WiFi automatically once /vendor is available");

static unsigned int fw_wait = 120;
module_param(fw_wait, uint, 0444);
MODULE_PARM_DESC(fw_wait, "Seconds to wait for the firmware directory (autoboot)");

#define HI110X_FW_PROBE_PATH "/vendor/firmware/hi1103/pilot/wifi_cfg"

static struct delayed_work hi110x_boot_work;
static unsigned long hi110x_boot_deadline;

static bool hi110x_fw_present(void)
{
	struct path p;

	if (kern_path(HI110X_FW_PROBE_PATH, LOOKUP_FOLLOW, &p))
		return false;
	path_put(&p);
	return true;
}

static void hi110x_boot_fn(struct work_struct *work)
{
	int ret;

	if (!hi110x_fw_present()) {
		if (time_before(jiffies, hi110x_boot_deadline)) {
			schedule_delayed_work(&hi110x_boot_work, HZ);
			return;
		}
		pr_err("hi110x: %s not found, giving up autoboot\n", HI110X_FW_PROBE_PATH);
		return;
	}

	ret = hi110x_boot_plat();
	if (ret) {
		pr_err("hi110x: platform init failed: %d\n", ret);
		return;
	}
	ret = hi110x_boot_wifi();
	if (ret)
		pr_err("hi110x: wifi init failed: %d\n", ret);

	if (bt) {
		ret = hi110x_bluez_register();
		if (ret)
			pr_err("hi110x: bluetooth registration failed: %d\n", ret);
	}
}

static int __init hi110x_module_init(void)
{
	int ret;

	ret = plat_sysfs_init();
	if (ret)
		return ret;
	ret = wifi_sysfs_init();
	if (ret) {
		plat_sysfs_exit();
		return ret;
	}

	INIT_DELAYED_WORK(&hi110x_boot_work, hi110x_boot_fn);
	if (autoboot) {
		hi110x_boot_deadline = jiffies + fw_wait * HZ;
		schedule_delayed_work(&hi110x_boot_work, 0);
	}
	return 0;
}
module_init(hi110x_module_init);

/*
 * The vendor code has no complete teardown path (it was always built in), so the
 * module cannot be unloaded.
 */

MODULE_DESCRIPTION("HiSilicon Hi110x WiFi/Bluetooth (vendor driver port)");
MODULE_LICENSE("GPL");
