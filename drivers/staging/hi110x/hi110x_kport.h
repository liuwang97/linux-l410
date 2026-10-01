/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kirin "kport" PCIe root-complex interface used by the Hi110x host driver.
 *
 * The chip sits behind RC1. The RC driver (drivers/pci/controller/dwc/pcie-kport*)
 * exports the pcie_kport_* API (same as the vendor 5.10 kernel); the 4.19 vendor
 * code calls the older kirin_pcie_* names, mapped here. When the RC driver is not
 * part of the tree the calls fail with -ENODEV, so the driver still builds.
 */
#ifndef __HI110X_KPORT_H__
#define __HI110X_KPORT_H__

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/completion.h>

#if __has_include(<linux/platform_drivers/pcie-kport-api.h>)
#include <linux/platform_drivers/pcie-kport-api.h>
#else
#include <linux/pci.h>

enum pcie_kport_event {
	PCIE_KPORT_EVENT_MIN_INVALID = 0x0,
	PCIE_KPORT_EVENT_LINKUP = 0x1,
	PCIE_KPORT_EVENT_LINKDOWN = 0x2,
	PCIE_KPORT_EVENT_WAKE = 0x4,
	PCIE_KPORT_EVENT_L1SS = 0x8,
	PCIE_KPORT_EVENT_CPL_TIMEOUT = 0x10,
	PCIE_KPORT_EVENT_MAX_INVALID = 0x1F,
};

enum pcie_kport_trigger {
	PCIE_KPORT_TRIGGER_CALLBACK,
	PCIE_KPORT_TRIGGER_COMPLETION,
};

enum {
	PCIE_DEVICE_WLAN = 0,
	PCIE_DEVICE_MAX,
};

struct pcie_kport_notify {
	enum pcie_kport_event event;
	void *user;
	void *data;
	u32 options;
};

struct pcie_kport_register_event {
	u32 events;
	void *user;
	enum pcie_kport_trigger mode;
	void (*callback)(struct pcie_kport_notify *notify);
	struct pcie_kport_notify notify;
	struct completion *completion;
	u32 options;
};

static inline int pcie_kport_register_event(struct pcie_kport_register_event *reg) { return -ENODEV; }
static inline int pcie_kport_deregister_event(struct pcie_kport_register_event *reg) { return -ENODEV; }
static inline int pcie_kport_pm_control(int power_ops, u32 rc_idx) { return -ENODEV; }
static inline int pcie_kport_lp_ctrl(u32 rc_idx, u32 enable) { return -ENODEV; }
static inline int pcie_kport_enumerate(u32 rc_idx) { return -ENODEV; }
static inline int pcie_kport_power_notifiy_register(u32 rc_id, int (*poweron)(void *data),
						    int (*poweroff)(void *data), void *data)
{
	return -ENODEV;
}
static inline void pcie_kport_refclk_device_vote(u32 ep_type, u32 rc_id, u32 vote) { }
#endif

/* only used by the vendor L1.2 self test; the RC driver has no such hook */
static inline u32 show_link_state(u32 rc_id)
{
	return 0;
}

#define KIRIN_PCIE_EVENT_LINKUP		PCIE_KPORT_EVENT_LINKUP
#define KIRIN_PCIE_EVENT_LINKDOWN	PCIE_KPORT_EVENT_LINKDOWN
#define KIRIN_PCIE_EVENT_WAKE		PCIE_KPORT_EVENT_WAKE
#define KIRIN_PCIE_EVENT_L1SS		PCIE_KPORT_EVENT_L1SS
#define KIRIN_PCIE_EVENT_CPL_TIMEOUT	PCIE_KPORT_EVENT_CPL_TIMEOUT
#define KIRIN_PCIE_TRIGGER_CALLBACK	PCIE_KPORT_TRIGGER_CALLBACK
#define KIRIN_PCIE_TRIGGER_COMPLETION	PCIE_KPORT_TRIGGER_COMPLETION

/* functions and struct tags */
#define kirin_pcie_enumerate			pcie_kport_enumerate
#define kirin_pcie_pm_control			pcie_kport_pm_control
#define kirin_pcie_lp_ctrl			pcie_kport_lp_ctrl
#define kirin_pcie_power_notifiy_register	pcie_kport_power_notifiy_register
#define kirin_pcie_register_event		pcie_kport_register_event
#define kirin_pcie_deregister_event		pcie_kport_deregister_event
#define kirin_pcie_notify			pcie_kport_notify
#define kirin_pcie_refclk_device_vote		pcie_kport_refclk_device_vote

#endif /* __HI110X_KPORT_H__ */
