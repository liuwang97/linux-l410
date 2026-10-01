/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Endpoint-facing API of the HiSilicon Kirin 990 ("kport") PCIe host driver.
 *
 * Endpoint drivers that own the power of their device (the Hi110x WiFi/BT
 * chip on RC1) use this to have the root complex brought up only after the
 * device is powered, to power the link down/up around device power cycles,
 * and to get told about link-down and completion-timeout events.
 *
 * Names and semantics follow the vendor 5.10 header of the same path.
 */
#ifndef _PCIE_KPORT_API_H
#define _PCIE_KPORT_API_H

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/types.h>

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

/* power_ops for pcie_kport_pm_control() */
enum pcie_kport_pm_ops {
	PCIE_KPORT_POWEROFF_BUSON = 0,	/* PME_Turn_Off, then power down */
	PCIE_KPORT_POWERON = 1,		/* power up, retrain, restore RC */
	PCIE_KPORT_POWEROFF_BUSDOWN = 2, /* power down without messages */
	PCIE_KPORT_POWERON_CLK = 3,	/* power up the RC only, no link */
};

/* link_status for pcie_kport_ep_link_ltssm_notify() */
enum {
	PCIE_KPORT_DEVICE_LINK_MIN = 0,
	PCIE_KPORT_DEVICE_LINK_UP = 1,
	PCIE_KPORT_DEVICE_LINK_ABNORMAL = 2,
	PCIE_KPORT_DEVICE_LINK_MAX = 3,
};

struct pcie_kport_notify {
	enum pcie_kport_event event;
	void *user;
	void *data;
	u32 options;
};

struct pcie_kport_register_event {
	u32 events;		/* mask of enum pcie_kport_event */
	void *user;		/* the endpoint's struct pci_dev */
	enum pcie_kport_trigger mode;
	void (*callback)(struct pcie_kport_notify *notify);
	struct pcie_kport_notify notify;
	struct completion *completion;
	u32 options;
};

#if IS_ENABLED(CONFIG_PCIE_KPORT)
int pcie_kport_enumerate(u32 rc_idx);
int pcie_kport_remove_ep(u32 rc_idx);
int pcie_kport_rescan_ep(u32 rc_idx);
int pcie_kport_pm_control(int power_ops, u32 rc_idx);
int pcie_kport_lp_ctrl(u32 rc_idx, u32 enable);
int pcie_kport_power_notifiy_register(u32 rc_id, int (*poweron)(void *data),
				      int (*poweroff)(void *data), void *data);
int pcie_kport_register_event(struct pcie_kport_register_event *reg);
int pcie_kport_deregister_event(struct pcie_kport_register_event *reg);
int pcie_kport_ep_link_ltssm_notify(u32 rc_id, u32 link_status);
void pcie_kport_refclk_device_vote(u32 ep_type, u32 rc_id, u32 vote);
#else
static inline int pcie_kport_enumerate(u32 rc_idx) { return -ENODEV; }
static inline int pcie_kport_remove_ep(u32 rc_idx) { return -ENODEV; }
static inline int pcie_kport_rescan_ep(u32 rc_idx) { return -ENODEV; }
static inline int pcie_kport_pm_control(int power_ops, u32 rc_idx) { return -ENODEV; }
static inline int pcie_kport_lp_ctrl(u32 rc_idx, u32 enable) { return -ENODEV; }
static inline int pcie_kport_power_notifiy_register(u32 rc_id,
						    int (*poweron)(void *data),
						    int (*poweroff)(void *data),
						    void *data)
{
	return -ENODEV;
}
static inline int pcie_kport_register_event(struct pcie_kport_register_event *reg)
{
	return -ENODEV;
}
static inline int pcie_kport_deregister_event(struct pcie_kport_register_event *reg)
{
	return -ENODEV;
}
static inline int pcie_kport_ep_link_ltssm_notify(u32 rc_id, u32 link_status)
{
	return -ENODEV;
}
static inline void pcie_kport_refclk_device_vote(u32 ep_type, u32 rc_id, u32 vote) {}
#endif

#endif /* _PCIE_KPORT_API_H */
