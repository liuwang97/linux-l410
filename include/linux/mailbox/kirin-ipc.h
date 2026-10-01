/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon Kirin IPC mailbox (HiIPCV230) client helpers.
 *
 * Channels are named after the vendor "rproc" strings found in the firmware
 * device tree, e.g. "HISI_ACPU_LPM3_MBX_1" (ACPU -> LPM3) or
 * "HISI_LPM3_ACPU_MBX_1" (LPM3 -> ACPU).
 *
 * Mainline mailbox clients may also request a channel through the regular
 * mailbox API; messages are passed as struct kirin_ipc_msg.
 */
#ifndef __LINUX_MAILBOX_KIRIN_IPC_H
#define __LINUX_MAILBOX_KIRIN_IPC_H

#include <linux/errno.h>
#include <linux/types.h>

struct notifier_block;

#define KIRIN_IPC_MAX_WORDS	8

/* payload passed to mbox_send_message() and to rx callbacks */
struct kirin_ipc_msg {
	const u32 *data;
	unsigned int len;	/* in 32-bit words */
};

#if IS_ENABLED(CONFIG_KIRIN_IPC_MBOX)
/*
 * Send @len words and wait for the remote acknowledge. If @ack is given, the
 * acknowledge payload (the mailbox data registers written back by the remote)
 * is copied to it. May sleep. Returns -EPROBE_DEFER until the IPC controller
 * that owns @mbox has probed.
 */
int kirin_ipc_send(const char *mbox, const u32 *msg, unsigned int len,
		   u32 *ack, unsigned int ack_len);
/* Queue a message; callable from any context. The acknowledge is dropped. */
int kirin_ipc_send_async(const char *mbox, const u32 *msg, unsigned int len);
/*
 * Receive messages on a remote -> ACPU mailbox. The notifier is called in
 * process context with action = number of words and data = u32 *payload.
 */
int kirin_ipc_register_rx(const char *mbox, struct notifier_block *nb);
int kirin_ipc_unregister_rx(const char *mbox, struct notifier_block *nb);
#else
static inline int kirin_ipc_send(const char *mbox, const u32 *msg,
				 unsigned int len, u32 *ack,
				 unsigned int ack_len)
{
	return -ENODEV;
}

static inline int kirin_ipc_send_async(const char *mbox, const u32 *msg,
				       unsigned int len)
{
	return -ENODEV;
}

static inline int kirin_ipc_register_rx(const char *mbox,
					struct notifier_block *nb)
{
	return -ENODEV;
}

static inline int kirin_ipc_unregister_rx(const char *mbox,
					  struct notifier_block *nb)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_MAILBOX_KIRIN_IPC_H */
