/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compatibility glue for the vendor Hi110x code (written for 4.19) on current kernels.
 * Force-included into every object of the driver (see Makefile).
 */
#ifndef __HI110X_COMPAT_H__
#define __HI110X_COMPAT_H__

#include <linux/version.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/time64.h>
#include <linux/rtc.h>
#include <linux/sched/clock.h>
#include <linux/dma-direction.h>
#include <linux/etherdevice.h>
#include <linux/panic_notifier.h>

/* ---- set_fs() is gone: all file I/O below goes through kernel_read/kernel_write ---- */
typedef struct { int seg; } mm_segment_t;
#define KERNEL_DS ((mm_segment_t){ 0 })
#define USER_DS   ((mm_segment_t){ 1 })
static inline mm_segment_t get_fs(void) { return KERNEL_DS; }
static inline void set_fs(mm_segment_t fs) { }

/* ---- old time types (removed from the kernel in 5.6) ---- */
struct timeval {
	long tv_sec;
	long tv_usec;
};

struct timespec {
	long tv_sec;
	long tv_nsec;
};

static inline void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec / NSEC_PER_USEC;
}

static inline struct timespec ktime_to_timespec(ktime_t kt)
{
	struct timespec64 t = ktime_to_timespec64(kt);

	return (struct timespec){ .tv_sec = t.tv_sec, .tv_nsec = t.tv_nsec };
}

static inline struct timespec current_kernel_time(void)
{
	struct timespec64 t;

	ktime_get_coarse_real_ts64(&t);
	return (struct timespec){ .tv_sec = t.tv_sec, .tv_nsec = t.tv_nsec };
}

#define rtc_time_to_tm(t, tm)	rtc_time64_to_tm((time64_t)(t), (tm))

/* ---- timers (6.2 / 6.16 renames) ---- */
#define del_timer(t)		timer_delete(t)
#define del_timer_sync(t)	timer_delete_sync(t)
#define from_timer(var, cb, field) timer_container_of(var, cb, field)

/*
 * ---- old PM QoS classes (removed in 5.7) ----
 * CPU_DMA_LATENCY maps to the CPU latency QoS; the vendor-only DDR/memory and network
 * latency classes have no equivalent and become no-ops.
 */
#include <linux/pm_qos.h>
#define PM_QOS_CPU_DMA_LATENCY	1
#define PM_QOS_NETWORK_LATENCY	2
#define PM_QOS_MEMORY_LATENCY	3

static inline void hi110x_pm_qos_add_request(struct pm_qos_request *req, int pm_qos_class, s32 value)
{
	if (pm_qos_class == PM_QOS_CPU_DMA_LATENCY && !cpu_latency_qos_request_active(req))
		cpu_latency_qos_add_request(req, value == PM_QOS_DEFAULT_VALUE ?
					    PM_QOS_CPU_LATENCY_DEFAULT_VALUE : value);
}

static inline void hi110x_pm_qos_update_request(struct pm_qos_request *req, s32 value)
{
	if (cpu_latency_qos_request_active(req))
		cpu_latency_qos_update_request(req, value == PM_QOS_DEFAULT_VALUE ?
					       PM_QOS_CPU_LATENCY_DEFAULT_VALUE : value);
}

static inline void hi110x_pm_qos_remove_request(struct pm_qos_request *req)
{
	if (cpu_latency_qos_request_active(req))
		cpu_latency_qos_remove_request(req);
}

#define pm_qos_add_request(r, c, v)	hi110x_pm_qos_add_request(r, c, v)
#define pm_qos_update_request(r, v)	hi110x_pm_qos_update_request(r, v)
#define pm_qos_remove_request(r)	hi110x_pm_qos_remove_request(r)
static inline int pm_qos_add_notifier(int pm_qos_class, struct notifier_block *nb) { return 0; }
static inline int pm_qos_remove_notifier(int pm_qos_class, struct notifier_block *nb) { return 0; }

/* ---- misc removed helpers ---- */
#define random_ether_addr(a)	eth_random_addr(a)
#define ioremap_nocache(a, s)	ioremap(a, s)
#define netif_rx_ni(skb)	netif_rx(skb)
#define PTR_RET(p)		PTR_ERR_OR_ZERO(p)
#define PCI_DMA_BIDIRECTIONAL	DMA_BIDIRECTIONAL
#define PCI_DMA_TODEVICE	DMA_TO_DEVICE
#define PCI_DMA_FROMDEVICE	DMA_FROM_DEVICE
#define PCI_DMA_NONE		DMA_NONE

/*
 * SecureC subset (hi110x_compat.c). Renamed so that other ported vendor drivers
 * carrying their own copy can never clash with this one.
 */
#define memcpy_s	hi110x_memcpy_s
#define memmove_s	hi110x_memmove_s
#define memset_s	hi110x_memset_s
#define strcpy_s	hi110x_strcpy_s
#define strncpy_s	hi110x_strncpy_s
#define strcat_s	hi110x_strcat_s
#define strncat_s	hi110x_strncat_s
#define snprintf_s	hi110x_snprintf_s
#define vsnprintf_s	hi110x_vsnprintf_s
#define sprintf_s	hi110x_sprintf_s
#define vsprintf_s	hi110x_vsprintf_s
#define sscanf_s	hi110x_sscanf_s
#define vsscanf_s	hi110x_vsscanf_s

#endif /* __HI110X_COMPAT_H__ */
