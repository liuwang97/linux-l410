/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2019 Collabora ltd. */

#ifndef __PANFROST_DEVFREQ_H__
#define __PANFROST_DEVFREQ_H__

#include <linux/devfreq.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/pm_qos.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>

struct devfreq;
struct dma_fence;
struct seq_file;
struct thermal_cooling_device;

struct panfrost_device;

enum panfrost_boost_reason {
	PANFROST_BOOST_IDLE,		/* first job after an idle period */
	PANFROST_BOOST_DEADLINE,	/* fence close to its deadline */
	PANFROST_BOOST_WAIT,		/* the CPU blocks on the GPU */
	PANFROST_BOOST_NR,
};

struct panfrost_devfreq {
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct devfreq_simple_ondemand_data gov_data;
	bool opp_of_table_added;

	unsigned long current_frequency;
	unsigned long fast_rate;

	ktime_t busy_time;
	ktime_t idle_time;
	ktime_t time_last_update;
	int busy_count;
	/*
	 * Protect busy_time, idle_time, time_last_update and busy_count
	 * because these can be updated concurrently between multiple jobs.
	 */
	spinlock_t lock;
	ktime_t idle_start;

	/*
	 * Boosts, after drivers/gpu/drm/msm/msm_gpu_devfreq.c: a temporary
	 * PM QoS minimum at a multiple of the current frequency, released
	 * after one polling period. Applied from a SCHED_FIFO worker, as the
	 * triggers run in atomic context or on the job submission path.
	 */
	struct dev_pm_qos_request boost_req;
	bool boost_req_added;
	struct kthread_worker *worker;
	struct kthread_work boost_work;
	struct kthread_delayed_work boost_release_work;
	unsigned int boost_factor;	/* pending, under lock */
	unsigned long boost_khz;	/* current request */
	unsigned long boosts[PANFROST_BOOST_NR];

	/* fence deadline boost, after drivers/gpu/drm/msm/msm_fence.c */
	spinlock_t deadline_lock;
	struct hrtimer deadline_timer;
	struct kthread_work deadline_work;
	struct dma_fence *deadline_fence;
	ktime_t next_deadline;
	unsigned long deadlines;
};

int panfrost_devfreq_init(struct panfrost_device *pfdev);
void panfrost_devfreq_fini(struct panfrost_device *pfdev);

void panfrost_devfreq_resume(struct panfrost_device *pfdev);
void panfrost_devfreq_suspend(struct panfrost_device *pfdev);

void panfrost_devfreq_record_busy(struct panfrost_devfreq *devfreq);
void panfrost_devfreq_record_idle(struct panfrost_devfreq *devfreq);

void panfrost_devfreq_boost(struct panfrost_devfreq *pfdevfreq,
			    enum panfrost_boost_reason reason);
void panfrost_devfreq_set_deadline(struct panfrost_devfreq *pfdevfreq,
				   struct dma_fence *fence, ktime_t deadline);
void panfrost_devfreq_debugfs_show(struct panfrost_devfreq *pfdevfreq,
				   struct seq_file *m);

#endif /* __PANFROST_DEVFREQ_H__ */
