// SPDX-License-Identifier: GPL-2.0
/* Copyright 2019 Collabora ltd. */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/dma-fence.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/units.h>

#include "panfrost_device.h"
#include "panfrost_devfreq.h"

/*
 * Boost factors (0 or 1: off). Utilisation based DVFS misses short,
 * deadline-bound work such as a compositor frame: the GPU is busy a few
 * percent of the time and stays at the lowest OPP. As on msm, the first job
 * after an idle polling period, a fence about to miss its deadline and a CPU
 * blocking on the GPU each ask for a multiple of the current frequency for
 * one polling period.
 */
static unsigned int idle_boost = 2;
module_param(idle_boost, uint, 0644);
MODULE_PARM_DESC(idle_boost, "Frequency factor for the first job after an idle polling period (0: off)");

static unsigned int deadline_boost = 2;
module_param(deadline_boost, uint, 0644);
MODULE_PARM_DESC(deadline_boost, "Frequency factor for a fence not done shortly before its deadline (0: off)");

static unsigned int deadline_margin_us = 3000;
module_param(deadline_margin_us, uint, 0644);
MODULE_PARM_DESC(deadline_margin_us, "How long before a fence deadline to check it (us)");

static unsigned int wait_boost = 2;
module_param(wait_boost, uint, 0644);
MODULE_PARM_DESC(wait_boost, "Frequency factor when the CPU waits for a busy buffer (0: off)");

static void panfrost_devfreq_update_utilization(struct panfrost_devfreq *pfdevfreq)
{
	ktime_t now, last;

	now = ktime_get();
	last = pfdevfreq->time_last_update;

	if (pfdevfreq->busy_count > 0)
		pfdevfreq->busy_time += ktime_sub(now, last);
	else
		pfdevfreq->idle_time += ktime_sub(now, last);

	pfdevfreq->time_last_update = now;
}

static int panfrost_devfreq_target(struct device *dev, unsigned long *freq,
				   u32 flags)
{
	struct panfrost_device *pfdev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	err = dev_pm_opp_set_rate(dev, *freq);
	if (!err)
		pfdev->pfdevfreq.current_frequency = *freq;

	return err;
}

static void panfrost_devfreq_reset(struct panfrost_devfreq *pfdevfreq)
{
	pfdevfreq->busy_time = 0;
	pfdevfreq->idle_time = 0;
	pfdevfreq->time_last_update = ktime_get();
}

static int panfrost_devfreq_get_dev_status(struct device *dev,
					   struct devfreq_dev_status *status)
{
	struct panfrost_device *pfdev = dev_get_drvdata(dev);
	struct panfrost_devfreq *pfdevfreq = &pfdev->pfdevfreq;
	unsigned long irqflags;

	status->current_frequency = clk_get_rate(pfdev->clock);

	spin_lock_irqsave(&pfdevfreq->lock, irqflags);

	panfrost_devfreq_update_utilization(pfdevfreq);

	status->total_time = ktime_to_ns(ktime_add(pfdevfreq->busy_time,
						   pfdevfreq->idle_time));

	status->busy_time = ktime_to_ns(pfdevfreq->busy_time);

	panfrost_devfreq_reset(pfdevfreq);

	spin_unlock_irqrestore(&pfdevfreq->lock, irqflags);

	dev_dbg(pfdev->dev, "busy %lu total %lu %lu %% freq %lu MHz\n",
		status->busy_time, status->total_time,
		status->busy_time / (status->total_time / 100),
		status->current_frequency / 1000 / 1000);

	return 0;
}

static struct devfreq_dev_profile panfrost_devfreq_profile = {
	.timer = DEVFREQ_TIMER_DELAYED,
	.polling_ms = 50, /* ~3 frames */
	.target = panfrost_devfreq_target,
	.get_dev_status = panfrost_devfreq_get_dev_status,
};

/* ------------------------------------------------------------------------ */
/* boosts */

static unsigned int panfrost_boost_factor(enum panfrost_boost_reason reason)
{
	switch (reason) {
	case PANFROST_BOOST_IDLE:
		return READ_ONCE(idle_boost);
	case PANFROST_BOOST_DEADLINE:
		return READ_ONCE(deadline_boost);
	case PANFROST_BOOST_WAIT:
		return READ_ONCE(wait_boost);
	default:
		return 0;
	}
}

/* may be called from atomic context */
void panfrost_devfreq_boost(struct panfrost_devfreq *pfdevfreq,
			    enum panfrost_boost_reason reason)
{
	unsigned int factor = panfrost_boost_factor(reason);
	unsigned long irqflags;

	if (!pfdevfreq->worker || factor < 2)
		return;

	spin_lock_irqsave(&pfdevfreq->lock, irqflags);
	pfdevfreq->boost_factor = max(pfdevfreq->boost_factor, factor);
	pfdevfreq->boosts[reason]++;
	spin_unlock_irqrestore(&pfdevfreq->lock, irqflags);

	kthread_queue_work(pfdevfreq->worker, &pfdevfreq->boost_work);
}

static void panfrost_devfreq_boost_work(struct kthread_work *work)
{
	struct panfrost_devfreq *pfdevfreq =
		container_of(work, struct panfrost_devfreq, boost_work);
	unsigned long irqflags, khz;
	unsigned int factor;

	spin_lock_irqsave(&pfdevfreq->lock, irqflags);
	factor = pfdevfreq->boost_factor;
	pfdevfreq->boost_factor = 0;
	spin_unlock_irqrestore(&pfdevfreq->lock, irqflags);
	if (!factor)
		return;

	/*
	 * Relative to the current frequency, so that repeated boosts escalate
	 * towards the fastest OPP. PM QoS works in kHz, devfreq in Hz.
	 */
	khz = READ_ONCE(pfdevfreq->current_frequency) / HZ_PER_KHZ * factor;
	pfdevfreq->boost_khz = khz;
	dev_pm_qos_update_request(&pfdevfreq->boost_req, khz);
	kthread_mod_delayed_work(pfdevfreq->worker, &pfdevfreq->boost_release_work,
				 msecs_to_jiffies(pfdevfreq->devfreq->profile->polling_ms));
}

static void panfrost_devfreq_boost_release(struct kthread_work *work)
{
	struct panfrost_devfreq *pfdevfreq =
		container_of(work, struct panfrost_devfreq, boost_release_work.work);

	pfdevfreq->boost_khz = 0;
	dev_pm_qos_update_request(&pfdevfreq->boost_req,
				  PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
}

/*
 * Fence deadlines (dma_fence_set_deadline(), e.g. KWin's SYNC_IOC_SET_DEADLINE
 * on its frame, forwarded by drm_sched to our hardware fence): check the
 * fence with the earliest deadline shortly before that deadline, boost if it
 * has not signalled. Only one fence is tracked, as on msm.
 */
void panfrost_devfreq_set_deadline(struct panfrost_devfreq *pfdevfreq,
				   struct dma_fence *fence, ktime_t deadline)
{
	struct dma_fence *old = NULL;
	unsigned long irqflags;
	unsigned int margin = READ_ONCE(deadline_margin_us);

	if (!pfdevfreq->worker || READ_ONCE(deadline_boost) < 2)
		return;

	deadline = ktime_sub(deadline, us_to_ktime(margin));

	spin_lock_irqsave(&pfdevfreq->deadline_lock, irqflags);
	pfdevfreq->deadlines++;
	if (pfdevfreq->deadline_fence &&
	    !dma_fence_is_signaled(pfdevfreq->deadline_fence) &&
	    !ktime_before(deadline, pfdevfreq->next_deadline)) {
		/* an earlier deadline is pending already */
		spin_unlock_irqrestore(&pfdevfreq->deadline_lock, irqflags);
		return;
	}

	if (pfdevfreq->deadline_fence != fence) {
		old = pfdevfreq->deadline_fence;
		pfdevfreq->deadline_fence = dma_fence_get(fence);
	}
	pfdevfreq->next_deadline = deadline;

	if (ktime_after(ktime_get(), deadline)) {
		hrtimer_try_to_cancel(&pfdevfreq->deadline_timer);
		kthread_queue_work(pfdevfreq->worker, &pfdevfreq->deadline_work);
	} else {
		hrtimer_start(&pfdevfreq->deadline_timer, deadline, HRTIMER_MODE_ABS);
	}
	spin_unlock_irqrestore(&pfdevfreq->deadline_lock, irqflags);

	dma_fence_put(old);
}

static enum hrtimer_restart panfrost_devfreq_deadline_timer(struct hrtimer *t)
{
	struct panfrost_devfreq *pfdevfreq =
		container_of(t, struct panfrost_devfreq, deadline_timer);

	kthread_queue_work(pfdevfreq->worker, &pfdevfreq->deadline_work);
	return HRTIMER_NORESTART;
}

static void panfrost_devfreq_deadline_work(struct kthread_work *work)
{
	struct panfrost_devfreq *pfdevfreq =
		container_of(work, struct panfrost_devfreq, deadline_work);
	struct dma_fence *fence;
	unsigned long irqflags;

	spin_lock_irqsave(&pfdevfreq->deadline_lock, irqflags);
	fence = pfdevfreq->deadline_fence;
	pfdevfreq->deadline_fence = NULL;
	spin_unlock_irqrestore(&pfdevfreq->deadline_lock, irqflags);

	if (fence && !dma_fence_is_signaled(fence))
		panfrost_devfreq_boost(pfdevfreq, PANFROST_BOOST_DEADLINE);
	dma_fence_put(fence);
}

static void panfrost_devfreq_boost_init(struct panfrost_devfreq *pfdevfreq,
					struct device *dev)
{
	struct kthread_worker *worker;
	int ret;

	spin_lock_init(&pfdevfreq->deadline_lock);
	hrtimer_setup(&pfdevfreq->deadline_timer, panfrost_devfreq_deadline_timer,
		      CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	kthread_init_work(&pfdevfreq->boost_work, panfrost_devfreq_boost_work);
	kthread_init_delayed_work(&pfdevfreq->boost_release_work,
				  panfrost_devfreq_boost_release);
	kthread_init_work(&pfdevfreq->deadline_work, panfrost_devfreq_deadline_work);

	/* devfreq aggregates the PM QoS requests of its parent device */
	ret = dev_pm_qos_add_request(dev, &pfdevfreq->boost_req,
				     DEV_PM_QOS_MIN_FREQUENCY,
				     PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
	if (ret < 0) {
		DRM_DEV_INFO(dev, "no frequency boosts: %d\n", ret);
		return;
	}
	pfdevfreq->boost_req_added = true;

	worker = kthread_run_worker(0, "panfrost-boost");
	if (IS_ERR(worker)) {
		DRM_DEV_INFO(dev, "no frequency boosts: %pe\n", worker);
		return;
	}
	/* like msm's gpu-worker: don't queue behind the work it speeds up */
	sched_set_fifo_low(worker->task);
	pfdevfreq->worker = worker;
}

static void panfrost_devfreq_boost_fini(struct panfrost_devfreq *pfdevfreq)
{
	struct kthread_worker *worker = pfdevfreq->worker;

	if (worker) {
		pfdevfreq->worker = NULL;
		hrtimer_cancel(&pfdevfreq->deadline_timer);
		kthread_cancel_work_sync(&pfdevfreq->deadline_work);
		kthread_cancel_work_sync(&pfdevfreq->boost_work);
		kthread_cancel_delayed_work_sync(&pfdevfreq->boost_release_work);
		kthread_destroy_worker(worker);
	}
	dma_fence_put(pfdevfreq->deadline_fence);
	pfdevfreq->deadline_fence = NULL;
	if (pfdevfreq->boost_req_added) {
		dev_pm_qos_remove_request(&pfdevfreq->boost_req);
		pfdevfreq->boost_req_added = false;
	}
}

void panfrost_devfreq_debugfs_show(struct panfrost_devfreq *pfdevfreq,
				   struct seq_file *m)
{
	if (!pfdevfreq->devfreq)
		return;

	seq_printf(m, "freq %lu kHz boost %lu kHz\n",
		   pfdevfreq->current_frequency / HZ_PER_KHZ, pfdevfreq->boost_khz);
	seq_printf(m, "boosts idle %lu deadline %lu wait %lu, deadlines seen %lu\n",
		   pfdevfreq->boosts[PANFROST_BOOST_IDLE],
		   pfdevfreq->boosts[PANFROST_BOOST_DEADLINE],
		   pfdevfreq->boosts[PANFROST_BOOST_WAIT], pfdevfreq->deadlines);
}

static int panfrost_read_speedbin(struct device *dev)
{
	u32 val;
	int ret;

	ret = nvmem_cell_read_variable_le_u32(dev, "speed-bin", &val);
	if (ret) {
		/*
		 * -ENOENT means that this platform doesn't support speedbins
		 * as it didn't declare any speed-bin nvmem: in this case, we
		 * keep going without it; any other error means that we are
		 * supposed to read the bin value, but we failed doing so.
		 */
		if (ret != -ENOENT && ret != -EOPNOTSUPP) {
			DRM_DEV_ERROR(dev, "Cannot read speed-bin (%d).", ret);
			return ret;
		}

		return 0;
	}
	DRM_DEV_DEBUG(dev, "Using speed-bin = 0x%x\n", val);

	return devm_pm_opp_set_supported_hw(dev, &val, 1);
}

int panfrost_devfreq_init(struct panfrost_device *pfdev)
{
	int ret;
	struct dev_pm_opp *opp;
	unsigned long cur_freq;
	struct device *dev = &pfdev->pdev->dev;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct panfrost_devfreq *pfdevfreq = &pfdev->pfdevfreq;
	unsigned long freq = ULONG_MAX;

	if (pfdev->comp->num_supplies > 1) {
		/*
		 * GPUs with more than 1 supply require platform-specific handling:
		 * continue without devfreq
		 */
		DRM_DEV_INFO(dev, "More than 1 supply is not supported yet\n");
		return 0;
	}

	ret = panfrost_read_speedbin(dev);
	if (ret)
		return ret;

	ret = pfdev->comp->supplies_are_switches ? 0 :
	      devm_pm_opp_set_regulators(dev, pfdev->comp->supply_names);
	if (ret) {
		/* Continue if the optional regulator is missing */
		if (ret != -ENODEV) {
			if (ret != -EPROBE_DEFER)
				DRM_DEV_ERROR(dev, "Couldn't set OPP regulators\n");
			return ret;
		}
	}

	ret = devm_pm_opp_of_add_table(dev);
	if (ret) {
		/* Optional, continue without devfreq */
		if (ret == -ENODEV)
			ret = 0;
		return ret;
	}
	pfdevfreq->opp_of_table_added = true;

	spin_lock_init(&pfdevfreq->lock);

	panfrost_devfreq_reset(pfdevfreq);

	cur_freq = clk_get_rate(pfdev->clock);

	opp = devfreq_recommended_opp(dev, &cur_freq, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	panfrost_devfreq_profile.initial_freq = cur_freq;

	/*
	 * We could wait until panfrost_devfreq_target() to set this value, but
	 * since the simple_ondemand governor works asynchronously, there's a
	 * chance by the time someone opens the device's fdinfo file, current
	 * frequency hasn't been updated yet, so let's just do an early set.
	 */
	pfdevfreq->current_frequency = cur_freq;

	/*
	 * Set the recommend OPP this will enable and configure the regulator
	 * if any and will avoid a switch off by regulator_late_cleanup()
	 */
	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);
	if (ret) {
		DRM_DEV_ERROR(dev, "Couldn't set recommended OPP\n");
		return ret;
	}

	/* Find the fastest defined rate  */
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	pfdevfreq->fast_rate = freq;

	dev_pm_opp_put(opp);

	/*
	 * Setup default thresholds for the simple_ondemand governor.
	 * The values are chosen based on experiments.
	 */
	pfdevfreq->gov_data.upthreshold = 45;
	pfdevfreq->gov_data.downdifferential = 5;

	devfreq = devm_devfreq_add_device(dev, &panfrost_devfreq_profile,
					  DEVFREQ_GOV_SIMPLE_ONDEMAND,
					  &pfdevfreq->gov_data);
	if (IS_ERR(devfreq)) {
		DRM_DEV_ERROR(dev, "Couldn't initialize GPU devfreq\n");
		return PTR_ERR(devfreq);
	}
	pfdevfreq->devfreq = devfreq;

	cooling = devfreq_cooling_em_register(devfreq, NULL);
	if (IS_ERR(cooling))
		DRM_DEV_INFO(dev, "Failed to register cooling device\n");
	else
		pfdevfreq->cooling = cooling;

	panfrost_devfreq_boost_init(pfdevfreq, dev);

	return 0;
}

void panfrost_devfreq_fini(struct panfrost_device *pfdev)
{
	struct panfrost_devfreq *pfdevfreq = &pfdev->pfdevfreq;

	panfrost_devfreq_boost_fini(pfdevfreq);

	if (pfdevfreq->cooling) {
		devfreq_cooling_unregister(pfdevfreq->cooling);
		pfdevfreq->cooling = NULL;
	}
}

void panfrost_devfreq_resume(struct panfrost_device *pfdev)
{
	struct panfrost_devfreq *pfdevfreq = &pfdev->pfdevfreq;

	if (!pfdevfreq->devfreq)
		return;

	panfrost_devfreq_reset(pfdevfreq);

	devfreq_resume_device(pfdevfreq->devfreq);
}

void panfrost_devfreq_suspend(struct panfrost_device *pfdev)
{
	struct panfrost_devfreq *pfdevfreq = &pfdev->pfdevfreq;

	if (!pfdevfreq->devfreq)
		return;

	devfreq_suspend_device(pfdevfreq->devfreq);
}

void panfrost_devfreq_record_busy(struct panfrost_devfreq *pfdevfreq)
{
	unsigned long irqflags;
	bool boost = false;

	if (!pfdevfreq->devfreq)
		return;

	spin_lock_irqsave(&pfdevfreq->lock, irqflags);

	panfrost_devfreq_update_utilization(pfdevfreq);

	/*
	 * Idle for longer than a polling period: the governor saw little
	 * load and will keep a low OPP for the burst that starts now.
	 */
	if (!pfdevfreq->busy_count && pfdevfreq->idle_start &&
	    ktime_ms_delta(pfdevfreq->time_last_update, pfdevfreq->idle_start) >
	    pfdevfreq->devfreq->profile->polling_ms)
		boost = true;

	pfdevfreq->busy_count++;

	spin_unlock_irqrestore(&pfdevfreq->lock, irqflags);

	if (boost)
		panfrost_devfreq_boost(pfdevfreq, PANFROST_BOOST_IDLE);
}

void panfrost_devfreq_record_idle(struct panfrost_devfreq *pfdevfreq)
{
	unsigned long irqflags;

	if (!pfdevfreq->devfreq)
		return;

	spin_lock_irqsave(&pfdevfreq->lock, irqflags);

	panfrost_devfreq_update_utilization(pfdevfreq);

	WARN_ON(--pfdevfreq->busy_count < 0);
	if (!pfdevfreq->busy_count)
		pfdevfreq->idle_start = pfdevfreq->time_last_update;

	spin_unlock_irqrestore(&pfdevfreq->lock, irqflags);
}
