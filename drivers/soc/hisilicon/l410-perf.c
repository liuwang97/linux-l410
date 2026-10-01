// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei Qingyun L410 (Kirin 990): performance modes and interaction boosts
 *
 * Three modes, switched by the power profile daemon:
 *
 *   powersave    no floors, no boosts
 *   balanced     no floors while idle; short frequency floors while the user
 *                types, clicks or scrolls (input boost), while an application
 *                starts (launch boost, kicked from user space) and a GPU floor
 *                while the compositor produces frames (frame floor: a 60 Hz
 *                composition is a few percent of GPU load, which utilisation
 *                based DVFS answers with the lowest OPP and missed frames)
 *   performance  every CPU cluster, the GPU and the memory at their fastest
 *                OPP, no cluster power down while idle
 *
 * Everything goes through PM QoS minimum requests (freq_qos for cpufreq,
 * dev_pm_qos for devfreq, cpu_latency_qos), so thermal limits, which are
 * maximum requests, always win.
 *
 * The input boost follows the vendor/CAF "cpu-boost" and kerneltoast's
 * cpu_input_boost: the input event handler only stamps a time; a SCHED_FIFO
 * worker applies and expires the floors. Pointer motion alone is a "light"
 * event (with a hardware cursor it needs no repaint), key presses, clicks,
 * scrolling and multi-finger touches are "heavy".
 */

#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/soc/hisilicon/l410-perf.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#define L410_NR_CLUSTERS	3
#define L410_FREQ_MAX		PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE

enum l410_mode {
	L410_POWERSAVE,
	L410_BALANCED,
	L410_PERFORMANCE,
	L410_NR_MODES,
};

static const char * const l410_mode_names[L410_NR_MODES] = {
	[L410_POWERSAVE] = "powersave",
	[L410_BALANCED] = "balanced",
	[L410_PERFORMANCE] = "performance",
};

/* devfreq devices with a minimum frequency request */
enum l410_dev {
	L410_GPU,
	L410_DDR,
	L410_NR_DEVS,
};

static const char * const l410_dev_compat[L410_NR_DEVS] = {
	[L410_GPU] = "hisilicon,kirin990-mali",
	[L410_DDR] = "hisilicon,kirin990-ddr",
};

/* a set of floors, in kHz (0: none, L410_FREQ_MAX: fastest) */
struct l410_floor {
	u32 cpu[L410_NR_CLUSTERS];
	u32 dev[L410_NR_DEVS];
};

struct l410_perf {
	struct mutex lock;
	enum l410_mode mode;

	/* first CPU of each cluster: little, middle, big */
	struct cpufreq_policy *policy[L410_NR_CLUSTERS];
	struct freq_qos_request cpu_req[L410_NR_CLUSTERS];
	struct device *dev[L410_NR_DEVS];
	struct dev_pm_qos_request dev_req[L410_NR_DEVS];
	struct pm_qos_request lat_req;
	struct l410_floor applied;

	/* memory follows CPU and GPU load (qcom: CPU OPPs carry DDR bandwidth) */
	struct devfreq *gpu_df;
	struct dev_pm_qos_request ddr_load_req;
	bool ddr_load_added;
	u32 ddr_load_khz;
	struct delayed_work ddr_work;

	struct kthread_worker *worker;
	struct kthread_work update_work;
	struct kthread_delayed_work expire_work;

	/* written from the input handler and frame hook (atomic) */
	spinlock_t ev_lock;
	unsigned long heavy_until;
	unsigned long light_until;
	unsigned long last_frame;
	unsigned long launch_until;

	/* statistics */
	unsigned long nr_heavy, nr_light, nr_launch, nr_frame;
	u64 boosted_ms;
	unsigned long boost_start;
	bool boosted;

	struct input_handler input_handler;
	struct kobject *kobj;
};

static struct l410_perf *l410;

/* tunables (sysfs) */
/* covers kinetic scrolling after the last wheel / touchpad event */
static unsigned int input_ms = 300;
static unsigned int frame_hold_ms = 100;
static unsigned int launch_max_ms = 3000;
static unsigned int perf_latency_us = 100;

/*
 * While the user types, clicks, scrolls or drags, everything runs at its
 * fastest OPP: scrolling a heavy web page (Chromium / Firefox on bilibili)
 * loads the GPU to ~50-65 % at 600 MHz and keeps four CPU cores busy, and
 * every step below the top OPPs cost frames. It only lasts while input comes.
 */
static struct l410_floor heavy_floor = {
	.cpu = { L410_FREQ_MAX, L410_FREQ_MAX, L410_FREQ_MAX },
	.dev = { [L410_GPU] = L410_FREQ_MAX, [L410_DDR] = L410_FREQ_MAX },
};

static struct l410_floor light_floor;

/*
 * While frames flow: KWin stays double buffered (predicted render time plus
 * 2.95 ms under 16.7 ms) from a 332 MHz floor on, measured on the 2160x1440
 * panel; at 166 MHz it drops to triple buffering and misses frames.
 */
static struct l410_floor frame_floor = {
	.dev = { [L410_GPU] = 332000, [L410_DDR] = 900000 },
};

static struct l410_floor launch_floor = {
	.cpu = { L410_FREQ_MAX, L410_FREQ_MAX, L410_FREQ_MAX },
	.dev = { [L410_GPU] = 461000, [L410_DDR] = 1660000 },
};

static struct l410_floor perf_floor = {
	.cpu = { L410_FREQ_MAX, L410_FREQ_MAX, L410_FREQ_MAX },
	.dev = { [L410_GPU] = L410_FREQ_MAX, [L410_DDR] = L410_FREQ_MAX },
};

static char *mode = "balanced";
module_param(mode, charp, 0444);
MODULE_PARM_DESC(mode, "Mode at boot: powersave, balanced or performance");

/*
 * Memory frequency from the load, sampled every ddr_poll_ms (deferrable: an
 * idle system is not woken up): the busiest of the middle and big clusters,
 * relative to its fastest OPP, and the GPU frequency.
 */
static unsigned int ddr_poll_ms = 50;
static const struct { u32 cpu_pct, ddr_khz; } l410_ddr_cpu[] = {
	{ 85, 1660000 }, { 60, 1370000 }, { 40, 1106000 }, { 20, 900000 },
};
static const struct { u32 gpu_khz, ddr_khz; } l410_ddr_gpu[] = {
	{ 461000, 1370000 }, { 304000, 1106000 }, { 208000, 900000 },
};

/* ------------------------------------------------------------------------ */
/* applying floors */

static void l410_floor_max(struct l410_floor *f, const struct l410_floor *g)
{
	int i;

	for (i = 0; i < L410_NR_CLUSTERS; i++)
		f->cpu[i] = max(f->cpu[i], g->cpu[i]);
	for (i = 0; i < L410_NR_DEVS; i++)
		f->dev[i] = max(f->dev[i], g->dev[i]);
}

/* late binding: cpufreq and devfreq drivers may come after us */
static void l410_bind(struct l410_perf *p)
{
	struct platform_device *pdev;
	struct device_node *np;
	int i;

	for (i = 0; i < L410_NR_CLUSTERS; i++) {
		static const unsigned int first_cpu[L410_NR_CLUSTERS] = { 0, 4, 6 };
		struct cpufreq_policy *policy;

		if (p->policy[i])
			continue;
		policy = cpufreq_cpu_get(first_cpu[i]);
		if (!policy)
			continue;
		if (freq_qos_add_request(&policy->constraints, &p->cpu_req[i],
					 FREQ_QOS_MIN, FREQ_QOS_MIN_DEFAULT_VALUE) < 0) {
			cpufreq_cpu_put(policy);
			continue;
		}
		p->policy[i] = policy;
	}

	for (i = 0; i < L410_NR_DEVS; i++) {
		if (p->dev[i])
			continue;
		np = of_find_compatible_node(NULL, NULL, l410_dev_compat[i]);
		if (!np)
			continue;
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (!pdev)
			continue;
		if (dev_pm_qos_add_request(&pdev->dev, &p->dev_req[i],
					   DEV_PM_QOS_MIN_FREQUENCY,
					   PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE) < 0) {
			put_device(&pdev->dev);
			continue;
		}
		p->dev[i] = &pdev->dev;
		if (i == L410_DDR &&
		    dev_pm_qos_add_request(&pdev->dev, &p->ddr_load_req,
					   DEV_PM_QOS_MIN_FREQUENCY,
					   PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE) >= 0) {
			p->ddr_load_added = true;
			queue_delayed_work(system_power_efficient_wq, &p->ddr_work,
					   msecs_to_jiffies(ddr_poll_ms));
		}
	}

	if (!p->gpu_df && p->dev[L410_GPU]) {
		struct devfreq *df = devfreq_get_devfreq_by_node(p->dev[L410_GPU]->of_node);

		if (!IS_ERR(df))
			p->gpu_df = df;
	}
}

static void l410_ddr_work(struct work_struct *work)
{
	struct l410_perf *p = container_of(to_delayed_work(work), struct l410_perf, ddr_work);
	u32 pct = 0, khz = 0, gpu_khz = 0;
	int i;

	if (p->mode != L410_PERFORMANCE) {
		/*
		 * Position within the cluster's range: the big cluster's lowest
		 * OPP is already 54 % of its fastest, which is idle, not load.
		 */
		for (i = 1; i < L410_NR_CLUSTERS; i++) {
			struct cpufreq_policy *policy = p->policy[i];
			unsigned int lo, hi, cur;

			if (!policy)
				continue;
			lo = policy->cpuinfo.min_freq;
			hi = policy->cpuinfo.max_freq;
			cur = READ_ONCE(policy->cur);
			if (hi > lo && cur > lo)
				pct = max(pct, (cur - lo) * 100 / (hi - lo));
		}
		for (i = 0; i < ARRAY_SIZE(l410_ddr_cpu); i++)
			if (pct >= l410_ddr_cpu[i].cpu_pct) {
				khz = l410_ddr_cpu[i].ddr_khz;
				break;
			}

		if (p->gpu_df)
			gpu_khz = READ_ONCE(p->gpu_df->previous_freq) / 1000;
		for (i = 0; i < ARRAY_SIZE(l410_ddr_gpu); i++)
			if (gpu_khz >= l410_ddr_gpu[i].gpu_khz) {
				khz = max(khz, l410_ddr_gpu[i].ddr_khz);
				break;
			}
	}

	if (khz != p->ddr_load_khz) {
		dev_pm_qos_update_request(&p->ddr_load_req, khz);
		p->ddr_load_khz = khz;
	}
	queue_delayed_work(system_power_efficient_wq, &p->ddr_work,
			   msecs_to_jiffies(ddr_poll_ms));
}

static void l410_apply(struct l410_perf *p, const struct l410_floor *f)
{
	int i;

	l410_bind(p);

	for (i = 0; i < L410_NR_CLUSTERS; i++) {
		if (!p->policy[i] || p->applied.cpu[i] == f->cpu[i])
			continue;
		/* freq_qos values are s32 kHz */
		freq_qos_update_request(&p->cpu_req[i],
					min_t(u32, f->cpu[i], FREQ_QOS_MAX_DEFAULT_VALUE));
		p->applied.cpu[i] = f->cpu[i];
	}
	for (i = 0; i < L410_NR_DEVS; i++) {
		if (!p->dev[i] || p->applied.dev[i] == f->dev[i])
			continue;
		dev_pm_qos_update_request(&p->dev_req[i],
					  min_t(u32, f->dev[i], L410_FREQ_MAX));
		p->applied.dev[i] = f->dev[i];
	}
}

/*
 * Recompute the floors from the mode and the active boosts, and schedule the
 * next expiry. Called with p->lock held.
 */
static void l410_update(struct l410_perf *p)
{
	struct l410_floor f = { };
	unsigned long now = jiffies, next = 0, flags;
	unsigned long heavy_until, light_until, frame_until, launch_until;
	bool boosted = false;

	spin_lock_irqsave(&p->ev_lock, flags);
	heavy_until = p->heavy_until;
	light_until = p->light_until;
	frame_until = READ_ONCE(p->last_frame) + msecs_to_jiffies(frame_hold_ms);
	launch_until = p->launch_until;
	spin_unlock_irqrestore(&p->ev_lock, flags);

#define L410_NEXT(t) do { if (!next || time_before(t, next)) next = (t); } while (0)

	switch (p->mode) {
	case L410_PERFORMANCE:
		f = perf_floor;
		break;
	case L410_BALANCED:
		if (time_before(now, launch_until)) {
			l410_floor_max(&f, &launch_floor);
			L410_NEXT(launch_until);
			boosted = true;
		}
		if (time_before(now, heavy_until)) {
			l410_floor_max(&f, &heavy_floor);
			L410_NEXT(heavy_until);
			boosted = true;
		}
		if (time_before(now, frame_until)) {
			l410_floor_max(&f, &frame_floor);
			L410_NEXT(frame_until);
			boosted = true;
		}
		if (time_before(now, light_until)) {
			l410_floor_max(&f, &light_floor);
			L410_NEXT(light_until);
			boosted = true;
		}
		break;
	default:
		break;
	}
#undef L410_NEXT

	l410_apply(p, &f);

	if (boosted != p->boosted) {
		if (boosted)
			p->boost_start = now;
		else
			p->boosted_ms += jiffies_to_msecs(now - p->boost_start);
		p->boosted = boosted;
	}

	if (next)
		kthread_mod_delayed_work(p->worker, &p->expire_work,
					 max_t(long, next - now, 1));
}

static void l410_update_work(struct kthread_work *work)
{
	struct l410_perf *p = container_of(work, struct l410_perf, update_work);

	mutex_lock(&p->lock);
	l410_update(p);
	mutex_unlock(&p->lock);
}

static void l410_expire_work(struct kthread_work *work)
{
	struct l410_perf *p = container_of(work, struct l410_perf, expire_work.work);

	mutex_lock(&p->lock);
	l410_update(p);
	mutex_unlock(&p->lock);
}

static void l410_set_mode(struct l410_perf *p, enum l410_mode m)
{
	mutex_lock(&p->lock);
	p->mode = m;
	if (m == L410_PERFORMANCE)
		cpu_latency_qos_update_request(&p->lat_req, perf_latency_us);
	else
		cpu_latency_qos_update_request(&p->lat_req, PM_QOS_DEFAULT_VALUE);
	l410_update(p);
	mutex_unlock(&p->lock);
}

/* ------------------------------------------------------------------------ */
/* event sources */

void l410_perf_frame(void)
{
	struct l410_perf *p = READ_ONCE(l410);
	unsigned long now = jiffies, prev;

	if (!p)
		return;
	prev = xchg(&p->last_frame, now);
	/* frames start again after a pause: apply now; while they flow the expiry re-arms */
	if (p->mode == L410_BALANCED &&
	    !time_before(now, prev + msecs_to_jiffies(frame_hold_ms))) {
		p->nr_frame++;
		kthread_queue_work(p->worker, &p->update_work);
	}
}
EXPORT_SYMBOL_GPL(l410_perf_frame);

/*
 * Motion while a button or several fingers are down is a drag or a
 * touchpad scroll / gesture: the compositor moves windows or the client
 * scrolls on every event, as heavy as a key press.
 */
static bool l410_dragging(struct input_dev *dev)
{
	static const unsigned int held[] = {
		BTN_LEFT, BTN_RIGHT, BTN_MIDDLE, BTN_TOOL_DOUBLETAP,
		BTN_TOOL_TRIPLETAP, BTN_TOOL_QUADTAP, BTN_TOOL_QUINTTAP,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(held); i++)
		if (test_bit(held[i], dev->key))
			return true;
	return false;
}

static bool l410_event_is_heavy(struct input_dev *dev, unsigned int type,
				unsigned int code, int value)
{
	switch (type) {
	case EV_KEY:
		/* touching the pad to move the pointer is not a click */
		if (code == BTN_TOUCH || code == BTN_TOOL_FINGER)
			return false;
		return value != 0;	/* press and autorepeat */
	case EV_REL:
		if (code == REL_WHEEL || code == REL_HWHEEL ||
		    code == REL_WHEEL_HI_RES || code == REL_HWHEEL_HI_RES)
			return true;
		return l410_dragging(dev);
	case EV_ABS:
		return l410_dragging(dev);
	default:
		return false;
	}
}

static void l410_input_event(struct input_handle *handle, unsigned int type,
			     unsigned int code, int value)
{
	struct l410_perf *p = handle->handler->private;
	unsigned long now = jiffies, until, *slot;
	bool heavy, kick;

	if (type != EV_KEY && type != EV_REL && type != EV_ABS)
		return;
	if (p->mode != L410_BALANCED)
		return;

	heavy = l410_event_is_heavy(handle->dev, type, code, value);
	until = now + msecs_to_jiffies(input_ms);

	spin_lock(&p->ev_lock);
	slot = heavy ? &p->heavy_until : &p->light_until;
	/* not running yet: apply now; running: the expiry picks it up */
	kick = !time_before(now, *slot);
	*slot = until;
	spin_unlock(&p->ev_lock);

	if (kick) {
		if (heavy)
			p->nr_heavy++;
		else
			p->nr_light++;
		kthread_queue_work(p->worker, &p->update_work);
	}
}

static int l410_input_connect(struct input_handler *handler, struct input_dev *dev,
			      const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "l410-perf";

	ret = input_register_handle(handle);
	if (ret)
		goto free;
	ret = input_open_device(handle);
	if (ret)
		goto unregister;
	return 0;

unregister:
	input_unregister_handle(handle);
free:
	kfree(handle);
	return ret;
}

static void l410_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id l410_input_ids[] = {
	{ .flags = INPUT_DEVICE_ID_MATCH_EVBIT, .evbit = { BIT_MASK(EV_KEY) } },
	{ .flags = INPUT_DEVICE_ID_MATCH_EVBIT, .evbit = { BIT_MASK(EV_REL) } },
	{ .flags = INPUT_DEVICE_ID_MATCH_EVBIT, .evbit = { BIT_MASK(EV_ABS) } },
	{ }
};

/* ------------------------------------------------------------------------ */
/* sysfs: /sys/kernel/l410_perf */

static ssize_t mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i, n = 0;

	for (i = 0; i < L410_NR_MODES; i++)
		n += sysfs_emit_at(buf, n, i == l410->mode ? "[%s] " : "%s ",
				   l410_mode_names[i]);
	buf[n - 1] = '\n';
	return n;
}

static ssize_t mode_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	int m = sysfs_match_string(l410_mode_names, buf);

	if (m < 0)
		return m;
	l410_set_mode(l410, m);
	return count;
}

/* write n ms (capped at launch_max_ms) to boost for an application start, 0 to end */
static ssize_t launch_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	struct l410_perf *p = l410;
	unsigned long flags;
	unsigned int ms;
	int ret;

	ret = kstrtouint(buf, 0, &ms);
	if (ret)
		return ret;
	ms = min(ms, launch_max_ms);

	spin_lock_irqsave(&p->ev_lock, flags);
	p->launch_until = jiffies + msecs_to_jiffies(ms);
	spin_unlock_irqrestore(&p->ev_lock, flags);
	if (ms)
		p->nr_launch++;

	mutex_lock(&p->lock);
	l410_update(p);
	mutex_unlock(&p->lock);
	return count;
}

static ssize_t stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct l410_perf *p = l410;
	int n, i;

	mutex_lock(&p->lock);
	n = sysfs_emit(buf, "mode %s boosted %d\n", l410_mode_names[p->mode], p->boosted);
	n += sysfs_emit_at(buf, n, "floors cpu %u %u %u kHz gpu %u ddr %u kHz, ddr load %u kHz\n",
			   p->applied.cpu[0], p->applied.cpu[1], p->applied.cpu[2],
			   p->applied.dev[L410_GPU], p->applied.dev[L410_DDR], p->ddr_load_khz);
	n += sysfs_emit_at(buf, n, "boosts heavy %lu light %lu launch %lu frame %lu, boosted %llu ms\n",
			   p->nr_heavy, p->nr_light, p->nr_launch, p->nr_frame,
			   p->boosted_ms + (p->boosted ? jiffies_to_msecs(jiffies - p->boost_start) : 0));
	n += sysfs_emit_at(buf, n, "bound cpufreq");
	for (i = 0; i < L410_NR_CLUSTERS; i++)
		n += sysfs_emit_at(buf, n, " %d", !!p->policy[i]);
	n += sysfs_emit_at(buf, n, " gpu %d ddr %d\n", !!p->dev[L410_GPU], !!p->dev[L410_DDR]);
	mutex_unlock(&p->lock);
	return n;
}

static ssize_t l410_uint_show(unsigned int *v, char *buf)
{
	return sysfs_emit(buf, "%u\n", *v);
}

static ssize_t l410_uint_store(unsigned int *v, const char *buf, size_t count)
{
	int ret = kstrtouint(buf, 0, v);

	return ret ? ret : count;
}

/* "little middle big gpu ddr" in kHz */
static ssize_t l410_floor_show(struct l410_floor *f, char *buf)
{
	return sysfs_emit(buf, "%u %u %u %u %u\n", f->cpu[0], f->cpu[1], f->cpu[2],
			  f->dev[L410_GPU], f->dev[L410_DDR]);
}

static ssize_t l410_floor_store(struct l410_floor *f, const char *buf, size_t count)
{
	u32 v[5];

	if (sscanf(buf, "%u %u %u %u %u", &v[0], &v[1], &v[2], &v[3], &v[4]) != 5)
		return -EINVAL;

	mutex_lock(&l410->lock);
	f->cpu[0] = v[0];
	f->cpu[1] = v[1];
	f->cpu[2] = v[2];
	f->dev[L410_GPU] = v[3];
	f->dev[L410_DDR] = v[4];
	l410_update(l410);
	mutex_unlock(&l410->lock);
	return count;
}

#define L410_UINT_ATTR(name)							\
static ssize_t name##_show(struct kobject *k, struct kobj_attribute *a, char *b) \
{ return l410_uint_show(&name, b); }						\
static ssize_t name##_store(struct kobject *k, struct kobj_attribute *a,	\
			    const char *b, size_t c)				\
{ return l410_uint_store(&name, b, c); }					\
static struct kobj_attribute name##_attr = __ATTR_RW(name)

#define L410_FLOOR_ATTR(name)							\
static ssize_t name##_show(struct kobject *k, struct kobj_attribute *a, char *b) \
{ return l410_floor_show(&name, b); }						\
static ssize_t name##_store(struct kobject *k, struct kobj_attribute *a,	\
			    const char *b, size_t c)				\
{ return l410_floor_store(&name, b, c); }					\
static struct kobj_attribute name##_attr = __ATTR_RW(name)

L410_UINT_ATTR(input_ms);
L410_UINT_ATTR(frame_hold_ms);
L410_UINT_ATTR(launch_max_ms);
L410_UINT_ATTR(perf_latency_us);
L410_FLOOR_ATTR(heavy_floor);
L410_FLOOR_ATTR(light_floor);
L410_FLOOR_ATTR(frame_floor);
L410_FLOOR_ATTR(launch_floor);
L410_FLOOR_ATTR(perf_floor);

static struct kobj_attribute mode_attr = __ATTR_RW(mode);
static struct kobj_attribute launch_attr = __ATTR_WO(launch);
static struct kobj_attribute stats_attr = __ATTR_RO(stats);

static struct attribute *l410_attrs[] = {
	&mode_attr.attr,
	&launch_attr.attr,
	&stats_attr.attr,
	&input_ms_attr.attr,
	&frame_hold_ms_attr.attr,
	&launch_max_ms_attr.attr,
	&perf_latency_us_attr.attr,
	&heavy_floor_attr.attr,
	&light_floor_attr.attr,
	&frame_floor_attr.attr,
	&launch_floor_attr.attr,
	&perf_floor_attr.attr,
	NULL
};

static const struct attribute_group l410_attr_group = {
	.attrs = l410_attrs,
};

/* ------------------------------------------------------------------------ */

static int __init l410_perf_init(void)
{
	struct l410_perf *p;
	int ret, m;

	if (!of_machine_is_compatible("hisilicon,kirin990") &&
	    !of_find_compatible_node(NULL, NULL, "hisilicon,kirin990-mali"))
		return -ENODEV;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	mutex_init(&p->lock);
	spin_lock_init(&p->ev_lock);
	kthread_init_work(&p->update_work, l410_update_work);
	kthread_init_delayed_work(&p->expire_work, l410_expire_work);
	INIT_DEFERRABLE_WORK(&p->ddr_work, l410_ddr_work);
	cpu_latency_qos_add_request(&p->lat_req, PM_QOS_DEFAULT_VALUE);

	p->worker = kthread_run_worker(0, "l410-perf");
	if (IS_ERR(p->worker)) {
		ret = PTR_ERR(p->worker);
		goto free;
	}
	/* boosts must not queue behind the load they are for */
	sched_set_fifo_low(p->worker->task);

	m = sysfs_match_string(l410_mode_names, mode);
	p->mode = m < 0 ? L410_BALANCED : m;
	l410 = p;

	p->kobj = kobject_create_and_add("l410_perf", kernel_kobj);
	if (!p->kobj) {
		ret = -ENOMEM;
		goto worker;
	}
	ret = sysfs_create_group(p->kobj, &l410_attr_group);
	if (ret)
		goto kobj;

	p->input_handler.event = l410_input_event;
	p->input_handler.connect = l410_input_connect;
	p->input_handler.disconnect = l410_input_disconnect;
	p->input_handler.name = "l410-perf";
	p->input_handler.id_table = l410_input_ids;
	p->input_handler.private = p;
	ret = input_register_handler(&p->input_handler);
	if (ret)
		goto group;

	l410_set_mode(p, p->mode);
	pr_info("l410-perf: mode %s\n", l410_mode_names[p->mode]);
	return 0;

group:
	sysfs_remove_group(p->kobj, &l410_attr_group);
kobj:
	kobject_put(p->kobj);
worker:
	l410 = NULL;
	kthread_destroy_worker(p->worker);
free:
	cpu_latency_qos_remove_request(&p->lat_req);
	kfree(p);
	return ret;
}
/* after cpufreq; devfreq devices that probe later are bound on first use */
late_initcall_sync(l410_perf_init);

MODULE_DESCRIPTION("Huawei L410 performance modes and interaction boosts");
MODULE_LICENSE("GPL");
