/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Glue between the HiSilicon vendor audio code (Kylin 4.19) and 6.18:
 * logging macros and a few helpers that went away upstream.
 */
#ifndef __HI6405_COMPAT_H__
#define __HI6405_COMPAT_H__

#include <linux/printk.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <sound/soc.h>

#define AUDIO_LOGE(fmt, ...) \
	pr_err("%s: %s: " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__)
#define AUDIO_LOGW(fmt, ...) \
	pr_warn("%s: %s: " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__)
#define AUDIO_LOGI(fmt, ...) \
	pr_debug("%s: %s: " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__)
#define AUDIO_LOGD(fmt, ...) \
	pr_debug("%s: %s: " fmt "\n", LOG_TAG, __func__, ##__VA_ARGS__)

#define IN_FUNCTION  AUDIO_LOGD("begin")
#define OUT_FUNCTION AUDIO_LOGD("end")

/* vendor clk debug helper; only used in log messages */
#define clk_get_enable_count(c) 0

/* charger switched-cap frequency hint (vendor power driver), not on the L410 */
static inline void vsys_switch_set_sc_frequency_mode(int mode) { }

/* rear (desktop) jack switch GPIOs, not present on the L410 */
static inline void hi64xx_set_headphone_switch_gpio_value(int value) { }
static inline void hi64xx_set_head_mic_switch_gpio_value(int value) { }

/*
 * Register IO of the vendor code.
 *
 * ASoC serialises snd_soc_component_read/write/update_bits() on
 * component->io_mutex and calls the codec's .read/.write callbacks with it
 * held. The vendor code re-enters register IO from inside those callbacks:
 * a DIG page access makes the resource manager switch the codec PLL on,
 * which writes codec registers again. Through the ASoC helpers that is a
 * self-deadlock on io_mutex (the 4.19 helpers the code was written for did
 * not lock). So the vendor code calls the callbacks directly; the SSI
 * controller serialises the bus accesses underneath. Code outside the
 * vendor port (hi6405_card.c, the TAS2562 driver) keeps the ASoC helpers.
 */
static inline unsigned int hi6405_raw_read(struct snd_soc_component *c,
	unsigned int reg)
{
	return c->driver->read(c, reg);
}

static inline int hi6405_raw_write(struct snd_soc_component *c,
	unsigned int reg, unsigned int val)
{
	return c->driver->write(c, reg, val);
}

static inline int hi6405_raw_update_bits(struct snd_soc_component *c,
	unsigned int reg, unsigned int mask, unsigned int val)
{
	unsigned int old = hi6405_raw_read(c, reg);
	unsigned int new = (old & ~mask) | (val & mask);

	if (new == old)
		return 0;
	hi6405_raw_write(c, reg, new);
	return 1;
}

#define snd_soc_component_read32(c, r)	hi6405_raw_read((c), (r))
#define snd_soc_component_read(c, r)	hi6405_raw_read((c), (r))
#define snd_soc_component_write(c, r, v) hi6405_raw_write((c), (r), (v))
#define snd_soc_component_update_bits(c, r, m, v) \
	hi6405_raw_update_bits((c), (r), (m), (v))

/* monotonic time in ns, used by the vendor code for period sanity checks */
static inline u64 hisi_getcurtime(void)
{
	return ktime_get_ns();
}

#endif /* __HI6405_COMPAT_H__ */
