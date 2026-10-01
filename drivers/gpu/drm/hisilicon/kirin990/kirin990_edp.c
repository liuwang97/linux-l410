// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kirin 990 DSS: the L410's TI SN65DSI86 DSI-to-eDP bridge, the eDP panel's
 * power and the backlight.
 *
 * Sequenced as the vendor kernel does it for this board: the firmware DT's
 * bridge node has product_type 3 ("laptop product UA", laptop_bridge.c with
 * sn65dsix6.c), whose supplies are SoC GPIOs:
 *
 *   GPIO_105  bridge 1.2 V         GPIO_206  bridge EN
 *   GPIO_047  bridge 1.8 V         GPIO_028  panel VDD
 *
 * and the bridge's 38.4 MHz reference is the PMIC clock clk_nfc. (The EC's
 * switched outputs EC_1V2_EN / EC_DSI_VCCIO_ON power the "product U" board;
 * nothing in the L410's DT uses them.) The backlight is a pwm-backlight
 * device (BLPWM + BL_EN on GPIO_027).
 */

#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/of.h>

#include <drm/display/drm_dp.h>
#include <drm/drm_print.h>

#include "kirin990_drv.h"

/* SN65DSI86 registers (datasheet SLLSEH2, mainline ti-sn65dsi86.c) */
#define SN_DEVICE_REV		0x08
#define SN_DPPLL_SRC		0x0a
#define   DPPLL_SRC_REFCLK_38M4	0x08
#define   DPPLL_LOCK		BIT(7)
#define SN_PLL_ENABLE		0x0d
#define SN_DSI_LANES		0x10
#define   CHA_DSI_LANES_MASK	GENMASK(4, 3)
#define SN_DSIA_EQ		0x11
#define SN_DSIA_CLK_FREQ	0x12
#define SN_CHA_ACTIVE_LINE_LEN	0x20
#define SN_CHA_VDISPLAY		0x24
#define SN_CHA_HSYNC_WIDTH	0x2c
#define SN_CHA_VSYNC_WIDTH	0x30
#define SN_CHA_HBP		0x34
#define SN_CHA_VBP		0x36
#define SN_CHA_HFP		0x38
#define SN_CHA_VFP		0x3a
#define SN_TEST_PATTERN		0x3c
#define SN_ENH_FRAME		0x5a
#define   VSTREAM_ENABLE	BIT(3)
#define SN_DATA_FORMAT		0x5b
#define SN_HPD_DISABLE		0x5c
#define   HPD_DISABLE		BIT(0)
#define SN_AUX_WDATA(x)		(0x64 + (x))
#define SN_AUX_ADDR_19_16	0x74
#define SN_AUX_ADDR_15_8	0x75
#define SN_AUX_ADDR_7_0		0x76
#define SN_AUX_LENGTH		0x77
#define SN_AUX_CMD		0x78
#define   AUX_CMD_SEND		BIT(0)
#define SN_AUX_RDATA(x)		(0x79 + (x))
#define SN_SSC_CONFIG		0x93
#define   DP_NUM_LANES_MASK	GENMASK(5, 4)
#define SN_DATARATE_CONFIG	0x94
#define   DP_DATARATE_MASK	GENMASK(7, 5)
#define SN_ML_TX_MODE		0x96
#define   ML_TX_MAIN_LINK_OFF	0x00
#define   ML_TX_NORMAL_MODE	0x01
#define   ML_TX_SEMI_AUTO_LT	0x0a
#define SN_AUX_CMD_STATUS	0xf4
#define   AUX_RPLY_TOUT		BIT(3)
#define   AUX_SHORT		BIT(5)
#define   AUX_NAT_I2C_FAIL	BIT(6)
#define SN_LINK_STATUS		0xf8

#define SN_LINK_TRIES		3
/* vendor bridge post_disable: keep the panel off this long before powering it again */
#define PANEL_OFF_MIN_MS	500

static int sn_read(struct kirin_dss *k, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(k->bridge, reg);

	if (ret < 0)
		drm_err(&k->drm, "SN65DSI86 read %#x: %d\n", reg, ret);
	return ret;
}

static void sn_write(struct kirin_dss *k, u8 reg, u8 val)
{
	int ret, tries = 3;

	do {
		ret = i2c_smbus_write_byte_data(k->bridge, reg, val);
	} while (ret < 0 && --tries);
	if (ret < 0)
		drm_err(&k->drm, "SN65DSI86 write %#x = %#x: %d\n", reg, val, ret);
}

static void sn_update(struct kirin_dss *k, u8 reg, u8 mask, u8 val)
{
	int v = sn_read(k, reg);

	if (v >= 0)
		sn_write(k, reg, (v & ~mask) | (val & mask));
}

static int sn_poll(struct kirin_dss *k, u8 reg, u8 mask, u8 want, unsigned int ms)
{
	ktime_t end = ktime_add_ms(ktime_get(), ms);
	int v;

	for (;;) {
		v = sn_read(k, reg);
		if (v >= 0 && (v & mask) == want)
			return v;
		if (ktime_after(ktime_get(), end))
			return v < 0 ? v : -ETIMEDOUT;
		usleep_range(1000, 1500);
	}
}

/* semi-automatic link training ends in normal mode, or main link off on failure */
static int sn_wait_training(struct kirin_dss *k)
{
	ktime_t end = ktime_add_ms(ktime_get(), 500);
	int v;

	do {
		usleep_range(1000, 1500);
		v = sn_read(k, SN_ML_TX_MODE);
		if (v == ML_TX_MAIN_LINK_OFF || v == ML_TX_NORMAL_MODE)
			return v;
	} while (ktime_before(ktime_get(), end));
	return -ETIMEDOUT;
}

/* native AUX transfer of up to 16 bytes to/from the panel's DPCD */
static int sn_aux(struct kirin_dss *k, u8 request, u32 addr, u8 *buf, unsigned int len)
{
	unsigned int i;
	int st;

	sn_write(k, SN_AUX_CMD_STATUS, AUX_RPLY_TOUT | AUX_SHORT | AUX_NAT_I2C_FAIL);
	if (request == DP_AUX_NATIVE_WRITE)
		for (i = 0; i < len; i++)
			sn_write(k, SN_AUX_WDATA(i), buf[i]);
	sn_write(k, SN_AUX_ADDR_19_16, (addr >> 16) & 0xf);
	sn_write(k, SN_AUX_ADDR_15_8, (addr >> 8) & 0xff);
	sn_write(k, SN_AUX_ADDR_7_0, addr & 0xff);
	sn_write(k, SN_AUX_LENGTH, len);
	sn_write(k, SN_AUX_CMD, (request << 4) | AUX_CMD_SEND);
	if (sn_poll(k, SN_AUX_CMD, AUX_CMD_SEND, 0, 50) < 0)
		return -ETIMEDOUT;
	st = sn_read(k, SN_AUX_CMD_STATUS);
	if (st < 0)
		return st;
	if (st & (AUX_RPLY_TOUT | AUX_SHORT | AUX_NAT_I2C_FAIL))
		return -EIO;
	if (request == DP_AUX_NATIVE_READ) {
		for (i = 0; i < len; i++) {
			st = sn_read(k, SN_AUX_RDATA(i));
			if (st < 0)
				return st;
			buf[i] = st;
		}
	}
	return 0;
}

static void sn_aux_write(struct kirin_dss *k, u32 addr, u8 val)
{
	int ret = sn_aux(k, DP_AUX_NATIVE_WRITE, addr, &val, 1);

	if (ret)
		drm_warn(&k->drm, "DPCD write %#x: %d\n", addr, ret);
}

static void kirin_backlight_work(struct work_struct *work)
{
	struct kirin_dss *k = container_of(work, struct kirin_dss, backlight_work.work);

	backlight_enable(k->backlight);
}

void kirin_backlight_on_later(struct kirin_dss *k, unsigned int ms)
{
	if (k->backlight)
		schedule_delayed_work(&k->backlight_work, msecs_to_jiffies(ms));
}

void kirin_backlight_off(struct kirin_dss *k)
{
	if (!k->backlight)
		return;
	cancel_delayed_work_sync(&k->backlight_work);
	backlight_disable(k->backlight);
}

void kirin_edp_dump(struct kirin_dss *k)
{
	static const u8 regs[] = {
		SN_DEVICE_REV, SN_DPPLL_SRC, SN_PLL_ENABLE, SN_DSI_LANES, SN_DSIA_EQ,
		SN_DSIA_CLK_FREQ, SN_ENH_FRAME, SN_DATA_FORMAT, SN_HPD_DISABLE,
		SN_SSC_CONFIG, SN_DATARATE_CONFIG, 0x95, SN_ML_TX_MODE, SN_LINK_STATUS,
		SN_CHA_ACTIVE_LINE_LEN, SN_CHA_ACTIVE_LINE_LEN + 1, SN_CHA_VDISPLAY,
		SN_CHA_VDISPLAY + 1, SN_CHA_HSYNC_WIDTH, SN_CHA_HSYNC_WIDTH + 1,
		SN_CHA_VSYNC_WIDTH, SN_CHA_VSYNC_WIDTH + 1, SN_CHA_HBP, SN_CHA_VBP,
		SN_CHA_HFP, SN_CHA_VFP,
	};
	char line[160];
	unsigned int i;
	int n = 0;

	u8 dpcd[8];
	int ret;

	if (!k->bridge)
		return;
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		n += scnprintf(line + n, sizeof(line) - n, " %02x=%02x", regs[i],
			       i2c_smbus_read_byte_data(k->bridge, regs[i]) & 0xff);
	drm_info(&k->drm, "SN65DSI86:%s\n", line);

	/* the panel's view: lanes 0x77 (CR/EQ/symbol lock), aligned, in sync */
	ret = sn_aux(k, DP_AUX_NATIVE_READ, DP_SINK_COUNT, dpcd, sizeof(dpcd));
	if (ret)
		drm_info(&k->drm, "panel DPCD: read failed (%d)\n", ret);
	else
		drm_info(&k->drm, "panel DPCD 0x200: %*ph (lanes %02x %02x align %02x sink %02x)\n",
			 (int)sizeof(dpcd), dpcd, dpcd[2], dpcd[3], dpcd[4], dpcd[5]);
}

/*
 * laptop_ua_bridge_runtime_resume() + laptop_ua_bridge_pre_enable() +
 * sn65dsi86_pre_enable(): supplies, reference clock, bridge enable, panel
 * VDD, then the bridge's reference clock setting and ASSR in the panel.
 */
void kirin_edp_power_on(struct kirin_dss *k)
{
	s64 off;

	if (!k->bridge)
		return;

	if (k->panel_off_time) {
		off = ktime_ms_delta(ktime_get(), k->panel_off_time);
		if (off >= 0 && off < PANEL_OFF_MIN_MS)
			msleep(PANEL_OFF_MIN_MS - off);
	}

	gpiod_set_value_cansleep(k->gpio_1v2, 1);
	msleep(20);
	gpiod_set_value_cansleep(k->gpio_1v8, 1);
	msleep(20);
	if (clk_prepare_enable(k->bridge_refclk))
		drm_err(&k->drm, "bridge reference clock won't start\n");
	msleep(5);
	gpiod_set_value_cansleep(k->gpio_bridge_en, 1);
	msleep(15);
	gpiod_set_value_cansleep(k->gpio_panel_vdd, 1);
	msleep(5);

	usleep_range(1000, 2000);
	sn_write(k, SN_DPPLL_SRC, DPPLL_SRC_REFCLK_38M4);
	sn_update(k, SN_HPD_DISABLE, HPD_DISABLE, HPD_DISABLE);
	msleep(100);	/* panel power-up (vendor: "wait for LCD init") */

	/* the bridge only speaks ASSR: enable it in the panel before training */
	sn_aux_write(k, DP_EDP_CONFIGURATION_SET, DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
}

/*
 * sn65dsi86_enable() for the laptop: DSI A 4 lanes, eDP 4 lanes at 1.62
 * Gbps, DP PLL, semi-automatic link training, then the video timing and the
 * stream. The DSI clock must already be running.
 */
int kirin_edp_enable(struct kirin_dss *k)
{
	const struct drm_display_mode *m = &k->fw_mode;
	int v, i;

	if (!k->bridge)
		return 0;

	msleep(20);
	sn_update(k, SN_DSI_LANES, CHA_DSI_LANES_MASK, 0);	/* 4 lanes */
	sn_write(k, SN_DSIA_EQ, 0xff);		/* vendor: better signal quality */
	sn_update(k, SN_SSC_CONFIG, DP_NUM_LANES_MASK, 3 << 4);	/* 4 lanes */
	sn_update(k, SN_DATARATE_CONFIG, DP_DATARATE_MASK, 1 << 5);	/* RBR */
	sn_write(k, SN_DSIA_CLK_FREQ, k->dsi_bit_clk_mhz / 5);

	sn_write(k, SN_PLL_ENABLE, 1);
	v = sn_poll(k, SN_DPPLL_SRC, DPPLL_LOCK, DPPLL_LOCK, 100);
	if (v < 0) {
		drm_err(&k->drm, "SN65DSI86 DP PLL not locked (%d)\n", v);
		goto fail;
	}
	sn_write(k, 0x95, 0);

	sn_write(k, SN_ENH_FRAME, 0x05);	/* enhanced framing, ASSR */
	sn_write(k, SN_ML_TX_MODE, 0x02);
	sn_write(k, SN_SSC_CONFIG, 0x34);
	for (i = 0; i < SN_LINK_TRIES; i++) {
		sn_write(k, SN_ML_TX_MODE, ML_TX_SEMI_AUTO_LT);
		msleep(10);
		sn_write(k, SN_ML_TX_MODE, ML_TX_SEMI_AUTO_LT);
		msleep(10);
		v = sn_wait_training(k);
		if (v == ML_TX_NORMAL_MODE)
			break;
		drm_warn(&k->drm, "eDP link training attempt %d failed (%d)\n", i + 1, v);
	}
	if (i == SN_LINK_TRIES)
		goto fail;

	/*
	 * Link training rewrites the TX swing; the vendor sets it back because
	 * the WiFi antenna cable next to the eDP cable makes the panel flicker.
	 */
	sn_write(k, SN_DATARATE_CONFIG, 0x21);

	sn_write(k, SN_CHA_ACTIVE_LINE_LEN, m->hdisplay & 0xff);
	sn_write(k, SN_CHA_ACTIVE_LINE_LEN + 1, m->hdisplay >> 8);
	sn_write(k, SN_CHA_ACTIVE_LINE_LEN + 2, 0);
	sn_write(k, SN_CHA_ACTIVE_LINE_LEN + 3, 0);
	sn_write(k, SN_CHA_VDISPLAY, m->vdisplay & 0xff);
	sn_write(k, SN_CHA_VDISPLAY + 1, m->vdisplay >> 8);
	sn_write(k, SN_CHA_HSYNC_WIDTH, (m->hsync_end - m->hsync_start) & 0xff);
	sn_write(k, SN_CHA_HSYNC_WIDTH + 1, (m->hsync_end - m->hsync_start) >> 8);
	sn_write(k, SN_CHA_VSYNC_WIDTH, (m->vsync_end - m->vsync_start) & 0xff);
	sn_write(k, SN_CHA_VSYNC_WIDTH + 1, (m->vsync_end - m->vsync_start) >> 8);
	sn_write(k, SN_CHA_HBP, m->htotal - m->hsync_end);
	sn_write(k, SN_CHA_VBP, m->vtotal - m->vsync_end);
	sn_write(k, SN_CHA_HFP, m->hsync_start - m->hdisplay);
	sn_write(k, SN_CHA_VFP, m->vsync_start - m->vdisplay);
	sn_write(k, SN_DATA_FORMAT, 0);		/* 24 bpp */
	sn_write(k, SN_TEST_PATTERN, 0);

	sn_write(k, SN_ENH_FRAME, 0x0d);	/* + video stream */
	drm_dbg_driver(&k->drm, "eDP link up\n");
	return 0;

fail:
	k->link_failures++;
	kirin_edp_dump(k);
	return -EIO;
}

/* sn65dsi86_disable(): stream, main link and DP PLL off */
void kirin_edp_disable(struct kirin_dss *k)
{
	if (!k->bridge)
		return;
	sn_update(k, SN_ENH_FRAME, VSTREAM_ENABLE, 0);
	sn_write(k, SN_ML_TX_MODE, ML_TX_MAIN_LINK_OFF);
	sn_write(k, SN_PLL_ENABLE, 0);
}

/* laptop_ua_bridge_runtime_suspend(): panel VDD, bridge, clock, supplies */
void kirin_edp_power_off(struct kirin_dss *k)
{
	if (!k->bridge)
		return;
	msleep(25);
	gpiod_set_value_cansleep(k->gpio_panel_vdd, 0);
	msleep(5);
	gpiod_set_value_cansleep(k->gpio_bridge_en, 0);
	clk_disable_unprepare(k->bridge_refclk);
	msleep(5);
	gpiod_set_value_cansleep(k->gpio_1v8, 0);
	msleep(20);
	gpiod_set_value_cansleep(k->gpio_1v2, 0);
	msleep(20);
	k->panel_off_time = ktime_get();
}

static void kirin_put_i2c(void *data)
{
	put_device(&((struct i2c_client *)data)->dev);
}

/*
 * A supply or enable GPIO. With the panel lit by UEFI the line is taken over
 * at the level it has (it should be driven high); otherwise it starts low.
 */
static int kirin_edp_gpio(struct kirin_dss *k, const char *name, bool running,
			  struct gpio_desc **out)
{
	struct device *dev = k->drm.dev;
	struct gpio_desc *d;
	int val = 0;

	d = devm_gpiod_get(dev, name, running ? GPIOD_ASIS : GPIOD_OUT_LOW);
	if (IS_ERR(d))
		return dev_err_probe(dev, PTR_ERR(d), "%s GPIO\n", name);
	if (running) {
		if (gpiod_get_direction(d) != 0)
			drm_warn(&k->drm, "%s GPIO is not an output\n", name);
		val = gpiod_get_value_cansleep(d);
		if (val < 0)
			return dev_err_probe(dev, val, "%s GPIO\n", name);
		gpiod_direction_output(d, val);
	}
	drm_dbg_driver(&k->drm, "%s GPIO %d\n", name, val);
	*out = d;
	return val;
}

static void kirin_clk_put(void *data)
{
	clk_put(data);
}

/*
 * Claim the bridge and panel resources. When UEFI lit the panel ("running")
 * the outputs keep their level and the reference clock stays on.
 */
int kirin_edp_init(struct kirin_dss *k, bool running)
{
	struct device *dev = k->drm.dev;
	struct device_node *np;
	struct i2c_client *client;
	struct clk *clk;
	int ret, v[4];

	INIT_DELAYED_WORK(&k->backlight_work, kirin_backlight_work);

	k->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(k->backlight))
		return dev_err_probe(dev, PTR_ERR(k->backlight), "backlight\n");

	np = of_parse_phandle(dev->of_node, "hisilicon,edp-bridge", 0);
	if (!np) {
		drm_warn(&k->drm, "no eDP bridge in the DT: panel power stays as UEFI left it\n");
		return 0;
	}
	client = of_find_i2c_device_by_node(np);
	clk = of_clk_get_by_name(np, "clk_nfc");
	of_node_put(np);
	if (!client) {
		if (!IS_ERR(clk))
			clk_put(clk);
		return dev_err_probe(dev, -EPROBE_DEFER, "eDP bridge I2C device\n");
	}
	ret = devm_add_action_or_reset(dev, kirin_put_i2c, client);
	if (ret)
		return ret;
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "bridge reference clock\n");
	ret = devm_add_action_or_reset(dev, kirin_clk_put, clk);
	if (ret)
		return ret;

	v[0] = kirin_edp_gpio(k, "bridge-1v2", running, &k->gpio_1v2);
	if (v[0] < 0)
		return v[0];
	v[1] = kirin_edp_gpio(k, "bridge-1v8", running, &k->gpio_1v8);
	if (v[1] < 0)
		return v[1];
	v[2] = kirin_edp_gpio(k, "bridge-enable", running, &k->gpio_bridge_en);
	if (v[2] < 0)
		return v[2];
	v[3] = kirin_edp_gpio(k, "panel-vdd", running, &k->gpio_panel_vdd);
	if (v[3] < 0)
		return v[3];
	if (running)
		drm_info(&k->drm, "panel power from UEFI: 1v2 %d 1v8 %d bridge EN %d panel VDD %d\n",
			 v[0], v[1], v[2], v[3]);

	if (running) {
		ret = clk_prepare_enable(clk);
		if (ret)
			return dev_err_probe(dev, ret, "bridge reference clock\n");
	}
	k->bridge_refclk = clk;
	k->bridge = client;
	return 0;
}
