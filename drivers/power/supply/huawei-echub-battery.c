// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery and AC adapter of the Huawei "echub" embedded controller
 *
 * The EC exposes the smart battery values; there is no event interrupt, so the
 * values are polled. The adapter state comes from a GPIO driven by the EC.
 *
 * Ported from the vendor 4.19 drivers/echub/battery.
 */

#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/huawei-echub.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

#define ECHUB_BAT_POLL_MS		10000
#define ECHUB_BAT_POLL_LOW_MS		5000
#define ECHUB_BAT_STATIC_EVERY		60	/* polls between cycle count updates */
#define ECHUB_AC_DEBOUNCE_MS		200

#define ECHUB_BAT_PRESENT		BIT(1)
#define ECHUB_BAT_ALARM_FAILURE		BIT(4)
#define ECHUB_BAT_ALARM_HOT		BIT(5)
#define ECHUB_BAT_ALARM_OVERVOLTAGE	BIT(6)

#define ECHUB_CELL_NOMINAL_MV		3800
#define ECHUB_CELL_MAX_MV		4400
#define ECHUB_CAPACITY_CRITICAL		3
#define ECHUB_CAPACITY_LOW		10

struct echub_battery {
	struct device *dev;
	struct huawei_echub *ec;
	struct power_supply *bat;
	struct power_supply *ac;
	struct gpio_desc *ac_gpio;
	struct delayed_work poll_work;
	struct delayed_work ac_work;
	struct mutex lock;		/* protects the cached values below */

	bool valid;
	bool present;
	bool ac_online;
	int capacity;			/* % */
	int voltage_mv;
	int current_ma;
	int charge_mah;
	int full_mah;
	int temp_dk;			/* 0.1 K */
	int alarm;
	int design_mah;
	int design_mv;
	int cycles;
	unsigned int polls;
};

static bool echub_ac_online(struct echub_battery *bat)
{
	if (!bat->ac_gpio)
		return false;
	return gpiod_get_value_cansleep(bat->ac_gpio) > 0;
}

static int echub_bat_read_static(struct echub_battery *bat)
{
	struct huawei_echub *ec = bat->ec;
	int cap, mv;

	cap = huawei_echub_read_u16_pair(ec, ECHUB_INFO_BAT_DESIGN_CAP_LO,
					 ECHUB_INFO_BAT_DESIGN_CAP_HI);
	if (cap < 0)
		return cap;
	mv = huawei_echub_read_u16_pair(ec, ECHUB_INFO_BAT_DESIGN_MV_LO,
					ECHUB_INFO_BAT_DESIGN_MV_HI);
	if (mv < 0)
		return mv;

	scoped_guard(mutex, &bat->lock) {
		bat->design_mah = cap;
		bat->design_mv = mv;
	}
	return 0;
}

static int echub_bat_read_cycles(struct echub_battery *bat)
{
	int cycles;

	cycles = huawei_echub_read_u16_pair(bat->ec, ECHUB_INFO_BAT_CYCLES_LO,
					    ECHUB_INFO_BAT_CYCLES_HI);
	if (cycles < 0)
		return cycles;

	guard(mutex)(&bat->lock);
	bat->cycles = cycles;
	return 0;
}

/* Returns 1 if a value that userspace cares about changed. */
static int echub_bat_update(struct echub_battery *bat)
{
	struct huawei_echub *ec = bat->ec;
	int state, capacity, voltage, current_ma, charge, full, temp, alarm;
	bool present, changed;

	state = huawei_echub_read_u8(ec, ECHUB_REG_INFO, ECHUB_INFO_BAT_STATE);
	if (state < 0)
		return state;
	present = state & ECHUB_BAT_PRESENT;

	if (!present) {
		guard(mutex)(&bat->lock);
		changed = !bat->valid || bat->present;
		bat->present = false;
		bat->valid = true;
		return changed;
	}

	capacity = huawei_echub_read_u8(ec, ECHUB_REG_INFO, ECHUB_INFO_BAT_RSOC);
	if (capacity < 0)
		return capacity;
	voltage = huawei_echub_read_le16(ec, ECHUB_BAT_VOLTAGE);
	if (voltage < 0)
		return voltage;
	current_ma = huawei_echub_read_le16(ec, ECHUB_BAT_CURRENT);
	if (current_ma < 0)
		return current_ma;
	charge = huawei_echub_read_le16(ec, ECHUB_BAT_REMAINING);
	if (charge < 0)
		return charge;
	full = huawei_echub_read_le16(ec, ECHUB_BAT_FULL);
	if (full < 0)
		return full;
	temp = huawei_echub_read_le16(ec, ECHUB_BAT_TEMP);
	if (temp < 0)
		return temp;
	alarm = huawei_echub_read_u8(ec, ECHUB_REG_INFO, ECHUB_INFO_BAT_ALARM);
	if (alarm < 0)
		return alarm;

	guard(mutex)(&bat->lock);
	changed = !bat->valid || !bat->present || bat->capacity != capacity ||
		  (bat->current_ma > 0) != ((s16)current_ma > 0) ||
		  (bat->current_ma < 0) != ((s16)current_ma < 0) ||
		  bat->alarm != alarm;
	bat->present = true;
	bat->capacity = min(capacity, 100);
	bat->voltage_mv = voltage;
	bat->current_ma = (s16)current_ma;
	bat->charge_mah = charge;
	bat->full_mah = full;
	bat->temp_dk = temp;
	bat->alarm = alarm;
	bat->valid = true;
	return changed;
}

/* Returns true if the adapter state changed. */
static bool echub_ac_update(struct echub_battery *bat)
{
	bool online = echub_ac_online(bat);
	bool changed;

	scoped_guard(mutex, &bat->lock) {
		changed = bat->ac_online != online;
		bat->ac_online = online;
	}
	if (changed)
		power_supply_changed(bat->ac);
	return changed;
}

static void echub_bat_poll_work(struct work_struct *work)
{
	struct echub_battery *bat = container_of(work, struct echub_battery, poll_work.work);
	unsigned int delay = ECHUB_BAT_POLL_MS;
	int ret;

	/* the interrupt may be missing; the poll catches adapter changes too */
	echub_ac_update(bat);

	if (!bat->design_mah)
		echub_bat_read_static(bat);
	if (bat->polls++ % ECHUB_BAT_STATIC_EVERY == 0)
		echub_bat_read_cycles(bat);

	ret = echub_bat_update(bat);
	if (ret < 0)
		dev_warn_ratelimited(bat->dev, "failed to read battery: %d\n", ret);
	else if (ret)
		power_supply_changed(bat->bat);

	if (bat->valid && bat->present && bat->capacity <= ECHUB_CAPACITY_LOW)
		delay = ECHUB_BAT_POLL_LOW_MS;
	queue_delayed_work(system_freezable_power_efficient_wq, &bat->poll_work, msecs_to_jiffies(delay));
}

static void echub_ac_work(struct work_struct *work)
{
	struct echub_battery *bat = container_of(work, struct echub_battery, ac_work.work);

	/* the battery status follows; let the EC settle first */
	if (echub_ac_update(bat)) {
		power_supply_changed(bat->bat);
		mod_delayed_work(system_freezable_power_efficient_wq, &bat->poll_work, msecs_to_jiffies(1000));
	}
}

static irqreturn_t echub_ac_irq(int irq, void *data)
{
	struct echub_battery *bat = data;

	mod_delayed_work(system_wq, &bat->ac_work, msecs_to_jiffies(ECHUB_AC_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

static int echub_bat_status(struct echub_battery *bat)
{
	if (bat->ac_online) {
		if (bat->capacity >= 100)
			return POWER_SUPPLY_STATUS_FULL;
		if (bat->current_ma > 0)
			return POWER_SUPPLY_STATUS_CHARGING;
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int echub_bat_health(struct echub_battery *bat)
{
	if (bat->alarm & ECHUB_BAT_ALARM_OVERVOLTAGE)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	if (bat->alarm & ECHUB_BAT_ALARM_HOT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (bat->alarm & ECHUB_BAT_ALARM_FAILURE)
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int echub_bat_capacity_level(struct echub_battery *bat)
{
	if (bat->capacity <= ECHUB_CAPACITY_CRITICAL)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	if (bat->capacity <= ECHUB_CAPACITY_LOW)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	if (bat->capacity >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
}

static int echub_bat_max_design_uv(struct echub_battery *bat)
{
	int cells = DIV_ROUND_CLOSEST(bat->design_mv, ECHUB_CELL_NOMINAL_MV);

	return max(cells, 1) * ECHUB_CELL_MAX_MV * 1000;
}

static int echub_bat_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct echub_battery *bat = power_supply_get_drvdata(psy);

	guard(mutex)(&bat->lock);

	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		val->intval = bat->valid && bat->present;
		return 0;
	}
	if (psp == POWER_SUPPLY_PROP_SCOPE) {
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	}
	if (!bat->valid || !bat->present)
		return -ENODATA;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = echub_bat_status(bat);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = echub_bat_health(bat);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat->capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = echub_bat_capacity_level(bat);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->design_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = echub_bat_max_design_uv(bat);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bat->current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bat->charge_mah * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bat->full_mah * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bat->design_mah * 1000;
		break;
	/* mAh * mV = uWh */
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = bat->charge_mah * bat->design_mv;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = bat->full_mah * bat->design_mv;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = bat->design_mah * bat->design_mv;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat->temp_dk - 2731;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = bat->cycles;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const enum power_supply_property echub_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static const struct power_supply_desc echub_bat_desc = {
	.name = "echub-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = echub_bat_props,
	.num_properties = ARRAY_SIZE(echub_bat_props),
	.get_property = echub_bat_get_property,
};

static int echub_ac_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct echub_battery *bat = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		guard(mutex)(&bat->lock);
		val->intval = bat->ac_online;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property echub_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc echub_ac_desc = {
	.name = "echub-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = echub_ac_props,
	.num_properties = ARRAY_SIZE(echub_ac_props),
	.get_property = echub_ac_get_property,
};

static char *echub_ac_supplied_to[] = { "echub-battery" };

static int echub_battery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct echub_battery *bat;
	int irq, ret;

	bat = devm_kzalloc(dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = dev;
	bat->ec = dev_get_drvdata(dev->parent);
	if (!bat->ec)
		return -EPROBE_DEFER;

	ret = devm_mutex_init(dev, &bat->lock);
	if (ret)
		return ret;

	bat->ac_gpio = devm_gpiod_get_optional(dev, "ac-detect", GPIOD_IN);
	if (IS_ERR(bat->ac_gpio))
		return dev_err_probe(dev, PTR_ERR(bat->ac_gpio), "no AC detect GPIO\n");
	if (!bat->ac_gpio)
		dev_warn(dev, "no AC detect GPIO, adapter will always read offline\n");
	bat->ac_online = echub_ac_online(bat);

	ret = echub_bat_read_static(bat);
	if (ret)
		dev_warn(dev, "failed to read battery design data: %d\n", ret);
	echub_bat_read_cycles(bat);
	ret = echub_bat_update(bat);
	if (ret < 0)
		dev_warn(dev, "failed to read battery: %d\n", ret);

	/* teardown runs in reverse: interrupt, then the work items, then the supplies */
	cfg.drv_data = bat;
	cfg.fwnode = dev_fwnode(dev);
	bat->bat = devm_power_supply_register(dev, &echub_bat_desc, &cfg);
	if (IS_ERR(bat->bat))
		return dev_err_probe(dev, PTR_ERR(bat->bat), "failed to register battery\n");

	cfg.fwnode = NULL;
	cfg.supplied_to = echub_ac_supplied_to;
	cfg.num_supplicants = ARRAY_SIZE(echub_ac_supplied_to);
	bat->ac = devm_power_supply_register(dev, &echub_ac_desc, &cfg);
	if (IS_ERR(bat->ac))
		return dev_err_probe(dev, PTR_ERR(bat->ac), "failed to register adapter\n");

	ret = devm_delayed_work_autocancel(dev, &bat->poll_work, echub_bat_poll_work);
	if (ret)
		return ret;
	ret = devm_delayed_work_autocancel(dev, &bat->ac_work, echub_ac_work);
	if (ret)
		return ret;

	if (bat->ac_gpio) {
		irq = gpiod_to_irq(bat->ac_gpio);
		if (irq > 0)
			ret = devm_request_irq(dev, irq, echub_ac_irq,
					       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					       "echub-ac", bat);
		if (irq <= 0 || ret)
			dev_warn(dev, "no AC detect interrupt (%d), polling\n", irq > 0 ? ret : irq);
	}

	dev_info(dev, "battery %s, %d%%, %d mV, design %d mAh @ %d mV, AC %s\n",
		 bat->present ? "present" : "absent", bat->capacity, bat->voltage_mv,
		 bat->design_mah, bat->design_mv, bat->ac_online ? "online" : "offline");

	queue_delayed_work(system_freezable_power_efficient_wq, &bat->poll_work, msecs_to_jiffies(ECHUB_BAT_POLL_MS));
	return 0;
}

static const struct platform_device_id echub_battery_id[] = {
	{ "huawei-echub-battery" },
	{ }
};
MODULE_DEVICE_TABLE(platform, echub_battery_id);

static struct platform_driver echub_battery_driver = {
	.driver = {
		.name = "huawei-echub-battery",
	},
	.probe = echub_battery_probe,
	.id_table = echub_battery_id,
};
module_platform_driver(echub_battery_driver);

MODULE_DESCRIPTION("Huawei echub EC battery and AC adapter");
MODULE_LICENSE("GPL");
