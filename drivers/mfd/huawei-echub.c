// SPDX-License-Identifier: GPL-2.0-only
/*
 * Huawei "echub" embedded controller
 *
 * Found on the Huawei Qingyun L410 (Kirin 990) laptop at I2C address 0x38.
 * The EC reports the battery, switches a few rails and the keyboard mute LED.
 * It also emulates the HID-over-I2C keyboard on the same bus.
 *
 * Every transaction is a 4 byte write followed by a read in one I2C transfer:
 *
 *   write:  reg[15:8] reg[7:0] 0x01 arg
 *   read:   status count data[count] pec    register read
 *           status pec                      write / command
 *
 * A zero status means success. The PEC is the SMBus CRC-8 over the whole
 * transfer including both address bytes. The EC is a small microcontroller that
 * drops requests arriving back to back, so transfers are spaced out.
 *
 * Ported from the vendor 4.19 drivers/echub.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/mfd/core.h>
#include <linux/mfd/huawei-echub.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/syscore_ops.h>
#include <linux/uaccess.h>

#define ECHUB_MAX_DATA		8
#define ECHUB_XFER_GAP_US	10000	/* the vendor driver waits 10 ms between requests */
#define ECHUB_XFER_TRIES	3

struct huawei_echub {
	struct device *dev;
	struct i2c_client *client;
	struct mutex lock;		/* serializes EC transactions */
	ktime_t next_xfer;

	/* statistics, for debugfs */
	unsigned int xfers;
	unsigned int errors;
	unsigned int retries;
	unsigned int status_errors;
	unsigned int pec_errors;

	struct dentry *debugfs;
	u16 dbg_reg;
	u8 dbg_arg;
	u8 dbg_len;
};

static int echub_xfer(struct huawei_echub *ec, u8 *tx, u8 *rx, size_t rx_len)
{
	struct i2c_client *client = ec->client;
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 4, .buf = tx },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = rx_len, .buf = rx },
	};
	s64 wait;
	int ret;

	lockdep_assert_held(&ec->lock);

	wait = ktime_us_delta(ec->next_xfer, ktime_get());
	if (wait > 0)
		fsleep(wait);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	ec->next_xfer = ktime_add_us(ktime_get(), ECHUB_XFER_GAP_US);
	ec->xfers++;
	if (ret < 0)
		return ret;
	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static u8 echub_pec(struct huawei_echub *ec, u8 *tx, u8 *rx, size_t rx_len)
{
	u8 addr = ec->client->addr << 1;
	u8 pec;

	pec = i2c_smbus_pec(0, &addr, 1);
	pec = i2c_smbus_pec(pec, tx, 4);
	addr |= 1;
	pec = i2c_smbus_pec(pec, &addr, 1);
	return i2c_smbus_pec(pec, rx, rx_len);
}

static int __echub_read(struct huawei_echub *ec, u16 reg, u8 arg, u8 *data, size_t len)
{
	u8 tx[4] = { reg >> 8, reg & 0xff, 1, arg };
	u8 rx[ECHUB_MAX_DATA + 3];
	int ret;

	ret = echub_xfer(ec, tx, rx, len + 3);
	if (ret)
		return ret;

	if (rx[0]) {
		ec->status_errors++;
		return -EIO;
	}
	if (rx[1] != len)
		return -EPROTO;

	/* The vendor driver only checked byte reads, but word reads carry a valid PEC too. */
	if (rx[len + 2] != echub_pec(ec, tx, rx, len + 2)) {
		ec->pec_errors++;
		return -EBADMSG;
	}

	memcpy(data, &rx[2], len);
	return 0;
}

/**
 * huawei_echub_read() - read an EC register
 * @ec: the EC
 * @reg: register (ECHUB_REG_*)
 * @arg: argument byte, selects the value within @reg
 * @data: buffer for the value
 * @len: number of data bytes the EC returns for @reg
 *
 * Return: 0 on success, negative errno otherwise.
 */
int huawei_echub_read(struct huawei_echub *ec, u16 reg, u8 arg, u8 *data, size_t len)
{
	int i, ret;

	if (!len || len > ECHUB_MAX_DATA)
		return -EINVAL;

	guard(mutex)(&ec->lock);
	for (i = 0; i < ECHUB_XFER_TRIES; i++) {
		if (i)
			ec->retries++;
		ret = __echub_read(ec, reg, arg, data, len);
		if (!ret)
			return 0;
	}
	ec->errors++;
	dev_dbg(ec->dev, "read %04x/%02x failed: %d\n", reg, arg, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(huawei_echub_read);

static int __echub_write(struct huawei_echub *ec, u16 reg, u8 arg)
{
	u8 tx[4] = { reg >> 8, reg & 0xff, 1, arg };
	u8 rx[2];
	int ret;

	ret = echub_xfer(ec, tx, rx, sizeof(rx));
	if (ret)
		return ret;

	if (rx[0]) {
		ec->status_errors++;
		return -EIO;
	}
	if (rx[1] != echub_pec(ec, tx, rx, 1)) {
		ec->pec_errors++;
		return -EBADMSG;
	}
	return 0;
}

/**
 * huawei_echub_write() - write an EC register or send a command
 * @ec: the EC
 * @reg: register (ECHUB_REG_*)
 * @arg: value, or the command for ECHUB_REG_COMMAND
 *
 * Return: 0 on success, negative errno otherwise.
 */
int huawei_echub_write(struct huawei_echub *ec, u16 reg, u8 arg)
{
	int i, ret;

	guard(mutex)(&ec->lock);
	for (i = 0; i < ECHUB_XFER_TRIES; i++) {
		if (i)
			ec->retries++;
		ret = __echub_write(ec, reg, arg);
		if (!ret)
			return 0;
	}
	ec->errors++;
	dev_dbg(ec->dev, "write %04x/%02x failed: %d\n", reg, arg, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(huawei_echub_write);

/*
 * debugfs: "stats", and "read" for poking at EC registers:
 *   echo "0x0280 0x90 1" > read; cat read
 */
static int echub_stats_show(struct seq_file *s, void *unused)
{
	struct huawei_echub *ec = s->private;

	guard(mutex)(&ec->lock);
	seq_printf(s, "xfers: %u\nerrors: %u\nretries: %u\nstatus_errors: %u\npec_errors: %u\n",
		   ec->xfers, ec->errors, ec->retries, ec->status_errors, ec->pec_errors);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(echub_stats);

static ssize_t echub_dbg_read_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct huawei_echub *ec = file->private_data;
	unsigned int reg, arg, len;
	char buf[32];

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%i %i %i", &reg, &arg, &len) != 3 ||
	    reg > 0xffff || arg > 0xff || !len || len > ECHUB_MAX_DATA)
		return -EINVAL;

	guard(mutex)(&ec->lock);
	ec->dbg_reg = reg;
	ec->dbg_arg = arg;
	ec->dbg_len = len;
	return count;
}

static ssize_t echub_dbg_read_read(struct file *file, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct huawei_echub *ec = file->private_data;
	u8 data[ECHUB_MAX_DATA];
	char buf[64];
	int ret, n;

	if (*ppos)
		return 0;
	if (!ec->dbg_len)
		return -EINVAL;

	ret = huawei_echub_read(ec, ec->dbg_reg, ec->dbg_arg, data, ec->dbg_len);
	if (ret)
		n = scnprintf(buf, sizeof(buf), "%04x/%02x: error %d\n",
			      ec->dbg_reg, ec->dbg_arg, ret);
	else
		n = scnprintf(buf, sizeof(buf), "%04x/%02x: %*ph\n",
			      ec->dbg_reg, ec->dbg_arg, ec->dbg_len, data);
	return simple_read_from_buffer(ubuf, count, ppos, buf, n);
}

static const struct file_operations echub_dbg_read_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = echub_dbg_read_read,
	.write = echub_dbg_read_write,
	.llseek = default_llseek,
};

static void echub_debugfs_remove(void *data)
{
	struct huawei_echub *ec = data;

	debugfs_remove_recursive(ec->debugfs);
}

static void echub_debugfs_init(struct huawei_echub *ec)
{
	ec->debugfs = debugfs_create_dir("huawei-echub", NULL);
	debugfs_create_file("stats", 0444, ec->debugfs, ec, &echub_stats_fops);
	debugfs_create_file("read", 0600, ec->debugfs, ec, &echub_dbg_read_fops);
	devm_add_action_or_reset(ec->dev, echub_debugfs_remove, ec);
}

static const struct mfd_cell echub_cells[] = {
	{
		.name = "huawei-echub-battery",
		.of_compatible = "huawei,echub-battery",
	},
	{
		.name = "huawei-echub-led",
		.of_compatible = "huawei,echub-keyboard",
	},
};

static int echub_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct huawei_echub *ec;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	ec->client = client;
	ret = devm_mutex_init(dev, &ec->lock);
	if (ret)
		return ret;
	i2c_set_clientdata(client, ec);

	ret = huawei_echub_read_u8(ec, ECHUB_REG_INFO, ECHUB_INFO_BAT_RSOC);
	if (ret < 0)
		return dev_err_probe(dev, ret, "EC does not answer\n");
	dev_info(dev, "EC online, battery at %d%%\n", ret);

	/* The vendor kernel tells the EC that the OS is up. */
	ret = huawei_echub_write(ec, ECHUB_REG_COMMAND, ECHUB_CMD_OS_ON);
	if (ret)
		dev_warn(dev, "failed to send OS-on message: %d\n", ret);

	echub_debugfs_init(ec);

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, echub_cells,
				   ARRAY_SIZE(echub_cells), NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add EC functions\n");

	return 0;
}

static const struct of_device_id echub_of_match[] = {
	{ .compatible = "huawei,echub_i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, echub_of_match);

static const struct i2c_device_id echub_id[] = {
	{ "echub_i2c" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, echub_id);

static struct i2c_driver echub_driver = {
	.driver = {
		.name = "huawei-echub",
		.of_match_table = echub_of_match,
	},
	.probe = echub_probe,
	.id_table = echub_id,
};

/*
 * EC state sync: a GPIO telling the EC whether the SoC is running (high) or
 * suspended (low). The EC needs the line low for at least 10 ms.
 */
#define ECHUB_SYNC_MIN_LOW_MS	10

struct echub_sync {
	struct gpio_desc *gpio;
	ktime_t suspended_at;
	struct syscore syscore;
};

static int echub_sync_suspend(void *data)
{
	struct echub_sync *sync = data;

	gpiod_set_value(sync->gpio, 0);
	sync->suspended_at = ktime_get();
	return 0;
}

static void echub_sync_resume(void *data)
{
	struct echub_sync *sync = data;
	s64 low_ms = ktime_ms_delta(ktime_get(), sync->suspended_at);

	if (low_ms < ECHUB_SYNC_MIN_LOW_MS)
		mdelay(ECHUB_SYNC_MIN_LOW_MS - low_ms);
	gpiod_set_value(sync->gpio, 1);
}

static const struct syscore_ops echub_sync_syscore_ops = {
	.suspend = echub_sync_suspend,
	.resume = echub_sync_resume,
};

static void echub_sync_unregister(void *data)
{
	struct echub_sync *sync = data;

	unregister_syscore(&sync->syscore);
}

static int echub_sync_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct echub_sync *sync;

	sync = devm_kzalloc(dev, sizeof(*sync), GFP_KERNEL);
	if (!sync)
		return -ENOMEM;

	sync->gpio = devm_gpiod_get(dev, "sync", GPIOD_OUT_HIGH);
	if (IS_ERR(sync->gpio))
		return dev_err_probe(dev, PTR_ERR(sync->gpio), "no sync GPIO\n");

	sync->syscore.ops = &echub_sync_syscore_ops;
	sync->syscore.data = sync;
	register_syscore(&sync->syscore);
	return devm_add_action_or_reset(dev, echub_sync_unregister, sync);
}

static const struct of_device_id echub_sync_of_match[] = {
	{ .compatible = "huawei,ec_state_sync" },
	{ }
};
MODULE_DEVICE_TABLE(of, echub_sync_of_match);

static struct platform_driver echub_sync_driver = {
	.driver = {
		.name = "huawei-echub-sync",
		.of_match_table = echub_sync_of_match,
	},
	.probe = echub_sync_probe,
};

static int __init echub_init(void)
{
	int ret;

	ret = platform_driver_register(&echub_sync_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&echub_driver);
	if (ret)
		platform_driver_unregister(&echub_sync_driver);
	return ret;
}
module_init(echub_init);

static void __exit echub_exit(void)
{
	i2c_del_driver(&echub_driver);
	platform_driver_unregister(&echub_sync_driver);
}
module_exit(echub_exit);

MODULE_DESCRIPTION("Huawei echub embedded controller");
MODULE_LICENSE("GPL");
