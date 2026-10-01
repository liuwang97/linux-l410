// SPDX-License-Identifier: GPL-2.0
/*
 * Read-only access to the Hi110x board EEPROM (WiFi calibration data and the WiFi/BT
 * MAC addresses), a 16-bit addressed EEPROM at 0x54 on I2C4. Same layout as the
 * vendor drivers/hisi/hi1103_eeprom (drv_e2prom_read). Never writes.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/errno.h>

#define HI110X_EEPROM_I2C_BUS	4
#define HI110X_EEPROM_ADDR	0x54
#define HI110X_EEPROM_MAX_XFER	128

struct hi110x_eeprom_part {
	const char *name;
	unsigned int start;
	unsigned int size;
};

static const struct hi110x_eeprom_part g_hi110x_eeprom_parts[] = {
	{ "WLAN", 0, 2048 },
	{ "BT", 2048, 2048 },
	{ "FAC", 4096, 128 },
	{ "MACWLAN", 4224, 128 },
	{ "MACBT", 4352, 128 },
};

static const struct hi110x_eeprom_part *hi110x_eeprom_find(const char *name)
{
	int i;

	/* vendor semantics: prefix match on the requested name */
	for (i = 0; i < ARRAY_SIZE(g_hi110x_eeprom_parts); i++)
		if (!strncmp(g_hi110x_eeprom_parts[i].name, name, strlen(name)))
			return &g_hi110x_eeprom_parts[i];
	return NULL;
}

int drv_e2prom_read(unsigned char *part_name, unsigned int offset, char *rbuf, unsigned int rsize)
{
	const struct hi110x_eeprom_part *part;
	struct i2c_adapter *adap;
	struct i2c_msg msg[2];
	unsigned int addr;
	u8 reg[2];
	int ret;

	if (!part_name || !rbuf || rsize == 0 || rsize > HI110X_EEPROM_MAX_XFER)
		return -1;
	part = hi110x_eeprom_find((const char *)part_name);
	if (!part)
		return -1;

	adap = i2c_get_adapter(HI110X_EEPROM_I2C_BUS);
	if (!adap)
		return -1;

	addr = part->start + offset;
	reg[0] = addr >> 8;
	reg[1] = addr & 0xff;
	msg[0].addr = HI110X_EEPROM_ADDR;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg;
	msg[1].addr = HI110X_EEPROM_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = rsize;
	msg[1].buf = (u8 *)rbuf;
	ret = i2c_transfer(adap, msg, 2);
	i2c_put_adapter(adap);

	return ret == 2 ? 0 : -1;
}
