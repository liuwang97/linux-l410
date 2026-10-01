/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Huawei "echub" embedded controller (Huawei Qingyun L410 and relatives).
 */
#ifndef __LINUX_MFD_HUAWEI_ECHUB_H
#define __LINUX_MFD_HUAWEI_ECHUB_H

#include <linux/types.h>

struct huawei_echub;

/* Byte-wide EC registers, selected by the argument byte */
#define ECHUB_REG_INFO			0x0280
/* Word-wide smart battery values, selected by the argument byte */
#define ECHUB_REG_BATTERY		0x0451
/* Command register: the argument byte is the command */
#define ECHUB_REG_COMMAND		0x02b2

/* ECHUB_REG_INFO arguments */
#define ECHUB_INFO_BAT_STATE		0x81	/* BIT(1): battery present */
#define ECHUB_INFO_BAT_ALARM		0x82	/* BIT(4) failure, BIT(5) hot, BIT(6) overvoltage */
#define ECHUB_INFO_BAT_RSOC		0x90	/* relative state of charge, % */
#define ECHUB_INFO_BAT_DESIGN_CAP_LO	0xa2	/* mAh */
#define ECHUB_INFO_BAT_DESIGN_CAP_HI	0xa3
#define ECHUB_INFO_BAT_DESIGN_MV_LO	0xa4	/* nominal voltage, mV */
#define ECHUB_INFO_BAT_DESIGN_MV_HI	0xa5
#define ECHUB_INFO_BAT_CYCLES_LO	0xaa
#define ECHUB_INFO_BAT_CYCLES_HI	0xab

/* ECHUB_REG_BATTERY arguments (little endian 16 bit values) */
#define ECHUB_BAT_CURRENT		0x01	/* mA, signed */
#define ECHUB_BAT_VOLTAGE		0x06	/* mV */
#define ECHUB_BAT_REMAINING		0x07	/* mAh */
#define ECHUB_BAT_FULL			0x08	/* mAh */
#define ECHUB_BAT_TEMP			0x09	/* 0.1 K */

/* ECHUB_REG_COMMAND commands */
#define ECHUB_CMD_OS_ON			0x16	/* OS is up (sent by the vendor kernel at boot) */

int huawei_echub_read(struct huawei_echub *ec, u16 reg, u8 arg, u8 *data, size_t len);
int huawei_echub_write(struct huawei_echub *ec, u16 reg, u8 arg);

static inline int huawei_echub_read_u8(struct huawei_echub *ec, u16 reg, u8 arg)
{
	u8 val;
	int ret;

	ret = huawei_echub_read(ec, reg, arg, &val, 1);
	return ret ? ret : val;
}

/* Two byte-wide registers holding the low and the high byte of a value */
static inline int huawei_echub_read_u16_pair(struct huawei_echub *ec, u8 arg_lo, u8 arg_hi)
{
	int lo, hi;

	lo = huawei_echub_read_u8(ec, ECHUB_REG_INFO, arg_lo);
	if (lo < 0)
		return lo;
	hi = huawei_echub_read_u8(ec, ECHUB_REG_INFO, arg_hi);
	if (hi < 0)
		return hi;
	return hi << 8 | lo;
}

static inline int huawei_echub_read_le16(struct huawei_echub *ec, u8 arg)
{
	u8 buf[2];
	int ret;

	ret = huawei_echub_read(ec, ECHUB_REG_BATTERY, arg, buf, sizeof(buf));
	return ret ? ret : (buf[1] << 8 | buf[0]);
}

#endif /* __LINUX_MFD_HUAWEI_ECHUB_H */
