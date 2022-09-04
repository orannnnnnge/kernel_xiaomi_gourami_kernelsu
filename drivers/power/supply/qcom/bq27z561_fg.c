/*
 * bq27z561 fuel gauge driver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[bq27z561] %s: " fmt, __func__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

#define FG_MAX_INDEX 2
#define RETRY_COUNT 5
#define INVALID_REG_ADDR 0xFF

#define MONITOR_WORK_10S 10
#define MONITOR_WORK_5S 5

#define FG_FLAGS_FD BIT(4)
#define FG_FLAGS_FC BIT(5)
#define FG_FLAGS_DSG BIT(6)
#define FG_FLAGS_RCA BIT(9)
#define FG_FLAGS_FASTCHAGE BIT(5)

#define BATTERY_DIGEST_LEN 32

#define DEFUALT_FULL_DESIGN 100000

#define BQ_REPORT_FULL_SOC 9800
#define BQ_CHARGE_FULL_SOC 9750
#define BQ_RECHARGE_SOC 9900

#define BQ27Z561_DEFUALT_TERM -200
#define BQ27Z561_DEFUALT_RECHARGE_VOL 4380

#define PD_CHG_UPDATE_DELAY_US 20 /*20 sec*/
#define BQ_I2C_FAILED_SOC 15
#define BQ_I2C_FAILED_TEMP 250
#define BQ_I2C_FAILED_TEMP_HIGH 500

enum bq_fg_reg_idx {
	BQ_FG_REG_CTRL = 0,
	BQ_FG_REG_TEMP, /* Battery Temperature */
	BQ_FG_REG_VOLT, /* Battery Voltage */
	BQ_FG_REG_CN, /* Current Now */
	BQ_FG_REG_AI, /* Average Current */
	BQ_FG_REG_BATT_STATUS, /* BatteryStatus */
	BQ_FG_REG_TTE, /* Time to Empty */
	BQ_FG_REG_TTF, /* Time to Full */
	BQ_FG_REG_FCC, /* Full Charge Capacity */
	BQ_FG_REG_RM, /* Remaining Capacity */
	BQ_FG_REG_CC, /* Cycle Count */
	BQ_FG_REG_SOC, /* Relative State of Charge */
	BQ_FG_REG_SOH, /* State of Health */
	BQ_FG_REG_CHG_VOL, /* Charging Voltage*/
	BQ_FG_REG_CHG_CUR, /* Charging Current*/
	BQ_FG_REG_DC, /* Design Capacity */
	BQ_FG_REG_ALT_MAC, /* AltManufactureAccess*/
	BQ_FG_REG_MAC_DATA, /* MACData*/
	BQ_FG_REG_MAC_CHKSUM, /* MACChecksum */
	BQ_FG_REG_MAC_DATA_LEN, /* MACDataLen */
	NUM_REGS,
};

static u8 bq27z561_regs[NUM_REGS] = {
	0x00, /* CONTROL */
	0x06, /* TEMP */
	0x08, /* VOLT */
	0x0C, /* CURRENT NOW */
	0x14, /* AVG CURRENT */
	0x0A, /* FLAGS */
	0x16, /* Time to empty */
	0x18, /* Time to full */
	0x12, /* Full charge capacity */
	0x10, /* Remaining Capacity */
	0x2A, /* CycleCount */
	0x2C, /* State of Charge */
	0x2E, /* State of Health */
	0x30, /* Charging Voltage*/
	0x32, /* Charging Current*/
	0x3C, /* Design Capacity */
	0x3E, /* AltManufacturerAccess*/
	0x40, /* MACData*/
	0x60, /* MACChecksum */
	0x61, /* MACDataLen */
};

enum bq_fg_mac_cmd {
	FG_MAC_CMD_CTRL_STATUS = 0x0000,
	FG_MAC_CMD_DEV_TYPE = 0x0001,
	FG_MAC_CMD_FW_VER = 0x0002,
	FG_MAC_CMD_HW_VER = 0x0003,
	FG_MAC_CMD_IF_SIG = 0x0004,
	FG_MAC_CMD_CHEM_ID = 0x0006,
	FG_MAC_CMD_SHUTDOWN = 0x0010,
	FG_MAC_CMD_GAUGING = 0x0021,
	FG_MAC_CMD_SEAL = 0x0030,
	FG_MAC_CMD_FASTCHARGE_EN = 0x003E,
	FG_MAC_CMD_FASTCHARGE_DIS = 0x003F,
	FG_MAC_CMD_DEV_RESET = 0x0041,
	FG_MAC_CMD_DEVICE_NAME = 0x004A,
	FG_MAC_CMD_DEVICE_CHEM = 0x004B,
	FG_MAC_CMD_MANU_NAME = 0x004C,
	FG_MAC_CMD_CHARGING_STATUS = 0x0055,
	FG_MAC_CMD_LIFETIME1 = 0x0060,
	FG_MAC_CMD_LIFETIME3 = 0x0062,
	FG_MAC_CMD_DASTATUS1 = 0x0071,
	FG_MAC_CMD_ITSTATUS1 = 0x0073,
	FG_MAC_CMD_QMAX = 0x0075,
	FG_MAC_CMD_FCC_SOH = 0x0077,
	FG_MAC_CMD_RA_TABLE = 0x40C0,
};

enum {
	SEAL_STATE_RSVED,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
	SEAL_STATE_FA,
};

enum bq_fg_device {
	BQ27Z561_MASTER = 0,
	BQ27Z561_SLAVE,
	BQ27Z561,
	BQ28Z610,
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct mutex i2c_rw_lock;
	struct mutex data_lock;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];
	char *model_name;

	/* status tracking */

	bool batt_fc;
	bool batt_fd; /* full depleted */
	bool shutdown_soc; /* shutdown */

	bool batt_dsg;
	bool batt_rca; /* remaining capacity alarm */

	int seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	int batt_tte;
	int batt_ttf;
	int batt_soc;
	int batt_fcc; /* Full charge capacity */
	int batt_rm; /* Remaining capacity */
	int batt_dc; /* Design Capacity */
	int batt_volt;
	int batt_temp;
	int batt_curr;
	int batt_resistance;
	int batt_cyclecnt; /* cycle count */
	int batt_st;
	int raw_soc;
	int last_soc;
	int last_rsoc;
	int last_rm;
	int last_fcc;

	int soh;
	int charging_current;
	int chip_ok;
	int charging_voltage;
	int avg_current;
	int plugout_update_count;

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;
	int fake_volt;
	int fake_chip_ok;

	struct delayed_work monitor_work;
	struct power_supply *fg_psy;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	struct power_supply_desc fg_psy_d;
	struct timeval suspend_time;

	u8 digest[BATTERY_DIGEST_LEN];
	int constant_charge_current_max;
	struct votable *fcc_votable;

	bool charge_done;
	bool charge_full;

	/* workaround for debug or other purpose */
	bool ignore_digest_for_debug;
	bool old_hw;

	int cell1_max;
	int max_charge_current;
	int max_discharge_current;
	int max_temp_cell;
	int min_temp_cell;
	int total_fw_runtime;
	int time_spent_in_lt;
	int time_spent_in_ht;
	int time_spent_in_ot;

	int cell_ov_check;
	/*add fuelgauge index*/
	int fg_index;
};

#define bq_dbg pr_info

static int fg_get_raw_soc(struct bq_fg_chip *bq);
static int fg_read_current(struct bq_fg_chip *bq, int *curr);
static int fg_read_temperature(struct bq_fg_chip *bq);
static int fg_check_full_status(struct bq_fg_chip *bq);

static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		bq_dbg("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}

static int __fg_read_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;
	int i;

	for (i = 0; i < len; i++) {
		ret = i2c_smbus_read_byte_data(client, reg + i);
		if (ret < 0) {
			bq_dbg("i2c read reg 0x%02X faild\n", reg + i);
			return ret;
		}
		buf[i] = ret;
	}

	return ret;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;
	int i = 0;

	for (i = 0; i < len; i++) {
		ret = i2c_smbus_write_byte_data(client, reg + i, buf[i]);
		if (ret < 0) {
			bq_dbg("i2c read reg 0x%02X faild\n", reg + i);
			return ret;
		}
	}

	return ret;
}

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_block(bq->client, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_block(bq->client, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static u8 checksum(u8 *data, u8 len)
{
	u8 i;
	u16 sum = 0;

	for (i = 0; i < len; i++) {
		sum += data[i];
	}

	sum &= 0xFF;

	return 0xFF - sum;
}

static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	bq_dbg("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	bq_dbg("%s\n", strbuf);
}

static int fg_mac_read_block(struct bq_fg_chip *bq, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 cksum_calc, cksum;
	u8 t_buf[40];
	u8 t_len;
	int i;

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(4);

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, 36);
	if (ret < 0)
		return ret;

	fg_print_buf("mac_read_block", t_buf, 36);

	cksum = t_buf[34];
	t_len = t_buf[35];

	cksum_calc = checksum(t_buf, t_len - 2);
	if (cksum_calc != cksum)
		return 1;

	for (i = 0; i < len; i++)
		buf[i] = t_buf[i + 2];

	return 0;
}

static int fg_mac_write_block(struct bq_fg_chip *bq, u16 cmd, u8 *data, u8 len)
{
	int ret;
	u8 cksum;
	u8 t_buf[40];
	int i;

	if (len > 32)
		return -1;

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);
	for (i = 0; i < len; i++)
		t_buf[i+2] = data[i];

	/*write command/addr, data*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, len + 2);
	if (ret < 0)
		return ret;

	fg_print_buf("mac_write_block", t_buf, len + 2);

	cksum = checksum(data, len + 2);
	t_buf[0] = cksum;
	t_buf[1] = len + 4; /*buf length, cmd, CRC and len byte itself*/

	/*write checksum and length*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MAC_CHKSUM], t_buf, 2);

	return ret;
}

static int fg_set_fastcharge_mode(struct bq_fg_chip *bq, bool enable)
{
	u8 data[2];
	int ret;

	data[0] = enable;

	if (enable) {
		ret = fg_mac_write_block(bq, FG_MAC_CMD_FASTCHARGE_EN, data, 2);
		if (ret < 0) {
			bq_dbg("could not write fastcharge = %d\n", ret);
			return ret;
		}
	} else {
		ret = fg_mac_write_block(bq, FG_MAC_CMD_FASTCHARGE_DIS, data, 2);
		if (ret < 0) {
			bq_dbg("could not write fastcharge = %d\n", ret);
			return ret;
		}
	}

	return ret;
}

static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0)
		return ret;
	bq->chip_ok = ret;
	bq->batt_fc = !!(flags & FG_FLAGS_FC);
	bq->batt_fd = !!(flags & FG_FLAGS_FD);
	bq->batt_rca = !!(flags & FG_FLAGS_RCA);
	bq->batt_dsg = !!(flags & FG_FLAGS_DSG);

	return 0;
}

enum manu_macro {
	TERMINATION = 0,
	RECHARGE_VOL,
	MANU_NAME,
	MANU_DATA_LEN,
};

#define TERMINATION_BYTE 6
#define TERMINATION_BASE 30
#define TERMINATION_STEP 5

#define RECHARGE_VOL_BYTE 7
#define RECHARGE_VOL_BASE 4200
#define RECHARGE_VOL_STEP 5

#define MANU_NAME_BYTE 3
#define MANU_NAME_BASE 0x0C
#define MANU_NAME_STEP 1

struct manu_data {
	int byte;
	int base;
	int step;
	int data;
};

struct manu_data manu_info[MANU_DATA_LEN] = {
	{ TERMINATION_BYTE, TERMINATION_BASE, TERMINATION_STEP },
	{ RECHARGE_VOL_BYTE, RECHARGE_VOL_BASE, RECHARGE_VOL_STEP },
	{ MANU_NAME, MANU_NAME_BASE, MANU_NAME_STEP },
};

static int fg_get_manu_info(unsigned char val, int base, int step)
{
	int index = 0;
	int data = 0;

	bq_dbg("val:%d, '0':%d, 'A':%d, 'a':%d\n", val, '0', 'A', 'a');
	if (val > '0' && val < '9')
		index = val - '0';
	if (val > 'A' && val < 'Z')
		index = val - 'A' + 10;
	if (val > 'a' && val < 'z')
		index = val - 'a' + 36;

	data = base + index * step;

	return data;
}

static int fg_get_manufacture_data(struct bq_fg_chip *bq)
{
	u8 t_buf[40];
	int ret;
	int i;
	int byte, base, step;

	for (i = 0; i < RETRY_COUNT; i++) {
		ret = fg_mac_read_block(bq, FG_MAC_CMD_MANU_NAME, t_buf, 32);
		if (ret < 0) {
			bq_dbg("failed to get MANE NAME\n");
			/* for draco p0 and p0.1 */
			if (bq->ignore_digest_for_debug)
				bq->old_hw = true;
			msleep(300);
		} else {
			if (bq->ignore_digest_for_debug)
				bq->old_hw = false;
			break;
		}
	}

	if (strncmp(t_buf, "MI", 2) != 0) {
		bq_dbg("Can not get MI battery data\n");
		manu_info[TERMINATION].data = BQ27Z561_DEFUALT_TERM;
		manu_info[RECHARGE_VOL].data = BQ27Z561_DEFUALT_RECHARGE_VOL;
		return 0;
	}

	for (i = 0; i < MANU_DATA_LEN; i++) {
		byte = manu_info[i].byte;
		base = manu_info[i].base;
		step = manu_info[i].step;
		manu_info[i].data = fg_get_manu_info(t_buf[byte], base, step);
	}

	return 0;
}

static int fg_read_rsoc(struct bq_fg_chip *bq)
{
	int soc, ret;

	if (bq->fake_soc > 0)
		return bq->fake_soc;

	if (bq->skip_reads)
		return bq->last_soc;

	ret = regmap_read(bq->regmap, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_dbg("could not read RSOC, ret = %d\n", ret);
		if (bq->last_rsoc >= 0)
			return bq->last_rsoc;
		else
			soc = BQ_I2C_FAILED_SOC;
	}

	bq->last_rsoc = soc;

	return soc;
}

static int i2c_error_cnt[FG_MAX_INDEX];
static int fg_read_temperature(struct bq_fg_chip *bq)
{
	int ret;
	u16 temp = 0;
	static int last_temp[FG_MAX_INDEX];

	if (bq->fake_temp > 0)
		return bq->fake_temp;

	if (bq->skip_reads)
		return last_temp[bq->fg_index];

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		bq_dbg("could not read temperature, ret = %d\n", ret);
		if (i2c_error_cnt[bq->fg_index]++ >= 3) {
			i2c_error_cnt[bq->fg_index] = 3;
			return BQ_I2C_FAILED_TEMP_HIGH;
		}
		return BQ_I2C_FAILED_TEMP;
	}
	i2c_error_cnt[bq->fg_index] = 0;
	last_temp[bq->fg_index] = temp - 2730;

	return temp - 2730;
}

static int fg_read_volt(struct bq_fg_chip *bq)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_dbg("could not read voltage, ret = %d\n", ret);
		return ret;
	}

	return volt;
}

static int fg_read_current(struct bq_fg_chip *bq, int *curr)
{
	int ret;
	s16 avg_curr = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CN], (u16 *)&avg_curr);
	if (ret < 0) {
		bq_dbg("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);

	return ret;
}

static int fg_read_avg_current(struct bq_fg_chip *bq, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_AI], &avg_curr);
	if (ret < 0) {
		bq_dbg("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);

	return ret;
}

static int fg_read_fcc(struct bq_fg_chip *bq)
{
	int ret;
	u16 fcc;

	if (bq->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		bq_dbg("FCC command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FCC], &fcc);
	if (ret < 0) {
		bq_dbg("could not read FCC, ret=%d\n", ret);
		fcc = bq->last_fcc;
		return fcc;
	}

	bq->last_fcc = fcc;

	return fcc;
}

static int fg_read_rm(struct bq_fg_chip *bq)
{
	int ret;
	u16 rm;

	if (bq->regs[BQ_FG_REG_RM] == INVALID_REG_ADDR) {
		bq_dbg("RemainingCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_RM], &rm);

	if (ret < 0) {
		bq_dbg("could not read DC, ret=%d\n", ret);
		rm = bq->last_rm;
		return rm;
	}

	bq->last_rm = rm;

	return rm;
}

static int fg_read_soh(struct bq_fg_chip *bq)
{
	int ret;
	u16 soh;

	if (bq->regs[BQ_FG_REG_SOH] == INVALID_REG_ADDR) {
		bq_dbg("SOH command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOH], &soh);
	if (ret < 0) {
		bq_dbg("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return soh;
}

static int fg_read_cyclecount(struct bq_fg_chip *bq)
{
	int ret;
	u16 cc;

	if (bq->regs[BQ_FG_REG_CC] == INVALID_REG_ADDR) {
		bq_dbg("Cycle Count not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CC], &cc);

	if (ret < 0) {
		bq_dbg("could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	return cc;
}

static int fg_read_tte(struct bq_fg_chip *bq)
{
	int ret;
	u16 tte;

	if (bq->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		bq_dbg("Time To Empty not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTE], &tte);

	if (ret < 0) {
		bq_dbg("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return tte;
}

static int fg_read_ttf(struct bq_fg_chip *bq)
{
	int ret;
	u16 ttf;

	if (bq->regs[BQ_FG_REG_TTF] == INVALID_REG_ADDR) {
		bq_dbg("Time To Empty not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTF], &ttf);

	if (ret < 0) {
		bq_dbg("could not read Time To Full, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return ttf;
}

static int fg_read_charging_current(struct bq_fg_chip *bq)
{
	int ret;
	u16 cc;

	if (bq->regs[BQ_FG_REG_CHG_CUR] == INVALID_REG_ADDR) {
		bq_dbg(" not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CHG_CUR], &cc);

	if (ret < 0) {
		bq_dbg("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return cc;
}

static int fg_read_charging_voltage(struct bq_fg_chip *bq)
{
	int ret;
	u16 cv;

	if (bq->regs[BQ_FG_REG_CHG_VOL] == INVALID_REG_ADDR) {
		bq_dbg(" not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CHG_VOL], &cv);

	if (ret < 0) {
		bq_dbg("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return cv;
}

static int fg_get_batt_status(struct bq_fg_chip *bq)
{
	fg_read_status(bq);

	if (bq->batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	else if (bq->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (bq->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int fg_get_batt_capacity_level(struct bq_fg_chip *bq)
{
	if (bq->batt_fc) {
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	} else if (bq->shutdown_soc) {
		bq_dbg("soc0 CAPACITY_LEVEL_CRITICAL");
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else if (bq->batt_rca && bq->batt_soc <= 15) {
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	} else if (bq->batt_fd && bq->batt_soc <= 15) {
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	} else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
}

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CAPACITY_RAW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_DEBUG_BATTERY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_SOH,
};

static int fg_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (bq->old_hw) {
			val->strval = "unknown";
			break;
		}
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (bq->fake_volt != -EINVAL) {
			val->intval = bq->fake_volt;
			break;
		}
		if (bq->old_hw) {
			val->intval = 3700 * 1000;
			break;
		}
		ret = fg_read_volt(bq);
		if (ret >= 0)
			bq->batt_volt = ret;
		val->intval = bq->batt_volt * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		if (bq->fake_volt != -EINVAL) {
			val->intval = bq->fake_volt;
			break;
		}
		if (bq->old_hw) {
			val->intval = 3700 * 1000;
			break;
		}
		ret = fg_read_volt(bq);
		if (ret >= 0)
			bq->batt_volt = ret;
		val->intval = bq->batt_volt * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (bq->old_hw) {
			val->intval = -500;
			break;
		}
		fg_read_current(bq, &bq->batt_curr);
		val->intval = bq->batt_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (bq->old_hw) {
			val->intval = -500;
			break;
		}
		fg_read_avg_current(bq, &bq->batt_curr);
		val->intval = bq->batt_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		ret = fg_read_rsoc(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_soc = ret;
		val->intval = bq->batt_soc;
		mutex_unlock(&bq->data_lock);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
		/*
		 * to be compatible with other fuel gauge
		 * for google_battery supporting
		 */
		val->intval = bq->raw_soc;
#if IS_ENABLED(CONFIG_GOOGLE_BMS)
		val->intval *= 255;
		val->intval /= 100;
#endif
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}
		if (bq->old_hw) {
			val->intval = 250;
			break;
		}
		val->intval = bq->batt_temp;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		if (bq->old_hw) {
			val->intval = bq->batt_ttf;
			break;
		}
		ret = fg_read_ttf(bq);
		if (ret >= 0)
			bq->batt_ttf = ret;

		val->intval = bq->batt_ttf;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		if (bq->old_hw) {
			val->intval = bq->batt_tte;
			break;
		}
		ret = fg_read_tte(bq);
		if (ret >= 0)
			bq->batt_tte = ret;

		val->intval = bq->batt_tte;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (bq->old_hw) {
			val->intval = 4050000;
			break;
		}
		val->intval = bq->batt_fcc * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq->batt_dc;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (bq->old_hw) {
			val->intval = 4450000;
			break;
		}
		val->intval = fg_read_charging_voltage(bq);
		bq_dbg("fg_read_gauge_voltage_max: %d\n", val->intval);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (bq->old_hw) {
			val->intval = 1;
			break;
		}
		ret = fg_read_cyclecount(bq);
		if (ret >= 0)
			bq->batt_cyclecnt = ret;
		val->intval = bq->batt_cyclecnt;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = bq->batt_resistance;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = 100000;
		break;
	case POWER_SUPPLY_PROP_DEBUG_BATTERY:
		if (bq->old_hw)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (bq->old_hw) {
			val->intval = 8000000;
			break;
		}
		val->intval = fg_read_charging_current(bq);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (bq->constant_charge_current_max != 0)
			val->intval = bq->constant_charge_current_max;
		else {
			val->intval = fg_read_charging_current(bq);
			val->intval *= 1000;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		if (bq->old_hw) {
			val->intval = 4050000;
			break;
		}
		val->intval = bq->batt_rm * 1000;
		break;
	case POWER_SUPPLY_PROP_SOH:
		bq->soh = fg_read_soh(bq);
		val->intval = bq->soh;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!bq->batt_psy) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			ret = power_supply_get_property(bq->batt_psy,
						       POWER_SUPPLY_PROP_STATUS,
						       val);
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bq->batt_psy) {
			ret = power_supply_get_property(bq->batt_psy,
					       POWER_SUPPLY_PROP_HEALTH, val);
		} else {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	return 0;
}

static int fg_set_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   const union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(bq->fg_psy);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		bq->constant_charge_current_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		bq->fake_volt = val->intval;
		break;
	default:
		break;
	}

	return 0;
}

static int fg_prop_is_writeable(struct power_supply *psy,
				enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static int fg_psy_register(struct bq_fg_chip *bq)
{
	struct power_supply_config fg_psy_cfg = {};

	if (bq->chip == BQ27Z561_MASTER)
		bq->fg_psy_d.name = "bms_master";
	else if (bq->chip == BQ27Z561_SLAVE)
		bq->fg_psy_d.name = "bms_slave";
	else
		bq->fg_psy_d.name = "bms";
	bq->fg_psy_d.type = POWER_SUPPLY_TYPE_BMS;
	bq->fg_psy_d.properties = fg_props;
	bq->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	bq->fg_psy_d.get_property = fg_get_property;
	bq->fg_psy_d.set_property = fg_set_property;
	bq->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = bq;
	fg_psy_cfg.num_supplicants = 0;
	bq->fg_psy =
		devm_power_supply_register(bq->dev, &bq->fg_psy_d, &fg_psy_cfg);
	if (IS_ERR(bq->fg_psy)) {
		bq_dbg("Failed to register fg_psy");
		return PTR_ERR(bq->fg_psy);
	}

	return 0;
}

static void fg_psy_unregister(struct bq_fg_chip *bq)
{
	power_supply_unregister(bq->fg_psy);
}

static int fg_get_lifetime_data(struct bq_fg_chip *bq)
{
	int ret;
	u8 t_buf[40];

	memset(t_buf, 0, sizeof(t_buf));

	ret = fg_mac_read_block(bq, FG_MAC_CMD_LIFETIME1, t_buf, 32);
	if (ret < 0)
		return ret;

	bq->cell1_max = (t_buf[1] << 8) | t_buf[0];
	bq->max_charge_current = (t_buf[3] << 8) | t_buf[2];
	bq->max_discharge_current =
		(signed short int)((t_buf[5] << 8) | t_buf[4]);
	bq->max_temp_cell = t_buf[6];
	bq->min_temp_cell = t_buf[7];

	memset(t_buf, 0, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_LIFETIME3, t_buf, 32);
	if (ret < 0)
		return ret;

	bq->total_fw_runtime = (t_buf[1] << 8) | t_buf[0];
	bq->time_spent_in_lt = (t_buf[5] << 8) | t_buf[4];
	bq->time_spent_in_ht = (t_buf[13] << 8) | t_buf[12];
	bq->time_spent_in_ot = (t_buf[15] << 8) | t_buf[14];

	return ret;
}

static void fg_update_status(struct bq_fg_chip *bq)
{
	static int last_st[FG_MAX_INDEX];
	static int last_soc[FG_MAX_INDEX];
	static int last_temp[FG_MAX_INDEX];
	mutex_lock(&bq->data_lock);

	bq->batt_soc = fg_read_rsoc(bq);
	bq->batt_volt = fg_read_volt(bq);
	fg_read_current(bq, &bq->batt_curr);
	bq->batt_temp = fg_read_temperature(bq);
	bq->batt_st = fg_get_batt_status(bq);
	bq->batt_rm = fg_read_rm(bq);
	bq->batt_fcc = fg_read_fcc(bq);
	bq->raw_soc = fg_get_raw_soc(bq);
	fg_get_lifetime_data(bq);
	bq->soh = fg_read_soh(bq);
	bq->charging_current = fg_read_charging_current(bq);
	bq->charging_voltage = fg_read_charging_voltage(bq);
	bq->batt_cyclecnt = fg_read_cyclecount(bq);
	bq->batt_tte = fg_read_tte(bq);
	bq->batt_ttf = fg_read_ttf(bq);
	mutex_unlock(&bq->data_lock);

	if ((last_soc[bq->fg_index] != bq->batt_soc) ||
	    (last_temp[bq->fg_index] != bq->batt_temp) ||
	    (last_st[bq->fg_index] != bq->batt_st)) {
		if (bq->fg_psy)
			power_supply_changed(bq->fg_psy);
	}
	if (bq->batt_st == POWER_SUPPLY_STATUS_DISCHARGING)
		bq->cell_ov_check = 0;

	last_soc[bq->fg_index] = bq->batt_soc;
	last_temp[bq->fg_index] = bq->batt_temp;
	last_st[bq->fg_index] = bq->batt_st;
}

static int fg_get_raw_soc(struct bq_fg_chip *bq)
{
	int rm, fcc;
	int raw_soc;

	rm = fg_read_rm(bq);
	fcc = fg_read_fcc(bq);

	//raw_soc = DIV_ROUND_CLOSEST(rm * 10000 / fcc, 100);
	raw_soc = rm * 10000 / fcc;

	return raw_soc;
}

static int fg_update_charge_full(struct bq_fg_chip *bq)
{
	int rc;
	union power_supply_propval prop = {
		0,
	};

	if (!bq->batt_psy) {
#if IS_ENABLED(CONFIG_GOOGLE_BMS)
		bq->batt_psy = power_supply_get_by_name("sm8150_bms");
#else
		bq->batt_psy = power_supply_get_by_name("battery");
#endif
		if (!bq->batt_psy) {
			return 0;
		}
	}

	rc = power_supply_get_property(bq->batt_psy,
				       POWER_SUPPLY_PROP_CHARGE_DONE, &prop);
	bq->charge_done = prop.intval;

	if (bq->charge_done && !bq->charge_full) {
		if (bq->raw_soc >= BQ_REPORT_FULL_SOC) {
			bq_dbg("Setting charge_full to true\n");
			bq->charge_full = true;
			bq->cell_ov_check = 0;
		} else {
			bq_dbg("charging is done raw soc:%d\n", bq->raw_soc);
		}
	} else if (bq->raw_soc <= BQ_CHARGE_FULL_SOC && !bq->charge_done &&
		   bq->charge_full) {
		if (bq->charge_done)
			goto out;

		bq->charge_full = false;
	}

	if ((bq->raw_soc <= BQ_RECHARGE_SOC) && bq->charge_done) {
		prop.intval = true;
	}

out:
	return 0;
}

static int fg_check_full_status(struct bq_fg_chip *bq)
{
	union power_supply_propval prop = {
		0,
	};
	int rc;
	int interval = MONITOR_WORK_10S;
	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy) {
			return interval;
		}
	}
	rc = power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_PRESENT,
				       &prop);
	if (rc < 0 || !prop.intval)
		return interval;

	/*NON-FFC charging,status = FULL,fg_slave reports 100 in time */
	if (bq->raw_soc >= BQ_REPORT_FULL_SOC) {
		if (bq->batt_soc < 100)
			interval = MONITOR_WORK_5S;
		else if (bq->batt_soc == 100)
			interval = MONITOR_WORK_10S;
	}

	return interval;
}

static void fg_monitor_workfunc(struct work_struct *work)
{
	struct bq_fg_chip *bq =
		container_of(work, struct bq_fg_chip, monitor_work.work);
	int period = MONITOR_WORK_10S;
	if (!bq->old_hw) {
		fg_update_status(bq);
		period = fg_check_full_status(bq);
		fg_update_charge_full(bq);
	}

	schedule_delayed_work(&bq->monitor_work, period * HZ);
}

static int bq_parse_dt(struct bq_fg_chip *bq)
{
	struct device_node *node = bq->dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "bq,charge-full-design", &bq->batt_dc);
	if (ret < 0) {
		bq_dbg("failed to get bq,charge-full-designe\n");
		bq->batt_dc = DEFUALT_FULL_DESIGN;
		return ret;
	}

	bq->ignore_digest_for_debug =
		of_property_read_bool(node, "bq,ignore-digest-debug");

	return 0;
}

static struct regmap_config i2c_bq27z561_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static int fg_check_device(struct bq_fg_chip *bq)
{
	u8 data[32];
	int ret = 0, i = 0;

	for (i = 0; i <= 3; i++) {
		ret = fg_mac_read_block(bq, 0x004A, data, 32);
		if (ret == 1)
			bq_dbg("failed to get FG_MAC_CMD_DEVICE_NAME with ret:%d and retry!\n",
			       ret);
		else if (ret < 0)
			bq_dbg("failed to get FG_MAC_CMD_DEVICE_NAME with ret:%d!\n",
			       ret);
		else
			break;
	}

	for (i = 0; i < 8; i++) {
		if (data[i] >= 'A' && data[i] <= 'Z')
			data[i] += 32;
	}

	bq_dbg("parse device_name: %s\n", data);
	if (!strncmp(data, "bq27z561", 8)) {
		strcpy(bq->model_name, "bq27z561");
	} else if (!strncmp(data, "bq28z610", 8)) {
		strcpy(bq->model_name, "bq28z610");
	} else if (!strncmp(data, "nfg1000a", 8)) {
		strcpy(bq->model_name, "nfg1000a");
	} else if (!strncmp(data, "nfg1000b", 8)) {
		strcpy(bq->model_name, "nfg1000b");
	} else {
		strcpy(bq->model_name, "UNKNOWN");
	}

	return ret;
}

static int bq_fg_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct bq_fg_chip *bq;
	u8 *regs;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_DMA);

	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;
	bq->model_name = (char *)id->name;

	bq->batt_soc = -ENODATA;
	bq->batt_fcc = -ENODATA;
	bq->batt_rm = -ENODATA;
	bq->batt_dc = -ENODATA;
	bq->batt_volt = -ENODATA;
	bq->batt_temp = -ENODATA;
	bq->batt_curr = -ENODATA;
	bq->batt_cyclecnt = -ENODATA;
	bq->batt_tte = -ENODATA;
	bq->batt_ttf = -ENODATA;
	bq->raw_soc = -ENODATA;
	bq->last_soc = -EINVAL;
	bq->cell_ov_check = 0;

	bq->fake_soc = -EINVAL;
	bq->fake_temp = -EINVAL;
	bq->fake_volt = -EINVAL;
	bq->fake_chip_ok = -EINVAL;
	bq->batt_cyclecnt = -EINVAL;
	bq->soh = -EINVAL;
	bq->charging_current = -EINVAL;
	bq->chip_ok = -EINVAL;
	bq->charging_voltage = -EINVAL;
	bq->avg_current = -EINVAL;
	bq->plugout_update_count = 0;
	bq->shutdown_soc = false;

	if (bq->chip == BQ27Z561 || bq->chip == BQ27Z561_MASTER ||
	    bq->chip == BQ27Z561_SLAVE) {
		regs = bq27z561_regs;
	} else {
		bq_dbg("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq27z561_regs;
	}

	if (bq->chip == BQ27Z561_SLAVE) {
		bq->fg_index = 1;
	} else {
		bq->fg_index = 0;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	fg_check_device(bq);

	i2c_set_clientdata(client, bq);

	bq_parse_dt(bq);

	bq->regmap = devm_regmap_init_i2c(client, &i2c_bq27z561_regmap_config);
	if (!bq->regmap)
		return -ENODEV;

	fg_get_manufacture_data(bq);
	fg_set_fastcharge_mode(bq, true);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	device_init_wakeup(bq->dev, 1);

	fg_psy_register(bq);
	fg_update_status(bq);

	INIT_DELAYED_WORK(&bq->monitor_work, fg_monitor_workfunc);
	schedule_delayed_work(&bq->monitor_work, 10 * HZ);

	bq_dbg("bq fuel gauge probe successfully, %s\n", bq->model_name);

	return 0;
}

static int bq_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);
	bq->skip_reads = true;
	do_gettimeofday(&bq->suspend_time);

	return 0;
}
#define BQ_RESUME_UPDATE_TIME 600

static int bq_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	bq->skip_reads = false;

	schedule_delayed_work(&bq->monitor_work, HZ);

	return 0;
}

static int bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	fg_psy_unregister(bq);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);

	return 0;
}

static void bq_fg_shutdown(struct i2c_client *client)
{
	bq_dbg("bq fuel gauge driver shutdown!\n");
}

static struct of_device_id bq_fg_match_table[] = {
	{
		.compatible = "ti,bq27z561_master",
	},
	{
		.compatible = "ti,bq27z561_slave",
	},
	{
		.compatible = "ti,bq27z561",
	},
	{
		.compatible = "ti,bq28z610",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static const struct i2c_device_id bq_fg_id[] = {
	{ "bq27z561_master", BQ27Z561_MASTER },
	{ "bq27z561_slave", BQ27Z561_SLAVE },
	{ "bq27z561", BQ27Z561 },
	{ "bq28z610", BQ28Z610 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume = bq_fg_resume,
	.suspend = bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver	= {
		.name   = "bq_fg",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm     = &bq_fg_pm_ops,
	},
	.id_table       = bq_fg_id,
	.probe          = bq_fg_probe,
	.remove		= bq_fg_remove,
	.shutdown	= bq_fg_shutdown,

};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ27Z561 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
