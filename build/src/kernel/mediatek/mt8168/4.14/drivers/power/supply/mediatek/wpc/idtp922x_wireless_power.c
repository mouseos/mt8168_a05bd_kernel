/************************************************************
 *
 * file: idtp922x_wireless_power.c
 *
 * Description: P922x Wireless Power Charger Driver
 *
 *------------------------------------------------------------
 *
 * Copyright (c) 2018, Integrated Device Technology Co., Ltd.
 * Copyright (C) 2019 Amazon.com Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *************************************************************/

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/switch.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_charger.h>
#include <mt-plat/mtk_boot.h>
#include "mtk_charger_intf.h"
#include "mtk_switch_charging.h"
#include "idtp922x_wireless_power.h"

static void p922x_fast_charge_enable(struct p922x_dev *chip, bool en);
static int p922x_enable_charge_flow(struct charger_device *chg_dev, bool en);
static bool p922x_get_pg_irq_status(struct p922x_dev *chip);

static int p922x_read(void *data, u16 reg, u8 *val)
{
	unsigned int temp = 0;
	int rc = 0;
	struct p922x_dev *di = (struct p922x_dev *)data;

	rc = regmap_read(di->regmap, reg, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

static int p922x_write(void *data, u16 reg, u8 val)
{
	int rc = 0;
	struct p922x_dev *di = (struct p922x_dev *)data;

	rc = regmap_write(di->regmap, reg, val);
	if (rc < 0) {
		dev_err(di->dev,
				"%s: error: %d, reg: %04x, val: %02x\n",
				__func__, rc, reg, val);
	}
	return rc;
}

static int p922x_read_buffer(void *data, u16 reg, u8 *buf, u32 size)
{
	struct p922x_dev *di = (struct p922x_dev *)data;

	return regmap_bulk_read(di->regmap, reg, buf, size);
}

static int p922x_write_buffer(void *data, u16 reg, u8 *buf, u32 size)
{
	int rc = 0;
	u8 val = 0;
	struct p922x_dev *di = (struct p922x_dev *)data;

	while (size--) {
		val = *buf;
		rc = di->bus.write(di, reg, val);
		if (rc < 0) {
			dev_err(di->dev,
				"%s: error: %d, reg: %04x, val: %02x\n",
				__func__, rc, reg, val);
			return rc;
		}
		reg++;
		buf++;
	}

	return rc;
}

static u16 p922x_get_vout(struct p922x_dev *chip)
{
	u8 value = 0;
	u16 vout = 0;

	chip->bus.read(chip, REG_VOUT_SET, &value);
	/* vout = value * 0.1V + 3.5V */
	vout = value * 100 + 3500;

	return vout;
}

static int p922x_set_vout(struct p922x_dev *chip, int vout)
{
	int ret = 0;

	if ((vout >= SET_VOUT_MIN) && (vout <= SET_VOUT_MAX)) {
		dev_info(chip->dev, "%s: Set vout: %dmv\n", __func__, vout);
		ret = chip->bus.write(chip, REG_VOUT_SET, vout/100 - 35);
	} else {
		dev_err(chip->dev, "%s: Set vout parameter error!\n", __func__);
		ret = -EINVAL;
	}

	if (ret)
		dev_err(chip->dev, "%s: failed!\n", __func__);

	return ret;
}

static u16 p922x_get_iout_adc(struct p922x_dev *chip)
{
	u8 buf[2] = {0};
	u16 iout = 0;

	chip->bus.read_buf(chip, REG_RX_LOUT, buf, 2);
	iout = buf[0]|(buf[1]<<8);
	dev_dbg(chip->dev, "%s: iout:%04x\n", __func__, iout);

	return iout;
}

static u16 p922x_get_vout_adc(struct p922x_dev *chip)
{
	u8 buf[2] = {0};
	u32 vout = 0;

	chip->bus.read_buf(chip, REG_ADC_VOUT, buf, 2);
	/* vout = val/4095*6*2.1 */
	vout = (buf[0] | (buf[1] << 8)) * 6 * 21 * 1000 / 40950;
	dev_dbg(chip->dev, "%s: vout:%d MV\n", __func__, vout);

	return vout;
}

static u16 p922x_get_vrect_adc(struct p922x_dev *chip)
{
	u8 buf[2] = {0};
	u32 vrect = 0;

	chip->bus.read_buf(chip, REG_ADC_VRECT, buf, 2);
	/* vrect = val/4095*10*2.1 , val = REG_ADC_VRECT bit0-11 */
	buf[1] &= 0xf;
	vrect = (buf[0] | (buf[1] << 8)) * 10 * 21 * 1000 / 40950;
	dev_dbg(chip->dev, "%s: vrect:%d MV\n", __func__, vrect);

	return vrect;
}

static u16 p922x_get_prx(struct p922x_dev *chip)
{
	u8 buf[2] = {0};
	u16 power_rx = 0;

	chip->bus.read_buf(chip, REG_PRX, buf, 2);
	power_rx = buf[0] | (buf[1] << 8);
	dev_dbg(chip->dev, "%s: power_rx:%d MW\n", __func__, power_rx);

	return power_rx;
}

static int p922x_get_temp_adc(struct p922x_dev *chip)
{
	u8 buf[2] = {0};
	int ret = 0;
	/* The default temp value is -273°C */
	int temp = P922X_DIE_TEMP_DEFAULT;

	if (p922x_get_pg_irq_status(chip))
		ret = chip->bus.read_buf(chip, REG_ADC_TEMP, buf, 2);
	else
		goto out;

	if (ret) {
		dev_err(chip->dev, "%s: Failed to read temp: %d\n",
			__func__, ret);
		goto out;
	}

	/* temp = (val-1350)*83/444-273 */
	temp = ((buf[0] | (buf[1] << 8)) - 1350) * 83 / 444 - 273;

out:
	dev_dbg(chip->dev, "%s: temp:%d degrees C\n", __func__, temp);

	return temp;
}

static u8 p922x_get_tx_signal_strength(struct p922x_dev *chip)
{
	int ret = 0;
	u8 ss = 0;

	ret = chip->bus.read(chip, REG_SS, &ss);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read tx signal strength: %d\n",
			__func__, ret);
		goto out;
	}

	dev_info(chip->dev, "%s: tx signal strength:%d\n", __func__, ss);
out:

	return ss;
}

static int p922x_metrics_tx_signal_strength(struct p922x_dev *chip)
{
	u8 ss = 0;

	ss = p922x_get_tx_signal_strength(chip);

	if (ss)
		p922x_metrics_log("wpc", "wpc:def:tx_signal_strength=%d;CT;1:NR", ss);

	return 0;
}

static void p922x_power_switch(struct p922x_dev *chip, ushort mv)
{
	chip->bus.write_buf(chip, REG_FC_VOLTAGE, (u8 *)&mv, 2);
	chip->bus.write(chip, REG_COMMAND, VSWITCH);
	dev_info(chip->dev, "%s: %dmv\n", __func__, mv);
}

static ssize_t p922x_fwver(struct p922x_dev *chip)
{
	u8 id[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	u8 ver[4] = { 0xff, 0xff, 0xff, 0xff };

	chip->bus.read_buf(chip, REG_CHIP_ID, id, 8);
	chip->bus.read_buf(chip, REG_CHIP_REV, ver, 4);

	pr_info("%s: ChipID: %04x\nFWVer:%02x.%02x.%02x.%02x\n",
			__func__, id[4] | (id[0] << 8), ver[3], ver[2], ver[1], ver[0]);

	return 0;
}

static ssize_t p922x_version_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 id[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	u8 ver[4] = { 0xff, 0xff, 0xff, 0xff };
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	chip->bus.read_buf(chip, REG_CHIP_ID, id, 8);
	chip->bus.read_buf(chip, REG_CHIP_REV, ver, 4);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%02x.%02x.%02x.%02x\n",
			ver[3], ver[2], ver[1], ver[0]);
}

static ssize_t p922x_id_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 id[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	chip->bus.read_buf(chip, REG_CHIP_ID, id, 8);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%04x\n", id[4] | (id[0] << 8));
}

static ssize_t p922x_id_authen_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->tx_id_authen_status);
}

static ssize_t p922x_dev_authen_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", chip->tx_dev_authen_status);
}

static ssize_t p922x_is_hv_adapter_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	return sprintf(buffer, "%d\n", chip->is_hv_adapter);
}

static ssize_t p922x_tx_adapter_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	return sprintf(buffer, "%d\n", chip->tx_adapter_type);
}

static ssize_t p922x_power_rx_mw_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int power_rx_mw = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	power_rx_mw = p922x_get_prx(chip);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%d\n", power_rx_mw);
}


static ssize_t p922x_vout_adc_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int vout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	vout = p922x_get_vout_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%d\n", vout);
}

static ssize_t p922x_vrect_adc_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int vrect = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	vrect = p922x_get_vrect_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%d\n", vrect);
}

static ssize_t p922x_vout_set_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int vout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	vout = p922x_get_vout(chip);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%d\n", vout);
}

static ssize_t p922x_vout_set_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret = 0, vout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	ret = kstrtoint(buf, 10, &vout);
	if (ret < 0) {
		dev_err(chip->dev, "%s: kstrtoint failed! ret:%d\n", __func__, ret);
		goto out;
	}
	p922x_set_vout(chip, vout);

out:
	mutex_unlock(&chip->sys_lock);
	return count;
}

static ssize_t p922x_iout_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 iout = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	iout = p922x_get_iout_adc(chip);
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%d\n", iout);
}

static ssize_t p922x_power_switch_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);
	enum charger_type chr_type;
	int wpc_power = 0;

	mutex_lock(&chip->sys_lock);
	chr_type = mt_get_charger_type();
	if (chr_type == WIRELESS_10W_CHARGER)
		wpc_power = 10;
	if (chr_type == WIRELESS_5W_CHARGER)
		wpc_power = 5;
	if (chr_type == WIRELESS_DEFAULT_CHARGER)
		wpc_power = 1;
	mutex_unlock(&chip->sys_lock);

	return sprintf(buffer, "%d\n", wpc_power);
}

static ssize_t p922x_power_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret = 0, power = 0;
	bool fast_switch = false;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->irq_lock);
	ret = kstrtoint(buf, 10, &power);
	if (ret < 0) {
		dev_err(chip->dev, "%s: kstrtoint failed! ret:%d\n",
			__func__, ret);
		goto out;
	}

	if (power == 5)
		fast_switch = false;
	else if (power == 10)
		fast_switch = true;
	else {
		dev_err(chip->dev, "%s: power_switch[%d] not 5 or 10\n",
			__func__, power);
		goto out;
	}

	chip->force_switch = true;
	if (chip->tx_authen_complete != true) {
		dev_info(chip->dev, "%s: tx authen not completed\n", __func__);
		goto out;
	}
	p922x_fast_charge_enable(chip, fast_switch);

out:
	mutex_unlock(&chip->irq_lock);
	return count;
}

static ssize_t p922x_regs_dump_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 reg = 0x00;
	u8 val = 0;
	ssize_t len = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	mutex_lock(&chip->sys_lock);
	for (reg = 0x00; reg <= REG_DUMP_MAX; reg++) {
		chip->bus.read(chip, reg, &val);
		len += snprintf(buffer+len, PAGE_SIZE-len,
			"reg:0x%02x=0x%02x\n", reg, val);
	}
	mutex_unlock(&chip->sys_lock);

	return len;
}

static ssize_t p922x_reg_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 buf[REG_DUMP_MAX] = {0};
	int ret = 0;
	int i = 0;
	int len = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	if (!p922x_get_pg_irq_status(chip))
		return -ENODEV;
	ret = chip->bus.read_buf(chip, chip->reg.addr, buf, chip->reg.size);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read reg: %d\n",
			__func__,  ret);
		return ret;
	}
	for (i = 0; i < chip->reg.size; i++)
		len += scnprintf(buffer + len, PAGE_SIZE - len, "addr:0x%04x = 0x%02x\n",
			chip->reg.addr + i, buf[i]);

	return len;
}

static ssize_t p922x_reg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret = 0;
	int i = 0;
	u8 regs_data[REG_DUMP_MAX] = {0};
	char *tmp_data = NULL;
	char *reg_data = NULL;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	if ((chip->reg.size + chip->reg.addr) > REG_DUMP_MAX ||
		!chip->reg.size) {
		dev_err(chip->dev, "%s: invalid parameters\n", __func__);
		return -EINVAL;
	}
	if (!p922x_get_pg_irq_status(chip))
		return -ENODEV;
	tmp_data = kzalloc(strlen(buf) + 1, GFP_KERNEL);
	if (!tmp_data)
		return -ENOMEM;
	strncpy(tmp_data, buf, strlen(buf));
	while (tmp_data && i < chip->reg.size) {
		reg_data = strsep(&tmp_data, " ");
		if (*reg_data) {
			ret = kstrtou8(reg_data, 0, &regs_data[i]);
			if (ret)
				break;
			i++;
		}
	}

	if (i != chip->reg.size || ret) {
		ret = -EINVAL;
		goto out;
	}

	ret = chip->bus.write_buf(chip, chip->reg.addr, regs_data, chip->reg.size);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to write reg: %d\n", __func__,  ret);
		goto out;
	}
	ret = count;
out:
	kfree(tmp_data);
	return ret;
}

static ssize_t p922x_addr_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "addr:0x%04x size:%d\n",
			chip->reg.addr, chip->reg.size);
}

static ssize_t p922x_addr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	unsigned int data[2] = {0};
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	if (sscanf(buf, "%x %x", &data[0], &data[1]) != 2)
		return -EINVAL;
	chip->reg.addr = data[0];
	chip->reg.size = data[1];

	return count;
}

static ssize_t p922x_over_reason_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	return scnprintf(buffer, PAGE_SIZE, "%u\n", chip->over_reason);
}

static ssize_t p922x_fod_regs_dump_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u16 reg = 0x00;
	u8 val = 0;
	ssize_t len = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	for (reg = REG_FOD_COEF_ADDR; reg <= REG_FOD_DUMP_MAX; reg++) {
		chip->bus.read(chip, reg, &val);
		len += scnprintf(buffer+len, PAGE_SIZE-len,
			"fod reg:0x%02x=0x%02x\n", reg, val);
	}

	return len;
}

static ssize_t p922x_dock_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int state;

	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	state = switch_get_state(&chip->dock_state)
		== TYPE_UNDOCKED ? 0 : 1;
	return scnprintf(buffer, PAGE_SIZE, "%d\n", state);
}

static ssize_t p922x_temp_adc_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	int temp;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	temp = p922x_get_temp_adc(chip);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", temp);
}

static ssize_t p922x_tx_signal_strength_show(struct device *dev,
		struct device_attribute *attr,
		char *buffer)
{
	u8 ss;
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	ss = p922x_get_tx_signal_strength(chip);

	return scnprintf(buffer, PAGE_SIZE, "%d\n", ss);
}

static DEVICE_ATTR(version, 0444, p922x_version_show, NULL);
static DEVICE_ATTR(id, 0444, p922x_id_show, NULL);
static DEVICE_ATTR(id_authen_status, 0444, p922x_id_authen_status_show, NULL);
static DEVICE_ATTR(dev_authen_status, 0444, p922x_dev_authen_status_show, NULL);
static DEVICE_ATTR(is_hv_adapter, 0444, p922x_is_hv_adapter_show, NULL);
static DEVICE_ATTR(tx_adapter_type, 0444, p922x_tx_adapter_type_show, NULL);
static DEVICE_ATTR(power_rx_mw, 0444, p922x_power_rx_mw_show, NULL);
static DEVICE_ATTR(vout_adc, 0444, p922x_vout_adc_show, NULL);
static DEVICE_ATTR(vrect_adc, 0444, p922x_vrect_adc_show, NULL);
static DEVICE_ATTR(vout_set, 0644, p922x_vout_set_show,
	p922x_vout_set_store);
static DEVICE_ATTR(iout_adc, 0444, p922x_iout_show, NULL);
static DEVICE_ATTR(power_switch, 0644, p922x_power_switch_show,
	p922x_power_switch_store);
static DEVICE_ATTR(registers_dump, 0444, p922x_regs_dump_show, NULL);
static DEVICE_ATTR(reg, 0644, p922x_reg_show, p922x_reg_store);
static DEVICE_ATTR(addr, 0644, p922x_addr_show, p922x_addr_store);
static DEVICE_ATTR(over_reason, 0444, p922x_over_reason_show, NULL);
static DEVICE_ATTR(fod_regs_dump, 0444, p922x_fod_regs_dump_show, NULL);
static DEVICE_ATTR(dock_state, 0444, p922x_dock_state_show, NULL);
static DEVICE_ATTR(temp_adc, 0444, p922x_temp_adc_show, NULL);
static DEVICE_ATTR(tx_signal_strength, 0444, p922x_tx_signal_strength_show, NULL);

static struct attribute *p922x_sysfs_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_id.attr,
	&dev_attr_id_authen_status.attr,
	&dev_attr_dev_authen_status.attr,
	&dev_attr_is_hv_adapter.attr,
	&dev_attr_tx_adapter_type.attr,
	&dev_attr_power_rx_mw.attr,
	&dev_attr_vout_adc.attr,
	&dev_attr_vrect_adc.attr,
	&dev_attr_vout_set.attr,
	&dev_attr_iout_adc.attr,
	&dev_attr_power_switch.attr,
	&dev_attr_registers_dump.attr,
	&dev_attr_reg.attr,
	&dev_attr_addr.attr,
	&dev_attr_over_reason.attr,
	&dev_attr_fod_regs_dump.attr,
	&dev_attr_dock_state.attr,
	&dev_attr_temp_adc.attr,
	&dev_attr_tx_signal_strength.attr,
	NULL,
};

static const struct attribute_group p922x_sysfs_group_attrs = {
	.attrs = p922x_sysfs_attrs,
};

static const struct of_device_id match_table[] = {
	{.compatible = "IDT,idt_wireless_power",},
	{},
};

static const struct i2c_device_id p922x_dev_id[] = {
	{"idt_wireless_power", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, p922x_dev_id);

/* first step: define regmap_config */
static const struct regmap_config p922x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

/*
 * The header implicitly provides the size of the message
 * contained in the Packet. The number of bytes in a message
 * is calculated from the value contained in the header of
 * the Packet.
 * Header  Message Size  Comment
 * 0x00...0x1F  1 + (Header - 0) / 32    1 * 32 messages(size 1)
 * 0x20...0x7F  2 + (Header - 32) / 16   6 * 16 messages(size 2...7)
 * 0x80...0xDF  8 + (Header - 128) / 8   12 * 8 messages(size 8...19)
 * 0xE0...0xFF  20 + (Header - 224) / 4  8 * 4 messages(size 20...27)
 */
static int p922x_extract_packet_size(u8 hdr)
{
	if (hdr < 0x20)
		return 1;
	if (hdr < 0x80)
		return (2 + ((hdr - 0x20) >> 4));
	if (hdr < 0xe0)
		return (8 + ((hdr - 0x80) >> 3));
	return (20 + ((hdr - 0xe0) >> 2));
}

static void p922x_write_fod(struct p922x_dev *chip, enum charger_type chr_type)
{
	int ret;
	u8 *fod_data = NULL;
	u8 fod_read[FOD_COEF_PARAM_LENGTH];

	if (chr_type == WIRELESS_10W_CHARGER &&
		chip->bpp_10w_fod_num == FOD_COEF_PARAM_LENGTH)
		fod_data = (u8 *)(chip->bpp_10w_fod);
	else if ((chr_type == WIRELESS_5W_CHARGER ||
		chr_type == WIRELESS_DEFAULT_CHARGER) &&
		chip->bpp_5w_fod_num == FOD_COEF_PARAM_LENGTH)
		fod_data = (u8 *)(chip->bpp_5w_fod);

	if (fod_data == NULL)
		goto no_fod_data;

	/*
	 * Manual power switch or automatic power switch may call this function
	 * at the same time, so add fod mutex lock to prevent concurrent access.
	 */
	mutex_lock(&chip->fod_lock);
	dev_info(chip->dev, "%s: chr_type: %d, writing bpp fod.\n", __func__, chr_type);
	ret = chip->bus.write_buf(chip, REG_FOD_COEF_ADDR, fod_data, FOD_COEF_PARAM_LENGTH);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write fod data: %d\n",
			__func__, ret);
		goto out;
	}

	ret = chip->bus.read_buf(chip, REG_FOD_COEF_ADDR, fod_read, FOD_COEF_PARAM_LENGTH);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read fod data: %d\n",
			__func__, ret);
		goto out;
	}

	if (memcmp(fod_data, fod_read, FOD_COEF_PARAM_LENGTH) == 0)
		goto out;

	dev_warn(chip->dev, "%s: compare error, chr_type:%d, fod read:%d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d",
		__func__, chr_type, fod_read[0], fod_read[1],
		fod_read[2], fod_read[3], fod_read[4], fod_read[5],
		fod_read[6], fod_read[7], fod_read[8], fod_read[9],
		fod_read[10], fod_read[11], fod_read[12], fod_read[13],
		fod_read[14], fod_read[15]);

out:
	mutex_unlock(&chip->fod_lock);
	return;
no_fod_data:
	dev_warn(chip->dev, "%s: Fod data not set.\n", __func__);
}

static void p922x_device_auth_req(struct p922x_dev *chip)
{
	int ret;

	/* set device authentication req */
	ret = chip->bus.write(chip, REG_COMMAND, SEND_DEVICE_AUTH);
	if (ret)
		dev_err(chip->dev, "%s: Failed to write command reg: %d\n",
			__func__, ret);
}

static void p922x_get_tx_adapter_type(struct p922x_dev *chip)
{
	u8 header = 0;
	int length;
	u8 data_list[16] = { 0 };
	int ret = 0;

	ret = chip->bus.read(chip, REG_BCHEADER_ADDR, &header);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read bcheader addr: %d\n",
			__func__, ret);
		return;
	}

	length = p922x_extract_packet_size(header);
	ret = chip->bus.read_buf(chip, REG_BCDATA_ADDR, data_list, length);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read bcdata addr: %d\n",
			__func__, ret);
		return;
	}

	dev_info(chip->dev, "%s: TX adapter type: %d\n", __func__, data_list[0]);
	chip->tx_adapter_type = data_list[0];

	if ((data_list[0] == ADAPTER_QC20) || (data_list[0] == ADAPTER_QC30)
		|| (data_list[0] == ADAPTER_PD))
		chip->is_hv_adapter = true;
}

static void p922x_sendpkt(struct p922x_dev *chip, struct propkt_type *pkt)
{
	/* include header by +1 */
	int length = p922x_extract_packet_size(pkt->header) + 1;
	int ret = 0;

	/*  write data into proprietary packet buffer */
	ret = chip->bus.write_buf(chip, REG_PROPPKT_ADDR, (u8 *)pkt, length);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to write proprietary packet buffer: %d\n", __func__, ret);
		return;
	}

	/* send proprietary packet */
	ret = chip->bus.write(chip, REG_COMMAND, SENDPROPP);
	if (ret)
		dev_err(chip->dev,
		"%s: Failed to write command reg: %d\n", __func__, ret);
}

static void p922x_detect_tx_adapter_type(struct p922x_dev *chip)
{
	struct propkt_type propkt;

	propkt.header = PROPRIETARY18;
	propkt.cmd = BC_ADAPTER_TYPE;

	dev_info(chip->dev, "%s: start\n", __func__);

	p922x_sendpkt(chip, &propkt);
}

static int p922x_send_eop(struct p922x_dev *chip, u8 reason)
{
	int ret;

	dev_info(chip->dev,
			"%s: Send EOP reason=%d\n", __func__, reason);
	ret = chip->bus.write(chip, REG_EPT, reason);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to write EPT: %d\n", __func__, ret);
		goto out;
	}
	ret = chip->bus.write(chip, REG_COMMAND, SENDEOP);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to send EOP: %d\n", __func__, ret);
		goto out;
	}

out:
	return ret;
}

static void p922x_over_handle(struct p922x_dev *chip, u16 irq_src)
{
	u8 reason = 0;
	int ret;

	dev_err(chip->dev,
			"%s: Received OVER INT: %02x\n", __func__, irq_src);
	if (irq_src & P922X_INT_OV_TEMP)
		reason = EOP_OVER_TEMP;
	else if (irq_src & P922X_INT_OV_VOLT)
		reason = EOP_OVER_VOLT;
	else
		reason = EOP_OVER_CURRENT;

	chip->over_reason = reason;
	ret = p922x_send_eop(chip, reason);
	if (ret)
		dev_err(chip->dev,
			"%s: Failed to send EOP %d: %d\n", __func__, reason, ret);
}

static void p922x_set_cm_cap_enable(struct p922x_dev *chip, bool en)
{

	int ret = 0;

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (chip->cm_cap_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
			__func__, en);
		return;
	}

	chip->cm_cap_en = en;
	if (en)
		ret = chip->bus.write(chip, REG_CM_CAP_EN_ADDR, EN_CM_CAP);
	else
		ret = chip->bus.write(chip, REG_CM_CAP_EN_ADDR, DIS_CM_CAP);
	if (ret)
		dev_err(chip->dev, "%s: Failed to enable cm cap: %d\n",
			__func__, ret);

}

static void p922x_set_tx_led_mode(struct p922x_dev *chip, enum led_mode mode)
{
	int ret = 0;

	ret = chip->bus.write(chip, REG_CHG_STATUS, mode);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write charge status: %d\n",
			__func__, ret);
		return;
	}
	ret = chip->bus.write(chip, REG_COMMAND, CHARGE_STAT);
	if (ret)
		dev_err(chip->dev, "%s: Failed to write command: %d\n",
			__func__, ret);
}

irqreturn_t p922x_int_thread(int irq, void *ptr)
{
	struct p922x_dev *chip = ptr;
	int ret = 0;
	u16 irq_src = 0, irq_status = 0;
	u8 buf[2] = {0};


	dev_info(chip->dev, "%s\n", __func__);

	ret = chip->bus.read_buf(chip, REG_STATUS, buf, 2);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to read status reg: %d\n", __func__, ret);
		goto out;
	}
	irq_status = buf[0] | (buf[1] << 8);

	dev_info(chip->dev, "%s: stat: 0x%04x\n", __func__, irq_status);

	ret = chip->bus.read_buf(chip, REG_INTR, buf, 2);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to read INT reg: %d\n", __func__, ret);
		goto out;
	}
	irq_src = buf[0] | (buf[1] << 8);

	dev_info(chip->dev, "%s: INT: 0x%04x\n", __func__, irq_src);
	if (!irq_src)
		goto out;

	ret = chip->bus.write_buf(chip, REG_INT_CLEAR, (u8 *)&irq_src, 2);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to clear INT reg: %d\n", __func__, ret);
		goto out;
	}

	ret = chip->bus.write(chip, REG_COMMAND, CLRINT);
	if (ret) {
		dev_err(chip->dev,
			"%s: Failed to reset INT: %d\n", __func__, ret);
		goto out;
	}

	irq_status |= irq_src;
	ret = chip->bus.write_buf(chip, REG_STATUS, (u8 *)&irq_status, 2);
	if (ret) {
		dev_err(chip->dev,
		"Failed to write IQR status: %d\n", ret);
		goto out;
	}

	if (irq_src & P922X_INT_VRECT)
		dev_info(chip->dev,
			"%s: Received VRECTON, online\n", __func__);

	if (irq_src & P922X_INT_TX_DATA_RECV)
		p922x_get_tx_adapter_type(chip);

	if ((irq_src & P922X_INT_DEVICE_AUTH_FAIL)) {
		if (chip->dev_auth_retry++ < 3) {
			dev_info(chip->dev, "%s: dev auth fail, retry = %d\n",
				__func__, chip->dev_auth_retry);
			p922x_device_auth_req(chip);
		} else
			chip->tx_authen_complete = true;
	}

	if (irq_src & P922X_INT_DEVICE_AUTH_SUCCESS) {
		chip->tx_dev_authen_status = true;
		chip->tx_authen_complete = true;
		p922x_detect_tx_adapter_type(chip);
	}

	if (irq_src & P922X_INT_ID_AUTH_FAIL)
		chip->tx_authen_complete = true;

	if (irq_src & P922X_INT_ID_AUTH_SUCCESS) {
		/* authentication success, can provide 10w charge */
		chip->tx_id_authen_status = true;
		chip->dev_auth_retry = 0;
		p922x_device_auth_req(chip);
	}

	if (irq_src & P922X_INT_LIMIT_MASK)
		p922x_over_handle(chip, irq_src);

out:

	return IRQ_HANDLED;
}

static int p922x_attached_vbus(struct p922x_dev *chip)
{
	int ret = 0;

	atomic_set(&chip->online, true);

	if (chip->use_buck) {
		/* set vout 6.5v */
		dev_info(chip->dev, "%s: use buck\n", __func__);
		ret = p922x_set_vout(chip, SET_VOUT_VAL);
		if (ret) {
			dev_err(chip->dev, "%s: Failed to set vout: %d\n", __func__, ret);
			p922x_enable_charge_flow(chip->chg_dev, false);
			return ret;
		}
	}

	p922x_enable_charge_flow(chip->chg_dev, true);
	return ret;
}

static void p922x_detached_vbus(struct p922x_dev *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	p922x_enable_charge_flow(chip->chg_dev, false);
}

static bool p922x_get_pg_irq_status(struct p922x_dev *chip)
{
	return gpio_get_value(chip->pg_gpio);
}

static irqreturn_t p922x_pg_thread(int irq, void *ptr)
{
	bool vbus_en = false;
	struct p922x_dev *chip = ptr;

	mutex_lock(&chip->irq_lock);
	vbus_en = p922x_get_pg_irq_status(chip);
	dev_info(chip->dev, "%s: vbus_en = %d\n", __func__, vbus_en);
	if (vbus_en)
		p922x_attached_vbus(chip);
	else
		p922x_detached_vbus(chip);
	mutex_unlock(&chip->irq_lock);

	return IRQ_HANDLED;
}

static int p922x_init_switch_voltage(struct p922x_dev *chip, int *data)
{
	if (!data) {
		dev_info(chip->dev, "%s: wpc-switch-voltage config incorrect in dts, use default value\n",
			__func__);
		chip->switch_voltage[CHARGE_5W_MODE].voltage_low = SWITCH_5W_VTH_L;
		chip->switch_voltage[CHARGE_5W_MODE].voltage_target = CHARGER_VOUT_5W;
		chip->switch_voltage[CHARGE_5W_MODE].voltage_high = SWITCH_5W_VTH_H;
		chip->switch_voltage[CHARGE_10W_MODE].voltage_low = SWITCH_10W_VTH_L;
		chip->switch_voltage[CHARGE_10W_MODE].voltage_target = CHARGER_VOUT_10W;
		chip->switch_voltage[CHARGE_10W_MODE].voltage_high = SWITCH_10W_VTH_H;
	} else {
		chip->switch_voltage[CHARGE_5W_MODE].voltage_low = data[0];
		chip->switch_voltage[CHARGE_5W_MODE].voltage_target = data[1];
		chip->switch_voltage[CHARGE_5W_MODE].voltage_high = data[2];
		chip->switch_voltage[CHARGE_10W_MODE].voltage_low = data[3];
		chip->switch_voltage[CHARGE_10W_MODE].voltage_target = data[4];
		chip->switch_voltage[CHARGE_10W_MODE].voltage_high = data[5];
	}
	return 0;
}

static int p922x_support_wpc_en_parse_dt(struct p922x_dev *chip)
{
	int ret;
	struct i2c_client *client = chip->client;

	chip->support_wpc_en = true;
	chip->wpc_en = true;
	chip->wpc_en_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chip->wpc_en_pinctrl)) {
		ret = PTR_ERR(chip->wpc_en_pinctrl);
		dev_err(chip->dev, "%s: Cannot find pinctrl\n", __func__);
		goto PINCTRL_ERR;
	}

	chip->wpc_en_default = pinctrl_lookup_state(chip->wpc_en_pinctrl, "wpc_en_default");
	if (IS_ERR(chip->wpc_en_default)) {
		ret = PTR_ERR(chip->wpc_en_default);
		dev_err(chip->dev, "%s: Cannot find pinctrl wpc_en_default\n", __func__);
		goto PINCTRL_ERR;
	}

	chip->wpc_en0 = pinctrl_lookup_state(chip->wpc_en_pinctrl, "wpc_en0");
	if (IS_ERR(chip->wpc_en0)) {
		ret = PTR_ERR(chip->wpc_en0);
		dev_err(chip->dev, "%s: Cannot find pinctrl wpc_en0!\n", __func__);
		goto PINCTRL_ERR;
	}

	chip->wpc_en1 = pinctrl_lookup_state(chip->wpc_en_pinctrl, "wpc_en1");
	if (IS_ERR(chip->wpc_en1)) {
		ret = PTR_ERR(chip->wpc_en1);
		dev_err(chip->dev, "%s: Cannot find pinctrl wpc_en1!\n", __func__);
		goto PINCTRL_ERR;
	}

	pinctrl_select_state(chip->wpc_en_pinctrl, chip->wpc_en0);

	return ret;

PINCTRL_ERR:
	chip->support_wpc_en = false;
	dev_err(chip->dev, "%s:not support_wpc_en!\n", __func__);

	return ret;
}

static int p922x_support_sleep_en_parse_dt(struct p922x_dev *chip)
{
	int ret;
	struct i2c_client *client = chip->client;

	chip->support_sleep_en = true;
	chip->sleep_en = false;
	chip->sleep_en_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chip->sleep_en_pinctrl)) {
		ret = PTR_ERR(chip->sleep_en_pinctrl);
		dev_err(chip->dev, "%s: Cannot find pinctrl\n", __func__);
		goto PINCTRL_ERR;
	}

	chip->sleep_en_default = pinctrl_lookup_state(chip->sleep_en_pinctrl, "sleep_en_default");
	if (IS_ERR(chip->sleep_en_default)) {
		ret = PTR_ERR(chip->sleep_en_default);
		dev_err(chip->dev, "%s: Cannot find pinctrl sleep_en_default\n", __func__);
		goto PINCTRL_ERR;
	}

	chip->sleep_en0 = pinctrl_lookup_state(chip->sleep_en_pinctrl, "sleep_en0");
	if (IS_ERR(chip->sleep_en0)) {
		ret = PTR_ERR(chip->sleep_en0);
		dev_err(chip->dev, "%s: Cannot find pinctrl sleep_en0!\n", __func__);
		goto PINCTRL_ERR;
	}

	chip->sleep_en1 = pinctrl_lookup_state(chip->sleep_en_pinctrl, "sleep_en1");
	if (IS_ERR(chip->sleep_en1)) {
		ret = PTR_ERR(chip->sleep_en1);
		dev_err(chip->dev, "%s: Cannot find pinctrl sleep_en1!\n", __func__);
		goto PINCTRL_ERR;
	}

	pinctrl_select_state(chip->sleep_en_pinctrl, chip->sleep_en0);

	return ret;

PINCTRL_ERR:
	chip->support_sleep_en = false;
	dev_err(chip->dev, "%s:not support_sleep_en!\n", __func__);

	return ret;
}

static int p922x_parse_dt(struct p922x_dev *chip)
{
	int ret, count;
	struct device_node *dt = chip->client->dev.of_node;
	struct i2c_client *client = chip->client;
	const struct of_device_id *match = NULL;
	int data[SWITCH_VOLTAGE_COUNT];
	u8 fod_data[FOD_COEF_PARAM_LENGTH];

	if (!dt) {
		dev_err(chip->dev, "%s: Device does not have associated DT data\n",
				__func__);
		return -EINVAL;
	}

	dev_err(chip->dev, "%s: Device have associated DT data\n", __func__);

	p922x_support_wpc_en_parse_dt(chip);
	p922x_support_sleep_en_parse_dt(chip);

	match = of_match_device(match_table, &client->dev);
	if (!match) {
		dev_err(chip->dev, "%s: Unknown device model\n", __func__);
		return -EINVAL;
	}

	chip->int_gpio = of_get_named_gpio(dt, "p922x-irq", 0);
	if (!gpio_is_valid(chip->int_gpio))
		dev_err(chip->dev, "%s: No valid irq gpio\n", __func__);

	chip->pg_gpio = of_get_named_gpio(dt, "p922x-pg", 0);
	if (!gpio_is_valid(chip->pg_gpio))
		dev_err(chip->dev, "%s: No valid pg gpio\n", __func__);

	if (of_property_read_u32(dt, "wpc-hardware-config",
				&chip->wpc_support) < 0)
		dev_err(chip->dev, "%s: Failed to parse wpc-hardware-config\n",
			__func__);

	chip->use_buck = of_property_read_bool(dt, "use-buck");

	ret = of_property_read_u32_array(dt, "wpc-mivr", chip->wpc_mivr,
					ARRAY_SIZE(chip->wpc_mivr));
	if (ret) {
		dev_info(chip->dev, "%s: wpc_mivr %d %d",
			__func__, chip->wpc_mivr[0], chip->wpc_mivr[1]);
		return -EINVAL;
	}

	count = of_property_count_u32_elems(dt, "wpc-switch-voltage");
	ret = of_property_read_u32_array(dt, "wpc-switch-voltage", data,
					ARRAY_SIZE(data));
	if (count == SWITCH_VOLTAGE_COUNT && !ret)
		p922x_init_switch_voltage(chip, data);
	else
		p922x_init_switch_voltage(chip, NULL);

	dev_info(chip->dev, "%s: wpc_mivr (%d %d)uV, voltage (%d %d %d %d %d %d)mV",
			__func__, chip->wpc_mivr[0], chip->wpc_mivr[1],
			chip->switch_voltage[CHARGE_5W_MODE].voltage_low,
			chip->switch_voltage[CHARGE_5W_MODE].voltage_target,
			chip->switch_voltage[CHARGE_5W_MODE].voltage_high,
			chip->switch_voltage[CHARGE_10W_MODE].voltage_low,
			chip->switch_voltage[CHARGE_10W_MODE].voltage_target,
			chip->switch_voltage[CHARGE_10W_MODE].voltage_high);

	chip->bpp_5w_fod_num = of_property_count_elems_of_size(dt, "bpp-5w-fod", sizeof(u8));
	if (chip->bpp_5w_fod_num != FOD_COEF_PARAM_LENGTH) {
		dev_err(chip->dev, "%s: Incorrect num of 5w fod data", __func__);
		chip->bpp_5w_fod_num = 0;
	} else {
		ret = of_property_read_u8_array(dt, "bpp-5w-fod", fod_data, sizeof(fod_data));
		if (ret == 0) {
			memcpy(chip->bpp_5w_fod, fod_data, sizeof(fod_data));
			dev_info(chip->dev, "%s: 5w fod data:%d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d",
				__func__, chip->bpp_5w_fod[0].gain, chip->bpp_5w_fod[0].offs,
				chip->bpp_5w_fod[1].gain, chip->bpp_5w_fod[1].offs,
				chip->bpp_5w_fod[2].gain, chip->bpp_5w_fod[2].offs,
				chip->bpp_5w_fod[3].gain, chip->bpp_5w_fod[3].offs,
				chip->bpp_5w_fod[4].gain, chip->bpp_5w_fod[4].offs,
				chip->bpp_5w_fod[5].gain, chip->bpp_5w_fod[5].offs,
				chip->bpp_5w_fod[6].gain, chip->bpp_5w_fod[6].offs,
				chip->bpp_5w_fod[7].gain, chip->bpp_5w_fod[7].offs);
		} else
			dev_err(chip->dev, "%s: Failed to parse bpp-5w-fod.\n",
				__func__);
	}

	chip->bpp_10w_fod_num = of_property_count_elems_of_size(dt, "bpp-10w-fod", sizeof(u8));
	if (chip->bpp_10w_fod_num != FOD_COEF_PARAM_LENGTH) {
		dev_err(chip->dev, "%s: Incorrect num of 10w fod data", __func__);
		chip->bpp_10w_fod_num = 0;
	} else {
		ret = of_property_read_u8_array(dt, "bpp-10w-fod", fod_data, sizeof(fod_data));
		if (ret == 0) {
			memcpy(chip->bpp_10w_fod, (u8 *)fod_data, sizeof(fod_data));
			dev_info(chip->dev, "%s: 10w fod data:%d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d",
				__func__, chip->bpp_10w_fod[0].gain, chip->bpp_10w_fod[0].offs,
				chip->bpp_10w_fod[1].gain, chip->bpp_10w_fod[1].offs,
				chip->bpp_10w_fod[2].gain, chip->bpp_10w_fod[2].offs,
				chip->bpp_10w_fod[3].gain, chip->bpp_10w_fod[3].offs,
				chip->bpp_10w_fod[4].gain, chip->bpp_10w_fod[4].offs,
				chip->bpp_10w_fod[5].gain, chip->bpp_10w_fod[5].offs,
				chip->bpp_10w_fod[6].gain, chip->bpp_10w_fod[6].offs,
				chip->bpp_10w_fod[7].gain, chip->bpp_10w_fod[7].offs);
		} else
			dev_err(chip->dev, "%s: Failed to parse bpp-10w-fod.\n",
				__func__);
	}

	return 0;
}

static int p922x_set_wpc_en(struct charger_device *chg_dev, bool en)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (!chip->support_wpc_en) {
		dev_dbg(chip->dev, "%s: not support_wpc_en in proto/hvt build\n",
				__func__);
		return 0;
	}

	if (chip->wpc_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}
	chip->wpc_en = en;

	if (en) {
		pinctrl_select_state(chip->wpc_en_pinctrl, chip->wpc_en0);
	} else {
		msleep(10);
		pinctrl_select_state(chip->wpc_en_pinctrl, chip->wpc_en1);
	}

	return 0;
}

static int p922x_set_sleep_en(struct charger_device *chg_dev, bool en)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (!chip->support_sleep_en) {
		dev_dbg(chip->dev, "%s: not support_sleep_en in proto/hvt build\n",
				__func__);
		return 0;
	}

	if (chip->sleep_en == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}
	chip->sleep_en = en;

	if (en)
		pinctrl_select_state(chip->sleep_en_pinctrl, chip->sleep_en1);
	else
		pinctrl_select_state(chip->sleep_en_pinctrl, chip->sleep_en0);

	return 0;
}

static int p922x_request_io_port(struct p922x_dev *chip)
{
	int ret = 0;

	ret = gpio_request(chip->int_gpio, "idt_int");
	if (ret < 0) {
		dev_err(chip->dev,
			"%s: Failed to request GPIO:%d, ERRNO:%d\n",
			__func__, (s32)chip->int_gpio, ret);
		return -ENODEV;
	}
	gpio_direction_input(chip->int_gpio);
	dev_info(chip->dev, "%s: Success request int-gpio\n", __func__);

	ret = gpio_request(chip->pg_gpio, "idt_pg");
	if (ret < 0) {
		dev_err(chip->dev,
			"%s: Failed to request GPIO:%d, ERRNO:%d\n",
			__func__, (s32)chip->pg_gpio, ret);

		if (gpio_is_valid(chip->int_gpio))
			gpio_free(chip->int_gpio);

		return -ENODEV;
	}
	gpio_direction_input(chip->pg_gpio);
	dev_info(chip->dev, "%s: Success request pg-gpio\n", __func__);

	return 0;
}

static int p922x_register_irq(struct p922x_dev *chip)
{
	int ret = 0;

	chip->int_num = gpio_to_irq(chip->int_gpio);
	dev_info(chip->dev, "%s: gpio:%d, gpio_irq:%d\n",
		__func__, chip->int_gpio, chip->int_num);

	ret = request_threaded_irq(chip->int_num, NULL,
				p922x_int_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, IDT_INT,
				chip);
	if (ret == 0) {
		dev_info(chip->dev, "%s: irq disable at gpio: %d\n",
					__func__, chip->int_num);
		disable_irq_nosync(chip->int_num);
	} else
		dev_err(chip->dev, "%s: request_irq failed\n", __func__);

	chip->pg_num = gpio_to_irq(chip->pg_gpio);
	dev_info(chip->dev, "%s: gpio:%d, gpio_irq:%d\n",
			__func__, chip->pg_gpio, chip->pg_num);

	ret = request_threaded_irq(chip->pg_num, NULL, p922x_pg_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			IDT_PG, chip);
	if (ret == 0) {
		dev_info(chip->dev, "%s: irq disable at gpio: %d\n",
					__func__, chip->pg_num);
		disable_irq_nosync(chip->pg_num);
	} else
		dev_err(chip->dev, "%s: request_irq failed, ret: %d\n",
					__func__, ret);

	return ret;
}

static int p922x_set_charger_mivr(struct p922x_dev *chip,
			struct charger_device *chg1_dev, u32 uV)
{
	int ret = 0;

	ret = charger_dev_set_mivr(chg1_dev, uV);
	if (ret < 0)
		dev_err(chip->dev, "%s: failed, ret = %d\n", __func__, ret);

	return ret;
}

static int p9221_set_charger_mivr_by_type(struct p922x_dev *chip,
		enum charger_type type)
{
	int ret = 0;
	bool is_en = false;
	bool power_path_en = !(type == CHARGER_UNKNOWN);
	struct charger_device *chg1_dev = get_charger_by_name("primary_chg");

	ret = charger_dev_is_powerpath_enabled(chg1_dev, &is_en);
	if (ret < 0) {
		dev_err(chip->dev, "%s: get is power path enabled failed\n", __func__);
		return ret;
	}

	if (is_en != power_path_en)
		charger_dev_enable_powerpath(chg1_dev,  power_path_en);

	switch (type) {
	case WIRELESS_DEFAULT_CHARGER:
	case WIRELESS_5W_CHARGER:
		p922x_set_charger_mivr(chip,
			chg1_dev, chip->wpc_mivr[CHARGE_5W_MODE]);
		break;
	case WIRELESS_10W_CHARGER:
		p922x_set_charger_mivr(chip,
			chg1_dev, chip->wpc_mivr[CHARGE_10W_MODE]);
		break;
	default:
		break;
	}

	return ret;
}

static int p922x_update_charge_online(struct p922x_dev *chip, bool online)
{
	int ret;
	union power_supply_propval propval;

	struct power_supply *chg_psy = power_supply_get_by_name("charger");

	ret = power_supply_get_property(chg_psy,
		POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0) {
		dev_err(chip->dev, "%s: get psy online failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	if (propval.intval == online) {
		dev_info(chip->dev, "%s: psy online status same as %d\n",
				__func__, online);
		return ret;
	}

	propval.intval = online;
	ret = power_supply_set_property(chg_psy,
		POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		dev_err(chip->dev, "%s: set psy online failed, ret = %d\n",
			__func__, ret);

	dev_info(chip->dev, "%s:online = %d\n", __func__, online);
	return ret;
}

static int p922x_update_charge_type(struct p922x_dev *chip,
			enum charger_type type)
{
	int ret;
	union power_supply_propval propval;

	struct power_supply *chg_psy = power_supply_get_by_name("charger");

	ret = power_supply_get_property(chg_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0) {
		dev_err(chip->dev, "%s: get psy type failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	if (propval.intval == type) {
		dev_info(chip->dev, "%s: psy type is same as %d, not need update\n",
			__func__, type);
		return ret;
	}

	p9221_set_charger_mivr_by_type(chip, type);

	propval.intval = type;
	ret = power_supply_set_property(chg_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		dev_err(chip->dev, "%s: set psy type failed, ret = %d\n",
			__func__, ret);

	power_supply_changed(chip->wpc_psy);
	dev_info(chip->dev, "%s:type = %d\n", __func__, type);
	return ret;
}

static void p922x_power_switch_work(struct work_struct *work)
{
	static int switch_detct_count;
	int vout = 0;
	struct p922x_dev *chip = container_of(work, struct p922x_dev,
						 power_switch_work.work);

	cancel_delayed_work(&chip->power_switch_work);

	vout = p922x_get_vout_adc(chip);
	dev_info(chip->dev, "%s\n", __func__);

	if (switch_detct_count > 2) {
		switch_detct_count = 0;
		return;
	}
	if (vout >= chip->switch_vth_low && vout <= chip->switch_vth_high) {
		switch_detct_count = 0;
		p922x_update_charge_type(chip, chip->power_switch_target);
		p922x_write_fod(chip, chip->power_switch_target);
		return;
	}

	switch_detct_count++;
	schedule_delayed_work(&chip->power_switch_work,  SWITCH_DETECT_TIME);

	/* if not match target voltage, resend switch command */
	if (chip->power_switch_target == WIRELESS_10W_CHARGER)
		p922x_power_switch(chip,
			chip->switch_voltage[CHARGE_10W_MODE].voltage_target);
	else
		p922x_power_switch(chip,
			chip->switch_voltage[CHARGE_5W_MODE].voltage_target);
}

static void p922x_clear_fast_charge_switch(struct p922x_dev *chip)
{
	dev_info(chip->dev, "p922x clear fast charge switch\n");
	cancel_delayed_work(&chip->power_switch_work);
	chip->tx_dev_authen_status = false;
	chip->tx_id_authen_status = false;
	chip->force_switch = false;
	chip->is_hv_adapter = false;
	chip->tx_authen_complete = false;
	chip->cm_cap_en = false;
	chip->tx_adapter_type = ADAPTER_UNKNOWN;
}

static void p922x_fast_charge_enable(struct p922x_dev *chip, bool en)
{
	bool stat;

	dev_info(chip->dev, "%s: en=%d\n", __func__, en);

	stat = atomic_read(&chip->online);
	if (!stat)
		return;

	cancel_delayed_work(&chip->power_switch_work);
	if (chip->tx_id_authen_status != true ||
		chip->tx_dev_authen_status != true) {
		dev_info(chip->dev, "%s: id authen:%d, dev authen:%d\n",
				__func__, chip->tx_id_authen_status,
				chip->tx_dev_authen_status);
		return;
	}

	if (en) {
		/* switch to 10w charge */
		p922x_power_switch(chip,
			chip->switch_voltage[CHARGE_10W_MODE].voltage_target);
		chip->switch_vth_low =
			chip->switch_voltage[CHARGE_10W_MODE].voltage_low;
		chip->switch_vth_high =
			chip->switch_voltage[CHARGE_10W_MODE].voltage_high;
		chip->power_switch_target = WIRELESS_10W_CHARGER;
	} else {
		/* switch to 5w charge */
		p922x_power_switch(chip,
			chip->switch_voltage[CHARGE_5W_MODE].voltage_target);
		chip->switch_vth_low =
			chip->switch_voltage[CHARGE_5W_MODE].voltage_low;
		chip->switch_vth_high =
			chip->switch_voltage[CHARGE_5W_MODE].voltage_high;
		chip->power_switch_target = WIRELESS_5W_CHARGER;
	}
	schedule_delayed_work(&chip->power_switch_work,  SWITCH_DETECT_TIME);
}

static int p922x_get_online(struct charger_device *chg_dev, bool *stat)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chip->irq_lock);
	*stat = atomic_read(&chip->online);
	dev_dbg(chip->dev, "%s: get online status: %d\n", __func__, *stat);
	mutex_unlock(&chip->irq_lock);

	return 0;
}

static int p922x_get_temp(struct charger_device *chg_dev)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);

	return p922x_get_temp_adc(chip);
}


static int p922x_do_algorithm(struct charger_device *chg_dev, void *data)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);
	struct charger_manager *info = (struct charger_manager *)data;
	enum charger_type chr_type = mt_get_charger_type();
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	bool chg_done, stat;

	mutex_lock(&chip->irq_lock);
	stat = atomic_read(&chip->online);
	if (!stat)
		goto out;

	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
	dev_info(chip->dev, "%s: chr_type=%d, chg_done=%d\n",
				__func__, chr_type, chg_done);

	if (chip->tx_authen_complete != true) {
		dev_info(chip->dev, "%s: tx authen not completed\n", __func__);
		goto out;
	}

	if (chr_type == WIRELESS_DEFAULT_CHARGER) {
		if (chip->tx_id_authen_status == true &&
			chip->tx_dev_authen_status == true)
			p922x_fast_charge_enable(chip, true);
		else
			p922x_update_charge_type(chip, WIRELESS_5W_CHARGER);

		p922x_metrics_tx_signal_strength(chip);
	}

	if (chr_type == WIRELESS_10W_CHARGER) {
		if (chg_done)
			p922x_set_tx_led_mode(chip, LED_CONSTANT_OFF);
		else
			p922x_set_tx_led_mode(chip, LED_CONSTANT_ON);
	}

out:
	mutex_unlock(&chip->irq_lock);

	dev_info(chip->dev, "%s: %d, %d, %d, %d\n",
				__func__, chr_type, info->bat_eoc_protect,
				info->custom_charging_cv, swchgalg->state);
	if (((info->bat_eoc_protect == 1) || (info->custom_charging_cv > 0))
		&& (swchgalg->state == CHR_BATFULL)) {
		if (info != NULL && info->change_current_setting)
			info->change_current_setting(info);
	}
	return 0;
}

static int p922x_force_enable_charge(struct charger_device *chg_dev, bool en)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s\n", __func__);
	mutex_lock(&chip->irq_lock);
	p922x_enable_charge_flow(chip->chg_dev, en);
	mutex_unlock(&chip->irq_lock);

	return 0;
}

static void p922x_switch_set_state(struct p922x_dev *chip, enum dock_state_type set_state)
{
	int cur_state;

	dev_info(chip->dev, "%s\n", __func__);

	cur_state = switch_get_state(&chip->dock_state);
	if (cur_state != set_state) {
		dev_info(chip->dev, "%s: state changed: %d -> %d\n",
			__func__, cur_state, set_state);
		switch_set_state(&chip->dock_state, set_state);
	} else
		dev_info(chip->dev, "%s: state is same as %d, not need update\n",
			__func__, set_state);
}

static int p922x_enable_charge_flow(struct charger_device *chg_dev, bool en)
{
	struct p922x_dev *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s: %s\n", __func__, en ? "true" : "false");

	if (chip->is_enabled == en) {
		dev_info(chip->dev, "%s: is_enabled status same as %d\n",
				__func__, en);
		return 0;
	}
	chip->is_enabled = en;

	if (en) {
		p922x_set_cm_cap_enable(chip, true);
		p922x_switch_set_state(chip, TYPE_DOCKED);
		p922x_update_charge_online(chip, true);
		p922x_update_charge_type(chip, WIRELESS_DEFAULT_CHARGER);
		p922x_write_fod(chip, WIRELESS_DEFAULT_CHARGER);
	} else {
		p922x_switch_set_state(chip, TYPE_UNDOCKED);
		p922x_clear_fast_charge_switch(chip);
		atomic_set(&chip->online, false);
		p922x_update_charge_online(chip, false);
		p922x_update_charge_type(chip, CHARGER_UNKNOWN);
	}

	return 0;
}

static struct charger_ops p922x_chg_ops = {
	.get_temp = p922x_get_temp,
	.get_wpc_online = p922x_get_online,
	.do_wpc_algorithm = p922x_do_algorithm,
	.force_enable_wpc_charge = p922x_force_enable_charge,
	.set_wpc_en = p922x_set_wpc_en,
	.set_sleep_en = p922x_set_sleep_en,
};

static struct p922x_desc p922x_default_desc = {
	.chg_dev_name = "wireless_chg",
	.alias_name = "p922x",
};


static int p922x_chager_device_register(struct p922x_dev *chip)
{
	int ret = 0;

	chip->desc = &p922x_default_desc;
	chip->chg_props.alias_name = chip->desc->alias_name;
	chip->chg_dev =	charger_device_register(chip->desc->chg_dev_name,
		chip->dev, chip, &p922x_chg_ops, &chip->chg_props);

	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		return ret;
	}

	return 0;
}

static bool p922x_get_attach_status(struct p922x_dev *chip)
{
	int ret = 0;
	bool attach_status = false;
	u16 irq_status = 0;
	u8 buf[2] = {0};

	/* powergood not high, tx is not attached */
	if (!p922x_get_pg_irq_status(chip))
		goto dettached;

	dev_info(chip->dev, "%s: powergood online\n", __func__);

	/* check triggered interrupt if tx is attached */
	ret = chip->bus.read_buf(chip, REG_STATUS, buf, 2);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read INT reg: %d\n",
			__func__, ret);
		goto dettached;
	}
	irq_status = buf[0] | (buf[1] << 8);

	dev_info(chip->dev, "%s: IRQ status: %04x\n", __func__, irq_status);
	/* Don't need to handle interrput if irq_status is zero. */
	if (!irq_status)
		goto attached;

	/* clear interrupt register */
	ret = chip->bus.write_buf(chip, REG_INT_CLEAR, (u8 *)&irq_status, 2);
	if (ret) {
		dev_err(chip->dev,
		"%s: Failed to clear INT reg: %d\n", __func__, ret);
		goto dettached;
	}

	ret = chip->bus.write(chip, REG_COMMAND, CLRINT);
	if (ret) {
		dev_err(chip->dev,
		"%s: Failed to reset INT: %d\n", __func__, ret);
		goto dettached;
	}

	/* handle triggered interrput */
	ret = chip->bus.write_buf(chip, REG_STATUS, (u8 *)&irq_status, 2);
	if (ret) {
		dev_err(chip->dev,
		"%s: Failed to write IQR status: %d\n", __func__,  ret);
		goto dettached;
	}

	if (irq_status & P922X_INT_ID_AUTH_FAIL)
		chip->tx_authen_complete = true;

	/* authentication success, can provide 10w charge */
	if (irq_status & P922X_INT_ID_AUTH_SUCCESS)
		chip->tx_id_authen_status = true;
	if (irq_status & P922X_INT_DEVICE_AUTH_SUCCESS) {
		chip->tx_dev_authen_status = true;
		chip->tx_authen_complete = true;
	}

attached:
	/* set true if RX attached TX works well */
	atomic_set(&chip->online, true);
	attach_status = true;
	return attach_status;

dettached:
	atomic_set(&chip->online, false);
	return attach_status;

}

static void p922x_determine_init_status(struct work_struct *work)
{
	struct p922x_dev *chip = container_of(work, struct p922x_dev,
						wpc_init_work.work);
	unsigned int boot_mode = get_boot_mode();

	if ((boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
	    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
		&& (!p922x_get_pg_irq_status(chip))) {
		dev_info(chip->dev, "%s: KPOC without WPC, not process WPC.\n",
			__func__);
		return;
	}

	if (p922x_get_pg_irq_status(chip))
		atomic_set(&chip->online, true);
	else
		atomic_set(&chip->online, false);

	if (!is_mtk_charger_init_done())
		goto out;

	if (p922x_get_attach_status(chip))
		p922x_attached_vbus(chip);

	enable_irq(chip->int_num);
	enable_irq(chip->pg_num);

	if (p922x_get_pg_irq_status(chip) && chip->tx_id_authen_status) {
		if (chip->tx_dev_authen_status)
			p922x_detect_tx_adapter_type(chip);
		else
			p922x_device_auth_req(chip);
	}

	pr_info("%s: wpc init successfully.\n", __func__);

	return;

out:
	pr_info("%s: mtk_charger not init done.\n", __func__);
	schedule_delayed_work(&chip->wpc_init_work,
				READY_DETECT_TIME);

}

static int p922x_psy_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct p922x_dev *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&chip->online);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!atomic_read(&chip->online))
			return -ENODEV;

		val->intval = p922x_get_temp_adc(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property p922x_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TEMP,
};

static int p922x_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	if (device_may_wakeup(chip->dev))
		enable_irq_wake(chip->pg_num);
	disable_irq(chip->pg_num);
	disable_irq(chip->int_num);
	dev_info(chip->dev, "%s\n", __func__);

	return 0;
}

static int p922x_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p922x_dev *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);
	enable_irq(chip->pg_num);
	enable_irq(chip->int_num);
	if (device_may_wakeup(chip->dev))
		disable_irq_wake(chip->pg_num);

	return 0;
}

static SIMPLE_DEV_PM_OPS(p922x_pm_ops,
			p922x_pm_suspend, p922x_pm_resume);

static int p922x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct p922x_dev *chip;
	int ret = 0;
	u8 val = 0;

	pr_info("%s: enter.\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);

	pr_info("%s: chip.\n", __func__);
	chip->regmap = regmap_init_i2c(client, &p922x_regmap_config);
	if (!chip->regmap) {
		pr_err("%s parent regmap is missing\n", __func__);
		ret = -EINVAL;
		goto out_kfree;
	}

	pr_info("%s: regmap.\n", __func__);
	chip->client = client;
	chip->dev = &client->dev;

	chip->bus.read = p922x_read;
	chip->bus.write = p922x_write;
	chip->bus.read_buf = p922x_read_buffer;
	chip->bus.write_buf = p922x_write_buffer;
	chip->wpc_support = 0;
	chip->use_buck = false;
	chip->force_switch = false;
	chip->is_hv_adapter = false;
	chip->tx_id_authen_status = false;
	chip->tx_dev_authen_status = false;
	chip->is_enabled = false;
	chip->over_reason = 0;
	chip->tx_adapter_type = ADAPTER_UNKNOWN;
	chip->reg.addr = 0x0000;
	chip->reg.size = 1;
	chip->dev_auth_retry = 0;
	chip->cm_cap_en = false;
	chip->tx_authen_complete = false;

	device_init_wakeup(chip->dev, true);
	mutex_init(&chip->sys_lock);
	mutex_init(&chip->irq_lock);
	mutex_init(&chip->fod_lock);

	ret = p922x_parse_dt(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: parse dts failed, ret: %d\n",
			__func__, ret);
		goto out_remap;
	}

	if (chip->wpc_support != 1) {
		dev_err(chip->dev, "%s: not support wireless charger!\n",
			__func__);
		ret = -ENODEV;
		goto out_remap;
	}

	chip->wpc_desc.name = "Wireless";
	chip->wpc_desc.type = POWER_SUPPLY_TYPE_WIRELESS_5W;
	chip->wpc_desc.properties = p922x_psy_properties;
	chip->wpc_desc.num_properties = ARRAY_SIZE(p922x_psy_properties);
	chip->wpc_desc.get_property = p922x_psy_get_property;
	chip->wpc_cfg.drv_data = chip;
	chip->wpc_psy = power_supply_register(chip->dev, &chip->wpc_desc,
		&chip->wpc_cfg);
	if (IS_ERR(chip->wpc_psy)) {
		dev_err(chip->dev, "Failed to register power supply: %ld\n",
			PTR_ERR(chip->wpc_psy));
		ret = PTR_ERR(chip->wpc_psy);
		goto out_remap;
	}

	chip->dock_state.name = "dock";
	chip->dock_state.index = 0;
	chip->dock_state.state = TYPE_UNDOCKED;
	ret = switch_dev_register(&chip->dock_state);
	if (ret) {
		dev_err(chip->dev, "%s: switch_dev_register dock_state Fail\n",
			__func__);
		goto out_wpc_psy;
	}

	ret = p922x_request_io_port(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Failed request IO port, ret: %d\n",
			__func__, ret);
		goto out_switch;
	}

	ret = p922x_register_irq(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Failed reqister irq, ret: %d\n",
			__func__, ret);
		goto out_gpio;
	}

	ret = p922x_chager_device_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: chager device reqister failed, ret: %d\n",
			__func__, ret);
		goto out_irq;
	}

	INIT_DELAYED_WORK(&chip->power_switch_work, p922x_power_switch_work);
	INIT_DELAYED_WORK(&chip->wpc_init_work, p922x_determine_init_status);

	chip->bus.read(chip, 0x5870, &val);
	pr_info("IDT 0x5870 %s:%d :%02x\n", __func__, __LINE__, val);
	chip->bus.read(chip, 0x5874, &val);
	pr_info("IDT 0x5874 %s:%d :%02x\n", __func__, __LINE__, val);

	p922x_fwver(chip);

	ret = sysfs_create_group(&client->dev.kobj, &p922x_sysfs_group_attrs);
	if (ret != 0) {
		pr_debug("[idt] %s: - ERROR: sysfs_create_group() failed.\n",
			 __func__);
		goto out_charger;
	}

	schedule_delayed_work(&chip->wpc_init_work, 0);

	pr_info("%s: successfully.\n", __func__);
	return 0;

out_charger:
	if (chip->chg_dev)
		charger_device_unregister(chip->chg_dev);
out_irq:
	if (chip->int_num)
		free_irq(chip->int_num, chip);
	if (chip->pg_num)
		free_irq(chip->pg_num, chip);
out_gpio:
	if (gpio_is_valid(chip->int_gpio))
		gpio_free(chip->int_gpio);
	if (gpio_is_valid(chip->pg_gpio))
		gpio_free(chip->pg_gpio);
out_switch:
	switch_dev_unregister(&chip->dock_state);
out_wpc_psy:
	power_supply_unregister(chip->wpc_psy);
out_remap:
	regmap_exit(chip->regmap);
	device_init_wakeup(chip->dev, false);
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->sys_lock);
	mutex_destroy(&chip->fod_lock);
out_kfree:
	devm_kfree(&client->dev, chip);

	return ret;

}

static int p922x_remove(struct i2c_client *client)
{
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);

	if (chip->int_num)
		free_irq(chip->int_num, chip);
	if (chip->pg_num)
		free_irq(chip->pg_num, chip);

	if (gpio_is_valid(chip->int_gpio))
		gpio_free(chip->int_gpio);
	if (gpio_is_valid(chip->pg_gpio))
		gpio_free(chip->pg_gpio);

	if (chip->chg_dev)
		charger_device_unregister(chip->chg_dev);
	mutex_destroy(&chip->irq_lock);
	mutex_destroy(&chip->sys_lock);
	mutex_destroy(&chip->fod_lock);
	sysfs_remove_group(&client->dev.kobj, &p922x_sysfs_group_attrs);
	device_init_wakeup(chip->dev, false);
	regmap_exit(chip->regmap);
	switch_dev_unregister(&chip->dock_state);
	power_supply_unregister(chip->wpc_psy);

	return 0;
}

static void p922x_shutdown(struct i2c_client *client)
{
	struct p922x_dev *chip = (struct p922x_dev *)i2c_get_clientdata(client);
	unsigned int boot_mode = get_boot_mode();

	dev_info(chip->dev, "%s: start\n", __func__);

	if ((boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
	    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
	    && (!p922x_get_pg_irq_status(chip))) {
	    dev_info(chip->dev, "%s: not switch 5v in KPOC mode.\n", __func__);
	    return;
	}

	mutex_lock(&chip->irq_lock);
	if (!atomic_read(&chip->online))
		goto out;

	chip->force_switch = true;
	cancel_delayed_work(&chip->power_switch_work);
	/* force switch to 5w to prevent lk over voltage. */
	if (chip->power_switch_target == WIRELESS_10W_CHARGER) {
		p922x_power_switch(chip,
			chip->switch_voltage[CHARGE_5W_MODE].voltage_target);
		p922x_write_fod(chip, WIRELESS_5W_CHARGER);
	}

out:
	mutex_unlock(&chip->irq_lock);
}

static struct i2c_driver p922x_driver = {
	.driver = {
		.name = "idt_wireless_power",
		.owner = THIS_MODULE,
		.of_match_table = match_table,
		.pm = &p922x_pm_ops,
	},
	.probe = p922x_probe,
	.remove = p922x_remove,
	.shutdown = p922x_shutdown,
	.id_table = p922x_dev_id,
};

static int __init p922x_driver_init(void)
{
	return i2c_add_driver(&p922x_driver);
}

static void __exit p922x_driver_exit(void)
{
	i2c_del_driver(&p922x_driver);
}

late_initcall(p922x_driver_init);
module_exit(p922x_driver_exit);

MODULE_AUTHOR("roy.luo@idt.com, simon.song.df@renesas.com");
MODULE_DESCRIPTION("P922x Wireless Power Charger Monitor driver");
MODULE_LICENSE("GPL v2");
