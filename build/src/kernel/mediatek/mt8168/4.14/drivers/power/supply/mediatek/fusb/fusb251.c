/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <tcpm.h>
#include "fusb251.h"

#define I2C_RETRIES 2
#define I2C_RETRY_DELAY 5 /* ms */

static struct fusb251 *g_fusb251;

static int fusb251_i2c_read(struct fusb251 *fusb251, unsigned char reg,
			int len, unsigned char value[])
{
	struct i2c_client *client;
	int err;
	int tries = 0;
	int error = 0;
	struct i2c_msg msgs[] = {
		{
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	client = fusb251->i2c;
	msgs[0].addr = client->addr;
	msgs[1].addr = client->addr;

	do {
		err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		pr_err("%s: i2c_read error, %d\n", __func__, err);
		error = -1;
	}

	return error;
}

static int fusb251_i2c_write(struct fusb251 *fusb251,
			char *writebuf, int writelen)
{
	int ret = 0;
	struct i2c_client *client;

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	client = fusb251->i2c;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("%s: i2c_write error, ret=%d", __func__, ret);
	}

	return ret;
}

static int fusb251_read_reg(struct fusb251 *fusb251,
			unsigned char addr, unsigned char *value)
{
	int ret = 0;

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&fusb251->fusb251_io_mutex);

	ret = fusb251_i2c_read(fusb251, addr, 1, value);

	mutex_unlock(&fusb251->fusb251_io_mutex);

	return ret;
}

static int fusb251_write_reg(struct fusb251 *fusb251,
			unsigned char addr, unsigned char value)
{
	int ret = 0;
	unsigned char buf[2] = {0};

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&fusb251->fusb251_io_mutex);

	buf[0] = addr;
	buf[1] = value;
	ret = fusb251_i2c_write(fusb251, buf, sizeof(buf));

	mutex_unlock(&fusb251->fusb251_io_mutex);

	return ret;
}

static void fusb251_read_interface(struct fusb251 *fusb251,
				unsigned char reg, unsigned char mask,
				unsigned char shift, unsigned char *value)
{
	unsigned char reg_val = 0;

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return;
	}

	mutex_lock(&fusb251->fusb251_io_mutex);

	fusb251_i2c_read(fusb251, reg, 1, &reg_val);
	reg_val &= (mask << shift);
	*value = (reg_val >> shift);

	mutex_unlock(&fusb251->fusb251_io_mutex);
}

static void fusb251_config_interface(struct fusb251 *fusb251,
				unsigned char reg, unsigned char mask,
				unsigned char shift, int value)
{
	unsigned char reg_val = 0;
	unsigned char buf[2] = {0};

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return;
	}

	mutex_lock(&fusb251->fusb251_io_mutex);

	fusb251_i2c_read(fusb251, reg, 1, &reg_val);

	reg_val &= ~(mask << shift);
	reg_val |= (value << shift);

	buf[0] = reg;
	buf[1] = reg_val;
	fusb251_i2c_write(fusb251, buf, sizeof(buf));

	mutex_unlock(&fusb251->fusb251_io_mutex);
}

static int fusb251_mos_det_R_convert_reg(int value)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(mos_det_R_table); i++) {
		if (mos_det_R_table[i] == value)
			return i;
	}

	return -1;
}

static int fusb251_sbuft_MV_convert_reg(int value)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(sbuft_det_MV_table); i++) {
		if (sbuft_det_MV_table[i] == value)
			return i;
	}

	return -1;
}

static int fusb251_init(struct fusb251 *fusb251);
static ssize_t reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	cancel_delayed_work_sync(&fusb251->routine_work);

	fusb251_write_reg(fusb251, FUSB251_RESET, RESET_DEVICE);
	mdelay(10);
	fusb251_init(fusb251);

	switch_set_state(&fusb251->st_switch, TYPE_DRY);

	queue_delayed_work(system_freezable_wq, &fusb251->routine_work, 0);

	pr_info("%s: reset device and ld_switch\n", __func__);

	return size;
}

static ssize_t sbu_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBU_DET_MASK, EN_SBU_DET_SHIFT, true);
		pr_info("%s: enable sbu detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBU_DET_MASK, EN_SBU_DET_SHIFT, false);
		pr_info("%s: disable sbu detection\n", __func__);
	}

	return size;
}

static ssize_t sbu_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_SBU_DET_MASK, EN_SBU_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t sbuft_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBUFT_DET_MASK, EN_SBUFT_DET_SHIFT, true);
		pr_info("%s: enable sbu detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBUFT_DET_MASK, EN_SBUFT_DET_SHIFT, false);
		pr_info("%s: disable sbu detection\n", __func__);
	}

	return size;
}

static ssize_t sbuft_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_SBUFT_DET_MASK, EN_SBUFT_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t status_look4sbu_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_STATUS,
			STATUS_LOOK4SBU_MASK, STATUS_LOOK4SBU_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t mos_status_sbu1_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->mos_status_sbu1 ? 1 : 0);
}

static ssize_t mos_status_sbu2_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->mos_status_sbu2 ? 1 : 0);
}

static ssize_t mos_status_sbu1ft_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->mos_status_sbu1_ft ? 1 : 0);
}

static ssize_t mos_status_sbu2ft_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->mos_status_sbu2_ft ? 1 : 0);
}

static ssize_t threshold1_sbu_dry2wet_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {
		ret = fusb251_mos_det_R_convert_reg(temp);
		if (ret >= 0) {
			fusb251->threshold_sbu[THRESHOLD_DRY2WET] = temp;

			pr_info("%s: set threshold1_sbu_dry2wet to %dk\n",
					 __func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold1_sbu_dry2wet_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->threshold_sbu[THRESHOLD_DRY2WET]);
}

static ssize_t threshold1_sbu_wet2dry_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {
		ret = fusb251_mos_det_R_convert_reg(temp);
		if (ret >= 0) {
			fusb251->threshold_sbu[THRESHOLD_WET2DRY] = temp;

			pr_info("%s: set threshold1_sbu to %dk\n",
					 __func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold1_sbu_wet2dry_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->threshold_sbu[THRESHOLD_WET2DRY]);
}

static ssize_t threshold2_sbuft_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_VOLTAGE) {
		ret = fusb251_sbuft_MV_convert_reg(temp);
		if (ret >= 0) {
			fusb251->threshold_sbuft = temp;
			fusb251_config_interface(fusb251, FUSB251_THRESHOLD2,
					SBU_FLOAT_DET_MASK, SBU_FLOAT_DET_SHIFT, ret);

			pr_info("%s: set threshold2_sbuft to %dmV\n",
					__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold2_sbuft_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", fusb251->threshold_sbuft);
}

static ssize_t interval_dry_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0) {
		fusb251->work_interval[INTERVAL_DRY] = temp;

		pr_info("%s: set dry work interval to %d\n", __func__, temp);

		return size;
	}

	return -EINVAL;
}

static ssize_t interval_dry_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->work_interval[INTERVAL_DRY]);
}

static ssize_t interval_wet_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0) {
		fusb251->work_interval[INTERVAL_WET] = temp;

		pr_info("%s: set dry work interval to %d\n", __func__, temp);

		return size;
	}

	return -EINVAL;
}

static ssize_t interval_wet_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->work_interval[INTERVAL_WET]);
}

static ssize_t interval_wet_vbus_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0) {
		fusb251->work_interval[INTERVAL_VBUS] = temp;

		pr_info("%s: set dry work interval to %d\n", __func__, temp);

		return size;
	}

	return -EINVAL;
}

static ssize_t interval_wet_vbus_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->work_interval[INTERVAL_VBUS]);
}

static ssize_t regdump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;
	unsigned char temp = 0;
	char temp_info[200] = "";
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	for (i = 1 ; i <= FUSB251_REG_NUM; i++) {
		fusb251_read_reg(fusb251, i, &temp);
		scnprintf(temp_info, PAGE_SIZE, "reg[0x%02x]=0x%02x\n",
				i, temp);
		strncat(buf, temp_info, strlen(temp_info));
	}

	return strlen(buf);
}

DEVICE_ATTR(reset, 0220, NULL, reset_store);
DEVICE_ATTR(sbu_det, 0664, sbu_det_show, sbu_det_store);
DEVICE_ATTR(sbuft_det, 0664, sbuft_det_show, sbuft_det_store);
DEVICE_ATTR(status_look4sbu, 0444, status_look4sbu_show, NULL);
DEVICE_ATTR(mosstatus_sbu1, 0444, mos_status_sbu1_show, NULL);
DEVICE_ATTR(mosstatus_sbu2, 0444, mos_status_sbu2_show, NULL);
DEVICE_ATTR(mosstatus_sbu1ft, 0444, mos_status_sbu1ft_show, NULL);
DEVICE_ATTR(mosstatus_sbu2ft, 0444, mos_status_sbu2ft_show, NULL);
DEVICE_ATTR(threshold_sbu_dry2wet, 0664, threshold1_sbu_dry2wet_show,
		threshold1_sbu_dry2wet_store);
DEVICE_ATTR(threshold_sbu_wet2dry, 0664, threshold1_sbu_wet2dry_show,
		threshold1_sbu_wet2dry_store);
DEVICE_ATTR(threshold_sbuft, 0664, threshold2_sbuft_show,
		threshold2_sbuft_store);
DEVICE_ATTR(interval_dry, 0664, interval_dry_show, interval_dry_store);
DEVICE_ATTR(interval_wet, 0664, interval_wet_show, interval_wet_store);
DEVICE_ATTR(interval_wet_vbus, 0664, interval_wet_vbus_show,
		interval_wet_vbus_store);
DEVICE_ATTR(regdump, 0444, regdump_show, NULL);


static int fusb251_create_attr(struct device *cdev)
{
	int ret = 0;

	ret = device_create_file(cdev, &dev_attr_reset);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_reset\n", __func__);
		goto err_attr_reset;
	}

	ret = device_create_file(cdev, &dev_attr_sbu_det);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_sbu_det\n", __func__);
		goto err_attr_sbu_det;
	}

	ret = device_create_file(cdev, &dev_attr_sbuft_det);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_sbuft_det\n", __func__);
		goto err_attr_sbuft_det;
	}

	ret = device_create_file(cdev, &dev_attr_status_look4sbu);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_status_look4sbu\n", __func__);
		goto err_attr_status_look4sbu;
	}

	ret = device_create_file(cdev, &dev_attr_mosstatus_sbu1);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_mosstatus_sbu1\n", __func__);
		goto err_attr_mosstatus_sbu1;
	}

	ret = device_create_file(cdev, &dev_attr_mosstatus_sbu2);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_mosstatus_sbu2\n", __func__);
		goto err_attr_mosstatus_sbu2;
	}

	ret = device_create_file(cdev, &dev_attr_mosstatus_sbu1ft);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_mosstatus_sbu1ft\n", __func__);
		goto err_attr_mosstatus_sbu1ft;
	}

	ret = device_create_file(cdev, &dev_attr_mosstatus_sbu2ft);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_mosstatus_sbu2ft\n", __func__);
		goto err_attr_mosstatus_sbu2ft;
	}

	ret = device_create_file(cdev, &dev_attr_threshold_sbu_dry2wet);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold_sbu_dry2wet\n", __func__);
		goto err_attr_threshold_sbu_dry2wet;
	}

	ret = device_create_file(cdev, &dev_attr_threshold_sbu_wet2dry);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold_sbu_wet2dry\n", __func__);
		goto err_attr_threshold_sbu_wet2dry;
	}

	ret = device_create_file(cdev, &dev_attr_threshold_sbuft);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold_sbuft\n", __func__);
		goto err_attr_threshold_sbuft;
	}

	ret = device_create_file(cdev, &dev_attr_interval_dry);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_interval_dry\n", __func__);
		goto err_attr_interval_dry;
	}

	ret = device_create_file(cdev, &dev_attr_interval_wet);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_interval_wet\n", __func__);
		goto err_attr_interval_wet;
	}

	ret = device_create_file(cdev, &dev_attr_interval_wet_vbus);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_interval_wet_vbus\n", __func__);
		goto err_attr_interval_wet_vbus;
	}

	ret = device_create_file(cdev, &dev_attr_regdump);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_regdump\n", __func__);
		goto err_attr_regdump;
	}

	return ret;

err_attr_regdump:
	device_remove_file(cdev, &dev_attr_interval_wet_vbus);
err_attr_interval_wet_vbus:
	device_remove_file(cdev, &dev_attr_interval_wet);
err_attr_interval_wet:
	device_remove_file(cdev, &dev_attr_interval_dry);
err_attr_interval_dry:
	device_remove_file(cdev, &dev_attr_threshold_sbuft);
err_attr_threshold_sbuft:
	device_remove_file(cdev, &dev_attr_threshold_sbu_wet2dry);
err_attr_threshold_sbu_wet2dry:
	device_remove_file(cdev, &dev_attr_threshold_sbu_dry2wet);
err_attr_threshold_sbu_dry2wet:
	device_remove_file(cdev, &dev_attr_mosstatus_sbu2ft);
err_attr_mosstatus_sbu2ft:
	device_remove_file(cdev, &dev_attr_mosstatus_sbu1ft);
err_attr_mosstatus_sbu1ft:
	device_remove_file(cdev, &dev_attr_mosstatus_sbu2);
err_attr_mosstatus_sbu2:
	device_remove_file(cdev, &dev_attr_mosstatus_sbu1);
err_attr_mosstatus_sbu1:
	device_remove_file(cdev, &dev_attr_status_look4sbu);
err_attr_status_look4sbu:
	device_remove_file(cdev, &dev_attr_sbuft_det);
err_attr_sbuft_det:
	device_remove_file(cdev, &dev_attr_sbu_det);
err_attr_sbu_det:
	device_remove_file(cdev, &dev_attr_reset);
err_attr_reset:
	return ret;
}

static void fusb251_delete_attr(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_regdump);
	device_remove_file(cdev, &dev_attr_interval_wet_vbus);
	device_remove_file(cdev, &dev_attr_interval_wet);
	device_remove_file(cdev, &dev_attr_interval_dry);
	device_remove_file(cdev, &dev_attr_threshold_sbuft);
	device_remove_file(cdev, &dev_attr_threshold_sbu_wet2dry);
	device_remove_file(cdev, &dev_attr_threshold_sbu_dry2wet);
	device_remove_file(cdev, &dev_attr_mosstatus_sbu2ft);
	device_remove_file(cdev, &dev_attr_mosstatus_sbu1ft);
	device_remove_file(cdev, &dev_attr_mosstatus_sbu2);
	device_remove_file(cdev, &dev_attr_mosstatus_sbu1);
	device_remove_file(cdev, &dev_attr_status_look4sbu);
	device_remove_file(cdev, &dev_attr_sbuft_det);
	device_remove_file(cdev, &dev_attr_sbu_det);
	device_remove_file(cdev, &dev_attr_reset);
}

static void fusb251_regdump(struct fusb251 *fusb251)
{
	int i = 0;
	unsigned char temp = 0;

	for (i = 1 ; i <= FUSB251_REG_NUM; i++) {
		fusb251_read_reg(fusb251, i, &temp);
		pr_debug("%s: reg[0x%02x] = 0x%02x\n", __func__, i, temp);
	}
}

static void fusb251_report_event(struct fusb251 *fusb251, int event)
{
	int duration_sec = 0;
	struct timespec now_ts;
	int pre_event = switch_get_state(&fusb251->st_switch);

	if (pre_event != event) {
		switch_set_state(&fusb251->st_switch, event);
		pr_info("%s: event change %d -> %d", __func__, pre_event, event);

		get_monotonic_boottime(&now_ts);

		/*
		* To avoid duration_sec number overflow on metrics service,
		* "duration_sec" report 0 when event change from EVENT_DRY.
		*/
		if (pre_event == TYPE_DRY)
			duration_sec = 0;
		else
			duration_sec = now_ts.tv_sec - fusb251->event_ts.tv_sec;
#ifdef CONFIG_AMAZON_METRICS_LOG
		fusb251_metrics_log("LiquidDetection",
				"LiquidDetection:def:ld_current_state=%d;CT;1,ld_previous_state=%d;CT;1,ld_duration_sec=%d;CT;1:NR",
				event, pre_event, duration_sec);
#endif

#ifdef CONFIG_AMAZON_MINERVA_METRICS_LOG
		minerva_metrics_log(g_m_buf, METRICS_BUFF_SIZE_FUSB251,
				"%s:%s:100:%s,%s,%s,ld_current_state=%d;IN,ld_previous_state=%d;IN,"
				"ld_duration_sec=%d;IN:us-east-1",
				METRICS_LD_GROUP_ID, METRICS_LD_SCHEMA_ID, PREDEFINED_ESSENTIAL_KEY,
				PREDEFINED_MODEL_KEY, PREDEFINED_TZ_KEY, event, pre_event, duration_sec);
#endif

		memcpy(&fusb251->event_ts, &now_ts, sizeof(struct timespec));
	} else {
		if (event == TYPE_DRY)
			pr_info("%s: dry\n", __func__);
		else if (event == TYPE_WET)
			pr_info("%s: wet\n", __func__);
		else
			pr_info("%s: wet_vbus\n", __func__);
	}
}

static bool fusb251_check_vbus_valid(struct fusb251 *fusb251)
{
	union power_supply_propval val;
	int ret = 0;
	bool is_valid = false;

	if (!fusb251->usb_psy) {
		fusb251->usb_psy = power_supply_get_by_name("usb");
		if (!fusb251->usb_psy) {
			pr_err("%s: not find usb_psy\n", __func__);
			return false;
		}
	}

	ret = power_supply_get_property(fusb251->usb_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0) {
		pr_err("%s: get voltage_now failed, ret = %d\n", __func__, ret);
		return false;
	}

	if (val.intval > INVALID_VBUS_UV)
		is_valid = true;

	return is_valid;
}

void fusb251_vbus_changed(void)
{
	int state;

	if (!g_fusb251)
		return;

	state = switch_get_state(&g_fusb251->st_switch);
	if (fusb251_check_vbus_valid(g_fusb251)) {
		if (state == TYPE_WET)
			fusb251_report_event(g_fusb251, TYPE_WET_VBUS);
	} else {
		if (state == TYPE_WET_VBUS)
			fusb251_report_event(g_fusb251, TYPE_WET);
	}
}

static int fusb251_limit_charging_current(struct fusb251 *fusb251, bool en)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!fusb251->bat_psy) {
		fusb251->bat_psy = power_supply_get_by_name("battery");
		if (!fusb251->bat_psy) {
			pr_err("%s: not find bat_psy\n", __func__);
			return ret;
		}
	}

	propval.intval = en ? IUSB_LIMITATION_UA : -1;
	ret = power_supply_set_property(fusb251->bat_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &propval);
	if (ret < 0)
		pr_err("%s: get psy online failed, ret = %d\n", __func__, ret);

	return ret;
}

static inline int fusb251_select_interval(struct fusb251 *fusb251, int state)
{
	if (state >= 0 && state < ARRAY_SIZE(fusb251->work_interval)) {
		return fusb251->work_interval[state];
	} else {
		pr_err("FUSB251_STATE_TYPE doesn't match work_interval,"
				"use work_interval[INTERVAL_WET] as the default\n");
		return fusb251->work_interval[INTERVAL_WET];
	}
}

static void fusb251_routine_work(struct work_struct *work)
{
	int state = 0;
	int interval = 0;
	struct fusb251 *fusb251 =
			container_of(work, struct fusb251, routine_work.work);

	__pm_stay_awake(&fusb251->wake_lock);

	if (unlikely(system_state < SYSTEM_RUNNING)) {
		interval = RECHECK_DELAY_SEC;
		goto recheck;
	}

	fusb251_read_interface(fusb251, FUSB251_STATUS,
			STATUS_OVP_CC_MASK, STATUS_OVP_CC_SHIFT,
			&fusb251->status_ovp_cc);
	fusb251_read_interface(fusb251, FUSB251_STATUS,
			STATUS_OVP_SBU_MASK, STATUS_OVP_SBU_SHIFT,
			&fusb251->status_ovp_sbu);

	state = switch_get_state(&fusb251->st_switch);

	if (state == TYPE_DRY) {
		if (fusb251->status_ovp_cc || fusb251->status_ovp_sbu) {
			pr_info("%s Line%d: OVP detected\n", __func__, __LINE__);
			fusb251_report_event(fusb251, TYPE_WET_VBUS);
			tcpm_typec_change_role(fusb251->tcpc, TYPEC_ROLE_SNK);
			fusb251_limit_charging_current(fusb251, true);
			goto skip;
		}

		fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_DISABLE_ALL);
		fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
				SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT,
				fusb251_mos_det_R_convert_reg(
				fusb251->threshold_sbu[THRESHOLD_DRY2WET]));
		fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_EN_SBUFT_DET);

		msleep(fusb251->det_delay[DET_DELAY_SBUFT]);

		fusb251_read_interface(fusb251, FUSB251_MOSSTATUS,
				MOSSTATUS_SBU1FT_MASK, MOSSTATUS_SBU1FT_SHIFT,
				&fusb251->mos_status_sbu1_ft);
		fusb251_read_interface(fusb251, FUSB251_MOSSTATUS,
				MOSSTATUS_SBU2FT_MASK, MOSSTATUS_SBU2FT_SHIFT,
				&fusb251->mos_status_sbu2_ft);

		pr_info("%s Line%d: mos_status_sbuft[%d %d]\n", __func__, __LINE__,
				fusb251->mos_status_sbu1_ft,
				fusb251->mos_status_sbu2_ft);

		if (fusb251->mos_status_sbu1_ft || fusb251->mos_status_sbu2_ft) {
			if (fusb251_check_vbus_valid(fusb251))
				fusb251_report_event(fusb251, TYPE_WET_VBUS);
			else
				fusb251_report_event(fusb251, TYPE_WET);

			tcpm_typec_change_role(fusb251->tcpc, TYPEC_ROLE_SNK);
			fusb251_limit_charging_current(fusb251, true);
		} else {
			fusb251_report_event(fusb251, TYPE_DRY);
		}
	} else {
		/* step1: check ovp */
		if (fusb251->status_ovp_cc || fusb251->status_ovp_sbu) {
			pr_info("%s Line%d: OVP detected\n", __func__, __LINE__);
			fusb251_report_event(fusb251, TYPE_WET_VBUS);
			goto skip;
		}

		/* step2: check sbu float */
		fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_DISABLE_ALL);
		fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_EN_SBUFT_DET);

		msleep(fusb251->det_delay[DET_DELAY_SBUFT]);

		fusb251_read_interface(fusb251, FUSB251_MOSSTATUS,
				MOSSTATUS_SBU1FT_MASK, MOSSTATUS_SBU1FT_SHIFT,
				&fusb251->mos_status_sbu1_ft);
		fusb251_read_interface(fusb251, FUSB251_MOSSTATUS,
				MOSSTATUS_SBU2FT_MASK, MOSSTATUS_SBU2FT_SHIFT,
				&fusb251->mos_status_sbu2_ft);

		pr_info("%s Line%d: mos_status_sbuft[%d %d]\n", __func__, __LINE__,
				fusb251->mos_status_sbu1_ft,
				fusb251->mos_status_sbu2_ft);

		if (fusb251->mos_status_sbu1_ft || fusb251->mos_status_sbu2_ft) {
			if (fusb251_check_vbus_valid(fusb251) && (state != TYPE_WET_VBUS))
				fusb251_report_event(fusb251, TYPE_WET_VBUS);
			else
				fusb251_report_event(fusb251, state);

			goto skip;
		}

		/* step3: check sbu */
		fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_DISABLE_ALL);
		fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
				SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT,
				fusb251_mos_det_R_convert_reg(
						fusb251->threshold_sbu[THRESHOLD_WET2DRY]));
		fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_EN_SBU_DET);

		msleep(fusb251->det_delay[DET_DELAY_SBU]);

		fusb251_read_interface(fusb251, FUSB251_MOSSTATUS,
				MOSSTATUS_SBU1MOS_MASK, MOSSTATUS_SBU1MOS_SHIFT,
				&fusb251->mos_status_sbu1);
		fusb251_read_interface(fusb251, FUSB251_MOSSTATUS,
				MOSSTATUS_SBU2MOS_MASK, MOSSTATUS_SBU2MOS_SHIFT,
				&fusb251->mos_status_sbu2);

		pr_info("%s Line%d: mos_status_sbu[%d %d]\n", __func__, __LINE__,
				fusb251->mos_status_sbu1,
				fusb251->mos_status_sbu2);

		if (!(fusb251->mos_status_sbu1 || fusb251->mos_status_sbu2)) {
			fusb251_report_event(fusb251, TYPE_DRY);
			tcpm_typec_change_role(fusb251->tcpc, TYPEC_ROLE_TRY_SNK);
			fusb251_limit_charging_current(fusb251, false);
		} else {
			if (fusb251_check_vbus_valid(fusb251) && (state != TYPE_WET_VBUS))
				fusb251_report_event(fusb251, TYPE_WET_VBUS);
			else
				fusb251_report_event(fusb251, state);
		}
	}

skip:
	/* close cc and sbu */
	fusb251_write_reg(fusb251, FUSB251_CONTROL, CTL_DISABLE_ALL);

	interval = fusb251_select_interval(fusb251, state);
recheck:
	queue_delayed_work(system_freezable_wq, &fusb251->routine_work,
		interval * HZ);

	__pm_relax(&fusb251->wake_lock);
}

static void fusb251_parse_dt(struct fusb251 *fusb251)
{
	struct device_node *np = fusb251->dev->of_node;
	int ret = 0;
	int det_delay[2] = {0};
	int threshold_sbu[2] = {0};
	int work_interval[3] = {0};

	ret = of_property_read_u32_array(np, "threshold_sbu", threshold_sbu,
			ARRAY_SIZE(threshold_sbu));
	if (ret < 0) {
		fusb251->threshold_sbu[THRESHOLD_DRY2WET] = THRESHOLD_SBU_DRY2WET;
		fusb251->threshold_sbu[THRESHOLD_WET2DRY] = THRESHOLD_SBU_WET2DRY;
		pr_err("%s: not define threshold_sbu, use"
				"THRESHOLD_SBU_DRY2WET / THRESHOLD_SBU_WET2DRY\n", __func__);
	} else {
		memcpy(&fusb251->threshold_sbu, threshold_sbu, sizeof(threshold_sbu));
	}

	ret = of_property_read_u32(np, "threshold_sbuft", &fusb251->threshold_sbuft);
	if (ret < 0) {
		pr_err("%s: not define threshold_sbuft, use THRESHOLD_SBUFT\n",
			__func__);
		fusb251->threshold_sbuft = THRESHOLD_SBUFT;
	}

	ret = of_property_read_u32_array(np, "work_interval", work_interval,
			ARRAY_SIZE(work_interval));
	if (ret < 0) {
		fusb251->work_interval[INTERVAL_DRY] = WORK_INTERVAL_DRY;
		fusb251->work_interval[INTERVAL_WET] = WORK_INTERVAL_WET;
		fusb251->work_interval[INTERVAL_VBUS] = WORK_INTERVAL_VBUS;
		pr_err("%s: not define work_interval, use WORK_INTERVAL_DRY /"
				"WORK_INTERVAL_WET / WORK_INTERVAL_VBUS\n", __func__);
	} else {
		memcpy(&fusb251->work_interval, work_interval, sizeof(work_interval));
	}

	ret = of_property_read_u32_array(np, "det_delay", det_delay,
			ARRAY_SIZE(det_delay));
	if (ret < 0) {
		fusb251->det_delay[DET_DELAY_SBUFT] = SBUFT_DET_DELAY;
		fusb251->det_delay[DET_DELAY_SBU] = SBU_DET_DELAY;
		pr_err("%s: not define work_interval, use"
				"SBUFT_DET_DELAY / SBU_DET_DELAY\n", __func__);
	} else {
		memcpy(&fusb251->det_delay, det_delay, sizeof(det_delay));
	}

	pr_info("%s: threshold_sbu[%d %d], threshold_sbuft[%d], "
			"work_interval[%d %d %d], sbu_det_delay[%d %d]\n", __func__,
			fusb251->threshold_sbu[THRESHOLD_DRY2WET],
			fusb251->threshold_sbu[THRESHOLD_WET2DRY],
			fusb251->threshold_sbuft,
			fusb251->work_interval[INTERVAL_DRY],
			fusb251->work_interval[INTERVAL_WET],
			fusb251->work_interval[INTERVAL_VBUS],
			fusb251->det_delay[DET_DELAY_SBUFT],
			fusb251->det_delay[DET_DELAY_SBU]);
}

static int fusb251_init(struct fusb251 *fusb251)
{
	pr_info("%s enter\n", __func__);

	/* reset moisture */
	if (fusb251_write_reg(fusb251, FUSB251_RESET,
			RESET_MOSSTURE) < 0) {
		pr_err("%s: reset moisture failed\n", __func__);
		return -1;
	}
	/* mask all the interrupt */
	if (fusb251_write_reg(fusb251, FUSB251_INT_MASK,
			MSK_I_MASK_ALL) < 0) {
		pr_err("%s: mask all interrupt failed\n", __func__);
		return -1;
	}

	/* set moisture detection threshold */
	fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
			SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT,
			fusb251_mos_det_R_convert_reg(
					fusb251->threshold_sbu[THRESHOLD_DRY2WET]));
	/* set sbu float detection threshold */
	fusb251_config_interface(fusb251, FUSB251_THRESHOLD2,
			SBU_FLOAT_DET_MASK, SBU_FLOAT_DET_SHIFT,
			fusb251_sbuft_MV_convert_reg(fusb251->threshold_sbuft));

	fusb251_regdump(fusb251);

	return 0;
}

static int fusb251_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	unsigned char id_val = 0;
	struct fusb251 *fusb251 = NULL;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: check functionality failed\n", __func__);
		return -ENODEV;
	}

	fusb251 = kzalloc(sizeof(*fusb251), GFP_KERNEL);
	if (!fusb251)
		return -ENOMEM;

	fusb251->i2c = client;
	fusb251->dev = &client->dev;
	i2c_set_clientdata(client, fusb251);

	mutex_init(&fusb251->fusb251_io_mutex);

	fusb251_read_reg(fusb251, FUSB251_PRODUCT_ID, &id_val);
	if (id_val != FUSB251_DEVICE_ID) {
		pr_err("%s: error device id %02x\n", __func__, id_val);
		ret = -ENODEV;
		goto err_id;
	}

	fusb251_parse_dt(fusb251);

	fusb251->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!fusb251->tcpc) {
		pr_err("%s: type_c_port0 is not ready yet\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_tcpc;
	}

	ret = fusb251_init(fusb251);
	if (ret < 0)
		goto err_init;

	fusb251->st_switch.name = "ld";
	fusb251->st_switch.index = 0;
	fusb251->st_switch.state = TYPE_DRY;
	ret = switch_dev_register(&fusb251->st_switch);
	if (ret) {
		pr_err("%s: switch_dev_register failed\n", __func__);
		goto err_switch;
	}

	ret = fusb251_create_attr(fusb251->dev);
	if (ret < 0)
		goto err_attr;

	g_fusb251 = fusb251;

	get_monotonic_boottime(&fusb251->event_ts);

	wakeup_source_init(&fusb251->wake_lock, "fusb251");
	INIT_DELAYED_WORK(&fusb251->routine_work, fusb251_routine_work);
	queue_delayed_work(system_freezable_wq, &fusb251->routine_work, 0);

	pr_info("%s: probe successfully\n", __func__);

	return 0;

err_attr:
	switch_dev_unregister(&fusb251->st_switch);
err_switch:
err_init:
err_tcpc:
err_id:
	kfree(fusb251);
	fusb251 = NULL;

	pr_err("%s: probe failed\n", __func__);

	return ret;
}

static int fusb251_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static void fusb251_i2c_shutdown(struct i2c_client *client)
{
	struct fusb251 *fusb251 =
		 (struct fusb251 *)i2c_get_clientdata(client);

	cancel_delayed_work_sync(&fusb251->routine_work);
	fusb251_delete_attr(fusb251->dev);
	kfree(fusb251);
	fusb251 = NULL;
	g_fusb251 = NULL;
}

static const struct i2c_device_id fusb251_i2c_id[] = {
	{"fusb251", 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id of_fusb251_match_table[] = {
	{.compatible = "on,fusb251", },
	{},
};
MODULE_DEVICE_TABLE(of, of_fusb251_match_table);
#endif

static struct i2c_driver fusb251_i2c_driver = {
	.driver = {
		.name = "fusb251",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_fusb251_match_table),
	},
	.probe =    fusb251_i2c_probe,
	.remove =   fusb251_i2c_remove,
	.shutdown = fusb251_i2c_shutdown,
	.id_table = fusb251_i2c_id,
};

static int __init fusb251_i2c_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&fusb251_i2c_driver);
	if (ret != 0)
		pr_err("%s: i2c init failed!\n", __func__);

	return ret;
}
module_init(fusb251_i2c_init);

static void __exit fusb251_i2c_exit(void)
{
	i2c_del_driver(&fusb251_i2c_driver);
}
module_exit(fusb251_i2c_exit);

MODULE_AUTHOR("Shine Yin");
MODULE_DESCRIPTION("TYPEC FUSB251 driver");
MODULE_LICENSE("GPL v2");
