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
#include <mtk_charger_intf.h>
#include "max20342.h"

#define I2C_RETRIES                 2
/* 5ms delay */
#define I2C_RETRY_DELAY             5

#define SW_RESET_RETRY              2
#define SW_RESET_RETRY_DELAY        10

#define result_is_dry               true
#define result_is_wet               false

static struct max20342 *g_max20342;

/* For metrics */
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#define METRICS_BUFF_SIZE_MAX20342 512
char g_max_buf_max20342[METRICS_BUFF_SIZE_MAX20342];

#define max20342_metrics_log(domain, fmt, ...) \
do { \
	memset(g_max_buf_max20342, 0, METRICS_BUFF_SIZE_MAX20342); \
	snprintf(g_max_buf_max20342, sizeof(g_max_buf_max20342), fmt, ##__VA_ARGS__); \
	log_to_metrics(ANDROID_LOG_INFO, domain, g_max_buf_max20342); \
} while (0)
#else
static inline void max20342_metrics_log(void) {}
#endif

static int max20342_i2c_read(struct max20342 *max20342,
	 unsigned char reg, int len, unsigned char value[])
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

	if (!max20342) {
		pr_err("[%s] No device is available\n", __func__);
		return -1;
	}
	client = max20342->i2c;
	msgs[0].addr = client->addr;
	msgs[1].addr = client->addr;

	do {
		err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		pr_err("[%s] i2c read error, err_value: %d\n", __func__, err);
		error = -1;
	}

	return error;
}

static int max20342_i2c_write(struct max20342 *max20342,
	 char *writebuf, int writelen)
{
	int ret = 0;
	struct i2c_client *client;

	if (!max20342) {
		pr_err("[%s] No device is available\n", __func__);
		return -1;
	}
	client = max20342->i2c;

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
			pr_err("[%s] i2c write error,ret_value:%d\n",
				 __func__, ret);
	}

	return ret;
}

static int max20342_read_reg(struct max20342 *max20342,
	 unsigned char addr, unsigned char *value)
{
	int ret = 0;

	if (!max20342) {
		pr_err("[%s] No device is available\n", __func__);
		return -1;
	}
	mutex_lock(&max20342->max20342_io_mutex);
	ret = max20342_i2c_read(max20342, addr, 1, value);
	mutex_unlock(&max20342->max20342_io_mutex);

	return ret;
}

static int max20342_write_reg(struct max20342 *max20342,
	 unsigned char addr, unsigned char value)
{
	int ret = 0;
	unsigned char buf[2] = {0};

	if (!max20342) {
		pr_err("[%s] No device is available\n", __func__);
		return -1;
	}
	mutex_lock(&max20342->max20342_io_mutex);
	buf[0] = addr;
	buf[1] = value;
	ret = max20342_i2c_write(max20342, buf, sizeof(buf));
	mutex_unlock(&max20342->max20342_io_mutex);

	return ret;
}

static void max20342_read_interface(struct max20342 *max20342,
	 unsigned char reg, unsigned char mask,
	 unsigned char shift, unsigned char *value)
{
	unsigned char reg_val = 0;

	if (!max20342) {
		pr_err("[%s] No device is available\n", __func__);
		return;
	}
	mutex_lock(&max20342->max20342_io_mutex);
	max20342_i2c_read(max20342, reg, 1, &reg_val);
	reg_val &= (mask << shift);
	*value = (reg_val >> shift);
	mutex_unlock(&max20342->max20342_io_mutex);
}

static void max20342_config_interface(struct max20342 *max20342,
	 unsigned char reg, unsigned char mask,
	 unsigned char shift, int value)
{
	unsigned char reg_val = 0;
	unsigned char buf[2] = {0};

	if (!max20342) {
		pr_err("[%s] No device is available\n", __func__);
		return;
	}
	mutex_lock(&max20342->max20342_io_mutex);
	max20342_i2c_read(max20342, reg, 1, &reg_val);
	reg_val &= ~(mask << shift);
	reg_val |= (value << shift);
	buf[0] = reg;
	buf[1] = reg_val;
	max20342_i2c_write(max20342, buf, sizeof(buf));
	mutex_unlock(&max20342->max20342_io_mutex);
}

static int max20342_init(struct max20342 *max20342);

static ssize_t target_resister_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->target_resister);
}

static ssize_t target_ipu_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->target_ipu);
}

static ssize_t target_ipu_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;
	max20342->target_ipu = temp;

	return size;
}

static ssize_t target_voltage_min_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->target_voltage_min);
}

static ssize_t target_voltage_min_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;
	max20342->target_voltage_min = temp;

	return size;
}

static ssize_t target_voltage_max_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->target_voltage_max);
}

static ssize_t target_voltage_max_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;
	max20342->target_voltage_max = temp;

	return size;
}

static ssize_t ipu_result1_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->ipu_result1);
}

static ssize_t ipu_result2_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->ipu_result2);
}

static ssize_t adc_result_avg1_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->adc_result_avg1);
}

static ssize_t adc_result_avg2_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 max20342->adc_result_avg2);
}

static ssize_t regdump_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	int i = 0;
	unsigned char temp = 0;
	char temp_info[200] = "";
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	for (i = 0 ; i < sizeof(dump_regs); i++) {
		max20342_read_reg(max20342, dump_regs[i], &temp);
		scnprintf(temp_info, PAGE_SIZE, "reg[0x%02x]=0x%02x\n",
			 dump_regs[i], temp);
		strncat(buf, temp_info, strlen(temp_info));
	}

	return strlen(buf);
}

static ssize_t reset_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max20342 *max20342 =
		(struct max20342 *)i2c_get_clientdata(client);

	cancel_delayed_work_sync(&max20342->routine_work);
	max20342_write_reg(max20342, COMM_CTRL3, SW_RESET_VALUE_TRUE);
	mdelay(10);
	max20342_init(max20342);
	switch_set_state(&max20342->st_switch, TYPE_DRY);
	queue_delayed_work(system_freezable_wq, &max20342->routine_work, 0);
	pr_info("[%s] reset device and switch\n", __func__);

	return size;
}

DEVICE_ATTR(target_resister, 0444, target_resister_show, NULL);
DEVICE_ATTR(target_ipu, 0664, target_ipu_show, target_ipu_store);
DEVICE_ATTR(target_voltage_min, 0664, target_voltage_min_show,
	 target_voltage_min_store);
DEVICE_ATTR(target_voltage_max, 0664, target_voltage_max_show,
	 target_voltage_max_store);
DEVICE_ATTR(ipu_result1, 0444, ipu_result1_show, NULL);
DEVICE_ATTR(ipu_result2, 0444, ipu_result2_show, NULL);
DEVICE_ATTR(adc_result_avg1, 0444, adc_result_avg1_show, NULL);
DEVICE_ATTR(adc_result_avg2, 0444, adc_result_avg2_show, NULL);
DEVICE_ATTR(regdump_liquid, 0444, regdump_show, NULL);
DEVICE_ATTR(resetdev, 0220, NULL, reset_store);

static int max20342_create_attr(struct device *cdev)
{
	int ret = 0;

	ret = device_create_file(cdev, &dev_attr_target_resister);
	if (ret < 0) {
		pr_err("%s: failed to create target_resister\n", __func__);
		goto err_dev_attr_target_resister;
	}

	ret = device_create_file(cdev, &dev_attr_target_ipu);
	if (ret < 0) {
		pr_err("%s: failed to create target_ipu\n", __func__);
		goto err_dev_attr_target_ipu;
	}

	ret = device_create_file(cdev, &dev_attr_target_voltage_min);
	if (ret < 0) {
		pr_err("%s: failed to create target_voltage_min\n", __func__);
		goto err_dev_attr_target_voltage_min;
	}

	ret = device_create_file(cdev, &dev_attr_target_voltage_max);
	if (ret < 0) {
		pr_err("%s: failed to create target_voltage_max\n", __func__);
		goto err_dev_attr_target_voltage_max;
	}

	ret = device_create_file(cdev, &dev_attr_ipu_result1);
	if (ret < 0) {
		pr_err("%s: failed to create ipu_result1\n", __func__);
		goto err_dev_attr_ipu_result1;
	}

	ret = device_create_file(cdev, &dev_attr_ipu_result2);
	if (ret < 0) {
		pr_err("%s: failed to create ipu_result2\n", __func__);
		goto err_dev_attr_ipu_result2;
	}

	ret = device_create_file(cdev, &dev_attr_adc_result_avg1);
	if (ret < 0) {
		pr_err("%s: failed to create adc_result_avg1\n", __func__);
		goto err_dev_attr_adc_result_avg1;
	}

	ret = device_create_file(cdev, &dev_attr_adc_result_avg2);
	if (ret < 0) {
		pr_err("%s: failed to create adc_result_avg2\n", __func__);
		goto err_dev_attr_adc_result_avg2;
	}

	ret = device_create_file(cdev, &dev_attr_regdump_liquid);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_regdump\n", __func__);
		goto err_dev_attr_regdump_liquid;
	}

	ret = device_create_file(cdev, &dev_attr_resetdev);
	if (ret < 0) {
		pr_err("[%s] failed to creat dev_attr_reset\n", __func__);
		goto err_dev_attr_resetdev;
	}

	return ret;

err_dev_attr_resetdev:
	device_remove_file(cdev, &dev_attr_resetdev);
err_dev_attr_regdump_liquid:
	device_remove_file(cdev, &dev_attr_regdump_liquid);
err_dev_attr_adc_result_avg2:
	device_remove_file(cdev, &dev_attr_adc_result_avg2);
err_dev_attr_adc_result_avg1:
	device_remove_file(cdev, &dev_attr_adc_result_avg1);
err_dev_attr_ipu_result2:
	device_remove_file(cdev, &dev_attr_ipu_result2);
err_dev_attr_ipu_result1:
	device_remove_file(cdev, &dev_attr_ipu_result1);
err_dev_attr_target_voltage_max:
	device_remove_file(cdev, &dev_attr_target_voltage_max);
err_dev_attr_target_voltage_min:
	device_remove_file(cdev, &dev_attr_target_voltage_min);
err_dev_attr_target_ipu:
	device_remove_file(cdev, &dev_attr_target_ipu);
err_dev_attr_target_resister:
	device_remove_file(cdev, &dev_attr_target_resister);

	return ret;
}

static void max20342_delete_attr(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_target_resister);
	device_remove_file(cdev, &dev_attr_target_ipu);
	device_remove_file(cdev, &dev_attr_target_voltage_min);
	device_remove_file(cdev, &dev_attr_target_voltage_max);
	device_remove_file(cdev, &dev_attr_ipu_result1);
	device_remove_file(cdev, &dev_attr_ipu_result2);
	device_remove_file(cdev, &dev_attr_adc_result_avg1);
	device_remove_file(cdev, &dev_attr_adc_result_avg2);
	device_remove_file(cdev, &dev_attr_regdump_liquid);
	device_remove_file(cdev, &dev_attr_resetdev);
}

static void max20342_regdump(struct max20342 *max20342)
{
	int i = 0;
	unsigned char temp = 0;

	for (i = 0 ; i < sizeof(dump_regs); i++) {
		max20342_read_reg(max20342, dump_regs[i], &temp);
		pr_debug("%s: reg[0x%02x] = 0x%02x\n", __func__,
			 dump_regs[i], temp);
	}
}

static void max20342_report_event(struct max20342 *max20342, int event)
{
	int duration_sec = 0;
	struct timespec now_ts;
	int pre_event = switch_get_state(&max20342->st_switch);

	if (pre_event != event) {
		switch_set_state(&max20342->st_switch, event);
		pr_info("[%s] event changed %d ->%d", __func__, pre_event,
			 event);
		get_monotonic_boottime(&now_ts);

		if (pre_event == TYPE_DRY)
			duration_sec = 0;
		else
			duration_sec = now_ts.tv_sec - max20342->event_ts.tv_sec;
#ifdef CONFIG_AMAZON_METRICS_LOG
		max20342_metrics_log("Liquid_Detection", "Liquid_Detection:\
			def:current_state=%d;CT;1,previous_state=%d;CT;1,\
			duration_sec=%d;CT;1:NR", event, pre_event, duration_sec);
#endif

#ifdef CONFIG_AMAZON_MINERVA_METRICS_LOG
		minerva_metrics_log(g_max_buf_max20342, METRICS_BUFF_SIZE_MAX20342,
				"%s:%s:100:%s,%s,%s,ld_current_state=%d;IN,ld_previous_state=%d;IN,ld_duration_sec=%d;IN:us-east-1",
				METRICS_LD_GROUP_ID, METRICS_LD_SCHEMA_ID, PREDEFINED_ESSENTIAL_KEY,
				PREDEFINED_MODEL_KEY, PREDEFINED_TZ_KEY, event, pre_event, duration_sec);
#endif

		memcpy(&max20342->event_ts, &now_ts, sizeof(struct timespec));
	} else {
		if (event == TYPE_DRY)
			pr_info("[%s] dry\n", __func__);
		else if (event == TYPE_WET)
			pr_info("[%s] wet\n", __func__);
		else
			pr_info("[%s] wet_vbus\n", __func__);
	}
}

static bool max20342_check_vbus_valid(struct max20342 *max20342)
{
	union power_supply_propval val;
	int ret = 0;
	bool is_valid = false;

	if (!max20342->usb_psy) {
		max20342->usb_psy = power_supply_get_by_name("usb");
		if (!max20342->usb_psy) {
			pr_err("[%s] not find usb_psy\n", __func__);
			return false;
		}
	}

	ret = power_supply_get_property(max20342->usb_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0) {
		pr_err("[%s] get voltage_now failed, ret = %d\n", __func__, ret);
		return false;
	}
	if (val.intval > INVALID_VBUS_UV)
		is_valid = true;

	return is_valid;
}

static int max20342_limit_charging_current(struct max20342 *max20342, bool en)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!max20342->bat_psy) {
		max20342->bat_psy = power_supply_get_by_name("battery");
		if (!max20342->bat_psy) {
			pr_err("[%s] not find bat_psy\n", __func__);
			return ret;
		}
	}

	propval.intval = en ? IUSB_LIMITATION_UA : -1;
	ret = power_supply_set_property(max20342->bat_psy,
		POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &propval);
	if (ret < 0)
		pr_err("[%s] get psy online failed, ret = %d\n", __func__, ret);

	return ret;
}

static inline int max20342_select_interval(struct max20342 *max20342, int state)
{
	if (state >= 0 && state < ARRAY_SIZE(max20342->work_interval))
		return max20342->work_interval[state];
	else {
		pr_err("MAX20342_STATE_TYPE doesn't match work_interval,"
			" use work_interval[INTERVAL_WET] as the default\n");
		return max20342->work_interval[INTERVAL_WET];
	}
}

static void max20342_sbu_is_wet(struct max20342 *max20342, int cur_state)
{
	if (cur_state == TYPE_DRY) {
		tcpm_typec_change_role(max20342->tcpc, TYPEC_ROLE_SNK);
		max20342_limit_charging_current(max20342, true);

		if (max20342_check_vbus_valid(max20342))
			max20342_report_event(max20342, TYPE_WET_VBUS);
		else
			max20342_report_event(max20342, TYPE_WET);
	} else {
		if (max20342_check_vbus_valid(max20342)) {
			if (cur_state != TYPE_WET_VBUS)
				max20342_report_event(max20342, TYPE_WET_VBUS);
		} else {
			if (cur_state != TYPE_WET)
				max20342_report_event(max20342, TYPE_WET);
		}
	}

	pr_info("[%s]detect_result: HAVE_WATER adc_result_avg1:%d\
		 adc_result_avg2:%d TARGET_IPU:%d\n", __func__,
		 max20342->adc_result_avg1, max20342->adc_result_avg2,
		 max20342->target_ipu);
}

static void max20342_sbu_is_dry(struct max20342 *max20342, int cur_state)
{
	if (cur_state != TYPE_DRY) {
		max20342_report_event(max20342, TYPE_DRY);
		tcpm_typec_change_role(max20342->tcpc, TYPEC_ROLE_TRY_SNK);
		max20342_limit_charging_current(max20342, false);
	}
	pr_info("[%s]detect_result: NO WATER", __func__);
}

static void max20342_start_detection(struct max20342 *max20342)
{
	/* open mos */
	pinctrl_select_state(max20342->mos_pinctrl, max20342->mos_ctrl1_high);

	/* check sbu1 */
	max20342_config_interface(max20342, MoistDetPUConfig, MOIST_PULL_UP_SWITCH_MASK,
		 MOIST_PULL_UP_SWITCH_SHIFT, MOIST_PULL_UP_SWITCH_SBU1);
	max20342_config_interface(max20342, MoistDetPDConfig, MOIST_PULL_DOWN_SWITCH_MASK,
		 MOIST_PULL_DOWN_SWITCH_SHIFT, MOIST_PULL_DOWN_SWITCH_SBU2);
	max20342_config_interface(max20342, MoistDetCtrl, MOIST_DET_MAN_EN_MASK,
		 MOIST_DET_MAN_EN_SHIFT, MOIST_DET_MAN_EN_VAL_TRUE);
	msleep(max20342->det_delay[DET_DELAY_SBU1]);

	/* get detection result */
	max20342_read_interface(max20342, ADCCtrl2, IPU_RESULT_MASK,
		 IPU_RESULT_SHIFT, &max20342->ipu_result1);
	max20342_read_interface(max20342, ADCResultAvg, ADC_RESUIT_AVG_MASK,
		 ADC_RESUIT_AVG_SHIFT, &max20342->adc_result_avg1);

	/* check sbu2 */
	max20342_config_interface(max20342, MoistDetPUConfig, MOIST_PULL_UP_SWITCH_MASK,
		 MOIST_PULL_UP_SWITCH_SHIFT, MOIST_PULL_UP_SWITCH_SBU2);
	max20342_config_interface(max20342, MoistDetPDConfig, MOIST_PULL_DOWN_SWITCH_MASK,
		 MOIST_PULL_DOWN_SWITCH_SHIFT, MOIST_PULL_DOWN_SWITCH_SBU1);
	max20342_config_interface(max20342, MoistDetCtrl, MOIST_DET_MAN_EN_MASK,
		 MOIST_DET_MAN_EN_SHIFT, MOIST_DET_MAN_EN_VAL_TRUE);
	msleep(max20342->det_delay[DET_DELAY_SBU2]);

	/* get detection result */
	max20342_read_interface(max20342, ADCCtrl2, IPU_RESULT_MASK,
		 IPU_RESULT_SHIFT, &max20342->ipu_result2);
	max20342_read_interface(max20342, ADCResultAvg, ADC_RESUIT_AVG_MASK,
		 ADC_RESUIT_AVG_SHIFT, &max20342->adc_result_avg2);

	/* close mos */
	pinctrl_select_state(max20342->mos_pinctrl, max20342->mos_ctrl1_low);
}

static bool max20342_get_detection_result(struct max20342 *max20342, int state)
{
	/* determine if there is water, no water conditions: ADC value is in the
	   set range and ipu_result is equal to the set value */
	if (state == TYPE_DRY)
		max20342->target_voltage_min = max20342->ld_buffer[THRESHOLD_L];
	else
		max20342->target_voltage_min = max20342->ld_buffer[THRESHOLD_H];

	if ((max20342->ipu_result1 == max20342->target_ipu) &&
		 (max20342->ipu_result2 == max20342->target_ipu) &&
		 (max20342->adc_result_avg1 <= max20342->target_voltage_max) &&
		 (max20342->adc_result_avg1 >= max20342->target_voltage_min) &&
		 (max20342->adc_result_avg2 <= max20342->target_voltage_max) &&
		 (max20342->adc_result_avg2 >= max20342->target_voltage_min))
		return result_is_dry;
	else
		return result_is_wet;
}

static void max20342_stop_detection(struct max20342 *max20342)
{
	/* close sbu switch and pick delay time */
	max20342_config_interface(max20342, MoistDetPUConfig, MOIST_PULL_UP_SWITCH_MASK,
		 MOIST_PULL_UP_SWITCH_SHIFT, MOIST_PULL_UP_ALL_OPEN);
	max20342_config_interface(max20342, MoistDetPDConfig, MOIST_PULL_DOWN_SWITCH_MASK,
		 MOIST_PULL_DOWN_SWITCH_SHIFT, MOIST_PULL_DOWN_ALL_OPEN);
}

static void max20342_routine_work(struct work_struct *work)
{
	int state = 0;
	int interval = 0;

	struct max20342 *max20342 =
		 container_of(work, struct max20342, routine_work.work);

	__pm_stay_awake(&max20342->wake_lock);

	if (unlikely(system_state < SYSTEM_RUNNING)) {
		interval = RECHECK_DELAY_SEC;
		goto recheck;
	}

	state = switch_get_state(&max20342->st_switch);

	max20342_start_detection(max20342);

	if (max20342_get_detection_result(max20342, state) == result_is_dry)
		max20342_sbu_is_dry(max20342, state);
	else
		max20342_sbu_is_wet(max20342, state);

	max20342_stop_detection(max20342);

	interval = max20342_select_interval(max20342, state);

recheck:
	queue_delayed_work(system_freezable_wq, &max20342->routine_work, interval * HZ);
	__pm_relax(&max20342->wake_lock);
}

static void max20342_parse_dt(struct max20342 *max20342)
{
	struct device_node *np = max20342->dev->of_node;
	int ret = 0;
	int det_delay[2] = {0};
	int work_interval[3] = {0};
	unsigned char ld_buffer[2] = {0};
	unsigned char target_ipu = 0;
	unsigned char target_resister = 0;
	unsigned char target_voltage_max = 0;
	unsigned char target_adcgroundvth = 0;

	ret = of_property_read_u32_array(np, "work_interval", work_interval,
		ARRAY_SIZE(work_interval));
	if (ret < 0) {
		max20342->work_interval[INTERVAL_DRY] = WORK_INTERVAL_DRY;
		max20342->work_interval[INTERVAL_WET] = WORK_INTERVAL_WET;
		max20342->work_interval[INTERVAL_VBUS] = WORK_INTERVAL_VBUS;
		pr_err("[%s] read work_interval dts failed,"
			 "use default parameter\n", __func__);
	} else {
		memcpy(&max20342->work_interval, work_interval, sizeof(work_interval));
	}

	ret = of_property_read_u32_array(np, "det_delay", det_delay,
		ARRAY_SIZE(det_delay));
	if (ret < 0) {
		max20342->det_delay[DET_DELAY_SBU1] = SBU1_DET_DELAY_VAL;
		max20342->det_delay[DET_DELAY_SBU2] = SBU2_DET_DELAY_VAL;
		pr_err("[%s] read det_delay dts failed,"
			"use default parameter\n", __func__);
	} else {
		memcpy(&max20342->det_delay, det_delay, sizeof(det_delay));
	}

	ret = of_property_read_u8(np, "target_ipu", &target_ipu);
	if (ret < 0) {
		max20342->target_ipu = TARGET_IPU;
		pr_err("[%s] read det_delay dts failed,"
			"use default parameter\n", __func__);
	} else {
		memcpy(&max20342->target_ipu, &target_ipu, sizeof(target_ipu));
	}

	ret = of_property_read_u8(np, "target_resister", &target_resister);
	if (ret < 0) {
		max20342->target_resister = TARGET_RESISTER;
		pr_err("[%s] read det_delay dts failed,"
			"use default parameter\n", __func__);
	} else {
		memcpy(&max20342->target_resister, &target_resister,
			 sizeof(target_resister));
	}

	ret = of_property_read_u8_array(np, "ld_buffer", ld_buffer, ARRAY_SIZE(ld_buffer));
	if (ret < 0) {
		max20342->ld_buffer[THRESHOLD_L] = THRESHOLD_L_VAL;
		max20342->ld_buffer[THRESHOLD_L] = THRESHOLD_H_VAL;
		pr_err("[%s] read det_delay dts failed,"
			"use default parameter\n", __func__);
	} else {
		memcpy(&max20342->ld_buffer, ld_buffer, sizeof(ld_buffer));
	}

	ret = of_property_read_u8(np, "target_voltage_max", &target_voltage_max);
	if (ret < 0) {
		max20342->target_voltage_max = TARGET_VOLTAGE_MAX;
		pr_err("[%s] read det_delay dts failed,"
			"use default parameter\n", __func__);
	} else {
		memcpy(&max20342->target_voltage_max, &target_voltage_max,
			 sizeof(target_voltage_max));
	}

	ret = of_property_read_u8(np, "target_adcgroundvth", &target_adcgroundvth);
	if (ret < 0) {
		max20342->target_adcgroundvth = TARGET_ADCGROUNDVTH;
		pr_err("[%s] read det_delay dts failed,"
			"use default parameter\n", __func__);
	} else {
		memcpy(&max20342->target_adcgroundvth, &target_adcgroundvth,
			 sizeof(target_adcgroundvth));
	}

	pr_info("[%s] [work_interval:%d, %d, %d] [det_delay:%d, %d]"
		" [target_ipu:%d] [target_resister:%d] [ld_buf:%d, %d]"
		" [target_voltage_max:%d] [target_adcgroundvth:%d]\n", __func__,
		 max20342->work_interval[INTERVAL_DRY],
		 max20342->work_interval[INTERVAL_WET],
		 max20342->work_interval[INTERVAL_VBUS],
		 max20342->det_delay[DET_DELAY_SBU1],
		 max20342->det_delay[DET_DELAY_SBU2],
		 max20342->target_ipu,
		 max20342->target_resister,
		 max20342->ld_buffer[THRESHOLD_L],
		 max20342->ld_buffer[THRESHOLD_H],
		 max20342->target_voltage_max,
		 max20342->target_adcgroundvth);
}

static int max20342_init(struct max20342 *max20342)
{
	int i = 0;

	max20342->ipu_result1 = 0;
	max20342->ipu_result2 = 0;
	max20342->adc_result_avg1 = 0;
	max20342->adc_result_avg2 = 0;

	/* disable all mode */
	max20342_config_interface(max20342, MoistDetCtrl, MOIST_DET_MAN_EN_MASK,
		 MOIST_DET_MAN_EN_SHIFT, MOIST_DET_MAN_EN_VAL_FALSE);

	max20342_config_interface(max20342, MoistDetCtrl, MOIST_DET_AUTO_CFG_MASK,
		 MOIST_DET_AUTO_CFG_SHIFT, MOIST_DET_AUTO_CFG_VAL_FALSE);

	max20342_config_interface(max20342, MoistDetCtrl, MOIST_DET_PER_EN_MASK,
		 MOIST_DET_PER_EN_SHIFT, MOIST_DET_PER_EN_VAL_FALSE);

	/* keep USB and VB switch open */
	max20342_config_interface(max20342, MoistDetPUConfig, MOIST_PULL_UP_SWITCH_MASK,
		 MOIST_PULL_UP_SWITCH_SHIFT, MOIST_PULL_UP_ALL_OPEN);
	max20342_config_interface(max20342, MoistDetPDConfig, MOIST_PULL_DOWN_SWITCH_MASK,
		 MOIST_PULL_DOWN_SWITCH_SHIFT, MOIST_PULL_DOWN_ALL_OPEN);

	/* mask all interrupt */
	for (i = 0; i < sizeof(mask_int_reg); i++) {
		if (max20342_write_reg(max20342, mask_int_reg[i], MASK_ALL_INT_VALUE) < 0) {
			pr_err("[%s] mask all int failed\n", __func__);
			return -1;
		};
	}

	/* disable bc1.2 detection */
	max20342_config_interface(max20342, BC_CTRL1, CHGDETEN_MASK,
		 CHGDETEN_SHIFT, CHGDETEN_VAL_DISABLE);

	/* disable typec detection */
	max20342_config_interface(max20342, CC_CTRL3, FSM_ENABLE_MASK,
		 FSM_ENABLE_SHIFT, FSM_ENABLE_VAL_FALSE);

	/* pull up current */
	max20342_config_interface(max20342, MoistDetCtrl, PULL_UP_CURRENT_MASK,
		 PULL_UP_CURRENT_SHIFT, max20342->target_ipu);

	/* adc result vth */
	max20342_write_reg(max20342, RMoistDetVth, max20342->target_voltage_min);

	/* adc ground vth */
	max20342_config_interface(max20342, ADCCtrl1, ADC_GROUND_VTH_MASK,
		 ADC_GROUND_VTH_SHIFT, max20342->target_adcgroundvth);

	/* LPDRP mode */
	max20342_config_interface(max20342, COMM_CTRL1, LPDRD_MASK,
		 LPDRD_SHIFT, LPDRD_VAL);

	max20342_regdump(max20342);

	return 0;
}

static void max20342_vbus_detect(struct work_struct *work)
{
	int state;

	pr_info("%s enter\n", __func__);
	state = switch_get_state(&g_max20342->st_switch);
	if (max20342_check_vbus_valid(g_max20342)) {
		if (state == TYPE_WET)
			max20342_report_event(g_max20342, TYPE_WET_VBUS);
	} else {
		if (state == TYPE_WET_VBUS)
			max20342_report_event(g_max20342, TYPE_WET);
	}
}

static void max20342_vbus_changed(void)
{
	pr_info("%s enter\n", __func__);
	if (!g_max20342)
		return;

	cancel_delayed_work(&g_max20342->delayed_work);
	queue_delayed_work(system_freezable_wq, &g_max20342->delayed_work,
		 msecs_to_jiffies(WORK_DETECT_VBUS));
}

static int liquid_dev_event(struct notifier_block *nb, unsigned long event, void *v)
{
	switch (event) {
	case CHARGER_DEV_NOTIFY_VBUS_EVENT:
		max20342_vbus_changed();
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_DONE;
}

static int max20342_pinctrl_init(struct max20342 *max20342)
{
	struct pinctrl *pinctrl = NULL;

	pinctrl = devm_pinctrl_get(max20342->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: can't find pinctrl\n", __func__);
		goto out;
	}

	max20342->mos_pinctrl = pinctrl;

	max20342->mos_ctrl1_init = pinctrl_lookup_state(pinctrl, "mosctrl1_init");
	if (IS_ERR(max20342->mos_ctrl1_init)) {
		pr_err("%s: can't find mos_ctrl1_init\n", __func__);
		goto out;
	}

	max20342->mos_ctrl1_low = pinctrl_lookup_state(pinctrl, "mosctrl1_low");
	if (IS_ERR(max20342->mos_ctrl1_low)) {
		pr_err("%s: can't find mos_ctrl1_low\n", __func__);
		goto out;
	}

	max20342->mos_ctrl1_high = pinctrl_lookup_state(pinctrl, "mosctrl1_high");
	if (IS_ERR(max20342->mos_ctrl1_high)) {
		pr_err("%s: can't find mos_ctrl1_high\n", __func__);
		goto out;
	}

	pinctrl_select_state(max20342->mos_pinctrl, max20342->mos_ctrl1_init);

	return 0;

out:
	return -1;
}

static int max20342_i2c_probe(struct i2c_client *client,
	 const struct i2c_device_id *id)
{
	int ret = 0;
	unsigned char id_val = 0;
	struct max20342 *max20342 = NULL;

	pr_info("enter %s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s check functionality failed\n", __func__);
		return -ENODEV;
	}
	max20342 = kzalloc(sizeof(*max20342), GFP_KERNEL);
	if (!max20342)
		return -ENOMEM;
	max20342->i2c = client;
	max20342->dev = &client->dev;
	i2c_set_clientdata(client, max20342);
	mutex_init(&max20342->max20342_io_mutex);

	max20342_read_reg(max20342, REVISION_ID, &id_val);
	if (id_val != MAX20342_REVISION_ID) {
		pr_err("%s:error revision id %x\n", __func__, id_val);
		ret = -ENODEV;
		goto err_id;
	}

	max20342->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!max20342->tcpc) {
		pr_err("%s:type_c_port0 is not ready\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_tcpc;
	}

	max20342->chg1_dev = get_charger_by_name("primary_chg");
	if (max20342->chg1_dev)
		pr_info("%s:found primary charger\n", __func__);
	else {
		pr_err("*** %s:can't find primary charger ***\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_getcharge;
	}
	max20342->chg1_nb.notifier_call = liquid_dev_event;
	register_charger_device_notifier(max20342->chg1_dev, &max20342->chg1_nb);

	ret = max20342_pinctrl_init(max20342);
	if (ret) {
		goto err_pinctrl;
	}

	max20342_parse_dt(max20342);

	ret = max20342_init(max20342);
	if (ret < 0)
		goto err_init;

	max20342->st_switch.name = "ld";
	max20342->st_switch.index = 0;
	max20342->st_switch.state = TYPE_DRY;
	ret = switch_dev_register(&max20342->st_switch);
	if (ret) {
		pr_err("[%s] switch_dev_register failed\n", __func__);
		goto err_switch;
	}

	ret = max20342_create_attr(max20342->dev);
	if (ret < 0)
		goto err_attr;

	g_max20342 = max20342;
	get_monotonic_boottime(&max20342->event_ts);
	wakeup_source_init(&max20342->wake_lock, "max20342");
	INIT_DELAYED_WORK(&max20342->routine_work, max20342_routine_work);
	INIT_DELAYED_WORK(&g_max20342->delayed_work, max20342_vbus_detect);
	queue_delayed_work(system_freezable_wq, &max20342->routine_work, 0);
	pr_info("[%s] probe sucessfull\n", __func__);

	return 0;

err_attr:
	switch_dev_unregister(&max20342->st_switch);
err_switch:
err_init:
err_pinctrl:
	unregister_charger_device_notifier(max20342->chg1_dev, &max20342->chg1_nb);
err_getcharge:
err_tcpc:
err_id:
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&max20342->max20342_io_mutex);
	kfree(max20342);
	max20342 = NULL;
	pr_err("[%s] probe failed\n", __func__);

	return ret;
}

static int max20342_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static void max20342_i2c_shutdown(struct i2c_client *client)
{
	struct max20342 *max20342 =
		 (struct max20342 *)i2c_get_clientdata(client);

	cancel_delayed_work_sync(&g_max20342->routine_work);
	cancel_delayed_work_sync(&g_max20342->delayed_work);
	max20342_delete_attr(max20342->dev);
	kfree(max20342);
	max20342 = NULL;
	g_max20342 = NULL;
}

static const struct i2c_device_id max20342_i2c_id[] = {
	{"max20342", 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id of_max20342_match_table[] = {
	{.compatible = "max20342",},
	{},
};
MODULE_DEVICE_TABLE(of, of_max20342_match_table);
#endif

static struct i2c_driver max20342_i2c_driver = {
	.driver = {
		.name = "max20342",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_max20342_match_table),
	},
	.probe = max20342_i2c_probe,
	.remove = max20342_i2c_remove,
	.shutdown = max20342_i2c_shutdown,
	.id_table = max20342_i2c_id,
};

static int __init max20342_i2c_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&max20342_i2c_driver);
	if (ret != 0)
		pr_err("[%s] i2c init failed\n", __func__);

	return ret;
}
module_init(max20342_i2c_init);

static void __exit max20342_i2c_exit(void)
{
	i2c_del_driver(&max20342_i2c_driver);
}
module_exit(max20342_i2c_exit);

MODULE_AUTHOR("Junbo Pan");
MODULE_DESCRIPTION("LD_MAX20342_driver");
MODULE_LICENSE("GPL v2");
