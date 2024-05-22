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
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <tcpm.h>
#include <linux/iio/consumer.h>
#include "amzn_ld.h"
#include <mtk_charger_intf.h>
#ifdef CONFIG_PUBADC_MANAGE
#include <pubadc_manage.h>
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#include <linux/metricslog.h>
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#define METRICS_BUFF_SIZE_LIQUID 512
static char g_m_buf_liquid[METRICS_BUFF_SIZE_LIQUID];
#endif

static struct liquid *g_liquid;

static ssize_t sbu1_step1_val_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct liquid *liquid = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 liquid->adc_step_val.sbu1_step1_val);
}

static ssize_t sbu2_step1_val_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct liquid *liquid = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 liquid->adc_step_val.sbu2_step1_val);
}

static ssize_t sbu1_step2_val_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct liquid *liquid = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 liquid->adc_step_val.sbu1_step2_val);
}

static ssize_t sbu2_step2_val_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct liquid *liquid = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 liquid->adc_step_val.sbu2_step2_val);
}

static ssize_t threshod_l_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct liquid *liquid = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 liquid->threshold[THRESHOLD_L]);
}

static ssize_t threshod_l_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buff, size_t size)
{
	struct liquid *liquid = dev_get_drvdata(dev);
	int temp = 0;

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;
	liquid->threshold[THRESHOLD_L] = temp;

	return size;
}

static ssize_t threshod_h_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct liquid *liquid = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		 liquid->threshold[THRESHOLD_H]);
}

static ssize_t threshod_h_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buff, size_t size)
{
	struct liquid *liquid = dev_get_drvdata(dev);
	int temp = 0;

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;
	liquid->threshold[THRESHOLD_H] = temp;

	return size;
}

DEVICE_ATTR(sbu1_step1_val, 0444, sbu1_step1_val_show, NULL);
DEVICE_ATTR(sbu2_step1_val, 0444, sbu2_step1_val_show, NULL);
DEVICE_ATTR(sbu1_step2_val, 0444, sbu1_step2_val_show, NULL);
DEVICE_ATTR(sbu2_step2_val, 0444, sbu2_step2_val_show, NULL);
DEVICE_ATTR(threshod_l, 0664, threshod_l_show, threshod_l_store);
DEVICE_ATTR(threshod_h, 0664, threshod_h_show, threshod_h_store);

static int liquid_create_attr(struct device *cdev)
{
	int ret = 0;

	ret = device_create_file(cdev, &dev_attr_sbu1_step1_val);
	if (ret < 0) {
		pr_err("[%s] failed to creat sbu1_step1_val\n", __func__);
		goto err_dev_attr_sbu1_step1_val;
	}

	ret = device_create_file(cdev, &dev_attr_sbu2_step1_val);
	if (ret < 0) {
		pr_err("[%s] failed to creat sbu2_step1_val\n", __func__);
		goto err_dev_attr_sbu2_step1_val;
	}

	ret = device_create_file(cdev, &dev_attr_sbu1_step2_val);
	if (ret < 0) {
		pr_err("[%s] failed to creat sbu1_step2_val\n", __func__);
		goto err_dev_attr_sbu1_step2_val;
	}

	ret = device_create_file(cdev, &dev_attr_sbu2_step2_val);
	if (ret < 0) {
		pr_err("[%s] failed to creat sbu2_step2_val\n", __func__);
		goto err_dev_attr_sbu2_step2_val;
	}

	ret = device_create_file(cdev, &dev_attr_threshod_l);
	if (ret < 0) {
		pr_err("[%s] failed to creat threshod_l\n", __func__);
		goto err_dev_attr_threshod_l;
	}

	ret = device_create_file(cdev, &dev_attr_threshod_h);
	if (ret < 0) {
		pr_err("[%s] failed to creat threshod_h\n", __func__);
		goto err_dev_attr_threshod_h;
	}

	return ret;

err_dev_attr_threshod_h:
	device_remove_file(cdev, &dev_attr_threshod_h);
err_dev_attr_threshod_l:
	device_remove_file(cdev, &dev_attr_threshod_l);
err_dev_attr_sbu2_step2_val:
	device_remove_file(cdev, &dev_attr_sbu2_step2_val);
err_dev_attr_sbu1_step2_val:
	device_remove_file(cdev, &dev_attr_sbu1_step2_val);
err_dev_attr_sbu2_step1_val:
	device_remove_file(cdev, &dev_attr_sbu2_step1_val);
err_dev_attr_sbu1_step1_val:
	device_remove_file(cdev, &dev_attr_sbu1_step1_val);

	return ret;
}

static void liquid_delete_attr(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_sbu1_step1_val);
	device_remove_file(cdev, &dev_attr_sbu2_step1_val);
	device_remove_file(cdev, &dev_attr_sbu1_step2_val);
	device_remove_file(cdev, &dev_attr_sbu2_step2_val);
	device_remove_file(cdev, &dev_attr_threshod_l);
	device_remove_file(cdev, &dev_attr_threshod_h);
}

static void liquid_report_event(struct liquid *liquid, int event)
{
	int duration_sec = 0;
	struct timespec now_ts;
	int pre_event = switch_get_state(&liquid->st_switch);

	if (pre_event != event) {
		switch_set_state(&liquid->st_switch, event);
		pr_info("[%s] event changed %d ->%d", __func__, pre_event,
			 event);
		get_monotonic_boottime(&now_ts);

		if (pre_event == TYPE_DRY)
			duration_sec = 0;
		else
			duration_sec = now_ts.tv_sec - liquid->event_ts.tv_sec;
#ifdef CONFIG_AMAZON_METRICS_LOG
		memset(g_m_buf_liquid, 0, METRICS_BUFF_SIZE_LIQUID);
		snprintf(g_m_buf_liquid, sizeof(g_m_buf_liquid),
				"Liquid_Detection:def:current_state=%d;CT;1,"
				"previous_state=%d;CT;1,duration_sec=%d;CT;1:NR",
				event, pre_event, duration_sec);
		log_to_metrics(ANDROID_LOG_INFO, "Liquid_Detection", g_m_buf_liquid);
#endif

#ifdef CONFIG_AMAZON_MINERVA_METRICS_LOG
		minerva_metrics_log(g_m_buf_liquid, METRICS_BUFF_SIZE_LIQUID,
				"%s:%s:100:%s,%s,%s,ld_current_state=%d;IN,"
				"ld_previous_state=%d;IN,ld_duration_sec=%d;IN:us-east-1",
				METRICS_LD_GROUP_ID, METRICS_LD_SCHEMA_ID, PREDEFINED_ESSENTIAL_KEY,
				PREDEFINED_MODEL_KEY, PREDEFINED_TZ_KEY, event, pre_event, duration_sec);
#endif
		memcpy(&liquid->event_ts, &now_ts, sizeof(struct timespec));
	} else {
		if (event == TYPE_DRY)
			pr_info("[%s] dry\n", __func__);
		else if (event == TYPE_WET)
			pr_info("[%s] wet\n", __func__);
		else
			pr_info("[%s] wet_vbus\n", __func__);
	}
}

static bool liquid_check_vbus_valid(struct liquid *liquid)
{
	union power_supply_propval val;
	int ret = 0;
	bool is_valid = false;

	if (!liquid->usb_psy) {
		liquid->usb_psy = power_supply_get_by_name("usb");
		if (!liquid->usb_psy) {
			pr_err("[%s] not find usb_psy\n", __func__);
			return false;
		}
	}
	ret = power_supply_get_property(liquid->usb_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0) {
		pr_err("[%s] get voltage_now failed, ret = %d\n",
			 __func__, ret);
		return false;
	}
	if (val.intval > INVALID_VBUS_UV)
		is_valid = true;

	return is_valid;
}

static int liquid_limit_charging_current(struct liquid *liquid, bool en)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!liquid->bat_psy) {
		liquid->bat_psy = power_supply_get_by_name("battery");
		if (!liquid->bat_psy) {
			pr_err("[%s] not find bat_psy\n", __func__);
			return ret;
		}
	}
	propval.intval = en ? IUSB_LIMITATION_UA : -1;
	ret = power_supply_set_property(liquid->bat_psy,
		POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &propval);
	if (ret < 0)
		pr_err("[%s] get psy online failed, ret = %d\n", __func__, ret);

	return ret;
}

static inline int liquid_select_interval(struct liquid *liquid, int state)
{
	if (state >= 0 && state < ARRAY_SIZE(liquid->work_interval))
		return liquid->work_interval[state];

	pr_err("liquid_STATE_TYPE doesn't match work_interval,"
		" use work_interval[INTERVAL_WET] as the default\n");

	return liquid->work_interval[INTERVAL_WET];
}

static int liquid_get_auxadc_channel(struct platform_device *dev)
{
	int ret;
	struct liquid *liquid = platform_get_drvdata(dev);

	liquid->adc_ch = iio_channel_get(&dev->dev, "ld_adc_channel");
	if (IS_ERR(liquid->adc_ch)) {
		ret = PTR_ERR(liquid->adc_ch);
		pr_err("%s: IIO channel not found: %d\n", __func__, ret);
		return -1;
	}

	return 0;
}

static int liquid_get_auxadc_value(struct iio_channel *ch)
{
	int ret = 0;
	int auxadc_cali_mv = 0;

	if (ch == NULL) {
		pr_err("%s: adc channel is null\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_PUBADC_MANAGE
	ret = pubadc_iio_read_channel(ch, &auxadc_cali_mv, PUBADC_LIQUID);
#else
	ret = iio_read_channel_processed(ch, &auxadc_cali_mv);
#endif
	if (ret < 0) {
		pr_err("%s: IIO channel read failed %d\n", __func__, ret);
		return ret;
	}
	auxadc_cali_mv = auxadc_cali_mv * 1500 / 4096;

	return auxadc_cali_mv;
}

static void liquid_select_detection_switch(struct liquid *liquid,
	 enum LD_DETECTION_SWITCH switch_val)
{
	switch (switch_val) {
	case DETECTION_SBU1:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_ctrl1_high);
		break;
	case DETECTION_SBU2:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_ctrl1_low);
		break;
#ifndef CONFIG_PUBADC_MANAGE
	case DETECTION_LD:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_ctrl2_high);
		break;
	case DETECTION_BATID:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_ctrl2_low);
		break;
#endif
	default:
		break;
	}
}

static void liquid_en_pull_sbu(struct liquid *liquid,
	 enum LD_SBU_PULL pull_val)
{
	switch (pull_val) {
	case SBU1_PULL_UP:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_detection1_high);
		break;
	case SBU1_PULL_DOWN:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_detection1_low);
		break;
	case SBU2_PULL_UP:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_detection2_high);
		break;
	case SBU2_PULL_DOWN:
		pinctrl_select_state(liquid->ld_pinctrl,
			 liquid->ld_detection2_low);
		break;
	default:
		break;
	}
}

static inline bool liquid_is_in_range(int mv,
	 int range_l, int range_h)
{
	return (mv >= range_l) && (mv <= range_h);
}

static void liquid_get_step1_adcval(struct liquid *liquid)
{
	liquid_select_detection_switch(liquid, DETECTION_SBU1);
	msleep(STATE_CHANGE_MS);
	liquid->adc_step_val.sbu1_step1_val =
		 liquid_get_auxadc_value(liquid->adc_ch);

	liquid_select_detection_switch(liquid, DETECTION_SBU2);
	msleep(STATE_CHANGE_MS);
	liquid->adc_step_val.sbu2_step1_val =
		 liquid_get_auxadc_value(liquid->adc_ch);
}

static void liquid_get_step2_adcval(struct liquid *liquid)
{
	liquid_select_detection_switch(liquid, DETECTION_SBU1);
	if (liquid->gpio_reverse_ctrl)
		liquid_en_pull_sbu(liquid, SBU1_PULL_DOWN);
	else
		liquid_en_pull_sbu(liquid, SBU1_PULL_UP);
	msleep(STATE_CHANGE_MS);
	liquid->adc_step_val.sbu1_step2_val =
		 liquid_get_auxadc_value(liquid->adc_ch);
	if (liquid->gpio_reverse_ctrl)
		liquid_en_pull_sbu(liquid, SBU1_PULL_UP);
	else
		liquid_en_pull_sbu(liquid, SBU1_PULL_DOWN);

	liquid_select_detection_switch(liquid, DETECTION_SBU2);
	if (liquid->gpio_reverse_ctrl)
		liquid_en_pull_sbu(liquid, SBU2_PULL_DOWN);
	else
		liquid_en_pull_sbu(liquid, SBU2_PULL_UP);
	msleep(STATE_CHANGE_MS);
	liquid->adc_step_val.sbu2_step2_val =
		 liquid_get_auxadc_value(liquid->adc_ch);
	if (liquid->gpio_reverse_ctrl)
		liquid_en_pull_sbu(liquid, SBU2_PULL_UP);
	else
		liquid_en_pull_sbu(liquid, SBU2_PULL_DOWN);
}

static void liquid_is_dry(struct liquid *liquid)
{
	pr_info("%s: Liquid disappear.\n", __func__);
	liquid_report_event(liquid, TYPE_DRY);
	tcpm_typec_change_role(liquid->tcpc, TYPEC_ROLE_TRY_SNK);
	liquid_limit_charging_current(liquid, false);
}

static void liquid_is_wet(struct liquid *liquid)
{
	pr_info("%s: Liquid detected.\n", __func__);
	tcpm_typec_change_role(liquid->tcpc, TYPEC_ROLE_SNK);
	liquid_limit_charging_current(liquid, true);
	if (liquid_check_vbus_valid(liquid))
		liquid_report_event(liquid, TYPE_WET_VBUS);
	else
		liquid_report_event(liquid, TYPE_WET);
}

static void liquid_clear_sbuval(struct liquid *liquid,
	 unsigned int step1_val, unsigned int step2_val)
{
	liquid->adc_step_val.sbu1_step1_val = step1_val;
	liquid->adc_step_val.sbu2_step1_val = step1_val;
	liquid->adc_step_val.sbu1_step2_val = step2_val;
	liquid->adc_step_val.sbu2_step2_val = step2_val;
}

static void liquid_routine_work(struct work_struct *work)
{
	int state = 0;
	int interval = 0;
	int threshold_l, threshold_h;
	int adc1_mv, adc2_mv;
	int step1_sbu1_show = -1;
	int step1_sbu2_show = -1;
	int step2_sbu1_show = -1;
	int step2_sbu2_show = -1;
	struct liquid *liquid =
		 container_of(work, struct liquid, routine_work.work);

	pr_info("%s enter\n", __func__);
	__pm_stay_awake(&liquid->wake_lock);

	if (unlikely(system_state < SYSTEM_RUNNING)) {
		interval = RECHECK_DELAY_SEC;
		goto recheck;
	}
	state = switch_get_state(&liquid->st_switch);
	threshold_l = liquid->threshold[THRESHOLD_L];
	threshold_h = liquid->threshold[THRESHOLD_H];

	liquid_clear_sbuval(liquid, STEP1_DEFAULT_VAL, STEP2_DEFAULT_VAL);
	liquid_get_step1_adcval(liquid);
	adc1_mv = liquid->adc_step_val.sbu1_step1_val;
	step1_sbu1_show = adc1_mv;
	adc2_mv = liquid->adc_step_val.sbu2_step1_val;
	step1_sbu2_show = adc2_mv;

	if (state == TYPE_DRY) {
		/* Dry -> Wet */
		if ((liquid_is_in_range(adc1_mv, threshold_l, ADC_MAX_MV)) || (
			 liquid_is_in_range(adc2_mv, threshold_l, ADC_MAX_MV))) {
			msleep(DETECT_TWICE_MS);
			liquid_get_step1_adcval(liquid);
			adc1_mv = liquid->adc_step_val.sbu1_step1_val;
			step1_sbu1_show = adc1_mv;
			adc2_mv = liquid->adc_step_val.sbu2_step1_val;
			step1_sbu2_show = adc2_mv;
			if ((liquid_is_in_range(adc1_mv, threshold_l, ADC_MAX_MV)) || (
				 liquid_is_in_range(adc2_mv, threshold_l, ADC_MAX_MV))) {
				liquid_is_wet(liquid);
				goto skip;
			}
		}
		pr_info("%s: No liquid detected.\n", __func__);
	} else {
		/* Wet -> Wet */
		if (liquid_is_in_range(adc1_mv, threshold_l, ADC_MAX_MV) ||
			liquid_is_in_range(adc2_mv, threshold_l, ADC_MAX_MV)) {
			pr_info("%s: Liquid still on SBU.\n", __func__);
			goto skip;
		}
		pr_info("%s: Liquid detection WET to DRY step1 passed.\n", __func__);
		liquid_get_step2_adcval(liquid);
		adc1_mv = liquid->adc_step_val.sbu1_step2_val;
		step2_sbu1_show = adc1_mv;
		adc2_mv = liquid->adc_step_val.sbu2_step2_val;
		step2_sbu2_show = adc2_mv;
		/* Wet -> Dry */
		if (liquid_is_in_range(adc1_mv, threshold_h, ADC_MAX_MV) &&
			liquid_is_in_range(adc2_mv, threshold_h, ADC_MAX_MV))
			liquid_is_dry(liquid);
		else
			pr_info("%s: Liquid still on SBU.\n", __func__);
	}

skip:
	interval = liquid_select_interval(liquid, state);
	pr_info("%s: SBU show: Step1 result: SBU1:%d SBU2:%d Step2 result: SBU1:%d SBU2:%d.\n",
	 __func__, step1_sbu1_show, step1_sbu2_show, step2_sbu1_show, step2_sbu2_show);
recheck:
	queue_delayed_work(system_freezable_wq,
		 &liquid->routine_work, interval * HZ);
	__pm_relax(&liquid->wake_lock);
}

/*
 * HW description:
 *     switch:
 *         SWITCH_CTRL1:(MT8168-AD29-DMIC0_CLK)
 *             0: SBU2
 *             1: SBU1
 *         SWITCH_CTRL2(MT8168-AC29-DMIC0_DAT0)
 *             0:ADC2 <--> ADC_BAT_MUX
 *             1:ADC2 <--> SBU1/SBU2
 *         Lquid_Detection1(MT8168-R27-NRNB)
 *         Lquid_Detection2(MT8168-R28-NREB)
 *
 *     adc:ADC2_BAT_ID(MT8168-AB18-AUXIN2)
 *     liquid ID:GPIO83
 *          0:init driver
 *          1:not init driver
 */
static int liquid_pinctrl_init(struct platform_device *pdev)
{
	struct liquid *liquid = platform_get_drvdata(pdev);
	struct pinctrl *pinctrl = NULL;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: can't find pinctrl\n", __func__);
		goto out;
	}

	liquid->ld_pinctrl = pinctrl;
	liquid->ld_ctrl1_init = pinctrl_lookup_state(pinctrl, "ldctrl1_init");
	if (IS_ERR(liquid->ld_ctrl1_init)) {
		pr_err("%s: can't find ld_ctrl1_init\n", __func__);
		goto out;
	}

	liquid->ld_ctrl1_low = pinctrl_lookup_state(pinctrl, "ldctrl1_low");
	if (IS_ERR(liquid->ld_ctrl1_low)) {
		pr_err("%s: can't find ld_ctrl1_low\n", __func__);
		goto out;
	}

	liquid->ld_ctrl1_high = pinctrl_lookup_state(pinctrl, "ldctrl1_high");
	if (IS_ERR(liquid->ld_ctrl1_high)) {
		pr_err("%s: can't find ld_ctrl1_high\n", __func__);
		goto out;
	}

	liquid->ld_detection1_init = pinctrl_lookup_state(pinctrl,
		 "lddetection1_init");
	if (IS_ERR(liquid->ld_detection1_init)) {
		pr_err("%s: can't find ld_detection1_init\n", __func__);
		goto out;
	}

	liquid->ld_detection1_low = pinctrl_lookup_state(pinctrl,
		 "lddetection1_low");
	if (IS_ERR(liquid->ld_detection1_init)) {
		pr_err("%s: can't find ld_detection1_init\n", __func__);
		goto out;
	}

	liquid->ld_detection1_high = pinctrl_lookup_state(pinctrl,
		 "lddetection1_high");
	if (IS_ERR(liquid->ld_detection1_high)) {
		pr_err("%s: can't find ld_detection1_high\n", __func__);
		goto out;
	}

	liquid->ld_detection2_init = pinctrl_lookup_state(pinctrl,
		 "lddetection2_init");
	if (IS_ERR(liquid->ld_detection2_init)) {
		pr_err("%s: can't find ld_detection2_init\n", __func__);
		goto out;
	}

	liquid->ld_detection2_low = pinctrl_lookup_state(pinctrl,
		 "lddetection2_low");
	if (IS_ERR(liquid->ld_detection2_low)) {
		pr_err("%s: can't find ld_detection2_low\n", __func__);
		goto out;
	}

	liquid->ld_detection2_high = pinctrl_lookup_state(pinctrl,
		 "lddetection2_high");
	if (IS_ERR(liquid->ld_detection2_high)) {
		pr_err("%s: can't find ld_detection2_high\n", __func__);
		goto out;
	}

#ifndef CONFIG_PUBADC_MANAGE
	liquid->ld_ctrl2_init = pinctrl_lookup_state(pinctrl, "ldctrl2_init");
	if (IS_ERR(liquid->ld_ctrl2_init)) {
		pr_err("%s: can't find ld_ctrl2_init\n", __func__);
		goto out;
	}

	liquid->ld_ctrl2_low = pinctrl_lookup_state(pinctrl, "ldctrl2_low");
	if (IS_ERR(liquid->ld_ctrl2_low)) {
		pr_err("%s: can't find ld_ctrl2_low\n", __func__);
		goto out;
	}

	liquid->ld_ctrl2_high = pinctrl_lookup_state(pinctrl, "ldctrl2_high");
	if (IS_ERR(liquid->ld_ctrl2_high)) {
		pr_err("%s: can't find ld_ctrl2_high\n", __func__);
		goto out;
	}
	pinctrl_select_state(liquid->ld_pinctrl, liquid->ld_ctrl2_init);
#endif
	pinctrl_select_state(liquid->ld_pinctrl, liquid->ld_ctrl1_init);
	pinctrl_select_state(liquid->ld_pinctrl, liquid->ld_detection1_init);
	pinctrl_select_state(liquid->ld_pinctrl, liquid->ld_detection2_init);

	return 0;
out:
	return -1;
}

static void liquid_parse_dt(struct platform_device *pdev)
{
	struct liquid *liquid = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	unsigned int work_interval[3] = {0};
	unsigned int threshold[2] = {0};

	ret = of_property_read_u32_array(np, "work_interval", work_interval,
		ARRAY_SIZE(work_interval));
	if (ret < 0) {
		liquid->work_interval[INTERVAL_DRY] = WORK_INTERVAL_DRY;
		liquid->work_interval[INTERVAL_WET] = WORK_INTERVAL_WET;
		liquid->work_interval[INTERVAL_VBUS] = WORK_INTERVAL_VBUS;
		pr_err("[%s] read work_interval dts failed,"
			 "use default parameter\n", __func__);
	} else {
		memcpy(&liquid->work_interval, work_interval,
			 sizeof(work_interval));
	}

	ret = of_property_read_u32_array(np, "threshold", threshold,
		ARRAY_SIZE(threshold));
	if (ret < 0) {
		liquid->threshold[THRESHOLD_L] = THRESHOLD_VAL_L;
		liquid->threshold[THRESHOLD_H] = THRESHOLD_VAL_H;
		pr_err("[%s] read threshold dts failed,"
			 "use default parameter\n", __func__);
	} else {
		memcpy(&liquid->threshold, threshold, sizeof(threshold));
	}

	liquid->gpio_reverse_ctrl = of_property_read_bool(np,
		 "gpio_reverse_ctrl");

	pr_info("[%s] work_interval:%d, %d, %d threshold:%d %d"
		" gpio_reverse_ctrl:%d\n", __func__,
		liquid->work_interval[INTERVAL_DRY],
		liquid->work_interval[INTERVAL_WET],
		liquid->work_interval[INTERVAL_VBUS],
		liquid->threshold[THRESHOLD_L],
		liquid->threshold[THRESHOLD_H],
		liquid->gpio_reverse_ctrl);
}

static void liquid_vbus_detect(struct work_struct *work)
{
	int state;

	pr_info("%s enter\n", __func__);
	state = switch_get_state(&g_liquid->st_switch);
	if (liquid_check_vbus_valid(g_liquid)) {
		if (state == TYPE_WET)
			liquid_report_event(g_liquid, TYPE_WET_VBUS);
	} else {
		if (state == TYPE_WET_VBUS)
			liquid_report_event(g_liquid, TYPE_WET);
	}
}

static void liquid_vbus_changed(void)
{
	pr_info("%s enter\n", __func__);
	if (!g_liquid)
		return;

	cancel_delayed_work(&g_liquid->delayed_work);
	queue_delayed_work(system_freezable_wq,
		&g_liquid->delayed_work, msecs_to_jiffies(WORK_DETECT_VBUS));
}

static int liquid_get_id(struct platform_device *pdev)
{
	int ret = 0;
	int ret_val;
	struct liquid *liquid = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;

	liquid->gpio_liquid_id = of_get_named_gpio(np, "liquid-id", 0);
	if (!gpio_is_valid(liquid->gpio_liquid_id)) {
		pr_err("%s: no valid gpio\n", __func__);
		return 0;
	}

	ret = gpio_request(liquid->gpio_liquid_id, "ldid");
	if (ret) {
		pr_err("[%s] request liquid gpio failed ret = %d\n",
			 __func__, ret);
		return 0;
	}
	gpio_direction_input(liquid->gpio_liquid_id);
	ret_val = gpio_get_value(liquid->gpio_liquid_id);
	gpio_free(liquid->gpio_liquid_id);

	return ret_val;
}

bool liquid_check_id(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	unsigned int liquid_dts_id;
	int pubadc_io_id;

	ret = of_property_read_u32(np, "liquid_dts_id", &liquid_dts_id);
	if (ret < 0) {
		pr_err("[%s] can not find id by dts\n", __func__);
		return false;
	}

	pubadc_io_id = liquid_get_id(pdev);

	if (liquid_dts_id == pubadc_io_id)
		return true;

	pr_err("[%s] id not match, pubadc_dts_id = %d\
		 pubadc_io_id = %d\n", __func__, liquid_dts_id, pubadc_io_id);

	return false;
}

static int liquid_dev_event(struct notifier_block *nb,
	 unsigned long event, void *v)
{
	switch (event) {
	case CHARGER_DEV_NOTIFY_VBUS_EVENT:
		liquid_vbus_changed();
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static int liquid_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct liquid *liquid = NULL;

	pr_info("enter %s\n", __func__);

	liquid = kzalloc(sizeof(struct liquid), GFP_KERNEL);
	if (!liquid)
		return -ENOMEM;
	liquid->pdev = pdev;
	platform_set_drvdata(pdev, liquid);
	mutex_init(&liquid->ld_io_mutex);
#ifdef CONFIG_PUBADC_MANAGE
	if (!get_pubadc_init_state()) {
		pr_err("[%s] can not find pubadc_management\n", __func__);
		ret = -ENODEV;
		goto err_id;
	}
#else
	if (!liquid_check_id(pdev)) {
		ret = -ENODEV;
		goto err_id;
	}
#endif

	if (liquid_get_auxadc_channel(pdev)) {
		ret = -EPROBE_DEFER;
		goto err_adcchnnel;
	}

	liquid->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!liquid->tcpc) {
		pr_err("%s: type_c_port0 is not ready yet\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_tcpc;
	}

	liquid->chg1_dev = get_charger_by_name("primary_chg");
	if (liquid->chg1_dev)
		pr_info("%s:found primary charger\n", __func__);
	else {
		pr_err("*** %s:can't find primary charger ***\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_getcharge;
	}
	liquid->chg1_nb.notifier_call = liquid_dev_event;
	register_charger_device_notifier(liquid->chg1_dev,
		 &liquid->chg1_nb);

	ret = liquid_pinctrl_init(pdev);
	if (ret) {
		ret = -ENODEV;
		goto err_pinctrl;
	}

	liquid_parse_dt(pdev);

	liquid->st_switch.name = "ld";
	liquid->st_switch.index = 0;
	liquid->st_switch.state = TYPE_DRY;
	ret = switch_dev_register(&liquid->st_switch);
	if (ret) {
		pr_err("%s: switch_dev_register fail: %d\n", __func__, ret);
		goto err_switch;
	}

	ret = liquid_create_attr(&pdev->dev);
	if (ret < 0)
		goto err_attr;

	g_liquid = liquid;
	get_monotonic_boottime(&liquid->event_ts);
	wakeup_source_init(&liquid->wake_lock, "ld");
	INIT_DELAYED_WORK(&liquid->routine_work, liquid_routine_work);
	INIT_DELAYED_WORK(&g_liquid->delayed_work, liquid_vbus_detect);
	queue_delayed_work(system_freezable_wq, &liquid->routine_work, 0);

	pr_info("[%s] probe sucessfull\n", __func__);
	return 0;

err_attr:
	switch_dev_unregister(&liquid->st_switch);
err_switch:
err_pinctrl:
	unregister_charger_device_notifier(liquid->chg1_dev, &liquid->chg1_nb);
err_getcharge:
err_tcpc:
err_adcchnnel:
err_id:
	mutex_destroy(&liquid->ld_io_mutex);
	if (liquid)
		kfree(liquid);
	liquid = NULL;
	pr_err("[%s] probe failed\n", __func__);
	return ret;
}

static int liquid_remove(struct platform_device *pdev)
{
	return 0;
}

static void liquid_shutdown(struct platform_device *pdev)
{
	struct liquid *liquid = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&g_liquid->routine_work);
	cancel_delayed_work_sync(&g_liquid->delayed_work);
	liquid_delete_attr(&pdev->dev);
	kfree(liquid);
	liquid = NULL;
	g_liquid = NULL;
}

static const struct platform_device_id match_id[] = {
	{"amzn,ld", 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id liquid_dt_match[] = {
	{.compatible = "amzn,ld",},
	{},
};
MODULE_DEVICE_TABLE(of, liquid_dt_match);
#endif

struct platform_driver liquid_driver = {
	.driver = {
		.name = "amzn_ld",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(liquid_dt_match),
	},
	.probe = liquid_probe,
	.remove = liquid_remove,
	.shutdown = liquid_shutdown,
	.id_table = match_id,
};

static int __init liquid_init(void)
{
	return platform_driver_register(&liquid_driver);
}
late_initcall(liquid_init);

static void __exit liquid_exit(void)
{
	platform_driver_unregister(&liquid_driver);
}
module_exit(liquid_exit);

MODULE_AUTHOR("Junbo Pan");
MODULE_DESCRIPTION("LD_separate_driver");
MODULE_LICENSE("GPL v2");
