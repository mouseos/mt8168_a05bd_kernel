/*
 *  Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/leds.h>

#include "../../flashlight/richtek/rtfled.h"
#include "inc/mt6370_pmu.h"
#include "inc/mt6370_pmu_bled.h"
#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
#include <mt-plat/mtk_boot_common.h>
#endif
#include <linux/leds.h>
#include <linux/debugfs.h>

struct mt6370_pmu_bled_data {
	struct rt_fled_dev base;
	struct mt6370_pmu_chip *chip;
	struct device *dev;
	struct platform_device *mt_flash_dev;
};

static uint8_t bled_init_data[] = {
	0x42, /* MT6370_PMU_REG_BLEN */
	0x89, /* MT6370_PMU_REG_BLBSTCTRL */
	0x00, /* MT6370_PMU_REG_BLPWM */
	0x00, /* MT6370_PMU_REG_BLCTRL */
	0x00, /* MT6370_PMU_REG_BLDIM2 */
	0x00, /* MT6370_PMU_REG_BLDIM1 */
	0x00, /* MT6370_PMU_REG_BLAFH */
	0x00, /* MT6370_PMU_REG_BLFL */
	0x8c, /* MT6370_PMU_REG_BLFLTO */
	0x80, /* MT6370_PMU_REG_BLTORCTRL */
	0xff, /* MT6370_PMU_REG_BLSTRBCTRL */
	0x00, /* MT6370_PMU_REG_BLAVG */
};

/* define int limit for brightness limitation */
static struct led_classdev mt6370_pmu_bled_dev;

int led_setMaxbrightness(int max_level, int enable)
{
	struct led_classdev *led_cdev = &mt6370_pmu_bled_dev;

	if ((max_level < 0) || (max_level > LED_FULL)) {
		dev_err(led_cdev->dev, "%s invalid parameter %d.", __func__, max_level);
		return -1;
	}
	if (enable == 1) {
		led_cdev->max_brightness = max_level;
		dev_warn(led_cdev->dev, "[warning][%s] current brightness:%d, max_brightness:%d",
					__func__, led_cdev->brightness, led_cdev->max_brightness);
		led_set_brightness(led_cdev, led_cdev->brightness);
	} else {
		led_cdev->max_brightness = LED_FULL;
	}
	return 0;
}
EXPORT_SYMBOL(led_setMaxbrightness);

#ifdef CONFIG_LEDS_BRIGHTNESS_NORMALIZE
#define CUST_MAX_PANEL_NUM (8)

struct norm_parm_item {
	unsigned int lcm_id;
	unsigned int alg_base;
	unsigned int alg_scale;
	unsigned int leds_def;
};

static unsigned int leds_normalize_enable;
static unsigned int leds_chosen_id;
static struct norm_parm_item leds_chosen_param;

int led_parse_normalize_dts(const struct device_node *led_node)
{
	struct norm_parm_item leds_norm_table[CUST_MAX_PANEL_NUM];
	unsigned int leds_norm_cnt;
	int i, ret;

	if (!led_node) {
		pr_err("%s: Invalid DTS node!\n", __func__);
		return -1;
	}
	ret = of_property_read_u32(led_node, "brightness-normalize-enable",
			&leds_normalize_enable);
	if (ret) {
		pr_err("DTS not found LED normalize enable!\n");
		goto err_exit;
	}
	ret = of_property_read_u32(led_node, "brightness-normalize-count",
			&leds_norm_cnt);
	if (ret) {
		pr_err("DTS not found LED panel count!\n");
		goto err_exit;
	}
	if (leds_norm_cnt > CUST_MAX_PANEL_NUM) {
		pr_err("Invalid panel number %d/%d!\n",
				leds_norm_cnt,
				CUST_MAX_PANEL_NUM);
		goto err_exit;
	}
	ret = of_property_read_u32_array(led_node, "brightness-normalize-table",
			(u32 *)leds_norm_table,
			leds_norm_cnt*sizeof(struct norm_parm_item)/sizeof(u32));
	if (ret) {
		pr_err("DTS not found LED normalize table!\n");
		goto err_exit;
	}
	ret = of_property_read_u32(led_node, "chosen-id", &leds_chosen_id);
	if (ret) {
		pr_err("DTS not found LED lcm-id(%d)!\n", leds_chosen_id);
		goto err_exit;
	}
	for (i = 0; i < leds_norm_cnt; i++) {
		if (leds_norm_table[i].lcm_id == leds_chosen_id) {
			leds_chosen_param = leds_norm_table[i];
			break;
		}
	}
	if (i >= leds_norm_cnt) {
		pr_err("No matched normalization params found on panel(%d), ignore.\n",
				leds_chosen_id);
		goto err_exit;
	}
	pr_info("LED normalize params: panel(%d) enable(%d) ratio(%d/%d) def(%d)\n",
			leds_chosen_id,
			leds_normalize_enable,
			leds_chosen_param.alg_base,
			leds_chosen_param.alg_scale,
			leds_chosen_param.leds_def);
	return 0;

err_exit:
	leds_normalize_enable = 0;
	pr_err("%s failed, force to turn off normalization.\n", __func__);
	return -1;
}

unsigned int led_normalize_brightness_level(unsigned int level)
{

	if (!leds_normalize_enable)
		return level;
	if (leds_chosen_param.alg_scale == 0 || leds_chosen_param.alg_base == 0) {
		pr_err("Invalid brightness normalization params (%d,%d) on panel(%d), ignore.\n",
				leds_chosen_param.alg_base,
				leds_chosen_param.alg_scale,
				leds_chosen_id);
		return level;
	}
	if (level > 0) {
		level = level * leds_chosen_param.alg_base / leds_chosen_param.alg_scale;
		if (level == 0)
			level = 1;
	}
	return level;
}

static int led_normalize_enable_set(void *data, u64 val)
{
	leds_normalize_enable = (val > 0) ? 1:0;
	return 0;
}

static int led_normalize_enable_get(void *data, u64 *val)
{
	*val = leds_normalize_enable;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(led_normalize_enable_fops,
		led_normalize_enable_get, led_normalize_enable_set, "%llu\n");
#endif

static int mt6370_bled_fled_set_mode(struct rt_fled_dev *fled_dev,
	enum flashlight_mode mode)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;
	int ret = 0;

	switch (mode) {
	case FLASHLIGHT_MODE_OFF:
		ret = mt6370_pmu_reg_update_bits(bled_data->chip,
				MT6370_PMU_REG_BLFL, MT6370_BLFLMODE_MASK,
				0 << MT6370_BLFLMODE_SHFT);
		break;
	case FLASHLIGHT_MODE_TORCH:
		ret = mt6370_pmu_reg_update_bits(bled_data->chip,
				MT6370_PMU_REG_BLFL, MT6370_BLFLMODE_MASK,
				2 << MT6370_BLFLMODE_SHFT);
		break;
	case FLASHLIGHT_MODE_FLASH:
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mt6370_bled_fled_get_mode(struct rt_fled_dev *fled_dev)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;
	int ret = 0;

	ret = mt6370_pmu_reg_read(bled_data->chip, MT6370_PMU_REG_BLFL);
	if (ret < 0)
		return ret;
	ret &= MT6370_BLFLMODE_MASK;
	ret >>= MT6370_BLFLMODE_SHFT;
	switch (ret) {
	case 0:
		ret = FLASHLIGHT_MODE_OFF;
		break;
	case 1:
		ret = FLASHLIGHT_MODE_FLASH;
		break;
	case 2:
	case 3:
		ret = FLASHLIGHT_MODE_TORCH;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mt6370_bled_fled_strobe(struct rt_fled_dev *fled_dev)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;
	int ret = 0;

	ret = mt6370_bled_fled_set_mode(fled_dev, FLASHLIGHT_MODE_OFF);
	if (ret < 0)
		return ret;
	return mt6370_pmu_reg_update_bits(bled_data->chip,
			MT6370_PMU_REG_BLFL, MT6370_BLFLMODE_MASK,
			1 << MT6370_BLFLMODE_SHFT);
}

static int mt6370_bled_fled_torch_current_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	if (selector >= 256)
		return -EINVAL;
	return 117 * selector;
}

static int mt6370_bled_fled_strobe_current_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	if (selector >= 256)
		return -EINVAL;
	return 117 * selector;
}

static int mt6370_bled_fled_timeout_level_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int mt6370_bled_fled_lv_protection_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int mt6370_bled_fled_strobe_timeout_list(struct rt_fled_dev *fled_dev,
	int selector)
{
	if (selector >= 128)
		return -EINVAL;
	return 16 * selector;
}

static int mt6370_bled_fled_set_torch_current_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;

	if (selector >= 256)
		return -EINVAL;
	return mt6370_pmu_reg_write(bled_data->chip,
			MT6370_PMU_REG_BLTORCTRL, selector);
}

static int mt6370_bled_fled_set_strobe_current_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;

	if (selector >= 256)
		return -EINVAL;
	return mt6370_pmu_reg_write(bled_data->chip,
			MT6370_PMU_REG_BLSTRBCTRL, selector);
}

static int mt6370_bled_fled_set_timeout_level_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int mt6370_bled_fled_set_lv_protection_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	return -EINVAL;
}

static int mt6370_bled_fled_set_strobe_timeout_sel(struct rt_fled_dev *fled_dev,
	int selector)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;

	if (selector >= 128)
		return -EINVAL;
	return mt6370_pmu_reg_update_bits(bled_data->chip,
			MT6370_PMU_REG_BLFLTO, MT6370_BLSTRB_TOMASK, selector);
}

static int mt6370_bled_fled_get_torch_current_sel(struct rt_fled_dev *fled_dev)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;

	return mt6370_pmu_reg_read(bled_data->chip, MT6370_PMU_REG_BLTORCTRL);
}

static int mt6370_bled_fled_get_strobe_current_sel(struct rt_fled_dev *fled_dev)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;

	return mt6370_pmu_reg_read(bled_data->chip, MT6370_PMU_REG_BLSTRBCTRL);
}

static int mt6370_bled_fled_get_timeout_level_sel(struct rt_fled_dev *fled_dev)
{
	return -EINVAL;
}

static int mt6370_bled_fled_get_lv_protection_sel(struct rt_fled_dev *fled_dev)
{
	return -EINVAL;
}

static int mt6370_bled_fled_get_strobe_timeout_sel(struct rt_fled_dev *fled_dev)
{
	struct mt6370_pmu_bled_data *bled_data =
			(struct mt6370_pmu_bled_data *)fled_dev;
	int ret = 0;

	ret = mt6370_pmu_reg_read(bled_data->chip, MT6370_PMU_REG_BLFLTO);
	return (ret < 0 ? ret : ret & MT6370_BLSTRB_TOMASK);
}

static void mt6370_bled_fled_shutdown(struct rt_fled_dev *fled_dev)
{
	mt6370_bled_fled_set_mode(fled_dev, FLASHLIGHT_MODE_OFF);
}

static struct rt_fled_hal mt6370_bledfl_hal = {
	.rt_hal_fled_set_mode = mt6370_bled_fled_set_mode,
	.rt_hal_fled_get_mode = mt6370_bled_fled_get_mode,
	.rt_hal_fled_strobe = mt6370_bled_fled_strobe,
	.rt_hal_fled_torch_current_list = mt6370_bled_fled_torch_current_list,
	.rt_hal_fled_strobe_current_list = mt6370_bled_fled_strobe_current_list,
	.rt_hal_fled_timeout_level_list = mt6370_bled_fled_timeout_level_list,
	.rt_hal_fled_lv_protection_list = mt6370_bled_fled_lv_protection_list,
	.rt_hal_fled_strobe_timeout_list = mt6370_bled_fled_strobe_timeout_list,
	.rt_hal_fled_set_torch_current_sel =
					mt6370_bled_fled_set_torch_current_sel,
	.rt_hal_fled_set_strobe_current_sel =
					mt6370_bled_fled_set_strobe_current_sel,
	.rt_hal_fled_set_timeout_level_sel =
					mt6370_bled_fled_set_timeout_level_sel,
	.rt_hal_fled_set_lv_protection_sel =
					mt6370_bled_fled_set_lv_protection_sel,
	.rt_hal_fled_set_strobe_timeout_sel =
					mt6370_bled_fled_set_strobe_timeout_sel,
	.rt_hal_fled_get_torch_current_sel =
					mt6370_bled_fled_get_torch_current_sel,
	.rt_hal_fled_get_strobe_current_sel =
					mt6370_bled_fled_get_strobe_current_sel,
	.rt_hal_fled_get_timeout_level_sel =
					mt6370_bled_fled_get_timeout_level_sel,
	.rt_hal_fled_get_lv_protection_sel =
					mt6370_bled_fled_get_lv_protection_sel,
	.rt_hal_fled_get_strobe_timeout_sel =
					mt6370_bled_fled_get_strobe_timeout_sel,
	.rt_hal_fled_shutdown = mt6370_bled_fled_shutdown,
};

static const struct flashlight_properties mt6370_bledfl_props = {
	.type = FLASHLIGHT_TYPE_LED,
	.torch_brightness = 128, /* default torch brightness 15mA */
	.torch_max_brightness = 255,
	.strobe_brightness = 255, /* default strobe brightness 30mA */
	.strobe_max_brightness = 255,
	.strobe_timeout = 208, /* default strobe timeout 208mS */
	.alias_name = "mt6370_pmu_bled",
};

static void mt6370_pmu_bled_bright_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct mt6370_pmu_bled_data *bled_data =
				dev_get_drvdata(led_cdev->dev->parent);
	struct mt6370_pmu_bled_platdata *pdata =
				dev_get_platdata(bled_data->dev);
	uint32_t bright = (pdata->max_bled_brightness << 8) / 255;
	int ret = 0;

#ifdef CONFIG_LEDS_BRIGHTNESS_NORMALIZE
	brightness = led_normalize_brightness_level(brightness);
#endif
	dev_dbg(led_cdev->dev, "%s: %d\n", __func__, brightness);
	bright = (bright * brightness) >> 8;
	ret = mt6370_pmu_reg_update_bits(bled_data->chip, MT6370_PMU_REG_BLDIM2,
					 MT6370_DIM2_MASK, bright & 0x7);
	if (ret < 0)
		goto out_bright_set;
	ret = mt6370_pmu_reg_write(bled_data->chip, MT6370_PMU_REG_BLDIM1,
				   (bright >> 3) & MT6370_DIM_MASK);
	if (ret < 0)
		goto out_bright_set;
	/* if choose external enable pin, no effect even config this bit */
	ret = mt6370_pmu_reg_update_bits(bled_data->chip, MT6370_PMU_REG_BLEN,
					 MT6370_BLED_EN,
					 brightness > 0 ?
					 MT6370_BLED_EN : ~MT6370_BLED_EN);
	if (ret < 0)
		goto out_bright_set;
	return;
out_bright_set:
	dev_err(led_cdev->dev, "%s error %d\n", __func__, ret);
}

static struct led_classdev mt6370_pmu_bled_dev = {
	.name = "mt6370_pmu_bled",
	.brightness_set = mt6370_pmu_bled_bright_set,
};

static irqreturn_t mt6370_pmu_bled_ocp_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_bled_data *bled_data = data;

	dev_notice(bled_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_bled_ovp_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_bled_data *bled_data = data;

	dev_notice(bled_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static struct mt6370_pmu_irq_desc mt6370_bled_irq_desc[] = {
	MT6370_PMU_IRQDESC(bled_ocp),
	MT6370_PMU_IRQDESC(bled_ovp),
};

static void mt6370_pmu_bled_irq_register(struct platform_device *pdev)
{
	struct resource *res;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(mt6370_bled_irq_desc); i++) {
		if (!mt6370_bled_irq_desc[i].name)
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					mt6370_bled_irq_desc[i].name);
		if (!res)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, res->start, NULL,
					mt6370_bled_irq_desc[i].irq_handler,
					IRQF_TRIGGER_FALLING,
					mt6370_bled_irq_desc[i].name,
					platform_get_drvdata(pdev));
		if (ret < 0) {
			dev_err(&pdev->dev, "request %s irq fail\n", res->name);
			continue;
		}
		mt6370_bled_irq_desc[i].irq = res->start;
	}
}

static inline int mt6370_pmu_bled_init_register(
	struct mt6370_pmu_bled_data *bled_data)
{
	if (bled_data->chip->chip_rev <= 1)
		bled_init_data[1] |= MT6370_BLED_OVOCSHDNDIS;
	return mt6370_pmu_reg_block_write(bled_data->chip, MT6370_PMU_REG_BLEN,
			ARRAY_SIZE(bled_init_data), bled_init_data);
}

static inline int mt6370_pmu_bled_parse_initdata(
	struct mt6370_pmu_bled_data *bled_data)
{
	struct mt6370_pmu_bled_platdata *pdata = dev_get_platdata(
							bled_data->dev);
	uint32_t bright = (pdata->max_bled_brightness << 8) / 255;

	if (pdata->ext_en_pin) {
		bled_init_data[0] &= ~(MT6370_BLED_EN | MT6370_BLED_EXTEN);
		bled_init_data[0] |= MT6370_BLED_EXTEN;
	}
	bled_init_data[0] |= (pdata->chan_en << MT6370_BLED_CHANENSHFT);
	if (!pdata->map_linear)
		bled_init_data[0] &= ~(MT6370_BLED_MAPLINEAR);
	bled_init_data[1] |= (pdata->bl_ovp_level << MT6370_BLED_OVPSHFT);
	bled_init_data[1] |= (pdata->bl_ocp_level << MT6370_BLED_OCPSHFT);
	bled_init_data[2] |= (pdata->use_pwm << MT6370_BLED_PWMSHIFT);
	bled_init_data[2] |= (pdata->pwm_fsample << MT6370_BLED_PWMFSHFT);
	bled_init_data[2] |= (pdata->pwm_deglitch << MT6370_BLED_PWMDSHFT);
	bled_init_data[3] |= (pdata->bled_ramptime << MT6370_BLED_RAMPTSHFT);
	if (pdata->ignore_initial_bled_brightness) {
		bled_init_data[5] = mt6370_pmu_reg_read(bled_data->chip,
					MT6370_PMU_REG_BLDIM1);
		bled_init_data[4] = mt6370_pmu_reg_read(bled_data->chip,
					MT6370_PMU_REG_BLDIM2);
	} else {
		bright = (bright * 255) >> 8;
		bled_init_data[4] |= (bright & 0x7);
		bled_init_data[5] |= ((bright >> 3) & 0xff);
	}
	bled_init_data[7] |= (pdata->bled_flash_ramp << MT6370_BLFLRAMP_SHFT);
	bled_init_data[10] |= (pdata->pwm_avg_cycle);
	if (pdata->bled_name)
		mt6370_pmu_bled_dev.name = pdata->bled_name;
	return 0;
}

static inline int mt_parse_dt(struct device *dev)
{
	struct mt6370_pmu_bled_platdata *pdata = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	const char *name;
	u32 tmp = 0;

	if (of_property_read_bool(np, "mt,ext_en_pin"))
		pdata->ext_en_pin = 1;
	if (of_property_read_u32(np, "mt,chan_en", &tmp) < 0)
		pdata->chan_en = 0x0; /* default 4 channel disable */
	else
		pdata->chan_en = tmp;
	if (of_property_read_bool(np, "mt,map_linear"))
		pdata->map_linear = 1;
	if (of_property_read_u32(np, "mt,bl_ovp_level", &tmp) < 0)
		pdata->bl_ovp_level = 0x3; /* default 29V */
	else
		pdata->bl_ovp_level = tmp;
	if (of_property_read_u32(np, "mt,bl_ocp_level", &tmp) < 0)
		pdata->bl_ocp_level = 0x2; /* default 1500mA */
	else
		pdata->bl_ocp_level = tmp;
	if (of_property_read_bool(np, "mt,use_pwm"))
		pdata->use_pwm = 1;
	if (of_property_read_u32(np, "mt,pwm_fsample", &tmp) < 0)
		pdata->pwm_fsample = 0x1; /* 4MHz */
	else
		pdata->pwm_fsample = tmp;
	if (of_property_read_u32(np, "mt,pwm_deglitch", &tmp) < 0)
		pdata->pwm_deglitch = 0x1;
	else
		pdata->pwm_deglitch = tmp;
	if (of_property_read_u32(np, "mt,pwm_hys_en", &tmp) < 0)
		pdata->pwm_hys_en = 0x1;
	else
		pdata->pwm_hys_en = tmp;
	if (of_property_read_u32(np, "mt,pwm_hys", &tmp) < 0)
		pdata->pwm_hys = 0x0;	/* 1 bit */
	else
		pdata->pwm_hys = tmp;
	if (of_property_read_u32(np, "mt,pwm_avg_cycle", &tmp) < 0)
		pdata->pwm_avg_cycle = 0;
	else
		pdata->pwm_avg_cycle = tmp;
	if (of_property_read_u32(np, "mt,bled_ramptime", &tmp) < 0)
		pdata->bled_ramptime = 0x3; /* default 1ms */
	else
		pdata->bled_ramptime = tmp;
	if (of_property_read_u32(np, "mt,bled_flash_ramp", &tmp) < 0)
		pdata->bled_flash_ramp = 0x1;
	else
		pdata->bled_flash_ramp = tmp;
	if (of_property_read_u32(np, "mt,max_bled_brightness", &tmp) < 0)
		pdata->max_bled_brightness = 1024;
	else
		pdata->max_bled_brightness = tmp;
#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
		if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT)
			pdata->max_bled_brightness = 0;
#endif
	if (of_property_read_string(np, "mt,bled_name", &name) < 0)
		pdata->bled_name = "mt6370_pmu_bled";
	else
		pdata->bled_name = name;
	if (of_property_read_u32(np,
			"mt,ignore_initial_bled_brightness", &tmp) < 0)
		pdata->ignore_initial_bled_brightness = 0;
	else
		pdata->ignore_initial_bled_brightness = tmp;
#ifdef CONFIG_LEDS_BRIGHTNESS_NORMALIZE
	if (led_parse_normalize_dts(np) < 0)
		pr_err("Fail to get normalize params from dts, ignore!\n");
#endif
	return 0;
}

static uint32_t mt_get_current_level(void *data)
{
	uint32_t val;
	struct mt6370_pmu_bled_data *bled_data = data;
	struct mt6370_pmu_bled_platdata *pdata =
				dev_get_platdata(bled_data->dev);

	val = ((bled_init_data[4] & 0x7) + (bled_init_data[5] << 3));
	val = (val << 8) / ((pdata->max_bled_brightness << 8) / 255);
	return val;
}

static int thermal_limit_set(void *data, u64 val)
{
	struct platform_device *pdev = (struct platform_device *)data;

	dev_notice(&pdev->dev, "%s, enter set brightness %llu.\n", __func__, val);
	if (val > 255 || val < 0)
		val = 255;
	if (val == 255)
		led_setMaxbrightness(255, 0);
	else
		led_setMaxbrightness(val, 1);

	return 0;
}

static int thermal_limit_get(void *data, u64 *val)
{
	*val = mt6370_pmu_bled_dev.max_brightness;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(thermal_limit_lcd_fops,
				thermal_limit_get, thermal_limit_set, "%llu\n");

int led_debugfs_init(struct platform_device *pdev)
{
	struct dentry *debug_file;
	struct dentry *debug_root;

	debug_root =  debugfs_create_dir("lcd-backlight", NULL);
	if (IS_ERR_OR_NULL(debug_root)) {
		dev_err(&pdev->dev, "thermal-lcd: failed to create debug dir.\n");
		return -1;
	}
	debug_file = debugfs_create_file("thermal_limit", 0644, debug_root, pdev, &thermal_limit_lcd_fops);
	if (IS_ERR_OR_NULL(debug_file)) {
		dev_err(&pdev->dev, "thermal-lcd: failed to create debug node.\n");
		return -1;
	}
#ifdef CONFIG_LEDS_BRIGHTNESS_NORMALIZE
	debug_file = debugfs_create_file("brightness_normalize_enable", 0644,
			debug_root, pdev,
			&led_normalize_enable_fops);
	if (IS_ERR_OR_NULL(debug_file)) {
		dev_err(&pdev->dev, "device_create_file normalize_enable fail!\n");
		return -1;
	}
#endif

	return 0;
}


static int mt6370_pmu_bled_probe(struct platform_device *pdev)
{
	struct mt6370_pmu_bled_platdata *pdata = dev_get_platdata(&pdev->dev);
	struct mt6370_pmu_bled_data *bled_data;
	bool use_dt = pdev->dev.of_node;
	int ret = 0;

	bled_data = devm_kzalloc(&pdev->dev, sizeof(*bled_data), GFP_KERNEL);
	if (!bled_data)
		return -ENOMEM;
	if (use_dt) {
		/* DTS used */
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto out_pdata;
		}
		pdev->dev.platform_data = pdata;
		ret = mt_parse_dt(&pdev->dev);
		if (ret < 0) {
			devm_kfree(&pdev->dev, pdata);
			goto out_pdata;
		}
	} else {
		if (!pdata) {
			ret = -EINVAL;
			goto out_pdata;
		}
	}
	bled_data->chip = dev_get_drvdata(pdev->dev.parent);
	bled_data->dev = &pdev->dev;
	platform_set_drvdata(pdev, bled_data);

	ret = mt6370_pmu_bled_parse_initdata(bled_data);
	if (ret < 0)
		goto out_init_data;

	ret = mt6370_pmu_bled_init_register(bled_data);
	if (ret < 0)
		goto out_init_reg;

	if (pdata->use_pwm)
		mt6370_pmu_bled_dev.default_trigger = "backlight";
	mt6370_pmu_bled_dev.brightness = mt_get_current_level(bled_data);
	ret = led_classdev_register(bled_data->dev, &mt6370_pmu_bled_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "register leddev fail\n");
		goto out_led_register;
	}

	bled_data->base.init_props = &mt6370_bledfl_props;
	bled_data->base.hal = &mt6370_bledfl_hal;
	bled_data->base.name = kstrndup(pdata->bled_name,
					strlen(pdata->bled_name), GFP_KERNEL);
	bled_data->base.chip_name = "mt6370_pmu_bled";
	bled_data->mt_flash_dev =
		platform_device_register_resndata(bled_data->dev,
						  "rt-flash-led", 2, NULL, 0,
						  NULL, 0);

	if (IS_ERR(bled_data->mt_flash_dev)) {
		dev_err(&pdev->dev, "register mt_flash_dev fail\n");
		goto out_mt_flash_register;
	}

	mt6370_pmu_bled_irq_register(pdev);
	led_debugfs_init(pdev);
	dev_info(&pdev->dev, "%s successfully\n", __func__);
	return 0;
out_mt_flash_register:
	led_classdev_unregister(&mt6370_pmu_bled_dev);
out_led_register:
out_init_reg:
out_init_data:
out_pdata:
	devm_kfree(&pdev->dev, bled_data);
	return ret;
}

static int mt6370_pmu_bled_remove(struct platform_device *pdev)
{
	struct mt6370_pmu_bled_data *bled_data = platform_get_drvdata(pdev);

	platform_device_unregister(bled_data->mt_flash_dev);
	led_classdev_unregister(&mt6370_pmu_bled_dev);
	dev_info(bled_data->dev, "%s successfully\n", __func__);
	return 0;
}

static const struct of_device_id mt_ofid_table[] = {
	{ .compatible = "mediatek,mt6370_pmu_bled", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt_ofid_table);

static const struct platform_device_id mt_id_table[] = {
	{ "mt6370_pmu_bled", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, mt_id_table);

static struct platform_driver mt6370_pmu_bled = {
	.driver = {
		.name = "mt6370_pmu_bled",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mt_ofid_table),
	},
	.probe = mt6370_pmu_bled_probe,
	.remove = mt6370_pmu_bled_remove,
	.id_table = mt_id_table,
};
module_platform_driver(mt6370_pmu_bled);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek MT6370 PMU Bled");
MODULE_VERSION("1.0.0_G");
