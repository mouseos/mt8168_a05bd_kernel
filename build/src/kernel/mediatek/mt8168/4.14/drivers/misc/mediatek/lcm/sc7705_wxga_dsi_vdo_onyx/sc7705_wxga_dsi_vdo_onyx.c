// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#endif
#endif
#include "lcm_drv.h"

#define LCM_CABC_MODE_REG	0x55
static unsigned char vendor_id = 0xFF;
static void get_lcm_id(void);

static struct lcm_platform_data {
	struct pinctrl *lcmctrl;
	struct pinctrl_state *lcm_pins_rst0;
	struct pinctrl_state *lcm_pins_rst1;
	struct pinctrl_state *lcm_pins_bl0;
	struct pinctrl_state *lcm_pins_bl1;
	struct regulator *vsim1_ldo;
	unsigned char lcm_id;
} lcm_data;

static int __init setup_lcm_id(char *str)
{
	int id;

	if (get_option(&str, &id))
		lcm_data.lcm_id = (unsigned char)id;
	else
		lcm_data.lcm_id = 0xFF;
	return 0;
}
__setup("LCM_VENDOR_ID=", setup_lcm_id);

static void get_lcm_id(void)
{
	if (lcm_data.lcm_id == 0xFF) {
		pr_err("get lcm_id failed.\n");
	} else {
		vendor_id = lcm_data.lcm_id;
		pr_debug("[sc7705] get lcm_id from cmdline.\n");
	}

	pr_info("[sc7705] %s, vendor id = 0x%x\n", __func__, lcm_data.lcm_id);
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	int ret = 0;

	pr_notice("[Kernel/LCM] %s Default transmission mode: mipi\n",
					__func__);

	lcm_data.vsim1_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_data.vsim1_ldo)) {
		ret = PTR_ERR(lcm_data.vsim1_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	ret = regulator_enable(lcm_data.vsim1_ldo);
	if (ret < 0) {
		dev_err(dev, "failed to enable vsim1_ldo\n");
		goto lcm_power_free;
	}

	lcm_data.lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcm_data.lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcm_data.lcmctrl);
		goto lcm_power_enalbe_free;
	}

	/*lcm power pin lookup */
	lcm_data.lcm_pins_rst0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst0");
	if (IS_ERR(lcm_data.lcm_pins_rst0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst0);
		dev_err(dev, "pinctrl err, lcm_pins_rst0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_rst1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst1");
	if (IS_ERR(lcm_data.lcm_pins_rst1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst1);
		dev_err(dev, "pinctrl err, lcm_pins_rst1\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bl0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bl0");
	if (IS_ERR(lcm_data.lcm_pins_bl0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bl0);
		dev_err(dev, "pinctrl err, lcm_pins_bl0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bl1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bl1");
	if (IS_ERR(lcm_data.lcm_pins_bl1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bl1);
		dev_err(dev, "pinctrl err, lcm_pins_bl1\n");
		goto lcm_pinctrl_free;
	}

	return ret;

lcm_pinctrl_free:
	devm_pinctrl_put(lcm_data.lcmctrl);
	lcm_data.lcmctrl = NULL;

lcm_power_enalbe_free:
	regulator_disable(lcm_data.vsim1_ldo);

lcm_power_free:
	regulator_put(lcm_data.vsim1_ldo);
	lcm_data.vsim1_ldo = NULL;
	dev_err(dev, "[Kernel/LCM] %s get lcm dev res failed.\n", __func__);
	return ret;
}

static void lcm_set_rst(int val)
{
	if (lcm_data.lcmctrl != NULL) {
		if (val == 0)
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_rst0);
		else
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_rst1);
	}
}

static void lcm_set_bl(int val)
{
	if (lcm_data.lcmctrl != NULL) {
		if (val == 0)
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_bl0);
		else
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_bl1);
	}
}

static int lcm_platform_remove(struct platform_device *pdev)
{
	/* free gpio */
	if (lcm_data.lcmctrl != NULL) {
		devm_pinctrl_put(lcm_data.lcmctrl);
		lcm_data.lcmctrl = NULL;
	}
	/* free lcm power */
	if (lcm_data.vsim1_ldo != NULL) {
		devm_regulator_put(lcm_data.vsim1_ldo);
		lcm_data.vsim1_ldo = NULL;
	}
	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{.compatible = "sc,sc7705_kd_boe"},
	{},
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *id;
	unsigned int panel_id = 0xff;
	int ret = 0;

	get_lcm_id();

	ret = of_property_read_u32(np, "vendor_id", &panel_id);
	if (ret < 0)
		pr_err("get dts vendor_id failed\n");

	if (panel_id != vendor_id)
		return -ENODEV;

	pr_notice("[Kernel/LCM] %s panel_id = %d, vendor_id = %d\n",
			__func__, panel_id, vendor_id);

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "sc7705",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
	},
	.remove = lcm_platform_remove,
};

static int __init lcm_drv_init(void)
{
	pr_notice("LCM:Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM:failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM:Unregister lcm driver done\n");
}

late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */

#define FRAME_WIDTH		(800)
#define FRAME_HEIGHT		(1280)
#define GPIO_OUT_ONE		1
#define GPIO_OUT_ZERO		0

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE	0x00

/* ID1 DAh - Vendor and Build Designation
 * Bit[7:5] - Vendor Code (Innolux, LGD, Samsung)
 * Bit[4:2] - Reserved
 * Bit[2:0] - Build (proto, evt, dvt, etc.)
 */
#define HVT		0x0
#define EVT1		0x1
#define EVT2		0x2
#define DVT		0x4
#define PVT		0x7
#define MP		0x6
#define BUILD_MASK	0x7

#define SITR_KD_BOE	0x8	/*0x8 TG, using SITR IC */

/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */

static struct LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)		(lcm_util.set_reset_pin((v)))
#define UDELAY(n)			(lcm_util.udelay(n))
#define MDELAY(n)			(lcm_util.mdelay(n))

/* ----------------------------------------------------------------- */
/* Local Functions */
/* ----------------------------------------------------------------- */
#define dsi_set_cmdq_V4(para_tbl, size, force_update) \
	lcm_util.dsi_set_cmdq_V4(para_tbl, size, force_update)
#define dsi_get_cmdq_V4(para_tbl, size, force_update) \
	lcm_util.dsi_get_cmdq_V4(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		 (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		 (lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd) \
		 (lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums) \
		 (lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg \
		 (lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size) \
		 (lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

static struct LCM_setting_table_V4 cabc_on_setting[] = {
	{0x15/*DSI_DCS_SHORT_PACKET_ID_1*/, LCM_CABC_MODE_REG, 1, {0x03}, 0 },
};

static struct LCM_setting_table_V4 cabc_off_setting[] = {
	{0x15/*DSI_DCS_SHORT_PACKET_ID_1*/, LCM_CABC_MODE_REG, 1, {0x00}, 0 },
};

static void init_onyx_sitr_kd_boe_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[sc7705] %s enter.\n", __func__);
	/* PASSWORD */
	data_array[0] = 0x00043902;
	data_array[1] = 0x8412F1B9;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x000B3902;
	data_array[1] = 0x237523B1;
	data_array[2] = 0x77332E2E;
	data_array[3] = 0x000CDB01;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x000840B2;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00093902;
	data_array[1] = 0x000000B3;
	data_array[2] = 0x28282800;
	data_array[3] = 0x00000028;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x20B41500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x000909B5;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x008686B6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x000377B8;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x001C3902;
	data_array[1] = 0x058133BA;
	data_array[2] = 0x200E0EF9;
	data_array[3] = 0x00000000;
	data_array[4] = 0x44000000;
	data_array[5] = 0x0A900025;
	data_array[6] = 0x4F020000;
	data_array[7] = 0x37000001;
	dsi_set_cmdq(data_array, 8, 1);
	data_array[0] = 0x46BC1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00043902;
	data_array[1] = 0x821100BF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x000A3902;
	data_array[1] = 0x507373C0;
	data_array[2] = 0x08008050;
	data_array[3] = 0x00000070;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x0BCC1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00233902;
	data_array[1] = 0x161201E0;
	data_array[2] = 0x443F382E;
	data_array[3] = 0x0C0A0438;
	data_array[4] = 0x12111311;
	data_array[5] = 0x1201140F;
	data_array[6] = 0x3F382E16;
	data_array[7] = 0x0A043844;
	data_array[8] = 0x1113110C;
	data_array[9] = 0x00140F12;
	dsi_set_cmdq(data_array, 10, 1);
	data_array[0] = 0x000B3902;
	data_array[1] = 0x000303E3;
	data_array[2] = 0x00030300;
	data_array[3] = 0x00C00000;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x00403902;
	data_array[1] = 0x0B0002E9;
	data_array[2] = 0x81801A05;
	data_array[3] = 0x5F233112;
	data_array[4] = 0x4728800E;
	data_array[5] = 0x00010808;
	data_array[6] = 0x08000000;
	data_array[7] = 0x00000001;
	data_array[8] = 0x77113300;
	data_array[9] = 0x88881855;
	data_array[10] = 0x88B48A88;
	data_array[11] = 0x44660022;
	data_array[12] = 0x88888808;
	data_array[13] = 0x0088B48A;
	data_array[14] = 0x00000000;
	data_array[15] = 0x00000000;
	data_array[16] = 0x00000000;
	dsi_set_cmdq(data_array, 17, 1);
	data_array[0] = 0x00403902;
	data_array[1] = 0x020C96EA;
	data_array[2] = 0x00B40202;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00664400;
	data_array[5] = 0x88880822;
	data_array[6] = 0x88B48A88;
	data_array[7] = 0x33117755;
	data_array[8] = 0x88888818;
	data_array[9] = 0x2388B48A;
	data_array[10] = 0x01000010;
	data_array[11] = 0x00000045;
	data_array[12] = 0x00000000;
	data_array[13] = 0x00000000;
	data_array[14] = 0x50000000;
	data_array[15] = 0x00008180;
	data_array[16] = 0x16010000;
	dsi_set_cmdq(data_array, 17, 1);
	/* cabc setting */
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00053902;
	data_array[1] = 0x0000A0C7;
	data_array[2] = 0x00000004;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00053902;
	data_array[1] = 0x060610C8;
	data_array[2] = 0x00000008;
	dsi_set_cmdq(data_array, 3, 1);
	/* pwm duty */
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SLP OUT*/
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

/* ----------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_sitr_onyx_kd_boe_params(struct LCM_PARAMS *params)
{
	pr_debug("[sc7705] %s enter.\n", __func__);

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM		= LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 11;
	params->dsi.vertical_frontporch			= 20;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 18;
	params->dsi.horizontal_backporch		= 40;
	params->dsi.horizontal_frontporch		= 40;
	params->dsi.horizontal_active_pixel		= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 234;
	params->dsi.clk_lp_per_line_enable = 1;

	/* params->dsi.ssc_disable = 1; */
	params->dsi.cont_clock = 0;
	params->dsi.DA_HS_EXIT = 10;
	params->dsi.CLK_ZERO = 16;
	params->dsi.HS_ZERO = 9;
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_HS_POST = 8;
	params->dsi.CLK_HS_EXIT = 6;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;

	params->dsi.lcm_esd_check_table[1].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x9C;

	params->dsi.lcm_esd_check_table[2].cmd = 0xBA;
	params->dsi.lcm_esd_check_table[2].count = 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x33;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_reset(void)
{
	/* GPIO20- Reset sc7705 */
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_set_rst(1);
	MDELAY(10);
	lcm_set_rst(0);
	MDELAY(10);
	lcm_set_rst(1);
	MDELAY(20);
}

static char *lcm_get_vendor_type(void)
{
	switch (vendor_id) {
	case SITR_KD_BOE: return "SITR_KD_BOE\0";
	default: return "Unknown\0";
	}
}

static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	get_lcm_id();

	pr_info("[Kernel/LCM] vendor type: %s\n",
			lcm_get_vendor_type());

	if (vendor_id == SITR_KD_BOE)
		init_onyx_sitr_kd_boe_lcm();	/*SITR KD panel*/
	else
		init_onyx_sitr_kd_boe_lcm();	/*SITR KD panel*/

	lcm_set_bl(1);
}

static void lcm_resume(void)
{
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
	char buf[METRICS_STR_LEN];
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG)
	snprintf(buf, sizeof(buf), "%s:lcd:resume=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf);
#endif

#if defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
	minerva_metrics_log(buf, METRICS_STR_LEN, "%s:%s:100:%s,%s,%s,%s,"
			"lcm_state=lcm_resume;SY,ESD_Recovery=0;IN:us-east-1",
			METRICS_LCD_GROUP_ID, METRICS_LCD_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_MODEL_KEY,
			PREDEFINED_TZ_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
#endif

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	get_lcm_id();

	MDELAY(5);

	lcm_reset();

	if (vendor_id == SITR_KD_BOE)
		init_onyx_sitr_kd_boe_lcm();	/*SITR KD panel*/
	else
		init_onyx_sitr_kd_boe_lcm();	/*SITR KD panel*/

	lcm_set_bl(1);
}

static void lcm_init_power(void)
{
	int ret;

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if (lcm_data.vsim1_ldo == NULL) {
		pr_err("vsim1_ldo is NULL!\n");
		return;
	}
	ret = regulator_enable(lcm_data.vsim1_ldo);
	MDELAY(30);
	display_bias_enable();
	MDELAY(5);
	lcm_reset();
}

static void lcm_resume_power(void)
{
	int ret;

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if (lcm_data.vsim1_ldo == NULL) {
		pr_err("vsim1_ldo is NULL!\n");
		return;
	}
	ret = regulator_enable(lcm_data.vsim1_ldo);
	MDELAY(30);
	display_bias_enable();
	MDELAY(5);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
	char buf[METRICS_STR_LEN];
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG)
	snprintf(buf, sizeof(buf), "%s:lcd:suspend=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf);
#endif

#if defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
	minerva_metrics_log(buf, METRICS_STR_LEN, "%s:%s:100:%s,%s,%s,%s,"
			"lcm_state=lcm_suspend;SY,ESD_Recovery=0;IN:us-east-1",
			METRICS_LCD_GROUP_ID, METRICS_LCD_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_MODEL_KEY,
			PREDEFINED_TZ_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
#endif

	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	lcm_set_bl(0);
	MDELAY(10);
	data_array[0] = 0x00280500; /* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(60);

	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);
}

static void lcm_suspend_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if (lcm_data.vsim1_ldo == NULL) {
		pr_err("vsim1_ldo is NULL!\n");
		return;
	}

	lcm_set_rst(0);
	MDELAY(2);
	display_bias_disable();
	MDELAY(5);
	regulator_disable(lcm_data.vsim1_ldo);
}

static void lcm_setbacklight_mode(unsigned int mode)
{
	if (mode >= CABC_MAX) {
		pr_err("%s invalid CABC mode=%d\n", __FUNCTION__, mode);
		return;
	}

	pr_info("lcm driver cabc mode =%x\n", mode);

	switch (mode) {
	case CABC_MOVIE:
		pr_info("[SC7705][LCM/K]CABC_MOVIE\n");
		dsi_set_cmdq_V4(cabc_on_setting,
			sizeof(cabc_on_setting) / sizeof(struct LCM_setting_table_V4), 1);
		break;
	case CABC_OFF:
		pr_info("[SC7705][LCM/K]CABC_OFF\n");
		dsi_set_cmdq_V4(cabc_off_setting,
			sizeof(cabc_off_setting) / sizeof(struct LCM_setting_table_V4), 1);
		break;
	default:
		pr_notice("CABC mode %x is not supported now\n", mode);
		break;
	}

}

static void lcm_setbacklight_power_off(void)
{
	pr_notice("[Kernel/LCM] %s BL pull down\n", __func__);
	lcm_set_bl(0);
}

struct LCM_DRIVER sc7705_wxga_dsi_vdo_sitr_kd_boe_onyx_lcm_drv = {
	.name		= "sc7705_wxga_dsi_vdo_sitr_kd_boe_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_sitr_onyx_kd_boe_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend	= lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};
