// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
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
		pr_debug("[jd936x] get lcm_id from cmdline.\n");
	}

	pr_debug("[jd936x] %s, vendor id = 0x%x\n", __func__, lcm_data.lcm_id);
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
		pr_err("pinctrl err, lcm_pins_rst0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_rst1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst1");
	if (IS_ERR(lcm_data.lcm_pins_rst1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst1);
		pr_err("pinctrl err, lcm_pins_rst1\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bl0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bl0");
	if (IS_ERR(lcm_data.lcm_pins_bl0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bl0);
		pr_err("pinctrl err, lcm_pins_bl0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bl1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bl1");
	if (IS_ERR(lcm_data.lcm_pins_bl1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bl1);
		pr_err("pinctrl err, lcm_pins_bl1\n");
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
	pr_err("[LCM] %s get lcm dev res failed.\n", __func__);
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
	{.compatible = "jd,jd936x_inx_inx"},
	{.compatible = "jd,jd936x_kd_hsd"},
	{.compatible = "jd,jd936x_tg_boe"},
	{.compatible = "jd,jd936x_kd_auo"},
	{.compatible = "jd,jd936x_tah_tg_boe"},
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
		   .name = "jd936x",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
	},
	.remove = lcm_platform_remove,
};

static int __init lcm_drv_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
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
#define REGFLAG_END_OF_TABLE		0x00

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

#define FITI_INX_INX		0x2		/* INX, using FITI IC */
#define FITI_KD_HSD		0x3		/* KD, using FITI IC */
#define FITI_KD_AUO		0x4		/* KD, using FITI IC */
#define FITI_TG_BOE		0x5		/* TG, using FITI IC */
#define	FITI_TAH_TG_BOE		0xD		/* TG, using FITI IC */
/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */

static struct LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)		(lcm_util.set_reset_pin((v)))
#define UDELAY(n)				(lcm_util.udelay(n))
#define MDELAY(n)				(lcm_util.mdelay(n))

/* ----------------------------------------------------------------- */
/* Local Functions */
/* ----------------------------------------------------------------- */
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

static void init_onyx_fiti_inx_inx_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[jd936x] %s enter.\n", __func__);
	/* PAGE 0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PASSWORD */
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);
	/* DS1 4 LANE*/
	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetVCOM */
	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x32011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x32041500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetGamma Power */
	data_array[0] = 0x10171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x101A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFE241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x703C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA8571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D5A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1A5B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x155C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F5D1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Gamma */
	data_array[0] = 0x6A5E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B5F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3B621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x61681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x586A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B6B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A6C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3F6D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2F6E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x256F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3B751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x457A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x617B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F7C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x587D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A7F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3F801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2F811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 2*/
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L Pin mapping */
	data_array[0] = 0x50001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x52031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x77041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x57051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A0A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x480B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x460D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x440E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x400F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x51161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x53191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x771A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x571B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F1C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F1D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D1E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F2A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L_GS Pin mapping */
	data_array[0] = 0x012C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x132F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B3A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x113B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F411500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R_GS Pin mapping */
	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17461500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17471500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F481500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C491500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E4A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x044C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x064D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x084F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F571500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP Timing */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x065C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x405D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6C631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6C641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xB4671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6C691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6C6A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C6B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xBB751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2A781500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 3*/
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x019A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x029B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01AF1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 4*/
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x11091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A961500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 0*/
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SLPOUT */
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_onyx_fiti_inx_inx_proto_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[jd936x] %s enter.\n", __func__);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6E011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x82041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xD7181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xD71B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x011C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x791F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x703C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x044B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA8571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x375A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x195B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x785D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B5E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A5F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x39611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x29621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2F651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x32691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x406A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x396B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C6C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x226D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E6E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x78701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x39741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x29751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2F781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C7A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x457B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x327C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x407D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x397E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C7F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x22801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02821500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x57071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x77081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x500A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x550B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x550C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x550D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x550E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x550F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x52131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55151500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B1A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F1C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x571D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x771E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x551F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x51201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x53291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x552A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x552B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x112C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F2D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D2E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B2F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x153A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x153B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x153C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x153D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x153E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x133F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08461500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06471500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04481500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17491500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x174A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x154B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x004C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x154D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x154E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x154F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x065C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x405D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x60631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x60641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xB4671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x60691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x606A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x106B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x046D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xBC751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x037D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x032D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x72121500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Page3 */
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x059B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x20AF1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*Page0 enable CABC*/
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02E71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04EC1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x81A91500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4FAC1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x33A01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_onyx_fiti_kd_hsd_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[jd936x] %s enter.\n", __func__);
	/* PAGE 0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PASSWORD */
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetVCOM */
	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x29011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2F041500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetGamma Power */
	data_array[0] = 0x10171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x101A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x011C1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* VCL_REG */
	data_array[0] = 0xFE241500;
	dsi_set_cmdq(data_array, 1, 1);
	/* AP */
	data_array[0] = 0x20251500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SAP */
	data_array[0] = 0x23351500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetPanel */
	data_array[0] = 0x09371500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SET RGBCYC */
	data_array[0] = 0x04381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x123A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x783C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x30451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x044B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* power voltage */
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x285A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F5B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Gamma */
	data_array[0] = 0x7C5D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x645E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x535F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3E611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x19641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x30661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x35691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3B6A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C6B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2A6C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F6D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x136E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C6F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7C701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x64711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x53721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3E741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x19771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x30791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E7A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A7B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x357C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3B7D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2A7F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x13811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 2*/
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5E001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x57021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x58031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x500F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x5E161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x57181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x58191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x451A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x471B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x491C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B1D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x411E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x51251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F2A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L_GS Pin mapping */
	data_array[0] = 0x1F2C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E2D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x172E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x182F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x11341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F411500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R_GS Pin mapping */
	data_array[0] = 0x1F421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x18451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A461500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08471500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06481500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04491500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x104A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F571500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP Timing */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x305B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B5C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x305D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x30601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1C631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x73671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1C691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A6A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xBB751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x24781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x037D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 3 lgx */
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x019A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x029B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01AF1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 4*/
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A0E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A961500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Add ESD Protect*/
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SLP OUT*/
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_onyx_fiti_kd_auo_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[jd936x] %s enter.\n", __func__);
	/* PAGE 0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PASSWORD */
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetVCOM */
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B041500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetGamma Power */
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xE7181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xE71B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* VCL_REG */
	data_array[0] = 0xFE241500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SAP */
	data_array[0] = 0x23351500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetPanel */
	data_array[0] = 0x59371500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SET RGBCYC */
	data_array[0] = 0x05381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x703C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x044B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* power voltage */
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x69571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x295A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E5B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Gamma */
	data_array[0] = 0x7F5D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x695E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A5F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x51671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6F681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5C691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x616A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x516B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B6C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D6D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2B6E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x69711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x517A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6F7B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5C7C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x617D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x517E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B7F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2B811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 2*/
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5C051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5C061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A0A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x470B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x470C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F0D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F0E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x450F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x51151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x5F161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A1A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B1C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x481D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x481E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x591F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x59201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x552A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x502B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L_GS Pin mapping */
	data_array[0] = 0x5F2C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F2D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x502E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C2F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4E341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x59371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x59381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x483A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B3B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B3C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x553F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40411500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R_GS Pin mapping */
	data_array[0] = 0x5F421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x51441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D461500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45471500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45481500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F491500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4F4A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x474B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x474C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A4D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A4E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x494F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5C511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5C521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41571500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP Timing */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x015C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xD05D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x70601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x19631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x66641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x11661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF7671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x19691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x666A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x186B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x046D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x046E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xD0771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x027A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x027B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x017C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x197D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x667E1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 3 */
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x019A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x029B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01AF1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 4*/
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x11091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x082B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x112D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x032E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A961500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49361500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Add ESD Protect*/
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SLP OUT*/
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
}

static void init_onyx_fiti_tg_boe_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[jd936x] %s enter.\n", __func__);
	/* PAGE 0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PASSWORD */
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetVCOM */
	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0041500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetGamma Power */
	data_array[0] = 0x10171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x101A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set Gate Power */
	data_array[0] = 0x3E1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2D211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E221500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetPanel */
	data_array[0] = 0x19371500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SET RGBCYC */
	data_array[0] = 0x05381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x123A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x783C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x044B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* power voltage */
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x89571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x285A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x195B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Gamma */
	data_array[0] = 0x7C5D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x625E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x515F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x38631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x24641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x66681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x54691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A6A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B6B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x466C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x396D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x276E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7C701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x62711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x51721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x31751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x38761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x24771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x467A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x667B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x547C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A7D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x467F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x39801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x27811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 2 */
	data_array[0] = 0x02E01500;
	/* GIP_L Pin mapping */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x430F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x46161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A1A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x481C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x481D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x401E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x42251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L_GS Pin mapping */
	data_array[0] = 0x112C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0F2D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0D2E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0B2F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x18331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x133F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F411500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R_GS Pin mapping */
	data_array[0] = 0x10421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08461500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06471500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04481500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x18491500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x174A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x004C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F571500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP Timing */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x305B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x405D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x62631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x62641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x74671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x62691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x666A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x086B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x086F1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 3*/
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x049B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x20AF1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 4*/
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF4B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 0*/
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06E71500;
	dsi_set_cmdq(data_array, 1, 1);
	/* CABC UI Mode*/
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SLP OUT*/
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
	/* For For CABC option*/
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0EAB1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_onyx_fiti_tah_tg_boe_lcm(void)
{
	unsigned int data_array[64];

	pr_debug("[jd936x] %s enter.\n", __func__);
	/* PAGE 0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PASSWORD */
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetVCOM */
	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x80011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x80041500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SetGamma Power */
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xAF181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xAF1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFE241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x19371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x013B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x703C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x13441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x044B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* power voltage */
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA9571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x375A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1A5B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Gamma */
	data_array[0] = 0x7F5D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A5E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B5F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x50601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3F621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x66681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x54691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A6A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C6B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x446C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x376D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x236E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x106F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5B721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x50731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3F751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x487A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x667B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x547C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5A7D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4C7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x447F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x37801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 2 */
	data_array[0] = 0x02E01500;
	/* GIP_L Pin mapping */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F0F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x57111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x77121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x35131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x4A161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x461A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x461B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x441C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x441D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x401E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5F261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x57271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x77281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x35291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L_GS Pin mapping */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x065C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x405D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x30601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6B641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0C661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x73671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x566A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x086B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x046D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x046E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xD5771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2E781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x037A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x037D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B7E1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 3*/
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PWM frq = 10k*/
	data_array[0] = 0x0E9B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x66A01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01AF1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40B01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 4*/
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xB3021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x60091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2A0E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x59361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01A91500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x68AA1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1A351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x35291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x242A1500;
	dsi_set_cmdq(data_array, 1, 1);
	/*PAGE 5*/
	data_array[0] = 0x05E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x18151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* PAGE 0*/
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* CABC UI Mode*/
	data_array[0] = 0xFF511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2C531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);
	/* SLP OUT*/
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);
	/* For For CABC option*/
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

/* ----------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_fiti_onyx_inx_inx_params(struct LCM_PARAMS *params)
{
	pr_debug("[jd936x] %s enter.\n", __func__);

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type      = LCM_TYPE_DSI;
	params->width     = FRAME_WIDTH;
	params->height    = FRAME_HEIGHT;
	params->dsi.mode  = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 12;
	params->dsi.vertical_frontporch			= 30;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 20;
	params->dsi.horizontal_backporch		= 30;
	params->dsi.horizontal_frontporch		= 30;
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
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_get_fiti_onyx_kd_hsd_params(struct LCM_PARAMS *params)
{
	pr_debug("[jd936x] %s enter.\n", __func__);

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type      = LCM_TYPE_DSI;
	params->width     = FRAME_WIDTH;
	params->height    = FRAME_HEIGHT;
	params->dsi.mode  = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 12;
	params->dsi.vertical_frontporch			= 30;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 20;
	params->dsi.horizontal_backporch		= 30;
	params->dsi.horizontal_frontporch		= 30;
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
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_get_fiti_onyx_kd_auo_params(struct LCM_PARAMS *params)
{
	pr_debug("[jd936x] %s enter.\n", __func__);

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type      = LCM_TYPE_DSI;
	params->width     = FRAME_WIDTH;
	params->height    = FRAME_HEIGHT;
	params->dsi.mode  = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 12;
	params->dsi.vertical_frontporch			= 30;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 20;
	params->dsi.horizontal_backporch		= 30;
	params->dsi.horizontal_frontporch		= 30;
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
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_get_fiti_onyx_tg_boe_params(struct LCM_PARAMS *params)
{
	pr_debug("[jd936x] %s enter.\n", __func__);

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type      = LCM_TYPE_DSI;
	params->width     = FRAME_WIDTH;
	params->height    = FRAME_HEIGHT;
	params->dsi.mode  = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 4;
	params->dsi.vertical_frontporch			= 38;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 20;
	params->dsi.horizontal_backporch		= 30;
	params->dsi.horizontal_frontporch		= 30;
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
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_get_fiti_tah_onyx_tg_boe_params(struct LCM_PARAMS *params)
{
	pr_debug("[jd936x] %s enter.\n", __func__);

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type      = LCM_TYPE_DSI;
	params->width     = FRAME_WIDTH;
	params->height    = FRAME_HEIGHT;
	params->dsi.mode  = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 22;
	params->dsi.vertical_frontporch			= 30;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 20;
	params->dsi.horizontal_backporch		= 23;
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
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_reset(void)
{
	/* GPIO20- Reset JD936X */
	pr_notice("[JD936X] %s enter\n", __func__);

	lcm_set_rst(1);
	MDELAY(6);
	lcm_set_rst(0);
	MDELAY(2);
	lcm_set_rst(1);
	MDELAY(6);
}

static char *lcm_get_vendor_type(void)
{
	switch (vendor_id) {
	case FITI_INX_INX: return "FITI_INX_INX\0";
	case FITI_KD_HSD: return "FITI_KD_HSD\0";
	case FITI_KD_AUO: return "FITI_KD_AUO\0";
	case FITI_TG_BOE: return "FITI_TG_BOE\0";
	case FITI_TAH_TG_BOE: return "FITI_TAH_TG_BOE\0";
	default: return "Unknown\0";
	}
}

static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	get_lcm_id();

	pr_info("[jd936x] %s enter, skip power_on & init lcm since it's done by lk\n",
			 __func__);
	pr_info("[jd936x] vendor type: %s\n",
			lcm_get_vendor_type());

	if (vendor_id == FITI_INX_INX)
		init_onyx_fiti_inx_inx_lcm();		/* FITI ONYX INX panel */
	else if (vendor_id == FITI_KD_HSD)
		init_onyx_fiti_kd_hsd_lcm();		/*FITI KD panel*/
	else if (vendor_id == FITI_KD_AUO)
		init_onyx_fiti_kd_auo_lcm();		/*FITI KD panel*/
	else if (vendor_id == FITI_TG_BOE)
		init_onyx_fiti_tg_boe_lcm();		/*FITI TG panel*/
	else if (vendor_id == FITI_TAH_TG_BOE)
		init_onyx_fiti_tah_tg_boe_lcm();
	else
		init_onyx_fiti_inx_inx_lcm();		/*FITI INX panel*/

	lcm_set_bl(1);
}

static void lcm_init_inx_inx_proto(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	get_lcm_id();

	init_onyx_fiti_inx_inx_proto_lcm();	/*FITI proto INX panel*/

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

	if (vendor_id == FITI_INX_INX)
		init_onyx_fiti_inx_inx_lcm();		/* FITI ONYX INX panel */
	else if (vendor_id == FITI_KD_HSD)
		init_onyx_fiti_kd_hsd_lcm();		/*FITI KD panel*/
	else if (vendor_id == FITI_KD_AUO)
		init_onyx_fiti_kd_auo_lcm();		/*FITI KD panel*/
	else if (vendor_id == FITI_TG_BOE)
		init_onyx_fiti_tg_boe_lcm();		/*FITI TG panel*/
	else if (vendor_id == FITI_TAH_TG_BOE)
		init_onyx_fiti_tah_tg_boe_lcm();
	else
		init_onyx_fiti_inx_inx_lcm();		/*FITI INX panel*/

	lcm_set_bl(1);
}

static void lcm_resume_proto(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	init_onyx_fiti_inx_inx_proto_lcm();	/*FITI proto INX panel*/

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
	MDELAY(6);
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
	MDELAY(6);
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
		return ;
	}
	pr_info("lcm driver cabc mode =%x\n", mode);
	dsi_set_cmdq_V2(LCM_CABC_MODE_REG, 1, ((unsigned char *)&mode), 1);
}

static void lcm_setbacklight_power_off(void)
{
	pr_notice("[Kernel/LCM] %s BL pull down \n", __func__);
	lcm_set_bl(0);
}

struct LCM_DRIVER jd9365d_wxga_dsi_vdo_fiti_inx_inx_onyx_lcm_drv = {
	.name		= "jd9365d_wxga_dsi_vdo_fiti_inx_inx_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_fiti_onyx_inx_inx_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};

struct LCM_DRIVER jd9365d_wxga_dsi_vdo_fiti_kd_hsd_onyx_lcm_drv = {
	.name		= "jd9365d_wxga_dsi_vdo_fiti_kd_hsd_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_fiti_onyx_kd_hsd_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};

struct LCM_DRIVER jd9365d_wxga_dsi_vdo_fiti_kd_auo_onyx_lcm_drv = {
	.name		= "jd9365d_wxga_dsi_vdo_fiti_kd_auo_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_fiti_onyx_kd_auo_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};

struct LCM_DRIVER jd9366ab_wxga_dsi_vdo_fiti_tg_boe_onyx_lcm_drv = {
	.name		= "jd9366ab_wxga_dsi_vdo_fiti_tg_boe_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_fiti_onyx_tg_boe_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};

struct LCM_DRIVER jd9365d_wxga_dsi_vdo_fiti_inx_inx_onyx_proto_lcm_drv = {
	.name		= "jd9365d_wxga_dsi_vdo_fiti_inx_inx_proto_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_fiti_onyx_inx_inx_params,
	.init           = lcm_init_inx_inx_proto,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume_proto,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};

struct LCM_DRIVER jd9365d_wxga_dsi_vdo_fiti_tah_tg_boe_onyx_lcm_drv = {
	.name		= "jd9365d_wxga_dsi_vdo_fiti_tah_tg_boe_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_fiti_tah_onyx_tg_boe_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};
