/*
 * sign_of_life_mtk.c
 *
 * MTK platform implementation
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Yang Liu (yangliu@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sign_of_life.h>
#include "../../misc/mediatek/include/mt-plat/mtk_rtc.h"
#include "../../misc/mediatek/rtc/include/mtk_rtc_hal.h"
#include "../../misc/mediatek/include/mt-plat/mtk_rtc_hal_common.h"
#include "../../misc/mediatek/include/mt-plat/mt8168/include/mach/mtk_pmic_wrap.h"
#include "../../misc/mediatek/rtc/mt6357/mtk_rtc_hw.h"
#include "../../misc/mediatek/include/mt-plat/mtk_boot_reason.h"

#define BOOT_REASON	"boot_reason="
#define BOOT_REASON_SZIE_MAX	8

/* RTC Spare Register Definition */

/*
 *	RTC_AL_DOW
 *	bit 0 ~ 7	:MTK use
 *	bit 8 ~ 12	:life cycle reasons
 *	bit 13		:life cycle reasons parity bit
 *	bit 14		:QUIESCENT mode
 *	bit 15		:reserved bits
 */
#define RTC_LIFE_CYCLE_REASON_FORBIDDEN_MASK		0x00ff
#define RTC_LIFE_CYCLE_REASON_MASK			0x1f00
#define RTC_LIFE_CYCLE_REASON_SHIFT			8
#define RTC_LIFE_CYCLE_REASON_PARITY_MASK		0x2000
#define RTC_LIFE_CYCLE_REASON_PARITY_SHIFT		13
#define PMIC_RTC_AL_DOW_QUIESCENT			(1U << 14)
#define PMIC_RTC_AL_DOW_QUIESCENT_MASK			(1U << 14)

/*match with rtc_life_cycle_reason_t*/
typedef enum {
	RTC_LIFE_CYCLE_NOT_AVAILABLE		= 0,
	/* Device Boot Reason */
	RTC_WARMBOOT_BY_KERNEL_PANIC,
	RTC_WARMBOOT_BY_KERNEL_WATCHDOG,
	RTC_WARMBOOT_BY_HW_WATCHDOG,
	RTC_WARMBOOT_BY_SW,
	RTC_COLDBOOT_BY_USB,
	RTC_COLDBOOT_BY_POWER_KEY,
	RTC_COLDBOOT_BY_POWER_SUPPLY,
	/* Device Shutdown Reason */
	RTC_SHUTDOWN_BY_LONG_PWR_KEY_PRESS,
	RTC_SHUTDOWN_BY_SW,
	RTC_SHUTDOWN_BY_PWR_KEY,
	RTC_SHUTDOWN_BY_SUDDEN_POWER_LOSS,
	RTC_SHUTDOWN_BY_UNKNOWN_REASONS,
	/* Device Thermal Shutdown Reason */
	RTC_THERMAL_SHUTDOWN_REASON_BATTERY,
	RTC_THERMAL_SHUTDOWN_REASON_PMIC,
	RTC_THERMAL_SHUTDOWN_REASON_SOC,
	RTC_THERMAL_SHUTDOWN_REASON_MODEM,
	RTC_THERMAL_SHUTDOWN_REASON_WIFI,
	RTC_THERMAL_SHUTDOWN_REASON_PCB,
	RTC_THERMAL_SHUTDOWN_REASON_BTS,
	/* LIFE CYCLE Special Mode */
	RTC_LIFE_CYCLE_SMODE_NONE,
	RTC_LIFE_CYCLE_SMODE_LOW_BATTERY,
	RTC_LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED,
	RTC_LIFE_CYCLE_SMODE_OTA,
	RTC_LIFE_CYCLE_SMODE_FACTORY_RESET,
	RTC_LIFE_CYCLE_REASON_MAX		= 31,
} rtc_life_cycle_reason_t;

/* sign_of_life_reason_data */
struct life_reason_data {
	rtc_life_cycle_reason_t lcr_rtc;
	life_cycle_reason_t lcr_reason;
};

static struct life_reason_data rtc_2_life_cycle_reason[] = {
	{RTC_LIFE_CYCLE_NOT_AVAILABLE, LIFE_CYCLE_NOT_AVAILABLE},
	/* Device Boot Reason */
	{RTC_WARMBOOT_BY_KERNEL_PANIC, WARMBOOT_BY_KERNEL_PANIC},
	{RTC_WARMBOOT_BY_KERNEL_WATCHDOG, WARMBOOT_BY_KERNEL_WATCHDOG},
	{RTC_WARMBOOT_BY_HW_WATCHDOG, WARMBOOT_BY_HW_WATCHDOG},
	{RTC_WARMBOOT_BY_SW, WARMBOOT_BY_SW},
	{RTC_COLDBOOT_BY_USB, COLDBOOT_BY_USB},
	{RTC_COLDBOOT_BY_POWER_KEY, COLDBOOT_BY_POWER_KEY},
	{RTC_COLDBOOT_BY_POWER_SUPPLY, COLDBOOT_BY_POWER_SUPPLY},
	/* Device Shutdown Reason */
	{RTC_SHUTDOWN_BY_LONG_PWR_KEY_PRESS, SHUTDOWN_BY_LONG_PWR_KEY_PRESS},
	{RTC_SHUTDOWN_BY_SW, SHUTDOWN_BY_SW},
	{RTC_SHUTDOWN_BY_PWR_KEY, SHUTDOWN_BY_PWR_KEY},
	{RTC_SHUTDOWN_BY_SUDDEN_POWER_LOSS, SHUTDOWN_BY_SUDDEN_POWER_LOSS},
	{RTC_SHUTDOWN_BY_UNKNOWN_REASONS, SHUTDOWN_BY_UNKNOWN_REASONS},
	/* Device Thermal Shutdown Reason */
	{RTC_THERMAL_SHUTDOWN_REASON_BATTERY, THERMAL_SHUTDOWN_REASON_BATTERY},
	{RTC_THERMAL_SHUTDOWN_REASON_PMIC, THERMAL_SHUTDOWN_REASON_PMIC},
	{RTC_THERMAL_SHUTDOWN_REASON_SOC, THERMAL_SHUTDOWN_REASON_SOC},
	{RTC_THERMAL_SHUTDOWN_REASON_MODEM, THERMAL_SHUTDOWN_REASON_MODEM},
	{RTC_THERMAL_SHUTDOWN_REASON_WIFI, THERMAL_SHUTDOWN_REASON_WIFI},
	{RTC_THERMAL_SHUTDOWN_REASON_PCB, THERMAL_SHUTDOWN_REASON_PCB},
	{RTC_THERMAL_SHUTDOWN_REASON_BTS, THERMAL_SHUTDOWN_REASON_BTS},
	/* LIFE CYCLE Special Mode */
	{RTC_LIFE_CYCLE_SMODE_NONE, LIFE_CYCLE_SMODE_NONE},
	{RTC_LIFE_CYCLE_SMODE_LOW_BATTERY, LIFE_CYCLE_SMODE_LOW_BATTERY},
	{RTC_LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED, LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED},
	{RTC_LIFE_CYCLE_SMODE_OTA, LIFE_CYCLE_SMODE_OTA},
	{RTC_LIFE_CYCLE_SMODE_FACTORY_RESET, LIFE_CYCLE_SMODE_FACTORY_RESET}
};

/*drivers/watchdog/mediatek/wdt/common/mtk_wdt.c*/
extern void mtk_wdt_mode_config(bool dual_mode_en, bool irq, bool ext_en, bool ext_pol, bool wdt_en);

extern void rtc_acquire_lock(void);
extern void rtc_release_lock(void);

static int search_rtc_use_lcr(const struct life_reason_data *table,
			      int num, life_cycle_reason_t reason)
{
	int i;

	for (i = 0; i < num; i++) {
		if (table[i].lcr_reason == reason)
			return i;
	}
	/* fails to match with any predefined reason, print a warnning msg */
	pr_warn("the lcr_reason %d is not in the pre-defined table\n", reason);

	/* fails to find a good reason, return the default index= 0 */
	return 0;
}

static int search_lcr_use_rtc(const struct life_reason_data *table,
			      int num, rtc_life_cycle_reason_t reason)
{
	int i;

	for (i = 0; i < num; i++) {
		if (table[i].lcr_rtc == reason)
			return i;
	}
	/* fails to match with any predefined reason, print a warnning msg */
	pr_warn("the lcr_rtc %d is not in the pre-defined table\n", reason);

	/* fails to find a good reason, return the default index= 0 */
	return 0;
}

static inline int life_cycle_reason_5bits_parity(rtc_life_cycle_reason_t v)
{
	int x = 0;
	int i;

	for (i = 0; i < 5; i++) {
		x ^= (v & 1);
		v >>= 1;
	}

	return x;
}

static void mtk_clear_life_cycle_reason(void)
{
	u16 rtc_value;

	rtc_acquire_lock();
	rtc_value = rtc_read(RTC_AL_DOW);
	rtc_value &= ~(RTC_LIFE_CYCLE_REASON_MASK |
			RTC_LIFE_CYCLE_REASON_PARITY_MASK);
	rtc_write(RTC_AL_DOW, rtc_value);
	rtc_write_trigger();
	rtc_release_lock();
}

static int mtk_read_life_cycle_reason(life_cycle_reason_t *reason)
{
	u16 rtc_value;
	rtc_life_cycle_reason_t rtc_breason = 0;
	u16 parity;
	int table_num = ARRAY_SIZE(rtc_2_life_cycle_reason);
	int index = 0;

	rtc_acquire_lock();
	rtc_value = rtc_read(RTC_AL_DOW);
	rtc_release_lock();

	rtc_breason = ((rtc_value & RTC_LIFE_CYCLE_REASON_MASK) >> RTC_LIFE_CYCLE_REASON_SHIFT);
	parity = ((rtc_value & RTC_LIFE_CYCLE_REASON_PARITY_MASK) >> RTC_LIFE_CYCLE_REASON_PARITY_SHIFT);

	pr_info("%s: rtc_value:0x%x,rtc_breason:0x%x,party:0x%x\n", __func__, rtc_value, rtc_breason, parity);

	if (life_cycle_reason_5bits_parity(rtc_breason) != parity) {
		pr_err("life cycle reason parity fail!\n");
		*reason = (life_cycle_reason_t)RTC_WARMBOOT_BY_SW;
	} else {
		index = search_lcr_use_rtc(rtc_2_life_cycle_reason, table_num, rtc_breason);
		*reason = rtc_2_life_cycle_reason[index].lcr_reason;
	}

	return 0;
}

static int mtk_write_life_cycle_reason(life_cycle_reason_t reason)
{
	u16 rtc_breason = 0;
	u16 life_cycle_breason = 0;
	int table_num = ARRAY_SIZE(rtc_2_life_cycle_reason);
	int index = 0;

	index = search_rtc_use_lcr(rtc_2_life_cycle_reason, table_num, reason);
	life_cycle_breason = rtc_2_life_cycle_reason[index].lcr_rtc;

	rtc_acquire_lock();
	rtc_breason = rtc_read(RTC_AL_DOW);
	rtc_release_lock();

	pr_info("%s:current 0x%x life cycle reason 0x%x\n", __func__, rtc_breason, reason);

	rtc_breason &= ~RTC_LIFE_CYCLE_REASON_MASK;
	rtc_breason |= ((life_cycle_breason << RTC_LIFE_CYCLE_REASON_SHIFT) & RTC_LIFE_CYCLE_REASON_MASK);

	rtc_breason &= ~RTC_LIFE_CYCLE_REASON_PARITY_MASK;
	rtc_breason |= ((life_cycle_reason_5bits_parity(life_cycle_breason) << RTC_LIFE_CYCLE_REASON_PARITY_SHIFT)
		& RTC_LIFE_CYCLE_REASON_PARITY_MASK);

	pr_info("%s:rtc_breason write 0x%x\n", __func__, rtc_breason);

	rtc_acquire_lock();
	rtc_write(RTC_AL_DOW, rtc_breason);
	rtc_write_trigger();
	rtc_release_lock();

	return 0;
}

static int mtk_read_reason_from_cmdline(life_cycle_reason_t *reason)
{
	int boot_reason = BR_UNKNOWN;
	char br_cmdline[BOOT_REASON_SZIE_MAX + 1] = {0};
	char *br_ptr = NULL;
	int ret = -1;
	int i = 0;
	int size = strlen(BOOT_REASON);

	br_ptr = strstr(saved_command_line, BOOT_REASON);
	if (br_ptr) {
		br_ptr += size;
		for (i = 0; i < BOOT_REASON_SZIE_MAX; i++) {
			br_cmdline[i] = *(br_ptr + i);
			if (br_cmdline[i] < '0' || br_cmdline[i] > '9') {
				br_cmdline[i] = '\0';
				break;
			}
		}

		ret = kstrtoint(br_cmdline, 10, &boot_reason);
		if (!ret) {
			if (boot_reason == BR_POWER_KEY)
				*reason = COLDBOOT_BY_POWER_KEY;
			else if (boot_reason == BR_USB)
				*reason = COLDBOOT_BY_USB;
		}
	}

	pr_info("%s: boot_reason = %d reason = 0x%x\n",
		__func__, boot_reason, *reason);

	return ret;
}

static int (mtk_read_boot_reason)(life_cycle_reason_t *boot_reason)
{
	return mtk_read_life_cycle_reason(boot_reason);
}

static int (mtk_write_boot_reason)(life_cycle_reason_t boot_reason)
{
	return mtk_write_life_cycle_reason(boot_reason);
}

static int (mtk_read_shutdown_reason)(life_cycle_reason_t *shutdown_reason)
{
	return mtk_read_life_cycle_reason(shutdown_reason);
}

static int (mtk_write_shutdown_reason)(life_cycle_reason_t shutdown_reason)
{
	return mtk_write_life_cycle_reason(shutdown_reason);
}

static int (mtk_read_thermal_shutdown_reason)(life_cycle_reason_t *thermal_shutdown_reason)
{
	return mtk_read_life_cycle_reason(thermal_shutdown_reason);
}

static int (mtk_write_thermal_shutdown_reason)(life_cycle_reason_t thermal_shutdown_reason)
{
	return mtk_write_life_cycle_reason(thermal_shutdown_reason);
}

static int (mtk_read_special_mode)(life_cycle_reason_t *special_mode)
{
	return mtk_read_life_cycle_reason(special_mode);
}

static int (mtk_write_special_mode)(life_cycle_reason_t special_mode)
{
	return mtk_write_life_cycle_reason(special_mode);
}

static int (mtk_read_from_cmdline)(life_cycle_reason_t *cmdline_reason)
{
	return mtk_read_reason_from_cmdline(cmdline_reason);
}

int rtc_mark_quiescent(int value)
{
	u32 rtc_special_mode;

	rtc_acquire_lock();
	pwrap_read(RTC_AL_DOW, &rtc_special_mode);
	if (value == 0)
		rtc_special_mode &= ~PMIC_RTC_AL_DOW_QUIESCENT;
	else
		rtc_special_mode |= PMIC_RTC_AL_DOW_QUIESCENT;
	rtc_write(RTC_AL_DOW, rtc_special_mode);
	rtc_write_trigger();
	rtc_release_lock();
	return 0;
}
EXPORT_SYMBOL(rtc_mark_quiescent);

int mtk_lcr_reset(void)
{
	mtk_clear_life_cycle_reason();
	return 0;
}

int life_cycle_platform_init(sign_of_life_ops *sol_ops)
{
	pr_info("%s: Support MTK platform\n", __func__);
	sol_ops->read_boot_reason = mtk_read_boot_reason;
	sol_ops->write_boot_reason = mtk_write_boot_reason;
	sol_ops->read_shutdown_reason = mtk_read_shutdown_reason;
	sol_ops->write_shutdown_reason = mtk_write_shutdown_reason;
	sol_ops->read_thermal_shutdown_reason = mtk_read_thermal_shutdown_reason;
	sol_ops->write_thermal_shutdown_reason = mtk_write_thermal_shutdown_reason;
	sol_ops->read_special_mode = mtk_read_special_mode;
	sol_ops->write_special_mode = mtk_write_special_mode;
	sol_ops->read_from_cmdline = mtk_read_from_cmdline;
	sol_ops->lcr_reset = mtk_lcr_reset;

	return 0;
}

void kree_disable_fiq(int);
void sysrq_trigger_lcr_test(int act)
{
	unsigned long flags;
	DEFINE_SPINLOCK(wdt_test_lock);

	switch (act) {
	case 3:
		spin_lock_irqsave(&wdt_test_lock, flags);
		/* mtk_wdt_mode_config(0, 0, 1, 0, 1); */
		while (1) {
			;
		};
		break;
	case 2:
		spin_lock(&wdt_test_lock);
		while (1)
			;
		break;
	default:
		*(volatile int *)NULL = 0;
		break;
	}
}
EXPORT_SYMBOL(sysrq_trigger_lcr_test);
