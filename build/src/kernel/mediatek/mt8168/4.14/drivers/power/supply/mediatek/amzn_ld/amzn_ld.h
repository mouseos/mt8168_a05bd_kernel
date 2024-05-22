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

#ifndef _AMZN_LD_H_
#define _AMZN_LD_H_

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/switch.h>

struct adc_step_val {
	int sbu1_step1_val;
	int sbu1_step2_val;
	int sbu2_step1_val;
	int sbu2_step2_val;
};

struct liquid {
	bool gpio_reverse_ctrl;
	unsigned int work_interval[3];
	unsigned int threshold[2];
	int gpio_liquid_id;

	struct adc_step_val adc_step_val;
	struct iio_channel *adc_ch;
	struct platform_device *pdev;
	struct tcpc_device *tcpc;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct switch_dev st_switch;
	struct wakeup_source wake_lock;
	struct delayed_work routine_work;
	struct delayed_work delayed_work;
	struct mutex ld_io_mutex;
	/* For metrics */
	struct timespec event_ts;

	struct pinctrl *ld_pinctrl;
	struct pinctrl_state *ld_ctrl1_init;
	struct pinctrl_state *ld_ctrl1_low;
	struct pinctrl_state *ld_ctrl1_high;
	struct pinctrl_state *ld_ctrl2_init;
	struct pinctrl_state *ld_ctrl2_low;
	struct pinctrl_state *ld_ctrl2_high;
	struct pinctrl_state *ld_detection1_init;
	struct pinctrl_state *ld_detection1_low;
	struct pinctrl_state *ld_detection1_high;
	struct pinctrl_state *ld_detection2_init;
	struct pinctrl_state *ld_detection2_low;
	struct pinctrl_state *ld_detection2_high;

	struct charger_device *chg1_dev;
	struct notifier_block chg1_nb;
};

enum LD_DETECT_DELY {
	DET_DELAY_SBU1 = 0,
	DET_DELAY_SBU2 = 1,
};

enum LD_STATE_TYPE {
	TYPE_DRY = 0,
	TYPE_WET = 1,
	TYPE_WET_VBUS = 2,
};

enum LD_WORK_INTERVAL {
	INTERVAL_DRY = 0,
	INTERVAL_WET = 1,
	INTERVAL_VBUS = 2,
};

enum LD_DETECTION_SWITCH {
	DETECTION_SBU1 = 1,
	DETECTION_SBU2 = 2,
	DETECTION_LD = 3,
	DETECTION_BATID = 4,
};

enum LD_SBU_PULL {
	SBU1_PULL_UP = 1,
	SBU1_PULL_DOWN = 2,
	SBU2_PULL_UP = 3,
	SBU2_PULL_DOWN = 4,
};

enum LD_THRESHOLD {
	THRESHOLD_L = 0,
	THRESHOLD_H = 1,
};

#define WORK_INTERVAL_DRY                            15
#define WORK_INTERVAL_WET                            30
#define WORK_INTERVAL_VBUS                           60
#define LD_ADC_CHANNEL                               4
#define IUSB_LIMITATION_UA                           10000
#define INVALID_VBUS_UV                              3000000
#define RECHECK_DELAY_SEC                            5
#define WORK_DETECT_VBUS                             300
#define STATE_CHANGE_MS                              20
#define ADC_MAX_MV                                   1500
#define THRESHOLD_VAL_L                              120
#define THRESHOLD_VAL_H                              1450
#define STEP1_DEFAULT_VAL                            0
#define STEP2_DEFAULT_VAL                            1499
#define DETECT_TWICE_MS                              20

#endif
