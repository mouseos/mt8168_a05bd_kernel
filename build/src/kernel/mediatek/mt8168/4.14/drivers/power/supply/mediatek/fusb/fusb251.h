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

#ifndef __FUSB251_INC__
#define __FUSB251_INC__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#include <linux/metricslog.h>
#endif

#define FUSB251_DEVICE_ID       0x80

/* Register map for the FUSB251 */
#define FUSB251_PRODUCT_ID      0x01
#define FUSB251_CONTROL         0x02
#define FUSB251_INTERRUPT       0x03
#define FUSB251_INT_MASK        0x04
#define FUSB251_STATUS          0x05
#define FUSB251_MOSSTATUS       0x06
#define FUSB251_SWITCHCONTROL   0x07
#define FUSB251_THRESHOLD1      0x08
#define FUSB251_THRESHOLD2      0x09
#define FUSB251_TIMER1          0x0A
#define FUSB251_RESET           0x0B
#define FUSB251_TIMER2          0x0C
#define FUSB251_REG_NUM         12

/* for interrupt */
#define MSK_I_OVP               1
#define MSK_I_OVP_REC           (1 << 1)
#define MSK_I_MOS_DET           (1 << 2)
#define MSK_I_DRY_DET           (1 << 3)
#define MSK_I_CC1_TIMER         (1 << 4)
#define MSK_I_CC2_TIMER         (1 << 5)
#define MSK_I_MASK_ALL          0x3F
/* for control */
#define CTL_EN_MAN_MODE         (1 << 0)
#define CTL_EN_CC_DET           (1 << 1)
#define CTL_EN_SBU_DET          (1 << 2)
#define CTL_EN_SBUFT_DET        (1 << 3)
#define CTL_EN_AUTO_SBU_DET     (1 << 4)
#define CTL_EN_DRY_DET          (1 << 5)
#define CTL_DISABLE_ALL         0x00
#define EN_CC_DET_MASK          0x01
#define EN_CC_DET_SHIFT         1
#define EN_SBU_DET_MASK         0x01
#define EN_SBU_DET_SHIFT        2
#define EN_SBUFT_DET_MASK       0x01
#define EN_SBUFT_DET_SHIFT      3
#define EN_AUTO_SBU_DET_MASK    0x01
#define EN_AUTO_SBU_DET_SHIFT   4
#define EN_DRY_DET_MASK         0x01
#define EN_DRY_DET_SHIFT        5
/* for status */
#define STATUS_OVP_CC_MASK      0x01
#define STATUS_OVP_CC_SHIFT     0
#define STATUS_OVP_SBU_MASK     0x01
#define STATUS_OVP_SBU_SHIFT    1
#define STATUS_LOOK4DRY_MASK    0x01
#define STATUS_LOOK4DRY_SHIFT   5
#define STATUS_LOOK4SBU_MASK    0x01
#define STATUS_LOOK4SBU_SHIFT   6
#define STATUS_LOOK4CC_MASK     0x01
#define STATUS_LOOK4CC_SHIFT    7
/* for moisture status */
#define MOSSTATUS_CC1MOS_MASK   0x01
#define MOSSTATUS_CC1MOS_SHIFT  0
#define MOSSTATUS_CC2MOS_MASK   0x01
#define MOSSTATUS_CC2MOS_SHIFT  1
#define MOSSTATUS_SBU1MOS_MASK  0x01
#define MOSSTATUS_SBU1MOS_SHIFT 2
#define MOSSTATUS_SBU2MOS_MASK  0x01
#define MOSSTATUS_SBU2MOS_SHIFT 3
#define MOSSTATUS_SBU1FT_MASK   0x01
#define MOSSTATUS_SBU1FT_SHIFT  4
#define MOSSTATUS_SBU2FT_MASK   0x01
#define MOSSTATUS_SBU2FT_SHIFT  5
/* for switch */
#define SWITCH_CC_MASK          0x01
#define SWITCH_CC_SHIFT         0
#define SWITCH_SBU_MASK         0x03
#define SWITCH_SBU_SHIFT        1
/* for reset*/
#define RESET_DEVICE            0x01
#define RESET_MOSSTURE          0x02
/* for threshold1 */
#define SBU_MOS_R_DET_MASK      0x0F
#define SBU_MOS_R_DET_SHIFT     4
#define CC_MOS_R_DET_MASK       0x0F
#define CC_MOS_R_DET_SHIFT      0
/* for threshold2 */
#define SBU_FLOAT_DET_MASK      0x07
#define SBU_FLOAT_DET_SHIFT     4
#define VDRY_R_DET_MASK         0x0F
#define VDRY_R_DET_SHIFT        0

enum FUSB251_STATE_TYPE {
	TYPE_DRY = 0,
	TYPE_WET = 1,
	TYPE_WET_VBUS = 2,
};

enum FUSB251_WORK_INTERVAL {
	INTERVAL_DRY = 0,
	INTERVAL_WET = 1,
	INTERVAL_VBUS = 2,
};

enum FUSB251_THRESHOLD_SBU {
	THRESHOLD_DRY2WET = 0,
	THRESHOLD_WET2DRY = 1,
};

enum FUSB251_DETECT_DELAY {
	DET_DELAY_SBUFT = 0,
	DET_DELAY_SBU = 1,
};

struct fusb251 {
	struct i2c_client *i2c;
	struct device *dev;
	struct tcpc_device *tcpc;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct switch_dev st_switch;
	struct wakeup_source wake_lock;
	struct delayed_work routine_work;
	struct mutex fusb251_io_mutex;

	/* For metrics */
	struct timespec event_ts;

	unsigned char status_ovp_cc;
	unsigned char status_ovp_sbu;
	unsigned char mos_status_sbu1;
	unsigned char mos_status_sbu2;
	unsigned char mos_status_sbu1_ft;
	unsigned char mos_status_sbu2_ft;

	int threshold_sbu[2];
	int threshold_sbuft;
	int work_interval[3];
	int det_delay[2];
};

/* Voltage table for sbu float detection */
int sbuft_det_MV_table[] = {100, 200, 300, 400, 500, 600,
	700, 800};

/* Resistance table for moisture detection */
int mos_det_R_table[] = {17, 36, 56, 80, 107, 137, 172, 213,
	262, 320, 391, 480, 594, 747, 960, 1280};

/* For metrics */
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#define METRICS_BUFF_SIZE_FUSB251 512
char g_m_buf[METRICS_BUFF_SIZE_FUSB251];

#define fusb251_metrics_log(domain, fmt, ...)			\
do {								\
	memset(g_m_buf, 0, METRICS_BUFF_SIZE_FUSB251);		\
	snprintf(g_m_buf, sizeof(g_m_buf), fmt, ##__VA_ARGS__);	\
	log_to_metrics(ANDROID_LOG_INFO, domain, g_m_buf);	\
} while (0)
#else
static inline void fusb251_metrics_log(void) {}
#endif

#define MAX_VOLTAGE             800
#define MAX_RESISTANCE          1280
#define IUSB_LIMITATION_UA      10000
#define INVALID_VBUS_UV         2600000
#define THRESHOLD_SBU_DRY2WET   747
#define THRESHOLD_SBU_WET2DRY   960
#define THRESHOLD_SBUFT         100
#define WORK_INTERVAL_DRY       15
#define WORK_INTERVAL_WET       30
#define WORK_INTERVAL_VBUS      60
#define SBUFT_DET_DELAY         100
#define SBU_DET_DELAY           700
#define RECHECK_DELAY_SEC       5

#endif