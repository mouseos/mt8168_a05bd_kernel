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

#ifndef _MAX20342_H_
#define _MAX20342_H_

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#include <linux/metricslog.h>
#endif

/* Start register map for the MAX20342 */
/* USER_INTERRUPTS */
#define REVISION_ID                     0x00
#define COMMON_INT                      0x01
#define CC_INT                          0x02
#define BC_INT                          0x03
#define OVP_INT                         0x04
#define RES_INT1                        0x05
#define RES_INT2                        0x06
#define COMMON_STATUS                   0x07
#define CC_STATUS1                      0x08
#define CC_STATUS2                      0x09
#define BC_STATUS                       0x0A
#define OVP_STATUS                      0x0B
#define COMMON_MASK                     0x0C
#define CC_MASK                         0x0D
#define BC_MASK                         0x0E
#define OVP_MASK                        0x0F
#define RES_MASK1                       0x10
#define RES_MASK2                       0x11
/* USER_COMMON */
#define COMM_CTRL1                      0x15
#define COMM_CTRL2                      0x16
#define RFU_RW                          0x17
#define RFU_RO                          0x18
#define COMM_CTRL3                      0x19
/* USER_OVP */
#define OVP_CTRL                        0x1A
/* USER_USBC */
#define CC_CTRL0                        0x20
#define CC_CTRL1                        0x21
#define CC_CTRL2                        0x22
#define CC_CTRL3                        0x23
#define CC_CTRL4                        0x24
#define CC_CTRL5                        0x25
#define CC_CTRL6                        0x26
#define VCONN_ILIM                      0x28
/* USER_BC12 */
#define BC_CTRL0                        0x2A
#define BC_CTRL1                        0x2B
/* SBUDetResult */
#define SBU1DetResult1                  0x2C
#define SBU1DetResult2                  0x2D
#define SBU2DetResult1                  0x2E
#define SBU2DetResult2                  0x2F
/* SBUDetConfig */
#define SBUDetCtrl                      0x30
#define RAcc1DetVMax                    0x31
#define RAcc1DetVmin                    0x32
#define RAcc1Detlpu                     0x33
#define RAcc2DetVmax                    0x34
#define RAcc2DetVmin                    0x35
#define RAcc2Detlpu                     0x36
#define RAcc3DetVmax                    0x37
#define RAcc3DetVmin                    0x38
#define RAcc3Detlpu                     0x39
#define RAcc4DetVmax                    0x3A
#define RAcc4DetVmin                    0x3B
#define RAcc4Detlpu                     0x3C
#define RAcc5DetVmax                    0x3D
#define RAcc5DetVmin                    0x3E
#define RAcc5Detlpu                     0x3F
/* MoistDet */
#define RMoistDetVth                    0x50
#define MoistDetCtrl                    0x51
#define MoistDetPUConfig                0x52
#define MoistDetPDConfig                0x53
#define MoistDetAutoCC1Result1          0x54
#define MoistDetAutoCC1Result2          0x55
#define MoistDetAutoCC2Result1          0x56
#define MoistDetAutoCC2Result2          0x57
#define MoistDetAutoSBU1Result1         0x58
#define MoistDetAutoSBU1Result2         0x59
#define MoistDetAutoSBU2Result1         0x5A
#define MoistDetAutoSBU2Result2         0x5B
/* ADCConfig */
#define ADCCtrl1                        0x5C
#define ADCCtrl2                        0x5D
#define ADC_CTRL3                       0x5E
#define ADC_CTRL4                       0x5F
#define ADCResultAvg                    0x60
#define ADCResultMax                    0x61
#define ADCResultMin                    0x62
/* USER_VB */
#define VB_CTRL                         0x63
/* End register map for the MAX20342 */

struct max20342 {
	unsigned char target_ipu;
	unsigned char target_resister;
	unsigned char target_voltage_min;
	unsigned char target_voltage_max;
	unsigned char target_adcgroundvth;

	unsigned char ipu_result1;
	unsigned char ipu_result2;
	unsigned char adc_result_avg1;
	unsigned char adc_result_avg2;
	unsigned char ld_buffer[2];

	int work_interval[3];
	int det_delay[2];

	struct i2c_client *i2c;
	struct device *dev;
	struct tcpc_device *tcpc;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct switch_dev st_switch;
	struct wakeup_source wake_lock;
	struct delayed_work routine_work;
	struct delayed_work delayed_work;
	struct mutex max20342_io_mutex;
	/* For metrics */
	struct timespec event_ts;
	struct charger_device *chg1_dev;
	struct notifier_block chg1_nb;

	struct pinctrl *mos_pinctrl;
	struct pinctrl_state *mos_ctrl1_init;
	struct pinctrl_state *mos_ctrl1_low;
	struct pinctrl_state *mos_ctrl1_high;
};

enum MAX20342_DETECT_DELY {
	DET_DELAY_SBU1 = 0,
	DET_DELAY_SBU2 = 1,
};

enum MAX20342_STATE_TYPE {
	TYPE_DRY = 0,
	TYPE_WET = 1,
	TYPE_WET_VBUS = 2,
};

enum MAX20342_WORK_INTERVAL {
	INTERVAL_DRY = 0,
	INTERVAL_WET = 1,
	INTERVAL_VBUS = 2,
};

enum LDBUFFER_THRESHOLD {
	THRESHOLD_L = 0,
	THRESHOLD_H = 1,
};

#define MAX20342_REVISION_ID            0x01

/* pull up current - RMoistDetIpu */
#define PULL_UP_CURRENT_MASK            0x03
#define PULL_UP_CURRENT_SHIFT           0x00
#define PULL_UP_CURRENT_VALUE_2UA       0x00
#define PULL_UP_CURRENT_VALUE_8UA       0x01
#define PULL_UP_CURRENT_VALUE_32UA      0x02
#define PULL_UP_CURRENT_VALUE_128UA     0x03

/* moisture detection mode */
#define MOIST_DET_MAN_EN_MASK           0x01
#define MOIST_DET_MAN_EN_SHIFT          0x02
#define MOIST_DET_MAN_EN_VAL_TRUE       0x01
#define MOIST_DET_MAN_EN_VAL_FALSE      0x00

#define MOIST_DET_PER_EN_MASK           0x01
#define MOIST_DET_PER_EN_SHIFT          0x03
#define MOIST_DET_PER_EN_VAL_FALSE      0x00

#define MOIST_DET_AUTO_CFG_MASK         0x01
#define MOIST_DET_AUTO_CFG_SHIFT        0x04
#define MOIST_DET_AUTO_CFG_VAL_FALSE    0x00

/* detection FSM enable - CCDetEn */
#define FSM_ENABLE_MASK                 0x01
#define FSM_ENABLE_SHIFT                0x00
#define FSM_ENABLE_VAL_TRUE             0x01
#define FSM_ENABLE_VAL_FALSE            0x00

/* adc sample - ADCAvgNum */
#define ADC_SAMPLE_MASK                 0x07
#define ADC_SAMPLE_SHIFT                0x05
#define ADC_SAMPLE_SAMPLE_VAL_1         0x00
#define ADC_SAMPLE_SAMPLE_VAL_2         0x01
#define ADC_SAMPLE_SAMPLE_VAL_4         0x02
#define ADC_SAMPLE_SAMPLE_VAL_8         0x03
#define ADC_SAMPLE_SAMPLE_VAL_16        0x04
#define ADC_SAMPLE_SAMPLE_VAL_32        0x05
#define ADC_SAMPLE_SAMPLE_VAL_64        0x06
#define ADC_SAMPLE_SAMPLE_VAL_128       0x07

/* adc retry num - ADCRetryNum */
#define ADC_RETRY_NUM_MASK              0x07
#define ADC_RETRY_NUM_SHIFT             0x00
#define ADC_RETRY_NUM_VAL_1             0x01

/* adc ground voltage - ADCGroundVth */
#define ADC_GROUND_VTH_MASK             0x0F
#define ADC_GROUND_VTH_SHIFT            0x03

/* pull_up_current result - IpuResult */
#define IPU_RESULT_MASK                 0x03
#define IPU_RESULT_SHIFT                0x06

/* adc measure result */
#define ADC_RESUIT_AVG_MASK             0xFF
#define ADC_RESUIT_AVG_SHIFT            0x00

#define ADC_RESUIT_MAX_MASK             0xFF
#define ADC_RESUIT_MAX_SHIFT            0x00

#define ADC_RESUIT_MIN_MASK             0xFF
#define ADC_RESUIT_MIN_SHIFT            0x00

/* adc noise - ADCNoiseClampRng */
#define ADC_NOISE_CLAMP_RNG_MASK        0x3F
#define ADC_NOISE_CLAMP_RNG_SHIFT       0x00

/* SwReset - COMM_CTRL3 */
#define SW_RESET_MASK                   0x01
#define SW_RESET_SHIFT                  0x01
#define SW_RESET_VALUE_TRUE             0x01

/* moist pull up switch close - MoistDetPUConfig */
#define MOIST_PULL_UP_SWITCH_MASK       0x3F
#define MOIST_PULL_UP_SWITCH_SHIFT      0x00
#define MOIST_PULL_UP_SWITCH_CC1        0x01
#define MOIST_PULL_UP_SWITCH_CC2        0x02
#define MOIST_PULL_UP_SWITCH_SBU1       0x04
#define MOIST_PULL_UP_SWITCH_SBU2       0x08
#define MOIST_PULL_UP_SWITCH_CDP        0x10
#define MOIST_PULL_UP_SWITCH_CDN        0x20
#define MOIST_PULL_UP_ALL_OPEN          0x00

/* moist pull down switch open - MoistDetPDConfig */
#define MOIST_PULL_DOWN_SWITCH_MASK     0x7F
#define MOIST_PULL_DOWN_SWITCH_SHIFT    0x00
#define MOIST_PULL_DOWN_SWITCH_CC1      0x01
#define MOIST_PULL_DOWN_SWITCH_CC2      0x02
#define MOIST_PULL_DOWN_SWITCH_SBU1     0x04
#define MOIST_PULL_DOWN_SWITCH_SBU2     0x08
#define MOIST_PULL_DOWN_SWITCH_CDP      0x10
#define MOIST_PULL_DOWN_SWITCH_CDN      0x20
#define MOIST_PULL_DOWN_SWITCH_VB       0x40
#define MOIST_PULL_DOWN_ALL_CLOSE       0x7F
#define MOIST_PULL_DOWN_ALL_OPEN        0x00

/* bc1.2 - BC_CTRL1  */
#define CHGDETEN_MASK                   0x01
#define CHGDETEN_SHIFT                  0x00
#define CHGDETEN_VAL_DISABLE            0x00

/* LPDRP mode  */
#define LPDRD_MASK                      0x01
#define LPDRD_SHIFT                     0x02
#define LPDRD_VAL                       0x01

/* user parameter  */
#define WORK_INTERVAL_DRY               15
#define WORK_INTERVAL_WET               30
#define WORK_INTERVAL_VBUS              60

#define WORK_DETECT_VBUS                300

#define SBU1_DET_DELAY_VAL              300
#define SBU2_DET_DELAY_VAL              300

#define RECHECK_DELAY_SEC               5

#define IUSB_LIMITATION_UA              10000
#define INVALID_VBUS_UV                 2600000

#define MASK_ALL_INT_VALUE              0x00

/* current:8uA */
#define TARGET_IPU                      1

/* resister:100K */
#define TARGET_RESISTER                 100


/*
r_max = 110K
current*resister/1000*(1+4.7%)*(1+1%)
= 8*110/1000*1.047*1.01 = 0.954
5.882*163 = 0.958
 */
#define TARGET_VOLTAGE_MAX              163

/* 4*5.882mv =  23.528mv */
#define TARGET_ADCGROUNDVTH             4
#define THRESHOLD_L_VAL                 86
#define THRESHOLD_H_VAL                 114

unsigned char mask_int_reg[] = {COMMON_MASK, CC_MASK, BC_MASK,
	 OVP_MASK, RES_MASK1, RES_MASK2};

unsigned char dump_regs[] = {REVISION_ID, COMMON_INT, CC_INT, BC_INT,
	 OVP_INT, RES_INT1, RES_INT2, COMMON_STATUS, CC_STATUS1, CC_STATUS2,
	 BC_STATUS, OVP_STATUS, COMMON_MASK, CC_MASK, BC_MASK, OVP_MASK,
	 RES_MASK1, RES_MASK2, COMM_CTRL1, COMM_CTRL2, RFU_RW, RFU_RO,
	 COMM_CTRL3, OVP_CTRL, CC_CTRL0, CC_CTRL1, CC_CTRL2, CC_CTRL3,
	 CC_CTRL4, CC_CTRL5, CC_CTRL6, VCONN_ILIM, BC_CTRL0, BC_CTRL1,
	 SBU1DetResult1, SBU1DetResult2, SBU2DetResult1, SBU2DetResult2,
	 SBUDetCtrl, RAcc1DetVMax, RAcc1DetVmin, RAcc1Detlpu, RAcc2DetVmax,
	 RAcc2DetVmin, RAcc2Detlpu, RAcc3DetVmax, RAcc3DetVmin, RAcc3Detlpu,
	 RAcc4DetVmax, RAcc4DetVmin, RAcc4Detlpu, RAcc5DetVmax, RAcc5DetVmin,
	 RAcc5Detlpu, RMoistDetVth, MoistDetCtrl, MoistDetPUConfig, MoistDetPDConfig,
	 MoistDetAutoCC1Result1, MoistDetAutoCC1Result2, MoistDetAutoCC2Result1,
	 MoistDetAutoCC2Result2, MoistDetAutoSBU1Result1, MoistDetAutoSBU1Result2,
	 MoistDetAutoSBU2Result1, MoistDetAutoSBU2Result2, ADCCtrl1, ADCCtrl2, ADC_CTRL3,
	 ADC_CTRL4, ADCResultAvg, ADCResultMax, ADCResultMin, VB_CTRL};

#endif
