/*
 * stk8baxx_driver.c - Linux driver for sensortek stk8baxx accelerometer
 * Copyright (C) 2017 Sensortek
 */
#include <accel.h>
#include "cust_acc.h"
#include "sensors_io.h"
#include "stk8baxx.h"

/********* Global *********/
/* #define STK_FIR */ /* low-pass mode */
/* Any motion only works under either STK_INTERRUPT_MODE or STK_POLLING_MODE */
#ifdef STK_FIR
	#define STK_FIR_LEN         5
	#define STK_FIR_LEN_MAX     32
#endif /* STK_FIR */

#define STK_ACC_TAG                 "[stk8baxx]"
#define STK_ACC_FUN(f)              pr_info(STK_ACC_TAG" %s\n", __func__)
#define STK_ACC_ERR(fmt, args...)   pr_err(STK_ACC_TAG" %s %d: "fmt"\n", __func__, __LINE__, ##args)
#define STK_ACC_LOG(fmt, args...)   pr_info(STK_ACC_TAG ""fmt"\n", ##args)
#define STK_ACC_DBG(fmt, args...)   pr_debug(STK_ACC_TAG ""fmt"\n", ##args)

/* STK8BAXX_REG_BWSEL */
#define STK8BAXX_BWSEL_INIT_ODR          0xB    /* ODR = BW x 2 = 125Hz */
#define STK_ACC_DEV_NAME "stk8baxx"

/* axis */
#define STK_AXIS_X                  0
#define STK_AXIS_Y                  1
#define STK_AXIS_Z                  2
#define STK_AXES_NUM                3
/* add for diag */
#define ACCEL_SELF_TEST_MIN_VAL 0
#define ACCEL_SELF_TEST_MAX_VAL 13000
#define STK_ACC_CALI_FILE "/data/inv_cal_data.bin"
#define STK_DATA_BUF_NUM 3
#define CALI_SIZE 3 /*CALI_SIZE should not less than 3*/


#ifdef STK_FIR
struct data_fir {
	s16 xyz[STK_FIR_LEN_MAX][3];
	int sum[3];
	int idx;
	int count;
};
#endif /* STK_FIR */

enum STK_TRC {
	STK_TRC_INFO = 0X01,
	STK_TRC_RAWDATA = 0x02,
};

struct stk8baxx_data {
	/* platform related */
	struct i2c_client           *client;
	struct acc_hw               hw;
	struct hwmsen_convert       cvt;
	/* chip informateion */
	u8                          pid;
	/* system operation */
	atomic_t                    suspend;            /* chip is suspend or not */
	atomic_t                    enabled;            /* chip is enabled or not */
	atomic_t                    trace;              /* trace gsensor data */
	int                         cali_sw[STK_AXES_NUM];  /* cali data */
	atomic_t                    recv;               /* recv data. DEVICE_ATTR(recv, ...) */
	struct mutex                reg_lock;           /* mutex lock for register R/W */
	u8                          power_mode;
	bool                        temp_enable;        /* record current power status. For Suspend/Resume used. */
	int                         sensitivity;        /* sensitivity, bit number per G */
	s16                         xyz[3];             /* The latest data of xyz */
	atomic_t                    selftest;           /* selftest result */
#ifdef STK_FIR
	struct data_fir             fir;
	/*
	 * fir_len
	 * 0/1: turn OFF/ON FIR operation
	 */
	atomic_t                    fir_len;
#endif /* STK_FIR */
};

/* ODR: 31.25, 62.5, 125 */
const static int STK8BAXX_SAMPLE_TIME[] = {32000, 16000, 8000}; /* usec */
static bool sensor_power = true;

static struct stk8baxx_data *stk_data;
static int stk8baxx_init_flag = -1;

static int accel_self_test[STK_AXES_NUM] = {0};
static s16 accel_xyz_offset[STK_AXES_NUM] = {0};
/* default tolenrance is 20% */
static int accel_cali_tolerance = 20;

static int stk_reg_init(struct stk8baxx_data *stk);
static int stk_readsensordata(int *pdata_x, int *pdata_y, int *pdata_z);
static int stk_writeCalibration(int *dat);

static int stk_acc_init(void);
static int stk_acc_remove(void);

extern unsigned int idme_get_sensorcal(s16 *data, uint8_t size);

static struct acc_init_info stk_acc_init_info = {
	.name = STK_ACC_DEV_NAME,
	.init = stk_acc_init,
	.uninit = stk_acc_remove,
};

static inline int stk_selftest_offset_factor(int sen)
{
	return sen * 3 / 10;
}
static inline int stk_selftest_noise_factor(int sen)
{
	return sen / 10;
}

/********* Functions *********/
static int stk8baxx_i2c_write(struct stk8baxx_data *stk, u8 reg, u8 val)
{
	int error = 0;

	error = i2c_smbus_write_byte_data(stk->client, reg, val);
	if (error)
		STK_ACC_ERR("transfer failed to write reg:0x%x with val:0x%x, error=%d", reg, val, error);

	return error;
}


static int stk8baxx_i2c_read(struct stk8baxx_data *stk, u8 reg, int len, u8 *val)
{
	int error = 0;
	struct i2c_msg msgs[2] = {
		{
			.addr = stk->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg
		},
		{
			.addr = stk->client->addr,
			.flags = I2C_M_RD,
			.len = (0 >= len) ? 1 : len,
			.buf = val
		}
	};

	error = i2c_transfer(stk->client->adapter, msgs, 2);

	if (error != 2) {
		STK_ACC_ERR("i2c_transfer error: (%d %p %d) %d", stk->client->addr, val, len, error);
		error = -EIO;
	} else {
		error = 0;
	}

	return error;
}


/**
 * stk8baxx register write
 * @brief: Register writing via I2C
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] reg: Register address
 * @param[in] val: Data, what you want to write.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk8baxx_reg_write(struct stk8baxx_data *stk, u8 reg, u8 val)
{
	int error = 0;

	if (!stk) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&stk->reg_lock);
	error = stk8baxx_i2c_write(stk, reg, val);
	mutex_unlock(&stk->reg_lock);

	return error;
}

/**
 * stk8baxx register read
 * @brief: Register reading via I2C
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] reg: Register address
 * @param[in] len: 0/1, for normal usage.
 * @param[out] val: Data, the register what you want to read.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk8baxx_reg_read(struct stk8baxx_data *stk, u8 reg, int len, u8 *val)
{
	int error = 0;

	if (!stk) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&stk->reg_lock);
	error = stk8baxx_i2c_read(stk, reg, len, val);
	mutex_unlock(&stk->reg_lock);

	return error;
}

static int stk8baxx_reg_update_bit(struct stk8baxx_data *stk, unsigned char reg,
					unsigned char mask, int value)
{
	int ret = 0;
	unsigned char reg_val = 0;

	if (!stk) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&stk->reg_lock);

	ret = stk8baxx_i2c_read(stk, reg, 1, &reg_val);

	reg_val &= mask;
	reg_val |= value;

	ret = stk8baxx_i2c_write(stk, reg, reg_val);

	mutex_unlock(&stk->reg_lock);

	return ret;
}

/**
 * @brief: Read PID and write to stk8baxx_data.pid.
 *
 * @param[in/out] stk: struct stk8baxx_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_check_pid(struct stk8baxx_data *stk)
{
	int error = 0;
	u8 pid = 0;

	error = stk8baxx_reg_read(stk, STK8BAXX_REG_CHIPID, 0, &pid);
	if (error) {
		STK_ACC_ERR("i2c error,failed to read PID");
		return STK8BAXX_ERR_IDENTIFICATION;
	}
	stk->pid = pid;

	STK_ACC_LOG("PID 0x%x", pid);

	if (STK8BA50_R_ID != pid
		&& STK8BA53_ID != pid) {
		STK_ACC_ERR("ID match fail");
		error = STK8BAXX_ERR_IDENTIFICATION;
	}

	return error;
}

/**
 * @brief: Initialize some data in stk8baxx_data.
 *
 * @param[in/out] stk: struct stk8baxx_data *
 */
static void stk_data_initialize(struct stk8baxx_data *stk)
{
	atomic_set(&stk->enabled, 0);
	memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));
	atomic_set(&stk->selftest, STK_SELFTEST_RESULT_NA);
	atomic_set(&stk->recv, 0);
	stk->power_mode = STK8BAXX_PWMD_NORMAL;
	stk->temp_enable = false;
#ifdef STK_FIR
	memset(&stk->fir, 0, sizeof(struct data_fir));
	atomic_set(&stk->fir_len, stk->hw.firlen);
	STK_ACC_LOG("fir_len=%d", stk->hw.firlen);
#endif /* STK_FIR */
}

/**
 * @brief: SW reset for stk8baxx
 *
 * @param[in/out] stk: struct stk8baxx_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_sw_reset(struct stk8baxx_data *stk)
{
	int error = 0;
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_SWRST, STK8BAXX_SWRST_VAL);

	if (error)
		return error;

	usleep_range(1000, 2000);
	stk->power_mode = STK8BAXX_PWMD_NORMAL;

	return 0;
}

static int stk_reg_powmode_update(struct stk8baxx_data *stk, u8 pwd_md)
{
	int error = 0;
	u8 reg = STK8BAXX_REG_POWMODE;

	error = stk8baxx_reg_update_bit(stk, reg, STK8BAXX_PWMD_SLP_MASK, pwd_md);
	if (error) {
		STK_ACC_ERR("%s write reg fail:%d", __func__, error);
		error = STK8BAXX_ERR_I2C;
		goto err;
	}

	error = STK8BAXX_SUCCESS;

err:
	return error;
}

/**
 * @brief: Change power mode
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] pwd_md: power mode for STK8BAXX_REG_POWMODE
 *              STK8BAXX_PWMD_SUSPEND
 *              STK8BAXX_PWMD_LOWPOWER
 *              STK8BAXX_PWMD_NORMAL
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_change_power_mode(struct stk8baxx_data *stk, u8 pwd_md)
{
	int error = 0;

	if (pwd_md != stk->power_mode) {
		error = stk_reg_powmode_update(stk, pwd_md);
		if (error) {
			STK_ACC_ERR("%s update reg fail:%d", __func__, error);
			return STK8BAXX_ERR_I2C;
		}
		stk->power_mode = pwd_md;
	} else {
		STK_ACC_DBG("Same as original power mode: 0x%X", stk->power_mode);
	}

	return STK8BAXX_SUCCESS;
}

/**
 * @brief: Get sensitivity. Set result to stk8baxx_data.sensitivity.
 *          sensitivity = number bit per G (LSB/g)
 *          Example: RANGESEL=8g, 12 bits for STK832x full resolution
 *          Ans: number bit per G = 2^12 / (8x2) = 256 (LSB/g)
 *
 * @param[in/out] stk: struct stk8baxx_data *
 */
static void stk_get_sensitivity(struct stk8baxx_data *stk)
{
	u8 val = 0;
	stk->sensitivity = 0;

	if (0 == stk8baxx_reg_read(stk, STK8BAXX_REG_RANGESEL, 0, &val)) {
		val &= STK8BAXX_RANGESEL_BW_MASK;
		switch (val) {
		case STK8BAXX_RANGESEL_2G:
			stk->sensitivity = 1024;
			break;

		case STK8BAXX_RANGESEL_4G:
			stk->sensitivity = 512;
			break;

		case STK8BAXX_RANGESEL_8G:
			stk->sensitivity = 256;
			break;

		default:
			break;
		}
		if (STK8BA50_R_ID == stk->pid) {
			stk->sensitivity /= 4;
		}
	}
}

/**
 * @brief: Set range
 *          1. Setting STK8BAXX_REG_RANGESEL
 *          2. Calculate sensitivity and store to stk8baxx_data.sensitivity
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] range: range for STK8BAXX_REG_RANGESEL
 *              STK8BAXX_RANGESEL_2G
 *              STK8BAXX_RANGESEL_4G
 *              STK8BAXX_RANGESEL_8G
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_range_selection(struct stk8baxx_data *stk, u8 range)
{
	int result = 0;

	result = stk8baxx_reg_write(stk, STK8BAXX_REG_RANGESEL, range);
	if (result)
		return result;

	stk_get_sensitivity(stk);
	return 0;
}

/**
 * stk_set_enable
 * @brief: Turn ON/OFF the power state of stk8baxx.
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] en: turn ON/OFF
 *              0 for suspend mode;
 *              1 for normal mode.
 */
static int stk_set_enable(struct stk8baxx_data *stk, bool en)
{
	if (en == sensor_power) {
		STK_ACC_DBG("Sensor power status is newest!");
		return STK8BAXX_SUCCESS;
	}

	if (en) {
		/* ID46: Low-power -> Suspend -> Normal */
		if (stk_change_power_mode(stk, STK8BAXX_PWMD_SUSPEND))
			return STK8BAXX_ERR_I2C;

		if (stk_change_power_mode(stk, STK8BAXX_PWMD_NORMAL))
			return STK8BAXX_ERR_I2C;
	} else {
		if (stk_change_power_mode(stk, STK8BAXX_PWMD_SUSPEND))
			return STK8BAXX_ERR_I2C;
	}
	sensor_power = en;
	atomic_set(&stk->enabled, en);
	STK_ACC_LOG("Sensor power update:%d", en);
	return STK8BAXX_SUCCESS;
}

/**
 * @brief: Get delay
 *
 * @param[in/out] stk: struct stk8baxx_data *
 *
 * @return: delay in usec
 *          Please refer STK8BAXX_SAMPLE_TIME[]
 */
static int stk_get_delay(struct stk8baxx_data *stk)
{
	u8 data = 0;
	int delay_us = 0;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_BWSEL, 0, &data)) {
		STK_ACC_ERR("failed to read delay");
	} else if ((STK8BAXX_SPTIME_BASE > data) || (STK8BAXX_SPTIME_BOUND < data)) {
		STK_ACC_ERR("BW out of range, 0x%X", data);
	} else {
		delay_us = STK8BAXX_SAMPLE_TIME[data - STK8BAXX_SPTIME_BASE];
	}

	return delay_us;
}

/**
 * @brief: Set delay
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_set_delay(struct stk8baxx_data *stk, int delay_us)
{
	int error = 0;
	bool enable = false;
	unsigned char sr_no;

	for (sr_no = 0; sr_no <= STK8BAXX_SPTIME_BOUND - STK8BAXX_SPTIME_BASE; sr_no++) {
		if (delay_us >= STK8BAXX_SAMPLE_TIME[sr_no])
			break;

		if (sr_no == STK8BAXX_SPTIME_BOUND - STK8BAXX_SPTIME_BASE + 1) {
			sr_no--;
		}

		delay_us = STK8BAXX_SAMPLE_TIME[sr_no];
		sr_no += STK8BAXX_SPTIME_BASE;

		if (atomic_read(&stk->enabled)) {
			stk_set_enable(stk, 0);
			enable = true;
		}

		error = stk8baxx_reg_write(stk, STK8BAXX_REG_BWSEL, sr_no);
		if (error)
			STK_ACC_ERR("failed to change ODR");
		if (enable) {
			stk_set_enable(stk, 1);
		}

		msleep(delay_us / 1000);
	}

	return error;
}

#ifdef STK_FIR
/**
 * @brief: low-pass filter operation
 *
 * @param[in/out] stk: struct stk8baxx_data *
 */
static void stk_low_pass_fir(struct stk8baxx_data *stk)
{
	int firlength = atomic_read(&stk->fir_len);

	if (0 == firlength) {
		/* stk8baxx_data.fir_len == 0: turn OFF FIR operation */
		return;
	}

	if (firlength > stk->fir.count) {
		stk->fir.xyz[stk->fir.idx][0] = stk->xyz[0];
		stk->fir.xyz[stk->fir.idx][1] = stk->xyz[1];
		stk->fir.xyz[stk->fir.idx][2] = stk->xyz[2];
		stk->fir.sum[0] += stk->xyz[0];
		stk->fir.sum[1] += stk->xyz[1];
		stk->fir.sum[2] += stk->xyz[2];
		stk->fir.count++;
		stk->fir.idx++;
	} else {
		if (firlength <= stk->fir.idx)
		stk->fir.idx = 0;

		stk->fir.sum[0] -= stk->fir.xyz[stk->fir.idx][0];
		stk->fir.sum[1] -= stk->fir.xyz[stk->fir.idx][1];
		stk->fir.sum[2] -= stk->fir.xyz[stk->fir.idx][2];
		stk->fir.xyz[stk->fir.idx][0] = stk->xyz[0];
		stk->fir.xyz[stk->fir.idx][1] = stk->xyz[1];
		stk->fir.xyz[stk->fir.idx][2] = stk->xyz[2];
		stk->fir.sum[0] += stk->xyz[0];
		stk->fir.sum[1] += stk->xyz[1];
		stk->fir.sum[2] += stk->xyz[2];

		stk->fir.idx++;
		stk->xyz[0] = stk->fir.sum[0] / firlength;
		stk->xyz[1] = stk->fir.sum[1] / firlength;
		stk->xyz[2] = stk->fir.sum[2] / firlength;
	}
}
#endif /* STK_FIR */

/**
 * @brief: read accel raw data from register.
 *
 * @param[in/out] stk: struct stk8baxx_data *
 */
static int stk_read_accel_rawdata(struct stk8baxx_data *stk)
{
	u8 dataL = 0;
	u8 dataH = 0;

	if (stk == NULL || atomic_read(&stk->suspend)) {
		stk->xyz[STK_AXIS_X] = 0;
		stk->xyz[STK_AXIS_Y] = 0;
		stk->xyz[STK_AXIS_Z] = 0;
		STK_ACC_ERR("The client == NULL or system is in suspend!");
		return STK8BAXX_ERR_GETGSENSORDATA;
	}

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_XOUT1, 0, &dataL))
		return STK8BAXX_ERR_I2C;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_XOUT2, 0, &dataH))
		return STK8BAXX_ERR_I2C;

	stk->xyz[0] = dataH << 8 | dataL;

	if (STK8BA53_ID == stk->pid)
		stk->xyz[0] >>= 4;
	else
		stk->xyz[0] >>= 6;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_YOUT1, 0, &dataL))
		return STK8BAXX_ERR_I2C;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_YOUT2, 0, &dataH))
		return STK8BAXX_ERR_I2C;

	stk->xyz[1] = dataH << 8 | dataL;

	if (STK8BA53_ID == stk->pid)
		stk->xyz[1] >>= 4;
	else
		stk->xyz[1] >>= 6;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_ZOUT1, 0, &dataL))
		return STK8BAXX_ERR_I2C;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_ZOUT2, 0, &dataH))
		return STK8BAXX_ERR_I2C;

	stk->xyz[2] = dataH << 8 | dataL;

	if (STK8BA53_ID == stk->pid)
		stk->xyz[2] >>= 4;
	else
		stk->xyz[2] >>= 6;

	return STK8BAXX_SUCCESS;
}

/**
 * @brief: read accel data from register.
 *          Store result to stk8baxx_data.xyz[].
 *
 * @param[in/out] stk: struct stk8baxx_data *
 */
static int stk_read_accel_data(struct stk8baxx_data *stk)
{
	int res = 0;

	res = stk_read_accel_rawdata(stk);
	if (res) {
		STK_ACC_ERR("I2C error: ret value=%d", res);
		return STK8BAXX_ERR_STATUS;
	}
#ifdef STK_FIR
	stk_low_pass_fir(stk);
#endif /* STK_FIR */
	return STK8BAXX_SUCCESS;
}

/**
 * @brief: Selftest for XYZ offset and noise.
 *
 * @param[in/out] stk: struct stk8baxx_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static char stk_testOffsetNoise(struct stk8baxx_data *stk)
{
	int read_delay_ms = 8; /* 125Hz = 8ms */
	int acc_ave[3] = {0, 0, 0};
	int acc_min[3] = {INT_MAX, INT_MAX, INT_MAX};
	int acc_max[3] = {INT_MIN, INT_MIN, INT_MIN};
	int noise[3] = {0, 0, 0};
	int sn = 0, axis = 0;
	int thresholdOffset, thresholdNoise;
	u8 localResult = 0;
	int res = 0;

	if (stk_sw_reset(stk))
		return -1;

	atomic_set(&stk->enabled, 1);

	if (stk8baxx_reg_write(stk, STK8BAXX_REG_BWSEL, 0x0B)) /* ODR = 125Hz */
		return -1;

	if (stk_range_selection(stk, STK8BAXX_RANGESEL_2G))
		return -1;

	thresholdOffset = stk_selftest_offset_factor(stk->sensitivity);
	thresholdNoise = stk_selftest_noise_factor(stk->sensitivity);

	for (sn = 0; sn < STK_SELFTEST_SAMPLE_NUM; sn++) {
		msleep(read_delay_ms);
		res = stk_read_accel_rawdata(stk);
		if (res) {
			STK_ACC_ERR("I2C error: ret value=%d", res);
			return STK8BAXX_ERR_STATUS;
		}
		STK_ACC_LOG("acc = %d, %d, %d", stk->xyz[0], stk->xyz[1], stk->xyz[2]);

		for (axis = 0; axis < 3; axis++) {
			acc_ave[axis] += stk->xyz[axis];

			if (stk->xyz[axis] > acc_max[axis])
				acc_max[axis] = stk->xyz[axis];

			if (stk->xyz[axis] < acc_min[axis])
				acc_min[axis] = stk->xyz[axis];
		}
	}

	for (axis = 0; axis < 3; axis++) {
		acc_ave[axis] /= STK_SELFTEST_SAMPLE_NUM;
		noise[axis] = acc_max[axis] - acc_min[axis];
	}

	STK_ACC_LOG("acc_ave=%d, %d, %d, noise=%d, %d, %d",
	acc_ave[0], acc_ave[1], acc_ave[2], noise[0], noise[1], noise[2]);
	STK_ACC_LOG("offset threshold=%d, noise threshold=%d", thresholdOffset, thresholdNoise);

	if (0 < acc_ave[2])
		acc_ave[2] -= stk->sensitivity;
	else
		acc_ave[2] += stk->sensitivity;

	if (0 == acc_ave[0] && 0 == acc_ave[1] && 0 == acc_ave[2])
		localResult |= STK_SELFTEST_RESULT_NO_OUTPUT;

	if (thresholdOffset <= abs(acc_ave[0])
		|| 0 == noise[0] || thresholdNoise <= noise[0])
		localResult |= STK_SELFTEST_RESULT_FAIL_X;

	if (thresholdOffset <= abs(acc_ave[1])
		|| 0 == noise[1] || thresholdNoise <= noise[1])
		localResult |= STK_SELFTEST_RESULT_FAIL_Y;

	if (thresholdOffset <= abs(acc_ave[2])
		|| 0 == noise[2] || thresholdNoise <= noise[2])
		localResult |= STK_SELFTEST_RESULT_FAIL_Z;

	if (0 == localResult)
		atomic_set(&stk->selftest, STK_SELFTEST_RESULT_NO_ERROR);
	else
		atomic_set(&stk->selftest, localResult);
	return 0;
}

/**
 * @brief: SW selftest function.
 *
 * @param[in/out] stk: struct stk8baxx_data *
 */
static void stk_selftest(struct stk8baxx_data *stk)
{
	int i = 0;
	u8 data = 0;

	STK_ACC_FUN();

	atomic_set(&stk->selftest, STK_SELFTEST_RESULT_RUNNING);

	/* Check PID */
	if (stk_check_pid(stk)) {
		atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
		return;
	}

	/* Touch all register */
	for (i = 0; i <= 0x3F; i++) {
		if (stk8baxx_reg_read(stk, i, 0, &data)) {
			atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
			return;
		}

		STK_ACC_LOG("[0x%2X]=0x%2X", i, data);
	}

	if (stk_testOffsetNoise(stk)) {
		atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
	}
	stk_reg_init(stk);
}

/**
 * @brief: Read all register (0x0 ~ 0x3F)
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[out] show_buffer: record all register value
 *
 * @return: buffer length or fail
 *          positive value: return buffer length
 *          -1: Fail
 */
static int stk_show_all_reg(struct stk8baxx_data *stk, char *show_buffer)
{
	int reg;
	int len = 0;
	u8 data = 0;

	if (NULL == show_buffer)
		return -1;

	for (reg = 0; reg <= 0x3F; reg++) {
		if (stk8baxx_reg_read(stk, reg, 0, &data)) {
			len = -1;
			goto exit;
		}

		if (0 >= (PAGE_SIZE - len)) {
			STK_ACC_ERR("print string out of PAGE_SIZE");
			goto exit;
		}

		len += scnprintf(show_buffer + len, PAGE_SIZE - len,
				"[0x%2X]=0x%2X, ", reg, data);
	}

	len += scnprintf(show_buffer + len, PAGE_SIZE - len, "\n");
exit:
	return len;
}

/**
 * @brief: Get offset
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[out] offset: offset value read from register
 *                  STK8BAXX_REG_OFSTX,  STK8BAXX_REG_OFSTY, STK8BAXX_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_get_offset(struct stk8baxx_data *stk, u8 offset[3])
{
	int error = 0;
	bool enable = false;

	if (!atomic_read(&stk->enabled))
		stk_set_enable(stk, 1);
	else
		enable = true;

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_OFSTX, 0, &offset[0])) {
		error = -1;
		goto exit;
	}

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_OFSTY, 0, &offset[1])) {
		error = -1;
		goto exit;
	}

	if (stk8baxx_reg_read(stk, STK8BAXX_REG_OFSTZ, 0, &offset[2])) {
		error = -1;
		goto exit;
	}

exit:
	if (!enable)
		stk_set_enable(stk, 0);

	return error;
}

/**
 * @brief: Set offset
 *
 * @param[in/out] stk: struct stk8baxx_data *
 * @param[in] offset: offset value write to register
 *                  STK8BAXX_REG_OFSTX,  STK8BAXX_REG_OFSTY, STK8BAXX_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_set_offset(struct stk8baxx_data *stk, u8 offset[3])
{
	int error = 0;
	bool enable = false;

	if (!atomic_read(&stk->enabled))
		stk_set_enable(stk, 1);
	else
		enable = true;

	if (stk8baxx_reg_write(stk, STK8BAXX_REG_OFSTX, offset[0])) {
		error = -1;
		goto exit;
	}

	if (stk8baxx_reg_write(stk, STK8BAXX_REG_OFSTY, offset[1])) {
		error = -1;
		goto exit;
	}

	if (stk8baxx_reg_write(stk, STK8BAXX_REG_OFSTZ, offset[2])) {
		error = -1;
		goto exit;
	}

exit:
	if (!enable)
		stk_set_enable(stk, 0);

	return error;
}

/**
 * @brief: Get power status
 *          Send 0 or 1 to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t enable_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	char en;

	if (NULL == stk)
		return 0;

	en = atomic_read(&stk->enabled);
	return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

/**
 * @brief: Set power status
 *          Get 0 or 1 from userspace, then set stk8baxx power status.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t enable_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	unsigned int data;
	int error;
	error = kstrtouint(buf, 10, &data);

	if (NULL == stk)
		return 0;

	if (error) {
		STK_ACC_ERR("kstrtoul failed, error=%d", error);
		return error;
	}

	if ((1 == data) || (0 == data))
		stk_set_enable(stk, data);
	else
		STK_ACC_ERR("invalid argument, en=%d", data);

	return count;
}

static ssize_t trace_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct stk8baxx_data *stk = stk_data;

	if (stk == NULL) {
		STK_ACC_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk->trace));
	return res;
}

static ssize_t trace_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	int trace;

	if (stk == NULL) {
		STK_ACC_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	if (sscanf(buf, "0x%11x", &trace) == 1)
		atomic_set(&stk->trace, trace);
	else
		STK_ACC_ERR("invalid content: '%s'\n", buf);

	return count;
}

/**
 * @brief: Get accel data
 *          Send accel data to userspce.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t sensordata_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	int data_x = 0, data_y = 0, data_z = 0;
	bool enable = true;

	if (NULL == stk)
		return 0;

	if (!atomic_read(&stk->enabled)) {
		stk_set_enable(stk, 1);
		enable = false;
	}

	stk_readsensordata(&data_x, &data_y, &data_z);
	if (!enable)
		stk_set_enable(stk, 0);

	return scnprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
					data_x, data_y, data_z);
}

/**
 * @brief: Get delay value in usec
 *          Send delay in usec to userspce.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t delay_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;

	if (NULL == stk)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)stk_get_delay(stk) * 1000);
}

/**
 * @brief: Set delay value in usec
 *          Get delay value in usec from userspace, then write to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t delay_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	long long data;
	int error;

	if (NULL == stk)
		return 0;

	error = kstrtoll(buf, 10, &data);

	if (error) {
		STK_ACC_ERR("kstrtoul failed, error=%d", error);
		return error;
	}

	stk_set_delay(stk, (int)(data / 1000));
	return count;
}

/**
 * @brief: Get offset value
 *          Send X/Y/Z offset value to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t offset_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	u8 offset[3] = {0, 0, 0};

	if (NULL == stk)
		return 0;

	stk_get_offset(stk, offset);
	return scnprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X\n", offset[0], offset[1], offset[2]);
}

/**
 * @brief: Set offset value
 *          Get X/Y/Z offset value from userspace, then write to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t offset_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	char *token[10];
	u8 r_offset[3];
	int error, data, i;

	if (NULL == stk)
		return 0;

	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	error = kstrtoint(token[0], 16, &data);
	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	r_offset[0] = (u8)data;
	error = kstrtoint(token[1], 16, &data);

	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	r_offset[1] = (u8)data;
	error = kstrtoint(token[2], 16, &data);

	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	r_offset[2] = (u8)data;
	STK_ACC_LOG("offset=0x%X, 0x%X, 0x%X", r_offset[0], r_offset[1], r_offset[2]);
	stk_set_offset(stk, r_offset);
	return count;
}

/**
 * @brief: Register writting
 *          Get address and content from userspace, then write to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t send_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	char *token[10];
	int addr, cmd, error, i;
	bool enable = false;

	if (NULL == stk)
		return 0;

	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");

	error = kstrtoint(token[0], 16, &addr);
	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	error = kstrtoint(token[1], 16, &cmd);
	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	STK_ACC_LOG("write reg[0x%X]=0x%X", addr, cmd);
	if (!atomic_read(&stk->enabled))
		stk_set_enable(stk, 1);
	else
		enable = true;

	if (stk8baxx_reg_write(stk, (u8)addr, (u8)cmd)) {
		error = -1;
		goto exit;
	}

exit:
	if (!enable)
		stk_set_enable(stk, 0);

	if (error)
		return -1;

	return count;
}

/**
 * @brief: Read stk8baxx_data.recv(from stk_recv_store), then send to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t recv_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;

	if (NULL == stk)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "0x%X\n", atomic_read(&stk->recv));
}

/**
 * @brief: Get the read address from userspace, then store the result to
 *          stk8baxx_data.recv.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t recv_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	int addr, error;
	u8 data = 0;
	bool enable = false;

	if (NULL == stk)
		return 0;

	error = kstrtoint(buf, 16, &addr);
	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	if (!atomic_read(&stk->enabled))
		stk_set_enable(stk, 1);
	else
		enable = true;

	if (stk8baxx_reg_read(stk, (u8)addr, 0, &data)) {
		error = -1;
		goto exit;
	}

	atomic_set(&stk->recv, data);
	STK_ACC_LOG("read reg[0x%X]=0x%X", addr, data);
exit:
	if (!enable)
		stk_set_enable(stk, 0);

	if (error)
		return -1;

	return count;
}

/**
 * @brief: Read all register value, then send result to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t allreg_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	int result;

	if (NULL == stk)
		return 0;

	result = stk_show_all_reg(stk, buf);
	return (ssize_t)result;
}

/**
 * @brief: Check PID, then send chip number to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t chipinfo_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;

	if (NULL == stk)
		return 0;

	if (STK8BA53_ID == stk->pid)
		return scnprintf(buf, PAGE_SIZE, "stk8ba53\n");
	else if (STK8BA50_R_ID == stk->pid)
		return scnprintf(buf, PAGE_SIZE, "stk8ba50-r\n");

	return scnprintf(buf, PAGE_SIZE, "unknown\n");
}

/**
 * TODO
 */
static ssize_t direction_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;

	if (NULL == stk)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", stk->hw.direction);
}

/**
 * TODO
 */
static ssize_t direction_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	unsigned long position = 0;
	int error = 0;

	if (NULL == stk)
		return 0;

	position = simple_strtoul(buf, NULL, 10);

	if (0 <= position && 7 >= position) {
		stk->hw.direction = position;
	}

	error = hwmsen_get_convert(stk->hw.direction, &stk->cvt);
	if (error) {
		STK_ACC_ERR("invalid direction: %d", stk->hw.direction);
		return -EINVAL;
	}

	return count;
}

/**
 * TODO
 */
static ssize_t selftest_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	u8 result;

	if (NULL == stk)
		return 0;

	result = atomic_read(&stk->selftest);

	if (STK_SELFTEST_RESULT_NA == result)
		return scnprintf(buf, PAGE_SIZE, "No result\n");
	if (STK_SELFTEST_RESULT_RUNNING == result)
		return scnprintf(buf, PAGE_SIZE, "selftest is running\n");
	else if (STK_SELFTEST_RESULT_NO_ERROR == result)
		return scnprintf(buf, PAGE_SIZE, "No error\n");
	else
		return scnprintf(buf, PAGE_SIZE, "Error code:0x%2X\n", result);
}

/**
 * TODO
 */
static ssize_t selftest_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;

	if (NULL == stk)
		return 0;

	stk_selftest(stk);
	return count;
}

#ifdef STK_FIR
/**
 * @brief: Get FIR parameter, then send to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t firlen_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	int len;

	if (NULL == stk)
		return 0;

	len = atomic_read(&stk->fir_len);

	if (len) {
		STK_ACC_LOG("FIR count=%2d, idx=%2d", stk->fir.count, stk->fir.idx);
		STK_ACC_LOG("sum = [\t%d \t%d \t%d]", stk->fir.sum[0], stk->fir.sum[1], stk->fir.sum[2]);
		STK_ACC_LOG("avg = [\t%d \t%d \t%d]", stk->fir.sum[0] / len, stk->fir.sum[1] / len, stk->fir.sum[2] / len);
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

/**
 * @brief: Get FIR length from userspace, then write to stk8baxx_data.fir_len.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t firlen_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_data *stk = stk_data;
	int firlen, error;

	if (NULL == stk)
		return 0;

	error = kstrtoint(buf, 10, &firlen);

	if (error) {
		STK_ACC_ERR("kstrtoint failed, error=%d", error);
		return error;
	}

	if (STK_FIR_LEN_MAX < firlen)
		STK_ACC_ERR("maximum FIR length is %d", STK_FIR_LEN_MAX);
	else {
		memset(&stk->fir, 0, sizeof(struct data_fir));
		atomic_set(&stk->fir_len, firlen);
	}

	return count;
}
#endif /* STK_FIR */

/*----------------------------------------------------------------------------*/
static ssize_t accelsetselftest_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	int avg[3] = {0}, out_nost[3] = {0};
	int err = -1, num = 0, count = 5;
	int data_x = 0, data_y = 0, data_z = 0;

	if (NULL == stk)
		return 0;

	accel_self_test[0] = accel_self_test[1] = accel_self_test[2] = 0;

	err = stk_set_enable(stk, true);
	if (err) {
		STK_ACC_ERR("enable gsensor fail: %d\n", err);
		goto stk_accel_self_test_exit;
	}

	msleep(100);

	while (num < count) {
		msleep(20);

		/* read gsensor data */
		err = stk_readsensordata(&data_x, &data_y, &data_z);
		if (err) {
			STK_ACC_ERR("read data fail: %d", err);
			goto stk_accel_self_test_exit;
		}

		avg[STK_AXIS_X] = data_x + avg[STK_AXIS_X];
		avg[STK_AXIS_Y] = data_y + avg[STK_AXIS_Y];
		avg[STK_AXIS_Z] = data_z + avg[STK_AXIS_Z];

		num++;
	}

	out_nost[0] = avg[STK_AXIS_X] / count;
	out_nost[1] = avg[STK_AXIS_Y] / count;
	out_nost[2] = avg[STK_AXIS_Z] / count;

	accel_self_test[0] = abs(out_nost[0]);
	accel_self_test[1] = abs(out_nost[1]);
	accel_self_test[2] = abs(out_nost[2]);

	/* disable sensor */
	err = stk_set_enable(stk, false);
	if (err < 0)
		goto stk_accel_self_test_exit;

	return scnprintf(buf, PAGE_SIZE,
			"[G Sensor] set_accel_self_test PASS\n");

stk_accel_self_test_exit:

	stk_set_enable(stk, false);

	return scnprintf(buf, PAGE_SIZE,
			"[G Sensor] exit - Fail , err=%d\n", err);

}

static ssize_t accelgetselftest_show(struct device_driver *ddri, char *buf)
{
	if (accel_self_test[0] < ACCEL_SELF_TEST_MIN_VAL ||
		accel_self_test[0] > ACCEL_SELF_TEST_MAX_VAL)
		return sprintf(buf, "X=%d , out of range\nFail\n",
				accel_self_test[0]);

	if (accel_self_test[1] < ACCEL_SELF_TEST_MIN_VAL ||
		accel_self_test[1] > ACCEL_SELF_TEST_MAX_VAL)
		return sprintf(buf, "Y=%d , out of range\nFail\n",
				accel_self_test[1]);

	if (accel_self_test[2] < ACCEL_SELF_TEST_MIN_VAL ||
		accel_self_test[2] > ACCEL_SELF_TEST_MAX_VAL)
		return sprintf(buf, "Z=%d , out of range\nFail\n",
				accel_self_test[2]);
	else
		return sprintf(buf, "%d , %d , %d\nPass\n", accel_self_test[0],
				accel_self_test[1], accel_self_test[2]);
}

static int acc_store_offset_in_file(const char *filename, s16 *offset)
{
	struct file *cali_file;
	char w_buf[STK_DATA_BUF_NUM*sizeof(s16)*2+1] = {0};
	char r_buf[STK_DATA_BUF_NUM*sizeof(s16)*2+1] = {0};
	int i;
	char *dest = w_buf;
	mm_segment_t fs;

	cali_file = filp_open(filename, O_CREAT | O_RDWR, 0777);
	if (IS_ERR(cali_file)) {
		STK_ACC_ERR("open error! exit!\n");
		return -1;
	}
	fs = get_fs();
	set_fs(get_ds());
	for (i = 0; i < STK_DATA_BUF_NUM; i++) {
		sprintf(dest, "%02X", offset[i]&0x00FF);
		dest += 2;
		sprintf(dest, "%02X", (offset[i] >> 8)&0x00FF);
		dest += 2;
	};
	STK_ACC_LOG("w_buf: %s\n", w_buf);
	vfs_write(cali_file, (void *)w_buf, STK_DATA_BUF_NUM*sizeof(s16)*2, &cali_file->f_pos);
	cali_file->f_pos = 0x00;
	vfs_read(cali_file, (void *)r_buf, STK_DATA_BUF_NUM*sizeof(s16)*2, &cali_file->f_pos);
	for (i = 0; i < STK_DATA_BUF_NUM*sizeof(s16)*2; i++) {
		if (r_buf[i] != w_buf[i]) {
			set_fs(fs);
			filp_close(cali_file, NULL);
			STK_ACC_ERR("read back error! exit!\n");
			return -1;
		}
	}
	set_fs(fs);

	filp_close(cali_file, NULL);
	STK_ACC_LOG("store_offset_in_file ok exit\n");
	return 0;
}

static ssize_t accelsetcali_show(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_data *stk = stk_data;
	int avg[3] = {0};
	int cali[3] = {0};
	int golden_x = 0;
	int golden_y = 0;
	int golden_z = -9800;
	int cali_last[3] = {0};
	int err = -1, num = 0, times = 20;
	int data_x = 0, data_y = 0, data_z = 0;

	if (NULL == stk_data) {
		STK_ACC_ERR("i2c client is null!!\n");
		return 0;
	}

	stk->cali_sw[STK_AXIS_X] = 0;
	stk->cali_sw[STK_AXIS_Y] = 0;
	stk->cali_sw[STK_AXIS_Z] = 0;

	while (num < times) {
		msleep(20);

		/* read gsensor data */
		err = stk_readsensordata(&data_x, &data_y, &data_z);
		if (err) {
			STK_ACC_ERR("read data fail: %d\n", err);
			return 0;
		}
		if (atomic_read(&stk->trace) & STK_TRC_INFO)
			STK_ACC_LOG("accel cali simple data x:%d, y:%d, z:%d", data_x, data_y, data_z);

		if (data_z > 8500)
			golden_z = 9800;
		else if (data_z < -8500)
			golden_z = -9800;
		else
			return 0;

		avg[STK_AXIS_X] = data_x + avg[STK_AXIS_X];
		avg[STK_AXIS_Y] = data_y + avg[STK_AXIS_Y];
		avg[STK_AXIS_Z] = data_z + avg[STK_AXIS_Z];

		num++;

	}

	avg[STK_AXIS_X] /= times;
	avg[STK_AXIS_Y] /= times;
	avg[STK_AXIS_Z] /= times;

	if (atomic_read(&stk->trace) & STK_TRC_INFO)
		STK_ACC_LOG("accel cali final data x:%d, y:%d, z:%d", avg[STK_AXIS_X], avg[STK_AXIS_Y], avg[STK_AXIS_Z]);

	cali[STK_AXIS_X] = golden_x - avg[STK_AXIS_X];
	cali[STK_AXIS_Y] = golden_y - avg[STK_AXIS_Y];
	cali[STK_AXIS_Z] = golden_z - avg[STK_AXIS_Z];


	if ((abs(cali[STK_AXIS_X]) >
		abs(accel_cali_tolerance * golden_z / 100))
		|| (abs(cali[STK_AXIS_Y]) >
		abs(accel_cali_tolerance * golden_z / 100))
		|| (abs(cali[STK_AXIS_Z]) >
		abs(accel_cali_tolerance * golden_z / 100))) {

		STK_ACC_ERR("X/Y/Z out of range  tolerance:[%d] avg_x:[%d] avg_y:[%d] avg_z:[%d]\n",
				accel_cali_tolerance,
				avg[STK_AXIS_X],
				avg[STK_AXIS_Y],
				avg[STK_AXIS_Z]);

		return scnprintf(buf, PAGE_SIZE,
				"Please place the Pad to a horizontal level.\ntolerance:[%d] avg_x:[%d] avg_y:[%d] avg_z:[%d]\n",
				accel_cali_tolerance,
				avg[STK_AXIS_X],
				avg[STK_AXIS_Y],
				avg[STK_AXIS_Z]);
	}

	cali_last[0] = cali[STK_AXIS_X];
	cali_last[1] = cali[STK_AXIS_Y];
	cali_last[2] = cali[STK_AXIS_Z];

	stk_writeCalibration(cali_last);

	cali[STK_AXIS_X] = stk->cali_sw[STK_AXIS_X];
	cali[STK_AXIS_Y] = stk->cali_sw[STK_AXIS_Y];
	cali[STK_AXIS_Z] = stk->cali_sw[STK_AXIS_Z];

	accel_xyz_offset[0] = (s16)cali[STK_AXIS_X];
	accel_xyz_offset[1] = (s16)cali[STK_AXIS_Y];
	accel_xyz_offset[2] = (s16)cali[STK_AXIS_Z];

	if (acc_store_offset_in_file(STK_ACC_CALI_FILE, accel_xyz_offset)) {
		return scnprintf(buf, PAGE_SIZE,
				"[G Sensor] set_accel_cali ERROR %d, %d, %d\n",
				accel_xyz_offset[0],
				accel_xyz_offset[1],
				accel_xyz_offset[2]);
	}

	return scnprintf(buf, PAGE_SIZE,
			"[G Sensor] set_accel_cali PASS  %d, %d, %d\n",
			accel_xyz_offset[0],
			accel_xyz_offset[1],
			accel_xyz_offset[2]);
}

static ssize_t accelgetcali_show(struct device_driver *ddri, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"x=%d , y=%d , z=%d\nx=0x%04x , y=0x%04x , z=0x%04x\nPass\n",
			accel_xyz_offset[0], accel_xyz_offset[1],
			accel_xyz_offset[2], accel_xyz_offset[0],
			accel_xyz_offset[1], accel_xyz_offset[2]);
}

static ssize_t power_mode_show(struct device_driver *ddri, char *buf)
{
	STK_ACC_LOG("[%s] STK8baxx_sensor_power: %d", __func__, sensor_power);
	return scnprintf(buf, PAGE_SIZE, "%d\n", sensor_power);
}

static ssize_t power_mode_store(struct device_driver *ddri, const char *buf, size_t tCount)
{
	struct stk8baxx_data *stk = stk_data;
	bool power_enable = false;
	int power_mode = 0;
	int ret = 0;

	if (NULL == stk)
		return 0;

	ret = kstrtoint(buf, 10, &power_mode);

	power_enable = (power_mode ? true:false);

	if (0 == ret)
		ret = stk_set_enable(stk, power_enable);

	if (ret) {
		STK_ACC_ERR("set power %s failed %d\n", (power_enable?"on":"off"), ret);
		return 0;
	} else
		STK_ACC_ERR("set power %s ok\n", (sensor_power?"on":"off"));

	return tCount;
}

static ssize_t cali_tolerance_show(struct device_driver *ddri, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "accel_cali_tolerance=%d\n", accel_cali_tolerance);
}

static ssize_t cali_tolerance_store(struct device_driver *ddri, const char *buf, size_t tCount)
{
	int temp_cali_tolerance = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &temp_cali_tolerance);

	if (0 == ret) {
		if (temp_cali_tolerance > 100)
			temp_cali_tolerance = 100;
		if (temp_cali_tolerance <= 0)
			temp_cali_tolerance = 1;

		accel_cali_tolerance = temp_cali_tolerance;
	}

	if (ret) {
		STK_ACC_ERR("set accel_cali_tolerance failed %d\n", ret);
		return 0;
	} else
		STK_ACC_ERR("set accel_cali_tolerance %d ok\n", accel_cali_tolerance);

	return tCount;
}

static void get_accel_idme_cali(void)
{

	s16 idmedata[CALI_SIZE] = {0};

	idme_get_sensorcal(idmedata, CALI_SIZE);
	accel_xyz_offset[0] = idmedata[0];
	accel_xyz_offset[1] = idmedata[1];
	accel_xyz_offset[2] = idmedata[2];

	STK_ACC_LOG("accel_xyz_offset =%d, %d, %d\n", accel_xyz_offset[0],
			accel_xyz_offset[1], accel_xyz_offset[2]);
}

static ssize_t accelgetidme_show(struct device_driver *ddri, char *buf)
{
	get_accel_idme_cali();
	return scnprintf(buf, PAGE_SIZE,
			"offset_x=%d , offset_y=%d , offset_z=%d\nPass\n",
			accel_xyz_offset[0],
			accel_xyz_offset[1],
			accel_xyz_offset[2]);
}

static DRIVER_ATTR_RW(enable);
static DRIVER_ATTR_RW(trace);
static DRIVER_ATTR_RO(sensordata);
static DRIVER_ATTR_RW(delay);
static DRIVER_ATTR_RW(offset);
static DRIVER_ATTR_WO(send);
static DRIVER_ATTR_RW(recv);
static DRIVER_ATTR_RO(allreg);
static DRIVER_ATTR_RO(chipinfo);
static DRIVER_ATTR_RW(direction);
static DRIVER_ATTR_RW(selftest);
#ifdef STK_FIR
static DRIVER_ATTR_RW(firlen);
#endif /* STK_FIR */
/* add for diag */
static DRIVER_ATTR_RO(accelsetselftest);
static DRIVER_ATTR_RO(accelgetselftest);
static DRIVER_ATTR_RO(accelsetcali);
static DRIVER_ATTR_RO(accelgetcali);
static DRIVER_ATTR_RW(power_mode);
static DRIVER_ATTR_RW(cali_tolerance);
static DRIVER_ATTR_RO(accelgetidme);


static struct driver_attribute *stk_attr_list[] = {
	&driver_attr_enable,
	&driver_attr_trace,
	&driver_attr_sensordata,
	&driver_attr_delay,
	&driver_attr_offset,
	&driver_attr_send,
	&driver_attr_recv,
	&driver_attr_allreg,
	&driver_attr_chipinfo,
	&driver_attr_direction,
	&driver_attr_selftest,
#ifdef STK_FIR
	&driver_attr_firlen,
#endif /* STK_FIR */
	/*add for diag*/
	&driver_attr_accelsetselftest,
	&driver_attr_accelgetselftest,
	&driver_attr_accelsetcali,
	&driver_attr_accelgetcali,
	&driver_attr_power_mode,
	&driver_attr_cali_tolerance,
	&driver_attr_accelgetidme,
};

static int stk_create_attr(struct device_driver *driver)
{
	int i, error = 0;
	int num = (int)(sizeof(stk_attr_list) / sizeof(stk_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		error = driver_create_file(driver, stk_attr_list[i]);
		if (error) {
			STK_ACC_ERR("driver_create_file (%s) = %d",
			stk_attr_list[i]->attr.name, error);
			break;
		}
	}

	return error;
}

static void stk_create_attr_exit(struct device_driver *driver)
{
	int i;
	int num = (int)ARRAY_SIZE(stk_attr_list);

	if (NULL == driver)
		return;

	for (i = 0; i < num; i++)
		driver_remove_file(driver, stk_attr_list[i]);
}

/**
 * @brief: stk8baxx register initialize
 *
 * @param[in/out] stk: struct stk8baxx_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_reg_init(struct stk8baxx_data *stk)
{
	int error = 0;

	STK_ACC_FUN();
	/* SW reset */
	error = stk_sw_reset(stk);
	if (error)
		return error;

	error = stk_set_enable(stk, true);
	if (error)
		return error;
	STK_ACC_DBG("STK change mode to normal");

	atomic_set(&stk->suspend, 0);
	atomic_set(&stk->enabled, 1);
	/* INT1, push-pull, active high. */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTCFG1,
			STK8BAXX_INTCFG1_INT1_ACTIVE_H | STK8BAXX_INTCFG1_INT1_OD_PUSHPULL);
	if (error)
		return error;

	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTMAP2, 0);
	if (error)
		return error;

	/* disable new data interrupt */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTEN2, 0);
	if (error)
		return error;

	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTMAP1, 0);
	if (error)
		return error;

	/* non-latch int */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTCFG2, STK8BAXX_INTCFG2_NOLATCHED);
	if (error)
		return error;

	/* disable new data interrupt for any motion */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTEN1, 0);
	if (error)
		return error;

	/* SIGMOT2 */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_SIGMOT2, 0);
	if (error)
		return error;

	/* SLOPE DELAY */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_SLOPEDLY, 0x00);
	if (error)
		return error;

	/* SLOPE THRESHOLD */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_SLOPETHD, STK8BAXX_SLOPETHD_DEF);
	if (error)
		return error;

	/* SIGMOT1 */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_SIGMOT1, STK8BAXX_SIGMOT1_SKIP_TIME_3SEC);
	if (error)
		return error;

	/* SIGMOT3 */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_SIGMOT3, STK8BAXX_SIGMOT3_PROOF_TIME_1SEC);
	if (error)
		return error;

	/* According to STK_DEF_DYNAMIC_RANGE */
	error = stk_range_selection(stk, STK8BAXX_RANGESEL_DEF);
	if (error)
		return error;
	STK_ACC_DBG("STK setBWRate!");

	/* ODR 125Hz*/
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_BWSEL, STK8BAXX_BWSEL_INIT_ODR);
	if (error)
		return error;

	/* i2c watchdog enable */
	error = stk8baxx_reg_write(stk, STK8BAXX_REG_INTFCFG, STK8BAXX_INTFCFG_I2C_WDT_EN);
	if (error)
		return error;

	/* SUSPEND */
	error = stk_set_enable(stk, false);
	if (error)
		return error;
	STK_ACC_DBG("STK reg_init pass!");

	return 0;
}

/**
 * @brief: Open data rerport to HAL.
 *      refer: drivers/misc/mediatek/accelerometer/inc/accel.h
 */
static int gsensor_open_report_data(int open)
{
	STK_ACC_FUN();
	/* TODO. should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/**
 * @brief: Only enable not report event to HAL.
 *      refer: drivers/misc/mediatek/accelerometer/inc/accel.h
 */
static int gsensor_enable_nodata(int en)
{
	struct stk8baxx_data *stk = stk_data;
	int res = 0;

	if (((en == 0) && (sensor_power == false))
		|| ((en == 1) && (sensor_power == true))) {
		STK_ACC_LOG("Gsensor device have updated!\n");
	} else {
		res = stk_set_enable(stk, !sensor_power);
	}

	STK_ACC_DBG("Enable gsensor: %d", en);
	return res;
}

static int gsensor_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int gsensor_flush(void)
{
	return acc_flush_report();
}

static int gsensor_set_delay(u64 delay_ns)
{
	STK_ACC_LOG("delay= %d ms", (int)(delay_ns / 1000));
	return 0;
}

static int stk_readsensordata(int *pdata_x, int *pdata_y, int *pdata_z)
{
	struct stk8baxx_data *stk = stk_data;
	int accelData[STK_AXES_NUM] = {0};
	int res = 0;

	if (sensor_power == false) {
		res = stk_set_enable(stk, true);
		if (res) {
			*pdata_x = 0;
			*pdata_y = 0;
			*pdata_z = 0;
			STK_ACC_ERR("Power on stk8ba53 error %d!\n", res);
			return STK8BAXX_ERR_STATUS;
		}
	}

	res = stk_read_accel_data(stk);
	if (res) {
		STK_ACC_ERR("I2C error: ret value=%d", res);
		return STK8BAXX_ERR_STATUS;
	}

	if (atomic_read(&stk->trace) & STK_TRC_RAWDATA)
		STK_ACC_LOG("raw data x:%d, y:%d, z:%d", stk->xyz[0], stk->xyz[1], stk->xyz[2]);

	stk->xyz[STK_AXIS_X] = stk->xyz[STK_AXIS_X] * GRAVITY_EARTH_1000 / stk->sensitivity;
	stk->xyz[STK_AXIS_Y] = stk->xyz[STK_AXIS_Y] * GRAVITY_EARTH_1000 / stk->sensitivity;
	stk->xyz[STK_AXIS_Z] = stk->xyz[STK_AXIS_Z] * GRAVITY_EARTH_1000 / stk->sensitivity;

	stk->xyz[STK_AXIS_X] += stk->cvt.sign[STK_AXIS_X] * (s16)stk->cali_sw[stk->cvt.map[STK_AXIS_X]];
	stk->xyz[STK_AXIS_Y] += stk->cvt.sign[STK_AXIS_Y] * (s16)stk->cali_sw[stk->cvt.map[STK_AXIS_Y]];
	stk->xyz[STK_AXIS_Z] += stk->cvt.sign[STK_AXIS_Z] * (s16)stk->cali_sw[stk->cvt.map[STK_AXIS_Z]];

	accelData[stk->cvt.map[STK_AXIS_X]] = (int)(stk->cvt.sign[STK_AXIS_X] * stk->xyz[STK_AXIS_X]);
	accelData[stk->cvt.map[STK_AXIS_Y]] = (int)(stk->cvt.sign[STK_AXIS_Y] * stk->xyz[STK_AXIS_Y]);
	accelData[stk->cvt.map[STK_AXIS_Z]] = (int)(stk->cvt.sign[STK_AXIS_Z] * stk->xyz[STK_AXIS_Z]);
	if (atomic_read(&stk->trace) & STK_TRC_RAWDATA) {
		STK_ACC_LOG("map data x:%d, y:%d, z:%d, ss:%d",
				accelData[STK_AXIS_X], accelData[STK_AXIS_Y],
				accelData[STK_AXIS_Z], stk->sensitivity);
		STK_ACC_LOG("cali data x:%d, y:%d, z:%d",
				(s16)stk->cali_sw[stk->cvt.map[STK_AXIS_X]],
				(s16)stk->cali_sw[stk->cvt.map[STK_AXIS_Y]],
				(s16)stk->cali_sw[stk->cvt.map[STK_AXIS_Z]]);
	}

	*pdata_x = accelData[STK_AXIS_X];
	*pdata_y = accelData[STK_AXIS_Y];
	*pdata_z = accelData[STK_AXIS_Z];

	return 0;
}

static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	int res = 0;

	res = stk_readsensordata(x, y, z);
	if (res) {
		STK_ACC_ERR("Read sensor data error:%d", res);
		return -1;
	}

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static void stk_readCalibration(int *dat)
{
	struct stk8baxx_data *stk = stk_data;

	STK_ACC_LOG("ori x:%d, y:%d, z:%d", stk->cali_sw[STK_AXIS_X],
			stk->cali_sw[STK_AXIS_Y], stk->cali_sw[STK_AXIS_Z]);
	dat[STK_AXIS_X] = stk->cali_sw[STK_AXIS_X];
	dat[STK_AXIS_Y] = stk->cali_sw[STK_AXIS_Y];
	dat[STK_AXIS_Z] = stk->cali_sw[STK_AXIS_Z];
}

static int stk_writeCalibration(int *dat)
{
	struct stk8baxx_data *stk = stk_data;
	int err = 0;
	int cali[STK_AXES_NUM];

	stk_readCalibration(cali);
	STK_ACC_LOG("raw cali_sw[%d][%d][%d] dat[%d][%d][%d]",
			cali[0], cali[1], cali[2], dat[0], dat[1], dat[2]);

	cali[STK_AXIS_X] += dat[STK_AXIS_X];
	cali[STK_AXIS_Y] += dat[STK_AXIS_Y];
	cali[STK_AXIS_Z] += dat[STK_AXIS_Z];

	stk->cali_sw[STK_AXIS_X] = cali[STK_AXIS_X];
	stk->cali_sw[STK_AXIS_Y] = cali[STK_AXIS_Y];
	stk->cali_sw[STK_AXIS_Z] = cali[STK_AXIS_Z];

	STK_ACC_LOG("new cali_sw[%d][%d][%d]",
			stk->cali_sw[0], stk->cali_sw[1], stk->cali_sw[2]);

	msleep(1);

	return err;
}

static int stk_factory_enable_sensor(bool enable, int64_t sample_ms)
{
	int en = (true == enable ? 1 : 0);

	if (gsensor_enable_nodata(en)) {
		STK_ACC_ERR("enable sensor failed");
		return -1;
	}

	return 0;
}

static int stk_factory_get_data(int32_t data[3], int *status)
{
	return gsensor_get_data(&data[0], &data[1], &data[2], status);
}

static int stk_factory_get_raw_data(int32_t data[3])
{
	struct stk8baxx_data *stk = stk_data;
	int res = 0;

	res = stk_read_accel_rawdata(stk);
	if (res) {
		STK_ACC_ERR("I2C error: ret value=%d", res);
		return STK8BAXX_ERR_STATUS;
	} else {
		data[0] = (int32_t)stk->xyz[0];
		data[1] = (int32_t)stk->xyz[1];
		data[2] = (int32_t)stk->xyz[2];
	}
	return 0;
}

static int stk_factory_enable_cali(void)
{
	return 0;
}

static int stk_factory_clear_cali(void)
{
	struct stk8baxx_data *stk = stk_data;

	memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));
	return 0;
}

static int stk_factory_set_cali(int32_t data[3])
{
	int error = 0;
	struct stk8baxx_data *stk = stk_data;
	int cali[3] = {0, 0, 0};

	cali[0] = data[0] * stk->sensitivity / GRAVITY_EARTH_1000;
	cali[1] = data[1] * stk->sensitivity / GRAVITY_EARTH_1000;
	cali[2] = data[2] * stk->sensitivity / GRAVITY_EARTH_1000;

	STK_ACC_LOG("new x:%d, y:%d, z:%d", cali[0], cali[1], cali[2]);

	error = stk_writeCalibration(cali);
	if (error) {
		STK_ACC_ERR("stk_writeCalibration failed!");
		return -1;
	}
	return 0;
}

static int stk_factory_get_cali(int32_t data[3])
{
	struct stk8baxx_data *stk = stk_data;

	data[0] = (int32_t)(stk->cali_sw[0] * GRAVITY_EARTH_1000 / stk->sensitivity);
	data[1] = (int32_t)(stk->cali_sw[1] * GRAVITY_EARTH_1000 / stk->sensitivity);
	data[2] = (int32_t)(stk->cali_sw[2] * GRAVITY_EARTH_1000 / stk->sensitivity);
	STK_ACC_LOG("x:%d, y:%d, z:%d", data[0], data[1], data[2]);

	return 0;
}

static int stk_factory_do_self_test(void)
{
	struct stk8baxx_data *stk = stk_data;

	stk_selftest(stk);

	if (STK_SELFTEST_RESULT_NO_ERROR == atomic_read(&stk->selftest))
		return 0;
	else
		return -1;
}

static struct accel_factory_fops stk_factory_fops = {
	.enable_sensor = stk_factory_enable_sensor,
	.get_data = stk_factory_get_data,
	.get_raw_data = stk_factory_get_raw_data,
	.enable_calibration = stk_factory_enable_cali,
	.clear_cali = stk_factory_clear_cali,
	.set_cali = stk_factory_set_cali,
	.get_cali = stk_factory_get_cali,
	.do_self_test = stk_factory_do_self_test,
};

static struct accel_factory_public stk_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &stk_factory_fops,
};

/**
 * @brief: Proble function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] id: struct i2c_device_id *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk8baxx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0;
	struct stk8baxx_data *stk;
	struct acc_control_path stk_acc_control_path = {0};
	struct acc_data_path stk_acc_data_path = {0};
	s16 idmedata[CALI_SIZE] = {0};

	STK_ACC_FUN();

	/* kzalloc: allocate memory and set to zero. */
	stk = kzalloc(sizeof(struct stk8baxx_data), GFP_KERNEL);
	if (!stk) {
		STK_ACC_ERR("memory allocation error");
		return -ENOMEM;
	}

	error = get_accel_dts_func(client->dev.of_node, &stk->hw);
	if (0 != error) {
		STK_ACC_ERR("Dts info fail");
		goto err_free_mem;
	}
	client->addr = STK8BAXX_SLAVE_ADDR;

#ifdef STK_FIR
	if (0 == stk->hw.firlen) {
		stk->hw.firlen = STK_FIR_LEN;
	}
#endif /* STK_FIR */
	stk->hw.is_batch_supported = 0;

	/* direction */
	error = hwmsen_get_convert(stk->hw.direction, &stk->cvt);
	if (error) {
		STK_ACC_ERR("invalid direction: %d", stk->hw.direction);
		goto err_free_mem;
	}

	stk_data = stk;
	stk->client = client;
	i2c_set_clientdata(client, stk);
	mutex_init(&stk->reg_lock);
	if (stk_check_pid(stk)) {
		STK_ACC_ERR("check sensor id fail!");
		error = -ENODEV;
		goto exit_stk_init_error;
	}

	stk_data_initialize(stk);
	atomic_set(&stk->trace, 0);

	error = stk_reg_init(stk);
	if (error) {
		STK_ACC_ERR("stk8baxx initialization failed:%d", error);
		error = -ENODEV;
		goto exit_stk_init_error;
	}

	error = stk_create_attr(&stk_acc_init_info.platform_diver_addr->driver);
	if (error) {
		STK_ACC_ERR("stk_create_attr failed");
		goto exit_misc_error;
	}

	/* MTK Android usage +++ */
	stk_acc_control_path.is_use_common_factory = false;
	/* factory */
	error = accel_factory_device_register(&stk_factory_device);
	if (error) {
		STK_ACC_ERR("accel_factory_device_register failed");
		goto exit_create_factory_attr_failed;
	}

	stk_acc_control_path.open_report_data = gsensor_open_report_data;
	stk_acc_control_path.enable_nodata = gsensor_enable_nodata;
	stk_acc_control_path.is_support_batch = false;
	stk_acc_control_path.batch = gsensor_batch;
	stk_acc_control_path.flush = gsensor_flush;
	stk_acc_control_path.set_delay = gsensor_set_delay;
	stk_acc_control_path.is_report_input_direct = false;

	error = acc_register_control_path(&stk_acc_control_path);
	if (error) {
		STK_ACC_ERR("acc_register_control_path fail");
		goto exit_register_control_path;
	}

	stk_acc_data_path.get_data = gsensor_get_data;
	stk_acc_data_path.vender_div = 1000;
	error = acc_register_data_path(&stk_acc_data_path);
	if (error) {
		STK_ACC_ERR("acc_register_data_path fail");
		goto exit_register_control_path;
	}

	error = idme_get_sensorcal(idmedata, CALI_SIZE);
	if (error)
		STK_ACC_ERR("Get gsensor offset fail,default offset x=y=z=0");

	accel_xyz_offset[0] = idmedata[0];
	accel_xyz_offset[1] = idmedata[1];
	accel_xyz_offset[2] = idmedata[2];
	stk->cali_sw[STK_AXIS_X] = accel_xyz_offset[0];
	stk->cali_sw[STK_AXIS_Y] = accel_xyz_offset[1];
	stk->cali_sw[STK_AXIS_Z] = accel_xyz_offset[2];
	STK_ACC_LOG("idme to read:%d %d %d", accel_xyz_offset[0], accel_xyz_offset[1], accel_xyz_offset[2]);

	/* MTK Android usage --- */
	stk8baxx_init_flag = 0;
	STK_ACC_LOG("Stk8baxx probe Success!");
	return 0;

exit_register_control_path:
	stk_create_attr_exit(
		&stk_acc_init_info.platform_diver_addr->driver);
	accel_factory_device_deregister(&stk_factory_device);
exit_create_factory_attr_failed:
exit_misc_error:
exit_stk_init_error:
	mutex_destroy(&stk->reg_lock);
err_free_mem:
	kfree(stk);
	stk_data = NULL;
	stk = NULL;
	stk8baxx_init_flag = -1;
	return error;
}

/**
 * @brief
 */
static int stk8baxx_i2c_remove(struct i2c_client *client)
{
	struct stk8baxx_data *stk = i2c_get_clientdata(client);

	accel_factory_device_deregister(&stk_factory_device);
	stk_create_attr_exit(&stk_acc_init_info.platform_diver_addr->driver);
	mutex_destroy(&stk->reg_lock);
	kfree(stk);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stk_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stk8baxx_data *stk = i2c_get_clientdata(client);
	int error = 0;


	STK_ACC_FUN();
	if (!stk) {
		STK_ACC_ERR("Null point to find stk8baxx_data");
		return -EINVAL;
	}
	acc_driver_pause_polling(1);
	atomic_set(&stk->suspend, 1);
	error = stk_set_enable(stk, false);
	if (error) {
		acc_driver_pause_polling(0);
		atomic_set(&stk->suspend, 0);
		STK_ACC_ERR("set suspend fail");
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief:
 */
static int stk_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stk8baxx_data *stk = i2c_get_clientdata(client);
	int error = 0;

	if (!stk) {
		STK_ACC_ERR("Null point to find stk8baxx_data");
		return -EINVAL;
	}
	if (acc_driver_query_polling_state() == 1) {
		error = stk_set_enable(stk, true);
		if (error) {
			STK_ACC_ERR("set resume fail");
			return -EINVAL;
		}
	}
	atomic_set(&stk->suspend, 0);
	acc_driver_pause_polling(0);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id stk8baxx_i2c_id[] = {
	{STK_ACC_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, stk8baxx_i2c_id);

static const struct of_device_id stk_acc_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
MODULE_DEVICE_TABLE(i2c, stk_acc_match);

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops stk_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk_suspend, stk_resume)
};
#endif /* CONFIG_PM_SLEEP */

static struct i2c_driver stk8baxx_i2c_driver = {
	.driver = {
		.name           = STK_ACC_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm             = &stk_i2c_pm_ops,
#endif /* CONFIG_PM_SLEEP */
		.of_match_table = stk_acc_match,
	},
	.probe          = stk8baxx_i2c_probe,
	.remove         = stk8baxx_i2c_remove,
	.id_table       = stk8baxx_i2c_id,
};

static int stk_acc_init(void)
{
	STK_ACC_FUN();

	if (i2c_add_driver(&stk8baxx_i2c_driver)) {
		STK_ACC_ERR("Add i2c driver fail");
		return -1;
	}

	if (-1 == stk8baxx_init_flag) {
		STK_ACC_ERR("stk8baxx init error");
		return -1;
	}

	return 0;
}

static int stk_acc_remove(void)
{
	i2c_del_driver(&stk8baxx_i2c_driver);
	return 0;
}

static int __init stk8baxx_init(void)
{
	STK_ACC_FUN();
	acc_driver_add(&stk_acc_init_info);
	return 0;
}

static void __exit stk8baxx_exit(void)
{
	STK_ACC_FUN();
}

module_init(stk8baxx_init);
module_exit(stk8baxx_exit);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk8baxx 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");
