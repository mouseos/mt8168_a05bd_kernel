/*
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/sched/clock.h>

#include "cust_alsps.h"
#include "ltr559.h"
#include "alsps.h"

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define LTR559_DEV_NAME   "ltr559"

#ifdef SUPPORT_PSENSOR
#define GN_MTK_BSP_PS_DYNAMIC_CALI
#endif
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ltr559] "
#define APS_FUN(f)               pr_debug(APS_TAG "%s\n", __func__)

#define APS_ERR(fmt, args...)   pr_err(APS_TAG "%s %d"fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   pr_info(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   pr_debug(APS_TAG fmt, ##args)

#define DEF_LUX_THRESHOLD (400)
#define LUX_THRESHOLD_20 (20)
#define DEF_ALSCAL_HIGH (1092)
#define DEF_ALSCAL_LOW (0)

#define ALS_CAL_FILE   "/data/als_cal_data.bin"
#define LTR_DATA_BUF_NUM 1

#define I2C_RETRIES 2
#define I2C_RETRY_DELAY 5 /* ms */
#define LTR_TRC_RAWDATA 0x01


/*----------------------------------------------------------------------------*/

static struct ltr559_priv *ltr559_obj;

static DEFINE_MUTEX(ltr559_i2c_mutex);
static DEFINE_MUTEX(ltr559_mutex);
static DEFINE_MUTEX(ltr559_workqueue_mutex);

static int ltr559_init_flag = -1;
static uint32_t als_cal;
static int als_gainrange;
static const uint32_t max_uint16 = 65535;

#ifdef SUPPORT_PSENSOR
static unsigned long long int_top_time;
#endif

static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

#ifdef SUPPORT_PSENSOR
struct PS_CALI_DATA_STRUCT {
	int close;
	int far_away;
	int valid;
};
static struct PS_CALI_DATA_STRUCT ps_cali = { 0, 0, 0 };
static int intr_flag_value;
#endif

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int last_min_value = 2047;
static int ltr559_dynamic_calibrate(void);
#endif

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int ltr559_als_read(struct i2c_client *client, u16 *data);

static int als_enable_nodata(int en);
static int als_get_data(int *value, int *status);

static int ltr559_local_init(void);
static int ltr559_remove(void);
static void get_ltr_dts_func(struct device_node *node);

#ifdef CONFIG_IDME
extern unsigned int idme_get_alscal_value(void);
extern unsigned int idme_get_tp_cg_color(void);
static void get_als_cali(void);
#endif

#ifdef CONFIG_PM_SLEEP
static int ltr559_suspend(struct device *dev);
static int ltr559_resume(struct device *dev);
#endif

static struct alsps_init_info ltr559_init_info = {
	.name = "ltr559",
	.init = ltr559_local_init,
	.uninit = ltr559_remove,
};

/*----------------------------------------------------------------------------*/

typedef enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
} CMC_BIT;

enum {
	CWF = 1,
	A_D50 = 2,
	OTHER = 3,
};

enum {
	NONE = 0,
	BLACK = 1,
	WHITE = 2,
};

/*----------------------------------------------------------------------------*/

struct ltr559_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;
	struct mutex lock;

	/*misc */
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t ps_deb_on;	/*indicates if the debounce is on */
	atomic_t ps_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_suspend;
	atomic_t als_suspend;
	atomic_t als_enabled;
	atomic_t init_done;
	struct device_node *irq_node;
	int irq;

	/*data */
	u16 als;
	u16 ps;
	u8 _align;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL - 1];
	u32 als_value[C_CUST_ALS_LEVEL];
	u16 ps_cali;

	atomic_t als_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_low;	/*the cmd value can't be read, stored in ram */
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */

	u32 lux_threshold;	/* The ALS calibration threshold for Diag . Default Value 400. */
	u32 als_cal_high;
	u32 als_cal_high_reading;
	u32 als_cal_low;
	u32 cal_alg_ver;
	u8 tp_cg_color;/* 0:EVT&HVT && 1:black 2:white for DVT */
	u8 light_type;
	u32 cwf_coeff;
	u32 lux_formula_ver;
	int lux_report;
	int lux_ratio;
	atomic_t trace;
};

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr559_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr559_suspend, ltr559_resume)
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,ltr559"},
	{},
};
#endif

static int ltr559_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id);
static int ltr559_i2c_remove(struct i2c_client *client);
static int ltr559_i2c_detect(struct i2c_client *client,
				struct i2c_board_info *info);

static const struct i2c_device_id ltr559_i2c_id[] = {
	{LTR559_DEV_NAME, 0},
	{}
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr559_i2c_driver = {
	.probe = ltr559_i2c_probe,
	.remove = ltr559_i2c_remove,
	.detect = ltr559_i2c_detect,
	.id_table = ltr559_i2c_id,
	.driver = {
		.name = LTR559_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &ltr559_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

static int ltr559_i2c_read(struct ltr559_priv *obj, unsigned char reg,
			int len, unsigned char value[])
{
	struct i2c_client *client = NULL;
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

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	client = obj->client;
	msgs[0].addr = client->addr;
	msgs[1].addr = client->addr;

	do {
		err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		pr_err("%s: i2c_read error, %d\n", __func__, err);
		error = -1;
	}

	return error;
}

static int ltr559_i2c_write(struct ltr559_priv *obj,
			char *writebuf, int writelen)
{
	struct i2c_client *client;
	int ret = 0;

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	client = obj->client;

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
			pr_err("%s: i2c_write error, ret=%d", __func__, ret);
	}

	return ret;
}


static int ltr559_i2c_read_reg(unsigned char addr, unsigned char *value)
{
	int ret = 0;

	if (!ltr559_obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&ltr559_i2c_mutex);
	ret = ltr559_i2c_read(ltr559_obj, addr, 1, value);
	mutex_unlock(&ltr559_i2c_mutex);

	return ret;
}

static int ltr559_i2c_write_reg(unsigned char addr, unsigned char value)
{
	int ret = 0;
	unsigned char buf[2] = {0};

	if (!ltr559_obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&ltr559_i2c_mutex);
	buf[0] = addr;
	buf[1] = value;
	ret = ltr559_i2c_write(ltr559_obj, buf, sizeof(buf));
	mutex_unlock(&ltr559_i2c_mutex);

	return ret;
}

static int ltr559_i2c_reg_update_bit(unsigned char reg, unsigned char mask,
					unsigned char shift, int value)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned char buf[2] = {0};

	if (!ltr559_obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&ltr559_i2c_mutex);

	ret = ltr559_i2c_read(ltr559_obj, reg, 1, &reg_val);

	reg_val &= ~(mask << shift);
	reg_val |= (value << shift);

	buf[0] = reg;
	buf[1] = reg_val;
	ret = ltr559_i2c_write(ltr559_obj, buf, sizeof(buf));

	mutex_unlock(&ltr559_i2c_mutex);

	return 0;
}

#ifdef SUPPORT_PSENSOR
static int ltr559_ps_set_thres(void)
{
	int res;
	struct ltr559_priv *obj = ltr559_obj;

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	APS_FUN();

	APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);
	if (1 == ps_cali.valid) {
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_0, (u8)(ps_cali.far_away & 0x00FF));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_1, (u8)((ps_cali.far_away & 0xFF00) >> 8));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_0, (u8)(ps_cali.close & 0x00FF));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_1, (u8)((ps_cali.close & 0xFF00) >> 8));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	} else {
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_0, (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_1, (u8)((atomic_read(&obj->ps_thd_val_low) >> 8) & 0x00FF));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_0, (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_1, (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF));
		if (res < 0) {
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

	}
	APS_DBG("ps low: %d high: %d\n", atomic_read(&obj->ps_thd_val_low),
		atomic_read(&obj->ps_thd_val_high));

	res = 0;
	return res;

EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;
}

static int ltr559_ps_enable(struct i2c_client *client, int enable)
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);
	u8 regdata = 0;
	int err;

	APS_LOG("ltr559_ps_enable() ...start!\n");

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	err = ltr559_i2c_write_reg(LTR559_PS_LED, 0x7F);
	if (err < 0) {
		APS_LOG("ltr559 set ps pulse error\n");
		return err;
	}

	err = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 0x08);
	if (err < 0) {
		APS_LOG("ltr559 set ps pulse error\n");
		return err;
	}

	err = ltr559_i2c_write_reg(0x84, 0x00);
	if (err < 0) {
		APS_LOG("ltr559 set ps meas error\n");
		return err;
	}

	err = ltr559_i2c_write_reg(LTR559_INTERRUPT, 0x01);
	if (err < 0) {
		APS_LOG("ltr559 set ps pulse error\n");
		return err;
	}

	err = ltr559_i2c_write_reg(LTR559_INTERRUPT_PERSIST, 0x10);
	if (err < 0) {
		APS_LOG("ltr559 set ps pulse error\n");
		return err;
	}

	res = ltr559_i2c_reg_update_bit(LTR559_PS_CONTR,
				PS_MODE_MASK, PS_MODE_SHIFT, enable);
	if (err < 0) {
		APS_ERR("ltr559 PS_CONTR update error\n");
		return err;
	}

	msleep(WAKEUP_DELAY);

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI

	err = ltr559_i2c_read_reg(LTR559_PS_CONTR, &regdata);
	if (err < 0) {
		APS_ERR("ltr559 read PS_CONTR error\n");
		return err;
	}

	if (regdata & 0x02) {
		if (ltr559_dynamic_calibrate() < 0)
			return -1;
	}

	ltr559_ps_set_thres();

#endif

	return 0;

}

static int ltr559_ps_read(struct i2c_client *client, u16 *data)
{
	int psdata, res;
	u8 psval_lo = 0, psval_hi = 0;

	res = ltr559_i2c_read_reg(LTR559_PS_DATA_0, &psval_lo);
	if (res < 0) {
		APS_ERR("ltr559 read PS_DATA error\n");
		goto out;
	}

	APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
	if (psval_lo < 0) {
		APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}

	res = ltr559_i2c_read_reg(LTR559_PS_DATA_1, &psval_hi);
	if (res < 0) {
		APS_ERR("ltr559 read PS_DATA_1 error\n");
		goto out;
	}

	APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);

	if (psval_hi < 0) {
		APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}

	psdata = ((psval_hi & 7) * 256) + psval_lo;
	APS_DBG("ps_rawdata = %d\n", psdata);

	*data = psdata;
	return 0;

out:

	return psdata;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t als_show(struct device_driver *ddri, char *buf)
{
	int res;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_als_read(ltr559_obj->client, &ltr559_obj->als);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->als);

}
#ifdef SUPPORT_PSENSOR
/*----------------------------------------------------------------------------*/
static ssize_t ps_show(struct device_driver *ddri, char *buf)
{
	int res;
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", ltr559_obj->ps);
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t status_show(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	if (ltr559_obj->hw) {
		len +=
			scnprintf(buf + len, PAGE_SIZE - len, "CUST: %d, (%d %d)\n",
				ltr559_obj->hw->i2c_num, ltr559_obj->hw->power_id,
				ltr559_obj->hw->power_vol);
	} else {
		len += scnprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	len +=
		scnprintf(buf + len, PAGE_SIZE - len, "MISC: %d %d\n",
			atomic_read(&ltr559_obj->als_suspend),
			atomic_read(&ltr559_obj->ps_suspend));

	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t reg_show(struct device_driver *ddri, char *buf)
{
	int i, len = 0, res;
	u8 regdata = 0;
	int reg[] = {
		0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
		0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
		0x92, 0x93, 0x94, 0x95, 0x97, 0x98, 0x99, 0x9a, 0x9e
	};

	for (i = 0; i < 27; i++) {
		res = ltr559_i2c_read_reg(reg[i], &regdata);
		if (res < 0) {
			APS_LOG("ltr559 read reg[%d]:%d error\n", i, reg[i]);
			return 0;
		}
		len += scnprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%04X value: 0x%04X\n", reg[i], regdata);
	}
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t reg_store(struct device_driver *ddri, const char *buf,
				size_t count)
{
	int res, value;
	u8 regdata = 0;
	u8 reg;
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	if (2 == sscanf(buf, "%hhx %x ", &reg, &value)) {
		res = ltr559_i2c_read_reg(reg, &regdata);
		if (res < 0) {
			APS_ERR("ltr559 read reg error\n");
			return 0;
		}
		APS_DBG("before write reg: %x, reg_value = %x  write value=%x\n",
						reg, regdata, value);
		res = ltr559_i2c_write_reg(reg, value);
		if (res < 0) {
			APS_LOG("ltr559 write reg error\n");
			return 0;
		}

		res = ltr559_i2c_read_reg(reg, &regdata);
		if (res < 0) {
			APS_ERR("ltr559 read reg error\n");
			return 0;
		}
		APS_DBG("after write reg: %x, reg_value = %x\n", reg, regdata);
	} else {
		APS_DBG("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}

/******************************************************************************
 * Sysfs attributes for Diag kaka
*******************************************************************************/

inline uint32_t ltr_alscode2lux(uint32_t alscode)
{
	alscode += ((alscode << 7) + (alscode << 3) + (alscode >> 1));
	alscode <<= 3;
	if (ltr559_obj->als_cal_high > 0)
		alscode /= ltr559_obj->als_cal_high;
	else
		return 0;

	return alscode;
}

static inline int32_t ltr559_als_get_data_avg(int sSampleNo)
{
	int32_t DataCount = 0;
	int32_t sAveAlsData = 0;

	u16 als_reading = 0;
	int result = 0;

	struct ltr559_priv *obj = NULL;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	obj = ltr559_obj;

	result = ltr559_als_read(obj->client, &als_reading);
	APS_LOG("[%s]: Ignore first als value:%d\n", __func__, als_reading);

	while (DataCount < sSampleNo) {
		msleep(100);
		result = ltr559_als_read(obj->client, &als_reading);
		APS_LOG("%s: [#23][LTR]als code = %d\n", __func__, als_reading);
		sAveAlsData += als_reading;
		DataCount++;
	}
	sAveAlsData /= sSampleNo;
	return sAveAlsData;
}

static bool als_store_cali_in_file(const char *filename,
						 unsigned int value, unsigned int value_low)
{
	struct file *cali_file;
	mm_segment_t fs;
	char w_buf[LTR_DATA_BUF_NUM * sizeof(unsigned int) * 2 + 1] = { 0 };
	char r_buf[LTR_DATA_BUF_NUM * sizeof(unsigned int) * 2 + 1] = { 0 };
	int i;
	char *dest = w_buf;

	APS_LOG("%s enter", __func__);
	cali_file = filp_open(filename, O_CREAT | O_RDWR, 0777);

	if (IS_ERR(cali_file)) {
		APS_ERR("open error! exit!\n");
		return false;
	} else {
		fs = get_fs();
		set_fs(get_ds());

		for (i = 0; i < LTR_DATA_BUF_NUM; i++) {
			sprintf(dest, "%02X", value & 0x000000FF);
			dest += 2;
			sprintf(dest, "%02X", (value >> 8) & 0x000000FF);
			dest += 2;
			sprintf(dest, "%02X", value_low & 0x000000FF);
			dest += 2;
			sprintf(dest, "%02X", (value_low >> 8) & 0x000000FF);
		}

		APS_LOG("w_buf: %s \n", w_buf);
		vfs_write(cali_file, (void *)w_buf,
				LTR_DATA_BUF_NUM * sizeof(unsigned int) * 2 + 1, &cali_file->f_pos);
		cali_file->f_pos = 0x00;
		vfs_read(cali_file, (void *)r_buf,
				LTR_DATA_BUF_NUM * sizeof(unsigned int) * 2 + 1, &cali_file->f_pos);

		for (i = 0; i < LTR_DATA_BUF_NUM * sizeof(unsigned int) * 2 + 1; i++) {
			if (r_buf[i] != w_buf[i]) {
				set_fs(fs);
				filp_close(cali_file, NULL);
				APS_ERR("read back error! exit!\n");
				return false;
			}
		}

		set_fs(fs);
	}

	filp_close(cali_file, NULL);
	APS_LOG("pass\n");
	return true;
}

/*--------------------------------------------------------------------------------------------*/

static ssize_t enable_show(struct device_driver *ddri, char *buf)
{
	int32_t enabled = 0;
	int32_t ret = 0;
	u8 regdata = 0;

	if (test_bit(CMC_BIT_ALS, &ltr559_obj->enable))
		enabled = 1;
	else
		enabled = 0;

	ret = ltr559_i2c_read_reg(LTR559_ALS_CONTR, &regdata);
	if (ret < 0) {
		APS_ERR("ltr559 read reg error\n");
		return 0;
	}

	if (regdata & 0x01) {
		APS_LOG("ALS Enabled \n");
		ret = 1;
	} else {
		APS_LOG("ALS Disabled \n");
		ret = 0;
	}

	if (enabled != ret)
		APS_ERR("driver_enable=0x%x, sensor_enable=%x\n", enabled, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t enable_store(struct device_driver *ddri,
				       const char *buf, size_t size)
{
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		APS_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	als_enable_nodata(en);

	APS_LOG("%s: Enable ALS : %d\n", __func__, en);

	return size;
}

static ssize_t lux_show(struct device_driver *ddri, char *buf)
{
	int32_t als_reading = 0;

	als_reading = ltr559_als_get_data_avg(5);

	als_reading = ltr_alscode2lux(als_reading);

	return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_reading);

}

static ssize_t lux_threshold_show(struct device_driver *ddri,
					     char *buf)
{
	int32_t lux_threshold;

	lux_threshold = ltr559_obj->lux_threshold;
	return scnprintf(buf, PAGE_SIZE, "%d\n", lux_threshold);
}

static ssize_t lux_threshold_store(struct device_driver *ddri,
					      const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		APS_ERR("strict_strtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ltr559_obj->lux_threshold = value;
	return size;
}

static ssize_t alscal_high_value_show(struct device_driver *ddri,
					     char *buf)
{
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->als_cal_high);
}

static ssize_t alscal_high_value_store(struct device_driver *ddri,
					      const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		APS_ERR("strict_strtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ltr559_obj->als_cal_high = value;
	return size;
}

static ssize_t alscal_low_value_show(struct device_driver *ddri,
					     char *buf)
{
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->als_cal_low);
}

static ssize_t alscal_low_value_store(struct device_driver *ddri,
					      const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		APS_ERR("strict_strtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ltr559_obj->als_cal_low = value;
	return size;
}

static ssize_t cali_Light_High_show(struct device_driver *ddri, char *buf)
{
	int32_t als_reading = 0;
	bool result = false;

	APS_LOG("%s:[#23][LTR]Start Cali light...\n", __func__);

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	ltr559_obj->lux_threshold = DEF_LUX_THRESHOLD;
	msleep(150);
	als_reading = ltr559_als_get_data_avg(10);

	if ((als_reading > 0) && (als_reading <= max_uint16)) {
		ltr559_obj->als_cal_high_reading = als_reading;

		result = als_store_cali_in_file(ALS_CAL_FILE,
						ltr559_obj->als_cal_high_reading, ltr559_obj->als_cal_low);
		/* transform to old ratio */
		ltr559_obj->als_cal_high = ltr559_obj->als_cal_high_reading * 1100 / DEF_LUX_THRESHOLD;
	} else {
		APS_ERR("[#23][LTR]cali light fail!!!als_value= %d\n", als_reading);
		result = false;
	}

	APS_LOG("Threshold:%d, als_cal:%d result:%s\n", ltr559_obj->lux_threshold, als_reading, result ? "PASSED" : "FAIL");
	return scnprintf(buf, PAGE_SIZE,
				"%s:Threshold = %d, als_value = %d\n",
				result ? "PASSED" : "FAIL", ltr559_obj->lux_threshold, als_reading);
}

static ssize_t cali_Light_Low_show(struct device_driver *ddri, char *buf)
{
	int32_t als_reading = 0;
	bool result = false;

	APS_LOG("%s:[#23][LTR]Start Cali light...\n", __func__);

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	ltr559_obj->lux_threshold = LUX_THRESHOLD_20;
	msleep(150);
	als_reading = ltr559_als_get_data_avg(10);

	if ((als_reading > 0) && (als_reading <= max_uint16)) {
		ltr559_obj->als_cal_low = als_reading;
		/* low value can't larger than half of high */
		if (ltr559_obj->als_cal_low > ltr559_obj->als_cal_high_reading / 2) {
			result = false;
			APS_ERR("als data is not suitable! Please check the light source or light sensor!\n");
			goto out;
		}

		result = als_store_cali_in_file(ALS_CAL_FILE,
						ltr559_obj->als_cal_high_reading, ltr559_obj->als_cal_low);

	} else {
		APS_ERR("[#23][LTR]cali light fail!!!als_value= %d\n", als_reading);
		result = false;
	}

out:
	APS_LOG("Threshold:%d, als_cal:%d result:%s\n", ltr559_obj->lux_threshold, als_reading, result ? "PASSED" : "FAIL");
	return scnprintf(buf, PAGE_SIZE,
				"%s:Threshold = %d, als_value = %d\n",
				result ? "PASSED" : "FAIL", ltr559_obj->lux_threshold, als_reading);
}

static ssize_t tp_cg_color_show(struct device_driver *ddri,
					     char *buf)
{
	int32_t tp_type;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	tp_type = ltr559_obj->tp_cg_color;
	return scnprintf(buf, PAGE_SIZE, "%d\n", tp_type);
}

static ssize_t tp_cg_color_store(struct device_driver *ddri,
					      const char *buf, size_t size)
{
	int value = 0;
	int ret;

	ret = kstrtoint(buf, 10, &value);
	if (ret < 0) {
		APS_ERR("strict_strtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ltr559_obj->tp_cg_color = value;
	return size;
}

static ssize_t light_type_show(struct device_driver *ddri,
					     char *buf)
{
	int32_t light_type;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	light_type = ltr559_obj->light_type;
	return scnprintf(buf, PAGE_SIZE, "%d\n", light_type);
}
/*
 * notice:Due to the black&white tp's raw data has great difference
 *        in factory AFT 400lux.We set tp_cg_color to 0 here,using
 *        original formula to get original raw data for fixture re-check.
 *
 */
static ssize_t als_check_show(struct device_driver *ddri,
					     char *buf)
{
	int res;
	u8 tp_cg_buf;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	tp_cg_buf = ltr559_obj->tp_cg_color;
	ltr559_obj->tp_cg_color = NONE;
	res = ltr559_als_read(ltr559_obj->client, &ltr559_obj->als);
	ltr559_obj->tp_cg_color = tp_cg_buf;/* restore tp cg color */
	return scnprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->als);
}

static ssize_t lux_report_show(struct device_driver *ddri,
					     char *buf)
{
	int32_t als_reading;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	als_reading = ltr559_obj->lux_report;
	return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_reading);
}

static ssize_t lux_ratio_show(struct device_driver *ddri,
					     char *buf)
{
	int32_t als_ratio;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	als_ratio = ltr559_obj->lux_ratio;
	return scnprintf(buf, PAGE_SIZE, "%d\n", als_ratio);
}

static ssize_t trace_show(struct device_driver *ddri,
					     char *buf)
{
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr559_obj->trace));
}

static ssize_t trace_store(struct device_driver *ddri,
					      const char *buf, size_t size)
{
	int trace;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	if (sscanf(buf, "0x%11x", &trace) == 1)
		atomic_set(&ltr559_obj->trace, trace);
	else
		APS_ERR("invalid content: '%s'\n", buf);
	return size;
}


/*----------------------------------------------------------------------------*/
/*
 * notice: following debug node can read different als data.
 * als:for read als raw data.
 * lux:for diag read lux,converted from als raw data. ==> raw_data*factory_ratio
 * als_check:for diag re-check tp color in AFT. ==> old_formula's raw data
 * lux_report:The als data report to hal layer. ==> raw_data*factory_ratio*coeff
 */
static DRIVER_ATTR_RO(als);
#ifdef SUPPORT_PSENSOR
static DRIVER_ATTR_RO(ps);
#endif
static DRIVER_ATTR_RO(status);
static DRIVER_ATTR_RW(reg);
/*----------------------------------------------------------------------------*/
/* For Diag to Calibrate the ALS */
static DRIVER_ATTR_RW(enable);
static DRIVER_ATTR_RO(lux);
static DRIVER_ATTR_RW(lux_threshold);
static DRIVER_ATTR_RW(alscal_high_value);
static DRIVER_ATTR_RW(alscal_low_value);
static DRIVER_ATTR_RO(cali_Light_High);
static DRIVER_ATTR_RO(cali_Light_Low);
static DRIVER_ATTR_RW(tp_cg_color);
static DRIVER_ATTR_RO(light_type);
static DRIVER_ATTR_RO(als_check);
static DRIVER_ATTR_RO(lux_report);
static DRIVER_ATTR_RO(lux_ratio);
static DRIVER_ATTR_RW(trace);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr559_attr_list[] = {
	&driver_attr_als,
#ifdef SUPPORT_PSENSOR
	&driver_attr_ps,
#endif
	&driver_attr_status,
	&driver_attr_reg,
	&driver_attr_enable,
	&driver_attr_lux,
	&driver_attr_lux_threshold,
	&driver_attr_alscal_high_value,
	&driver_attr_alscal_low_value,
	&driver_attr_cali_Light_High,
	&driver_attr_cali_Light_Low,
	&driver_attr_tp_cg_color,
	&driver_attr_light_type,
	&driver_attr_als_check,
	&driver_attr_lux_report,
	&driver_attr_lux_ratio,
	&driver_attr_trace,
};

/*----------------------------------------------------------------------------*/
static int ltr559_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(ltr559_attr_list);

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, ltr559_attr_list[idx]);
		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n",
				ltr559_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int ltr559_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(ltr559_attr_list);

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, ltr559_attr_list[idx]);
	}

	return err;
}

/*
 * ################
 * ## ALS CONFIG ##
 * ################
 */

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr559_dynamic_calibrate(void)
{
	int i = 0;
	int j = 0;
	int data = 0;
	int noise = 0;
	int max = 0;
	unsigned long data_total = 0;
	struct ltr559_priv *obj = ltr559_obj;

	APS_FUN(f);
	if (!obj)
		goto err;

	msleep(20);
	for (i = 0; i < 5; i++) {
		if (max++ > 5) {
			goto err;
		}
		msleep(15);

		ltr559_ps_read(obj->client, &obj->ps);
		data = obj->ps;

		if (data == 0) {
			j++;
		}
		data_total += data;
	}
	noise = data_total / (5 - j);
	if ((noise < last_min_value + 100)) {
		last_min_value = noise;
		if (noise < 50) {
			atomic_set(&obj->ps_thd_val_high, noise + 50);
			atomic_set(&obj->ps_thd_val_low, noise + 20);

		} else if (noise < 100) {
			atomic_set(&obj->ps_thd_val_high, noise + 60);
			atomic_set(&obj->ps_thd_val_low, noise + 30);
		} else if (noise < 200) {
			atomic_set(&obj->ps_thd_val_high, noise + 90);
			atomic_set(&obj->ps_thd_val_low, noise + 60);
		} else if (noise < 300) {
			atomic_set(&obj->ps_thd_val_high, noise + 130);
			atomic_set(&obj->ps_thd_val_low, noise + 90);
		} else if (noise < 400) {
			atomic_set(&obj->ps_thd_val_high, noise + 150);
			atomic_set(&obj->ps_thd_val_low, noise + 120);
		} else if (noise < 600) {
			atomic_set(&obj->ps_thd_val_high, noise + 220);
			atomic_set(&obj->ps_thd_val_low, noise + 180);
		} else if (noise < 800) {
			atomic_set(&obj->ps_thd_val_high, noise + 280);
			atomic_set(&obj->ps_thd_val_low, noise + 240);
		} else {
			atomic_set(&obj->ps_thd_val_high, 1000);
			atomic_set(&obj->ps_thd_val_low, 880);
		}

	}
	APS_DBG(" calibrate:noise=%d, thdlow= %d , thdhigh = %d\n", noise,
		atomic_read(&obj->ps_thd_val_low),
		atomic_read(&obj->ps_thd_val_high));

	return 0;
      err:
	APS_ERR("ltr559_dynamic_calibrate fail!!!\n");
	return -1;
}
#endif

static int ltr559_als_enable(struct i2c_client *client, bool enable)
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);
	int res = 0;

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	if (enable == atomic_read(&obj->als_enabled)) {
		APS_DBG("als status is newest:%d\n", enable);
		return 0;
	}

	res = ltr559_i2c_reg_update_bit(LTR559_ALS_CONTR,
				ALS_MODE_MASK, ALS_MODE_SHIFT, enable);
	if (res < 0) {
		APS_ERR("reg_update fail res = %d\n", res);
		return res;
	}

	atomic_set(&obj->als_enabled, enable);
	mdelay(WAKEUP_DELAY);

	return 0;

}

static int ltr559_als_formula_0(int alsval_ch0, int alsval_ch1)
{
	int ratio, luxdata_int;

	if ((alsval_ch1 == 0) || (alsval_ch0 == 0)) {
		ratio = 0;
	} else {
		ratio = (alsval_ch1 * 100) / (alsval_ch0 + alsval_ch1);
	}
	ltr559_obj->lux_ratio = ratio;
	if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
		APS_LOG("ratio = %d  gainrange = %d\n", ratio, als_gainrange);
	/*
	 * Lux = (coef_a * ch0 + coef_b * ch1) / gain / intergration.
	 * Formula_0 is a default formula from LTR-559_Appendix A Ver_1.0.pdf.
	 * Onyx evt/hvt device will go this formula.
	 */

	/*CWF light*/
	if (ratio < 50) {
		luxdata_int =
			(((17743 * alsval_ch0) +
			(11059 * alsval_ch1)) / als_gainrange) / 1040;
	} /*D65 light*/
	else if ((ratio < 66) && (ratio >= 50)) {
		luxdata_int =
			(((5926 * alsval_ch0) +
			(1185 * alsval_ch1)) / als_gainrange) / 884;
	} /* A light */
	else if ((ratio < 99) && (ratio >= 66)) {
		luxdata_int =
			(((5926 * alsval_ch0) +
			(1185 * alsval_ch1)) / als_gainrange) / 1030;
	} else {
		luxdata_int = 0;
	}
	if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
		APS_LOG("Als_org_value_lux = %d\n", luxdata_int);
	return luxdata_int;
}

static int ltr559_als_formula_1(int alsval_ch0, int alsval_ch1)
{
	int ratio, luxdata_int;

	if ((alsval_ch1 == 0) || (alsval_ch0 == 0)) {
		ratio = 0;
	} else {
		ratio = (alsval_ch1 * 100) / (alsval_ch0 + alsval_ch1);
	}
	ltr559_obj->lux_ratio = ratio;
	if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
		APS_LOG("ratio = %d  gainrange = %d\n", ratio, als_gainrange);
	/*
	 * Lux = (coef_a * ch0 + coef_b * ch1) / gain / intergration.
	 * Onyx dvt&after device will go this formula.
	 * For tp with cg color info,tp_cg_color(store in idme,written in factory),
	 *     we test and generate particular formula for white/black tp of
	 *     different light source & lux strength.
	 * If there is no tp info,we set default formula from
	 *     LTR-559_Appendix A Ver_1.0.pdf.
	 */
	if (ltr559_obj->tp_cg_color == BLACK) {/* black tp */
		/*CWF light*/
		if (ratio < 52) {
			ltr559_obj->light_type = CWF;
			luxdata_int =
				(((-1561 * alsval_ch0) +
				(6061 * alsval_ch1)) / als_gainrange) / 100;
		} else if (ratio < 78) {/* A&D50 light */
			ltr559_obj->light_type = A_D50;
			if (ratio == 77 && alsval_ch0 < 2000) {
				luxdata_int =
					(((471 * alsval_ch0) +
					(-58 * alsval_ch1)) / als_gainrange) / 100;
				if (luxdata_int < 30)
					luxdata_int = luxdata_int * 76 / 100;
				else if (luxdata_int < 40)
					luxdata_int = luxdata_int * ((luxdata_int - 30) * 24 / 10 + 76) / 100;
			} else {
				luxdata_int =
					(((8620 * alsval_ch0) +
					(-614 * alsval_ch1)) / als_gainrange) / 1000;
			}
			luxdata_int = luxdata_int * 105/100;
		} else {
			ltr559_obj->light_type = OTHER;
			if (ratio > 85) {
				luxdata_int =
					(((5926 * alsval_ch0) +
					(1185 * alsval_ch1)) / als_gainrange) / 10000;
			} else {
				if (alsval_ch0 > 5000 && ratio == 78) {
					luxdata_int =
						(((8620 * alsval_ch0) +
						(-614 * alsval_ch1)) / als_gainrange) / 1000;
				} else {
					luxdata_int =
						(((471 * alsval_ch0) +
						(-58 * alsval_ch1)) / als_gainrange) / 100;
				}
				if (luxdata_int < 30)
					luxdata_int = luxdata_int * 76 / 100;
				else if (luxdata_int < 40)
					luxdata_int = luxdata_int * ((luxdata_int - 30) * 24 / 10 + 76) / 100;
				else if (luxdata_int < 100)
					luxdata_int = luxdata_int * 105 / 100;
			}
			luxdata_int = luxdata_int * 105/100;
		}
		if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
			APS_LOG("black_als_org_value_lux = %d\n", luxdata_int);
	} else if (ltr559_obj->tp_cg_color == WHITE) {/* white tp */
		/*CWF light*/
		if (ratio < 52) {
			ltr559_obj->light_type = CWF;
			luxdata_int =
				37 * ((17743 * alsval_ch0 +
				11059 * alsval_ch1) / als_gainrange) / 10000;
		} else if (ratio < 67 && ratio >= 50) {/* A&D50 light*/
			ltr559_obj->light_type = A_D50;
			luxdata_int =
				8 * ((42785 * alsval_ch0 -
				19548 * alsval_ch1) / als_gainrange) / 1000;
		} else if (ratio >= 67) {
			ltr559_obj->light_type = A_D50;
			luxdata_int =
				3 * (((5926 * alsval_ch0) +
				(1185 * alsval_ch1)) / als_gainrange) / 1000;

			if (luxdata_int < 600 && ratio >= 77) {
				if (luxdata_int < 100)
					luxdata_int = luxdata_int * 15 / 100;
				else if (luxdata_int < 200)
					luxdata_int = luxdata_int * ((luxdata_int - 100) * 5 / 100 + 15) / 100;
				else if (luxdata_int < 500)
					luxdata_int = luxdata_int * 3 / 10;
				else
					luxdata_int = luxdata_int * ((luxdata_int - 500) * 70 / 100 + 30) / 100;
			}
			if (luxdata_int > 300 && luxdata_int < 900) {
				if (luxdata_int < 400)
					luxdata_int = luxdata_int * (100 + (luxdata_int - 300) * 20 / 100) / 100;
				else
					luxdata_int = luxdata_int * 12 / 10;
			}
		}
		if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
			APS_LOG("white_als_org_value_lux = %d\n", luxdata_int);
	} else {/* no tp info */
		/*CWF light*/
		if (ratio < 50) {
			luxdata_int =
				(((17743 * alsval_ch0) +
				(11059 * alsval_ch1)) / als_gainrange) / 1040;
		} /*D65 light*/
		else if ((ratio < 66) && (ratio >= 50)) {
			luxdata_int =
				(((5926 * alsval_ch0) +
				(1185 * alsval_ch1)) / als_gainrange) / 884;
		} /* A light */
		else if ((ratio < 99) && (ratio >= 66)) {
			luxdata_int =
				(((5926 * alsval_ch0) +
				(1185 * alsval_ch1)) / als_gainrange) / 1030;
		} else {
			luxdata_int = 0;
		}
		if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
			APS_LOG("als_org_value_lux = %d\n", luxdata_int);
	}
	return luxdata_int;
}

static int ltr559_als_read(struct i2c_client *client, u16 *data)
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);
	u8 alsval_ch0_lo = 0, alsval_ch0_hi = 0;
	u8 alsval_ch1_lo = 0, alsval_ch1_hi = 0;
	int alsval_ch0, alsval_ch1;
	int luxdata;
	int res;

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	if (atomic_read(&obj->als_suspend)) {
		luxdata = 0;
		goto out;
	}

	res = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0, &alsval_ch1_lo);
	if (res < 0) {
		APS_ERR("ltr559 read ALS_DATA_CH1_0 error\n");
		return -1;
	}
	res = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1, &alsval_ch1_hi);
	if (res < 0) {
		APS_ERR("ltr559 read ALS_DATA_CH1_1 error\n");
		return -1;
	}

	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	res = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0, &alsval_ch0_lo);
	if (res < 0) {
		APS_ERR("ltr559 read ALS_DATA_CH0_0 error\n");
		return -1;
	}
	res = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1, &alsval_ch0_hi);
	if (res < 0) {
		APS_ERR("ltr559 read ALS_DATA_CH0_1 error\n");
		return -1;
	}

	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

	if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA) {
		APS_LOG("alsval_ch0_lo = %d,alsval_ch0_hi=%d,alsval_ch0=%d\n",
			alsval_ch0_lo, alsval_ch0_hi, alsval_ch0);
		APS_LOG("alsval_ch1_lo = %d,alsval_ch1_hi=%d,alsval_ch1=%d\n",
			alsval_ch1_lo, alsval_ch1_hi, alsval_ch1);
	}
	/* we can compatible for different formula here.For onyx:dvt 1,evt/hvt 0 */
	if (ltr559_obj->lux_formula_ver == 0)
		luxdata = ltr559_als_formula_0(alsval_ch0, alsval_ch1);
	else if (ltr559_obj->lux_formula_ver == 1)
		luxdata = ltr559_als_formula_1(alsval_ch0, alsval_ch1);
	else
		luxdata = ltr559_als_formula_0(alsval_ch0, alsval_ch1);

	if (luxdata < 0)
		luxdata = 0;

out:
	*data = luxdata;
	return 0;

}

/*-----------------------------------------------------------------------------*/
#ifdef SUPPORT_PSENSOR
int ltr559_eint_func(void)
{
	struct ltr559_priv *obj = ltr559_obj;

	APS_FUN();

	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
	return 0;
}

static irqreturn_t ltr559_eint_handler(int irq, void *desc)
{
	int res = 0;
	disable_irq_nosync(ltr559_obj->irq);
	res = ltr559_eint_func();
	if (res < 0)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
int ltr559_setup_eint(struct i2c_client *client)
{
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = { 0, 0 };

	APS_FUN();
	if (!ltr559_obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	/* gpio setting */
        pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("Cannot find alsps pinctrl!\n");
		return ret;
	}
        pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		pr_err("Cannot find alsps pinctrl default!\n");
	}

        pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		pr_err("Cannot find alsps pinctrl pin_cfg!\n");
		return ret;
	}
        pinctrl_select_state(pinctrl, pins_cfg);

	/* eint request */
	if (ltr559_obj->irq_node) {
		of_property_read_u32_array(ltr559_obj->irq_node, "debounce",
					   ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		ltr559_obj->irq = irq_of_parse_and_map(ltr559_obj->irq_node, 0);
		APS_LOG("ltr559_obj->irq = %d\n", ltr559_obj->irq);
		if (!ltr559_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq (ltr559_obj->irq, ltr559_eint_handler, IRQF_TRIGGER_FALLING,
			"PS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
#endif

static int ltr559_check_and_clear_intr(struct i2c_client *client)
{
	int res, intp, intl;
	u8 buffer[2];

	APS_FUN();

	res = ltr559_i2c_read_reg(LTR559_ALS_PS_STATUS, buffer);
	if (res < 0) {
		goto EXIT_ERR;
	}

	res = 1;
	intp = 0;
	intl = 0;
	if (0 != (buffer[0] & 0x02)) {
		res = 0;
		intp = 1;
	}
	if (0 != (buffer[0] & 0x08)) {
		res = 0;
		intl = 1;
	}

	if (0 == res) {
		if ((1 == intp) && (0 == intl)) {
			buffer[1] = buffer[0] & 0xfD;
		} else if ((0 == intp) && (1 == intl)) {
			buffer[1] = buffer[0] & 0xf7;
		} else {
			buffer[1] = buffer[0] & 0xf5;
		}
		res = ltr559_i2c_write_reg(LTR559_ALS_PS_STATUS, buffer[1]);
		if (res < 0) {
			goto EXIT_ERR;
		} else {
			res = 0;
		}
	} else
		return 0;

EXIT_ERR:
	APS_ERR("ltr559_check_and_clear_intr fail\n");
	return 1;
}


/*----------------------------------------------------------------------------*/
#ifdef SUPPORT_PSENSOR
static int ltr559_check_intr(struct i2c_client *client)
{

	int res, intp, intl;
	u8 buffer[2];

	APS_FUN();

	res = ltr559_i2c_read_reg(LTR559_ALS_PS_STATUS, buffer);
	if (res < 0) {
		APS_ERR("ltr559 read ALS_PS_STATUS error\n");
		return -1;
	}

	APS_LOG("status = %x\n", buffer[0]);

	res = 1;
	intp = 0;
	intl = 0;
	if (0 != (buffer[0] & 0x02)) {
		res = 0;
		intp = 1;
	}
	if (0 != (buffer[0] & 0x08)) {
		res = 0;
		intl = 1;
	}

	if (0 == res) {
		if ((1 == intp) && (0 == intl)) {
			APS_LOG("PS interrupt\n");
			buffer[1] = buffer[0] & 0xfD;

		} else if ((0 == intp) && (1 == intl)) {
			APS_LOG("ALS interrupt\n");
			buffer[1] = buffer[0] & 0xf7;
		} else {
			APS_LOG("Check ALS/PS interrup error\n");
			buffer[1] = buffer[0] & 0xf5;
		}
	}

	return res;
}

static int ltr559_clear_intr(struct i2c_client *client)
{
	u8 buffer;
	int res;

	APS_FUN();

	res = ltr559_i2c_read_reg(LTR559_ALS_PS_STATUS, &buffer);
	if (res < 0) {
		APS_ERR("ltr559 read ALS_PS_STATUS error\n");
		return -1;
	}
	APS_LOG("status = %x\n", buffer);

	return 0;
}
#endif

static int ltr559_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf;

	struct i2c_client *client = ltr559_obj->client;

	struct ltr559_priv *obj = ltr559_obj;

	if (!obj || !client) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mdelay(PON_DELAY);

	init_ps_gain = MODE_PS_Gain16;

	APS_LOG("LTR559_PS setgain = %d!\n", init_ps_gain);
	res = ltr559_i2c_write_reg(LTR559_PS_CONTR, init_ps_gain);
	if (res < 0) {
		APS_LOG("ltr559 set ps gain error\n");
		return res;
	}

	res = ltr559_i2c_write_reg(LTR559_ALS_MEAS_RATE, 0x01);
	if (res < 0) {
		APS_LOG("ltr559 set als meas rate error\n");
		return res;
	}

	mdelay(WAKEUP_DELAY);

	res = ltr559_i2c_write_reg(LTR559_PS_LED, 0x0);
	if (res < 0) {
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	}

	res = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 0x01);
	if (res < 0) {
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	}
	ltr559_obj->tp_cg_color = NONE;
	ltr559_obj->lux_threshold = DEF_LUX_THRESHOLD;
	ltr559_obj->als_cal_high = DEF_ALSCAL_HIGH;
	ltr559_obj->als_cal_low = DEF_ALSCAL_LOW;
#ifdef CONFIG_IDME
	get_als_cali();
#endif
	get_ltr_dts_func(client->dev.of_node);
	/* Enable ALS to Full Range at startup */
	if (ltr559_obj->lux_formula_ver == 1) {
		if (ltr559_obj->tp_cg_color == WHITE)
			als_gainrange = ALS_RANGE_600;
		else
			als_gainrange = ALS_RANGE_1300;
	} else {
		als_gainrange = ALS_RANGE_1300;
	}

	init_als_gain = als_gainrange;

	switch (init_als_gain) {
	case ALS_RANGE_64K:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range1);
		break;

	case ALS_RANGE_32K:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range2);
		break;

	case ALS_RANGE_16K:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range3);
		break;

	case ALS_RANGE_8K:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range4);
		break;

	case ALS_RANGE_1300:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range5);
		break;

	case ALS_RANGE_600:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range6);
		break;

	default:
		res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range1);
		APS_ERR("proxmy sensor gainrange %d!\n", init_als_gain);
		break;
	}

	/*for interrup work mode support */
	if (0 == obj->hw->polling_mode_ps) {
		APS_LOG("eint enable");
#ifdef SUPPORT_PSENSOR
		ltr559_ps_set_thres();
#endif
		databuf = 0x01;
		res = ltr559_i2c_write_reg(LTR559_INTERRUPT, databuf);
		if (res < 0) {
			goto EXIT_ERR;
		}

		databuf = 0x20;
		res = ltr559_i2c_write_reg(LTR559_INTERRUPT_PERSIST, databuf);
		if (res < 0) {
			goto EXIT_ERR;
		}
	}

	res = ltr559_check_and_clear_intr(client);
	if (res) {
		APS_ERR("check/clear intr: %d\n", res);
		return res;
	}

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}

static int ltr559_check_device_id(void)
{
	u8 pid = 0;
	int res;

	res = ltr559_i2c_read_reg(LTR559_MANUFACTURER_ID, &pid);
	if (res < 0) {
		APS_ERR("ltr559 read LTR559_MANUFACTURER_ID error\n");
		return ltr559_ERR_I2C;
	}

	if (pid < 0) {
		APS_ERR("read ltr559 id fail!\n");
		return ltr559_ERR_I2C;
	} else if (pid == LTR559_DEVICE_ID) {
		APS_LOG("ltr559 id match success:%#x!\n", pid);
	} else {
		APS_ERR("ltr559 match fail:%#x!\n", pid);
		return ltr559_ERR_IDENTIFICATION;
	}
	return 0;
}


#ifdef SUPPORT_PSENSOR
/*----------------------------------------------------------------------------*/
static int ltr559_get_ps_value(struct ltr559_priv *obj, u16 ps)
{
	int val, invalid = 0;

	static int val_temp = 1;
	if (!obj) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	if ((ps > atomic_read(&obj->ps_thd_val_high))) {
		val = 0;	/*close */
		val_temp = 0;
		intr_flag_value = 1;
	} else if ((ps < atomic_read(&obj->ps_thd_val_low))) {
		val = 1;	/*far away */
		val_temp = 1;
		intr_flag_value = 0;
	} else
		val = val_temp;

	if (atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if (1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if (time_after(jiffies, endt)) {
			atomic_set(&obj->ps_deb_on, 0);
		}

		if (1 == atomic_read(&obj->ps_deb_on)) {
			invalid = 1;
		}
	} else if (obj->als > 50000) {
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;	/*far away */
	}

	if (!invalid) {
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	} else {
		return -1;
	}
}
#endif
/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("als enable value = %d\n", en);

	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	mutex_unlock(&ltr559_mutex);
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_als_enable(ltr559_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs,
                     int64_t maxBatchReportLatencyNs)
{
        return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
        return als_flush_report();
}

static int als_get_data(int *value, int *status)
{
	int err = 0;

	struct ltr559_priv *obj = NULL;
	int cali_lux = 0;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}

	obj = ltr559_obj;
	err = ltr559_als_read(obj->client, &obj->als);
	if (err) {
		APS_ERR("ltr559_als_read error!\n");
		return -1;
	}
	/*
	 * There is some little difference between factory light source and RD cwf light,
	 * we add different cwf_coeff(black or white) for light source compensation.
	 * If no need,just set cwf_coeff to default value 100 in dts.
	 */
	if (ltr559_obj->lux_formula_ver == 1) {
		cali_lux = ltr_alscode2lux(obj->als) * ltr559_obj->cwf_coeff / 100;
	} else {
		cali_lux = ltr_alscode2lux(obj->als);
	}
	*value = cali_lux;
	ltr559_obj->lux_report = cali_lux;
	if (atomic_read(&ltr559_obj->trace) & LTR_TRC_RAWDATA)
		APS_LOG("als after cali value: %d\n", *value);
	if (*value < 0)
		err = -1;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
#ifdef SUPPORT_PSENSOR
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ltr559_obj als enable value = %d\n", en);

	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr559_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr559_obj->enable);

	mutex_unlock(&ltr559_mutex);
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_ps_enable(ltr559_obj->client, en);

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs,
                    int64_t maxBatchReportLatencyNs)
{
        return 0;
}

static int ps_flush(void)
{
        return ps_flush_report();
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}

	err = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	if (err)
		err = -1;
	else {
		*value = ltr559_get_ps_value(ltr559_obj, ltr559_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
/*for interrup work mode support */
#ifdef SUPPORT_PSENSOR
static void ltr559_eint_work(struct work_struct *work)
{
	struct ltr559_priv *obj =
		(struct ltr559_priv *)container_of(work, struct ltr559_priv, eint_work);
	int err;
	u8 databuf;
	int res = 0;
	int value = 1;
	int i;
	int reg[] = { 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
		      0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91,
		      0x92, 0x93, 0x94, 0x95, 0x97, 0x98, 0x99, 0x9a, 0x9e
	};

	APS_FUN();

	mutex_lock(&ltr559_workqueue_mutex);

	err = ltr559_check_intr(obj->client);
	if (err < 0) {
		APS_ERR("ltr559_eint_work check intrs: %d\n", err);
	} else {
		/* get raw data */
		ltr559_ps_read(obj->client, &obj->ps);
		if (obj->ps < 0) {
			err = -1;
			goto REPORT;
			return;
		}

		APS_DBG("ltr559_eint_work rawdata ps=%d als_ch0=%d!\n", obj->ps,
			obj->als);
		value = ltr559_get_ps_value(obj, obj->ps);
		APS_DBG("intr_flag_value=%d\n", intr_flag_value);
		if (intr_flag_value) {
			APS_DBG(" interrupt value ps will < ps_threshold_low");

			databuf = (u8) ((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_0, databuf);
			if (res < 0) {
				goto REPORT;
			}

			databuf = (u8) (((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_1, databuf);
			if (res < 0) {
				goto REPORT;
			}

			databuf = (u8) (0x00FF);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_0, databuf);
			if (res < 0) {
				goto REPORT;
			}

			databuf = (u8) ((0xFF00) >> 8);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_1, databuf);
			if (res < 0) {
				goto REPORT;
			}
		} else {
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
			if (obj->ps < (last_min_value - 100)) {
				last_min_value = obj->ps;
				APS_DBG(" last_min_value is %d,noise is %d\n",
					last_min_value, obj->ps);
				if (obj->ps < 50) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 50);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 20);
				} else if (obj->ps < 100) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 60);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 30);
				} else if (obj->ps < 200) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 90);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 60);
				} else if (obj->ps < 300) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 130);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 90);
				} else if (obj->ps < 400) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 150);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 120);
				} else if (obj->ps < 600) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 220);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 180);
				} else if (obj->ps < 800) {
					atomic_set(&obj->ps_thd_val_high,
						   obj->ps + 280);
					atomic_set(&obj->ps_thd_val_low,
						   obj->ps + 240);
				} else {
					atomic_set(&obj->ps_thd_val_high, 1000);
					atomic_set(&obj->ps_thd_val_low, 880);
				}
			}
#endif
			APS_DBG(" interrupt value ps will > ps_threshold_high\n");
			databuf = (u8) (0 & 0x00FF);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_0, databuf);
			if (res < 0) {
				goto REPORT;
			}

			databuf = (u8) ((0 & 0xFF00) >> 8);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_LOW_1, databuf);
			if (res < 0) {
				goto REPORT;
			}

			databuf = (u8) ((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_0, databuf);
			if (res < 0) {
				goto REPORT;
			}


			databuf = (u8) (((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
			res = ltr559_i2c_write_reg(LTR559_PS_THRES_UP_1, databuf);
			if (res < 0) {
				goto REPORT;
			}
		}
	}
	ltr559_clear_intr(obj->client);
REPORT:
	enable_irq(ltr559_obj->irq);
	mutex_unlock(&ltr559_workqueue_mutex);
	ps_report_interrupt_data(value);
}
#endif

#ifdef CONFIG_IDME
/*
 * notice: The EVT and later device will calibrate with two point,400lux&20lux in factory.
 *         The new alscal saved format will be lux not transmitance,so we need transform lux
 *         to the old transmitance if cal_alg_ver is 1.The HVT/PROTO cal_alg_ver is 0.
 *         als_cal * 1100 / 400 can transform new als_cal to old transmitance.
 */
static void get_als_cali(void)
{
	als_cal = idme_get_alscal_value();
	APS_LOG("idme to read:als_cal:%d\n", als_cal);
	ltr559_obj->tp_cg_color = idme_get_tp_cg_color();
	APS_LOG("tp_cg_color: %d\n", ltr559_obj->tp_cg_color);
}
#endif

static void get_ltr_dts_func(struct device_node *node)
{
	int err;
	u32 cal_alg_ver = 0;
	u32 lux_formula_ver = 0;
	u32 cwf_coeff[2] = { 0 };

	/* get the cal algo version to select old or one/two point algo */
	err = of_property_read_u32_array(node,
				"cal_alg_ver", &cal_alg_ver, 1);
	if (err == 0)
		ltr559_obj->cal_alg_ver = cal_alg_ver;
	else
		ltr559_obj->cal_alg_ver = 0;
	APS_LOG("cal algo version: %d\n", ltr559_obj->cal_alg_ver);

	/* get the lux formula version compatible different lux formula */
	err = of_property_read_u32_array(node,
				"lux_formula_ver", &lux_formula_ver, 1);
	if (err == 0)
		ltr559_obj->lux_formula_ver = lux_formula_ver;
	else
		ltr559_obj->lux_formula_ver = 0;
	APS_LOG("lux formula version: %d\n", ltr559_obj->lux_formula_ver);

	/* get cwf coeff for black&white tp */
	err = of_property_read_u32_array(node,
				"cwf_coeff", cwf_coeff, 2);
	if (err == 0) {
		if (ltr559_obj->tp_cg_color == BLACK)
			ltr559_obj->cwf_coeff = cwf_coeff[0];
		else if (ltr559_obj->tp_cg_color == WHITE)
			ltr559_obj->cwf_coeff = cwf_coeff[1];
		else
			ltr559_obj->cwf_coeff = 100;
	} else {
		/* set default value */
		ltr559_obj->cwf_coeff = 100;
	}
	APS_LOG("cwf coeff: %d %d %d\n", ltr559_obj->cwf_coeff, cwf_coeff[0], cwf_coeff[1]);

	switch (ltr559_obj->cal_alg_ver) {
	case 0:
		if (als_cal != 0)
			ltr559_obj->als_cal_high = als_cal & 0x0000FFFF;
		else
			ltr559_obj->als_cal_high = DEF_ALSCAL_HIGH;
		break;
	case 1:
		if (als_cal != 0)
			ltr559_obj->als_cal_high = (als_cal & 0x0000FFFF) * 1100 / DEF_LUX_THRESHOLD;
		else
			ltr559_obj->als_cal_high = DEF_ALSCAL_HIGH;
		break;
	default:
		if (als_cal != 0)
			ltr559_obj->als_cal_high = als_cal & 0x0000FFFF;
		else
			ltr559_obj->als_cal_high = DEF_ALSCAL_HIGH;
		break;
	}

	APS_LOG("als_cal_high:%d, cal_alg_ver:%d\n", ltr559_obj->als_cal_high, ltr559_obj->cal_alg_ver);
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
#ifdef CONFIG_PM_SLEEP
static int ltr559_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_priv *obj = i2c_get_clientdata(client);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	alsps_driver_pause_polling(1);
	atomic_set(&obj->als_suspend, 1);
	err = ltr559_als_enable(obj->client, 0);
	if (err < 0) {
		alsps_driver_pause_polling(0);
		atomic_set(&obj->als_suspend, 0);
		APS_ERR("disable als: %d\n", err);
		return err;
	}

#ifdef SUPPORT_PSENSOR
	atomic_set(&obj->ps_suspend, 1);
	err = ltr559_ps_enable(obj->client, 0);
	if (err < 0) {
		alsps_driver_pause_polling(0);
		atomic_set(&obj->ps_suspend, 0);
		APS_ERR("disable ps:  %d\n", err);
		return err;
	}
#endif

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr559_priv *obj = i2c_get_clientdata(client);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	if (alsps_driver_query_polling_state(ID_LIGHT) == 1) {
		err = ltr559_als_enable(obj->client, 1);
		if (err < 0) {
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	atomic_set(&obj->als_suspend, 0);
#ifdef SUPPORT_PSENSOR
	if (alsps_driver_query_polling_state(ID_PROXIMITY) == 1) {
		err = ltr559_ps_enable(obj->client, 1);
		if (err < 0) {
			APS_ERR("enable ps fail: %d\n", err);
		}

	}
	atomic_set(&obj->ps_suspend, 0);
#endif
	alsps_driver_pause_polling(0);

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_detect(struct i2c_client *client,
			     struct i2c_board_info *info)
{
	strncpy(info->type, LTR559_DEV_NAME, sizeof(info->type));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ltr559_priv *obj;
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };
#ifdef SUPPORT_PSENSOR
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
#endif
	int err = 0;

	APS_FUN();

	err = get_alsps_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		return -EFAULT;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	ltr559_obj = obj;

	obj->hw = hw;
#ifdef SUPPORT_PSENSOR
	INIT_WORK(&obj->eint_work, ltr559_eint_work);
#endif
	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_enabled, 0);
	atomic_set(&obj->trace, 0);
#ifdef SUPPORT_PSENSOR
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_suspend, 0);
	atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw->ps_threshold_low);
	atomic_set(&obj->ps_thd_val, obj->hw->ps_threshold);
#endif

	if (ltr559_check_device_id()) {
		err = -ENODEV;
		goto exit_init_failed;
	}

	obj->irq_node = client->dev.of_node;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num =
	    sizeof(obj->hw->als_level) / sizeof(obj->hw->als_level[0]);
	obj->als_value_num =
	    sizeof(obj->hw->als_value) / sizeof(obj->hw->als_value[0]);
	/* (1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value */
	obj->als_modulus = (400 * 100) / (16 * 150);

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	APS_LOG("ltr559_devinit() start...!\n");
	err = ltr559_devinit();
	if (err) {
		goto exit_init_failed;
	}
	APS_LOG("ltr559_devinit() ...OK!\n");

	/* Register sysfs attribute */
	err = ltr559_create_attr(&(ltr559_init_info.platform_diver_addr->driver));
	if (err) {
		APS_ERR("err = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
#ifdef SUPPORT_PSENSOR
	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = ltr559_setup_eint(client);
	if (err != 0) {
		pr_err("setup eint: %d\n", err);
		return err;
	}
#endif

	ltr559_init_flag = 0;
	APS_LOG("ltr559 probe OK\n");

	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_init_failed:
	kfree(obj);
exit:
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr559_init_flag = -1;
	return err;

}

/*----------------------------------------------------------------------------*/

static int ltr559_i2c_remove(struct i2c_client *client)
{
	int err;
	err = ltr559_delete_attr(&ltr559_i2c_driver.driver);
	if (err) {
		APS_ERR("ltr559_delete_attr fail: %d\n", err);
	}

	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr559_remove(void)
{
	APS_FUN();

	i2c_del_driver(&ltr559_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr559_local_init(void)
{
	if (i2c_add_driver(&ltr559_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == ltr559_init_flag)
		return -1;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init ltr559_init(void)
{
	APS_FUN();

	alsps_driver_add(&ltr559_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit ltr559_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(ltr559_init);
module_exit(ltr559_exit);
/*----------------------------------------------------------------------------*/

MODULE_AUTHOR("MingHsien Hsieh");
MODULE_DESCRIPTION("LTR-559ALS Driver");
MODULE_LICENSE("GPL v2");
