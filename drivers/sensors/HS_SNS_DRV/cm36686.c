/* drivers/input/misc/CM36686.c - CM36686 optical sensors driver
 *
 * Copyright (C) 2012 Capella Microsystems Inc.
 * Author: Frank Hsieh <pengyueh@gmail.com>
 *
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/of_gpio.h>
#include <linux/cm36686.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/sensors.h>
#include <linux/productinfo.h>

/*#define Android5_0*/

static const int ps_duty_array[] = {
	CM36686_PS_DR_1_40,
	CM36686_PS_DR_1_80,
	CM36686_PS_DR_1_160,
	CM36686_PS_DR_1_320
};

static const int ps_it_array[] = {
	CM36686_PS_IT_1T,
	CM36686_PS_IT_1_5T,
	CM36686_PS_IT_2T,
	CM36686_PS_IT_2_5T,
	CM36686_PS_IT_3T,
	CM36686_PS_IT_3_5T,
	CM36686_PS_IT_4T,
	CM36686_PS_IT_8T
};

static const int ps_led_drv_array[] = {
	CM36686_PS_LED_I_50,
	CM36686_PS_LED_I_75,
	CM36686_PS_LED_I_100,
	CM36686_PS_LED_I_120,
	CM36686_PS_LED_I_140,
	CM36686_PS_LED_I_160,
	CM36686_PS_LED_I_180,
	CM36686_PS_LED_I_200
};

static const int als_it_array[] = {
	CM36686_ALS_IT_80,
	CM36686_ALS_IT_160,
	CM36686_ALS_IT_320,
	CM36686_ALS_IT_640
};

struct crosstalk {
	int ten;
	int hundred;
};

/* cm36686 polling rate in ms */
#define CM36686_LS_MIN_POLL_DELAY	1
#define CM36686_LS_MAX_POLL_DELAY	1000


struct CM36686_info {
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct delayed_work als_dwork;
	int als_enable;
	bool als_function_on;
	int als_fittness;
	int als_range;
	uint16_t als_conf;
	struct input_dev *ps_input_dev;
	struct delayed_work ps_dwork;
	struct mutex dw_mlock;
	bool ps_function_on;
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;
	int ps_duty;
	int ps_it;
	int ps_led_drv;
	uint16_t ps_conf12;
	uint16_t ps_conf3;
	uint16_t last_ps_canc;
	uint16_t ps_crosstalk_min;
	uint16_t ps_crosstalk_max;
	struct crosstalk ps_crosstalk;
	struct work_struct ps_poll_dwork;
	struct wake_lock ps_wake_lock;
	atomic_t ls_poll_delay;
	int irq;
	int intr_pin;
	struct regulator *vdd;
	struct regulator *vio;
	bool power_on;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

};

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "capella",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
#ifdef CONFIG_SMALL_HOLE_CM36686S
	.max_range = "200000",
#else
	.max_range = "5240",
#endif
	.resolution = "0.08",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE,
	.enabled = 0,
	.delay_msec = 500,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "capella",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 1,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE | SENSOR_FLAG_WAKE_UP,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct workqueue_struct *cm36686_workqueue;

static void psensor_set_duty(struct CM36686_info *lpi)
{
	lpi->ps_conf12 &= CM36686_PS_DR_MASK;
	lpi->ps_conf12 |= ps_duty_array[lpi->ps_duty];
}

static void psensor_set_it(struct CM36686_info *lpi)
{
	lpi->ps_conf12 &= CM36686_PS_IT_MASK;
	lpi->ps_conf12 |= ps_it_array[lpi->ps_it];
}

static void psensor_set_led_drv(struct CM36686_info *lpi)
{
	lpi->ps_conf3 &= CM36686_PS_LED_I_MASK;
	lpi->ps_conf3 |= ps_led_drv_array[lpi->ps_led_drv];
}

static void psensor_report_dist(struct CM36686_info *lpi, int dist_code)
{
	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, dist_code ? 1000 : 1);
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, dist_code ? 1023 : 0);
	input_event(lpi->ps_input_dev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(lpi->ps_input_dev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(lpi->ps_input_dev);
	dev_err(&lpi->i2c_client->dev,
		"%s(): dist_code=%d\n", __func__, dist_code);
}

static int cm36686_cross_talk_calibration(struct i2c_client *client)
{
	struct CM36686_info *lpi = i2c_get_clientdata(client);
	unsigned int test_pdata;
	int ret;
	/* if the crosstalk change small we should clean it and recalibrate*/
	ret = i2c_smbus_write_word_data(client, REG_PS_CANC, 0);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CANC error 0000!\n", __func__);
		return ret;
	}
	lpi->ps_crosstalk.hundred = 0;
	lpi->ps_crosstalk.ten = 0;

	/*ps power on bit set*/
	lpi->ps_conf12 &= CM36686_PS_EN;
	ret = i2c_smbus_write_word_data(client, REG_PS_CONF12, lpi->ps_conf12);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CONF12 err\n", __func__);
		return ret;
	}

	msleep(30);
	test_pdata = i2c_smbus_read_word_data(client, REG_PS_DATA);
	if (test_pdata <= lpi->ps_crosstalk_max) {
		if (lpi->ps_crosstalk_min) {/*cal crosstalk to < 10*/
			if ((test_pdata/10) > 0) {/* do cal */
				lpi->ps_crosstalk.ten = (test_pdata/10);
				lpi->last_ps_canc = (lpi->ps_crosstalk.ten)*10;
				ret = i2c_smbus_write_word_data(client, REG_PS_CANC,
							(lpi->ps_crosstalk.ten)*10);
				if (ret < 0) {
					dev_err(&lpi->i2c_client->dev,
						"%s(): write PS_CANC error 1111!\n", __func__);
					return ret;
				}
			} else {
			/* the crosstalk is good and do not cal  but should update the last_ps_canc*/
				lpi->last_ps_canc = 0;
			}
		} else {/*cal crosstalk to 0*/
			lpi->last_ps_canc = test_pdata;
			ret = i2c_smbus_write_word_data(client, REG_PS_CANC,
									test_pdata);
			if (ret < 0) {
				dev_err(&lpi->i2c_client->dev,
						"%s(): write PS_CANC error 2222!\n", __func__);
				return ret;
			}
		}
		lpi->ps_conf12 |= CM36686_PS_DIS;
		ret = i2c_smbus_write_word_data(client, REG_PS_CONF12,
								lpi->ps_conf12);
		if (ret < 0) {
			dev_err(&lpi->i2c_client->dev,
				"write PS_CONF12 err in function cross_talk_calibration\n");
			return ret;
		}
		return 0;
	} else {/* crosstalk is too large*/
		/* we do not calibrate but use the last_ps_canc*/
		dev_err(&lpi->i2c_client->dev, "%s(): corsstalk is %d it's too large !\n",
				__func__,
				test_pdata);
		/* not the first time use prox after power on and "last_ps_canc"
		had been updated last time*/
		if (0xffff != lpi->last_ps_canc) {
			ret = i2c_smbus_write_word_data(client, REG_PS_CANC,
				lpi->last_ps_canc);
			if (ret < 0) {
				dev_err(&lpi->i2c_client->dev,
						"%s(): write PS_CANC error 3333!\n",
						__func__);
				return ret;
			}
		}
		/*irst time use prox after power on and crosstalk too large*/

		/*ps power on bit clear */
		lpi->ps_conf12 |= CM36686_PS_DIS;
		ret = i2c_smbus_write_word_data(client,
			REG_PS_CONF12, lpi->ps_conf12);
		if (ret < 0) {
			dev_err(&lpi->i2c_client->dev,
				"%s(): write PS_CONF12 err\n", __func__);
			return ret;
		}
		return 0;
	}
}

static int psensor_function_on(struct CM36686_info *lpi)
{
	int ret = 0;

	/*2. set interrupt threshold value*/
	ret = i2c_smbus_write_word_data(lpi->i2c_client, REG_PS_THDL, 0x0);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_THDL error!\n", __func__);
		return ret;
	}
	ret = i2c_smbus_write_word_data(lpi->i2c_client, REG_PS_THDH, 0xffff);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_THDH error!\n", __func__);
		return ret;
	}
	/*set led current and mode*/
	psensor_set_led_drv(lpi);
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
		REG_PS_CONF3, lpi->ps_conf3);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CONF3 error!\n", __func__);
		return ret;
	}
	psensor_set_duty(lpi);
	psensor_set_it(lpi);
	lpi->ps_conf12 &= CM36686_PS_HD_12BIT;
	lpi->ps_conf12 |= (CM36686_PS_PERS_3 | CM36686_PS_INT_BOTH);

	cm36686_cross_talk_calibration(lpi->i2c_client);
	/*2. set interrupt threshold value*/
	ret = i2c_smbus_write_word_data(lpi->i2c_client, REG_PS_THDL,
		lpi->ps_away_thd_set);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_THDL error!\n", __func__);
		return ret;
	}
	if (0xffff != lpi->last_ps_canc) {
		ret = i2c_smbus_write_word_data(lpi->i2c_client, REG_PS_THDH,
			lpi->ps_close_thd_set);
		if (ret < 0) {
			dev_err(&lpi->i2c_client->dev,
				"%s(): write PS_THDH error!\n", __func__);
			return ret;
		}
	}
	/*4. ps power on bit set*/
	lpi->ps_conf12 &= CM36686_PS_EN;
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
		REG_PS_CONF12, lpi->ps_conf12);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CONF1 error!\n", __func__);
		return ret;
	}
	lpi->ps_function_on = true;

	mdelay(16);
	if (i2c_smbus_read_word_data(lpi->i2c_client, REG_PS_DATA)
		< lpi->ps_close_thd_set) {
		psensor_report_dist(lpi, FAR_CODE);
	}
	return 0;
}

static int psensor_function_off(struct CM36686_info *lpi)
{
	int ret = 0;

	/*1. disable interrupt */
	lpi->ps_conf12 &= CM36686_PS_INT_MASK;

	/*2. ps power on bit unset*/
	lpi->ps_conf12 |= CM36686_PS_DIS;
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
		REG_PS_CONF12, lpi->ps_conf12);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CONF1 error!\n", __func__);
		return ret;
	}

	lpi->ps_function_on = false;
	psensor_report_dist(lpi, FAR_CODE);
	return 0;
}


static void ps_dwork_handler(struct work_struct *work)
{
	struct CM36686_info *lpi =
		container_of((struct delayed_work *)work,
			struct CM36686_info, ps_dwork);
	s32 temp_val = 0;
	u16 int_flag = 0;

	mutex_lock(&lpi->dw_mlock);
	temp_val = i2c_smbus_read_word_data(lpi->i2c_client, REG_INT_FLAG);
	if (temp_val < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s: CM36686 read INT_FLAG error!\n", __func__);
		goto turn_on_lcd;
	}

	int_flag = (u16)(temp_val & 0xFF00);
	dev_err(&lpi->i2c_client->dev, "int_flag = 0x%x\n", int_flag);

	if (int_flag & INT_FLAG_PS_SPFLAG) {
		dev_err(&lpi->i2c_client->dev,
			"%s: Saturation caused the chip turn to protection mode,\
			light on the LCD.\n", __func__);
		goto turn_on_lcd;
	}
	if ((int_flag & INT_FLAG_PS_IF_CLOSE) &&
		(int_flag & INT_FLAG_PS_IF_AWAY)) {
		dev_err(&lpi->i2c_client->dev,
			"%s: PS_NEAR_TO_FAR & PS_FAR_TO_NEAR occurred at same time,\
			error! Light on the LCD.\n", __func__);
		goto turn_on_lcd;
	}
	if (int_flag & INT_FLAG_PS_IF_CLOSE) {
		psensor_report_dist(lpi, NEAR_CODE);
		dev_err(&lpi->i2c_client->dev, "far_to_near!\n");
		goto quit;
	}

	if (int_flag & INT_FLAG_PS_IF_AWAY) {
		dev_err(&lpi->i2c_client->dev, "near_to_far!\n");
		goto turn_on_lcd;
	}
	dev_err(&lpi->i2c_client->dev, "error! Neither PS_FAR_TO_NEAR,\
			nor PS_NEAR_TO_FAR int occurred,\
			but irq triggered.\n");
turn_on_lcd:
	psensor_report_dist(lpi, FAR_CODE);
	if (!wake_lock_active(&lpi->ps_wake_lock)) {
		dev_err(&lpi->i2c_client->dev,
			"wake_lock PROX_NEAR_TO_FAR_WLOCK not be locked,\
			and lock it!\n");
		wake_lock_timeout(&lpi->ps_wake_lock, HZ);
	} else {
		dev_err(&lpi->i2c_client->dev,
			"wake_lock PROX_NEAR_TO_FAR_WLOCK be locked,\
			do nothing!\n");
	}
quit:
	mutex_unlock(&lpi->dw_mlock);
	return;
}

static void ps_poll_dwork_handler(struct work_struct *work)
{
	struct CM36686_info *lpi =
		container_of(work, struct CM36686_info, ps_poll_dwork);
	int ret = 0;

    /*set led current and mode*/
	psensor_set_led_drv(lpi);
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
		REG_PS_CONF3, lpi->ps_conf3);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CONF3 error,prob cali failed!\n", __func__);
		return;
	}
	psensor_set_duty(lpi);
	psensor_set_it(lpi);
	lpi->ps_conf12 &= CM36686_PS_HD_12BIT;

	ret = cm36686_cross_talk_calibration(lpi->i2c_client);
	if (!ret)
		dev_err(&lpi->i2c_client->dev, "cm36686 prob cali success!!\n");
	else
		dev_err(&lpi->i2c_client->dev, "cm36686 prob cali failed!!\n");

	return;
}
static irqreturn_t cm36686_ps_irq_handler(int irq, void *data)
{
	struct CM36686_info *lpi = data;
	dev_err(&lpi->i2c_client->dev, "%s\n", __func__);

	if (!lpi->ps_function_on) {
		dev_err(&lpi->i2c_client->dev,
				"%s(): Chip not functioned, report nothing!\n",
				__func__);
		goto quit;
	}
	cancel_delayed_work(&lpi->ps_dwork);
	queue_delayed_work(cm36686_workqueue, &lpi->ps_dwork, 0);
quit:
	return IRQ_HANDLED;
}

static int cm36686_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct  CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, ps_cdev);

	if ((enable < 0) || (enable > 1)) {
		dev_err(&lpi->i2c_client->dev,
				"%s(): It is an illegal para %d!\n",
				__func__,
				enable);
		return -EINVAL;
	}
	dev_err(&lpi->i2c_client->dev, "%s: ps_en = %d\n", __func__, enable);
	if (enable)
		psensor_function_on(lpi);
	else
		psensor_function_off(lpi);
	/* Force report a FAR value when excute this function.*/
	return 0;
}



static ssize_t ps_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
		struct CM36686_info, ps_cdev);
	struct i2c_client *client = lpi->i2c_client;
	int temp_val;
	mutex_lock(&lpi->dw_mlock);
	temp_val = i2c_smbus_read_word_data(client, REG_PS_DATA);
	mutex_unlock(&lpi->dw_mlock);
	return snprintf(buf, 10 , "%d\n", temp_val);
}
static DEVICE_ATTR(ps_data, 0640, ps_data_show, NULL);

static ssize_t ps_conf1_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
		struct CM36686_info, ps_cdev);
	int temp_val = 0;
	ssize_t ret = 0;
	int i = 0;
	for (i = 0; i < 0x0c; i++) {
		temp_val = i2c_smbus_read_word_data(lpi->i2c_client, i);
		ret += snprintf(buf + ret, 20,
			"REG [%d]: 0x%x\n", i, temp_val);
	}
	return ret;
}

static ssize_t ps_conf1_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
		struct CM36686_info, ps_cdev);
	int code = 0;

	sscanf(buf, "0x%x", &code);

	lpi->ps_conf12 = code;
	if (0 > i2c_smbus_write_word_data(lpi->i2c_client,
		REG_PS_CONF12, lpi->ps_conf12)) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write PS_CONF1 error!\n", __func__);
		return -EIO;
	}
	return count;
}
static DEVICE_ATTR(ps_conf1, 0600, ps_conf1_show, ps_conf1_store);

static ssize_t ps_thd_close_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
		struct CM36686_info, ps_cdev);
	return snprintf(buf, 10, "%d\n",
		lpi->ps_close_thd_set);
}

static ssize_t ps_thd_close_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
		struct CM36686_info, ps_cdev);
	int code;

	sscanf(buf, "%d", &code);
	if ((code > 0) && (code < 65535)) {
		lpi->ps_close_thd_set = (uint16_t)code;
		if (!lpi->ps_function_on) {
			dev_err(&lpi->i2c_client->dev,
				"PS function is DISABLED, do not set thds,\
				but stored it!\n");
			return -EINVAL;
		}
		if (0 > i2c_smbus_write_word_data(lpi->i2c_client, REG_PS_THDH,
			lpi->ps_close_thd_set)) {
			dev_err(&lpi->i2c_client->dev,
				"%s(): write PS_THD error!\n", __func__);
			return -EIO;
		}
	} else {
		dev_err(&lpi->i2c_client->dev,
				"%s(): It is an illegal para, %d!",
				__func__,
				code);
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(ps_thd_close, 0640, ps_thd_close_show, ps_thd_close_store);

static ssize_t ps_thd_away_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, ps_cdev);
	return snprintf(buf, 10, "%d\n",
		lpi->ps_away_thd_set);
}

static ssize_t ps_thd_away_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, ps_cdev);
	int code;
	sscanf(buf, "%d", &code);
	if ((code > 0) && (code < 65535)) {
		lpi->ps_away_thd_set = (uint16_t)code;
		if (!lpi->ps_function_on) {
			dev_err(&lpi->i2c_client->dev,
					"ps function is off can not set the thd\n");
			return -EINVAL;
		}
		if (0 > i2c_smbus_write_word_data(lpi->i2c_client, REG_PS_THDL,
			lpi->ps_away_thd_set)) {
			dev_err(&lpi->i2c_client->dev,
				"%s(): write PS_THD error!\n", __func__);
			return -EIO;
		}
	} else {
		dev_err(&lpi->i2c_client->dev,
			"%s(): It is an illegal para, %d!", __func__, code);
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(ps_thd_away, 0640, ps_thd_away_show, ps_thd_away_store);


static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi =
		container_of(sensors_cdev, struct CM36686_info, ps_cdev);
	return snprintf(buf, 10, "%d\n", lpi->last_ps_canc);
}
static DEVICE_ATTR(ps_canc, 0440, ps_canc_show, NULL);

static ssize_t ps_crosstalk_maxthd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, ps_cdev);
	return snprintf(buf, 10, "%d\n",
		lpi->ps_crosstalk_max);
}
static ssize_t ps_crosstalk_maxthd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, ps_cdev);
	int code;
	sscanf(buf, "%d", &code);
	if ((code > 0) && (code < 65535)) {
		lpi->ps_crosstalk_max = (uint16_t)code;
	} else {
		dev_err(&lpi->i2c_client->dev,
			"%s(): It is an illegal para, %d!", __func__, code);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_crosstalk_maxthd, 0640, ps_crosstalk_maxthd_show,
				ps_crosstalk_maxthd_store);

static int cm36686_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct  CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, ps_cdev);
	struct i2c_client *client = lpi->i2c_client;
	unsigned int test_pdata = 0;
    test_pdata = i2c_smbus_read_word_data(client, REG_PS_DATA);
    if(test_pdata > lpi->ps_close_thd_set) {
        psensor_report_dist(lpi, NEAR_CODE);
    } else {
        psensor_report_dist(lpi, FAR_CODE);
    }
	return 0;
}
static int cm36686_ps_set_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec) {
    return 0;
}
static struct attribute *attributes_ps[] = {
	&dev_attr_ps_thd_close.attr,
	&dev_attr_ps_thd_away.attr,
	&dev_attr_ps_conf1.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_ps_canc.attr,
	&dev_attr_ps_crosstalk_maxthd.attr,
	NULL
};

static struct attribute_group attribute_group_ps = {
	.attrs = attributes_ps
};

static void als_set_it(struct CM36686_info *lpi)
{
	lpi->als_conf &= (CM36686_ALS_IT_MASK);
	lpi->als_conf |= (als_it_array[lpi->als_range]);
}

static int als_function_on(struct CM36686_info *lpi)
{
	s32 ret;
	/*1. set als_it */
	als_set_it(lpi);

	/*2. als interrupt disable*/
	lpi->als_conf &= CM36686_ALS_INT_DIS;

	/*3. als power on bit set*/
	lpi->als_conf &= CM36686_ALS_EN;
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
		REG_ALS_CONF, lpi->als_conf);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write ALS_CONF error!\n", __func__);
		return -EIO;
	}
	lpi->als_function_on = true;
	return 0;
}

static int als_function_off(struct CM36686_info *lpi)
{
	s32 ret;
	lpi->als_conf |= CM36686_ALS_DIS;
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
			REG_ALS_CONF, lpi->als_conf);
	if (0 > ret) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write REG_ALS_CONF error!\n", __func__);
		return -EIO;
	}

	lpi->als_function_on = false;
	return 0;
}

static bool check_als_function(struct CM36686_info *info)
{
	if (!info->power_on) {
		printk(KERN_ERR "chip not powered.\n");
		return false;
	} else if (!info->als_function_on)
		return als_function_on(info) ? false : true;
	else
		return true;
}

#ifdef Android5_0
static char resolution_lut[4][5] = {"0.08", "0.04", "0.02", "0.01"};
static int ls_resolution_get(struct sensors_classdev *sensors_cdev,
						char *resolution_result)
{
	struct  CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, als_cdev);
	u32 temp_val = 0;
	/* Read ALS IT: */
	temp_val = i2c_smbus_read_word_data(lpi->i2c_client, REG_ALS_CONF);
	lpi->als_it_index = (temp_val&0xc0)>>6;
	dev_err(&lpi->i2c_client->dev,
			"%s:als_it=%d\n", __func__, lpi->als_it_index);
	strlcpy(resolution_result, resolution_lut[lpi->als_it_index]);
	dev_err(&lpi->i2c_client->dev,
			("%s:resolution_result=%s\n",
			__func__, resolution_result);
	return 0;
}
#endif
static int ls_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, als_cdev);

	if ((delay_msec < CM36686_LS_MIN_POLL_DELAY) ||
			(delay_msec > CM36686_LS_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&lpi->ls_poll_delay, delay_msec);

	return 0;
}

static void als_dwork_handler(struct work_struct *work)
{
	struct CM36686_info *lpi =
		container_of((struct delayed_work *)work, struct CM36686_info,
					als_dwork);
	u32 adc_value = 0;
	s32 temp_val = 0;
	ktime_t timestamp;

	if (0 == lpi->als_enable) {
		dev_err(&lpi->i2c_client->dev,
				"%s(): als not enabled, do not re-schedule als_dwork!\n",
				__func__);
		return;
	}
	/* Read ALS data: */
	temp_val = i2c_smbus_read_word_data(lpi->i2c_client, REG_ALS_DATA);
	if (temp_val < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s: CM36686 read ALS_DATA error!\n", __func__);
		return;
	}
	/*printk("Raw ALS COUNT: %d\n", (uint)temp_val);*/
	adc_value =
		(temp_val * (1 << (3 - lpi->als_range)) *
		lpi->als_fittness) / 100;
#ifndef CONFIG_SMALL_HOLE_CM36686S
	if (adc_value > 5240)
		adc_value = 5240;
#endif
	/*printk("Final ALS: %d\n", adc_value);*/

	timestamp = ktime_get_boottime();
	input_report_abs(lpi->als_input_dev, ABS_MISC, adc_value);
	input_event(lpi->als_input_dev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(lpi->als_input_dev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(lpi->als_input_dev);
	queue_delayed_work(cm36686_workqueue, &lpi->als_dwork,
		msecs_to_jiffies(CM36686_LS_DEFAULT_POLL_DELAY));
	return;
}

static int cm36686_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct  CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, als_cdev);
	struct input_dev *input_dev = lpi->als_input_dev;
	int ret = 0;
	if ((enable < 0) || (enable > 1)) {
		dev_err(&lpi->i2c_client->dev,
				"%s(): It is an illegal para %d!\n",
				__func__, enable);
		return -EINVAL;
	}
	dev_err(&lpi->i2c_client->dev, "receive als_en is %d\n", enable);
	mutex_lock(&input_dev->mutex);
	if (enable != lpi->als_enable) {
		if (enable) {
			if (!check_als_function(lpi)) {
				dev_err(&lpi->i2c_client->dev,
						"%s(): Chip not functioned, report nothing!\n",
						__func__);
				ret = -EINVAL;
			} else {
				queue_delayed_work(cm36686_workqueue, &lpi->als_dwork,
							msecs_to_jiffies(CM36686_LS_DEFAULT_POLL_DELAY));
				lpi->als_enable = enable;
			}
		} else {
			cancel_delayed_work_sync(&lpi->als_dwork);
			ret = als_function_off(lpi);
			if (0 <= ret)
				lpi->als_enable = enable;
		}
	}
	mutex_unlock(&input_dev->mutex);
	return ret;
}


static ssize_t als_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, als_cdev);
	return snprintf(buf, 20,
		"ALS_CONF = %x\n", lpi->als_conf);
}
static ssize_t als_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct CM36686_info *lpi = container_of(sensors_cdev,
			struct CM36686_info, als_cdev);
	int value = 0;
	int ret;
	sscanf(buf, "0x%x", &value);

	lpi->als_conf = value;
	ret = i2c_smbus_write_word_data(lpi->i2c_client,
		REG_ALS_CONF, lpi->als_conf);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): write ALS_CONF error!\n", __func__);
		return ret;
	}
	return count;
}
static DEVICE_ATTR(als_conf, 0600, als_conf_show, als_conf_store);

static struct attribute *attributes_als[] = {
	&dev_attr_als_conf.attr,
	NULL
};

static struct attribute_group attribute_group_als = {
	.attrs = attributes_als
};

static int als_input_init(struct CM36686_info *lpi)
{
	int ret = 0;

	lpi->als_input_dev = input_allocate_device();
	if (!lpi->als_input_dev) {
		pr_err(
			"[LS][CM36686 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->als_input_dev->name = "light";
	set_bit(EV_ABS, lpi->als_input_dev->evbit);
#ifdef CONFIG_SMALL_HOLE_CM36686S
	input_set_abs_params(lpi->als_input_dev, ABS_MISC, 0, 200000, 0, 0);
#else
	input_set_abs_params(lpi->als_input_dev, ABS_MISC, 0, 5240, 0, 0);
#endif
	input_set_drvdata(lpi->als_input_dev, lpi);

	ret = input_register_device(lpi->als_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_als_input_device;
	}
	return ret;

err_free_als_input_device:
	input_free_device(lpi->als_input_dev);
	return ret;
}

static int ps_input_init(struct CM36686_info *lpi)
{
	int ret = 0;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[PS][CM36686 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1023, 0, 0);
	input_set_drvdata(lpi->ps_input_dev, lpi);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	return ret;

err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

static int ps_irq_init(struct CM36686_info *lpi)
{
	int ret = 0;

	ret = gpio_request(lpi->intr_pin, "gpio_cm36686_intr");
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	ret = request_irq(lpi->irq,
					cm36686_ps_irq_handler,
					IRQF_TRIGGER_FALLING,
					"cm36686",
					lpi);
	if (ret) {
		pr_err("Request IRQ for cm36686 failed,\
			return:%d\n", ret);
		goto fail_free_intr_pin;
	}

	ret = enable_irq_wake(lpi->irq);
	if (0 != ret) {
		pr_err(" enable_irq_wake failed for cm36686_ps_irq_handler\n");
		goto fail_set_irq_wake;
	}

	return ret;
fail_set_irq_wake:
	free_irq(lpi->irq, lpi);
fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}
static int CM36686_parse_dt(struct device *dev,
				struct CM36686_info *lpi)
{
	struct device_node *np = dev->of_node;
	int temp_val = 0;
	int rc = 0;

	rc = of_get_named_gpio_flags(np, "capella,interrupt-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} else {
		lpi->intr_pin = rc;
	}
	rc = of_property_read_u32(np, "capella,ps_duty", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_duty\n");
		return rc;
	} else {
		lpi->ps_duty = (u8)temp_val;
	}
	rc = of_property_read_u32(np, "capella,ps_it", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_it\n");
		return rc;
	} else {
		lpi->ps_it = (u8)temp_val;
	}
	rc = of_property_read_u32(np, "capella,ps_led_drv", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_led_drv\n");
		return rc;
	} else {
		lpi->ps_led_drv = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_close_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd_set\n");
		return rc;
	} else {
		lpi->ps_close_thd_set = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_away_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd_set\n");
		return rc;
	} else {
		lpi->ps_away_thd_set = (uint16_t)temp_val;
	}
	rc = of_property_read_u32(np, "capella,als_range", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read als_range\n");
		return rc;
	} else {
		lpi->als_range = (int)temp_val;
	}
	rc = of_property_read_u32(np, "capella,als_fittness", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read als_fittness\n");
		return rc;
	} else {
		lpi->als_fittness = (int)temp_val;
	}
	rc = of_property_read_u32(np, "capella,ps_crosstalk_min", &temp_val);
	if (rc) {
		lpi->ps_crosstalk_min = PS_CROSSTALK_MIN;
		dev_err(dev, "ps_crosstalk_min do not set and use default value%d\n",
				lpi->ps_crosstalk_min);
	} else {
		lpi->ps_crosstalk_min = (int)temp_val;
	}
	rc = of_property_read_u32(np, "capella,ps_crosstalk_max", &temp_val);
	if (rc) {
		lpi->ps_crosstalk_max = PS_CROSSTALK_MAX;
		dev_err(dev, "ps_crosstalk_max do not set and use default value%d\n",
				lpi->ps_crosstalk_max);
	} else {
		lpi->ps_crosstalk_max = (int)temp_val;
	}
	return 0;
}

static int CM36686_power_init(struct CM36686_info *info, bool on)
{
	int rc;

	if (on) {
		info->vdd = regulator_get(&info->i2c_client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}
		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,
					CM36686_VDD_MIN_UV, CM36686_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}
		info->vio = regulator_get(&info->i2c_client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}
		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				CM36686_VI2C_MIN_UV, CM36686_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}
	} else {
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0, CM36686_VDD_MAX_UV);
		regulator_put(info->vdd);
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0, CM36686_VI2C_MAX_UV);
		regulator_put(info->vio);
	}
	return 0;
err_vio_set_vtg:
	regulator_put(info->vio);
err_vio_get:
	if (regulator_count_voltages(info->vdd) > 0)
		regulator_set_voltage(info->vdd, 0, CM36686_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(info->vdd);
err_vdd_get:
	return rc;
}

static int CM36686_power_switch(struct CM36686_info *info, bool on)
{
	int rc;

	if (on && !info->power_on) {
		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}
		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}
		info->power_on = true;
		msleep(80);
		dev_err(&info->i2c_client->dev,
			"%s(): regulator switch ON.\n", __func__);
	} else if (!on && info->power_on) {
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		info->power_on = false;
		dev_err(&info->i2c_client->dev,
			"%s(): regulator switch OFF.\n", __func__);
	}
	return 0;
err_vio_ena:
	regulator_disable(info->vdd);
err_vdd_ena:
	return rc;
}

static int CM36686_check_chip(struct CM36686_info *lpi)
{
	int temp_val;
	temp_val = i2c_smbus_read_word_data(lpi->i2c_client, REG_ID_REG);
	if (temp_val < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: read ID_REG error!\n",
				__func__);
		return -EIO;
	}
	temp_val &= 0xFF;
	if (temp_val != ID_CM36686) {
		dev_err(&lpi->i2c_client->dev,
			"%s(): Searching for Capella chip failed! temp_val = 0x%x\n",
			__func__, temp_val);
		return -ENODEV;
	}
	dev_err(&lpi->i2c_client->dev,
			"%s(): find Capella CM36686.\n", __func__);
	return 0;
}

static int CM36686_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct CM36686_info *lpi = NULL;
	int ret = 0;
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_I2C | I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}
	lpi = kzalloc(sizeof(struct CM36686_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	/* 1. parse_dt */
	if (client->dev.of_node) {
		ret = CM36686_parse_dt(&client->dev, lpi);
		if (ret) {
			dev_err(&client->dev, "Failed to get lpi from device tree\n");
			goto err_parse_dt;
		}
		/*lpi->ps_close_thd_int=1023;//rhe first int must be faraway;
		lpi->ps_away_thd_int=lpi->ps_away_thd_set;*/
	} else {
		goto err_parse_dt;
	}
	/* 2. power on chip*/
	lpi->power_on = false;
	ret = CM36686_power_init(lpi, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): CM36686 power on error!\n",
				__func__);
		goto err_CM36686_power_on;
	}
	ret = CM36686_power_switch(lpi, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): CM36686 power switch error!\n",
				__func__);
		goto err_CM36686_power_switch;
	}
	/* 3. chip verify */
	ret = CM36686_check_chip(lpi);
	if (ret < 0)
		goto err_chip_verify;

	/* 4. initialize registers and variables */
	lpi->als_conf = 0;
	lpi->ps_conf12 = 0;
	lpi->ps_conf3 = 0;
	lpi->last_ps_canc = 0xffff;/*for first time use prox when power on*/
	atomic_set(&lpi->ls_poll_delay,
			    (unsigned int) CM36686_LS_DEFAULT_POLL_DELAY);
	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);

	/* 5. establish data channel  -- input device*/
	ret = ps_input_init(lpi);
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: ps_input_init error!!\n",
			__func__);
		goto err_ps_input_init;
	}

	ret = als_input_init(lpi);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: als_input_init error!!\n",
			__func__);
		goto err_als_input_init;
	}

	/* Register to sensors class */
	lpi->als_cdev = sensors_light_cdev;
	lpi->als_cdev.sensors_enable = cm36686_als_set_enable;
	lpi->als_cdev.sensors_poll_delay = ls_poll_delay_set;
#ifdef Android5_0
	lpi->als_cdev.sensors_resolution_get = ls_resolution_get;
#endif

	lpi->ps_cdev = sensors_proximity_cdev;
	lpi->ps_cdev.sensors_enable = cm36686_ps_set_enable;
	lpi->ps_cdev.sensors_poll_delay = cm36686_ps_set_delay;
	lpi->ps_cdev.sensors_flush = cm36686_ps_flush;
	ret = sensors_classdev_register(&lpi->ps_input_dev->dev, &lpi->ps_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, ret);
		goto err_unregister_ps_class;
	}

	ret = sensors_classdev_register(&lpi->als_input_dev->dev, &lpi->als_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
			       __func__, ret);
		goto err_unregister_als_class;
	}
	/* 6. establish control channel -- sysfs interface*/
	ret = sysfs_create_group(&((lpi->ps_cdev.dev)->kobj),
							&attribute_group_ps);
	if (ret)
		goto err_create_ps_device_file;
	ret = sysfs_create_group(&((lpi->als_cdev.dev)->kobj),
							&attribute_group_als);
	if (ret)
		goto err_create_als_device_file;

	/* 7. request irq */
	ret = ps_irq_init(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36686 error]%s: ps_irq_init error!\n",
			__func__);
		goto err_ps_irq_init;
	}
	/* 8. wake lock */
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
	INIT_DELAYED_WORK(&lpi->als_dwork, als_dwork_handler);
	INIT_DELAYED_WORK(&lpi->ps_dwork,  ps_dwork_handler);
	INIT_WORK(&lpi->ps_poll_dwork,  ps_poll_dwork_handler);
	mutex_init(&lpi->dw_mlock);

	queue_work(cm36686_workqueue, &lpi->ps_poll_dwork);
#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
	productinfo_register(PRODUCTINFO_SENSOR_ALPS_ID,
		"cm36686", "capella");
#endif	/* CONFIG_HISENSE_PRODUCT_DEVINFO */
	dev_err(&client->dev, "%s(): Probe success!\n", __func__);
	return ret;

err_ps_irq_init:
	sysfs_remove_group(&((lpi->als_cdev.dev)->kobj), &attribute_group_als);
err_create_als_device_file:
	sysfs_remove_group(&((lpi->ps_cdev.dev)->kobj), &attribute_group_ps);
err_create_ps_device_file:
	sensors_classdev_unregister(&lpi->als_cdev);
err_unregister_als_class:
	sensors_classdev_unregister(&lpi->ps_cdev);
err_unregister_ps_class:
	input_unregister_device(lpi->als_input_dev);
err_als_input_init:
	input_unregister_device(lpi->ps_input_dev);
err_ps_input_init:
err_chip_verify:
	CM36686_power_switch(lpi, false);
err_CM36686_power_switch:
	CM36686_power_init(lpi, false);
err_CM36686_power_on:
err_parse_dt:
	kfree(lpi);
	dev_err(&client->dev, "%s(): error exit! ret = %d\n", __func__, ret);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int CM36686_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct CM36686_info *lpi = i2c_get_clientdata(client);

	disable_irq_nosync(lpi->i2c_client->irq);
	if (lpi->als_enable) {
		cancel_delayed_work_sync(&lpi->als_dwork);
		als_function_off(lpi);
	}
	dev_err(&client->dev, "%s().\n", __func__);
	return 0;
}

static int CM36686_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct CM36686_info *lpi = i2c_get_clientdata(client);
	dev_err(&client->dev, "%s()\n", __func__);
	if (lpi->als_enable) {
		if (!check_als_function(lpi)) {
		    dev_err(&client->dev,
				"%s(): Chip not functioned, report nothing!\n",
				__func__);
	    } else
			queue_delayed_work(cm36686_workqueue,
				&lpi->als_dwork,
				msecs_to_jiffies(CM36686_LS_DEFAULT_POLL_DELAY));
	}
	enable_irq(lpi->i2c_client->irq);
	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(CM36686_pm, CM36686_suspend, CM36686_resume, NULL);

static const struct i2c_device_id CM36686_i2c_id[] = {
	{CM36686_I2C_NAME, 0},
	{}
};

static struct of_device_id CM36686_match_table[] = {
	{.compatible = "capella,CM36686",},
	{ },
};

static struct i2c_driver CM36686_driver = {
	.id_table = CM36686_i2c_id,
	.probe = CM36686_probe,
	.driver = {
		.name = CM36686_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &CM36686_pm,
		.of_match_table = CM36686_match_table,
	},
};

static int __init CM36686_init(void)
{
	cm36686_workqueue = create_workqueue("proximity_als_cm36686");
	if (!cm36686_workqueue) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}
	return i2c_add_driver(&CM36686_driver);
}

static void __exit CM36686_exit(void)
{
	if (cm36686_workqueue)
		destroy_workqueue(cm36686_workqueue);
	i2c_del_driver(&CM36686_driver);
}

module_init(CM36686_init);
module_exit(CM36686_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36686 Driver");
MODULE_AUTHOR("Song Tao <songtao1@hisense.com>");
