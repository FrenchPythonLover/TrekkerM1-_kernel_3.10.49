/* drivers/misc/akm09916.c - akm09916 compass driver
 *
 * Copyright (c) 2014-2015, Linux Foundation. All rights reserved.
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/productinfo.h>

#define AKM_INPUT_DEVICE_NAME	"compass"
#define AKM_DRDY_TIMEOUT_MS		100
#define AKM_BASE_NUM			10

#define AKM_MIN_ODR				(10 * 1000000)
#define AK09916_REG_WIA1			0x00
#define AK09916_REG_WIA2			0x01

#define AK09916_REG_ST1				0x10
#define AK09916_REG_HXL				0x11
#define AK09916_REG_HXH				0x12
#define AK09916_REG_HYL				0x13
#define AK09916_REG_HYH				0x14
#define AK09916_REG_HZL				0x15
#define AK09916_REG_HZH				0x16
#define AK09916_REG_TMPS			0x17
#define AK09916_REG_ST2				0x18

#define AK09916_REG_CNTL1			0x30
#define AK09916_REG_CNTL2			0x31
#define AK09916_REG_CNTL3			0x32
#define AK09916_REG_TST1			0x33
#define AK09916_REG_TST2			0x34

#define AK09916_MODE_SNG_MEASURE	0x01
#define AK09916_MODE_SELF_TEST		0x10
#define AK09916_MODE_FUSE_ACCESS	0x1F
#define AK09916_MODE_POWERDOWN		0x00

#define AK09916_MODE_CONTINUOUS_10HZ	0x02 /* 10Hz */
#define AK09916_MODE_CONTINUOUS_20HZ	0x04 /* 20Hz */
#define AK09916_MODE_CONTINUOUS_50HZ	0x06 /* 50Hz */
#define AK09916_MODE_CONTINUOUS_100HZ	0x08 /* 100Hz */
#define AK09916_RESET_DATA			0x01

#define AK09916_REGS_SIZE		13
#define AK09916_WIA1_VALUE		0x48
#define AK09916_WIA2_VALUE		0x09

/*** Limit of factory shipment test *******************************************/


/* To avoid device dependency, convert to general name */
#define AKM_I2C_NAME			"akm09916"
#define AKM_SYSCLS_NAME			"compass"

#define AKM_REG_MODE			AK09916_REG_CNTL2
#define AKM_REG_RESET			AK09916_REG_CNTL3
#define AKM_REG_STATUS			AK09916_REG_ST1

#define AKM_SENSOR_INFO_SIZE	2
#define AKM_SENSOR_DATA_SIZE	9

#define AKM_MODE_SNG_MEASURE	AK09916_MODE_SNG_MEASURE
#define AKM_MODE_SELF_TEST		AK09916_MODE_SELF_TEST
#define AKM_MODE_FUSE_ACCESS	AK09916_MODE_FUSE_ACCESS
#define AKM_MODE_POWERDOWN		AK09916_MODE_POWERDOWN
#define AKM_MODE_CONTINUOUS_10HZ	AK09916_MODE_CONTINUOUS_10HZ
#define AKM_MODE_CONTINUOUS_20HZ	AK09916_MODE_CONTINUOUS_20HZ
#define AKM_MODE_CONTINUOUS_50HZ	AK09916_MODE_CONTINUOUS_50HZ
#define AKM_MODE_CONTINUOUS_100HZ	AK09916_MODE_CONTINUOUS_100HZ
#define AKM_RESET_DATA			AK09916_RESET_DATA

#define MAG_DATA_FLAG		1
#define AKM_NUM_SENSORS		3

struct akm09916_platform_data {
	char layout;
};

#define AKM_IS_MAG_DATA_ENABLED() (akm->enable_flag & (1 << MAG_DATA_FLAG))

/* POWER SUPPLY VOLTAGE RANGE */
#define AKM09911_VDD_MIN_UV	2000000
#define AKM09911_VDD_MAX_UV	3300000
#define AKM09911_VIO_MIN_UV	1750000
#define AKM09911_VIO_MAX_UV	1950000

#define STATUS_ERROR(st)		(((st)&0x08) != 0x0)

struct akm_compass_data {
	struct i2c_client *i2c;
	struct input_dev *input;
	struct sensors_classdev	cdev;

	struct delayed_work	dwork;
	struct workqueue_struct	*work_queue;
	struct mutex op_mutex;

	/* These two buffers are initialized at start up.
	   After that, the value is not changed */
	uint8_t sense_info[AKM_SENSOR_INFO_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[AKM_SENSOR_DATA_SIZE];

	uint32_t enable_flag;
	int64_t delay[AKM_NUM_SENSORS];

	char layout;
	bool power_enabled;

	int	last_x;
	int	last_y;
	int	last_z;
	int	last_st2;

	struct regulator *vdd;
	struct regulator *vio;
	struct hrtimer	poll_timer;
};

static struct sensors_classdev sensors_cdev = {
	.name = "compass",
	.vendor = "akm",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "4900.0",
	.resolution = "0.15",
	.sensor_power = "0.35",
	.min_delay = 10000,
	.max_delay = 1500,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = SENSOR_FLAG_CONTINUOUS_MODE,
	.enabled = 0,
	.delay_msec = 20,  /*20ms*/
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct akm_compass_data *s_akm;
static uint16_t confuse_i = 0;

/***** I2C I/O function ***********************************************/
static int akm_i2c_rxdata(struct i2c_client *i2c, uint8_t *rxData, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n", __func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int akm_i2c_txdata(struct i2c_client *i2c, uint8_t *txData, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).", __func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

/***** akm miscdevice functions *************************************/
static int AKECS_Set_CNTL(struct akm_compass_data *akm, uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);
	/* Set measure mode */
	buffer[0] = AKM_REG_MODE;
	buffer[1] = mode;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s: Can not set CNTL.", __func__);
	} else {
		dev_dbg(&akm->i2c->dev, "Mode is set to (%d).", mode);
		/* wait at least 100us after changing mode */
		udelay(100);
	}
	mutex_unlock(&akm->sensor_mutex);

	return err;
}

static int AKECS_Reset(struct akm_compass_data *akm, int hard)
{
	uint8_t buffer[2] = {AKM_REG_RESET, AKM_RESET_DATA};
	int err;

	mutex_lock(&akm->sensor_mutex);
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	dev_err(&akm->i2c->dev, "%s(): %s!",
		__func__, (err < 0) ? "failed" : "done");

	udelay(100);
	mutex_unlock(&akm->sensor_mutex);

	return err;
}

static int akm_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct akm_compass_data *akm =
		container_of(sensors_cdev, struct akm_compass_data, cdev);
	int ret = 0;

	akm->enable_flag &= ~(1 << MAG_DATA_FLAG);
	akm->enable_flag |= ((uint32_t)(enable)) << MAG_DATA_FLAG;

	if (enable) {
		AKECS_Set_CNTL(akm, AK09916_MODE_SNG_MEASURE);
		hrtimer_start(&akm->poll_timer,
			ns_to_ktime(akm->delay[MAG_DATA_FLAG]), HRTIMER_MODE_REL);
	} else {
		hrtimer_cancel(&akm->poll_timer);
		AKECS_Set_CNTL(akm, AKM_MODE_POWERDOWN);
	}
	return ret;
}

/***** sysfs delay **************************************************/
static int akm_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct akm_compass_data *akm =
		container_of(sensors_cdev, struct akm_compass_data, cdev);
	akm->delay[MAG_DATA_FLAG] = delay_msec * 1000000;
	return 0;
}

/***** akm input device functions ***********************************/
static int akm_compass_input_init(struct input_dev **input)
{
	int err = 0;

	/* Declare input device */
	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	/* Setup input device */
	set_bit(EV_ABS, (*input)->evbit);

	input_set_abs_params(*input, ABS_X, -11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_Y, -11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_Z, -11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

	(*input)->name = AKM_INPUT_DEVICE_NAME;

	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
	}

	return err;
}

static int akm_compass_suspend(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);

	if (AKM_IS_MAG_DATA_ENABLED()) {
		hrtimer_cancel(&akm->poll_timer);
		if (AKECS_Set_CNTL(akm, AKM_MODE_POWERDOWN) < 0)
			dev_err(&akm->i2c->dev, "Failed to set to POWERDOWN mode.\n");
	}
	dev_dbg(&akm->i2c->dev, "suspended\n");

	return 0;
}

static int akm_compass_resume(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);

	if (AKM_IS_MAG_DATA_ENABLED()) {
		if (AKECS_Set_CNTL(akm, AK09916_MODE_SNG_MEASURE) < 0) {
			dev_err(&akm->i2c->dev, "Failed to set mode\n");
		} else {
			hrtimer_start(&akm->poll_timer,
				ns_to_ktime(akm->delay[MAG_DATA_FLAG]), HRTIMER_MODE_REL);
		}
	}

	dev_dbg(&akm->i2c->dev, "resumed\n");

	return 0;
}

static int akm09916_i2c_check_device(struct i2c_client *client)
{
	/* AK09916 specific function */
	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int err;

	akm->sense_info[0] = AK09916_REG_WIA1;
	err = akm_i2c_rxdata(client, akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	/* Check read data */
	if ((akm->sense_info[0] != AK09916_WIA1_VALUE)
		|| (akm->sense_info[1] != AK09916_WIA2_VALUE)) {
		dev_err(&client->dev,
			"%s: The device is not AKM Compass.", __func__);
		return -ENXIO;
	}

	return err;
}

static int akm_compass_power_set(struct akm_compass_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		dev_warn(&data->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int akm_compass_power_init(struct akm_compass_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				AKM09911_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				AKM09911_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				AKM09911_VDD_MIN_UV, AKM09911_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				AKM09911_VIO_MIN_UV, AKM09911_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AKM09911_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

#ifdef CONFIG_OF
static int akm_compass_parse_dt(struct device *dev,
				struct akm_compass_data *akm)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "akm,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read akm,layout\n");
		return rc;
	} else {
		akm->layout = temp_val;
	}

	return 0;
}
#else
static int akm_compass_parse_dt(struct device *dev,
				struct akm_compass_data *akm)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */

static int akm09916_poll_raw(struct akm_compass_data *akm)
{
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int ret = 0;

	dat_buf[0] = AKM_REG_STATUS;
	ret = akm_i2c_rxdata(akm->i2c, dat_buf, AKM_SENSOR_DATA_SIZE);
	if (0 == ret) {
		if (STATUS_ERROR(dat_buf[8])) {
			dev_err(&akm->i2c->dev, "Status error. Reset...\n");
			AKECS_Reset(akm, 0);
			ret = -EIO;
		} else {
			int mag_x, mag_y, mag_z;
			int tmp;
			
			tmp = (int)((int16_t)(dat_buf[2] << 8) + ((int16_t)dat_buf[1]));
			//mag_x = tmp * 15 / 100;
			mag_x = tmp;
			tmp = (int)((int16_t)(dat_buf[4] << 8) + ((int16_t)dat_buf[3]));
			//mag_y = tmp * 15 / 100;
			mag_y = tmp;
			tmp = (int)((int16_t)(dat_buf[6] << 8) + ((int16_t)dat_buf[5]));
			//mag_z = tmp * 15 / 100;
			mag_z = tmp;

			switch (akm->layout) {
			case 0:
			case 1:
				/* Fall into the default direction */
				break;
			case 2:
				tmp = mag_x;
				mag_x = mag_y;
				mag_y = -tmp;
				break;
			case 3:
				mag_x = -mag_x;
				mag_y = -mag_y;
				break;
			case 4:
				tmp = mag_x;
				mag_x = -mag_y;
				mag_y = tmp;
				break;
			case 5:
				mag_x = -mag_x;
				mag_z = -mag_z;
				break;
			case 6:
				tmp = mag_x;
				mag_x = mag_y;
				mag_y = tmp;
				mag_z = -mag_z;
				break;
			case 7:
				mag_y = -mag_y;
				mag_z = -mag_z;
				break;
			case 8:
				tmp = mag_x;
				mag_x = -mag_y;
				mag_y = -tmp;
				mag_z = -mag_z;
				break;
			}

			//dev_err(&akm->i2c->dev,
			//	"raw_data: %x %x, %x %x, %x %x.\n",
			//		dat_buf[2], dat_buf[1],
			//		dat_buf[4], dat_buf[3],
			//		dat_buf[6], dat_buf[5]);

			akm->last_x = mag_x;
			akm->last_y = mag_y;
			akm->last_z = mag_z;
			akm->last_st2 = dat_buf[8];
		}
	}

	return ret;
}

static void akm09916_report_data(struct akm_compass_data *akm)
{
	ktime_t timestamp = ktime_get_boottime();
	confuse_i = confuse_i + 1;
	input_report_abs(akm->input, ABS_X, (akm->last_x << 16) | confuse_i);
	input_report_abs(akm->input, ABS_Y, (akm->last_y << 16) | confuse_i);
	input_report_abs(akm->input, ABS_Z, (akm->last_z << 16) | confuse_i);
	input_report_abs(akm->input, ABS_MISC, (akm->last_st2 << 16) | confuse_i);
	input_event(akm->input, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(akm->input, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(akm->input);
	//dev_err(&akm->i2c->dev,
	//	"report_data: %x, %x, %x.\n",
	//	akm->last_x, akm->last_y, akm->last_z);
}

static void akm_dev_poll(struct work_struct *work)
{
	struct akm_compass_data *akm =
		container_of((struct delayed_work *)work,
			struct akm_compass_data,  dwork);

	if (akm09916_poll_raw(akm) < 0)
		dev_err(&akm->i2c->dev, "Failed to report data\n");

	if (AKECS_Set_CNTL(akm, AK09916_MODE_SNG_MEASURE) < 0)
		dev_warn(&akm->i2c->dev, "Failed to set mode\n");

	akm09916_report_data(akm);
}

static enum hrtimer_restart akm_timer_func(struct hrtimer *timer)
{
	struct akm_compass_data *akm =
		container_of(timer, struct akm_compass_data, poll_timer);

	queue_work(akm->work_queue, &akm->dwork.work);
	hrtimer_forward_now(&akm->poll_timer,
			ns_to_ktime(akm->delay[MAG_DATA_FLAG]));

	return HRTIMER_RESTART;
}

int akm_compass_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm09916_platform_data *pdata;
	int err = 0;
	int i;

	dev_dbg(&client->dev, "start probing.");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	s_akm = kzalloc(sizeof(struct akm_compass_data), GFP_KERNEL);
	if (!s_akm) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}

	mutex_init(&s_akm->sensor_mutex);
	mutex_init(&s_akm->op_mutex);

	s_akm->enable_flag = 0;

	for (i = 0; i < AKM_NUM_SENSORS; i++)
		s_akm->delay[i] = -1;

	if (client->dev.of_node) {
		err = akm_compass_parse_dt(&client->dev, s_akm);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse platfrom data err=%d\n", err);
			goto exit2;
		}
	} else {
		if (client->dev.platform_data) {
			/* Copy platform data to local. */
			pdata = client->dev.platform_data;
			s_akm->layout = pdata->layout;
		} else {
			/*
			 * Platform data is not available.
			 * Layout and information should be set by each application.
			 */
			s_akm->layout = 0;
			dev_warn(&client->dev, "%s: No platform data.",
				__func__);
		}
	}

	/***** I2C initialization *****/
	s_akm->i2c = client;
	/* set client data */
	i2c_set_clientdata(client, s_akm);

	/* check connection */
	err = akm_compass_power_init(s_akm, 1);
	if (err < 0)
		goto exit2;
	err = akm_compass_power_set(s_akm, 1);
	if (err < 0)
		goto exit3;

	err = akm09916_i2c_check_device(client);
	if (err < 0)
		goto exit4;

	/***** input *****/
	err = akm_compass_input_init(&s_akm->input);
	if (err) {
		dev_err(&client->dev,
			"%s: input_dev register failed", __func__);
		goto exit4;
	}
	input_set_drvdata(s_akm->input, s_akm);

	hrtimer_init(&s_akm->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	s_akm->poll_timer.function = akm_timer_func;
	s_akm->work_queue =
		alloc_workqueue("akm_poll_work",
				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	INIT_WORK(&s_akm->dwork.work, akm_dev_poll);

	s_akm->cdev = sensors_cdev;
	s_akm->cdev.sensors_enable = akm_enable_set;
	s_akm->cdev.sensors_poll_delay = akm_poll_delay_set;

	s_akm->delay[MAG_DATA_FLAG] = sensors_cdev.delay_msec * 1000000;

	err = sensors_classdev_register(&s_akm->input->dev, &s_akm->cdev);
	if (err) {
		dev_err(&client->dev, "class device create failed: %d\n", err);
		goto exit7;
	}
#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
	productinfo_register(PRODUCTINFO_SENSOR_COMPASS_ID,
		"akm09916", "akm");
#endif	/* CONFIG_HISENSE_PRODUCT_DEVINFO */
	dev_info(&client->dev, "successfully probed.");
	return 0;

exit7:
	input_unregister_device(s_akm->input);
exit4:
	akm_compass_power_set(s_akm, 0);
exit3:
	akm_compass_power_init(s_akm, 0);
exit2:
	kfree(s_akm);
exit1:
exit0:
	return err;
}

static int akm_compass_remove(struct i2c_client *client)
{
	struct akm_compass_data *akm = i2c_get_clientdata(client);

	hrtimer_cancel(&akm->poll_timer);
	cancel_work_sync(&akm->dwork.work);
	destroy_workqueue(akm->work_queue);

	if (akm_compass_power_set(akm, 0))
		dev_err(&client->dev, "power set failed.");
	if (akm_compass_power_init(akm, 0))
		dev_err(&client->dev, "power deinit failed.");
	sensors_classdev_unregister(&akm->cdev);

	input_unregister_device(akm->input);
	kfree(akm);
	dev_info(&client->dev, "successfully removed.");
	return 0;
}

static const struct i2c_device_id akm_compass_id[] = {
	{AKM_I2C_NAME, 0 },
	{ }
};

static const struct dev_pm_ops akm_compass_pm_ops = {
	.suspend	= akm_compass_suspend,
	.resume		= akm_compass_resume,
};

static struct of_device_id akm09916_match_table[] = {
	{ .compatible = "ak,ak09916", },
	{ .compatible = "akm,akm09916", },
	{ },
};

static struct i2c_driver akm_compass_driver = {
	.probe		= akm_compass_probe,
	.remove		= akm_compass_remove,
	.id_table	= akm_compass_id,
	.driver = {
		.name	= AKM_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = akm09916_match_table,
		.pm		= &akm_compass_pm_ops,
	},
};

static int __init akm_compass_init(void)
{
	return i2c_add_driver(&akm_compass_driver);
}

static void __exit akm_compass_exit(void)
{
	i2c_del_driver(&akm_compass_driver);
}

module_init(akm_compass_init);
module_exit(akm_compass_exit);

MODULE_AUTHOR("Damon Song <songtao1@hisense.com>");
MODULE_DESCRIPTION("HMCT AK09916 compass driver");
MODULE_LICENSE("GPL");

