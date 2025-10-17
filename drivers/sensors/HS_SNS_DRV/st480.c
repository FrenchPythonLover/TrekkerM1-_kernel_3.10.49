/*
 * Copyright (C) 2015 Senodia.
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
#include <linux/st480.h>
#include <linux/sensors.h>
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
#include <linux/productinfo.h>


#define VENDOR_NAME			"SENODIA"
#define MODULE_NAME			"ST480"

#define BIST_SINGLE_MEASUREMENT_MODE_CMD 0x38
#define ONE_INIT_BIST_TEST 0x01
#define BIST_READ_MEASUREMENT_CMD 0x48

/* POWER SUPPLY VOLTAGE RANGE */
#define ST480_VDD_MIN_UV	2000000
#define ST480_VDD_MAX_UV	3300000
#define ST480_VIO_MIN_UV	1750000
#define ST480_VIO_MAX_UV	1950000



struct st480_data {
	struct device	*class_dev;
	struct class	*compass;
	struct sensors_classdev	cdev;
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	struct delayed_work	dwork;
	struct workqueue_struct	*work_queue;
	struct hrtimer	poll_timer;
	struct mutex	lock;
	atomic_t	enable;
	char	layout;
	int	power_enabled;
	struct regulator	*vdd;
	struct regulator	*vio;
};

static struct sensors_classdev sensors_cdev = {
	.name = "compass",
	.vendor = "Senodia",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "3200",
	.resolution = "0.1",
	.sensor_power = "0.4",
	.min_delay = 40000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct st480_data *st480;

struct mag_3 {
	s16  mag_x,
	mag_y,
	mag_z;
};
volatile static struct mag_3 mag;

/*
 * i2c transfer
 * read/write
 */
static int st480_i2c_transfer_data(struct i2c_client *client,
								int len, char *buf, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  len,
			.buf  =  buf,
		},
		{
			.addr  =  client->addr,
			.flags  = I2C_M_RD,
			.len  =  length,
			.buf  =  buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&client->dev, "%s error:%d\n", __func__, ret);
	return (ret == 2) ? 0 : ret;
}

/*
 * Device detect and init
 *
 */
static int st480_setup(struct i2c_client *client)
{
	int ret;
	unsigned char buf[5];

	memset(buf, 0, 5);

	buf[0] = READ_REGISTER_CMD;
	buf[1] = 0x00;
	ret = 0;

#ifdef IC_CHECK
	while (st480_i2c_transfer_data(client, 2, buf, 3) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 2, buf, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	if (buf[2] != ST480_DEVICE_ID)
		return -ENODEV;
#endif

/*init register step 1*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*init register step 2*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*disable temperature compensation register*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TEMP_DATA_HIGH;
	buf[2] = TEMP_DATA_LOW;
	buf[3] = TEMP_REG;

	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*set calibration register*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*set mode config*/
	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(client, 1, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 1, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	return 0;
}

static void st480_work_func(void)
{
	char buffer[9];
	int ret;
	int tmp;

	memset(buffer, 0, 9);

	buffer[0] = READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 9) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 9) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return;
	}

	if (!((buffer[0]>>4) & 0X01)) {
		if (ST480MB_SIZE_2X2) {
			mag.mag_x = (buffer[3]<<8)|buffer[4];
			mag.mag_y = (buffer[5]<<8)|buffer[6];
			mag.mag_z = (buffer[7]<<8)|buffer[8];
		} else if (ST480MW_SIZE_1_6X1_6) {
			mag.mag_x = (buffer[5]<<8)|buffer[6];
			mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
			mag.mag_z = (buffer[7]<<8)|buffer[8];
		} else if (ST480MC_SIZE_1_2X1_2) {
			mag.mag_x = (buffer[5]<<8)|buffer[6];
			mag.mag_y = (buffer[3]<<8)|buffer[4];
			mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
		}

		switch (st480->layout) {
		case 0:
		case 1:
			/* Fall into the default direction */
			break;
		case 2:
			tmp = mag.mag_x;
			mag.mag_x = mag.mag_y;
			mag.mag_y = -tmp;
			break;
		case 3:
			mag.mag_x = -mag.mag_x;
			mag.mag_y = -mag.mag_y;
			break;
		case 4:
			tmp = mag.mag_x;
			mag.mag_x = -mag.mag_y;
			mag.mag_y = tmp;
			break;
		case 5:
			mag.mag_x = -mag.mag_x;
			mag.mag_z = -mag.mag_z;
			break;
		case 6:
			tmp = mag.mag_x;
			mag.mag_x = mag.mag_y;
			mag.mag_y = tmp;
			mag.mag_z = -mag.mag_z;
			break;
		case 7:
			mag.mag_y = -mag.mag_y;
			mag.mag_z = -mag.mag_z;
			break;
		case 8:
			tmp = mag.mag_x;
			mag.mag_x = -mag.mag_y;
			mag.mag_y = -tmp;
			mag.mag_z = -mag.mag_z;
			break;
		}

		SENODIADBG("st480 raw data: x = %d, y = %d, z = %d \n",
			mag.mag_x, mag.mag_y, mag.mag_z);
	} else
		dev_err(&st480->client->dev, "ecc error detected!\n");

	/*set mode config*/
	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return;
	}
}

static void st480_input_func(struct work_struct *work)
{
	ktime_t timestamp;
	struct st480_data *st480 = container_of((struct delayed_work *)work,
									struct st480_data, dwork);

	SENODIAFUNC("st480_input_func");
	st480_work_func();

	timestamp = ktime_get_boottime();
	input_report_abs(st480->input_dev, ABS_X, mag.mag_x);
	input_report_abs(st480->input_dev, ABS_Y, mag.mag_y);
	input_report_abs(st480->input_dev, ABS_Z, mag.mag_z);
	input_event(st480->input_dev, EV_SYN, SYN_TIME_SEC,
					ktime_to_timespec(timestamp).tv_sec);
	input_event(st480->input_dev, EV_SYN, SYN_TIME_NSEC,
					ktime_to_timespec(timestamp).tv_nsec);
	input_sync(st480->input_dev);

}

static enum hrtimer_restart st480_timer_func(struct hrtimer *timer)
{
	st480 = container_of(timer, struct st480_data, poll_timer);

	queue_work(st480->work_queue, &st480->dwork.work);
	hrtimer_start(&st480->poll_timer,
			ns_to_ktime(ST480_DEFAULT_DELAY), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

#if ST480_AUTO_TEST
static int st480_sensor_test_read(void)

{
		st480_work_func();
		return 0;
}

static int st480_auto_test_read(void *unused)
{
	while (1) {
		st480_sensor_test_read();
		msleep(200);
	}
	return 0;
}
#endif

static int st480_set_enable(struct st480_data *st480, bool on)
{
	int retval = 0;
	struct i2c_client *client = st480->client;

	dev_info(&client->dev, "enable:%s\n", on ? "on" : "off");

	mutex_lock(&st480->lock);
	if (on) {
		hrtimer_start(&st480->poll_timer,
						ns_to_ktime(ST480_DEFAULT_DELAY), HRTIMER_MODE_REL);
		atomic_set(&st480->enable, 1);
	} else {
		hrtimer_cancel(&st480->poll_timer);
		cancel_work_sync(&st480->dwork.work);
		atomic_set(&st480->enable, 0);
	}
	mutex_unlock(&st480->lock);
	return retval;
}

static int st480_cdev_set_enable(struct sensors_classdev *sensors_cdev,
							unsigned int enable)
{
	struct st480_data *st480 =
		container_of(sensors_cdev, struct st480_data, cdev);

	return st480_set_enable(st480, enable);
}

static int st480_cdev_set_poll_delay(struct sensors_classdev *sensors_cdev,
								unsigned int msecs)
{
	return 0;
}

static int st480_single_measure_mode_bist(void)
{
	char buffer[1];
	int ret;

	/*set mode config*/
	buffer[0] = BIST_SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}
	return 0;
}

static ssize_t st480_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static DEVICE_ATTR(vendor, S_IRUGO, st480_vendor_show, NULL);

static ssize_t st480_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODULE_NAME);
}

static DEVICE_ATTR(name, S_IRUGO, st480_name_show, NULL);

static ssize_t st480_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int sf_ret = 0;
	unsigned char buffer[4] = {0, 0, 0, 0};
	s16 zh[3] = {0};

	pr_info("[SENSOR]: %s start\n", __func__);

	mutex_lock(&st480->lock);
	if (atomic_read(&st480->enable) == 1)
		hrtimer_cancel(&st480->poll_timer);

	usleep_range(30000, 30100);

	zh[0] = mag.mag_z;

/*self test*/
	buffer[0] = WRITE_REGISTER_CMD;
	buffer[1] = ONE_INIT_BIST_TEST;
	buffer[2] = ONE_INIT_DATA_LOW;
	buffer[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 4, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 4, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n", __func__);
		}
	}

	usleep_range(150000, 150100);
	st480_single_measure_mode_bist();

	usleep_range(30000, 30100);

	buffer[0] = BIST_READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 3) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n", __func__);
		}
	}

	if (((buffer[0]>>4) & 0X01)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s error\n", __func__);
	}

	zh[1] = (buffer[1]<<8)|buffer[2];

	zh[2] = (abs(zh[1])) - (abs(zh[0]));

	pr_info("[SENSOR]: %s zh[0] = %d, zh[1] = %d, zh[2] = %d\n",
		__func__, zh[0], zh[1], zh[2]);

/*Reduce to the initial setup*/
	buffer[0] = WRITE_REGISTER_CMD;
	buffer[1] = ONE_INIT_DATA_HIGH;
	buffer[2] = ONE_INIT_DATA_LOW;
	buffer[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 4, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 4, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n", __func__);
		}
	}

/*set mode*/
	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0)
			break;

		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n", __func__);
		}
	}

	if ((abs(zh[2]) <= 90) || (abs(zh[2]) >= 180)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s BIST test error\n", __func__);
	}
	if (sf_ret == 0)
		pr_info("[SENSOR]: %s success\n", __func__);
	else
		pr_info("[SENSOR]: %s fail\n", __func__);

	if (atomic_read(&st480->enable) == 1) {
		hrtimer_start(&st480->poll_timer,
						ns_to_ktime(ST480_DEFAULT_DELAY), HRTIMER_MODE_REL);
	}

	mutex_unlock(&st480->lock);
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
			zh[0], zh[1], zh[2]);
}

static DEVICE_ATTR(selftest, S_IRUGO, st480_selftest_show, NULL);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	NULL,
};

static void st480_set_sensor_attr(struct device *dev,
				struct device_attribute *attributes[])
{
		int i;

		for (i = 0; attributes[i] != NULL; i++)
				if ((device_create_file(dev, attributes[i])) < 0)
						pr_err("[SENSOR CORE] fail device_create_file"\
								"(dev, attributes[%d])\n", i);
}

int st480_sensors_register(struct class *compass, struct device *dev,
			void *drvdata, struct device_attribute *attributes[], char *name)
{
		int ret = 0;

		dev = device_create(compass, NULL, 0, drvdata, "%s", name);
		if (IS_ERR(dev)) {
				ret = PTR_ERR(dev);
				pr_err("[SENSORS CORE] device_create failed![%d]\n", ret);
				return ret;
		}

		st480_set_sensor_attr(dev, attributes);

		return ret;
}

void st480_sensors_unregister(struct device *dev,
					struct device_attribute *attributes[])
{
		int i;

		for (i = 0; attributes[i] != NULL; i++)
			device_remove_file(dev, attributes[i]);
}

static int st480_compass_power_init(struct st480_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				ST480_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				ST480_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				ST480_VDD_MIN_UV, ST480_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				ST480_VIO_MIN_UV, ST480_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
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
		regulator_set_voltage(data->vdd, 0, ST480_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}


static int st480_compass_power_set(struct st480_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
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
		dev_warn(&data->client->dev,
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
		dev_warn(&data->client->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int st480_compass_suspend(struct device *dev)
{
	struct st480_data *st480 = dev_get_drvdata(dev);
	printk("%s\n", __func__);

	if (atomic_read(&st480->enable) == 1){
		dev_err(&st480->client->dev, "st480_hrtimer_cancel\n");
		hrtimer_cancel(&st480->poll_timer);
	}
	return 0;
}

static int st480_compass_resume(struct device *dev)
{
	struct st480_data *st480 = dev_get_drvdata(dev);
	printk("%s\n", __func__);

	if (atomic_read(&st480->enable) == 1){
		dev_err(&st480->client->dev, "st480_hrtimer_start\n");
		hrtimer_start(&st480->poll_timer,
						ns_to_ktime(ST480_DEFAULT_DELAY), HRTIMER_MODE_REL);
	}
	return 0;
}


static int st480_input_init(struct st480_data *st480)
{
	int ret = 0;

	st480->input_dev = input_allocate_device();
	if (!st480->input_dev) {
		dev_err(&st480->client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	/* Setup input device */
	set_bit(EV_ABS, st480->input_dev->evbit);
	/* x-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(st480->input_dev, ABS_X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* y-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(st480->input_dev, ABS_Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* z-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(st480->input_dev, ABS_Z, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* Set name */
	st480->input_dev->name = "compass";
	st480->input_dev->id.bustype = BUS_I2C;

	ret = input_register_device(st480->input_dev);
	if (ret < 0) {
		dev_err(&st480->client->dev, "Unable to register input device\n");
		goto err_free_input_device;
	}

	return ret;

err_free_input_device:
	input_free_device(st480->input_dev);
	return ret;
}

static int st480_compass_parse_dt(struct device *dev,
				struct st480_data *st480)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "senodia,layout", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read senodia,layout\n");
		return rc;
	} else {
		st480->layout = temp_val;
	}
	return 0;
}

static int st480_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
#if ST480_AUTO_TEST
	struct task_struct *thread;
#endif

	SENODIAFUNC("st480_probe");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "SENODIA st480_probe: check_functionality failed.\n");
		err = -ENODEV;
		goto exit1;
	}

	/* Allocate memory for driver data */
	st480 = kzalloc(sizeof(struct st480_data), GFP_KERNEL);
	if (!st480) {
		printk(KERN_ERR "SENODIA st480_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit1;
	}

	st480->client = client;
	i2c_set_clientdata(client, st480);

	if (client->dev.of_node) {
		err = st480_compass_parse_dt(&client->dev, st480);
		if (err) {
			dev_err(&client->dev,
				"SENODIA st480_probe: Unable to parse platfrom data.\n");
			goto exit2;
		}
	} else
		goto exit2;
	err = st480_compass_power_init(st480, 1);
	if (err) {
			dev_err(&client->dev,
				"SENODIA st480_probe: power_init failed.\n");
			goto exit2;
		}
	err = st480_compass_power_set(st480, 1);
	if (err) {
			dev_err(&client->dev,
				"SENODIA st480_probe: power_set failed.\n");
			goto exit3;
		}

	client->addr = 0x0c;
	err = st480_setup(st480->client);
	if (err) {
		dev_err(&client->dev, "SENODIA st480_probe:st480 setup error.\n");
		goto exit4;
	}

	err = st480_input_init(st480);
	if (err) {
		dev_err(&client->dev, "SENODIA st480_probe: st480 input init failed\n");
		goto exit4;
	}

	st480->work_queue = create_singlethread_workqueue("st480_poll_work");
	if (!st480->work_queue) {
		err = -ENOMEM;
		dev_err(&client->dev, "SENODIA st480_probe: st480 creat workqueue failed\n");
		goto exit5;
	}
	INIT_WORK(&st480->dwork.work, st480_input_func);

	hrtimer_init(&st480->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	st480->poll_timer.function = st480_timer_func;

	mutex_init(&st480->lock);

	st480->cdev = sensors_cdev;
	st480->cdev.sensors_enable = st480_cdev_set_enable;
	st480->cdev.sensors_poll_delay = st480_cdev_set_poll_delay;

	err = sensors_classdev_register(&st480->input_dev->dev, &st480->cdev);
	if (err) {
		dev_err(&client->dev, "sensors class register failed!\n");
		goto exit5;
	}

	st480->compass = class_create(THIS_MODULE, "compass");
	if (IS_ERR(st480->compass)) {
		pr_err("%s, create compass is failed.(err=%ld)\n",
				__func__, IS_ERR(st480->compass));
		goto exit6;
	}

	err = st480_sensors_register(st480->compass, st480->class_dev, st480,
				sensor_attrs, MODULE_NAME);
	if (err) {
		pr_err("%s, failed to st480_sensors_register (%d)\n",
			__func__, err);
		goto exit7;
	}

#if ST480_AUTO_TEST
	thread = kthread_run(st480_auto_test_read, NULL, "st480_read_test");
#endif
#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
		productinfo_register(PRODUCTINFO_SENSOR_COMPASS_ID,
			"st480", "Senodia");
#endif	/* CONFIG_HISENSE_PRODUCT_DEVINFO */
	printk("st480 probe done.");
	return 0;

exit7:
	class_destroy(st480->compass);
exit6:
	sensors_classdev_unregister(&st480->cdev);
exit5:
	input_unregister_device(st480->input_dev);
exit4:
	st480_compass_power_set(st480, 0);
exit3:
	st480_compass_power_init(st480, 0);
exit2:
	kfree(st480);
exit1:
	return err;

}

static int st480_remove(struct i2c_client *client)
{
	struct st480_data *st480 = i2c_get_clientdata(client);

	hrtimer_cancel(&st480->poll_timer);
	cancel_work_sync(&st480->dwork.work);
	destroy_workqueue(st480->work_queue);

	if (st480_compass_power_set(st480, 0))
		dev_err(&client->dev, "power set failed.");
	if (st480_compass_power_init(st480, 0))
		dev_err(&client->dev, "power deinit failed.");

	sensors_classdev_unregister(&st480->cdev);
	input_unregister_device(st480->input_dev);
	i2c_set_clientdata(client, NULL);
	st480_sensors_unregister(st480->class_dev, sensor_attrs);
	class_destroy(st480->compass);
	kfree(st480);
	return 0;
}

static const struct i2c_device_id st480_id_table[] = {
	{ ST480_I2C_NAME, 0 },
	{ },
};

static const struct dev_pm_ops st480_compass_pm_ops = {
	.suspend	= st480_compass_suspend,
	.resume		= st480_compass_resume,
};

static struct of_device_id st480_match_table[] = {
	{ .compatible = "senodia,st480", },
	{ },
};

static struct i2c_driver st480_driver = {
	.probe		= st480_probe,
	.remove		= st480_remove,
	.id_table	= st480_id_table,
	.driver = {
		.name = ST480_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = st480_match_table,
		.pm		= &st480_compass_pm_ops,
	},
};

static int __init st480_init(void)
{

	return i2c_add_driver(&st480_driver);
}

static void __exit st480_exit(void)
{

	i2c_del_driver(&st480_driver);
}

module_init(st480_init);
module_exit(st480_exit);

MODULE_AUTHOR("Zhang Lanpeng <zhanglanpeng@hisense.com>");
MODULE_DESCRIPTION("senodia st480 linux driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0.0");
