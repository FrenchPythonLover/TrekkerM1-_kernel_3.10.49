/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtj9.h>
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/productinfo.h>

#define ACCEL_INPUT_DEV_NAME	"accelerometer"
#define DEVICE_NAME		"kxtj9"

#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F		3
#define ODR200F		4
#define ODR400F		5
#define ODR800F		6
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3
/* POWER SUPPLY VOLTAGE RANGE */
#define KXTJ9_VDD_MIN_UV	2000000
#define KXTJ9_VDD_MAX_UV	3300000
#define KXTJ9_VIO_MIN_UV	1750000
#define KXTJ9_VIO_MAX_UV	1950000

#define DEVICE_ID_KXTF9	0X01
#define DEVICE_ID_KXTI9	0X04
#define DEVICE_ID_KXTIK	0X05
#define DEVICE_ID_KXTJ2	0X09

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */

static struct sensors_classdev sensors_cdev = {
	.name = "accelerometer",
	.vendor = "Kionix",
	.version = 1,
	.handle = 0,
	.type = 1,
	.max_range = "19.6",
	.resolution = "0.01",
	.sensor_power = "0.2",
	.min_delay = 10000,	/* microsecond */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,	/* millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
	struct input_dev *input_dev;
	unsigned int last_poll_interval;
	bool	enable;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
	bool	power_enabled;
	struct regulator *vdd;
	struct regulator *vio;
	struct sensors_classdev cdev;
	struct delayed_work	 dwork;
	bool   chip_function_on;
};

static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;
	ktime_t timestamp;

	err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0)
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");

	x = le16_to_cpu(acc_data[tj9->pdata.axis_map_x]);
	y = le16_to_cpu(acc_data[tj9->pdata.axis_map_y]);
	z = le16_to_cpu(acc_data[tj9->pdata.axis_map_z]);

	/* 8 bits output mode support */
	if (!(tj9->ctrl_reg1 & RES_12BIT)) {
		x <<= 4;
		y <<= 4;
		z <<= 4;
	}

	x >>= tj9->shift;
	y >>= tj9->shift;
	z >>= tj9->shift;

	timestamp = ktime_get_boottime();
	input_report_abs(tj9->input_dev, ABS_X, tj9->pdata.negate_x ? -x : x);
	input_report_abs(tj9->input_dev, ABS_Y, tj9->pdata.negate_y ? -y : y);
	input_report_abs(tj9->input_dev, ABS_Z, tj9->pdata.negate_z ? -z : z);
	input_event(tj9->input_dev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(tj9->input_dev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(tj9->input_dev);
}


static int kxtj9_power_on(struct kxtj9_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator vdd enable failed rc=%d\n",
					rc);
			}
		}
		data->power_enabled = false;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
		}
		data->power_enabled = true;
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return rc;
}

static int kxtj9_power_init(struct kxtj9_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, KXTJ9_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, KXTJ9_VIO_MAX_UV);

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
			rc = regulator_set_voltage(data->vdd, KXTJ9_VDD_MIN_UV,
						   KXTJ9_VDD_MAX_UV);
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
			rc = regulator_set_voltage(data->vio, KXTJ9_VIO_MIN_UV,
						   KXTJ9_VIO_MAX_UV);
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
		regulator_set_voltage(data->vdd, 0, KXTJ9_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
	int err = 0;
	if (tj9->pdata.power_on) {
		err = tj9->pdata.power_on();
	} else {
		err = kxtj9_power_on(tj9, true);
		if (err) {
			dev_err(&tj9->client->dev, "power on failed");
			goto err_exit;
		}
		/* Use 80ms as vendor suggested. */
		msleep(80);
	}

err_exit:
	dev_dbg(&tj9->client->dev, "soft power on complete err=%d.\n", err);
	return err;
}

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
	int err;

	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		dev_err(&tj9->client->dev, "soft power off failed\n");

	if (tj9->pdata.power_off)
		tj9->pdata.power_off();
	else
		kxtj9_power_on(tj9, false);

	dev_dbg(&tj9->client->dev, "soft power off complete.\n");
	return ;
}

static int kxtj9_function_on(struct kxtj9_data *tj9)
{
	int err = 0;

	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0) {
		dev_err(&tj9->client->dev, "chip function off failed\n");
		return err;
	}
	msleep(15);
	tj9->chip_function_on = true;
	return 0;
}

static int kxtj9_function_off(struct kxtj9_data *tj9)
{
	int err = 0;
	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0) {
		dev_err(&tj9->client->dev, "chip function off failed\n");
		return err;
	}
	tj9->chip_function_on = false;
	return 0;
}

static bool kxtj9_check_chip_function(struct kxtj9_data *tj9)
{
	if (!tj9->power_enabled)
		return false;
	else if (!tj9->chip_function_on)
		return kxtj9_function_on(tj9) ? false : true;
	else
		return true;
}

static void kxtj9_dwork_handler(struct work_struct *work)
{
	struct kxtj9_data *tj9 =
		container_of((struct delayed_work *)work, struct kxtj9_data, dwork);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	if (tj9->enable) {
		if (kxtj9_check_chip_function(tj9))
			kxtj9_report_acceleration_data(tj9);
		else
			printk(KERN_ERR "Chip not functioned, report nothing!\n");
		schedule_delayed_work(&tj9->dwork,
				msecs_to_jiffies(tj9->last_poll_interval));
		/*printk("last_poll_interval is %d!\n",tj9->last_poll_interval);
		printk("power_enabled is%d!\n",tj9->power_enabled);*/
	} else
		printk(KERN_ERR "%s(): Chip not enabled, do not re-schedule dwork!\n",
			__func__);
	mutex_unlock(&input_dev->mutex);
}


static void kxtj9_init_input_device(struct kxtj9_data *tj9,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = ACCEL_INPUT_DEV_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj9->client->dev;
}

static int kxtj9_setup_input_device(struct kxtj9_data *tj9)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj9->input_dev = input_dev;

	input_set_drvdata(input_dev, tj9);

	kxtj9_init_input_device(tj9, input_dev);

	err = input_register_device(tj9->input_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"unable to register input polled device %s: %d\n",
			tj9->input_dev->name, err);
		input_free_device(tj9->input_dev);
		return err;
	}

	return 0;
}

static int kxtj9_enable_set(struct sensors_classdev *sensors_cdev,
					unsigned int enabled)
{
	struct kxtj9_data *tj9 = container_of(sensors_cdev,
					struct kxtj9_data, cdev);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	if (enabled == 0) {
		printk(KERN_INFO "kxtj9_enable_set disable\n");
		cancel_delayed_work(&tj9->dwork);
		kxtj9_function_off(tj9);
		tj9->enable = false;
	} else if (enabled == 1) {
		printk(KERN_INFO "kxtj9_enable_set enabled\n");
		tj9->enable = true;
		schedule_delayed_work(&tj9->dwork,
			msecs_to_jiffies(tj9->last_poll_interval));
	} else {
		dev_err(&tj9->client->dev,
			"Invalid value of input, input=%d\n", enabled);
		mutex_unlock(&input_dev->mutex);
		return -EINVAL;
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}


/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */
static int kxtj9_poll_delay_set(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec)
{
	struct kxtj9_data *tj9 = container_of(sensors_cdev,
					struct kxtj9_data, cdev);
	struct input_dev *input_dev = tj9->input_dev;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	tj9->last_poll_interval = max(delay_msec, (unsigned int)(tj9->cdev.min_delay / 1000));

	mutex_unlock(&input_dev->mutex);

	return 0;
}


static int kxtj9_verify(struct kxtj9_data *tj9)
{
	int retval;

	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&tj9->client->dev, "read err int source\n");
		goto out;
	}
	printk(KERN_ERR "KIONIX chip id is %d\n", retval);
	retval = (retval != DEVICE_ID_KXTIK && retval != DEVICE_ID_KXTI9
			&& retval != DEVICE_ID_KXTF9 && retval != DEVICE_ID_KXTJ2)
			? -EIO : 0;

out:
	return retval;
}
#ifdef CONFIG_OF
static int kxtj9_parse_dt(struct device *dev,
				struct kxtj9_platform_data *kxtj9_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	/*
	rc = of_property_read_u32(np, "kionix,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		kxtj9_pdata->min_interval = temp_val;
	}
	*/
	rc = of_property_read_u32(np, "kionix,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		kxtj9_pdata->init_interval = temp_val;
	}

	rc = of_property_read_u32(np, "kionix,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		kxtj9_pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kionix,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis_map_y\n");
		return rc;
	} else {
		kxtj9_pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kionix,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map-z\n");
		return rc;
	} else {
		kxtj9_pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kionix,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
		case 2:
			kxtj9_pdata->g_range = KXTJ9_G_2G;
			break;
		case 4:
			kxtj9_pdata->g_range = KXTJ9_G_4G;
			break;
		case 8:
			kxtj9_pdata->g_range = KXTJ9_G_8G;
			break;
		default:
			kxtj9_pdata->g_range = KXTJ9_G_2G;
			break;
		}
	}

	kxtj9_pdata->negate_x = of_property_read_bool(np, "kionix,negate-x");

	kxtj9_pdata->negate_y = of_property_read_bool(np, "kionix,negate-y");

	kxtj9_pdata->negate_z = of_property_read_bool(np, "kionix,negate-z");

	if (of_property_read_bool(np, "kionix,res-12bit"))
		kxtj9_pdata->res_ctl = RES_12BIT;
	else
		kxtj9_pdata->res_ctl = RES_8BIT;

	return 0;
}
#else
static int kxtj9_parse_dt(struct device *dev,
				struct kxtj9_platform_data *kxtj9_pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int kxtj9_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct kxtj9_data *tj9;
	int err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		memset(&tj9->pdata, 0 , sizeof(tj9->pdata));
		err = kxtj9_parse_dt(&client->dev, &tj9->pdata);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse platfrom data err=%d\n", err);
			return err;
		}
	} else {
		if (client->dev.platform_data)
			tj9->pdata = *(struct kxtj9_platform_data *)
					client->dev.platform_data;
		else {
			dev_err(&client->dev,
				"platform data is NULL; exiting\n");
			err = -EINVAL;
			goto err_free_mem;
		}
	}

	tj9->client = client;
	tj9->power_enabled = false;
	tj9->chip_function_on = false;
	tj9->enable = false;

	if (tj9->pdata.init) {
		err = tj9->pdata.init();
		if (err < 0)
			goto err_free_mem;
	}

	err = kxtj9_power_init(tj9, true);
	if (err < 0) {
		dev_err(&tj9->client->dev, "power init failed! err=%d", err);
		goto err_pdata_exit;
	}

	err = kxtj9_device_power_on(tj9);
	if (err < 0) {
		dev_err(&client->dev, "power on failed! err=%d\n", err);
		goto err_power_deinit;
	}
	err = kxtj9_verify(tj9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_power_off;
	}

	i2c_set_clientdata(client, tj9);

	tj9->ctrl_reg1 = tj9->pdata.res_ctl | tj9->pdata.g_range;
	tj9->shift = 4;
	printk(KERN_ERR "%s:ctrl_reg1=%x\n", __func__, tj9->ctrl_reg1);
	tj9->last_poll_interval = tj9->pdata.init_interval;
	tj9->data_ctrl = 0x3;
	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0) {
		dev_err(&tj9->client->dev, "Set ODR failed\n");
		goto err_power_off;
	}

	err = kxtj9_setup_input_device(tj9);
	if (err) {
		dev_err(&client->dev, "setup input device failed: %d\n", err);
		goto err_power_off;
	}

	tj9->cdev = sensors_cdev;
	/* The min_delay is used by userspace and the unit is microsecond. */
	//tj9->cdev.min_delay = tj9->pdata.min_interval * 1000;
	tj9->cdev.delay_msec = tj9->pdata.init_interval;
	tj9->cdev.sensors_enable = kxtj9_enable_set;
	tj9->cdev.sensors_poll_delay = kxtj9_poll_delay_set;
	err = sensors_classdev_register(&tj9->input_dev->dev, &tj9->cdev);
	if (err) {
		dev_err(&client->dev, "class device create failed: %d\n", err);
		goto err_register_classdev;
	}
	INIT_DELAYED_WORK(&tj9->dwork, kxtj9_dwork_handler);
#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
	productinfo_register(PRODUCTINFO_SENSOR_ACCELEROMETER_ID,
						"kxtj9", "kionix");
#endif	/* CONFIG_HISENSE_PRODUCT_DEVINFO */
	dev_dbg(&client->dev, "%s: kxtj9_probe OK.\n", __func__);
	return 0;

err_register_classdev:
	input_unregister_device(tj9->input_dev);
err_power_off:
	kxtj9_device_power_off(tj9);
err_power_deinit:
	kxtj9_power_init(tj9, false);
err_pdata_exit:
	if (tj9->pdata.exit)
		tj9->pdata.exit();
err_free_mem:
	kfree(tj9);
	dev_err(&client->dev, "%s: kxtj9_probe err=%d\n", __func__, err);
	return err;
}

static int kxtj9_remove(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	if (tj9->enable) {
			cancel_delayed_work(&tj9->dwork);
			kxtj9_function_off(tj9);
	}
	input_unregister_device(tj9->input_dev);
	kxtj9_device_power_off(tj9);
	kxtj9_power_init(tj9, false);

	kfree(tj9);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj9_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	if (tj9->enable) {
			cancel_delayed_work(&tj9->dwork);
			kxtj9_function_off(tj9);
	}
	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int kxtj9_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (tj9->enable)
		schedule_delayed_work(&tj9->dwork,
				msecs_to_jiffies(tj9->last_poll_interval));

	mutex_unlock(&input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

static struct of_device_id kxtj9_match_table[] = {
	{ .compatible = "kionix,kxtj9", },
	{ },
};


MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = kxtj9_match_table,
		.pm	= &kxtj9_pm_ops,
	},
	.probe		= kxtj9_probe,
	.remove		= kxtj9_remove,
	.id_table	= kxtj9_id,
};

module_i2c_driver(kxtj9_driver);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
