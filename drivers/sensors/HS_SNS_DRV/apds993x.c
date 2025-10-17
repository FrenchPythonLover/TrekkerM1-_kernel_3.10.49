/*
 * apds993x.c - Linux kernel modules for ambient light + proximity sensor
 *
 * Copyright (C) 2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 * Copyright (C) 2012 Avago Technologies
 * Copyright (C) 2013 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>

#include <linux/productinfo.h>
#include "apds993x.h"

#define APDS993X_DRV_NAME	"apds993x"
#define DRIVER_VERSION		"1.0.0"

#define APDS993X_ALS_THRESHOLD_HSYTERESIS	20	/* % */

#define APDS_CAL_SKIP_COUNT     5
#define APDS_MAX_CAL	(10 + APDS_CAL_SKIP_COUNT)
#define CAL_NUM		99
#define CALIBRATE_PS_DELAY     6000/*us*/


/* Change History
 *
 * 1.0.0	Fundamental Functions of APDS-993x
 *
 */
#define APDS993X_IOCTL_PS_ENABLE	1
#define APDS993X_IOCTL_PS_GET_ENABLE	2
#define APDS993X_IOCTL_PS_GET_PDATA	3	/* pdata */
#define APDS993X_IOCTL_ALS_ENABLE	4
#define APDS993X_IOCTL_ALS_GET_ENABLE	5
#define APDS993X_IOCTL_ALS_GET_CH0DATA	6	/* ch0data */
#define APDS993X_IOCTL_ALS_GET_CH1DATA	7	/* ch1data */
#define APDS993X_IOCTL_ALS_DELAY	8

/*
 * Defines
 */
#define	APDS9930_ID	0x30
#define	APDS9931_ID	0x39
#define	APDS9900_ID	0x29
#define	APDS9901_ID	0x20

#define APDS993X_ENABLE_REG	0x00
#define APDS993X_ATIME_REG	0x01
#define APDS993X_PTIME_REG	0x02
#define APDS993X_WTIME_REG	0x03
#define APDS993X_AILTL_REG	0x04
#define APDS993X_AILTH_REG	0x05
#define APDS993X_AIHTL_REG	0x06
#define APDS993X_AIHTH_REG	0x07
#define APDS993X_PILTL_REG	0x08
#define APDS993X_PILTH_REG	0x09
#define APDS993X_PIHTL_REG	0x0A
#define APDS993X_PIHTH_REG	0x0B
#define APDS993X_PERS_REG	0x0C
#define APDS993X_CONFIG_REG	0x0D
#define APDS993X_PPCOUNT_REG	0x0E
#define APDS993X_CONTROL_REG	0x0F
#define APDS993X_REV_REG	0x11
#define APDS993X_ID_REG		0x12
#define APDS993X_STATUS_REG	0x13
#define APDS993X_CH0DATAL_REG	0x14
#define APDS993X_CH0DATAH_REG	0x15
#define APDS993X_CH1DATAL_REG	0x16
#define APDS993X_CH1DATAH_REG	0x17
#define APDS993X_PDATAL_REG	0x18
#define APDS993X_PDATAH_REG	0x19
#define APDS993X_POFFSET_REG	0x1E


#define CMD_BYTE		0x80
#define CMD_WORD		0xA0
#define CMD_SPECIAL		0xE0

#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_PS_ALS_INT	0xE7


/* Register Value define : ATIME */
#define APDS993X_100MS_ADC_TIME	0xDB  /* 100.64ms integration time */
#define APDS993X_50MS_ADC_TIME	0xED  /* 51.68ms integration time */
#define APDS993X_27MS_ADC_TIME	0xF6  /* 27.2ms integration time */

/* Register Value define : PRXCNFG */
#define APDS993X_ALS_REDUCE	0x04  /* ALSREDUCE - ALS Gain reduced by 4x */

/* Register Value define : PERS */
#define APDS993X_PPERS_0	0x00  /* Every proximity ADC cycle */
#define APDS993X_PPERS_1	0x10  /* 1 consecutive proximity value out of range */
#define APDS993X_PPERS_2	0x20  /* 2 consecutive proximity value out of range */
#define APDS993X_PPERS_3	0x30  /* 3 consecutive proximity value out of range */
#define APDS993X_PPERS_4	0x40  /* 4 consecutive proximity value out of range */
#define APDS993X_PPERS_5	0x50  /* 5 consecutive proximity value out of range */
#define APDS993X_PPERS_6	0x60  /* 6 consecutive proximity value out of range */
#define APDS993X_PPERS_7	0x70  /* 7 consecutive proximity value out of range */
#define APDS993X_PPERS_8	0x80  /* 8 consecutive proximity value out of range */
#define APDS993X_PPERS_9	0x90  /* 9 consecutive proximity value out of range */
#define APDS993X_PPERS_10	0xA0  /* 10 consecutive proximity value out of range */
#define APDS993X_PPERS_11	0xB0  /* 11 consecutive proximity value out of range */
#define APDS993X_PPERS_12	0xC0  /* 12 consecutive proximity value out of range */
#define APDS993X_PPERS_13	0xD0  /* 13 consecutive proximity value out of range */
#define APDS993X_PPERS_14	0xE0  /* 14 consecutive proximity value out of range */
#define APDS993X_PPERS_15	0xF0  /* 15 consecutive proximity value out of range */

#define APDS993X_APERS_0	0x00  /* Every ADC cycle */
#define APDS993X_APERS_1	0x01  /* 1 consecutive proximity value out of range */
#define APDS993X_APERS_2	0x02  /* 2 consecutive proximity value out of range */
#define APDS993X_APERS_3	0x03  /* 3 consecutive proximity value out of range */
#define APDS993X_APERS_5	0x04  /* 5 consecutive proximity value out of range */
#define APDS993X_APERS_10	0x05  /* 10 consecutive proximity value out of range */
#define APDS993X_APERS_15	0x06  /* 15 consecutive proximity value out of range */
#define APDS993X_APERS_20	0x07  /* 20 consecutive proximity value out of range */
#define APDS993X_APERS_25	0x08  /* 25 consecutive proximity value out of range */
#define APDS993X_APERS_30	0x09  /* 30 consecutive proximity value out of range */
#define APDS993X_APERS_35	0x0A  /* 35 consecutive proximity value out of range */
#define APDS993X_APERS_40	0x0B  /* 40 consecutive proximity value out of range */
#define APDS993X_APERS_45	0x0C  /* 45 consecutive proximity value out of range */
#define APDS993X_APERS_50	0x0D  /* 50 consecutive proximity value out of range */
#define APDS993X_APERS_55	0x0E  /* 55 consecutive proximity value out of range */
#define APDS993X_APERS_60	0x0F  /* 60 consecutive proximity value out of range */

/* Register Value define : CONTROL */
#define APDS993X_AGAIN_1X	0x00  /* 1X ALS GAIN */
#define APDS993X_AGAIN_8X	0x01  /* 8X ALS GAIN */
#define APDS993X_AGAIN_16X	0x02  /* 16X ALS GAIN */
#define APDS993X_AGAIN_120X	0x03  /* 120X ALS GAIN */

#define APDS993X_PRX_IR_DIOD	0x20  /* Proximity uses CH1 diode */

#define APDS993X_PGAIN_1X	0x00  /* PS GAIN 1X */
#define APDS993X_PGAIN_2X	0x04  /* PS GAIN 2X */
#define APDS993X_PGAIN_4X	0x08  /* PS GAIN 4X */
#define APDS993X_PGAIN_8X	0x0C  /* PS GAIN 8X */

#define APDS993X_PDRVIE_100MA	0x00  /* PS 100mA LED drive */
#define APDS993X_PDRVIE_50MA	0x40  /* PS 50mA LED drive */
#define APDS993X_PDRVIE_25MA	0x80  /* PS 25mA LED drive */
#define APDS993X_PDRVIE_12_5MA	0xC0  /* PS 12.5mA LED drive */


typedef enum {
	APDS993X_ALS_RES_10240 = 0,    /* 27.2ms integration time */
	APDS993X_ALS_RES_19456 = 1,    /* 51.68ms integration time */
	APDS993X_ALS_RES_37888 = 2     /* 100.64ms integration time */
} apds993x_als_res_e;

typedef enum {
	APDS993X_ALS_GAIN_1X    = 0,    /* 1x AGAIN */
	APDS993X_ALS_GAIN_8X    = 1,    /* 8x AGAIN */
	APDS993X_ALS_GAIN_16X   = 2,    /* 16x AGAIN */
	APDS993X_ALS_GAIN_120X  = 3     /* 120x AGAIN */
} apds993x_als_gain_e;

/*
 * Structs
 */
struct apds993x_data {
	struct i2c_client *client;
	struct mutex op_mutex;
	struct delayed_work	dwork;		/* for PS interrupt */
	struct delayed_work	als_dwork;	/* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct apds993x_platform_data *platform_data;
	int irq;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	/* register configuration*/
	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;
	unsigned int pgain;
	unsigned int pdrive;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	/* save sensor enabling state for resume */
	unsigned int als_enable_state;
	bool als_function_on;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold; 	/* always lower than ps_threshold */
	unsigned int ps_data;			/* to store PS data */
	struct wake_lock ps_wake_lock;

	/*calibration*/
	unsigned int cross_talk;		/* cross_talk value */
	unsigned int avg_cross_talk;		/* average cross_talk  */
	unsigned int ps_cal_result;		/* result of calibration*/
	unsigned int ps_crosstalk_max;

	int ps_cal_data;
	char calibrate_buf[CAL_NUM];
	int ps_cal_params[3];
	int pre_enable_ps;

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;		/* to store ALS data */
	int als_prev_lux;		/* to store previous lux value */

	unsigned int als_gain;		/* needed for Lux calculation */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	unsigned int als_atime_index;	/* storage for als integratiion time */
	unsigned int als_again_index;	/* storage for als GAIN */
	unsigned int als_reduce;	/* flag indicate ALS 6x reduction */
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65536.0",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 30000, /* in microseconds */
	.max_delay = 8393,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
	.sensors_calibrate = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 30000, /* in microseconds */
	.max_delay = 8393,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 3,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
	.sensors_calibrate = NULL,
};

/*PS tuning value*/
static int apds993x_ps_detection_threshold;
static int apds993x_ps_hsyteresis_threshold;
static int apds993x_ps_pulse_number;

/*
 * Global data
 */
static struct apds993x_data *pdev_data;

/* global i2c_client to support ioctl */
static struct workqueue_struct *apds993x_workqueue;

static unsigned char apds993x_als_atime_tb[] = { 0xF6, 0xED, 0xDB };
/*static unsigned short apds993x_als_integration_tb[] = {2720, 5168, 10064};
static unsigned short apds993x_als_res_tb[] = { 10240, 19456, 37888 };*/
static unsigned char apds993x_als_again_tb[] = { 1, 8, 16, 120 };
static unsigned char apds993x_als_again_bit_tb[] = { 0x00, 0x01, 0x02, 0x03 };

/*calibration*/
static int apds993x_cross_talk_val;

/* ALS tuning */
static int apds993x_ga;
static int apds993x_coe_b;
static int apds993x_coe_c;
static int apds993x_coe_d;
static int light_fix_factor;

#define APDS993X_DF	52
#define ALS_MAX_RANGE	65535
/* global i2c_client to support ioctl */
static unsigned char apds993x_ps_pdrive_tb[] = {
	APDS993X_PDRVIE_100MA,
	APDS993X_PDRVIE_50MA,
	APDS993X_PDRVIE_25MA,
	APDS993X_PDRVIE_12_5MA
};

static unsigned char apds993x_ps_pgain_tb[] = {
	APDS993X_PGAIN_1X,
	APDS993X_PGAIN_2X,
	APDS993X_PGAIN_4X,
	APDS993X_PGAIN_8X
};


#define NEAR_CODE	1
#define FAR_CODE	0
/* Register Value define : ENABLE */
#define APDS993X_POWER_ON				0x01
#define APDS993X_ENABLE_ALS				0x02
#define APDS993X_ENABLE_PS					0x04
#define APSD993X_ENABLE_PS_INTERRUPT		0x20
#ifdef CONFIG_PS_CROSSTALK_CALIBRATE_AUTO
/*calibration*/
#define DEFAULT_CROSS_TALK	400
#define ADD_TO_CROSS_TALK	300
#define SUB_FROM_PS_THRESHOLD	100

#define APDS9930_PS_CALIBRATED_XTALK    50
#define APDS9930_PS_CALIBRATED_XTALK_BASELINE 10
#else
#define DEFAULT_CROSS_TALK	100
#define ADD_TO_CROSS_TALK	300
#define SUB_FROM_PS_THRESHOLD	100
#endif

static int apds993x_init_device(struct i2c_client *client);

/*
 * Management functions
 */
static int apds993x_set_command(struct i2c_client *client, int command)
{
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;

	ret = i2c_smbus_write_byte(client, clearInt);

	return ret;
}

static int apds993x_set_enable(struct i2c_client *client, int enable)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ENABLE_REG, enable);

	data->enable = enable;

	return ret;
}

static int apds993x_set_atime(struct i2c_client *client, int atime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ATIME_REG, atime);

	data->atime = atime;

	return ret;
}

static int apds993x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_PTIME_REG, ptime);

	data->ptime = ptime;

	return ret;
}

static int apds993x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_WTIME_REG, wtime);

	data->wtime = wtime;

	return ret;
}

static int apds993x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_AILTL_REG, threshold);

	data->ailt = threshold;

	return ret;
}

static int apds993x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_AIHTL_REG, threshold);

	data->aiht = threshold;

	return ret;
}

static int apds993x_set_pilt(struct i2c_client *client, int threshold)
{
	int ret;

	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PILTL_REG, threshold);

	return ret;
}

static int apds993x_set_piht(struct i2c_client *client, int threshold)
{
	int ret;

	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PIHTL_REG, threshold);

	return ret;
}

static int apds993x_set_pers(struct i2c_client *client, int pers)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_PERS_REG, pers);

	data->pers = pers;

	return ret;
}

static int apds993x_set_config(struct i2c_client *client, int config)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_CONFIG_REG, config);

	data->config = config;

	return ret;
}

static int apds993x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_PPCOUNT_REG, ppcount);

	data->ppcount = ppcount;

	return ret;
}

static int apds993x_set_control(struct i2c_client *client, int control)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_CONTROL_REG, control);

	data->control = control;

	return ret;
}

static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int luxValue = 0;
	int IAC1 = 0;
	int IAC2 = 0;
	int IAC = 0;

	if (ch0data >= 0x9400) {
		ch0data = 0x9400;
		ch1data = 0x41d6;
	}

	IAC1 =
		100 * ch0data
		- apds993x_coe_b * ch1data;
	IAC2 =
		apds993x_coe_c * ch0data
		- apds993x_coe_d * ch1data;

	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

	if (IAC1 < 0 && IAC2 < 0) {
		IAC = 0;/* cdata and irdata saturated*/
		return -EPERM;/* don't report first, change gain may help*/
	}
	IAC = IAC / 100;
	luxValue =
		(u32)IAC * apds993x_ga * APDS993X_DF * light_fix_factor /
		(273 * (256 - data->atime) * apds993x_als_again_tb[data->als_again_index]);
	return luxValue;
}


static void psensor_report_dist(struct apds993x_data *data, int dist_code)
{
	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(data->input_dev_ps, ABS_DISTANCE, dist_code ? 1000 : 1);
	input_report_abs(data->input_dev_ps, ABS_DISTANCE, dist_code ? 1023 : 0);
	input_event(data->input_dev_ps, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(data->input_dev_ps, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(data->input_dev_ps);


	if (dist_code == FAR_CODE) {
		if (!wake_lock_active(&data->ps_wake_lock)) {
			printk("wake_lock PROX_NEAR_TO_FAR_WLOCK not be locked, and lock it!\n");
			wake_lock_timeout(&data->ps_wake_lock, HZ);
		} else {
			printk("wake_lock PROX_NEAR_TO_FAR_WLOCK be locked, do nothing!\n");
		}
	}
}

static void apds993x_report_als_event(struct input_dev *als_dev,
									const unsigned int lux)
{
	ktime_t timestamp;
	timestamp = ktime_get_boottime();

	input_report_abs(als_dev, ABS_MISC, lux);
	input_event(als_dev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(als_dev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(als_dev);
}

static void apds993x_change_ps_threshold(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);

	data->ps_data =
		i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);
	printk("%s:ps_data=%d\n", __func__, data->ps_data);

	if (data->ps_data >= data->platform_data->piht) {
		psensor_report_dist(data, NEAR_CODE);

		i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PILTL_REG, data->platform_data->pilt);
		i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PIHTL_REG, 1023);

		printk("far-to-near detected\n");
	} else if (data->ps_data <= data->platform_data->pilt) {
		psensor_report_dist(data, FAR_CODE);

		i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PIHTL_REG, data->platform_data->piht);

		printk("near-to-far detected\n");
	}
}


/* ALS polling routine */
static void apds993x_als_polling_work_handler(struct work_struct *work)
{
	struct apds993x_data *data =
		container_of(work, struct apds993x_data, als_dwork.work);
	struct i2c_client *client = data->client;

	int ch0data = 0, ch1data = 0;
	int luxValue = 0;
	unsigned char lux_is_valid = 1;

	if (!(data->enable_als_sensor)) {
		printk("%s(): als not enabled, do nothing\n", __func__);
		return;
	}

	if (!data->als_function_on) {
		unsigned int temp_reg =
			(data->enable | APDS993X_ENABLE_ALS | APDS993X_POWER_ON);
		printk("%s(): als not func on, turn it on\n", __func__);
		if (0 != apds993x_set_enable(data->client, temp_reg)) {
			printk("%s(): als function on failded! Try again!\n",
				__func__);
			goto quit;
		}
		data->enable = temp_reg;
		data->als_function_on = true;
		mdelay(10);
	}

	ch0data = i2c_smbus_read_word_data(client,
				CMD_WORD|APDS993X_CH0DATAL_REG);
	ch1data = i2c_smbus_read_word_data(client,
				CMD_WORD|APDS993X_CH1DATAL_REG);

	luxValue = LuxCalculation(client, ch0data, ch1data);

	if (luxValue >= 0) {
		luxValue =
			(luxValue < ALS_MAX_RANGE) ? luxValue : ALS_MAX_RANGE;
	} else {
	/* don't report, this is invalid lux value*/
		lux_is_valid = 0;
	}

	if (lux_is_valid) {
		/* report the lux level*/
		if (data->als_data == luxValue)
			luxValue++;
		apds993x_report_als_event(data->input_dev_als, luxValue);
		data->als_data = luxValue;
	}
quit:
	/*restart timer*/
	queue_delayed_work(apds993x_workqueue,
				&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
}


/* PS interrupt routine */
static void apds993x_work_handler(struct work_struct *work)
{
	struct apds993x_data *data =
		container_of(work, struct apds993x_data, dwork.work);
	struct i2c_client *client = data->client;
	int status =
		i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);
	apds993x_set_command(client, 2);

	/* disable 993x's ADC first */
	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, 1);

	printk("%s(): status = 0x%x, enable = 0x%x\n",
		__func__, status, data->enable);

	if ((status & data->enable & 0x20) == 0x20) {
		/* check if this is triggered by background ambient noise */
		int ch0data =
			i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
		if (ch0data < (75 * 0x9400) / 100) {
			apds993x_change_ps_threshold(client);
		} else {
			printk("ch0data is %d\n", ch0data);
			printk("Triggered by background ambient noise\n");
			psensor_report_dist(data, FAR_CODE);

			i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PILTL_REG, 0);
			i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PIHTL_REG, data->platform_data->piht);
		}
	} else {
		printk("PS int not valid, do nothing!\n");
	}
	apds993x_set_command(client, 2);

	i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ENABLE_REG, data->enable);
}

/* assume this is ISR */
static irqreturn_t apds993x_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct apds993x_data *data = i2c_get_clientdata(client);

	if (data->enable_ps_sensor == 1) {
		cancel_delayed_work(&data->dwork);
		queue_delayed_work(apds993x_workqueue, &data->dwork, 0);
	} else {
		printk("%s(): ps not enabled\n", __func__);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_PS_CROSSTALK_CALIBRATE_AUTO
static int apds9930_Enable_runtime_calibration(struct i2c_client *client)
{
	unsigned short i2c_pon_data_ps = 0x05;
	unsigned short i2c_data_power_down = 0x00;
	unsigned char APDS9930_STATUS_PVALID = 0x02;

	unsigned char temp_status;
	unsigned char temp_offset = 64;
	unsigned short pdata;
	unsigned char i, j;
	unsigned char Loop_ur = 64;
	int status;

	struct apds993x_data *data = i2c_get_clientdata(client);
	status = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ENABLE_REG, i2c_data_power_down);/*disable*/

	for (i = 0; i < 10; i++) {
		if (i == 0) {
			status = i2c_smbus_write_byte_data(client,
						 CMD_BYTE|APDS993X_POFFSET_REG, 0);
		}
		status = i2c_smbus_write_byte_data(client,
				CMD_BYTE|APDS993X_ENABLE_REG, i2c_pon_data_ps);/* PEN and PON*/

		for (j = 0; j < 10; j++) {
			temp_status =
				i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);
			if ((temp_status & APDS9930_STATUS_PVALID) == APDS9930_STATUS_PVALID)
				break;
			mdelay(1);
		}

		pdata =
			i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);
		if ((i == 0) && (pdata > data->ps_crosstalk_max)) {
			printk("Crosstalk pdata:%d is larger than the max:%d,Faulty phone!!!\n",
					 pdata, data->ps_crosstalk_max);
			status =
				i2c_smbus_write_byte_data(client,
					CMD_BYTE|APDS993X_POFFSET_REG, 0);
			status =
				i2c_smbus_write_byte_data(client,
					CMD_BYTE|APDS993X_ENABLE_REG, i2c_data_power_down);/*disable*/
			data->cross_talk = 0;
			return -EPERM;
		} else if (i != 0) {
			Loop_ur = Loop_ur / 2;
			printk(KERN_ERR "After writting %d to OffsetReg,pdata is %d\n", temp_offset, pdata);
			if ((pdata <= APDS9930_PS_CALIBRATED_XTALK)
				&& (pdata >= APDS9930_PS_CALIBRATED_XTALK_BASELINE)) {
				printk("Calibration finished successfully!!\n");
				break;/*end calibration*/
			} else {
				if (pdata > APDS9930_PS_CALIBRATED_XTALK) {
					/* reduce*/
					if ((temp_offset >= 0) && (temp_offset < 127)) {
						 temp_offset += Loop_ur;
					} else {
						if (temp_offset == 127)
							temp_offset = 1;
						temp_offset -= 1;
					}
				} else if (pdata < APDS9930_PS_CALIBRATED_XTALK_BASELINE) {
					/* increase*/
					if ((temp_offset > 0) && (temp_offset <= 127)) {
						temp_offset -= Loop_ur;
					} else {
						if (temp_offset == 0) {
							temp_offset = 127; /* start from 128*/
							Loop_ur = 64;
						}
						if (temp_offset == 255)
							temp_offset = 0; /* something is wrong*/

						temp_offset += 1;	/* start from 128*/
						temp_offset = temp_offset&0xFF;
					}
				}
			}
		}
		status =
			i2c_smbus_write_byte_data(client,
				CMD_BYTE|APDS993X_POFFSET_REG, temp_offset);
		status =
			i2c_smbus_write_byte_data(client,
				CMD_BYTE|APDS993X_ENABLE_REG, i2c_data_power_down);/*disable*/

	}

	status =
		i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ENABLE_REG, data->enable);
	data->cross_talk = temp_offset;

	return 0;
}
#endif

static int apds993x_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	unsigned int temp_reg = data->enable;
	int ret = 0;
#ifdef CONFIG_PS_CROSSTALK_CALIBRATE_AUTO
	int cali_flag = 0;
#endif

	pr_debug("%s: val = %d\n", __func__, val);
	mutex_lock(&data->op_mutex);

	if ((val == 1) && (data->enable_ps_sensor == 0)) {

#ifdef CONFIG_PS_CROSSTALK_CALIBRATE_AUTO
		cali_flag = apds9930_Enable_runtime_calibration(client);
#endif
		printk("%s(): pilt = %d, piht = %d\n", __func__,
			data->platform_data->pilt, data->platform_data->piht);
		/* init threshold for proximity */
		apds993x_set_pilt(client, data->platform_data->pilt);
		apds993x_set_piht(client, data->platform_data->piht);

		/* enable PS interrupt */
		temp_reg |= (APDS993X_ENABLE_PS |
					APDS993X_ENABLE_ALS |
					APDS993X_POWER_ON);
		ret = i2c_smbus_write_byte_data(data->client,
			CMD_BYTE | APDS993X_ENABLE_REG, temp_reg);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"%s(): write  APDS993X_ENABLE_REG error!\n", __func__);
			goto quit;
		}

		temp_reg |= APSD993X_ENABLE_PS_INTERRUPT ;
		ret = i2c_smbus_write_byte_data(data->client,
			CMD_BYTE | APDS993X_ENABLE_REG, temp_reg);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"%s(): write  APDS993X_ENABLE_REG error2!\n", __func__);
			goto quit;
		}
		data->enable_ps_sensor = 1;
	} else if ((val == 0) && (data->enable_ps_sensor == 1)) {
		temp_reg &= ((~APSD993X_ENABLE_PS_INTERRUPT) &
						(~APDS993X_ENABLE_PS));

		if (data->enable_als_sensor == 0) {
			temp_reg &= (~APDS993X_ENABLE_ALS);
			data->als_function_on = false;
		}
		ret = i2c_smbus_write_byte_data(data->client,
			CMD_BYTE | APDS993X_ENABLE_REG, temp_reg);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"%s(): write  APDS993X_ENABLE_REG error!\n", __func__);
			goto quit;
		}
		apds993x_set_pilt(client, 0);
		apds993x_set_piht(client, 1023);
		data->enable_ps_sensor = 0;
	}

	data->enable = temp_reg;
quit:
	mutex_unlock(&data->op_mutex);
	return ret;
}

static ssize_t apds993x_pdata_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data =
		container_of(sensors_cdev, struct apds993x_data, ps_cdev);
	struct i2c_client *client = data->client;
	int n = 0;

	if (data->enable_ps_sensor == 0) {
		n = printk("%s:PS function not enabled!\n", __func__);
	} else {
		int pdata =
			i2c_smbus_read_word_data(client, CMD_WORD | APDS993X_PDATAL_REG);
		n = snprintf(buf, 10, "%d\n", pdata);
	}

	return n;
}
static DEVICE_ATTR(ps_data, 0440, apds993x_pdata_show, NULL);

static ssize_t ps_thd_close_show(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data =
		container_of(sensors_cdev, struct apds993x_data, ps_cdev);
	return snprintf(buf, 10, "%d\n", data->platform_data->piht);
}

static ssize_t ps_thd_close_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data =
		container_of(sensors_cdev, struct apds993x_data, ps_cdev);
	struct i2c_client *client = data->client;
	if (data->enable_ps_sensor == 0) {
		printk("%s: ps not enabled, can not set ps_thd_close\n", __func__);
    } else {
		unsigned long val = simple_strtoul(buf, NULL, 10);
		apds993x_set_piht(client, val);
		data->platform_data->piht = val;
	}
	return count;
}

static DEVICE_ATTR(ps_thd_close, 0640,
	ps_thd_close_show, ps_thd_close_store);

static ssize_t ps_thd_away_show(struct device *dev,
							struct device_attribute *attr,
							char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data =
		container_of(sensors_cdev, struct apds993x_data, ps_cdev);
	return snprintf(buf, 10, "%d\n", data->platform_data->pilt);
}

static ssize_t ps_thd_away_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data =
		container_of(sensors_cdev, struct apds993x_data, ps_cdev);
	struct i2c_client *client = data->client;
	if (data->enable_ps_sensor == 0) {
		printk("%s: ps not enabled, can not set ps_thd_away\n", __func__);
	} else {
		unsigned long val = simple_strtoul(buf, NULL, 10);
		apds993x_set_pilt(client, val);
		data->platform_data->pilt = val;
	}
	return count;
}
static DEVICE_ATTR(ps_thd_away, 0640,
	ps_thd_away_show, ps_thd_away_store);

#ifdef CONFIG_PS_CROSSTALK_CALIBRATE_AUTO
static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data =
		container_of(sensors_cdev, struct apds993x_data, ps_cdev);
	return snprintf(buf, 10, "%d\n", data->cross_talk);
}
static DEVICE_ATTR(ps_canc, 0440, ps_canc_show, NULL);

static ssize_t ps_crosstalk_maxthd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data = container_of(sensors_cdev,
			struct apds993x_data, ps_cdev);
	return snprintf(buf, 10, "%d\n",
		data->ps_crosstalk_max);
}
static ssize_t ps_crosstalk_maxthd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct apds993x_data *data = container_of(sensors_cdev,
			struct apds993x_data, ps_cdev);
	int code;
	sscanf(buf, "%d", &code);
	if ((code > 0) && (code < 1024)) {
		data->ps_crosstalk_max = (uint16_t)code;
	} else {
		dev_err(&data->client->dev,
			"%s(): It is an illegal para, %d!", __func__, code);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_crosstalk_maxthd, 0640, ps_crosstalk_maxthd_show,  ps_crosstalk_maxthd_store);
#endif

static int apds993x_enable_als_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret = 0;
	mutex_lock(&data->op_mutex);
	if ((val == 1) && (data->enable_als_sensor == 0)) {
		cancel_delayed_work(&data->als_dwork);
		queue_delayed_work(apds993x_workqueue,
			&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
		data->enable_als_sensor = 1;
	} else if ((val == 0) && (data->enable_als_sensor == 1)) {
		cancel_delayed_work(&data->als_dwork);
		if (data->enable_ps_sensor == 0) {
			unsigned int temp_reg = data->enable & (~APDS993X_ENABLE_ALS);
			ret = apds993x_set_enable(client, temp_reg);
			if (ret < 0) {
				printk("%s(): apds993x_set_enable(ALS_OFF) failed!\n", __func__);
				goto quit;
			}
			data->enable = temp_reg;
			data->als_function_on = false;
		}
		data->enable_als_sensor = 0;
    }
quit:
    mutex_unlock(&data->op_mutex);
    return ret;
}

static int apds993x_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds993x_data *data = container_of(sensors_cdev,
			struct apds993x_data, als_cdev);
	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return apds993x_enable_als_sensor(data->client, enable);
}

static int apds993x_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds993x_data *data = container_of(sensors_cdev,
			struct apds993x_data, ps_cdev);
	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	printk("%s: enable ps senosr ( %d)\n", __func__, enable);
	return apds993x_enable_ps_sensor(data->client, enable);
}

static struct attribute *apds993x_prox_attributes[] = {
	&dev_attr_ps_data.attr,
	&dev_attr_ps_thd_close.attr,
	&dev_attr_ps_thd_away.attr,
#ifdef CONFIG_PS_CROSSTALK_CALIBRATE_AUTO
	&dev_attr_ps_canc.attr,
	&dev_attr_ps_crosstalk_maxthd.attr,
#endif
    NULL
};

static const struct attribute_group apds993x_prox_attr_group = {
	.attrs = apds993x_prox_attributes,
};

static int apds993x_check_chip_id(struct i2c_client *client)
{
	int id;

	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ID_REG);
	switch (id) {
	case APDS9931_ID:
		dev_dbg(&client->dev, "APDS9931\n");
		break;

	case APDS9930_ID:
		dev_dbg(&client->dev, "APDS9930\n");
		break;

	case APDS9900_ID:
		dev_dbg(&client->dev, "APDS9900\n");
		break;

	case APDS9901_ID:
		dev_dbg(&client->dev, "APDS9931\n");
		break;
	default:
		dev_err(&client->dev, "Neither APDS993x nor APDS990x\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * Initialization function
 */
static int apds993x_init_device(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int err;

	err = apds993x_set_enable(client, 0);
	if (err < 0)
		return err;

	/* 100.64ms ALS integration time */
	err = apds993x_set_atime(client,
			apds993x_als_atime_tb[data->als_atime_index]);
	if (err < 0)
		return err;

	/* 2.72ms Prox integration time */
	err = apds993x_set_ptime(client, 0xFF);
	if (err < 0)
		return err;

	/* 2.72ms Wait time */
	err = apds993x_set_wtime(client, 0xFF);
	if (err < 0)
		return err;

	err = apds993x_set_ppcount(client, apds993x_ps_pulse_number);
	if (err < 0)
		return err;

	/* no long wait */
	err = apds993x_set_config(client, 0);
	if (err < 0)
		return err;

	err = apds993x_set_control(client,
				apds993x_ps_pdrive_tb[data->pdrive] |
				APDS993X_PRX_IR_DIOD |
				apds993x_ps_pgain_tb[data->pgain] |
				apds993x_als_again_bit_tb[data->als_again_index]);
	if (err < 0)
		return err;

	/* init threshold for proximity */
	err = apds993x_set_pilt(client, 0);
	if (err < 0)
		return err;

	err = apds993x_set_piht(client, 1023);
	if (err < 0)
		return err;


	/* force first ALS interrupt to get the environment reading */
	err = apds993x_set_ailt(client, 0xFFFF);
	if (err < 0)
		return err;

	err = apds993x_set_aiht(client, 0);
	if (err < 0)
		return err;

	/* 2 consecutive Interrupt persistence */
	err = apds993x_set_pers(client, APDS993X_PPERS_2|APDS993X_APERS_2);
	if (err < 0)
		return err;

	/* sensor is in disabled mode but all the configurations are preset */
	return 0;
}

static int apds993x_suspend(struct device *dev)
{
	struct apds993x_data *data = dev_get_drvdata(dev);

	disable_irq_nosync(data->client->irq);

	if (data->enable_als_sensor == 1) {
		cancel_delayed_work(&data->als_dwork);
		if (data->enable_ps_sensor == 0) {
			data->enable &= (~APDS993X_ENABLE_ALS);
			if (apds993x_set_enable(data->client, data->enable) < 0) {
				printk("%s(): apds993x_set_enable(ALS_OFF) failed!\n", __func__);
				return -EIO;
			}
			data->als_function_on = false;
		}
	}
	printk("%s.\n", __func__);
	return 0;
}


static int apds993x_resume(struct device *dev)
{
    struct apds993x_data *data = dev_get_drvdata(dev);
    printk("%s.\n", __func__);
    if (data->enable_als_sensor == 1) {
		schedule_delayed_work(&data->als_dwork,
			msecs_to_jiffies(data->als_poll_delay));
	}
    enable_irq(data->client->irq);
    return 0;
}


static int apds993x_regulator_configure(struct apds993x_data *data, bool on)
{
	int rc;

	if (!on) {

		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				APDS993X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				APDS993X_VIO_MAX_UV);

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
				APDS993X_VDD_MIN_UV, APDS993X_VDD_MAX_UV);
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
				APDS993X_VIO_MIN_UV, APDS993X_VIO_MAX_UV);
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
		regulator_set_voltage(data->vdd, 0, APDS993X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int apds993x_regulator_power_on(struct apds993x_data *data, bool on)
{
	int rc = 0;

	if (!on) {
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
			dev_err(&data->client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
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
			return rc;
		}
	}

enable_delay:
	msleep(130);
	dev_dbg(&data->client->dev,
		"Sensor regulator power on =%d\n", on);
	return rc;
}

static int apds993x_platform_hw_power_on(bool on)
{
	struct apds993x_data *data;
	int err = 0;

	if (pdev_data == NULL)
		return -ENODEV;

	data = pdev_data;
	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				err = pinctrl_select_state(data->pinctrl,
					data->pin_default);
			else
				err = pinctrl_select_state(data->pinctrl,
					data->pin_sleep);
			if (err)
				dev_err(&data->client->dev,
					"Can't select pinctrl state\n");
		}

		err = apds993x_regulator_power_on(data, on);
		if (err)
			dev_err(&data->client->dev,
					"Can't configure regulator!\n");
		else
			data->power_on = on;
	}

	return err;
}

static int apds993x_platform_hw_init(void)
{
	struct i2c_client *client;
	struct apds993x_data *data;
	int error;

	if (pdev_data == NULL)
		return -ENODEV;

	data = pdev_data;
	client = data->client;

	error = apds993x_regulator_configure(data, true);
	if (error < 0) {
		dev_err(&client->dev, "unable to configure regulator\n");
		return error;
	}

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure apds993x irq gpio */
		error = gpio_request_one(data->platform_data->irq_gpio,
				GPIOF_DIR_IN,
				"apds993x_irq_gpio");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				data->platform_data->irq_gpio);
		}
		data->irq = client->irq =
			gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		dev_err(&client->dev, "irq gpio not provided\n");
	}
	return 0;
}

static void apds993x_platform_hw_exit(void)
{
	struct apds993x_data *data = pdev_data;

	if (data == NULL)
		return;

	apds993x_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
}

static int apds993x_pinctrl_init(struct apds993x_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep =
		pinctrl_lookup_state(data->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(data->pin_sleep);
	}

	return 0;
}

static int apds993x_parse_dt(struct device *dev,
		struct apds993x_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;

	/* set functions of platform data */
	pdata->init = apds993x_platform_hw_init;
	pdata->exit = apds993x_platform_hw_exit;
	pdata->power_on = apds993x_platform_hw_power_on;

	/* irq gpio */
	rc = of_get_named_gpio_flags(np, "avago,irq-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read irq gpio\n");
		return rc;
	}
	pdata->irq_gpio = rc;

	/* ps tuning data*/
	rc = of_property_read_u32(np, "avago,ps_close_thd", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd\n");
		return rc;
	}
	pdata->piht = tmp;

	rc = of_property_read_u32(np, "avago,ps_away_thd", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd\n");
		return rc;
	}
	pdata->pilt = tmp;

	rc = of_property_read_u32(np, "avago,cross-talk", &tmp);
	pdata->cross_talk = rc ? DEFAULT_CROSS_TALK : tmp;

	rc = of_property_read_u32(np, "avago,ps_pulse", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_pulse\n");
		return rc;
	}
	pdata->prox_pulse = tmp;

	rc = of_property_read_u32(np, "avago,ps_crosstalk_max", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_pulse\n");
		return rc;
	}
	pdata->ps_crosstalk_max = tmp;

	rc = of_property_read_u32(np, "avago,ps_pgain", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_pgain\n");
		return rc;
	}
	pdata->prox_gain = tmp;

	/* ALS tuning value */
	rc = of_property_read_u32(np, "avago,als-B", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read apds993x coefficient b\n");
		return rc;
	}
	pdata->als_B = tmp;

	rc = of_property_read_u32(np, "avago,als-C", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read apds993x coefficient c\n");
		return rc;
	}
	pdata->als_C = tmp;

	rc = of_property_read_u32(np, "avago,als-D", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read apds993x coefficient d\n");
		return rc;
	}
	pdata->als_D = tmp;

	rc = of_property_read_u32(np, "avago,ga-value", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read gain value\n");
		return rc;
	}
	pdata->ga_value = tmp;

	rc = of_property_read_u32(np, "avago,light_fix_factor", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read light_fix_factor value\n");
		return rc;
	}
	pdata->light_fix_factor = tmp;

	pdata->default_cal = of_property_read_bool(np, "avago,default-cal");

	rc = of_property_read_u32(np, "avago,ps_drive", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_drive\n");
		return rc;
	}
	pdata->pdrive = tmp;
	return 0;
}

static int als_input_init(struct apds993x_data *data)
{
	int err = 0;

    /* Register to ALS Input Device */
	data->input_dev_als = devm_input_allocate_device(&data->client->dev);
	if (!data->input_dev_als) {
		pr_err("%s: Failed to allocate input device als\n", __func__);
		return -ENOMEM;
    }

	data->input_dev_als->name = "light";
	set_bit(EV_ABS, data->input_dev_als->evbit);
	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 65535, 0, 0);
	input_set_drvdata(data->input_dev_als, data);

	err = input_register_device(data->input_dev_als);
	if (err < 0) {
		pr_err("%s: Unable to register input device als: %s\n",
			__func__, data->input_dev_als->name);
		goto err_free_als_input_device;
	}
	return err;
err_free_als_input_device:
	input_free_device(data->input_dev_als);
	return err;
}


static int ps_input_init(struct apds993x_data *data)
{
	int err = 0;

    /* Register to PS Input Device */
	data->input_dev_ps = devm_input_allocate_device(&data->client->dev);
	if (!data->input_dev_ps) {
		pr_err("%s: Failed to allocate input device ps\n", __func__);
		return -ENOMEM;
	}

	data->input_dev_ps->name = "proximity";
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1023, 0, 0);
	input_set_drvdata(data->input_dev_ps, data);

	err = input_register_device(data->input_dev_ps);
	if (err < 0) {
		pr_err("%s: Unable to register input device ps: %s\n",
			__func__, data->input_dev_ps->name);
		goto err_free_ps_input_device;
	}
	return err;

err_free_ps_input_device:
	input_free_device(data->input_dev_ps);
	return err;
}


/*
 * I2C init/probing/exit functions
 */
static int apds993x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct apds993x_data *data;
	struct apds993x_platform_data *pdata;
	int err = 0;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA)) {
		err = -ESPIPE;
		goto exit;
	}

    /*1.parse devicetree*/
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct apds993x_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory.\n");
			err = -ENOMEM;
			goto exit;
		}

		client->dev.platform_data = pdata;
		err = apds993x_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "apds993x_parse_dt() err.\n");
			kfree(pdata);
			goto exit;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data.\n");
			err = -ENODEV;
			goto exit;
		}
	}

	/* Set the default parameters */
	apds993x_ps_detection_threshold = pdata->piht;
	apds993x_ps_hsyteresis_threshold = pdata->pilt;
	apds993x_ps_pulse_number = pdata->prox_pulse;

	apds993x_coe_b = pdata->als_B;
	apds993x_coe_c = pdata->als_C;
	apds993x_coe_d = pdata->als_D;
	apds993x_ga = pdata->ga_value;
	light_fix_factor = pdata->light_fix_factor;

	data = kzalloc(sizeof(struct apds993x_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory.\n");
		err = -ENOMEM;
		goto exit;
	}
	pdev_data = data;

	data->platform_data = pdata;
	data->client = client;

	/* 2.initialize pinctrl */
	err = apds993x_pinctrl_init(data);
	if (err) {
		dev_err(&client->dev, "Can't initialize pinctrl.\n");
		goto exit_kfree;
	}
	err = pinctrl_select_state(data->pinctrl, data->pin_default);
	if (err) {
		dev_err(&client->dev, "Can't select pinctrl default state.\n");
		goto exit_kfree;
	}

	/* 3.h/w initialization */
	if (pdata->init) {
		err = pdata->init();
		if (err) {
			dev_err(&client->dev, "pdata init failed.\n");
			goto exit_pdata_init;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "pdata power on failed.\n");
			goto exit_pdata_power_on;
		}
	}
	i2c_set_clientdata(client, data);

	/*4. init paras */
	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = apds993x_ps_detection_threshold;
	data->ps_hysteresis_threshold = apds993x_ps_hsyteresis_threshold;
	data->enable_als_sensor = 0;	/* default to 0*/
	data->enable_ps_sensor = 0;	/*default to 0*/
	data->als_poll_delay = 500;	/* default to 500ms */
	data->als_atime_index = APDS993X_ALS_RES_37888;	/* 100ms ATIME */
	data->als_again_index = APDS993X_ALS_GAIN_1X;	/* 8x AGAIN */
	data->als_reduce = 0;	/* no ALS 6x reduction*/
	data->als_function_on = false;
	data->pdrive = pdata->pdrive;
	data->pgain = pdata->prox_gain;
	data->ps_crosstalk_max = pdata->ps_crosstalk_max;

	if (apds993x_cross_talk_val > 0 && apds993x_cross_talk_val < 1000) {
		data->cross_talk = apds993x_cross_talk_val;
	} else {
		/*
		 * default value: Get the cross-talk value from the devicetree.
		 * This value is saved during the cross-talk calibration
		 */
		data->cross_talk = pdata->cross_talk;
	}

    /*5.verify chip*/
	err = apds993x_check_chip_id(client);
	if (err < 0) {
		dev_err(&client->dev, "Not a valid chip ID.\n");
		goto exit_check_chip;
	}
	/*6. Initialize the APDS993X chip */
	err = apds993x_init_device(client);
	if (err < 0) {
		dev_err(&client->dev, "Failed to init apds993x.\n");
		goto exit_init_device;
	}

    /*7. Establish data channel  -- input device*/
	err = als_input_init(data);
	if (err < 0) {
		dev_err(&client->dev, "als_input_init error!!\n");
		goto exit_unregister_input_als;
	}

	err = ps_input_init(data);
	if (err < 0) {
		dev_err(&client->dev, "ps_input_init error!!\n");
		goto exit_unregister_input_ps;
	}

	/*8. Register  class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = apds993x_als_set_enable;
	data->als_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&data->input_dev_als->dev, &data->als_cdev);
	if (err) {
		dev_err(&client->dev, "Unable to register to sensors class.\n");
		goto exit_unregister_als_class;
    }

	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = apds993x_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&data->input_dev_ps->dev, &data->ps_cdev);
	if (err) {
		dev_err(&client->dev, "Unable to register to sensors class.\n");
		goto exit_unregister_ps_class;
	}

    /*9. Register sysfs hooks */
	err = sysfs_create_group(&((data->ps_cdev.dev)->kobj),
							&apds993x_prox_attr_group);
	if (err) {
		dev_err(&client->dev, "Unable to creat sysfs ps.\n");
		goto exit_free_sysfs_ps;
		}

	/*10. request irq */
	if (data->irq) {
		err = request_irq(data->irq,
					apds993x_interrupt,
					IRQF_TRIGGER_FALLING,
					APDS993X_DRV_NAME,
					(void *)client);
		if (err < 0) {
			dev_err(&client->dev, "Could not allocate APDS993X_INT !\n");
			goto exit_free_irq;
		}
		irq_set_irq_wake(data->client->irq, 1);
	}

	/*11. wake lock & queues */
	mutex_init(&data->op_mutex);
	wake_lock_init(&(data->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
	INIT_DELAYED_WORK(&data->dwork, apds993x_work_handler);
	INIT_DELAYED_WORK(&data->als_dwork, apds993x_als_polling_work_handler);

#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
	productinfo_register(PRODUCTINFO_SENSOR_ALPS_ID,
				"apds9930", "avago");
#endif /* CONFIG_HISENSE_PRODUCT_DEVINFO */
	printk("%s:apds9930 probe ok!! Support ver:%s \n", __func__, DRIVER_VERSION);
	return 0;

exit_free_irq:
    sysfs_remove_group(&((data->ps_cdev.dev)->kobj), &apds993x_prox_attr_group);
exit_free_sysfs_ps:
    sensors_classdev_unregister(&data->ps_cdev);
exit_unregister_ps_class:
	sensors_classdev_unregister(&data->als_cdev);
exit_unregister_als_class:
	input_unregister_device(data->input_dev_ps);
exit_unregister_input_ps:
    input_unregister_device(data->input_dev_als);
exit_unregister_input_als:
exit_init_device:
exit_check_chip:
	if (pdata->power_on)
		pdata->power_on(false);
exit_pdata_power_on:
	if (pdata->exit)
		pdata->exit();
exit_pdata_init:
exit_kfree:
	kfree(data);
	pdev_data = NULL;
exit:
	return err;
}

static int apds993x_remove(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	struct apds993x_platform_data *pdata = data->platform_data;

	/* Power down the device */
	apds993x_set_enable(client, 0);
	sysfs_remove_group(&((data->ps_cdev.dev)->kobj), &apds993x_prox_attr_group);
	sensors_classdev_unregister(&data->ps_cdev);

	sensors_classdev_unregister(&data->als_cdev);

	free_irq(client->irq, data);

	if (pdata->power_on)
		pdata->power_on(false);

	if (pdata->exit)
		pdata->exit();

	kfree(data);
	pdev_data = NULL;

	return 0;
}

static const struct i2c_device_id apds993x_id[] = {
	{ "apds993x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds993x_id);

static struct of_device_id apds993X_match_table[] = {
	{ .compatible = "avago,apds9930",},
	{ .compatible = "avago,apds9900",},
	{ },
};

static const struct dev_pm_ops apds993x_pm_ops = {
	.suspend	= apds993x_suspend,
	.resume 	= apds993x_resume,
};

static struct i2c_driver apds993x_driver = {
	.driver = {
		.name   = APDS993X_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = apds993X_match_table,
		.pm = &apds993x_pm_ops,
	},
	.probe  = apds993x_probe,
	.remove = apds993x_remove,
	.id_table = apds993x_id,
};

static int __init apds993x_init(void)
{
	apds993x_workqueue = create_freezable_workqueue("proximity_als");
	if (!apds993x_workqueue) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	return i2c_add_driver(&apds993x_driver);
}

static void __exit apds993x_exit(void)
{
	if (apds993x_workqueue)
		destroy_workqueue(apds993x_workqueue);
	i2c_del_driver(&apds993x_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS993X ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds993x_init);
module_exit(apds993x_exit);
