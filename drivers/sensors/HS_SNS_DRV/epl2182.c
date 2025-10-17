/* drivers/i2c/chips/epl2182.c - light and proxmity sensors driver
 * Copyright (C) 2011 ELAN Corporation.
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

#include <linux/hrtimer.h>
#include <linux/timer.h>
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
#include <asm/uaccess.h>
//#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/productinfo.h>

/******************************************************************************
 * configuration
*******************************************************************************/
#define ELAN_CHIP_NAME	"EPL2182"

#define REG_0			0X00
#define REG_1			0X01
#define REG_2			0X02
#define REG_3			0X03
#define REG_4			0X04
#define REG_5			0X05
#define REG_6			0X06
#define REG_7			0X07
#define REG_8			0X08
#define REG_9			0X09
#define REG_10			0X0A
#define REG_11			0X0B
#define REG_12			0X0C
#define REG_13			0X0D
#define REG_14			0X0E
#define REG_15			0X0F
#define REG_16			0X10
#define REG_17			0X11
#define REG_18			0X12
#define REG_19			0X13
#define REG_20			0X14
#define REG_21			0X15
	
#define W_SINGLE_BYTE	0X00
#define W_TWO_BYTE		0X01
#define W_THREE_BYTE	0X02
#define W_FOUR_BYTE		0X03
#define W_FIVE_BYTE		0X04
#define W_SIX_BYTE		0X05
#define W_SEVEN_BYTE	0X06
#define W_EIGHT_BYTE	0X07
	
#define R_SINGLE_BYTE	0X00
#define R_TWO_BYTE		0X01
#define R_THREE_BYTE	0X02
#define R_FOUR_BYTE		0X03
#define R_FIVE_BYTE		0X04
#define R_SIX_BYTE		0X05
#define R_SEVEN_BYTE	0X06
#define R_EIGHT_BYTE	0X07
	
#define EPL_SENSING_1_TIME		(0 << 5)
#define EPL_SENSING_2_TIME		(1 << 5)
#define EPL_SENSING_4_TIME		(2 << 5)
#define EPL_SENSING_8_TIME		(3 << 5)
#define EPL_SENSING_16_TIME		(4 << 5)
#define EPL_SENSING_32_TIME		(5 << 5)
#define EPL_SENSING_64_TIME		(6 << 5)
#define EPL_SENSING_128_TIME	(7 << 5)

#define EPL_C_SENSING_MODE		(0 << 4)
#define EPL_S_SENSING_MODE		(1 << 4)

#define EPL_ALS_MODE			(0 << 2)
#define EPL_PS_MODE				(1 << 2)
#define EPL_TEMP_MODE 			(2 << 2)

#define EPL_H_GAIN			(0)
#define EPL_M_GAIN			(1)
#define EPL_L_GAIN			(3)
#define EPL_AUTO_GAIN		(2)

#define EPL_8BIT_ADC		0
#define EPL_10BIT_ADC		1
#define EPL_12BIT_ADC		2
#define EPL_14BIT_ADC		3

#define EPL_C_RESET			0x00
#define EPL_C_START_RUN		0x04
#define EPL_C_P_UP			0x04
#define EPL_C_P_DOWN		0x06
#define EPL_DATA_LOCK_ONLY	0x01
#define EPL_DATA_LOCK		0x05
#define EPL_DATA_UNLOCK		0x04
	
#define EPL_GO_MID				0x3E
#define EPL_GO_LOW				0x3E
	
#define EPL_DRIVE_60MA          	(0<<4)
#define EPL_DRIVE_120MA         	(1<<4)

#define EPL_INT_BINARY			0
#define EPL_INT_DISABLE			2
#define EPL_INT_ACTIVE_LOW		3
#define EPL_INT_FRAME_ENABLE		4
	
#define EPL_PST_1_TIME		(0 << 2)
#define EPL_PST_4_TIME		(1 << 2)
#define EPL_PST_8_TIME		(2 << 2)
#define EPL_PST_16_TIME		(3 << 2)
	
//ALS integration time
#define EPL_ALS_INTT_8     (1)
#define EPL_ALS_INTT_16    (2)
#define EPL_ALS_INTT_32    (3)
#define EPL_ALS_INTT_64    (4)
#define EPL_ALS_INTT_128   (5)
#define EPL_ALS_INTT_256   (6)
#define EPL_ALS_INTT_512   (7)
#define EPL_ALS_INTT_640   (8)
#define EPL_ALS_INTT_768   (9)
#define EPL_ALS_INTT_1024  (10)
#define EPL_ALS_INTT_2048  (11)
#define EPL_ALS_INTT_4096  (12)
#define EPL_ALS_INTT_6144  (13)
#define EPL_ALS_INTT_8192  (14)
#define EPL_ALS_INTT_10240 (15)

#define LUX_PER_COUNT			2100		// 660 = 1.1*0.6*1000

typedef enum {
    CMC_MODE_ALS    = 0x00,
    CMC_MODE_PS     = 0x10,
}CMC_MODE;

#define PACKAGE_SIZE		8
#define I2C_RETRY_COUNT		10

//ELAN Robert modify ePL2182 ps integration time at 2014/5/19
#define PS_INTT				(4 << 4)

#define ALS_INTT			(6 << 4)	//5-8

#define NEAR_CODE	1
#define FAR_CODE	0

static void psensor_delay_work_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(epl_sensor_irq_work, psensor_delay_work_handler);

static void psensor_poll_work_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(ps_poll_dwork, psensor_poll_work_handler);

static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);

#define PS_DELAY			50
#define ALS_DELAY			55

/* POWER SUPPLY VOLTAGE RANGE */
#define EPLD2182_VDD_MIN_UV  2000000
#define EPLD2182_VDD_MAX_UV  3300000
#define EPLD2182_VIO_MIN_UV  1750000
#define EPLD2182_VIO_MAX_UV  1950000

/* primitive raw data from I2C */
struct epl_raw_data {
    uint8_t		raw_bytes[PACKAGE_SIZE];
    uint16_t	ps_int_state;
    uint16_t	ps_ch1_raw;
    uint16_t	als_ch1_raw;
    uint16_t	als_lux;
};

struct elan_epl_data {
	struct i2c_client *client;
	struct input_dev *als_input_dev;
	struct sensors_classdev als_cdev;
	int enable_lflag;
	uint32_t last_lux;

	struct input_dev *ps_input_dev;
	struct sensors_classdev ps_cdev;
	struct workqueue_struct *epl_wq;
	int enable_pflag;
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;
	bool high_ps_drive;

	struct epl_raw_data gRawData;

	uint16_t ps_canc;
	uint16_t last_ps_canc;
	uint16_t ps_crosstalk_min;
	uint16_t ps_crosstalk_max;
	uint16_t als_factor;

	int intr_pin;
	int irq;

	struct regulator *vdd;
	struct regulator *vio;

	struct wake_lock ps_wake_lock;
	struct mutex operation_mode_lock;
};

struct elan_epl_data *epld = NULL;

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "ELAN",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65536.0",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "ELAN",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
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
	.sensors_flush = NULL,
};

static int elan_sensor_I2C_Write(struct i2c_client *client,
	uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t *data)
{
    uint8_t buffer[8] = {0};
    int ret = 0;
    int retry = 0, val = 0;
	int i = 0;

	if (txbyte > 9) {
		dev_err(&client->dev,
			"%s: error: most write 8 bytes\n", __func__);
		return -EINVAL;
	}

	buffer[0] = (regaddr << 3) | bytecount ;
	for (i = 1; i < txbyte; i++) {
		buffer[i] = data[i - 1];
	}

    for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		ret = i2c_master_send(client, buffer, txbyte);
		if (ret == txbyte) {
			dev_dbg(&client->dev,
				"%s(): sent bytes success\n", __func__);
            break;
        }

        val = gpio_get_value(epld->intr_pin);
        dev_err(&client->dev,
			"%s: INTERRUPT GPIO val = %d\n", __func__, val);
        mdelay(10);
    }

    if (retry >= I2C_RETRY_COUNT) {
        dev_err(&client->dev,
			"%s: i2c write retry over %d times\n", __func__, I2C_RETRY_COUNT);
        return -EIO;
    }

    return (ret == txbyte ? 0 : ret);
}

static int elan_sensor_I2C_Read(struct i2c_client *client, uint8_t rxbyte)
{
    uint8_t buffer[8] = {0};
    int ret = 0, i = 0;
    int retry = 0, val = 0;

	if (rxbyte > 8) {
		dev_err(&epld->client->dev,
			"%s: error: must read 8 bytes\n", __func__);
		return -EINVAL;
	}

    for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
        ret = i2c_master_recv(client, buffer, rxbyte);
        if (ret == rxbyte)
            break;

        val = gpio_get_value(epld->intr_pin);
        dev_err(&epld->client->dev,
			"%s: INTERRUPT GPIO val = %d\n", __func__, val);
        mdelay(10);
    }

    if (retry >= I2C_RETRY_COUNT) {
        dev_err(&epld->client->dev,
			"%s: i2c read retry over %d times\n", __func__, I2C_RETRY_COUNT);
        return -EIO;
    }

    for (i = 0; i < rxbyte; i++) {
        epld->gRawData.raw_bytes[i] = buffer[i];
    }

    return (ret == rxbyte ? 0 : ret);
}

static int elan_reg0_op(struct elan_epl_data *epld,
					uint8_t mode_code, uint8_t gain_code)
{
	uint8_t regdata[2] = {0, 0};
	regdata[0] = EPL_SENSING_8_TIME |
				EPL_C_SENSING_MODE |
				mode_code |
				gain_code ;
	return
		elan_sensor_I2C_Write(epld->client,
			REG_0, W_SINGLE_BYTE, 0X02, regdata) ?
		-EIO : 0;
}

static int elan_reg1_op(struct elan_epl_data *epld,
		uint8_t it_code, uint8_t pers_code, uint8_t adcr_code)
{
	uint8_t regdata[2] = {0, 0};
	regdata[0] = it_code | pers_code | adcr_code;
	return
		elan_sensor_I2C_Write(epld->client,
			REG_1, W_SINGLE_BYTE, 0X02, regdata) ?
		-EIO : 0;
}

static int elan_update_ps_close_thd(struct elan_epl_data *epld, uint16_t close_thd)
{
	uint8_t regdata[2] = {0};
	
	regdata[0] = (uint8_t)((close_thd + epld->ps_canc) & 0x00ff);
	regdata[1] = (uint8_t)((close_thd + epld->ps_canc) >> 8);

	return
		elan_sensor_I2C_Write(epld->client, REG_2, W_TWO_BYTE, 0x03, regdata) ?
		-EIO : 0;
}

static int elan_update_ps_away_thd(struct elan_epl_data *epld, uint16_t away_thd)
{
	uint8_t regdata[2] = {0};
	regdata[0] = (uint8_t)((away_thd + epld->ps_canc) & 0x00ff);
	regdata[1] = (uint8_t)((away_thd + epld->ps_canc) >> 8);
	return
		elan_sensor_I2C_Write(epld->client, REG_4, W_TWO_BYTE, 0x03, regdata) ?
		-EIO : 0;
}

static int elan_soft_reset(struct elan_epl_data *epld)
{
	uint8_t regdata[2] = {EPL_C_RESET, 0};
	int ret = 0;

	ret = elan_sensor_I2C_Write(epld->client,
		REG_7, W_SINGLE_BYTE, 0X02, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_7 to reset failed! ret = %d\n",
			__func__, ret);
		return ret;
	}

	mdelay(1);
	
	regdata[0] = EPL_C_START_RUN;
	ret = elan_sensor_I2C_Write(epld->client,
		REG_7, W_SINGLE_BYTE, 0x02, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_7 to start_to_run failed! ret = %d\n",
			__func__, ret);
	}
	
	return ret;
}

static int elan_goto_idle(struct i2c_client *client)
{
	uint8_t regdata[2] = {EPL_C_P_DOWN, 0};
	int ret = elan_sensor_I2C_Write(client,
		REG_7, W_SINGLE_BYTE, 0x02, regdata);
	if (ret) {
		dev_err(&client->dev,
			"%s: Write REG_7 to idle failed! ret = %d\n", __func__, ret);
	}
	mdelay(5);
	return ret;
}

static int elan_reg9_op(struct elan_epl_data *epld,
		uint8_t ps_drv_code, uint8_t int_mode)
{
	uint8_t regdata[2] = {0, 0};
	regdata[0] = ps_drv_code | int_mode;
	return
		elan_sensor_I2C_Write(epld->client,
			REG_9, W_SINGLE_BYTE, 0X02, regdata) ?
		-EIO : 0;
}

static int elan_set_als_range(struct elan_epl_data *epld)
{
	uint8_t regdata[2] = {0};
	int ret = 0;

	regdata[0] = EPL_GO_MID;
	ret = elan_sensor_I2C_Write(epld->client,
		REG_10, W_SINGLE_BYTE, 0x02, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_10 failed! ret = %d\n",
			__func__, ret);
		return ret;
	}
	
	regdata[0] = EPL_GO_LOW;
	ret = elan_sensor_I2C_Write(epld->client,
		REG_11, W_SINGLE_BYTE, 0x02, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_11 failed! ret = %d\n",
			__func__, ret);
	}
	return ret;
}

static void elan_ps_report_dist(struct elan_epl_data *epld, int dist_code)
{
	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(epld->ps_input_dev, ABS_DISTANCE, dist_code ? 1000 : 1);
	input_report_abs(epld->ps_input_dev, ABS_DISTANCE, dist_code ? 1023 : 0);
	input_event(epld->ps_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(epld->ps_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	input_sync(epld->ps_input_dev);	
}

static int elan_ps_get_data(struct elan_epl_data *epld)
{
	uint8_t regdata[2] = {0};
	int ret = elan_sensor_I2C_Write(epld->client,
		REG_16, R_TWO_BYTE, 0x01, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_16 failed! ret = %d\n", __func__, ret);
		return ret;
	}

	ret = elan_sensor_I2C_Read(epld->client, 2);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Read REG_16 failed! ret = %d\n", __func__, ret);
		return ret;
	}
	epld->gRawData.ps_ch1_raw =
		(epld->gRawData.raw_bytes[1] << 8) | epld->gRawData.raw_bytes[0];

	return ret;	
}

static void elan_ps_report_data(struct elan_epl_data *epld)
{
	elan_ps_report_dist(epld, epld->gRawData.ps_int_state);

	if (epld->gRawData.ps_int_state == FAR_CODE) {
		dev_err(&epld->client->dev,
			"%s: PS_STATE = NEAR_TO_FAR \n",
			__func__);
		if (!wake_lock_active(&epld->ps_wake_lock)) {
			dev_err(&epld->client->dev,
				"%s: wake_lock PROX_NEAR_TO_FAR_WLOCK not be locked, and lock it!\n",
				__func__);
			wake_lock_timeout(&epld->ps_wake_lock, HZ);
		} else {
			dev_err(&epld->client->dev,
				"%s: wake_lock PROX_NEAR_TO_FAR_WLOCK be locked, do nothing!\n",
				__func__);
		}
	} else {
		dev_err(&epld->client->dev,
			"%s: PS_STATE = FAR_TO_NEAR \n",
			__func__);
	}
}

static void psensor_delay_work_handler(struct work_struct *work)
{
	int mode = 0;
	int ret = 0;
	uint8_t regdata[2] = {0};


	regdata[0] = EPL_DATA_LOCK;
	ret = elan_sensor_I2C_Write(epld->client,
		REG_7, W_SINGLE_BYTE, 0x02, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_7 to data_lock failed! ret = %d\n", __func__, ret);
	}

	regdata[0] = 0;
	ret = elan_sensor_I2C_Write(epld->client,
		REG_13, R_SINGLE_BYTE, 0x01, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_13 failed! ret = %d\n", __func__, ret);
	}
	
	ret = elan_sensor_I2C_Read(epld->client, 1);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Read REG_13 failed! ret = %d\n", __func__, ret);
	}

	mode = epld->gRawData.raw_bytes[0] & (3 << 4);

	dev_err(&epld->client->dev,
		"%s: mode = %d, enable_pflag = %d",
		__func__, mode, epld->enable_pflag);

	// 0x10 is ps mode
	if (mode == CMC_MODE_PS) {
		epld->gRawData.ps_int_state = ((epld->gRawData.raw_bytes[0] & 0x04) >> 2);
		elan_ps_report_data(epld);
		/*
		ret = elan_ps_get_data(epld);
		if (ret) {
			dev_err(&epld->client->dev,
				"%s: get ps data failed! ret = %d\n", __func__, ret);
		}
		*/
	} else {
		dev_err(&epld->client->dev,
			"%s: error: interrupt in als\n", __func__);
	}

	regdata[0] = EPL_DATA_UNLOCK;
	ret = elan_sensor_I2C_Write(epld->client,
		REG_7, W_SINGLE_BYTE, 0x02, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_7 to data_unlock failed! ret = %d\n",
			__func__, ret);
	}
	
	return;
}

static irqreturn_t elan_ps_irq_handle(int irqNo, void *handle)
{
	if (epld->enable_pflag) {
		cancel_delayed_work(&epl_sensor_irq_work);
		queue_delayed_work(epld->epl_wq, &epl_sensor_irq_work, 0);
	} else
		dev_err(&epld->client->dev, "%s(): ps not enabled\n", __func__);

    return IRQ_HANDLED;
}

/* syfs interface to function test */
static ssize_t ps_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int16_t p_data = 0;
	if (epld->enable_pflag == 0) {
		return sprintf(buf, "PS function not enabled!\n");
	}
	
	elan_ps_get_data(epld);
	p_data = epld->gRawData.ps_ch1_raw - epld->ps_canc;
	return sprintf(buf, "%d\n", p_data < 0 ? 0 : p_data);
}
static DEVICE_ATTR(ps_data, 0440, ps_data_show, NULL);

static ssize_t ps_thd_close_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!epld) {
		printk("%s: epl_data is null!!\n", __func__);
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", epld->ps_close_thd_set + epld->ps_canc);
}

static ssize_t ps_thd_close_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{	
	int code = 0;	
		
	sscanf(buf, "%d", &code);

	if (!epld->enable_pflag) {	
		dev_err(&epld->client->dev,
			"Not permitting to set PS close thd while PS function off!\n");
		return -EPERM;
	}
		
	if ((code > 0) && (code < 65535)) {
		epld->ps_close_thd_set = (uint16_t)code;
		if (elan_update_ps_close_thd(epld, (uint16_t)code))
			return -EIO;	
	} else {
		dev_err(&epld->client->dev,
			"%s: It is an illegal para for PS close thd: %d!\n",
			__func__, code);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_thd_close,
		0640, ps_thd_close_show, ps_thd_close_store);

static ssize_t ps_thd_away_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!epld) {
		printk("%s: epl_data is null!!\n", __func__);
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", epld->ps_away_thd_set + epld->ps_canc);
}

static ssize_t ps_thd_away_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{	
	int code = 0;
		
	sscanf(buf, "%d", &code);

	if (!epld->enable_pflag) {	
		dev_err(&epld->client->dev,
			"Not permitting to set PS away thd while PS function off!\n");
		return -EPERM;
	}
		
	if ((code > 0) && (code < 65536)) {	
		epld->ps_away_thd_set = (uint16_t)code;
		if (elan_update_ps_away_thd(epld, (uint16_t)code))
			return -EIO;
	} else {
		dev_err(&epld->client->dev,
			"%s: It is an illegal para for PS close thd: %d!\n", __func__, code);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_thd_away,
		0640, ps_thd_away_show, ps_thd_away_store);

static ssize_t ps_crosstalk_maxthd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!epld) {
		printk("%s: epl_data is null!!\n", __func__);
		return -EINVAL;
	}
	return snprintf(buf, 10, "%d\n",
		epld->ps_crosstalk_max);
}
static ssize_t ps_crosstalk_maxthd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	sscanf(buf, "%d", &code);
	if ((code > 0) && (code < 65535)) {
		epld->ps_crosstalk_max= (uint16_t)code;
	} else {
		dev_err(&epld->client->dev,
			"%s(): It is an illegal para, %d!", __func__, code);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_crosstalk_maxthd,
		0640, ps_crosstalk_maxthd_show,  ps_crosstalk_maxthd_store);

static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!epld) {
		printk("%s: epl_data is null!!\n", __func__);
		return -EINVAL;
	}
	return snprintf(buf, 10, "%d\n",epld->ps_canc);
}
static DEVICE_ATTR(ps_canc, 0440, ps_canc_show, NULL);


static struct attribute *elan_ps_attributes[] = {
	&dev_attr_ps_data.attr,
	&dev_attr_ps_thd_close.attr,
	&dev_attr_ps_thd_away.attr,
	&dev_attr_ps_crosstalk_maxthd.attr,
	&dev_attr_ps_canc.attr,
	NULL,
};

static struct attribute_group elan_attr_ps_group = {
    .attrs = elan_ps_attributes,
};

static int elan_cross_talk_calibration(struct elan_epl_data *epld)
{
	int ret = 0;
	dev_err(&epld->client->dev,
			"%s: --- Proximity sensor Calibration --- \n", __func__);

	/* Prepare to scheduling a calib procedure */

	if (elan_soft_reset(epld))
		return -EIO;

	mdelay(PS_DELAY);

	ret = elan_ps_get_data(epld);
	if (ret) {
		dev_err(&epld->client->dev, "%s: get ps data failed! ret = %d\n",
			__func__, ret);
		epld->ps_canc = 0;
	} else {
		epld->ps_canc = epld->gRawData.ps_ch1_raw; 
		if (epld->ps_canc <= epld->ps_crosstalk_max) {
			/*
			 * It can be caliberated.
			 */
			if (0 != epld->ps_crosstalk_min) {
				/* calib crosstalk to < 10 */
				if (epld->ps_canc > epld->ps_crosstalk_min) {
					/* do calib */
					epld->ps_canc -= epld->ps_crosstalk_min;
				} else {
					/*
					 * The crosstalk is good enough,
					 * but should update the last_ps_canc
					 */
					epld->ps_canc = 0;
				}
			}
			epld->last_ps_canc = epld->ps_canc;
			//ret = 0;
		} else {
			/*
			 * crosstalk is too large to calib,
			 * so try to use the last_ps_canc at first time!
			 */
			dev_err(&epld->client->dev,
				"%s(): corsstalk is %d it's too large !\n",
				__func__, epld->ps_canc);
			
			if (0xffff == epld->last_ps_canc) {
				/*
				 * It is the very 1st time use prox after power on,
				 * and got a high crosstalk. Then let it be...
				 */
				epld->ps_canc = 0;
			} else {
				/*
				 * It is not the first time use prox after power on
				 * and "last_ps_canc" had been updated once upon.
				 */
				epld->ps_canc = epld->last_ps_canc;
			}
			//ret = 0;
		}
	}

	/*ps power on bit clear */
	dev_err(&epld->client->dev,
				"%s: The current corsstalk = %d \n", __func__, epld->ps_canc);
	return ret;
}

static void psensor_poll_work_handler(struct work_struct *work)
{
	int ret = 0;
	dev_err(&epld->client->dev,
		"%s: --- Proximity First Calibration --- \n", __func__);
	ret = elan_reg9_op(epld,
			epld->high_ps_drive ? EPL_DRIVE_120MA : EPL_DRIVE_60MA,
			EPL_INT_DISABLE);
	if (ret){
		dev_err(&epld->client->dev,
			"%s: reg9_op failed. ret = %d\n", __func__, ret);
		goto exit_to_idle;
	}
	ret = elan_reg0_op(epld, EPL_PS_MODE, EPL_M_GAIN);
	if (ret){
		dev_err(&epld->client->dev,
			"%s: reg0_op failed. ret = %d\n", __func__, ret);
		goto exit_to_idle;
	}
	ret = elan_reg1_op(epld, PS_INTT, EPL_PST_1_TIME, EPL_10BIT_ADC);
	if (ret){
		dev_err(&epld->client->dev,
			"%s: reg1_op failed. ret = %d\n", __func__, ret);
		goto exit_to_idle;
	}
	ret = elan_cross_talk_calibration(epld);
	if (ret){
		dev_err(&epld->client->dev,
			"%s: EPL2182 first cali failed. ret = %d\n", __func__, ret);
		goto exit_to_idle;
	}
	else
		dev_err(&epld->client->dev,
			"%s: EPL2182 first cali success.\n", __func__);

	ret = elan_reg9_op(epld,
			epld->high_ps_drive ? EPL_DRIVE_120MA : EPL_DRIVE_60MA,
			EPL_INT_ACTIVE_LOW);
	if (ret){
		dev_err(&epld->client->dev,
			"%s: reg9_op failed. ret = %d\n", __func__, ret);
		goto exit_to_idle;
	}
	ret = elan_soft_reset(epld);
	if (ret){
		dev_err(&epld->client->dev,
			"%s: soft_reset failed. ret = %d\n", __func__, ret);
	}
exit_to_idle:
	ret = elan_goto_idle(epld->client);
	if (ret)
		dev_err(&epld->client->dev,
			"%s: set to idle failed. ret = %d\n", __func__, ret);
	return;
}


static int elan_sensor_psensor_enable(struct elan_epl_data *epld)
{
	int ret = 0;
	dev_err(&epld->client->dev,
		"%s: --- Proximity sensor Enable --- \n", __func__);
	epld->ps_canc = 0;
	if (elan_reg9_op(epld,
			epld->high_ps_drive ? EPL_DRIVE_120MA : EPL_DRIVE_60MA,
			EPL_INT_DISABLE))
		return -EIO;
	if (elan_update_ps_away_thd(epld, 0))
		return -EIO;
	if (elan_update_ps_close_thd(epld, 0xFFFF))
		return -EIO;
	if (elan_reg0_op(epld, EPL_PS_MODE, EPL_M_GAIN))
		return -EIO;
	if (elan_reg1_op(epld, PS_INTT, EPL_PST_1_TIME, EPL_10BIT_ADC))
		return -EIO;
	if (elan_cross_talk_calibration(epld))
		return -EIO;
	if (!((0xffff == epld->last_ps_canc) && (0 == epld->ps_canc))) {
		if (elan_update_ps_away_thd(epld, epld->ps_away_thd_set))
			return -EIO;
		if (elan_update_ps_close_thd(epld, epld->ps_close_thd_set))
			return -EIO;
	}
	if (elan_reg9_op(epld,
			epld->high_ps_drive ? EPL_DRIVE_120MA : EPL_DRIVE_60MA,
			EPL_INT_ACTIVE_LOW))
		return -EIO;
	if (elan_soft_reset(epld))
		return -EIO;

	mdelay(PS_DELAY);

	if (elan_ps_get_data(epld)) {
		dev_err(&epld->client->dev, "%s: get ps data failed! ret = %d\n",
			__func__, ret);
		return -EIO;
	}

	if (epld->gRawData.ps_ch1_raw < (epld->ps_close_thd_set + epld->ps_canc))
		elan_ps_report_dist(epld, FAR_CODE);
	else if(0xffff != epld->last_ps_canc)
		elan_ps_report_dist(epld, NEAR_CODE);

	return 0;
}

static int elan_sensor_lsensor_enable(struct elan_epl_data *epld)
{
	if (elan_reg9_op(epld,
			epld->high_ps_drive ? EPL_DRIVE_120MA : EPL_DRIVE_60MA,
			EPL_INT_DISABLE))
		return -EIO;

	if (elan_reg0_op(epld, EPL_ALS_MODE, EPL_AUTO_GAIN))
		return -EIO;
	if (elan_reg1_op(epld, ALS_INTT, EPL_PST_1_TIME, EPL_10BIT_ADC))
		return -EIO;

	if (elan_set_als_range(epld))
		return -EIO;
	
	if (elan_soft_reset(epld))
		return -EIO;
	
	mdelay(ALS_DELAY);
	return 0;
}

static int elan_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct elan_epl_data *epld =
		container_of(sensors_cdev, struct elan_epl_data, ps_cdev);

	int ret = 0;

	if ((enable != 0) && (enable != 1)) {
		dev_err(&epld->client->dev,
			"%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	disable_irq(epld->irq);
	mutex_lock(&epld->operation_mode_lock);
	if ((1 == enable) && (0 == epld->enable_pflag)) {
		if (1 == epld->enable_lflag) {
			cancel_delayed_work_sync(&polling_work);
		}

		ret = elan_sensor_psensor_enable(epld);
		if (ret) {
			dev_err(&epld->client->dev, "%s: set to %d failed. ret = %d\n",
				__func__, enable, ret);
			goto exit_ps_en;
		}
		epld->enable_pflag = 1;
	} else if ((0 == enable) && (1 == epld->enable_pflag)) {
		if (0 == epld->enable_lflag) {
			ret = elan_goto_idle(epld->client);
			if (ret) {
				dev_err(&epld->client->dev,
					"%s: set to idle failed. ret = %d\n", __func__, ret);
				goto exit_ps_en;
			}
			epld->enable_pflag = 0;
		} else {
			ret = elan_sensor_lsensor_enable(epld);
			if (ret) {
				dev_err(&epld->client->dev,
					"%s: set als enable failed, ret = %d\n", __func__, ret);
				goto exit_ps_en;
			} else {
				epld->enable_pflag = 0;
				queue_delayed_work(epld->epl_wq,
					&polling_work, msecs_to_jiffies(0));
				elan_ps_report_dist(epld, FAR_CODE);
			}
		}
	}
	dev_err(&epld->client->dev, "%s(): PS_EN = %d\n", __func__, enable);
exit_ps_en:
	mutex_unlock(&epld->operation_mode_lock);
	enable_irq(epld->irq);
	return ret;
}

static int elan_ps_set_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec)
{
	return 0;
}

static void elan_als_report_data(struct elan_epl_data *epld)
{
	int ret = 0;
	uint8_t regdata[2] = {0, 0};
	uint32_t lux = 0;
	ktime_t timestamp;

	ret = elan_sensor_I2C_Write(epld->client,
		REG_16, R_TWO_BYTE, 0x01, regdata);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Write REG_16 failed! ret = %d\n", __func__, ret);
		return;
	}
	
	ret = elan_sensor_I2C_Read(epld->client, 2);
	if (ret) {
		dev_err(&epld->client->dev,
			"%s: Read REG_16 failed! ret = %d\n", __func__, ret);
		return;
	}
	
	epld->gRawData.als_ch1_raw =
		(epld->gRawData.raw_bytes[1] << 8) | epld->gRawData.raw_bytes[0];

	lux = (epld->gRawData.als_ch1_raw * LUX_PER_COUNT) / 1000 * 15 / 100 * epld->als_factor / 100;
	if (lux > 19999)
		lux = 19999;

	if (lux == epld->last_lux) {
		lux++;
	}
	timestamp = ktime_get_boottime();
	input_report_abs(epld->als_input_dev, ABS_MISC, lux);
	input_event(epld->als_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(epld->als_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	input_sync(epld->als_input_dev);

	epld->last_lux = lux;
	dev_dbg(&epld->client->dev,
		"%s: ALS raw = %d, lux = %d\n",
		__func__, epld->gRawData.als_ch1_raw, lux);
}

static void polling_do_work(struct work_struct *work)
{
	if (!epld->enable_lflag) {
		dev_err(&epld->client->dev,
			"%s(): als not enabled, do not re-schedule ldwork!\n", __func__);
		return;
	}

	if (epld->enable_pflag) {
		dev_err(&epld->client->dev,
			"%s(): ps enabled, do not permit enable als!\n",__func__);
		return;
	}

	elan_als_report_data(epld);
	queue_delayed_work(epld->epl_wq,
		&polling_work, msecs_to_jiffies(500));
}

static int elan_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;

	if ((enable != 0) && (enable != 1)) {
		dev_err(&epld->client->dev,
			"%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	if ((1 == enable) && (0 == epld->enable_lflag)) {
		if (0 == epld->enable_pflag) {
			ret = elan_sensor_lsensor_enable(epld);
			if (ret) {
				dev_err(&epld->client->dev,
					"%s: set als enable failed, ret = %d\n", __func__, ret);
				return ret;
			}	
			queue_delayed_work(epld->epl_wq, &polling_work, 0);
		}
	} else if ((0 == enable) && (1 == epld->enable_lflag)) {
		if (0 == epld->enable_pflag) {
			ret = elan_goto_idle(epld->client);
			if(ret){
				dev_err(&epld->client->dev,
					"%s: set to idle failed. ret = %d\n", __func__, ret);
				return ret;
			}
			cancel_delayed_work_sync(&polling_work);
		}
	}

	epld->enable_lflag = enable;

	dev_err(&epld->client->dev, "%s(): ALS_EN = %d\n", __func__, enable);
	return ret;
}

static int elan_als_set_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec)
{
	return 0;
}

static int elan_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct elan_epl_data *epld =
		container_of(sensors_cdev, struct elan_epl_data, ps_cdev);

	if (epld->enable_pflag == 0) {
		printk("PS function not enabled!\n");
		return -1;
	}

	elan_ps_get_data(epld);

	if(epld->gRawData.ps_ch1_raw >= (epld->ps_close_thd_set + epld->ps_canc))
		elan_ps_report_dist(epld, NEAR_CODE);
	else
		elan_ps_report_dist(epld, FAR_CODE);

	return 0;
}

static int psensor_irq_init(struct elan_epl_data *epld)
{
	int err = 0;

	if (!gpio_is_valid(epld->intr_pin)) {
		dev_err(&epld->client->dev,
			"invalid interrupt pin: %d\n", epld->intr_pin);		
	}

	err = gpio_request(epld->intr_pin, "epl_irq_gpio");
	if (err < 0) {
		dev_err(&epld->client->dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, epld->intr_pin, err);
		return err;
	}

	err = gpio_direction_input(epld->intr_pin);
	if (err < 0) {
		dev_err(&epld->client->dev,
			"%s: fail to set gpio %d as input. ret = %d\n",
			__func__, epld->intr_pin, err);
		goto fail_free_intr_pin;
	}
	epld->irq = gpio_to_irq(epld->intr_pin);

	err = request_irq(epld->irq,
					elan_ps_irq_handle,
					IRQF_TRIGGER_FALLING,
					epld->client->dev.driver->name,
					epld);
	if (err) {
		dev_err(&epld->client->dev,
			"Request IRQ for cm36283 failed, ret = %d\n", err);
		goto fail_free_intr_pin;
	}

	err = enable_irq_wake(epld->irq);
	if (0 != err) {
		dev_err(&epld->client->dev,
			"enable_irq_wake failed %d\n", err);
		goto fail_set_irq_wake;
	}

	return err;
fail_set_irq_wake:
	free_irq(epld->irq, epld);
fail_free_intr_pin:
	gpio_free(epld->intr_pin);
	return err;
}

static int lightsensor_input_init(struct elan_epl_data *epld)
{
    int err = 0;

    epld->als_input_dev = input_allocate_device();
    if (!epld->als_input_dev) {
        dev_err(&epld->client->dev,
			"%s: could not allocate ls input device\n", __func__);
        return -ENOMEM;
    }
    epld->als_input_dev->name = "light";
    set_bit(EV_ABS, epld->als_input_dev->evbit);
    input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 65535, 0, 0);

    err = input_register_device(epld->als_input_dev);
    if (err < 0) {
        dev_err(&epld->client->dev,
			"%s: can not register ls input device. ret = %d\n", __func__, err);
        goto err_free_ls_input_device;
    }
    return 0;

err_free_ls_input_device:
    input_free_device(epld->als_input_dev);
    return err;
}

static int psensor_input_init(struct elan_epl_data *epld)
{
    int err = 0;

    epld->ps_input_dev = input_allocate_device();
    if (!epld->ps_input_dev){
        dev_err(&epld->client->dev,
			"%s: could not allocate ps input device\n", __func__);
        return -ENOMEM;
    }
    epld->ps_input_dev->name = "proximity";

    set_bit(EV_ABS, epld->ps_input_dev->evbit);
    input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1023, 0, 0);

    err = input_register_device(epld->ps_input_dev);
    if (err < 0) {
        dev_err(&epld->client->dev,
			"%s: could not register ps input device. ret = %d\n",
			__func__, err);
        goto err_free_ps_input_device;
    }

    return 0;

err_free_ps_input_device:
    input_free_device(epld->ps_input_dev);
    return err;
}

static int elan_regulator_on(struct elan_epl_data *epld, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(epld->vdd);
		if (rc) {
			dev_err(&epld->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(epld->vio);
		if (rc) {
			dev_err(&epld->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(epld->vdd);
		}
	} else {
		rc = regulator_enable(epld->vdd);
		if (rc) {
			dev_err(&epld->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(epld->vio);
		if (rc) {
			dev_err(&epld->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			rc = regulator_disable(epld->vdd);
		}
		mdelay(10);
	}

	return rc;
}

static int elan_regulator_configure(struct elan_epl_data *epld, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(epld->vdd) > 0)
			regulator_set_voltage(epld->vdd, 0,
				EPLD2182_VDD_MAX_UV);

		regulator_put(epld->vdd);

		if (regulator_count_voltages(epld->vio) > 0)
			regulator_set_voltage(epld->vio, 0,
				EPLD2182_VIO_MAX_UV);

		regulator_put(epld->vio);
	} else {
		epld->vdd = regulator_get(&epld->client->dev, "vdd");
		if (IS_ERR(epld->vdd)) {
			rc = PTR_ERR(epld->vdd);
			dev_err(&epld->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(epld->vdd) > 0) {
			rc = regulator_set_voltage(epld->vdd,
				EPLD2182_VDD_MIN_UV, EPLD2182_VDD_MAX_UV);
			if (rc) {
				dev_err(&epld->client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}

		epld->vio = regulator_get(&epld->client->dev, "vio");
		if (IS_ERR(epld->vio)) {
			rc = PTR_ERR(epld->vio);
			dev_err(&epld->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(epld->vio) > 0) {
			rc = regulator_set_voltage(epld->vio,
				EPLD2182_VIO_MIN_UV, EPLD2182_VIO_MAX_UV);
			if (rc) {
				dev_err(&epld->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(epld->vio);
reg_vdd_set:
	if (regulator_count_voltages(epld->vdd) > 0)
		regulator_set_voltage(epld->vdd, 0, EPLD2182_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(epld->vdd);
	return rc;
}

static int sensor_parse_dt(struct device *dev,
		struct elan_epl_data *epld)
{
	struct device_node *np = dev->of_node;
	int rc = 0;
	u32 temp_val = 0;

	/* irq gpio */
	rc = of_get_named_gpio_flags(np,
			"epl,irq-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read irq gpio. ret = %d\n", rc);
		return rc;
	}
	epld->intr_pin = rc;
	dev_err(dev, "epld->intr_pin = %d\n", epld->intr_pin);

	rc = of_property_read_u32(np, "epl,ps_close_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd_set. ret = %d\n", rc);
		return rc;
	} else {
		epld->ps_close_thd_set = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,ps_away_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd_set. ret = %d\n", rc);
		return rc;
	} else {
		epld->ps_away_thd_set = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,ps_crosstalk_min", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_crosstalk_min. ret = %d\n", rc);
		return rc;
	} else {
		epld->ps_crosstalk_min = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,ps_crosstalk_max", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_crosstalk_max. ret = %d\n", rc);
		return rc;
	} else {
		epld->ps_crosstalk_max = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,als_factor", &temp_val);
	if (rc) {
		epld->als_factor = 100;
		dev_err(dev, "Unable to read als_factor. ret = %d\n", rc);
	} else {
		epld->als_factor = (uint16_t)temp_val;
	}

	return 0;
}

static int elan_sensor_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{
	int err = 0;
    printk("%s\n",__func__);

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	epld = kzalloc(sizeof(struct elan_epl_data), GFP_KERNEL);
	if (!epld)
		return -ENOMEM;

	epld->client = client;
	epld->irq = client->irq;
	i2c_set_clientdata(client, epld);

	/* 1. parse_dt */
	if (client->dev.of_node) {
		err = sensor_parse_dt(&client->dev, epld);
		if (err) {
			dev_err(&client->dev,"%s: sensor_parse_dt() err\n", __func__);
			goto err_parse_dt;
		}
	} else {
		dev_err(&client->dev, "No dts data\n");
		goto err_parse_dt;
	}

	/* 2. power on chip*/
	err = elan_regulator_configure(epld, true);
	if (err < 0) {
		dev_err(&client->dev,
			"unable to configure regulator. err = %d\n", err);
		goto err_sensor_regu_conf;
	}

	err = elan_regulator_on(epld, true);
	if (err < 0) {
		dev_err(&client->dev,
			"%s(): epl power switch error! err = %d\n", __func__, err);
		goto err_sensor_regu_power_on;
	}

	/* 3. chip verify */
	if ((i2c_smbus_read_byte_data(client, 0x98)) != 0x68) {
		err = -ENODEV;
		dev_err(&client->dev, "%s:elan ALS/PS sensor is failed.\n", __func__);
		goto err_chip_verify;
	}

	err = elan_goto_idle(epld->client);
	if (err < 0) {
		dev_err(&client->dev,
			"%s(): epl power switch error! err = %d\n", __func__, err);
		goto err_chip_verify;
	}

	epld->enable_lflag = 0;
	epld->enable_pflag = 0;
	epld->ps_canc = 0;
	/* for first time use prox when power on */
	epld->last_ps_canc = 0xffff;

	epld->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
	if (!epld->epl_wq) {
		dev_err(&client->dev, "can't create workqueue\n");
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	
	mutex_init(&epld->operation_mode_lock);

	/* 5. establish data channel  -- input device*/
	err = psensor_input_init(epld);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: psensor_input_init error! ret = %d\n", __func__, err);
		goto err_psensor_input_init;
	}

	err = lightsensor_input_init(epld);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: lightsensor_input_init error! ret = %d\n", __func__, err);
		goto err_lightsensor_input_init;
	}

	/* 6. request irq */
	err = psensor_irq_init(epld);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: psensor_irq_init error! ret = %d\n", __func__, err);
		goto err_psensor_irq_init;
	}

	/* 7. wake lock */
	wake_lock_init(&epld->ps_wake_lock, WAKE_LOCK_SUSPEND, "ps_wakelock");

	/* 8. establish control channel -- sysfs interface */
	epld->als_cdev = sensors_light_cdev;
	epld->als_cdev.sensors_enable = elan_als_set_enable;
	epld->als_cdev.sensors_poll_delay = elan_als_set_delay;
	err = sensors_classdev_register(&epld->als_input_dev->dev, &epld->als_cdev);
	if (err) {
		dev_err(&client->dev,
			"%s: Unable to register to sensors class: %d\n", __func__, err);
		goto err_register_als_classdev;
	}

	epld->ps_cdev = sensors_proximity_cdev;
	epld->ps_cdev.sensors_enable = elan_ps_set_enable;
	epld->ps_cdev.sensors_poll_delay = elan_ps_set_delay;
	epld->ps_cdev.sensors_flush = elan_ps_flush;
	err = sensors_classdev_register(&epld->ps_input_dev->dev, &epld->ps_cdev);
	if (err) {
		dev_err(&client->dev,
			"%s: Unable to register to sensors class: %d\n", __func__, err);
		goto err_register_ps_classdev;
	}

	err = sysfs_create_group(&epld->ps_cdev.dev->kobj, &elan_attr_ps_group);
	if (err !=0) {
		dev_err(&client->dev,
			"%s:create sysfs group error. ret = %d\n", __func__, err);
		goto err_create_ps_sysfs_group;
	}
	queue_delayed_work(epld->epl_wq,
						&ps_poll_dwork, msecs_to_jiffies(0));
	productinfo_register(PRODUCTINFO_SENSOR_ALPS_ID,
		"epl2182", "elan");

	printk("EPL2182 probe ok!!! \n");

	return err;

err_create_ps_sysfs_group:
	sensors_classdev_unregister(&epld->ps_cdev);
err_register_ps_classdev:
	sensors_classdev_unregister(&epld->als_cdev);
err_register_als_classdev:
err_psensor_irq_init:
	input_unregister_device(epld->als_input_dev);
	input_free_device(epld->als_input_dev);
err_lightsensor_input_init:
	input_unregister_device(epld->ps_input_dev);
	input_free_device(epld->ps_input_dev);
err_psensor_input_init:
	destroy_workqueue(epld->epl_wq);
err_create_singlethread_workqueue:
err_chip_verify:
	elan_regulator_on(epld, false);
err_sensor_regu_power_on:
	elan_regulator_configure(epld, false);
err_sensor_regu_conf:
err_parse_dt:
	kfree(epld);
	epld = NULL;
	dev_err(&client->dev, "%s: failed! err = %d\n", __func__, err);
	return err;
}

static int elan_sensor_remove(struct i2c_client *client)
{
    struct elan_epl_data *epld = i2c_get_clientdata(client);

	sysfs_remove_group(&epld->ps_cdev.dev->kobj, &elan_attr_ps_group);
	sensors_classdev_unregister(&epld->ps_cdev);
	sensors_classdev_unregister(&epld->als_cdev);
	input_unregister_device(epld->als_input_dev);
	input_free_device(epld->als_input_dev);
	input_unregister_device(epld->ps_input_dev);
	input_free_device(epld->ps_input_dev);
	destroy_workqueue(epld->epl_wq);
	elan_regulator_on(epld, false);
	elan_regulator_configure(epld, false);
	kfree(epld);

    dev_err(&client->dev, "%s: enter.\n", __func__);
    return 0;
}

#ifdef CONFIG_PM
static int elan_sensor_suspend(struct device *dev)
{
	if (!epld) {
		printk("%s: null pointer!\n",__func__);
		return -EINVAL;
	}
	disable_irq(epld->irq);

	if ((epld->enable_lflag) && (0 == epld->enable_pflag)) {
		cancel_delayed_work_sync(&polling_work);
		elan_goto_idle(epld->client);
	}

	dev_err(&epld->client->dev, "%s() done!\n", __func__);
	return 0;
}

static int elan_sensor_resume(struct device *dev)
{
	if (!epld) {
		printk("%s: null pointer!!\n", __func__);
		return -EINVAL;
	}
	if ((epld->enable_lflag) && (0 == epld->enable_pflag)) {
		elan_sensor_lsensor_enable(epld);
		queue_delayed_work(epld->epl_wq, &polling_work, 0);
	}
	enable_irq(epld->irq);
	dev_err(&epld->client->dev, "%s done!\n", __func__);
	return 0;
}
#else
#define elan_sensor_suspend	NULL
#define elan_sensor_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id elan_sensor_id[] = {
	{ELAN_CHIP_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, elan_sensor_id);

static struct of_device_id epl_match_table[] = {
    {.compatible = "elan,epl2182",},
    {},
};

static const struct dev_pm_ops epl2182_pm_ops = {
	.suspend = elan_sensor_suspend,
	.resume = elan_sensor_resume,
};

static struct i2c_driver elan_sensor_driver =
{
	.probe = elan_sensor_probe,
	.remove	= elan_sensor_remove,
	.id_table = elan_sensor_id,
	.driver	= {
		.name = ELAN_CHIP_NAME,
		.owner = THIS_MODULE,
		.of_match_table =epl_match_table,
		.pm = &epl2182_pm_ops,
    },
};

static int __init elan_sensor_init(void)
{
	return i2c_add_driver(&elan_sensor_driver);
}

static void __exit elan_sensor_exit(void)
{
	i2c_del_driver(&elan_sensor_driver);
}

module_init(elan_sensor_init);
module_exit(elan_sensor_exit);

MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl2182 driver");
MODULE_LICENSE("GPL");
