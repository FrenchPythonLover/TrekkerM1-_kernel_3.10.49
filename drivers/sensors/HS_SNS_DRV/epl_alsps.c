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
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include "epl_alsps.h"
#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
#include <linux/productinfo.h>
#endif

/******************************************************************************
 *  ALS DYN INTT
 ******************************************************************************/
//Dynamic INTT
int dynamic_intt_idx;
int dynamic_intt_init_idx = 1;	//initial dynamic_intt_idx
int c_gain;
int dynamic_intt_lux = 0;

uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 18000;
uint32_t dynamic_intt_min_lux = 0;
uint32_t dynamic_intt_min_unit = 16;

static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_1024, EPL_ALS_INTT_1024};
static int als_dynamic_intt_value[] = {1024, 1024};
static int als_dynamic_intt_gain[] = {EPL_GAIN_MID, EPL_GAIN_LOW};
static int als_dynamic_intt_high_thr[] = {16000, 60000};
static int als_dynamic_intt_low_thr[] = {200, 1500};


static int als_rs_value[] = {0, 2, 4, 8, 16, 32, 64, 128};
int rs_num = sizeof(als_rs_value)/sizeof(int);

static int wait_value[] = {
	0,	2,	4,	8,	12,
	20, 30, 40, 50, 75,
	100, 150, 200, 300, 400
};

int wait_len = sizeof(wait_value) / sizeof(int);

static int adc_value[] = {128, 256, 512, 1024};
static int cycle_value[] = {1, 2, 4, 8, 16, 32, 64};

static int als_intt_value[] = {
	2, 4, 8, 16, 32,
	64, 128, 256, 512, 768,
	1024, 2048, 4096, 6144, 8192, 10240
};

static int ps_intt_value[] = {
	4, 8, 16, 24, 32,
	48, 80, 144, 272, 384,
	520, 784, 1040, 2064, 4112, 6160
};
/*
static int als_level[] = {
	20,		45,		70,		90,		150,
	300,	500,	700,	1150,	2250,
	4500,	8000,	15000,	30000,	50000
};

static int als_value[] = {
	10,		30,		60,		80,		100,
	200,	400,	600,	800,	1500,
	3000,	6000,	10000,	20000,	40000, 60000
};
*/

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "ELAN",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65535.0",
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
	.fifo_max_event_count = 0,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE | SENSOR_FLAG_WAKE_UP,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int epl_i2c_wr_cmd(struct i2c_client *client,
		uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
	uint8_t buffer[2] = {0};
	int ret = 0;
	int retry = 0;

	buffer[0] = regaddr ;
	buffer[1] = data;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		ret = i2c_master_send(client, buffer, txbyte);
		if (ret == txbyte)
			break;
		dev_err(&client->dev,
			"i2c write error, TXBYTES %d\n", ret);
		mdelay(10);
	}

	if (retry >= I2C_RETRY_COUNT) {
		dev_err(&client->dev,
			"i2c write retry over %d\n", I2C_RETRY_COUNT);
		return -EINVAL;
	}

	return ret;
}

static int epl_i2c_wr(struct i2c_client *client,
		uint8_t regaddr, uint8_t data)
{
	return epl_i2c_wr_cmd(client, regaddr, data, 0x02);
}

static int epl_i2c_rd(struct i2c_client *client,
		uint8_t regaddr, uint8_t bytecount)
{
	int ret = 0;
	int retry = 0;
	int read_count = 0;
	int rx_count = 0;
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);

	while (bytecount > 0) {
		epl_i2c_wr_cmd(client,
		regaddr + read_count, EPL_REG_CTRL_1, 0x01);

		for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
			rx_count = (bytecount > I2C_MAX_COUNT) ? I2C_MAX_COUNT : bytecount;
			ret = i2c_master_recv(client, &epld->gRawData.raw_bytes[read_count], rx_count);
			if (ret == rx_count)
				break;

			dev_err(&client->dev,
				"i2c read error,RXBYTES %d\r\n",ret);
			mdelay(10);
		}

		if (retry >= I2C_RETRY_COUNT) {
			dev_err(&client->dev,
				"i2c read retry over %d\n", I2C_RETRY_COUNT);
			return -EINVAL;
		}
		bytecount -= rx_count;
		read_count += rx_count;
	}
	return ret;
}

static void inline epl_sensor_fresh_rst_wait(struct epl_sensor_priv *epld)
{
	if (epld->epl_sensor.wait == EPL_WAIT_SINGLE)
		epl_i2c_wr(epld->client, EPL_REG_CTRL_2,
			epld->epl_sensor.power | epld->epl_sensor.reset);
}

static void epl_sensor_fresh_sensor_mode(struct epl_sensor_priv *epld)
{
	int temp_mode = (epld->enable_pflag << 1) | epld->enable_lflag;

	dev_err(&epld->client->dev,
		"%s(): mode selection = 0x%x\n", __func__, temp_mode);
	if (epld->epl_sensor.mode != temp_mode) {
		epld->epl_sensor.mode = (u8)temp_mode;
	}
}

static void epl_sensor_wait_for_valid_data(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);

	if (epld->epl_sensor.mode == EPL_MODE_PS) {
		msleep(epld->epl_sensor.ps.ps_frame_time);
		dev_err(&client->dev,
			"[%s] PS only(%dms)\r\n", __func__, epld->epl_sensor.ps.ps_frame_time);
	} else if (epld->epl_sensor.mode == EPL_MODE_ALS) {
		msleep(epld->epl_sensor.als.als_frame_time);
		dev_err(&client->dev,
			"[%s] ALS only(%dms)\r\n", __func__, epld->epl_sensor.als.als_frame_time);
	} else if (epld->epl_sensor.mode == EPL_MODE_ALS_PS ) {
		msleep(epld->epl_sensor.ps.ps_frame_time + epld->epl_sensor.als.als_frame_time);
		dev_err(&client->dev,
			"[%s] PS+ALS(%dms)\r\n", __func__, epld->epl_sensor.ps.ps_frame_time + epld->epl_sensor.als.als_frame_time);
	} else {
		dev_err(&client->dev,
			"[%s] PS+ALS all disable\r\n", __func__);
	}
}

static int inline epl_als_sensing_time(int intt, int adc, int cycle)
{
    int als_intt = als_intt_value[intt >> 2];
	int als_adc = adc_value[adc >> 3];
	int als_cycle = cycle_value[cycle];

	long sensing_us_time = (als_intt + als_adc * 2 * 2) * 2 * als_cycle;
	int sensing_ms_time = sensing_us_time / 1000;

    return (sensing_ms_time + 5);
}

static int epl_als_read_raw(struct epl_sensor_priv *epld)
{
	epl_i2c_rd(epld->client, EPL_REG_ALS_DATA_CH0, 4);

	epld->epl_sensor.als.data.channels[0] =
		(epld->gRawData.raw_bytes[1] << 8) | epld->gRawData.raw_bytes[0];
	epld->epl_sensor.als.data.channels[1] =
		(epld->gRawData.raw_bytes[3] << 8) | epld->gRawData.raw_bytes[2];

	epl_sensor_fresh_rst_wait(epld);
	return 0;
}

long raw_convert_to_lux(struct epl_sensor_priv *epld, u16 raw_data)
{
	long lux = 0;

	lux = c_gain * raw_data;

	return lux;
}

static int epl_sensor_get_als_value(struct epl_sensor_priv *epld, u16 als)
{
	long now_lux=0;

	now_lux = raw_convert_to_lux(epld, als);

	dynamic_intt_lux = now_lux/dynamic_intt_min_unit;
	dynamic_intt_lux = dynamic_intt_lux * epld->als_numerator / epld->als_root;
	return dynamic_intt_lux;
}

static void epl_als_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *epld =
		container_of((struct delayed_work *)work,
			struct epl_sensor_priv, polling_work);

    bool enable_als =
		(epld->enable_lflag == 1) && (atomic_read(&epld->chip_suspend) == 0);

    if (enable_als) {
		ktime_t timestamp;
		int report_lux = 0;
        schedule_delayed_work(&epld->polling_work,
			msecs_to_jiffies(LIGHT_DEFAULT_DELAY));
        mutex_lock(&epld->epl_mutex);
	    epl_als_read_raw(epld);
	    mutex_unlock(&epld->epl_mutex);
		report_lux = epl_sensor_get_als_value(epld, epld->epl_sensor.als.data.channels[1]);
		timestamp = ktime_get_boottime();
		input_report_abs(epld->als_input_dev, ABS_MISC, report_lux);
		input_event(epld->als_input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(timestamp).tv_sec);
		input_event(epld->als_input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);
		input_sync(epld->als_input_dev);
    }
}

static void epl_als_function_on(struct epl_sensor_priv *epld)
{
	epl_sensor_fresh_sensor_mode(epld);

	//ALS unlock and reset
	epld->epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epld->epl_sensor.als.lock = EPL_UN_LOCK;
	epl_i2c_wr(epld->client, EPL_REG_ALS_STATE,
		epld->epl_sensor.als.compare_reset | epld->epl_sensor.als.lock);

	//**** write setting ****
	// step 1. set sensor at idle mode
	// step 2. set sensor at operation mode
	// step 3. delay sensing time
	// step 4. unlock and run als / ps status
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1, epld->epl_sensor.wait | EPL_MODE_IDLE);

	if ((epld->enable_lflag == 1) && (epld->enable_pflag == 0)) {
		epl_i2c_wr(epld->client, 0xfc,
			EPL_A_D | EPL_NORMAL | EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
	}

	epl_i2c_wr(epld->client, EPL_REG_CTRL_1,
		epld->epl_sensor.wait | epld->epl_sensor.mode);

	//ALS unlock and run
	epld->epl_sensor.als.compare_reset = EPL_CMP_RUN;
	epld->epl_sensor.als.lock = EPL_UN_LOCK;
	epl_i2c_wr(epld->client, EPL_REG_ALS_STATE,
		epld->epl_sensor.als.compare_reset | epld->epl_sensor.als.lock);
}

static void epl_als_function_off(struct epl_sensor_priv *epld)
{
	epl_sensor_fresh_sensor_mode(epld);

	//ALS unlock and reset
	epld->epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epld->epl_sensor.als.lock = EPL_UN_LOCK;
	epl_i2c_wr(epld->client, EPL_REG_ALS_STATE,
		epld->epl_sensor.als.compare_reset | epld->epl_sensor.als.lock);

	//**** write setting ****
	// step 1. set sensor at idle mode
	// step 2. set sensor at operation mode
	// step 3. delay sensing time
	// step 4. unlock and run als / ps status
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1, epld->epl_sensor.wait | EPL_MODE_IDLE);
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1, epld->epl_sensor.wait | epld->epl_sensor.mode);
}

static int epl_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, als_cdev);
	u8 buf = 0;

	if (atomic_read(&epld->chip_suspend)) {
		printk("Chip suspended, do not call this process!\n");
		return -EPERM;
	}

	if ((enable < 0) || (enable > 1)) {
		dev_err(&epld->client->dev,
				"%s(): It is an illegal para %d!\n", __func__, enable);
		return -EINVAL;
	}

	dev_err(&epld->client->dev, "%s(): enable = %d\n", __func__, enable);

	mutex_lock(&epld->epl_mutex);
	if (epld->enable_lflag != enable) {

		/* ALS_DYN_INTT */
		dynamic_intt_idx = dynamic_intt_init_idx;
		epld->epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
		epld->epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
		buf = epld->epl_sensor.als.integration_time | epld->epl_sensor.als.gain;
		epl_i2c_wr(epld->client, EPL_REG_ALS_CTRL_1, buf);

		dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
		dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
		dev_err(&epld->client->dev, "%s: dynamic_intt_high_thr is %d, dynamic_intt_low_thr is %d",
			__func__, dynamic_intt_high_thr, dynamic_intt_low_thr);

		epld->enable_lflag = enable;
		cancel_delayed_work(&epld->polling_work);
		if (enable) {
			epl_als_function_on(epld);
			schedule_delayed_work(&epld->polling_work,
				msecs_to_jiffies(epld->enable_pflag ?
					epld->epl_sensor.als.als_frame_time + epld->epl_sensor.ps.ps_frame_time :
					epld->epl_sensor.als.als_frame_time));
		} else {
			epl_als_function_off(epld);
		}
	}
	mutex_unlock(&epld->epl_mutex);

	return 0;
}

static int epl_als_set_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
    return 0;
}

////////////////////////////////////////////

static int inline epl_ps_sensing_time(int intt, int adc, int cycle)
{
	int ps_intt = ps_intt_value[intt >> 2];
	int ps_adc = adc_value[adc >> 3];
	int ps_cycle = cycle_value[cycle];

	long sensing_us_time = (ps_intt * 3 + ps_adc * 2 * 3) * ps_cycle;
	int sensing_ms_time = sensing_us_time / 1000;

    return (sensing_ms_time + 5);
}

static int epl_ps_read_raw(struct epl_sensor_priv *epld)
{
	if (0 > epl_i2c_rd(epld->client, EPL_REG_PS_DATA_CH0, 4))
		return -EIO;
	epld->epl_sensor.ps.data.ir_data =
		(epld->gRawData.raw_bytes[1] << 8) | epld->gRawData.raw_bytes[0];
	epld->epl_sensor.ps.data.data =
		(epld->gRawData.raw_bytes[3] << 8) | epld->gRawData.raw_bytes[2];

	epl_sensor_fresh_rst_wait(epld);

	return 0;
}

static int epl_ps_update_status(struct epl_sensor_priv *epld)
{
	if (0 > epl_i2c_rd(epld->client, EPL_REG_PS_STATE, 1))
		return -EIO;

	epld->epl_sensor.ps.saturation =
		(epld->gRawData.raw_bytes[0] & 0x20);
	epld->epl_sensor.ps.compare_high =
		(epld->gRawData.raw_bytes[0] & 0x10);
	epld->epl_sensor.ps.compare_low =
		(epld->gRawData.raw_bytes[0] & 0x08);
	epld->epl_sensor.ps.interrupt_flag =
		(epld->gRawData.raw_bytes[0] & 0x04);
	epld->epl_sensor.ps.compare_reset =
		(epld->gRawData.raw_bytes[0] & 0x02);
	epld->epl_sensor.ps.lock =
		(epld->gRawData.raw_bytes[0] & 0x01);

	return 0;
}

static void epl_ps_report_dist(struct epl_sensor_priv *epld, int dist_code)
{
	ktime_t timestamp = ktime_get_boottime();
	int real_code = (dist_code & FLUSH_UNMASK);
	int is_flush = (dist_code & FLUSH_MASK);
	input_report_abs(epld->ps_input_dev, ABS_DISTANCE,
		real_code ? (1000 - is_flush) : (1 + is_flush));
	input_report_abs(epld->ps_input_dev, ABS_DISTANCE,
		real_code ? (1023 - is_flush) : (0 + is_flush));
	input_event(epld->ps_input_dev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(epld->ps_input_dev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(epld->ps_input_dev);
}

static int inline epl_ps_update_crosstalk_cancel(struct epl_sensor_priv *epld,
		u16 canc_val)
{
	return i2c_smbus_write_word_data(epld->client,
		EPL_REG_PS_CANCEL, canc_val);
}

static int epl_ps_update_intr_thds(struct i2c_client *client,
		u16 low_thd, u16 high_thd)
{
	int err = 0;

	err = i2c_smbus_write_word_data(client, EPL_REG_PS_THD_L, low_thd);
	if (err < 0) {
		dev_err(&client->dev, "%s(): update EPL_REG_PS_THD_L failed, %d!\n",
			__func__, err);
		return err;
	}

	err = i2c_smbus_write_word_data(client, EPL_REG_PS_THD_H, high_thd);
	if (err < 0) {
		dev_err(&client->dev, "%s(): update EPL_REG_PS_THD_H failed, %d!\n",
			__func__, err);
		return err;
	}

	return 0;
}

static void epl_sensor_eint_work(struct work_struct *work)
{
    struct epl_sensor_priv *epld = 
		container_of((struct delayed_work *)work,
			struct epl_sensor_priv, eint_work);

	mutex_lock(&epld->epl_mutex);
	
	epl_ps_read_raw(epld);
	dev_err(&epld->client->dev,
			"---epld->epl_sensor.ps.data.data=%d, value=%d \n",
			epld->epl_sensor.ps.data.data, epld->epl_sensor.ps.compare_high>>4);
	epl_ps_update_status(epld);
	if (epld->epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER) {
		if(epld->epl_sensor.ps.compare_high) {
			dev_err(&epld->client->dev, "far_to_near!\n");
			epl_ps_report_dist(epld, NEAR_CODE);
		} else {
			dev_err(&epld->client->dev, "near_to_far!\n");
			epl_ps_report_dist(epld, FAR_CODE);
			if (!wake_lock_active(&epld->ps_lock)) {
				dev_err(&epld->client->dev,
					"wake_lock PROX_NEAR_TO_FAR_WLOCK not be locked, and lock it!\n");
				wake_lock_timeout(&epld->ps_lock, HZ);
			} else {
				dev_err(&epld->client->dev,
					"wake_lock PROX_NEAR_TO_FAR_WLOCK be locked, do nothing!\n");
			}
		}
	}

	epld->epl_sensor.ps.compare_reset = EPL_CMP_RUN;
	epl_i2c_wr(epld->client, EPL_REG_PS_STATE, epld->epl_sensor.ps.compare_reset);
	mutex_unlock(&epld->epl_mutex);
}

static irqreturn_t epl_sensor_eint_handler(int irqNo, void *handle)
{
    struct epl_sensor_priv *epld = (struct epl_sensor_priv*)handle;
	dev_err(&epld->client->dev, "%s()!\n", __func__);
	if (epld->enable_pflag == 1) {
		cancel_delayed_work(&epld->eint_work);
		schedule_delayed_work(&epld->eint_work, 0);
	} else {
		dev_err(&epld->client->dev,
			"%s(): PS func not enabled, do nothing!\n", __func__);
	}

    return IRQ_HANDLED;
}

static int epl_ps_do_calib(struct epl_sensor_priv *epld)
{
	int ret = 0;
    u16 temp_ps = 0;

	ret = epl_ps_read_raw(epld);
	if (0 > ret) {
		dev_err(&epld->client->dev,
			"%s(): epl_ps_read_raw() failed!\n", __func__);
		return ret;
	}

	temp_ps = epld->epl_sensor.ps.data.data;
	if (temp_ps < epld->ps_crosstalk_max) {
		dev_err(&epld->client->dev,
			"%s(): get new crosstalk cancel value: %d.\n", __func__, temp_ps);
		if (temp_ps != epld->epl_sensor.ps.cancelation) {
			epld->epl_sensor.ps.cancelation = temp_ps;
		}
	} else {
		dev_err(&epld->client->dev,
			"%s(): crosstalk is too big: %d.\n", __func__, temp_ps);
		if (0xFFFF == epld->epl_sensor.ps.cancelation) {
			dev_err(&epld->client->dev,
				"%s(): There is no valid crosstalk cancel value since bootup! Cease PS funciton!",
				__func__);
			return 0;
		} else {
			dev_err(&epld->client->dev,
				"%s(): There is a former crosstalk cancel value exist %d, try it!\n",
				__func__, epld->epl_sensor.ps.cancelation);
		}
	}

	return epl_ps_update_crosstalk_cancel(epld, epld->epl_sensor.ps.cancelation);
}

static int epl_ps_function_on(struct epl_sensor_priv *epld)
{
	int ret = 0;

	ret = epl_ps_update_intr_thds(epld->client, 0x00, 0xFFFF);
	if (ret < 0) {
		dev_err(&epld->client->dev,
			"%s(): clean ps thds failed!\n", __func__);
		return ret;
	}

	ret = epl_ps_update_crosstalk_cancel(epld, 0);
	if (ret < 0) {
		dev_err(&epld->client->dev,
			"%s(): clean PS_CANC failed!\n", __func__);
		return ret;
	}

	epl_sensor_fresh_sensor_mode(epld);
	//**** write setting ****
	// step 1. set sensor at idle mode
	// step 2. set sensor at operation mode
	// step 3. delay sensing time
	// step 4. unlock and run als / ps status
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1,
		epld->epl_sensor.wait | EPL_MODE_IDLE);
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1,
		epld->epl_sensor.wait | epld->epl_sensor.mode);

	epl_sensor_wait_for_valid_data(epld->client);

	//PS unlock and run
	epld->epl_sensor.ps.compare_reset = EPL_CMP_RUN;
	epld->epl_sensor.ps.lock = EPL_UN_LOCK;
	epl_i2c_wr(epld->client, EPL_REG_PS_STATE,
		epld->epl_sensor.ps.compare_reset | epld->epl_sensor.ps.lock);

	ret = epl_ps_do_calib(epld);
	if (ret < 0) {
		dev_err(&epld->client->dev,
			"%s(): epl_ps_do_calib() failed!\n", __func__);
		return ret;
	}

	if (0xFFFF != epld->epl_sensor.ps.cancelation) {
		epl_ps_update_intr_thds(epld->client,
			epld->epl_sensor.ps.low_threshold,
			epld->epl_sensor.ps.high_threshold);
		// report the current ps status
        ret = epl_ps_read_raw(epld);
		if (0 > ret) {
			dev_err(&epld->client->dev,
				"%s(): check PS Far state failed!\n", __func__);
			return ret;
		} else {
			if (epld->epl_sensor.ps.data.data < epld->epl_sensor.ps.high_threshold) {
				epl_ps_report_dist(epld, FAR_CODE);
			}
		}
	} else {
		dev_err(&epld->client->dev, "PS function doesn't work normal!");
	}
	return 0;
}

static int epl_ps_function_off(struct epl_sensor_priv *epld)
{
	int ret = 0;
	epl_sensor_fresh_sensor_mode(epld);

	//PS unlock and reset
	epld->epl_sensor.ps.compare_reset = EPL_CMP_RESET;
	epld->epl_sensor.ps.lock = EPL_UN_LOCK;
	epl_i2c_wr(epld->client, EPL_REG_PS_STATE,
		epld->epl_sensor.ps.compare_reset |epld->epl_sensor.ps.lock);

	//**** write setting ****
	// step 1. set sensor at idle mode
	// step 2. set sensor at operation mode
	// step 3. delay sensing time
	// step 4. unlock and run als / ps status
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1,
		epld->epl_sensor.wait | EPL_MODE_IDLE);
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1,
		epld->epl_sensor.wait | epld->epl_sensor.mode);
	return ret;
}

static int epl_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
	int ret = 0;

	if (atomic_read(&epld->chip_suspend)) {
		printk("Chip suspended, do not call this process!\n");
		return -EPERM;
	}

	if ((enable < 0) || (enable > 1)) {
		dev_err(&epld->client->dev,
			"%s(): It is an illegal para %d!\n", __func__, enable);
		return -EINVAL;
	}

	dev_err(&epld->client->dev, "%s(): enable = %d\n", __func__, enable);

	mutex_lock(&epld->epl_mutex);
	if (epld->enable_pflag != enable) {
		epld->enable_pflag = enable;
		ret = enable ?
			epl_ps_function_on(epld) :
			epl_ps_function_off(epld);
    }
	mutex_unlock(&epld->epl_mutex);

	return ret;
}

static int epl_ps_set_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
    return 0;
}

static int epl_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);

	if (atomic_read(&epld->chip_suspend)) {
		printk("Chip suspended, do not call this process!\n");
		return -EPERM;
	}
	mutex_lock(&epld->epl_mutex);
	if (epld->enable_pflag == 1) {
		if (0 > epl_ps_read_raw(epld)) {
			dev_err(&epld->client->dev,
				"%s(): get current PS state failed!\n", __func__);
			mutex_unlock(&epld->epl_mutex);
			return -EIO;
		} else {
			if (epld->epl_sensor.ps.data.data < epld->epl_sensor.ps.high_threshold) {
				epl_ps_report_dist(epld, FAR_CODE | FLUSH_MASK);
			} else {
				epl_ps_report_dist(epld, NEAR_CODE | FLUSH_MASK);
			}
		}
    }
	mutex_unlock(&epld->epl_mutex);

	return 0;
}

static ssize_t ps_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);

	if ((epld->enable_pflag == 1) && (atomic_read(&epld->chip_suspend) == 0)) {
		mutex_lock(&epld->epl_mutex);
        epl_ps_read_raw(epld);
        mutex_unlock(&epld->epl_mutex);
		return snprintf(buf, PAGE_SIZE, "%d\n", epld->epl_sensor.ps.data.data);
	} else {
		return -EPERM;
	}
}
static DEVICE_ATTR(ps_data, 0220, ps_data_show, NULL);

static ssize_t ps_thd_away_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", epld->epl_sensor.ps.low_threshold);
}

static ssize_t ps_thd_away_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
    int thd_away = 0;
	sscanf(buf, "%d", &thd_away);
	if ((thd_away >= 0) && (thd_away < 65535)) {
		epld->epl_sensor.ps.low_threshold = thd_away;
		if (epld->enable_pflag == 1) {
			 int ret = epl_ps_update_intr_thds(epld->client,
				epld->epl_sensor.ps.low_threshold,
				epld->epl_sensor.ps.high_threshold);
			 if (0 > ret) return ret;
		}
	} else {
		dev_err(&epld->client->dev,
			"%s(): It is an illegal para, %d!", __func__, thd_away);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_thd_away, 0640, ps_thd_away_show, ps_thd_away_store);

static ssize_t ps_thd_close_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", epld->epl_sensor.ps.high_threshold);
}

static ssize_t ps_thd_close_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
    int thd_close = 0;
	sscanf(buf, "%d", &thd_close);
	if ((thd_close > 0) && (thd_close <= 65535)) {
		epld->epl_sensor.ps.high_threshold = (uint16_t)thd_close;
		if (epld->enable_pflag == 1) {
			int ret = epl_ps_update_intr_thds(epld->client,
				epld->epl_sensor.ps.low_threshold,
				epld->epl_sensor.ps.high_threshold);
			if (0 > ret) return ret;
		}
	} else {
		dev_err(&epld->client->dev,
			"%s(): It is an illegal para, %d!", __func__, thd_close);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(ps_thd_close, 0640, ps_thd_close_show, ps_thd_close_store);

static ssize_t ps_canc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
    return snprintf(buf, PAGE_SIZE, "%d\n", epld->epl_sensor.ps.cancelation);
}
static DEVICE_ATTR(ps_canc, 0440, ps_canc_show, NULL);

static ssize_t ps_canc_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", epld->ps_crosstalk_max);
}

static ssize_t ps_canc_max_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct epl_sensor_priv *epld =
		container_of(sensors_cdev, struct epl_sensor_priv, ps_cdev);
    int temp_val = 0;
	sscanf(buf, "%d", &temp_val);
	if ((temp_val > 0) && (temp_val <= 65535)) {
		epld->ps_crosstalk_max = temp_val;
		return count;
	} else {
		return -EINVAL;
	}
}
static DEVICE_ATTR(ps_crosstalk_maxthd, 0640,
	ps_canc_max_show, ps_canc_max_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_ps_data.attr,
	&dev_attr_ps_thd_away.attr,
	&dev_attr_ps_thd_close.attr,
	&dev_attr_ps_canc.attr,
	&dev_attr_ps_crosstalk_maxthd.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static void ps_poll_dwork_handler(struct work_struct *work)
{
    struct epl_sensor_priv *epld =
        container_of((struct delayed_work *)work,struct epl_sensor_priv, ps_poll_dwork);

	mutex_lock(&epld->epl_mutex);
	epld->enable_pflag = 1;
	epl_ps_function_on(epld);
	epld->enable_pflag = 0;
	epl_ps_function_off(epld);
	mutex_unlock(&epld->epl_mutex);

    return;
}

static int epl_sensor_setup_interrupt(struct epl_sensor_priv *epld)
{
	struct i2c_client *client = epld->client;
	int err = 0;

	if (gpio_is_valid(epld->intr_pin)) {
		err = gpio_request(epld->intr_pin, "alsps_irq_gpio");
		if (err) {
			dev_err(&client->dev,
				"%s: irq gpio request failed", __func__);
			goto initial_fail;
		}

		err = gpio_direction_input(epld->intr_pin);
		if (err) {
			dev_err(&client->dev,
				"%s: set_direction for irq gpio failed\n", __func__);
			goto initial_fail;
		}
	}
	err = request_irq(epld->irq,
				epl_sensor_eint_handler,
				IRQF_TRIGGER_FALLING,
				client->dev.driver->name,
				epld);
	if (err <0) {
		dev_err(&client->dev,
			"%s: request irq pin %d fail for gpio\n", __func__, err);
		goto fail_free_intr_pin;
	}

	err = enable_irq_wake(epld->irq);
	if (0 != err) {
		dev_err(&epld->client->dev,
			"enable_irq_wake failed %d\n", err);
		goto fail_set_irq_wake;
	}

	return 0;

fail_set_irq_wake:
	free_irq(epld->irq, epld);
initial_fail:
fail_free_intr_pin:
	gpio_free(epld->intr_pin);
	return err;
}

static int epl_sensor_setup_ps_sensor(struct epl_sensor_priv *epld)
{
	struct i2c_client *client = epld->client;
	int err = 0;

	epld->ps_input_dev = input_allocate_device();
	if (!epld->ps_input_dev) {
		dev_err(&client->dev,
			"%s: could not allocate ps input device\n", __func__);
		return -ENOMEM;
	}
	epld->ps_input_dev->name = "proximity";

	set_bit(EV_ABS, epld->ps_input_dev->evbit);
	input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1023, 0, 0);
	err = input_register_device(epld->ps_input_dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: could not register ps input device\n", __func__);
		goto err_free_ps_input_device;
	}

	epld->ps_cdev = sensors_proximity_cdev;
	epld->ps_cdev.sensors_enable = epl_ps_set_enable;
	epld->ps_cdev.sensors_poll_delay = epl_ps_set_delay;
	epld->ps_cdev.sensors_flush = epl_ps_flush;
	err = sensors_classdev_register(&epld->ps_input_dev->dev, &epld->ps_cdev);
	if (err) {
		dev_err(&client->dev,
			"%s: Unable to register to sensors class: %d\n", __func__, err);
		goto err_register_ps_classdev;
	}

	err = sysfs_create_group(&epld->ps_cdev.dev->kobj, &proximity_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"%s: PS could not create sysfs group\n", __func__);
		goto err_free_ps_input_device;
	}

    return 0;

err_register_ps_classdev:
	input_unregister_device(epld->ps_input_dev);
err_free_ps_input_device:
	input_free_device(epld->ps_input_dev);
	return err;
}

static int epl_sensor_setup_als_sensor(struct epl_sensor_priv *epld)
{
	struct i2c_client *client = epld->client;
	int err = 0;

	epld->als_input_dev = input_allocate_device();
	if (!epld->als_input_dev) {
		dev_err(&client->dev,
			"%s: could not allocate ls input device\n", __func__);
		return -ENOMEM;
	}
	epld->als_input_dev->name = "light";

	set_bit(EV_ABS, epld->als_input_dev->evbit);

	input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 65535, 0, 0);

	err = input_register_device(epld->als_input_dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s(): can not register ls input device\n", __func__);
		goto err_free_ls_input_device;
	}

	/* establish control channel -- sysfs interface */
	epld->als_cdev = sensors_light_cdev;
	epld->als_cdev.sensors_enable = epl_als_set_enable;
	epld->als_cdev.sensors_poll_delay = epl_als_set_delay;
	err = sensors_classdev_register(&epld->als_input_dev->dev, &epld->als_cdev);
	if (err) {
		dev_err(&client->dev,
			"%s: Unable to register to sensors class: %d\n", __func__, err);
		goto err_register_als_classdev;
	}

	return 0;

err_register_als_classdev:
	input_unregister_device(epld->als_input_dev);
err_free_ls_input_device:
	input_free_device(epld->als_input_dev);
	return err;
}

/*
//====================write global variable===============//
*/
static void write_global_variable(struct epl_sensor_priv *epld)
{
	u8 buf = 0;

	/*wake up chip*/
	buf = epld->epl_sensor.reset | epld->epl_sensor.power;
	epl_i2c_wr(epld->client, EPL_REG_CTRL_2, buf);

	/*chip refrash*/
	epl_i2c_wr(epld->client, 0xfd, 0x8e);
	epl_i2c_wr(epld->client, 0xfe, 0x22);
	epl_i2c_wr(epld->client, 0xfe, 0x02);
	epl_i2c_wr(epld->client, 0xfd, 0x00);

    epl_i2c_wr(epld->client, 0xfc,
		EPL_A_D | EPL_NORMAL | EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);

	/*ps setting*/
	buf = epld->epl_sensor.ps.integration_time | epld->epl_sensor.ps.gain;
	epl_i2c_wr(epld->client, EPL_REG_PS_CTRL_1, buf);

	buf = epld->epl_sensor.ps.adc | epld->epl_sensor.ps.cycle;
	epl_i2c_wr(epld->client, EPL_REG_PS_CTRL_2, buf);

	buf = epld->epl_sensor.ps.ir_on_control |
			epld->epl_sensor.ps.ir_mode |
			epld->epl_sensor.ps.ir_drive;
	epl_i2c_wr(epld->client, EPL_REG_PS_CTRL_3, buf);

	buf = epld->epl_sensor.interrupt_control |
		epld->epl_sensor.ps.persist |
		epld->epl_sensor.ps.interrupt_type;
	epl_i2c_wr(epld->client, EPL_REG_INT_CTRL_1, buf);

	buf = epld->epl_sensor.ps.compare_reset | epld->epl_sensor.ps.lock;
	epl_i2c_wr(epld->client, EPL_REG_PS_STATE, buf);

	/*als setting*/
	buf = epld->epl_sensor.als.integration_time | epld->epl_sensor.als.gain;
	epl_i2c_wr(epld->client, EPL_REG_ALS_CTRL_1, buf);

	buf = epld->epl_sensor.als.adc | epld->epl_sensor.als.cycle;
	epl_i2c_wr(epld->client, EPL_REG_ALS_CTRL_2, buf);

	buf = epld->epl_sensor.als.compare_reset | epld->epl_sensor.als.lock;
	epl_i2c_wr(epld->client, EPL_REG_ALS_STATE, buf);

	/*set mode and wait*/
	buf = epld->epl_sensor.wait | epld->epl_sensor.mode;
	epl_i2c_wr(epld->client, EPL_REG_CTRL_1, buf);
}

static void epl_sensor_initial_global_variable(struct epl_sensor_priv *epld)
{
	/* ALS_DYN_INTT */
	int idx=0, gain_value=0, intt_value=0, total_value=0;
	/*general setting*/
	epld->epl_sensor.power = EPL_POWER_ON;
	epld->epl_sensor.reset = EPL_RESETN_RUN;
	epld->epl_sensor.mode = EPL_MODE_IDLE;
	epld->epl_sensor.wait = EPL_WAIT_0_MS;
	epld->epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

	/*als setting*/
	epld->epl_sensor.als.integration_time = EPL_ALS_INTT_1024;
	epld->epl_sensor.als.gain = EPL_GAIN_LOW; /*EPL_GAIN_MID;*/
	epld->epl_sensor.als.adc = EPL_PSALS_ADC_11;
	epld->epl_sensor.als.cycle = EPL_CYCLE_16;
	epld->epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
	epld->epl_sensor.als.persist = EPL_PERIST_1;
	epld->epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epld->epl_sensor.als.lock = EPL_UN_LOCK;
	epld->epl_sensor.als.lux_per_count = LUX_PER_COUNT;

	/* ALS_DYN_INTT */
	dynamic_intt_idx = dynamic_intt_init_idx;
	epld->epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
	epld->epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
	dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
	dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
	dev_err(&epld->client->dev, "%s: dynamic_intt_high_thr is %d, dynamic_intt_low_thr is %d",
		__func__, dynamic_intt_high_thr, dynamic_intt_low_thr);

	c_gain = 1; // 1/16=0.0625 /*Lux per count*/

	if(epld->epl_sensor.revno == 0x9188) { //3638
		if(als_dynamic_intt_gain[0] == EPL_GAIN_MID) {
			gain_value = 8;
		} else if (als_dynamic_intt_gain[0] == EPL_GAIN_LOW) {
			gain_value = 1;
		}

		intt_value = als_dynamic_intt_value[0] / als_dynamic_intt_value[1];
		total_value = gain_value * intt_value;

		for(idx = 0; idx < rs_num;  idx++) {
			if(total_value < als_rs_value[idx]) {
				break;
			}
		}
		dev_err(&epld->client->dev, "[%s]: idx=%d, als_rs_value=%d, total_value=%d\r\n",
			__func__, idx, als_rs_value[idx-1], total_value);
		epld->epl_sensor.als.als_rs = ((idx-1)<<5);
		als_dynamic_intt_high_thr[0] = als_dynamic_intt_high_thr[0]/total_value;
		als_dynamic_intt_low_thr[0] = als_dynamic_intt_low_thr[0]/total_value;
	}

	/*ps setting*/
	epld->epl_sensor.ps.integration_time = EPL_PS_INTT_272;
	epld->epl_sensor.ps.gain = EPL_GAIN_LOW;
	epld->epl_sensor.ps.adc = EPL_PSALS_ADC_12;
	epld->epl_sensor.ps.cycle = EPL_CYCLE_32;
	epld->epl_sensor.ps.persist = EPL_PERIST_1;
	epld->epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
	epld->epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
	epld->epl_sensor.ps.ir_drive = EPL_IR_DRIVE_100;
	epld->epl_sensor.ps.compare_reset = EPL_CMP_RESET;
	epld->epl_sensor.ps.lock = EPL_UN_LOCK;

	epld->epl_sensor.interrupt_control = EPL_INT_CTRL_PS;
	epld->epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
	epld->epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;

	//write setting to sensor
	write_global_variable(epld);

	epld->epl_sensor.als.als_frame_time =
		epl_als_sensing_time(epld->epl_sensor.als.integration_time,
							epld->epl_sensor.als.adc,
							epld->epl_sensor.als.cycle);
	epld->epl_sensor.ps.ps_frame_time =
		epl_ps_sensing_time(epld->epl_sensor.ps.integration_time,
							epld->epl_sensor.ps.adc,
							epld->epl_sensor.ps.cycle);
}

static int epl_sensor_chip_verify(struct i2c_client *client)
{
	int temp_val = i2c_smbus_read_byte_data(client, EPL_REG_VER);
	if (temp_val < 0) {
		dev_err(&client->dev,
			"read chip-id failed. err = %d\n", temp_val);
		return -EIO;
	} else {
		temp_val &= 0xFF;
		if (temp_val != 0x81) {
			dev_err(&client->dev,
				"it is not a epl chip. id = %d\n", temp_val);
			return -ENODEV;
		}
		return 0;
	}
}

static int epl_sensor_regulator_on(struct epl_sensor_priv *epld, bool on)
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

static int epl_sensor_regulator_configure(struct epl_sensor_priv *epld, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(epld->vdd) > 0)
			regulator_set_voltage(epld->vdd, 0,
				EPLD259x_VDD_MAX_UV);

		regulator_put(epld->vdd);

		if (regulator_count_voltages(epld->vio) > 0)
			regulator_set_voltage(epld->vio, 0,
				EPLD259x_VIO_MAX_UV);

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
				EPLD259x_VDD_MIN_UV, EPLD259x_VDD_MAX_UV);
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
				EPLD259x_VIO_MIN_UV, EPLD259x_VIO_MAX_UV);
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
		regulator_set_voltage(epld->vdd, 0, EPLD259x_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(epld->vdd);
	return rc;
}

static int epl_sensor_parse_dt(struct device *dev,
		struct epl_sensor_priv *epld)
{
	struct device_node *np = dev->of_node;
	int rc = 0;
	u32 temp_val = 0;

	/* irq gpio */
	rc = of_get_named_gpio_flags(np, "epl,irq-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read irq gpio. ret = %d\n", rc);
		return rc;
	}
	epld->intr_pin = rc;

	rc = of_property_read_u32(np, "epl,ps_close_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd_set. ret = %d\n", rc);
		return rc;
	} else {
		epld->epl_sensor.ps.high_threshold = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,ps_away_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd_set. ret = %d\n", rc);
		return rc;
	} else {
		epld->epl_sensor.ps.low_threshold = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,ps_crosstalk_max", &temp_val);
	if (rc) {
		epld->ps_crosstalk_max = EPL_PS_CROSSTALK_MAX;
		dev_err(dev, "ps_crosstalk_max do not set and use default value%d\n",
				epld->ps_crosstalk_max);
	} else {
		epld->ps_crosstalk_max = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,als_fittness", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read als_fittness\n");
		return rc;
	} else {
		epld->als_fittness = (int)temp_val;
	}

	rc = of_property_read_u32(np, "epl,als_numerator", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read als_numerator. ret = %d\n", rc);
		return rc;
	} else {
		epld->als_numerator = (uint16_t)temp_val;
	}

	rc = of_property_read_u32(np, "epl,als_root", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read als_root. ret = %d\n", rc);
		return rc;
	} else {
		epld->als_root = (uint16_t)temp_val;
	}
	return 0;
}

static int epl_sensor_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct epl_sensor_priv *epld = NULL;
	int err = 0;

	dev_err(&client->dev, "%s(): Start!\n", __func__);

	epld = kzalloc(sizeof(struct epl_sensor_priv), GFP_KERNEL);
	if (!epld) {
		err = -ENOMEM;
		goto quit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "No supported i2c func what we need?!!\n");
		err = -ENOTSUPP;
		goto i2c_fail;
	}
	epld->client = client;

	if (client->dev.of_node) {
		err = epl_sensor_parse_dt(&client->dev, epld);
		if (err) {
			dev_err(&client->dev, "%s(): failed to parse dts!\n", __func__);
			goto err_parse_dt;
		}
	} else {
		dev_err(&client->dev, "No dts data\n");
		goto err_parse_dt;
	}

	err = epl_sensor_regulator_configure(epld, true);
	if (err < 0) {
		dev_err(&client->dev,
			"%s(): unable to configure regulator. err = %d\n", __func__, err);
		goto err_sensor_regu_conf;
	}
	
	err = epl_sensor_regulator_on(epld, true);
	if (err < 0) {
		dev_err(&client->dev,
			"%s(): epl power switch error! err = %d\n", __func__, err);
		goto err_sensor_regu_power_on;
	}

	err = epl_sensor_chip_verify(client);
	if (err < 0) {
		dev_err(&client->dev,
			"%s(): epl chip not found! err = %d\n", __func__, err);
		goto err_chip_verify;
	}

	epld->irq = client->irq;
	i2c_set_clientdata(client, epld);

	INIT_DELAYED_WORK(&epld->eint_work, epl_sensor_eint_work);
	INIT_DELAYED_WORK(&epld->polling_work, epl_als_polling_work);
	INIT_DELAYED_WORK(&epld->ps_poll_dwork, ps_poll_dwork_handler);
	mutex_init(&epld->epl_mutex);

	//initial global variable and write to senosr
    epl_sensor_initial_global_variable(epld);

	err = epl_sensor_setup_als_sensor(epld);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: setup als failed!\n", __func__);
		goto err_lightsensor_setup;
	}

	err = epl_sensor_setup_ps_sensor(epld);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: epl_sensor_setup_ps_sensor error!!\n", __func__);
		goto err_psensor_setup;
	}

	err = epl_sensor_setup_interrupt(epld);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: setup interrupt error!\n", __func__);
		goto err_irq_setup;
	}

	wake_lock_init(&epld->ps_lock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	epld->epl_sensor.ps.cancelation = 0xFFFF;/*for first time use prox when power on*/
#ifdef CONFIG_HISENSE_PRODUCT_DEVINFO
	productinfo_register(PRODUCTINFO_SENSOR_ALPS_ID,
		"epl88055/epl2590", "Elan");
#endif	/* CONFIG_HISENSE_PRODUCT_DEVINFO */
	schedule_delayed_work(&epld->ps_poll_dwork, msecs_to_jiffies(0));
	dev_err(&client->dev, "%s(): success.\n", __func__);

    return err;

err_irq_setup:
	sysfs_remove_group(&((epld->ps_cdev.dev)->kobj), &proximity_attribute_group);
	sensors_classdev_unregister(&epld->ps_cdev);
	input_unregister_device(epld->ps_input_dev);
err_psensor_setup:
	sensors_classdev_unregister(&epld->als_cdev);
	input_unregister_device(epld->als_input_dev);
err_lightsensor_setup:
err_chip_verify:
	epl_sensor_regulator_on(epld, false);
err_sensor_regu_power_on:
	epl_sensor_regulator_configure(epld, false);
err_sensor_regu_conf:
err_parse_dt:
i2c_fail:
	kfree(epld);
quit:
	dev_err(&client->dev,
		"%s(): failed! err = %d.", __func__, err);
	return err;
}

static int epl_sensor_remove(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);

	dev_err(&client->dev, "%s: enter.\n", __func__);
	sysfs_remove_group(&((epld->ps_cdev.dev)->kobj), &proximity_attribute_group);
	sensors_classdev_unregister(&epld->ps_cdev);
	input_unregister_device(epld->ps_input_dev);
	sensors_classdev_unregister(&epld->als_cdev);
	input_unregister_device(epld->als_input_dev);

	epl_sensor_regulator_on(epld, false);
	epl_sensor_regulator_configure(epld, false);
	free_irq(epld->irq, epld);
	gpio_free(epld->intr_pin);
	kfree(epld);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int epl_sensor_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);

	dev_err(&client->dev, "%s()\n.", __func__);

	atomic_set(&epld->chip_suspend, 1);
	if (epld->enable_lflag == 1) {
		cancel_delayed_work_sync(&epld->polling_work);
		epl_als_function_off(epld);
    }
	disable_irq(client->irq);
    return 0;
}

static int epl_sensor_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);

	atomic_set(&epld->chip_suspend, 0);
	if (epld->enable_lflag == 1) {
		epl_als_function_on(epld);
		schedule_delayed_work(&epld->polling_work,
			msecs_to_jiffies(epld->enable_pflag ?
				epld->epl_sensor.als.als_frame_time + epld->epl_sensor.ps.ps_frame_time :
				epld->epl_sensor.als.als_frame_time));
	}
	dev_err(&client->dev, "%s()\n.", __func__);

	enable_irq(client->irq);
	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(epl_sensor_pm,
	epl_sensor_suspend, epl_sensor_resume, NULL);

static const struct i2c_device_id epl_sensor_id[] = {
	{EPL_DEV_NAME, 0},
	{}
};

static struct of_device_id epl_match_table[] = {
	{.compatible = "elan,epl_alsps",},
	{},
};

static struct i2c_driver epl_sensor_driver = {
	.probe = epl_sensor_probe,
    .remove	= epl_sensor_remove,
    .id_table = epl_sensor_id,
    .driver	= {
        .name = EPL_DEV_NAME,
        .owner = THIS_MODULE,
        .pm = &epl_sensor_pm,
        .of_match_table = epl_match_table,
    },
};

static int __init epl_sensor_init(void)
{
    return i2c_add_driver(&epl_sensor_driver);
}

static void __exit epl_sensor_exit(void)
{
    i2c_del_driver(&epl_sensor_driver);
}

module_init(epl_sensor_init);
module_exit(epl_sensor_exit);

MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl88055 driver");
MODULE_LICENSE("GPL");
