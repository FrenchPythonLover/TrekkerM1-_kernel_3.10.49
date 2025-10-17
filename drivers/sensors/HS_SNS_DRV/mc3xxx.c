/*****************************************************************************
 *
 * Copyright (c) 2014 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
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
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>

#include <linux/fs.h>
#include <linux/productinfo.h>



/********* CONFIGURATIONS ***********/
/*#define DOT_CALI*/

/************Log manager*******************/
#define GSE_ERR(x...)            printk(x)
static int mc3xxx_debug;
#define mcprintkreg(x...) ({if (mc3xxx_debug) printk(x); })
#define mcprintkfunc(x...) ({if (mc3xxx_debug) printk(x); })
#define GSE_LOG(x...) ({if (mc3xxx_debug) printk(x); })

#define SENSOR_NAME              "accelerometer"
#define SENSOR_DRIVER_VERSION    "1.1.1"
#define SENSOR_DATA_SIZE         3
#define AVG_NUM                  16

/* addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
} u_i2c_addr = {{0x00},};


#define GRAVITY_1G_VALUE    1000


/*#define MC3XXX_CONVERT_PARAMETER    (1.5f * (9.80665f) / 256.0f)*/
#define MC3XXX_DISPLAY_NAME         SENSOR_NAME
#define MC3XXX_DIPLAY_VENDOR        "mCube"


#define MC3XXX_AXIS_X      0
#define MC3XXX_AXIS_Y      1
#define MC3XXX_AXIS_Z      2
#define MC3XXX_AXES_NUM    3
#define MC3XXX_DATA_LEN    6

#define MC3XXX_XOUT_REG						0x00
#define MC3XXX_YOUT_REG						0x01
#define MC3XXX_ZOUT_REG						0x02
#define MC3XXX_Tilt_Status_REG				0x03
#define MC3XXX_SAMPLING_RATE_STATUS_REG     0x04
#define MC3XXX_SLEEP_COUNT_REG 				0x05
#define MC3XXX_INTERRUPT_ENABLE_REG         0x06
#define MC3XXX_MODE_FEATURE_REG				0x07
#define MC3XXX_SAMPLE_RATE_REG				0x08
#define MC3XXX_TAP_DETECTION_ENABLE_REG		0x09
#define MC3XXX_TAP_DWELL_REJECT_REG			0x0a
#define MC3XXX_DROP_CONTROL_REG				0x0b
#define MC3XXX_SHAKE_DEBOUNCE_REG			0x0c
#define MC3XXX_XOUT_EX_L_REG				0x0d
#define MC3XXX_XOUT_EX_H_REG				0x0e
#define MC3XXX_YOUT_EX_L_REG				0x0f
#define MC3XXX_YOUT_EX_H_REG				0x10
#define MC3XXX_ZOUT_EX_L_REG				0x11
#define MC3XXX_ZOUT_EX_H_REG				0x12
#define MC3XXX_CHIP_ID_REG					0x18
#define MC3XXX_RANGE_CONTROL_REG			0x20
#define MC3XXX_SHAKE_THRESHOLD_REG			0x2B
#define MC3XXX_UD_Z_TH_REG					0x2C
#define MC3XXX_UD_X_TH_REG					0x2D
#define MC3XXX_RL_Z_TH_REG					0x2E
#define MC3XXX_RL_Y_TH_REG					0x2F
#define MC3XXX_FB_Z_TH_REG					0x30
#define MC3XXX_DROP_THRESHOLD_REG			0x31
#define MC3XXX_TAP_THRESHOLD_REG			0x32

#define MC3XXX_HIGH_END    0x01
/*******MC3210/20 define this**********/
#define MCUBE_8G_14BIT     0x10

#define MC3XXX_LOW_END     0x02
/*******mc3xxx define this**********/
#define MCUBE_1_5G_8BIT    0x20
#define MC3XXX_MODE_DEF    0x43

#define MC3XXX_RESOLUTION_LOW     1
#define MC3XXX_RESOLUTION_HIGH    2

/***********************************************
 *** RETURN CODE
 ***********************************************/
#define MC3XXX_RETCODE_SUCCESS                 (0)
#define MC3XXX_RETCODE_ERROR_I2C               (-1)
#define MC3XXX_RETCODE_ERROR_STATUS            (-3)
#define MC3XXX_RETCODE_ERROR_SETUP             (-4)
#define MC3XXX_RETCODE_ERROR_GET_DATA          (-5)
#define MC3XXX_RETCODE_ERROR_IDENTIFICATION    (-6)

/***********************************************
 *** PRODUCT ID
 ***********************************************/
#define MC3XXX_PCODE_3210     0x90
#define MC3XXX_PCODE_3230     0x19
#define MC3XXX_PCODE_3250     0x88
#define MC3XXX_PCODE_3410     0xA8
#define MC3XXX_PCODE_3410N    0xB8
#define MC3XXX_PCODE_3430     0x29
#define MC3XXX_PCODE_3430N    0x39
#define MC3XXX_PCODE_3510     0x40
#define MC3XXX_PCODE_3530     0x30
#define MC3XXX_PCODE_3216     0x10
#define MC3XXX_PCODE_3236     0x60

#define MC3XXX_PCODE_RESERVE_1    0x20
#define MC3XXX_PCODE_RESERVE_2    0x11
#define MC3XXX_PCODE_RESERVE_3    0x21
#define MC3XXX_PCODE_RESERVE_4    0x61
#define MC3XXX_PCODE_RESERVE_5    0xA0
#define MC3XXX_PCODE_RESERVE_6    0xE0
#define MC3XXX_PCODE_RESERVE_7    0x91
#define MC3XXX_PCODE_RESERVE_8    0xA1
#define MC3XXX_PCODE_RESERVE_9    0xE1

#define MC3XXX_PCODE_RESERVE_10    0x99


#define MC3XXX_I2C_NAME            SENSOR_NAME
#define SENSOR_DEV_COUNT           1
#define SENSOR_DURATION_MAX        200
#define SENSOR_DURATION_MIN        10
#define SENSOR_DURATION_DEFAULT    100

#define INPUT_FUZZ    0
#define INPUT_FLAT    0

#define MC3XXX_BUFSIZE    256
static unsigned char offset_buf[9] = { 0 };
static signed int offset_data[3] = { 0 };
static signed int gain_data[3] = { 0 };
typedef struct {
	unsigned short	x;		/**< X axis */
	unsigned short	y;		/**< Y axis */
	unsigned short	z;		/**< Z axis */
} GSENSOR_VECTOR3D;

static GSENSOR_VECTOR3D gsensor_gain = { 0 };
static struct file *fd_file;
static mm_segment_t oldfs = { 0 };
#define DATA_PATH              "/sdcard/mcube-register-map.txt"
static       unsigned short     mc3xxx_i2c_auto_probe_addr[] = { 0x4C, 0x6C, 0x4E, 0x6D, 0x6E, 0x6F };

/* POWER SUPPLY VOLTAGE RANGE */
#define MC3XXX_VDD_MIN_UV	2000000
#define MC3XXX_VDD_MAX_UV	3300000
#define MC3XXX_VIO_MIN_UV	1750000
#define MC3XXX_VIO_MAX_UV	1950000

#define MC3XXX_STANDBY_MODE_MASK 0xFC
#define RES_8G_14BIT 1024
#define GMAX_14BIT 0x3FFF


static unsigned char    s_bResolution = 0x00;
static unsigned char    s_bPCODE      = 0x00;
static unsigned char    s_bPCODER     = 0x00;
static unsigned char    s_bHWID       = 0x00;
static unsigned char    s_bMPOL       = 0x00;

static unsigned char    s_baOTP_OffsetData[6] = { 0 };

#ifdef DOT_CALI

/*#define CALIB_PATH              "/data/data/com.mcube.acc/files/mcube-calib.txt"*/
static char file_path[MC3XXX_BUFSIZE] = "/data/data/com.mcube.acc/files/mcube-calib.txt";
/*static char factory_path[MC3XXX_BUFSIZE] ="/data/data/com.mcube.acc/files/fac-calib.txt";*/

static signed int enable_RBM_calibration;

typedef struct{
	int x;
	int y;
	int z;
} SENSOR_DATA;

static int load_cali_flg;

#endif  /* END OF #ifdef DOT_CALI*/


#define MC3XXX_WAKE       1
#define MC3XXX_SNIFF      2
#define MC3XXX_STANDBY    3

#define MCUBE_RREMAP(nDataX, nDataY)                                           \
			{if (MC3XXX_PCODE_3250 == s_bPCODE) {                             \
				int    _nTemp = 0;                                             \
				_nTemp = nDataX;                                               \
				nDataX = nDataY;                                               \
				nDataY = -_nTemp;                                              \
				GSE_LOG("[%s] 3250 read remap\n", __FUNCTION__);               \
			} else {                                                           \
				if (s_bMPOL & 0x01)											   \
					nDataX = -nDataX;                       				   \
				if (s_bMPOL & 0x02)											   \
					nDataY = -nDataY;                       				   \
				GSE_LOG("[%s] 35X0 remap [s_bMPOL: %d]\n", __FUNCTION__, s_bMPOL);\
			} }

#define MCUBE_WREMAP(nDataX, nDataY)										   \
			{if (MC3XXX_PCODE_3250 == s_bPCODE) {                               \
				int    _nTemp = 0;                                             \
				_nTemp = nDataX;                                               \
				nDataX = -nDataY;                                              \
				nDataY = _nTemp;                                               \
				GSE_LOG("[%s] 3250 write remap\n", __FUNCTION__);              \
			} else {                                                           \
				if (s_bMPOL & 0x01)											   \
					nDataX = -nDataX;                       				   \
				if (s_bMPOL & 0x02)											   \
					nDataY = -nDataY;										   \
				GSE_LOG("[%s] 35X0 remap [s_bMPOL: %d]\n", __FUNCTION__, s_bMPOL);\
			} }

#define IS_MCFM12()    ((0xC0 <= s_bHWID) && (s_bHWID <= 0xCF))
#define IS_MCFM3X()    ((0x20 == s_bHWID) || ((0x22 <= s_bHWID) && (s_bHWID <= 0x2F)))


struct acceleration {
	int x;
	int y;
	int z;
};

struct mc3xxx_platform_data {
	unsigned int min_interval;	/* minimum poll interval (in milli-seconds) */
	unsigned int init_interval;	/* initial poll interval (in milli-seconds) */

	/*
	 * By default, x is axis 0, y is axis 1, z is axis 2; these can be
	 * changed to account for sensor orientation within the host device.
	 */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	/*
	 * Each axis can be negated to account for sensor orientation within
	 * the host device.
	 */
	bool negate_x;
	bool negate_y;
	bool negate_z;

	/* CTRL_REG1: set resolution, g-range, data ready enable */
	/* Output resolution: 8-bit valid or 12-bit valid */

	u8 res_ctl;
	/* Output g-range: +/-2g, 4g, or 8g ,12g,16g*/

	u8 g_range;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct mc3xxx_data {
	struct mutex lock;
	struct i2c_client *client;
	struct mc3xxx_platform_data pdata;
	struct work_struct  work;
	struct workqueue_struct *mc3xxx_wq;
	struct sensors_classdev cdev;
	struct hrtimer timer;
	struct device *device;
	struct input_dev *input_dev;
	int use_count;
	int enabled;
	volatile unsigned int duration;
	int use_irq;
	int irq;
	unsigned long irqflags;
	int gpio;
	u8 mode_reg;
	bool	power_enabled;
	struct regulator *vdd;
	struct regulator *vio;
	unsigned int last_poll_interval;
	bool chip_mode;
};

static struct sensors_classdev sensors_cdev = {
	.name = "accelerometer",
	.vendor = "mCube",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "78.4",
	.resolution = "0.01",
	.sensor_power = "0.2",
	.min_delay = 5000,	/* microsecond */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = SENSOR_FLAG_CONTINUOUS_MODE,
	.enabled = 0,
	.delay_msec = 200,	/* millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#define RES_6BIT         0
#define RES_7BIT        (1 << 0)
#define RES_8BIT        (1 << 1)
#define RES_10BIT       (3 << 0)
#define RES_12BIT       (1 << 2)
#define RES_14BIT       (5 << 0)
#define CM3XXX_G_2G      0
#define CM3XXX_G_4G      (1 << 4)
#define CM3XXX_G_8G      (1 << 5)
#define CM3XXX_G_12G     (1 << 6)
#define CM3XXX_G_16G     (3 << 4)


volatile static unsigned int sensor_duration = SENSOR_DURATION_DEFAULT;


static int MC3XXX_ReadRegMap(struct i2c_client *p_i2c_client, u8 *pbUserBuf);

/*****************************************
 *** mc3xxx_validate_sensor_IC
 *****************************************/
static int mc3xxx_validate_sensor_IC(unsigned char *pbPCode, unsigned char *pbHwID)
{
	GSE_LOG("[%s] *pbPCode: 0x%02X, *pbHwID: 0x%02X\n", __FUNCTION__, *pbPCode, *pbHwID);

	if ((0x01 == *pbHwID)
		|| (0x03 == *pbHwID)
		|| ((0x04 <= *pbHwID) && (*pbHwID <= 0x0F))) {
		if ((MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3230 == *pbPCode)
			|| (MC3XXX_PCODE_3250 == *pbPCode))
			return MC3XXX_RETCODE_SUCCESS;
	} else if ((0x02 == *pbHwID)
			|| (0x21 == *pbHwID)
			|| ((0x10 <= *pbHwID) && (*pbHwID <= 0x1F))) {
		if ((MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3230 == *pbPCode)
			|| (MC3XXX_PCODE_3250 == *pbPCode)
			|| (MC3XXX_PCODE_3410 == *pbPCode) || (MC3XXX_PCODE_3410N == *pbPCode)
			|| (MC3XXX_PCODE_3430 == *pbPCode) || (MC3XXX_PCODE_3430N == *pbPCode)) {
			return MC3XXX_RETCODE_SUCCESS;
		}
	} else if ((0xC0 <= *pbHwID) && (*pbHwID <= 0xCF)) {
		*pbPCode = (*pbPCode & 0x71);

		if ((MC3XXX_PCODE_3510 == *pbPCode) || (MC3XXX_PCODE_3530 == *pbPCode))
			return MC3XXX_RETCODE_SUCCESS;
	} else if ((0x20 == *pbHwID) || ((0x22 <= *pbHwID) && (*pbHwID <= 0x2F))) {
		*pbPCode = (*pbPCode & 0xF1);

		if ((MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3216 == *pbPCode)
			|| (MC3XXX_PCODE_3236 == *pbPCode) || (MC3XXX_PCODE_RESERVE_1 == *pbPCode)
			|| (MC3XXX_PCODE_RESERVE_2 == *pbPCode) || (MC3XXX_PCODE_RESERVE_3 == *pbPCode)
			|| (MC3XXX_PCODE_RESERVE_4 == *pbPCode) || (MC3XXX_PCODE_RESERVE_5 == *pbPCode)
			|| (MC3XXX_PCODE_RESERVE_6 == *pbPCode) || (MC3XXX_PCODE_RESERVE_7 == *pbPCode)
			|| (MC3XXX_PCODE_RESERVE_8 == *pbPCode) || (MC3XXX_PCODE_RESERVE_9 == *pbPCode)
			|| (0x50 == *pbPCode)) {
			return MC3XXX_RETCODE_SUCCESS;
		}
	}

	return MC3XXX_RETCODE_ERROR_IDENTIFICATION;
}

/*****************************************
 *** mc3xxx_set_resolution
 *****************************************/
static void mc3xxx_set_resolution(void)
{
	GSE_LOG("[%s]\n", __FUNCTION__);

	switch (s_bPCODE) {
	case MC3XXX_PCODE_3230:
	case MC3XXX_PCODE_3430:
	case MC3XXX_PCODE_3430N:
	case MC3XXX_PCODE_3530:
	case MC3XXX_PCODE_3236:
		s_bResolution = MC3XXX_RESOLUTION_LOW;
		break;

	case MC3XXX_PCODE_3210:
	case MC3XXX_PCODE_3250:
	case MC3XXX_PCODE_3410:
	case MC3XXX_PCODE_3410N:
	case MC3XXX_PCODE_3510:
	case MC3XXX_PCODE_3216:
		s_bResolution = MC3XXX_RESOLUTION_HIGH;
		break;

	/*=== RESERVED ====================BGN===*/
	/*=== (move to normal section once it is confirmed) ===*/
	case MC3XXX_PCODE_RESERVE_10:
		GSE_ERR("RESERVED ONLINE!\n");
		/* TODO: should have a default configuration...*/
		break;

	case MC3XXX_PCODE_RESERVE_1:
	case MC3XXX_PCODE_RESERVE_3:
	case MC3XXX_PCODE_RESERVE_4:
	case MC3XXX_PCODE_RESERVE_5:
	case MC3XXX_PCODE_RESERVE_6:
	case MC3XXX_PCODE_RESERVE_8:
	case MC3XXX_PCODE_RESERVE_9:
		GSE_ERR("RESERVED ONLINE!\n");
		s_bResolution = MC3XXX_RESOLUTION_LOW;
		break;

	case MC3XXX_PCODE_RESERVE_2:
	case MC3XXX_PCODE_RESERVE_7:
		GSE_ERR("RESERVED ONLINE!\n");
		s_bResolution = MC3XXX_RESOLUTION_HIGH;
		break;
	/* === RESERVED =====================END===*/

	default:
		GSE_ERR("ERR: no resolution assigned!\n");
		break;
	}

	GSE_LOG("[%s] s_bResolution: %d\n", __FUNCTION__, s_bResolution);
}

/*****************************************
 *** mc3xxx_set_sample_rate
 *****************************************/
static void mc3xxx_set_sample_rate(struct i2c_client *pt_i2c_client)
{
	unsigned char _baDataBuf[2] = { 0 };

	GSE_LOG("[%s]\n", __FUNCTION__);

	_baDataBuf[0] = MC3XXX_SAMPLE_RATE_REG;
	_baDataBuf[1] = 0x00;

	if (IS_MCFM12() || IS_MCFM3X()) {
		unsigned char _baData2Buf[2] = { 0 };

		_baData2Buf[0] = 0x2A;
		i2c_master_send(pt_i2c_client, &(_baData2Buf[0]), 1);
		i2c_master_recv(pt_i2c_client, &(_baData2Buf[0]), 1);

		GSE_LOG("[%s] REG(0x2A) = 0x%02X\n", __FUNCTION__, _baData2Buf[0]);

		_baData2Buf[0] = (_baData2Buf[0] & 0xC0);

		switch (_baData2Buf[0]) {
		case 0x00:
			_baDataBuf[1] = 0x09;
			break;
		case 0x40:
			_baDataBuf[1] = 0x09;
			break;
		case 0x80:
			_baDataBuf[1] = 0x09;
			break;
		case 0xC0:
			_baDataBuf[1] = 0x09;
			break;
		default:
			GSE_ERR("[%s] no chance to get here... check code!\n", __FUNCTION__);
			break;
		}
	}
	i2c_master_send(pt_i2c_client, _baDataBuf, 0x2);
}

/*****************************************
 *** mc3xxx_config_range
 *****************************************/
static void mc3xxx_config_range(struct i2c_client *pt_i2c_client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(pt_i2c_client);

	unsigned char    _baDataBuf[2] = { 0 };

	_baDataBuf[0] = MC3XXX_RANGE_CONTROL_REG;
	_baDataBuf[1] = 0x3F;

	if (MC3XXX_RESOLUTION_LOW == s_bResolution)
		_baDataBuf[1] = 0x32;

	if (IS_MCFM12() || IS_MCFM3X()) {
		if (MC3XXX_RESOLUTION_LOW == s_bResolution)
			_baDataBuf[1] = 0x02;
		else
			_baDataBuf[1] = data->pdata.g_range | data->pdata.res_ctl;
	}

	i2c_master_send(pt_i2c_client, _baDataBuf, 0x2);

	GSE_LOG("[%s] set 0x%X\n", __FUNCTION__, _baDataBuf[1]);
}

int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode)
{
	int ret = 0;
	unsigned char reg_data = 0;
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	if (mode >= 4) {
		printk("mc3xxx set wrong mode\n");
		return -EINVAL;
	} else {
		reg_data = (0x40 | mode);
		ret = i2c_smbus_write_byte_data(client, MC3XXX_MODE_FEATURE_REG, reg_data);
		if ((ret >= 0) && (mode == MC3XXX_WAKE))
			data->chip_mode = true;
		else
			data->chip_mode = false;
	}

	return ret;
}

struct file *openFile(char *path, int flag, int mode)
{
	struct file *fp = NULL;

	fp = filp_open(path, flag, mode);

	if (IS_ERR(fp) || !fp->f_op) {
		GSE_LOG("Calibration File filp_open return NULL\n");
		return NULL;
	}

	return fp;
}

int readFile(struct file *fp, char *buf, int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp, buf, readlen, &fp->f_pos);
	else
		return -EPERM;
}

int writeFile(struct file *fp, char *buf, int writelen)
{
	if (fp->f_op && fp->f_op->write)
		return fp->f_op->write(fp, buf, writelen, &fp->f_pos);
	else
		return -EPERM;
}

int closeFile(struct file *fp)
{
	filp_close(fp, NULL);
	return 0;
}

void initKernelEnv(void)
{
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	printk(KERN_INFO "initKernelEnv\n");
}


#ifdef DOT_CALI

int MC3XXX_WriteCalibration(struct i2c_client *client, int dat[MC3XXX_AXES_NUM])
{
	int err = 0;
	u8 buf[9] = { 0 };
	s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
	s32 x_off = 0, y_off = 0, z_off = 0;
	int temp_cali_dat[MC3XXX_AXES_NUM] = { 0 };
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	u8  bMsbFilter       = 0x3F;
	s16 wSignBitMask     = 0x2000;
	s16 wSignPaddingBits = 0xC000;
	s32 dwRangePosLimit  = 0x1FFF;
	s32 dwRangeNegLimit  = -0x2000;

	temp_cali_dat[data->pdata.axis_map_x] = (data->pdata.negate_x ? -1:1) * dat[MC3XXX_AXIS_X];
	temp_cali_dat[data->pdata.axis_map_y] = (data->pdata.negate_y ? -1:1) * dat[MC3XXX_AXIS_Y];
	temp_cali_dat[data->pdata.axis_map_z] = (data->pdata.negate_z ? -1:1) * dat[MC3XXX_AXIS_Z];

	MCUBE_WREMAP(temp_cali_dat[MC3XXX_AXIS_X], temp_cali_dat[MC3XXX_AXIS_Y]);

	dat[MC3XXX_AXIS_X] = temp_cali_dat[MC3XXX_AXIS_X];
	dat[MC3XXX_AXIS_Y] = temp_cali_dat[MC3XXX_AXIS_Y];
	dat[MC3XXX_AXIS_Z] = temp_cali_dat[MC3XXX_AXIS_Z];

	GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", dat[MC3XXX_AXIS_X], dat[MC3XXX_AXIS_Y], dat[MC3XXX_AXIS_Z]);

	/* read register 0x21~0x29*/
	err  = i2c_smbus_read_i2c_block_data(client, 0x21, 3, &buf[0]);
	err |= i2c_smbus_read_i2c_block_data(client, 0x24, 3, &buf[3]);
	err |= i2c_smbus_read_i2c_block_data(client, 0x27, 3, &buf[6]);

	if (IS_MCFM12() || IS_MCFM3X()) {
		bMsbFilter       = 0x7F;
		wSignBitMask     = 0x4000;
		wSignPaddingBits = 0x8000;
		dwRangePosLimit  = 0x3FFF;
		dwRangeNegLimit  = -0x4000;
	}

	/* get x,y,z offset*/
	tmp = ((buf[1] & bMsbFilter) << 8) + buf[0];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	x_off = tmp;

	tmp = ((buf[3] & bMsbFilter) << 8) + buf[2];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	y_off = tmp;

	tmp = ((buf[5] & bMsbFilter) << 8) + buf[4];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	z_off = tmp;

	/* get x,y,z gain*/
	x_gain = ((buf[1] >> 7) << 8) + buf[6];
	y_gain = ((buf[3] >> 7) << 8) + buf[7];
	z_gain = ((buf[5] >> 7) << 8) + buf[8];

	/* prepare new offset*/
	x_off = x_off + 16 * dat[MC3XXX_AXIS_X] * 256 * 128 / 3 / gsensor_gain.x / (40 + x_gain);
	y_off = y_off + 16 * dat[MC3XXX_AXIS_Y] * 256 * 128 / 3 / gsensor_gain.y / (40 + y_gain);
	z_off = z_off + 16 * dat[MC3XXX_AXIS_Z] * 256 * 128 / 3 / gsensor_gain.z / (40 + z_gain);

	/*range check*/
	if (x_off > dwRangePosLimit)
		x_off = dwRangePosLimit;
	else if (x_off < dwRangeNegLimit)
		x_off = dwRangeNegLimit;

	if (y_off > dwRangePosLimit)
		y_off = dwRangePosLimit;
	else if (y_off < dwRangeNegLimit)
		y_off = dwRangeNegLimit;

	if (z_off > dwRangePosLimit)
		z_off = dwRangePosLimit;
	else if (z_off < dwRangeNegLimit)
		z_off = dwRangeNegLimit;

	/*storege the cerrunt offset data with DOT format*/
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	/*storege the cerrunt Gain data with GOT format*/
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);
	GSE_LOG("%d %d ======================\n\n ", gain_data[0], x_gain);

	buf[0] = 0x43;
	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	buf[0] = x_off & 0xff;
	buf[1] = ((x_off >> 8) & bMsbFilter) | (x_gain & 0x0100 ? 0x80 : 0);
	buf[2] = y_off & 0xff;
	buf[3] = ((y_off >> 8) & bMsbFilter) | (y_gain & 0x0100 ? 0x80 : 0);
	buf[4] = z_off & 0xff;
	buf[5] = ((z_off >> 8) & bMsbFilter) | (z_gain & 0x0100 ? 0x80 : 0);

	i2c_smbus_write_i2c_block_data(client, 0x21,   2, &buf[0]);
	i2c_smbus_write_i2c_block_data(client, 0x21+2, 2, &buf[2]);
	i2c_smbus_write_i2c_block_data(client, 0x21+4, 2, &buf[4]);

	buf[0] = 0x41;
	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	msleep(50);

	return err;
}


int mcube_read_cali_file(struct i2c_client *client)
{
	int cali_data[3] = { 0 };
	int err = 0;
	char buf[64] = { 0 };

	GSE_LOG("%s %d\n", __func__, __LINE__);

	initKernelEnv();

	fd_file = openFile(file_path, O_RDONLY, 0);

	if (fd_file == NULL) {
		GSE_LOG("[%s]:fail to open\n", __func__);
		cali_data[0] = 0;
		cali_data[1] = 0;
		cali_data[2] = 0;

		return -EPERM;
	} else {
		memset(buf, 0, 64);
		err = readFile(fd_file, buf, 64);
		if (err > 0)
			GSE_LOG("buf:%s\n", buf);
		else
			GSE_LOG("read file error %d\n", err);

		set_fs(oldfs);
		closeFile(fd_file);

		sscanf(buf, "%d %d %d", &cali_data[MC3XXX_AXIS_X], &cali_data[MC3XXX_AXIS_Y], &cali_data[MC3XXX_AXIS_Z]);
		GSE_LOG("cali_data: %d %d %d\n", cali_data[MC3XXX_AXIS_X], cali_data[MC3XXX_AXIS_Y], cali_data[MC3XXX_AXIS_Z]);

		MC3XXX_WriteCalibration(client, cali_data);
	}

	return 0;
}

/*****************************************
 *** mc3xxx_set_gain
 *****************************************/
static void mc3xxx_set_gain(void)
{
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = RES_8G_14BIT;

	if (MC3XXX_RESOLUTION_LOW == s_bResolution) {
		gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 86;

		if (IS_MCFM12() || IS_MCFM3X()) {
			gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 64;
		}
	}

	GSE_LOG("[%s] gain: %d / %d / %d\n", __FUNCTION__, gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
}

void MC3XXX_rbm(struct i2c_client *client, int enable)
{
	char    _baDataBuf[3] = { 0 };

	_baDataBuf[0] = 0x43;
	i2c_smbus_write_byte_data(client, 0x07, _baDataBuf[0]);

	_baDataBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

	GSE_LOG("[%s] REG(0x04): 0x%X, enable: %d\n", __FUNCTION__, _baDataBuf[0], enable);

	if (0x00 == (_baDataBuf[0] & 0x40)) {
		_baDataBuf[0] = 0x6D;
		i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);

		_baDataBuf[0] = 0x43;
		i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);
	}

	GSE_LOG("BEGIN - REG(0x04): 0x%X\n", _baDataBuf[0]);

	if (1 == enable) {
		_baDataBuf[0] = 0x00;
		i2c_smbus_write_byte_data(client, 0x3B, _baDataBuf[0]);

		_baDataBuf[0] = 0x02;
		i2c_smbus_write_byte_data(client, 0x14, _baDataBuf[0]);

		mc3xxx_set_gain();

		enable_RBM_calibration = 1;

		GSE_LOG("set rbm!!\n");
} else if (0 == enable) {
		_baDataBuf[0] = 0x00;
		i2c_smbus_write_byte_data(client, 0x14, _baDataBuf[0]);

		_baDataBuf[0] = s_bPCODER;
		i2c_smbus_write_byte_data(client, 0x3B, _baDataBuf[0]);

		mc3xxx_set_gain();

		enable_RBM_calibration = 0;

		GSE_LOG("clear rbm!!\n");
	}

	_baDataBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

	GSE_LOG("RBM CONTROL DONE - REG(0x04): 0x%X\n", _baDataBuf[0]);

	if (_baDataBuf[0] & 0x40) {
		_baDataBuf[0] = 0x6D;
		i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);

		_baDataBuf[0] = 0x43;
		i2c_smbus_write_byte_data(client, 0x1B, _baDataBuf[0]);
	}

	GSE_LOG("END - REG(0x04): 0x%X\n", _baDataBuf[0]);

	_baDataBuf[0] = 0x41;
	i2c_smbus_write_byte_data(client, 0x07, _baDataBuf[0]);

	msleep(220);
}


int MC3XXX_ReadOffset(struct i2c_client *client, s16 ofs[MC3XXX_AXES_NUM])
{
	int err = 0;
	u8 off_data[6] = { 0 };

	if (MC3XXX_RESOLUTION_HIGH == s_bResolution) {
		err = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, MC3XXX_DATA_LEN, off_data);

		ofs[MC3XXX_AXIS_X] = ((s16)(off_data[0]))|((s16)(off_data[1])<<8);
		ofs[MC3XXX_AXIS_Y] = ((s16)(off_data[2]))|((s16)(off_data[3])<<8);
		ofs[MC3XXX_AXIS_Z] = ((s16)(off_data[4]))|((s16)(off_data[5])<<8);
	} else if (MC3XXX_RESOLUTION_LOW == s_bResolution) {
		err = i2c_smbus_read_i2c_block_data(client, 0, 3, off_data);

		ofs[MC3XXX_AXIS_X] = (s8)off_data[0];
		ofs[MC3XXX_AXIS_Y] = (s8)off_data[1];
		ofs[MC3XXX_AXIS_Z] = (s8)off_data[2];
	}

	MCUBE_RREMAP(ofs[MC3XXX_AXIS_X], ofs[MC3XXX_AXIS_Y]);

	GSE_LOG("MC3XXX_ReadOffset %d %d %d\n", ofs[MC3XXX_AXIS_X], ofs[MC3XXX_AXIS_Y], ofs[MC3XXX_AXIS_Z]);

	return err;
}


int MC3XXX_ResetCalibration(struct i2c_client *client)
{
	u8 buf[6] = { 0 };
	s16 tmp = 0;
	int err = 0;

	u8  bMsbFilter       = 0x3F;
	s16 wSignBitMask     = 0x2000;
	s16 wSignPaddingBits = 0xC000;

	buf[0] = 0x43;
	err = i2c_smbus_write_byte_data(client, 0x07, buf[0]);
	if (err)
		GSE_ERR("error 0x07: %d\n", err);

	err = i2c_smbus_write_i2c_block_data(client, 0x21, 6, offset_buf);
	if (err)
		GSE_ERR("error: %d\n", err);

	buf[0] = 0x41;
	err = i2c_smbus_write_byte_data(client, 0x07, buf[0]);
	if (err)
		GSE_ERR("error: %d\n", err);

	msleep(20);

	if (IS_MCFM12() || IS_MCFM3X()) {
		bMsbFilter       = 0x7F;
		wSignBitMask     = 0x4000;
		wSignPaddingBits = 0x8000;
	}

	tmp = ((offset_buf[1] & bMsbFilter) << 8) + offset_buf[0];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[0] = tmp;

	tmp = ((offset_buf[3] & bMsbFilter) << 8) + offset_buf[2];
	if (tmp & wSignBitMask)
			tmp |= wSignPaddingBits;
	offset_data[1] = tmp;

	tmp = ((offset_buf[5] & bMsbFilter) << 8) + offset_buf[4];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[2] = tmp;

	return 0;
}

int MC3XXX_ReadData(struct i2c_client *client, s16 buffer[MC3XXX_AXES_NUM])
{
	unsigned char buf[6] = { 0 };
	signed char buf1[6] = { 0 };
	char rbm_buf[6] = { 0 };
	int ret = 0;

	ret = mc3xxx_set_mode(client, MC3XXX_WAKE);
	if (ret < 0)
		GSE_ERR("%s:mc3xxx_set_mode failed\n", __func__);

	if (enable_RBM_calibration == 0)
		/*err = hwmsen_read_block(client, addr, buf, 0x06);*/
	else if (enable_RBM_calibration == 1) {
		memset(rbm_buf, 0, 6);
		i2c_smbus_read_i2c_block_data(client, 0x0d  , 2, &rbm_buf[0]);
		i2c_smbus_read_i2c_block_data(client, 0x0d+2, 2, &rbm_buf[2]);
		i2c_smbus_read_i2c_block_data(client, 0x0d+4, 2, &rbm_buf[4]);
	}

	if (enable_RBM_calibration == 0) {
		if (MC3XXX_RESOLUTION_HIGH == s_bResolution) {
			ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, 6, buf);
			if (0 > ret)
				return -EIO;
			buffer[0] = (signed short)((buf[0])|(buf[1]<<8));
			buffer[1] = (signed short)((buf[2])|(buf[3]<<8));
			buffer[2] = (signed short)((buf[4])|(buf[5]<<8));
		} else if (MC3XXX_RESOLUTION_LOW == s_bResolution) {
			ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_REG, 3, buf1);
			buffer[0] = (signed short)buf1[0];
			buffer[1] = (signed short)buf1[1];
			buffer[2] = (signed short)buf1[2];
		}
		mcprintkreg("MC3XXX_ReadData: %d %d %d\n", buffer[0], buffer[1], buffer[2]);
	} else if (enable_RBM_calibration == 1) {
		buffer[MC3XXX_AXIS_X] = (s16)((rbm_buf[0]) | (rbm_buf[1] << 8));
		buffer[MC3XXX_AXIS_Y] = (s16)((rbm_buf[2]) | (rbm_buf[3] << 8));
		buffer[MC3XXX_AXIS_Z] = (s16)((rbm_buf[4]) | (rbm_buf[5] << 8));

		GSE_LOG("%s RBM<<<<<[%08d %08d %08d]\n", __func__, buffer[MC3XXX_AXIS_X], buffer[MC3XXX_AXIS_Y], buffer[MC3XXX_AXIS_Z]);

		if (gain_data[0] == 0) {
			buffer[MC3XXX_AXIS_X] = 0;
			buffer[MC3XXX_AXIS_Y] = 0;
			buffer[MC3XXX_AXIS_Z] = 0;

			return 0;
		}

		buffer[MC3XXX_AXIS_X] = (buffer[MC3XXX_AXIS_X] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		buffer[MC3XXX_AXIS_Y] = (buffer[MC3XXX_AXIS_Y] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		buffer[MC3XXX_AXIS_Z] = (buffer[MC3XXX_AXIS_Z] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];

		GSE_LOG("%s offset_data <<<<<[%d %d %d]\n", __func__, offset_data[0], offset_data[1], offset_data[2]);
		GSE_LOG("%s gsensor_gain <<<<<[%d %d %d]\n", __func__, gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
		GSE_LOG("%s gain_data <<<<<[%d %d %d]\n", __func__, gain_data[0], gain_data[1], gain_data[2]);
		GSE_LOG("%s RBM->RAW <<<<<[%d %d %d]\n", __func__, buffer[MC3XXX_AXIS_X], buffer[MC3XXX_AXIS_Y], buffer[MC3XXX_AXIS_Z]);
	}

	MCUBE_RREMAP(buffer[MC3XXX_AXIS_X], buffer[MC3XXX_AXIS_Y]);

	return 0;
}


int MC3XXX_ReadRawData(struct i2c_client *client, s16 *buf)
{
	int res = 0;
	s16 raw_buf[3] = { 0 };
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	if (!buf || !client)
		return -EINVAL;

	res = MC3XXX_ReadData(client, &raw_buf[0]);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	} else {
		GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", raw_buf[MC3XXX_AXIS_X], raw_buf[MC3XXX_AXIS_Y], raw_buf[MC3XXX_AXIS_Z]);

		buf[MC3XXX_AXIS_X] = (data->pdata.negate_x ? -1:1) * raw_buf[data->pdata.axis_map_x];
		buf[MC3XXX_AXIS_Y] = (data->pdata.negate_y ? -1:1) * raw_buf[data->pdata.axis_map_y];
		buf[MC3XXX_AXIS_Z] = (data->pdata.negate_z ? -1:1) * raw_buf[data->pdata.axis_map_z];
	}

	return 0;
}
#endif

static ssize_t mc3xxx_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char baChipID[4] = { 0 };
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);

	i2c_smbus_read_i2c_block_data(data->client, 0x3C, 4, baChipID);

	return snprintf(buf, 20, "%02X-%02X-%02X-%02X\n", baChipID[3], baChipID[2], baChipID[1], baChipID[0]);
}


static ssize_t mc3xxx_regmap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8         _bIndex       = 0;
	u8         _baRegMap[64] = { 0 };
	ssize_t    _tLength      = 0;

	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);
	mutex_lock(&data->lock);
	MC3XXX_ReadRegMap(data->client, _baRegMap);
	mutex_unlock(&data->lock);

	for (_bIndex = 0; _bIndex < 64; _bIndex++)
		_tLength += snprintf((buf + _tLength), (PAGE_SIZE - _tLength), "Reg[0x%02X]: 0x%02X\n", _bIndex, _baRegMap[_bIndex]);

	return _tLength;
}

static ssize_t chip_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);
	char temp_val;

	mutex_lock(&data->lock);
	temp_val = i2c_smbus_read_byte_data(data->client, 0x07);
	mutex_unlock(&data->lock);

	return snprintf(buf, 10, "0X%x\n", temp_val);
}

#ifdef DOT_CALI
static ssize_t rbm_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	GSE_LOG("rbm_mode_show\n");

	return snprintf(buf, 10, "%d\n", enable_RBM_calibration);
}


static ssize_t rbm_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);
	int en;
	sscanf(buf, "%d", &en);

	if ((en == 0) || (en == 1)) {
		GSE_LOG("MC3XXX rbm_mode_store\n");
		mutex_lock(&data->lock);
		MC3XXX_rbm(data->client, en);
		mutex_unlock(&data->lock);
	} else
		dev_err(&data->client->dev,
			"%s(): It is an illegal para, %d!", __func__, en);

	return count;
}

static ssize_t raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);
	s16 raw_buf[3] = {0};

	GSE_LOG("MC3XXX raw_data_show\n");

	mutex_lock(&data->lock);
	MC3XXX_ReadRawData(data->client, raw_buf);
	mutex_unlock(&data->lock);

	return snprintf(buf, 20, "%+3d %+3d %+3d\n", raw_buf[0], raw_buf[1], raw_buf[2]);
}

static ssize_t write_cali_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);
	int cali[3] = { 0 };
	int ret = 0;

	GSE_LOG("mc3xxx write_cali_store\n");

	if (!strncmp(buf, "rst", 3)) {
		mutex_lock(&data->lock);
		MC3XXX_ResetCalibration(data->client);
		mutex_unlock(&data->lock);
		if (ret < 0)
			GSE_ERR("reset offset ret = %d\n", ret);
	} else if (3 == sscanf(buf, "%d %d %d", &cali[0], &cali[1], &cali[2])) {

		mutex_lock(&data->lock);
		ret = MC3XXX_WriteCalibration(data->client, cali);
		mutex_unlock(&data->lock);
		if (ret < 0)
			GSE_ERR("write calibration ret = %d\n", ret);
	} else
		GSE_ERR("invalid format\n");

	return count;
}

static ssize_t read_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct mc3xxx_data *data = container_of(sensors_cdev,
								struct mc3xxx_data, cdev);
	int ret = 0;
	s16 cali[3] = { 0 };

	GSE_LOG("read_offset_show\n");

	mutex_lock(&data->lock);
	ret = MC3XXX_ReadOffset(data->client, cali);
	mutex_unlock(&data->lock);
	if (ret < 0)
		GSE_ERR("fwq mc3xxx MC3XXX_ReadOffset error!!!!\n");

	return snprintf(buf, 20, "%+3d %+3d %+3d\n", cali[0], cali[1], cali[2]);
}
#endif

static DEVICE_ATTR(chipid, 0400, mc3xxx_chip_id_show, NULL);
static DEVICE_ATTR(regmap, 0600, mc3xxx_regmap_show, NULL);
static DEVICE_ATTR(mode, 0400, chip_mode_show, NULL);
#ifdef DOT_CALI
static DEVICE_ATTR(rbm_mode, 0600, rbm_mode_show, rbm_mode_store);
static DEVICE_ATTR(raw_data, 0400, raw_data_show, NULL);
static DEVICE_ATTR(write_cali, 0200, NULL, write_cali_store);
static DEVICE_ATTR(read_offset, 0400, read_offset_show, NULL);
#endif


static struct attribute *mc3xxx_attrs[] = {
	&dev_attr_chipid.attr,
	&dev_attr_regmap.attr,
	&dev_attr_mode.attr,
#ifdef DOT_CALI
	&dev_attr_rbm_mode.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_write_cali.attr,
	&dev_attr_read_offset.attr,
#endif
	NULL
};

static struct attribute_group mc3xxx_group = {
	.attrs = mc3xxx_attrs,
};


static int mc3xxx_chip_init(struct i2c_client *client)
{
	unsigned char  _baDataBuf[2] = { 0 };

	_baDataBuf[0] = MC3XXX_MODE_FEATURE_REG;
	_baDataBuf[1] = 0x43;
	i2c_smbus_write_byte_data(client, _baDataBuf[0], _baDataBuf[1]);

	mc3xxx_set_resolution();
	mc3xxx_set_sample_rate(client);
	mc3xxx_config_range(client);
	/*mc3xxx_set_gain();*/

	_baDataBuf[0] = MC3XXX_TAP_DETECTION_ENABLE_REG;
	_baDataBuf[1] = 0x00;
	i2c_master_send(client, _baDataBuf, 0x2);

	_baDataBuf[0] = MC3XXX_INTERRUPT_ENABLE_REG;
	_baDataBuf[1] = 0x00;
	i2c_master_send(client, _baDataBuf, 0x2);

	_baDataBuf[0] = 0x2A;
	i2c_master_send(client, &(_baDataBuf[0]), 1);
	i2c_master_recv(client, &(_baDataBuf[0]), 1);
	s_bMPOL = (_baDataBuf[0] & 0x03);

	GSE_LOG("[%s] init ok.\n", __FUNCTION__);

	return MC3XXX_RETCODE_SUCCESS;
}

static int mcube_write_log_data(struct i2c_client *client, u8 data[0x3f])
{
	#define _WRT_LOG_DATA_BUFFER_SIZE    (66 * 50)

	s16 rbm_data[3] = {0}, raw_data[3] = {0};
	int err = 0;
	char *_pszBuffer = NULL;
	int n = 0, i = 0;

	initKernelEnv();
	fd_file = openFile(DATA_PATH, O_RDWR | O_CREAT, 0);
	if (fd_file == NULL) {
		GSE_LOG("mcube_write_log_data fail to open\n");
		return -ENOENT;
	} else {
		rbm_data[MC3XXX_AXIS_X] = (s16)((data[0x0d]) | (data[0x0e] << 8));
		rbm_data[MC3XXX_AXIS_Y] = (s16)((data[0x0f]) | (data[0x10] << 8));
		rbm_data[MC3XXX_AXIS_Z] = (s16)((data[0x11]) | (data[0x12] << 8));

		raw_data[MC3XXX_AXIS_X] = (rbm_data[MC3XXX_AXIS_X] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		raw_data[MC3XXX_AXIS_Y] = (rbm_data[MC3XXX_AXIS_Y] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		raw_data[MC3XXX_AXIS_Z] = (rbm_data[MC3XXX_AXIS_Z] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];

		_pszBuffer = kzalloc(_WRT_LOG_DATA_BUFFER_SIZE, GFP_KERNEL);
		if (NULL == _pszBuffer) {
			GSE_ERR("fail to allocate memory for buffer\n");
			closeFile(fd_file);
			return -ENOMEM;
		}
		memset(_pszBuffer, 0, _WRT_LOG_DATA_BUFFER_SIZE);

		n += snprintf(_pszBuffer+n, 50, "G-sensor RAW X = %d  Y = %d  Z = %d\n", raw_data[0], raw_data[1], raw_data[2]);
		n += snprintf(_pszBuffer+n, 50, "G-sensor RBM X = %d  Y = %d  Z = %d\n", rbm_data[0], rbm_data[1], rbm_data[2]);
		for (i = 0; i < 63; i++)
			n += snprintf(_pszBuffer+n, 50, "mCube register map Register[%x] = 0x%x\n", i, data[i]);
		msleep(50);

		err = writeFile(fd_file, _pszBuffer, n);
		if (err > 0)
			GSE_LOG("buf:%s\n", _pszBuffer);
		else
			GSE_LOG("write file error %d\n", err);

		kfree(_pszBuffer);

		set_fs(oldfs);
		closeFile(fd_file);
	}
	return 0;
}

static int MC3XXX_ReadRegMap(struct i2c_client *p_i2c_client, u8 *pbUserBuf)
{
	#define MC3XXX_REGMAP_LENGTH    (64)

	u8     _baData[MC3XXX_REGMAP_LENGTH] = { 0 };
	int    _nIndex = 0;

	GSE_LOG("[%s]\n", __func__);

	if (NULL == p_i2c_client)
		return -EINVAL;

	for (_nIndex = 0; _nIndex < MC3XXX_REGMAP_LENGTH; _nIndex++) {
		_baData[_nIndex] = i2c_smbus_read_byte_data(p_i2c_client, _nIndex);

		if (NULL != pbUserBuf)
			pbUserBuf[_nIndex] = _baData[_nIndex];

		printk(KERN_INFO "[%s] REG[0x%02X] = 0x%02X\n", __FUNCTION__, _nIndex, _baData[_nIndex]);
	}

	mcube_write_log_data(p_i2c_client, _baData);

	return 0;

	#undef MC3XXX_REGMAP_LENGTH
}


void MC3XXX_Reset(struct i2c_client *client)
{
	unsigned char    _baBuf[2] = { 0 };

	s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
	s32 x_off = 0, y_off = 0, z_off = 0;
	u8  bMsbFilter       = 0x3F;
	s16 wSignBitMask     = 0x2000;
	s32 wSignPaddingBits = 0xC000;
	int ret = 0;

	_baBuf[0] = 0x43;
	ret = i2c_smbus_write_byte_data(client, 0x07, _baBuf[0]);
	if (ret < 0)
		GSE_LOG("%s:i2c_smbus_write_byte_data failed\n", __func__);

	_baBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

	if (0x00 == (_baBuf[0] & 0x40)) {
		_baBuf[0] = 0x6D;
		i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);

		_baBuf[0] = 0x43;
		i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);
	}

	_baBuf[0] = 0x43;
	i2c_smbus_write_byte_data(client, 0x07, _baBuf[0]);

	_baBuf[0] = 0x80;
	i2c_smbus_write_byte_data(client, 0x1C, _baBuf[0]);

	_baBuf[0] = 0x80;
	i2c_smbus_write_byte_data(client, 0x17, _baBuf[0]);

	msleep(5);

	_baBuf[0] = 0x00;
	i2c_smbus_write_byte_data(client, 0x1C, _baBuf[0]);

	_baBuf[0] = 0x00;
	i2c_smbus_write_byte_data(client, 0x17, _baBuf[0]);

	msleep(5);

	i2c_smbus_read_i2c_block_data(client, 0x21, 9, offset_buf);

	_baBuf[0] = i2c_smbus_read_byte_data(client, 0x04);

	if (_baBuf[0] & 0x40) {
		_baBuf[0] = 0x6D;
		i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);

		_baBuf[0] = 0x43;
		i2c_smbus_write_byte_data(client, 0x1B, _baBuf[0]);
	}

	if (IS_MCFM12() || IS_MCFM3X()) {
		bMsbFilter       = 0x7F;
		wSignBitMask     = 0x4000;
		wSignPaddingBits = 0x8000;
	}

	tmp = ((offset_buf[1] & bMsbFilter) << 8) + offset_buf[0];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	x_off = tmp;

	tmp = ((offset_buf[3] & bMsbFilter) << 8) + offset_buf[2];
	if (tmp & wSignBitMask)
			tmp |= wSignPaddingBits;
	y_off = tmp;

	tmp = ((offset_buf[5] & bMsbFilter) << 8) + offset_buf[4];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	z_off = tmp;

	/* get x,y,z gain*/
	x_gain = ((offset_buf[1] >> 7) << 8) + offset_buf[6];
	y_gain = ((offset_buf[3] >> 7) << 8) + offset_buf[7];
	z_gain = ((offset_buf[5] >> 7) << 8) + offset_buf[8];

	/*storege the cerrunt offset data with DOT format*/
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	/*storege the cerrunt Gain data with GOT format*/
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);

	GSE_LOG("offser gain = %d %d %d %d %d %d======================\n\n ", gain_data[0], gain_data[1], gain_data[2], offset_data[0], offset_data[1], offset_data[2]);
}

static void MC3XXX_SaveDefaultOffset(struct i2c_client *p_i2c_client)
{
	GSE_LOG("[%s]\n", __func__);

	i2c_smbus_read_i2c_block_data(p_i2c_client, 0x21, 3, &s_baOTP_OffsetData[0]);
	i2c_smbus_read_i2c_block_data(p_i2c_client, 0x24, 3, &s_baOTP_OffsetData[3]);

	GSE_LOG("s_baOTP_OffsetData: 0x%02X - 0x%02X - 0x%02X - 0x%02X - 0x%02X - 0x%02X\n",
		s_baOTP_OffsetData[0], s_baOTP_OffsetData[1], s_baOTP_OffsetData[2],
		s_baOTP_OffsetData[3], s_baOTP_OffsetData[4], s_baOTP_OffsetData[5]);
}

int mc3xxx_read_accel_xyz(struct i2c_client *client, s16 *acc)
{
	int ret = 0;
	s16 raw_data[MC3XXX_AXES_NUM] = { 0 };
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	#ifdef DOT_CALI
	ret = MC3XXX_ReadData(client, &raw_data[0]);
	#else
	unsigned char raw_buf[6] = {0};
	signed char raw_buf1[3] = {0};

	if (MC3XXX_RESOLUTION_HIGH == s_bResolution) {
		ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, 6, raw_buf);

		acc[0] = (signed short)((raw_buf[0])|(raw_buf[1]<<8));
		acc[1] = (signed short)((raw_buf[2])|(raw_buf[3]<<8));
		acc[2] = (signed short)((raw_buf[4])|(raw_buf[5]<<8));
	} else if (MC3XXX_RESOLUTION_LOW == s_bResolution) {
		ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_REG, 3, raw_buf1);

		acc[0] = (signed short)raw_buf1[0];
		acc[1] = (signed short)raw_buf1[1];
		acc[2] = (signed short)raw_buf1[2];
	}

	raw_data[MC3XXX_AXIS_X] = acc[MC3XXX_AXIS_X];
	raw_data[MC3XXX_AXIS_Y] = acc[MC3XXX_AXIS_Y];
	raw_data[MC3XXX_AXIS_Z] = acc[MC3XXX_AXIS_Z];

	MCUBE_RREMAP(raw_data[MC3XXX_AXIS_X], raw_data[MC3XXX_AXIS_Y]);
	#endif

	acc[MC3XXX_AXIS_X] = (data->pdata.negate_x ? -1:1) * raw_data[data->pdata.axis_map_x];
	acc[MC3XXX_AXIS_Y] = (data->pdata.negate_y ? -1:1) * raw_data[data->pdata.axis_map_y];
	acc[MC3XXX_AXIS_Z] = (data->pdata.negate_z ? -1:1) * raw_data[data->pdata.axis_map_z];

	mcprintkreg("MC3XXX_DataAferMap: %d %d %d\n", acc[MC3XXX_AXIS_X], acc[MC3XXX_AXIS_Y] , acc[MC3XXX_AXIS_Z]);

	return ret;
}

static void mc3xxx_work_func(struct work_struct *work)
{
	struct mc3xxx_data *data = container_of(work, struct mc3xxx_data, work);
	struct acceleration accel = { 0 };
	s16 raw[3] = { 0 };
	int ret = 0;
	ktime_t timestamp;

	ret = mc3xxx_read_accel_xyz(data->client, &raw[0]);

	if (ret < 0)
		GSE_ERR("mc3xxx_read_accel_xyz ERROR!!!\n");
	else {
		if (MC3XXX_RESOLUTION_LOW == s_bResolution) {
			raw[0] = raw[0] << 4;
			raw[1] = raw[1] << 4;
			raw[2] = raw[2] << 4;
		}
		
		accel.x = raw[0];
		accel.y = raw[1];
		accel.z = raw[2];
		timestamp = ktime_get_boottime();
		input_report_abs(data->input_dev, ABS_X, accel.x);
		input_report_abs(data->input_dev, ABS_Y, accel.y);
		input_report_abs(data->input_dev, ABS_Z, accel.z);
		input_event(data->input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
		input_event(data->input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
		input_sync(data->input_dev);
	}
}

static enum hrtimer_restart mc3xxx_timer_func(struct hrtimer *timer)
{
	struct mc3xxx_data *data = container_of(timer, struct mc3xxx_data, timer);
	if (data->enabled) {
		queue_work(data->mc3xxx_wq, &data->work);

		hrtimer_start(&data->timer,
						ktime_set(0, (data->last_poll_interval)*1000000),
						HRTIMER_MODE_REL);
	} else
		printk("mc3xxx chip is not enabled\n");

	return HRTIMER_NORESTART;
}

static bool check_wake_function(struct mc3xxx_data *data)
{
	int ret = 0;

	if (!data->power_enabled) {
		printk("chip not powered.\n");
		return false;
	} else if (!data->chip_mode) {
		mc3xxx_chip_init(data->client);
		ret = mc3xxx_set_mode(data->client, MC3XXX_WAKE);
		if (ret < 0)
			return false;
	}
	return true;
}

static int mc3xxx_enable_set(struct sensors_classdev *sensors_cdev,
					unsigned int enabled)
{
	struct mc3xxx_data *data =
		container_of(sensors_cdev, struct mc3xxx_data, cdev);
	struct input_dev *input_dev = data->input_dev;
	int ret = 0;

	if ((enabled < 0) || (enabled > 1)) {
		GSE_ERR("%s(): It is an illegal para %d!\n", __func__, enabled);
		return -EINVAL;
	}

	mutex_lock(&input_dev->mutex);
	if (enabled != data->enabled) {
		if (enabled) {
			printk("mc3xxx_enable_set ENABLE,last_poll_interval=%d\n",
				data->last_poll_interval);
			if (!check_wake_function(data)) {
				GSE_ERR("%s(): Chip not functioned, report nothing!\n", __func__);
				ret = -EINVAL;
			} else {
				hrtimer_start(&data->timer,
					ktime_set(0, (data->last_poll_interval)*1000000),
					HRTIMER_MODE_REL);
			}
		} else {
			printk("mc3xxx_enable_set DIS_ENABLE\n");
			hrtimer_cancel(&data->timer);
		}
		data->enabled = enabled;
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int mc3xxx_poll_delay_set(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec)
{
	struct mc3xxx_data *data =
		container_of(sensors_cdev, struct mc3xxx_data, cdev);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	data->last_poll_interval = max(delay_msec, (unsigned int)(data->cdev.min_delay / 1000));

	mutex_unlock(&input_dev->mutex);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mc3xxx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int ret = 0;

	printk("%s\n", __func__);
	mutex_lock(&input_dev->mutex);

	if (data->enabled) {
		hrtimer_cancel(&data->timer);
		ret = mc3xxx_set_mode(data->client, MC3XXX_STANDBY);
		if (ret < 0)
			GSE_ERR("%s:mc3xxx_set_mode failed\n", __func__);
	}
	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int mc3xxx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int ret = 0;


	printk("%s\n", __func__);
	mutex_lock(&input_dev->mutex);

	if (data->enabled) {
		mc3xxx_chip_init(data->client);
		#ifdef DOT_CALI
		MC3XXX_ResetCalibration(data->client);
		mcube_read_cali_file(data->client);
		#endif
		ret = mc3xxx_set_mode(data->client, MC3XXX_WAKE);
		if (ret < 0)
			GSE_ERR("%s:mc3xxx_set_mode failed\n", __func__);

		hrtimer_start(&data->timer,
						ktime_set(0, (data->last_poll_interval)*1000000),
						HRTIMER_MODE_REL);
	}
	mutex_unlock(&input_dev->mutex);
	return 0;
}

#endif

/**
 * gsensor_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
static int gsensor_fetch_sysconfig_para(void)
{
	u_i2c_addr.dirty_addr_buf[0] = 0x4c;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

	return 0;
}

/****************************************
***mc3xxx_power_on
*****************************************/
static int mc3xxx_power_on(struct mc3xxx_data *data, bool on)
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

/****************************************
***mc3xxx_power_init
*****************************************/
static int mc3xxx_power_init(struct mc3xxx_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, MC3XXX_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, MC3XXX_VIO_MAX_UV);

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
			rc = regulator_set_voltage(data->vdd, MC3XXX_VDD_MIN_UV,
						   MC3XXX_VDD_MAX_UV);
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
			rc = regulator_set_voltage(data->vio, MC3XXX_VIO_MIN_UV,
						   MC3XXX_VIO_MAX_UV);
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
		regulator_set_voltage(data->vdd, 0, MC3XXX_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

/****************************************
***mc3xxx_device_power_off
*****************************************/
static void mc3xxx_device_power_off(struct mc3xxx_data *data)
{
	int err;

	data->mode_reg &= MC3XXX_STANDBY_MODE_MASK;
	err = i2c_smbus_write_byte_data(data->client, MC3XXX_MODE_FEATURE_REG, data->mode_reg);
	if (err < 0)
		dev_err(&data->client->dev, "device goto standby failed\n");

	mc3xxx_power_on(data, false);

	dev_dbg(&data->client->dev, "soft power off complete.\n");
	return ;
}

static int mc3xxx_input_init(struct mc3xxx_data *data)
{
	int ret = 0;

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		dev_err(&data->client->dev,
				"could not allocate input device\n");
		return -ENOMEM;
	}

	set_bit(EV_ABS, data->input_dev->evbit);
	input_set_abs_params(data->input_dev, ABS_X, -GMAX_14BIT, GMAX_14BIT, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Y, -GMAX_14BIT, GMAX_14BIT, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Z, -GMAX_14BIT, GMAX_14BIT, INPUT_FUZZ, INPUT_FLAT);

	data->input_dev->name = SENSOR_NAME;

	ret = input_register_device(data->input_dev);
	if (ret) {
		dev_err(&data->client->dev,
				"could not register input device\n");
		goto err_free_input_device;
	}

	return ret;

err_free_input_device:
	input_free_device(data->input_dev);
	return ret;
}


/****************************************
***mc3xxx_parse_dt
*****************************************/
static int mc3xxx_parse_dt(struct device *dev,
				struct mc3xxx_platform_data *mc3xxx_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	/*
	rc = of_property_read_u32(np, "mcube,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else
		mc3xxx_pdata->min_interval = temp_val;
	*/
	rc = of_property_read_u32(np, "mcube,init-interval", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else
		mc3xxx_pdata->init_interval = temp_val;

	rc = of_property_read_u32(np, "mcube,axis-map-x", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else
		mc3xxx_pdata->axis_map_x = (u8)temp_val;

	rc = of_property_read_u32(np, "mcube,axis-map-y", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read axis_map_y\n");
		return rc;
	} else
		mc3xxx_pdata->axis_map_y = (u8)temp_val;

	rc = of_property_read_u32(np, "mcube,axis-map-z", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read axis-map-z\n");
		return rc;
	} else
		mc3xxx_pdata->axis_map_z = (u8)temp_val;

	rc = of_property_read_u32(np, "mcube,g-range", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
		case 2:
			mc3xxx_pdata->g_range = CM3XXX_G_2G;
			break;
		case 4:
			mc3xxx_pdata->g_range = CM3XXX_G_4G;
			break;
		case 8:
			mc3xxx_pdata->g_range = CM3XXX_G_8G;
			break;
		case 12:
			mc3xxx_pdata->g_range = CM3XXX_G_12G;
			break;
		case 16:
			mc3xxx_pdata->g_range = CM3XXX_G_16G;
			break;
		default:
			mc3xxx_pdata->g_range = CM3XXX_G_2G;
			break;
		}
	}

	mc3xxx_pdata->negate_x = of_property_read_bool(np, "mcube,negate-x");

	mc3xxx_pdata->negate_y = of_property_read_bool(np, "mcube,negate-y");

	mc3xxx_pdata->negate_z = of_property_read_bool(np, "mcube,negate-z");

	if (of_property_read_bool(np, "mcube,res-14bit"))
		mc3xxx_pdata->res_ctl = RES_14BIT;
	else
		mc3xxx_pdata->res_ctl = RES_12BIT;

	return 0;
}



/*****************************************
 *** _mc3xxx_i2c_auto_probe
 *****************************************/
static int mc3xxx_i2c_auto_probe(struct i2c_client *client)
{
	#define _MC3XXX_I2C_PROBE_ADDR_COUNT_    (sizeof(mc3xxx_i2c_auto_probe_addr) / sizeof(mc3xxx_i2c_auto_probe_addr[0]))

	unsigned char    _baData1Buf[2] = { 0 };
	unsigned char    _baData2Buf[2] = { 0 };

	int              _nCount = 0;
	int              _naCheckCount[_MC3XXX_I2C_PROBE_ADDR_COUNT_] = { 0 };

	memset(_naCheckCount, 0, sizeof(_naCheckCount));

_I2C_AUTO_PROBE_RECHECK_:
	s_bPCODE  = 0x00;
	s_bPCODER = 0x00;
	s_bHWID   = 0x00;

	for (_nCount = 0; _nCount < _MC3XXX_I2C_PROBE_ADDR_COUNT_; _nCount++) {
		client->addr = mc3xxx_i2c_auto_probe_addr[_nCount];

		GSE_LOG("[%s][%d] probing addr: 0x%X\n", __FUNCTION__, _nCount, client->addr);

		_baData1Buf[0] = 0x3B;
		if (0 > i2c_master_send(client, &(_baData1Buf[0]), 1)) {
			GSE_ERR("ERR: addr: 0x%X fail to communicate-2!\n", client->addr);
			continue;
		}

		if (0 > i2c_master_recv(client, &(_baData1Buf[0]), 1)) {
			GSE_ERR("ERR: addr: 0x%X fail to communicate-3!\n", client->addr);
			continue;
		}

		_naCheckCount[_nCount]++;

		GSE_LOG("[%s][%d] addr: 0x%X ok to read REG(0x3B): 0x%X\n", __FUNCTION__, _nCount, client->addr, _baData1Buf[0]);

		if (0x00 == _baData1Buf[0]) {
			if (1 == _naCheckCount[_nCount]) {
				MC3XXX_Reset(client);
				msleep(3);
				goto _I2C_AUTO_PROBE_RECHECK_;
			} else
				continue;
		}

		_baData2Buf[0] = 0x18;
		i2c_master_send(client, &(_baData2Buf[0]), 1);
		i2c_master_recv(client, &(_baData2Buf[0]), 1);

		s_bPCODER = _baData1Buf[0];

		if (MC3XXX_RETCODE_SUCCESS == mc3xxx_validate_sensor_IC(&_baData1Buf[0], &_baData2Buf[0])) {
			s_bPCODE = _baData1Buf[0];
			s_bHWID  = _baData2Buf[0];

			MC3XXX_SaveDefaultOffset(client);

			printk("[%s] addr: 0x%X confirmed ok to use. s_bPCODE: 0x%02X, s_bHWID: 0x%02X\n", __FUNCTION__, client->addr, s_bPCODE, s_bHWID);

			return MC3XXX_RETCODE_SUCCESS;
		}
	}

	return MC3XXX_RETCODE_ERROR_I2C;

	#undef _MC3XXX_I2C_PROBE_ADDR_COUNT_
}

static int mc3xxx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct mc3xxx_data *data = NULL;
	printk("%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	data = kzalloc(sizeof(struct mc3xxx_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	data->client = client;

	if (client->dev.of_node) {
		memset(&data->pdata, 0 , sizeof(data->pdata));
		ret = mc3xxx_parse_dt(&client->dev, &data->pdata);
		if (ret) {
			dev_err(&client->dev, "Unable to parse platfrom data err=%d\n", ret);
			goto err_parse_dt;
		}
	} else {
		ret = -EINVAL;
		goto err_parse_dt;
	}

	data->power_enabled = false;
	data->chip_mode = false;

	ret = mc3xxx_power_init(data, true);
	if (ret < 0) {
		dev_err(&client->dev, "power init failed! err=%d", ret);
		goto err_power_init;
	}

	ret = mc3xxx_power_on(data, true);
	if (ret < 0) {
		dev_err(&client->dev, "power on failed! err=%d\n", ret);
		goto err_power_on;
	}

	ret = mc3xxx_i2c_auto_probe(client);
	if (ret != MC3XXX_RETCODE_SUCCESS) {
		GSE_ERR("ERR: fail to probe mCube sensor!\n");
		goto err_i2c_auto_probe;
	}

#ifdef DOT_CALI
		load_cali_flg = 30;
		MC3XXX_Reset(client);
#endif

	printk("[%s] confirmed i2c addr: 0x%X\n", __FUNCTION__, client->addr);

	i2c_set_clientdata(client, data);

	ret = mc3xxx_input_init(data);
	if (ret) {
		dev_err(&client->dev, "input init failed: %d\n", ret);
		goto err_input_init;
	}

	data->last_poll_interval = data->pdata.init_interval;
	data->cdev = sensors_cdev;
	/* The min_delay is used by userspace and the unit is microsecond. */
	//data->cdev.min_delay = data->pdata.min_interval * 1000;
	data->cdev.delay_msec = data->pdata.init_interval;
	data->cdev.sensors_enable = mc3xxx_enable_set;
	data->cdev.sensors_poll_delay = mc3xxx_poll_delay_set;
	ret = sensors_classdev_register(&data->input_dev->dev, &data->cdev);
	if (ret) {
		dev_err(&client->dev, "class device create failed: %d\n", ret);
		goto err_register_class;
	}

	ret = sysfs_create_group(&((data->cdev.dev)->kobj), &mc3xxx_group);
	if (ret)
		goto err_create_group;

	data->mc3xxx_wq = create_singlethread_workqueue("mc3xxx_wq");
	if (!data->mc3xxx_wq) {
		ret = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	INIT_WORK(&data->work, mc3xxx_work_func);
	mutex_init(&data->lock);

	if (!data->use_irq) {
		hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		data->timer.function = mc3xxx_timer_func;
		/*hrtimer_start(&data->timer, ktime_set(1, 0),
		    HRTIMER_MODE_REL);*/
	}

	printk("mc3xxx probe ok!!! \n");
	productinfo_register(PRODUCTINFO_SENSOR_ACCELEROMETER_ID,
		"mc3xxx", "mCube");

	return 0;

err_create_workqueue_failed:
	sysfs_remove_group(&((data->cdev.dev)->kobj), &mc3xxx_group);
err_create_group:
	sensors_classdev_unregister(&data->cdev);
err_register_class:
	input_unregister_device(data->input_dev);
err_input_init:
err_i2c_auto_probe:
	mc3xxx_device_power_off(data);
err_power_on:
	mc3xxx_power_init(data, false);
err_power_init:
err_parse_dt:
	kfree(data);
	dev_err(&client->dev, "%s(): error exit! ret = %d\n", __func__, ret);
	return ret;
}

static int mc3xxx_remove(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	hrtimer_cancel(&data->timer);
	input_unregister_device(data->input_dev);
	sysfs_remove_group(&((data->cdev.dev)->kobj), &mc3xxx_group);
	mc3xxx_power_init(data, false);
	kfree(data);
	return 0;
}

static SIMPLE_DEV_PM_OPS(mc3xxx_pm_ops, mc3xxx_suspend, mc3xxx_resume);

static const struct i2c_device_id mc3xxx_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};


static struct of_device_id mc3xxx_match_table[] = {
	{ .compatible = "mcube,mc3xxx", },
	{ },
};



MODULE_DEVICE_TABLE(i2c, mc3xxx_id);

static struct i2c_driver mc3xxx_driver = {
.class = I2C_CLASS_HWMON,
.driver = {
.owner = THIS_MODULE,
.name	 = SENSOR_NAME,
.of_match_table = mc3xxx_match_table,
.pm	= &mc3xxx_pm_ops,
},
.id_table	  = mc3xxx_id,
.probe		  = mc3xxx_probe,
.remove		  = mc3xxx_remove,
.address_list = u_i2c_addr.normal_i2c,
};

static int __init mc3xxx_init(void)
{
	int ret = -1;

	printk("mc3xxx: init\n");

	if (gsensor_fetch_sysconfig_para()) {
		GSE_ERR("%s: err.\n", __func__);
		return -EPERM;
	}

	GSE_LOG("%s: after fetch_sysconfig_para:  \normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx\n",
			__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	ret = i2c_add_driver(&mc3xxx_driver);

	return ret;
}

static void __exit mc3xxx_exit(void)
{
	i2c_del_driver(&mc3xxx_driver);
}

module_init(mc3xxx_init);
module_exit(mc3xxx_exit);

MODULE_DESCRIPTION("mc3xxx accelerometer driver");
MODULE_AUTHOR("Lisa,Liu <liuwenshu@hisense.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(SENSOR_DRIVER_VERSION);
module_param(mc3xxx_debug, int, 0600);
