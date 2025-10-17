#ifndef __ELAN_EPL259x__
#define __ELAN_EPL259x__

#define LOG_TAG					"[epl88055]:"
#define LOG_FUN(f)				printk(KERN_INFO LOG_TAG " %s\n", __FUNCTION__)
#define LOG_INFO(fmt, args...)	printk(KERN_INFO LOG_TAG fmt, ##args)
#define LOG_ERR(fmt, args...)	printk(KERN_ERR  LOG_TAG " %s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#define PACKAGE_SIZE		8
#define I2C_RETRY_COUNT		10
#define I2C_MAX_COUNT		8

#define EPL_DEV_NAME   		    "epl_alsps"
#define DRIVER_VERSION          "1.0.8"

#define LUX_PER_COUNT			700

#define ALS_LEVEL    16

/* POWER SUPPLY VOLTAGE RANGE */
#define EPLD259x_VDD_MIN_UV  2000000
#define EPLD259x_VDD_MAX_UV  3300000
#define EPLD259x_VIO_MIN_UV  1750000
#define EPLD259x_VIO_MAX_UV  1950000

#define EPL_PS_CROSSTALK_MAX	1000

#define ELAN_LS_8852 	"elan-epl8852"
#define ELAN_LS_8882 	"elan-epl8882"
#define ELAN_LS_88051 	"elan-epl88051"

#define NEAR_CODE	1
#define FAR_CODE	0
#define FLUSH_MASK	2
#define FLUSH_UNMASK	(~FLUSH_MASK)

typedef enum {
    CMC_BIT_RAW				= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_DEF_LIGHT		= 0x4,
    CMC_BIT_TABLE			= 0x8,
} CMC_ALS_REPORT_TYPE;

struct epl_raw_data {
    u8 raw_bytes[PACKAGE_SIZE];
    u16 renvo;
};

#define LIGHT_DEFAULT_DELAY		500   /* 500 ms */

/************************************************************************************/
#define EPL_REG_CTRL_1		0x00
#define EPL_REG_ALS_CTRL_1	0x01
#define EPL_REG_ALS_CTRL_2	0x02
#define EPL_REG_PS_CTRL_1	0x03
#define EPL_REG_PS_CTRL_2	0x04
#define EPL_REG_PS_CTRL_3	0x05
#define EPL_REG_INT_CTRL_1	0x06
#define EPL_REG_INT_CTRL_2	0x07
#define EPL_REG_ALS_THD_L	0x08
#define EPL_REG_ALS_THD_H	0x0A
#define EPL_REG_PS_THD_L	0x0C
#define EPL_REG_PS_THD_H	0x0E
#define EPL_REG_CTRL_2		0x11
#define EPL_REG_ALS_STATE	0x12
#define EPL_REG_ALS_DATA_CH0	0x13
#define EPL_REG_ALS_DATA_CH1	0x15
#define EPL_REG_PS_STATE	0x1B
#define EPL_REG_PS_DATA_CH0	0x1C
#define EPL_REG_PS_DATA_CH1	0x1E
#define EPL_REG_VER			0x21
#define EPL_REG_PS_CANCEL	0x22

#define EPL_WAIT_0_MS			(0x0<<4)
#define EPL_WAIT_2_MS			(0x1<<4)
#define EPL_WAIT_4_MS			(0x2<<4)
#define EPL_WAIT_8_MS			(0x3<<4)
#define EPL_WAIT_12_MS			(0x4<<4)
#define EPL_WAIT_20_MS			(0x5<<4)
#define EPL_WAIT_30_MS			(0x6<<4)
#define EPL_WAIT_40_MS			(0x7<<4)
#define EPL_WAIT_50_MS			(0x8<<4)
#define EPL_WAIT_75_MS			(0x9<<4)
#define EPL_WAIT_100_MS		    (0xA<<4)
#define EPL_WAIT_150_MS		    (0xB<<4)
#define EPL_WAIT_200_MS		    (0xC<<4)
#define EPL_WAIT_300_MS		    (0xD<<4)
#define EPL_WAIT_400_MS		    (0xE<<4)
#define EPL_WAIT_SINGLE		    (0x0F << 4)

#define EPL_MODE_IDLE		(0x00)
#define EPL_MODE_ALS		(0x01)
#define EPL_MODE_PS			(0x02)
#define EPL_MODE_ALS_PS		(0x03)

#define POWER_DOWN		    (1)
#define POWER_WAKE			(0)

#define RESET				(0<<1)
#define RUN					(1<<1)

#define EPL_ALS_INTT_2			(0<<2)
#define EPL_ALS_INTT_4			(1<<2)
#define EPL_ALS_INTT_8			(2<<2)
#define EPL_ALS_INTT_16			(3<<2)
#define EPL_ALS_INTT_32			(4<<2)
#define EPL_ALS_INTT_64			(5<<2)
#define EPL_ALS_INTT_128		(6<<2)
#define EPL_ALS_INTT_256		(7<<2)
#define EPL_ALS_INTT_512		(8<<2)
#define EPL_ALS_INTT_768		(9<<2)
#define EPL_ALS_INTT_1024		(10<<2)
#define EPL_ALS_INTT_2048		(11<<2)
#define EPL_ALS_INTT_4096		(12<<2)
#define EPL_ALS_INTT_6144		(13<<2)
#define EPL_ALS_INTT_8192		(14<<2)
#define EPL_ALS_INTT_10240		(15<<2)

#define EPL_GAIN_HIGH		(0x00)
#define EPL_GAIN_MID		(0x01)
#define EPL_GAIN_LOW		    (0x03)

#define EPL_PSALS_ADC_11	(0x00 << 3)
#define EPL_PSALS_ADC_12	(0x01 << 3)
#define EPL_PSALS_ADC_13	(0x02 << 3)
#define EPL_PSALS_ADC_14	(0x03 << 3)

#define EPL_CYCLE_1			(0x00)
#define EPL_CYCLE_2			(0x01)
#define EPL_CYCLE_4			(0x02)
#define EPL_CYCLE_8			(0x03)
#define EPL_CYCLE_16		(0x04)
#define EPL_CYCLE_32		(0x05)
#define EPL_CYCLE_64		(0x06)

#define EPL_PS_INTT_4			(0<<2)
#define EPL_PS_INTT_8			(1<<2)
#define EPL_PS_INTT_16			(2<<2)
#define EPL_PS_INTT_24			(3<<2)
#define EPL_PS_INTT_32			(4<<2)
#define EPL_PS_INTT_48			(5<<2)
#define EPL_PS_INTT_80			(6<<2)
#define EPL_PS_INTT_144			(7<<2)
#define EPL_PS_INTT_272			(8<<2)
#define EPL_PS_INTT_384			(9<<2)
#define EPL_PS_INTT_520			(10<<2)
#define EPL_PS_INTT_784			(11<<2)
#define EPL_PS_INTT_1040		(12<<2)
#define EPL_PS_INTT_2064		(13<<2)
#define EPL_PS_INTT_4112		(14<<2)
#define EPL_PS_INTT_6160		(15<<2)

#define EPL_IR_ON_CTRL_OFF	(0x00 << 5)
#define EPL_IR_ON_CTRL_ON	(0x01 << 5)

#define EPL_IR_MODE_CURRENT	(0x00 << 4)
#define EPL_IR_MODE_VOLTAGE	(0x01 << 4)

#define EPL_IR_DRIVE_200	(0x00)
#define EPL_IR_DRIVE_100	(0x01)
#define EPL_IR_DRIVE_50		(0x02)
#define EPL_IR_DRIVE_10		(0x03)


#define EPL_INT_CTRL_ALS_OR_PS		(0x00 << 4)
#define EPL_INT_CTRL_ALS			(0x01 << 4)
#define EPL_INT_CTRL_PS				(0x02 << 4)

#define EPL_PERIST_1		(0x00 << 2)
#define EPL_PERIST_4		(0x01 << 2)
#define EPL_PERIST_8		(0x02 << 2)
#define EPL_PERIST_16		(0x03 << 2)

#define EPL_INTTY_DISABLE	(0x00)
#define EPL_INTTY_BINARY	(0x01)
#define EPL_INTTY_ACTIVE	(0x02)
#define EPL_INTTY_FRAME	    (0x03)

#define EPL_RESETN_RESET	(0x00 << 1)
#define EPL_RESETN_RUN		(0x01 << 1)

#define EPL_POWER_OFF		(0x01)
#define EPL_POWER_ON		(0x00)

#define EPL_ALS_INT_CHSEL_0	(0x00 << 4)
#define EPL_ALS_INT_CHSEL_1	(0x01 << 4)

#define EPL_SATURATION				(0x01 << 5)
#define EPL_SATURATION_NOT		    (0x00 << 5)

#define EPL_CMP_H_TRIGGER		(0x01 << 4)
#define EPL_CMP_H_CLEAR		    (0x00 << 4)

#define EPL_CMP_L_TRIGGER		(0x01 << 3)
#define EPL_CMP_L_CLEAR		    (0x00 << 3)

#define EPL_INT_TRIGGER		(0x01 << 2)
#define EPL_INT_CLEAR		(0x00 << 2)

#define EPL_CMP_RESET		(0x00 << 1)
#define EPL_CMP_RUN			(0x01 << 1)

#define EPL_LOCK			(0x01)
#define EPL_UN_LOCK		(0x00)

#define EPL_OSC_SEL_1MHZ	(0x07)

#define EPL_REVNO       (0x81)

#define EPL_A_D      (0x7 << 4)

#define EPL_NORMAL      (0 << 3)
#define EPL_BYBASS      (1 << 3)

#define EPL_GFIN_DISABLE      (0 << 2)
#define EPL_GFIN_ENABLE       (1 << 2)

#define EPL_VOS_DISABLE      (0 << 1)
#define EPL_VOS_ENABLE       (1 << 1)

#define EPL_DOC_OFF         (0)
#define EPL_DOC_ON          (1)

struct _ps_data {
	u16 ir_data;
	u16 data;
};

struct _ges_data
{
	u16 ir_data;
	u16 data;
};

#define ALS_CHANNEL_SIZE	2
struct _als_data {
	u16 channels[ALS_CHANNEL_SIZE];
	u16 lux;
};

struct _ps_setting {
	bool polling_mode;
	u8 integration_time;
	u8 gain;
	u8 adc;
	u8 cycle;
	u16 high_threshold;
	u16 low_threshold;
	u8 ir_on_control;
	u8 ir_mode;
	u8 ir_drive;
	u8 persist;
	u8 interrupt_type;
	u8 saturation;
	u8 compare_high;
	u8 compare_low;
	u8 interrupt_flag;
	u8 compare_reset;
	u8 lock;
	u16 cancelation;
	struct _ps_data data;
	int ps_frame_time;
};

struct _als_setting {
	bool polling_mode;
	u8 als_rs;
	u8 lsrc_type;
	u16 dyn_intt_raw;
	u8 integration_time;
	u8 gain;
	u8 adc;
	u8 cycle;
	u8 persist;
	u8 interrupt_type;
	u8 saturation;
	u8 compare_high;
	u8 compare_low;
	u8 interrupt_flag;
	u8 compare_reset;
	u8 lock;
	u8 interrupt_channel_select;
	struct _als_data data;
	int lux_per_count;
	int als_frame_time;
};

struct epl_optical_sensor {
	u8 wait;
	u8 mode;
	u8 osc_sel;
	u8 interrupt_control;
	u8 reset;
	u8 power;
	struct _ps_setting ps;
	struct _als_setting als;
	u16 revno;
};

struct epl_sensor_priv {
	struct i2c_client *client;
	struct epl_optical_sensor epl_sensor;
	struct epl_raw_data gRawData;
	struct mutex epl_mutex;
	struct wake_lock ps_lock;

	struct input_dev *als_input_dev;
	struct sensors_classdev als_cdev;
	struct delayed_work polling_work;
	int als_fittness;
	int enable_lflag;
	int als_numerator;
	int als_root;

    struct input_dev *ps_input_dev;
	struct sensors_classdev ps_cdev;
    struct delayed_work eint_work;
    struct delayed_work ps_poll_dwork;
    int enable_pflag;	
	atomic_t chip_suspend;
	uint16_t ps_crosstalk_max;

    int intr_pin;
    int irq;

	struct regulator *vdd;
	struct regulator *vio;
} ;
#endif
