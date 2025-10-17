/* Copyright (c) 2013, HMCT. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MEretHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define QPNP_REG_TYPE		0x04
#define QPNP_REG_SUB_TYPE	0x05
#define QPNP_REG_STATUS		0x08
#define BIT_VREG_OK		7
#define BIT_VREG_NPM		1
#define BIT_SMPS_VREG_FAULT	6

#define QPNP_REG_STATUS2	0x09
#define BIT_LDO_VREG_ON		5
#define BIT_LDO_VREG_OC		6


#define QPNP_REG_MODE		0x45
#define QPNP_REG_EN		0x46
#define BIT_SMPS_VREG_ON	7

#define QPNP_REG_VL_CTL1	0x40
#define QPNP_REG_VL_CTL2	0x41
#define QPNP_REG_VL_CTL3	0x44
#define BIT_PFM_VOFFSET_EN	7 
	
#define QPNP_REG_CT_LTD		0x4A
#define BIT_CURRENT_LIM_EN 	7


#define	QPNP_REG_SMPS_OFFS	0x300
#define QPNP_REG_LDO_OFFS	0x100


#define QPNP_PIN_STATUS		0x08
#define BIT_PIN_OK		7
#define BIT_PIN_VL		0

#define QPNP_PIN_MODE		0x40
#define PIN_MODE_FUNC(VAL)	((VAL & 0x70) >> 4)
#define PIN_MODE_SRC(VAL)	(VAL & 0x0f)
#define GPIO_SPEC_FUNC(VAL)	((VAL == 0x4 || VAL ==0x6) ?(VAL/2 -1): 0)
#define GPIO_SPEC_CALC(VAL,NUM) ((NUM -1)*2 + VAL -1)

#define QPNP_PIN_VIN		0x41
#define PIN_VIN_SEL(VAL)	(VAL & 0x07)

#define QPNP_PIN_PULL		0x42
#define PIN_PULL_SEL(VAL)	(VAL & 0x07)

#define QPNP_PIN_IN		0x43
#define QPIN_GPIO_OUT		0x45
#define GPIO_OUT_TYPE(VAL)	((VAL & 0x30) >>4)
#define GPIO_OUT_DRV(VAL)	(VAL & 0x03)		

#define QPNP_PIN_EN		0x46
#define BIT_PIN_EN		7

#define QPNP_MPP_AOUT		0x48
#define MPP_AOUT_REF(VAL)	(VAL & 0x07)

#define QPNP_MPP_AIN		0x4A
#define MPP_AIN_ROUTE(VAL)	(VAL & 0x07)

#define QPNP_MPP_SINK		0x4C
#define MPP_SINK_SEL(VAL)	(VAL & 0x07)


#define QPNP_PIN_OFFS		0x100

#define QPNP_GET_BIT(VAL,B)	((VAL & BIT(B)) >> B)


#define BORAD_ID_STR		"board_id"


static const char * const mpp_func[]=
{
	"D_IN",
	"D_OUT",
	"D_IN_OUT",
	"Reserved",
	"A_IN",
	"A_OUT",
	"CUR_SINK",
	"Reserved"
};

static const char * const gpio_func[]=
{
	"D_IN",
	"D_OUT",
	"D_IN_OUT",
	"Reserved",
};

static const char * const gpio_spec_func[]=
{
	"GND", "SCAN_OUT<3>",
	"DIV_CLK2", "SCAN_OUT<4>",
	"BATT_ALARM_OUT", "SCAN_OUT<5>",
	"SLEEP_CLK2", "SCAN_OUT<6>",
};

static const char * const mpp_pull[]=
{
	"0.6K",
	"OPEN",
	"10K",
	"30K",
};

static const char * const gpio_pull[]=
{
	"UP_30",
	"UP_1p5",
	"UP_31p5",
	"UP_1p5->30",
	"DN_10",
	"NO_PL",
	"Reserved",
	"Reserved",
};

static const char * const gpio_out_type[]=
{
	"CMOS",
	"NMOS",
	"PMOS",
	"Reserved",
};

static const char * const gpio_out_drv[]=
{
	"Reserved",
	"LOW",
	"MID",
	"HIGH",
};

static const char * const mpp_sink[]=
{
	"5",
	"10",
	"15",
	"20",
	"25",
	"30",
	"35",
	"40",
};

enum qpnp_regulator_type {
	HF_SMPS	= 0x22,
	LDO	= 0x21,
	LOW_noise_LDO = 0x04,
};

enum qpnp_regulator_sub_type {
	ULT_BUCK_CTL1	= 0x0d,
	ULT_BUCK_CTL2	= 0x0d,
	ULT_BUCK_CTL3	= 0x0d,
	ULT_BUCK_CTL4	= 0x10,
	N300_LDO		= 0x15,
	N600_LDO		= 0x06,
	N1200_LDO		= 0x07,
	N900_LDO        = 0x14,
	LV_P50     	    = 0x28,
	LV_P150     	= 0x29,
	LV_P300     	= 0x2a,
	P50_LDO			= 0x08,
	P150_LDO		= 0x09,
	P300_LDO        = 0x0a,
	P450_LDO        = 0x2d,
	P600_LDO    	= 0x0b,
	P1200_LDO       = 0x0c,	
	LN_LDO      	= 0x10,
	
};

typedef struct
{
    u32    range_min;
    u32    range_max;
    u32    vstep;			
}qpnp_reg_volt_info;

struct qpnp_vreg_status {
	u8 npm;
	u8 on_off;
	u8 vreg_ok;
	u32 volt;
	u32 volt_pfm;//only for smps
	u8 over_current;
	u8 oc_en;//only for ldo
	u8 mode;
	u8 ctrl;
};
//only odd PM8x26 MPPs can be configured as analog outputs
//only even PM8x26 MPPs have current sink capability

enum qpnp_pin_type {
	MPP	= 0x11,
	GPIO	= 0x10,
};

struct qpnp_pin_status {
	u8 ok;
	u8 val;
	u8 mode;
	u8 src;
	u8 vol;
	u8 pull;
	u8 dig_in;
	u8 dig_out_type;//GPIO
	u8 dig_out_drv;//GPIO
	u8 en;
	u8 ana_out;//MPP
	u8 ana_in;//MPP
	u8 sink;//MPP
};

struct qpnp_reg_dbg {
	struct spmi_device *spmi;
	bool adc_support;
	bool pins_support;
	u16 ldo_base_addr;
	u16 smps_base_addr;
	u16 mpp_base_addr;
	u16 gpio_base_addr;
	u16 ldo_count;
	u16 smps_count;
	u16 mpp_count;
	u16 gpio_count;
};

static struct qpnp_reg_dbg *vreg_dev;//for proc fs ops

qpnp_reg_volt_info pmos_volt[1] = 
{
    { 1750000,  3337500,         12500},  //same for P50, P150, P300 and P600 PLDO
};

qpnp_reg_volt_info nmos_volt[1] = 
{
    { 375000,   1537500,         12500},  //same for N300, N600 and N1200 NLDO
};

qpnp_reg_volt_info clk_ldo_volt[2] = 
{
    { 690000,   1110000,         60000},
    { 1380000,  2220000,         120000},//CLK LDO for VREG_XO and VREG_RF
};

qpnp_reg_volt_info ult_buck_volt_1[2] = 
{
    { 375000,   1562500,         12500},  //ULT Buck1,2 3 - Range 0
    { 750000,   1525000,         25000},  //ULT Buck1,2 3    Range 1
};

qpnp_reg_volt_info ult_buck_volt_2[1] = 
{
    { 1550000,   3125000,         25000},  //ULT Buck 4
};
extern int qpnp_vadc_get_ch_counts(void);
extern int qpnp_vadc_get_adc_status(struct qpnp_vadc_status *vadc_sts,int ch_num);

extern int32_t qpnp_vadc_read(struct qpnp_vadc_chip *vadc,
	enum qpnp_vadc_channels channel,struct qpnp_vadc_result *result);


static int qpnp_reg_read_u8(struct qpnp_reg_dbg *chip, u8 *data, u16 reg)
{
	int ret;

	ret = spmi_ext_register_readl(chip->spmi->ctrl, chip->spmi->sid,
							reg, data, 1);
	if (ret < 0)
		dev_err(&chip->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, ret);

	return ret;
}
//DEVICE TREE POSITION CALC(sid) FOR GPIO COMP
static int qpnp_reg_read_pins(struct qpnp_reg_dbg *chip, u8 *data, u16 reg)
{
	int ret;

	ret = spmi_ext_register_readl(chip->spmi->ctrl, (chip->spmi->sid-1),
							reg, data, 1);
	if (ret < 0)
		dev_err(&chip->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, ret);

	return ret;
}
/*
static int qpnp_reg_write_u8(struct qpnp_reg_dbg *chip, u8 *data, u16 reg)
{
	int ret;

	ret = spmi_ext_register_writel(chip->spmi->ctrl, chip->spmi->sid,
							reg, data, 1);
	if (ret < 0)
		dev_err(&chip->spmi->dev,
			"Error writing address: %X - ret %X\n", reg, ret);

	return ret;
}*/

static int reg_dbg_get_vreg_volt_info(enum qpnp_regulator_type type,
	enum qpnp_regulator_sub_type sub_type,u8 *array_size,qpnp_reg_volt_info **vreg_volt)
{

	switch (type) {
		
	case HF_SMPS:
		
		switch (sub_type){
			case ULT_BUCK_CTL1:	
			*vreg_volt = ult_buck_volt_1;
			*array_size = sizeof(ult_buck_volt_1)/sizeof(qpnp_reg_volt_info);
			break;
			
			case ULT_BUCK_CTL4:	
			*vreg_volt = ult_buck_volt_2;
			*array_size = sizeof(ult_buck_volt_2)/sizeof(qpnp_reg_volt_info);
			break;
			
			default:
				pr_err("%s():unknow smps type %d\n",__func__,sub_type);
				return -1;	
		}
		break;
		
	case LDO:
	case LOW_noise_LDO:
		
		switch (sub_type) {

			case N300_LDO:
			case N600_LDO:
			case N900_LDO:
			case N1200_LDO:
				*vreg_volt = nmos_volt;
				*array_size = sizeof(nmos_volt)/sizeof(qpnp_reg_volt_info);
				break;

			case P50_LDO:
			case P150_LDO:
			case P300_LDO:
			case P450_LDO:
			case P600_LDO:
			case P1200_LDO:
			case LV_P50:
			case LV_P150:
			case LV_P300:
				*vreg_volt = pmos_volt;
				*array_size = sizeof(pmos_volt)/sizeof(qpnp_reg_volt_info);
				break;
				
			case LN_LDO:
				*vreg_volt = clk_ldo_volt;
				*array_size = sizeof(clk_ldo_volt)/sizeof(qpnp_reg_volt_info);
				break;
				
			default:
				pr_err("%s():unknow ldo type %d\n",__func__,sub_type);
				return -1;	
		} 
		break;
		
	default:
		pr_err("%s():unknow vreg type %d\n",__func__,type);
		return -1;
	}

	return 0;
}


static int reg_dbg_dump_reg_status(struct qpnp_reg_dbg *chip,
	struct qpnp_vreg_status *sts,u16 base_addr)
{
	int ret;
	enum qpnp_regulator_type type;
	enum qpnp_regulator_sub_type  sub_type;
	u8  reg_val;
	u8  volt_array_size;
	qpnp_reg_volt_info *vreg_volt;

	ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_TYPE));
	if(ret)
		return ret;
	type = reg_val;
	printk("lcq type is %x\n",type);
	ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_SUB_TYPE));
	if(ret)
		return ret;
	sub_type = reg_val;
	printk("lcq sub_type is %x\n",sub_type);
	ret = reg_dbg_get_vreg_volt_info(type,sub_type,&volt_array_size,&vreg_volt);
	if(ret)
		return ret;
	
	ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_STATUS));
	if(ret)
		return ret;
	sts->vreg_ok = QPNP_GET_BIT(reg_val,BIT_VREG_OK);
	if(type == LDO)
		sts->npm = QPNP_GET_BIT(reg_val,BIT_VREG_NPM);

	if(type == LDO) {
		ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_STATUS2));
		if(ret)
			return ret;
		sts->on_off = QPNP_GET_BIT(reg_val,BIT_LDO_VREG_ON);
	}
	ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_MODE));
	if(ret)
		return ret;
	sts->mode = reg_val;

	ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_EN));
	if(ret)
		return ret;
	sts->ctrl = reg_val;

	if(type == HF_SMPS)
		sts->on_off = QPNP_GET_BIT(reg_val,BIT_SMPS_VREG_ON);//SMPS Use ctl indicator

	ret = qpnp_reg_read_u8(chip,&reg_val,(base_addr+ QPNP_REG_VL_CTL2));
	if(ret)
		return ret;
	if(type == LDO)
		sts->volt = vreg_volt[0].range_min + reg_val * vreg_volt[0].vstep;
	else if(type == HF_SMPS)
	{
		if(sub_type == ULT_BUCK_CTL1)
		{
			if(reg_val < 96)//1100000
				sts->volt = vreg_volt[0].range_min + reg_val * vreg_volt[0].vstep;
			else
				sts->volt = vreg_volt[1].range_min + reg_val * vreg_volt[1].vstep;
		}
		else if(sub_type == ULT_BUCK_CTL4)
			sts->volt = vreg_volt[0].range_min + reg_val * vreg_volt[0].vstep;
		
	}
		
	return 0;

}

static int reg_dbg_dump_pin_status(struct qpnp_reg_dbg *chip,
	struct qpnp_pin_status *pin_sts,u16 base_addr) 
{	
	int ret;
	enum qpnp_pin_type type;	
	u8  reg_val;	
	ret= qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_REG_TYPE));
	
	if(ret)
		return ret;
	type = reg_val;

	ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_PIN_STATUS));
	if(ret)
		return ret;
	pin_sts->ok  = QPNP_GET_BIT(reg_val,BIT_PIN_OK);
	pin_sts->val = QPNP_GET_BIT(reg_val,BIT_PIN_VL);

	ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_PIN_MODE));
	if(ret)
		return ret;
	pin_sts->mode = PIN_MODE_FUNC(reg_val);
	pin_sts->src = PIN_MODE_SRC(reg_val);

	ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_PIN_VIN));
	if(ret)
		return ret;
	pin_sts->vol = PIN_VIN_SEL(reg_val);


	ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_PIN_PULL));
	if(ret)
		return ret;
	pin_sts->pull = PIN_PULL_SEL(reg_val);

	ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_PIN_IN));
	if(ret)
		return ret;
	pin_sts->dig_in = reg_val;

	if(type == GPIO) {
		ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPIN_GPIO_OUT));
		if(ret)
			return ret;
		pin_sts->dig_out_type = GPIO_OUT_TYPE(reg_val);
		pin_sts->dig_out_drv = GPIO_OUT_DRV(reg_val);
	}

	ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_PIN_EN));
	if(ret)
		return ret;
	pin_sts->en = QPNP_GET_BIT(reg_val,BIT_PIN_EN);

	if(type == MPP) {
		ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_MPP_AOUT));
		if(ret)
			return ret;
		pin_sts->ana_out = MPP_AOUT_REF(reg_val);

		ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_MPP_AIN));
		if(ret)
			return ret;
		pin_sts->ana_in = MPP_AIN_ROUTE(reg_val);

		ret = qpnp_reg_read_pins(chip,&reg_val,(base_addr+ QPNP_MPP_SINK));
		if(ret)
			return ret;
		pin_sts->sink = MPP_SINK_SEL(reg_val);
	}

	return 0;

}


static ssize_t reg_dbg_status_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct qpnp_reg_dbg *chip = dev_get_drvdata(dev);
	struct qpnp_vreg_status reg_sts;
	char * s = buf;
	int ret,i;

        s += sprintf(s, "SMPS Status:\n");
	for( i= 0;i < chip->smps_count;i++)
	{

		ret = reg_dbg_dump_reg_status(chip,&reg_sts,(chip->smps_base_addr + QPNP_REG_SMPS_OFFS * i));
		if(ret >= 0)
		s += sprintf(s, "VREG_S%2d;ON:%u; OK:%u; VOL_PWM:%7u; MODE:0x%2x; CTL:0x%2x\n",(i+1),
	reg_sts.on_off,reg_sts.vreg_ok,reg_sts.volt,reg_sts.mode,reg_sts.ctrl);
		else
		s += sprintf(s, "VREG_S%d status get failed\n",(i+1));
	}


        s += sprintf(s, "LDO Status:\n");
	for( i= 0;i < chip->ldo_count;i++)
	{
		if(i!=15){
		ret = reg_dbg_dump_reg_status(chip,&reg_sts,(chip->ldo_base_addr + QPNP_REG_LDO_OFFS * i));
		if(ret >= 0)
		s += sprintf(s, "VREG_L%2d;ON:%u; OK:%u; NPM:%u; VOL:%7u; MODE:0x%2x; CTL:0x%2x\n",(i+1),
	reg_sts.on_off,reg_sts.vreg_ok,reg_sts.npm,reg_sts.volt,reg_sts.mode,reg_sts.ctrl);
		else
		s += sprintf(s, "VREG_L%d status get failed\n",(i+1));
		}
		else//ldo16 is not supported to read
			continue;
	}

	return (s - buf);
}

static DEVICE_ATTR(status, S_IRUGO, reg_dbg_status_show,
	NULL);

static struct attribute *reg_dbg_attributes[] = {
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group reg_dbg_attr_group = {
	.attrs = reg_dbg_attributes,
};


static int vreg_proc_show(struct seq_file *m, void *v)
{
	struct qpnp_reg_dbg *chip = vreg_dev;
	struct qpnp_vreg_status reg_sts;
	int ret,i;

	if(!chip)
		return 0;

        seq_printf(m, "SMPS Status:\n");
	seq_printf(m, "NAME    ON  OK  VOL_PWM  MODE  CTL\n");
	for( i= 0;i < chip->smps_count;i++)
	{

		ret = reg_dbg_dump_reg_status(chip,&reg_sts,(chip->smps_base_addr + QPNP_REG_SMPS_OFFS * i));
		if(ret >= 0)
		seq_printf(m, "VREG_S%d%3u%3u%10u  0x%2x  0x%2x\n",(i+1),
	reg_sts.on_off,reg_sts.vreg_ok,reg_sts.volt,reg_sts.mode,reg_sts.ctrl);
		else
		seq_printf(m, "VREG_S%d status get failed\n",(i+1));
		memset(&reg_sts, 0, sizeof(reg_sts));
	}


        seq_printf(m, "LDO Status:\n");
	seq_printf(m, "NAME   \t          ON OK NPM   VOL     MODE  CTL\n");
	for( i= 0;i < chip->ldo_count;i++)
	{
		if(i!=15){
		ret = reg_dbg_dump_reg_status(chip,&reg_sts,(chip->ldo_base_addr + QPNP_REG_LDO_OFFS * i));
		if(ret >= 0)
		seq_printf(m, "VREG_L%d \t %3u%3u%3u%10u  0x%2x  0x%2x\n",(i+1),
	reg_sts.on_off,reg_sts.vreg_ok,reg_sts.npm,reg_sts.volt,reg_sts.mode,reg_sts.ctrl);
		else
		seq_printf(m, "VREG_L%1d status get failed\n",(i+1));
		memset(&reg_sts, 0, sizeof(reg_sts));
		}
		else//ldo16 is not supported to read
			continue;
	}
	
	return 0;
}

void vreg_status_sleep_show(void)
{
	struct qpnp_reg_dbg *chip = vreg_dev;
	struct qpnp_vreg_status reg_sts;
	int ret,i;
	
	if(!chip)
		return;

	printk("SMPS Status:\n");
	printk("NAME    ON  OK  VOL_PWM  MODE  CTL\n");
	for( i= 0;i < chip->smps_count;i++)
	{

		ret = reg_dbg_dump_reg_status(chip,&reg_sts,(chip->smps_base_addr + QPNP_REG_SMPS_OFFS * i));
		if(ret >= 0)
		printk("VREG_S%d%3u%3u%10u  0x%2x  0x%2x\n",(i+1),
	reg_sts.on_off,reg_sts.vreg_ok,reg_sts.volt,reg_sts.mode,reg_sts.ctrl);
		else
		printk("VREG_S%d status get failed\n",(i+1));
		memset(&reg_sts, 0, sizeof(reg_sts));
	}


	printk("LDO Status:\n");
	printk("NAME   \t          ON OK NPM   VOL     MODE  CTL\n");
	for( i= 0;i < chip->ldo_count;i++)
	{
	if(i!=15){
		ret = reg_dbg_dump_reg_status(chip,&reg_sts,(chip->ldo_base_addr + QPNP_REG_LDO_OFFS * i));
		if(ret >= 0)
		printk("VREG_L%d \t %3u%3u%3u%10u  0x%2x  0x%2x\n",(i+1),
			reg_sts.on_off,reg_sts.vreg_ok,reg_sts.npm,reg_sts.volt,reg_sts.mode,reg_sts.ctrl);
		else
		printk("VREG_L%1d status get failed\n",(i+1));
		memset(&reg_sts, 0, sizeof(reg_sts));
	}
	else
		continue;
	}
	
}


static int vreg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vreg_proc_show, NULL);
}

static const struct file_operations vreg_proc_fops = {
	.open		= vreg_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int pins_proc_show(struct seq_file *m, void *v)
{
	struct qpnp_reg_dbg *chip = vreg_dev;
	struct qpnp_pin_status pin_sts;
	int ret,i;

	if(!chip)
		return 0;

        seq_printf(m, "MPP Status:\n");
	seq_printf(m, "NAME  EN OK VAL   FUNC   SRC    VOL   PULL  IN  AOUT AIN SINK\n");
	for( i= 0;i < chip->mpp_count;i++)
	{

		ret = reg_dbg_dump_pin_status(chip,&pin_sts,(chip->mpp_base_addr + QPNP_PIN_OFFS * i));
		if(ret >= 0)
		seq_printf(m, "MPP%d%3u%3u%3u %8s  0x%2x   0x%2x   %s 0x%2x 0x%2x 0x%2x %s\n",(i+1),
			pin_sts.en,pin_sts.ok,pin_sts.val,mpp_func[pin_sts.mode],pin_sts.src,pin_sts.vol,
			mpp_pull[pin_sts.pull],pin_sts.dig_in,pin_sts.ana_out,pin_sts.ana_in,mpp_sink[pin_sts.sink]);
		else
		seq_printf(m, "MPP%d status get failed\n",(i+1));
		memset(&pin_sts, 0, sizeof(pin_sts));
	}


        seq_printf(m, "GPIO Status:\n");
	seq_printf(m, "NAME   EN OK VAL  FUNC       SRC      VOL  PULL  IN  DOUT_TYPE DOUT_DRV\n");
	for( i= 0;i < chip->gpio_count;i++)
	{

		ret = reg_dbg_dump_pin_status(chip,&pin_sts,(chip->gpio_base_addr + QPNP_PIN_OFFS * i));
		if(ret >= 0) {
			if(GPIO_SPEC_FUNC(pin_sts.src))
				seq_printf(m, "GPIO%d%3u%3u%3u %8s %12s 0x%2x %s 0x%2x   %s      %s\n",(i+1),
			pin_sts.en,pin_sts.ok,pin_sts.val,gpio_func[pin_sts.mode],gpio_spec_func[GPIO_SPEC_CALC(GPIO_SPEC_FUNC(pin_sts.src),(i+1))],pin_sts.vol,
			gpio_pull[pin_sts.pull],pin_sts.dig_in,gpio_out_type[pin_sts.dig_out_type],gpio_out_drv[pin_sts.dig_out_drv]);
			else
				seq_printf(m, "GPIO%d%3u%3u%3u %8s     0x%2x     0x%2x %s 0x%2x   %s      %s\n",(i+1),
			pin_sts.en,pin_sts.ok,pin_sts.val,gpio_func[pin_sts.mode],pin_sts.src,pin_sts.vol,
			gpio_pull[pin_sts.pull],pin_sts.dig_in,gpio_out_type[pin_sts.dig_out_type],gpio_out_drv[pin_sts.dig_out_drv]);
		}
		else
		seq_printf(m, "GPIO%d status get failed\n",(i+1));
		memset(&pin_sts, 0, sizeof(pin_sts));
	}

	return 0;
}


static int pins_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pins_proc_show, NULL);
}

static const struct file_operations pins_proc_fops = {
	.open		= pins_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static int adc_proc_show(struct seq_file *m, void *v)
{
	struct qpnp_vadc_status adc_sts;
	int ch_counts;
	int i,ret;

	ch_counts = qpnp_vadc_get_ch_counts();

	if(ch_counts <= 0) {
		seq_printf(m, "NO ADC channel availiable!\n");
		return 0;
	}
	else
        	seq_printf(m, " CH          NAME       RAW      VOL(uV)\n");


	for( i= 0;i < ch_counts ;i++)
	{
		ret = qpnp_vadc_get_adc_status(&adc_sts,i);
		if(ret >= 0)
		seq_printf(m, "%3d%17s%8d%11lld\n",adc_sts.ch_num,adc_sts.ch_name,adc_sts.ch_raw,adc_sts.ch_vol);
		else
		seq_printf(m, "ch %3d;name:%15s;result get failed\n",adc_sts.ch_num,adc_sts.ch_name);
	}

	return 0;
}



static int adc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, adc_proc_show, NULL);
}

static const struct file_operations adc_proc_fops = {
	.open		= adc_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int board_proc_show(struct seq_file *m, void *v)
{
	struct qpnp_vadc_status adc_sts;
	int ch_counts;
	int i,ret;

	ch_counts = qpnp_vadc_get_ch_counts();

	if(ch_counts <= 0) {
		seq_printf(m, "NO ADC channel availiable!\n");
		return 0;
	}

	for( i= 0;i < ch_counts ;i++)
	{
		ret = qpnp_vadc_get_adc_status(&adc_sts,i);
		if(ret >= 0 && strcmp(adc_sts.ch_name,BORAD_ID_STR) == 0) {
			seq_printf(m, "%11lld uV\n",adc_sts.ch_vol);
			break;
		}
	}

	return 0;
}



static int board_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, board_proc_show, NULL);
}

static const struct file_operations boardid_proc_fops = {
	.open		= board_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int qpnp_reg_dbg_probe(struct spmi_device *spmi)
{
	struct qpnp_reg_dbg *chip;
	struct proc_dir_entry *p;
	int ret;
	u32 temp_val;

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spmi = spmi;

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,smps-base-addr", &temp_val);
	if (!ret) {
		chip->smps_base_addr = temp_val;
	} else if (ret != EINVAL) {
		dev_err(&spmi->dev, "Unable to smps-base-addr\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,ldo-base-addr", &temp_val);
	if (!ret) {
		chip->ldo_base_addr = temp_val;
	} else if (ret != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read ldo-base-addr\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,mpp-base-addr", &temp_val);
	if (!ret) {
		chip->mpp_base_addr = temp_val;
	} else if (ret != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read mpp-base-addr\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,gpio-base-addr", &temp_val);
	if (!ret) {
		chip->gpio_base_addr = temp_val;
	} else if (ret != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read gpio-base-addr\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,smps-count", &temp_val);
	if (!ret) {
		chip->smps_count = temp_val;
	} else if (ret != EINVAL) {
		dev_err(&spmi->dev, "Unable to read smps-count\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,ldo-count", &temp_val);
	if (!ret) {
		chip->ldo_count = temp_val;
	} else if (ret != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read ldo-count\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,mpp-count", &temp_val);
	if (!ret) {
		chip->mpp_count = temp_val;
	} else if (ret != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read mpp-count\n");
		return ret;
	}

	ret = of_property_read_u32(spmi->dev.of_node,
			"qcom,gpio-count", &temp_val);
	if (!ret) {
		chip->gpio_count = temp_val;
	} else if (ret != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read gpio-count\n");
		return ret;
	}

	ret = sysfs_create_group(&spmi->dev.kobj, &reg_dbg_attr_group);
	if (ret) {
		dev_err(&spmi->dev, "failed to create sysfs group\n");
		return ret;
	}

	pr_info("smps addr:0x%x,ldo addr:0x%x,smps count:0x%x ldo count:0x%x\n",chip->smps_base_addr,chip->ldo_base_addr,chip->smps_count,chip->ldo_count);
	
	//back compatible
	p = proc_create("vreg_sts", 0, NULL, &vreg_proc_fops);
	if (!p)
		dev_err(&spmi->dev, "failed to create vreg_sts proc\n");

	p = proc_create("adc_sts", 0, NULL, &adc_proc_fops);
	if (!p)
		dev_err(&spmi->dev, "failed to create adc_sts proc\n");

	p = proc_create("pmic_pin_sts", 0, NULL, &pins_proc_fops);
	if (!p)
		dev_err(&spmi->dev, "failed to create pmu_pin_sts proc\n");

	p = proc_create(BORAD_ID_STR, 0, NULL, &boardid_proc_fops);
	if (!p)
		dev_err(&spmi->dev, "failed to create board_id proc\n");

	dev_set_drvdata(&spmi->dev, chip);

	vreg_dev = chip;

	return ret;
}

static int qpnp_reg_dbg_remove(struct spmi_device *spmi)
{
	struct qpnp_reg_dbg *chip = dev_get_drvdata(&spmi->dev);
	sysfs_remove_group(&spmi->dev.kobj, &reg_dbg_attr_group);
	remove_proc_entry("vreg_sts", NULL);
	remove_proc_entry("adc_sts", NULL);
	remove_proc_entry("pmu_pin_sts", NULL);
	remove_proc_entry(BORAD_ID_STR, NULL);
	dev_set_drvdata(&spmi->dev, NULL);
	if(chip)
		kfree(chip);
	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-reg-dbg",
	},
	{}
};

static struct spmi_driver qpnp_reg_dbg_driver = {
	.driver		= {
		.name	= "qcom,qpnp-reg-dbg",
		.of_match_table = spmi_match_table,
	},
	.probe		= qpnp_reg_dbg_probe,
	.remove		= qpnp_reg_dbg_remove,
};

static int __init qpnp_reg_dbg_init(void)
{
	return spmi_driver_register(&qpnp_reg_dbg_driver);
}
module_init(qpnp_reg_dbg_init);

static void __exit qpnp_reg_dbg_exit(void)
{
	return spmi_driver_unregister(&qpnp_reg_dbg_driver);
}
module_exit(qpnp_reg_dbg_exit);

MODULE_DESCRIPTION("qpnp reg debug driver");
MODULE_AUTHOR("Andrew.C.Lee <lichuan@hisense.com>");
MODULE_LICENSE("GPL v2");
