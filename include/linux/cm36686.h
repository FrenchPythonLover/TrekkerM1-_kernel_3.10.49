/* include/linux/cm36283.h
 *
 * Copyright (C) 2012 Capella Microsystems Inc.
 * Author: Frank Hsieh <pengyueh@gmail.com>
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

#ifndef _LINUX_CM36686_H_
#define _LINUX_CM36686_H_

#include <linux/bitops.h>

#define CM36686_I2C_NAME "cm36686"

/*
 * Define Command Code
 */
#define REG_ALS_CONF	0x00
#define REG_ALS_THDH	0x01
#define REG_ALS_THDL	0x02
#define REG_PS_CONF12	0x03
#define REG_PS_CONF3	0x04
#define REG_PS_CANC		0x05
#define REG_PS_THDL		0x06
#define REG_PS_THDH	0x07
#define REG_PS_DATA	0x08
#define REG_ALS_DATA	0x09
#define REG_WHITE_DATA	0x0A
#define REG_INT_FLAG	0x0B
#define REG_ID_REG		0x0C

/* for ALS CONF command */
#define CM36686_ALS_DIS			(1 << 0)
#define CM36686_ALS_EN			(~CM36686_ALS_DIS)

#define CM36686_ALS_INT_EN	 	(1 << 1)
#define CM36686_ALS_INT_DIS	 	(~CM36686_ALS_INT_EN)

#define CM36686_ALS_PERS_1		(0 << 2)
#define CM36686_ALS_PERS_2		(1 << 2)
#define CM36686_ALS_PERS_4		(2 << 2)
#define CM36686_ALS_PERS_8		(3 << 2)
#define CM36686_ALS_PERS_MASK	(~CM36686_ALS_PERS_8)

#define CM36686_ALS_IT_80 		(0 << 6)
#define CM36686_ALS_IT_160 		(1 << 6)
#define CM36686_ALS_IT_320 		(2 << 6)
#define CM36686_ALS_IT_640 		(3 << 6)
#define CM36686_ALS_IT_MASK		(~CM36686_ALS_IT_640)


/* for PS CONF12 command */
#define CM36686_PS_DIS			(1 << 0)
#define CM36686_PS_EN			(~CM36686_PS_DIS)

#define CM36686_PS_IT_1T			(0 << 1)
#define CM36686_PS_IT_1_5T		(1 << 1)
#define CM36686_PS_IT_2T			(2 << 1)
#define CM36686_PS_IT_2_5T		(3 << 1)
#define CM36686_PS_IT_3T			(4 << 1)
#define CM36686_PS_IT_3_5T		(5 << 1)
#define CM36686_PS_IT_4T			(6 << 1)
#define CM36686_PS_IT_8T			(7 << 1)
#define CM36686_PS_IT_MASK		(~CM36686_PS_IT_8T)

#define CM36686_PS_PERS_1		(0 << 4)
#define CM36686_PS_PERS_2		(1 << 4)
#define CM36686_PS_PERS_3		(2 << 4)
#define CM36686_PS_PERS_4		(3 << 4)
#define CM36686_PS_PERS_MASK	(~CM36686_PS_PERS_4)

#define CM36686_PS_DR_1_40		(0 << 6)
#define CM36686_PS_DR_1_80		(1 << 6)
#define CM36686_PS_DR_1_160		(2 << 6)
#define CM36686_PS_DR_1_320		(3 << 6)
#define CM36686_PS_DR_MASK		(~CM36686_PS_DR_1_320)

#define CM36686_PS_INT_OFF		(0 << 8)
#define CM36686_PS_INT_CLOSE	(1 << 8)
#define CM36686_PS_INT_AWAY		(2 << 8)
#define CM36686_PS_INT_BOTH		(3 << 8)
#define CM36686_PS_INT_MASK		(~CM36686_PS_INT_BOTH)

#define CM36686_PS_HD_16BIT		(1 << 11)
#define CM36686_PS_HD_12BIT		(~CM36686_PS_HD_16BIT)


/*for PS CONF3 command*/
#define CM36686_PS_ACTIVE_FORCE_TRIG  (1 << 2)
#define CM36686_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define CM36686_PS_SMART_PERS_ENABLE  (1 << 4)

#define CM36686_PS_LED_I_50		(0 << 8)
#define CM36686_PS_LED_I_75		(1 << 8)
#define CM36686_PS_LED_I_100	(2 << 8)
#define CM36686_PS_LED_I_120	(3 << 8)
#define CM36686_PS_LED_I_140	(4 << 8)
#define CM36686_PS_LED_I_160	(5 << 8)
#define CM36686_PS_LED_I_180	(6 << 8)
#define CM36686_PS_LED_I_200	(7 << 8)
#define CM36686_PS_LED_I_MASK	(~CM36686_PS_LED_I_200)

#define CM36686_PS_MS_NORMAL        (0 << 14)
#define CM36686_PS_MS_LOGIC_ENABLE  (1 << 14)


/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG           (1<<14)
#define INT_FLAG_ALS_IF_L            (1<<13)
#define INT_FLAG_ALS_IF_H            (1<<12)
#define INT_FLAG_PS_IF_CLOSE         (1<<9)
#define INT_FLAG_PS_IF_AWAY          (1<<8)  

#define CM36686_VDD_MIN_UV	2750000
#define CM36686_VDD_MAX_UV	2950000
#define CM36686_VI2C_MIN_UV	1750000
#define CM36686_VI2C_MAX_UV	1950000

/* CM36686 polling rate in ms */
#define CM36686_LS_DEFAULT_POLL_DELAY	500 //500ms
#define CM36686_LS_MIN_POLL_DELAY	1
#define CM36686_LS_MAX_POLL_DELAY	1000

#define NEAR_CODE	1
#define FAR_CODE	0

#define LUX_RANGE_MAX	5240
#define PS_CROSSTALK_MAX	75
#define PS_CROSSTALK_MIN	0
#define ID_CM36686	0x86

#endif
