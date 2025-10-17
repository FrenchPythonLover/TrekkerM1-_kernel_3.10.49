/* arch/arm/mach-msm/bootinfo.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, The Linux Foundation. All rights reserved.
 * Author: kongzhiqiang <kongzhiqiang@hisensecom.com>
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


#define PM_PWR_ON_EVENT_KEYPAD		0x1
#define PM_PWR_ON_EVENT_RTC			0x2
#define PM_PWR_ON_EVENT_CABLE		0x4
#define PM_PWR_ON_EVENT_SMPL		0x8
#define PM_PWR_ON_EVENT_WDOG		0x10
#define PM_PWR_ON_EVENT_USB_CHG		0x20
#define PM_PWR_ON_EVENT_WALL_CHG	0x40
#define PM_PWR_ON_EVENT_CAL    		0x80
#define PM_PWR_ON_EVENT_FTM			0x100

enum {
	MSM_POWERON_REASON_NULL= 0,
	MSM_POWERON_REASON_KEYPAD,
	MSM_POWERON_REASON_RTC,
	MSM_POWERON_REASON_CABLE,
	MSM_POWERON_REASON_SMPL,
	MSM_POWERON_REASON_WDOG,
	MSM_POWERON_REASON_USB_CHG,
	MSM_POWERON_REASON_WALL_CHG,
	MSM_POWERON_REASON_CAL,
	MSM_POWERON_REASON_FTM,
	MSM_POWERON_REASON_MAX_NUM,
};

