/* arch/arm/mach-msm/bootinfo.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <asm/setup.h>

#include "bootinfo.h"

int msm_proc_comm_is_meid_null_flag = 1;


#define BOOTINFO_ATTR(_name) \
static struct kobj_attribute _name##_attr = {   \
        .attr   = {                             \
                .name = __stringify(_name),     \
                .mode = 0444,                   \
        },                                      \
        .show   = _name##_show,                 \
        .store  = NULL,                			\
}

int poweron_reason=0;

static char *power_up_reasons[] = {"no_reason","power_key","rtc_alarm","cable_pwr","smpl_pwr",
	"watch_dog","usb_charger","wall_charger","cal_mode","ftm_mode"};


static int __init powerup_reason_setup(char *p)
{
	if (!strcmp(p, "0x80"))
	{
		poweron_reason = MSM_POWERON_REASON_KEYPAD;
	}
	else if(!strcmp(p, "0x40"))
	{
		poweron_reason = MSM_POWERON_REASON_CABLE;
	}
	else if(!strcmp(p, "0x20"))
	{
		poweron_reason = MSM_POWERON_REASON_KEYPAD;//PON1
	}
	else if(!strcmp(p, "0x10"))
	{
		poweron_reason = MSM_POWERON_REASON_USB_CHG;
	}
	else if(!strcmp(p, "0x8"))
	{
		poweron_reason = MSM_POWERON_REASON_WALL_CHG;//DC
	}
	else if(!strcmp(p, "0x4"))
	{
		poweron_reason = MSM_POWERON_REASON_RTC;
	}
	else if(!strcmp(p, "0x2"))
	{
		poweron_reason = MSM_POWERON_REASON_SMPL;
	}	
	else if(!strcmp(p, "0x1"))
	{
		poweron_reason = MSM_POWERON_REASON_CAL;//hard reset
	}	
	else if(!strcmp(p, "0x100"))
	{
		poweron_reason = MSM_POWERON_REASON_FTM;
	}
	else
	{
		poweron_reason = MSM_POWERON_REASON_KEYPAD;
	}
	pr_err("boot_charger_mode_init   buffer = %s  poweron_reason = 0x%x\n",p,poweron_reason);
	return 0;
}
early_param("powerup_reason", powerup_reason_setup);

static ssize_t powerup_reason_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%s\n", power_up_reasons[poweron_reason]);		

	return ret;
}
BOOTINFO_ATTR(powerup_reason);

static int __init meid_status_setup(char *p)
{
	if (!strcmp(p, "0"))
		msm_proc_comm_is_meid_null_flag = 1;
	else
		msm_proc_comm_is_meid_null_flag = 0;

	pr_err("boot_meid_mode_init meid_status = %d\n",msm_proc_comm_is_meid_null_flag);
	return 0;
}
early_param("androidboot.meid", meid_status_setup);


static ssize_t meid_is_null_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	char *meid_is_null_text[] = {
		"is_null",
		"is_not_null",
	};

	if(msm_proc_comm_is_meid_null_flag)
		s+=sprintf(s, "%s",meid_is_null_text[0]); 
	else
		s+=sprintf(s, "%s",meid_is_null_text[1]); 

	return s-buf;
}
BOOTINFO_ATTR(meid_is_null);


int boot_charger_status = 0;
int boot_ftm_mode = 0;
int __init boot_charger_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		boot_charger_status = 1;

	if (!strcmp(s, "factory2"))
		boot_ftm_mode = 1;

	printk("boot_charger_mode_init   buffer = %s  boot_charger_status = %d, boot_ftm_mode = %d\n",
		s,boot_charger_status,boot_ftm_mode);

	return 1;
}
__setup("androidboot.mode=", boot_charger_mode_init);

EXPORT_SYMBOL_GPL(boot_ftm_mode);


static struct attribute * bootattr[] = {
	&powerup_reason_attr.attr,
	&meid_is_null_attr.attr,
	NULL,
};

static struct attribute_group bootattr_group = {
	.attrs = bootattr,
};

static int __init bootinfo_init(void)
{
	int ret=-ENOMEM;
	struct kobject *bootinfo_kobj = NULL;
	
	bootinfo_kobj = kobject_create_and_add("bootinfo", NULL);
	if (bootinfo_kobj == NULL) {
		pr_err("bootinfo_init: subsystem_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(bootinfo_kobj, &bootattr_group);
	if (ret) {
		pr_err("bootinfo_init: subsystem_register failed\n");
		goto sys_fail;
	}
	
	return ret;

sys_fail:
	kobject_del(bootinfo_kobj);
	return ret;
}
core_initcall(bootinfo_init);

