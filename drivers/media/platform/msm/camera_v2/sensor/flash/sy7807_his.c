/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include "../msm_sensor.h"
#include "../cci/msm_cci.h"
#include <linux/debugfs.h>

#define FLASH_NAME "slj,sy7807"

#define CONFIG_MSMB_CAMERA_FLASH_DEBUG
#ifdef CONFIG_MSMB_CAMERA_FLASH_DEBUG
#define SY7807_DBG(fmt, args...) pr_err("[william]---%s:"fmt , __func__ , ##args)
#else
#define SY7807_DBG(fmt, args...)
#endif

#define SY7807_ERR(fmt, args...) pr_err("error:%s:%d "fmt , __func__ , __LINE__ , ##args)

static struct msm_led_flash_ctrl_t sy7807_fctrl;
static struct i2c_driver sy7807_i2c_driver;

void  set_gcc_camss_gp0_1_clk_duty(int duty);

static struct msm_cam_clk_info cam_8909_gpio_31_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"torch_src_clk", 192000},
	[SENSOR_CAM_CLK] = {"torch_clk", 0},
};

static struct msm_cam_clk_info cam_8909_gpio_32_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"flash_src_clk", 192000},
	[SENSOR_CAM_CLK] = {"flash_clk", 0},
};

static const int flash_max_current = 1500; 
static const int flash_op_current = 1000; 
static const int torch_max_current = 300;
static const int torch_op_current = 290;
static const int flash_duty_cycle = 66;
static const int torch_duty_cycle = 96;

static int isTorchOn = 0;
static int isFlashOn = 0;

int msm_sy7807_init_flashlight(struct msm_led_flash_ctrl_t *fctrl)
{
    	int rc = 0;
	int i = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SY7807_DBG("led_state = %d %d  \n" , fctrl->led_state , __LINE__);
	if (!fctrl) {
		SY7807_ERR("fctrl NULL\n");
		 return -EINVAL;
	}

	if (fctrl->led_state != MSM_CAMERA_LED_RELEASE) {
		SY7807_ERR("has been inited led_state=%d\n" , fctrl->led_state);
		return rc;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__ , __LINE__);
	}

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n" , __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		SY7807_DBG("%d PC:: flash pins setting to active state",
				 __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}

	for (i = 0; i < fctrl->torch_num_sources; i++) {
		fctrl->torch_duty_cycle[i] = fctrl->torch_op_current[i] * 100 / fctrl->torch_max_current[i];
		SY7807_DBG("fctrl->torch_op_current[i]:%d\n", fctrl->torch_op_current[i]);
		SY7807_DBG("fctrl->torch_max_current[i]:%d\n", fctrl->torch_max_current[i]);
		//duty_cycle >= 99 and duty_cycle < 10 can not be set
		if(fctrl->torch_duty_cycle[i] > 96){
			fctrl->torch_duty_cycle[i] = 96;
		}else if(fctrl->torch_duty_cycle[i] < 10){
			fctrl->torch_duty_cycle[i] = 10;
		}
	}
	SY7807_DBG("torch_duty_cycle:%d\n", fctrl->torch_duty_cycle[0]);
	SY7807_DBG("torch_num_sources:%d\n", fctrl->torch_num_sources);
	
	for (i = 0; i < fctrl->flash_num_sources; i++) {
		fctrl->flash_duty_cycle[i] = fctrl->flash_op_current[i] * 100 / fctrl->flash_max_current[i];
		SY7807_DBG("fctrl->flash_op_current[i]:%d\n", fctrl->flash_op_current[i]);
		SY7807_DBG("fctrl->flash_max_current[i]:%d\n", fctrl->flash_max_current[i]);
		//duty_cycle >= 99 and duty_cycle < 10 can not be set
		if(fctrl->flash_duty_cycle[i] > 96){
			fctrl->flash_duty_cycle[i] = 96;
		}else if(fctrl->flash_duty_cycle[i] < 10){
			fctrl->flash_duty_cycle[i] =10;
		}
	}
	SY7807_DBG("flash_duty_cycle:%d\n", fctrl->flash_duty_cycle[0]);
	SY7807_DBG("flash_num_sources:%d\n", fctrl->flash_num_sources);

	fctrl->led_state = MSM_CAMERA_LED_INIT;
	SY7807_DBG(" X.\n");

	return rc;


}


int msm_sy7807_release_flashlight(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0, ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7807_DBG(" called\n");
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
		&fctrl->fl_en_clk_info[0],
		(struct clk **)&fctrl->data_fl_en[0],
		fctrl->fl_en_clk_info_size,
		0);
	if (rc < 0){
		SY7807_ERR(": disable flash clk failed\n");
	}

	rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
		&fctrl->tor_en_clk_info[0],
		(struct clk **)&fctrl->data_tor_en[0],
		fctrl->tor_en_clk_info_size,
		0);
	if (rc < 0){
		SY7807_ERR(": disable torch clk failed\n");
	}


	if (fctrl->pinctrl_info.use_pinctrl == true) {
		ret = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (ret < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	SY7807_DBG(":  end\n");

	return 0;
  
}

static const struct of_device_id sy7807_i2c_trigger_dt_match[] = {
	{.compatible = "slj,sy7807"},
	{}
};

MODULE_DEVICE_TABLE(of, sy7807_i2c_trigger_dt_match);
static const struct i2c_device_id sy7807_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&sy7807_fctrl},
	{ }
};

static void msm_sy7807_led_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct msm_led_flash_ctrl_t *fctrl = container_of(led_cdev , struct msm_led_flash_ctrl_t , led_cl_dev);
	SY7807_DBG("set brightness = %d \n" , value);
	SY7807_DBG("current cdev.brightness = %d \n" , fctrl->led_cl_dev.brightness);

	if (NULL != fctrl) {
		if (LED_OFF == value) {
			if (fctrl->led_state != MSM_CAMERA_LED_RELEASE) {
				if (fctrl->func_tbl->flash_led_off)
					fctrl->func_tbl->flash_led_off(fctrl);
				if (fctrl->func_tbl->flash_led_release_flashlight) {
					fctrl->func_tbl->flash_led_release_flashlight(fctrl);
					SY7807_DBG("flashlight has released\n");
				}
			}
		} else if (LED_FULL == value) {
			if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
				if (fctrl->func_tbl->flash_led_init_flashlight)
					fctrl->func_tbl->flash_led_init_flashlight(fctrl);
				if (fctrl->func_tbl->flash_led_low) {
					fctrl->func_tbl->flash_led_low(fctrl);
					SY7807_DBG("flashlight on low\n");
				}
			}
		} else if (LED_MAIN_FLASH == value) {
			if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
				if (fctrl->func_tbl->flash_led_init_flashlight)
					fctrl->func_tbl->flash_led_init_flashlight(fctrl);
				if (fctrl->func_tbl->flash_led_high) {
					fctrl->func_tbl->flash_led_high(fctrl);
					SY7807_DBG("flashlight on high\n");
				}
			}
		} else {
				pr_err("error: func = %s brightness = %d is unvalid \n", __func__, value);
		}
	} else {
		pr_err("error: flashlight node obtain fctrl failed , the fctrl == null \n");
	}

};


static int32_t msm_sy7807_set_flash_max_current(struct msm_led_flash_ctrl_t *fctrl, int cur, int id)
{
	if(!fctrl){
		SY7807_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7807_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > flash_max_current){
		SY7807_ERR("target %d > max %d fail!\n", cur, flash_max_current);
		cur = flash_max_current;
	}

	fctrl->flash_max_current[id] = cur;	
	
	SY7807_DBG("flash_max_current[%d] = %d\n", id, cur);

	return 0;
}

static int32_t msm_sy7807_set_flash_op_current(struct msm_led_flash_ctrl_t *fctrl, int cur, int id)
{	
	if(!fctrl){
		SY7807_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7807_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > fctrl->flash_max_current[id]){
		SY7807_ERR("flash_op_current %d > max %d! -> %d\n", cur, 
			fctrl->flash_max_current[id],
			fctrl->flash_max_current[id]);		
		cur = fctrl->flash_max_current[id];
	}

	fctrl->flash_op_current[id] = cur;	

	fctrl->flash_duty_cycle[id] = fctrl->flash_op_current[id] * 100 / fctrl->flash_max_current[id];

	return 0;

}

static int32_t msm_sy7807_set_torch_max_current(struct msm_led_flash_ctrl_t *fctrl, int cur, int id)
{
	if(!fctrl){
		SY7807_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7807_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > torch_max_current){
		SY7807_ERR("target %d > max %d fail!\n", cur, torch_max_current);
		cur = torch_max_current;
	}

	fctrl->torch_max_current[id] = cur;		
	SY7807_DBG("torch_max_current[%d] = %d\n", id, cur);

	return 0;
}

static int32_t msm_sy7807_set_torch_op_current(struct msm_led_flash_ctrl_t *fctrl, int cur, int id)
{	
	if(!fctrl){
		SY7807_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7807_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > fctrl->torch_max_current[id]){
		SY7807_ERR("torch_op_current %d > max %d! -> %d\n", cur, 
			fctrl->torch_max_current[id],
			fctrl->torch_max_current[id]);		
		cur = fctrl->torch_max_current[id];
	}

	fctrl->torch_op_current[id] = cur;	

	fctrl->torch_duty_cycle[id] = fctrl->torch_op_current[id] * 100 / fctrl->torch_max_current[id];

	return 0;

}


static int32_t msm_sy7807_set_flash_duty_cycle(struct msm_led_flash_ctrl_t *fctrl, int cur, int id)
{
	if(!fctrl){
		SY7807_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7807_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > 96){
		cur = 96;
	}else if(cur < 10){
		cur = 10;
	}

	fctrl->flash_duty_cycle[id] = cur;	

	fctrl->flash_op_current[id] = fctrl->flash_duty_cycle[id] * fctrl->flash_max_current[id] / 100;

	return 0;

}


static int32_t msm_sy7807_set_torch_duty_cycle(struct msm_led_flash_ctrl_t *fctrl, int cur, int id)
{
	if(!fctrl){
		SY7807_ERR(" fail!\n");
		return -EINVAL;
	}
	
	if(id >= MAX_LED_TRIGGERS){
		SY7807_ERR("id %d >= MAX_LED_TRIGGERS %d fail!\n", id, MAX_LED_TRIGGERS);
		return -EINVAL;
	}

	if(cur > 96){
		cur = 96;
	}else if(cur < 10){
		cur = 10;
	}
	
	fctrl->torch_duty_cycle[id] = cur;	
	
	fctrl->torch_op_current[id] = fctrl->torch_duty_cycle[id] * fctrl->torch_max_current[id] / 100;

	return 0;

}


static ssize_t attr_get_flash_max_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_max_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7807_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_flash_max_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7807_DBG(" cur=%d\n", cur);
		
	msm_sy7807_set_flash_max_current(fctrl, cur, 0);

	return size;
}

static ssize_t attr_get_flash_op_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_op_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7807_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_flash_op_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7807_DBG(" cur=%d\n", cur);
		
	msm_sy7807_set_flash_op_current(fctrl, cur, 0);

	return size;
}


static ssize_t attr_get_torch_max_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	
	led_num = fctrl->torch_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->torch_max_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7807_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_torch_max_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7807_DBG(" cur=%d\n", cur);
		
	msm_sy7807_set_torch_max_current(fctrl, cur, 0);

	return size;
}

static ssize_t attr_get_torch_op_current(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	
	led_num = fctrl->torch_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->torch_op_current[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7807_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_torch_op_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7807_DBG(" cur=%d\n", cur);
		
	msm_sy7807_set_torch_op_current(fctrl, cur, 0);

	return size;
}

static ssize_t attr_get_flash_duty_cycle(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	
	led_num = fctrl->flash_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->flash_duty_cycle[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7807_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_flash_duty_cycle(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7807_DBG(" cur=%d\n", cur);
		
	msm_sy7807_set_flash_duty_cycle(fctrl, cur, 0);

	return size;
}

static ssize_t attr_get_torch_duty_cycle(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, led_num, len;
	int offset = 0;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	
	led_num = fctrl->torch_num_sources;
	
	for(i = 0; i < led_num; i++){		
		len = sprintf(buf + offset, "%d ", fctrl->torch_duty_cycle[i]);
		offset += len;
	}
	
	len = sprintf(buf + offset, "\n");

	SY7807_DBG(" %s", buf);

	return offset + len;		
}

static ssize_t attr_set_torch_duty_cycle(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned int cur;
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;

	if (kstrtouint(buf, 10, &cur))
		return -EINVAL;
	
	SY7807_DBG(" cur=%d\n", cur);
		
	msm_sy7807_set_torch_duty_cycle(fctrl, cur, 0);

	return size;
}


static struct device_attribute attributes[] = {	
	
	__ATTR(flash_max_current, 0664, attr_get_flash_max_current, attr_set_flash_max_current),
	__ATTR(flash_op_current, 0664, attr_get_flash_op_current, attr_set_flash_op_current),
	__ATTR(torch_max_current, 0664, attr_get_torch_max_current, attr_set_torch_max_current),
	__ATTR(torch_op_current, 0664, attr_get_torch_op_current, attr_set_torch_op_current),

	__ATTR(flash_duty_cycle, 0664, attr_get_flash_duty_cycle, attr_set_flash_duty_cycle),
	__ATTR(torch_duty_cycle, 0664, attr_get_torch_duty_cycle, attr_set_torch_duty_cycle),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;

	//sy7807_dev = dev;
	
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


static int32_t msm_sy7807_create_flashlight(struct device *dev ,
				void *data)
{
	int rc;	
	struct msm_led_flash_ctrl_t *fctrl = &sy7807_fctrl;
	SY7807_DBG("%s E\n",__func__);
	
	if (!fctrl) {
		pr_err("Invalid fctrl\n");
		return -EINVAL;
	}
	
	fctrl->led_cl_dev.name = "flashlight";
	fctrl->led_cl_dev.brightness_set = msm_sy7807_led_brightness_set;
	fctrl->led_cl_dev.brightness = LED_OFF;
	rc = led_classdev_register(dev, &fctrl->led_cl_dev);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	create_sysfs_interfaces(fctrl->led_cl_dev.dev);
	
	SY7807_DBG("---X\n");
	return 0;
};


int msm_flash_sy7807_led_init(struct msm_led_flash_ctrl_t *fctrl)
{	
	int rc = 0;
	int i = 0;
	
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SY7807_DBG("called  led_state = %d\n", fctrl->led_state);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		 return -EINVAL;
	}

	if ((fctrl->led_state != MSM_CAMERA_LED_RELEASE)
		&& ((0 == fctrl->led_cl_dev.brightness) || (128 == fctrl->led_cl_dev.brightness))) {
		pr_err("error:%s:%d has been inited led_state=%d\n", __func__, __LINE__, fctrl->led_state);
		return rc;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}
	
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		SY7807_DBG("%d PC:: flash pins setting to active state", __LINE__);
		if(fctrl->pinctrl_info.pinctrl == NULL){
			pr_err("%s:%d  pinctrl null",
					__func__, __LINE__);
			return 0;
		}	
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}	

	for (i = 0; i < fctrl->torch_num_sources; i++) {
		fctrl->torch_duty_cycle[i] = fctrl->torch_op_current[i] * 100 / fctrl->torch_max_current[i];
		SY7807_DBG("fctrl->torch_op_current[i]:%d\n", fctrl->torch_op_current[i]);
		SY7807_DBG("fctrl->torch_max_current[i]:%d\n", fctrl->torch_max_current[i]);
		//duty_cycle >= 99 and duty_cycle < 10 can not be set
		if(fctrl->torch_duty_cycle[i] > 96){
			fctrl->torch_duty_cycle[i] = 96;
		}else if(fctrl->torch_duty_cycle[i] < 10){
			fctrl->torch_duty_cycle[i] = 10;
		}
	}
	SY7807_DBG("torch_duty_cycle:%d\n", fctrl->torch_duty_cycle[0]);
	SY7807_DBG("torch_num_sources:%d\n", fctrl->torch_num_sources);
	
	for (i = 0; i < fctrl->flash_num_sources; i++) {
		fctrl->flash_duty_cycle[i] = fctrl->flash_op_current[i] * 100 / fctrl->flash_max_current[i];
		SY7807_DBG("fctrl->flash_op_current[i]:%d\n", fctrl->flash_op_current[i]);
		SY7807_DBG("fctrl->flash_max_current[i]:%d\n", fctrl->flash_max_current[i]);
		//duty_cycle >= 99 and duty_cycle < 10 can not be set
		if(fctrl->flash_duty_cycle[i] > 96){
			fctrl->flash_duty_cycle[i] = 96;
		}else if(fctrl->flash_duty_cycle[i] < 10){
			fctrl->flash_duty_cycle[i] =10;
		}
	}
	SY7807_DBG("flash_duty_cycle:%d\n", fctrl->flash_duty_cycle[0]);
	SY7807_DBG("flash_num_sources:%d\n", fctrl->flash_num_sources);


	fctrl->func_tbl->flash_led_off(fctrl);
	
	fctrl->led_cl_dev.brightness = 128;	
	fctrl->led_state = MSM_CAMERA_LED_INIT;
	SY7807_DBG(" X.\n");
	
	return rc;


}

int msm_flash_sy7807_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7807_DBG(" called\n");
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	fctrl->func_tbl->flash_led_off(fctrl);
	
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	fctrl->led_cl_dev.brightness = LED_OFF;
	SY7807_DBG(" led state=%d ,brightness=%d\n", fctrl->led_state, fctrl->led_cl_dev.brightness);
	return rc;

}

int msm_flash_sy7807_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	SY7807_DBG(":  called\n");

	if (!fctrl || !fctrl->flashdata) {
               pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
               return -EINVAL;
       }
	   
	if(isFlashOn){
		rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
			&fctrl->fl_en_clk_info[0],
			(struct clk **)&fctrl->data_fl_en[0],
			fctrl->fl_en_clk_info_size,
			0);
		if (rc < 0){
			  SY7807_ERR(": disable flash clk failed\n");
		}		
	}
	
	if(isTorchOn){
		rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
			&fctrl->tor_en_clk_info[0],
			(struct clk **)&fctrl->data_tor_en[0],
			fctrl->tor_en_clk_info_size,
			0);
	    	if (rc < 0){
		  	SY7807_ERR(": disable torch clk failed\n");
	    	}			
	}

	isFlashOn = isTorchOn = 0;
	
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}		
	
	SY7807_DBG(":  end\n");
	
	return rc;
}


int msm_flash_sy7807_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	if(isTorchOn){
		pr_err("repeated calling led low, return\n");
		return 0;
	}

	SY7807_DBG(":called\n");

	set_gcc_camss_gp0_1_clk_duty(fctrl->torch_duty_cycle[0]);

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
				__func__, __LINE__);
		}
	}	
	
	
	rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
		&fctrl->tor_en_clk_info[0],
		(struct clk **)&fctrl->data_tor_en[0],
		fctrl->tor_en_clk_info_size,
		1);
    	if (rc < 0){
	  	SY7807_ERR(": enable torch clk failed\n");
    	}	

	isTorchOn = 1;
	
	SY7807_DBG(":end\n");

	return rc;
}

int msm_flash_sy7807_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	if(isFlashOn){
		pr_err("repeated calling led high, return\n");
		return 0;
	}
	
	SY7807_DBG(" called\n");

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
				__func__, __LINE__);
		}
	}

	if(isTorchOn){
		rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
			&fctrl->tor_en_clk_info[0],
			(struct clk **)&fctrl->data_tor_en[0],
			fctrl->tor_en_clk_info_size,
			0);
	    	if (rc < 0){
		  	SY7807_ERR(": disable torch clk failed\n");
	    	}
		isTorchOn = 0;
	}
	
	set_gcc_camss_gp0_1_clk_duty(fctrl->flash_duty_cycle[0]);

	rc = msm_cam_clk_enable_not_check_rate(&fctrl->flash_i2c_client->client->dev,
		&fctrl->fl_en_clk_info[0],
		(struct clk **)&fctrl->data_fl_en[0],
		fctrl->fl_en_clk_info_size,
		1);
    	if (rc < 0){
	  	SY7807_ERR(": enable flash clk failed\n");
    	}
	isFlashOn = 1;
	
	SY7807_DBG(": end\n");

	return rc;
}


static int msm_flash_sy7807_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0 ;
	struct msm_led_flash_ctrl_t *fctrl = NULL;
	fctrl = &sy7807_fctrl;
	SY7807_DBG(" entry\n");
	if (!id) {
		pr_err("msm_flash_sy7807_i2c_probe: id is NULL");
		id = sy7807_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);
	if (!rc) {
		msm_sy7807_create_flashlight(&(client->dev), NULL);
		SY7807_DBG(" creat flashlight done\n");
	}
	return rc;
}

static int msm_flash_sy7807_i2c_remove(struct i2c_client *client)
{
	int rc = 0 ;
	SY7807_DBG("entry\n");

	return rc;
}

#if 0
static const struct of_device_id sy7807_dt_match[] = {
	{.compatible = "slj,sy7807", .data = &sy7807_fctrl},
	{}
};
#endif
#if 0
static int msm_flash_sy7807_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int rc = 0;
	
	match = of_match_device(sy7807_dt_match, &pdev->dev);
	SY7807_DBG("%s ygy platform_probe\n",__func__);
	if (!match)
		return -EFAULT;
	
	rc = msm_flash_probe(pdev, match->data);
	
	if(!rc){ 
		msm_sy7807_create_classdev(&pdev->dev, NULL);
		
		pr_err(" msm_flash_probe success and creat flashlight\n");
	}
	return rc;
}
#endif

static struct i2c_driver sy7807_i2c_driver = {
	.id_table = sy7807_i2c_id,
	.probe  = msm_flash_sy7807_i2c_probe,
	.remove = msm_flash_sy7807_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7807_i2c_trigger_dt_match,
	},
};

#if 0
MODULE_DEVICE_TABLE(of, sy7807_dt_match);

static struct platform_driver sy7807_platform_driver = {
	.probe = msm_flash_sy7807_platform_probe,
	.driver = {
		.name = "slj,sy7807",
		.owner = THIS_MODULE,
		.of_match_table = sy7807_dt_match,
	},
};
#endif

static struct msm_camera_i2c_client sy7807_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static int __init msm_flash_sy7807_init(void)
{
	SY7807_DBG("%s entry\n", __func__);
	return i2c_add_driver(&sy7807_i2c_driver);
}

static void __exit msm_flash_sy7807_exit(void)
{
	SY7807_DBG("%s entry\n", __func__);
	if(sy7807_fctrl.led_cl_dev.dev){
		remove_sysfs_interfaces(sy7807_fctrl.led_cl_dev.dev);
	}
	i2c_del_driver(&sy7807_i2c_driver);
	return;
}

static struct msm_flash_fn_t sy7807_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sy7807_led_init,
	.flash_led_release = msm_flash_sy7807_led_release,
	.flash_led_off = msm_flash_sy7807_led_off,
	.flash_led_low = msm_flash_sy7807_led_low,
	.flash_led_high = msm_flash_sy7807_led_high,
	.flash_led_init_flashlight = msm_sy7807_init_flashlight,
	.flash_led_release_flashlight = msm_sy7807_release_flashlight,
};

static struct msm_led_flash_ctrl_t sy7807_fctrl = {
	.flash_i2c_client = &sy7807_i2c_client,
	.func_tbl = &sy7807_func_tbl,

	.tor_en_clk_info = cam_8909_gpio_31_clk_info,
	.tor_en_clk_info_size = ARRAY_SIZE(cam_8909_gpio_31_clk_info),

	.fl_en_clk_info = cam_8909_gpio_32_clk_info,
	.fl_en_clk_info_size = ARRAY_SIZE(cam_8909_gpio_32_clk_info),
       
};

module_init(msm_flash_sy7807_init);
module_exit(msm_flash_sy7807_exit);
MODULE_DESCRIPTION("sy7807 FLASH");
MODULE_LICENSE("GPL v2");
