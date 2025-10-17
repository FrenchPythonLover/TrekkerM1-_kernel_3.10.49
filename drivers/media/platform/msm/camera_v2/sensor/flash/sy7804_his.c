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





#define FLASH_NAME "slj,sy7804"

/***#define CONFIG_MSMB_CAMERA_DEBUG***/
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define SY7804_DBG(fmt, args...) pr_err("[william]---%s:"fmt , __func__ , ##args)
#else
#define SY7804_DBG(fmt, args...)
#endif

#define SY7804_ERR(fmt, args...) pr_err("error:%s:%d "fmt , __func__ , __LINE__ , ##args)

static struct msm_led_flash_ctrl_t sy7804_fctrl;
static struct i2c_driver sy7804_i2c_driver;

static struct msm_camera_i2c_reg_array sy7804_init_array[] = {
{0x0A, 0x80},/* bit[7]IVFM, [6]TXI, [5]Flash_pin, [4]Torch_pin,
				      [1:0]  00:Shutdown  01: Indicator 10:Torch mode 11: Flash mode*/
{0x01, 0x8C},/* bit[7]UVLO enable,  [4,3,2] IVM-Down threshold=3.2V*/
{0x06, 0x00},/* Torch Ramp-up time= 16ms,  Torch Ramp-Down time=16ms */
{0x08, 0x15},/* Flash Ramp time=1ms, Flash timeout=600ms*/
{0x09, 0x39},/* Torch cur=187.5mA, Flash cur=937.5mA*/

};

static struct msm_camera_i2c_reg_array sy7804_off_array[] = {
{0x0A, 0x80},
};

static struct msm_camera_i2c_reg_array sy7804_release_array[] = {
{0x0A, 0x80},
};

static struct msm_camera_i2c_reg_array sy7804_low_array[] = {
{0x0A, 0x82},
};

static struct msm_camera_i2c_reg_array sy7804_high_array[] = {
	{0x0A, 0x83},
};

static struct msm_camera_i2c_reg_array sy7804_status_array[] = {
	{0x0B, 0x00},/*Read only*/
};


static int32_t msm_sy7804_clear_status(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	int i = 0;
	struct msm_camera_i2c_reg_setting *status = NULL;
	struct msm_camera_i2c_client *client = NULL;
	client = fctrl->flash_i2c_client;
	if (!fctrl || !client) {
		SY7804_ERR(" fail!\n");
		return -EINVAL;
}
	if (fctrl->reg_setting && fctrl->reg_setting->status_reg) {
		status = fctrl->reg_setting->status_reg;
		for (i = 0; i < status->size; i++) {
			rc = client->i2c_func_tbl->i2c_read(client,
						status->reg_setting[i].reg_addr,
						&status->reg_setting[i].reg_data,
						MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0) {
				SY7804_ERR(" i2c_read failed rc=%d\n", rc);
				return -EIO;
			}
			SY7804_DBG("sy7804 status flag[%d] = 0x%x\n" , i , status->reg_setting[i].reg_data);
		}
	}
	return rc;
}


int msm_sy7804_init_flashlight(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SY7804_DBG("led_state = %d %d  \n" , fctrl->led_state , __LINE__);
	if (!fctrl) {
		SY7804_ERR("fctrl NULL\n");
		 return -EINVAL;
	}

	if (fctrl->led_state != MSM_CAMERA_LED_RELEASE) {
		SY7804_ERR("has been inited led_state=%d\n" , fctrl->led_state);
		return rc;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (fctrl->pinctrl_info.use_gpio_power == true) {
			fctrl->reg_flash = regulator_get(&(fctrl->flash_i2c_client->client->dev) , flashdata->cam_vreg[0].reg_name);
			SY7804_DBG("flashdata->cam_vreg[0].reg_name %s\n" , flashdata->cam_vreg[0].reg_name);
			if (IS_ERR(fctrl->reg_flash)) {
					pr_err("%s: %s get failed\n", __func__,
						flashdata->cam_vreg[0].reg_name);
					fctrl->reg_flash = NULL;
					return -ENODEV;
				}
			rc = regulator_enable(fctrl->reg_flash);
			if (rc < 0) {
				pr_err("%s: %s enable failed\n",
					__func__, flashdata->cam_vreg[0].reg_name);
				regulator_put(fctrl->reg_flash);
				fctrl->reg_flash = NULL;
				return -ENODEV;
			}
	}
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__ , __LINE__);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n" , __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		SY7804_DBG("%d PC:: flash pins setting to active state",
				 __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_EN] == 1) {
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	SY7804_DBG("valid[SENSOR_GPIO_FL_EN] == 1\n");
	}
	msleep(5);
	msm_sy7804_clear_status(fctrl);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	fctrl->led_state = MSM_CAMERA_LED_INIT;
	return rc;
}



int msm_sy7804_release_flashlight(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0, ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7804_DBG(" called\n");
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

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	msm_sy7804_clear_status(fctrl);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_EN] == 1){
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	SY7804_DBG("valid[SENSOR_GPIO_FL_EN] == 0\n");
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
	if (fctrl->pinctrl_info.use_gpio_power == true) {
			if (fctrl->reg_flash != NULL) {
			   regulator_disable(fctrl->reg_flash);
			   regulator_put(fctrl->reg_flash);
			   fctrl->reg_flash = NULL;
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
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}
	return 0;
}

static const struct of_device_id sy7804_i2c_trigger_dt_match[] = {
	{.compatible = "slj,sy7804"},
	{}
};

MODULE_DEVICE_TABLE(of, sy7804_i2c_trigger_dt_match);
static const struct i2c_device_id sy7804_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&sy7804_fctrl},
	{ }
};

static void msm_sy7804_led_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
    struct msm_led_flash_ctrl_t *fctrl = container_of(led_cdev , struct msm_led_flash_ctrl_t , led_cl_dev);
	SY7804_DBG("set brightness = %d \n" , value);
	SY7804_DBG("current cdev.brightness = %d \n" , fctrl->led_cl_dev.brightness);

	if (NULL != fctrl) {
		if (LED_OFF == value) {
			if (fctrl->led_state != MSM_CAMERA_LED_RELEASE) {
				if (fctrl->func_tbl->flash_led_off)
					fctrl->func_tbl->flash_led_off(fctrl);
				if (fctrl->func_tbl->flash_led_release_flashlight) {
					fctrl->func_tbl->flash_led_release_flashlight(fctrl);
					SY7804_DBG("flashlight has released\n");
				}
		    }
		} else if (LED_FULL == value) {
			if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
				if (fctrl->func_tbl->flash_led_init_flashlight)
					fctrl->func_tbl->flash_led_init_flashlight(fctrl);
				if (fctrl->func_tbl->flash_led_low) {
					fctrl->func_tbl->flash_led_low(fctrl);
					SY7804_DBG("flashlight on low\n");
				}
			}
		} else if (LED_MAIN_FLASH == value) {
			if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
				if (fctrl->func_tbl->flash_led_init_flashlight)
					fctrl->func_tbl->flash_led_init_flashlight(fctrl);
				if (fctrl->func_tbl->flash_led_high) {
					fctrl->func_tbl->flash_led_high(fctrl);
					SY7804_DBG("flashlight on high\n");
				}
			}
		} else {
				pr_err("error: func = %s brightness = %d is unvalid \n", __func__, value);
		}
	} else {
				pr_err("error: flashlight node obtain fctrl failed , the fctrl == null \n");
		}

};


static int32_t msm_sy7804_create_flashlight(struct device *dev ,
				void *data)
{
	struct msm_led_flash_ctrl_t *fctrl = &sy7804_fctrl;
	int rc;
	if (!fctrl) {
		pr_err("Invalid fctrl\n");
		return -EINVAL;
	}
    SY7804_DBG("E\n");
    fctrl->led_cl_dev.name = "flashlight";
    fctrl->led_cl_dev.brightness_set = msm_sy7804_led_brightness_set;
	fctrl->led_cl_dev.brightness = LED_OFF;
    rc = led_classdev_register(dev, &fctrl->led_cl_dev);
    msm_sy7804_led_brightness_set(&fctrl->led_cl_dev, LED_OFF);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}
    SY7804_DBG(" X\n");
	return 0;
};

int msm_flash_sy7804_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SY7804_DBG("called  led_state = %d\n", fctrl->led_state);
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
	if (fctrl->led_cl_dev.brightness > 0) {
		msm_sy7804_led_brightness_set(&fctrl->led_cl_dev, LED_OFF);
	}
	fctrl->led_cl_dev.brightness = 128;
	if (fctrl->pinctrl_info.use_gpio_power == true) {
			fctrl->reg_flash = regulator_get(&(fctrl->flash_i2c_client->client->dev), flashdata->cam_vreg[0].reg_name);
			SY7804_DBG("flashdata->cam_vreg[0].reg_name  %s\n", flashdata->cam_vreg[0].reg_name);
			if (IS_ERR(fctrl->reg_flash)) {
					pr_err("%s: %s get failed\n", __func__,
						flashdata->cam_vreg[0].reg_name);
					fctrl->reg_flash = NULL;
					return -ENODEV;
				}
			rc = regulator_enable(fctrl->reg_flash);
			if (rc < 0) {
				pr_err("%s: %s enable failed\n",
					__func__, flashdata->cam_vreg[0].reg_name);
				regulator_put(fctrl->reg_flash);
				fctrl->reg_flash = NULL;
				return -ENODEV;
			}
	}
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		SY7804_DBG("%d PC:: flash pins setting to active state", __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_EN] == 1){
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	SY7804_DBG("valid[SENSOR_GPIO_FL_EN] == 1\n");
	}
	msleep(5);
	msm_sy7804_clear_status(fctrl);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	fctrl->led_state = MSM_CAMERA_LED_INIT;
	return rc;



}

int msm_flash_sy7804_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0, ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7804_DBG(" called\n");
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	msm_sy7804_clear_status(fctrl);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_EN] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		ret = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (ret < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	if (fctrl->pinctrl_info.use_gpio_power == true) {
			if (fctrl->reg_flash != NULL) {
			   regulator_disable(fctrl->reg_flash);
			   regulator_put(fctrl->reg_flash);
			   fctrl->reg_flash = NULL;
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
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}
	fctrl->led_cl_dev.brightness = LED_OFF;
	SY7804_DBG(" led state=%d ,brightness=%d\n", fctrl->led_state, fctrl->led_cl_dev.brightness);
	return 0;

}

int msm_flash_sy7804_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	SY7804_DBG(" called\n");

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	msm_sy7804_clear_status(fctrl);
	return rc;
}

int msm_flash_sy7804_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SY7804_DBG(" called\n");

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	msm_sy7804_clear_status(fctrl);

	return rc;
}

int msm_flash_sy7804_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SY7804_DBG(" called\n");
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	msm_sy7804_clear_status(fctrl);

	return rc;
}
static int msm_flash_sy7804_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0 ;
	struct msm_led_flash_ctrl_t *fctrl = NULL;
	fctrl = &sy7804_fctrl;
	SY7804_DBG(" entry\n");
	if (!id) {
		pr_err("msm_flash_sy7804_i2c_probe: id is NULL");
		id = sy7804_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);
	if (!rc) {
		msm_sy7804_create_flashlight(&(client->dev), NULL);
		SY7804_DBG(" creat flashlight done\n");
	}
	if (fctrl->func_tbl->flash_led_init_flashlight)
		fctrl->func_tbl->flash_led_init_flashlight(fctrl);
	if (fctrl->func_tbl->flash_led_off)
		fctrl->func_tbl->flash_led_off(fctrl);
	if (fctrl->func_tbl->flash_led_release_flashlight)
		fctrl->func_tbl->flash_led_release_flashlight(fctrl);
	return rc;
}

static int msm_flash_sy7804_i2c_remove(struct i2c_client *client)
{
	int rc = 0 ;
	SY7804_DBG("entry\n");
#if 0
	flashdata = sy7804_fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
#endif
	return rc;
}


static struct i2c_driver sy7804_i2c_driver = {
	.id_table = sy7804_i2c_id,
	.probe  = msm_flash_sy7804_i2c_probe,
	.remove = msm_flash_sy7804_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7804_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_sy7804_init(void)
{
	SY7804_DBG("%s entry\n", __func__);
	return i2c_add_driver(&sy7804_i2c_driver);
}

static void __exit msm_flash_sy7804_exit(void)
{
	SY7804_DBG("%s entry\n", __func__);
	i2c_del_driver(&sy7804_i2c_driver);
	return;
}


static struct msm_camera_i2c_client sy7804_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sy7804_init_setting = {
	.reg_setting = sy7804_init_array,
	.size = ARRAY_SIZE(sy7804_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_off_setting = {
	.reg_setting = sy7804_off_array,
	.size = ARRAY_SIZE(sy7804_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_release_setting = {
	.reg_setting = sy7804_release_array,
	.size = ARRAY_SIZE(sy7804_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_low_setting = {
	.reg_setting = sy7804_low_array,
	.size = ARRAY_SIZE(sy7804_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_high_setting = {
	.reg_setting = sy7804_high_array,
	.size = ARRAY_SIZE(sy7804_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7804_status_reg = {
	.reg_setting = sy7804_status_array,
	.size = ARRAY_SIZE(sy7804_status_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};


static struct msm_led_flash_reg_t sy7804_regs = {
	.init_setting = &sy7804_init_setting,
	.off_setting = &sy7804_off_setting,
	.low_setting = &sy7804_low_setting,
	.high_setting = &sy7804_high_setting,
	.release_setting = &sy7804_release_setting,
	.status_reg = &sy7804_status_reg,
};

static struct msm_flash_fn_t sy7804_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sy7804_led_init,
	.flash_led_release = msm_flash_sy7804_led_release,
	.flash_led_off = msm_flash_sy7804_led_off,
	.flash_led_low = msm_flash_sy7804_led_low,
	.flash_led_high = msm_flash_sy7804_led_high,
	.flash_led_init_flashlight = msm_sy7804_init_flashlight,
	.flash_led_release_flashlight = msm_sy7804_release_flashlight,
};

static struct msm_led_flash_ctrl_t sy7804_fctrl = {
	.flash_i2c_client = &sy7804_i2c_client,
	.reg_setting = &sy7804_regs,
	.func_tbl = &sy7804_func_tbl,
};

module_init(msm_flash_sy7804_init);
module_exit(msm_flash_sy7804_exit);
MODULE_DESCRIPTION("sy7804 FLASH");
MODULE_LICENSE("GPL v2");
