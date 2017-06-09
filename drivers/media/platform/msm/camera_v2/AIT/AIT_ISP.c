/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include "AIT.h"
#include "AIT_spi.h"
#include <linux/errno.h>
#include "AIT_ISP_API.h"
#include "AIT9882_sub_FW.h"
#include "AIT9882_front_FW.h"
#include "AIT_ISP_System.h"
#define MSM_AIT_ISP_NAME "AIT"

#define AIT_PINCTRL_STATE_SLEEP "AIT_suspend"
#define AIT_PINCTRL_STATE_DEFAULT "AIT_default"

#undef CDBG
#define CONFIG_AIT_DEBUG
#ifdef CONFIG_AIT_DEBUG
#define CDBG(fmt, args...) pr_info("[CAM][AIT] : " fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


static struct class *AIT_ISP_class;
static dev_t AIT_ISP_devno;


struct AIT_ISP_int_t {
	spinlock_t AIT_spin_lock;
	wait_queue_head_t AIT_wait;
};

static struct AIT_ISP_ctrl *AIT_ISPCtrl = NULL;
static struct camera_vreg_t ISP_vreg[3];
static void *ISP_vreg_data[3];
unsigned char *pAIT_fw_sub=AIT_fw_sub;
unsigned char *pAIT_fw_front=AIT_fw_front;
static int camera_index = CAMERA_INDEX_SUB_OV2722;
static struct msm_cam_clk_info ISP_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};
static int AIT_ISP_fops_open(struct inode *inode, struct file *filp)
{
	struct AIT_ISP_ctrl *raw_dev = container_of(inode->i_cdev,
		struct AIT_ISP_ctrl, cdev);
	filp->private_data = raw_dev;
	return 0;
}

static  const struct  file_operations AIT_ISP_fops = {
	.owner	  = THIS_MODULE,
	.open	   = AIT_ISP_fops_open,
};

static int msm_camera_pinctrl_init(struct AIT_ISP_ctrl *ctrl)
{
	struct msm_pinctrl_info *sensor_pctrl = NULL;

	sensor_pctrl = &ctrl->AIT_pinctrl;
	sensor_pctrl->pinctrl = devm_pinctrl_get(ctrl->dev);
	if (IS_ERR_OR_NULL(sensor_pctrl->pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	sensor_pctrl->gpio_state_active =
		pinctrl_lookup_state(sensor_pctrl->pinctrl,
				AIT_PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	sensor_pctrl->gpio_state_suspend
		= pinctrl_lookup_state(sensor_pctrl->pinctrl,
				AIT_PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(sensor_pctrl->gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}
static int setup_AIT_ISP_cdev(void)
{
	int rc = 0;
	struct device *dev;
	CDBG("%s\n", __func__);

	rc = alloc_chrdev_region(&AIT_ISP_devno, 0, 1, MSM_AIT_ISP_NAME);
	if (rc < 0) {
		pr_err("%s: failed to allocate chrdev\n", __func__);
		goto alloc_chrdev_region_failed;
	}

	if (!AIT_ISP_class) {
		AIT_ISP_class = class_create(THIS_MODULE, MSM_AIT_ISP_NAME);
		if (IS_ERR(AIT_ISP_class)) {
			rc = PTR_ERR(AIT_ISP_class);
			pr_err("%s: create device class failed\n",
				__func__);
			goto class_create_failed;
		}
	}

	dev = device_create(AIT_ISP_class, NULL,
		MKDEV(MAJOR(AIT_ISP_devno), MINOR(AIT_ISP_devno)), NULL,
		"%s%d", MSM_AIT_ISP_NAME, 0);
	if (IS_ERR(dev)) {
		pr_err("%s: error creating device\n", __func__);
		rc = -ENODEV;
		goto device_create_failed;
	}

	cdev_init(&AIT_ISPCtrl->cdev, &AIT_ISP_fops);
	AIT_ISPCtrl->cdev.owner = THIS_MODULE;
	AIT_ISPCtrl->cdev.ops   =
		(const struct file_operations *) &AIT_ISP_fops;
	rc = cdev_add(&AIT_ISPCtrl->cdev, AIT_ISP_devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev\n", __func__);
		rc = -ENODEV;
		goto cdev_add_failed;
	}

	return rc;

cdev_add_failed:
	device_destroy(AIT_ISP_class, AIT_ISP_devno);
device_create_failed:
	class_destroy(AIT_ISP_class);
class_create_failed:
	unregister_chrdev_region(AIT_ISP_devno, 1);
alloc_chrdev_region_failed:
	return rc;
}

static void AIT_ISP_tear_down_cdev(void)
{
	CDBG("%s\n", __func__);
	cdev_del(&AIT_ISPCtrl->cdev);
	device_destroy(AIT_ISP_class, AIT_ISP_devno);
	class_destroy(AIT_ISP_class);
	unregister_chrdev_region(AIT_ISP_devno, 1);
}


#ifdef CONFIG_CAMERA_AIT
static struct kobject *AIT_ISP_status_obj;

uint32_t AIT_ISP_id;

static ssize_t probed_AIT_ISP_id_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "0x%x\n", AIT_ISP_id);
	return length;
}

static DEVICE_ATTR(probed_AIT_ISP_id, 0444,
	probed_AIT_ISP_id_get,
	NULL);

int msm_AIT_ISP_attr_node(void)
{
	int ret = 0;
	CDBG("%s \n", __func__);
	AIT_ISP_status_obj = kobject_create_and_add("camera_AIT_ISP_status", NULL);
	if (AIT_ISP_status_obj == NULL) {
		pr_err("msm_camera: create camera_AIT_ISP_status failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(AIT_ISP_status_obj,
		&dev_attr_probed_AIT_ISP_id.attr);
	if (ret) {
		CDBG("%s, sysfs_create_file failed\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	return ret;

error:
	kobject_del(AIT_ISP_status_obj);
	return ret;
}
#endif

void HTC_DelayMS(uint16_t ms)
{
	mdelay(ms);
}

static int ISP_vreg_on(void)
{
	int rc = 0;
	int i = 0;
	int ISP_gpio = 0;
	int ISP_gpio1 = 0;
	int ISP_gpio2 = 0;
	int ret = 0;

	CDBG("%s +\n", __func__);
	ret = msm_camera_pinctrl_init(AIT_ISPCtrl);
	if (ret < 0) {
		pr_err("%s:%d Initialization of pinctrl failed\n",
				__func__, __LINE__);
		AIT_ISPCtrl->AIT_pinctrl_status = 0;
	} else {
		AIT_ISPCtrl->AIT_pinctrl_status = 1;
		
	}
	
	
	for (i = 0 ; i < 2; i++){
		if (i == 0)
			ISP_gpio = AIT_ISPCtrl->pdata->ISP_1v2_enable;
		else
			ISP_gpio = AIT_ISPCtrl->pdata->ISP_1v8_enable;

		if (ISP_vreg[i].type == VREG_TYPE_GPIO) {
			rc = gpio_request(ISP_gpio, "AIT_ISP");
			if (rc < 0) {
			pr_err("GPIO(%d) request failed", ISP_gpio);
			}
			gpio_direction_output(ISP_gpio, 1);
			gpio_free(ISP_gpio);
		} else {
			
			msm_camera_config_single_vreg(AIT_ISPCtrl->dev,
				&ISP_vreg[i],
				(struct regulator **)&ISP_vreg_data[i],
				1);
		}
		mdelay(1);
	}
	
	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_gpio_vana;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	    }
	gpio_direction_output(ISP_gpio, 1);
	mdelay(1);

	    
	
	msm_camera_config_single_vreg(AIT_ISPCtrl->dev,
				&ISP_vreg[2],
				(struct regulator **)&ISP_vreg_data[2],
				1);
	mdelay(1);
	    
	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_enable;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 1);
	gpio_free(ISP_gpio);
	mdelay(1);

	
	if(camera_index == CAMERA_INDEX_SUB_OV2722)
	CDBG("%s sub CAM_SEL low\n", __func__);
	else
	CDBG("%s sub CAM_SEL high\n", __func__);
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_CAM_SEL;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	}
	if(camera_index == CAMERA_INDEX_SUB_OV2722)
	gpio_direction_output(ISP_gpio, 0);
	else
	gpio_direction_output(ISP_gpio, 1);
	gpio_free(ISP_gpio);
	mdelay(1);
	    
	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_CAM_SEL2;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 0);
	gpio_free(ISP_gpio);
	mdelay(1);

	
	if(camera_index == CAMERA_INDEX_SUB_OV2722)
	CDBG("%s ISP_MCLK_SEL high\n", __func__);
	else
	CDBG("%s ISP_MCLK_SEL low\n", __func__);
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_MCLK_SEL;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	}
	
	if(camera_index == CAMERA_INDEX_SUB_OV2722)
	gpio_direction_output(ISP_gpio, 1);
	else
	gpio_direction_output(ISP_gpio, 0);
	
	gpio_free(ISP_gpio);
	mdelay(1);

	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_V_SR_3V;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 1);
	gpio_free(ISP_gpio);
	mdelay(1);

	
	
	rc = gpio_request_one(AIT_ISPCtrl->pdata->ISP_mclk, 1, "CAMIF_MCLK");
	if (rc < 0) {
		pr_err("%s, CAMIF_MCLK (%d) request failed", __func__, AIT_ISPCtrl->pdata->ISP_mclk);
		gpio_free(AIT_ISPCtrl->pdata->ISP_mclk);
	}

	if (AIT_ISPCtrl->AIT_pinctrl_status) {
		ret = pinctrl_select_state(AIT_ISPCtrl->AIT_pinctrl.pinctrl,
			AIT_ISPCtrl->AIT_pinctrl.gpio_state_active);
		if (ret)
			pr_err("%s:%d cannot set pin to active state",
				__func__, __LINE__);
		else
		CDBG("%s  pinctrl_select_state ok\n", __func__);
	}
	
	rc = msm_cam_clk_enable(AIT_ISPCtrl->sensor_dev, ISP_clk_info,
		AIT_ISPCtrl->ISP_clk, ARRAY_SIZE(ISP_clk_info), 1);
	if (rc < 0) {
		pr_err("%s: clk enable failed\n",__func__);
	}
	mdelay(1);
	    
	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_APG6;
	rc = gpio_request_one(ISP_gpio, 0, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) ISP_APG6 gpio_request_one failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 1);
	
	ISP_gpio1 = AIT_ISPCtrl->pdata->ISP_APG7;
	rc = gpio_request_one(ISP_gpio1, 0, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) ISP_APG7 request failed", ISP_gpio1);
	}
	gpio_direction_output(ISP_gpio1, 1);
	mdelay(1);

	
	
	ISP_gpio2 = AIT_ISPCtrl->pdata->ISP_reset;
	rc = gpio_request(ISP_gpio2, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio2);
	}
	gpio_direction_output(ISP_gpio2, 1);
	gpio_free(ISP_gpio2);
	mdelay(1);

	
	
	gpio_direction_output(ISP_gpio, 0);
	gpio_free(ISP_gpio);
	
	gpio_direction_output(ISP_gpio1, 0);
	gpio_free(ISP_gpio1);
	mdelay(5);

	CDBG("%s - \n", __func__);
	return rc;
}

static int ISP_vreg_off(void)
{

	int rc = 0;
	int i = 0;
	int ISP_gpio = 0;
	int ret = 0;

	CDBG("%s +\n", __func__);
	
	
	
	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_reset;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) ISP_reset request failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 0);
	gpio_free(ISP_gpio);
	mdelay(1);

	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_enable;
	rc = gpio_request(ISP_gpio, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) request failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 0);
	gpio_free(ISP_gpio);
	mdelay(1);

	
	
	rc = msm_cam_clk_enable(AIT_ISPCtrl->sensor_dev, ISP_clk_info,
		AIT_ISPCtrl->ISP_clk, ARRAY_SIZE(ISP_clk_info), 0);
	if (rc < 0)
		pr_err("disable MCLK failed\n");

	if (AIT_ISPCtrl->AIT_pinctrl_status) {
		ret = pinctrl_select_state(AIT_ISPCtrl->AIT_pinctrl.pinctrl,
				AIT_ISPCtrl->AIT_pinctrl.gpio_state_suspend);
		if (ret)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		devm_pinctrl_put(AIT_ISPCtrl->AIT_pinctrl.pinctrl);
	}
	AIT_ISPCtrl->AIT_pinctrl_status = 0;
	
	gpio_free(AIT_ISPCtrl->pdata->ISP_mclk);
	mdelay(1);

	
	
	ISP_gpio = AIT_ISPCtrl->pdata->ISP_APG6;
	rc = gpio_request_one(ISP_gpio, 0, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) ISP_APG6 gpio_request_one failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 0);
	gpio_free(ISP_gpio);

	ISP_gpio = AIT_ISPCtrl->pdata->ISP_APG7;
	rc = gpio_request_one(ISP_gpio, 0, "AIT_ISP");
	if (rc < 0) {
		pr_err("GPIO(%d) ISP_APG7 gpio_request_one failed", ISP_gpio);
	}
	gpio_direction_output(ISP_gpio, 0);
	gpio_free(ISP_gpio);
	mdelay(1);

	
	
	msm_camera_config_single_vreg(AIT_ISPCtrl->dev,
				&ISP_vreg[2],
				(struct regulator **)&ISP_vreg_data[2],
				0);
	mdelay(1);

	
	
	gpio_direction_output(AIT_ISPCtrl->pdata->ISP_gpio_vana, 0);
	gpio_free(AIT_ISPCtrl->pdata->ISP_gpio_vana);
	mdelay(1);
	    
	
	for (i = 1 ; i >= 0; i--){
		if (i == 0)
			ISP_gpio = AIT_ISPCtrl->pdata->ISP_1v2_enable;
		else
			ISP_gpio = AIT_ISPCtrl->pdata->ISP_1v8_enable;

		if (ISP_vreg[i].type == VREG_TYPE_GPIO) {
			rc = gpio_request(ISP_gpio, "AIT_ISP");
			if (rc < 0) {
				pr_err("GPIO(%d) request failed", ISP_gpio);
			}
			gpio_direction_output(ISP_gpio, 0);
			gpio_free(ISP_gpio);
		} else {
			
			msm_camera_config_single_vreg(AIT_ISPCtrl->dev,
				&ISP_vreg[i],
				(struct regulator **)&ISP_vreg_data[i],
				0);
		}
		mdelay(1);
	}
	CDBG("%s - \n", __func__);
	return rc;
}

int AIT_ISP_power_up(const struct msm_camera_AIT_info *pdata)
{
	int rc = 0;
	
	struct device *dev = NULL;
	dev = AIT_ISPCtrl->dev;
	
	CDBG("[CAM] %s\n", __func__);

	if (pdata->camera_ISP_power_on == NULL) {
		pr_err("AIT_ISP power on platform_data didn't register\n");
		return -EIO;
	}
	rc = pdata->camera_ISP_power_on();
	if (rc < 0) {
		pr_err("AIT_ISP power on failed\n");
	}
	return rc;
}

int AIT_ISP_power_down(const struct msm_camera_AIT_info *pdata)
{
	int rc = 0;
	
	struct device *dev = NULL;
	dev = AIT_ISPCtrl->dev;
	
	CDBG("%s\n", __func__);

	if (pdata->camera_ISP_power_off == NULL) {
		pr_err("AIT_ISP power off platform_data didn't register\n");
		return -EIO;
	}

	rc = pdata->camera_ISP_power_off();
	if (rc < 0)
		pr_err("AIT_ISP power off failed\n");

	return rc;
}

int AIT_ISP_match_id(void)
{
	int rc = 0;
	unsigned char read_id =0;
	uint16_t read =0;
	read_id = GetVenusRegB(0x6900);
	if(read_id != 0xd0)
	    pr_err("AIT_ISP_match_id read 0x6900 failure:0x%x\n", read_id);
	
	
	read_id = GetVenusRegB(0x6902);
	if(read_id != 0x5)
	    pr_err("AIT_ISP_match_id read 0x6902 failure:0x%x\n", read_id);
	
	
	SetVenusRegB(0x69F0, 0x01);
	read = GetVenusRegW(0x69FE);
	if(read != 0x8980)
	{
	    pr_err("AIT_ISP_match_id read 0x69FE failure:0x%x\n", read);
	    rc = -1;
	}
	else
	    CDBG("%s  match ok\n", __func__);
	
	return rc;
}

void AIT_ISP_release(void)
{
	struct msm_camera_AIT_info *pdata = AIT_ISPCtrl->pdata;
	CDBG("[CAM]%s\n", __func__);
	AIT_ISP_power_down(pdata);
}

int AIT_ISP_open_init(int cam_index)
{
	int rc = 0;
	struct msm_camera_AIT_info *pdata = AIT_ISPCtrl->pdata;
	int read_id_retry = 0;

	CDBG("%s, cam_index:%d\n", __func__, cam_index);
        camera_index = cam_index;
open_read_id_retry:
	rc = AIT_ISP_power_up(pdata);
	if (rc < 0)
		return rc;
	rc = AIT_ISP_match_id();
	if (rc < 0) {
		if (read_id_retry < 3) {
			CDBG("%s, retry read AIT_ISP ID: %d\n", __func__, read_id_retry);
			read_id_retry++;
			AIT_ISP_power_down(pdata);
			goto open_read_id_retry;
		}
		goto open_init_failed;
	}
	if(camera_index == CAMERA_INDEX_FRONT_S5K5E)
	{
		pr_info("%s: V3A_FirmwareStart: sizeof(AIT_fw_front):%lu, \n", __func__, sizeof(AIT_fw_sub));
		rc = V3A_FirmwareStart(pAIT_fw_front, sizeof(AIT_fw_front));
	}
	else
	{
		pr_info("%s: V3A_FirmwareStart: sizeof(AIT_fw_sub):%lu, \n", __func__, sizeof(AIT_fw_sub));
		rc = V3A_FirmwareStart(pAIT_fw_sub, sizeof(AIT_fw_sub));
	}

        CDBG("%s -\n", __func__);
	return rc;
#if 1
open_init_failed:
	pr_err("%s: AIT_ISP_open_init failed\n", __func__);
	AIT_ISP_power_down(pdata);
#endif
	return rc;
}

int AIT_ISP_probe_init(struct device *dev, int cam_index)
{
	int rc = 0;
	struct msm_camera_AIT_info *pdata = NULL;
	int read_id_retry = 0;

	CDBG("%s, cam_index:%d\n", __func__, cam_index);
        camera_index = cam_index;
	if (AIT_ISPCtrl == NULL) {
		pr_err("already failed in __msm_AIT_ISP_probe\n");
		return -EINVAL;
	} else
		pdata = AIT_ISPCtrl->pdata;

	AIT_ISPCtrl->sensor_dev = dev;
probe_read_id_retry:
	rc = AIT_ISP_power_up(pdata);
	if (rc < 0)
		return rc;

	rc = AIT_ISP_match_id();

	if (rc < 0) {
		if (read_id_retry < 3) {
			CDBG("%s, retry read AIT_ISP ID: %d\n", __func__, read_id_retry);
			read_id_retry++;
			AIT_ISP_power_down(pdata);
			goto probe_read_id_retry;
		}
		goto probe_init_fail;
	}
	
	CDBG("%s: probe_success\n", __func__);
	return rc;

probe_init_fail:
	pr_err("%s: AIT_ISP_probe_init failed\n", __func__);
	AIT_ISP_power_down(pdata);
	return rc;

}

void AIT_ISP_probe_deinit(void)
{
	struct msm_camera_AIT_info *pdata = AIT_ISPCtrl->pdata;
	CDBG("%s\n", __func__);

	AIT_ISP_power_down(pdata);
}


void AIT_ISP_sensor_init(void)
{
	int rc = 0;
	CDBG("%s +\n", __func__);
	rc = VA_InitializeSensor();
	CDBG("%s -\n", __func__);
}

void AIT_ISP_sensor_set_resolution(int width, int height)
{
	int rc = 0;
	CDBG("%s (%d x %d)\n", __func__, width, height);
	CDBG("%s VA_SetPreviewResolution+\n", __func__);
	rc = VA_SetPreviewResolution( width, height );
	CDBG("%s VA_SetPreviewResolution-\n", __func__);
	
	CDBG("%s VA_SetPreviewFormat+\n", __func__);
	rc = VA_SetPreviewFormat(VENUS_PREVIEW_FORMAT_YUV422);
	CDBG("%s VA_SetPreviewFormat-\n", __func__);
}

void AIT_ISP_sensor_start_stream(void)
{
	int rc = 0;
	CDBG("%s \n", __func__);
	CDBG("%s AIT_ISP_sensor_start_stream+\n", __func__);
	rc = VA_SetPreviewControl(VA_PREVIEW_STATUS_START);
	CDBG("%s AIT_ISP_sensor_start_stream-\n", __func__);
}

void AIT_ISP_sensor_stop_stream(void)
{
	int rc = 0;
	CDBG("%s \n", __func__);
	CDBG("%s AIT_ISP_sensor_stop_stream+\n", __func__);
	rc = VA_SetPreviewControl(VA_PREVIEW_STATUS_STOP);
	CDBG("%s AIT_ISP_sensor_stop_stream-\n", __func__);
}


static int AIT_ISP_driver_probe(struct platform_device *pdev)
{
	int rc = 0;
	int rc1 = 0;
	int i = 0;
	uint32_t tmp_array[5];
	CDBG("%s +\n", __func__);

	AIT_ISPCtrl = kzalloc(sizeof(struct AIT_ISP_ctrl), GFP_ATOMIC);
	if (!AIT_ISPCtrl) {
		pr_err("%s: could not allocate mem for AIT_ISP_dev\n", __func__);
		return -ENOMEM;
	}

	rc = setup_AIT_ISP_cdev();
	if (rc < 0) {
		kfree(AIT_ISPCtrl);
		return rc;
	}

	mutex_init(&AIT_ISPCtrl->AIT_ioctl_lock);

	if (!pdev->dev.of_node) {
		pr_err("%s:%d of_node NULL \n", __func__, __LINE__);
		return -EINVAL;
	}

	AIT_ISPCtrl->pdata = kzalloc(sizeof(struct msm_camera_AIT_info),
		GFP_ATOMIC);
	if (!AIT_ISPCtrl->pdata) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	memset(ISP_vreg, 0x0, sizeof(ISP_vreg));

	AIT_ISPCtrl->pdata->ISP_reset = of_get_named_gpio((&pdev->dev)->of_node, "ISP_reset",0);
	CDBG("ISP_reset %d\n", AIT_ISPCtrl->pdata->ISP_reset);
	if (AIT_ISPCtrl->pdata->ISP_reset < 0) {
		pr_err("%s:%d ISP_reset rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_reset);
	}

	AIT_ISPCtrl->pdata->ISP_enable = of_get_named_gpio((&pdev->dev)->of_node, "ISP_enable",0);
	CDBG("ISP_enable %d\n", AIT_ISPCtrl->pdata->ISP_enable);
	if (AIT_ISPCtrl->pdata->ISP_enable < 0) {
		pr_err("%s:%d ISP_enable rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_enable);
	}

	AIT_ISPCtrl->pdata->ISP_APG6 = of_get_named_gpio((&pdev->dev)->of_node, "ISP_APG6",0);
	CDBG("ISP_APG6 %d\n", AIT_ISPCtrl->pdata->ISP_APG6);
	if (AIT_ISPCtrl->pdata->ISP_APG6 < 0) {
		pr_err("%s:%d ISP_APG6 rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_APG6);
	}

	AIT_ISPCtrl->pdata->ISP_APG7 = of_get_named_gpio((&pdev->dev)->of_node, "ISP_APG7",0);
	CDBG("ISP_APG7 %d\n", AIT_ISPCtrl->pdata->ISP_APG7);
	if (AIT_ISPCtrl->pdata->ISP_APG7 < 0) {
		pr_err("%s:%d ISP_APG7 rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_APG7);
	}


	AIT_ISPCtrl->pdata->ISP_mclk = of_get_named_gpio((&pdev->dev)->of_node, "ISP_mclk",0);
	CDBG("ISP_mclk %d\n", AIT_ISPCtrl->pdata->ISP_mclk);
	if (AIT_ISPCtrl->pdata->ISP_mclk < 0) {
		pr_err("%s:%d ISP_mclk rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_mclk);
	}
	
	AIT_ISPCtrl->pdata->ISP_CAM_SEL = of_get_named_gpio((&pdev->dev)->of_node, "ISP_CAM_SEL",0);
	CDBG("ISP_CAM_SEL %d\n", AIT_ISPCtrl->pdata->ISP_CAM_SEL);
	if (AIT_ISPCtrl->pdata->ISP_CAM_SEL < 0) {
		pr_err("%s:%d ISP_CAM_SEL rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_CAM_SEL);
	}

	AIT_ISPCtrl->pdata->ISP_CAM_SEL2 = of_get_named_gpio((&pdev->dev)->of_node, "ISP_CAM_SEL2",0);
	CDBG("ISP_CAM_SEL2 %d\n", AIT_ISPCtrl->pdata->ISP_CAM_SEL2);
	if (AIT_ISPCtrl->pdata->ISP_CAM_SEL2 < 0) {
		pr_err("%s:%d ISP_CAM_SEL2 rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_CAM_SEL2);
	}

	AIT_ISPCtrl->pdata->ISP_MCLK_SEL = of_get_named_gpio((&pdev->dev)->of_node, "ISP_MCLK_SEL",0);
	CDBG("ISP_MCLK_SEL %d\n", AIT_ISPCtrl->pdata->ISP_MCLK_SEL);
	if (AIT_ISPCtrl->pdata->ISP_MCLK_SEL < 0) {
		pr_err("%s:%d ISP_MCLK_SEL rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_MCLK_SEL);
	}


	AIT_ISPCtrl->pdata->ISP_V_SR_3V = of_get_named_gpio((&pdev->dev)->of_node, "ISP_V_SR_3V",0);
	CDBG("ISP_V_SR_3V %d\n", AIT_ISPCtrl->pdata->ISP_V_SR_3V);
	if (AIT_ISPCtrl->pdata->ISP_V_SR_3V < 0) {
		pr_err("%s:%d ISP_V_SR_3V rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_V_SR_3V);
	}

	AIT_ISPCtrl->pdata->ISP_gpio_vana = of_get_named_gpio((&pdev->dev)->of_node, "ISP_gpio_vana",0);
	CDBG("ISP_gpio_vana %d\n", AIT_ISPCtrl->pdata->ISP_gpio_vana);
	if (AIT_ISPCtrl->pdata->ISP_gpio_vana < 0) {
		pr_err("%s:%d ISP_gpio_vana rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_gpio_vana);
	}

	ISP_vreg[0].reg_name = "ISP_1v2";
	ISP_vreg[1].reg_name = "ISP_1v8";
	ISP_vreg[2].reg_name = "ISP_A2v8";
	ISP_vreg_data[0] = NULL;
	ISP_vreg_data[1] = NULL;
	ISP_vreg_data[2] = NULL;

	AIT_ISPCtrl->pdata->camera_ISP_power_on = ISP_vreg_on;
	AIT_ISPCtrl->pdata->camera_ISP_power_off = ISP_vreg_off;

	rc = of_property_read_u32_array((&pdev->dev)->of_node, "ISP-vreg-type",
		tmp_array, 3);
	if (rc < 0) {
		pr_err("%s %d, ISP-vreg-type failed \n", __func__, __LINE__);
	}
	for (i = 0; i < 3; i++) {
		ISP_vreg[i].type = tmp_array[i];
		CDBG("%s ISP_vreg[%d].type = %d\n", __func__, i,
			ISP_vreg[i].type);
	}

	if (ISP_vreg[1].type == 2){
	    AIT_ISPCtrl->pdata->ISP_1v8_enable= of_get_named_gpio((&pdev->dev)->of_node, "ISP_1v8-gpio",0);
	    CDBG("ISP_1v8 %d\n", AIT_ISPCtrl->pdata->ISP_1v8_enable);
	    if (AIT_ISPCtrl->pdata->ISP_1v8_enable < 0) {
		pr_err("%s:%d ISP_1v2 rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_1v8_enable);
	    }
	}

	if (ISP_vreg[0].type == 2){
	    AIT_ISPCtrl->pdata->ISP_1v2_enable= of_get_named_gpio((&pdev->dev)->of_node, "ISP_1v2-gpio",0);
	    CDBG("ISP_1v2 %d\n", AIT_ISPCtrl->pdata->ISP_1v2_enable);
	    if (AIT_ISPCtrl->pdata->ISP_1v2_enable < 0) {
		pr_err("%s:%d ISP_1v2 rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_1v2_enable);
	    }
	}

	if (ISP_vreg[2].type == 2){
	    AIT_ISPCtrl->pdata->ISP_A2v8= of_get_named_gpio((&pdev->dev)->of_node, "ISP_A2v8-gpio",0);
	    CDBG("ISP_A2v8 %d\n", AIT_ISPCtrl->pdata->ISP_A2v8);
	    if (AIT_ISPCtrl->pdata->ISP_A2v8 < 0) {
		pr_err("%s:%d ISP_A2v8 rc %d\n", __func__, __LINE__, AIT_ISPCtrl->pdata->ISP_A2v8);
	    }
	}


	
	AIT_ISPCtrl->dev = &pdev->dev;
	

	rc1 = msm_AIT_ISP_attr_node();
	CDBG("%s -\n", __func__);

	return rc;
}

static int AIT_ISP_driver_remove(struct platform_device *pdev)
{
	CDBG("%s\n", __func__);
	AIT_ISP_tear_down_cdev();

	mutex_destroy(&AIT_ISPCtrl->AIT_ioctl_lock);

	kfree(AIT_ISPCtrl);

	return 0;
}

static const struct of_device_id AIT_dt_match[] = {
	{.compatible = "AIT_ISP", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, AIT_dt_match);



static struct  platform_driver AIT_ISP_driver = {
	.remove =AIT_ISP_driver_remove,
	.driver = {
		.name = "AIT_ISP",
		.owner = THIS_MODULE,
		.of_match_table = AIT_dt_match,
	},
};

static int __init AIT_ISP_driver_init(void)
{
	int rc;
	CDBG("%s + \n", __func__);
	rc = AIT_spi_init();
	if (rc < 0) {
		pr_err("%s: failed to register spi driver\n", __func__);
		return rc;
	}
	rc = platform_driver_probe(&AIT_ISP_driver, AIT_ISP_driver_probe);
	CDBG("%s - \n", __func__);
	return rc;
}

static void __exit AIT_ISP_driver_exit(void)
{
	CDBG("%s\n", __func__);
	platform_driver_unregister(&AIT_ISP_driver);
}

MODULE_DESCRIPTION("AIT_ISP driver");
MODULE_VERSION("AIT_ISP 0.1");

module_init(AIT_ISP_driver_init);
module_exit(AIT_ISP_driver_exit);

