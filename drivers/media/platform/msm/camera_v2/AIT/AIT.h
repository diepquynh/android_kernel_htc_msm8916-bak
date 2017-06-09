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
 */

#ifndef AIT_H
#define AIT_H

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>


#include <linux/kernel.h>
#include <linux/string.h>

#include <media/linux_AIT_ISP.h>
#include "../msm.h"
#include "../sensor/msm_sensor.h"
#include "../sensor/io/msm_camera_io_util.h"

#define CAMERA_INDEX_SUB_OV2722 0
#define CAMERA_INDEX_FRONT_S5K5E 1

struct AIT_ISP_ctrl {
	struct msm_camera_AIT_info *pdata;
	struct cdev   cdev;
	struct mutex AIT_ioctl_lock;
	struct device *dev;
	struct clk *ISP_clk[2];
	struct device *sensor_dev;
	struct msm_pinctrl_info AIT_pinctrl;
	uint8_t AIT_pinctrl_status;
};

int AIT_ISP_probe_init(struct device *dev, int cam_index);
void AIT_ISP_release(void);
void AIT_ISP_probe_deinit(void);
int AIT_ISP_open_init(int cam_index);
#if 1
void AIT_ISP_sensor_init(void);
void AIT_ISP_sensor_set_resolution(int width, int height);
void AIT_ISP_sensor_start_stream(void);
void AIT_ISP_sensor_stop_stream(void);
#endif
#endif
