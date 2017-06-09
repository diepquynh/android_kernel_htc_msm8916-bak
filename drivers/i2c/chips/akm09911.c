/* drivers/misc/akm09911.c - akm09911 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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


#include <linux/ak09911.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/export.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>



#define AKM_DEBUG_IF			0
#define AKM_HAS_RESET			1
#define AKM_INPUT_DEVICE_NAME		"compass"
#define AKM_DRDY_TIMEOUT_MS		100
#define AKM_BASE_NUM			10

#define D(x...) printk(KERN_DEBUG "[COMP][AKM09911] " x)
#define I(x...) printk(KERN_INFO "[COMP][AKM09911] " x)
#define E(x...) printk(KERN_ERR "[COMP][AKM09911] " x)


struct akm_compass_data {
	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct device		*class_dev;
	struct class		*compass;

	wait_queue_head_t	drdy_wq;
	wait_queue_head_t	open_wq;

	uint8_t sense_info[AKM_SENSOR_INFO_SIZE];
	uint8_t sense_conf[AKM_SENSOR_CONF_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[AKM_SENSOR_DATA_SIZE];
	struct mutex accel_mutex;
	int16_t accel_data[3];

	int8_t	is_busy;

	struct mutex	val_mutex;
	uint32_t		enable_flag;
	int64_t			delay[AKM_NUM_SENSORS];

	atomic_t	active;
	atomic_t	drdy;

	char layout;
	int	irq;
	int	gpio_rstn;
};

static struct akm_compass_data *s_akm;



static int akm_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *rxData,
	int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int akm_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

static int AKECS_Set_CNTL(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	
	mutex_lock(&akm->sensor_mutex);
	
	if (akm->is_busy > 0) {
		dev_err(&akm->i2c->dev,
				"%s: device is busy.", __func__);
		err = -EBUSY;
	} else {
		
		buffer[0] = AKM_REG_MODE;
		buffer[1] = mode;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&akm->i2c->dev,
					"%s: Can not set CNTL.", __func__);
		} else {
			dev_vdbg(&akm->i2c->dev,
					"Mode is set to (%d).", mode);
			
			akm->is_busy = 1;
			atomic_set(&akm->drdy, 0);
			
			udelay(100);
		}
	}

	mutex_unlock(&akm->sensor_mutex);
	

	return err;
}

static int AKECS_Set_PowerDown(
	struct akm_compass_data *akm)
{
	uint8_t buffer[2];
	int err;

	
	mutex_lock(&akm->sensor_mutex);

	
	buffer[0] = AKM_REG_MODE;
	buffer[1] = AKM_MODE_POWERDOWN;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		dev_err(&akm->i2c->dev,
			"%s: Can not set to powerdown mode.", __func__);
	} else {
		dev_dbg(&akm->i2c->dev, "Powerdown mode is set.");
		
		udelay(100);
	}
	
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	

	return err;
}

static int AKECS_Reset(
	struct akm_compass_data *akm,
	int hard)
{
	int err;

#if AKM_HAS_RESET
	uint8_t buffer[2];

	
	mutex_lock(&akm->sensor_mutex);

	if (hard != 0) {
		gpio_set_value(akm->gpio_rstn, 0);
		udelay(5);
		gpio_set_value(akm->gpio_rstn, 1);
		
		err = 0;
	} else {
		buffer[0] = AKM_REG_RESET;
		buffer[1] = AKM_RESET_DATA;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&akm->i2c->dev,
				"%s: Can not set SRST bit.", __func__);
		} else {
			dev_dbg(&akm->i2c->dev, "Soft reset is done.");
		}
	}
	
	udelay(100);
	
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	

#else
	err = AKECS_Set_PowerDown(akm);
#endif

	return err;
}

static int AKECS_SetMode(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	int err;

	switch (mode & 0x1F) {
	case AKM_MODE_SNG_MEASURE:
	case AKM_MODE_SELF_TEST:
	case AKM_MODE_FUSE_ACCESS:
		err = AKECS_Set_CNTL(akm, mode);
		break;
	case AKM_MODE_POWERDOWN:
		err = AKECS_Set_PowerDown(akm);
		break;
	default:
		dev_err(&akm->i2c->dev,
			"%s: Unknown mode(%d).", __func__, mode);
		return -EINVAL;
	}

	return err;
}

static void AKECS_SetYPR(
	struct akm_compass_data *akm,
	int32_t *rbuf)
{
	uint32_t ready;
	dev_vdbg(&akm->i2c->dev, "%s: flag =0x%X", __func__, rbuf[0]);
	dev_vdbg(&akm->input->dev, "  Acc [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
	dev_vdbg(&akm->input->dev, "  Geo [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[5], rbuf[6], rbuf[7], rbuf[8]);
	dev_vdbg(&akm->input->dev, "  Gyro[LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[9], rbuf[10], rbuf[11], rbuf[12]);
	dev_vdbg(&akm->input->dev, "  Orientation : %6d,%6d,%6d",
		rbuf[13], rbuf[14], rbuf[15]);
	dev_vdbg(&akm->input->dev, "  Gravity     : %6d,%6d,%6d",
		rbuf[16], rbuf[17], rbuf[18]);
	dev_vdbg(&akm->input->dev, "  Linear Acc  : %6d,%6d,%6d",
		rbuf[19], rbuf[20], rbuf[21]);
	dev_vdbg(&akm->input->dev, "  Geomagnetic Rov  : %6d,%6d,%6d,%6d,%6d",
		rbuf[22], rbuf[23], rbuf[24], rbuf[25], rbuf[26]);
	dev_vdbg(&akm->input->dev, "  Uncalibrated MagV  : %6d,%6d,%6d",
		rbuf[27], rbuf[28], rbuf[29]);
	dev_vdbg(&akm->input->dev, "  Bias Magv  : %6d,%6d,%6d",
		rbuf[30], rbuf[31], rbuf[32]);

	
	if (!rbuf[0]) {
		dev_dbg(&akm->i2c->dev, "Don't waste a time.");
		return;
	}

	mutex_lock(&akm->val_mutex);
	ready = (akm->enable_flag & (uint32_t)rbuf[0]);
	mutex_unlock(&akm->val_mutex);

	
	if (ready & ACC_DATA_READY) {
		input_report_abs(akm->input, AKM_EVABS_ACC_X, 10000);
		input_report_abs(akm->input, AKM_EVABS_ACC_X, rbuf[1]);
		input_report_abs(akm->input, AKM_EVABS_ACC_Y, rbuf[2]);
		input_report_abs(akm->input, AKM_EVABS_ACC_Z, rbuf[3]);
		input_report_abs(akm->input, AKM_EVABS_ACC_ST, rbuf[4]);
	}
	
	if (ready & MAG_DATA_READY) {
		input_report_abs(akm->input, AKM_EVABS_MAG_X, 10000);
		input_report_abs(akm->input, AKM_EVABS_MAG_X, rbuf[5]);
		input_report_abs(akm->input, AKM_EVABS_MAG_Y, rbuf[6]);
		input_report_abs(akm->input, AKM_EVABS_MAG_Z, rbuf[7]);
		input_report_abs(akm->input, AKM_EVABS_MAG_ST, rbuf[8]);
	}
	
	if (ready & FUSION_DATA_READY) {
		
		input_report_abs(akm->input, AKM_EVABS_GYR_X, rbuf[9]);
		input_report_abs(akm->input, AKM_EVABS_GYR_Y, rbuf[10]);
		input_report_abs(akm->input, AKM_EVABS_GYR_Z, rbuf[11]);
		input_report_abs(akm->input, AKM_EVABS_GYR_ST, rbuf[12]);
		
		input_report_abs(akm->input, AKM_EVABS_ORI_Y, 10000);
		input_report_abs(akm->input, AKM_EVABS_ORI_Y, rbuf[13]);
		input_report_abs(akm->input, AKM_EVABS_ORI_P, rbuf[14]);
		input_report_abs(akm->input, AKM_EVABS_ORI_R, rbuf[15]);
		input_report_abs(akm->input, AKM_EVABS_MAG_ST, rbuf[8]);
		
		input_report_abs(akm->input, AKM_EVABS_GRV_X, rbuf[16]);
		input_report_abs(akm->input, AKM_EVABS_GRV_Y, rbuf[17]);
		input_report_abs(akm->input, AKM_EVABS_GRV_Z, rbuf[18]);
		
		input_report_abs(akm->input, AKM_EVABS_LAC_X, rbuf[19]);
		input_report_abs(akm->input, AKM_EVABS_LAC_Y, rbuf[20]);
		input_report_abs(akm->input, AKM_EVABS_LAC_Z, rbuf[21]);
		
		input_report_abs(akm->input, AKM_EVABS_GEORV_X, 10000);
		input_report_abs(akm->input, AKM_EVABS_GEORV_X, rbuf[22]);
		input_report_abs(akm->input, AKM_EVABS_GEORV_Y, rbuf[23]);
		input_report_abs(akm->input, AKM_EVABS_GEORV_Z, rbuf[24]);
		input_report_abs(akm->input, AKM_EVABS_GEORV_W, rbuf[25]);
		input_report_abs(akm->input, AKM_EVABS_GEORV_ST, rbuf[26]);
	}
	

	if (ready & MAGUC_DATA_READY) {
		
		input_report_abs(akm->input, AKM_EVABS_MUC_X, 10000);
		input_report_abs(akm->input, AKM_EVABS_MUC_X, rbuf[27]);
		input_report_abs(akm->input, AKM_EVABS_MUC_Y, rbuf[28]);
		input_report_abs(akm->input, AKM_EVABS_MUC_Z, rbuf[29]);
		
		input_report_abs(akm->input, AKM_EVABS_MUB_X, rbuf[30]);
		input_report_abs(akm->input, AKM_EVABS_MUB_Y, rbuf[31]);
		input_report_abs(akm->input, AKM_EVABS_MUB_Z, rbuf[32]);
	}
	input_sync(akm->input);
}

static int AKECS_GetData(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	int err;

	
	err = wait_event_interruptible_timeout(
			akm->drdy_wq,
			atomic_read(&akm->drdy),
			msecs_to_jiffies(AKM_DRDY_TIMEOUT_MS));

	if (err < 0) {
		dev_err(&akm->i2c->dev,
			"%s: wait_event failed (%d).", __func__, err);
		return err;
	}
	if (!atomic_read(&akm->drdy)) {
		dev_err(&akm->i2c->dev,
			"%s: DRDY is not set.", __func__);
		return -ENODATA;
	}

	
	mutex_lock(&akm->sensor_mutex);

	memcpy(rbuf, akm->sense_data, size);
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	

	return 0;
}

static int AKECS_GetData_Poll(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, 1);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s failed.", __func__);
		return err;
	}

	
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		return -EAGAIN;

	
	buffer[1] = AKM_REG_STATUS + 1;
	err = akm_i2c_rxdata(akm->i2c, &(buffer[1]), AKM_SENSOR_DATA_SIZE-1);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s failed.", __func__);
		return err;
	}

	memcpy(rbuf, buffer, size);
	atomic_set(&akm->drdy, 0);

	
	mutex_lock(&akm->sensor_mutex);
	akm->is_busy = 0;
	mutex_unlock(&akm->sensor_mutex);
	

	return 0;
}

static int AKECS_GetOpenStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) > 0));
}

static int AKECS_GetCloseStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) <= 0));
}

static int AKECS_Open(struct inode *inode, struct file *file)
{
	file->private_data = s_akm;
	return nonseekable_open(inode, file);
}

static int AKECS_Release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
AKECS_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct akm_compass_data *akm = file->private_data;

	
	uint8_t i2c_buf[AKM_RWBUF_SIZE] = {0};		
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];
	int32_t ypr_buf[AKM_YPR_DATA_SIZE] = {0};		
	int64_t delay[AKM_NUM_SENSORS];	
	int16_t acc_buf[3];	
	uint8_t mode = 0;			
	int status;			
	int ret = 0;		

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf))) {
			dev_err(&akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			dev_err(&akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			dev_err(&akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
	case ECS_IOCTL_GET_INFO:
	case ECS_IOCTL_GET_CONF:
	case ECS_IOCTL_GET_DATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
	case ECS_IOCTL_GET_ACCEL:
		
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		dev_vdbg(&akm->i2c->dev, "IOCTL_READ called.");
		if ((i2c_buf[0] < 1) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = akm_i2c_rxdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		dev_vdbg(&akm->i2c->dev, "IOCTL_WRITE called.");
		if ((i2c_buf[0] < 2) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = akm_i2c_txdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		dev_vdbg(&akm->i2c->dev, "IOCTL_RESET called.");
		ret = AKECS_Reset(akm, akm->gpio_rstn);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		dev_vdbg(&akm->i2c->dev, "IOCTL_SET_MODE called.");
		ret = AKECS_SetMode(akm, mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		dev_vdbg(&akm->i2c->dev, "IOCTL_SET_YPR called.");
		AKECS_SetYPR(akm, ypr_buf);
		break;
	case ECS_IOCTL_GET_DATA:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_DATA called.");
		if (akm->irq)
			ret = AKECS_GetData(akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		else
			ret = AKECS_GetData_Poll(
					akm, dat_buf, AKM_SENSOR_DATA_SIZE);

		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_OPEN_STATUS called.");
		ret = AKECS_GetOpenStatus(akm);
		if (ret < 0) {
			dev_err(&akm->i2c->dev,
				"Get Open returns error (%d).", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_CLOSE_STATUS called.");
		ret = AKECS_GetCloseStatus(akm);
		if (ret < 0) {
			dev_err(&akm->i2c->dev,
				"Get Close returns error (%d).", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_DELAY called.");
		mutex_lock(&akm->val_mutex);
		delay[ACC_DATA_FLAG] = ((akm->enable_flag & ACC_DATA_READY) ?
				akm->delay[ACC_DATA_FLAG] : -1);
		delay[MAG_DATA_FLAG] = ((akm->enable_flag & MAG_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		delay[MAGUC_DATA_FLAG] = ((akm->enable_flag & MAGUC_DATA_READY) ?
				akm->delay[MAGUC_DATA_FLAG] : -1);
		delay[FUSION_DATA_FLAG] = ((akm->enable_flag & FUSION_DATA_READY) ?
				akm->delay[FUSION_DATA_FLAG] : -1);
		mutex_unlock(&akm->val_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_INFO called.");
		break;
	case ECS_IOCTL_GET_CONF:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_CONF called.");
		break;
	case ECS_IOCTL_GET_LAYOUT:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_LAYOUT called.");
		break;
	case ECS_IOCTL_GET_ACCEL:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_ACCEL called.");
		mutex_lock(&akm->accel_mutex);
		acc_buf[0] = akm->accel_data[0];
		acc_buf[1] = akm->accel_data[1];
		acc_buf[2] = akm->accel_data[2];
		mutex_unlock(&akm->accel_mutex);
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		
		if (copy_to_user(argp, &i2c_buf, i2c_buf[0]+1)) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		if (copy_to_user(argp, &akm->sense_info,
					sizeof(akm->sense_info))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		if (copy_to_user(argp, &akm->sense_conf,
					sizeof(akm->sense_conf))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &dat_buf, sizeof(dat_buf))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = atomic_read(&akm->active);
		if (copy_to_user(argp, &status, sizeof(status))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &akm->layout, sizeof(akm->layout))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations AKECS_fops = {
	.owner = THIS_MODULE,
	.open = AKECS_Open,
	.release = AKECS_Release,
	.unlocked_ioctl = AKECS_ioctl,
};

static struct miscdevice akm_compass_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AKM_MISCDEV_NAME,
	.fops = &AKECS_fops,
};

static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		device_remove_file(dev, &attrs[i]);
}

static int create_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;
	int err = 0;

	err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = sysfs_create_bin_file(kobj, &attrs[i]);
		if (0 != err)
			break;
	}

	if (0 != err) {
		for (--i; i >= 0 ; --i)
			sysfs_remove_bin_file(kobj, &attrs[i]);
	}

	return err;
}

static void remove_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		sysfs_remove_bin_file(kobj, &attrs[i]);
}


static void akm_compass_sysfs_update_status(
	struct akm_compass_data *akm)
{
	uint32_t en;
	mutex_lock(&akm->val_mutex);
	en = akm->enable_flag;
	mutex_unlock(&akm->val_mutex);

	if (en == 0) {
		if (atomic_cmpxchg(&akm->active, 1, 0) == 1) {
			wake_up(&akm->open_wq);
			dev_dbg(akm->class_dev, "Deactivated");
		}
	} else {
		if (atomic_cmpxchg(&akm->active, 0, 1) == 0) {
			wake_up(&akm->open_wq);
			dev_dbg(akm->class_dev, "Activated");
		}
	}
	dev_dbg(&akm->i2c->dev,
		"Status updated: enable=0x%X, active=%d",
		en, atomic_read(&akm->active));
}

static ssize_t akm_compass_sysfs_enable_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int flag;

	mutex_lock(&akm->val_mutex);
	flag = ((akm->enable_flag >> pos) & 1);
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", flag);
}

static ssize_t akm_compass_sysfs_enable_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long en = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<pos);
	akm->enable_flag |= ((uint32_t)(en))<<pos;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);

	return count;
}

static ssize_t akm_enable_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t akm_enable_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

static ssize_t akm_enable_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t akm_enable_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

static ssize_t akm_enable_maguc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, MAGUC_DATA_FLAG);
}
static ssize_t akm_enable_maguc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, MAGUC_DATA_FLAG);
}

static ssize_t akm_enable_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t akm_enable_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

static ssize_t akm_compass_sysfs_delay_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int64_t val;

	mutex_lock(&akm->val_mutex);
	val = akm->delay[pos];
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", val);
}

static ssize_t akm_compass_sysfs_delay_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtoll(buf, AKM_BASE_NUM, &val))
		return -EINVAL;

	mutex_lock(&akm->val_mutex);
	akm->delay[pos] = val;
	mutex_unlock(&akm->val_mutex);

	return count;
}

static ssize_t akm_delay_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t akm_delay_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

static ssize_t akm_delay_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t akm_delay_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

static ssize_t akm_delay_maguc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, MAGUC_DATA_FLAG);
}
static ssize_t akm_delay_maguc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, MAGUC_DATA_FLAG);
}

static ssize_t akm_delay_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t akm_delay_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

static ssize_t akm_bin_accel_write(
	struct file *file,
	struct kobject *kobj,
	struct bin_attribute *attr,
		char *buf,
		loff_t pos,
		size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int16_t *accel_data;

	if (size == 0)
		return 0;

	accel_data = (int16_t *)buf;

	mutex_lock(&akm->accel_mutex);
	akm->accel_data[0] = accel_data[0];
	akm->accel_data[1] = accel_data[1];
	akm->accel_data[2] = accel_data[2];
	mutex_unlock(&akm->accel_mutex);

	dev_vdbg(&akm->i2c->dev, "accel:%d,%d,%d\n",
			accel_data[0], accel_data[1], accel_data[2]);

	return size;
}


#if AKM_DEBUG_IF
static ssize_t akm_sysfs_mode_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	long mode = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &mode))
		return -EINVAL;

	if (AKECS_SetMode(akm, (uint8_t)mode) < 0)
		return -EINVAL;

	return 1;
}

static ssize_t akm_buf_print(
	char *buf, uint8_t *data, size_t num)
{
	int sz, i;
	char *cur;
	size_t cur_len;

	cur = buf;
	cur_len = PAGE_SIZE;
	sz = snprintf(cur, cur_len, "(HEX):");
	if (sz < 0)
		return sz;
	cur += sz;
	cur_len -= sz;
	for (i = 0; i < num; i++) {
		sz = snprintf(cur, cur_len, "%02X,", *data);
		if (sz < 0)
			return sz;
		cur += sz;
		cur_len -= sz;
		data++;
	}
	sz = snprintf(cur, cur_len, "\n");
	if (sz < 0)
		return sz;
	cur += sz;

	return (ssize_t)(cur - buf);
}

static ssize_t akm_sysfs_bdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	uint8_t rbuf[AKM_SENSOR_DATA_SIZE];

	mutex_lock(&akm->sensor_mutex);
	memcpy(&rbuf, akm->sense_data, sizeof(rbuf));
	mutex_unlock(&akm->sensor_mutex);

	return akm_buf_print(buf, rbuf, AKM_SENSOR_DATA_SIZE);
}

static ssize_t akm_sysfs_asa_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t asa[3];

	err = AKECS_SetMode(akm, AKM_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	asa[0] = AKM_FUSE_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, asa, 3);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return akm_buf_print(buf, asa, 3);
}

static ssize_t akm_sysfs_regs_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t regs[AKM_REGS_SIZE];

	
	regs[0] = AKM_REGS_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, regs, AKM_REGS_SIZE);
	if (err < 0)
		return err;

	return akm_buf_print(buf, regs, AKM_REGS_SIZE);
}
#endif

static struct device_attribute akm_compass_attributes[] = {
	__ATTR(enable_acc, 0660, akm_enable_acc_show, akm_enable_acc_store),
	__ATTR(enable_mag, 0660, akm_enable_mag_show, akm_enable_mag_store),
	__ATTR(enable_maguc, 0660, akm_enable_maguc_show, akm_enable_maguc_store),
	__ATTR(enable_fusion, 0660, akm_enable_fusion_show,
			akm_enable_fusion_store),
	__ATTR(delay_acc,  0660, akm_delay_acc_show,  akm_delay_acc_store),
	__ATTR(delay_mag,  0660, akm_delay_mag_show,  akm_delay_mag_store),
	__ATTR(delay_maguc,  0660, akm_delay_maguc_show,  akm_delay_maguc_store),
	__ATTR(delay_fusion, 0660, akm_delay_fusion_show,
			akm_delay_fusion_store),
#if AKM_DEBUG_IF
	__ATTR(mode,  0220, NULL, akm_sysfs_mode_store),
	__ATTR(bdata, 0440, akm_sysfs_bdata_show, NULL),
	__ATTR(asa,   0440, akm_sysfs_asa_show, NULL),
	__ATTR(regs,  0440, akm_sysfs_regs_show, NULL),
#endif
	__ATTR_NULL,
};

#define __BIN_ATTR(name_, mode_, size_, private_, read_, write_) \
	{ \
		.attr    = { .name = __stringify(name_), .mode = mode_ }, \
		.size    = size_, \
		.private = private_, \
		.read    = read_, \
		.write   = write_, \
	}

#define __BIN_ATTR_NULL \
	{ \
		.attr   = { .name = NULL }, \
	}

static struct bin_attribute akm_compass_bin_attributes[] = {
	__BIN_ATTR(accel, 0220, 6, NULL,
				NULL, akm_bin_accel_write),
	__BIN_ATTR_NULL
};

static char const *const device_link_name = "i2c";
static dev_t const akm_compass_device_dev_t = MKDEV(MISC_MAJOR, 240);

static int create_sysfs_interfaces(struct akm_compass_data *akm)
{
	int err;

	if (NULL == akm)
		return -EINVAL;

	err = 0;

	akm->compass = class_create(THIS_MODULE, AKM_SYSCLS_NAME);
	if (IS_ERR(akm->compass)) {
		err = PTR_ERR(akm->compass);
		goto exit_class_create_failed;
	}

	akm->class_dev = device_create(
						akm->compass,
						NULL,
						akm_compass_device_dev_t,
						akm,
						AKM_SYSDEV_NAME);
	if (IS_ERR(akm->class_dev)) {
		err = PTR_ERR(akm->class_dev);
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&akm->class_dev->kobj,
			&akm->i2c->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	err = create_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
	if (0 > err)
		goto exit_device_binary_attributes_create_failed;

	return err;

exit_device_binary_attributes_create_failed:
	remove_device_attributes(akm->class_dev, akm_compass_attributes);
exit_device_attributes_create_failed:
	sysfs_remove_link(&akm->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(akm->compass, akm_compass_device_dev_t);
exit_class_device_create_failed:
	akm->class_dev = NULL;
	class_destroy(akm->compass);
exit_class_create_failed:
	akm->compass = NULL;
	return err;
}

static void remove_sysfs_interfaces(struct akm_compass_data *akm)
{
	if (NULL == akm)
		return;

	if (NULL != akm->class_dev) {
		remove_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
		remove_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
		sysfs_remove_link(
			&akm->class_dev->kobj,
			device_link_name);
		akm->class_dev = NULL;
	}
	if (NULL != akm->compass) {
		device_destroy(
			akm->compass,
			akm_compass_device_dev_t);
		class_destroy(akm->compass);
		akm->compass = NULL;
	}
}


static int akm_compass_input_init(
	struct input_dev **input)
{
	int err = 0;

	
	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	
	set_bit(EV_ABS, (*input)->evbit);
	
	input_set_abs_params(*input, AKM_EVABS_ACC_X,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_ACC_Y,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_ACC_Z,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_ACC_ST,
			0, 3, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_MAG_X,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MAG_Y,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MAG_Z,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MAG_ST,
			0, 3, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_GYR_X,
			-128000, 128000, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GYR_Y,
			-128000, 128000, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GYR_Z,
			-128000, 128000, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GYR_ST,
			0, 3, 0, 0);
	
	
	input_set_abs_params(*input, AKM_EVABS_ORI_Y,
			0, 23040, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_ORI_P,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_ORI_R,
			-5760, 5760, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_GRV_X,
			-1440, 1440, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GRV_Y,
			-1440, 1440, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GRV_Z,
			-1440, 1440, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_LAC_X,
			-115200, 115200, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_LAC_Y,
			-115200, 115200, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_LAC_Z,
			-115200, 115200, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_GEORV_X,
			-16384, 16384, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GEORV_Y,
			-16384, 16384, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GEORV_Z,
			-16384, 16384, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_GEORV_W,
			-16384, 16384, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_GEORV_ST,
			0, 11520, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_MUC_X,
			-524288, 524287, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MUC_Y,
			-524288, 524287, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MUC_Z,
			-524288, 524287, 0, 0);
	
	input_set_abs_params(*input, AKM_EVABS_MUB_X,
			-524288, 524287, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MUB_Y,
			-524288, 524287, 0, 0);
	input_set_abs_params(*input, AKM_EVABS_MUB_Z,
			-524288, 524287, 0, 0);

	
	(*input)->name = AKM_INPUT_DEVICE_NAME;

	
	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

static irqreturn_t akm_compass_irq(int irq, void *handle)
{
	struct akm_compass_data *akm = handle;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	memset(buffer, 0, sizeof(buffer));

	
	mutex_lock(&akm->sensor_mutex);

	
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "IRQ I2C error.");
		akm->is_busy = 0;
		mutex_unlock(&akm->sensor_mutex);
		

		return IRQ_HANDLED;
	}
	
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		goto work_func_none;

	memcpy(akm->sense_data, buffer, AKM_SENSOR_DATA_SIZE);
	akm->is_busy = 0;

	mutex_unlock(&akm->sensor_mutex);
	

	atomic_set(&akm->drdy, 1);
	wake_up(&akm->drdy_wq);

	dev_vdbg(&akm->i2c->dev, "IRQ handled.");
	return IRQ_HANDLED;

work_func_none:
	mutex_unlock(&akm->sensor_mutex);
	

	dev_vdbg(&akm->i2c->dev, "IRQ not handled.");
	return IRQ_NONE;
}

static int akm_compass_suspend(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	dev_dbg(&akm->i2c->dev, "suspended\n");

	return 0;
}

static int akm_compass_resume(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	dev_dbg(&akm->i2c->dev, "resumed\n");

	return 0;
}

static int akm09911_i2c_check_device(
	struct i2c_client *client)
{
	
	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int err;

	akm->sense_info[0] = AK09911_REG_WIA1;
	err = akm_i2c_rxdata(client, akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	
	err = AKECS_SetMode(akm, AK09911_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	akm->sense_conf[0] = AK09911_FUSE_ASAX;
	err = akm_i2c_rxdata(client, akm->sense_conf, AKM_SENSOR_CONF_SIZE);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AK09911_MODE_POWERDOWN);
	if (err < 0)
		return err;

	
	if ((akm->sense_info[0] != AK09911_WIA1_VALUE) ||
			(akm->sense_info[1] != AK09911_WIA2_VALUE)) {
		dev_err(&client->dev,
			"%s: The device is not AKM Compass.", __func__);
		return -ENXIO;
	}

	return err;
}

static int akm09911_compass_parse_dt(struct device *dev, struct akm09911_platform_data *pdata)
{
        struct property *prop = NULL;
        struct device_node *dt = dev->of_node;
        u32 buf = 0;
        uint32_t irq_gpio_flags = 0;

        if (s_akm == NULL) {
                E("%s: s_akm is NULL\n", __func__);
                return -EINVAL;
        }

        prop = of_find_property(dt, "compass_akm09911,layout", NULL);
        if (prop) {
                of_property_read_u32(dt, "compass_akm09911,layout", &buf);
                pdata->layout = buf;
                I("%s: layout = %d\n", __func__, pdata->layout);
        } else
                I("%s: compass_akm09911,layout not found", __func__);

        
        
        
        
        
        
        
#if 0
        pdata->gpio_DRDY = of_get_named_gpio_flags(dt,
                                                  "compass_akm09911,gpio_DRDY",
                                                  0,
                                                  &irq_gpio_flags);
        if (pdata->gpio_DRDY < 0) {
                E("%s: of_get_named_gpio_flags fails: pdata->gpio_DRDY\n", __func__);
                return -EINVAL;
        }
        I("%s: gpio_DRDY = %d", __func__, pdata->gpio_DRDY);
#endif

        pdata->gpio_RSTN = of_get_named_gpio_flags(dt,
                                                  "compass_akm09911,gpio_RSTN",
                                                  0,
                                                  &irq_gpio_flags);

	I("%s: RSTN = %d\n", __func__, pdata->gpio_RSTN);
        if (pdata->gpio_RSTN < 0) {
                E("%s: of_get_named_gpio_flags fails: pdata->gpio_RSTN\n", __func__);
                pdata->gpio_RSTN = 0;
        }

	
        

        return 0;
}

int akm_compass_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm09911_platform_data *pdata;
	int err = 0;
	int i;

	dev_dbg(&client->dev, "start probing.");
        I("AKM09911_probe.\n");


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	
	s_akm = kzalloc(sizeof(struct akm_compass_data), GFP_KERNEL);
	if (!s_akm) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}

	
	init_waitqueue_head(&s_akm->drdy_wq);
	init_waitqueue_head(&s_akm->open_wq);

	mutex_init(&s_akm->sensor_mutex);
	mutex_init(&s_akm->accel_mutex);
	mutex_init(&s_akm->val_mutex);

	atomic_set(&s_akm->active, 0);
	atomic_set(&s_akm->drdy, 0);

	s_akm->is_busy = 0;
	s_akm->enable_flag = 0;

	
	s_akm->accel_data[0] = 0;
	s_akm->accel_data[1] = 0;
	s_akm->accel_data[2] = 720;

	for (i = 0; i < AKM_NUM_SENSORS; i++)
		s_akm->delay[i] = -1;

	

        if (client->dev.of_node) {
                I("Device Tree parsing.");
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
                if (pdata == NULL) {
                        err = -ENOMEM;
                        dev_err(&client->dev, "%s: memory allocation "
                                        "for pdata failed.",
                                        __func__);
                        goto exit2;
                }

                err = akm09911_compass_parse_dt(&client->dev, pdata);
                if (err) {
                        dev_err(&client->dev,
                                "Unable to parse platfrom data err=%d\n", err);
                        goto exit3; 
                }
        } else {
		pdata = client->dev.platform_data;
	}

	if(pdata) {
                
                s_akm->layout = pdata->layout;
                s_akm->gpio_rstn = pdata->gpio_RSTN;
       

	} else {
                dev_dbg(&client->dev, "%s: No platform data.", __func__);
                s_akm->layout = 0;
                s_akm->gpio_rstn = 0;
       
	}

	
	s_akm->i2c = client;
	
	i2c_set_clientdata(client, s_akm);
	
	err = akm09911_i2c_check_device(client);
	if (err < 0)
		goto exit3;

	
	err = akm_compass_input_init(&s_akm->input);
	if (err) {
		dev_err(&client->dev,
			"%s: input_dev register failed", __func__);
		goto exit4;
	}
	input_set_drvdata(s_akm->input, s_akm);

	
	s_akm->irq = client->irq;

	dev_dbg(&client->dev, "%s: IRQ is #%d.",
			__func__, s_akm->irq);

	if (s_akm->irq) {
		err = request_threaded_irq(
				s_akm->irq,
				NULL,
				akm_compass_irq,
				IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
				dev_name(&client->dev),
				s_akm);
		if (err < 0) {
			dev_err(&client->dev,
				"%s: request irq failed.", __func__);
			goto exit5;
		}
	}

	
	err = misc_register(&akm_compass_dev);
	if (err) {
		E("%s: akm_compass_dev register failed\n", __func__);
		goto exit6;
	}

	
	err = create_sysfs_interfaces(s_akm);
	if (0 > err) {
		E("%s: create sysfs failed.\n", __func__);
		goto exit7;
	}

	if (pdata) {
		kfree(pdata);
		pdata = NULL;
	}
	I("successfully probed.");
	return 0;
exit7:
	misc_deregister(&akm_compass_dev);
exit6:
	if (s_akm->irq)
		free_irq(s_akm->irq, s_akm);
exit5:
	input_unregister_device(s_akm->input);
exit4:
exit3:
	if (pdata) {
		kfree(pdata);
		pdata = NULL;
	}
exit2:
	if(s_akm)
		kfree(s_akm);
exit1:
exit0:
	return err;
}

static int akm_compass_remove(struct i2c_client *client)
{
	struct akm_compass_data *akm = i2c_get_clientdata(client);

	remove_sysfs_interfaces(akm);
	if (misc_deregister(&akm_compass_dev) < 0)
		dev_err(&client->dev, "misc deregister failed.");
	if (akm->irq)
		free_irq(akm->irq, akm);
	input_unregister_device(akm->input);
	kfree(akm);
	dev_info(&client->dev, "successfully removed.");
	return 0;
}

static const struct i2c_device_id akm_compass_id[] = {
	{AKM_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, akm_compass_id);
#ifdef CONFIG_OF
static struct of_device_id akm09911_match_table[] = {
        {.compatible = "htc_compass,akm09911" },
        {},
};
#else
#define akm09911_match_table NULL
#endif

static const struct dev_pm_ops akm_compass_pm_ops = {
#ifdef CONFIG_PM
	.suspend	= akm_compass_suspend,
	.resume		= akm_compass_resume,
#endif
};

static struct i2c_driver akm_compass_driver = {
#ifdef CONFIG_OF
	.probe		= akm_compass_probe,
	.remove		= akm_compass_remove,
	.id_table	= akm_compass_id,
	.driver = {
		.name	= AKM_I2C_NAME,
		.owner 	= THIS_MODULE,
		.of_match_table = akm09911_match_table,
#ifdef CONFIG_PM
		.pm	= &akm_compass_pm_ops,
#endif 
	},
#else 
        .probe          = akm_compass_probe,
        .remove         = akm_compass_remove,
        .id_table       = akm_compass_id,
        .driver = {
                .name   = AKM_I2C_NAME,
                .pm             = &akm_compass_pm_ops,
	},
#endif 
};

#ifdef CONFIG_OF

module_i2c_driver(akm_compass_driver);

#else 

static int __init akm_compass_init(void)
{
	pr_info("AKM compass driver: initialize.");
	return i2c_add_driver(&akm_compass_driver);
}

static void __exit akm_compass_exit(void)
{
	pr_info("AKM compass driver: release.");
	i2c_del_driver(&akm_compass_driver);
}

module_init(akm_compass_init);
module_exit(akm_compass_exit);

#endif 

MODULE_DESCRIPTION("AKM compass driver");
MODULE_LICENSE("GPL");
