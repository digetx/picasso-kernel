/**
 * Copyright (c) 2008 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <media/ltc3216.h>
#include <media/tegra_camera.h>

#define LTC3216_MAX_RETRIES (3)

struct ltc3216_info {
	struct i2c_client *i2c_client;
	int torch_on;
};

struct ltc3216_info *info_flash = NULL;

static int ltc3216_write(struct i2c_client *client, u16 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (value >> 8);
	data[1] = (u8) (value & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(data);
	msg[0].buf = data;

	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		dev_err(&client->dev, "i2c transfer failed, retrying %x\n",
			value);
		msleep(3);
	} while (retry <= LTC3216_MAX_RETRIES);

	return -EIO;
}


static long ltc3216_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ltc3216_info *info_flash = file->private_data;

	switch (cmd) {
	case LTC3216_IOCTL_FLASH_ON:
		ltc3216_write(info_flash->i2c_client, 0x90);
		ltc3216_write(info_flash->i2c_client, (u16) arg);
		break;

	case LTC3216_IOCTL_FLASH_OFF:
		ltc3216_write(info_flash->i2c_client, 0x00);
		ltc3216_write(info_flash->i2c_client, 0x104);
		break;

	case LTC3216_IOCTL_TORCH_ON:
		info_flash->torch_on = true;
		ltc3216_write(info_flash->i2c_client, (u16) arg);
		break;

	case LTC3216_IOCTL_TORCH_OFF:
		info_flash->torch_on = false;
		ltc3216_write(info_flash->i2c_client, 0x00);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int ltc3216_open(struct inode *inode, struct file *file)
{
	file->private_data = info_flash;
	return 0;
}

int ltc3216_release(struct inode *inode, struct file *file)
{
	if (info_flash->torch_on)
		ltc3216_write(info_flash->i2c_client, 0x00);
	file->private_data = NULL;
	return 0;
}


static const struct file_operations ltc3216_fileops = {
	.owner = THIS_MODULE,
	.open = ltc3216_open,
	.unlocked_ioctl = ltc3216_ioctl,
	.release = ltc3216_release,
};

static struct miscdevice ltc3216_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltc3216",
	.fops = &ltc3216_fileops,
};

void ltc3216_turn_on_flash(void)
{
	if (!info_flash)
		return;

	// set STIM to 11111b, Timer = STIM x 32.8ms = 1.02s
	ltc3216_write(info_flash->i2c_client, 0x03DF);

	ltc3216_write(info_flash->i2c_client, 0x0090);
	ltc3216_write(info_flash->i2c_client, 0x018E);
}

void ltc3216_turn_off_flash(void)
{
	if (!info_flash)
		return;

	ltc3216_write(info_flash->i2c_client, 0x0000);
	ltc3216_write(info_flash->i2c_client, 0x0104);
}

void ltc3216_turn_on_torch(void)
{
	if (!info_flash)
		return;

	ltc3216_write(info_flash->i2c_client, 0x0053);
}

void ltc3216_turn_off_torch(void)
{
	if (!info_flash)
		return;

	ltc3216_write(info_flash->i2c_client, 0x0000);
}

static ssize_t torch_read(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", !!info_flash->torch_on);
}

static ssize_t torch_write(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	if (sscanf(buf, "%d\n", &info_flash->torch_on) > 0) {
		dev_dbg(dev, "%s %s\n",
			__func__, info_flash->torch_on ? "on" : "off");

		if (info_flash->torch_on)
			ltc3216_turn_on_torch();
		else
			ltc3216_turn_off_torch();
	} else
		dev_err(dev, "%s: input error\n", __func__);

	return size;
}

static DEVICE_ATTR(brightness, 0664, torch_read, torch_write);

static struct attribute *torch_attributes[] = {
	&dev_attr_brightness.attr,
	NULL,
};

static struct attribute_group torch_group = {
	.attrs  = torch_attributes,
};

static int ltc3216_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	info_flash = devm_kzalloc(&client->dev, sizeof(*info_flash), GFP_KERNEL);
	if (!info_flash) {
		dev_err(&client->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}
	info_flash->i2c_client = client;

	err = misc_register(&ltc3216_device);
	if (err) {
		dev_err(&client->dev, "unable to register misc device\n");
		return err;
	}

	err = sysfs_create_group(&ltc3216_device.this_device->kobj,
				 &torch_group);
	if (err) {
		dev_err(&client->dev, "sysfs group create failed\n");
		misc_deregister(&ltc3216_device);
		return err;
	}

	return 0;
}

static int ltc3216_remove(struct i2c_client *client)
{
	sysfs_remove_group(&ltc3216_device.this_device->kobj, &torch_group);
	misc_deregister(&ltc3216_device);
	return 0;
}

static const struct i2c_device_id ltc3216_id[] = {
	{ "ltc3216", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ltc3216_id);

static struct of_device_id ltc3216_of_match[] = {
	{ .compatible = "nvidia,ltc3216", },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc3216_of_match);

static struct i2c_driver ltc3216_i2c_driver = {
	.driver = {
		.name = "ltc3216",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ltc3216_of_match),
	},
	.probe = ltc3216_probe,
	.remove = ltc3216_remove,
	.id_table = ltc3216_id,
};
module_i2c_driver(ltc3216_i2c_driver);
