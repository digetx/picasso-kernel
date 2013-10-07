/*
 * drivers/media/platform/tegra/tegra_camera.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <media/tegra_camera.h>

#include <mach/clk.h>
#include <mach/powergate.h>

/* Eventually this should handle all clock and reset calls for the isp, vi,
 * vi_sensor, and csi modules, replacing nvrm and nvos completely for camera
 */
#define TEGRA_CAMERA_NAME "tegra_camera"

#define TEGRA_CLK_RESET_BASE		0x60006000
#define TEGRA_CLK_RESET_SIZE		SZ_4K

#define TEGRA_APB_MISC_BASE		0x70000000
#define TEGRA_APB_MISC_SIZE		SZ_4K

struct tegra_camera_dev {
	struct device *dev;
	struct miscdevice misc_dev;
	struct clk *isp_clk;
	struct clk *vi_clk;
	struct clk *vi_sensor_clk;
	struct clk *csus_clk;
	struct clk *csi_clk;
	struct regulator *reg;
	struct tegra_camera_clk_info info;
	struct mutex tegra_camera_lock;
	int camera_power_gpio;
	void __iomem *car;
	void __iomem *apb_misc;
	int power_refcnt;
} *tegra_camera = NULL;

struct tegra_camera_block {
	int (*enable) (struct tegra_camera_dev *dev);
	int (*disable) (struct tegra_camera_dev *dev);
	bool is_enabled;
};

static int tegra_camera_enable_isp(struct tegra_camera_dev *dev)
{
	return clk_prepare_enable(dev->isp_clk);
}

static int tegra_camera_disable_isp(struct tegra_camera_dev *dev)
{
	clk_disable_unprepare(dev->isp_clk);
	return 0;
}

static int tegra_camera_enable_vi(struct tegra_camera_dev *dev)
{
	int ret = 0;

	ret |= clk_prepare_enable(dev->vi_clk);
	ret |= clk_prepare_enable(dev->vi_sensor_clk);
	ret |= clk_prepare_enable(dev->csus_clk);
	return ret;
}

static int tegra_camera_disable_vi(struct tegra_camera_dev *dev)
{
	clk_disable_unprepare(dev->vi_clk);
	clk_disable_unprepare(dev->vi_sensor_clk);
	clk_disable_unprepare(dev->csus_clk);
	return 0;
}

static int tegra_camera_enable_csi(struct tegra_camera_dev *dev)
{
	return clk_prepare_enable(dev->csi_clk);
}

static int tegra_camera_disable_csi(struct tegra_camera_dev *dev)
{
	clk_disable_unprepare(dev->csi_clk);
	return 0;
}

struct tegra_camera_block tegra_camera_block[] = {
	[TEGRA_CAMERA_MODULE_ISP] = {tegra_camera_enable_isp,
		tegra_camera_disable_isp, false},
	[TEGRA_CAMERA_MODULE_VI] = {tegra_camera_enable_vi,
		tegra_camera_disable_vi, false},
	[TEGRA_CAMERA_MODULE_CSI] = {tegra_camera_enable_csi,
		tegra_camera_disable_csi, false},
};

#define TEGRA_CAMERA_VI_CLK_SEL_INTERNAL 0
#define TEGRA_CAMERA_VI_CLK_SEL_EXTERNAL (1<<24)
#define TEGRA_CAMERA_PD2VI_CLK_SEL_VI_SENSOR_CLK (1<<25)
#define TEGRA_CAMERA_PD2VI_CLK_SEL_PD2VI_CLK 0

static bool tegra_camera_enabled(struct tegra_camera_dev *dev)
{
	bool ret = false;

	mutex_lock(&dev->tegra_camera_lock);
	ret = tegra_camera_block[TEGRA_CAMERA_MODULE_ISP].is_enabled == true ||
			tegra_camera_block[TEGRA_CAMERA_MODULE_VI].is_enabled == true ||
			tegra_camera_block[TEGRA_CAMERA_MODULE_CSI].is_enabled == true;
	mutex_unlock(&dev->tegra_camera_lock);
	return ret;
}

int tegra_camera_clk_set_rate(struct tegra_camera_clk_info *info)
{
	struct tegra_camera_dev *dev = tegra_camera;
	struct clk *clk;
	u32 offset;

	if (!dev)
		return -EINVAL;

	if (!info) {
		dev_err(dev->dev,
				"%s: no clock info\n", __func__);
		return -EINVAL;
	}

	if (info->id != TEGRA_CAMERA_MODULE_VI) {
		dev_err(dev->dev,
				"%s: set rate only aplies to vi module %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	switch (info->clk_id) {
	case TEGRA_CAMERA_VI_CLK:
		clk = dev->vi_clk;
		offset = 0x148;
		break;
	case TEGRA_CAMERA_VI_SENSOR_CLK:
		clk = dev->vi_sensor_clk;
		offset = 0x1a8;
		break;
	default:
		dev_err(dev->dev,
				"%s: invalid clk id for set rate %d\n",
				__func__, info->clk_id);
		return -EINVAL;
	}

	clk_set_rate(clk, info->rate);

	if (info->clk_id == TEGRA_CAMERA_VI_CLK) {
		u32 val = 0x2;

		if (info->flag == TEGRA_CAMERA_ENABLE_PD2VI_CLK) {
			val |= TEGRA_CAMERA_PD2VI_CLK_SEL_VI_SENSOR_CLK;
		}

		writel(val, dev->car + offset);

		val = readl(dev->apb_misc + 0x42c);
		writel(val | 0x1, dev->apb_misc + 0x42c);
	}

	info->rate = clk_get_rate(clk);

	return 0;
}

bool tegra_camera_probed(void)
{
	return !!tegra_camera;
}

void tegra_camera_gpio_set(bool enable)
{
	if (tegra_camera)
		gpio_direction_output(tegra_camera->camera_power_gpio, enable);
}

void tegra_camera_clk_enable(void)
{
	if (tegra_camera) {
		/*
		 * When booting device, camera needs to turn on vcsi power once.
		 * After turning on vcsi power, it always remains on with camera
		 * standby mode. So camera don't need to turn on/off vcsi power
		 * via tegra_camera_power_on() and tegra_camera_power_off().
		 */
		if (tegra_camera->reg) {
			if (regulator_enable(tegra_camera->reg)) {
				dev_err(tegra_camera->dev,
				"%s: enable csi regulator failed.\n",
				__func__);
			}
		}

		tegra_camera_enable_csi(tegra_camera);
		tegra_camera_enable_isp(tegra_camera);
		tegra_camera_enable_vi(tegra_camera);
	}
}

void tegra_camera_clk_disable(void)
{
	if (tegra_camera) {
		tegra_camera_disable_vi(tegra_camera);
		tegra_camera_disable_isp(tegra_camera);
		tegra_camera_disable_csi(tegra_camera);
	}
}

static int tegra_camera_reset(struct tegra_camera_dev *dev, uint id)
{
	struct clk *clk;

	switch (id) {
	case TEGRA_CAMERA_MODULE_VI:
		clk = dev->vi_clk;
		break;
	case TEGRA_CAMERA_MODULE_ISP:
		clk = dev->isp_clk;
		break;
	case TEGRA_CAMERA_MODULE_CSI:
		clk = dev->csi_clk;
		break;
	default:
		return -EINVAL;
	}
	tegra_periph_reset_assert(clk);
	udelay(10);
	tegra_periph_reset_deassert(clk);

	return 0;
}

static int tegra_camera_power_on(struct tegra_camera_dev *dev)
{
	int ret = 0;

	if (dev->power_refcnt++ == 0) {
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		/* Unpowergate VE */
		ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
		if (ret)
			dev_err(dev->dev,
				"%s: unpowergate failed.\n",
				__func__);
#endif
	}

	return ret;
}

static int tegra_camera_power_off(struct tegra_camera_dev *dev)
{
	int ret = 0;

	if (--dev->power_refcnt == 0) {
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		/* Powergate VE */
		ret = tegra_powergate_partition(TEGRA_POWERGATE_VENC);
		if (ret)
			dev_err(dev->dev,
				"%s: powergate failed.\n",
				__func__);
#endif
	}
	return ret;
}

static long tegra_camera_ioctl(struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	uint id;
	struct tegra_camera_dev *dev = file->private_data;

	/* first element of arg must be u32 with id of module to talk to */
	if (copy_from_user(&id, (const void __user *)arg, sizeof(uint))) {
		dev_err(dev->dev,
				"%s: Failed to copy arg from user", __func__);
		return -EFAULT;
	}

	if (id >= ARRAY_SIZE(tegra_camera_block)) {
		dev_err(dev->dev,
				"%s: Invalid id to tegra isp ioctl%d\n",
				__func__, id);
		return -EINVAL;
	}

	switch (cmd) {
	case TEGRA_CAMERA_IOCTL_ENABLE:
	{
		int ret = 0;

		mutex_lock(&dev->tegra_camera_lock);
		/* Unpowergate camera blocks (vi, csi and isp)
		   before enabling clocks */
		ret = tegra_camera_power_on(dev);
		if (ret) {
			dev->power_refcnt = 0;
			mutex_unlock(&dev->tegra_camera_lock);
			return ret;
		}

		if (!tegra_camera_block[id].is_enabled) {
			ret = tegra_camera_block[id].enable(dev);
			tegra_camera_block[id].is_enabled = true;
		}
		mutex_unlock(&dev->tegra_camera_lock);
		return ret;
	}
	case TEGRA_CAMERA_IOCTL_DISABLE:
	{
		int ret = 0;

		mutex_lock(&dev->tegra_camera_lock);
		if (tegra_camera_block[id].is_enabled) {
			ret = tegra_camera_block[id].disable(dev);
			tegra_camera_block[id].is_enabled = false;
		}
		/* Powergate camera blocks (vi, csi and isp)
		   after disabling all the clocks */
		if (!ret) {
			ret = tegra_camera_power_off(dev);
		}
		mutex_unlock(&dev->tegra_camera_lock);
		return ret;
	}
	case TEGRA_CAMERA_IOCTL_CLK_SET_RATE:
	{
		int ret;

		if (copy_from_user(&dev->info, (const void __user *)arg,
				   sizeof(struct tegra_camera_clk_info))) {
			dev_err(dev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}
		ret = tegra_camera_clk_set_rate(&dev->info);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &dev->info,
				 sizeof(struct tegra_camera_clk_info))) {
			dev_err(dev->dev,
				"%s: Failed to copy arg to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case TEGRA_CAMERA_IOCTL_RESET:
		return tegra_camera_reset(dev, id);
	default:
		dev_err(dev->dev,
				"%s: Unknown tegra_camera ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tegra_camera_dev *dev = container_of(miscdev,
						struct tegra_camera_dev,
						misc_dev);
	dev_info(dev->dev, "%s\n", __func__);
	file->private_data = dev;

	return 0;
}

static int tegra_camera_release(struct inode *inode, struct file *file)
{
	struct tegra_camera_dev *dev = file->private_data;
	int i;

	dev_info(dev->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(tegra_camera_block); i++)
		if (tegra_camera_block[i].is_enabled) {
			tegra_camera_block[i].disable(dev);
			tegra_camera_block[i].is_enabled = false;
		}

	/* If camera blocks are not powergated yet, do it now */
	if (dev->power_refcnt > 0) {
		mutex_lock(&dev->tegra_camera_lock);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		if (tegra_powergate_partition(TEGRA_POWERGATE_VENC))
			dev_err(dev->dev, "%s: powergate failed.\n", __func__);
#endif
		dev->power_refcnt = 0;
		mutex_unlock(&dev->tegra_camera_lock);
	}

	return 0;
}

static const struct file_operations tegra_camera_fops = {
	.owner = THIS_MODULE,
	.open = tegra_camera_open,
	.unlocked_ioctl = tegra_camera_ioctl,
	.release = tegra_camera_release,
};

static int tegra_camera_clk_get(struct platform_device *pdev, const char *name,
				struct clk **clk)
{
	*clk = devm_clk_get(&pdev->dev, name);
	if (IS_ERR(*clk)) {
		dev_err(&pdev->dev, "%s: unable to get clock for %s\n",
			__func__, name);
		return -ENOMEM;
	}
	return 0;
}

static int tegra_camera_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct tegra_camera_dev *dev;
	int err;

	dev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_camera_dev),
			GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "%s: unable to allocate memory\n",
			__func__);
		return -ENOMEM;
	}

	mutex_init(&dev->tegra_camera_lock);

	/* Powergate VE when boot */
	mutex_lock(&dev->tegra_camera_lock);
	dev->power_refcnt = 0;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	if (err)
		dev_err(&pdev->dev, "%s: powergate failed.\n", __func__);
#endif
	mutex_unlock(&dev->tegra_camera_lock);

	dev->dev = &pdev->dev;

	/* Get regulator pointer */
	dev->reg = devm_regulator_get(&pdev->dev, "vcsi");

	if (IS_ERR(dev->reg)) {
		dev_err(&pdev->dev, "%s: couldn't get regulator\n", __func__);
		return PTR_ERR(dev->reg);
	}

	dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	dev->misc_dev.name = TEGRA_CAMERA_NAME;
	dev->misc_dev.fops = &tegra_camera_fops;
	dev->misc_dev.parent = &pdev->dev;

	err = tegra_camera_clk_get(pdev, "isp", &dev->isp_clk);
	if (err)
		return err;
	err = tegra_camera_clk_get(pdev, "vi", &dev->vi_clk);
	if (err)
		return err;
	err = tegra_camera_clk_get(pdev, "vi_sensor", &dev->vi_sensor_clk);
	if (err)
		return err;
	err = tegra_camera_clk_get(pdev, "csus", &dev->csus_clk);
	if (err)
		return err;
	err = tegra_camera_clk_get(pdev, "csi", &dev->csi_clk);
	if (err)
		return err;

	dev->car = devm_ioremap(&pdev->dev, TEGRA_CLK_RESET_BASE,
				TEGRA_CLK_RESET_SIZE);
	if (!dev->car) {
		dev_err(&pdev->dev, "%s: unable to remap car\n", __func__);
		return -ENOMEM;
	}

	dev->apb_misc = devm_ioremap(&pdev->dev, TEGRA_APB_MISC_BASE,
				     TEGRA_APB_MISC_SIZE);
	if (!dev->apb_misc) {
		dev_err(&pdev->dev, "%s: unable to remap apb_misc\n", __func__);
		return -ENOMEM;
	}

	dev->camera_power_gpio = of_get_named_gpio(np, "power-gpio", 0);
	if (dev->camera_power_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (gpio_is_valid(dev->camera_power_gpio)) {
		err = devm_gpio_request_one(&pdev->dev, dev->camera_power_gpio,
					    GPIOF_OUT_INIT_LOW, "cam_pwr_en");
		if (err) {
			dev_err(&pdev->dev, "cannot get camera_power gpio\n");
			return err;
		}
	} else
		return -ENODEV;

	err = misc_register(&dev->misc_dev);
	if (err) {
		dev_err(&pdev->dev,
			"%s: unable to register misc device\n", __func__);
		return err;
	}

	/* dev is set in order to restore in _remove */
	platform_set_drvdata(pdev, dev);

	tegra_camera = dev;

	return 0;
}

static int tegra_camera_remove(struct platform_device *pdev)
{
	struct tegra_camera_dev *dev = platform_get_drvdata(pdev);

	misc_deregister(&dev->misc_dev);
	mutex_destroy(&dev->tegra_camera_lock);

	tegra_camera = NULL;

	return 0;
}

static int tegra_camera_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_camera_dev *dev = platform_get_drvdata(pdev);

	if (tegra_camera_enabled(dev)) {
		dev_err(&pdev->dev, "tegra_camera cannot suspend, " \
					"application is holding camera on\n");
		return -EBUSY;
	}

	return 0;
}

static int tegra_camera_resume(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id tegra_camera_of_match[] = {
	{ .compatible = "nvidia,tegra20-camera", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_camera_of_match);

static struct platform_driver tegra_camera_driver = {
	.probe = tegra_camera_probe,
	.remove = tegra_camera_remove,
	.suspend = tegra_camera_suspend,
	.resume = tegra_camera_resume,
	.driver = {
		.name = TEGRA_CAMERA_NAME,
		.of_match_table = of_match_ptr(tegra_camera_of_match),
	}
};
module_platform_driver(tegra_camera_driver);
