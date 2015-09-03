/*
 * arch/arm/mach-tegra/board-picasso.c
 *
 * Copyright (C) 2015 Dmitry Osipenko <digetx@gmail.com>
 *
 * Based on board-paz00.c
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

#include <linux/gpio/machine.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rfkill-gpio.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

static struct rfkill_gpio_platform_data wifi_rfkill_platform_data = {
	.name	= "wifi_rfkill",
	.type	= RFKILL_TYPE_WLAN,
};

static struct platform_device wifi_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 0,
	.dev	= {
		.platform_data = &wifi_rfkill_platform_data,
	},
};

static struct gpiod_lookup_table wifi_gpio_lookup = {
	.dev_id = "rfkill_gpio.0",
	.table = {
		GPIO_LOOKUP_IDX("tegra-gpio", 86, NULL, 0, 0),
		{ },
	},
};

static struct rfkill_gpio_platform_data bluetooth_rfkill_platform_data = {
	.name	= "bluetooth_rfkill",
	.type	= RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bluetooth_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 1,
	.dev	= {
		.platform_data = &bluetooth_rfkill_platform_data,
	},
};

static struct gpiod_lookup_table bluetooth_gpio_lookup = {
	.dev_id = "rfkill_gpio.1",
	.table = {
		GPIO_LOOKUP_IDX("tegra-gpio", 161, NULL, 0, GPIO_ACTIVE_LOW),
		GPIO_LOOKUP_IDX("tegra-gpio", 160, NULL, 1, 0),
		{ },
	},
};

static void picasso_reboot_cmd(char *cmd)
{
	struct file *fp;
	mm_segment_t oldfs;

	fp = filp_open("/dev/block/mmcblk0p5", O_WRONLY | O_LARGEFILE, 0);
	if (WARN_ON(!fp))
		return;

	oldfs = get_fs();
	set_fs( get_ds() );

	pr_emerg("%s: %s\n", __func__, cmd);
	fp->f_op->write(fp, cmd, strlen(cmd), &fp->f_pos);

	set_fs(oldfs);
	filp_close(fp, 0);

	sys_sync();
}

static int picasso_reboot_event(struct notifier_block *notifier,
				unsigned long event, void *cmd)
{
	if (event == SYS_RESTART && cmd != NULL) {
		if (!strncmp(cmd, "recovery", 8))
			picasso_reboot_cmd("FOTA");
		else if (!strncmp(cmd, "bootloader", 10))
			picasso_reboot_cmd("FastbootMode");
		else if (!strncmp(cmd, "bootmenu", 8))
			picasso_reboot_cmd("BootmenuMode");
	}

	return NOTIFY_OK;
}

static struct notifier_block picasso_reboot_notifier = {
	.notifier_call = picasso_reboot_event,
	.priority = INT_MAX,
};

void __init tegra_picasso_rfkill_init(void)
{
	gpiod_add_lookup_table(&wifi_gpio_lookup);
	gpiod_add_lookup_table(&bluetooth_gpio_lookup);

// 	platform_device_register(&wifi_rfkill_device);
	platform_device_register(&bluetooth_rfkill_device);

	register_reboot_notifier(&picasso_reboot_notifier);
}
