/*
 * arch/arm/mach-tegra/board-picasso.c
 *
 * Author:
 *	Dmitry Osipenko <digetx@gmail.com>
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

#include <linux/platform_device.h>
#include <linux/rfkill-gpio.h>
#include <linux/gpio.h>
#include "board-picasso.h"

static struct rfkill_gpio_platform_data bluetooth_rfkill_platform_data = {
	.name = "bluetooth_rfkill",
	.reset_gpio = PICASSO_BLUETOOTH_RST,
	.shutdown_gpio = PICASSO_BCM_VDD,
	.type = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bluetooth_rfkill_device = {
	.name = "rfkill_gpio",
	.id = PLATFORM_DEVID_NONE,
	.dev = {
		.platform_data = &bluetooth_rfkill_platform_data,
	},
};

void __init tegra_picasso_bluetooth_rfkill_init(void)
{
	gpio_export(PICASSO_BLUETOOTH_RST, "PICASSO_BLUETOOTH_RST");
	gpio_export(PICASSO_BCM_VDD, "PICASSO_BCM_VDD");

	platform_device_register(&bluetooth_rfkill_device);
}
