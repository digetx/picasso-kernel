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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/rfkill-gpio.h>
#include <linux/gpio.h>
#include <linux/wlan_plat.h>

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

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static DEFINE_SPINLOCK(brcm_4329_enable_lock);
static int brcm_4329_enable_count;

void change_power_brcm_4329(bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&brcm_4329_enable_lock, flags);
	if (enable) {
		gpio_direction_output(PICASSO_BCM_VDD, enable);
		brcm_4329_enable_count++;
		// The following shouldn't happen but protect
		// if the user doesn't cleanup.
		if (brcm_4329_enable_count > 2)
			brcm_4329_enable_count = 2;
	} else {
		if (brcm_4329_enable_count > 0)
			brcm_4329_enable_count--;
		if (!brcm_4329_enable_count)
			gpio_direction_output(PICASSO_BCM_VDD, enable);
	}
	spin_unlock_irqrestore(&brcm_4329_enable_lock, flags);
}

static int picasso_wifi_power(int on)
{
	pr_debug("%s: %d\n", __func__, on);

	change_power_brcm_4329(on);
	msleep(300);
	gpio_direction_output(PICASSO_WLAN_PWR, on);
	msleep(200);

	return 0;
}

static void *picasso_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

static void __init picasso_init_wifi_mem(void)
{
	int i;

	for (i = 0; i < WLAN_SKB_BUF_NUM; i++) {
		if (i < (WLAN_SKB_BUF_NUM / 2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}

	for (i = 0; i < PREALLOC_WLAN_NUMBER_OF_SECTIONS; i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL) {
			pr_err("%s: static buffer allocation failed\n",
			       __func__);
			return;
		}
	}
}

static struct wifi_platform_data picasso_wifi_control = {
	.set_power = picasso_wifi_power,
	.mem_prealloc = picasso_wifi_mem_prealloc,
};

static struct resource wifi_irq = {
	.name = "bcmdhd_wlan_irq",
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
};

static struct platform_device wifi_device = {
	.name = "bcmdhd_wlan",
	.id = PLATFORM_DEVID_NONE,
	.num_resources = 1,
	.resource = &wifi_irq,
	.dev.platform_data = &picasso_wifi_control,
};

void __init tegra_picasso_wifi_init(void)
{
	gpio_request(PICASSO_BCM_VDD, "bcm_vdd");
	gpio_request(PICASSO_WLAN_PWR, "wlan_pwr");
	gpio_request(PICASSO_WLAN_IRQ, "oob_irq");
	gpio_direction_input(PICASSO_WLAN_IRQ);

	wifi_irq.start = gpio_to_irq(PICASSO_WLAN_IRQ);

	if (IS_ENABLED(CONFIG_DHD_USE_STATIC_BUF))
		picasso_init_wifi_mem();

	platform_device_register(&wifi_device);
}
