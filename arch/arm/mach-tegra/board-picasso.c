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

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/tegra_uart.h>
#include <linux/err.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/platform_data/mmc-sdhci-tegra.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>

#include "board-picasso.h"
#include "clock.h"
#include "pm.h"

/******************************************************************************
* Wifi
*****************************************************************************/
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

static int picasso_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int picasso_wifi_power(int on);
static int picasso_wifi_set_carddetect(int val);
static void *picasso_wifi_mem_prealloc(int section, unsigned long size);

struct wifi_platform_data picasso_wifi_control = {
	.set_power      = picasso_wifi_power,
	.set_carddetect = picasso_wifi_set_carddetect,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc   = picasso_wifi_mem_prealloc,
#else
	.mem_prealloc   = NULL,
#endif
};

int picasso_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int picasso_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	picasso_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static unsigned int picasso_wifi_status(struct device *dev)
{
	return picasso_wifi_cd;
}

static int picasso_wifi_power(int on)
{
	static bool wifi_gpios_requested = false;
	pr_debug("%s: %d\n", __func__, on);

	if (!wifi_gpios_requested) {
		gpio_request(PICASSO_WLAN_IRQ,"oob irq");
		gpio_direction_input(PICASSO_WLAN_IRQ);

		gpio_request(PICASSO_WLAN_RST, "wlan_rst");
		gpio_direction_output(PICASSO_WLAN_RST, 0);

		wifi_gpios_requested = true;
	}

	msleep(100);
	change_power_brcm_4329(on);
	msleep(300);
	gpio_set_value(PICASSO_WLAN_RST, on);
	msleep(200);

	return 0;
}

static __maybe_unused void *picasso_wifi_mem_prealloc(int section,
						      unsigned long size)
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
#ifdef CONFIG_DHD_USE_STATIC_BUF
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
#endif
}

static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
	.cis  = {
		.vendor	= 0x02d0,
		.device	= 0x4329,
	},
};

struct tegra_sdhci_platform_data picasso_wlan_sdhci_pdata = {
	.mmc_data = {
		.register_status_notify	= picasso_wifi_status_register,
		.embedded_sdio		= &embedded_sdio_data0,
		.status			= picasso_wifi_status,
		.built_in		= 0,
		.card_present		= 0,
	},
	.cd_gpio    = -1,
	.wp_gpio    = -1,
	.power_gpio = -1,
	.is_8bit    = 0,
};

static void __init picasso_wifi_init(void)
{
	picasso_init_wifi_mem();
}

/******************************************************************************
* Uart
*****************************************************************************/
extern struct tegra_uart_platform_data tegra_uartb_pdata;
extern struct tegra_uart_platform_data tegra_uartc_pdata;
extern struct tegra_uart_platform_data tegra_uartd_pdata;

/*
 * dynamic selection of slowest parent clk suitable
 * for requested baud rate reduces power consumption
 */
static struct uart_clk_parent uart_parent_clk[] = {
	{ .name = "pll_p" },
	{ .name = "pll_m" },
	{ .name = "clk_m" },
};

static void __init picasso_uart_init(void)
{
	struct clk *c;
	int i, cnt = 0;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); i++) {
		c = clk_get(NULL, uart_parent_clk[i].name);
		if (IS_ERR(c)) {
			pr_err("Unable to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[cnt].parent_clk = c;
		uart_parent_clk[cnt].fixed_clk_rate = clk_get_rate(c);
		cnt++;
	}

	/* GPS */
	tegra_uartb_pdata.parent_clk_list = uart_parent_clk;
	tegra_uartb_pdata.parent_clk_count = cnt;

	/* Bluetooth */
	tegra_uartc_pdata.parent_clk_list = uart_parent_clk;
	tegra_uartc_pdata.parent_clk_count = cnt;
	tegra_uartc_pdata.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked;
	tegra_uartc_pdata.rx_done_cb = bcm_bt_rx_done_locked;

	/* Dock */
	tegra_uartd_pdata.parent_clk_list = uart_parent_clk;
	tegra_uartd_pdata.parent_clk_count = cnt;
}

static DEFINE_SPINLOCK(brcm_4329_enable_lock);
static int brcm_4329_enable_count;

void change_power_brcm_4329(bool enable)
{
	static bool bcm_vdd_gpio_requested = false;
	unsigned long flags;

	if (!bcm_vdd_gpio_requested) {
		gpio_request(PICASSO_BCM_VDD, "bcm_vdd");
		gpio_direction_output(PICASSO_BCM_VDD, 0);

		bcm_vdd_gpio_requested = true;
	}

	spin_lock_irqsave(&brcm_4329_enable_lock, flags);
	if (enable) {
		gpio_set_value(PICASSO_BCM_VDD, enable);
		brcm_4329_enable_count++;
		// The following shouldn't happen but protect
		// if the user doesn't cleanup.
		if (brcm_4329_enable_count > 2)
			brcm_4329_enable_count = 2;
	} else {
		if (brcm_4329_enable_count > 0)
			brcm_4329_enable_count--;
		if (!brcm_4329_enable_count)
			gpio_set_value(PICASSO_BCM_VDD, enable);
	}
	spin_unlock_irqrestore(&brcm_4329_enable_lock, flags);
}

/******************************************************************************
* Reboot cmd
*****************************************************************************/
static void picasso_reboot_cmd(char *cmd)
{
	struct file *fp;
	mm_segment_t oldfs;

	fp = filp_open("/dev/block/mmcblk0p5", O_RDWR | O_LARGEFILE, 0);
	if (!fp)
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
	}

	return NOTIFY_OK;
}

static struct notifier_block picasso_reboot_notifier = {
	.notifier_call = picasso_reboot_event,
	.priority = INT_MAX,
};

static __initdata struct tegra_clk_init_table tegra_picasso_clk_init_table[] = {
	{ "pwm",	"clk_m",	12000000,	false },
	{ "mpe",	"pll_c",	0,		false },
	{ "epp",	"pll_c",	0,		false },
	{ "vi_sensor",	"pll_c",	0,		false },
	{ "vi",		"pll_c",	0,		false },
	{ "2d",		"pll_c",	0,		false },
	{ "3d",		"pll_c",	0,		false },
	{ NULL,		NULL,		0,		0 },
};

static struct suspend_params picasso_sparams = {
	.core_timer = 0x7e7e,
	.core_off_timer = 0xf,
	.corereq_high = false,
	.sysclkreq_high = true,
	.combined_req = false,
};

void __init picasso_machine_init(void)
{
	tegra_clk_init_from_table(tegra_picasso_clk_init_table);

	picasso_uart_init();
	picasso_wifi_init();

	tegra_init_suspend(&picasso_sparams);
	register_reboot_notifier(&picasso_reboot_notifier);
}
