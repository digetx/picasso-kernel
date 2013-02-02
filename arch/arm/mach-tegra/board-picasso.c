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

#include "board-picasso.h"
#include "clock.h"

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
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = clk_get(NULL, uart_parent_clk[i].name);
		if (IS_ERR(c)) {
			pr_err("Unable to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}

	/* GPS */
	tegra_uartb_pdata.parent_clk_list = uart_parent_clk;
	tegra_uartb_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	/* Bluetooth */
	tegra_uartc_pdata.parent_clk_list = uart_parent_clk;
	tegra_uartc_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uartc_pdata.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked;
	tegra_uartc_pdata.rx_done_cb = bcm_bt_rx_done_locked;

	/* Dock */
	tegra_uartd_pdata.parent_clk_list = uart_parent_clk;
	tegra_uartd_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
}

static DEFINE_SPINLOCK(brcm_4329_enable_lock);
static int brcm_4329_enable_count;
static bool brcm_4329_inited;

void change_power_brcm_4329(bool enable)
{
	unsigned long flags;

	if (!brcm_4329_inited) {
		gpio_request(PICASSO_BCM_VDD, "bcm_vdd");
		gpio_direction_output(PICASSO_BCM_VDD, 0);
		brcm_4329_inited = true;
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

static __initdata struct tegra_clk_init_table tegra_picasso_clk_init_table[] = {
	{ "pwm",	"clk_m",	12000000,	false },
	{ NULL,		NULL,		0,		0},
};

void __init picasso_machine_init(void)
{
	tegra_clk_init_from_table(tegra_picasso_clk_init_table);

	picasso_uart_init();
}
