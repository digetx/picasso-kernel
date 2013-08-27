/*
 * arch/arm/mach-tegra/board-picasso.h
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

#ifndef _MACH_TEGRA_BOARD_PICASSO_H
#define _MACH_TEGRA_BOARD_PICASSO_H

#include "gpio-names.h"

#define PICASSO_BCM_VDD			TEGRA_GPIO_PD1
#define PICASSO_WLAN_PWR		TEGRA_GPIO_PK5
#define PICASSO_WLAN_RST		TEGRA_GPIO_PK6

void change_power_brcm_4329(bool enable);

/* Uart driver must call this every time it beings TX, to ensure
 * this driver keeps WAKE asserted during TX. Called with uart
 * spinlock held. */
void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport);

/* Uart driver must call this when the rx is done.*/
void bcm_bt_rx_done_locked(struct uart_port *uport);

#endif