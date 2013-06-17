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
#define PICASSO_BLUETOOTH_RST		TEGRA_GPIO_PU0

#define PICASSO_BCM_VDD			TEGRA_GPIO_PD1
#define PICASSO_WLAN_PWR		TEGRA_GPIO_PK6
#define PICASSO_WLAN_IRQ		TEGRA_GPIO_PS0

#endif
