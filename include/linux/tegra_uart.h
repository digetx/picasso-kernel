/* include/linux/tegra_uart.h
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _TEGRA_UART_H_
#define _TEGRA_UART_H_

#include <linux/serial_core.h>
#include <linux/clk.h>

struct uart_clk_parent {
	struct clk	*parent_clk;
	const char	*name;
	unsigned long	fixed_clk_rate;
};

struct tegra_uart_platform_data {
	struct uart_clk_parent *parent_clk_list;
	int parent_clk_count;
	int dma_req_sel;
	int line;
	void (*exit_lpm_cb)(struct uart_port *);
	void (*rx_done_cb)(struct uart_port *);
};

#endif /* _TEGRA_UART_H_ */

