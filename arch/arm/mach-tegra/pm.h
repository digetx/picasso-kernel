/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2012 NVIDIA Corporation. All rights reserved.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MACH_TEGRA_PM_H_
#define _MACH_TEGRA_PM_H_

extern unsigned long l2x0_saved_regs_addr;

void save_cpu_arch_register(void);
void restore_cpu_arch_register(void);

void tegra_clear_cpu_in_lp2(int phy_cpu_id);
bool tegra_set_cpu_in_lp2(int phy_cpu_id);

void tegra_idle_lp2_last(u32 cpu_on_time, u32 cpu_off_time);
extern void (*tegra_tear_down_cpu)(void);

#ifdef CONFIG_PM_SLEEP
struct suspend_params {
	unsigned long core_timer;	/* core power good time in ticks, LP0 */
	unsigned long core_off_timer;	/* core power off time ticks, LP0 */
	bool corereq_high;	/* Core power request active-high */
	bool sysclkreq_high;	/* System clock request is active-high */
	bool combined_req;	/* if core & CPU power requests are combined */
};

void tegra_init_suspend(struct suspend_params *sparams);
void tegra2_lp0_suspend_init(void);
void tegra_timer_suspend(void);
void tegra_timer_resume(void);
void pmc_32kwritel(u32 val, unsigned long offs);
void tegra_cpu_reset_handler_save(void);
void tegra_cpu_reset_handler_restore(void);
#else
#define tegra_init_suspend() do {} while(0)
#endif

#endif /* _MACH_TEGRA_PM_H_ */
