/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#ifndef __TEGRA_FUSE_H
#define __TEGRA_FUSE_H

#include <mach/fuse.h>

unsigned long long tegra_chip_uid(void);
void tegra_init_fuse(void);
bool tegra_spare_fuse(int bit);
u32 tegra_fuse_readl(unsigned long offset);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void tegra20_init_speedo_data(void);
#else
static inline void tegra20_init_speedo_data(void) {}
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
void tegra30_init_speedo_data(void);
#else
static inline void tegra30_init_speedo_data(void) {}
#endif

#endif
