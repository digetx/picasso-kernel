/*
 * arch/arm/mach-tegra/include/mach/memory.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_MEMORY_H
#define __MACH_TEGRA_MEMORY_H

/* physical offset of RAM */
#define PLAT_PHYS_OFFSET		UL(0)

/*
 * Unaligned DMA causes tegra dma to place data on 4-byte boundary after
 * expected address. Call to skb_reserve(skb, NET_IP_ALIGN) was causing skb
 * buffers in usbnet.c to become unaligned.
 */
#define NET_IP_ALIGN	0

#endif

