/*
 * arch/arm/mach-tegra/board-picasso-panel.c
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/nvmap.h>
#include <linux/nvhost.h>
#include <linux/memblock.h>
#include <linux/highmem.h>

#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/irqs.h>

#include "iomap.h"
#include "tegra2_host1x_devices.h"
#include "reserve.h"
#include "gpio-names.h"

#define TEGRA_RESET_HANDLER_SIZE	SZ_1K

#if defined(CONFIG_TEGRA_NVMAP)
#define NVMAP_HEAP_CARVEOUT_IRAM_INIT	\
	{	.name		= "iram",					\
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,			\
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,	\
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,	\
		.buddy_size	= 0, /* no buddy allocation for IRAM */		\
	}
#endif

#define PICASSO_GPIO_PNL_PWR_ENB            TEGRA_GPIO_PC6
#define PICASSO_GPIO_BACKLIGHT              TEGRA_GPIO_PD4
#define PICASSO_GPIO_LVDS_SHUTDOWN          TEGRA_GPIO_PB2
#define PICASSO_GPIO_HDMI_HPD               TEGRA_GPIO_PN7

/*panel power on sequence timing*/
#define picasso_lvds_to_bl_ms	330

static struct regulator *picasso_hdmi_reg = NULL;
static struct regulator *picasso_hdmi_pll = NULL;

static struct nvmap_platform_carveout picasso_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data picasso_nvmap_data = {
	.carveouts    = picasso_carveouts,
	.nr_carveouts = ARRAY_SIZE(picasso_carveouts),
};

int picasso_panel_enable(void)
{
	gpio_set_value(PICASSO_GPIO_PNL_PWR_ENB, 1);
	gpio_set_value(PICASSO_GPIO_LVDS_SHUTDOWN, 1);
	mdelay(picasso_lvds_to_bl_ms);
	gpio_set_value(PICASSO_GPIO_BACKLIGHT, 1);

	return 0;
}

int picasso_panel_disable(void)
{
	gpio_set_value(PICASSO_GPIO_BACKLIGHT, 0);
	gpio_set_value(PICASSO_GPIO_LVDS_SHUTDOWN, 0);
	gpio_set_value(PICASSO_GPIO_PNL_PWR_ENB, 0);

	return 0;
}

int picasso_hdmi_enable(void)
{
// 	if (IS_ERR_OR_NULL(picasso_hdmi_reg)) {
// 		picasso_hdmi_reg =
// 			regulator_get(NULL, "vdd_ldo7,avdd_hdmi,vdd_fuse");
// 
// 		if (IS_ERR_OR_NULL(picasso_hdmi_reg)) {
// 			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
// 			return PTR_ERR(picasso_hdmi_reg);
// 		}
// 	}
// 
// 	if (IS_ERR_OR_NULL(picasso_hdmi_pll)) {
// 		picasso_hdmi_pll =
// 			regulator_get(NULL, "vdd_ldo8,avdd_hdmi_pll");
// 
// 		if (IS_ERR_OR_NULL(picasso_hdmi_pll)) {
// 			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
// 			return PTR_ERR(picasso_hdmi_pll);
// 		}
// 	}

	gpio_set_value(PICASSO_GPIO_HDMI_HPD, 1);
// 	regulator_enable(picasso_hdmi_reg);
// 	regulator_enable(picasso_hdmi_pll);

	return 0;
}

int picasso_hdmi_disable(void)
{
// 	regulator_disable(picasso_hdmi_reg);
// 	regulator_disable(picasso_hdmi_pll);
	gpio_set_value(PICASSO_GPIO_HDMI_HPD, 0);

	return 0;
}

static void picasso_panel_power_on(void)
{
	gpio_request(PICASSO_GPIO_PNL_PWR_ENB, "pnl_pwr_enb");
	gpio_direction_output(PICASSO_GPIO_PNL_PWR_ENB, 1);

	gpio_request(PICASSO_GPIO_LVDS_SHUTDOWN, "lvds_shdn");
	gpio_direction_output(PICASSO_GPIO_LVDS_SHUTDOWN, 1);

	gpio_request(PICASSO_GPIO_HDMI_HPD, "hdmi_hpd");
	gpio_direction_input(PICASSO_GPIO_HDMI_HPD);
}

static struct resource picasso_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode picasso_panel_modes[] = {
	{
		.pclk          = 70500000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width  = 58,
		.v_sync_width  = 4,
		.h_back_porch  = 58,
		.v_back_porch  = 4,
		.h_active      = 1280,
		.v_active      = 800,
		.h_front_porch = 58,
		.v_front_porch = 4,
	},
};

static struct tegra_fb_data picasso_fb_data = {
	.win            = 0,
	.xres           = 1280,
	.yres           = 800,
	.bits_per_pixel = 32,
};

static struct tegra_dc_out picasso_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.depth		= 18,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes	 	= picasso_panel_modes,
	.n_modes 	= ARRAY_SIZE(picasso_panel_modes),

	.enable		= picasso_panel_enable,
	.disable	= picasso_panel_disable,
};

static struct tegra_dc_platform_data picasso_disp1_pdata = {
	.flags       = TEGRA_DC_FLAG_ENABLED,
	.default_out = &picasso_disp1_out,
	.fb          = &picasso_fb_data,
};

struct nvhost_device picasso_disp1_device = {
	.name          = "tegradc",
	.id            = 0,
	.resource      = picasso_disp1_resources,
	.num_resources = ARRAY_SIZE(picasso_disp1_resources),
	.dev = {
		.platform_data = &picasso_disp1_pdata,
	},
};

static struct resource picasso_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_out picasso_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 5,
	.hotplug_gpio	= PICASSO_GPIO_HDMI_HPD,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= picasso_hdmi_enable,
	.disable	= picasso_hdmi_disable,
};

static struct tegra_fb_data picasso_hdmi_fb_data = {
	.win            = 0,
	.xres           = 1280,
	.yres           = 720,
	.bits_per_pixel = 32,
};

static struct tegra_dc_platform_data picasso_disp2_pdata = {
	.flags       = 0,
	.default_out = &picasso_disp2_out,
	.fb          = &picasso_hdmi_fb_data,
};

struct nvhost_device picasso_disp2_device = {
	.name          = "tegradc",
	.id            = 1,
	.resource      = picasso_disp2_resources,
	.num_resources = ARRAY_SIZE(picasso_disp2_resources),
	.dev = {
		.platform_data = &picasso_disp2_pdata,
	},
};

struct platform_device tegra_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
};

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);

	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}

	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}

		for (i = 0; i < size; i += 4)
			writel(readl(from_io + i), to_io + i);

		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

void __init tegra_release_bootloader_fb(void)
{
	/* Since bootloader fb is reserved in common.c, it is freed here. */
	if (get_bootloader_fb_size())
		if (memblock_free(get_bootloader_fb_mem(),
						get_bootloader_fb_size()))
			pr_err("Failed to free bootloader fb.\n");
}

static int picasso_panel_do_init(void)
{
	int err;
	struct resource *res;

	picasso_panel_power_on();

	picasso_carveouts[1].base = get_carveout_mem();
	picasso_carveouts[1].size = CARVEOUT_MEM_SIZE;
	tegra_nvmap_device.dev.platform_data = &picasso_nvmap_data;

	err = platform_device_register(&tegra_nvmap_device);
	if (err)
		return err;

	err = tegra2_register_host1x_devices();
	if (err)
		return err;

	/*
	 * main lcd
	 */ 
	tegra_move_framebuffer(get_fb1_mem(), get_bootloader_fb_mem(),
			       min(FB1_MEM_SIZE, get_bootloader_fb_size()));

	res = nvhost_get_resource_byname(&picasso_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = get_fb1_mem();
	res->end   = get_fb1_mem() + FB1_MEM_SIZE - 1;

	err = nvhost_device_register(&picasso_disp1_device);
	if (err)
		return err;

	/*
	 * hdmi
	 */ 
// 	tegra_move_framebuffer(get_fb2_mem(), get_bootloader_fb_mem(),
// 			       min(FB2_MEM_SIZE, get_bootloader_fb_size()));
// 
// 	res = nvhost_get_resource_byname(&picasso_disp2_device,
// 					 IORESOURCE_MEM, "fbmem");
// 	res->start = get_fb2_mem();
// 	res->end   = get_fb2_mem() + FB2_MEM_SIZE - 1;
// 
// 	err = nvhost_device_register(&picasso_disp2_device);
// 	if (err)
// 		return err;

	tegra_release_bootloader_fb();

	return 0;
}

void __init picasso_panel_init(void)
{
	int err = picasso_panel_do_init();

	if (err)
		pr_err("%s failed with err: %d\n", __func__, err);
}
