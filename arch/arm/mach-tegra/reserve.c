#include <linux/of.h>
#include <linux/memblock.h>

#include "reserve.h"

#define RAM_CONSOLE_SIZE	SZ_1M

static phys_addr_t ram_console_mem;

static const struct of_device_id ram_console_matches[] __initconst = {
	{ .compatible = "android,ram-console" },
	{ }
};

static void __init reserve_ram_console_mem(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#ifndef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
	ram_console_mem = memblock_end_of_DRAM() - RAM_CONSOLE_SIZE;

	if (memblock_remove(ram_console_mem, RAM_CONSOLE_SIZE))
#endif
#endif
		ram_console_mem = 0;
}

/*
 * Assign reserved mem to ram console resource
 */
static void __init ram_console_dt_resource_init(void)
{
	if (ram_console_mem && of_have_populated_dt()) {
		struct device_node *np;

		np = of_find_matching_node(NULL, ram_console_matches);
		if (np) {
			struct property *prop;

			prop = of_find_property(np, "reg", NULL);
			if (prop) {
				__be32 *reg = prop->value;
				/* Override start; size */
				reg[0] = cpu_to_be32(ram_console_mem);
				reg[1] = cpu_to_be32(RAM_CONSOLE_SIZE);
			}
		}
	}
}

/*
 * Video
 */
static phys_addr_t tegra_carveout_mem;
static phys_addr_t tegra_fb1_mem;
static phys_addr_t tegra_fb2_mem;
static phys_addr_t tegra_bootloader_fb_mem;
static ulong tegra_bootloader_fb_size;

phys_addr_t get_carveout_mem(void)
{
	return tegra_carveout_mem;
}

phys_addr_t get_fb1_mem(void)
{
	return tegra_fb1_mem;
}

phys_addr_t get_fb2_mem(void)
{
	return tegra_fb2_mem;
}

phys_addr_t get_bootloader_fb_mem(void)
{
	return tegra_bootloader_fb_mem;
}

ulong get_bootloader_fb_size(void)
{
	return tegra_bootloader_fb_size;
}

static int __init reserve_bootloader_fbmem(char *arg)
{
	int err;

	tegra_bootloader_fb_size = PAGE_ALIGN(memparse(arg, &arg));

	err = ((*arg == '@') && tegra_bootloader_fb_size) ? 0 : -EINVAL;

	if (!err) {
		tegra_bootloader_fb_mem = memparse(arg + 1, &arg);
	
		err = memblock_reserve(tegra_bootloader_fb_mem,
				       tegra_bootloader_fb_size);
	}

	if (err)
		tegra_bootloader_fb_size = 0;

	return err;
}
early_param("tegra_fbmem", reserve_bootloader_fbmem);

static void __init reserve_video_mem(void)
{
	tegra_carveout_mem = memblock_end_of_DRAM() - CARVEOUT_MEM_SIZE;

	if (memblock_remove(tegra_carveout_mem, CARVEOUT_MEM_SIZE))
		tegra_carveout_mem = ~0ul;

	tegra_fb1_mem = memblock_end_of_DRAM() - FB1_MEM_SIZE;

	if (memblock_remove(tegra_fb1_mem, FB1_MEM_SIZE))
		tegra_fb1_mem = ~0ul;

	tegra_fb2_mem = memblock_end_of_DRAM() - FB2_MEM_SIZE;

	if (memblock_remove(tegra_fb2_mem, FB2_MEM_SIZE))
		tegra_fb2_mem = ~0ul;
}

void __init tegra_reserve_common(void)
{
	reserve_ram_console_mem();
	reserve_video_mem();
}

void __init tegra_dt_reserved_init(void)
{
	ram_console_dt_resource_init();
}
