#ifndef __MACH_TEGRA_RESERVE_H
#define __MACH_TEGRA_RESERVE_H

#define FB1_MEM_SIZE		SZ_8M + SZ_1M
#define FB2_MEM_SIZE		SZ_16M

phys_addr_t get_carveout_mem(void);
phys_addr_t get_fb1_mem(void);
phys_addr_t get_fb2_mem(void);
phys_addr_t get_bootloader_fb_mem(void);
ulong get_bootloader_fb_size(void);

void tegra_reserve_common(void);
void tegra_dt_reserved_init(void);
#endif
