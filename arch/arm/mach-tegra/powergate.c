/*
 * drivers/powergate/tegra-powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <mach/clk.h>
#include <mach/powergate.h>

#include "fuse.h"
#include "iomap.h"
#include "clock.h"

#define PWRGATE_TOGGLE		0x30
#define  PWRGATE_TOGGLE_START	(1 << 8)

#define REMOVE_CLAMPING		0x34

#define PWRGATE_STATUS		0x38

#define MC_CLIENT_CTRL		0x100
#define  MC_CLIENT_HOTRESETN	0x104
#define  MC_CLIENT_ORRC_BASE	0x140

enum mc_client {
	MC_CLIENT_AVPC		= 0,
	MC_CLIENT_DC		= 1,
	MC_CLIENT_DCB		= 2,
	MC_CLIENT_EPP		= 3,
	MC_CLIENT_G2		= 4,
	MC_CLIENT_HC		= 5,
	MC_CLIENT_ISP		= 6,
	MC_CLIENT_MPCORE	= 7,
	MC_CLIENT_MPEA		= 8,
	MC_CLIENT_MPEB		= 9,
	MC_CLIENT_MPEC		= 10,
	MC_CLIENT_NV		= 11,
	MC_CLIENT_PPCS		= 12,
	MC_CLIENT_VDE		= 13,
	MC_CLIENT_VI		= 14,
	MC_CLIENT_LAST		= -1,
	MC_CLIENT_AFI		= MC_CLIENT_LAST,
};

struct partition_clk_info {
	const char *clk_name;
	/*
	 * true if clk is only used in assert/deassert reset
	 * and not while enable-den
	 */
	struct clk *clk_ptr;
};

#define MAX_HOTRESET_CLIENT_NUM		4
#define MAX_CLK_EN_NUM			4
struct powergate_partition {
	enum mc_client hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
	struct partition_clk_info clk_info[MAX_CLK_EN_NUM];
};

static struct powergate_partition powergate_partition_info[] = {
	[TEGRA_POWERGATE_CPU]	= { {MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_L2]	= { {MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_3D]	= { {MC_CLIENT_NV, MC_CLIENT_LAST},
				    {{"3d"} }, },
	[TEGRA_POWERGATE_PCIE]	= { {MC_CLIENT_AFI, MC_CLIENT_LAST},
				    {{"afi"}, {"pcie"}, {"pciex"} }, },
	[TEGRA_POWERGATE_VDEC]	= { {MC_CLIENT_VDE, MC_CLIENT_LAST},
				    {{"vde"} }, },
	[TEGRA_POWERGATE_MPE]	= { {MC_CLIENT_MPEA, MC_CLIENT_MPEB,
				     MC_CLIENT_MPEC, MC_CLIENT_LAST},
				    {{"mpe"} }, },
	[TEGRA_POWERGATE_VENC]	= { {MC_CLIENT_ISP, MC_CLIENT_VI, MC_CLIENT_LAST},
				    {{"isp"}, {"vi"}, {"csi"} }, },
};

static int tegra_num_powerdomains;
static int tegra_num_cpu_domains;
static u8 *tegra_cpu_domains;
static u8 tegra30_cpu_domains[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static DEFINE_SPINLOCK(tegra_powergate_lock);

static void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

static u32 mc_read(unsigned long reg)
{
	return readl(mc + reg);
}

static void mc_write(u32 val, unsigned long reg)
{
	writel(val, mc + reg);
}

int tegra_powergate_mc_disable(int id)
{
	u32 idx, clt_ctrl, orrc_reg;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* clear client enable bit */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl &= ~(1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);

		/* wait for outstanding requests to reach 0 */
		orrc_reg = MC_CLIENT_ORRC_BASE + (mcClientBit * 4);
		while (mc_read(orrc_reg) != 0)
			udelay(10);
	}

	return 0;
}

int tegra_powergate_mc_flush(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* assert hotreset (client module is currently in reset) */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn &= ~(1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	return 0;
}

int tegra_powergate_mc_flush_done(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* deassert hotreset */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn |= (1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	return 0;
}

int tegra_powergate_mc_enable(int id)
{
	u32 idx, clt_ctrl;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* enable client */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl |= (1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	return 0;
}

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static u32 pmc_read(unsigned long reg)
{
	return readl(pmc + reg);
}

static void pmc_write(u32 val, unsigned long reg)
{
	writel(val, pmc + reg);
}

static int tegra_powergate_set(int id, bool new_state)
{
	bool status;
	unsigned long flags;
	/* 10us timeout for toggle operation if it takes affect*/
	int toggle_timeout = 10;
	/* 100 * 10 = 1000us timeout for toggle command to take affect in case
	   of contention with h/w initiated CPU power gating */
	int contention_timeout = 100;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	if (id == TEGRA_POWERGATE_CPU) {
		/* CPU ungated in s/w only during boot/resume with outer
		   waiting loop and no contention from other CPUs */
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	do {
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		do {
			udelay(1);
			status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

			toggle_timeout--;
		} while ((status != new_state) && (toggle_timeout > 0));

		contention_timeout--;
	} while ((status != new_state) && (contention_timeout > 0));

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	if (status != new_state) {
		WARN(1, "Could not set powergate %d to %d", id, new_state);
		return -EBUSY;
	}

	return 0;
}

int tegra_powergate_power_on(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	return tegra_powergate_set(id, true);
}

int tegra_powergate_power_off(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	return tegra_powergate_set(id, false);
}

int tegra_powergate_is_powered(int id)
{
	u32 status;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	status = pmc_read(PWRGATE_STATUS) & (1 << id);
	return !!status;
}

int tegra_powergate_remove_clamping(int id)
{
	u32 mask;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	/*
	 * Tegra 2 has a bug where PCIE and VDE clamping masks are
	 * swapped relatively to the partition ids
	 */
	if (id ==  TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if	(id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	pmc_write(mask, REMOVE_CLAMPING);

	return 0;
}

/* Must be called with clk disabled, and returns with clk enabled */
int tegra_powergate_sequence_power_up(int id, struct clk *clk)
{
	int ret;

	tegra_periph_reset_assert(clk);

	ret = tegra_powergate_power_on(id);
	if (ret)
		goto err_power;

	ret = clk_prepare_enable(clk);
	if (ret)
		goto err_clk;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);
	tegra_periph_reset_deassert(clk);

	return 0;

err_clamp:
	clk_disable_unprepare(clk);
err_clk:
	tegra_powergate_power_off(id);
err_power:
	return ret;
}

static int partition_clk_set(int id, bool enable)
{
	struct partition_clk_info *clk_info;
	struct clk *clk;
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;

		if (clk) {
			if (enable && clk_prepare_enable(clk))
				goto err_clk_en;
			else if (!enable)
				clk_disable_unprepare(clk);
		}
	}

	return 0;

err_clk_en:
	while (idx--) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;

		if (clk)
			clk_disable_unprepare(clk);
	}

	return -EINVAL;
}

static void powergate_partition_reset(int id, bool assert)
{
	struct partition_clk_info *clk_info;
	struct clk *clk;
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;

		if (clk) {
			if (assert)
				tegra_periph_reset_assert(clk);
			else
				tegra_periph_reset_deassert(clk);
		}
	}
}

/* Must be called with clk disabled. Powergates the partition only */
int tegra_powergate_partition(int id)
{
	struct partition_clk_info *clk_info;
	struct clk *clk;
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;

		if (clk && tegra_is_clk_enabled(clk))
			return -EINVAL;
	}

	powergate_partition_reset(id, true);

	return tegra_powergate_power_off(id);
}

/* Must be called with clk disabled, and returns with clk disabled */
static int tegra_powergate_reset_module(int id)
{
	int ret;

	powergate_partition_reset(id, true);

	udelay(10);

	ret = partition_clk_set(id, true);
	if (ret)
		return ret;

	udelay(10);

	powergate_partition_reset(id, false);

	partition_clk_set(id, false);

	return 0;
}

/*
 * Must be called with clk disabled, and returns with clk disabled
 * Drivers should enable clks for partition. Unpowergates only the
 * partition.
 */
int tegra_unpowergate_partition(int id)
{
	int ret;

	if (tegra_powergate_is_powered(id))
		return tegra_powergate_reset_module(id);

	ret = tegra_powergate_power_on(id);
	if (ret)
		goto err_power;

	powergate_partition_reset(id, true);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_set(id, true);
	if (ret)
		goto err_clk_on;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);
	powergate_partition_reset(id, false);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	partition_clk_set(id, false);

	return 0;

err_clamp:
	partition_clk_set(id, false);
err_clk_on:
	tegra_powergate_power_off(id);
err_power:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

int tegra_cpu_powergate_id(int cpuid)
{
	if (cpuid > 0 && cpuid < tegra_num_cpu_domains)
		return tegra_cpu_domains[cpuid];

	return -EINVAL;
}

int __init tegra_powergate_init(void)
{
	int id, idx;

	switch (tegra_chip_id) {
	case TEGRA20:
		tegra_num_powerdomains = 7;
		break;
	case TEGRA30:
		tegra_num_powerdomains = 14;
		tegra_num_cpu_domains = 4;
		tegra_cpu_domains = tegra30_cpu_domains;
		break;
	default:
		/* Unknown Tegra variant. Disable powergating */
		tegra_num_powerdomains = 0;
		break;
	}

	for (id = 0; id < tegra_num_powerdomains; id++) {
		for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
			if (!powergate_partition_info[id].clk_info[idx].clk_name)
				break;
			powergate_partition_info[id].
					clk_info[idx].clk_ptr =
						tegra_get_clock_by_name(
				powergate_partition_info[id].clk_info[idx].clk_name);
		}
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static const char * const *powergate_name;

static const char * const powergate_name_t20[] = {
	[TEGRA_POWERGATE_CPU]	= "cpu",
	[TEGRA_POWERGATE_3D]	= "3d",
	[TEGRA_POWERGATE_VENC]	= "venc",
	[TEGRA_POWERGATE_VDEC]	= "vdec",
	[TEGRA_POWERGATE_PCIE]	= "pcie",
	[TEGRA_POWERGATE_L2]	= "l2",
	[TEGRA_POWERGATE_MPE]	= "mpe",
};

static const char * const powergate_name_t30[] = {
	[TEGRA_POWERGATE_CPU]	= "cpu0",
	[TEGRA_POWERGATE_3D]	= "3d0",
	[TEGRA_POWERGATE_VENC]	= "venc",
	[TEGRA_POWERGATE_VDEC]	= "vdec",
	[TEGRA_POWERGATE_PCIE]	= "pcie",
	[TEGRA_POWERGATE_L2]	= "l2",
	[TEGRA_POWERGATE_MPE]	= "mpe",
	[TEGRA_POWERGATE_HEG]	= "heg",
	[TEGRA_POWERGATE_SATA]	= "sata",
	[TEGRA_POWERGATE_CPU1]	= "cpu1",
	[TEGRA_POWERGATE_CPU2]	= "cpu2",
	[TEGRA_POWERGATE_CPU3]	= "cpu3",
	[TEGRA_POWERGATE_CELP]	= "celp",
	[TEGRA_POWERGATE_3D1]	= "3d1",
};

static int powergate_show(struct seq_file *s, void *data)
{
	int i;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < tegra_num_powerdomains; i++)
		seq_printf(s, " %9s %7s\n", powergate_name[i],
			tegra_powergate_is_powered(i) ? "yes" : "no");
	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int __init tegra_powergate_debugfs_init(void)
{
	struct dentry *d;

	switch (tegra_chip_id) {
	case TEGRA20:
		powergate_name = powergate_name_t20;
		break;
	case TEGRA30:
		powergate_name = powergate_name_t30;
		break;
	}

	if (powergate_name) {
		d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
			&powergate_fops);
		if (!d)
			return -ENOMEM;
	}

	return 0;
}

#endif
