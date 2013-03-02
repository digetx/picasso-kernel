/*
 * CPU complex suspend & resume functions for Tegra SoCs
 *
 * Copyright (c) 2009-2012, NVIDIA Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>

#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <asm/idmap.h>
#include <asm/proc-fns.h>
#include <asm/tlbflush.h>
#include <asm/pgalloc.h>
#include <asm/outercache.h>

#include "iomap.h"
#include "reset.h"
#include "flowctrl.h"
#include "sleep.h"
#include "tegra_cpu_car.h"
#include "pm.h"

/* PMC_SCRATCH39 stores the reset vector of the AVP (always 0) after LP0 */
#define PMC_SCRATCH39			0x138
/* PMC_SCRATCH41 stores the reset vector of the CPU after LP0 and LP1 */
#define PMC_SCRATCH41			0x140

#define PMC_SCRATCH1			0x54
#define PMC_SCRATCH0			0x50

#define TEGRA_POWER_PWRREQ_POLARITY	(1 << 8)   /* core pwr req polarity */
#define TEGRA_POWER_PWRREQ_OE		(1 << 9)   /* core pwr req enable */
#define TEGRA_POWER_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define TEGRA_POWER_PWRGATE_DIS		(1 << 12)  /* power gate disabled */
#define TEGRA_POWER_EFFECT_LP0		(1 << 14)  /* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY (1 << 15)  /* CPU pwr req polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	(1 << 16)  /* CPU pwr req enable */

#define EMC_MRW_0		0x0e8
#define EMC_MRW_DEV_SELECTN     30
#define EMC_MRW_DEV_NONE	(3 << EMC_MRW_DEV_SELECTN)

#define PMC_COREPWRGOOD_TIMER	0x3c
#define PMC_COREPWROFF_TIMER	0xe0

#define PMC_CTRL		0x0
#define PMC_CPUPWRGOOD_TIMER	0xc8
#define PMC_CPUPWROFF_TIMER	0xcc
#define PMC_DPD_SAMPLE		0x20

#ifdef CONFIG_PM_SLEEP
static unsigned int g_diag_reg;
static DEFINE_SPINLOCK(tegra_lp2_lock);
static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
static struct clk *tegra_pclk;
void (*tegra_tear_down_cpu)(void);

static unsigned long tegra_lp0_vec_start;
static unsigned long tegra_lp0_vec_size;

static u8 *iram_save;
static unsigned long iram_save_size;
static void __iomem *iram_code;

static struct suspend_params *tegra_sparams;

void save_cpu_arch_register(void)
{
	/* read diagnostic register */
	asm("mrc p15, 0, %0, c15, c0, 1" : "=r"(g_diag_reg) : : "cc");
	return;
}

void restore_cpu_arch_register(void)
{
	/* write diagnostic register */
	asm("mcr p15, 0, %0, c15, c0, 1" : : "r"(g_diag_reg) : "cc");
	return;
}

static void set_power_timers(unsigned long us_on, unsigned long us_off)
{
	unsigned long long ticks;
	unsigned long long pclk;
	unsigned long rate;
	static unsigned long tegra_last_pclk;

	if (tegra_pclk == NULL) {
		tegra_pclk = clk_get_sys(NULL, "pclk");
		WARN_ON(IS_ERR(tegra_pclk));
	}

	rate = clk_get_rate(tegra_pclk);

	if (WARN_ON_ONCE(rate <= 0))
		pclk = 100000000;
	else
		pclk = rate;

	if ((rate != tegra_last_pclk)) {
		ticks = (us_on * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWRGOOD_TIMER);

		ticks = (us_off * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWROFF_TIMER);
		wmb();
	}
	tegra_last_pclk = pclk;
}

/*
 * restore_cpu_complex
 *
 * restores cpu clock setting, clears flow controller
 *
 * Always called on CPU 0.
 */
static void restore_cpu_complex(void)
{
	int cpu = smp_processor_id();

	BUG_ON(cpu != 0);

#ifdef CONFIG_SMP
	cpu = cpu_logical_map(cpu);
#endif

	/* Restore the CPU clock settings */
	tegra_cpu_clock_resume();

	flowctrl_cpu_suspend_exit(cpu);

	restore_cpu_arch_register();
}

/*
 * suspend_cpu_complex
 *
 * saves pll state for use by restart_plls, prepares flow controller for
 * transition to suspend state
 *
 * Must always be called on cpu 0.
 */
static void suspend_cpu_complex(void)
{
	int cpu = smp_processor_id();

	BUG_ON(cpu != 0);

#ifdef CONFIG_SMP
	cpu = cpu_logical_map(cpu);
#endif

	/* Save the CPU clock settings */
	tegra_cpu_clock_suspend();

	flowctrl_cpu_suspend_enter(cpu);

	save_cpu_arch_register();
}

void __cpuinit tegra_clear_cpu_in_lp2(int phy_cpu_id)
{
	u32 *cpu_in_lp2 = tegra_cpu_lp2_mask;

	spin_lock(&tegra_lp2_lock);

	BUG_ON(!(*cpu_in_lp2 & BIT(phy_cpu_id)));
	*cpu_in_lp2 &= ~BIT(phy_cpu_id);

	spin_unlock(&tegra_lp2_lock);
}

bool __cpuinit tegra_set_cpu_in_lp2(int phy_cpu_id)
{
	bool last_cpu = false;
	cpumask_t *cpu_lp2_mask = tegra_cpu_lp2_mask;
	u32 *cpu_in_lp2 = tegra_cpu_lp2_mask;

	spin_lock(&tegra_lp2_lock);

	BUG_ON((*cpu_in_lp2 & BIT(phy_cpu_id)));
	*cpu_in_lp2 |= BIT(phy_cpu_id);

	if ((phy_cpu_id == 0) && cpumask_equal(cpu_lp2_mask, cpu_online_mask))
		last_cpu = true;

	spin_unlock(&tegra_lp2_lock);
	return last_cpu;
}

static int tegra_sleep_cpu(unsigned long v2p)
{
	/* Switch to the identity mapping. */
	cpu_switch_mm(idmap_pgd, &init_mm);

	/* Flush the TLB. */
	local_flush_tlb_all();

	tegra_sleep_cpu_finish(v2p);

	/* should never here */
	BUG();

	return 0;
}

void tegra_idle_lp2_last(u32 cpu_on_time, u32 cpu_off_time)
{
	u32 mode;

	/* Only the last cpu down does the final suspend steps */
	mode = readl(pmc + PMC_CTRL);
	mode |= TEGRA_POWER_CPU_PWRREQ_OE;
	writel(mode, pmc + PMC_CTRL);

	set_power_timers(cpu_on_time, cpu_off_time);

	cpu_cluster_pm_enter();
	suspend_cpu_complex();

	cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, &tegra_sleep_cpu);

	restore_cpu_complex();
	cpu_cluster_pm_exit();
}

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);

	if (!tegra_lp0_vec_size || !tegra_lp0_vec_start) {
		tegra_lp0_vec_size = 0;
		tegra_lp0_vec_start = 0;
	}

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
void pmc_32kwritel(u32 val, unsigned long offs)
{
	writel(val, pmc + offs);
	udelay(130);
}

static void tegra_common_suspend(void)
{
	/* copy the reset vector and SDRAM shutdown code into IRAM */
	memcpy(iram_save, iram_code, iram_save_size);
	memcpy(iram_code, &tegra20_iram_start, iram_save_size);
}

static void tegra_common_resume(void)
{
	void __iomem *emc = IO_ADDRESS(TEGRA_EMC_BASE);

	/* If an immedidate cluster switch is being perfomed, restore the
	   local timer registers. For calls resulting from CPU LP2 in
	   idle or system suspend, the local timer was shut down and
	   timekeeping switched over to the global system timer. In this
	   case keep local timer disabled, and restore only periodic load. */

	/* Clear DPD sample */
	writel(0x0, pmc + PMC_DPD_SAMPLE);

	/* trigger emc mode write */
	writel(EMC_MRW_DEV_NONE, emc + EMC_MRW_0);
	/* clear scratch registers shared by suspend and the reset pen */
	writel(0x0, pmc + PMC_SCRATCH39);
	writel(0x0, pmc + PMC_SCRATCH41);

	/* restore IRAM */
	memcpy(iram_code, iram_save, iram_save_size);
}

static void tegra_pm_set(void)
{
	u32 reg, boot_flag;

	reg = readl(pmc + PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	if (tegra_sparams->combined_req)
		reg &= ~TEGRA_POWER_PWRREQ_OE;
	else
		reg |= TEGRA_POWER_PWRREQ_OE;
	reg |= TEGRA_POWER_EFFECT_LP0;

	if (tegra_sparams->combined_req) {
		reg |= TEGRA_POWER_PWRREQ_OE;
		reg &= ~TEGRA_POWER_CPU_PWRREQ_OE;
	}

	/*
	 * LP0 boots through the AVP, which then resumes the AVP to
	 * the address in scratch 39, and the cpu to the address in
	 * scratch 41 to tegra_resume
	 */
	writel(0x0, pmc + PMC_SCRATCH39);

	/*
	 * Enable DPD sample to trigger sampling pads data and direction
	 * in which pad will be driven during lp0 mode
	 */
	writel(0x1, pmc + PMC_DPD_SAMPLE);

	/* Set warmboot flag */
	boot_flag = readl(pmc + PMC_SCRATCH0);
	pmc_32kwritel(boot_flag | 1, PMC_SCRATCH0);

	pmc_32kwritel(tegra_lp0_vec_start, PMC_SCRATCH1);

	__raw_writel(virt_to_phys(tegra_resume), pmc + PMC_SCRATCH41);
	wmb();

	pmc_32kwritel(reg, PMC_CTRL);
}

static int tegra20_sleep_core(unsigned long v2p)
{
	/* Switch to the identity mapping. */
	cpu_switch_mm(idmap_pgd, &init_mm);

 	/* Flush the TLB. */
	local_flush_tlb_all();

	tegra20_sleep_core_finish(v2p);

	return 0;
}

static void tegra_suspend_dram(void)
{
	unsigned long tmp = l2x0_saved_regs_addr;
	l2x0_saved_regs_addr = 0;

	cpu_pm_enter();
	cpu_cluster_pm_enter();

	tegra_timer_suspend();
	tegra_common_suspend();
	tegra_pm_set();
	tegra_cpu_reset_handler_save();
	suspend_cpu_complex();
	outer_disable();

	cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, &tegra20_sleep_core);

	outer_resume();
	tegra_cpu_reset_handler_restore();
	restore_cpu_complex();
	tegra_common_resume();
	tegra_timer_resume();

	cpu_cluster_pm_exit();
	cpu_pm_exit();

	l2x0_saved_regs_addr = tmp;
}

static __cpuinit int tegra_suspend_enter(suspend_state_t state)
{
	tegra_suspend_dram();

	return 0;
}

static const struct platform_suspend_ops tegra_suspend_ops = {
	.valid = suspend_valid_only_mem,
	.enter = tegra_suspend_enter,
};

void __init tegra_init_suspend(struct suspend_params *sparams)
{
	unsigned char *reloc_lp0;
	unsigned long tmp;
	void __iomem *orig;
	u32 reg;

	if (!sparams)
		return;

	if (memblock_remove(tegra_lp0_vec_start, tegra_lp0_vec_size)) {
		pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
			tegra_lp0_vec_size, tegra_lp0_vec_start);
		return;
	}

	orig = ioremap(tegra_lp0_vec_start, tegra_lp0_vec_size);
	if (!orig) {
		pr_err("%s: Failed to map tegra_lp0_vec_start %08lx\n",
			__func__, tegra_lp0_vec_start);
		memblock_add(tegra_lp0_vec_start, tegra_lp0_vec_size);
		return;
	}

	reloc_lp0 = kmalloc(tegra_lp0_vec_size + L1_CACHE_BYTES - 1, GFP_KERNEL);
	if (!reloc_lp0) {
		pr_err("%s: Failed to allocate reloc_lp0\n", __func__);
		iounmap(orig);
		memblock_add(tegra_lp0_vec_start, tegra_lp0_vec_size);
		return;
	}

	tmp = (unsigned long) reloc_lp0;
	tmp = (tmp + L1_CACHE_BYTES - 1) & ~(L1_CACHE_BYTES - 1);
	reloc_lp0 = (unsigned char *)tmp;
	memcpy(reloc_lp0, orig, tegra_lp0_vec_size);
	iounmap(orig);

	memblock_add(tegra_lp0_vec_start, tegra_lp0_vec_size);

	tegra_lp0_vec_start = virt_to_phys(reloc_lp0);

	iram_save_size = &tegra20_iram_end - &tegra20_iram_start;

	iram_code = ioremap(TEGRA_IRAM_CODE_AREA, iram_save_size);
	if (!iram_code) {
		pr_err("%s: Failed to map iram_code\n", __func__);
		return;
	}

	iram_save = kmalloc(iram_save_size, GFP_KERNEL);
	if (!iram_save) {
		pr_err("%s: unable to allocate memory for SDRAM self-refresh\n",
		       __func__);
		kfree(reloc_lp0);
		return;
	}

	tegra_sparams = sparams;

	/* Always enable CPU power request; just normal polarity is supported */
	reg = readl(pmc + PMC_CTRL);
	BUG_ON(reg & TEGRA_POWER_CPU_PWRREQ_POLARITY);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	pmc_32kwritel(reg, PMC_CTRL);

	/* Configure core power request and system clock control */
	__raw_writel(tegra_sparams->core_timer, pmc + PMC_COREPWRGOOD_TIMER);
	__raw_writel(tegra_sparams->core_off_timer, pmc + PMC_COREPWROFF_TIMER);

	reg = readl(pmc + PMC_CTRL);

	if (!tegra_sparams->sysclkreq_high)
		reg |= TEGRA_POWER_SYSCLK_POLARITY;
	else
		reg &= ~TEGRA_POWER_SYSCLK_POLARITY;

	if (!tegra_sparams->corereq_high)
		reg |= TEGRA_POWER_PWRREQ_POLARITY;
	else
		reg &= ~TEGRA_POWER_PWRREQ_POLARITY;

	/* configure output inverters while the request is tristated */
	pmc_32kwritel(reg, PMC_CTRL);

	/* now enable requests */
	reg |= TEGRA_POWER_SYSCLK_OE;
	if (!tegra_sparams->combined_req)
		reg |= TEGRA_POWER_PWRREQ_OE;
	pmc_32kwritel(reg, PMC_CTRL);

	tegra2_lp0_suspend_init();

	suspend_set_ops(&tegra_suspend_ops);
}
#endif
