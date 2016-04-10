/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/err.h>

#include "clk.h"

static u8 clk_periph_get_parent(struct clk_hw *hw)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *mux_ops = periph->mux_ops;
	struct clk_hw *mux_hw = &periph->mux.hw;

	mux_hw->clk = hw->clk;

	return mux_ops->get_parent(mux_hw);
}

static int clk_periph_set_parent(struct clk_hw *hw, u8 index)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *mux_ops = periph->mux_ops;
	struct clk_hw *mux_hw = &periph->mux.hw;

	mux_hw->clk = hw->clk;

	return mux_ops->set_parent(mux_hw, index);
}

static unsigned long clk_periph_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *div_ops = periph->div_ops;
	struct clk_hw *div_hw = &periph->divider.hw;

	div_hw->clk = hw->clk;

	return div_ops->recalc_rate(div_hw, parent_rate);
}

static long clk_periph_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *div_ops = periph->div_ops;
	struct clk_hw *div_hw = &periph->divider.hw;

	div_hw->clk = hw->clk;

	return div_ops->round_rate(div_hw, rate, prate);
}

static int clk_periph_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *div_ops = periph->div_ops;
	struct clk_hw *div_hw = &periph->divider.hw;

	div_hw->clk = hw->clk;

	return div_ops->set_rate(div_hw, rate, parent_rate);
}

static int clk_periph_is_enabled(struct clk_hw *hw)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *gate_ops = periph->gate_ops;
	struct clk_hw *gate_hw = &periph->gate.hw;

	gate_hw->clk = hw->clk;

	return gate_ops->is_enabled(gate_hw);
}

static int clk_periph_enable(struct clk_hw *hw)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *gate_ops = periph->gate_ops;
	struct clk_hw *gate_hw = &periph->gate.hw;

	gate_hw->clk = hw->clk;

	return gate_ops->enable(gate_hw);
}

static void clk_periph_disable(struct clk_hw *hw)
{
	struct tegra_clk_periph *periph = to_clk_periph(hw);
	const struct clk_ops *gate_ops = periph->gate_ops;
	struct clk_hw *gate_hw = &periph->gate.hw;

	gate_ops->disable(gate_hw);
}

const struct clk_ops tegra_clk_periph_ops = {
	.get_parent = clk_periph_get_parent,
	.set_parent = clk_periph_set_parent,
	.recalc_rate = clk_periph_recalc_rate,
	.round_rate = clk_periph_round_rate,
	.set_rate = clk_periph_set_rate,
	.is_enabled = clk_periph_is_enabled,
	.enable = clk_periph_enable,
	.disable = clk_periph_disable,
};

static const struct clk_ops tegra_clk_periph_nodiv_ops = {
	.get_parent = clk_periph_get_parent,
	.set_parent = clk_periph_set_parent,
	.is_enabled = clk_periph_is_enabled,
	.enable = clk_periph_enable,
	.disable = clk_periph_disable,
};

static const struct clk_ops tegra_clk_periph_no_gate_ops = {
	.get_parent = clk_periph_get_parent,
	.set_parent = clk_periph_set_parent,
	.recalc_rate = clk_periph_recalc_rate,
	.round_rate = clk_periph_round_rate,
	.set_rate = clk_periph_set_rate,
};

extern int tegra_emc_set_rate(unsigned long rate);
extern long tegra_emc_round_rate(unsigned long rate);

static long tegra20_emc_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	long emc_rate;
	long clk_rate;

	/*
	 * The slowest entry in the EMC clock table that is at least as
	 * fast as rate.
	 */
	emc_rate = tegra_emc_round_rate(rate);
	if (emc_rate < 0)
		emc_rate = rate;

	/*
	 * The fastest rate the PLL will generate that is at most the
	 * requested rate.
	 */
	clk_rate = clk_periph_round_rate(hw, emc_rate, prate);

	/*
	 * If this fails, and emc_rate > clk_rate, it's because the maximum
	 * rate in the EMC tables is larger than the maximum rate of the EMC
	 * clock. The EMC clock's max rate is the rate it was running when the
	 * kernel booted. Such a mismatch is probably due to using the wrong
	 * BCT, i.e. using a Tegra20 BCT with an EMC table written for Tegra25.
	 */
	WARN_ONCE(emc_rate != clk_rate, "emc_rate %ld != clk_rate %ld",
		  emc_rate, clk_rate);

	return emc_rate;
}

static int tegra20_emc_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	int ret;

	/*
	 * The Tegra2 memory controller has an interlock with the clock
	 * block that allows memory shadowed registers to be updated,
	 * and then transfer them to the main registers at the same
	 * time as the clock update without glitches.
	 */
	ret = tegra_emc_set_rate(rate);
	if (ret < 0)
		return ret;

	ret = clk_periph_set_rate(hw, rate, parent_rate);
	udelay(1);

	return ret;
}

const struct clk_ops tegra_clk_emc_ops = {
	.get_parent = clk_periph_get_parent,
	.set_parent = clk_periph_set_parent,
	.recalc_rate = clk_periph_recalc_rate,
	.round_rate = tegra20_emc_clk_round_rate,
	.set_rate = tegra20_emc_clk_set_rate,
	.is_enabled = clk_periph_is_enabled,
	.enable = clk_periph_enable,
	.disable = clk_periph_disable,
};

static struct clk *_tegra_clk_register_periph(const char *name,
			const char **parent_names, int num_parents,
			struct tegra_clk_periph *periph,
			void __iomem *clk_base, u32 offset,
			unsigned long flags)
{
	struct clk *clk;
	struct clk_init_data init;
	struct tegra_clk_periph_regs *bank;
	bool div = !(periph->gate.flags & TEGRA_PERIPH_NO_DIV);

	if (periph->gate.flags & TEGRA_PERIPH_NO_DIV) {
		flags |= CLK_SET_RATE_PARENT;
		init.ops = &tegra_clk_periph_nodiv_ops;
	} else if (periph->gate.flags & TEGRA_PERIPH_NO_GATE)
		init.ops = &tegra_clk_periph_no_gate_ops;
	else if (flags & TEGRA_PERIPH_EMC)
		init.ops = &tegra_clk_emc_ops;
	else
		init.ops = &tegra_clk_periph_ops;

	init.name = name;
	init.flags = flags;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	bank = get_reg_bank(periph->gate.clk_num);
	if (!bank)
		return ERR_PTR(-EINVAL);

	/* Data in .init is copied by clk_register(), so stack variable OK */
	periph->hw.init = &init;
	periph->magic = TEGRA_CLK_PERIPH_MAGIC;
	periph->mux.reg = clk_base + offset;
	periph->divider.reg = div ? (clk_base + offset) : NULL;
	periph->gate.clk_base = clk_base;
	periph->gate.regs = bank;
	periph->gate.enable_refcnt = periph_clk_enb_refcnt;

	clk = clk_register(NULL, &periph->hw);
	if (IS_ERR(clk))
		return clk;

	periph->mux.hw.clk = clk;
	periph->divider.hw.clk = div ? clk : NULL;
	periph->gate.hw.clk = clk;

	return clk;
}

struct clk *tegra_clk_register_periph(const char *name,
		const char **parent_names, int num_parents,
		struct tegra_clk_periph *periph, void __iomem *clk_base,
		u32 offset, unsigned long flags)
{
	return _tegra_clk_register_periph(name, parent_names, num_parents,
			periph, clk_base, offset, flags);
}

struct clk *tegra_clk_register_periph_nodiv(const char *name,
		const char **parent_names, int num_parents,
		struct tegra_clk_periph *periph, void __iomem *clk_base,
		u32 offset)
{
	periph->gate.flags |= TEGRA_PERIPH_NO_DIV;
	return _tegra_clk_register_periph(name, parent_names, num_parents,
			periph, clk_base, offset, CLK_SET_RATE_PARENT);
}
