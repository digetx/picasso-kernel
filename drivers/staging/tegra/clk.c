/*
 * Author: Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

int tegra_is_clk_enabled(struct clk *c)
{
	return __clk_get_enable_count(c);
}
EXPORT_SYMBOL(tegra_is_clk_enabled);

static struct clk *emc_clk;

enum {
	AVP,
	CPU,
	DISP1,
	DISP2,
	HDMI,
	G3D,
	G2D,
	MPE,
	USBD,
	USB1,
	USB2,
	USB3,
	CAMERA,
	SHARED_CLK_NB,
};

static struct shared_emc_clk {
	struct clk_hw hw;
	unsigned long rate;
	int enabled;
} virtual_emc_clks[SHARED_CLK_NB];

static void update_emc_clk_rate(void)
{
	struct shared_emc_clk *vclk;
	unsigned long rate = 0;
	int max, i;

	for (i = 0; i < SHARED_CLK_NB; i++) {
		vclk = &virtual_emc_clks[i];

		if (vclk->enabled && vclk->rate > rate) {
			rate = vclk->rate;
			max = i;
		}
	}

	clk_set_rate(emc_clk, rate);

#ifdef DEBUG
	if (rate != 0) {
		struct clk_hw *hw = &virtual_emc_clks[max].hw;
		pr_debug("emc_rate: %lu %s\n", rate, __clk_get_name(hw->clk));
	}
#endif
}

static int shared_emc_clk_prepare(struct clk_hw *hw)
{
	struct shared_emc_clk *vclk =
				container_of(hw, struct shared_emc_clk, hw);
	vclk->enabled = 1;

	pr_debug("%s: %s\n", __func__, __clk_get_name(hw->clk));

	update_emc_clk_rate();

	return 0;
}

static void shared_emc_clk_unprepare(struct clk_hw *hw)
{
	struct shared_emc_clk *vclk =
				container_of(hw, struct shared_emc_clk, hw);
	vclk->enabled = 0;

	pr_debug("%s: %s\n", __func__, __clk_get_name(hw->clk));

	update_emc_clk_rate();
}

static int shared_emc_clk_is_enabled(struct clk_hw *hw)
{
	struct shared_emc_clk *vclk =
				container_of(hw, struct shared_emc_clk, hw);
	return vclk->enabled;
}

static unsigned long shared_emc_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct shared_emc_clk *vclk =
				container_of(hw, struct shared_emc_clk, hw);
	return vclk->rate;
}

static long shared_emc_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	return rate;
}

static int shared_emc_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct shared_emc_clk *vclk =
				container_of(hw, struct shared_emc_clk, hw);
	vclk->rate = rate;

	pr_debug("%s: %s %lu\n", __func__, __clk_get_name(hw->clk), rate);

	if (vclk->enabled)
		update_emc_clk_rate();

	return 0;
}

static const struct clk_ops shared_emc_clk_ops = {
	.prepare	= shared_emc_clk_prepare,
	.unprepare	= shared_emc_clk_unprepare,
	.is_enabled	= shared_emc_clk_is_enabled,
	.recalc_rate	= shared_emc_clk_recalc_rate,
	.round_rate	= shared_emc_clk_round_rate,
	.set_rate	= shared_emc_clk_set_rate,
};

static void register_shared_emc_clk(struct shared_emc_clk *shrd_clk,
				    const char *name, const char *dev_fmt)
{
	struct clk *vemc_clk;
	struct clk_init_data init = {
		.ops  = &shared_emc_clk_ops,
		.name = name,
	};

	shrd_clk->hw.init = &init;

	vemc_clk = clk_register(NULL, &shrd_clk->hw);

	if (IS_ERR(vemc_clk)) {
		pr_err("Failed to register %s clk\n", name);
	}

	if (clk_register_clkdev(vemc_clk, "emc", dev_fmt) != 0) {
		pr_err("Failed to register %s clkdev %s\n", name, dev_fmt);
	}
}

static int register_shared_emc_clks(void)
{
	emc_clk = clk_get_sys(NULL, "emc");
	if (IS_ERR(emc_clk)) {
		pr_err("Failed to get EMC clock\n");
		return PTR_ERR(emc_clk);
	}

	register_shared_emc_clk(&virtual_emc_clks[AVP], "avp_emc", "tegra-avp");
	register_shared_emc_clk(&virtual_emc_clks[CPU], "cpu_emc", "cpu");
	register_shared_emc_clk(&virtual_emc_clks[DISP1], "disp1_emc", "tegradc.0");
	register_shared_emc_clk(&virtual_emc_clks[DISP2], "disp2_emc", "tegradc.1");
	register_shared_emc_clk(&virtual_emc_clks[HDMI], "hdmi_emc", "hdmi");
	register_shared_emc_clk(&virtual_emc_clks[G3D], "3d_emc", "tegra_gr3d");
	register_shared_emc_clk(&virtual_emc_clks[G2D], "2d_emc", "tegra_gr2d");
	register_shared_emc_clk(&virtual_emc_clks[MPE], "mpe_emc", "tegra_mpe");
	register_shared_emc_clk(&virtual_emc_clks[USBD], "usbd_emc", "c5000000.usb");
	register_shared_emc_clk(&virtual_emc_clks[USB1], "usb1_emc", "c5000000.usb");
	register_shared_emc_clk(&virtual_emc_clks[USB2], "usb2_emc", "c5004000.usb");
	register_shared_emc_clk(&virtual_emc_clks[USB3], "usb3_emc", "c5008000.usb");
	register_shared_emc_clk(&virtual_emc_clks[CAMERA], "camera_emc", "tegra_camera");

	return 0;
}

subsys_initcall(register_shared_emc_clks)
