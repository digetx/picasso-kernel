/*
 * arch/arm/mach-tegra/tegra2_clocks.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2012 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk-private.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include "clock.h"
#include "fuse.h"
#include "tegra2_emc.h"
#include "tegra20_clocks.h"
#include "tegra_cpu_car.h"

/* Clock definitions */

#define DEFINE_CLK_TEGRA(_name, _rate, _ops, _flags,		\
		   _parent_names, _parents, _parent)		\
	static struct clk tegra_##_name = {			\
		.hw = &tegra_##_name##_hw.hw,			\
		.name = #_name,					\
		.rate = _rate,					\
		.ops = _ops,					\
		.flags = _flags,				\
		.parent_names = _parent_names,			\
		.parents = _parents,				\
		.num_parents = ARRAY_SIZE(_parent_names),	\
		.parent = _parent,				\
	};

static struct clk tegra_clk_32k;
static struct clk_tegra tegra_clk_32k_hw = {
	.hw = {
		.clk = &tegra_clk_32k,
	},
	.fixed_rate = 32768,
};

static struct clk tegra_clk_32k = {
	.name = "clk_32k",
	.rate = 32768,
	.ops = &tegra_clk_32k_ops,
	.hw = &tegra_clk_32k_hw.hw,
	.flags = CLK_IS_ROOT,
};

static struct clk tegra_clk_m;
static struct clk_tegra tegra_clk_m_hw = {
	.hw = {
		.clk = &tegra_clk_m,
	},
	.flags = ENABLE_ON_INIT,
	.reg = 0x1fc,
	.reg_shift = 28,
	.max_rate = 26000000,
	.fixed_rate = 0,
};

static struct clk tegra_clk_m = {
	.name = "clk_m",
	.ops = &tegra_clk_m_ops,
	.hw = &tegra_clk_m_hw.hw,
	.flags = CLK_IS_ROOT,
};

#define DEFINE_PLL(_name, _flags, _reg, _max_rate, _input_min,	\
		   _input_max, _cf_min, _cf_max, _vco_min,	\
		   _vco_max, _freq_table, _lock_delay, _ops,	\
		   _fixed_rate, _parent)			\
	static const char *tegra_##_name##_parent_names[] = {	\
		#_parent,					\
	};							\
	static struct clk *tegra_##_name##_parents[] = {	\
		&tegra_##_parent,				\
	};							\
	static struct clk tegra_##_name;			\
	static struct clk_tegra tegra_##_name##_hw = {		\
		.hw = {						\
			.clk = &tegra_##_name,			\
		},						\
		.flags = _flags,				\
		.reg = _reg,					\
		.max_rate = _max_rate,				\
		.u.pll = {					\
			.input_min = _input_min,		\
			.input_max = _input_max,		\
			.cf_min = _cf_min,			\
			.cf_max = _cf_max,			\
			.vco_min = _vco_min,			\
			.vco_max = _vco_max,			\
			.freq_table = _freq_table,		\
			.lock_delay = _lock_delay,		\
			.fixed_rate = _fixed_rate,		\
		},						\
	};							\
	static struct clk tegra_##_name = {			\
		.name = #_name,					\
		.ops = &_ops,					\
		.hw = &tegra_##_name##_hw.hw,			\
		.parent = &tegra_##_parent,			\
		.parent_names = tegra_##_name##_parent_names,	\
		.parents = tegra_##_name##_parents,		\
		.num_parents = 1,				\
	};

#define DEFINE_PLL_OUT(_name, _flags, _reg, _reg_shift,		\
		_max_rate, _ops, _parent, _clk_flags)		\
	static const char *tegra_##_name##_parent_names[] = {	\
		#_parent,					\
	};							\
	static struct clk *tegra_##_name##_parents[] = {	\
		&tegra_##_parent,				\
	};							\
	static struct clk tegra_##_name;			\
	static struct clk_tegra tegra_##_name##_hw = {		\
		.hw = {						\
			.clk = &tegra_##_name,			\
		},						\
		.flags = _flags,				\
		.reg = _reg,					\
		.max_rate = _max_rate,				\
		.reg_shift = _reg_shift,			\
	};							\
	static struct clk tegra_##_name = {			\
		.name = #_name,					\
		.ops = &tegra_pll_div_ops,			\
		.hw = &tegra_##_name##_hw.hw,			\
		.parent = &tegra_##_parent,			\
		.parent_names = tegra_##_name##_parent_names,	\
		.parents = tegra_##_name##_parents,		\
		.num_parents = 1,				\
		.flags = _clk_flags,				\
	};


static struct clk_pll_freq_table tegra_pll_s_freq_table[] = {
	{32768, 12000000, 366, 1, 1, 0},
	{32768, 13000000, 397, 1, 1, 0},
	{32768, 19200000, 586, 1, 1, 0},
	{32768, 26000000, 793, 1, 1, 0},
	{0, 0, 0, 0, 0, 0},
};

DEFINE_PLL(pll_s, PLL_ALT_MISC_REG, 0xf0, 26000000, 32768, 32768, 0,
		0, 12000000, 26000000, tegra_pll_s_freq_table, 300,
		tegra_pll_ops, 0, clk_32k);

static struct clk_pll_freq_table tegra_pll_c_freq_table[] = {
	{ 12000000, 600000000, 600, 12, 1, 8 },
	{ 13000000, 600000000, 600, 13, 1, 8 },
	{ 19200000, 600000000, 500, 16, 1, 6 },
	{ 26000000, 600000000, 600, 26, 1, 8 },
	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_c, PLL_HAS_CPCON, 0x80, 600000000, 2000000, 31000000, 1000000,
		6000000, 20000000, 1400000000, tegra_pll_c_freq_table, 300,
		tegra_pll_ops, 0, clk_m);

DEFINE_PLL_OUT(pll_c_out1, DIV_U71, 0x84, 0, 600000000,
		tegra_pll_div_ops, pll_c, 0);

static struct clk_pll_freq_table tegra_pll_m_freq_table[] = {
	{ 12000000, 666000000, 666, 12, 1, 8},
	{ 13000000, 666000000, 666, 13, 1, 8},
	{ 19200000, 666000000, 555, 16, 1, 8},
	{ 26000000, 666000000, 666, 26, 1, 8},
	{ 12000000, 600000000, 600, 12, 1, 8},
	{ 13000000, 600000000, 600, 13, 1, 8},
	{ 19200000, 600000000, 375, 12, 1, 6},
	{ 26000000, 600000000, 600, 26, 1, 8},
	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_m, PLL_HAS_CPCON, 0x90, 800000000, 2000000, 31000000, 1000000,
		6000000, 20000000, 1200000000, tegra_pll_m_freq_table, 300,
		tegra_pll_ops, 0, clk_m);

DEFINE_PLL_OUT(pll_m_out1, DIV_U71, 0x94, 0, 600000000,
		tegra_pll_div_ops, pll_m, 0);

static struct clk_pll_freq_table tegra_pll_p_freq_table[] = {
	{ 12000000, 216000000, 432, 12, 2, 8},
	{ 13000000, 216000000, 432, 13, 2, 8},
	{ 19200000, 216000000, 90,   4, 2, 1},
	{ 26000000, 216000000, 432, 26, 2, 8},
	{ 12000000, 432000000, 432, 12, 1, 8},
	{ 13000000, 432000000, 432, 13, 1, 8},
	{ 19200000, 432000000, 90,   4, 1, 1},
	{ 26000000, 432000000, 432, 26, 1, 8},
	{ 0, 0, 0, 0, 0, 0 },
};


DEFINE_PLL(pll_p, ENABLE_ON_INIT | PLL_FIXED | PLL_HAS_CPCON, 0xa0, 432000000,
		2000000, 31000000, 1000000, 6000000, 20000000, 1400000000,
		tegra_pll_p_freq_table, 300, tegra_pll_ops, 216000000, clk_m);

DEFINE_PLL_OUT(pll_p_out1, ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED, 0xa4, 0,
		432000000, tegra_pll_div_ops, pll_p, 0);
DEFINE_PLL_OUT(pll_p_out2, ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED, 0xa4, 16,
		432000000, tegra_pll_div_ops, pll_p, 0);
DEFINE_PLL_OUT(pll_p_out3, ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED, 0xa8, 0,
		432000000, tegra_pll_div_ops, pll_p, 0);
DEFINE_PLL_OUT(pll_p_out4, ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED, 0xa8, 16,
		432000000, tegra_pll_div_ops, pll_p, 0);

static struct clk_pll_freq_table tegra_pll_a_freq_table[] = {
	{ 28800000, 56448000, 49, 25, 1, 1},
	{ 28800000, 73728000, 64, 25, 1, 1},
	{ 28800000, 24000000,  5,  6, 1, 1},
	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_a, PLL_HAS_CPCON, 0xb0, 73728000, 2000000, 31000000, 1000000,
		6000000, 20000000, 1400000000, tegra_pll_a_freq_table, 300,
		tegra_pll_ops, 0, pll_p_out1);

DEFINE_PLL_OUT(pll_a_out0, DIV_U71, 0xb4, 0, 73728000,
		tegra_pll_div_ops, pll_a, 0);

static struct clk_pll_freq_table tegra_pll_d_freq_table[] = {
	{ 12000000,   5000000, 10, 24, 1, 4},
	{ 12000000,  10000000, 10, 12, 1, 4},
	{ 12000000, 161500000, 323, 24, 1, 4},
	{ 12000000, 162000000, 162, 12, 1, 4},

	{ 12000000, 216000000, 216, 12, 1, 4},
	{ 13000000, 216000000, 216, 13, 1, 4},
	{ 19200000, 216000000, 135, 12, 1, 3},
	{ 26000000, 216000000, 216, 26, 1, 4},

	{ 12000000, 252000000, 252, 12, 1, 4},
	{ 13000000, 252000000, 252, 13, 1, 4},
	{ 19200000, 252000000, 210, 16, 1, 3},
	{ 26000000, 252000000, 252, 26, 1, 4},

	{ 12000000, 297000000, 297, 12, 1, 4},
	{ 13000000, 297000000, 297, 13, 1, 4},
	{ 19200000, 297000000, 248, 16, 1, 4},
	{ 26000000, 297000000, 297, 26, 1, 4},

	{ 12000000, 504000000, 504, 12, 1, 8},
	{ 13000000, 504000000, 504, 13, 1, 8},
	{ 19200000, 504000000, 420, 16, 1, 8},
	{ 26000000, 504000000, 504, 26, 1, 8},

	{ 12000000, 594000000, 594, 12, 1, 8},
	{ 13000000, 594000000, 594, 13, 1, 8},
	{ 19200000, 594000000, 495, 16, 1, 8},
	{ 26000000, 594000000, 594, 26, 1, 8},

	{ 12000000, 1000000000, 1000, 12, 1, 12},
	{ 13000000, 1000000000, 1000, 13, 1, 12},
	{ 19200000, 1000000000, 625,  12, 1, 8},
	{ 26000000, 1000000000, 1000, 26, 1, 12},

	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_d, PLL_HAS_CPCON | PLLD, 0xd0, 1000000000, 2000000, 40000000,
		1000000, 6000000, 40000000, 1000000000, tegra_pll_d_freq_table,
		1000, tegra_pll_ops, 0, clk_m);

DEFINE_PLL_OUT(pll_d_out0, DIV_2 | PLLD, 0, 0, 500000000,
		tegra_pll_div_ops, pll_d, CLK_SET_RATE_PARENT);

static struct clk_pll_freq_table tegra_pll_u_freq_table[] = {
	{ 12000000, 480000000, 960, 12, 2, 0},
	{ 13000000, 480000000, 960, 13, 2, 0},
	{ 19200000, 480000000, 200, 4,  2, 0},
	{ 26000000, 480000000, 960, 26, 2, 0},
	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_u, PLLU, 0xc0, 480000000, 2000000, 40000000, 1000000, 6000000,
		48000000, 960000000, tegra_pll_u_freq_table, 1000,
		tegra_pll_ops, 0, clk_m);

static struct clk_pll_freq_table tegra_pll_x_freq_table[] = {
	/* 1 GHz */
	{ 12000000, 1000000000, 1000, 12, 1, 12},
	{ 13000000, 1000000000, 1000, 13, 1, 12},
	{ 19200000, 1000000000, 625,  12, 1, 8},
	{ 26000000, 1000000000, 1000, 26, 1, 12},

	/* 912 MHz */
	{ 12000000, 912000000,  912,  12, 1, 12},
	{ 13000000, 912000000,  912,  13, 1, 12},
	{ 19200000, 912000000,  760,  16, 1, 8},
	{ 26000000, 912000000,  912,  26, 1, 12},

	/* 816 MHz */
	{ 12000000, 816000000,  816,  12, 1, 12},
	{ 13000000, 816000000,  816,  13, 1, 12},
	{ 19200000, 816000000,  680,  16, 1, 8},
	{ 26000000, 816000000,  816,  26, 1, 12},

	/* 760 MHz */
	{ 12000000, 760000000,  760,  12, 1, 12},
	{ 13000000, 760000000,  760,  13, 1, 12},
	{ 19200000, 760000000,  950,  24, 1, 8},
	{ 26000000, 760000000,  760,  26, 1, 12},

	/* 750 MHz */
	{ 12000000, 750000000,  750,  12, 1, 12},
	{ 13000000, 750000000,  750,  13, 1, 12},
	{ 19200000, 750000000,  625,  16, 1, 8},
	{ 26000000, 750000000,  750,  26, 1, 12},

	/* 608 MHz */
	{ 12000000, 608000000,  608,  12, 1, 12},
	{ 13000000, 608000000,  608,  13, 1, 12},
	{ 19200000, 608000000,  380,  12, 1, 8},
	{ 26000000, 608000000,  608,  26, 1, 12},

	/* 456 MHz */
	{ 12000000, 456000000,  456,  12, 1, 12},
	{ 13000000, 456000000,  456,  13, 1, 12},
	{ 19200000, 456000000,  380,  16, 1, 8},
	{ 26000000, 456000000,  456,  26, 1, 12},

	/* 312 MHz */
	{ 12000000, 312000000,  312,  12, 1, 12},
	{ 13000000, 312000000,  312,  13, 1, 12},
	{ 19200000, 312000000,  260,  16, 1, 8},
	{ 26000000, 312000000,  312,  26, 1, 12},

	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_x, PLL_HAS_CPCON | PLL_ALT_MISC_REG, 0xe0, 1000000000, 2000000,
		31000000, 1000000, 6000000, 20000000, 1200000000,
		tegra_pll_x_freq_table, 300, tegra_pllx_ops, 0, clk_m);

static struct clk_pll_freq_table tegra_pll_e_freq_table[] = {
	{ 12000000, 100000000,  200,  24, 1, 0 },
	{ 0, 0, 0, 0, 0, 0 },
};

DEFINE_PLL(pll_e, PLL_ALT_MISC_REG, 0xe8, 100000000, 12000000, 12000000, 0, 0,
		0, 0, tegra_pll_e_freq_table, 0, tegra_plle_ops, 0, clk_m);

static const char *tegra_common_parent_names[] = {
	"clk_m",
};

static struct clk *tegra_common_parents[] = {
	&tegra_clk_m,
};

static struct clk tegra_clk_d;
static struct clk_tegra tegra_clk_d_hw = {
	.hw = {
		.clk = &tegra_clk_d,
	},
	.flags = PERIPH_NO_RESET,
	.reg = 0x34,
	.reg_shift = 12,
	.max_rate = 52000000,
	.u.periph = {
		.clk_num = 90,
	},
};

static struct clk tegra_clk_d = {
	.name = "clk_d",
	.hw = &tegra_clk_d_hw.hw,
	.ops = &tegra_clk_double_ops,
	.parent = &tegra_clk_m,
	.parent_names = tegra_common_parent_names,
	.parents = tegra_common_parents,
	.num_parents = ARRAY_SIZE(tegra_common_parent_names),
};

static struct clk tegra_cdev1;
static struct clk_tegra tegra_cdev1_hw = {
	.hw = {
		.clk = &tegra_cdev1,
	},
	.fixed_rate = 26000000,
	.u.periph = {
		.clk_num = 94,
	},
};
static struct clk tegra_cdev1 = {
	.name = "cdev1",
	.hw = &tegra_cdev1_hw.hw,
	.ops = &tegra_cdev_clk_ops,
	.flags = CLK_IS_ROOT,
};

/* dap_mclk2, belongs to the cdev2 pingroup. */
static struct clk tegra_cdev2;
static struct clk_tegra tegra_cdev2_hw = {
	.hw = {
		.clk = &tegra_cdev2,
	},
	.fixed_rate = 26000000,
	.u.periph = {
		.clk_num  = 93,
	},
};
static struct clk tegra_cdev2 = {
	.name = "cdev2",
	.hw = &tegra_cdev2_hw.hw,
	.ops = &tegra_cdev_clk_ops,
	.flags = CLK_IS_ROOT,
};

/* initialized before peripheral clocks */
static struct clk_mux_sel mux_audio_sync_clk[8+1];
static const struct audio_sources {
	const char *name;
	int value;
} mux_audio_sync_clk_sources[] = {
	{ .name = "spdif_in", .value = 0 },
	{ .name = "i2s1", .value = 1 },
	{ .name = "i2s2", .value = 2 },
	{ .name = "pll_a_out0", .value = 4 },
#if 0 /* FIXME: not implemented */
	{ .name = "ac97", .value = 3 },
	{ .name = "ext_audio_clk2", .value = 5 },
	{ .name = "ext_audio_clk1", .value = 6 },
	{ .name = "ext_vimclk", .value = 7 },
#endif
	{ NULL, 0 }
};

static const char *audio_parent_names[] = {
	"spdif_in",
	"i2s1",
	"i2s2",
	"dummy",
	"pll_a_out0",
	"dummy",
	"dummy",
	"dummy",
};

static struct clk *audio_parents[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static struct clk tegra_audio;
static struct clk_tegra tegra_audio_hw = {
	.hw = {
		.clk = &tegra_audio,
	},
	.reg = 0x38,
	.max_rate = 73728000,
};
DEFINE_CLK_TEGRA(audio, 0, &tegra_audio_sync_clk_ops, 0, audio_parent_names,
		audio_parents, NULL);

static const char *audio_2x_parent_names[] = {
	"audio",
};

static struct clk *audio_2x_parents[] = {
	&tegra_audio,
};

static struct clk tegra_audio_2x;
static struct clk_tegra tegra_audio_2x_hw = {
	.hw = {
		.clk = &tegra_audio_2x,
	},
	.flags = PERIPH_NO_RESET,
	.max_rate = 48000000,
	.reg = 0x34,
	.reg_shift = 8,
	.u.periph = {
		.clk_num = 89,
	},
};
DEFINE_CLK_TEGRA(audio_2x, 0, &tegra_clk_double_ops, 0, audio_2x_parent_names,
		audio_2x_parents, &tegra_audio);

static struct clk_lookup tegra_audio_clk_lookups[] = {
	{ .con_id = "audio", .clk = &tegra_audio },
	{ .con_id = "audio_2x", .clk = &tegra_audio_2x }
};

/* This is called after peripheral clocks are initialized, as the
 * audio_sync clock depends on some of the peripheral clocks.
 */

static void init_audio_sync_clock_mux(void)
{
	int i;
	struct clk_mux_sel *sel = mux_audio_sync_clk;
	const struct audio_sources *src = mux_audio_sync_clk_sources;
	struct clk_lookup *lookup;

	for (i = 0; src->name; i++, sel++, src++) {
		sel->input = tegra_get_clock_by_name(src->name);
		if (!sel->input)
			pr_err("%s: could not find clk %s\n", __func__,
				src->name);
		audio_parents[src->value] = sel->input;
		sel->value = src->value;
	}

	lookup = tegra_audio_clk_lookups;
	for (i = 0; i < ARRAY_SIZE(tegra_audio_clk_lookups); i++, lookup++) {
		struct clk *c = lookup->clk;
		struct clk_tegra *clk = to_clk_tegra(c->hw);
		__clk_init(NULL, c);
		INIT_LIST_HEAD(&clk->shared_bus_list);
		clk->lookup.con_id = lookup->con_id;
		clk->lookup.clk = c;
		clkdev_add(&clk->lookup);
		tegra_clk_add(c);
	}
}

static const char *mux_cclk[] = {
	"clk_m",
	"pll_c",
	"clk_32k",
	"pll_m",
	"pll_p",
	"pll_p_out4",
	"pll_p_out3",
	"clk_d",
	"pll_x",
};


static struct clk *mux_cclk_p[] = {
	&tegra_clk_m,
	&tegra_pll_c,
	&tegra_clk_32k,
	&tegra_pll_m,
	&tegra_pll_p,
	&tegra_pll_p_out4,
	&tegra_pll_p_out3,
	&tegra_clk_d,
	&tegra_pll_x,
};

static const char *mux_sclk[] = {
	"clk_m",
	"pll_c_out1",
	"pll_p_out4",
	"pllp_p_out3",
	"pll_p_out2",
	"clk_d",
	"clk_32k",
	"pll_m_out1",
};

static struct clk *mux_sclk_p[] = {
	&tegra_clk_m,
	&tegra_pll_c_out1,
	&tegra_pll_p_out4,
	&tegra_pll_p_out3,
	&tegra_pll_p_out2,
	&tegra_clk_d,
	&tegra_clk_32k,
	&tegra_pll_m_out1,
};

static struct clk tegra_cclk;
static struct clk_tegra tegra_cclk_hw = {
	.hw = {
		.clk = &tegra_cclk,
	},
	.reg = 0x20,
	.max_rate = 1000000000,
};
DEFINE_CLK_TEGRA(cclk, 0, &tegra_super_ops, 0, mux_cclk,
		mux_cclk_p, NULL);

static const char *mux_twd[] = {
	"cclk",
};

static struct clk *mux_twd_p[] = {
	&tegra_cclk,
};

static struct clk tegra_clk_twd;
static struct clk_tegra tegra_clk_twd_hw = {
	.hw = {
		.clk = &tegra_clk_twd,
	},
	.max_rate = 1000000000,
	.mul = 1,
	.div = 4,
};

static struct clk tegra_clk_twd = {
	.name = "twd",
	.ops = &tegra_twd_ops,
	.hw = &tegra_clk_twd_hw.hw,
	.parent = &tegra_cclk,
	.parent_names = mux_twd,
	.parents = mux_twd_p,
	.num_parents = ARRAY_SIZE(mux_twd),
};

static struct clk tegra_sclk;
static struct clk_tegra tegra_sclk_hw = {
	.hw = {
		.clk = &tegra_sclk,
	},
	.reg = 0x28,
	.max_rate = 240000000,
	.min_rate = 40000000,
};
DEFINE_CLK_TEGRA(sclk, 0, &tegra_super_ops, 0, mux_sclk,
		mux_sclk_p, NULL);

static const char *tegra_cop_parent_names[] = {
	"tegra_sclk",
};

static struct clk *tegra_cop_parents[] = {
	&tegra_sclk,
};

static struct clk tegra_cop;
static struct clk_tegra tegra_cop_hw = {
	.hw = {
		.clk = &tegra_cop,
	},
	.max_rate  = 240000000,
	.reset = &tegra2_cop_clk_reset,
};
DEFINE_CLK_TEGRA(cop, 0, &tegra_cop_ops, CLK_SET_RATE_PARENT,
		tegra_cop_parent_names, tegra_cop_parents, &tegra_sclk);

static const char *tegra_hclk_parent_names[] = {
	"tegra_sclk",
};

static struct clk *tegra_hclk_parents[] = {
	&tegra_sclk,
};

static struct clk tegra_hclk;
static struct clk_tegra tegra_hclk_hw = {
	.hw = {
		.clk = &tegra_hclk,
	},
	.flags = DIV_BUS,
	.reg = 0x30,
	.reg_shift = 4,
	.max_rate = 240000000,
	.min_rate = 36000000,
};
DEFINE_CLK_TEGRA(hclk, 0, &tegra_bus_ops, 0, tegra_hclk_parent_names,
		tegra_hclk_parents, &tegra_sclk);

static const char *tegra_pclk_parent_names[] = {
	"tegra_hclk",
};

static struct clk *tegra_pclk_parents[] = {
	&tegra_hclk,
};

static struct clk tegra_pclk;
static struct clk_tegra tegra_pclk_hw = {
	.hw = {
		.clk = &tegra_pclk,
	},
	.flags = DIV_BUS,
	.reg = 0x30,
	.reg_shift = 0,
	.max_rate = 120000000,
	.min_rate = 36000000,
};
DEFINE_CLK_TEGRA(pclk, 0, &tegra_bus_ops, 0, tegra_pclk_parent_names,
		tegra_pclk_parents, &tegra_hclk);

static const char *tegra_blink_parent_names[] = {
	"clk_32k",
};

static struct clk *tegra_blink_parents[] = {
	&tegra_clk_32k,
};

static struct clk tegra_blink;
static struct clk_tegra tegra_blink_hw = {
	.hw = {
		.clk = &tegra_blink,
	},
	.reg = 0x40,
	.max_rate = 32768,
};
DEFINE_CLK_TEGRA(blink, 0, &tegra_blink_clk_ops, 0, tegra_blink_parent_names,
		tegra_blink_parents, &tegra_clk_32k);

static const char *mux_pllm_pllc_pllp_plla[] = {
	"pll_m",
	"pll_c",
	"pll_p",
	"pll_a_out0",
};

static struct clk *mux_pllm_pllc_pllp_plla_p[] = {
	&tegra_pll_m,
	&tegra_pll_c,
	&tegra_pll_p,
	&tegra_pll_a_out0,
};

static const char *mux_pllm_pllc_pllp_clkm[] = {
	"pll_m",
	"pll_c",
	"pll_p",
	"clk_m",
};

static struct clk *mux_pllm_pllc_pllp_clkm_p[] = {
	&tegra_pll_m,
	&tegra_pll_c,
	&tegra_pll_p,
	&tegra_clk_m,
};

static const char *mux_pllp_pllc_pllm_clkm[] = {
	"pll_p",
	"pll_c",
	"pll_m",
	"clk_m",
};

static struct clk *mux_pllp_pllc_pllm_clkm_p[] = {
	&tegra_pll_p,
	&tegra_pll_c,
	&tegra_pll_m,
	&tegra_clk_m,
};

static const char *mux_pllaout0_audio2x_pllp_clkm[] = {
	"pll_a_out0",
	"audio_2x",
	"pll_p",
	"clk_m",
};

static struct clk *mux_pllaout0_audio2x_pllp_clkm_p[] = {
	&tegra_pll_a_out0,
	&tegra_audio_2x,
	&tegra_pll_p,
	&tegra_clk_m,
};

static const char *mux_pllp_plld_pllc_clkm[] = {
	"pllp",
	"pll_d_out0",
	"pll_c",
	"clk_m",
};

static struct clk *mux_pllp_plld_pllc_clkm_p[] = {
	&tegra_pll_p,
	&tegra_pll_d_out0,
	&tegra_pll_c,
	&tegra_clk_m,
};

static const char *mux_pllp_pllc_audio_clkm_clk32[] = {
	"pll_p",
	"pll_c",
	"audio",
	"clk_m",
	"clk_32k",
};

static struct clk *mux_pllp_pllc_audio_clkm_clk32_p[] = {
	&tegra_pll_p,
	&tegra_pll_c,
	&tegra_audio,
	&tegra_clk_m,
	&tegra_clk_32k,
};

static const char *mux_pllp_pllc_pllm[] = {
	"pll_p",
	"pll_c",
	"pll_m"
};

static struct clk *mux_pllp_pllc_pllm_p[] = {
	&tegra_pll_p,
	&tegra_pll_c,
	&tegra_pll_m,
};

static const char *mux_clk_m[] = {
	"clk_m",
};

static struct clk *mux_clk_m_p[] = {
	&tegra_clk_m,
};

static const char *mux_pllp_out3[] = {
	"pll_p_out3",
};

static struct clk *mux_pllp_out3_p[] = {
	&tegra_pll_p_out3,
};

static const char *mux_plld[] = {
	"pll_d",
};

static struct clk *mux_plld_p[] = {
	&tegra_pll_d,
};

static const char *mux_clk_32k[] = {
	"clk_32k",
};

static struct clk *mux_clk_32k_p[] = {
	&tegra_clk_32k,
};

static const char *mux_pclk[] = {
	"pclk",
};

static struct clk *mux_pclk_p[] = {
	&tegra_pclk,
};

static struct clk tegra_emc;
static struct clk_tegra tegra_emc_hw = {
	.hw = {
		.clk = &tegra_emc,
	},
	.reg = 0x19c,
	.max_rate = 800000000,
	.flags = MUX | DIV_U71 | PERIPH_EMC_ENB,
	.reset = &tegra2_periph_clk_reset,
	.u.periph = {
		.clk_num = 57,
	},
};
DEFINE_CLK_TEGRA(emc, 0, &tegra_emc_clk_ops, 0, mux_pllm_pllc_pllp_clkm,
		mux_pllm_pllc_pllp_clkm_p, NULL);


#define PERIPH_CLK(_name, _dev, _con, _clk_num, _reg,	\
		_reg_shift, _max, _inputs, _flags) 	\
	static struct clk tegra_##_name;		\
	static struct clk_tegra tegra_##_name##_hw = {	\
		.hw = {					\
			.clk = &tegra_##_name,		\
		},					\
		.lookup = {				\
			.dev_id = _dev,			\
			.con_id = _con,			\
		},					\
		.reg = _reg,				\
		.reg_shift = _reg_shift,		\
		.flags = _flags,			\
		.max_rate = _max,			\
		.u.periph = {				\
			.clk_num = _clk_num,		\
		},					\
		.reset = tegra2_periph_clk_reset,	\
	};						\
	static struct clk tegra_##_name = {		\
		.name = #_name,				\
		.ops = &tegra_periph_clk_ops,		\
		.hw = &tegra_##_name##_hw.hw,		\
		.parent_names = _inputs,		\
		.parents = _inputs##_p,			\
		.num_parents = ARRAY_SIZE(_inputs),	\
	};

PERIPH_CLK(apbdma,	"tegra-apbdma",		NULL,	34,	0,	0x31E,	108000000, mux_pclk,			0);
PERIPH_CLK(rtc,		"rtc-tegra",		NULL,	4,	0,	0x31E,	32768,     mux_clk_32k,			PERIPH_NO_RESET | PERIPH_ON_APB);
PERIPH_CLK(timer,	"timer",		NULL,	5,	0,	0x31E,	26000000,  mux_clk_m,			0);
PERIPH_CLK(i2s1,	"tegra20-i2s.0",	NULL,	11,	0x100,	0x31E,	26000000,  mux_pllaout0_audio2x_pllp_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(i2s2,	"tegra20-i2s.1",	NULL,	18,	0x104,	0x31E,	26000000,  mux_pllaout0_audio2x_pllp_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(spdif_out,	"tegra20-spdif", "spdif_out",	10,	0x108,	0x31E,	100000000, mux_pllaout0_audio2x_pllp_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(spdif_in,	"tegra20-spdif",  "spdif_in",	10,	0x10c,	0x31E,	100000000, mux_pllp_pllc_pllm,		MUX | DIV_U71);
PERIPH_CLK(pwm,		"tegra-pwm",		NULL,	17,	0x110,	0x71C,	432000000, mux_pllp_pllc_audio_clkm_clk32,	MUX | DIV_U71 | MUX_PWM | PERIPH_ON_APB);
PERIPH_CLK(spi,		"spi",			NULL,	43,	0x114,	0x31E,	40000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(xio,		"xio",			NULL,	45,	0x120,	0x31E,	150000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71);
PERIPH_CLK(twc,		"twc",			NULL,	16,	0x12c,	0x31E,	150000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(sbc1,	"spi_tegra.0",		NULL,	41,	0x134,	0x31E,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(sbc2,	"spi_tegra.1",		NULL,	44,	0x118,	0x31E,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(sbc3,	"spi_tegra.2",		NULL,	46,	0x11c,	0x31E,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(sbc4,	"spi_tegra.3",		NULL,	68,	0x1b4,	0x31E,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(ide,		"ide",			NULL,	25,	0x144,	0x31E,	100000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* requires min voltage */
PERIPH_CLK(ndflash,	"tegra_nand",		NULL,	13,	0x160,	0x31E,	164000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* scales with voltage */
PERIPH_CLK(vfir,	"vfir",			NULL,	7,	0x168,	0x31E,	72000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(sdmmc1,	"sdhci-tegra.0",	NULL,	14,	0x150,	0x31E,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* scales with voltage */
PERIPH_CLK(sdmmc2,	"sdhci-tegra.1",	NULL,	9,	0x154,	0x31E,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* scales with voltage */
PERIPH_CLK(sdmmc3,	"sdhci-tegra.2",	NULL,	69,	0x1bc,	0x31E,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* scales with voltage */
PERIPH_CLK(sdmmc4,	"sdhci-tegra.3",	NULL,	15,	0x164,	0x31E,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* scales with voltage */
PERIPH_CLK(vcp,		"tegra-avp",		"vcp",	29,	0,	0x31E,	250000000, mux_clk_m,			0);
PERIPH_CLK(bsea,	"tegra-avp",		"bsea",	62,	0,	0x31E,	250000000, mux_clk_m,			0);
PERIPH_CLK(bsev,	"tegra-aes",		"bsev",	63,	0,	0x31E,	250000000, mux_clk_m,			0);
PERIPH_CLK(vde,		"tegra-avp",		"vde",	61,	0x1c8,	0x31E,	250000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* scales with voltage and process_id */
PERIPH_CLK(csite,	"csite",		NULL,	73,	0x1d4,	0x31E,	144000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* max rate ??? */
/* FIXME: what is la? */
PERIPH_CLK(la,		"la",			NULL,	76,	0x1f8,	0x31E,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71);
PERIPH_CLK(owr,		"tegra_w1",		NULL,	71,	0x1cc,	0x31E,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB);
PERIPH_CLK(nor,		"nor",			NULL,	42,	0x1d0,	0x31E,	92000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71); /* requires min voltage */
PERIPH_CLK(mipi,	"mipi",			NULL,	50,	0x174,	0x31E,	60000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71 | PERIPH_ON_APB); /* scales with voltage */
PERIPH_CLK(i2c1,	"tegra-i2c.0",		"div-clk", 12,	0x124,	0x31E,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16 | PERIPH_ON_APB);
PERIPH_CLK(i2c2,	"tegra-i2c.1",		"div-clk", 54,	0x198,	0x31E,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16 | PERIPH_ON_APB);
PERIPH_CLK(i2c3,	"tegra-i2c.2",		"div-clk", 67,	0x1b8,	0x31E,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16 | PERIPH_ON_APB);
PERIPH_CLK(dvc,		"tegra-i2c.3",		"div-clk", 47,	0x128,	0x31E,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16 | PERIPH_ON_APB);
PERIPH_CLK(uarta,	"tegra-uart.0",		"uart-clk", 6,	0x178,	0x31E,	600000000, mux_pllp_pllc_pllm_clkm,	MUX | PERIPH_ON_APB);
PERIPH_CLK(uartb,	"tegra-uart.1",		"uart-clk", 7,	0x17c,	0x31E,	600000000, mux_pllp_pllc_pllm_clkm,	MUX | PERIPH_ON_APB);
PERIPH_CLK(uartc,	"tegra-uart.2",		"uart-clk", 55,	0x1a0,	0x31E,	600000000, mux_pllp_pllc_pllm_clkm,	MUX | PERIPH_ON_APB);
PERIPH_CLK(uartd,	"tegra-uart.3",		"uart-clk", 65,	0x1c0,	0x31E,	600000000, mux_pllp_pllc_pllm_clkm,	MUX | PERIPH_ON_APB);
PERIPH_CLK(uarte,	"tegra-uart.4",		"uart-clk", 66,	0x1c4,	0x31E,	600000000, mux_pllp_pllc_pllm_clkm,	MUX | PERIPH_ON_APB);
PERIPH_CLK(3d,		"3d",			NULL,	24,	0x158,	0x31E,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71 | PERIPH_MANUAL_RESET); /* scales with voltage and process_id */
PERIPH_CLK(2d,		"2d",			NULL,	21,	0x15c,	0x31E,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71); /* scales with voltage and process_id */
PERIPH_CLK(vi,		"tegra_camera",		"vi",	20,	0x148,	0x31E,	150000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71); /* scales with voltage and process_id */
PERIPH_CLK(vi_sensor,	"tegra_camera",	"vi_sensor",	20,	0x1a8,	0x31E,	150000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71 | PERIPH_NO_RESET); /* scales with voltage and process_id */
PERIPH_CLK(epp,		"epp",			NULL,	19,	0x16c,	0x31E,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71); /* scales with voltage and process_id */
PERIPH_CLK(mpe,		"mpe",			NULL,	60,	0x170,	0x31E,	250000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71); /* scales with voltage and process_id */
PERIPH_CLK(host1x,	"host1x",		NULL,	28,	0x180,	0x31E,	166000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71); /* scales with voltage and process_id */
PERIPH_CLK(cve,		"cve",			NULL,	49,	0x140,	0x31E,	250000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71); /* requires min voltage */
PERIPH_CLK(tvo,		"tvo",			NULL,	49,	0x188,	0x31E,	250000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71); /* requires min voltage */
PERIPH_CLK(hdmi,	"hdmi",			NULL,	51,	0x18c,	0x31E,	600000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71); /* requires min voltage */
PERIPH_CLK(tvdac,	"tvdac",		NULL,	53,	0x194,	0x31E,	250000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71); /* requires min voltage */
PERIPH_CLK(disp1,	"tegradc.0",		NULL,	27,	0x138,	0x31E,	216000000, mux_pllp_plld_pllc_clkm,	MUX); /* scales with voltage and process_id */
PERIPH_CLK(disp2,	"tegradc.1",		NULL,	26,	0x13c,	0x31E,	216000000, mux_pllp_plld_pllc_clkm,	MUX); /* scales with voltage and process_id */
PERIPH_CLK(usbd,	"tegra-udc.0",		NULL,	22,	0,	0x31E,	480000000, mux_clk_m,			0); /* requires min voltage */
PERIPH_CLK(usb2,	"tegra-ehci.1",		NULL,	58,	0,	0x31E,	480000000, mux_clk_m,			0); /* requires min voltage */
PERIPH_CLK(usb3,	"tegra-ehci.2",		NULL,	59,	0,	0x31E,	480000000, mux_clk_m,			0); /* requires min voltage */
PERIPH_CLK(dsia,	"tegradc.0",		"dsia",	48,	0,	0x31E,	500000000, mux_plld,			0); /* scales with voltage */
PERIPH_CLK(csi,		"tegra_camera",		"csi",	52,	0,	0x31E,	72000000,  mux_pllp_out3,		0);
PERIPH_CLK(isp,		"tegra_camera",		"isp",	23,	0,	0x31E,	150000000, mux_clk_m,			0); /* same frequency as VI */
PERIPH_CLK(csus,	"tegra_camera",		"csus",	92,	0,	0x31E,	150000000, mux_clk_m,			PERIPH_NO_RESET);
PERIPH_CLK(pex,		NULL,			"pex",	70,     0,	0x31E,	26000000,  mux_clk_m,			PERIPH_MANUAL_RESET);
PERIPH_CLK(afi,		NULL,			"afi",	72,     0,	0x31E,	26000000,  mux_clk_m,			PERIPH_MANUAL_RESET);
PERIPH_CLK(pcie_xclk,	NULL,		  "pcie_xclk",	74,     0,	0x31E,	26000000,  mux_clk_m,			PERIPH_MANUAL_RESET);
PERIPH_CLK(fuse,	"fuse-tegra",	       "fuse",	39,	0,	0x31E,	26000000,  mux_clk_m,			PERIPH_ON_APB);
PERIPH_CLK(fuse_burn,	"fuse-tegra",	  "fuse_burn",	39,	0,	0x31E,	26000000,  mux_clk_m,			PERIPH_ON_APB);
PERIPH_CLK(kfuse,	"kfuse-tegra",		NULL,	40,	0,	0x31E,	26000000,  mux_clk_m,			0);

static struct clk *tegra_list_clks[] = {
	&tegra_apbdma,
	&tegra_rtc,
	&tegra_timer,
	&tegra_i2s1,
	&tegra_i2s2,
	&tegra_spdif_out,
	&tegra_spdif_in,
	&tegra_pwm,
	&tegra_spi,
	&tegra_xio,
	&tegra_twc,
	&tegra_sbc1,
	&tegra_sbc2,
	&tegra_sbc3,
	&tegra_sbc4,
	&tegra_ide,
	&tegra_ndflash,
	&tegra_vfir,
	&tegra_sdmmc1,
	&tegra_sdmmc2,
	&tegra_sdmmc3,
	&tegra_sdmmc4,
	&tegra_vcp,
	&tegra_bsea,
	&tegra_bsev,
	&tegra_vde,
	&tegra_csite,
	&tegra_la,
	&tegra_owr,
	&tegra_nor,
	&tegra_mipi,
	&tegra_i2c1,
	&tegra_i2c2,
	&tegra_i2c3,
	&tegra_dvc,
	&tegra_uarta,
	&tegra_uartb,
	&tegra_uartc,
	&tegra_uartd,
	&tegra_uarte,
	&tegra_3d,
	&tegra_2d,
	&tegra_vi,
	&tegra_vi_sensor,
	&tegra_epp,
	&tegra_mpe,
	&tegra_host1x,
	&tegra_cve,
	&tegra_tvo,
	&tegra_hdmi,
	&tegra_tvdac,
	&tegra_disp1,
	&tegra_disp2,
	&tegra_usbd,
	&tegra_usb2,
	&tegra_usb3,
	&tegra_dsia,
	&tegra_csi,
	&tegra_isp,
	&tegra_csus,
	&tegra_pex,
	&tegra_afi,
	&tegra_pcie_xclk,
	&tegra_fuse,
	&tegra_fuse_burn,
	&tegra_kfuse,
};

static const char *emc[] = {
	"emc",
};

#define SHARED_CLK(_name, _dev, _con)			\
	static struct clk tegra_##_name;		\
	static struct clk_tegra tegra_##_name##_hw = {	\
		.hw = {					\
			.clk = &tegra_##_name,		\
		},					\
		.lookup = {				\
			.dev_id = _dev,			\
			.con_id = #_con,		\
		},					\
	};						\
	static struct clk tegra_##_name = {		\
		.name = #_name,				\
		.ops = &tegra_clk_shared_bus_ops,	\
		.hw = &tegra_##_name##_hw.hw,		\
		.parent = &tegra_##_con,		\
		.parent_names = _con,			\
		.num_parents = 1,			\
		.flags = CLK_SET_RATE_PARENT  |		\
			 CLK_SET_RATE_NOCACHE,		\
	};

SHARED_CLK(avp_emc,   "tegra-avp",    emc);
SHARED_CLK(cpu_emc,   "cpu",          emc);
SHARED_CLK(disp1_emc, "tegradc.0",    emc);
SHARED_CLK(disp2_emc, "tegradc.1",    emc);
SHARED_CLK(hdmi_emc,  "hdmi",         emc);
SHARED_CLK(3d_emc,    "tegra_gr3d",   emc);
SHARED_CLK(2d_emc,    "tegra_gr2d",   emc);
SHARED_CLK(mpe_emc,   "tegra_mpe",    emc);
SHARED_CLK(usbd_emc,  "tegra-udc.0",  emc);
SHARED_CLK(usb1_emc,  "tegra-ehci.0", emc);
SHARED_CLK(usb2_emc,  "tegra-ehci.1", emc);
SHARED_CLK(usb3_emc,  "tegra-ehci.2", emc);
SHARED_CLK(camera_emc,"tegra_camera", emc);

static struct clk *tegra_list_shared_clks[] = {
	&tegra_avp_emc,
	&tegra_cpu_emc,
	&tegra_disp1_emc,
	&tegra_disp2_emc,
	&tegra_hdmi_emc,
	&tegra_3d_emc,
	&tegra_2d_emc,
	&tegra_mpe_emc,
	&tegra_usbd_emc,
	&tegra_usb1_emc,
	&tegra_usb2_emc,
	&tegra_usb3_emc,
	&tegra_camera_emc,
};

#define CLK_DUPLICATE(_name, _dev, _con)	\
	{					\
		.name	= _name,		\
		.lookup	= {			\
			.dev_id	= _dev,		\
			.con_id	= _con,		\
		},				\
	}

/* Some clocks may be used by different drivers depending on the board
 * configuration.  List those here to register them twice in the clock lookup
 * table under two names.
 */
static struct clk_duplicate tegra_clk_duplicates[] = {
	CLK_DUPLICATE("uarta",      "serial8250.0", NULL),
	CLK_DUPLICATE("uartb",      "serial8250.1", NULL),
	CLK_DUPLICATE("uartc",      "serial8250.2", NULL),
	CLK_DUPLICATE("uartd",      "serial8250.3", NULL),
	CLK_DUPLICATE("uarte",      "serial8250.4", NULL),
	CLK_DUPLICATE("usbd",       "utmip-pad",    NULL),
	CLK_DUPLICATE("usbd",       "tegra-ehci.0", NULL),
	CLK_DUPLICATE("usbd",       "tegra-otg",    NULL),
	CLK_DUPLICATE("hdmi",       "tegradc.0",    "hdmi"),
	CLK_DUPLICATE("hdmi",       "tegradc.1",    "hdmi"),
	CLK_DUPLICATE("dsia",       "tegradc.1",    "dsia"),
	CLK_DUPLICATE("pwm",        "tegra_pwm.0",  NULL),
	CLK_DUPLICATE("pwm",        "tegra_pwm.1",  NULL),
	CLK_DUPLICATE("pwm",        "tegra_pwm.2",  NULL),
	CLK_DUPLICATE("pwm",        "tegra_pwm.3",  NULL),
	CLK_DUPLICATE("host1x",     "tegra_host1x", "host1x"),
	CLK_DUPLICATE("2d",         "tegra_gr2d",   "gr2d"),
	CLK_DUPLICATE("3d",         "tegra_gr3d",   "gr3d"),
	CLK_DUPLICATE("epp",        "tegra_gr2d",   "epp"),
	CLK_DUPLICATE("mpe",        "tegra_mpe",    "mpe"),
	CLK_DUPLICATE("cop",        "tegra-avp",    "cop"),
	CLK_DUPLICATE("sclk",       "tegra-avp",    "sclk"),
	CLK_DUPLICATE("vde",        "tegra-aes",    "vde"),
	CLK_DUPLICATE("twd",        "smp_twd",      NULL),
	CLK_DUPLICATE("bsea",       "tegra-aes",    "bsea"),
	CLK_DUPLICATE("pll_p_out3", "tegra-i2c.0", "fast-clk"),
	CLK_DUPLICATE("pll_p_out3", "tegra-i2c.1", "fast-clk"),
	CLK_DUPLICATE("pll_p_out3", "tegra-i2c.2", "fast-clk"),
	CLK_DUPLICATE("pll_p_out3", "tegra-i2c.3", "fast-clk"),
	CLK_DUPLICATE("cclk",        NULL,         "cpu"),

	CLK_DUPLICATE("disp1",       "tegra-dvfs",   "disp1"),
	CLK_DUPLICATE("disp2",       "tegra-dvfs",   "disp2"),
	CLK_DUPLICATE("hdmi",        "tegra-dvfs",   "hdmi"),
	CLK_DUPLICATE("host1x",      "tegra-dvfs",   "host1x"),
	CLK_DUPLICATE("epp",         "tegra-dvfs",   "epp"),
	CLK_DUPLICATE("2d",          "tegra-dvfs",   "2d"),
	CLK_DUPLICATE("3d",          "tegra-dvfs",   "3d"),
	CLK_DUPLICATE("mpe",         "tegra-dvfs",   "mpe"),
	CLK_DUPLICATE("vi",          "tegra-dvfs",   "vi"),
	CLK_DUPLICATE("csi",         "tegra-dvfs",   "csi"),
	CLK_DUPLICATE("vde",         "tegra-dvfs",   "vde"),
	CLK_DUPLICATE("mipi",        "tegra-dvfs",   "mipi"),
	CLK_DUPLICATE("usbd",        "tegra-dvfs",   "usbd"),
	CLK_DUPLICATE("usb2",        "tegra-dvfs",   "usb2"),
	CLK_DUPLICATE("usb3",        "tegra-dvfs",   "usb3"),
	CLK_DUPLICATE("avp_emc",     "tegra-dvfs",   "avp_emc"),
	CLK_DUPLICATE("cpu_emc",     "tegra-dvfs",   "cpu_emc"),
	CLK_DUPLICATE("disp1_emc",   "tegra-dvfs",   "disp1_emc"),
	CLK_DUPLICATE("disp2_emc",   "tegra-dvfs",   "disp2_emc"),
	CLK_DUPLICATE("hdmi_emc",    "tegra-dvfs",   "hdmi_emc"),
	CLK_DUPLICATE("3d_emc",      "tegra-dvfs",   "3d_emc"),
	CLK_DUPLICATE("2d_emc",      "tegra-dvfs",   "2d_emc"),
	CLK_DUPLICATE("mpe_emc",     "tegra-dvfs",   "mpe_emc"),
	CLK_DUPLICATE("usbd_emc",    "tegra-dvfs",   "usbd_emc"),
	CLK_DUPLICATE("usb1_emc",    "tegra-dvfs",   "usb1_emc"),
	CLK_DUPLICATE("usb2_emc",    "tegra-dvfs",   "usb2_emc"),
	CLK_DUPLICATE("usb3_emc",    "tegra-dvfs",   "usb3_emc"),
	CLK_DUPLICATE("camera_emc",  "tegra-dvfs",   "camera_emc"),
};

#define CLK(dev, con, ck)	\
	{			\
		.dev_id	= dev,	\
		.con_id	= con,	\
		.clk	= ck,	\
	}

static struct clk *tegra_ptr_clks[] = {
	&tegra_clk_32k,
	&tegra_pll_s,
	&tegra_clk_m,
	&tegra_pll_m,
	&tegra_pll_m_out1,
	&tegra_pll_c,
	&tegra_pll_c_out1,
	&tegra_pll_p,
	&tegra_pll_p_out1,
	&tegra_pll_p_out2,
	&tegra_pll_p_out3,
	&tegra_pll_p_out4,
	&tegra_pll_a,
	&tegra_pll_a_out0,
	&tegra_pll_d,
	&tegra_pll_d_out0,
	&tegra_pll_u,
	&tegra_pll_x,
	&tegra_pll_e,
	&tegra_cclk,
	&tegra_clk_twd,
	&tegra_sclk,
	&tegra_hclk,
	&tegra_pclk,
	&tegra_clk_d,
	&tegra_cdev1,
	&tegra_cdev2,
	&tegra_blink,
	&tegra_cop,
	&tegra_emc,
};

static void tegra2_init_one_clock(struct clk *c)
{
	struct clk_tegra *clk = to_clk_tegra(c->hw);
	int ret;

	ret = __clk_init(NULL, c);
	if (ret)
		pr_err("clk init failed %s\n", __clk_get_name(c));

	INIT_LIST_HEAD(&clk->shared_bus_list);
	if (!clk->lookup.dev_id && !clk->lookup.con_id)
		clk->lookup.con_id = c->name;
	clk->lookup.clk = c;
	clkdev_add(&clk->lookup);
	tegra_clk_add(c);
}

void __init tegra2_init_clocks(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(tegra_ptr_clks); i++)
		tegra2_init_one_clock(tegra_ptr_clks[i]);

	for (i = 0; i < ARRAY_SIZE(tegra_list_clks); i++)
		tegra2_init_one_clock(tegra_list_clks[i]);

	for (i = 0; i < ARRAY_SIZE(tegra_list_shared_clks); i++)
		tegra2_init_one_clock(tegra_list_shared_clks[i]);

	for (i = 0; i < ARRAY_SIZE(tegra_clk_duplicates); i++) {
		c = tegra_get_clock_by_name(tegra_clk_duplicates[i].name);
		if (!c) {
			pr_err("%s: Unknown duplicate clock %s\n", __func__,
				tegra_clk_duplicates[i].name);
			continue;
		}

		tegra_clk_duplicates[i].lookup.clk = c;
		clkdev_add(&tegra_clk_duplicates[i].lookup);
	}

	init_audio_sync_clock_mux();
	tegra20_cpu_car_ops_init();
}
