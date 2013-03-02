/*
 * OF helpers for parsing display timings
 *
 * Copyright (c) 2012 Steffen Trumtrar <s.trumtrar@pengutronix.de>, Pengutronix
 *
 * based on of_videomode.c by Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This file is released under the GPLv2
 */
#include <linux/display_timing.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_display_timing.h>
#include <linux/slab.h>

/**
 * parse_property - parse timing_entry from device_node
 * @np: device_node with the property
 * @name: name of the property
 * @result: will be set to the return value
 *
 * DESCRIPTION:
 * Every display_timing can be specified with either just the typical value or
 * a range consisting of min/typ/max. This function helps handling this
 **/
static int parse_property(const struct device_node *np, const char *name,
			  struct timing_entry *result)
{
	struct property *prop;
	int length, cells, ret;

	prop = of_find_property(np, name, &length);
	if (!prop) {
		pr_err("%s: could not find property %s\n", __func__, name);
		return -EINVAL;
	}

	cells = length / sizeof(u32);
	if (cells == 1) {
		ret = of_property_read_u32(np, name, &result->typ);
		result->min = result->typ;
		result->max = result->typ;
	} else if (cells == 3) {
		ret = of_property_read_u32_array(np, name, &result->min, cells);
	} else {
		pr_err("%s: illegal timing specification in %s\n", __func__,
			name);
		return -EINVAL;
	}

	return ret;
}

/**
 * of_get_display_timing - parse display_timing entry from device_node
 * @np: device_node with the properties
 **/
static struct display_timing *of_get_display_timing(const struct device_node
						    *np)
{
	struct display_timing *dt;
	int ret = 0;

	dt = kzalloc(sizeof(*dt), GFP_KERNEL);
	if (!dt) {
		pr_err("%s: could not allocate display_timing struct\n",
			__func__);
		return NULL;
	}

	ret |= parse_property(np, "hback-porch", &dt->hback_porch);
	ret |= parse_property(np, "hfront-porch", &dt->hfront_porch);
	ret |= parse_property(np, "hactive", &dt->hactive);
	ret |= parse_property(np, "hsync-len", &dt->hsync_len);
	ret |= parse_property(np, "vback-porch", &dt->vback_porch);
	ret |= parse_property(np, "vfront-porch", &dt->vfront_porch);
	ret |= parse_property(np, "vactive", &dt->vactive);
	ret |= parse_property(np, "vsync-len", &dt->vsync_len);
	ret |= parse_property(np, "clock-frequency", &dt->pixelclock);

	of_property_read_u32(np, "vsync-active", &dt->vsync_pol_active);
	of_property_read_u32(np, "hsync-active", &dt->hsync_pol_active);
	of_property_read_u32(np, "de-active", &dt->de_pol_active);
	of_property_read_u32(np, "pixelclk-inverted", &dt->pixelclk_pol);
	dt->interlaced = of_property_read_bool(np, "interlaced");
	dt->doublescan = of_property_read_bool(np, "doublescan");

	if (ret) {
		pr_err("%s: error reading timing properties\n", __func__);
		kfree(dt);
		return NULL;
	}

	return dt;
}

/**
 * of_get_display_timings - parse all display_timing entries from a device_node
 * @np: device_node with the subnodes
 **/
struct display_timings *of_get_display_timings(struct device_node *np)
{
	struct device_node *timings_np;
	struct device_node *entry;
	struct device_node *native_mode;
	struct display_timings *disp;

	if (!np) {
		pr_err("%s: no devicenode given\n", __func__);
		return NULL;
	}

	timings_np = of_find_node_by_name(np, "display-timings");
	if (!timings_np) {
		pr_err("%s: could not find display-timings node\n", __func__);
		return NULL;
	}

	disp = kzalloc(sizeof(*disp), GFP_KERNEL);
	if (!disp) {
		pr_err("%s: could not allocate struct disp'\n", __func__);
		goto dispfail;
	}

	entry = of_parse_phandle(timings_np, "native-mode", 0);
	/* assume first child as native mode if none provided */
	if (!entry)
		entry = of_get_next_child(np, NULL);
	/* if there is no child, it is useless to go on */
	if (!entry) {
		pr_err("%s: no timing specifications given\n", __func__);
		goto entryfail;
	}

	pr_info("%s: using %s as default timing\n", __func__, entry->name);

	native_mode = entry;

	disp->num_timings = of_get_child_count(timings_np);
	if (disp->num_timings == 0) {
		/* should never happen, as entry was already found above */
		pr_err("%s: no timings specified\n", __func__);
		goto entryfail;
	}

	disp->timings = kzalloc(sizeof(struct display_timing *) * disp->num_timings,
				GFP_KERNEL);
	if (!disp->timings) {
		pr_err("%s: could not allocate timings array\n", __func__);
		goto entryfail;
	}

	disp->num_timings = 0;
	disp->native_mode = 0;

	for_each_child_of_node(timings_np, entry) {
		struct display_timing *dt;

		dt = of_get_display_timing(entry);
		if (!dt) {
			/*
			 * to not encourage wrong devicetrees, fail in case of
			 * an error
			 */
			pr_err("%s: error in timing %d\n", __func__,
			       disp->num_timings + 1);
			goto timingfail;
		}

		if (native_mode == entry)
			disp->native_mode = disp->num_timings;

		disp->timings[disp->num_timings] = dt;
		disp->num_timings++;
	}
	of_node_put(timings_np);
	/*
	 * native_mode points to the device_node returned by of_parse_phandle
	 * therefore call of_node_put on it
	 */
	of_node_put(native_mode);

	pr_info("%s: got %d timings. Using timing #%d as default\n",
		__func__, disp->num_timings, disp->native_mode + 1);

	return disp;

timingfail:
	if (native_mode)
		of_node_put(native_mode);
	display_timings_release(disp);
entryfail:
	if (disp)
		kfree(disp);
dispfail:
	of_node_put(timings_np);
	return NULL;
}
EXPORT_SYMBOL_GPL(of_get_display_timings);

/**
 * of_display_timings_exists - check if a display-timings node is provided
 * @np: device_node with the timing
 **/
int of_display_timings_exists(struct device_node *np)
{
	struct device_node *timings_np;

	if (!np)
		return -EINVAL;

	timings_np = of_parse_phandle(np, "display-timings", 0);
	if (!timings_np)
		return -EINVAL;

	of_node_put(timings_np);
	return 1;
}
EXPORT_SYMBOL_GPL(of_display_timings_exists);
