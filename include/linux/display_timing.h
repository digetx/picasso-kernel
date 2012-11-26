/*
 * Copyright 2012 Steffen Trumtrar <s.trumtrar@pengutronix.de>
 *
 * description of display timings
 *
 * This file is released under the GPLv2
 */

#ifndef __LINUX_DISPLAY_TIMING_H
#define __LINUX_DISPLAY_TIMING_H

#include <linux/types.h>

/*
 * A single signal can be specified via a range with a typical value, that lies
 * somewhere inbetween. Do not use an array, to prevent any confusion about the
 * meaning of every entry.
 */
struct timing_entry {
	u32 min;
	u32 typ;
	u32 max;
};

enum timing_entry_index {
	TE_MIN = 0,
	TE_TYP = 1,
	TE_MAX = 2,
};

/*
 * Single "mode" entry. This describes one set of signal timings a display can
 * have in one setting. This struct can later be converted to struct videomode
 * (see include/linux/videomode.h). As each timing_entry can be defined as a
 * range, one struct display_timing may become multiple struct videomodes.
 */
struct display_timing {
	struct timing_entry pixelclock;

	struct timing_entry hactive;
	struct timing_entry hfront_porch;
	struct timing_entry hback_porch;
	struct timing_entry hsync_len;

	struct timing_entry vactive;
	struct timing_entry vfront_porch;
	struct timing_entry vback_porch;
	struct timing_entry vsync_len;

	unsigned int vsync_pol_active;
	unsigned int hsync_pol_active;
	unsigned int de_pol_active;
	unsigned int pixelclk_pol;
	bool interlaced;
	bool doublescan;
};

/*
 * This describes all timing settings a display provides.
 * The native_mode is the default setting for this display. It can be specified
 * in the devicetree or will be the first that is provided. Drivers that can
 * handle multiple videomode should work with this struct and convert each entry
 * to the desired end result.
 */
struct display_timings {
	unsigned int num_timings;
	unsigned int native_mode;

	struct display_timing **timings;
};

/* get value specified by index from struct timing_entry */
static inline u32 display_timing_get_value(const struct timing_entry *te,
					   enum timing_entry_index index)
{
	switch (index) {
	case TE_MIN:
		return te->min;
		break;
	case TE_TYP:
		return te->typ;
		break;
	case TE_MAX:
		return te->max;
		break;
	default:
		return te->typ;
	}
}

/* get one entry from struct display_timings */
static inline struct display_timing *display_timings_get(const struct
							 display_timings *disp,
							 unsigned int index)
{
	if (disp->num_timings > index)
		return disp->timings[index];
	else
		return NULL;
}

void display_timings_release(struct display_timings *disp);

#endif
