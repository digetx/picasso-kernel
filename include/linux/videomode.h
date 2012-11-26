/*
 * Copyright 2012 Steffen Trumtrar <s.trumtrar@pengutronix.de>
 *
 * generic videomode description
 *
 * This file is released under the GPLv2
 */

#ifndef __LINUX_VIDEOMODE_H
#define __LINUX_VIDEOMODE_H

#include <linux/types.h>

struct display_timings;

/*
 * Subsystem independent description of a videomode.
 * Can be generated from struct display_timing.
 */
struct videomode {
	u32 pixelclock;		/* pixelclock in Hz */

	u32 hactive;
	u32 hfront_porch;
	u32 hback_porch;
	u32 hsync_len;

	u32 vactive;
	u32 vfront_porch;
	u32 vback_porch;
	u32 vsync_len;

	u32 hah;		/* hsync active high */
	u32 vah;		/* vsync active high */
	u32 de;			/* data enable */
	u32 pixelclk_pol;

	bool interlaced;
	bool doublescan;
};

/**
 * videomode_from_timing - convert display timing to videomode
 * @disp: structure with all possible timing entries
 * @vm: return value
 * @index: index into the list of display timings in devicetree
 *
 * DESCRIPTION:
 * This function converts a struct display_timing to a struct videomode.
 */
int videomode_from_timing(const struct display_timings *disp,
			  struct videomode *vm, unsigned int index);

#endif
