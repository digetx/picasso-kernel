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

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of_irq.h>

#include "wakeups.h"

static const struct of_device_id tegra_dt_wakeups_match[] = {
	{ .compatible = "nvidia,tegra20-wakeups", },
	{ .compatible = "nvidia,tegra30-wakeups", },
	{ }
};

static struct device_node *wakeups_np = NULL;

static int update_wake_mask(unsigned int index, unsigned int virq,
			    int flow_type, struct wake_mask_types *wake_msk)
{
	/*
	 * set wake function calls with flow_type as -1
	 * set wake type function calls update_wake_mask with
	 * the wake polarity
	 */
	if (flow_type == -1) {
		pr_debug("Wake%d flow_type=%d\n", index, flow_type);
		/* use argument wake_mask_hi to return mask */
		wake_msk->wake_mask_hi |= (1ULL << index);
	} else {
		int trigger_val = flow_type & IRQF_TRIGGER_MASK;

		switch (trigger_val) {
		case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
			pr_debug("Wake%d flow_type=ANY\n", index);
			wake_msk->wake_mask_any |= (1ULL << index);
			break;

		case IRQF_TRIGGER_HIGH:
		case IRQF_TRIGGER_RISING:
			pr_debug("Wake%d flow_type=HI\n", index);
			wake_msk->wake_mask_hi |= (1ULL << index);
			break;

		case IRQF_TRIGGER_LOW:
		case IRQF_TRIGGER_FALLING:
			pr_debug("Wake%d flow_type=LO\n", index);
			wake_msk->wake_mask_lo |= (1ULL << index);
			break;

		default:
			pr_err("Error: Wake%d UNKNOWN flow_type=%d\n",
				index, flow_type);
		}
	}

	if (wake_msk->wake_mask_hi || wake_msk->wake_mask_lo ||
		wake_msk->wake_mask_any) {
		pr_debug("Enabling wake sources for irq=%d, mask hi=%#llx, "
			 "lo=%#llx, any=%#llx, flow_type=%d\n",
			virq, wake_msk->wake_mask_hi, wake_msk->wake_mask_lo,
			wake_msk->wake_mask_any, flow_type);
		return 0;
	}

	return -EINVAL;
}

int tegra_irq_to_wake(unsigned int virq, int flow_type,
		      struct wake_mask_types *wake_msk)
{
	struct device_node *controller, *child;
	u32 wake = 0, cnt, i;

	wake_msk->wake_mask_hi = 0ULL;
	wake_msk->wake_mask_lo = 0ULL;
	wake_msk->wake_mask_any = 0ULL;

	if (!wakeups_np) {
		wakeups_np = of_find_matching_node(NULL,
						   tegra_dt_wakeups_match);
		if (!wakeups_np)
			return -EINVAL;
	}

	for_each_child_of_node(wakeups_np, child) {
		controller = of_irq_find_parent(child);

		if (WARN_ON(of_property_read_u32(child, "count", &cnt)))
			return -EINVAL;
		wake += cnt;

		/* suppress warns until irq domain not available, skip unused */
		if (!irq_find_host(controller))
			continue;

		for (i = 0; i < cnt; i++) {
			if (irq_of_parse_and_map(child, i) == virq)
				return update_wake_mask(wake - cnt + i , virq,
							flow_type, wake_msk);
		};
	}

	return -EINVAL;
}

int tegra_wake_to_irq(int wake)
{
	struct device_node *child;
	int virq = 0, cnt;

	if (wake < 0)
		return -EINVAL;

	if (!wakeups_np) {
		wakeups_np = of_find_matching_node(NULL,
						   tegra_dt_wakeups_match);
		if (!wakeups_np)
			return -EINVAL;
	}

	for_each_child_of_node(wakeups_np, child) {
		of_property_read_u32(child, "count", &cnt);
		if (cnt - 1 < wake) {
			wake -= cnt;
			continue;
		}

		virq = irq_of_parse_and_map(child, wake);
		break;
	}

	return virq ?: -EINVAL;
}
