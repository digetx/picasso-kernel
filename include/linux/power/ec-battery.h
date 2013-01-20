/*
 * Gas Gauge driver for Acer EC Compliant Gas Gauges
 */

#ifndef __LINUX_POWER_EC_BATTERY_H_
#define __LINUX_POWER_EC_BATTERY_H_

#include <linux/power_supply.h>
#include <linux/types.h>

/**
 * struct ec_platform_data - platform data for ec devices
 * @i2c_retry_count:		# of times to retry on i2c IO failure
 * @poll_time_msec:		# of time to check for capacity change
 */
struct ec_platform_data {
	int i2c_retry_count;
};

#endif
