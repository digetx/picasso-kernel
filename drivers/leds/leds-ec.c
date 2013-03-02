/*
 * Leds driver for acer a50x tablets
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <linux/mfd/ec-control.h>

struct ec_led {
	struct led_classdev	cdev;
	struct work_struct	work;
	u8			new_state;
};

EC_REG_DATA(RESET_LED,		0x40,	100);
EC_REG_DATA(POWER_LED_ON,	0x42,	100);
EC_REG_DATA(CHARGE_LED_ON,	0x43,	100);
EC_REG_DATA(ANDROID_LEDS_OFF,	0x5A,	100);

#define EC_LED_WORK(_color, _addr)					\
static void ec_##_color##_led_work(struct work_struct *work)		\
{									\
	struct ec_led *led = container_of(work, struct ec_led, work);	\
									\
	if (led->new_state == LED_OFF) {				\
		ec_write_word_data(RESET_LED, 0);			\
		ec_write_word_data(ANDROID_LEDS_OFF, 0);		\
	} else								\
		ec_write_word_data(_addr, 0);				\
}

EC_LED_WORK(white, POWER_LED_ON);
EC_LED_WORK(orange, CHARGE_LED_ON);

static void ec_led_set(struct led_classdev *led_cdev, 
		       enum led_brightness value)
{
	struct ec_led *led = container_of(led_cdev, struct ec_led, cdev);

	led->new_state = value;
	schedule_work(&led->work);
}

static struct ec_led ec_white_led = {
	.cdev = {
		.name			= "power-button:white",
		.brightness_set		= ec_led_set,
		.max_brightness		= 1,
	},
};

static struct ec_led ec_orange_led = {
	.cdev = {
		.name			= "power-button:orange",
		.brightness_set		= ec_led_set,
		.max_brightness		= 1,
	},
};

static int ec_leds_probe(struct platform_device *pdev)
{
	int ret;

	INIT_WORK(&ec_white_led.work, ec_white_led_work);
	INIT_WORK(&ec_orange_led.work, ec_orange_led_work);

	ret = led_classdev_register(&pdev->dev, &ec_white_led.cdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Failed to register white led\n", __func__);
		return ret;
	}

	ret = led_classdev_register(&pdev->dev, &ec_orange_led.cdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Failed to register orange led\n", __func__);
		led_classdev_unregister(&ec_white_led.cdev);
		return ret;
	}

	return 0;
}

static int ec_leds_remove(struct platform_device *pdev)
{
	cancel_work_sync(&ec_white_led.work);
	led_classdev_unregister(&ec_white_led.cdev);

	cancel_work_sync(&ec_orange_led.work);
	led_classdev_unregister(&ec_orange_led.cdev);

	return 0;
}

static struct platform_driver ec_leds_driver = {
	.driver = {
		.name	= "ec-leds",
		.owner	= THIS_MODULE,
	},
	.probe	= ec_leds_probe,
	.remove	= ec_leds_remove,
};
module_platform_driver(ec_leds_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitry Osipenko");
MODULE_DESCRIPTION("EC leds driver");
