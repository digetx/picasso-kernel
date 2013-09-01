/*
 * Bluetooth Broadcomm low power control via GPIO
 *
 *  Copyright (C) 2010 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/rfkill.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/serial_core.h>
#include <linux/of_gpio.h>
#include <asm/mach-types.h>

static struct brcm_4329_gpios {
	int bt_wake;
	int bt_reset;
	int bt_hostwake;
} brcm_4329_gpios;

extern void change_power_brcm_4329(bool);
static struct rfkill *bt_rfkill;

struct bcm_bt_lpm {
	int wake;
	int host_wake;
	bool irq_disabled;
	bool rx_wake_lock_released;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct wake_lock wake_lock_tx;
	struct wake_lock wake_lock_rx;
} bt_lpm;

static int bcm4329_bt_rfkill_set_power(void *data, bool blocked)
{
	int irq;
	irq = gpio_to_irq(brcm_4329_gpios.bt_hostwake);

	// rfkill_ops callback. Turn transmitter on when blocked is false
	if (!blocked) {
		if (bt_lpm.irq_disabled) {
			enable_irq(irq);
			enable_irq_wake(irq);
			bt_lpm.irq_disabled = false;
		}

		change_power_brcm_4329(true);
		gpio_direction_output(brcm_4329_gpios.bt_reset, 1);
	} else {
		change_power_brcm_4329(false);
		gpio_direction_output(brcm_4329_gpios.bt_reset, 0);

		// There is no resistor associated with this GPIO
		// so the value can float. Disable and release the
		// wakelock.
		if (!bt_lpm.irq_disabled) {
			disable_irq(irq);
			disable_irq_wake(irq);
			bt_lpm.irq_disabled = true;
		}

		if (bt_lpm.host_wake && !bt_lpm.rx_wake_lock_released)
			wake_lock_timeout(&bt_lpm.wake_lock_rx, HZ/2);
	}

	return 0;
}

static const struct rfkill_ops bcm4329_bt_rfkill_ops = {
	.set_block = bcm4329_bt_rfkill_set_power,
};

static void set_wake_locked(int wake)
{
	bt_lpm.wake = wake;

	if (!wake)
		wake_unlock(&bt_lpm.wake_lock_tx);
	else
		wake_lock_timeout(&bt_lpm.wake_lock_tx, HZ * 2);

	gpio_set_value(brcm_4329_gpios.bt_wake, wake);
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer)
{
	unsigned long flags;
	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	set_wake_locked(0);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return HRTIMER_NORESTART;
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport)
{
	bt_lpm.uport = uport;

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

	set_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);
}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

void bcm_bt_rx_done_locked(struct uart_port *uport)
{
	if (bt_lpm.host_wake) {
		// Release wake in 500 ms so that higher layers can take it.
		wake_lock_timeout(&bt_lpm.wake_lock_rx, HZ/2);
		bt_lpm.rx_wake_lock_released = true;
	}
}
EXPORT_SYMBOL(bcm_bt_rx_done_locked);

static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		bt_lpm.rx_wake_lock_released = false;
		wake_lock(&bt_lpm.wake_lock_rx);
	} else if (!bt_lpm.rx_wake_lock_released) {
		// Failsafe timeout of wakelock.
		// If the host wake pin is asserted and no data is sent,
		// when its deasserted we will enter this path
		wake_lock_timeout(&bt_lpm.wake_lock_rx, HZ/2);
	}

}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;
	unsigned long flags;

	host_wake = gpio_get_value(brcm_4329_gpios.bt_hostwake);
	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	update_host_wake_locked(host_wake);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return IRQ_HANDLED;
}

static int __init bcm_bt_lpm_init(struct platform_device *pdev)
{
	int irq;
	int ret;
	int rc;

	rc = devm_gpio_request(&pdev->dev, brcm_4329_gpios.bt_wake,
						"bcm4329_wake_gpio");
	if (unlikely(rc))
		return rc;

	rc = devm_gpio_request(&pdev->dev, brcm_4329_gpios.bt_hostwake,
						"bcm4329_host_wake_gpio");
	if (unlikely(rc))
		return rc;

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;

	irq = gpio_to_irq(brcm_4329_gpios.bt_hostwake);
	ret = devm_request_irq(&pdev->dev, irq, host_wake_isr,
			       IRQF_TRIGGER_HIGH, "bt host_wake", NULL);
	if (ret)
		return ret;

	ret = irq_set_irq_wake(irq, 1);
	if (ret)
		dev_warn(&pdev->dev, "Failed to set wake irq\n");

	bt_lpm.irq_disabled = false;

	gpio_direction_output(brcm_4329_gpios.bt_wake, 0);
	gpio_direction_input(brcm_4329_gpios.bt_hostwake);

	wake_lock_init(&bt_lpm.wake_lock_tx, WAKE_LOCK_SUSPEND, "BTLowPowerTx");
	wake_lock_init(&bt_lpm.wake_lock_rx, WAKE_LOCK_SUSPEND, "BTLowPowerRx");
	return 0;
}

static int bcm4329_bluetooth_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int rc = 0;
	int ret = 0;

	brcm_4329_gpios.bt_wake = of_get_named_gpio(np, "wake-gpio", 0);
	brcm_4329_gpios.bt_reset = of_get_named_gpio(np, "reset-gpio", 0);
	brcm_4329_gpios.bt_hostwake = of_get_named_gpio(np, "host-wake-gpio", 0);

	rc = devm_gpio_request(&pdev->dev, brcm_4329_gpios.bt_reset,
						"bcm4329_nreset_gpio");
	if (unlikely(rc))
		return rc;

	bt_lpm.irq_disabled = true;

	bcm4329_bt_rfkill_set_power(NULL, true);

	bt_rfkill = rfkill_alloc("bcm4329 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4329_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill))
		return -ENOMEM;

	rfkill_set_states(bt_rfkill, true, false);

	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		rfkill_destroy(bt_rfkill);
		return -1;
	}

	ret = bcm_bt_lpm_init(pdev);
	if (ret) {
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);
	}

	return ret;
}

static int bcm4329_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	wake_lock_destroy(&bt_lpm.wake_lock_rx);
	wake_lock_destroy(&bt_lpm.wake_lock_tx);
	return 0;
}

static struct of_device_id bcm4329_bluetooth_of_match[] = {
	{ .compatible = "bcm4329,bluetooth-power", },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm4329_bluetooth_of_match);

static struct platform_driver bcm4329_bluetooth_platform_driver = {
	.probe = bcm4329_bluetooth_probe,
	.remove = bcm4329_bluetooth_remove,
	.driver = {
		.name = "bcm4329_bluetooth",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bcm4329_bluetooth_of_match),
	},
};
module_platform_driver(bcm4329_bluetooth_platform_driver);

MODULE_ALIAS("platform:bcm4329");
MODULE_DESCRIPTION("bcm4329_bluetooth");
MODULE_AUTHOR("Jaikumar Ganesh <jaikumar@google.com>");
MODULE_LICENSE("GPL");
