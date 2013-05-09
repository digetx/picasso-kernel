#include <linux/module.h>
#include <linux/switch.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>

#define IRQ_FLAGS (IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING)

struct simdet_data {
	struct switch_dev sdev;
	struct delayed_work work;
#ifdef CONFIG_PM_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	unsigned gpio;
	u32 debounce_interval;
};

#ifdef CONFIG_PM_EARLYSUSPEND
static void simdet_late_resume(struct early_suspend *es)
{
	struct simdet_data *switch_data =
			container_of(es, struct simdet_data, early_suspend);
	int state = gpio_get_value(switch_data->gpio);

	switch_set_state(&switch_data->sdev, state);
}
#endif

static void simdet_switch_work(struct work_struct *work)
{
	struct simdet_data *switch_data =
			container_of(work, struct simdet_data, work.work);

	int state = gpio_get_value(switch_data->gpio);

	switch_set_state(&switch_data->sdev, state);
}

static irqreturn_t simdet_isr(int irq, void *dev_id)
{
	struct simdet_data *switch_data = dev_id;

	schedule_delayed_work(&switch_data->work,
			      msecs_to_jiffies(switch_data->debounce_interval));

	return IRQ_HANDLED;
}

static int simdet_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct simdet_data *switch_data;
	int ret, gpio;

	if (!np)
		return -ENODEV;

	if (!of_get_property(np, "gpio", NULL))
		return -ENODEV;

	gpio = of_get_named_gpio(np, "gpio", 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "Invalid gpio pin\n");
		return -EINVAL;
	}

	switch_data = devm_kzalloc(&pdev->dev, sizeof(*switch_data),
				   GFP_KERNEL);
	if (!switch_data){
		dev_err(&pdev->dev, "Can't alloc switch data\n");
		return -ENOMEM;
	}

	switch_data->gpio = gpio;
	switch_data->sdev.name = "simdetect";

	INIT_DELAYED_WORK(&switch_data->work, simdet_switch_work);

	ret = devm_gpio_request(&pdev->dev, gpio, dev_name(&pdev->dev));
	if (ret) {
		dev_err(&pdev->dev, "Can't request GPIO\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, gpio_to_irq(gpio), simdet_isr,
			       IRQ_FLAGS, dev_name(&pdev->dev), switch_data);
	if (ret) {
		dev_err(&pdev->dev, "Can't request IRQ\n");
		return ret;
	}

	if (of_property_read_u32(np, "debounce-interval",
					&switch_data->debounce_interval))
		switch_data->debounce_interval = 250;

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't register switch dev\n");
		return ret;
	}

#ifdef CONFIG_PM_EARLYSUSPEND
	switch_data->early_suspend.resume = simdet_late_resume;
	register_early_suspend(&switch_data->early_suspend);
#endif

	/* set current status */
	schedule_work(&switch_data->work.work);

	platform_set_drvdata(pdev, switch_data);

	return 0;
}

static int simdet_remove(struct platform_device *pdev)
{
	struct simdet_data *switch_data = platform_get_drvdata(pdev);

#ifdef CONFIG_PM_EARLYSUSPEND
	unregister_early_suspend(&switch_data->early_suspend);
#endif
	cancel_delayed_work_sync(&switch_data->work);
	switch_dev_unregister(&switch_data->sdev);

	return 0;
}

static const struct of_device_id simdet_switch_match[] = {
	{ .compatible = "a501,simdetect-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, simdet_switch_match);

static struct platform_driver simdet_switch_driver = {
	.probe	 = simdet_probe,
	.remove	 = simdet_remove,
	.driver	 = {
		.name   = "simdetect",
		.owner  = THIS_MODULE,
		.of_match_table = simdet_switch_match,
	},
};

module_platform_driver(simdet_switch_driver);

MODULE_DESCRIPTION("simdetect driver");
MODULE_LICENSE("GPL");
