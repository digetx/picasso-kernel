#include <linux/module.h>
#include <linux/switch.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/mfd/ec-control.h>

#define BOARD_PICASSO_3G	3

#define IRQ_FLAGS (IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING)

extern int get_sku_id(void);

struct psensor_data {
	struct switch_dev sdev;
	struct work_struct work;
#ifdef CONFIG_PM_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int gpio;
};

int check_board_version(void)
{
	if (get_sku_id() == BOARD_PICASSO_3G) {
		if (get_board_id() == 0xfefe)
			return false;

		if (get_board_id() > 2)
			return true;
	}

	return false;
}

#ifdef CONFIG_PM_EARLYSUSPEND
static void psensor_early_suspend(struct early_suspend *es)
{

	struct psensor_data *psensor =
			container_of(es, struct psensor_data, early_suspend);
	int irq = gpio_to_irq(psensor->gpio);

	disable_irq(irq);
	cancel_work_sync(&psensor->work);
}

static void psensor_late_resume(struct early_suspend *es)
{
	struct psensor_data *psensor =
			container_of(es, struct psensor_data, early_suspend);
	int irq = gpio_to_irq(psensor->gpio);

	enable_irq(irq);
}
#endif

static void psensor_work(struct work_struct *work)
{
	struct psensor_data *psensor =
			container_of(work, struct psensor_data, work);
	int psensor_state = gpio_get_value(psensor->gpio);

	switch_set_state(&psensor->sdev, psensor_state);
}

static irqreturn_t psensor_isr(int irq, void *dev_id)
{
	struct psensor_data *psensor = dev_id;

	schedule_work(&psensor->work);

	return IRQ_HANDLED;
}

static int psensor_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct psensor_data *psensor;
	int ret, gpio;

	if (!np)
		return -ENODEV;

	if (!check_board_version())
		return -ENODEV;

	if (!of_get_property(np, "gpio", NULL))
		return -ENODEV;

	gpio = of_get_named_gpio(np, "gpio", 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "Invalid gpio pin\n");
		return -EINVAL;
	}

	psensor = devm_kzalloc(&pdev->dev, sizeof(*psensor), GFP_KERNEL);
	if (!psensor) {
		dev_err(&pdev->dev, "Can't alloc psensor\n");
		return -ENOMEM;
	}

	INIT_WORK(&psensor->work, psensor_work);

	psensor->gpio = gpio;
	psensor->sdev.name = "psensor";

	ret = devm_gpio_request_one(&pdev->dev, gpio, GPIOF_IN,
				    dev_name(&pdev->dev));
	if (ret) {
		dev_err(&pdev->dev, "Can't request GPIO\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, gpio_to_irq(gpio), psensor_isr,
			       IRQ_FLAGS, dev_name(&pdev->dev), psensor);
	if (ret) {
		dev_err(&pdev->dev, "Can't request IRQ\n");
		return ret;
	}

	ret = switch_dev_register(&psensor->sdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't register switch dev\n");
		return ret;
	}

#ifdef CONFIG_PM_EARLYSUSPEND
	psensor->early_suspend.suspend = psensor_early_suspend;
	psensor->early_suspend.resume = psensor_late_resume;
	register_early_suspend(&psensor->early_suspend);
#endif

	/* set current status */
	schedule_work(&psensor->work);

	platform_set_drvdata(pdev, psensor);

	return 0;
}

static int psensor_remove(struct platform_device *pdev)
{
	struct psensor_data *psensor = platform_get_drvdata(pdev);

#ifdef CONFIG_PM_EARLYSUSPEND
	unregister_early_suspend(&psensor->early_suspend);
#endif
	cancel_work_sync(&psensor->work);
	switch_dev_unregister(&psensor->sdev);

	return 0;
}

static const struct of_device_id psensor_switch_match[] = {
	{ .compatible = "a501,psensor-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, psensor_switch_match);

static struct platform_driver psensor_driver = {
	.probe	= psensor_probe,
	.remove	= psensor_remove,
	.driver	= {
		.name = "psensor",
		.owner = THIS_MODULE,
		.of_match_table = psensor_switch_match,
	},
};

module_platform_driver(psensor_driver);

MODULE_DESCRIPTION("psensor driver");
MODULE_LICENSE("GPL");
