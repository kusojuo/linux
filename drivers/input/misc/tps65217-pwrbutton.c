/*
 * tps65217-pwrbutton.c
 *
 * TPS65217 chip family power button support
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/tps65217.h>

static irqreturn_t tps65217_pwrbutton_interrupt(int irq, void *dev_id)
{
	struct input_dev *idev = dev_id;
	struct tps65217 *tps = input_get_drvdata(idev);
	unsigned int status;
	int state;
	int ret;

	ret = tps65217_reg_read(tps, TPS65217_REG_STATUS, &status);
	if (ret < 0)
		return IRQ_NONE;

	state = !!(status & TPS65217_STATUS_PB);
	if (!!test_bit(KEY_POWER, idev->key) == state) {
		/* Don't drop events we missed */
		input_report_key(idev, KEY_POWER, !state);
		input_sync(idev);
	}
	input_report_key(idev, KEY_POWER, state);
	input_sync(idev);

	return IRQ_HANDLED;
}

static int __devinit tps65217_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *idev;
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	idev = input_allocate_device();
	if (!idev)
		return -ENOMEM;

	idev->name = "Power Button";
	idev->phys = "tps65217_pwrbutton/input0";
	set_bit(EV_KEY, idev->evbit);
	set_bit(KEY_POWER, idev->keybit);

	idev->dev.parent = &pdev->dev;
	//device_init_wakeup(&idev->dev, 1);

	input_set_drvdata(idev, dev_to_tps65217(pdev->dev.parent));

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			tps65217_pwrbutton_interrupt, IRQF_ONESHOT,
			"pwrbutton", idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request power button irq: %d\n",
					ret);
		goto free_input_dev;
	}

	ret = input_register_device(idev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register power button: %d\n",
					ret);
		goto free_input_dev;
	}

	platform_set_drvdata(pdev, idev);

	return 0;

free_input_dev:
	input_free_device(idev);
	return ret;
}

static int __devexit tps65217_pwrbutton_remove(struct platform_device *pdev)
{
	struct input_dev *idev = platform_get_drvdata(pdev);

	input_unregister_device(idev);
	input_free_device(idev);

	return 0;
}

static struct platform_driver tps65217_pwrbutton_driver = {
	.driver = {
		.name = "tps65217-pwrbutton",
	},
	.probe = tps65217_pwrbutton_probe,
	.remove = __devexit_p(tps65217_pwrbutton_remove),
};

static int __init tps65217_pwrbutton_init(void)
{
	return platform_driver_register(&tps65217_pwrbutton_driver);
}

static void __exit tps65217_pwrbutton_exit(void)
{
	platform_driver_unregister(&tps65217_pwrbutton_driver);
}

module_init(tps65217_pwrbutton_init);
module_exit(tps65217_pwrbutton_exit);

MODULE_ALIAS("platform:tps65217-pwrbutton");
MODULE_DESCRIPTION("TPS65217 power button");
MODULE_AUTHOR("Russ Dill <Russ.Dill@ti.com>");
MODULE_LICENSE("GPL v2");

