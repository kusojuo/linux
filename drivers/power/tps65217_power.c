/*
 * tps65217_power.c
 *
 * TPS65217 chip family power supply driver
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/tps65217.h>

struct tps65217_power_dev {
	struct tps65217 *tps;
	struct power_supply ac;
	struct power_supply usb;
};

static enum power_supply_property tps65217_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int tps65217_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct tps65217_power_dev *dev;
	unsigned int status;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		dev = container_of(psy, struct tps65217_power_dev, ac);
		ret = tps65217_reg_read(dev->tps, TPS65217_REG_STATUS, &status);
		if (ret == 0)
			val->intval = !!(status & TPS65217_STATUS_ACPWR);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct power_supply tps65217_ac = {
	.name		= "tps65217-ac",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= tps65217_props,
	.num_properties	= ARRAY_SIZE(tps65217_props),
	.get_property	= tps65217_ac_get_property,
};

static int tps65217_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct tps65217_power_dev *dev;
	unsigned int status;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		dev = container_of(psy, struct tps65217_power_dev, usb);
		ret = tps65217_reg_read(dev->tps, TPS65217_REG_STATUS, &status);
		if (ret == 0)
			val->intval = !!(status & TPS65217_STATUS_USBPWR);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct power_supply tps65217_usb = {
	.name		= "tps65217-usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= tps65217_props,
	.num_properties	= ARRAY_SIZE(tps65217_props),
	.get_property	= tps65217_usb_get_property,
};

static irqreturn_t tps65217_power_interrupt(int irq, void *dev_id)
{
	struct power_supply *psy = dev_id;
	power_supply_changed(psy);
	return IRQ_HANDLED;
}

static int __devinit tps65217_power_probe(struct platform_device *pdev)
{
	struct tps65217_power_dev *dev;
	int ac_irq, usb_irq;
	int ret;

	ac_irq = platform_get_irq(pdev, 0);
	usb_irq = platform_get_irq(pdev, 1);
	if (ac_irq < 0 || usb_irq < 0)
		return -ENODEV;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->tps = dev_to_tps65217(pdev->dev.parent);
	dev->ac = tps65217_ac;
	dev->usb = tps65217_usb;
	platform_set_drvdata(pdev, dev);

	ret = power_supply_register(&pdev->dev, &dev->ac);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(&pdev->dev, ac_irq, NULL,
				tps65217_power_interrupt, IRQF_ONESHOT,
				"ac_power", &dev->ac);
	if (ret < 0)
		goto unreg_ac;

	ret = power_supply_register(&pdev->dev, &dev->usb);
	if (ret)
		goto unreg_ac;

	ret = devm_request_threaded_irq(&pdev->dev, usb_irq, NULL,
				tps65217_power_interrupt, IRQF_ONESHOT,
				"usb_power", &dev->usb);
	if (ret < 0)
		goto unreg_usb;

	return 0;

unreg_usb:
	power_supply_unregister(&dev->usb);
unreg_ac:
	power_supply_unregister(&dev->ac);
	return ret;
}

static int __devexit tps65217_power_remove(struct platform_device *pdev)
{
	struct tps65217_power_dev *dev = platform_get_drvdata(pdev);

	power_supply_unregister(&dev->usb);
	power_supply_unregister(&dev->ac);

	return 0;
}

static struct platform_driver tps65217_power_driver = {
	.driver = {
		.name = "tps65217-power",
	},
	.probe = tps65217_power_probe,
	.remove = __devexit_p(tps65217_power_remove),
};

static int __init tps65217_power_init(void)
{
	return platform_driver_register(&tps65217_power_driver);
}

static void __exit tps65217_power_exit(void)
{
	platform_driver_unregister(&tps65217_power_driver);
}

module_init(tps65217_power_init);
module_exit(tps65217_power_exit);

MODULE_AUTHOR("Russ Dill <Russ.Dill@ti.com>");
MODULE_DESCRIPTION("TPS65217 chip family power supply driver");
MODULE_LICENSE("GPL v2");
