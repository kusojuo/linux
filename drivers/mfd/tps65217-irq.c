/*
 * tps65217-irq.c  --  TI TPS65217
 *
 * Copyright 2013 Texas Instruments Inc.
 *
 * Author: Graeme Gregory <gg@slimlogic.co.uk>
 * Author: Jorge Eduardo Candelaria <jedu@slimlogic.co.uk>
 * Author: Russ Dill <Russ.Dill@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/tps65217.h>

/*
 * This is a threaded IRQ handler so can access I2C.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t tps65217_interrupt(int irq, void *dev_id)
{
	struct tps65217 *tps = dev_id;
	unsigned int val;
	int ret;
	int i;

	ret = tps65217_reg_read(tps, TPS65217_REG_INT, &val);
	if (ret < 0)
		return IRQ_NONE;

	/* Clear masked bits */
	val &= ~(val >> TPS65217_INT_MASK_SHIFT);

	for (i = 0; i < TPS65217_NUM_IRQ; i++)
		if (val & (1 << i))
			handle_nested_irq(tps->irq_base + i);

	return IRQ_HANDLED;
}

static void tps65217_irq_lock(struct irq_data *data)
{
	struct tps65217 *tps = irq_data_get_irq_chip_data(data);

	mutex_lock(&tps->irq_lock);
}

static void tps65217_irq_sync_unlock(struct irq_data *data)
{
	struct tps65217 *tps = irq_data_get_irq_chip_data(data);

	tps65217_reg_write(tps, TPS65217_REG_INT,
				tps->irq_mask << TPS65217_INT_MASK_SHIFT,
				TPS65217_PROTECT_NONE);
	mutex_unlock(&tps->irq_lock);
}

static void tps65217_irq_enable(struct irq_data *data)
{
	struct tps65217 *tps = irq_data_get_irq_chip_data(data);
	tps->irq_mask &= ~(1 << (data->irq - tps->irq_base));
}

static void tps65217_irq_disable(struct irq_data *data)
{
	struct tps65217 *tps = irq_data_get_irq_chip_data(data);
	tps->irq_mask |= 1 << (data->irq - tps->irq_base);
}

static struct irq_chip tps65217_irq_chip = {
	.name			= "TPS65217",
	.irq_bus_lock		= tps65217_irq_lock,
	.irq_bus_sync_unlock	= tps65217_irq_sync_unlock,
	.irq_enable		= tps65217_irq_enable,
	.irq_disable		= tps65217_irq_disable,
};

int tps65217_irq_init(struct tps65217 *tps, int irq, int irq_base)
{
	int ret;
	int i;

	if (irq <= 0 || irq_base <= 0)
		return -ENODEV;

	mutex_init(&tps->irq_lock);
	tps->irq_base = irq_base;
	tps->irq_mask = (1 << TPS65217_NUM_IRQ) - 1;

	ret = tps65217_reg_write(tps, TPS65217_REG_INT,
				tps->irq_mask << TPS65217_INT_MASK_SHIFT,
				TPS65217_PROTECT_NONE);
	if (ret < 0)
		return ret;

	/* Register with genirq */
	for (i = tps->irq_base; i < TPS65217_NUM_IRQ + tps->irq_base; i++) {
		irq_set_chip_data(i, tps);
		irq_set_chip_and_handler(i, &tps65217_irq_chip,
						handle_edge_irq);
		irq_set_nested_thread(i, 1);
#ifdef CONFIG_ARM
		set_irq_flags(i, IRQF_VALID);
#else
		irq_set_noprobe(i);
#endif
	}

	ret = devm_request_threaded_irq(tps->dev, irq, NULL,
				tps65217_interrupt, IRQF_ONESHOT,
				"tps65217", tps);
	if (ret)
		dev_err(tps->dev, "Failure requesting irq %i\n", irq);

	return ret;
}
