
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <dt-bindings/mfd/richtek,rt9467.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/regmap.h>

#define RT9466_VENDOR_ID		0x80
#define RT9467_VENDOR_ID		0x90
#define RT9467_REG_CORE_CTRL0		0x00
#define RT9467_REG_CHG_CTRL1		0x01
#define RT9467_REG_CHG_CTRL13		0x0D
#define RT9467_REG_DEVICE_ID		0x40
#define RT9467_REG_CHG_IRQ_START	0x50
#define RT9467_REG_CHG_IRQ_MASK_START	0x60
#define RT9467_NUM_IRQ_REGS		7

#define RT9467_RST_REG_MASK		BIT(7)
#define RT9467_HZ_MASK			BIT(2)
#define RT9467_INT_REZ_MASK		BIT(0)

struct rt9467_data {
	struct i2c_client *i2c;
	struct device *dev;
	struct regmap *regmap;
	struct irq_domain *irq_domain;
	struct mutex lock;
	struct irq_chip irq_chip;
	u8 mask_buf[RT9467_NUM_IRQ_REGS];
};

static const struct mfd_cell rt9467_devs[] = {
	OF_MFD_CELL("rt9467-adc", NULL, NULL, 0, 0, "richtek,rt9467-adc"),
	OF_MFD_CELL("rt9467-chg", NULL, NULL, 0, 0, "richtek,rt9467-chg")
};

static void rt9467_irq_bus_lock(struct irq_data *id)
{
	struct rt9467_data *data = irq_data_get_irq_chip_data(id);

	mutex_lock(&data->lock);
}

static void rt9467_irq_bus_unlock(struct irq_data *irq_data)
{
	struct rt9467_data *data = irq_data_get_irq_chip_data(irq_data);
	unsigned int idx = irq_data->hwirq / 8;
	int ret;

	ret = regmap_write(data->regmap, RT9467_REG_CHG_IRQ_MASK_START + idx,
			   data->mask_buf[idx]);
	if (ret)
		dev_err(data->dev, "Failed to mask/unmask irq %d\n",
			irq_data->hwirq);
	mutex_unlock(&data->lock);
}

static void rt9467_irq_enable(struct irq_data *irq_data)
{
	struct rt9467_data *data = irq_data_get_irq_chip_data(irq_data);

	data->mask_buf[irq_data->hwirq / 8] &= ~BIT(irq_data->hwirq % 8);
}

static void rt9467_irq_disable(struct irq_data *irq_data)
{
	struct rt9467_data *data = irq_data_get_irq_chip_data(irq_data);

	data->mask_buf[irq_data->hwirq / 8] |= BIT(irq_data->hwirq % 8);
}

static int rt9467_irq_map(struct irq_domain *h, unsigned int virq,
			  irq_hw_number_t hwirq)
{
	struct rt9467_data *data = h->host_data;
	struct i2c_client *i2c = to_i2c_client(data->dev);

	irq_set_chip_data(virq, data);
	irq_set_chip(virq, &data->irq_chip);
	irq_set_nested_thread(virq, true);
	irq_set_parent(virq, i2c->irq);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops rt9467_irq_domain_ops = {
	.map = rt9467_irq_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static irqreturn_t rt9467_irq_handler(int irq, void *priv)
{
	struct rt9467_data *data = priv;
	u8 stat_buf[RT9467_NUM_IRQ_REGS] = { 0 }, evts;
	bool handled = false;
	int i, j, ret;

	ret = regmap_raw_read(data->regmap, RT9467_REG_CHG_IRQ_START, stat_buf,
			      RT9467_NUM_IRQ_REGS);
	if (ret) {
		dev_err(data->dev, "Failed to read irq flag\n");
		return IRQ_NONE;
	}

	for (i = 0; i < RT9467_NUM_IRQ_REGS; i++) {
		evts = stat_buf[i] & ~data->mask_buf[i];
		if (!evts)
			continue;

		for (j = 0; j < 8; j++) {
			if (!(evts & BIT(j)))
				continue;
			handle_nested_irq(irq_find_mapping(data->irq_domain,
							   i * 8 + j));
			handled = true;
		}
	}

	regmap_update_bits(data->regmap, RT9467_REG_CHG_CTRL13,
			   RT9467_INT_REZ_MASK, 1);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int rt9467_add_irq_chip(struct rt9467_data *data)
{
	struct i2c_client *i2c = to_i2c_client(data->dev);
	u8 stat_buf[RT9467_NUM_IRQ_REGS];
	int ret;

	mutex_init(&data->lock);
	data->irq_chip.name = dev_name(data->dev);
	data->irq_chip.irq_bus_lock = rt9467_irq_bus_lock;
	data->irq_chip.irq_bus_sync_unlock = rt9467_irq_bus_unlock;
	data->irq_chip.irq_enable = rt9467_irq_enable;
	data->irq_chip.irq_disable = rt9467_irq_disable;

	/* Mask and read clear all events */
	memset(data->mask_buf, 0xFF, RT9467_NUM_IRQ_REGS);
	ret = regmap_raw_write(data->regmap, RT9467_REG_CHG_IRQ_MASK_START,
			       data->mask_buf, RT9467_NUM_IRQ_REGS);
	if (ret) {
		dev_err(data->dev, "Failed to mask all irqs\n");
		return ret;
	}

	ret = regmap_raw_read(data->regmap, RT9467_REG_CHG_IRQ_START, stat_buf,
			      RT9467_NUM_IRQ_REGS);
	if (ret) {
		dev_err(data->dev, "Failed to read clear irqs\n");
		return ret;
	}

	/* Add irq domain */
	data->irq_domain = irq_domain_add_linear(data->dev->of_node,
						 RT9467_NUM_IRQ_REGS * 8,
						 &rt9467_irq_domain_ops, data);
	if (IS_ERR_OR_NULL(data->irq_domain)) {
		dev_err(data->dev, "Failed to create irq domain\n");
		return -ENOMEM;
	}

	ret = devm_request_threaded_irq(data->dev, i2c->irq, NULL,
					rt9467_irq_handler, IRQF_ONESHOT,
					dev_name(data->dev), data);
	if (ret) {
		dev_err(data->dev, "Failed to request irq(%d)\n", ret);
		irq_domain_remove(data->irq_domain);
		return ret;
	}

	return 0;
}

static void rt9467_del_irq_chip(struct rt9467_data *data)
{
	struct i2c_client *i2c = to_i2c_client(data->dev);

	free_irq(i2c->irq, data);
	irq_domain_remove(data->irq_domain);
	mutex_destroy(&data->lock);
}

static bool rt9467_is_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x1A:
	case 0x20 ... 0x38:
	case 0x40 ... 0x49:
	case 0x50 ... 0x57:
	case 0x60 ... 0x67:
	case 0x70 ... 0x79:
	case 0x82 ... 0x85:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rt9467_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x85,
	.writeable_reg = rt9467_is_accessible_reg,
	.readable_reg = rt9467_is_accessible_reg,
};

static int rt9467_check_vendor_info(struct rt9467_data *data)
{
	unsigned int vid = 0;
	int ret;

	ret = regmap_read(data->regmap, RT9467_REG_DEVICE_ID, &vid);
	if (ret)
		return ret;

	vid &= 0xF0;
	if ((vid != RT9466_VENDOR_ID) && (vid != RT9467_VENDOR_ID)) {
		dev_err(data->dev, "VID not correct [0x%02X]\n", vid);
		return -ENODEV;
	}

	dev_info(data->dev, "VID = 0x%02X\n", vid);
	return 0;
}

static int rt9467_reset_chip(struct rt9467_data *data)
{
	int ret;

	/* Disable HZ before reset chip */
	ret = regmap_update_bits(data->regmap, RT9467_REG_CHG_CTRL1,
				 RT9467_HZ_MASK, 0);
	if (ret)
		return ret;

	return regmap_write(data->regmap, RT9467_REG_CORE_CTRL0,
			    RT9467_RST_REG_MASK);
}

static int rt9467_probe(struct i2c_client *i2c)
{
	struct rt9467_data *data;
	int ret;

	data = devm_kzalloc(&i2c->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &i2c->dev;
	data->i2c = i2c;
	i2c_set_clientdata(i2c, data);

	data->regmap = devm_regmap_init_i2c(i2c, &rt9467_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	ret = rt9467_check_vendor_info(data);
	if (ret) {
		dev_err(&i2c->dev, "Failed to check vendor info\n");
		return ret;
	}

	ret = rt9467_reset_chip(data);
	if (ret) {
		dev_err(&i2c->dev, "Failed to reset chip\n");
		return ret;
	}

	ret = rt9467_add_irq_chip(data);
	if (ret)
		return ret;

	ret = devm_mfd_add_devices(&i2c->dev, PLATFORM_DEVID_NONE, rt9467_devs,
				   ARRAY_SIZE(rt9467_devs), NULL, 0,
				   data->irq_domain);
	if (ret) {
		dev_err(&i2c->dev, "Failed to add sub devices\n");
		rt9467_del_irq_chip(data);
	}

	return 0;
}

static int rt9467_remove(struct i2c_client *i2c)
{
	struct rt9467_data *data = i2c_get_clientdata(i2c);

	rt9467_del_irq_chip(data);
	return 0;
}

static int rt9467_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(i2c->irq);

	return 0;
}

static int rt9467_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(i2c->irq);

	return 0;
}
static SIMPLE_DEV_PM_OPS(rt9467_pm_ops, rt9467_suspend, rt9467_resume);

static const struct of_device_id rt9467_of_match_table[] = {
	{ .compatible = "richtek,rt9467", },
	{}
};
MODULE_DEVICE_TABLE(of, rt9467_of_match_table);

static struct i2c_driver rt9467_driver = {
	.driver = {
		.name = "rt9467",
		.of_match_table = rt9467_of_match_table,
		.pm = &rt9467_pm_ops,
	},
	.probe_new = rt9467_probe,
	.remove = rt9467_remove,
};
module_i2c_driver(rt9467_driver);

MODULE_DESCRIPTION("Richtek rt9467 MFD driver");
MODULE_AUTHOR("ChiYuan Huang <cy_huang@richtek.com>");
MODULE_LICENSE("GPL");
