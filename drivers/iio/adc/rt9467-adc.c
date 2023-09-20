// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <dt-bindings/iio/adc/richtek,rt9467-adc.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>

#define RT9467_REG_CHG_ADC		0x11
#define RT9467_REG_ADC_DATA_H		0x44
#define RT9467_REG_ADC_DATA_L		0x45

#define RT9467_MASK_ADC_IN_SEL		GENMASK(7, 4)
#define RT9467_SHFT_ADC_IN_SEL		4
#define RT9467_MASK_ADC_START		BIT(0)

struct rt9467_adc_data {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
};

static int rt9467_read_channel(struct rt9467_adc_data *data,
			       const struct iio_chan_spec *chan, int *val)
{
	int ret;
	__be16 chan_raw_data;
	unsigned int adc_stat;

	mutex_lock(&data->lock);

	ret = regmap_write(data->regmap, RT9467_REG_CHG_ADC, 0);
	if (ret) {
		dev_err(data->dev, "Failed to clear channels\n");
		goto out;
	}

	/*
	 * 1. Configure the desired ADC channel
	 * 2. Start ADC conversion
	 */
	ret = regmap_update_bits(data->regmap, RT9467_REG_CHG_ADC,
				 RT9467_MASK_ADC_IN_SEL,
				 chan->scan_index << RT9467_SHFT_ADC_IN_SEL);
	ret |= regmap_update_bits(data->regmap, RT9467_REG_CHG_ADC,
				  RT9467_MASK_ADC_START, RT9467_MASK_ADC_START);
	if (ret) {
		dev_err(data->dev, "Failed to configure ADC (%d)\n",
			chan->scan_index);
		goto out;
	}

	msleep(35);
	ret = regmap_read_poll_timeout(data->regmap, RT9467_REG_CHG_ADC,
				       adc_stat,
				       !(adc_stat & RT9467_MASK_ADC_START),
				       1000, 35000);
	if (ret == -ETIMEDOUT) {
		dev_warn(data->dev, "Wait con failed, sel = %d\n",
			 chan->scan_index);
		goto out;
	}

	ret = regmap_raw_read(data->regmap, chan->address,
			      &chan_raw_data, sizeof(chan_raw_data));
	if (ret) {
		dev_err(data->dev, "Failed to read chan%d raw data\n",
			chan->scan_index);
		goto out;
	}

	*val = be16_to_cpu(chan_raw_data);
	ret = IIO_VAL_INT;

out:
	mutex_unlock(&data->lock);
	return ret;
}

static int rt9467_read_scale(struct rt9467_adc_data *data,
			     const struct iio_chan_spec *chan, int *val,
			     int *val2)
{
	switch (chan->channel) {
	case RT9467_CHAN_VBUS_DIV5:
		*val = 25000;
		return IIO_VAL_INT;
	case RT9467_CHAN_VBUS_DIV2:
		*val = 10000;
		return IIO_VAL_INT;
	case RT9467_CHAN_VBAT:
	case RT9467_CHAN_VSYS:
	case RT9467_CHAN_REGN:
		*val = 5000;
		return IIO_VAL_INT;
	case RT9467_CHAN_TS_BAT:
		*val = 25;
		return IIO_VAL_INT;
	case RT9467_CHAN_IBUS:
	case RT9467_CHAN_IBAT:
		*val = 50000;
		return IIO_VAL_INT;
	case RT9467_CHAN_TEMP_JC:
		*val = 2;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int rt9467_read_offset(struct rt9467_adc_data *data,
			      const struct iio_chan_spec *chan, int *val)
{
	*val = chan->channel == RT9467_CHAN_TEMP_JC ? -40 : 0;
	return IIO_VAL_INT;
}

static int rt9467_adc_read_raw(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan, int *val,
			       int *val2, long mask)
{
	struct rt9467_adc_data *data = iio_priv(iio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return rt9467_read_channel(data, chan, val);
	case IIO_CHAN_INFO_SCALE:
		return rt9467_read_scale(data, chan, val, val2);
	case IIO_CHAN_INFO_OFFSET:
		return rt9467_read_offset(data, chan, val);
	}

	return -EINVAL;
}

static const struct iio_info rt9467_adc_info = {
	.read_raw = rt9467_adc_read_raw,
};

#define RT9467_ADC_CHANNEL(_ch_name, _sc_idx, _type) \
{ \
	.type = _type, \
	.channel = RT9467_CHAN_##_ch_name, \
	.datasheet_name = #_ch_name, \
	.address = RT9467_REG_ADC_DATA_H, \
	.scan_index = _sc_idx, \
	.indexed = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			      BIT(IIO_CHAN_INFO_SCALE) | \
			      BIT(IIO_CHAN_INFO_OFFSET), \
}

static const struct iio_chan_spec rt9467_adc_channels[] = {
	RT9467_ADC_CHANNEL(VBUS_DIV5, 1, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(VBUS_DIV2, 2, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(VSYS, 3, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(VBAT, 4, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(TS_BAT, 6, IIO_TEMP),
	RT9467_ADC_CHANNEL(IBUS, 8, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(IBAT, 9, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(REGN, 11, IIO_VOLTAGE),
	RT9467_ADC_CHANNEL(TEMP_JC, 12, IIO_TEMP),
};

static int rt9467_adc_probe(struct platform_device *pdev)
{
	struct rt9467_adc_data *data;
	struct iio_dev *iio_dev;
	int ret;

	iio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (IS_ERR(iio_dev)) {
		dev_err(&pdev->dev, "Failed to allocate iio device\n");
		return PTR_ERR(iio_dev);
	}

	data = iio_priv(iio_dev);
	data->dev = &pdev->dev;
	mutex_init(&data->lock);

	data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!data->regmap) {
		dev_err(&pdev->dev, "Failed to get parent regmap\n");
		return -ENODEV;
	}

	ret = regmap_write(data->regmap, RT9467_REG_CHG_ADC, 0x00);
	if (ret) {
		dev_err(&pdev->dev, "Failed to reset adc\n");
		return ret;
	}

	iio_dev->name = dev_name(&pdev->dev);
	iio_dev->info = &rt9467_adc_info;
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->channels = rt9467_adc_channels;
	iio_dev->num_channels = ARRAY_SIZE(rt9467_adc_channels);

	return devm_iio_device_register(&pdev->dev, iio_dev);
}

static const struct of_device_id rt9467_adc_of_match_table[] = {
	{ .compatible = "richtek,rt9467-adc", },
	{}
};
MODULE_DEVICE_TABLE(of, rt9467_adc_of_match_table);

static struct platform_driver rt9467_adc_driver = {
	.driver = {
		.name = "rt9467-adc",
		.of_match_table = rt9467_adc_of_match_table,
	},
	.probe = rt9467_adc_probe,
};
module_platform_driver(rt9467_adc_driver);

MODULE_DESCRIPTION("Richtek RT9467 ADC driver");
MODULE_AUTHOR("ChiYuan Huang <cy_huang@richtek.com>");
MODULE_LICENSE("GPL");
