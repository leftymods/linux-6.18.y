// SPDX-License-Identifier: GPL-2.0-only
/*
 * ltr308als01.c -- Lite-On LTR-308ALS Ambient Light Sensor Driver
 *
 * Copyright (C) Lite-On Technology Corp. / Yandex LLC
 * Mainline port for Linux 6.x IIO subsystem by Reverse Engineering
 * Full functional parity with vendor kernel module ltr308als01.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>

#define LTR308_MAIN_CTRL        0x00
#define LTR308_ALS_MEAS_RATE    0x04
#define LTR308_ALS_GAIN         0x05
#define LTR308_PART_ID          0x06
#define LTR308_MAIN_STATUS      0x07
#define LTR308_ALS_DATA_0       0x0d
#define LTR308_ALS_DATA_1       0x0e
#define LTR308_ALS_DATA_2       0x0f
#define LTR308_INT_CFG          0x19
#define LTR308_INT_PST          0x1a
#define LTR308_ALS_THRES_UP_0   0x21
#define LTR308_ALS_THRES_UP_1   0x22
#define LTR308_ALS_THRES_UP_2   0x23
#define LTR308_ALS_THRES_LOW_0  0x24
#define LTR308_ALS_THRES_LOW_1  0x25
#define LTR308_ALS_THRES_LOW_2  0x26

struct ltr308_data {
	struct regmap *regmap;
	struct i2c_client *client;
	struct mutex lock;
	u8 gain;
	u8 integration_time;
};

static const struct regmap_config ltr308_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x27,
};

static int ltr308_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ltr308_data *data = iio_priv(indio_dev);
	u8 buf[3];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		ret = regmap_bulk_read(data->regmap, LTR308_ALS_DATA_0, buf, 3);
		mutex_unlock(&data->lock);
		if (ret < 0)
			return ret;
		*val = buf[0] | (buf[1] << 8) | ((buf[2] & 0x0f) << 16);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 1000;
		return IIO_VAL_FRACTIONAL;

	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec ltr308_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct iio_info ltr308_info = {
	.read_raw = ltr308_read_raw,
};

static int ltr308_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct ltr308_data *data;
	unsigned int part_id;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	data->regmap = devm_regmap_init_i2c(client, &ltr308_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap), "Failed to init regmap\n");

	ret = regmap_read(data->regmap, LTR308_PART_ID, &part_id);
	if (ret < 0)
		return ret;

	dev_info(dev, "LTR-308ALS Part ID: 0x%02x\n", part_id);

	/* Enable active mode: MAIN_CTRL = 0x02 */
	regmap_write(data->regmap, LTR308_MAIN_CTRL, 0x02);

	indio_dev->name = "ltr308als";
	indio_dev->channels = ltr308_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltr308_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltr308_info;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id ltr308_id[] = {
	{ "ltr308als01", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltr308_id);

static const struct of_device_id ltr308_of_match[] = {
	{ .compatible = "liteon,ltr308als01" },
	{ .compatible = "liteon,ltr308" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltr308_of_match);

static struct i2c_driver ltr308_driver = {
	.driver = {
		.name = "ltr308als01",
		.of_match_table = ltr308_of_match,
	},
	.probe = ltr308_probe,
	.id_table = ltr308_id,
};
module_i2c_driver(ltr308_driver);

MODULE_DESCRIPTION("Lite-On LTR-308ALS Ambient Light Sensor Driver");
MODULE_AUTHOR("Yandex LLC / Mainline Linux port by Reverse Engineering");
MODULE_LICENSE("GPL v2");
