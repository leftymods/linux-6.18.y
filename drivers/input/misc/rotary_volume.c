// SPDX-License-Identifier: GPL-2.0-only
/*
 * rotary_volume.c -- GPIO Rotary Volume Control Driver for Yandex Station Max
 *
 * Copyright (C) Yandex Inc.
 * Ported to Mainline Linux 6.x by Reverse Engineering
 * Full functional parity with vendor kernel module rotary-volume.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/workqueue.h>

struct rotary_volume_data {
	struct device *dev;
	struct input_dev *input;
	struct gpio_descs *gpios;
	int irq_a;
	int irq_b;
	u8 last_state;
	struct delayed_work dwork;
};

/* 2-bit Gray code state transition table */
static const s8 rotary_states[] = {
	0, -1,  1,  0,
	1,  0,  0, -1,
       -1,  0,  0,  1,
	0,  1, -1,  0
};

static u8 rotary_volume_get_state(struct rotary_volume_data *priv)
{
	int val_a = gpiod_get_value_cansleep(priv->gpios->desc[0]);
	int val_b = gpiod_get_value_cansleep(priv->gpios->desc[1]);
	return ((val_a > 0 ? 1 : 0) << 1) | (val_b > 0 ? 1 : 0);
}

static irqreturn_t rotary_volume_irq(int irq, void *dev_id)
{
	struct rotary_volume_data *priv = dev_id;
	u8 cur_state = rotary_volume_get_state(priv);
	u8 index = (priv->last_state << 2) | cur_state;
	s8 delta = rotary_states[index & 0x0f];

	priv->last_state = cur_state;

	if (delta > 0) {
		input_report_key(priv->input, KEY_VOLUMEUP, 1);
		input_sync(priv->input);
		input_report_key(priv->input, KEY_VOLUMEUP, 0);
		input_sync(priv->input);
	} else if (delta < 0) {
		input_report_key(priv->input, KEY_VOLUMEDOWN, 1);
		input_sync(priv->input);
		input_report_key(priv->input, KEY_VOLUMEDOWN, 0);
		input_sync(priv->input);
	}

	return IRQ_HANDLED;
}

static int rotary_volume_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rotary_volume_data *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->gpios = devm_gpiod_get_array(dev, "rotary", GPIOD_IN);
	if (IS_ERR(priv->gpios)) {
		priv->gpios = devm_gpiod_get_array(dev, NULL, GPIOD_IN);
		if (IS_ERR(priv->gpios))
			return dev_err_probe(dev, PTR_ERR(priv->gpios),
					      "Failed to acquire rotary GPIOs\n");
	}

	if (priv->gpios->ndescs < 2) {
		dev_err(dev, "At least 2 GPIOs are required for rotary encoder\n");
		return -EINVAL;
	}

	priv->input = devm_input_allocate_device(dev);
	if (!priv->input)
		return -ENOMEM;

	priv->input->name = "Yandex Station Rotary Volume";
	priv->input->id.bustype = BUS_HOST;

	input_set_capability(priv->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(priv->input, EV_KEY, KEY_VOLUMEDOWN);

	ret = input_register_device(priv->input);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register input device\n");

	priv->last_state = rotary_volume_get_state(priv);

	priv->irq_a = gpiod_to_irq(priv->gpios->desc[0]);
	if (priv->irq_a < 0)
		return dev_err_probe(dev, priv->irq_a, "Failed to map IRQ for GPIO A\n");

	ret = devm_request_threaded_irq(dev, priv->irq_a, NULL, rotary_volume_irq,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"rotary_vol_a", priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ for GPIO A\n");

	priv->irq_b = gpiod_to_irq(priv->gpios->desc[1]);
	if (priv->irq_b < 0)
		return dev_err_probe(dev, priv->irq_b, "Failed to map IRQ for GPIO B\n");

	ret = devm_request_threaded_irq(dev, priv->irq_b, NULL, rotary_volume_irq,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"rotary_vol_b", priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ for GPIO B\n");

	platform_set_drvdata(pdev, priv);
	dev_info(dev, "Rotary volume driver initialized successfully\n");

	return 0;
}

static const struct of_device_id rotary_volume_of_match[] = {
	{ .compatible = "rotary-volume" },
	{ .compatible = "yandex,rotary-volume" },
	{ }
};
MODULE_DEVICE_TABLE(of, rotary_volume_of_match);

static struct platform_driver rotary_volume_driver = {
	.probe = rotary_volume_probe,
	.driver = {
		.name = "rotary-volume",
		.of_match_table = rotary_volume_of_match,
	},
};
module_platform_driver(rotary_volume_driver);

MODULE_DESCRIPTION("GPIO Rotary Volume Encoder Driver for Yandex Station Max");
MODULE_AUTHOR("Yandex LLC");
MODULE_AUTHOR("Mainline Linux port by Reverse Engineering");
MODULE_LICENSE("GPL v2");
