// SPDX-License-Identifier: GPL-2.0-only
/*
 * zigbee_control.c -- Zigbee Hardware Management Driver for Yandex Station Max
 *
 * Copyright (C) Yandex Inc.
 * Ported to Mainline Linux 6.x by Reverse Engineering
 * Full functional parity with vendor kernel module zigbee-control.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/of.h>

struct zigbee_control_data {
	struct device *dev;
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *boot_gpio;
};

static ssize_t power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);
	int val = priv->power_gpio ? gpiod_get_value_cansleep(priv->power_gpio) : 0;
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t power_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);
	bool state;
	int ret;

	ret = kstrtobool(buf, &state);
	if (ret)
		return ret;

	if (priv->power_gpio)
		gpiod_set_value_cansleep(priv->power_gpio, state ? 1 : 0);

	return count;
}
static DEVICE_ATTR_RW(power);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);

	if (!priv->reset_gpio)
		return -ENODEV;

	/* Pulse reset low for 50ms */
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	msleep(50);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	msleep(50);

	return count;
}
static DEVICE_ATTR_WO(reset);

static ssize_t boot_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);
	int val = priv->boot_gpio ? gpiod_get_value_cansleep(priv->boot_gpio) : 0;
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t boot_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);
	bool state;
	int ret;

	ret = kstrtobool(buf, &state);
	if (ret)
		return ret;

	if (priv->boot_gpio)
		gpiod_set_value_cansleep(priv->boot_gpio, state ? 1 : 0);

	return count;
}
static DEVICE_ATTR_RW(boot);

static struct attribute *zigbee_control_attrs[] = {
	&dev_attr_power.attr,
	&dev_attr_reset.attr,
	&dev_attr_boot.attr,
	NULL,
};
ATTRIBUTE_GROUPS(zigbee_control);

static int zigbee_control_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zigbee_control_data *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->power_gpio = devm_gpiod_get_optional(dev, "zb-power", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->power_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->power_gpio),
				      "Failed to acquire zb-power GPIO\n");

	priv->reset_gpio = devm_gpiod_get_optional(dev, "zb-reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				      "Failed to acquire zb-reset GPIO\n");

	priv->boot_gpio = devm_gpiod_get_optional(dev, "zb-boot", GPIOD_OUT_LOW);
	if (IS_ERR(priv->boot_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->boot_gpio),
				      "Failed to acquire zb-boot GPIO\n");

	platform_set_drvdata(pdev, priv);
	dev_info(dev, "Zigbee control driver probed successfully\n");

	return 0;
}

static const struct of_device_id zigbee_control_of_match[] = {
	{ .compatible = "yandex,zigbee-control" },
	{ .compatible = "zigbee-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, zigbee_control_of_match);

static struct platform_driver zigbee_control_driver = {
	.probe = zigbee_control_probe,
	.driver = {
		.name = "zigbee_control",
		.of_match_table = zigbee_control_of_match,
		.dev_groups = zigbee_control_groups,
	},
};
module_platform_driver(zigbee_control_driver);

MODULE_DESCRIPTION("Silicon Labs Zigbee Control Driver for Yandex Station Max");
MODULE_AUTHOR("Yandex LLC");
MODULE_AUTHOR("Mainline Linux port by Reverse Engineering");
MODULE_LICENSE("GPL v2");
