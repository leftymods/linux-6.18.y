// SPDX-License-Identifier: GPL-2.0-only
/*
 * zigbee_control.c -- Zigbee Hardware Management Driver for Yandex Station Max / AtriStation
 *
 * Copyright (C) Yandex LLC
 * Complete mainline Linux port matching the exact Yandex vendor zigbee-control.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/of.h>

struct zigbee_control_data {
	struct device *dev;
	struct gpio_desc *power;
	struct gpio_desc *reset;
	struct gpio_desc *boot;
};

static ssize_t power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);
	int val = priv->power ? gpiod_get_value_cansleep(priv->power) : 1;
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

	if (priv->power)
		gpiod_set_value_cansleep(priv->power, state ? 1 : 0);

	return count;
}
static DEVICE_ATTR_RW(power);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);

	if (!priv->reset)
		return -ENODEV;

	/* Pulse reset for 50ms */
	gpiod_set_value_cansleep(priv->reset, 1);
	msleep(50);
	gpiod_set_value_cansleep(priv->reset, 0);
	msleep(50);

	return count;
}
static DEVICE_ATTR_WO(reset);

static ssize_t boot_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct zigbee_control_data *priv = dev_get_drvdata(dev);
	int val = priv->boot ? gpiod_get_value_cansleep(priv->boot) : 0;
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

	if (priv->boot)
		gpiod_set_value_cansleep(priv->boot, state ? 1 : 0);

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

static int zb_claim_gpio(struct device *dev, const char *name, const char *alt_name,
			 enum gpiod_flags flags, struct gpio_desc **pdesc)
{
	struct gpio_desc *desc;
	int ret;

	desc = devm_gpiod_get_optional(dev, name, flags);
	if (!desc && alt_name)
		desc = devm_gpiod_get_optional(dev, alt_name, flags);

	if (IS_ERR(desc)) {
		dev_err(dev, "Can't claim GPIO '%s'\n", name);
		return PTR_ERR(desc);
	}

	if (!desc)
		return 0;

	ret = gpiod_export(desc, false);
	if (ret)
		dev_err(dev, "Can't export GPIO '%s'\n", name);

	ret = gpiod_export_link(dev, name, desc);
	if (ret)
		dev_err(dev, "Can't create sysfs link for GPIO '%s'\n", name);

	*pdesc = desc;
	return 0;
}

static void zb_unclaim_gpio(struct device *dev, const char *name, struct gpio_desc *desc)
{
	if (!desc)
		return;

	sysfs_remove_link(&dev->kobj, name);
	gpiod_unexport(desc);
}

static int zigbee_control_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zigbee_control_data *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "failed to allocate zb_control\n");
		return -ENOMEM;
	}

	priv->dev = dev;

	/* zb-power: optional power enable line */
	ret = zb_claim_gpio(dev, "zb-power", "power", GPIOD_OUT_HIGH, &priv->power);
	if (ret)
		return ret;

	/* zb-reset: reset line (asserted low) */
	ret = zb_claim_gpio(dev, "zb-reset", "reset", GPIOD_OUT_LOW, &priv->reset);
	if (ret)
		goto err_power;

	/* zb-boot: bootloader entry line */
	ret = zb_claim_gpio(dev, "zb-boot", "boot", GPIOD_OUT_LOW, &priv->boot);
	if (ret)
		goto err_reset;

	platform_set_drvdata(pdev, priv);
	dev_info(dev, "zigbee-control driver registered\n");
	return 0;

err_reset:
	zb_unclaim_gpio(dev, "zb-reset", priv->reset);
err_power:
	zb_unclaim_gpio(dev, "zb-power", priv->power);
	return ret;
}

static void zigbee_control_remove(struct platform_device *pdev)
{
	struct zigbee_control_data *priv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	zb_unclaim_gpio(dev, "zb-boot", priv->boot);
	zb_unclaim_gpio(dev, "zb-reset", priv->reset);
	zb_unclaim_gpio(dev, "zb-power", priv->power);
}

static const struct of_device_id zigbee_control_of_match[] = {
	{ .compatible = "yandex,zigbee-control" },
	{ .compatible = "zigbee-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, zigbee_control_of_match);

static struct platform_driver zigbee_control_driver = {
	.probe = zigbee_control_probe,
	.remove = zigbee_control_remove,
	.driver = {
		.name = "zigbee_control",
		.of_match_table = zigbee_control_of_match,
		.dev_groups = zigbee_control_groups,
	},
};
module_platform_driver(zigbee_control_driver);

MODULE_DESCRIPTION("ZigBee sysfs controls");
MODULE_AUTHOR("Yandex LLC");
MODULE_AUTHOR("Mainline Linux port by Reverse Engineering");
MODULE_LICENSE("GPL v2");
