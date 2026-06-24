// SPDX-License-Identifier: GPL-2.0-only
/*
 * AtriStation Zigbee radio power/reset control
 *
 * Based on Yandex zigbee-control.ko for Yandex Station (4.9 kernel).
 * Rewritten for mainline 6.18.y using GPIO descriptor API.
 *
 * Provides sysfs interface for Zigbee module power/reset GPIO control.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/device.h>

struct zigbee_control_data {
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;
};

static ssize_t power_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct zigbee_control_data *data = dev_get_drvdata(dev);

	if (!data->power_gpio)
		return sysfs_emit(buf, "unavailable\n");

	return sysfs_emit(buf, "%d\n", gpiod_get_value(data->power_gpio));
}

static ssize_t power_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct zigbee_control_data *data = dev_get_drvdata(dev);
	bool val;

	if (!data->power_gpio)
		return -ENODEV;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	gpiod_set_value(data->power_gpio, val);
	return count;
}
static DEVICE_ATTR_RW(power);

static ssize_t reset_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct zigbee_control_data *data = dev_get_drvdata(dev);

	if (!data->reset_gpio)
		return sysfs_emit(buf, "unavailable\n");

	return sysfs_emit(buf, "%d\n", gpiod_get_value(data->reset_gpio));
}

static ssize_t reset_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct zigbee_control_data *data = dev_get_drvdata(dev);
	bool val;

	if (!data->reset_gpio)
		return -ENODEV;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	gpiod_set_value(data->reset_gpio, val);
	return count;
}
static DEVICE_ATTR_RW(reset);

static struct attribute *zigbee_control_attrs[] = {
	&dev_attr_power.attr,
	&dev_attr_reset.attr,
	NULL,
};
ATTRIBUTE_GROUPS(zigbee_control);

static int zigbee_control_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zigbee_control_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(data->power_gpio))
		return dev_err_probe(dev, PTR_ERR(data->power_gpio),
				     "failed to get power-gpios\n");

	data->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(data->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(data->reset_gpio),
				     "failed to get reset-gpios\n");

	if (!data->power_gpio && !data->reset_gpio)
		return dev_err_probe(dev, -ENODEV,
				     "no power-gpios or reset-gpios specified\n");

	platform_set_drvdata(pdev, data);

	dev_info(dev, "Zigbee control initialized\n");
	return 0;
}

static void zigbee_control_remove(struct platform_device *pdev)
{
	struct zigbee_control_data *data = platform_get_drvdata(pdev);

	if (data->power_gpio)
		gpiod_set_value(data->power_gpio, 0);

	if (data->reset_gpio)
		gpiod_set_value(data->reset_gpio, 1);
}

static const struct of_device_id zigbee_control_of_match[] = {
	{ .compatible = "yandex,zigbee-control", },
	{ }
};
MODULE_DEVICE_TABLE(of, zigbee_control_of_match);

static struct platform_driver zigbee_control_driver = {
	.probe = zigbee_control_probe,
	.remove = zigbee_control_remove,
	.driver = {
		.name = "zigbee-control",
		.of_match_table = zigbee_control_of_match,
		.dev_groups = zigbee_control_groups,
	},
};
module_platform_driver(zigbee_control_driver);

MODULE_AUTHOR("AtriMods");
MODULE_DESCRIPTION("Zigbee radio power/reset control for AtriStation");
MODULE_LICENSE("GPL");
