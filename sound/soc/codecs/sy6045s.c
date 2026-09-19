// SPDX-License-Identifier: GPL-2.0-only
/*
 * sy6045s.c -- Silergy SY6045S Class-D Audio Amplifier Driver
 *
 * Copyright (C) Silergy Semiconductor Corp.
 * Mainline port for Linux 6.x / modern ASoC by Reverse Engineering
 * Full functional parity with vendor kernel module snd-soc-sy6045s.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "sy6045s_firmware.h"

#define SY6045S_REG_DEV_ID          0x00
#define SY6045S_REG_REV_ID          0x01
#define SY6045S_REG_INPUT_FMT       0x04
#define SY6045S_REG_STANDBY_CTRL    0x05
#define SY6045S_REG_CH1_VOL         0x11
#define SY6045S_REG_CH2_VOL         0x12
#define SY6045S_REG_CH3_VOL         0x13
#define SY6045S_REG_CH4_VOL         0x14
#define SY6045S_REG_MUTE_CTRL       0x19
#define SY6045S_REG_SYS_RESET       0x0f

#define SY6045S_NUM_SUPPLIES        2

static const char *const sy6045s_supply_names[SY6045S_NUM_SUPPLIES] = {
	"vddio",
	"pvdd",
};

struct sy6045s_priv {
	struct regmap *regmap;
	struct snd_soc_component *component;
	struct i2c_client *i2c;
	unsigned int format;
	char default_fw_name[64];
	char last_fw_name[128];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *fault_gpio;
	struct regulator_bulk_data supplies[SY6045S_NUM_SUPPLIES];
	int num_supplies;
	struct delayed_work spk_off_work;
	struct delayed_work spk_on_work;
	struct mutex io_lock;
	bool is_pbtl;
	u8 restore_regs[256];
	size_t restore_reg_size;
};

static const struct reg_default sy6045s_reg_defaults[] = {
	{ 0x04, 0x9e },
	{ 0x05, 0x02 },
	{ 0x0f, 0x00 },
	{ 0x10, 0x77 },
	{ 0x11, 0x00 },
	{ 0x12, 0x06 },
	{ 0x13, 0x0c },
	{ 0x14, 0x12 },
	{ 0x19, 0x15 },
	{ 0x1b, 0xbd },
	{ 0x23, 0x1a },
	{ 0x76, 0x0f },
};

static bool sy6045s_readable_register(struct device *dev, unsigned int reg)
{
	return reg <= 0xff;
}

static bool sy6045s_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SY6045S_REG_DEV_ID:
	case SY6045S_REG_REV_ID:
	case SY6045S_REG_SYS_RESET:
		return true;
	default:
		return false;
	}
}

static int sy6045s_reset_chip(struct sy6045s_priv *priv)
{
	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		msleep(5);
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		msleep(10);
	}

	return regmap_write(priv->regmap, SY6045S_REG_SYS_RESET, 0x01);
}

/*
 * Parse text settings file format exported by Silergy GUI tool.
 * Lines format:
 *   # comments
 *   d 10 (delay ms)
 *   w 2b 11 00 (write i2c_addr reg val...)
 */
static int sy6045s_apply_settings(struct sy6045s_priv *priv, const u8 *data, size_t size, const char *source_name)
{
	const char *ptr = (const char *)data;
	const char *end = ptr + size;
	char line[256];
	int ret = 0;

	mutex_lock(&priv->io_lock);

	while (ptr < end) {
		size_t len = 0;
		while (ptr + len < end && ptr[len] != '\n' && ptr[len] != '\r' && len < sizeof(line) - 1)
			len++;

		if (len > 0) {
			memcpy(line, ptr, len);
			line[len] = '\0';
		} else {
			line[0] = '\0';
		}

		ptr += len;
		while (ptr < end && (*ptr == '\n' || *ptr == '\r'))
			ptr++;

		if (line[0] == '#' || line[0] == '\0')
			continue;

		if (line[0] == 'd' || line[0] == 'D') {
			unsigned int delay_ms = 0;
			if (sscanf(line + 1, "%u", &delay_ms) == 1)
				msleep(delay_ms);
			continue;
		}

		if (line[0] == 'w' || line[0] == 'W') {
			unsigned int i2c_addr, reg_addr;
			int offset = 0, nread;
			u8 bytes[64];
			int byte_count = 0;

			if (sscanf(line + 1, "%x %x%n", &i2c_addr, &reg_addr, &nread) >= 2) {
				const char *pargs = line + 1 + nread;
				unsigned int byte_val;

				while (sscanf(pargs, "%x%n", &byte_val, &offset) == 1 && byte_count < sizeof(bytes)) {
					bytes[byte_count++] = (u8)byte_val;
					pargs += offset;
				}

				if (byte_count == 1) {
					regmap_write(priv->regmap, reg_addr, bytes[0]);
				} else if (byte_count > 1) {
					regmap_raw_write(priv->regmap, reg_addr, bytes, byte_count);
				}
			}
		}
	}

	mutex_unlock(&priv->io_lock);
	dev_info(&priv->i2c->dev, "Applied settings from '%s'\n", source_name);
	return ret;
}

static int sy6045s_load_firmware(struct sy6045s_priv *priv, const char *filename)
{
	const struct firmware *fw;
	int ret;

	if (filename && filename[0]) {
		ret = request_firmware(&fw, filename, &priv->i2c->dev);
		if (ret == 0) {
			ret = sy6045s_apply_settings(priv, fw->data, fw->size, filename);
			release_firmware(fw);
			return ret;
		}
		dev_info(&priv->i2c->dev,
			 "Firmware file '%s' not found (%d), using baked-in settings\n",
			 filename, ret);
	}

	/* Fall back to baked-in vendor firmware */
	if ((filename && strstr(filename, "woofer")) || priv->i2c->addr == 0x2b) {
		return sy6045s_apply_settings(priv, sy6045s_woofer_default_settings,
					      sizeof(sy6045s_woofer_default_settings),
					      "embedded:sy6045s-woofer-settings.txt");
	} else if ((filename && strstr(filename, "tweeter")) || priv->i2c->addr == 0x2a) {
		return sy6045s_apply_settings(priv, sy6045s_tweeter_default_settings,
					      sizeof(sy6045s_tweeter_default_settings),
					      "embedded:sy6045s-tweeters-settings.txt");
	}

	dev_err(&priv->i2c->dev, "No embedded firmware found for address 0x%02x\n", priv->i2c->addr);
	return -ENOENT;
}

/* Sysfs interfaces */
static ssize_t settings_file_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", priv->last_fw_name[0] ? priv->last_fw_name : "none");
}

static ssize_t settings_file_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	char fname[128];
	int ret;

	if (count >= sizeof(fname))
		return -EINVAL;

	strscpy(fname, buf, sizeof(fname));
	strim(fname);

	ret = sy6045s_load_firmware(priv, fname);
	if (ret)
		return ret;

	strscpy(priv->last_fw_name, fname, sizeof(priv->last_fw_name));
	return count;
}
static DEVICE_ATTR_RW(settings_file);

static ssize_t default_settings_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	int ret;

	if (!priv->default_fw_name[0])
		return -ENODEV;

	sy6045s_reset_chip(priv);
	ret = sy6045s_load_firmware(priv, priv->default_fw_name);
	if (ret)
		return ret;

	strscpy(priv->last_fw_name, priv->default_fw_name, sizeof(priv->last_fw_name));
	return count;
}
static DEVICE_ATTR_WO(default_settings);

static ssize_t settings_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	sy6045s_apply_settings(priv, (const u8 *)buf, count, "sysfs:settings");
	return count;
}
static DEVICE_ATTR_WO(settings);

static struct attribute *sy6045s_attrs[] = {
	&dev_attr_settings_file.attr,
	&dev_attr_default_settings.attr,
	&dev_attr_settings.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sy6045s);

static void sy6045s_spk_on_work(struct work_struct *work)
{
	struct sy6045s_priv *priv = container_of(work, struct sy6045s_priv, spk_on_work.work);
	regmap_update_bits(priv->regmap, SY6045S_REG_STANDBY_CTRL, 0x03, 0x00);
}

static void sy6045s_spk_off_work(struct work_struct *work)
{
	struct sy6045s_priv *priv = container_of(work, struct sy6045s_priv, spk_off_work.work);
	regmap_update_bits(priv->regmap, SY6045S_REG_STANDBY_CTRL, 0x03, 0x02);
}

static const DECLARE_TLV_DB_SCALE(sy6045s_vol_tlv, -12600, 50, 0);

static const struct snd_kcontrol_new sy6045s_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Master Playback Volume", SY6045S_REG_CH1_VOL,
			 SY6045S_REG_CH2_VOL, 0, 0xff, 1, sy6045s_vol_tlv),
	SOC_SINGLE("Master Playback Switch", SY6045S_REG_MUTE_CTRL, 0, 1, 1),
};

static int sy6045s_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&priv->spk_on_work);
	cancel_delayed_work_sync(&priv->spk_off_work);

	if (mute) {
		regmap_update_bits(priv->regmap, SY6045S_REG_MUTE_CTRL, 0x01, 0x01);
		queue_delayed_work(system_power_efficient_wq, &priv->spk_off_work, msecs_to_jiffies(10));
	} else {
		queue_delayed_work(system_power_efficient_wq, &priv->spk_on_work, 0);
		regmap_update_bits(priv->regmap, SY6045S_REG_MUTE_CTRL, 0x01, 0x00);
	}
	return 0;
}

static int sy6045s_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	u8 sfmt = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		sfmt = 0x00;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		sfmt = 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		sfmt = 0x02;
		break;
	default:
		return -EINVAL;
	}

	priv->format = fmt;
	return regmap_update_bits(priv->regmap, SY6045S_REG_INPUT_FMT, 0x03, sfmt);
}

static int sy6045s_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops sy6045s_dai_ops = {
	.set_fmt	= sy6045s_set_dai_fmt,
	.hw_params	= sy6045s_hw_params,
	.mute_stream	= sy6045s_mute,
};

static struct snd_soc_dai_driver sy6045s_dai = {
	.name = "sy6045s-amplifier",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE |
			    SNDRV_PCM_FMTBIT_S24_LE |
			    SNDRV_PCM_FMTBIT_S32_LE),
	},
	.ops = &sy6045s_dai_ops,
};

static int sy6045s_probe(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	priv->component = component;
	return 0;
}

static const struct snd_soc_component_driver soc_component_dev_sy6045s = {
	.probe			= sy6045s_probe,
	.controls		= sy6045s_snd_controls,
	.num_controls		= ARRAY_SIZE(sy6045s_snd_controls),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config sy6045s_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= 0xff,
	.reg_defaults		= sy6045s_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(sy6045s_reg_defaults),
	.cache_type		= REGCACHE_MAPLE,
	.readable_reg		= sy6045s_readable_register,
	.volatile_reg		= sy6045s_volatile_register,
};

static int sy6045s_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct sy6045s_priv *priv;
	const char *dfw = NULL;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i2c = i2c;
	mutex_init(&priv->io_lock);
	INIT_DELAYED_WORK(&priv->spk_on_work, sy6045s_spk_on_work);
	INIT_DELAYED_WORK(&priv->spk_off_work, sy6045s_spk_off_work);
	i2c_set_clientdata(i2c, priv);

	for (i = 0; i < SY6045S_NUM_SUPPLIES; i++)
		priv->supplies[i].supply = sy6045s_supply_names[i];

	ret = devm_regulator_bulk_get_optional(dev, SY6045S_NUM_SUPPLIES, priv->supplies);
	if (ret > 0) {
		priv->num_supplies = ret;
		ret = regulator_bulk_enable(priv->num_supplies, priv->supplies);
		if (ret)
			dev_warn(dev, "Failed to enable supplies: %d\n", ret);
	}

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	priv->fault_gpio = devm_gpiod_get_optional(dev, "fault", GPIOD_IN);

	if (device_property_read_bool(dev, "pbtl-mode"))
		priv->is_pbtl = true;

	priv->regmap = devm_regmap_init_i2c(i2c, &sy6045s_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap), "Failed to init regmap\n");

	sy6045s_reset_chip(priv);

	if (!device_property_read_string(dev, "firmware", &dfw) ||
	    !device_property_read_string(dev, "settings-file", &dfw)) {
		strscpy(priv->default_fw_name, dfw, sizeof(priv->default_fw_name));
	} else if (i2c->addr == 0x2b) {
		strscpy(priv->default_fw_name, "sy6045s-woofer-settings.txt", sizeof(priv->default_fw_name));
	} else if (i2c->addr == 0x2a) {
		strscpy(priv->default_fw_name, "sy6045s-tweeters-settings.txt", sizeof(priv->default_fw_name));
	}

	if (priv->default_fw_name[0]) {
		ret = sy6045s_load_firmware(priv, priv->default_fw_name);
		if (ret == 0)
			strscpy(priv->last_fw_name, priv->default_fw_name, sizeof(priv->last_fw_name));
	}

	if (dev->of_node) {
		int num_restore = of_property_count_u32_elems(dev->of_node, "restore-regs");
		if (num_restore > 0 && !(num_restore % 2)) {
			u32 *regs = kmalloc_array(num_restore, sizeof(u32), GFP_KERNEL);
			if (regs) {
				if (!of_property_read_u32_array(dev->of_node, "restore-regs", regs, num_restore)) {
					for (int r = 0; r < num_restore; r += 2)
						regmap_write(priv->regmap, regs[r], regs[r + 1]);
				}
				kfree(regs);
			}
		}
	}

	return devm_snd_soc_register_component(dev, &soc_component_dev_sy6045s,
					       &sy6045s_dai, 1);
}

static void sy6045s_i2c_remove(struct i2c_client *i2c)
{
	struct sy6045s_priv *priv = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&priv->spk_on_work);
	cancel_delayed_work_sync(&priv->spk_off_work);

	if (priv->num_supplies > 0)
		regulator_bulk_disable(priv->num_supplies, priv->supplies);
}

static const struct i2c_device_id sy6045s_i2c_id[] = {
	{ "sy6045s", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sy6045s_i2c_id);

static const struct of_device_id sy6045s_of_match[] = {
	{ .compatible = "silergy,sy6045s" },
	{ }
};
MODULE_DEVICE_TABLE(of, sy6045s_of_match);

static struct i2c_driver sy6045s_i2c_driver = {
	.driver = {
		.name = "sy6045s",
		.of_match_table = sy6045s_of_match,
		.dev_groups = sy6045s_groups,
	},
	.probe = sy6045s_i2c_probe,
	.remove = sy6045s_i2c_remove,
	.id_table = sy6045s_i2c_id,
};
module_i2c_driver(sy6045s_i2c_driver);

MODULE_DESCRIPTION("ASoC Silergy SY6045S audio amplifier driver");
MODULE_AUTHOR("Silergy Corp.");
MODULE_AUTHOR("Mainline Linux port for Yandex Station Max / AtriStation");
MODULE_LICENSE("GPL v2");
