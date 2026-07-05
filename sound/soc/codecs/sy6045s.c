// SPDX-License-Identifier: GPL-2.0-only
/*
 * SY6045S Class-D audio amplifier driver
 *
 * Supports DT properties from original Silergy vendor driver (bin.zhang@silergycorp.com):
 *   restore-regs = <addr val addr val ...>
 *   firmware = "sy6045s-woofer-settings.txt"
 *   reset-gpios = <&gpio PHANDLE GPIO_ACTIVE_LOW>;
 *   pbtl-mode
 *
 * Ported from Yandex Station 2 (yandex_1.99.img) vendor kernel module
 * snd-soc-sy6045s.ko and expanded for mainline Linux 6.18 compatibility.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define SY6045S_NUM_SUPPLIES 2
#define SY6045S_MAX_REGISTER 0x1f
#define SY6045S_RESTORE_REGS_MAX 32

static const char *const sy6045s_supply_names[SY6045S_NUM_SUPPLIES] = {
	"vddio",
	"pvdd",
};

struct sy6045s_priv {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[SY6045S_NUM_SUPPLIES];
	struct mutex io_mutex;
	struct gpio_desc *reset_gpio;
	bool pbtl_mode;

	/* restore-regs */
	u32 restore_regs[SY6045S_RESTORE_REGS_MAX];
	int num_restore_regs;

	/* firmware name */
	const char *fw_name;
};

static const struct reg_default sy6045s_reg_defaults[] = {
	{ 0x00, 0x00 },
};

static bool sy6045s_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg <= 0x1f;
}

static bool sy6045s_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= 0x1f;
}

static bool sy6045s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config sy6045s_regmap = {
	.reg_bits     = 8,
	.val_bits     = 8,
	.max_register = SY6045S_MAX_REGISTER,
	.writeable_reg = sy6045s_writeable_reg,
	.readable_reg  = sy6045s_readable_reg,
	.volatile_reg  = sy6045s_volatile_reg,
	.cache_type    = REGCACHE_MAPLE,
};

static DECLARE_TLV_DB_SCALE(sy6045s_vol_tlv, -12600, 50, 0);
static DECLARE_TLV_DB_SCALE(sy6045s_ch_vol_tlv, -10050, 50, 0);

static const struct snd_kcontrol_new sy6045s_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Master Playback Volume",
			 0x07, 0x07, 0, 0xff, 0, sy6045s_vol_tlv),
	SOC_SINGLE_R_TLV("Ch1 Playback Volume",
			 0x08, 0, 0xff, 0, sy6045s_ch_vol_tlv),
	SOC_SINGLE_R_TLV("Ch2 Playback Volume",
			 0x09, 0, 0xff, 0, sy6045s_ch_vol_tlv),
};

static const struct snd_soc_dapm_widget sy6045s_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),
	SND_SOC_DAPM_AIF_IN("Playback", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route sy6045s_dapm_routes[] = {
	{ "SPK_OUT", NULL, "Playback" },
};

/* -------- sysfs: default_settings ------------- */
static ssize_t default_settings_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR_WO(default_settings);

/* -------- sysfs: settings_file ------------- */
static ssize_t settings_file_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	size_t len;

	mutex_lock(&priv->io_mutex);
	if (!priv->fw_name)
		len = sysfs_emit(buf, "\n");
	else
		len = sysfs_emit(buf, "%s\n", priv->fw_name);
	mutex_unlock(&priv->io_mutex);

	return len;
}

static ssize_t settings_file_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR_RW(settings_file);

/* -------- sysfs: settings ------------- */
static ssize_t settings_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR_WO(settings);

static struct attribute *sy6045s_attrs[] = {
	&dev_attr_default_settings.attr,
	&dev_attr_settings_file.attr,
	&dev_attr_settings.attr,
	NULL,
};

static const struct attribute_group sy6045s_attrs_group = {
	.name  = "amplifier",
	.attrs = sy6045s_attrs,
};

/* -------- restore-regs DT parsing ------------- */
static int sy6045s_parse_restore_regs(struct device *dev,
				      struct sy6045s_priv *priv)
{
	int num_elems, num_pairs;

	num_elems = of_property_count_u32_elems(dev->of_node, "restore-regs");
	if (num_elems <= 0)
		return 0;

	if (num_elems % 2) {
		dev_err(dev, "restore-regs must have even number of elements\n");
		return -EINVAL;
	}

	num_pairs = num_elems / 2;

	if (num_pairs * 2 > SY6045S_RESTORE_REGS_MAX) {
		dev_err(dev, "restore-regs too large (max %d values)\n",
			SY6045S_RESTORE_REGS_MAX);
		return -EINVAL;
	}

	of_property_read_u32_array(dev->of_node, "restore-regs",
				   priv->restore_regs, num_elems);

	priv->num_restore_regs = num_pairs;

	return 0;
}

static int sy6045s_restore_single_reg(struct sy6045s_priv *priv,
				      u32 addr, u32 val)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "restore reg: addr=0x%02x val=0x%02x\n",
		addr, val);

	ret = regmap_write(priv->regmap, addr, val);
	if (ret)
		dev_err(&priv->i2c->dev,
			"failed to restore reg 0x%02x: %d\n", addr, ret);

	return ret;
}

static int sy6045s_restore_regs(struct sy6045s_priv *priv)
{
	int i, ret;

	if (!priv->num_restore_regs)
		return 0;

	for (i = 0; i < priv->num_restore_regs; i++) {
		u32 addr = priv->restore_regs[i * 2];
		u32 val  = priv->restore_regs[i * 2 + 1];

		ret = sy6045s_restore_single_reg(priv, addr, val);
		if (ret)
			return ret;
	}

	return 0;
}

/* -------- reset chip via GPIO ------------- */
static int sy6045s_reset_chip(struct sy6045s_priv *priv)
{
	if (!priv->reset_gpio)
		return 0;

	gpiod_set_value(priv->reset_gpio, 1);
	msleep(1);
	gpiod_set_value(priv->reset_gpio, 0);
	msleep(1);
	gpiod_set_value(priv->reset_gpio, 1);

	dev_dbg(&priv->i2c->dev, "chip reset completed\n");

	return 0;
}

/* -------- firmware loading + settings parser ------------- */
static int sy6045s_apply_settings(struct sy6045s_priv *priv,
				  const u8 *data, size_t size)
{
	unsigned int line_num = 0;

	while (size > 0) {
		unsigned int i2c_addr, reg_addr, val;
		char cmd;
		int consumed;

		line_num++;

		/* skip whitespace and comments */
		while (size > 0 && (*data == ' ' || *data == '\t'))
			data++, size--;

		/* skip newlines */
		while (size > 0 && (*data == '\n' || *data == '\r'))
			data++, size--;

		if (size == 0)
			break;

		/* skip comment lines */
		if (*data == '#') {
			while (size > 0 && *data != '\n' && *data != '\r')
				data++, size--;
			continue;
		}

		/* parse "w <i2c_addr> <reg> <val> [<val2> ...]" */
		if (sscanf(data, " %c %x %x %x%n",
			   &cmd, &i2c_addr, &reg_addr, &val, &consumed) < 4) {
			dev_err(&priv->i2c->dev,
				"Error parsing settings at line %d\n", line_num);
			return -EINVAL;
		}

		if (cmd != 'w' && cmd != 'r') {
			dev_err(&priv->i2c->dev,
				"Unknown command '%c' at line %d\n", cmd, line_num);
			return -EINVAL;
		}

		if (cmd == 'w') {
			/* verify I2C address matches our chip
			 * (7-bit addr << 1 with write bit) */
			unsigned int our_addr = (priv->i2c->addr << 1);

			if (i2c_addr != our_addr) {
				dev_err(&priv->i2c->dev,
					"Settings not compatible with amplifier "
					"(i2c addr mismatch: fw=0x%02x, chip=0x%02x)\n",
					i2c_addr, our_addr);
				return -EINVAL;
			}

			dev_dbg(&priv->i2c->dev,
				"fw: reg=0x%02x val=0x%02x\n", reg_addr, val);
			regmap_write(priv->regmap, reg_addr, val);

			/* Check if more bytes follow on the same line */
			while (consumed < (int)size &&
			       *data == ' ') {
				unsigned int extra_val;

				consumed++;
				data++, size--;
				if (sscanf(data, "%x%n", &extra_val, &extra_val) < 1)
					break;
				/* For multi-byte writes, use next reg address */
				reg_addr++;
				dev_dbg(&priv->i2c->dev,
					"fw: reg=0x%02x val=0x%02x\n",
					reg_addr, extra_val);
				regmap_write(priv->regmap, reg_addr, extra_val);
				data += extra_val;
				size -= extra_val;

				while (size > 0 && (*data == ' ' || *data == '\t'))
					data++, size--;
			}
		}

		/* advance past this line to next */
		while (size > 0 && *data != '\n' && *data != '\r')
			data++, size--;
	}

	dev_info(&priv->i2c->dev, "loaded %u lines of firmware settings\n",
		 line_num);
	return 0;
}

static int sy6045s_load_firmware(struct sy6045s_priv *priv)
{
	const struct firmware *fw;
	int ret;

	if (!priv->fw_name)
		return 0;

	ret = request_firmware(&fw, priv->fw_name, &priv->i2c->dev);
	if (ret) {
		dev_err(&priv->i2c->dev,
			"failed to load firmware '%s': %d\n",
			priv->fw_name, ret);
		return ret;
	}

	ret = sy6045s_apply_settings(priv, fw->data, fw->size);

	release_firmware(fw);

	return ret;
}

/* -------- component probe ------------- */
static int sy6045s_component_probe(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regulator_bulk_enable(SY6045S_NUM_SUPPLIES, priv->supplies);
	if (ret)
		return ret;

	/* Reset chip if GPIO provided */
	sy6045s_reset_chip(priv);

	/* Apply restore-regs from DT */
	ret = sy6045s_restore_regs(priv);
	if (ret) {
		dev_err(component->dev,
			"failed to apply restore-regs: %d\n", ret);
		return ret;
	}

	/* Load firmware settings */
	ret = sy6045s_load_firmware(priv);
	if (ret)
		dev_warn(component->dev,
			 "firmware load failed: %d\n", ret);

	return 0;
}

static void sy6045s_component_remove(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	regulator_bulk_disable(SY6045S_NUM_SUPPLIES, priv->supplies);
}

static int sy6045s_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	mutex_lock(&priv->io_mutex);

	if (mute)
		regmap_write(priv->regmap, 0x06, 0x08);
	else
		regmap_write(priv->regmap, 0x06, 0x00);

	mutex_unlock(&priv->io_mutex);

	return 0;
}

static int sy6045s_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	return 0;
}

static int sy6045s_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static const struct snd_soc_dai_ops sy6045s_dai_ops = {
	.hw_params   = sy6045s_hw_params,
	.set_fmt     = sy6045s_set_dai_fmt,
	.mute_stream = sy6045s_mute_stream,
};

static struct snd_soc_dai_driver sy6045s_dai = {
	.name = "sy6045s-hifi",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &sy6045s_dai_ops,
};

static const struct snd_soc_component_driver sy6045s_component_driver = {
	.probe          = sy6045s_component_probe,
	.remove         = sy6045s_component_remove,
	.controls       = sy6045s_snd_controls,
	.num_controls   = ARRAY_SIZE(sy6045s_snd_controls),
	.dapm_widgets   = sy6045s_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sy6045s_dapm_widgets),
	.dapm_routes    = sy6045s_dapm_routes,
	.num_dapm_routes  = ARRAY_SIZE(sy6045s_dapm_routes),
	.endianness     = 1,
};

static int sy6045s_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct sy6045s_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i2c = i2c;
	i2c_set_clientdata(i2c, priv);

	mutex_init(&priv->io_mutex);

	priv->pbtl_mode = of_property_read_bool(dev->of_node, "pbtl-mode");

	/* reset-gpios (optional) */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "error requesting reset_gpio\n");

	/* regmap */
	priv->regmap = devm_regmap_init_i2c(i2c, &sy6045s_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/* supplies */
	priv->supplies[0].supply = "vddio";
	priv->supplies[1].supply = "pvdd";
	ret = devm_regulator_bulk_get(dev, SY6045S_NUM_SUPPLIES,
				      priv->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	/* firmware name */
	of_property_read_string(dev->of_node, "firmware", &priv->fw_name);

	/* parse restore-regs */
	ret = sy6045s_parse_restore_regs(dev, priv);
	if (ret)
		return ret;

	/* sysfs attributes */
	ret = devm_device_add_group(dev, &sy6045s_attrs_group);
	if (ret)
		dev_warn(dev, "Failed to create amplifier sysfs: %d\n", ret);

	ret = snd_soc_register_component(dev, &sy6045s_component_driver,
					 &sy6045s_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register sy6045s component\n");

	dev_info(dev, "SY6045S probe success%s\n",
		 priv->pbtl_mode ? " (PBTL mode)" : "");

	return 0;
}

static void sy6045s_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_component(&i2c->dev);

	mutex_destroy(&((struct sy6045s_priv *)i2c_get_clientdata(i2c))->io_mutex);
}

static const struct i2c_device_id sy6045s_i2c_id[] = {
	{ "sy6045s" },
	{}
};
MODULE_DEVICE_TABLE(i2c, sy6045s_i2c_id);

static const struct of_device_id sy6045s_of_match[] = {
	{ .compatible = "silergy,sy6045s" },
	{}
};
MODULE_DEVICE_TABLE(of, sy6045s_of_match);

static struct i2c_driver sy6045s_i2c_driver = {
	.driver = {
		.name = "sy6045s",
		.of_match_table = sy6045s_of_match,
	},
	.probe    = sy6045s_i2c_probe,
	.remove   = sy6045s_i2c_remove,
	.id_table = sy6045s_i2c_id,
};
module_i2c_driver(sy6045s_i2c_driver);

MODULE_DESCRIPTION("ASoC Silergy SY6045S Class-D audio amplifier driver");
MODULE_AUTHOR("Bin Zhang <bin.zhang@silergycorp.com>, AtriOS Team");
MODULE_LICENSE("GPL v2");