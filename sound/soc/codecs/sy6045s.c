// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define SY6045S_NUM_SUPPLIES 2

static const char *const sy6045s_supply_names[SY6045S_NUM_SUPPLIES] = {
	"vddio",
	"pvdd",
};

struct sy6045s_priv {
	struct regmap *regmap;
	struct regulator_bulk_data supplies[SY6045S_NUM_SUPPLIES];
	bool pbtl_mode;
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
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0x1f,
	.writeable_reg	= sy6045s_writeable_reg,
	.readable_reg	= sy6045s_readable_reg,
	.volatile_reg	= sy6045s_volatile_reg,
	.cache_type	= REGCACHE_MAPLE,
};

static const struct snd_kcontrol_new sy6045s_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Master Playback Volume",
			 0x02, 0x03,
			 0, 0xff, 0, NULL),
};

static const struct snd_soc_dapm_widget sy6045s_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),
	SND_SOC_DAPM_AIF_IN("Playback", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route sy6045s_dapm_routes[] = {
	{ "SPK_OUT", NULL, "Playback" },
};

static int sy6045s_component_probe(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regulator_bulk_enable(SY6045S_NUM_SUPPLIES, priv->supplies);
	if (ret)
		return ret;

	return 0;
}

static void sy6045s_component_remove(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	regulator_bulk_disable(SY6045S_NUM_SUPPLIES, priv->supplies);
}

static const struct snd_soc_component_driver sy6045s_component_driver = {
	.probe			= sy6045s_component_probe,
	.remove			= sy6045s_component_remove,
	.controls		= sy6045s_snd_controls,
	.num_controls		= ARRAY_SIZE(sy6045s_snd_controls),
	.dapm_widgets		= sy6045s_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(sy6045s_dapm_widgets),
	.dapm_routes		= sy6045s_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(sy6045s_dapm_routes),
	.endianness		= 1,
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
};

static int sy6045s_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct sy6045s_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(i2c, priv);

	priv->pbtl_mode = of_property_read_bool(dev->of_node, "pbtl-mode");

	priv->regmap = devm_regmap_init_i2c(i2c, &sy6045s_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->supplies[0].supply = "vddio";
	priv->supplies[1].supply = "pvdd";
	ret = devm_regulator_bulk_get(dev, SY6045S_NUM_SUPPLIES, priv->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	return devm_snd_soc_register_component(dev, &sy6045s_component_driver,
					       &sy6045s_dai, 1);
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
	.probe = sy6045s_i2c_probe,
	.id_table = sy6045s_i2c_id,
};
module_i2c_driver(sy6045s_i2c_driver);

MODULE_DESCRIPTION("ASoC Silergy SY6045S Class-D audio amplifier driver");
MODULE_AUTHOR("AtriOS Team");
MODULE_LICENSE("GPL v2");
