// SPDX-License-Identifier: GPL-2.0
//
// Driver for Silergy SY6045S Class-D Audio Amplifier
//
// Author: Lefty <leftymods@gmail.com>
//
// This is a minimal ASoC codec driver for the SY6045S amplifier.
// It handles regulator management (vddio, pvdd) and registers
// a simple I2S/TDM playback DAI.

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>

struct sy6045s_priv {
	struct regulator *vddio;
	struct regulator *pvdd;
	bool pbtl_mode;
};

static int sy6045s_component_probe(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regulator_enable(priv->vddio);
	if (ret) {
		dev_err(component->dev, "Failed to enable vddio: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(priv->pvdd);
	if (ret) {
		dev_err(component->dev, "Failed to enable pvdd: %d\n", ret);
		regulator_disable(priv->vddio);
		return ret;
	}

	dev_info(component->dev, "SY6045S ready (pbtl=%d)\n", priv->pbtl_mode);
	return 0;
}

static void sy6045s_component_remove(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	regulator_disable(priv->pvdd);
	regulator_disable(priv->vddio);
}

static const struct snd_soc_dapm_widget sy6045s_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Speaker"),
};

static const struct snd_soc_dapm_route sy6045s_dapm_routes[] = {
	{"Speaker", NULL, "HiFi Playback"},
};

static const struct snd_soc_component_driver sy6045s_component_driver = {
	.probe			= sy6045s_component_probe,
	.remove			= sy6045s_component_remove,
	.dapm_widgets		= sy6045s_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(sy6045s_dapm_widgets),
	.dapm_routes		= sy6045s_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(sy6045s_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static struct snd_soc_dai_driver sy6045s_dai_driver = {
	.name		= "sy6045s-hifi",
	.playback	= {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,
		.rates		= SNDRV_PCM_RATE_8000 |
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_44100 |
					SNDRV_PCM_RATE_48000 |
					SNDRV_PCM_RATE_96000,
		.rate_min	= 8000,
		.rate_max	= 96000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
};

static int sy6045s_i2c_probe(struct i2c_client *client)
{
	struct sy6045s_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->vddio = devm_regulator_get(&client->dev, "vddio");
	if (IS_ERR(priv->vddio))
		return dev_err_probe(&client->dev, PTR_ERR(priv->vddio),
				     "Failed to get vddio supply\n");

	priv->pvdd = devm_regulator_get(&client->dev, "pvdd");
	if (IS_ERR(priv->pvdd))
		return dev_err_probe(&client->dev, PTR_ERR(priv->pvdd),
				     "Failed to get pvdd supply\n");

	priv->pbtl_mode = device_property_read_bool(&client->dev, "pbtl-mode");

	dev_set_drvdata(&client->dev, priv);

	ret = devm_snd_soc_register_component(&client->dev,
					      &sy6045s_component_driver,
					      &sy6045s_dai_driver, 1);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to register codec\n");

	return 0;
}

static const struct of_device_id sy6045s_of_match[] = {
	{ .compatible = "silergy,sy6045s", },
	{}
};
MODULE_DEVICE_TABLE(of, sy6045s_of_match);

static const struct i2c_device_id sy6045s_i2c_id[] = {
	{ "sy6045s", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sy6045s_i2c_id);

static struct i2c_driver sy6045s_i2c_driver = {
	.driver = {
		.name = "sy6045s",
		.of_match_table = sy6045s_of_match,
	},
	.probe = sy6045s_i2c_probe,
	.id_table = sy6045s_i2c_id,
};
module_i2c_driver(sy6045s_i2c_driver);

MODULE_DESCRIPTION("Silergy SY6045S Class-D Amplifier Codec Driver");
MODULE_AUTHOR("Lefty <leftymods@gmail.com>");
MODULE_LICENSE("GPL v2");
