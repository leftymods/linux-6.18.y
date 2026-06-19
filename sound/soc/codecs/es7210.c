// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "es7210.h"

struct es7210_priv {
	struct regmap *regmap;
	unsigned int sysclk;
	unsigned int rate;
	unsigned int fmt;
};

static const DECLARE_TLV_DB_SCALE(es7210_mic_gain_tlv, 0, 300, 0);

static const struct snd_kcontrol_new es7210_snd_controls[] = {
	SOC_SINGLE_TLV("MIC1 Boost Volume", ES7210_REG_MIC1_POWER, 4, 7, 0,
		       es7210_mic_gain_tlv),
	SOC_SINGLE_TLV("MIC2 Boost Volume", ES7210_REG_MIC2_POWER, 4, 7, 0,
		       es7210_mic_gain_tlv),
	SOC_SINGLE_TLV("MIC3 Boost Volume", ES7210_REG_MIC3_POWER, 4, 7, 0,
		       es7210_mic_gain_tlv),
	SOC_SINGLE_TLV("MIC4 Boost Volume", ES7210_REG_MIC4_POWER, 4, 7, 0,
		       es7210_mic_gain_tlv),
	SOC_SINGLE("ALC Switch", ES7210_REG_ALC_CONFIG_1, 0, 1, 0),
};

static const struct snd_soc_dapm_widget es7210_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("MIC3"),
	SND_SOC_DAPM_INPUT("MIC4"),
};

static const struct snd_soc_dapm_route es7210_dapm_routes[] = {
	{ "ADC", NULL, "MIC1" },
	{ "ADC", NULL, "MIC2" },
	{ "ADC", NULL, "MIC3" },
	{ "ADC", NULL, "MIC4" },
};

static int es7210_set_dai_sysclk(struct snd_soc_dai *dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct es7210_priv *priv = snd_soc_component_get_drvdata(component);

	priv->sysclk = freq;
	return 0;
}

static int es7210_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct es7210_priv *priv = snd_soc_component_get_drvdata(component);
	u8 sdp = 0;
	u8 ms = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		ms &= ~ES7210_MS_MODE_MASK;
		break;
	case SND_SOC_DAIFMT_CBP_CFP:
		ms |= ES7210_MS_MODE_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		sdp &= ~ES7210_SDP_FMT_MASK;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		sdp |= 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		sdp |= 0x04;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(priv->regmap, ES7210_REG_MASTER_SLAVE,
			   ES7210_MS_MODE_MASK, ms);
	regmap_update_bits(priv->regmap, ES7210_REG_SDP_INTERFACE1,
			   ES7210_SDP_FMT_MASK, sdp);

	priv->fmt = fmt;
	return 0;
}

static int es7210_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es7210_priv *priv = snd_soc_component_get_drvdata(component);
	u8 wordlen = 0;
	u8 fs_ratio = 0;

	priv->rate = params_rate(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		wordlen = 0x00;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		wordlen = 0x08;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		wordlen = 0x10;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		wordlen = 0x18;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(priv->regmap, ES7210_REG_SDP_INTERFACE1,
			   ES7210_SDP_WL_MASK, wordlen);

	return 0;
}

#define ES7210_RATES SNDRV_PCM_RATE_8000_96000
#define ES7210_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops es7210_dai_ops = {
	.set_sysclk	= es7210_set_dai_sysclk,
	.set_fmt	= es7210_set_dai_fmt,
	.hw_params	= es7210_hw_params,
};

static struct snd_soc_dai_driver es7210_dai = {
	.name = "es7210-hifi",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		.rates = ES7210_RATES,
		.formats = ES7210_FORMATS,
	},
	.ops = &es7210_dai_ops,
};

static int es7210_probe(struct snd_soc_component *component)
{
	struct es7210_priv *priv = snd_soc_component_get_drvdata(component);

	regmap_write(priv->regmap, ES7210_REG_RESET, ES7210_RESET_ALL);
	regmap_write(priv->regmap, ES7210_REG_MASTER_SLAVE, 0x32);
	regmap_write(priv->regmap, ES7210_REG_CLOCK_MANAGEMENT, 0x20);
	regmap_write(priv->regmap, ES7210_REG_TIME_CONTROL_1, 0x30);
	regmap_write(priv->regmap, ES7210_REG_TIME_CONTROL_2, 0x30);
	regmap_write(priv->regmap, ES7210_REG_ADC_INPUT_CONTROL, 0x66);
	regmap_write(priv->regmap, ES7210_REG_SDP_INTERFACE1, 0x90);
	regmap_write(priv->regmap, ES7210_REG_SDP_INTERFACE2, 0x00);
	regmap_write(priv->regmap, ES7210_REG_MIC1_POWER, 0x70);
	regmap_write(priv->regmap, ES7210_REG_MIC2_POWER, 0x70);
	regmap_write(priv->regmap, ES7210_REG_MIC3_POWER, 0x70);
	regmap_write(priv->regmap, ES7210_REG_MIC4_POWER, 0x70);
	regmap_write(priv->regmap, ES7210_REG_ADC_SAMPLE_RATE, 0x05);

	return 0;
}

static const struct snd_soc_component_driver es7210_component_driver = {
	.probe			= es7210_probe,
	.controls		= es7210_snd_controls,
	.num_controls		= ARRAY_SIZE(es7210_snd_controls),
	.dapm_widgets		= es7210_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es7210_dapm_widgets),
	.dapm_routes		= es7210_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es7210_dapm_routes),
	.endianness		= 1,
};

static bool es7210_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg == ES7210_REG_CHIP_ID)
		return true;
	return false;
}

static bool es7210_readable_reg(struct device *dev, unsigned int reg)
{
	if (reg <= 0x4C)
		return true;
	return false;
}

static bool es7210_writeable_reg(struct device *dev, unsigned int reg)
{
	if (reg < ES7210_REG_CHIP_ID)
		return true;
	return false;
}

static const struct regmap_config es7210_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES7210_MAX_REGISTER,
	.volatile_reg	= es7210_volatile_reg,
	.readable_reg	= es7210_readable_reg,
	.writeable_reg	= es7210_writeable_reg,
	.cache_type	= REGCACHE_MAPLE,
};

static int es7210_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es7210_priv *priv;
	unsigned int val;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(i2c, priv);

	priv->regmap = devm_regmap_init_i2c(i2c, &es7210_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	if (regmap_read(priv->regmap, ES7210_REG_CHIP_ID, &val) < 0)
		dev_warn(dev, "chip not responding, trying reset\n");
	else if (val != ES7210_CHIP_ID)
		dev_warn(dev, "chip ID mismatch: 0x%02x, continuing\n", val);

	return devm_snd_soc_register_component(dev, &es7210_component_driver,
					       &es7210_dai, 1);
}

static const struct i2c_device_id es7210_i2c_id[] = {
	{ "es7210" },
	{}
};
MODULE_DEVICE_TABLE(i2c, es7210_i2c_id);

static const struct of_device_id es7210_of_match[] = {
	{ .compatible = "everest,es7210" },
	{}
};
MODULE_DEVICE_TABLE(of, es7210_of_match);

static struct i2c_driver es7210_i2c_driver = {
	.driver = {
		.name = "es7210",
		.of_match_table = es7210_of_match,
	},
	.probe = es7210_i2c_probe,
	.id_table = es7210_i2c_id,
};
module_i2c_driver(es7210_i2c_driver);

MODULE_DESCRIPTION("ASoC ES7210 4-channel ADC driver");
MODULE_AUTHOR("AtriOS Team");
MODULE_LICENSE("GPL v2");
