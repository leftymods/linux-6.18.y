// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "es8156.h"

struct es8156_priv {
	struct regmap *regmap;
	unsigned int sysclk;
	unsigned int rate;
	unsigned int fmt;
	bool is_master;
};

static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8156_dac_vol_tlv, -9600, 50, 1);

static int es8156_dac_vol_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;

	regmap_read(priv->regmap, ES8156_REG_VOLUME_CONTROL, &val);
	ucontrol->value.integer.value[0] = val & 0xFF;
	return 0;
}

static int es8156_dac_vol_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);

	return regmap_write(priv->regmap, ES8156_REG_VOLUME_CONTROL,
			    ucontrol->value.integer.value[0] & 0xFF);
}

static const struct snd_kcontrol_new es8156_snd_controls[] = {
	SOC_SINGLE("Master Playback Volume", ES8156_REG_VOLUME_CONTROL,
		   0, 0xFF, 0),
	SOC_SINGLE("DAC Mute Switch", ES8156_REG_MUTE_CONTROL, 3, 1, 1),
	SOC_SINGLE("ALC Switch", ES8156_REG_ALC_CONFIG_1, 0, 1, 0),
};

static const struct snd_soc_dapm_widget es8156_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("AOUT1"),
	SND_SOC_DAPM_OUTPUT("AOUT2"),
	SND_SOC_DAPM_OUTPUT("AOUT3"),
	SND_SOC_DAPM_OUTPUT("AOUT4"),
	SND_SOC_DAPM_OUTPUT("AOUT5"),
	SND_SOC_DAPM_OUTPUT("AOUT6"),
	SND_SOC_DAPM_OUTPUT("AOUT7"),
	SND_SOC_DAPM_OUTPUT("AOUT8"),
};

static const struct snd_soc_dapm_route es8156_dapm_routes[] = {
	{ "AOUT1", NULL, "DAC" },
	{ "AOUT2", NULL, "DAC" },
	{ "AOUT3", NULL, "DAC" },
	{ "AOUT4", NULL, "DAC" },
	{ "AOUT5", NULL, "DAC" },
	{ "AOUT6", NULL, "DAC" },
	{ "AOUT7", NULL, "DAC" },
	{ "AOUT8", NULL, "DAC" },
};

static int es8156_set_dai_sysclk(struct snd_soc_dai *dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);

	priv->sysclk = freq;
	return 0;
}

static int es8156_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);
	u8 mode = 0;
	u8 sdp = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		mode &= ~ES8156_MODE_MS;
		priv->is_master = false;
		break;
	case SND_SOC_DAIFMT_CBP_CFP:
		mode |= ES8156_MODE_MS;
		priv->is_master = true;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		sdp &= ~ES8156_SDP_PROTOCOL_MASK;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		sdp |= 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		sdp |= 0x02;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		mode &= ~ES8156_MODE_SCLK_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		mode |= ES8156_MODE_SCLK_INV;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(priv->regmap, ES8156_REG_MODE_CONFIG,
			   ES8156_MODE_MS | ES8156_MODE_SCLK_INV, mode);
	regmap_update_bits(priv->regmap, ES8156_REG_SDP_INTERFACE_CONFIG_1,
			   ES8156_SDP_PROTOCOL_MASK, sdp);

	priv->fmt = fmt;
	return 0;
}

static int es8156_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);
	u8 wordlen = 0;
	unsigned int rate = params_rate(params);

	priv->rate = rate;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		wordlen = 0x60;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		wordlen = 0x40;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		wordlen = 0x00;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		wordlen = 0x80;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(priv->regmap, ES8156_REG_SDP_INTERFACE_CONFIG_1,
			   ES8156_SDP_WL_MASK, wordlen);

	return 0;
}

static int es8156_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);

	regmap_update_bits(priv->regmap, ES8156_REG_SDP_INTERFACE_CONFIG_1,
			   ES8156_SDP_MUTE, mute ? ES8156_SDP_MUTE : 0);
	return 0;
}

#define ES8156_RATES SNDRV_PCM_RATE_8000_192000
#define ES8156_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops es8156_dai_ops = {
	.set_sysclk	= es8156_set_dai_sysclk,
	.set_fmt	= es8156_set_dai_fmt,
	.hw_params	= es8156_hw_params,
	.mute_stream	= es8156_mute,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver es8156_dai = {
	.name = "es8156-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = ES8156_RATES,
		.formats = ES8156_FORMATS,
	},
	.ops = &es8156_dai_ops,
};

static int es8156_probe(struct snd_soc_component *component)
{
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);

	regmap_write(priv->regmap, ES8156_REG_RESET_CONTROL, 0xFF);
	usleep_range(5000, 5500);
	regmap_write(priv->regmap, ES8156_REG_RESET_CONTROL,
		     ES8156_RESET_CSM_ON);
	msleep(30);

	regmap_write(priv->regmap, ES8156_REG_CLOCK_OFF, 0x3F);
	regmap_write(priv->regmap, ES8156_REG_ANALOG_SYSTEM_4, 0x70);
	regmap_write(priv->regmap, ES8156_REG_ANALOG_SYSTEM_6, 0x00);
	regmap_write(priv->regmap, ES8156_REG_MISC_CONTROL_2, 0x10);
	regmap_write(priv->regmap, ES8156_REG_TIME_CONTROL_1, 0x30);
	regmap_write(priv->regmap, ES8156_REG_TIME_CONTROL_2, 0x30);
	regmap_write(priv->regmap, ES8156_REG_DAC_COUNTER_PARAMETER, 0x20);
	regmap_write(priv->regmap, ES8156_REG_MUTE_CONTROL, 0x20);
	regmap_write(priv->regmap, ES8156_REG_VOLUME_CONTROL, 0xC0);
	regmap_write(priv->regmap, ES8156_REG_ANALOG_SYSTEM_1, 0xFF);

	return 0;
}

static const struct snd_soc_component_driver es8156_component_driver = {
	.probe			= es8156_probe,
	.controls		= es8156_snd_controls,
	.num_controls		= ARRAY_SIZE(es8156_snd_controls),
	.dapm_widgets		= es8156_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8156_dapm_widgets),
	.dapm_routes		= es8156_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8156_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static bool es8156_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ES8156_REG_CHIP_STATUS:
		return true;
	default:
		return false;
	}
}

static bool es8156_readable_reg(struct device *dev, unsigned int reg)
{
	if (reg <= 0x25 || reg == 0xFC || reg >= 0xFD)
		return true;
	return false;
}

static bool es8156_writeable_reg(struct device *dev, unsigned int reg)
{
	if (reg <= 0x25 || reg == 0xFC)
		return true;
	return false;
}

static const struct regmap_config es8156_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES8156_MAX_REGISTER,
	.volatile_reg	= es8156_volatile_reg,
	.readable_reg	= es8156_readable_reg,
	.writeable_reg	= es8156_writeable_reg,
	.cache_type	= REGCACHE_MAPLE,
};

static int es8156_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es8156_priv *priv;
	unsigned int val;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(i2c, priv);

	priv->regmap = devm_regmap_init_i2c(i2c, &es8156_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	regmap_read(priv->regmap, ES8156_REG_CHIP_ID1, &val);
	if (val != (ES8156_CHIP_ID >> 8)) {
		dev_err(dev, "chip ID1 mismatch: 0x%02x\n", val);
		return -ENODEV;
	}

	return devm_snd_soc_register_component(dev, &es8156_component_driver,
					       &es8156_dai, 1);
}

static const struct i2c_device_id es8156_i2c_id[] = {
	{ "es8156" },
	{}
};
MODULE_DEVICE_TABLE(i2c, es8156_i2c_id);

static const struct of_device_id es8156_of_match[] = {
	{ .compatible = "everest,es8156" },
	{}
};
MODULE_DEVICE_TABLE(of, es8156_of_match);

static struct i2c_driver es8156_i2c_driver = {
	.driver = {
		.name = "es8156",
		.of_match_table = es8156_of_match,
	},
	.probe = es8156_i2c_probe,
	.id_table = es8156_i2c_id,
};
module_i2c_driver(es8156_i2c_driver);

MODULE_DESCRIPTION("ASoC ES8156 8-channel DAC driver");
MODULE_AUTHOR("AtriOS Team");
MODULE_LICENSE("GPL v2");
