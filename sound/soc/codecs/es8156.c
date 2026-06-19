// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#define ES8156_RESET		0x00
#define ES8156_MAIN_CLK		0x01
#define ES8156_SCLK_DIV		0x02
#define ES8156_LRCK_DIV_H	0x03
#define ES8156_LRCK_DIV_L	0x04
#define ES8156_FMT		0x05
#define ES8156_VOLUME		0x14
#define ES8156_DAC_MUTE		0x19

struct es8156_priv {
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	unsigned int mclk_rate;
};

static const struct reg_default es8156_reg_defaults[] = {
	{ ES8156_RESET,      0x00 },
	{ ES8156_MAIN_CLK,   0x00 },
	{ ES8156_SCLK_DIV,   0x00 },
	{ ES8156_FMT,        0x00 },
	{ ES8156_VOLUME,     0xbf },
	{ ES8156_DAC_MUTE,   0x00 },
};

static bool es8156_readable_register(struct device *dev, unsigned int reg)
{
	return reg <= ES8156_DAC_MUTE;
}

static bool es8156_writeable_register(struct device *dev, unsigned int reg)
{
	return reg <= ES8156_DAC_MUTE;
}

static const struct regmap_config es8156_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ES8156_DAC_MUTE,
	.readable_reg = es8156_readable_register,
	.writeable_reg = es8156_writeable_register,
	.reg_defaults = es8156_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8156_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

static const DECLARE_TLV_DB_SCALE(es8156_dac_tlv, -9550, 50, 0);

static const struct snd_kcontrol_new es8156_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Playback Volume", ES8156_VOLUME, ES8156_VOLUME,
			 0, 0xff, 0, es8156_dac_tlv),
	SOC_SINGLE("Playback Switch", ES8156_DAC_MUTE, 2, 1, 1),
};

static int es8156_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);
	unsigned int width = params_width(params);
	u8 fmt = 0;

	switch (width) {
	case 16:
		fmt = 0x03;
		break;
	case 24:
		fmt = 0x00;
		break;
	case 32:
		fmt = 0x04;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(es8156->regmap, ES8156_FMT, 0x1c, fmt);
	return 0;
}

static int es8156_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);
	u8 iface = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	case SND_SOC_DAIFMT_CBP_CFC:
		iface |= 0x20;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x00;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x01;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(es8156->regmap, ES8156_FMT, 0x23, iface);
	return 0;
}

static const struct snd_soc_dai_ops es8156_dai_ops = {
	.hw_params = es8156_hw_params,
	.set_fmt = es8156_set_dai_fmt,
};

static struct snd_soc_dai_driver es8156_dai = {
	.name = "es8156-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es8156_dai_ops,
};

static const struct snd_soc_component_driver soc_component_dev_es8156 = {
	.controls		= es8156_snd_controls,
	.num_controls		= ARRAY_SIZE(es8156_snd_controls),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int es8156_i2c_probe(struct i2c_client *i2c)
{
	struct es8156_priv *es8156;
	struct device *dev = &i2c->dev;
	int ret;

	es8156 = devm_kzalloc(dev, sizeof(*es8156), GFP_KERNEL);
	if (!es8156)
		return -ENOMEM;

	es8156->regmap = devm_regmap_init_i2c(i2c, &es8156_regmap_config);
	if (IS_ERR(es8156->regmap))
		return PTR_ERR(es8156->regmap);

	es8156->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(es8156->reset_gpio))
		return PTR_ERR(es8156->reset_gpio);

	if (es8156->reset_gpio) {
		gpiod_set_value_cansleep(es8156->reset_gpio, 1);
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(es8156->reset_gpio, 0);
	}

	es8156->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(es8156->mclk)) {
		if (PTR_ERR(es8156->mclk) != -ENOENT)
			return PTR_ERR(es8156->mclk);
		es8156->mclk = NULL;
	}

	if (es8156->mclk) {
		ret = clk_prepare_enable(es8156->mclk);
		if (ret)
			return ret;
	}

	i2c_set_clientdata(i2c, es8156);

	ret = devm_snd_soc_register_component(dev, &soc_component_dev_es8156,
					      &es8156_dai, 1);
	if (ret && es8156->mclk)
		clk_disable_unprepare(es8156->mclk);

	return ret;
}

static void es8156_i2c_remove(struct i2c_client *i2c)
{
	struct es8156_priv *es8156 = i2c_get_clientdata(i2c);

	if (es8156->mclk)
		clk_disable_unprepare(es8156->mclk);
}

static const struct i2c_device_id es8156_i2c_id[] = {
	{ "es8156", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8156_i2c_id);

static const struct of_device_id es8156_of_match[] = {
	{ .compatible = "everest,es8156" },
	{ }
};
MODULE_DEVICE_TABLE(of, es8156_of_match);

static struct i2c_driver es8156_i2c_driver = {
	.driver = {
		.name = "es8156",
		.of_match_table = es8156_of_match,
	},
	.probe = es8156_i2c_probe,
	.remove = es8156_i2c_remove,
	.id_table = es8156_i2c_id,
};

module_i2c_driver(es8156_i2c_driver);

MODULE_DESCRIPTION("ASoC Everest Semiconductor ES8156 DAC Driver");
MODULE_AUTHOR("Rockchip Linux Team");
MODULE_LICENSE("GPL v2");
