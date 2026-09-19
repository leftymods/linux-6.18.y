// SPDX-License-Identifier: GPL-2.0-only
/*
 * es8156.c -- Everest ES8156 ALSA SoC Audio Driver
 *
 * Copyright (C) Everest Semiconductor Co.,Ltd.
 * Author: Will <pengxiaoxin@everest-semi.com>
 * Mainline port for Linux 6.x / modern ASoC by Reverse Engineering
 * Full functional parity with vendor kernel module snd-soc-es8156.ko
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "es8156.h"

struct es8156_priv {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct gpio_desc *spk_gpio;
	struct gpio_desc *hp_det_gpio;
	int hp_irq;
	struct clk *mclk;
	unsigned int debounce_time;
	int hp_det_invert;
	struct delayed_work work;
	bool muted;
	bool hp_inserted;
	bool spk_active_level;
	unsigned int mclk_mult;
	bool sclk_as_mclk;
	bool dll_enable;
	int pwr_count;
};

static struct es8156_priv *static_es8156;

static const struct reg_default es8156_reg_defaults[] = {
	{ 0x00, 0x1c },
	{ 0x01, 0x20 },
	{ 0x02, 0x00 },
	{ 0x03, 0x01 },
	{ 0x04, 0x00 },
	{ 0x05, 0x04 },
	{ 0x06, 0x11 },
	{ 0x07, 0x00 },
	{ 0x08, 0x06 },
	{ 0x09, 0x00 },
	{ 0x0a, 0x50 },
	{ 0x0b, 0x50 },
	{ 0x0c, 0x00 },
	{ 0x0d, 0x10 },
	{ 0x11, 0x00 },
	{ 0x12, 0x04 },
	{ 0x13, 0x11 },
	{ 0x14, 0xbf },
	{ 0x15, 0x00 },
	{ 0x16, 0x00 },
	{ 0x17, 0xf7 },
	{ 0x18, 0x00 },
	{ 0x19, 0x20 },
	{ 0x1a, 0x00 },
	{ 0x20, 0x16 },
	{ 0x21, 0x7f },
	{ 0x22, 0x00 },
	{ 0x23, 0x86 },
	{ 0x24, 0x00 },
	{ 0x25, 0x07 },
};

static bool es8156_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x25:
	case ES8156_CHIP_STATUS_REGFC:
	case ES8156_CHIP_ID_REGFD:
	case ES8156_CHIP_ID_REGFE:
	case ES8156_CHIP_VERSION_REGFF:
		return true;
	default:
		return false;
	}
}

static bool es8156_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ES8156_CHIP_STATUS_REGFC:
	case ES8156_CHIP_ID_REGFD:
	case ES8156_CHIP_ID_REGFE:
	case ES8156_CHIP_VERSION_REGFF:
		return true;
	default:
		return false;
	}
}

static void es8156_enable_spk(struct es8156_priv *es8156, bool enable)
{
	if (!es8156->spk_gpio)
		return;

	gpiod_set_value_cansleep(es8156->spk_gpio,
				 enable ? es8156->spk_active_level : !es8156->spk_active_level);
}

static void hp_work(struct work_struct *work)
{
	struct es8156_priv *es8156 = container_of(work, struct es8156_priv, work.work);
	int val = 0;

	if (es8156->hp_det_gpio)
		val = gpiod_get_value_cansleep(es8156->hp_det_gpio);

	if (es8156->hp_det_invert)
		val = !val;

	es8156->hp_inserted = val ? true : false;

	if (!es8156->muted) {
		/* If headphone is inserted, disable loudspeaker */
		es8156_enable_spk(es8156, !es8156->hp_inserted);
	}
}

static irqreturn_t es8156_irq_handler(int irq, void *data)
{
	struct es8156_priv *es8156 = data;

	queue_delayed_work(system_power_efficient_wq, &es8156->work,
			   msecs_to_jiffies(es8156->debounce_time));

	return IRQ_HANDLED;
}

int es8156_headset_detect(int enable)
{
	if (!static_es8156)
		return -ENODEV;

	static_es8156->hp_inserted = (enable != 0);
	if (enable)
		es8156_enable_spk(static_es8156, false);
	else if (!static_es8156->muted)
		es8156_enable_spk(static_es8156, true);

	return 0;
}
EXPORT_SYMBOL_GPL(es8156_headset_detect);

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -9550, 50, 0);

static const struct snd_kcontrol_new es8156_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Playback Volume", ES8156_VOLUME_CONTROL_REG14,
			 ES8156_VOLUME_CONTROL_REG15, 0, 0xff, 0, dac_vol_tlv),
	SOC_SINGLE("Playback Switch", ES8156_VOLUME_CONTROL_REG14, 7, 1, 1),
	SOC_SINGLE("Automute Switch", ES8156_DAC_MUTE_REG13, 3, 1, 0),
	SOC_SINGLE("EQ Switch", ES8156_EQ_CONTROL1_REG18, 7, 1, 0),
};

static const struct snd_soc_dapm_widget es8156_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", ES8156_ANALOG_SYS1_REG20, 4, 1),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route es8156_dapm_routes[] = {
	{ "LOUT", NULL, "DAC" },
	{ "ROUT", NULL, "DAC" },
	{ "SPK", NULL, "DAC" },
};

static int es8156_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);

	es8156->muted = mute;

	if (mute) {
		es8156_enable_spk(es8156, false);
		snd_soc_component_update_bits(component, ES8156_DAC_MUTE_REG13, 0x01, 0x01);
	} else {
		snd_soc_component_update_bits(component, ES8156_DAC_MUTE_REG13, 0x01, 0x00);
		if (!es8156->hp_inserted)
			es8156_enable_spk(es8156, true);
	}
	return 0;
}

static int es8156_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	u8 sfmt = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		sfmt = 0x00;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		sfmt = 0x01;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		sfmt = 0x02;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		sfmt = 0x03;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ES8156_SDP_INTERFACE1_REG11, 0x03, sfmt);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		snd_soc_component_update_bits(component, ES8156_SDP_INTERFACE1_REG11, 0x20, 0x00);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		snd_soc_component_update_bits(component, ES8156_SDP_INTERFACE1_REG11, 0x20, 0x20);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int es8156_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	u8 wl = 0;

	switch (params_width(params)) {
	case 16:
		wl = 0x03;
		break;
	case 20:
		wl = 0x01;
		break;
	case 24:
		wl = 0x00;
		break;
	case 32:
		wl = 0x04;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ES8156_SDP_INTERFACE1_REG11, 0x1c, wl << 2);
	return 0;
}

static int es8156_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		snd_soc_component_update_bits(component, ES8156_ANALOG_SYS1_REG20, 0x18, 0x18);
		break;
	case SND_SOC_BIAS_STANDBY:
		snd_soc_component_update_bits(component, ES8156_ANALOG_SYS1_REG20, 0x18, 0x00);
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_component_update_bits(component, ES8156_RESET_REG00, 0x1f, 0x1f);
		break;
	}
	return 0;
}

static const struct snd_soc_dai_ops es8156_dai_ops = {
	.set_fmt	= es8156_set_dai_fmt,
	.hw_params	= es8156_pcm_hw_params,
	.mute_stream	= es8156_mute,
};

static struct snd_soc_dai_driver es8156_dai = {
	.name = "ES8156 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE |
			    SNDRV_PCM_FMTBIT_S20_3LE |
			    SNDRV_PCM_FMTBIT_S24_LE |
			    SNDRV_PCM_FMTBIT_S32_LE),
	},
	.ops = &es8156_dai_ops,
};

static int es8156_init_regs(struct snd_soc_component *component)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(es8156_reg_defaults); i++) {
		snd_soc_component_write(component,
					es8156_reg_defaults[i].reg,
					es8156_reg_defaults[i].def);
	}
	return 0;
}

static int es8156_probe(struct snd_soc_component *component)
{
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);

	es8156_init_regs(component);

	/* Trigger initial check of headset presence */
	queue_delayed_work(system_power_efficient_wq, &es8156->work,
			   msecs_to_jiffies(es8156->debounce_time));

	return 0;
}

static void es8156_remove(struct snd_soc_component *component)
{
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&es8156->work);
	es8156_enable_spk(es8156, false);
}

static const struct snd_soc_component_driver soc_component_dev_es8156 = {
	.probe			= es8156_probe,
	.remove			= es8156_remove,
	.set_bias_level		= es8156_set_bias_level,
	.controls		= es8156_snd_controls,
	.num_controls		= ARRAY_SIZE(es8156_snd_controls),
	.dapm_widgets		= es8156_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8156_dapm_widgets),
	.dapm_routes		= es8156_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8156_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config es8156_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= ES8156_CHIP_VERSION_REGFF,
	.reg_defaults		= es8156_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(es8156_reg_defaults),
	.cache_type		= REGCACHE_MAPLE,
	.readable_reg		= es8156_readable_register,
	.volatile_reg		= es8156_volatile_register,
};

static int es8156_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es8156_priv *es8156;
	unsigned int id_fd, id_fe;
	u32 val32;
	int ret;

	es8156 = devm_kzalloc(dev, sizeof(*es8156), GFP_KERNEL);
	if (!es8156)
		return -ENOMEM;

	es8156->i2c = i2c;
	es8156->debounce_time = 200; /* 200 ms default */
	es8156->spk_active_level = true;
	es8156->mclk_mult = 256;
	static_es8156 = es8156;
	i2c_set_clientdata(i2c, es8156);

	/* Device tree optional configuration */
	if (!device_property_read_u32(dev, "debounce-time", &val32))
		es8156->debounce_time = val32;

	if (device_property_read_bool(dev, "hp-det-invert"))
		es8156->hp_det_invert = 1;

	if (device_property_read_bool(dev, "spk-active-low"))
		es8156->spk_active_level = false;

	if (!device_property_read_u32(dev, "mclk-mult", &val32))
		es8156->mclk_mult = val32;

	es8156->sclk_as_mclk = device_property_read_bool(dev, "sclk-as-mclk");
	es8156->dll_enable = device_property_read_bool(dev, "dll-enable");

	es8156->regmap = devm_regmap_init_i2c(i2c, &es8156_regmap_config);
	if (IS_ERR(es8156->regmap))
		return dev_err_probe(dev, PTR_ERR(es8156->regmap),
				      "Failed to init regmap\n");

	ret = regmap_read(es8156->regmap, ES8156_CHIP_ID_REGFD, &id_fd);
	ret |= regmap_read(es8156->regmap, ES8156_CHIP_ID_REGFE, &id_fe);
	if (ret < 0 || id_fd != 0x81) {
		dev_warn(dev, "Unexpected chip ID: 0x%02x:0x%02x (ret=%d)\n",
			 id_fd, id_fe, ret);
	} else {
		dev_info(dev, "Detected Everest ES8156 audio DAC (ID 0x%02x%02x)\n",
			 id_fd, id_fe);
	}

	es8156->spk_gpio = devm_gpiod_get_optional(dev, "spk-con", GPIOD_OUT_LOW);
	if (IS_ERR(es8156->spk_gpio))
		return dev_err_probe(dev, PTR_ERR(es8156->spk_gpio),
				      "Failed to get spk-con GPIO\n");

	es8156->hp_det_gpio = devm_gpiod_get_optional(dev, "hp-det", GPIOD_IN);
	if (IS_ERR(es8156->hp_det_gpio))
		return dev_err_probe(dev, PTR_ERR(es8156->hp_det_gpio),
				      "Failed to get hp-det GPIO\n");

	INIT_DELAYED_WORK(&es8156->work, hp_work);

	if (es8156->hp_det_gpio) {
		es8156->hp_irq = gpiod_to_irq(es8156->hp_det_gpio);
		if (es8156->hp_irq > 0) {
			ret = devm_request_threaded_irq(dev, es8156->hp_irq, NULL,
							es8156_irq_handler,
							IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							"es8156_hp", es8156);
			if (ret)
				dev_warn(dev, "Failed to request hp interrupt (%d)\n", ret);
		}
	}

	return devm_snd_soc_register_component(dev, &soc_component_dev_es8156,
					       &es8156_dai, 1);
}

static void es8156_i2c_remove(struct i2c_client *i2c)
{
	struct es8156_priv *es8156 = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&es8156->work);
	static_es8156 = NULL;
}

static void es8156_i2c_shutdown(struct i2c_client *i2c)
{
	struct es8156_priv *es8156 = i2c_get_clientdata(i2c);

	es8156_enable_spk(es8156, false);
}

static int __maybe_unused es8156_suspend(struct device *dev)
{
	struct es8156_priv *es8156 = dev_get_drvdata(dev);

	es8156_enable_spk(es8156, false);
	regcache_cache_only(es8156->regmap, true);
	regcache_mark_dirty(es8156->regmap);
	return 0;
}

static int __maybe_unused es8156_resume(struct device *dev)
{
	struct es8156_priv *es8156 = dev_get_drvdata(dev);

	regcache_cache_only(es8156->regmap, false);
	regcache_sync(es8156->regmap);
	queue_delayed_work(system_power_efficient_wq, &es8156->work,
			   msecs_to_jiffies(es8156->debounce_time));
	return 0;
}

static const struct dev_pm_ops es8156_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(es8156_suspend, es8156_resume)
};

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
		.pm = &es8156_pm,
	},
	.probe = es8156_i2c_probe,
	.remove = es8156_i2c_remove,
	.shutdown = es8156_i2c_shutdown,
	.id_table = es8156_i2c_id,
};
module_i2c_driver(es8156_i2c_driver);

MODULE_DESCRIPTION("ASoC Everest ES8156 audio DAC driver");
MODULE_AUTHOR("Will <pengxiaoxin@everest-semi.com>");
MODULE_AUTHOR("Mainline Linux port for Yandex Station Max");
MODULE_LICENSE("GPL v2");
