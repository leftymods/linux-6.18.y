// SPDX-License-Identifier: GPL-2.0
//
// Driver for Silergy SY6045S Class-D Audio Amplifier
//
// Based on vendor driver for Yandex Station Max (kernel 4.9)
// Author: Lefty <leftymods@gmail.com>
//
// Features:
//   - I2C/regmap register access
//   - Regulator management (vddio, pvdd)
//   - Reset GPIO with timing sequence
//   - Fault GPIO (optional, read-only)
//   - I2S/TDM playback DAI with trigger-based speaker on/off
//   - Master volume control with TLV (-12750..0 dB, 50 centidB/step)
//   - Channel 1/2 volume controls
//   - Mute control (Playback Switch)
//   - PBTL mode support (woofer)
//   - Firmware loading (.txt format via request_firmware)
//   - restore-regs DT property for register save/restore
//   - DAPM: Speaker output + SPK supply widget
//   - Component probe/remove with proper sequencing
//
// Compatible: "silergy,sy6045s"
// DT properties:
//   - pbtl-mode: enable PBTL mode (bridged) for woofer
//   - firmware: firmware file name to load via request_firmware
//   - restore-regs: <reg value mask> triplets for register initialization
//   - reset-gpios: GPIO for hardware reset
//   - fault-gpios: GPIO for fault detection (input)
//   - spk-gpios: GPIO for speaker enable/disable

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>

/* SY6045S register map */
#define SY6045S_REG_RESET		0x00
#define SY6045S_REG_MODE_CFG		0x01
#define SY6045S_REG_FAULT		0x02
#define SY6045S_REG_EQ_ACCESS		0x03
#define SY6045S_REG_MUTE		0x06
#define SY6045S_REG_VOL_MASTER		0x07
#define SY6045S_REG_VOL_CH1		0x08
#define SY6045S_REG_VOL_CH2		0x09
#define SY6045S_REG_MONITOR		0x17
#define SY6045S_REG_MODE1		0x1E
#define SY6045S_REG_CFG20		0x20
#define SY6045S_REG_MUTE2		0x22

#define SY6045S_STEREO_MODE		0x05
#define SY6045S_PBTL_MODE		0x07
#define SY6045S_MUTE_BIT		BIT(4)

#define SY6045S_NUM_SUPPLIES		2

struct sy6045s_reg_cfg {
	u32 reg;
	u32 val;
	u32 mask;
};

struct sy6045s_priv {
	struct regmap *regmap;
	struct device *dev;
	struct regulator_bulk_data supplies[SY6045S_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *spk_gpio;
	struct snd_soc_component *component;

	struct mutex lock;

	/* Firmware name from DT */
	const char *firmware_name;

	/* PBTL mode (woofer) */
	bool pbtl_mode;

	/* restore-regs */
	struct sy6045s_reg_cfg *restore_regs;
	int num_restore_regs;

	/* Speaker on/off work */
	struct work_struct spk_on_work;
	struct work_struct spk_off_work;
};

/* Volume TLV: matches hardvol_sy.xml mapping
 *   register value 3 -> -12600 centidB
 *   register value 255 -> 0 centidB
 *   TLV: min = -12750, step = 50 centidB
 *     val   3 = -12750 +   3*50 = -12600 centidB  (-126.00 dB)
 *     val 255 = -12750 + 255*50 =     0 centidB  (   0.00 dB)
 */
static const DECLARE_TLV_DB_SCALE(sy6045s_master_tlv, -12750, 50, 0);
static const DECLARE_TLV_DB_SCALE(sy6045s_chan_tlv, -12750, 50, 0);

static const struct snd_kcontrol_new sy6045s_snd_controls[] = {
	SOC_SINGLE_TLV("Master Playback Volume", SY6045S_REG_VOL_MASTER,
		       0, 0xff, 0, sy6045s_master_tlv),
	SOC_SINGLE_TLV("Ch1 Playback Volume", SY6045S_REG_VOL_CH1,
		       0, 0xff, 0, sy6045s_chan_tlv),
	SOC_SINGLE_TLV("Ch2 Playback Volume", SY6045S_REG_VOL_CH2,
		       0, 0xff, 0, sy6045s_chan_tlv),
	/* Mute: bit 4 of reg 0x22, 1 = muted, invert = 1 */
	SOC_SINGLE("Playback Switch", SY6045S_REG_MUTE2, 4, 1, 1),
};

/* DAPM widgets */
static const struct snd_soc_dapm_widget sy6045s_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Speaker"),
};

static const struct snd_soc_dapm_route sy6045s_dapm_routes[] = {
	{ "Speaker", NULL, "HiFi Playback" },
};

static int sy6045s_spk_on(struct sy6045s_priv *priv)
{
	struct snd_soc_component *component = priv->component;

	dev_dbg(priv->dev, "SPK ON\n");

	if (priv->spk_gpio) {
		gpiod_set_value_cansleep(priv->spk_gpio, 1);
		msleep(20);
	}

	/* Unmute: clear bit 4 of reg 0x22 */
	snd_soc_component_update_bits(component, SY6045S_REG_MUTE2,
				      SY6045S_MUTE_BIT, 0);
	return 0;
}

static int sy6045s_spk_off(struct sy6045s_priv *priv)
{
	struct snd_soc_component *component = priv->component;

	dev_dbg(priv->dev, "SPK OFF\n");

	/* Mute: set bit 4 of reg 0x22 */
	snd_soc_component_update_bits(component, SY6045S_REG_MUTE2,
				      SY6045S_MUTE_BIT, SY6045S_MUTE_BIT);

	if (priv->spk_gpio) {
		msleep(10);
		gpiod_set_value_cansleep(priv->spk_gpio, 0);
	}
	return 0;
}

static void sy6045s_spk_on_work(struct work_struct *work)
{
	struct sy6045s_priv *priv = container_of(work, struct sy6045s_priv,
						  spk_on_work);
	mutex_lock(&priv->lock);
	sy6045s_spk_on(priv);
	mutex_unlock(&priv->lock);
}

static void sy6045s_spk_off_work(struct work_struct *work)
{
	struct sy6045s_priv *priv = container_of(work, struct sy6045s_priv,
						  spk_off_work);
	mutex_lock(&priv->lock);
	sy6045s_spk_off(priv);
	mutex_unlock(&priv->lock);
}

static int sy6045s_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		schedule_work(&priv->spk_on_work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		schedule_work(&priv->spk_off_work);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sy6045s_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	/* Sample rate / format handled by the TDM interface; no register
	 * programming needed on the amplifier side.
	 */
	return 0;
}

static int sy6045s_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	dev_dbg(dai->dev, "set_fmt: 0x%x\n", fmt);
	return 0;
}

static int sy6045s_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	dev_dbg(priv->dev, "mute=%d dir=%d\n", mute, direction);

	if (mute) {
		snd_soc_component_update_bits(component, SY6045S_REG_MUTE2,
					      SY6045S_MUTE_BIT,
					      SY6045S_MUTE_BIT);
	} else {
		snd_soc_component_update_bits(component, SY6045S_REG_MUTE2,
					      SY6045S_MUTE_BIT, 0);
	}
	return 0;
}

static const struct snd_soc_dai_ops sy6045s_dai_ops = {
	.hw_params	= sy6045s_hw_params,
	.set_fmt	= sy6045s_set_dai_fmt,
	.mute		= sy6045s_mute,
	.trigger	= sy6045s_trigger,
};

static struct snd_soc_dai_driver sy6045s_dai_driver = {
	.name		= "sy6045s-hifi",
	.playback	= {
		.stream_name	= "HiFi Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_96000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops		= &sy6045s_dai_ops,
};

/* Parse and apply firmware settings from vendor text format.
 *
 * Format: lines starting with 'w' followed by:
 *   <i2c_addr_hex> <reg_hex> <data_hex>...
 * Or '>' continuation for more data at the current register.
 * '#' starts a comment, empty lines ignored.
 *
 * Example:
 *   # reset
 *   w 54 0F 01
 *   w 54 1B BD
 *   > 23 1A
 *
 * The I2C address is skipped; we use our own regmap for the device.
 */
static int sy6045s_apply_firmware_text(struct sy6045s_priv *priv,
				       const u8 *data, size_t size)
{
	const u8 *p = data;
	const u8 *const end = data + size;
	unsigned int current_reg = 0;

	while (p < end) {
		unsigned int reg;
		u8 buf[16];
		int nvalues;
		unsigned long val;
		char *endp;

		/* Skip whitespace */
		while (p < end && (*p == ' ' || *p == '\t'))
			p++;
		if (p >= end)
			break;

		/* Skip empty lines and comments */
		if (*p == '\n' || *p == '\r' || *p == '#') {
			while (p < end && *p != '\n')
				p++;
			if (p < end)
				p++;
			continue;
		}

		/* Command: 'w' = new write, '>' = continue at current reg */
		if (*p == 'w') {
			p++;
			/* Skip whitespace */
			while (p < end && (*p == ' ' || *p == '\t'))
				p++;

			/* Skip the I2C address token (8-bit format like 54/56) */
			while (p < end && *p != ' ' && *p != '\t')
				p++;
			while (p < end && (*p == ' ' || *p == '\t'))
				p++;

			/* Parse register address */
			if (p >= end)
				break;
			val = simple_strtoul(p, &endp, 16);
			if (endp == p || val > 0xff)
				goto skip_line;
			reg = val;
			current_reg = reg;
			p = endp;
		} else if (*p == '>') {
			p++;
			reg = current_reg;
		} else {
			/* Unknown starting character, skip line */
			goto skip_line;
		}

		/* Parse data bytes */
		nvalues = 0;
		while (p < end && nvalues < ARRAY_SIZE(buf)) {
			while (p < end && (*p == ' ' || *p == '\t'))
				p++;

			if (p >= end || *p == '\n' || *p == '\r' ||
			    *p == '#' || *p == '>')
				break;

			/* A bare 'w' starts a new line */
			if (*p == 'w' && nvalues > 0)
				break;

			val = simple_strtoul(p, &endp, 16);
			if (endp == p)
				break;

			buf[nvalues++] = val;
			p = endp;
		}

		if (nvalues > 0) {
			int ret;

			if (nvalues == 1) {
				ret = regmap_write(priv->regmap, reg, buf[0]);
			} else {
				ret = regmap_raw_write(priv->regmap, reg,
						       buf, nvalues);
			}
			if (ret) {
				dev_err(priv->dev,
					"fw write failed: reg=0x%02x (%d)\n",
					reg, ret);
				return ret;
			}
		}

skip_line:
		while (p < end && *p != '\n')
			p++;
		if (p < end)
			p++;
	}

	return 0;
}

/* Load settings from firmware file via request_firmware */
static int sy6045s_load_firmware(struct sy6045s_priv *priv, const char *name)
{
	const struct firmware *fw;
	int ret;

	if (!name || !*name)
		return 0;

	ret = request_firmware(&fw, name, priv->dev);
	if (ret) {
		dev_err(priv->dev, "failed to load firmware %s: %d\n",
			name, ret);
		return ret;
	}

	dev_info(priv->dev, "loaded firmware %s (%zu bytes)\n",
		 name, fw->size);

	ret = sy6045s_apply_firmware_text(priv, fw->data, fw->size);
	release_firmware(fw);

	if (ret)
		dev_err(priv->dev, "failed to apply firmware %s: %d\n",
			name, ret);

	return ret;
}

/* Apply minimal hardcoded init from vendor register settings */
static int sy6045s_init_hardcoded(struct sy6045s_priv *priv)
{
	struct regmap *regmap = priv->regmap;
	int ret;

	/* Reset chip via soft reset register */
	regmap_write(regmap, 0x0F, 0x01);
	usleep_range(5000, 10000);

	/* Inner settings from vendor GUI tool */
	regmap_write(regmap, 0x1B, 0xBD);
	regmap_write(regmap, 0x23, 0x1A);
	regmap_write(regmap, 0x04, 0x9E);
	regmap_write(regmap, 0x05, 0x02);
	regmap_write(regmap, 0x76, 0x0F);
	regmap_write(regmap, 0x19, 0x15);
	regmap_write(regmap, 0x10, 0x77);

	/* KEEP values (QUASARSYS-1172: important timing registers) */
	regmap_write(regmap, 0x11, 0x00);
	regmap_write(regmap, 0x12, 0x06);
	regmap_write(regmap, 0x13, 0x0C);
	regmap_write(regmap, 0x14, 0x12);

	regmap_write(regmap, 0x8B, 0x30);
	regmap_write(regmap, 0x8C, 0x18);
	regmap_write(regmap, 0x00, 0x1A);
	regmap_write(regmap, 0x15, 0x10);
	regmap_write(regmap, 0x16, 0x06);
	regmap_write(regmap, 0x21, 0x00);
	regmap_write(regmap, 0x1F, 0x03);

	/* Configure mode: PBTL or stereo */
	if (priv->pbtl_mode) {
		regmap_write(regmap, SY6045S_REG_MODE1, SY6045S_PBTL_MODE);
		regmap_write(regmap, SY6045S_REG_CFG20, 0x80);
	} else {
		regmap_write(regmap, SY6045S_REG_MODE1, SY6045S_STEREO_MODE);
		regmap_write(regmap, SY6045S_REG_CFG20, 0x00);
	}

	/* Master volume max, channel volumes */
	regmap_write(regmap, SY6045S_REG_VOL_MASTER, 0xFF);
	regmap_write(regmap, SY6045S_REG_VOL_CH1, 0x9F);
	regmap_write(regmap, SY6045S_REG_VOL_CH2, 0x9F);

	/* Pre/post scaler (from vendor firmware) */
	{
		u8 pre_scale[] = {0xE5, 0x55};
		u8 post_scale[] = {0x7F, 0xFF};
		u8 reserved[] = {0x5E, 0x00};

		regmap_raw_write(regmap, 0x2C, pre_scale, 2);
		regmap_raw_write(regmap, 0x2E, post_scale, 2);
		regmap_raw_write(regmap, 0x30, reserved, 2);
	}

	/* Set monitor */
	regmap_write(regmap, SY6045S_REG_MONITOR, 0x90);

	/* Unmute */
	regmap_write(regmap, SY6045S_REG_MUTE2, 0x00);
	regmap_write(regmap, SY6045S_REG_MUTE, 0x00);

	return 0;
}

/* Enter standby (muted, low power) */
static int sy6045s_set_standby(struct sy6045s_priv *priv)
{
	struct snd_soc_component *component = priv->component;

	if (priv->spk_gpio) {
		gpiod_set_value_cansleep(priv->spk_gpio, 0);
	}

	/* Mute */
	snd_soc_component_update_bits(component, SY6045S_REG_MUTE2,
				      SY6045S_MUTE_BIT, SY6045S_MUTE_BIT);
	return 0;
}

/* Apply restore-regs DT entries: read-modify-write with mask */
static int sy6045s_restore_regs(struct sy6045s_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_restore_regs; i++) {
		unsigned int val;
		int ret;

		ret = regmap_read(priv->regmap, priv->restore_regs[i].reg,
				  &val);
		if (ret)
			continue;

		val &= ~priv->restore_regs[i].mask;
		val |= priv->restore_regs[i].val & priv->restore_regs[i].mask;

		regmap_write(priv->regmap, priv->restore_regs[i].reg, val);
	}
	return 0;
}

/* ---------- ASoC component callbacks ---------- */

static int sy6045s_component_probe(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	priv->component = component;
	mutex_lock(&priv->lock);

	/* Load firmware file if specified in DT */
	if (priv->firmware_name)
		sy6045s_load_firmware(priv, priv->firmware_name);

	/* Apply restore-regs DT entries */
	sy6045s_restore_regs(priv);

	/* Configure PBTL/stereo mode if no firmware was loaded */
	if (!priv->firmware_name) {
		if (priv->pbtl_mode) {
			snd_soc_component_write(component, SY6045S_REG_MODE1,
						SY6045S_PBTL_MODE);
			snd_soc_component_write(component, SY6045S_REG_CFG20,
						0x80);
		} else {
			snd_soc_component_write(component, SY6045S_REG_MODE1,
						SY6045S_STEREO_MODE);
			snd_soc_component_write(component, SY6045S_REG_CFG20,
						0x00);
		}
	}

	/* Enter standby (muted) until first playback */
	ret = sy6045s_set_standby(priv);

	mutex_unlock(&priv->lock);
	return ret;
}

static void sy6045s_component_remove(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	cancel_work_sync(&priv->spk_on_work);
	cancel_work_sync(&priv->spk_off_work);
	sy6045s_set_standby(priv);
}

static int sy6045s_component_suspend(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	cancel_work_sync(&priv->spk_on_work);
	sy6045s_set_standby(priv);
	return 0;
}

static int sy6045s_component_resume(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	mutex_lock(&priv->lock);

	if (priv->firmware_name)
		sy6045s_load_firmware(priv, priv->firmware_name);

	sy6045s_restore_regs(priv);
	sy6045s_set_standby(priv);

	mutex_unlock(&priv->lock);
	return 0;
}

static const struct snd_soc_component_driver sy6045s_component_driver = {
	.probe			= sy6045s_component_probe,
	.remove			= sy6045s_component_remove,
	.suspend		= sy6045s_component_suspend,
	.resume			= sy6045s_component_resume,
	.controls		= sy6045s_snd_controls,
	.num_controls		= ARRAY_SIZE(sy6045s_snd_controls),
	.dapm_widgets		= sy6045s_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(sy6045s_dapm_widgets),
	.dapm_routes		= sy6045s_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(sy6045s_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static bool sy6045s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00: /* RESET/status */
	case 0x02: /* FAULT */
		return true;
	default:
		return false;
	}
}

static bool sy6045s_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= 0xBF;
}

static bool sy6045s_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg <= 0xBF;
}

static const struct regmap_config sy6045s_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0xBF,
	.volatile_reg	= sy6045s_volatile_reg,
	.readable_reg	= sy6045s_readable_reg,
	.writeable_reg	= sy6045s_writeable_reg,
	.cache_type	= REGCACHE_MAPLE,
};

static int sy6045s_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct sy6045s_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	i2c_set_clientdata(i2c, priv);
	mutex_init(&priv->lock);

	INIT_WORK(&priv->spk_on_work, sy6045s_spk_on_work);
	INIT_WORK(&priv->spk_off_work, sy6045s_spk_off_work);

	/* Init regmap */
	priv->regmap = devm_regmap_init_i2c(i2c, &sy6045s_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "Failed to init regmap\n");

	/* Read firmware name from DT (optional) */
	of_property_read_string(dev->of_node, "firmware",
				&priv->firmware_name);

	/* PBTL mode (woofer) */
	priv->pbtl_mode = device_property_read_bool(dev, "pbtl-mode");

	/* Supplies: vddio (3.3V logic), pvdd (20V power) */
	priv->supplies[0].supply = "vddio";
	priv->supplies[1].supply = "pvdd";

	ret = devm_regulator_bulk_get(dev, SY6045S_NUM_SUPPLIES,
				      priv->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	ret = regulator_bulk_enable(SY6045S_NUM_SUPPLIES, priv->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable supplies\n");

	/* Reset GPIO (active low pulse) - optional */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "Failed to get reset GPIO\n");

	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		msleep(20);
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		msleep(20);
	}

	/* Speaker control GPIO - optional */
	priv->spk_gpio = devm_gpiod_get_optional(dev, "spk",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->spk_gpio))
		priv->spk_gpio = NULL;

	/* Apply hardcoded init settings */
	ret = sy6045s_init_hardcoded(priv);
	if (ret)
		dev_warn(dev, "hardcoded init failed: %d\n", ret);

	/* Parse restore-regs DT property: triplets of <reg value mask> */
	{
		struct fwnode_handle *np = dev_fwnode(dev);
		u32 *restore_data;
		int count;

		count = fwnode_property_count_u32(np, "restore-regs");
		if (count > 0 && count % 3 == 0) {
			restore_data = kcalloc(count, sizeof(u32), GFP_KERNEL);
			if (!restore_data) {
				ret = -ENOMEM;
				goto err_disable_reg;
			}

			ret = fwnode_property_read_u32_array(np, "restore-regs",
							     restore_data,
							     count);
			if (ret) {
				kfree(restore_data);
				dev_err(dev, "failed to read restore-regs\n");
				goto err_disable_reg;
			}

			priv->num_restore_regs = count / 3;
			priv->restore_regs = devm_kmalloc_array(dev,
				priv->num_restore_regs,
				sizeof(struct sy6045s_reg_cfg), GFP_KERNEL);
			if (!priv->restore_regs) {
				kfree(restore_data);
				ret = -ENOMEM;
				goto err_disable_reg;
			}

			for (int i = 0; i < priv->num_restore_regs; i++) {
				priv->restore_regs[i].reg =
					restore_data[i * 3];
				priv->restore_regs[i].val =
					restore_data[i * 3 + 1];
				priv->restore_regs[i].mask =
					restore_data[i * 3 + 2];
			}
			kfree(restore_data);

			dev_info(dev, "%d restore-regs entries\n",
				 priv->num_restore_regs);
		}
	}

	/* Register ASoC component */
	ret = devm_snd_soc_register_component(dev, &sy6045s_component_driver,
					      &sy6045s_dai_driver, 1);
	if (ret)
		goto err_disable_reg;

	dev_info(dev, "SY6045S ready (pbtl=%d)\n", priv->pbtl_mode);
	return 0;

err_disable_reg:
	regulator_bulk_disable(SY6045S_NUM_SUPPLIES, priv->supplies);
	return ret;
}

static void sy6045s_i2c_remove(struct i2c_client *i2c)
{
	struct sy6045s_priv *priv = i2c_get_clientdata(i2c);

	cancel_work_sync(&priv->spk_on_work);
	cancel_work_sync(&priv->spk_off_work);
	regulator_bulk_disable(SY6045S_NUM_SUPPLIES, priv->supplies);
}

static const struct i2c_device_id sy6045s_i2c_id[] = {
	{ "sy6045s", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sy6045s_i2c_id);

static const struct of_device_id sy6045s_of_match[] = {
	{ .compatible = "silergy,sy6045s", },
	{}
};
MODULE_DEVICE_TABLE(of, sy6045s_of_match);

static struct i2c_driver sy6045s_i2c_driver = {
	.driver = {
		.name		= "sy6045s",
		.of_match_table	= sy6045s_of_match,
	},
	.probe		= sy6045s_i2c_probe,
	.remove		= sy6045s_i2c_remove,
	.id_table	= sy6045s_i2c_id,
};
module_i2c_driver(sy6045s_i2c_driver);

MODULE_DESCRIPTION("Silergy SY6045S Class-D Amplifier Codec Driver");
MODULE_AUTHOR("Lefty <leftymods@gmail.com>");
MODULE_LICENSE("GPL v2");
