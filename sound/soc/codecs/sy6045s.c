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
#define SY6045S_MAX_REGISTER 0xb0
#define SY6045S_RESTORE_REGS_MAX 192

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

	/* firmware name (owned copy when set via sysfs) */
	const char *fw_name;
	char *fw_name_owned;
};

/*
 * Full register range is writable: firmware settings files program DSP/EQ
 * registers across 0x00-0xb0 (vendor GUI exports). Restricting the range
 * here silently drops most of the tuning.
 */
static bool sy6045s_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg <= SY6045S_MAX_REGISTER;
}

static bool sy6045s_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= SY6045S_MAX_REGISTER;
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

static const struct reg_default sy6045s_reg_defaults[] = {
	/* sane power-up state: unmuted, volume ramp enabled */
	{ 0x06, 0x00 },		/* mute/filter control */
	{ 0x07, 0xff },		/* master volume */
};

static const struct regmap_config sy6045s_regmap = {
	.reg_bits     = 8,
	.val_bits     = 8,
	.max_register = SY6045S_MAX_REGISTER,
	.writeable_reg = sy6045s_writeable_reg,
	.readable_reg  = sy6045s_readable_reg,
	.volatile_reg  = sy6045s_volatile_reg,
	.cache_type    = REGCACHE_MAPLE,
	.reg_defaults   = sy6045s_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(sy6045s_reg_defaults),
};

static DECLARE_TLV_DB_SCALE(sy6045s_vol_tlv, -12600, 50, 0);
static DECLARE_TLV_DB_SCALE(sy6045s_ch_vol_tlv, -10050, 50, 0);

static const struct snd_kcontrol_new sy6045s_snd_controls[] = {
	SOC_SINGLE_TLV("Master Playback Volume",
		       0x07, 0, 0xff, 0, sy6045s_vol_tlv),
	SOC_SINGLE_TLV("Ch1 Playback Volume",
		       0x08, 0, 0xff, 0, sy6045s_ch_vol_tlv),
	SOC_SINGLE_TLV("Ch2 Playback Volume",
		       0x09, 0, 0xff, 0, sy6045s_ch_vol_tlv),
};

static const struct snd_soc_dapm_widget sy6045s_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),
	SND_SOC_DAPM_AIF_IN("Playback", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route sy6045s_dapm_routes[] = {
	{ "SPK_OUT", NULL, "Playback" },
};

static int sy6045s_reset_chip(struct sy6045s_priv *priv);
static int sy6045s_restore_regs(struct sy6045s_priv *priv);
static int sy6045s_apply_settings(struct sy6045s_priv *priv,
				  const u8 *data, size_t size);
static int sy6045s_load_firmware(struct sy6045s_priv *priv);

/* -------- sysfs: default_settings ------------- */
/* re-apply reset + restore-regs + current firmware (tuning recovery) */
static ssize_t default_settings_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	int ret;

	sy6045s_reset_chip(priv);

	ret = sy6045s_restore_regs(priv);
	if (ret)
		return ret;

	ret = sy6045s_load_firmware(priv);
	return ret ? ret : count;
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

/* echo <firmware-name> > settings_file : (re)load named settings file */
static ssize_t settings_file_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	char name[64];
	int ret;

	if (count >= sizeof(name))
		return -EINVAL;
	memcpy(name, buf, count);
	name[count] = '\0';
	strim(name);
	if (!name[0])
		return -EINVAL;

	mutex_lock(&priv->io_mutex);
	kfree(priv->fw_name_owned);
	priv->fw_name_owned = kstrdup(name, GFP_KERNEL);
	priv->fw_name = priv->fw_name_owned;
	mutex_unlock(&priv->io_mutex);

	ret = sy6045s_load_firmware(priv);
	return ret ? ret : count;
}

static DEVICE_ATTR_RW(settings_file);

/* -------- sysfs: settings ------------- */
/* echo "w 56 07 aa" > settings : apply raw vendor settings text on the fly */
static ssize_t settings_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sy6045s_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = sy6045s_apply_settings(priv, buf, count);
	return ret ? ret : count;
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
/*
 * Apply vendor GUI-export settings text ("w <i2c> <reg> <val> [val...]").
 * The dump intentionally ends with its own final EQ-lock/volume/unmute
 * state -- no register save/restore around it (would clobber tuning).
 */
{
	unsigned int line_num = 0;

	mutex_lock(&priv->io_mutex);

	while (size > 0) {
		unsigned int i2c_addr, reg_addr, val;
		char cmd;
		int consumed;

		if (size == 0)
			break;

		line_num++;

		while (size > 0 && (*data == ' ' || *data == '\t'))
			data++, size--;
		while (size > 0 && (*data == '\n' || *data == '\r'))
			data++, size--;

		if (size == 0)
			break;

		if (*data == '#') {
			while (size > 0 && *data != '\n' && *data != '\r')
				data++, size--;
			continue;
		}

		if (sscanf(data, " %c %x %x %x%n",
			   &cmd, &i2c_addr, &reg_addr, &val, &consumed) < 4) {
			mutex_unlock(&priv->io_mutex);
			dev_err(&priv->i2c->dev,
				"Error parsing settings at line %d\n", line_num);
			return -EINVAL;
		}

		if (cmd != 'w' && cmd != 'r') {
			mutex_unlock(&priv->io_mutex);
			dev_err(&priv->i2c->dev,
				"Unknown command '%c' at line %d\n",
				cmd, line_num);
			return -EINVAL;
		}

		if (cmd == 'w') {
			unsigned int our_addr = (priv->i2c->addr << 1);

			if (i2c_addr != our_addr) {
				mutex_unlock(&priv->io_mutex);
				dev_err(&priv->i2c->dev,
					"Settings not compatible with amplifier: "
					"fw i2c=0x%02x, chip=0x%02x\n",
					i2c_addr, our_addr);
				return -EINVAL;
			}

			regmap_write(priv->regmap, reg_addr, val);

			/* handle multi-byte writes (reg_addr increments) */
			data += consumed;
			size -= consumed;
			while (size > 0 && (*data == ' ' || *data == '\t')) {
				unsigned int extra_val;
				int nxt;

				data++, size--;
				if (sscanf(data, "%x%n", &extra_val, &nxt) < 1)
					break;
				reg_addr++;
				regmap_write(priv->regmap, reg_addr, extra_val);
				data += nxt;
				size -= nxt;
			}
		}

		while (size > 0 && *data != '\n' && *data != '\r')
			data++, size--;
	}

	mutex_unlock(&priv->io_mutex);

	dev_info(&priv->i2c->dev, "applied %u lines of firmware settings\n",
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
/*
 * Anti-pop power sequencing (mirrors the vendor flow):
 *
 *   1. VDDIO only  — I2C interface alive, output stage still dead
 *   2. reset + restore-regs + firmware settings
 *   3. force mute  (the settings dump itself ends unmuted!)
 *   4. PVDD on     — output stage charges while muted
 *   5. settle 150ms
 *   6. unmute deferred to trigger(SNDRV_PCM_TRIGGER_START)
 *
 * Both amplifiers share the PVDD rail; the regulator core refcounts,
 * so the second probe's enable is a no-op and its settle delay is
 * harmless.
 */
static int sy6045s_component_probe(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val = 0;
	int ret;

	dev_info(component->dev, "anti-pop: step 1 — VDDIO on\n");
	ret = regulator_enable(priv->supplies[0].consumer);
	if (ret) {
		dev_err(component->dev, "anti-pop: VDDIO enable failed: %d\n",
			ret);
		return ret;
	}

	/* Reset chip if GPIO provided */
	sy6045s_reset_chip(priv);
	dev_info(component->dev, "anti-pop: step 2a — chip reset done\n");

	/* Apply restore-regs from DT */
	ret = sy6045s_restore_regs(priv);
	if (ret) {
		dev_err(component->dev,
			"anti-pop: restore-regs failed: %d\n", ret);
		regulator_disable(priv->supplies[0].consumer);
		return ret;
	}
	dev_info(component->dev, "anti-pop: step 2b — %d restore-regs applied\n",
		 priv->num_restore_regs);

	/* Load firmware settings (EQ/DRC tables from /lib/firmware) */
	ret = sy6045s_load_firmware(priv);
	if (ret) {
		dev_warn(component->dev,
			 "anti-pop: firmware load failed: %d "
			 "(amp stays muted on defaults)\n", ret);
	} else {
		dev_info(component->dev,
			 "anti-pop: step 2c — firmware settings applied\n");
	}

	/* Step 3: force mute — never trust the dump's final state here.
	 * Unmute happens exclusively via trigger(START). */
	ret = regmap_write(priv->regmap, 0x06, 0x08);
	if (ret == 0)
		regmap_read(priv->regmap, 0x06, &val);
	dev_info(component->dev,
		 "anti-pop: step 3 — forced mute (reg06=0x%02x, want 0x08)\n",
		 val);

	/* Step 4: PVDD on */
	ret = regulator_enable(priv->supplies[1].consumer);
	if (ret) {
		dev_err(component->dev, "anti-pop: PVDD enable failed: %d\n",
			ret);
		regulator_disable(priv->supplies[0].consumer);
		return ret;
	}

	/* Step 5: output stage charge-up while muted */
	msleep(150);
	dev_info(component->dev,
		 "anti-pop: steps 4-5 — PVDD on, settled 150 ms; "
		 "amp ready, unmute on playback start\n");

	return 0;
}

static void sy6045s_component_remove(struct snd_soc_component *component)
{
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	/* mute first so PVDD collapse is silent */
	regmap_write(priv->regmap, 0x06, 0x08);
	regulator_disable(priv->supplies[1].consumer);	/* PVDD */
	regulator_disable(priv->supplies[0].consumer);	/* VDDIO */
	dev_dbg(component->dev, "powered down (muted before PVDD off)\n");
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

static int sy6045s_trigger(struct snd_pcm_substream *substream,
			   int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sy6045s_priv *priv = snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* Ensure amp is unmuted and active */
		mutex_lock(&priv->io_mutex);
		regmap_write(priv->regmap, 0x06, 0x00);
		regmap_write(priv->regmap, 0x22, 0x00);
		mutex_unlock(&priv->io_mutex);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* Mute amp */
		mutex_lock(&priv->io_mutex);
		regmap_write(priv->regmap, 0x06, 0x08);
		mutex_unlock(&priv->io_mutex);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sy6045s_dai_ops = {
	.hw_params   = sy6045s_hw_params,
	.set_fmt     = sy6045s_set_dai_fmt,
	.mute_stream = sy6045s_mute_stream,
	.trigger     = sy6045s_trigger,
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
	for (int i = 0; i < SY6045S_NUM_SUPPLIES; i++)
		priv->supplies[i].supply = sy6045s_supply_names[i];
	ret = devm_regulator_bulk_get(dev, SY6045S_NUM_SUPPLIES,
				      priv->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	/* firmware name (borrowed from DT node lifetime) */
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
	struct sy6045s_priv *priv = i2c_get_clientdata(i2c);

	snd_soc_unregister_component(&i2c->dev);
	kfree(priv->fw_name_owned);
	mutex_destroy(&priv->io_mutex);
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