// SPDX-License-Identifier: GPL-2.0-only
/*
 * gowin_led_screen.c -- Gowin FPGA 25x16 / 28x16 SPI LED Matrix & Touch Driver
 *
 * Copyright (C) Yandex LLC
 * Complete mainline Linux port matching the exact Yandex protocol from gowin_led_device.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/input.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include "yandex_fpga_bitstream.h"

#define DEFAULT_WIDTH           25
#define DEFAULT_HEIGHT          16
#define MAX_BUFFER_SIZE         (320 * 320)

/* Yandex SPI Protocol Commands */
#define CMD_NOP                 0x00
#define CMD_GET_STATUS          0x01
#define CMD_SHOW_PIC            0x03
#define CMD_SET_BRIGHTNESS      0x05
#define CMD_SET_GPIO            0x06
#define CMD_SET_ANIMATION       0x07
#define CMD_UART_SEND_DATA      0x09
#define CMD_UART_RECV_DATA      0x0a
#define CMD_UART_FLUSH_DATA     0x0b
#define CMD_SET_POWER_SAVING    0x0c
#define CMD_WRITE               0x10
#define CMD_GET_STATUS_LEN      32

/* Gowin JTAG Commands (TN653) */
#define GOWIN_JTAG_NOOP         0x02
#define GOWIN_JTAG_ERASE        0x05
#define GOWIN_JTAG_IDCODE       0x11
#define GOWIN_JTAG_USERCODE     0x13
#define GOWIN_JTAG_PROG         0x15
#define GOWIN_JTAG_RECONFIG     0x3c

struct gowin_resolution {
	int width;
	int height;
};

static const struct gowin_resolution res_panel_25x16 = { .width = 25, .height = 16 };
static const struct gowin_resolution res_screen_28x16 = { .width = 28, .height = 16 };

struct gowin_led_screen {
	struct spi_device *spi;
	struct device *dev;
	struct fb_info *fb;
	struct backlight_device *bl;
	struct input_dev *input;
	u8 *vmem;
	u8 *tx_buf;
	int width;
	int height;
	int buffer_size;
	struct mutex lock;
	struct mutex tx_lock;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *reconfig_gpio;
	struct gpio_desc *mode_gpio;
	struct gpio_desc *touch_irq_gpio;
	struct gpio_desc *jtag_tck_gpio;
	struct gpio_desc *jtag_tms_gpio;
	struct gpio_desc *jtag_tdi_gpio;
	struct gpio_desc *jtag_tdo_gpio;

	int touch_irq;
	struct work_struct touch_work;
	unsigned int brightness;
	unsigned int max_brightness;
	bool power_saving;
	u32 idcode;
	u32 usercode;
	char fw_upd_status[64];
};

/* --- Low-level SPI Commands (Yandex Protocol) --- */

static int show_pic_app_cmd(struct gowin_led_screen *screen)
{
	u8 buf[3] = { CMD_SHOW_PIC, 0x00, 0x00 };
	return spi_write(screen->spi, buf, 3);
}

static int set_brightness_app_cmd(struct gowin_led_screen *screen, unsigned int val)
{
	u8 buf[3] = { CMD_SET_BRIGHTNESS, (u8)(val & 0xff), (u8)((val >> 8) & 0xff) };
	return spi_write(screen->spi, buf, 3);
}

static int set_power_saving_app_cmd(struct gowin_led_screen *screen, bool on)
{
	u8 buf[2] = { CMD_SET_POWER_SAVING, on ? 0x01 : 0x00 };
	return spi_write(screen->spi, buf, 2);
}

static bool gowin_fpga_is_alive(struct gowin_led_screen *screen)
{
	u8 cmd = CMD_GET_STATUS;
	u8 status[CMD_GET_STATUS_LEN] = { 0 };
	struct spi_transfer t[2] = {
		{ .tx_buf = &cmd, .len = 1 },
		{ .rx_buf = status, .len = sizeof(status) },
	};
	bool all_zeros = true, all_ones = true;
	int ret, i;

	ret = spi_sync_transfer(screen->spi, t, 2);
	if (ret)
		return false;

	for (i = 0; i < sizeof(status); i++) {
		if (status[i] != 0x00)
			all_zeros = false;
		if (status[i] != 0xff)
			all_ones = false;
	}
	return !(all_zeros || all_ones);
}

/* --- Gowin JTAG Bit-Banging Engine --- */

static void jtag_clock_cycle(struct gowin_led_screen *screen, int tms, int tdi)
{
	if (screen->jtag_tms_gpio)
		gpiod_set_value(screen->jtag_tms_gpio, tms ? 1 : 0);
	if (screen->jtag_tdi_gpio)
		gpiod_set_value(screen->jtag_tdi_gpio, tdi ? 1 : 0);
	ndelay(100);
	if (screen->jtag_tck_gpio)
		gpiod_set_value(screen->jtag_tck_gpio, 1);
	ndelay(100);
	if (screen->jtag_tck_gpio)
		gpiod_set_value(screen->jtag_tck_gpio, 0);
	ndelay(100);
}

static void jtag_tap_reset(struct gowin_led_screen *screen)
{
	int i;
	for (i = 0; i < 6; i++)
		jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
}

static void jtag_write_instruction(struct gowin_led_screen *screen, u8 inst, int bits)
{
	int i;
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	jtag_clock_cycle(screen, 0, 0);

	for (i = 0; i < bits; i++) {
		int last = (i == bits - 1);
		int bit = (inst >> i) & 1;
		jtag_clock_cycle(screen, last ? 1 : 0, bit);
	}
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
}

static u32 jtag_read_data(struct gowin_led_screen *screen, int bits)
{
	u32 data = 0;
	int i;

	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	jtag_clock_cycle(screen, 0, 0);

	for (i = 0; i < bits; i++) {
		int last = (i == bits - 1);
		int tdo_val = 0;
		if (screen->jtag_tdo_gpio)
			tdo_val = gpiod_get_value(screen->jtag_tdo_gpio);
		if (tdo_val)
			data |= (1U << i);
		jtag_clock_cycle(screen, last ? 1 : 0, 0);
	}
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	return data;
}

static int jtag_prog_fpga(struct gowin_led_screen *screen, const u8 *bitstream, size_t size)
{
	size_t i;
	int b;

	mutex_lock(&screen->lock);
	strscpy(screen->fw_upd_status, "in_progress", sizeof(screen->fw_upd_status));

	jtag_tap_reset(screen);
	jtag_write_instruction(screen, GOWIN_JTAG_IDCODE, 8);
	screen->idcode = jtag_read_data(screen, 32);

	/* Erase SRAM/flash */
	jtag_write_instruction(screen, GOWIN_JTAG_ERASE, 8);
	msleep(100);

	/* Program bitstream */
	jtag_write_instruction(screen, GOWIN_JTAG_PROG, 8);

	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	jtag_clock_cycle(screen, 0, 0);

	for (i = 0; i < size; i++) {
		u8 byte = bitstream[i];
		for (b = 0; b < 8; b++) {
			int last = (i == size - 1 && b == 7);
			int bit = (byte >> b) & 1;
			jtag_clock_cycle(screen, last ? 1 : 0, bit);
		}
	}
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);

	/* Reconfig device */
	jtag_write_instruction(screen, GOWIN_JTAG_RECONFIG, 8);
	msleep(50);
	jtag_tap_reset(screen);

	strscpy(screen->fw_upd_status, "success", sizeof(screen->fw_upd_status));
	mutex_unlock(&screen->lock);
	dev_info(screen->dev, "FPGA programmed successfully (%zu bytes)\n", size);
	return 0;
}

/* --- Framebuffer Operations (Yandex Protocol: CMD_WRITE + CMD_SHOW_PIC) --- */

static int gowin_led_screen_update(struct gowin_led_screen *screen)
{
	int len = screen->width * screen->height;
	int ret;

	if (screen->power_saving)
		return 0;

	mutex_lock(&screen->tx_lock);

	screen->tx_buf[0] = CMD_WRITE;
	screen->tx_buf[1] = len & 0xff;
	screen->tx_buf[2] = (len >> 8) & 0xff;
	memcpy(screen->tx_buf + 3, screen->vmem, len);

	ret = spi_write(screen->spi, screen->tx_buf, 3 + len);
	if (ret == 0)
		ret = show_pic_app_cmd(screen);
	else
		dev_err(screen->dev, "CMD_WRITE failed: %d\n", ret);

	mutex_unlock(&screen->tx_lock);
	return ret;
}

static int gowin_fb_sync(struct fb_info *info)
{
	return gowin_led_screen_update(info->par);
}

static ssize_t gowin_fb_write(struct fb_info *info, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	ssize_t res = fb_sys_write(info, buf, count, ppos);
	if (res > 0)
		gowin_led_screen_update(info->par);
	return res;
}

static int gowin_fb_blank(int blank, struct fb_info *info)
{
	struct gowin_led_screen *screen = info->par;
	screen->power_saving = (blank != FB_BLANK_UNBLANK);
	set_power_saving_app_cmd(screen, screen->power_saving);
	if (!screen->power_saving)
		gowin_led_screen_update(screen);
	return 0;
}

static const struct fb_ops gowin_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_read        = fb_sys_read,
	.fb_write       = gowin_fb_write,
	.fb_fillrect    = sys_fillrect,
	.fb_copyarea    = sys_copyarea,
	.fb_imageblit   = sys_imageblit,
	.fb_blank       = gowin_fb_blank,
	.fb_sync        = gowin_fb_sync,
};

/* --- Backlight Operations --- */

static int gowin_bl_update_status(struct backlight_device *bl)
{
	struct gowin_led_screen *screen = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);
	screen->brightness = brightness;
	return set_brightness_app_cmd(screen, brightness);
}

static int gowin_bl_get_brightness(struct backlight_device *bl)
{
	struct gowin_led_screen *screen = bl_get_data(bl);
	return screen->brightness;
}

static const struct backlight_ops gowin_bl_ops = {
	.update_status  = gowin_bl_update_status,
	.get_brightness = gowin_bl_get_brightness,
};

/* --- Touch Subsystem --- */

static void touch_work_func(struct work_struct *work)
{
	struct gowin_led_screen *screen = container_of(work, struct gowin_led_screen, touch_work);
	u8 cmd = CMD_GET_STATUS;
	u8 status[CMD_GET_STATUS_LEN] = { 0 };
	struct spi_transfer t[2] = {
		{ .tx_buf = &cmd, .len = 1 },
		{ .rx_buf = status, .len = sizeof(status) },
	};
	int ret;

	mutex_lock(&screen->lock);
	ret = spi_sync_transfer(screen->spi, t, 2);
	mutex_unlock(&screen->lock);

	if (ret == 0 && screen->input) {
		bool touched = (status[0] & 0x01) || (status[1] & 0x01);
		input_report_key(screen->input, BTN_TOUCH, touched ? 1 : 0);
		input_sync(screen->input);
	}
}

static irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	struct gowin_led_screen *screen = dev_id;
	schedule_work(&screen->touch_work);
	return IRQ_HANDLED;
}

/* --- Sysfs Attributes --- */

static ssize_t prog_fpga_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "yandex_led_screen_fpga.bin", dev);
	if (ret == 0) {
		jtag_prog_fpga(screen, fw->data, fw->size);
		release_firmware(fw);
	} else {
		/* Fall back to embedded bitstream */
		jtag_prog_fpga(screen, fpga_bitstream, sizeof(fpga_bitstream));
	}
	return count;
}
static DEVICE_ATTR_WO(prog_fpga);

static ssize_t reconfig_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	if (screen->reconfig_gpio) {
		gpiod_set_value_cansleep(screen->reconfig_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(screen->reconfig_gpio, 0);
	}
	return count;
}
static DEVICE_ATTR_WO(reconfig);

static ssize_t fw_upd_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", screen->fw_upd_status);
}
static DEVICE_ATTR_RO(fw_upd_status);

static ssize_t brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", screen->brightness);
}

static ssize_t brightness_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	unsigned int val;
	if (kstrtouint(buf, 0, &val) == 0) {
		screen->brightness = min_t(unsigned int, val, screen->max_brightness);
		set_brightness_app_cmd(screen, screen->brightness);
	}
	return count;
}
static DEVICE_ATTR_RW(brightness);

static struct attribute *gowin_screen_attrs[] = {
	&dev_attr_prog_fpga.attr,
	&dev_attr_reconfig.attr,
	&dev_attr_fw_upd_status.attr,
	&dev_attr_brightness.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gowin_screen);

/* --- Probe & Remove --- */

static int gowin_led_screen_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct gowin_led_screen *screen;
	struct fb_info *fb;
	const struct of_device_id *match;
	const struct gowin_resolution *res = NULL;
	u32 val;
	int ret;

	screen = devm_kzalloc(dev, sizeof(*screen), GFP_KERNEL);
	if (!screen)
		return -ENOMEM;

	screen->spi = spi;
	screen->dev = dev;
	mutex_init(&screen->lock);
	mutex_init(&screen->tx_lock);
	INIT_WORK(&screen->touch_work, touch_work_func);
	spi_set_drvdata(spi, screen);

	/* Determine resolution */
	match = of_match_device(dev->driver->of_match_table, dev);
	if (match && match->data)
		res = match->data;

	if (res) {
		screen->width = res->width;
		screen->height = res->height;
	} else {
		screen->width = DEFAULT_WIDTH;
		screen->height = DEFAULT_HEIGHT;
	}

	if (!of_property_read_u32(dev->of_node, "width", &val))
		screen->width = val;
	if (!of_property_read_u32(dev->of_node, "height", &val))
		screen->height = val;

	screen->buffer_size = screen->width * screen->height;
	screen->max_brightness = 200;
	if (!of_property_read_u32(dev->of_node, "max-brightness", &val))
		screen->max_brightness = val;
	screen->brightness = screen->max_brightness;

	/* GPIO assignments */
	screen->reconfig_gpio = devm_gpiod_get_optional(dev, "reconfig", GPIOD_OUT_LOW);
	if (!screen->reconfig_gpio)
		screen->reconfig_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	screen->mode_gpio = devm_gpiod_get_optional(dev, "mode", GPIOD_OUT_LOW);
	screen->touch_irq_gpio = devm_gpiod_get_optional(dev, "touch-irq", GPIOD_IN);
	if (!screen->touch_irq_gpio)
		screen->touch_irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);

	screen->jtag_tck_gpio = devm_gpiod_get_optional(dev, "jtag-tck", GPIOD_OUT_LOW);
	screen->jtag_tms_gpio = devm_gpiod_get_optional(dev, "jtag-tms", GPIOD_OUT_HIGH);
	screen->jtag_tdi_gpio = devm_gpiod_get_optional(dev, "jtag-tdi", GPIOD_OUT_LOW);
	screen->jtag_tdo_gpio = devm_gpiod_get_optional(dev, "jtag-tdo", GPIOD_IN);

	/* Check if FPGA application protocol is already running */
	if (gowin_fpga_is_alive(screen)) {
		strscpy(screen->fw_upd_status, "skipped", sizeof(screen->fw_upd_status));
		dev_info(dev, "Gowin FPGA already running application protocol, skipping reflash\n");
	} else {
		/* Flash embedded bitstream */
		jtag_prog_fpga(screen, fpga_bitstream, sizeof(fpga_bitstream));
	}

	/* Allocate Framebuffer */
	fb = framebuffer_alloc(0, dev);
	if (!fb)
		return -ENOMEM;

	screen->fb = fb;
	screen->vmem = devm_kzalloc(dev, screen->buffer_size, GFP_KERNEL);
	screen->tx_buf = devm_kzalloc(dev, screen->buffer_size + 8, GFP_KERNEL);
	if (!screen->vmem || !screen->tx_buf) {
		framebuffer_release(fb);
		return -ENOMEM;
	}

	fb->par = screen;
	fb->fbops = &gowin_fb_ops;
	fb->screen_base = (char __iomem *)screen->vmem;
	fb->screen_size = screen->buffer_size;

	fb->var.xres = screen->width;
	fb->var.yres = screen->height;
	fb->var.xres_virtual = screen->width;
	fb->var.yres_virtual = screen->height;
	fb->var.bits_per_pixel = 8;
	fb->var.grayscale = 1;

	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	fb->fix.line_length = screen->width;
	fb->fix.smem_len = screen->buffer_size;
	strscpy(fb->fix.id, "gowin_led", sizeof(fb->fix.id));

	ret = register_framebuffer(fb);
	if (ret) {
		dev_err(dev, "Failed to register framebuffer: %d\n", ret);
		framebuffer_release(fb);
		return ret;
	}

	/* Backlight class device */
	{
		struct backlight_properties props = {
			.type = BACKLIGHT_RAW,
			.max_brightness = screen->max_brightness,
			.brightness = screen->brightness,
		};
		screen->bl = devm_backlight_device_register(dev, "gowin-backlight",
							    dev, screen, &gowin_bl_ops, &props);
		if (IS_ERR(screen->bl))
			dev_warn(dev, "Failed to register backlight device\n");
	}

	/* Touch input registration */
	screen->input = devm_input_allocate_device(dev);
	if (screen->input) {
		screen->input->name = "gowin-touch";
		screen->input->phys = "spi/gowin_touch";
		screen->input->id.bustype = BUS_SPI;

		input_set_capability(screen->input, EV_KEY, BTN_TOUCH);

		ret = input_register_device(screen->input);
		if (ret) {
			dev_warn(dev, "Failed to register touch input: %d\n", ret);
			screen->input = NULL;
		}
	}

	/* Request IRQ if available */
	if (screen->touch_irq_gpio) {
		screen->touch_irq = gpiod_to_irq(screen->touch_irq_gpio);
		if (screen->touch_irq > 0) {
			ret = devm_request_threaded_irq(dev, screen->touch_irq,
							NULL, touch_irq_handler,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							"gowin_touch", screen);
			if (ret)
				dev_warn(dev, "Failed to request touch IRQ: %d\n", ret);
		}
	}

	/* Set initial brightness */
	set_brightness_app_cmd(screen, screen->brightness);

	dev_info(dev, "Gowin FPGA LED screen (%dx%d) & touch driver loaded\n",
		 screen->width, screen->height);
	return 0;
}

static void gowin_led_screen_remove(struct spi_device *spi)
{
	struct gowin_led_screen *screen = spi_get_drvdata(spi);

	cancel_work_sync(&screen->touch_work);
	if (screen->fb) {
		unregister_framebuffer(screen->fb);
		framebuffer_release(screen->fb);
	}
}

static const struct of_device_id gowin_led_screen_of_match[] = {
	{ .compatible = "gowin,led-screen", .data = &res_panel_25x16 },
	{ .compatible = "yandex,station-max-led-screen", .data = &res_screen_28x16 },
	{ .compatible = "yandex,led-screen", .data = &res_panel_25x16 },
	{ .compatible = "ya,led_screen", .data = &res_screen_28x16 },
	{ .compatible = "ya,led_panel", .data = &res_panel_25x16 },
	{ .compatible = "atri,led-panel", .data = &res_panel_25x16 },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_led_screen_of_match);

static struct spi_driver gowin_led_screen_driver = {
	.driver = {
		.name = "gowin_led_screen",
		.of_match_table = gowin_led_screen_of_match,
		.dev_groups = gowin_screen_groups,
	},
	.probe = gowin_led_screen_probe,
	.remove = gowin_led_screen_remove,
};
module_spi_driver(gowin_led_screen_driver);

MODULE_DESCRIPTION("Gowin FPGA 25x16/28x16 LED Screen & Touch Driver (Yandex Protocol)");
MODULE_AUTHOR("Yandex LLC / Mainline Linux port");
MODULE_LICENSE("GPL v2");
