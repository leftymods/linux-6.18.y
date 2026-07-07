// SPDX-License-Identifier: GPL-2.0
/*
 * Quasar LED Screen FB driver for Yandex Station 2 (Gowin FPGA)
 *
 * SPI framebuffer driver for 25x16 monochrome LED screen
 * driven by a Gowin FPGA with embedded soft-core MCU.
 *
 * Copyright (C) 2026 leftymods
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "quasar_led_screen"

#define SCR_WIDTH  25
#define SCR_HEIGHT 16
#define SCR_BYTES  ((SCR_WIDTH * SCR_HEIGHT + 7) / 8)

struct qls_device {
	struct spi_device	*spi;
	struct gpio_desc	*reset;
	struct fb_info		*fb;
	struct backlight_device	*bl;
	uint8_t			* videomemory;
	uint32_t		xres;
	uint32_t		yres;
	uint32_t		row_bytes;
	int			max_brightness;
	int			brightness;
};

/* Send a row of pixel data to the screen */
static int qls_send_row(struct qls_device *qls, int row)
{
	uint8_t tx[5];

	if (row < 0 || row >= qls->yres)
		return -EINVAL;

	tx[0] = row + 1;
	memcpy(&tx[1], qls->videomemory + row * qls->row_bytes, qls->row_bytes);

	return spi_write(qls->spi, tx, qls->row_bytes + 1);
}

/* Update entire screen from framebuffer */
static void qls_update_display(struct qls_device *qls)
{
	int y;

	for (y = 0; y < qls->yres; y++)
		qls_send_row(qls, y);
}

/* Register a single pixel in the framebuffer */
static void qls_set_pixel(struct qls_device *qls, int x, int y, int on)
{
	int addr, byte, bit;

	if (x < 0 || x >= qls->xres || y < 0 || y >= qls->yres)
		return;

	addr = y * qls->xres + x;
	byte = addr / 8;
	bit = addr % 8;

	if (on)
		qls->videomemory[byte] |= (1 << bit);
	else
		qls->videomemory[byte] &= ~(1 << bit);
}

/* Fill a rectangle */
static void qls_fill_rect(struct qls_device *qls, int x, int y,
			  int w, int h, int on)
{
	int row, col;

	for (row = y; row < y + h && row < qls->yres; row++)
		for (col = x; col < x + w && col < qls->xres; col++)
			qls_set_pixel(qls, col, row, on);
}

/* Framebuffer operations */

static int qls_fb_open(struct fb_info *info, int user)
{
	return 0;
}

static int qls_fb_release(struct fb_info *info, int user)
{
	return 0;
}

static int qls_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->xres != SCR_WIDTH || var->yres != SCR_HEIGHT ||
	    var->xres_virtual != SCR_WIDTH || var->yres_virtual != SCR_HEIGHT)
		return -EINVAL;

	if (var->bits_per_pixel != 1)
		return -EINVAL;

	var->red.offset   = 0;
	var->red.length   = 1;
	var->green.offset = 0;
	var->green.length = 1;
	var->blue.offset  = 0;
	var->blue.length  = 1;
	var->transp.offset = 0;
	var->transp.length = 0;

	var->grayscale = 1;
	var->pixclock  = 0;
	var->left_margin  = 0;
	var->right_margin = 0;
	var->upper_margin  = 0;
	var->lower_margin  = 0;
	var->hsync_len = 0;
	var->vsync_len = 0;
	var->sync  = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	return 0;
}

static int qls_fb_set_par(struct fb_info *info)
{
	struct qls_device *qls = info->par;

	qls->xres = info->var.xres;
	qls->yres = info->var.yres;
	qls->row_bytes = (qls->xres + 7) / 8;

	return 0;
}

static int qls_fb_blank(int blank, struct fb_info *info)
{
	struct qls_device *qls = info->par;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		qls_update_display(qls);
		break;
	case FB_BLANK_POWERDOWN:
		qls_fill_rect(qls, 0, 0, qls->xres, qls->yres, 0);
		qls_update_display(qls);
		break;
	default:
		break;
	}
	return 0;
}

static void qls_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct qls_device *qls = info->par;
	int on = rect->color ? 1 : 0;

	qls_fill_rect(qls, rect->dx, rect->dy, rect->width, rect->height, on);
	qls_update_display(qls);
}

static void qls_fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	/* Not implemented - update full display from shadow buffer */
	struct qls_device *qls = info->par;

	cfb_copyarea(info, area);
	qls_update_display(qls);
}

static void qls_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct qls_device *qls = info->par;

	cfb_imageblit(info, image);
	qls_update_display(qls);
}

static int qls_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	return 0;
}

static const struct fb_ops qls_fb_ops = {
	.owner        = THIS_MODULE,
	.fb_open      = qls_fb_open,
	.fb_release   = qls_fb_release,
	.fb_read      = fb_sys_read,
	.fb_write     = fb_sys_write,
	.fb_mmap      = fb_sys_mmap,
	.fb_check_var = qls_fb_check_var,
	.fb_set_par   = qls_fb_set_par,
	.fb_blank     = qls_fb_blank,
	.fb_fillrect  = qls_fb_fillrect,
	.fb_copyarea  = qls_fb_copyarea,
	.fb_imageblit = qls_fb_imageblit,
	.fb_ioctl     = qls_fb_ioctl,
};

/* Backlight operations */
static int qls_bl_update_status(struct backlight_device *bd)
{
	struct qls_device *qls = bl_get_data(bd);
	int brightness = bd->props.brightness;

	if (brightness < 0)
		brightness = 0;
	if (brightness > qls->max_brightness)
		brightness = qls->max_brightness;

	qls->brightness = brightness;

	/* Send brightness command to screen (SA_COMMAND__SET_BRIGHTNESS) */
	if (qls->spi) {
		uint8_t cmd[3] = { 0xFE, 0x01, brightness };
		spi_write(qls->spi, cmd, 3);
	}

	return 0;
}

static int qls_bl_get_brightness(struct backlight_device *bd)
{
	struct qls_device *qls = bl_get_data(bd);

	return qls->brightness;
}

static const struct backlight_ops qls_bl_ops = {
	.update_status  = qls_bl_update_status,
	.get_brightness = qls_bl_get_brightness,
};

/* SPI driver probe */
static int qls_probe(struct spi_device *spi)
{
	struct qls_device *qls;
	struct fb_info *info;
	struct backlight_device *bl;
	struct backlight_properties bl_props;
	int ret;

	qls = devm_kzalloc(&spi->dev, sizeof(*qls), GFP_KERNEL);
	if (!qls)
		return -ENOMEM;

	qls->spi = spi;
	qls->xres = SCR_WIDTH;
	qls->yres = SCR_HEIGHT;
	qls->row_bytes = (SCR_WIDTH + 7) / 8;

	spi_set_drvdata(spi, qls);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	/* Parse DT properties */
	if (of_property_read_u32(spi->dev.of_node, "max-brightness",
				 &qls->max_brightness))
		qls->max_brightness = 200;

	/* Reset GPIO (GPIO_ACTIVE_LOW: assert=pin LOW, de-assert=pin HIGH) */
	qls->reset = devm_gpiod_get(&spi->dev, "reset", GPIOD_ASIS);
	if (IS_ERR(qls->reset)) {
		dev_warn(&spi->dev, "reset GPIO not defined, skipping\n");
		qls->reset = NULL;
	} else {
		gpiod_set_value_cansleep(qls->reset, 1);
		usleep_range(10000, 15000);
		gpiod_set_value_cansleep(qls->reset, 0);
		usleep_range(100000, 120000);
	}

	/* Allocate framebuffer memory */
	qls->videomemory = devm_kzalloc(&spi->dev,
					SCR_BYTES, GFP_KERNEL);
	if (!qls->videomemory)
		return -ENOMEM;

	/* Register framebuffer */
	info = framebuffer_alloc(0, &spi->dev);
	if (!info)
		return -ENOMEM;

	qls->fb = info;
	info->par = qls;
	info->screen_base = (char __iomem *)qls->videomemory;
	info->screen_size = SCR_BYTES;
	info->fbops = &qls_fb_ops;
	info->pseudo_palette = NULL;
	info->flags = FBINFO_VIRTFB;

	info->var.xres = SCR_WIDTH;
	info->var.yres = SCR_HEIGHT;
	info->var.xres_virtual = SCR_WIDTH;
	info->var.yres_virtual = SCR_HEIGHT;
	info->var.bits_per_pixel = 1;
	info->var.grayscale = 1;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->fix.smem_start = 0;
	info->fix.smem_len = SCR_BYTES;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_MONO01;
	info->fix.line_length = qls->row_bytes;
	info->fix.accel = FB_ACCEL_NONE;
	snprintf(info->fix.id, sizeof(info->fix.id), "quasar-led");

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to register framebuffer: %d\n", ret);
		framebuffer_release(info);
		return ret;
	}
	dev_info(&spi->dev, "registered framebuffer '%s'\n", info->fix.id);

	/* Register backlight */
	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.max_brightness = qls->max_brightness;
	bl_props.brightness = qls->max_brightness;

	bl = devm_backlight_device_register(&spi->dev, "quasar_led_screen",
					    &spi->dev, qls,
					    &qls_bl_ops, &bl_props);
	if (IS_ERR(bl)) {
		ret = PTR_ERR(bl);
		dev_err(&spi->dev, "failed to register backlight: %d\n", ret);
		unregister_framebuffer(info);
		return ret;
	}
	qls->bl = bl;

	/* Show initial test pattern */
	qls_fill_rect(qls, 0, 0, 1, qls->yres, 1);
	qls_update_display(qls);
	msleep(100);

	qls_fill_rect(qls, 0, 0, qls->xres, qls->yres, 0);
	qls_update_display(qls);

	return 0;
}

static void qls_remove(struct spi_device *spi)
{
	struct qls_device *qls = spi_get_drvdata(spi);

	if (qls->fb)
		unregister_framebuffer(qls->fb);
}

static const struct of_device_id qls_of_match[] = {
	{ .compatible = "ya,led_screen" },
	{},
};
MODULE_DEVICE_TABLE(of, qls_of_match);

static const struct spi_device_id qls_spi_ids[] = {
	{ "quasar_led_screen", 0 },
	{ "ya,led_screen", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, qls_spi_ids);

static struct spi_driver qls_spi_driver = {
	.driver = {
		.name   = DRIVER_NAME,
		.of_match_table = qls_of_match,
	},
	.id_table = qls_spi_ids,
	.probe    = qls_probe,
	.remove   = qls_remove,
};

module_spi_driver(qls_spi_driver);

MODULE_DESCRIPTION("Quasar LED Screen FB driver (Gowin FPGA)");
MODULE_AUTHOR("leftymods");
MODULE_LICENSE("GPL v2");
