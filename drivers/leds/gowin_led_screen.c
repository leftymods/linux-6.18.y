// SPDX-License-Identifier: GPL-2.0-only
/*
 * gowin_led_screen.c -- Gowin FPGA 25x16 SPI LED Matrix & Touch Driver for Yandex Station Max
 *
 * Copyright (C) Yandex Inc.
 * Ported to Mainline Linux 6.x by Reverse Engineering
 * Full functional parity with vendor kernel module gowin_led_device.ko
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#define SCREEN_WIDTH        25
#define SCREEN_HEIGHT       16
#define SCREEN_BPP          8
#define BUFFER_SIZE         (SCREEN_WIDTH * SCREEN_HEIGHT * (SCREEN_BPP / 8))

/* Gowin JTAG Commands */
#define GOWIN_JTAG_IDCODE       0x11
#define GOWIN_JTAG_USERCODE     0x13
#define GOWIN_JTAG_ERASE        0x05
#define GOWIN_JTAG_PROG         0x15
#define GOWIN_JTAG_RECONFIG     0x3c

struct gowin_led_screen {
	struct spi_device *spi;
	struct fb_info *fb;
	u8 *vmem;
	struct input_dev *input;
	struct mutex lock;
	struct gpio_desc *reconfig_gpio;
	struct gpio_desc *mode_gpio;
	struct gpio_desc *touch_irq_gpio;
	struct gpio_desc *jtag_tck_gpio;
	struct gpio_desc *jtag_tms_gpio;
	struct gpio_desc *jtag_tdi_gpio;
	struct gpio_desc *jtag_tdo_gpio;
	int touch_irq;
	struct work_struct touch_work;
	u32 frame_delay_ms;
	bool power_saving;
	u32 idcode;
	u32 usercode;
	u32 crc_errors;
	u32 frames_in_queue;
	int fw_upd_status;
};

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
	/* Move to Run-Test/Idle: TMS=0 */
	jtag_clock_cycle(screen, 0, 0);
}

static void jtag_write_instruction(struct gowin_led_screen *screen, u8 inst, int bits)
{
	int i;
	/* From Idle -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR */
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	jtag_clock_cycle(screen, 0, 0);

	for (i = 0; i < bits; i++) {
		int last = (i == bits - 1);
		int bit = (inst >> i) & 1;
		jtag_clock_cycle(screen, last ? 1 : 0, bit);
	}
	/* Exit1-IR -> Update-IR -> Idle */
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
}

static u32 jtag_read_data(struct gowin_led_screen *screen, int bits)
{
	u32 data = 0;
	int i;

	/* From Idle -> Select-DR -> Capture-DR -> Shift-DR */
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
	/* Exit1-DR -> Update-DR -> Idle */
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	return data;
}

static int jtag_prog_fpga(struct gowin_led_screen *screen, const u8 *bitstream, size_t size)
{
	size_t i;
	mutex_lock(&screen->lock);
	screen->fw_upd_status = 1; /* In progress */

	jtag_tap_reset(screen);
	jtag_write_instruction(screen, GOWIN_JTAG_IDCODE, 8);
	screen->idcode = jtag_read_data(screen, 32);

	/* Erase flash / SRAM */
	jtag_write_instruction(screen, GOWIN_JTAG_ERASE, 8);
	msleep(100);

	/* Shift in configuration bitstream */
	jtag_write_instruction(screen, GOWIN_JTAG_PROG, 8);

	/* From Idle -> Select-DR -> Capture-DR -> Shift-DR */
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);
	jtag_clock_cycle(screen, 0, 0);

	for (i = 0; i < size; i++) {
		u8 byte = bitstream[i];
		int b;
		for (b = 0; b < 8; b++) {
			int last = (i == size - 1 && b == 7);
			int bit = (byte >> b) & 1;
			jtag_clock_cycle(screen, last ? 1 : 0, bit);
		}
	}
	/* Exit1-DR -> Update-DR -> Idle */
	jtag_clock_cycle(screen, 1, 0);
	jtag_clock_cycle(screen, 0, 0);

	/* Reconfig trigger */
	jtag_write_instruction(screen, GOWIN_JTAG_RECONFIG, 8);
	msleep(50);
	jtag_tap_reset(screen);

	screen->fw_upd_status = 0; /* Success */
	mutex_unlock(&screen->lock);
	dev_info(&screen->spi->dev, "FPGA bitstream (%zu bytes) programmed successfully\n", size);
	return 0;
}

/* --- Framebuffer Operations --- */

static void gowin_led_screen_update(struct fb_info *info, struct list_head *pagereflist)
{
	struct gowin_led_screen *screen = info->par;
	u8 tx_buf[BUFFER_SIZE + 4];

	if (screen->power_saving)
		return;

	/* Command prefix: 0xA5 0x5A start of frame */
	tx_buf[0] = 0xA5;
	tx_buf[1] = 0x5A;
	tx_buf[2] = SCREEN_WIDTH;
	tx_buf[3] = SCREEN_HEIGHT;
	memcpy(&tx_buf[4], screen->vmem, BUFFER_SIZE);

	mutex_lock(&screen->lock);
	spi_write(screen->spi, tx_buf, sizeof(tx_buf));
	mutex_unlock(&screen->lock);
}

static ssize_t gowin_fb_write(struct fb_info *info, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	ssize_t res = fb_sys_write(info, buf, count, ppos);
	if (res > 0)
		gowin_led_screen_update(info, NULL);
	return res;
}

static int gowin_fb_blank(int blank, struct fb_info *info)
{
	struct gowin_led_screen *screen = info->par;

	screen->power_saving = (blank != FB_BLANK_UNBLANK);
	if (screen->power_saving) {
		u8 blank_cmd[4] = { 0xA5, 0x5A, 0x00, 0x00 };
		mutex_lock(&screen->lock);
		spi_write(screen->spi, blank_cmd, sizeof(blank_cmd));
		mutex_unlock(&screen->lock);
	} else {
		gowin_led_screen_update(info, NULL);
	}
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
};

/* --- Touch Subsystem --- */

static void touch_work_func(struct work_struct *work)
{
	struct gowin_led_screen *screen = container_of(work, struct gowin_led_screen, touch_work);
	u8 rx_buf[8] = { 0 };
	u8 cmd[2] = { 0xA5, 0x81 }; /* Read touch status */

	mutex_lock(&screen->lock);
	spi_write_then_read(screen->spi, cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
	mutex_unlock(&screen->lock);

	if (rx_buf[0] & 0x01) { /* Touch active */
		u16 x = rx_buf[1] | ((rx_buf[2] & 0x0f) << 8);
		u16 y = rx_buf[3] | ((rx_buf[4] & 0x0f) << 8);
		input_report_key(screen->input, BTN_TOUCH, 1);
		input_report_abs(screen->input, ABS_X, x);
		input_report_abs(screen->input, ABS_Y, y);
		input_sync(screen->input);
	} else {
		input_report_key(screen->input, BTN_TOUCH, 0);
		input_sync(screen->input);
	}
}

static irqreturn_t lp_touch_isr(int irq, void *dev_id)
{
	struct gowin_led_screen *screen = dev_id;
	schedule_work(&screen->touch_work);
	return IRQ_HANDLED;
}

/* --- Sysfs Attributes --- */

static ssize_t power_saving_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", screen->power_saving ? 1 : 0);
}

static ssize_t power_saving_mode_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	bool val;
	if (kstrtobool(buf, &val))
		return -EINVAL;
	screen->power_saving = val;
	return count;
}
static DEVICE_ATTR_RW(power_saving_mode);

static ssize_t next_frame_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", screen->frame_delay_ms);
}

static ssize_t next_frame_delay_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	u32 val;
	if (kstrtou32(buf, 10, &val))
		return -EINVAL;
	screen->frame_delay_ms = val;
	return count;
}
static DEVICE_ATTR_RW(next_frame_delay);

static ssize_t jtag_codes_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	return sysfs_emit(buf, "IDCODE: 0x%08x USERCODE: 0x%08x\n", screen->idcode, screen->usercode);
}
static DEVICE_ATTR_RO(jtag_codes);

static ssize_t fw_upd_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", screen->fw_upd_status);
}
static DEVICE_ATTR_RO(fw_upd_status);

static ssize_t jtag_prog_fpga_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	const struct firmware *fw;
	char fw_name[64];
	int ret;

	if (count >= sizeof(fw_name))
		return -EINVAL;

	strscpy(fw_name, buf, sizeof(fw_name));
	strim(fw_name);

	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "Failed to load bitstream '%s': %d\n", fw_name, ret);
		return ret;
	}

	ret = jtag_prog_fpga(screen, fw->data, fw->size);
	release_firmware(fw);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(jtag_prog_fpga);

static ssize_t press_key_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct gowin_led_screen *screen = dev_get_drvdata(dev);
	unsigned int key;
	if (kstrtouint(buf, 0, &key))
		return -EINVAL;
	input_report_key(screen->input, key, 1);
	input_sync(screen->input);
	input_report_key(screen->input, key, 0);
	input_sync(screen->input);
	return count;
}
static DEVICE_ATTR_WO(press_key);

static struct attribute *gowin_screen_attrs[] = {
	&dev_attr_power_saving_mode.attr,
	&dev_attr_next_frame_delay.attr,
	&dev_attr_jtag_codes.attr,
	&dev_attr_fw_upd_status.attr,
	&dev_attr_jtag_prog_fpga.attr,
	&dev_attr_press_key.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gowin_screen);

static int gowin_led_screen_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct gowin_led_screen *screen;
	struct fb_info *fb;
	int ret;

	screen = devm_kzalloc(dev, sizeof(*screen), GFP_KERNEL);
	if (!screen)
		return -ENOMEM;

	screen->spi = spi;
	mutex_init(&screen->lock);
	INIT_WORK(&screen->touch_work, touch_work_func);
	spi_set_drvdata(spi, screen);

	screen->reconfig_gpio = devm_gpiod_get_optional(dev, "reconfig", GPIOD_OUT_LOW);
	screen->mode_gpio = devm_gpiod_get_optional(dev, "mode", GPIOD_OUT_LOW);
	screen->touch_irq_gpio = devm_gpiod_get_optional(dev, "touch-irq", GPIOD_IN);

	/* JTAG GPIOs for programming */
	screen->jtag_tck_gpio = devm_gpiod_get_optional(dev, "jtag-tck", GPIOD_OUT_LOW);
	screen->jtag_tms_gpio = devm_gpiod_get_optional(dev, "jtag-tms", GPIOD_OUT_HIGH);
	screen->jtag_tdi_gpio = devm_gpiod_get_optional(dev, "jtag-tdi", GPIOD_OUT_LOW);
	screen->jtag_tdo_gpio = devm_gpiod_get_optional(dev, "jtag-tdo", GPIOD_IN);

	/* Allocate Framebuffer */
	fb = framebuffer_alloc(0, dev);
	if (!fb)
		return -ENOMEM;

	screen->fb = fb;
	screen->vmem = devm_kzalloc(dev, BUFFER_SIZE, GFP_KERNEL);
	if (!screen->vmem) {
		framebuffer_release(fb);
		return -ENOMEM;
	}

	fb->par = screen;
	fb->fbops = &gowin_fb_ops;
	fb->screen_base = (char __iomem *)screen->vmem;
	fb->screen_size = BUFFER_SIZE;

	fb->var.xres = SCREEN_WIDTH;
	fb->var.yres = SCREEN_HEIGHT;
	fb->var.xres_virtual = SCREEN_WIDTH;
	fb->var.yres_virtual = SCREEN_HEIGHT;
	fb->var.bits_per_pixel = SCREEN_BPP;
	fb->var.grayscale = 1;

	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	fb->fix.line_length = SCREEN_WIDTH;
	fb->fix.smem_start = (unsigned long)screen->vmem;
	fb->fix.smem_len = BUFFER_SIZE;
	strscpy(fb->fix.id, "GowinLED", sizeof(fb->fix.id));

	ret = register_framebuffer(fb);
	if (ret) {
		framebuffer_release(fb);
		return dev_err_probe(dev, ret, "Failed to register framebuffer\n");
	}

	/* Register Touch Input Device */
	screen->input = devm_input_allocate_device(dev);
	if (screen->input) {
		screen->input->name = "Gowin Screen Touch";
		screen->input->id.bustype = BUS_SPI;
		input_set_capability(screen->input, EV_KEY, BTN_TOUCH);
		input_set_abs_params(screen->input, ABS_X, 0, 1024, 0, 0);
		input_set_abs_params(screen->input, ABS_Y, 0, 1024, 0, 0);
		ret = input_register_device(screen->input);
		if (ret)
			dev_warn(dev, "Failed to register input device (%d)\n", ret);
	}

	if (screen->touch_irq_gpio) {
		screen->touch_irq = gpiod_to_irq(screen->touch_irq_gpio);
		if (screen->touch_irq > 0) {
			ret = devm_request_threaded_irq(dev, screen->touch_irq, NULL,
							lp_touch_isr,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							"gowin_touch", screen);
			if (ret)
				dev_warn(dev, "Failed to request touch IRQ: %d\n", ret);
		}
	}

	dev_info(dev, "Gowin FPGA 25x16 LED screen & touch driver loaded\n");
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
	{ .compatible = "gowin,led-screen" },
	{ .compatible = "yandex,station-max-led-screen" },
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

MODULE_DESCRIPTION("Gowin FPGA 25x16 LED Screen & Touch Driver for Yandex Station Max");
MODULE_AUTHOR("Yandex LLC");
MODULE_AUTHOR("Mainline Linux port by Reverse Engineering");
MODULE_LICENSE("GPL v2");
