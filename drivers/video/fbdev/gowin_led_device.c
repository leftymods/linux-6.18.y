// SPDX-License-Identifier: GPL-2.0
/*
 * gowin_led_device.c - Quasar LED panel/screen SPI driver
 *
 * Copyright (C) 2025 AtriOS Team
 *
 * SPI-driven LED panel/screen with framebuffer, backlight,
 * virtual GPIO, virtual UART, touch input, JTAG FPGA programming,
 * and firmware update support.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/pinctrl/consumer.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include "yandex_fpga_bitstream.h"

#define GOWIN_DRV_NAME		"atri_led_panel"
#define GOWIN_FB_NAME		"atri_led_panel_"

#define GOWIN_GPIO_NGPIO	2
#define GOWIN_TTY_MINOR_COUNT	1
#define GOWIN_TTY_BUF_SIZE	64
#define GOWIN_FB_WIDTH_MAX	320
#define GOWIN_FB_HEIGHT_MAX	320

/* SPI command IDs */
#define CMD_NOP			0x00
#define CMD_GET_STATUS		0x01
#define CMD_SHOW_PIC		0x03
#define CMD_SET_BRIGHTNESS	0x05
#define CMD_SET_GPIO		0x06
#define CMD_SET_ANIMATION	0x07
#define CMD_UART_SEND_DATA	0x09
#define CMD_UART_RECV_DATA	0x0a
#define CMD_UART_FLUSH_DATA	0x0b
#define CMD_SET_POWER_SAVING	0x0c
#define CMD_WRITE		0x10
#define CMD_GET_STATUS_LEN	32

/* Status bits */
#define STATUS_TOUCH_MASK(n)	BIT(n)
#define STATUS_CRC_ERROR_S	BIT(2)
#define STATUS_CRC_ERROR_M	BIT(3)

/* Gowin JTAG instruction opcodes (TN653) */
#define GWJ_NOOP		0x02
#define GWJ_ERASE_SRAM		0x05
#define GWJ_XFER_DONE		0x09
#define GWJ_READ_IDCODE		0x11
#define GWJ_READ_USERCODE	0x13
#define GWJ_CONFIG_ENABLE	0x15
#define GWJ_CONFIG_DISABLE	0x3a
#define GWJ_RELOAD		0x3c
#define GWJ_READ_STATUS		0x41
#define GWJ_EF_PROGRAM		0x71
#define GWJ_EFLASH_ERASE	0x75

/*
 * Expected FPGA: Gowin GW1N-4B. The fs bitstream header stores the
 * idcode as 0x1100381b; the value shifted out of the TAP masked with
 * 0x0fffffff is 0x0100381b.
 */
#define GOWIN_IDCODE		0x0100381b

/* Gowin STATUS register bits */
#define GWS_CRC_ERROR		BIT(0)
#define GWS_BAD_COMMAND		BIT(1)
#define GWS_ID_VERIFY_FAILED	BIT(2)
#define GWS_TIMEOUT		BIT(3)
#define GWS_MEMORY_ERASE	BIT(5)
#define GWS_PREAMBLE		BIT(6)
#define GWS_SYSTEM_EDIT_MODE	BIT(7)
#define GWS_GOWIN_VLD		BIT(12)
#define GWS_DONE_FINAL		BIT(13)
#define GWS_SECURITY_FINAL	BIT(14)
#define GWS_READY		BIT(15)
#define GWS_POR			BIT(16)
#define GWS_FLASH_LOCK		BIT(17)

#define GWS_ERROR_MASK		(GWS_CRC_ERROR | GWS_BAD_COMMAND | \
				 GWS_ID_VERIFY_FAILED | GWS_TIMEOUT)

struct gowin_resolution {
	int width;
	int height;
};

static const struct gowin_resolution rev2res_panel = { .width = 25, .height = 16 };
static const struct gowin_resolution rev2res_screen = { .width = 28, .height = 16 };

struct gowin_touch_regs {
	u32 dbg_val[5];
	u32 metric[5];
	u32 touch_state;
	u32 touch_mask;
};

struct gowin_priv {
	struct spi_device *spi;
	struct device *dev;
	struct fb_info *fbi;
	struct backlight_device *bl;
	struct gpio_chip gpio_chip;
	struct tty_driver *tty_drv;
	struct tty_port tty_port;
	struct input_dev *touch_dev;
	struct workqueue_struct *touch_wq;
	struct work_struct touch_work;
	struct work_struct uart_rx_work;
	struct timer_list uart_timer;

	/* GPIO pins */
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *jtag_tck;
	struct gpio_desc *jtag_tdo;
	struct gpio_desc *jtag_tdi;
	struct gpio_desc *jtag_tms;
	struct gpio_desc *jtag_sel;
	struct gpio_desc *jtag_reconfig;
	void __iomem *jtag_tck_reg;
	u32 jtag_tck_bit;
	bool jtag_tms_via_cs;

	/* JTAG debug counters */
	unsigned int erase_attempts;
	unsigned int prog_attempts;

	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct pinctrl_state *pinctrl_jtag;

	/* Mode */
	bool jtag_mode;

	/* Resolution */
	int width;
	int height;

	/* GPIO virtual chip cached byte */
	u8 gpio_cached;

	/* Framebuffer */
	void *fb_mem;
	int fb_open_count;
	u32 pseudo_palette[16];

	/* Backlight */
	unsigned int brightness;

	/* Frame queue state */
	unsigned int next_frame_delay;
	unsigned int frames_in_queue;
	unsigned int frame_queue_overflows;

	/* CRC errors */
	unsigned int crc_errors_master;
	unsigned int crc_errors_slave;

	/* Power saving */
	bool power_saving;

	/* Touch state */
	u8 touch_irq_number;

	/* Firmware status */
	char fw_upd_status[40];

	/* Lock */
	struct mutex lock;

	/* TTY xmit buffer */
	unsigned char xmit_buf[GOWIN_TTY_BUF_SIZE];
	int xmit_count;
};

/* ------------------------------------------------------------------ */
/* App protocol -- SPI command layer                                   */
/* ------------------------------------------------------------------ */

static int lp_write(struct gowin_priv *priv, const u8 *data, int len)
{
	struct spi_transfer t = {
		.tx_buf = data,
		.len = len,
	};
	struct spi_message m;
	int ret;

	spi_message_init_with_transfers(&m, &t, 1);
	mutex_lock(&priv->lock);
	if (priv->jtag_mode) {
		/* JTAG owns the pins right now; SPI must stay quiet */
		mutex_unlock(&priv->lock);
		return -EBUSY;
	}
	ret = spi_sync(priv->spi, &m);
	mutex_unlock(&priv->lock);
	if (ret)
		dev_err(priv->dev, "spi_write(%d) failed: %d\n", len, ret);
	return ret;
}

static int get_lp_status(struct gowin_priv *priv, u8 *status_buf)
{
	u8 cmd = CMD_GET_STATUS;
	struct spi_transfer tx = {
		.tx_buf = &cmd,
		.len = 1,
	};
	struct spi_transfer rx = {
		.rx_buf = status_buf,
		.len = CMD_GET_STATUS_LEN,
	};
	struct spi_message m;
	int ret;

	spi_message_init_with_transfers(&m, &tx, 1);
	spi_message_add_tail(&rx, &m);
	mutex_lock(&priv->lock);
	if (priv->jtag_mode) {
		mutex_unlock(&priv->lock);
		return -EBUSY;
	}
	ret = spi_sync(priv->spi, &m);
	mutex_unlock(&priv->lock);
	if (ret)
		dev_err(priv->dev, "get_lp_status failed: %d\n", ret);
	return ret;
}

static int show_pic_app_cmd(struct gowin_priv *priv)
{
	u8 buf[3] = { CMD_SHOW_PIC, 0x00, 0x00 };
	int ret = lp_write(priv, buf, 3);
	if (ret)
		dev_err(priv->dev, "show_pic failed: %d\n", ret);
	return ret;
}

static int set_brightness_app_cmd(struct gowin_priv *priv, unsigned int val)
{
	u8 buf[3] = { CMD_SET_BRIGHTNESS, val & 0xff, (val >> 8) & 0xff };
	int ret = lp_write(priv, buf, 3);
	if (ret)
		dev_err(priv->dev, "set_brightness(%u) failed: %d\n", val, ret);
	return ret;
}

static int set_gpio_app_cmd(struct gowin_priv *priv)
{
	u8 buf[3] = { CMD_SET_GPIO, priv->gpio_cached, 0x00 };
	return lp_write(priv, buf, 3);
}

static int set_animation_app_cmd(struct gowin_priv *priv, int on)
{
	u8 buf[3] = { CMD_SET_ANIMATION, !!on, 0x00 };
	return lp_write(priv, buf, 3);
}

static int set_power_saving_mode_cmd(struct gowin_priv *priv, int on)
{
	u8 buf[3] = { CMD_SET_POWER_SAVING, !!on, 0x00 };
	return lp_write(priv, buf, 3);
}

static int uart_send_data_app_cmd(struct gowin_priv *priv,
				  const u8 *data, int len)
{
	u8 buf[4] = { CMD_UART_SEND_DATA, len & 0xff, (len >> 8) & 0xff, 0 };
	int ret;

	ret = lp_write(priv, buf, 4);
	if (ret)
		return ret;

	return lp_write(priv, data, len);
}

static int uart_receive_data(struct gowin_priv *priv,
			     u8 *rx_data, int *rx_len)
{
	u8 cmd[3] = { CMD_UART_RECV_DATA, 0, 0 };
	u8 header[4];
	struct spi_transfer t[3];
	struct spi_message m;
	int ret;

	memset(t, 0, sizeof(t));
	t[0].tx_buf = cmd;
	t[0].len = 3;
	t[1].delay.value = 100;
	t[1].delay.unit = SPI_DELAY_UNIT_USECS;
	t[1].len = 0;
	t[2].rx_buf = header;
	t[2].len = 4;
	spi_message_init_with_transfers(&m, t, 3);

	mutex_lock(&priv->lock);
	if (priv->jtag_mode) {
		mutex_unlock(&priv->lock);
		return 0;
	}
	ret = spi_sync(priv->spi, &m);
	if (ret) {
		mutex_unlock(&priv->lock);
		return -EIO;
	}

	*rx_len = header[0] | (header[1] << 8);
	if (*rx_len > GOWIN_TTY_BUF_SIZE)
		*rx_len = GOWIN_TTY_BUF_SIZE;
	if (*rx_len == 0) {
		mutex_unlock(&priv->lock);
		return 0;
	}

	{
		struct spi_transfer r = {
			.rx_buf = rx_data,
			.len = *rx_len,
		};
		spi_message_init_with_transfers(&m, &r, 1);
		ret = spi_sync(priv->spi, &m);
		mutex_unlock(&priv->lock);
		if (ret)
			dev_dbg(priv->dev, "uart_rx len=%d failed: %d\n", *rx_len, ret);
		else
			dev_dbg(priv->dev, "uart_rx len=%d\n", *rx_len);
		return ret;
	}
}

static int uart_flush_data_app_cmd(struct gowin_priv *priv)
{
	u8 buf[3] = { CMD_UART_FLUSH_DATA, 0, 0 };
	return lp_write(priv, buf, 3);
}

/* ------------------------------------------------------------------ */
/* GPIO chip                                                           */
/* ------------------------------------------------------------------ */

static int lp_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static int lp_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return -EINVAL;
}

static int lp_gpio_get_value(struct gpio_chip *chip, unsigned int offset)
{
	struct gowin_priv *priv = gpiochip_get_data(chip);
	return (priv->gpio_cached >> offset) & 1;
}

static int lp_gpio_set_value(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct gowin_priv *priv = gpiochip_get_data(chip);

	if (value)
		priv->gpio_cached |= BIT(offset);
	else
		priv->gpio_cached &= ~BIT(offset);

	set_gpio_app_cmd(priv);
	return 0;
}

static int lp_gpio_direction_output(struct gpio_chip *chip,
				    unsigned int offset, int value)
{
	return lp_gpio_set_value(chip, offset, value);
}

static int gpio_init(struct gowin_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->gpio_chip.label = "led_panel";
	priv->gpio_chip.base = -1;
	priv->gpio_chip.ngpio = GOWIN_GPIO_NGPIO;
	priv->gpio_chip.get_direction = lp_gpio_get_direction;
	priv->gpio_chip.direction_input = lp_gpio_direction_input;
	priv->gpio_chip.direction_output = lp_gpio_direction_output;
	priv->gpio_chip.get = lp_gpio_get_value;
	priv->gpio_chip.set = lp_gpio_set_value;
	priv->gpio_chip.parent = dev;
	priv->gpio_chip.fwnode = dev_fwnode(dev);
	priv->gpio_chip.of_gpio_n_cells = 2;
	priv->gpio_chip.can_sleep = true;

	ret = gpiochip_add_data(&priv->gpio_chip, priv);
	if (ret)
		dev_err(dev, "gpiochip_add_data failed: %d\n", ret);
	else
		dev_dbg(dev, "gpiochip added (%d GPIOs)\n", GOWIN_GPIO_NGPIO);

	return ret;
}

static void gpio_exit(struct gowin_priv *priv)
{
	gpiochip_remove(&priv->gpio_chip);
}

/* ------------------------------------------------------------------ */
/* TTY / UART                                                         */
/* ------------------------------------------------------------------ */

static int lp_uart_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void lp_uart_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(tty->port, tty, filp);
}

static unsigned int lp_uart_write_room(struct tty_struct *tty)
{
	return GOWIN_TTY_BUF_SIZE;
}

static ssize_t lp_uart_write(struct tty_struct *tty, const u8 *buf,
			     size_t count)
{
	struct gowin_priv *priv = container_of(tty->port, struct gowin_priv,
					       tty_port);
	int ret;

	if (count > GOWIN_TTY_BUF_SIZE)
		count = GOWIN_TTY_BUF_SIZE;

	ret = uart_send_data_app_cmd(priv, buf, count);
	if (ret) {
		dev_dbg(priv->dev, "uart write failed: %d\n", ret);
		return ret;
	}

	return count;
}

static void lp_uart_rx_work(struct work_struct *work)
{
	struct gowin_priv *priv = container_of(work, struct gowin_priv, uart_rx_work);
	u8 rx_buf[GOWIN_TTY_BUF_SIZE];
	int rx_len;
	int ret;

	ret = uart_receive_data(priv, rx_buf, &rx_len);
	if (ret || rx_len <= 0)
		return;

	tty_insert_flip_string_fixed_flag(&priv->tty_port, rx_buf,
					  TTY_NORMAL, rx_len);
	tty_flip_buffer_push(&priv->tty_port);
}

static void lp_uart_poll(struct timer_list *t)
{
	struct gowin_priv *priv = container_of(t, struct gowin_priv,
					       uart_timer);

	if (priv->tty_port.tty)
		schedule_work(&priv->uart_rx_work);

	mod_timer(&priv->uart_timer, jiffies + HZ / 10);
}

static const struct tty_operations lp_uart_ops = {
	.open = lp_uart_open,
	.close = lp_uart_close,
	.write = lp_uart_write,
	.write_room = lp_uart_write_room,
};

static int uart_init(struct gowin_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->tty_drv = tty_alloc_driver(GOWIN_TTY_MINOR_COUNT,
					 TTY_DRIVER_REAL_RAW |
					 TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(priv->tty_drv))
		return PTR_ERR(priv->tty_drv);

	priv->tty_drv->driver_name = GOWIN_DRV_NAME;
	priv->tty_drv->name = "ttyLP";
	priv->tty_drv->major = 0;
	priv->tty_drv->minor_start = 0;
	priv->tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	priv->tty_drv->subtype = SERIAL_TYPE_NORMAL;
	priv->tty_drv->init_termios = tty_std_termios;
	priv->tty_drv->init_termios.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	tty_set_operations(priv->tty_drv, &lp_uart_ops);

	INIT_WORK(&priv->uart_rx_work, lp_uart_rx_work);

	tty_port_init(&priv->tty_port);
	tty_port_link_device(&priv->tty_port, priv->tty_drv, 0);

	ret = tty_register_driver(priv->tty_drv);
	if (ret) {
		dev_err(dev, "tty_register_driver failed: %d\n", ret);
		tty_driver_kref_put(priv->tty_drv);
		tty_port_destroy(&priv->tty_port);
		return ret;
	}

	timer_setup(&priv->uart_timer, lp_uart_poll, 0);
	mod_timer(&priv->uart_timer, jiffies + HZ / 10);

	dev_info(dev, "uart ttyLP0 initialized\n");
	return 0;
}

static void uart_exit(struct gowin_priv *priv)
{
	timer_delete_sync(&priv->uart_timer);
	flush_work(&priv->uart_rx_work);
	tty_unregister_driver(priv->tty_drv);
	tty_port_destroy(&priv->tty_port);
	tty_driver_kref_put(priv->tty_drv);
	priv->tty_drv = NULL;
}

/* ------------------------------------------------------------------ */
/* Touch                                                              */
/* ------------------------------------------------------------------ */

static void touch_work(struct work_struct *work)
{
	struct gowin_priv *priv = container_of(work, struct gowin_priv, touch_work);
	struct gowin_touch_regs regs;
	u8 status[CMD_GET_STATUS_LEN];
	int ret;

	ret = get_lp_status(priv, status);
	if (ret)
		return;

	regs.touch_state = status[1];
	regs.touch_mask = status[2];
	regs.dbg_val[0] = status[3] | (status[4] << 8);
	regs.dbg_val[2] = status[5] | (status[6] << 8);
	regs.dbg_val[4] = status[7] | (status[8] << 8);
	regs.metric[0] = status[9];
	regs.metric[2] = status[10];
	regs.metric[4] = status[11];

	if (regs.touch_mask && priv->touch_dev) {
		dev_dbg(priv->dev, "touch event mask=0x%x state=0x%x\n",
			regs.touch_mask, regs.touch_state);
		input_event(priv->touch_dev, EV_KEY, KEY_BACK, 1);
		input_event(priv->touch_dev, EV_SYN, SYN_REPORT, 0);
		input_event(priv->touch_dev, EV_KEY, KEY_BACK, 0);
		input_event(priv->touch_dev, EV_SYN, SYN_REPORT, 0);
	}
}

static irqreturn_t lp_touch_isr(int irq, void *data)
{
	struct gowin_priv *priv = data;

	queue_work(priv->touch_wq, &priv->touch_work);
	return IRQ_HANDLED;
}

static int init_touch(struct gowin_priv *priv)
{
	int ret;

	priv->touch_dev = input_allocate_device();
	if (!priv->touch_dev)
		return -ENOMEM;

	priv->touch_dev->name = "midi_touch";
	priv->touch_dev->phys = "led_panel/input0";
	priv->touch_dev->id.bustype = BUS_SPI;
	input_set_capability(priv->touch_dev, EV_KEY, KEY_BACK);

	priv->touch_wq = alloc_workqueue("touch_wq", WQ_UNBOUND, 1);
	if (!priv->touch_wq) {
		input_free_device(priv->touch_dev);
		priv->touch_dev = NULL;
		return -ENOMEM;
	}

	INIT_WORK(&priv->touch_work, touch_work);

	ret = input_register_device(priv->touch_dev);
	if (ret) {
		dev_err(priv->dev, "touch: input_register_device failed: %d\n", ret);
		destroy_workqueue(priv->touch_wq);
		priv->touch_wq = NULL;
		input_free_device(priv->touch_dev);
		priv->touch_dev = NULL;
		return ret;
	}

	if (priv->irq_gpio) {
		int irq = gpiod_to_irq(priv->irq_gpio);
		if (irq > 0) {
			ret = request_threaded_irq(irq, NULL, lp_touch_isr,
						   IRQF_TRIGGER_FALLING |
						   IRQF_ONESHOT,
						   GOWIN_DRV_NAME, priv);
			if (ret) {
				dev_err(priv->dev, "touch: irq request failed: %d\n", ret);
				destroy_workqueue(priv->touch_wq);
				priv->touch_wq = NULL;
				input_unregister_device(priv->touch_dev);
				priv->touch_dev = NULL;
				return ret;
			}
			priv->touch_irq_number = irq;
		}
	}

	dev_dbg(priv->dev, "touch initialized (irq=%d)\n", priv->touch_irq_number);
	return 0;
}

static void deinit_touch(struct gowin_priv *priv)
{
	if (priv->touch_irq_number) {
		dev_dbg(priv->dev, "touch: freeing irq %d\n", priv->touch_irq_number);
		free_irq(priv->touch_irq_number, priv);
		priv->touch_irq_number = 0;
	}
	if (priv->touch_wq) {
		flush_work(&priv->touch_work);
		destroy_workqueue(priv->touch_wq);
		priv->touch_wq = NULL;
	}
	if (priv->touch_dev) {
		input_unregister_device(priv->touch_dev);
		priv->touch_dev = NULL;
	}
	dev_dbg(priv->dev, "touch deinitialized\n");
}

static void reset_touch_state(struct gowin_priv *priv)
{
	priv->fb_open_count = 0;
}

/* ------------------------------------------------------------------ */
/* Framebuffer                                                        */
/* ------------------------------------------------------------------ */

static int lp_fb_open(struct fb_info *fbi, int user)
{
	struct gowin_priv *priv = (struct gowin_priv *)fbi->par;

	priv->fb_open_count++;
	dev_dbg(priv->dev, "fb_open count=%d\n", priv->fb_open_count);
	return 0;
}

static int lp_fb_release(struct fb_info *fbi, int user)
{
	struct gowin_priv *priv = (struct gowin_priv *)fbi->par;

	if (priv->fb_open_count > 0)
		priv->fb_open_count--;
	dev_dbg(priv->dev, "fb_release count=%d\n", priv->fb_open_count);
	return 0;
}

static int lp_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->bits_per_pixel = 8;
	var->grayscale = 0;
	var->red.offset = 0;
	var->red.length = 8;
	var->green.offset = 0;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
	var->transp.offset = 0;
	var->transp.length = 0;
	var->nonstd = 0;
	var->activate = FB_ACTIVATE_NOW;
	var->height = -1;
	var->width = -1;
	var->pixclock = 0;
	var->left_margin = 0;
	var->right_margin = 0;
	var->upper_margin = 0;
	var->lower_margin = 0;
	var->hsync_len = 0;
	var->vsync_len = 0;
	return 0;
}

static int lp_fb_set_par(struct fb_info *fbi)
{
	return 0;
}

static int lp_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	return 0;
}

static int lp_fb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	struct gowin_priv *priv = (struct gowin_priv *)fbi->par;
	int ret;

	ret = remap_vmalloc_range(vma, fbi->screen_buffer, vma->vm_pgoff);
	if (ret)
		dev_err(priv->dev, "fb mmap failed: %d\n", ret);
	return ret;
}

static int lp_fb_sync(struct fb_info *fbi)
{
	struct gowin_priv *priv = (struct gowin_priv *)fbi->par;
	u8 *buf;
	int len;
	int ret;

	len = priv->width * priv->height;
	if (len > GOWIN_FB_WIDTH_MAX * GOWIN_FB_HEIGHT_MAX)
		return -EINVAL;

	buf = kmalloc(3 + len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = CMD_WRITE;
	buf[1] = len & 0xff;
	buf[2] = (len >> 8) & 0xff;
	memcpy(buf + 3, fbi->screen_buffer, len);

	ret = lp_write(priv, buf, 3 + len);
	kfree(buf);

	if (ret == 0)
		show_pic_app_cmd(priv);

	dev_dbg(priv->dev, "fb_sync len=%d ret=%d\n", len, ret);
	return ret;
}

static const struct fb_ops fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = lp_fb_open,
	.fb_release = lp_fb_release,
	.fb_check_var = lp_fb_check_var,
	.fb_set_par = lp_fb_set_par,
	.fb_pan_display = lp_fb_pan_display,
	.fb_mmap = lp_fb_mmap,
	.fb_sync = lp_fb_sync,
	.fb_read = fb_sys_read,
	.fb_write = fb_sys_write,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
};

static int fb_init(struct gowin_priv *priv)
{
	struct device *dev = priv->dev;
	struct fb_info *fbi;
	int fb_len;
	int ret;

	fbi = framebuffer_alloc(0, dev);
	if (!fbi)
		return -ENOMEM;

	priv->fbi = fbi;
	fbi->par = priv;

	fb_len = priv->width * priv->height;
	priv->fb_mem = vzalloc(fb_len);
	if (!priv->fb_mem) {
		ret = -ENOMEM;
		goto err_fb;
	}

	fbi->screen_buffer = priv->fb_mem;
	fbi->screen_size = fb_len;
	fbi->fix.smem_len = fb_len;
	fbi->fix.smem_start = 0;
	fbi->fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	fbi->fix.line_length = priv->width;
	fbi->fix.accel = FB_ACCEL_NONE;
	snprintf(fbi->fix.id, sizeof(fbi->fix.id), GOWIN_FB_NAME);

	fbi->var.xres = priv->width;
	fbi->var.yres = priv->height;
	fbi->var.xres_virtual = priv->width;
	fbi->var.yres_virtual = priv->height;
	fbi->var.bits_per_pixel = 8;
	fbi->var.activate = FB_ACTIVATE_NOW;
	fbi->var.red.offset = 0;
	fbi->var.red.length = 8;

	fbi->pseudo_palette = priv->pseudo_palette;
	fbi->flags = 0;
	fbi->fbops = &fb_ops;

	ret = register_framebuffer(fbi);
	if (ret) {
		dev_err(dev, "register_framebuffer failed: %d\n", ret);
		goto err_mem;
	}

	dev_info(dev, "fb registered: %dx%d\n", priv->width, priv->height);
	return 0;

err_mem:
	vfree(priv->fb_mem);
	priv->fb_mem = NULL;
err_fb:
	framebuffer_release(fbi);
	priv->fbi = NULL;
	return ret;
}

static void fb_exit(struct gowin_priv *priv)
{
	if (priv->fbi) {
		unregister_framebuffer(priv->fbi);
		vfree(priv->fb_mem);
		priv->fb_mem = NULL;
		framebuffer_release(priv->fbi);
		priv->fbi = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* Backlight                                                          */
/* ------------------------------------------------------------------ */

static int lp_bl_get_brightness(struct backlight_device *bl)
{
	struct gowin_priv *priv = bl_get_data(bl);
	return priv->brightness;
}

static int lp_bl_update_status(struct backlight_device *bl)
{
	struct gowin_priv *priv = bl_get_data(bl);

	priv->brightness = bl->props.brightness;
	dev_dbg(priv->dev, "backlight brightness=%u\n", priv->brightness);
	set_brightness_app_cmd(priv, priv->brightness);

	return 0;
}

static const struct backlight_ops lp_bl_ops = {
	.get_brightness = lp_bl_get_brightness,
	.update_status = lp_bl_update_status,
};

static int backlight_init(struct gowin_priv *priv)
{
	struct device *dev = priv->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = 255,
		.brightness = 128,
	};

	priv->bl = backlight_device_register(GOWIN_DRV_NAME, dev, priv,
					     &lp_bl_ops, &props);
	if (IS_ERR(priv->bl))
		return PTR_ERR(priv->bl);

	return 0;
}

static void backlight_exit(struct gowin_priv *priv)
{
	if (priv->bl)
		backlight_device_unregister(priv->bl);
	priv->bl = NULL;
}

/* ------------------------------------------------------------------ */
/* JTAG -- Gowin FPGA programming                                      */
/* ------------------------------------------------------------------ */

static void jtag_tck_lo(struct gowin_priv *priv)
{
	if (priv->jtag_tck_reg)
		writel_relaxed(readl_relaxed(priv->jtag_tck_reg) & ~BIT(priv->jtag_tck_bit),
			       priv->jtag_tck_reg);
	else if (priv->jtag_tck)
		gpiod_set_value(priv->jtag_tck, 0);
}

static void jtag_tck_hi(struct gowin_priv *priv)
{
	if (priv->jtag_tck_reg)
		writel_relaxed(readl_relaxed(priv->jtag_tck_reg) | BIT(priv->jtag_tck_bit),
			       priv->jtag_tck_reg);
	else if (priv->jtag_tck)
		gpiod_set_value(priv->jtag_tck, 1);
}

static void jtag_delay(void)
{
	int i;

	for (i = 0; i < 100; i++)
		cpu_relax();
}

static void jtag_tck_pulse(struct gowin_priv *priv)
{
	jtag_tck_lo(priv);
	jtag_delay();
	jtag_tck_hi(priv);
	jtag_delay();
}

static void jtag_set_tms(struct gowin_priv *priv, int val)
{
	if (!priv->jtag_tms)
		return;
	/*
	 * When TMS rides on the SPI CS descriptor the line is flagged
	 * active-low (cs-gpios); JTAG needs physical levels, hence raw.
	 */
	if (priv->jtag_tms_via_cs)
		gpiod_set_raw_value(priv->jtag_tms, val);
	else
		gpiod_set_value(priv->jtag_tms, val);
}

static void jtag_clock(struct gowin_priv *priv, int tms, int tdi)
{
	if (priv->jtag_tdi)
		gpiod_set_value(priv->jtag_tdi, tdi);
	jtag_set_tms(priv, tms);
	jtag_tck_pulse(priv);
}

/* Idle clocks in Run-Test/Idle */
static void jtag_clocks(struct gowin_priv *priv, unsigned int n)
{
	while (n--)
		jtag_clock(priv, 0, 0);
}

/* Any TAP state -> Test-Logic-Reset -> Run-Test/Idle */
static void jtag_to_idle(struct gowin_priv *priv)
{
	int i;

	for (i = 0; i < 6; i++)
		jtag_clock(priv, 1, 0);
	jtag_clock(priv, 0, 0);
}

/* Run-Test/Idle -> Shift-IR */
static void jtag_enter_shift_ir(struct gowin_priv *priv)
{
	jtag_clock(priv, 1, 0);	/* Select-DR-Scan */
	jtag_clock(priv, 1, 0);	/* Select-IR-Scan */
	jtag_clock(priv, 0, 0);	/* Capture-IR */
	jtag_clock(priv, 0, 0);	/* Shift-IR */
}

/* Run-Test/Idle -> Shift-DR */
static void jtag_enter_shift_dr(struct gowin_priv *priv)
{
	jtag_clock(priv, 1, 0);	/* Select-DR-Scan */
	jtag_clock(priv, 0, 0);	/* Capture-DR */
	jtag_clock(priv, 0, 0);	/* Shift-DR */
}

/*
 * Shift @nbits through the current shift register, LSB-first.
 * The last bit is clocked with TMS=1 and the TAP is walked through
 * Update back to Run-Test/Idle, so the shifted content always takes
 * effect. When @rx is given, TDO is sampled between the falling and
 * rising TCK edges.
 */
static void jtag_shift(struct gowin_priv *priv, const u8 *tx, u32 *rx,
		       int nbits)
{
	u32 val = 0;
	int i;

	for (i = 0; i < nbits; i++) {
		int tdi = tx ? (tx[i >> 3] >> (i & 7)) & 1 : 0;
		int last = i == nbits - 1;

		if (priv->jtag_tdi)
			gpiod_set_value(priv->jtag_tdi, tdi);
		jtag_set_tms(priv, last);
		jtag_tck_lo(priv);
		jtag_delay();
		if (rx && priv->jtag_tdo)
			val |= (u32)gpiod_get_value(priv->jtag_tdo) << i;
		jtag_tck_hi(priv);
		jtag_delay();
	}
	if (rx)
		*rx = val;

	jtag_clock(priv, 1, 0);	/* Exit1 -> Update */
	jtag_clock(priv, 0, 0);	/* Update -> Run-Test/Idle */
}

static void jtag_write_ir(struct gowin_priv *priv, u8 inst)
{
	jtag_enter_shift_ir(priv);
	jtag_shift(priv, &inst, NULL, 8);
	jtag_clocks(priv, 6);	/* Gowin: 6 idle clocks after each instruction */
}

static u32 jtag_read_reg32(struct gowin_priv *priv, u8 inst)
{
	static const u8 ones[4] = { 0xff, 0xff, 0xff, 0xff };
	u32 val = 0;

	jtag_write_ir(priv, inst);
	jtag_enter_shift_dr(priv);
	jtag_shift(priv, ones, &val, 32);
	return val;
}

/* Shift one 32-bit config word into DR: high byte first, LSB-first per byte */
static void jtag_write_word(struct gowin_priv *priv, const u8 *p)
{
	u8 tmp[4] = { p[3], p[2], p[1], p[0] };

	jtag_enter_shift_dr(priv);
	jtag_shift(priv, tmp, NULL, 32);
}

static u32 gowin_status(struct gowin_priv *priv)
{
	return jtag_read_reg32(priv, GWJ_READ_STATUS);
}

static int gowin_poll_status(struct gowin_priv *priv, u32 mask, u32 value,
			     unsigned int timeout_ms)
{
	u32 status;

	do {
		status = gowin_status(priv);
		if ((status & mask) == value)
			return 0;
		msleep(1);
	} while (timeout_ms--);

	dev_err(priv->dev,
		"jtag: status poll timeout (mask 0x%x, want 0x%x, have 0x%08x)\n",
		mask, value, status);
	return -ETIMEDOUT;
}

static int gowin_config_enable(struct gowin_priv *priv)
{
	jtag_write_ir(priv, GWJ_CONFIG_ENABLE);
	return gowin_poll_status(priv, GWS_SYSTEM_EDIT_MODE,
				 GWS_SYSTEM_EDIT_MODE, 100);
}

static int gowin_config_disable(struct gowin_priv *priv)
{
	jtag_write_ir(priv, GWJ_CONFIG_DISABLE);
	jtag_write_ir(priv, GWJ_NOOP);
	return gowin_poll_status(priv, GWS_SYSTEM_EDIT_MODE, 0, 100);
}

/* TN653 p.9-10: erase the configuration SRAM */
static int gowin_erase_sram(struct gowin_priv *priv)
{
	int ret;

	ret = gowin_config_enable(priv);
	if (ret)
		return ret;

	jtag_write_ir(priv, GWJ_ERASE_SRAM);
	jtag_write_ir(priv, GWJ_NOOP);

	/* MEMORY_ERASE goes high once the erase completes (TN653: ~4ms) */
	ret = gowin_poll_status(priv, GWS_MEMORY_ERASE, GWS_MEMORY_ERASE, 100);
	if (ret)
		return ret;

	jtag_write_ir(priv, GWJ_XFER_DONE);
	jtag_write_ir(priv, GWJ_NOOP);

	return gowin_config_disable(priv);
}

/* TN653 p.14-17: erase the embedded flash */
static int gowin_erase_flash(struct gowin_priv *priv)
{
	static const u8 zero[4] = { 0, 0, 0, 0 };
	u32 status;
	int ret;

	priv->erase_attempts++;

	ret = gowin_config_enable(priv);
	if (ret)
		return ret;

	jtag_write_ir(priv, GWJ_EFLASH_ERASE);

	jtag_enter_shift_dr(priv);
	jtag_shift(priv, zero, NULL, 32);

	/*
	 * TN653: no completion bit exists; wait out the erase (~160ms)
	 * with the clock running (150ms at 2.5MHz ~ 375000 clocks).
	 */
	jtag_clocks(priv, 375000);
	msleep(100);

	ret = gowin_config_disable(priv);
	if (ret)
		return ret;

	msleep(500);

	status = gowin_status(priv);
	if (status & GWS_DONE_FINAL) {
		dev_err(priv->dev, "jtag: flash erase failed, status 0x%08x\n",
			status);
		return -EIO;
	}
	return 0;
}

/* TN653 p.17-21: program the embedded flash in 256-byte pages */
static int gowin_write_flash(struct gowin_priv *priv, const u8 *data, int len)
{
	int pages = len / 256;
	int page, word;
	unsigned long flags;

	jtag_to_idle(priv);

	for (page = 0; page < pages; page++) {
		u8 addr[4];
		u32 a = (u32)page << 6;

		jtag_write_ir(priv, GWJ_CONFIG_ENABLE);
		jtag_write_ir(priv, GWJ_EF_PROGRAM);

		if (page)
			jtag_clocks(priv, 312);

		addr[0] = a & 0xff;
		addr[1] = (a >> 8) & 0xff;
		addr[2] = (a >> 16) & 0xff;
		addr[3] = (a >> 24) & 0xff;

		local_irq_save(flags);
		jtag_enter_shift_dr(priv);
		jtag_shift(priv, addr, NULL, 32);
		jtag_clocks(priv, 312);

		for (word = 0; word < 64; word++) {
			jtag_write_word(priv, data + page * 256 + word * 4);
			jtag_clocks(priv, 40);
		}
		local_irq_restore(flags);
	}

	jtag_to_idle(priv);
	return 0;
}

/* Full internal-flash programming sequence (TN653 p.9-21) */
static int gowin_program_flash(struct gowin_priv *priv, const u8 *data,
			       int len)
{
	u32 idcode, status, usercode;
	int ret;

	/* Talk to the TAP first: right chip, live link? */
	idcode = jtag_read_reg32(priv, GWJ_READ_IDCODE);
	dev_info(priv->dev, "jtag: FPGA IDCODE 0x%08x\n", idcode);
	if ((idcode & 0x0fffffff) != GOWIN_IDCODE) {
		dev_err(priv->dev,
			"jtag: unexpected FPGA IDCODE 0x%08x (expected 0x%08x)\n",
			idcode, GOWIN_IDCODE);
		return -ENODEV;
	}

	status = gowin_status(priv);
	if (!(status & (GWS_GOWIN_VLD | GWS_POR))) {
		dev_err(priv->dev,
			"jtag: neither VLD nor POR set, status 0x%08x\n", status);
		return -EIO;
	}

	ret = gowin_erase_sram(priv);
	if (ret)
		return ret;

	ret = gowin_erase_flash(priv);
	if (ret)
		return ret;

	ret = gowin_write_flash(priv, data, len);
	if (ret)
		return ret;

	ret = gowin_config_disable(priv);
	if (ret)
		return ret;

	jtag_write_ir(priv, GWJ_RELOAD);
	jtag_write_ir(priv, GWJ_NOOP);

	/* The FPGA now reloads its configuration from flash */
	msleep(600);

	status = gowin_status(priv);
	usercode = jtag_read_reg32(priv, GWJ_READ_USERCODE);
	dev_info(priv->dev, "jtag: post-program status 0x%08x usercode 0x%08x\n",
		 status, usercode);

	if (status & GWS_ERROR_MASK) {
		dev_err(priv->dev, "jtag: status error bits set: 0x%08x\n",
			status);
		return -EIO;
	}
	if (!(status & GWS_DONE_FINAL) || !(status & GWS_GOWIN_VLD)) {
		dev_err(priv->dev,
			"jtag: FPGA not configured after reload, status 0x%08x\n",
			status);
		return -EIO;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Firmware update                                                     */
/* ------------------------------------------------------------------ */

static void fpga_hw_reset(struct gowin_priv *priv)
{
	if (priv->reset_gpio) {
		dev_dbg(priv->dev, "fpga hardware reset\n");
		gpiod_set_value(priv->reset_gpio, 0);
		msleep(10);
		gpiod_set_value(priv->reset_gpio, 1);
		msleep(100);
	} else {
		dev_warn(priv->dev, "no reset GPIO, skipping FPGA hardware reset\n");
	}
}

static int select_jtag_mode(struct gowin_priv *priv)
{
	int ret;

	if (priv->pinctrl_jtag) {
		ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_jtag);
		if (ret) {
			dev_err(priv->dev, "failed to select jtag pinctrl state: %d\n", ret);
			return ret;
		}
	}

	if (priv->jtag_sel)
		gpiod_set_value(priv->jtag_sel, 0);

	priv->jtag_mode = true;
	dev_dbg(priv->dev, "switched to JTAG mode\n");
	return 0;
}

static int select_spi_mode(struct gowin_priv *priv)
{
	if (priv->jtag_sel)
		gpiod_set_value(priv->jtag_sel, 1);

	if (priv->pinctrl_default) {
		int ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_default);
		if (ret) {
			dev_err(priv->dev, "failed to select default pinctrl state: %d\n", ret);
			return ret;
		}
	}

	priv->jtag_mode = false;
	dev_dbg(priv->dev, "switched to SPI mode\n");
	return 0;
}

static int lp_update_firmware(struct gowin_priv *priv,
			      const u8 *data, int len)
{
	int attempt, ret = -EIO;

	for (attempt = 1; attempt <= 3; attempt++) {
		ret = select_jtag_mode(priv);
		if (ret) {
			dev_err(priv->dev, "failed to select JTAG mode: %d\n", ret);
			return ret;
		}

		fpga_hw_reset(priv);
		ret = gowin_program_flash(priv, data, len);
		select_spi_mode(priv);
		priv->prog_attempts = attempt;
		if (!ret) {
			dev_info(priv->dev, "firmware update: done\n");
			return 0;
		}

		dev_warn(priv->dev, "jtag: programming attempt %d failed: %d\n",
			 attempt, ret);
		msleep(200);
	}

	return ret;
}

static bool jtag_pins_available(struct gowin_priv *priv)
{
	bool ok = true;

	if (!priv->jtag_tms) {
		dev_err(priv->dev, "jtag: TMS line unavailable, cannot program\n");
		ok = false;
	}
	if (!priv->jtag_tdi) {
		dev_err(priv->dev, "jtag: TDI line unavailable, cannot program\n");
		ok = false;
	}
	if (!priv->jtag_tdo) {
		dev_err(priv->dev, "jtag: TDO line unavailable, cannot program\n");
		ok = false;
	}
	if (!priv->jtag_tck && !priv->jtag_tck_reg) {
		dev_err(priv->dev, "jtag: TCK line unavailable, cannot program\n");
		ok = false;
	}
	return ok;
}

/*
 * Build the flash image around the embedded bitstream: the "GW1N"
 * autoboot magic plus 20 dummy bytes precede the bitstream, and the
 * whole image is padded with 0xff to the 256-byte flash page
 * (TN653 p.17).
 */
static int lp_flash_embedded_bitstream(struct gowin_priv *priv)
{
	const size_t hdr = 24;
	size_t img_len;
	u8 *img;
	int ret;

	img_len = round_up(hdr + fpga_bitstream_len, 256);
	img = kvzalloc(img_len, GFP_KERNEL);
	if (!img)
		return -ENOMEM;

	memset(img, 0xff, img_len);
	memcpy(img, "GW1N", 4);
	memcpy(img + hdr, fpga_bitstream, fpga_bitstream_len);

	ret = lp_update_firmware(priv, img, img_len);

	kvfree(img);
	return ret;
}

static int lp_handle_firmware_update(struct gowin_priv *priv)
{
	int ret;

	if (!jtag_pins_available(priv)) {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "FAILED: JTAG pins unavailable");
		return -ENODEV;
	}

	dev_info(priv->dev, "programming embedded FPGA bitstream (%zu bytes)\n",
		 fpga_bitstream_len);

	snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
		 "flashing embedded bitstream (%u bytes)", fpga_bitstream_len);

	ret = lp_flash_embedded_bitstream(priv);

	if (ret) {
		dev_err(priv->dev, "embedded FPGA bitstream programming FAILED: %d\n", ret);
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "FAILED: %d", ret);
	} else {
		dev_info(priv->dev, "embedded FPGA bitstream programmed successfully\n");
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "ok: embedded bitstream programmed");
	}

	return ret;
}

/* ------------------------------------------------------------------ */
/* Sysfs                                                              */
/* ------------------------------------------------------------------ */

static ssize_t help_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf,
		"help                 - this help\n"
		"open_fb_count        - number of open fb instances\n"
		"fw_upd_status        - firmware update status\n"
		"crc_errors           - slave/master CRC errors\n"
		"power_saving_mode    - (rw) 0=OFF 1=ON\n"
		"set_animation_mode   - (wo) 0=stop 1=start anim\n"
		"gpio_mode            - (rw) jtag|spi\n"
		"frame_queue_overflows- frame queue overflow count\n"
		"next_frame_delay     - (rw) frame delay\n"
		"show_debug_info      - touch debug registers\n"
		"frames_in_queue      - frames in queue\n"
		"press_key            - (wo) keycode to send\n"
		"jtag/jtag_reconfig   - (rw) JTAG reconfig pin\n"
		"jtag/jtag_codes      - ID/USER/STATUS codes\n"
		"jtag/jtag_erase_flash- flash erase attempts\n"
		"jtag/jtag_fpga_prog  - FPGA prog attempts\n"
		"jtag/test_fpga_prog  - (wo) test FPGA prog N times\n");
}

static ssize_t open_fb_count_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", priv->fb_open_count);
}

static ssize_t fw_upd_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", priv->fw_upd_status);
}

static ssize_t crc_errors_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "master=%u slave=%u\n",
			  priv->crc_errors_master, priv->crc_errors_slave);
}

static ssize_t power_saving_mode_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", priv->power_saving ? 1 : 0);
}

static ssize_t power_saving_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	priv->power_saving = !!val;
	set_power_saving_mode_cmd(priv, priv->power_saving);

	return len;
}

static ssize_t set_animation_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	set_animation_app_cmd(priv, val);
	return len;
}

static ssize_t gpio_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", priv->jtag_mode ? "jtag" : "spi");
}

static ssize_t gpio_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	int ret;

	if (sysfs_streq(buf, "jtag"))
		ret = select_jtag_mode(priv);
	else
		ret = select_spi_mode(priv);

	return ret ? ret : len;
}

static ssize_t frame_queue_overflows_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->frame_queue_overflows);
}

static ssize_t next_frame_delay_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->next_frame_delay);
}

static ssize_t next_frame_delay_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = kstrtouint(buf, 0, &priv->next_frame_delay);
	if (ret)
		return ret;

	return len;
}

static ssize_t show_debug_info_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "touch: state=%d mask=%d fqc=%u\n",
			  0, 0, priv->frame_queue_overflows);
}

static ssize_t frames_in_queue_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->frames_in_queue);
}

static ssize_t press_key_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	unsigned int key;
	int ret;

	if (!priv->touch_dev)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &key);
	if (ret)
		return ret;

	input_event(priv->touch_dev, EV_KEY, key, 1);
	input_event(priv->touch_dev, EV_SYN, SYN_REPORT, 0);
	input_event(priv->touch_dev, EV_KEY, key, 0);
	input_event(priv->touch_dev, EV_SYN, SYN_REPORT, 0);

	return len;
}

/* Main group attributes */
static DEVICE_ATTR_RO(help);
static DEVICE_ATTR_RO(open_fb_count);
static DEVICE_ATTR_RO(fw_upd_status);
static DEVICE_ATTR_RO(crc_errors);
static DEVICE_ATTR_RW(power_saving_mode);
static DEVICE_ATTR_WO(set_animation_mode);
static DEVICE_ATTR_RW(gpio_mode);
static DEVICE_ATTR_RO(frame_queue_overflows);
static DEVICE_ATTR_RW(next_frame_delay);
static DEVICE_ATTR_RO(show_debug_info);
static DEVICE_ATTR_RO(frames_in_queue);
static DEVICE_ATTR_WO(press_key);

static struct attribute *lp_common_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_open_fb_count.attr,
	&dev_attr_fw_upd_status.attr,
	&dev_attr_crc_errors.attr,
	&dev_attr_power_saving_mode.attr,
	&dev_attr_set_animation_mode.attr,
	&dev_attr_gpio_mode.attr,
	&dev_attr_frame_queue_overflows.attr,
	&dev_attr_next_frame_delay.attr,
	&dev_attr_show_debug_info.attr,
	&dev_attr_frames_in_queue.attr,
	&dev_attr_press_key.attr,
	NULL,
};

static const struct attribute_group lp_common_group = {
	.attrs = lp_common_attrs,
};

/* JTAG group */
static ssize_t jtag_reconfig_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	int val = priv->jtag_reconfig ? gpiod_get_value(priv->jtag_reconfig) : 0;
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t jtag_reconfig_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (priv->jtag_reconfig)
		gpiod_set_value(priv->jtag_reconfig, !!val);

	return len;
}

static ssize_t jtag_fpga_prog_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->prog_attempts);
}

static ssize_t jtag_codes_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	u32 idcode = 0, usercode = 0, status = 0;

	if (!jtag_pins_available(priv))
		return sysfs_emit(buf, "ID=0x%08x USER=0x%08x STATUS=0x%08x\n",
				  idcode, usercode, status);

	if (select_jtag_mode(priv) == 0) {
		jtag_to_idle(priv);
		idcode = jtag_read_reg32(priv, GWJ_READ_IDCODE);
		usercode = jtag_read_reg32(priv, GWJ_READ_USERCODE);
		status = gowin_status(priv);
		select_spi_mode(priv);
	}

	return sysfs_emit(buf, "ID=0x%08x USER=0x%08x STATUS=0x%08x\n",
			  idcode, usercode, status);
}

static ssize_t jtag_erase_flash_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->erase_attempts);
}

static ssize_t test_fpga_prog_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct gowin_priv *priv = dev_get_drvdata(dev);
	unsigned int n, i;
	int ret;

	ret = kstrtouint(buf, 0, &n);
	if (ret)
		return ret;
	if (n > 10)
		n = 10;

	if (!jtag_pins_available(priv))
		return -ENODEV;

	for (i = 0; i < n; i++) {
		ret = lp_flash_embedded_bitstream(priv);
		if (ret) {
			dev_err(dev, "test cycle %u/%u failed: %d\n",
				i + 1, n, ret);
			return ret;
		}
	}

	return len;
}

static DEVICE_ATTR_RW(jtag_reconfig);
static DEVICE_ATTR_RO(jtag_fpga_prog);
static DEVICE_ATTR_RO(jtag_codes);
static DEVICE_ATTR_RO(jtag_erase_flash);
static DEVICE_ATTR_WO(test_fpga_prog);

static struct attribute *jtag_attrs[] = {
	&dev_attr_jtag_reconfig.attr,
	&dev_attr_jtag_fpga_prog.attr,
	&dev_attr_jtag_codes.attr,
	&dev_attr_jtag_erase_flash.attr,
	&dev_attr_test_fpga_prog.attr,
	NULL,
};

static const struct attribute_group jtag_group = {
	.name = "jtag",
	.attrs = jtag_attrs,
};

static const struct attribute_group *lp_attr_groups[] = {
	&lp_common_group,
	&jtag_group,
	NULL,
};

/* ------------------------------------------------------------------ */
/* SPI driver probe/remove                                            */
/* ------------------------------------------------------------------ */

static const struct spi_device_id led_device_id[] = {
	{ "atri,led-panel" },
	{ "ya,led-panel" },
	{ "ya,led_screen" },
	{ }
};

MODULE_DEVICE_TABLE(spi, led_device_id);

static const struct of_device_id led_device_of_match[] = {
	{ .compatible = "atri,led-panel", .data = &rev2res_panel },
	{ .compatible = "ya,led-panel", .data = &rev2res_panel },
	{ .compatible = "ya,led_screen", .data = &rev2res_screen },
	{ }
};

MODULE_DEVICE_TABLE(of, led_device_of_match);

static int probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct of_device_id *match;
	const struct gowin_resolution *res;
	struct gowin_priv *priv;
	int ret;

	match = of_match_node(led_device_of_match, dev->of_node);
	if (!match || !match->data)
		return -EINVAL;

	res = match->data;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi = spi;
	priv->dev = dev;
	priv->width = res->width;
	priv->height = res->height;
	mutex_init(&priv->lock);
	spi_set_drvdata(spi, priv);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = 10 * 1000 * 1000;
	ret = spi_setup(spi);
	if (ret)
		return ret;

/* GPIOs */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		ret = PTR_ERR(priv->reset_gpio);
		dev_err(dev, "failed to get reset GPIO: %d\n", ret);
		return ret;
	}

	priv->irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
	if (IS_ERR(priv->irq_gpio)) {
		ret = PTR_ERR(priv->irq_gpio);
		dev_err(dev, "failed to get irq GPIO: %d\n", ret);
		return ret;
	}

	priv->jtag_tck = devm_gpiod_get_optional(dev, "jtag_tck", GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_tck)) {
		dev_warn(dev, "jtag_tck GPIO unavailable (%ld), proceeding without\n",
			 PTR_ERR(priv->jtag_tck));
		priv->jtag_tck = NULL;
	}

	priv->jtag_tdo = devm_gpiod_get_optional(dev, "jtag_tdo", GPIOD_IN);
	if (IS_ERR(priv->jtag_tdo)) {
		dev_warn(dev, "jtag_tdo GPIO unavailable (%ld), proceeding without\n",
			 PTR_ERR(priv->jtag_tdo));
		priv->jtag_tdo = NULL;
	}

	priv->jtag_tdi = devm_gpiod_get_optional(dev, "jtag_tdi", GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_tdi)) {
		dev_warn(dev, "jtag_tdi GPIO unavailable (%ld), proceeding without\n",
			 PTR_ERR(priv->jtag_tdi));
		priv->jtag_tdi = NULL;
	}

	priv->jtag_tms = devm_gpiod_get_optional(dev, "jtag_tms", GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_tms)) {
		/*
		 * Stock wiring shares JTAG TMS with the SPI chip-select
		 * line (GPIOH_7): the SPI core already owns it as CS, so
		 * reuse that descriptor instead of failing. JTAG only
		 * runs while the SPI bus is idle (jtag_mode).
		 */
		if (PTR_ERR(priv->jtag_tms) == -EBUSY && spi->cs_gpiod[0]) {
			priv->jtag_tms = spi->cs_gpiod[0];
			priv->jtag_tms_via_cs = true;
			dev_info(dev, "jtag_tms: reusing SPI CS line (shared pin)\n");
		} else {
			dev_warn(dev, "jtag_tms GPIO unavailable (%ld), proceeding without\n",
				 PTR_ERR(priv->jtag_tms));
			priv->jtag_tms = NULL;
		}
	}

	priv->jtag_sel = devm_gpiod_get_optional(dev, "jtag_sel", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->jtag_sel)) {
		dev_warn(dev, "jtag_sel GPIO unavailable (%ld), proceeding without\n",
			 PTR_ERR(priv->jtag_sel));
		priv->jtag_sel = NULL;
	}

	priv->jtag_reconfig = devm_gpiod_get_optional(dev, "jtag_reconfig",
						       GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_reconfig)) {
		dev_warn(dev, "jtag_reconfig GPIO unavailable (%ld), proceeding without\n",
			 PTR_ERR(priv->jtag_reconfig));
		priv->jtag_reconfig = NULL;
	}

	/* Optional memory-mapped JTAG TCK descriptor: <addr bit> */
	{
		struct device_node *np = dev->of_node;
		u32 tck_desc[2] = { 0, 0 };

		if (!of_property_read_u32_array(np, "jtag_tck_desc", tck_desc, 2)) {
			phys_addr_t pa = tck_desc[0];

			priv->jtag_tck_bit = tck_desc[1];
			priv->jtag_tck_reg = devm_ioremap(dev, pa, sizeof(u32));
			if (!priv->jtag_tck_reg)
				dev_warn(dev, "failed to ioremap jtag_tck_desc %pa\n", &pa);
			else
				dev_info(dev, "JTAG TCK via MMIO %pa bit %u\n",
					 &pa, priv->jtag_tck_bit);
		}
	}

	/* pinctrl */
	priv->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(priv->pinctrl)) {
		priv->pinctrl_default = pinctrl_lookup_state(priv->pinctrl,
							    "default");
		if (IS_ERR(priv->pinctrl_default))
			priv->pinctrl_default = NULL;

		priv->pinctrl_jtag = pinctrl_lookup_state(priv->pinctrl,
							  "jtag");
		if (IS_ERR(priv->pinctrl_jtag))
			priv->pinctrl_jtag = NULL;
	} else {
		priv->pinctrl = NULL;
	}

	/* Select SPI mode by default */
	select_spi_mode(priv);

	/* Initialize subsystems */
	dev_dbg(dev, "initializing GPIO\n");
	ret = gpio_init(priv);
	if (ret)
		return ret;

	dev_dbg(dev, "initializing framebuffer\n");
	ret = fb_init(priv);
	if (ret)
		goto err_gpio;

	dev_dbg(dev, "initializing backlight\n");
	ret = backlight_init(priv);
	if (ret)
		goto err_fb;

	dev_dbg(dev, "initializing UART\n");
	ret = uart_init(priv);
	if (ret)
		goto err_bl;

	dev_dbg(dev, "initializing touch\n");
	ret = init_touch(priv);
	if (ret)
		goto err_uart;

	dev_dbg(dev, "creating sysfs groups\n");
	ret = sysfs_create_groups(&dev->kobj, lp_attr_groups);
	if (ret)
		goto err_touch;

	/* Firmware update */
	lp_handle_firmware_update(priv);

	dev_info(dev, "led panel probed: %dx%d\n", priv->width, priv->height);
	return 0;

err_touch:
	dev_err(dev, "probe failed at touch init: %d\n", ret);
	deinit_touch(priv);
err_uart:
	dev_err(dev, "probe failed at UART init: %d\n", ret);
	uart_exit(priv);
err_bl:
	dev_err(dev, "probe failed at backlight init: %d\n", ret);
	backlight_exit(priv);
err_fb:
	dev_err(dev, "probe failed at fb init: %d\n", ret);
	fb_exit(priv);
err_gpio:
	dev_err(dev, "probe failed at gpio init: %d\n", ret);
	gpio_exit(priv);
	return ret;
}

static void remove(struct spi_device *spi)
{
	struct gowin_priv *priv = spi_get_drvdata(spi);

	sysfs_remove_groups(&priv->dev->kobj, lp_attr_groups);
	deinit_touch(priv);
	uart_exit(priv);
	backlight_exit(priv);
	fb_exit(priv);
	gpio_exit(priv);

	dev_info(priv->dev, "led panel removed\n");
}

static struct spi_driver led_device_driver = {
	.driver = {
		.name = GOWIN_DRV_NAME,
		.of_match_table = led_device_of_match,
	},
	.probe = probe,
	.remove = remove,
	.id_table = led_device_id,
};

module_spi_driver(led_device_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("AtriOS Team");
MODULE_DESCRIPTION("Quasar led panel (SPI)");
