// SPDX-License-Identifier: GPL-2.0
/*
 * atri_led_panel.c - Quasar LED panel/screen SPI driver
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
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/firmware.h>
#include <linux/pinctrl/consumer.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/string.h>

/* CRC-ITU-T V.41 (x^16 + x^12 + x^5 + 1, poly 0x1021) - embedded so we
 * don't depend on CONFIG_CRC_ITU_T being built-in.
 */
static const u16 alp_crc_itu_t_table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static u16 alp_crc_itu_t(u16 crc, const u8 *buffer, size_t len)
{
	while (len--) {
		u8 b = *buffer++;

		crc = alp_crc_itu_t_table[((crc >> 8) ^ b) & 0xff] ^ (crc << 8);
	}
	return crc;
}

#define ATRI_DRV_NAME		"atri_led_panel"
#define ATRI_FB_NAME		"atri_led_panel_fb"

#define ATRI_GPIO_NGPIO	2
#define ATRI_TTY_MINOR_COUNT	1
#define ATRI_TTY_BUF_SIZE	64
#define ATRI_FB_WIDTH_MAX	320
#define ATRI_FB_HEIGHT_MAX	320

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

struct atri_resolution {
	int width;
	int height;
};

static const struct atri_resolution rev2res_panel = { .width = 25, .height = 16 };
static const struct atri_resolution rev2res_screen = { .width = 28, .height = 16 };

struct atri_touch_regs {
	u32 dbg_val[5];
	u32 metric[5];
	u32 touch_state;
	u32 touch_mask;
};

struct atri_priv {
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

	/* memory-mapped JTAG TCK */
	void __iomem *jtag_tck_reg;
	u32 jtag_tck_bit;

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
	char fw_upd_status[64];

	/* Subsystem registration flags for safe teardown */
	bool tty_registered;
	bool touch_input_registered;

	/* Lock */
	struct mutex lock;

	/* TTY xmit buffer */
	unsigned char xmit_buf[ATRI_TTY_BUF_SIZE];
	int xmit_count;
};

/* ------------------------------------------------------------------ */
/* App protocol -- SPI command layer                                   */
/* ------------------------------------------------------------------ */

static int alp_write(struct atri_priv *priv, const u8 *data, int len)
{
	struct spi_transfer t = {
		.tx_buf = data,
		.len = len,
	};
	struct spi_message m;

	spi_message_init_with_transfers(&m, &t, 1);
	return spi_sync(priv->spi, &m);
}

static int get_alp_status(struct atri_priv *priv, u8 *status_buf)
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

	spi_message_init_with_transfers(&m, &tx, 1);
	spi_message_add_tail(&rx, &m);
	return spi_sync(priv->spi, &m);
}

static int show_pic_app_cmd(struct atri_priv *priv)
{
	u8 buf[3] = { CMD_SHOW_PIC, 0x00, 0x00 };
	return alp_write(priv, buf, 3);
}

static int set_brightness_app_cmd(struct atri_priv *priv, unsigned int val)
{
	u8 buf[3] = { CMD_SET_BRIGHTNESS, val & 0xff, (val >> 8) & 0xff };
	return alp_write(priv, buf, 3);
}

static int set_gpio_app_cmd(struct atri_priv *priv)
{
	u8 buf[3] = { CMD_SET_GPIO, priv->gpio_cached, 0x00 };
	return alp_write(priv, buf, 3);
}

static int set_animation_app_cmd(struct atri_priv *priv, int on)
{
	u8 buf[3] = { CMD_SET_ANIMATION, !!on, 0x00 };
	return alp_write(priv, buf, 3);
}

static int set_power_saving_mode_cmd(struct atri_priv *priv, int on)
{
	u8 buf[3] = { CMD_SET_POWER_SAVING, !!on, 0x00 };
	return alp_write(priv, buf, 3);
}

static int uart_send_data_app_cmd(struct atri_priv *priv,
				  const u8 *data, int len)
{
	u8 buf[4] = { CMD_UART_SEND_DATA, len & 0xff, (len >> 8) & 0xff, 0 };
	int ret;

	ret = alp_write(priv, buf, 4);
	if (ret)
		return ret;

	return alp_write(priv, data, len);
}

static int uart_receive_data(struct atri_priv *priv,
			     u8 *rx_data, int *rx_len)
{
	u8 cmd[3] = { CMD_UART_RECV_DATA, 0, 0 };
	u8 header[4];
	struct spi_transfer t[3];
	struct spi_message m;

	memset(t, 0, sizeof(t));
	t[0].tx_buf = cmd;
	t[0].len = 3;
	t[1].delay.value = 100;
	t[1].delay.unit = SPI_DELAY_UNIT_USECS;
	t[1].len = 0;
	t[2].rx_buf = header;
	t[2].len = 4;
	spi_message_init_with_transfers(&m, t, 3);

	if (spi_sync(priv->spi, &m))
		return -EIO;

	*rx_len = header[0] | (header[1] << 8);
	if (*rx_len > ATRI_TTY_BUF_SIZE)
		*rx_len = ATRI_TTY_BUF_SIZE;
	if (*rx_len == 0)
		return 0;

	{
		struct spi_transfer r = {
			.rx_buf = rx_data,
			.len = *rx_len,
		};
		spi_message_init_with_transfers(&m, &r, 1);
		return spi_sync(priv->spi, &m);
	}
}

static int __maybe_unused uart_flush_data_app_cmd(struct atri_priv *priv)
{
	u8 buf[3] = { CMD_UART_FLUSH_DATA, 0, 0 };
	return alp_write(priv, buf, 3);
}

/* ------------------------------------------------------------------ */
/* GPIO chip                                                           */
/* ------------------------------------------------------------------ */

static int alp_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static int alp_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return -EINVAL;
}

static int alp_gpio_get_value(struct gpio_chip *chip, unsigned int offset)
{
	struct atri_priv *priv = gpiochip_get_data(chip);
	return (priv->gpio_cached >> offset) & 1;
}

static int alp_gpio_set_value(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct atri_priv *priv = gpiochip_get_data(chip);

	if (value)
		priv->gpio_cached |= BIT(offset);
	else
		priv->gpio_cached &= ~BIT(offset);

	set_gpio_app_cmd(priv);
	return 0;
}

static int alp_gpio_direction_output(struct gpio_chip *chip,
				    unsigned int offset, int value)
{
	return alp_gpio_set_value(chip, offset, value);
}

static int gpio_init(struct atri_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->gpio_chip.label = "atri_led_panel";
	priv->gpio_chip.base = -1;
	priv->gpio_chip.ngpio = ATRI_GPIO_NGPIO;
	priv->gpio_chip.get_direction = alp_gpio_get_direction;
	priv->gpio_chip.direction_input = alp_gpio_direction_input;
	priv->gpio_chip.direction_output = alp_gpio_direction_output;
	priv->gpio_chip.get = alp_gpio_get_value;
	priv->gpio_chip.set = alp_gpio_set_value;
	priv->gpio_chip.parent = dev;
	priv->gpio_chip.fwnode = dev_fwnode(dev);
	priv->gpio_chip.of_gpio_n_cells = 2;
	priv->gpio_chip.can_sleep = true;

	ret = gpiochip_add_data(&priv->gpio_chip, priv);
	if (ret)
		dev_err(dev, "gpiochip_add_data failed: %d\n", ret);

	return ret;
}

static void gpio_exit(struct atri_priv *priv)
{
	gpiochip_remove(&priv->gpio_chip);
}

/* ------------------------------------------------------------------ */
/* TTY / UART                                                         */
/* ------------------------------------------------------------------ */

static int alp_uart_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void alp_uart_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(tty->port, tty, filp);
}

static unsigned int alp_uart_write_room(struct tty_struct *tty)
{
	return ATRI_TTY_BUF_SIZE;
}

static ssize_t alp_uart_write(struct tty_struct *tty, const u8 *buf,
			     size_t count)
{
	struct atri_priv *priv = container_of(tty->port, struct atri_priv,
					       tty_port);

	if (count > ATRI_TTY_BUF_SIZE)
		count = ATRI_TTY_BUF_SIZE;

	if (uart_send_data_app_cmd(priv, buf, count))
		return 0;

	return count;
}

static void alp_uart_rx_work(struct work_struct *work)
{
	struct atri_priv *priv = container_of(work, struct atri_priv, uart_rx_work);
	u8 rx_buf[ATRI_TTY_BUF_SIZE];
	int rx_len;
	int ret;

	ret = uart_receive_data(priv, rx_buf, &rx_len);
	if (ret || rx_len <= 0)
		return;

	tty_insert_flip_string_fixed_flag(&priv->tty_port, rx_buf,
					  TTY_NORMAL, rx_len);
	tty_flip_buffer_push(&priv->tty_port);
}

static void alp_uart_poll(struct timer_list *t)
{
	struct atri_priv *priv = container_of(t, struct atri_priv,
					       uart_timer);

	if (priv->tty_port.tty)
		schedule_work(&priv->uart_rx_work);

	mod_timer(&priv->uart_timer, jiffies + HZ / 10);
}

static const struct tty_operations alp_uart_ops = {
	.open = alp_uart_open,
	.close = alp_uart_close,
	.write = alp_uart_write,
	.write_room = alp_uart_write_room,
};

static int uart_init(struct atri_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->tty_drv = tty_alloc_driver(ATRI_TTY_MINOR_COUNT,
					 TTY_DRIVER_REAL_RAW |
					 TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(priv->tty_drv))
		return PTR_ERR(priv->tty_drv);

	priv->tty_drv->driver_name = ATRI_DRV_NAME;
	priv->tty_drv->name = "ttyLP";
	priv->tty_drv->major = 0;
	priv->tty_drv->minor_start = 0;
	priv->tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	priv->tty_drv->subtype = SERIAL_TYPE_NORMAL;
	priv->tty_drv->init_termios = tty_std_termios;
	priv->tty_drv->init_termios.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	tty_set_operations(priv->tty_drv, &alp_uart_ops);

	INIT_WORK(&priv->uart_rx_work, alp_uart_rx_work);

	tty_port_init(&priv->tty_port);
	tty_port_link_device(&priv->tty_port, priv->tty_drv, 0);

	ret = tty_register_driver(priv->tty_drv);
	if (ret) {
		dev_err(dev, "tty_register_driver failed: %d\n", ret);
		return ret;
	}

	priv->tty_registered = true;
	timer_setup(&priv->uart_timer, alp_uart_poll, 0);
	mod_timer(&priv->uart_timer, jiffies + HZ / 10);

	dev_info(dev, "uart ttyLP0 initialized\n");
	return 0;
}

static void uart_exit(struct atri_priv *priv)
{
	if (!IS_ERR_OR_NULL(priv->tty_drv) && priv->tty_registered) {
		timer_delete_sync(&priv->uart_timer);
		flush_work(&priv->uart_rx_work);
		tty_unregister_driver(priv->tty_drv);
		priv->tty_registered = false;
	}

	if (!IS_ERR_OR_NULL(priv->tty_drv)) {
		tty_driver_kref_put(priv->tty_drv);
		priv->tty_drv = NULL;
	}

	tty_port_destroy(&priv->tty_port);
}

/* ------------------------------------------------------------------ */
/* Touch                                                              */
/* ------------------------------------------------------------------ */

static void touch_work(struct work_struct *work)
{
	struct atri_priv *priv = container_of(work, struct atri_priv, touch_work);
	struct atri_touch_regs regs;
	u8 status[CMD_GET_STATUS_LEN];
	int ret;

	ret = get_alp_status(priv, status);
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

	if (regs.touch_mask) {
		input_event(priv->touch_dev, EV_KEY, KEY_BACK, 1);
		input_event(priv->touch_dev, EV_SYN, SYN_REPORT, 0);
		input_event(priv->touch_dev, EV_KEY, KEY_BACK, 0);
		input_event(priv->touch_dev, EV_SYN, SYN_REPORT, 0);
	}
}

static irqreturn_t alp_touch_isr(int irq, void *data)
{
	struct atri_priv *priv = data;

	queue_work(priv->touch_wq, &priv->touch_work);
	return IRQ_HANDLED;
}

static int init_touch(struct atri_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->touch_dev = devm_input_allocate_device(dev);
	if (!priv->touch_dev)
		return -ENOMEM;

	priv->touch_dev->name = "midi_touch";
	priv->touch_dev->phys = "atri_led_panel/input0";
	priv->touch_dev->id.bustype = BUS_SPI;
	input_set_capability(priv->touch_dev, EV_KEY, KEY_BACK);

	priv->touch_wq = alloc_workqueue("touch_wq", WQ_UNBOUND, 1);
	if (!priv->touch_wq)
		return -ENOMEM;

	INIT_WORK(&priv->touch_work, touch_work);

	ret = input_register_device(priv->touch_dev);
	if (ret)
		return ret;
	priv->touch_input_registered = true;

	if (priv->irq_gpio) {
		int irq = gpiod_to_irq(priv->irq_gpio);
		if (irq > 0) {
			ret = devm_request_threaded_irq(dev, irq, NULL,
							alp_touch_isr,
							IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT,
							ATRI_DRV_NAME, priv);
			if (ret)
				return ret;
			priv->touch_irq_number = irq;
		}
	}

	return 0;
}

static void deinit_touch(struct atri_priv *priv)
{
	if (priv->touch_irq_number)
		devm_free_irq(priv->dev, priv->touch_irq_number, priv);

	if (priv->touch_input_registered && priv->touch_dev) {
		input_unregister_device(priv->touch_dev);
		priv->touch_input_registered = false;
	}

	if (priv->touch_wq) {
		flush_work(&priv->touch_work);
		destroy_workqueue(priv->touch_wq);
		priv->touch_wq = NULL;
	}
}

static void __maybe_unused reset_touch_state(struct atri_priv *priv)
{
	priv->fb_open_count = 0;
}

/* ------------------------------------------------------------------ */
/* Framebuffer                                                        */
/* ------------------------------------------------------------------ */

static int alp_fb_open(struct fb_info *fbi, int user)
{
	struct atri_priv *priv = (struct atri_priv *)fbi->par;

	priv->fb_open_count++;
	return 0;
}

static int alp_fb_release(struct fb_info *fbi, int user)
{
	struct atri_priv *priv = (struct atri_priv *)fbi->par;

	if (priv->fb_open_count > 0)
		priv->fb_open_count--;
	return 0;
}

static int alp_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
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

static int alp_fb_set_par(struct fb_info *fbi)
{
	return 0;
}

static int alp_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	return 0;
}

static int alp_fb_sync(struct fb_info *fbi)
{
	struct atri_priv *priv = (struct atri_priv *)fbi->par;
	u8 *buf;
	int len;
	int ret;

	len = priv->width * priv->height;
	if (len > ATRI_FB_WIDTH_MAX * ATRI_FB_HEIGHT_MAX)
		return -EINVAL;

	buf = kmalloc(3 + len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = CMD_WRITE;
	buf[1] = len & 0xff;
	buf[2] = (len >> 8) & 0xff;
	memcpy(buf + 3, fbi->screen_buffer, len);

	ret = alp_write(priv, buf, 3 + len);
	kfree(buf);

	if (ret == 0)
		show_pic_app_cmd(priv);

	return ret;
}

static void alp_fb_deferred_io(struct fb_info *fbi,
			       struct list_head *pagelist)
{
	alp_fb_sync(fbi);
}

static struct fb_deferred_io alp_fb_defio = {
	.delay = HZ / 30,
	.deferred_io = alp_fb_deferred_io,
};

static int alp_fb_blank(int blank, struct fb_info *fbi)
{
	if (blank == FB_BLANK_UNBLANK)
		return alp_fb_sync(fbi);
	return 0;
}

static void alp_fb_destroy(struct fb_info *fbi)
{
	struct atri_priv *priv = (struct atri_priv *)fbi->par;

	fb_deferred_io_cleanup(fbi);

	if (priv->fb_mem) {
		vfree(priv->fb_mem);
		priv->fb_mem = NULL;
	}
}

static const struct fb_ops fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = alp_fb_open,
	.fb_release = alp_fb_release,
	.fb_check_var = alp_fb_check_var,
	.fb_set_par = alp_fb_set_par,
	.fb_pan_display = alp_fb_pan_display,
	.fb_blank = alp_fb_blank,
	.fb_sync = alp_fb_sync,
	.fb_mmap = fb_deferred_io_mmap,
	.fb_destroy = alp_fb_destroy,
	.fb_read = fb_sys_read,
	.fb_write = fb_sys_write,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
};

static int fb_init(struct atri_priv *priv)
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

	fbi->screen_base = (char __iomem *)priv->fb_mem;
	fbi->screen_buffer = priv->fb_mem;
	fbi->screen_size = fb_len;
	fbi->fix.smem_len = fb_len;
	fbi->fix.smem_start = 0;
	fbi->fix.type = FB_TYPE_PACKED_PIXELS;
	fbi->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	fbi->fix.line_length = priv->width;
	fbi->fix.accel = FB_ACCEL_NONE;
	strscpy(fbi->fix.id, ATRI_FB_NAME, sizeof(fbi->fix.id));

	fbi->var.xres = priv->width;
	fbi->var.yres = priv->height;
	fbi->var.xres_virtual = priv->width;
	fbi->var.yres_virtual = priv->height;
	fbi->var.bits_per_pixel = 8;
	fbi->var.activate = FB_ACTIVATE_NOW;
	fbi->var.red.offset = 0;
	fbi->var.red.length = 8;

	fbi->pseudo_palette = priv->pseudo_palette;
	fbi->flags = FBINFO_VIRTFB;
	fbi->fbops = &fb_ops;
	fbi->fbdefio = &alp_fb_defio;

	ret = fb_deferred_io_init(fbi);
	if (ret) {
		dev_err(dev, "fb_deferred_io_init failed: %d\n", ret);
		goto err_mem;
	}

	ret = register_framebuffer(fbi);
	if (ret) {
		dev_err(dev, "register_framebuffer failed: %d\n", ret);
		goto err_mem;
	}

	dev_info(dev, "fb registered: %dx%d\n", priv->width, priv->height);
	return 0;

err_mem:
	if (fbi->fbdefio_state)
		fb_deferred_io_cleanup(fbi);
	vfree(priv->fb_mem);
err_fb:
	framebuffer_release(fbi);
	return ret;
}

static void fb_exit(struct atri_priv *priv)
{
	if (priv->fbi) {
		unregister_framebuffer(priv->fbi);
		framebuffer_release(priv->fbi);
		priv->fbi = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* Backlight                                                          */
/* ------------------------------------------------------------------ */

static int alp_bl_get_brightness(struct backlight_device *bl)
{
	struct atri_priv *priv = bl_get_data(bl);
	return priv->brightness;
}

static int alp_bl_update_status(struct backlight_device *bl)
{
	struct atri_priv *priv = bl_get_data(bl);

	priv->brightness = bl->props.brightness;
	set_brightness_app_cmd(priv, priv->brightness);

	return 0;
}

static const struct backlight_ops alp_bl_ops = {
	.get_brightness = alp_bl_get_brightness,
	.update_status = alp_bl_update_status,
};

static int backlight_init(struct atri_priv *priv)
{
	struct device *dev = priv->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = 255,
		.brightness = 128,
	};
	u32 max_brightness = 0;

	if (!of_property_read_u32(dev->of_node, "max-brightness", &max_brightness))
		props.max_brightness = min_t(u32, max_brightness, 255);

	priv->bl = devm_backlight_device_register(dev, ATRI_DRV_NAME, dev, priv,
						    &alp_bl_ops, &props);
	if (IS_ERR(priv->bl))
		return PTR_ERR(priv->bl);

	return 0;
}

static void backlight_exit(struct atri_priv *priv)
{
	/* devm-managed, no manual unregister needed */
	(void)priv;
}

/* ------------------------------------------------------------------ */
/* JTAG -- Gowin FPGA programming (reverse engineered from             */
/*          vendor gowin_led_device.ko for Linux 4.9)                  */
/* ------------------------------------------------------------------ */

static void jtag_tck_lo(struct atri_priv *priv)
{
	if (priv->jtag_tck_reg)
		writel_relaxed(readl_relaxed(priv->jtag_tck_reg) & ~BIT(priv->jtag_tck_bit),
			       priv->jtag_tck_reg);
	else if (priv->jtag_tck)
		gpiod_set_value(priv->jtag_tck, 0);
}

static void jtag_tck_hi(struct atri_priv *priv)
{
	if (priv->jtag_tck_reg)
		writel_relaxed(readl_relaxed(priv->jtag_tck_reg) | BIT(priv->jtag_tck_bit),
			       priv->jtag_tck_reg);
	else if (priv->jtag_tck)
		gpiod_set_value(priv->jtag_tck, 1);
}

/* Vendor bit-bang delay: ~100 CPU loops per half-period */
static void jtag_delay(void)
{
	int i;

	for (i = 0; i < 100; i++)
		cpu_relax();
}

static void jtag_tck_pulse(struct atri_priv *priv)
{
	jtag_tck_lo(priv);
	jtag_delay();
	jtag_tck_hi(priv);
	jtag_delay();
}

static void jtag_write(struct atri_priv *priv, u8 byte, int last)
{
	int bit;

	gpiod_set_value(priv->jtag_tms, 0);

	for (bit = 0; bit < 8; bit++) {
		if (bit == 7)
			gpiod_set_value(priv->jtag_tms, last);
		gpiod_set_value(priv->jtag_tdi, (byte >> bit) & 1);
		jtag_tck_pulse(priv);
	}
}

static void jtag_update_tms(struct atri_priv *priv, int tms)
{
	gpiod_set_value(priv->jtag_tms, tms);
	jtag_tck_pulse(priv);
}

static void jtag_tap_move(struct atri_priv *priv, u8 from, u8 to)
{
	int i;

	if (from == 1 && to == 11) {
		/* Run-Test/Idle -> Shift-IR */
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	} else if (from == 1 && to == 12) {
		/* Shift-IR -> Run-Test/Idle */
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	} else if (from == 1 && to == 4) {
		/* Run-Test/Idle -> Shift-DR */
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	} else if (from == 5 && to == 1) {
		/* Shift-DR -> Run-Test/Idle */
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	} else if (from == 1 && to == 1) {
		/* Idle clocks */
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	} else if (from == 16 && to == 1) {
		/* Test-Logic-Reset -> Run-Test/Idle */
		for (i = 0; i < 8; i++)
			jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	} else {
		dev_err(priv->dev, "unsupported JTAG state move %u->%u\n", from, to);
	}
}

static void jtag_write_inst(struct atri_priv *priv, u8 inst)
{
	jtag_tap_move(priv, 1, 11);
	jtag_write(priv, inst, 1);
	jtag_tap_move(priv, 1, 12);
	jtag_tap_move(priv, 1, 1);
	jtag_tap_move(priv, 1, 1);
	jtag_tap_move(priv, 1, 1);
}

static int jtag_read_code(struct atri_priv *priv, u8 inst, u32 *code)
{
	u32 val = 0;
	int i, bit;

	jtag_write_inst(priv, inst);
	jtag_tap_move(priv, 1, 4);

	gpiod_set_value(priv->jtag_tms, 0);
	jtag_tck_pulse(priv);

	for (i = 0; i < 32; i++) {
		if (i == 31)
			gpiod_set_value(priv->jtag_tms, 1);
		jtag_tck_lo(priv);
		jtag_delay();
		bit = gpiod_get_value(priv->jtag_tdo);
		val |= bit << i;
		jtag_tck_hi(priv);
		jtag_delay();
	}

	jtag_tap_move(priv, 5, 1);
	*code = val;
	return 0;
}

static int check_status_code(struct atri_priv *priv, u32 status)
{
	if (status & BIT(0))
		dev_err(priv->dev, "Status Code: CRC Error\n");
	if (status & BIT(1))
		dev_err(priv->dev, "Status Code: Bad Command\n");
	if (status & BIT(2))
		dev_err(priv->dev, "Status Code: ID Verify failed\n");
	if (status & BIT(3))
		dev_err(priv->dev, "Status Code: Timeout\n");
	if (status & BIT(12))
		dev_err(priv->dev, "Status Code: VLD bit isn't set\n");
	if (status & BIT(15))
		dev_err(priv->dev, "Status Code: READY bit isn't set\n");
	if (status & BIT(16))
		dev_err(priv->dev, "Status Code: POR bit isn't set\n");

	status = (status & ~BIT(17)) - 0x19000 - 0x20;
	return (status != 0) ? -1 : 0;
}

static int jtag_erase_flash(struct atri_priv *priv)
{
	u32 status;
	unsigned long flags;
	int ret, i;

	ret = jtag_read_code(priv, 0x41, &status);
	if (ret)
		return ret;

	if (check_status_code(priv, status) != 0) {
		dev_info(priv->dev, "Start erasing the FPGA SRAM\n");
		dev_info(priv->dev, "     status code = 0x%x\n", status);

		jtag_write_inst(priv, 0x11);
		jtag_write_inst(priv, 0x15);
		jtag_write_inst(priv, 0x05);
		for (i = 0; i < 4000; i++)
			jtag_update_tms(priv, 0);

		jtag_write_inst(priv, 0x09);
		jtag_write_inst(priv, 0x02);
		jtag_write_inst(priv, 0x3a);
		jtag_write_inst(priv, 0x02);
		for (i = 0; i < 1000; i++)
			jtag_update_tms(priv, 0);

		ret = jtag_read_code(priv, 0x41, &status);
		if (ret)
			return ret;
		if (check_status_code(priv, status) != 0) {
			dev_err(priv->dev, "Erasing the FPGA SRAM was failed\n");
			dev_err(priv->dev, "                status code = 0x%x\n", status);
			return -EIO;
		}
		dev_info(priv->dev, "Erasing the FPGA SRAM was successful\n");
	} else {
		dev_info(priv->dev, "FPGA SRAM doesn't need to be erased\n");
	}

	dev_info(priv->dev, "Start erasing the FPGA FLASH\n");

	local_irq_save(flags);

	jtag_write_inst(priv, 0x11);
	jtag_write_inst(priv, 0x15);
	jtag_write_inst(priv, 0x75);

	for (i = 0; i < 6; i++)
		jtag_update_tms(priv, 0);

	jtag_tap_move(priv, 1, 4);

	for (i = 0; i < 6; i++)
		jtag_update_tms(priv, 0);

	jtag_update_tms(priv, 0);
	for (i = 0; i < 16; i++) {
		jtag_update_tms(priv, 1);
		jtag_update_tms(priv, 0);
		jtag_update_tms(priv, 0);
	}
	jtag_update_tms(priv, 1);
	jtag_update_tms(priv, 0);
	jtag_update_tms(priv, 1);
	jtag_update_tms(priv, 1);
	jtag_update_tms(priv, 0);
	jtag_update_tms(priv, 0);
	jtag_update_tms(priv, 0);
	jtag_update_tms(priv, 0);

	for (i = 0x400e2; i != 0; i--) {
		jtag_tck_pulse(priv);
		jtag_tck_pulse(priv);
	}

	jtag_write_inst(priv, 0x3a);
	jtag_write_inst(priv, 0x02);

	local_irq_restore(flags);

	msleep(200);

	jtag_write_inst(priv, 0x11);
	jtag_write_inst(priv, 0x3c);
	jtag_write_inst(priv, 0x02);

	msleep(200);

	ret = jtag_read_code(priv, 0x41, &status);
	if (ret)
		return ret;
	if (check_status_code(priv, status) == 0) {
		dev_info(priv->dev, "Erasing the FPGA FLASH was successful\n");
		return 0;
	}
	dev_err(priv->dev, "Erasing the FPGA FLASH was failed\n");
	return -EIO;
}

static int jtag_prog_fpga(struct atri_priv *priv, const struct firmware *fw,
			  u16 expected_user_code)
{
	size_t prog_size = fw->size;
	u8 *buf;
	u32 id, user;
	size_t blocks, block;
	unsigned long flags;
	int ret, i;

	dev_info(priv->dev, "Start FPGA programming\n");

	if (fw->data[0] != 0)
		prog_size = fw->size + 0x100 - fw->data[0];
	dev_info(priv->dev, "fw size = %zu\n", prog_size);

	ret = jtag_read_code(priv, 0x11, &id);
	if (ret)
		return ret;
	dev_info(priv->dev, "FPGA ID = 0x%08x\n", id);

	buf = kmalloc(prog_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0xff, prog_size);
	memcpy(buf, fw->data, fw->size);
	buf[0] = 0x47;
	buf[1] = 0x57;
	buf[2] = 0x31;
	buf[3] = 0x4e;

	jtag_write_inst(priv, 0x15);
	for (i = 0; i < 4000; i++)
		jtag_update_tms(priv, 0);

	blocks = prog_size >> 8;

	for (block = 0; block < blocks; block++) {
		u8 *block_ptr = &buf[block * 256];

		if (block != 0) {
			for (i = 0; i < 16; i++)
				jtag_tck_pulse(priv);
		}

		local_irq_save(flags);

		jtag_write_inst(priv, 0x15);
		jtag_write_inst(priv, 0x71);

		jtag_tap_move(priv, 1, 4);
		gpiod_set_value(priv->jtag_tdi, 0);
		for (i = 0; i < 6; i++)
			jtag_tck_pulse(priv);

		for (i = 0; i < 26; i++) {
			gpiod_set_value(priv->jtag_tdi, (block >> i) & 1);
			if (i == 25)
				gpiod_set_value(priv->jtag_tms, 1);
			jtag_tck_pulse(priv);
			jtag_tck_pulse(priv);
		}
		jtag_tap_move(priv, 5, 1);

		for (i = 0; i < 360; i++) {
			jtag_tck_pulse(priv);
			jtag_tck_pulse(priv);
		}

		for (i = 0; i < 64; i++) {
			u8 *p = block_ptr + i * 4;
			int b;

			jtag_tap_move(priv, 1, 4);
			for (b = 0; b < 4; b++) {
				int last = (b == 3);

				jtag_write(priv, p[3 - b], last);
			}
			jtag_tap_move(priv, 5, 1);
			jtag_tap_move(priv, 1, 1);
			jtag_tap_move(priv, 1, 1);
			jtag_tap_move(priv, 1, 1);
			jtag_tap_move(priv, 1, 1);

			for (b = 0; b < 24; b++) {
				jtag_tck_pulse(priv);
				jtag_tck_pulse(priv);
			}
		}

		for (i = 0; i < 14; i++) {
			jtag_tck_pulse(priv);
			jtag_tck_pulse(priv);
		}

		local_irq_restore(flags);
	}

	jtag_write_inst(priv, 0x3a);
	jtag_write_inst(priv, 0x02);
	for (i = 0; i < 300; i++)
		jtag_update_tms(priv, 0);

	jtag_write_inst(priv, 0x11);
	jtag_write_inst(priv, 0x3c);
	jtag_write_inst(priv, 0x02);
	msleep(300);

	kfree(buf);

	ret = jtag_read_code(priv, 0x13, &user);
	if (ret)
		return ret;
	dev_info(priv->dev, "user_code: expected = %u; actual = %u\n",
		 expected_user_code, user);
	if (user != expected_user_code) {
		dev_err(priv->dev, "Failed to programming FPGA\n");
		return -EIO;
	}
	dev_info(priv->dev, "FPGA has been successfully programmed\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Firmware update                                                     */
/* ------------------------------------------------------------------ */

struct alp_fw_meta {
	u8	fw_version;
	u16	binary_crc16;
	u16	user_flash_crc16;
	u16	expected_user_code;
	u8	force_upd;
};

static void fpga_hw_reset(struct atri_priv *priv)
{
	if (priv->reset_gpio) {
		gpiod_set_value(priv->reset_gpio, 0);
		msleep(10);
		gpiod_set_value(priv->reset_gpio, 1);
		msleep(100);
	}
}

static int select_jtag_mode(struct atri_priv *priv)
{
	int ret;

	if (priv->pinctrl_jtag) {
		ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_jtag);
		if (ret)
			return ret;
	}

	if (priv->jtag_sel)
		gpiod_set_value(priv->jtag_sel, 0);

	priv->jtag_mode = true;
	return 0;
}

static int select_spi_mode(struct atri_priv *priv)
{
	int ret = 0;

	if (priv->jtag_sel)
		gpiod_set_value(priv->jtag_sel, 1);

	if (priv->pinctrl_default)
		ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_default);

	priv->jtag_mode = false;
	return ret;
}

static int alp_update_firmware(struct atri_priv *priv,
			      const struct firmware *fw,
			      u16 expected_user_code)
{
	int ret, erase_attempt, prog_attempt, reset_attempt;

	for (reset_attempt = 0; reset_attempt < 2; reset_attempt++) {
		for (erase_attempt = 1; erase_attempt <= 4; erase_attempt++) {
			ret = jtag_erase_flash(priv);
			if (ret == 0)
				goto erased;
			msleep(200);
		}
		dev_info(priv->dev, "erase flash attempts = %d\n", erase_attempt);
		fpga_hw_reset(priv);
	}
	dev_info(priv->dev, "FPGA reset attempts = %d\n", reset_attempt);
	return ret;

erased:
	dev_info(priv->dev, "erase flash attempts = %d\n", erase_attempt);

	for (prog_attempt = 1; prog_attempt <= 4; prog_attempt++) {
		ret = jtag_prog_fpga(priv, fw, expected_user_code);
		if (ret == 0)
			return 0;
		msleep(200);
	}
	dev_info(priv->dev, "FPGA program attempts = %d\n", prog_attempt);
	return ret;
}

static void alp_parse_manifest(const char *data, struct alp_fw_meta *meta)
{
	const char *p, *eq;

	memset(meta, 0, sizeof(*meta));

	for (p = data; p && *p; p = strchr(p, '\n')) {
		if (*p == '\n')
			p++;
		eq = strchr(p, '=');
		if (!eq)
			continue;
		while (*eq == ' ' || *eq == '=')
			eq++;
		while (*eq == ' ')
			eq++;

		if (!strncmp(p, "fw_version", 10))
			meta->fw_version = (u8)simple_strtoul(eq, NULL, 0);
		else if (!strncmp(p, "crc16", 5))
			meta->binary_crc16 = (u16)simple_strtoul(eq, NULL, 0);
		else if (!strncmp(p, "user_code", 9))
			meta->user_flash_crc16 = (u16)simple_strtoul(eq, NULL, 0);
		else if (!strncmp(p, "uf_crc16", 8))
			meta->expected_user_code = (u16)simple_strtoul(eq, NULL, 0);
		else if (!strncmp(p, "force_upd", 9))
			meta->force_upd = (u8)simple_strtoul(eq, NULL, 0);
	}
}

static int alp_handle_firmware_update(struct atri_priv *priv)
{
	const char *manifest_name, *bin_name;
	const struct firmware *manifest, *fw;
	struct alp_fw_meta meta = {};
	u16 actual_crc;
	int ret;

	if (of_device_is_compatible(priv->dev->of_node, "atri,led-panel") ||
	    of_device_is_compatible(priv->dev->of_node, "ya,led-panel")) {
		manifest_name = "yandex_led_panel.bin.manifest";
		bin_name = "yandex_led_panel.bin";
	} else {
		manifest_name = "yandex_led_screen_fpga.bin.manifest";
		bin_name = "yandex_led_screen_fpga.bin";
	}

	ret = request_firmware(&manifest, manifest_name, priv->dev);
	if (ret) {
		/* Fallback for panels that ship with the screen FPGA firmware */
		if (of_device_is_compatible(priv->dev->of_node, "atri,led-panel") ||
		    of_device_is_compatible(priv->dev->of_node, "ya,led-panel")) {
			manifest_name = "yandex_led_screen_fpga.bin.manifest";
			bin_name = "yandex_led_screen_fpga.bin";
			ret = request_firmware(&manifest, manifest_name, priv->dev);
		}
		if (ret) {
			snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
				 "manifest not found: %s", manifest_name);
			return 0;
		}
	}

	alp_parse_manifest(manifest->data, &meta);
	release_firmware(manifest);

	if (!meta.binary_crc16 && !meta.fw_version) {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "manifest parse failed: %s", manifest_name);
		return 0;
	}

	ret = request_firmware(&fw, bin_name, priv->dev);
	if (ret) {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "fw not found: %s", bin_name);
		return 0;
	}

	actual_crc = alp_crc_itu_t(0, fw->data, fw->size) & 0xffff;
	if (actual_crc != meta.binary_crc16) {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "crc16 mismatch: expected %u, actual %u",
			 meta.binary_crc16, actual_crc);
		release_firmware(fw);
		return 0;
	}

	snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
		 "flashing %s (%zu bytes, crc ok)", bin_name, fw->size);

	ret = select_jtag_mode(priv);
	if (ret) {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "jtag mode failed: %d", ret);
		release_firmware(fw);
		return ret;
	}

	ret = alp_update_firmware(priv, fw, meta.expected_user_code);
	release_firmware(fw);
	select_spi_mode(priv);

	if (ret) {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "flash failed: %d", ret);
	} else {
		snprintf(priv->fw_upd_status, sizeof(priv->fw_upd_status),
			 "ok: %s programmed", bin_name);
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
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", priv->fb_open_count);
}

static ssize_t fw_upd_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", priv->fw_upd_status);
}

static ssize_t crc_errors_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "master=%u slave=%u\n",
			  priv->crc_errors_master, priv->crc_errors_slave);
}

static ssize_t power_saving_mode_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", priv->power_saving ? 1 : 0);
}

static ssize_t power_saving_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
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
	struct atri_priv *priv = dev_get_drvdata(dev);
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
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%s\n", priv->jtag_mode ? "jtag" : "spi");
}

static ssize_t gpio_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
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
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->frame_queue_overflows);
}

static ssize_t next_frame_delay_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->next_frame_delay);
}

static ssize_t next_frame_delay_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
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
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "touch: state=%d mask=%d fqc=%u\n",
			  0, 0, priv->frame_queue_overflows);
}

static ssize_t frames_in_queue_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", priv->frames_in_queue);
}

static ssize_t press_key_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	unsigned int key;
	int ret;

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

static struct attribute *alp_common_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_open_fb_count.attr,
	&dev_attr_fw_upd_status.attr,
	&dev_attr_crc_errors.attr,
	&dev_attr_power_saving_mode.attr,
	&dev_attr_set_animation_mode.attr,
	&dev_attr_gpio_mode.attr,
	&dev_attr_press_key.attr,
	NULL,
};

static const struct attribute_group alp_common_group = {
	.attrs = alp_common_attrs,
};

static struct attribute *alp_frame_queue_attrs[] = {
	&dev_attr_frame_queue_overflows.attr,
	&dev_attr_next_frame_delay.attr,
	&dev_attr_frames_in_queue.attr,
	NULL,
};

static const struct attribute_group alp_frame_queue_group = {
	.name = "frame_queue",
	.attrs = alp_frame_queue_attrs,
};

static struct attribute *alp_midi_touch_attrs[] = {
	&dev_attr_show_debug_info.attr,
	NULL,
};

static const struct attribute_group alp_midi_touch_group = {
	.name = "midi_touch",
	.attrs = alp_midi_touch_attrs,
};

/* JTAG group */
static ssize_t jtag_reconfig_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
	int val = priv->jtag_reconfig ? gpiod_get_value(priv->jtag_reconfig) : 0;
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t jtag_reconfig_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct atri_priv *priv = dev_get_drvdata(dev);
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
	return sysfs_emit(buf, "0\n");
}

static ssize_t jtag_codes_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	u32 idcode = 0, usercode = 0;

	return sysfs_emit(buf, "ID=0x%08x USER=0x%08x STATUS=0\n",
			  idcode, usercode);
}

static ssize_t jtag_erase_flash_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0\n");
}

static ssize_t test_fpga_prog_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
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

static const struct attribute_group *alp_attr_groups[] = {
	&alp_common_group,
	&alp_frame_queue_group,
	&alp_midi_touch_group,
	&jtag_group,
	NULL,
};

/* ------------------------------------------------------------------ */
/* SPI driver probe/remove                                            */
/* ------------------------------------------------------------------ */

static const struct spi_device_id atri_led_panel_device_id[] = {
	{ "atri_led_panel", (kernel_ulong_t)&rev2res_panel },
	{ "led_screen", (kernel_ulong_t)&rev2res_screen },
	{ "ya,led-panel", (kernel_ulong_t)&rev2res_panel },
	{ "ya,led_screen", (kernel_ulong_t)&rev2res_screen },
	{ }
};

MODULE_DEVICE_TABLE(spi, atri_led_panel_device_id);

static const struct of_device_id atri_led_panel_device_of_match[] = {
	{ .compatible = "atri,led-panel", .data = &rev2res_panel },
	{ .compatible = "atri,led-screen", .data = &rev2res_screen },
	{ .compatible = "ya,led-panel", .data = &rev2res_panel },
	{ .compatible = "ya,led_screen", .data = &rev2res_screen },
	{ }
};

MODULE_DEVICE_TABLE(of, atri_led_panel_device_of_match);

static int probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct of_device_id *match;
	const struct atri_resolution *res;
	struct atri_priv *priv;
	int ret;

	match = of_match_node(atri_led_panel_device_of_match, dev->of_node);
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

	/* GPIOs */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	priv->irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
	if (IS_ERR(priv->irq_gpio))
		return PTR_ERR(priv->irq_gpio);

	priv->jtag_tck = devm_gpiod_get_optional(dev, "jtag_tck", GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_tck))
		return PTR_ERR(priv->jtag_tck);

	priv->jtag_tdo = devm_gpiod_get_optional(dev, "jtag_tdo", GPIOD_IN);
	if (IS_ERR(priv->jtag_tdo))
		return PTR_ERR(priv->jtag_tdo);

	priv->jtag_tdi = devm_gpiod_get_optional(dev, "jtag_tdi", GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_tdi))
		return PTR_ERR(priv->jtag_tdi);

	priv->jtag_tms = devm_gpiod_get_optional(dev, "jtag_tms", GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_tms))
		return PTR_ERR(priv->jtag_tms);

	priv->jtag_sel = devm_gpiod_get_optional(dev, "jtag_sel", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->jtag_sel))
		return PTR_ERR(priv->jtag_sel);

	priv->jtag_reconfig = devm_gpiod_get_optional(dev, "jtag_reconfig",
						       GPIOD_OUT_LOW);
	if (IS_ERR(priv->jtag_reconfig))
		return PTR_ERR(priv->jtag_reconfig);

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
	ret = gpio_init(priv);
	if (ret)
		return ret;

	ret = fb_init(priv);
	if (ret)
		goto err_gpio;

	ret = backlight_init(priv);
	if (ret)
		goto err_fb;

	ret = uart_init(priv);
	if (ret)
		goto err_bl;

	ret = init_touch(priv);
	if (ret)
		goto err_uart;

	ret = sysfs_create_groups(&dev->kobj, alp_attr_groups);
	if (ret)
		goto err_sysfs;

	/* Firmware update */
	alp_handle_firmware_update(priv);

	dev_info(dev, "led panel probed: %dx%d\n", priv->width, priv->height);
	return 0;

err_sysfs:
	sysfs_remove_groups(&dev->kobj, alp_attr_groups);
	deinit_touch(priv);
err_uart:
	uart_exit(priv);
err_bl:
	backlight_exit(priv);
err_fb:
	fb_exit(priv);
err_gpio:
	gpio_exit(priv);
	return ret;
}

static void remove(struct spi_device *spi)
{
	struct atri_priv *priv = spi_get_drvdata(spi);

	sysfs_remove_groups(&priv->dev->kobj, alp_attr_groups);
	deinit_touch(priv);
	uart_exit(priv);
	backlight_exit(priv);
	fb_exit(priv);
	gpio_exit(priv);

	dev_info(priv->dev, "led panel removed\n");
}

static struct spi_driver atri_led_panel_device_driver = {
	.driver = {
		.name = ATRI_DRV_NAME,
		.of_match_table = atri_led_panel_device_of_match,
	},
	.probe = probe,
	.remove = remove,
	.id_table = atri_led_panel_device_id,
};

module_spi_driver(atri_led_panel_device_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("AtriOS Team");
MODULE_DESCRIPTION("Quasar led panel (SPI)");
