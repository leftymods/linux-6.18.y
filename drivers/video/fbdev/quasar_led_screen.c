// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/crc-itu-t.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "quasar_led_screen"

/* SA command opcodes */
#define SA_CMD_NONE            0x16
#define SA_CMD_REBOOT          0x0B
#define SA_CMD_SHOW_PIC        0x07
#define SA_CMD_FLUSH_BUFFER    0x2D
#define SA_CMD_SET_BRIGHTNESS  0x1B
#define SA_CMD_SAVE_TEST       0x09
#define SA_CMD_SET_CRC_CHECK   0x36

/* Bootloader command opcodes */
#define BL_CMD_RESTART         0xBE
#define BL_CMD_RUN_APP         0x4B

/* Bootloader states */
#define BL_STATE_IDLE          0
#define BL_STATE_BUSY          3

/* Response magic */
#define SA_RESP_MAGIC          0xFE

/* SPI speed (from disasm: 0x0F4240 = 1MHz) */
#define SA_SPI_SPEED           1000000

/* Max firmware chunk size (from disasm: 0x8000 = 32KB) */
#define FW_CHUNK_SIZE          0x8000

/* Bootloader delay constant (from disasm: 0x418958 cycles) */
#define BL_DELAY_CYCLES        0x418958
#define BL_DELAY_ITERATIONS    50

/* Buffer sizes */
#define SA_CMD_BUF_SIZE        64
#define SA_RESP_BUF_SIZE       64
#define SA_INLINE_BUF_SIZE     20

struct sa_device {
	struct spi_device	*spi;
	int			crc_check;
	int			fw_version;
	int			api_version;
	u8			cmdbuf[SA_CMD_BUF_SIZE];
	u8			respbuf[SA_RESP_BUF_SIZE];
	struct mutex		lock;
};

struct qls_priv {
	struct spi_device	*spi;
	struct sa_device	*sa;
	struct fb_info		*fb;
	struct backlight_device	*bl;
	struct gpio_desc	*reset;

	u8			*videomemory;
	u32			xres;
	u32			yres;
	u32			pixel_depth;

	int			max_brightness;
	int			brightness;

	/* Firmware info */
	u16			fw_version;
	u8			min_api_version;
	u8			max_api_version;
	u8			fw_crc;
	u16			expected_crc;

	/* Sysfs/debug state */
	u32			reset_count;
	int			frames_in_queue;
	int			frame_queue_overflows;
	int			next_frame_delay;
	int			update_result;
	int			disable_test_mode;
	u8			screen_fw_after_start;
};

/* CRC16-ITU with byte swap (as seen in original: rev16 after crc_itu_t) */
static u16 qls_crc16(const u8 *data, int len)
{
	return swab16(crc_itu_t(0, data, len));
}

/* ---- SA protocol helpers ---- */

static int sa_send_command(struct sa_device *sa, u8 cmd,
			   const u8 *data, int data_len,
			   u8 *resp, int resp_len)
{
	u8 buf[SA_CMD_BUF_SIZE];
	u16 crc;
	int ret, txlen;

	buf[0] = cmd;
	if (data && data_len > 0 && data_len <= SA_CMD_BUF_SIZE - 3)
		memcpy(buf + 1, data, data_len);

	txlen = 1 + data_len;
	crc = qls_crc16(buf, txlen);
	buf[txlen] = crc & 0xff;
	buf[txlen + 1] = (crc >> 8) & 0xff;
	txlen += 2;

	ret = spi_write_then_read(sa->spi, buf, txlen, resp, resp_len);
	return ret;
}

static int sa_send_cmd_short(struct sa_device *sa, u8 cmd,
			     u8 param, int param_len)
{
	u8 buf[SA_CMD_BUF_SIZE];
	u16 crc;
	int txlen;

	buf[0] = cmd;
	if (param_len > 0 && param_len <= SA_CMD_BUF_SIZE - 3) {
		buf[1] = param;
		txlen = 1 + param_len;
	} else {
		txlen = 1;
	}

	crc = qls_crc16(buf, txlen);
	buf[txlen] = crc & 0xff;
	buf[txlen + 1] = (crc >> 8) & 0xff;
	txlen += 2;

	return spi_write(sa->spi, buf, txlen);
}

static int sa_none(struct sa_device *sa, u8 *resp, int resp_len)
{
	return sa_send_command(sa, SA_CMD_NONE, NULL, 0, resp, resp_len);
}

static int sa_reboot(struct sa_device *sa)
{
	return sa_send_cmd_short(sa, SA_CMD_REBOOT, 0, 0);
}

static int sa_set_brightness(struct sa_device *sa, u8 brightness)
{
	return sa_send_cmd_short(sa, SA_CMD_SET_BRIGHTNESS, brightness, 1);
}

static int sa_set_crc_check(struct sa_device *sa, u8 enable)
{
	return sa_send_cmd_short(sa, SA_CMD_SET_CRC_CHECK, enable, 1);
}

static int sa_flush_buffer(struct sa_device *sa)
{
	return sa_send_cmd_short(sa, SA_CMD_FLUSH_BUFFER, 0, 0);
}

static int sa_show_pic(struct sa_device *sa, const u8 *data, int len)
{
	u8 buf[SA_CMD_BUF_SIZE];
	u16 crc;
	int txlen;

	if (len + 3 > SA_CMD_BUF_SIZE)
		return -EINVAL;

	buf[0] = SA_CMD_SHOW_PIC;
	memcpy(buf + 1, data, len);
	txlen = 1 + len;

	crc = qls_crc16(buf, txlen);
	buf[txlen] = crc & 0xff;
	buf[txlen + 1] = (crc >> 8) & 0xff;
	txlen += 2;

	return spi_write(sa->spi, buf, txlen);
}

/* ---- Bootloader protocol ---- */

static int bl_send_cmd(struct spi_device *spi, u8 cmd)
{
	u8 buf[3];
	u16 crc;

	buf[0] = cmd;
	crc = qls_crc16(buf, 1);
	buf[1] = crc & 0xff;
	buf[2] = (crc >> 8) & 0xff;

	return spi_write(spi, buf, 3);
}

struct bl_info {
	u8  magic;
	u8  fw_version;
	u32 state;
	u32 info_lo;
	u64 info;
	u16 crc;
};

static int bl_get_info(struct spi_device *spi, struct bl_info *info)
{
	u8 buf[28] = {0};
	u16 crc;
	int ret;

	ret = spi_write_then_read(spi, buf, 1, buf, sizeof(buf));
	if (ret)
		return ret;

	if (buf[0] != SA_RESP_MAGIC)
		return -EIO;

	crc = qls_crc16(buf, 12);
	if ((buf[12] | (buf[13] << 8)) != crc)
		return -EILSEQ;

	info->magic = buf[0];
	info->fw_version = buf[1];
	info->state = buf[2] | (buf[3] << 8);
	info->info_lo = buf[4] | (buf[5] << 8) | (buf[6] << 16) | (buf[7] << 24);
	info->info = info->info_lo;
	info->crc = buf[12] | (buf[13] << 8);

	return 0;
}

static int bl_restart(struct spi_device *spi)
{
	struct bl_info info;
	int ret;

	ret = bl_get_info(spi, &info);
	if (ret)
		return ret;
	if (info.state == BL_STATE_BUSY)
		return -EBUSY;

	return bl_send_cmd(spi, BL_CMD_RESTART);
}

static int bl_run_app(struct spi_device *spi)
{
	return bl_send_cmd(spi, BL_CMD_RUN_APP);
}

static int bl_write_fw(struct spi_device *spi, const u8 *data, int len)
{
	int ret, offset = 0;
	struct bl_info info;

	while (offset < len) {
		int chunk = min(len - offset, FW_CHUNK_SIZE);
		int retries = BL_DELAY_ITERATIONS;
		u8 *buf;
		int bl_state;

		buf = kmalloc(chunk + 4, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		buf[0] = 0;
		buf[1] = 0;
		memcpy(buf + 4, data + offset, chunk);

		ret = spi_write(spi, buf, chunk + 4);
		kfree(buf);
		if (ret)
			return ret;

		do {
			udelay(BL_DELAY_CYCLES);
			ret = bl_get_info(spi, &info);
			if (ret)
				return ret;
			bl_state = info.state;
		} while (bl_state == BL_STATE_BUSY && --retries > 0);

		if (bl_state != BL_STATE_IDLE)
			return -ETIMEDOUT;

		offset += chunk;
	}
	return 0;
}

/* ---- Framebuffer operations ---- */

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
	if (var->xres != 25 || var->yres != 16 ||
	    var->xres_virtual != 25 || var->yres_virtual != 16)
		return -EINVAL;
	if (var->bits_per_pixel != 8)
		return -EINVAL;

	var->red.offset   = 0;
	var->red.length   = 8;
	var->green.offset = 0;
	var->green.length = 8;
	var->blue.offset  = 0;
	var->blue.length  = 8;
	var->transp.offset = 0;
	var->transp.length = 0;
	var->grayscale = 0;
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
	struct qls_priv *qls = info->par;

	qls->xres = info->var.xres;
	qls->yres = info->var.yres;
	info->fix.line_length = qls->xres * 3;
	return 0;
}

static int qls_fb_blank(int blank, struct fb_info *info)
{
	struct qls_priv *qls = info->par;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		sa_set_brightness(qls->sa, qls->brightness);
		break;
	case FB_BLANK_POWERDOWN:
		sa_set_brightness(qls->sa, 0);
		break;
	}
	return 0;
}

static int qls_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	return 0;
}

static int qls_fb_sync(struct fb_info *info)
{
	struct qls_priv *qls = info->par;
	int i, ret;

	mutex_lock(&qls->sa->lock);
	for (i = 0; i < qls->frames_in_queue; i++) {
		ret = sa_flush_buffer(qls->sa);
		if (ret)
			break;
	}
	qls->frames_in_queue = 0;
	mutex_unlock(&qls->sa->lock);
	return ret;
}

static void qls_fb_destroy(struct fb_info *info)
{
	struct qls_priv *qls = info->par;

	if (qls->bl)
		backlight_device_unregister(qls->bl);
	unregister_framebuffer(info);
	vfree(qls->videomemory);
}

static int qls_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	return remap_vmalloc_range(vma, info->screen_buffer, vma->vm_pgoff);
}

static void qls_draw(struct qls_priv *qls)
{
	int y, ret;
	u8 *line;
	int bpp = qls->pixel_depth > 0 ? qls->pixel_depth : 3;
	int line_len = qls->xres * bpp;

	line = kmalloc(line_len + 4, GFP_KERNEL);
	if (!line)
		return;

	for (y = 0; y < qls->yres; y++) {
		u8 *src = qls->videomemory + y * line_len;
		memcpy(line, src, line_len);
		mutex_lock(&qls->sa->lock);
		ret = sa_show_pic(qls->sa, line, line_len);
		if (ret == 0)
			ret = sa_flush_buffer(qls->sa);
		mutex_unlock(&qls->sa->lock);
		if (ret)
			break;
	}
	kfree(line);
}

static void qls_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	cfb_fillrect(info, rect);
	qls_draw(info->par);
}

static void qls_fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	cfb_copyarea(info, area);
	qls_draw(info->par);
}

static void qls_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	cfb_imageblit(info, image);
	qls_draw(info->par);
}

static const struct fb_ops qls_fb_ops = {
	.owner        = THIS_MODULE,
	.fb_open      = qls_fb_open,
	.fb_release   = qls_fb_release,
	.fb_check_var = qls_fb_check_var,
	.fb_set_par   = qls_fb_set_par,
	.fb_blank     = qls_fb_blank,
	.fb_pan_display = qls_fb_pan_display,
	.fb_fillrect  = qls_fb_fillrect,
	.fb_copyarea  = qls_fb_copyarea,
	.fb_imageblit = qls_fb_imageblit,
	.fb_sync      = qls_fb_sync,
	.fb_destroy   = qls_fb_destroy,
	.fb_mmap      = qls_fb_mmap,
};

/* ---- Backlight ---- */

static int qls_bl_update_status(struct backlight_device *bd)
{
	struct qls_priv *qls = bl_get_data(bd);

	qls->brightness = clamp(bd->props.brightness, 0, qls->max_brightness);
	sa_set_brightness(qls->sa, qls->brightness);
	return 0;
}

static int qls_bl_get_brightness(struct backlight_device *bd)
{
	return bl_get_data(bd)->brightness;
}

static const struct backlight_ops qls_bl_ops = {
	.update_status  = qls_bl_update_status,
	.get_brightness = qls_bl_get_brightness,
};

/* ---- Hwmon ---- */

static int qls_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long *val)
{
	*val = 25000;
	return 0;
}

static umode_t qls_hwmon_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	return 0444;
}

static const struct hwmon_channel_info *qls_hwmon_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL,
};

static const struct hwmon_ops qls_hwmon_ops = {
	.is_visible = qls_hwmon_is_visible,
	.read = qls_hwmon_read,
};

static const struct hwmon_chip_info qls_hwmon_chip_info = {
	.ops = &qls_hwmon_ops,
	.info = qls_hwmon_info,
};

/* ---- Firmware update ---- */

static int qls_load_firmware_update(struct qls_priv *qls)
{
	const struct firmware *manifest_fw, *fw;
	char manifest_name[] = "yandex-led-screen.bin.manifest";
	char fw_name[] = "yandex-led-screen.bin";
	int ret, parsed;
	u8 min_api, max_api, version, _crc;
	u16 crc16;

	ret = request_firmware(&manifest_fw, manifest_name, &qls->spi->dev);
	if (ret) {
		dev_warn(&qls->spi->dev,
			 "no firmware manifest, skipping update\n");
		return 0;
	}

	parsed = sscanf(manifest_fw->data,
			"%hhu.%hhu.%hhu.%hhu.%hu.%hhu",
			&qls->fw_version,
			&min_api, &max_api,
			&version, &crc16, &qls->fw_crc);
	release_firmware(manifest_fw);

	if (parsed < 3) {
		dev_err(&qls->spi->dev,
			"failed to parse firmware manifest\n");
		return -EINVAL;
	}

	ret = request_firmware(&fw, fw_name, &qls->spi->dev);
	if (ret) {
		dev_err(&qls->spi->dev,
			"failed to load firmware update\n");
		return ret;
	}

	if (crc16 != qls_crc16(fw->data, fw->size)) {
		dev_err(&qls->spi->dev,
			"firmware CRC mismatch\n");
		release_firmware(fw);
		return -EILSEQ;
	}

	dev_info(&qls->spi->dev,
		 "firmware update: version %hhu, size %zu\n",
		 version, fw->size);

	ret = bl_restart(qls->spi);
	if (ret) {
		dev_err(&qls->spi->dev,
			"failed to reboot into bootloader: %d\n", ret);
		release_firmware(fw);
		return ret;
	}

	msleep(100);

	ret = bl_write_fw(qls->spi, fw->data, fw->size);
	if (ret) {
		dev_err(&qls->spi->dev,
			"failed to write firmware: %d\n", ret);
		release_firmware(fw);
		return ret;
	}

	ret = bl_run_app(qls->spi);
	if (ret) {
		dev_err(&qls->spi->dev,
			"failed to run new firmware: %d\n", ret);
		release_firmware(fw);
		return ret;
	}

	release_firmware(fw);
	dev_info(&qls->spi->dev, "firmware update completed\n");
	return 0;
}

/* ---- Sysfs attributes ---- */

static ssize_t next_frame_delay_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", qls->next_frame_delay);
}

static ssize_t next_frame_delay_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct qls_priv *qls = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	qls->next_frame_delay = val;
	return count;
}

static ssize_t screen_fw_after_start_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", qls->screen_fw_after_start);
}

static ssize_t screen_reset_count_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", qls->reset_count);
}

static ssize_t update_result_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", qls->update_result);
}

static ssize_t update_info_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "fw:%u;api:%u;min_api:%hhu;max_api:%hhu;crc:%u\n",
		       qls->fw_version, qls->sa->api_version,
		       qls->min_api_version, qls->max_api_version,
		       qls->fw_crc);
}

static ssize_t frames_in_frame_queue_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", qls->frames_in_queue);
}

static ssize_t frame_queue_overflows_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", qls->frame_queue_overflows);
}

static ssize_t screen_fw_current_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", qls->fw_version);
}

static ssize_t disable_test_mode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct qls_priv *qls = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	qls->disable_test_mode = val;
	return count;
}

static ssize_t flush_frame_queue_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct qls_priv *qls = dev_get_drvdata(dev);

	mutex_lock(&qls->sa->lock);
	sa_flush_buffer(qls->sa);
	qls->frames_in_queue = 0;
	mutex_unlock(&qls->sa->lock);
	return count;
}

static DEVICE_ATTR_RW(next_frame_delay);
static DEVICE_ATTR_RO(screen_fw_after_start);
static DEVICE_ATTR_RO(screen_reset_count);
static DEVICE_ATTR_RO(update_result);
static DEVICE_ATTR_RO(update_info);
static DEVICE_ATTR_RO(frames_in_frame_queue);
static DEVICE_ATTR_RO(frame_queue_overflows);
static DEVICE_ATTR_RO(screen_fw_current);
static DEVICE_ATTR_WO(disable_test_mode);
static DEVICE_ATTR_WO(flush_frame_queue);

static struct attribute *qls_attrs[] = {
	&dev_attr_next_frame_delay.attr,
	&dev_attr_screen_fw_after_start.attr,
	&dev_attr_screen_reset_count.attr,
	&dev_attr_update_result.attr,
	&dev_attr_update_info.attr,
	&dev_attr_frames_in_frame_queue.attr,
	&dev_attr_frame_queue_overflows.attr,
	&dev_attr_screen_fw_current.attr,
	&dev_attr_disable_test_mode.attr,
	&dev_attr_flush_frame_queue.attr,
	NULL,
};

static const struct attribute_group qls_attr_group = {
	.attrs = qls_attrs,
};

/* ---- Probe / Remove ---- */

static int qls_probe(struct spi_device *spi)
{
	struct qls_priv *qls;
	struct sa_device *sa;
	struct fb_info *info;
	struct backlight_device *bl;
	struct backlight_properties bl_props;
	struct device *hwmon_dev;
	int ret;

	qls = devm_kzalloc(&spi->dev, sizeof(*qls), GFP_KERNEL);
	if (!qls)
		return -ENOMEM;

	sa = devm_kzalloc(&spi->dev, sizeof(*sa), GFP_KERNEL);
	if (!sa)
		return -ENOMEM;

	sa->spi = spi;
	mutex_init(&sa->lock);
	qls->spi = spi;
	qls->sa = sa;
	qls->xres = 25;
	qls->yres = 16;
	qls->pixel_depth = 3;
	qls->max_brightness = 200;
	qls->brightness = 200;

	spi_set_drvdata(spi, qls);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = SA_SPI_SPEED;
	spi_setup(spi);

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

	/* DT max-brightness */
	of_property_read_u32(spi->dev.of_node, "max-brightness",
			     &qls->max_brightness);
	if (qls->max_brightness > 255)
		qls->max_brightness = 255;

	/* Check if SA is alive */
	ret = sa_none(sa, sa->respbuf, SA_RESP_BUF_SIZE);
	if (ret) {
		dev_warn(&spi->dev, "SA not responding: %d, trying fw update\n",
			 ret);
		qls_load_firmware_update(qls);
		ret = sa_none(sa, sa->respbuf, SA_RESP_BUF_SIZE);
		if (ret) {
			dev_err(&spi->dev,
				"SA still not responding after update\n");
			return ret;
		}
	}

	/* Set CRC check mode */
	sa_set_crc_check(sa, 1);

	/* Allocate framebuffer (8bpp, 3 bytes per pixel RGB) */
	qls->videomemory = vzalloc(qls->xres * qls->yres * 3);
	if (!qls->videomemory)
		return -ENOMEM;

	info = framebuffer_alloc(0, &spi->dev);
	if (!info) {
		vfree(qls->videomemory);
		return -ENOMEM;
	}

	qls->fb = info;
	info->par = qls;
	info->screen_buffer = (char __iomem *)qls->videomemory;
	info->screen_size = qls->xres * qls->yres * 3;
	info->fbops = &qls_fb_ops;
	info->flags = 0;

	info->var.xres = qls->xres;
	info->var.yres = qls->yres;
	info->var.xres_virtual = qls->xres;
	info->var.yres_virtual = qls->yres;
	info->var.bits_per_pixel = 8;
	info->var.red.offset = 0;
	info->var.red.length = 8;
	info->var.green.offset = 0;
	info->var.green.length = 8;
	info->var.blue.offset = 0;
	info->var.blue.length = 8;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->fix.smem_start = 0;
	info->fix.smem_len = qls->xres * qls->yres * 3;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = qls->xres * 3;
	info->fix.accel = FB_ACCEL_NONE;
	snprintf(info->fix.id, sizeof(info->fix.id), "quasar-led");

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to register framebuffer: %d\n", ret);
		framebuffer_release(info);
		vfree(qls->videomemory);
		return ret;
	}

	/* Backlight */
	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.max_brightness = qls->max_brightness;
	bl_props.brightness = qls->brightness;

	bl = devm_backlight_device_register(&spi->dev, "quasar_led_screen",
					    &spi->dev, qls,
					    &qls_bl_ops, &bl_props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	qls->bl = bl;

	/* Hwmon */
	hwmon_dev = devm_hwmon_device_register_with_info(&spi->dev,
							 "quasar_led_screen",
							 qls,
							 &qls_hwmon_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		dev_warn(&spi->dev, "hwmon registration failed\n");

	/* Sysfs */
	ret = sysfs_create_group(&spi->dev.kobj, &qls_attr_group);
	if (ret)
		dev_warn(&spi->dev, "failed to create sysfs group: %d\n", ret);

	dev_info(&spi->dev, "registered framebuffer '%s', %ux%u 8bpp\n",
		 info->fix.id, qls->xres, qls->yres);

	return 0;
}

static void qls_remove(struct spi_device *spi)
{
	struct qls_priv *qls = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &qls_attr_group);

	if (qls->fb) {
		unregister_framebuffer(qls->fb);
		framebuffer_release(qls->fb);
	}
	vfree(qls->videomemory);
}

static const struct of_device_id qls_of_match[] = {
	{ .compatible = "ya,led_screen" },
	{},
};
MODULE_DEVICE_TABLE(of, qls_of_match);

static const struct spi_device_id qls_spi_ids[] = {
	{ "quasar_led_screen", 0 },
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

MODULE_DESCRIPTION("Quasar LED Screen FB driver (Gowin FPGA, SA protocol)");
MODULE_AUTHOR("leftymods");
MODULE_LICENSE("GPL v2");