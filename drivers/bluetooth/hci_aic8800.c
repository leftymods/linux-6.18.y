// SPDX-License-Identifier: GPL-2.0
/*
 * AIC8800 Bluetooth HCI UART (serdev) driver
 *
 * AIC8800 is a WiFi + BT combo chip. The Bluetooth firmware is loaded by the
 * WiFi driver; the Bluetooth side is a plain H4 HCI stream over UART.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/serdev.h>
#include <linux/skbuff.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/device.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

#if IS_ENABLED(CONFIG_AIC8800_SDIO_WLAN)
/* Provided by the aic8800 WiFi driver (aicbsp); true once the combo BT
 * firmware has been loaded over SDIO.
 */
extern bool aicbsp_is_bt_fw_ready(void);
#endif

#define AIC_DEFAULT_BAUDRATE	115200

struct aic_dev {
	struct hci_dev *hdev;
	struct serdev_device *serdev;

	struct gpio_desc *enable_gpio;		/* BT_REG_ON */
	struct gpio_desc *device_wakeup_gpio;	/* HOST_WAKE_BT */
	struct gpio_desc *host_wakeup_gpio;	/* BT_WAKE_HOST */

	u32 baudrate;
	u32 post_power_on_delay_ms;

	struct sk_buff *rx_skb;
	struct hci_uart hu;
};

static const struct h4_recv_pkt aic_recv_pkts[] = {
	{ H4_RECV_ACL,   .recv = hci_recv_frame },
	{ H4_RECV_SCO,   .recv = hci_recv_frame },
	{ H4_RECV_EVENT, .recv = hci_recv_frame },
};

static size_t aic_receive_buf(struct serdev_device *serdev,
			      const u8 *data, size_t count)
{
	struct aic_dev *adev = serdev_device_get_drvdata(serdev);

	adev->rx_skb = h4_recv_buf(&adev->hu, adev->rx_skb, data, count,
				   aic_recv_pkts, ARRAY_SIZE(aic_recv_pkts));
	if (IS_ERR(adev->rx_skb)) {
		int err = PTR_ERR(adev->rx_skb);

		bt_dev_err(adev->hdev, "Frame reassembly failed (%d)", err);
		adev->rx_skb = NULL;
		return 0;
	}

	adev->hdev->stat.byte_rx += count;
	return count;
}

static void aic_write_wakeup(struct serdev_device *serdev)
{
	serdev_device_write_wakeup(serdev);
}

static const struct serdev_device_ops aic_serdev_ops = {
	.receive_buf = aic_receive_buf,
	.write_wakeup = aic_write_wakeup,
};

static int aic_open(struct hci_dev *hdev)
{
	struct aic_dev *adev = hci_get_drvdata(hdev);
	int err;

	bt_dev_info(hdev, "aic_open: baudrate=%u post_delay=%u",
		    adev->baudrate, adev->post_power_on_delay_ms);

	/* Assert HOST_WAKE_BT before powering the chip on, matching the
	 * vendor power sequence; the BT firmware samples it while booting.
	 */
	if (adev->device_wakeup_gpio) {
		gpiod_set_value_cansleep(adev->device_wakeup_gpio, 1);
		bt_dev_info(hdev, "aic_open: device_wakeup(HOST_WAKE_BT)=%d",
			    gpiod_get_value_cansleep(adev->device_wakeup_gpio));
	}

	if (adev->enable_gpio) {
		/* Power the BT core on. No low->high reset pulse here: the
		 * combo BT firmware is loaded by the WiFi driver (aicbsp) and
		 * a pulse would reset the core and drop the already-loaded
		 * firmware on a reopen.
		 */
		gpiod_set_value_cansleep(adev->enable_gpio, 1);
		bt_dev_info(hdev, "aic_open: enable(BT_REG_ON)=%d",
			    gpiod_get_value_cansleep(adev->enable_gpio));
	}

	/* Allow the chip to power up and its clock to settle */
	msleep(adev->post_power_on_delay_ms);

	err = serdev_device_open(adev->serdev);
	if (err) {
		bt_dev_err(hdev, "Unable to open UART device");
		goto err_open;
	}

	err = serdev_device_set_baudrate(adev->serdev, adev->baudrate);
	bt_dev_info(hdev, "aic_open: set_baudrate ret=%d", err);
	/* The AIC8800 BT firmware is configured (via the WiFi driver's
	 * aicbt_patch_table_load) with flow control enabled (uart_flowctrl=1).
	 * Keep hardware RTS/CTS flow control on, otherwise the chip never
	 * asserts its TX path and HCI commands time out.
	 */
	serdev_device_set_flow_control(adev->serdev, true);

#if IS_ENABLED(CONFIG_AIC8800_SDIO_WLAN)
	/* The combo BT firmware is loaded by the WiFi driver over SDIO. If
	 * hci0 is brought up (e.g. by an init script at boot) before that
	 * load finishes, the HCI Reset races ahead of the firmware and times
	 * out with -110. Wait here so the stack only sends Reset once the BT
	 * core can actually answer.
	 */
	{
		unsigned long timeout = jiffies + msecs_to_jiffies(8000);

		while (!aicbsp_is_bt_fw_ready()) {
			if (time_after(jiffies, timeout)) {
				bt_dev_err(hdev,
					   "aic_open: BT firmware not ready within 8s");
				break;
			}
			msleep(50);
		}
		bt_dev_info(hdev, "aic_open: bt_fw_ready=%d",
			    aicbsp_is_bt_fw_ready());
	}
#endif

	return 0;

err_open:
	if (adev->enable_gpio)
		gpiod_set_value_cansleep(adev->enable_gpio, 0);
	return err;
}

static int aic_close(struct hci_dev *hdev)
{
	struct aic_dev *adev = hci_get_drvdata(hdev);

	if (adev->device_wakeup_gpio)
		gpiod_set_value_cansleep(adev->device_wakeup_gpio, 0);

	serdev_device_close(adev->serdev);

	/* Keep BT_REG_ON asserted: powering the combo BT core off would drop
	 * the firmware loaded by the WiFi driver, so a later reopen would get
	 * no HCI response (-110).
	 */

	return 0;
}

static int aic_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct aic_dev *adev = hci_get_drvdata(hdev);

	/* Prepend H4 packet type byte */
	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
	serdev_device_write_buf(adev->serdev, skb->data, skb->len);

	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);

	return 0;
}

static int aic_flush(struct hci_dev *hdev)
{
	struct aic_dev *adev = hci_get_drvdata(hdev);

	serdev_device_write_flush(adev->serdev);

	kfree_skb(adev->rx_skb);
	adev->rx_skb = NULL;

	return 0;
}

static int aic_setup(struct hci_dev *hdev)
{
	/* The Bluetooth firmware of the AIC8800 combo chip is loaded by the
	 * WiFi driver; nothing to do here.
	 */
	return 0;
}

static irqreturn_t aic_host_wakeup_irq(int irq, void *data)
{
	struct aic_dev *adev = data;

	bt_dev_dbg(adev->hdev, "host wakeup irq");
	return IRQ_HANDLED;
}

static int aic_parse_dt(struct serdev_device *serdev)
{
	struct aic_dev *adev = serdev_device_get_drvdata(serdev);
	struct device *dev = &serdev->dev;
	int err;

	device_property_read_u32(dev, "max-speed", &adev->baudrate);
	if (!adev->baudrate)
		adev->baudrate = AIC_DEFAULT_BAUDRATE;

	device_property_read_u32(dev, "post-power-on-delay-ms",
				 &adev->post_power_on_delay_ms);
	if (!adev->post_power_on_delay_ms)
		adev->post_power_on_delay_ms = 100;

	adev->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						    GPIOD_OUT_LOW);
	if (IS_ERR(adev->enable_gpio))
		return PTR_ERR(adev->enable_gpio);

	adev->device_wakeup_gpio = devm_gpiod_get_optional(dev, "device-wakeup",
							   GPIOD_OUT_LOW);
	if (IS_ERR(adev->device_wakeup_gpio))
		return PTR_ERR(adev->device_wakeup_gpio);

	adev->host_wakeup_gpio = devm_gpiod_get_optional(dev, "host-wakeup",
							 GPIOD_IN);
	if (IS_ERR(adev->host_wakeup_gpio))
		return PTR_ERR(adev->host_wakeup_gpio);

	if (adev->host_wakeup_gpio) {
		int irq = gpiod_to_irq(adev->host_wakeup_gpio);

		if (irq < 0)
			return irq;

		err = devm_request_threaded_irq(dev, irq, NULL,
						aic_host_wakeup_irq,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"aic8800-bt-wakeup", adev);
		if (err)
			return err;
	}

	return 0;
}

static int aic_probe(struct serdev_device *serdev)
{
	struct aic_dev *adev;
	struct hci_dev *hdev;
	int err;

	adev = devm_kzalloc(&serdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->serdev = serdev;
	serdev_device_set_drvdata(serdev, adev);

	serdev_device_set_client_ops(serdev, &aic_serdev_ops);

	err = aic_parse_dt(serdev);
	if (err)
		return err;

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	adev->hdev = hdev;
	adev->hu.hdev = hdev;

	hdev->bus = HCI_UART;
	hci_set_drvdata(hdev, adev);

	hdev->open	= aic_open;
	hdev->close	= aic_close;
	hdev->flush	= aic_flush;
	hdev->send	= aic_send_frame;
	hdev->setup	= aic_setup;
	SET_HCIDEV_DEV(hdev, &serdev->dev);

	if (hci_register_dev(hdev) < 0) {
		bt_dev_err(hdev, "Can't register HCI device");
		hci_free_dev(hdev);
		return -ENODEV;
	}

	return 0;
}

static void aic_remove(struct serdev_device *serdev)
{
	struct aic_dev *adev = serdev_device_get_drvdata(serdev);

	hci_unregister_dev(adev->hdev);
	hci_free_dev(adev->hdev);
}

static const struct of_device_id aic_of_match_table[] = {
	{ .compatible = "aic,aic8800-bt" },
	{ }
};
MODULE_DEVICE_TABLE(of, aic_of_match_table);

static struct serdev_device_driver aic_serdev_driver = {
	.probe = aic_probe,
	.remove = aic_remove,
	.driver = {
		.name = "hci_aic8800",
		.of_match_table = of_match_ptr(aic_of_match_table),
	},
};

module_serdev_device_driver(aic_serdev_driver);

MODULE_AUTHOR("AICSemi");
MODULE_DESCRIPTION("AIC8800 Bluetooth HCI UART driver");
MODULE_LICENSE("GPL");
