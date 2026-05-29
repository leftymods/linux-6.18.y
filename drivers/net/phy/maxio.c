// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxio Ethernet PHY driver
 *
 * Supports the MAE0621A PHY used by the MD1000 RK3566 board.  The PHY
 * may report a temporary 0xffff4411 ID before the vendor-page init
 * sequence is applied; keep that ID so phylib can bind early enough to
 * initialize the device.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/phy.h>

#define PHY_ID_MAE0621A_Q2C              0x7b744411
#define PHY_ID_MAE0621A_Q3C              0x7b744412
#define PHY_ID_MAE0621A_Q2C_BOOT         0xffff4411
#define MAE0621A_PHY_ID_MASK             0xffffffff

#define MAE0621A_PAGE_SELECT             0x1f

static int maxio_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, MAE0621A_PAGE_SELECT);
}

static int maxio_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, MAE0621A_PAGE_SELECT, page);
}

static int maxio_read_paged(struct phy_device *phydev, int page, u32 regnum)
{
	int oldpage, ret;

	oldpage = phy_select_page(phydev, page);
	if (oldpage < 0)
		return oldpage;

	ret = __phy_read(phydev, regnum);

	return phy_restore_page(phydev, oldpage, ret);
}

static int maxio_write_paged(struct phy_device *phydev, int page,
			     u32 regnum, u16 val)
{
	int oldpage, ret;

	oldpage = phy_select_page(phydev, page);
	if (oldpage < 0)
		return oldpage;

	ret = __phy_write(phydev, regnum, val);

	return phy_restore_page(phydev, oldpage, ret);
}

static int maxio_write_mmd(struct phy_device *phydev, int devnum, u16 regnum,
			   u16 val)
{
	int oldpage, ret;

	if (devnum != MDIO_MMD_AN || regnum != MDIO_AN_EEE_ADV)
		return -EOPNOTSUPP;

	oldpage = phy_select_page(phydev, 0);
	if (oldpage < 0)
		return oldpage;

	ret = __phy_write(phydev, MII_MMD_CTRL, MDIO_MMD_AN);
	if (ret < 0)
		goto out;

	ret = __phy_write(phydev, MII_MMD_DATA, MDIO_AN_EEE_ADV);
	if (ret < 0)
		goto out;

	ret = __phy_write(phydev, MII_MMD_CTRL, MII_MMD_CTRL_NOINCR |
			  MDIO_MMD_AN);
	if (ret < 0)
		goto out;

	ret = __phy_write(phydev, MII_MMD_DATA, val);

out:
	return phy_restore_page(phydev, oldpage, ret);
}

static int mae0621a_adc_ready(struct phy_device *phydev)
{
	static const u16 values[] = { 0xf908, 0xfa08, 0xfb08, 0xfc08 };
	int i, ret;

	ret = maxio_write_paged(phydev, 0xd96, 0x02, 0x1fff);
	if (ret < 0)
		return ret;

	ret = maxio_write_paged(phydev, 0xd96, 0x02, 0x1000);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(values); i++) {
		ret = maxio_write_paged(phydev, 0xd8f, 0x0b, values[i]);
		if (ret < 0)
			return ret;

		ret = maxio_read_paged(phydev, 0xd92, 0x0b);
		if (ret < 0)
			return ret;
		if (!(ret & 0x01ff))
			return -EAGAIN;
	}

	return 0;
}

static int mae0621a_self_check(struct phy_device *phydev)
{
	int i, ret = -EAGAIN;

	for (i = 0; i < 50; i++) {
		ret = mae0621a_adc_ready(phydev);
		if (!ret)
			break;

		ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x1940);
		if (ret < 0)
			return ret;
		msleep(10);

		ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x1140);
		if (ret < 0)
			return ret;

		ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x9140);
		if (ret < 0)
			return ret;
	}

	ret = maxio_write_paged(phydev, 0xd96, 0x02, 0x0fff);
	if (ret < 0)
		return ret;

	ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x9140);
	if (ret < 0)
		return ret;

	ret = maxio_write_page(phydev, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int mae0621a_config_init(struct phy_device *phydev)
{
	int ret;

	phydev_info(phydev, "initializing Maxio MAE0621A PHY\n");

	ret = maxio_write_paged(phydev, 0xd92, 0x02, 0x200a);
	if (ret < 0)
		return ret;
	ret = maxio_write_page(phydev, 0);
	if (ret < 0)
		return ret;
	msleep(100);

	ret = maxio_write_paged(phydev, 0xda0, 0x10, 0x0f13);
	ret |= maxio_write_paged(phydev, 0xd92, 0x02, 0x200a);
	ret |= maxio_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);
	phy_disable_eee_mode(phydev, ETHTOOL_LINK_MODE_100baseT_Full_BIT);
	phy_disable_eee_mode(phydev, ETHTOOL_LINK_MODE_1000baseT_Full_BIT);

	ret |= maxio_write_paged(phydev, 0xd8f, 0x00, 0x0300);
	ret |= maxio_write_paged(phydev, 0xd90, 0x02, 0x1555);
	ret |= maxio_write_paged(phydev, 0xd90, 0x05, 0x2b15);
	ret |= maxio_write_paged(phydev, 0xd96, 0x13, 0x07bc);
	ret |= maxio_write_paged(phydev, 0xd8f, 0x08, 0x2500);
	ret |= maxio_write_paged(phydev, 0xd91, 0x06, 0x6880);
	ret |= maxio_write_paged(phydev, 0xd92, 0x14, 0x000a);
	ret |= maxio_write_paged(phydev, 0xd91, 0x07, 0x5b00);
	ret |= maxio_write_paged(phydev, 0xa43, 0x19, 0x0823);
	ret |= maxio_write_page(phydev, 0);
	if (ret < 0)
		return ret;

	ret = mae0621a_self_check(phydev);
	if (ret < 0)
		phydev_warn(phydev, "self check did not report ready: %d\n", ret);

	msleep(100);
	return 0;
}

static struct phy_driver maxio_drivers[] = {
	{
		PHY_ID_MATCH_EXACT(PHY_ID_MAE0621A_Q2C),
		.name		= "Maxio MAE0621A Gigabit Ethernet",
		.config_init	= mae0621a_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= maxio_read_page,
		.write_page	= maxio_write_page,
	}, {
		PHY_ID_MATCH_EXACT(PHY_ID_MAE0621A_Q3C),
		.name		= "Maxio MAE0621A Q3C Gigabit Ethernet",
		.config_init	= mae0621a_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= maxio_read_page,
		.write_page	= maxio_write_page,
	}, {
		PHY_ID_MATCH_EXACT(PHY_ID_MAE0621A_Q2C_BOOT),
		.name		= "Maxio MAE0621A Gigabit Ethernet (ffff4411)",
		.config_init	= mae0621a_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_page	= maxio_read_page,
		.write_page	= maxio_write_page,
	},
};
module_phy_driver(maxio_drivers);

static struct mdio_device_id __maybe_unused maxio_tbl[] = {
	{ PHY_ID_MAE0621A_Q2C, MAE0621A_PHY_ID_MASK },
	{ PHY_ID_MAE0621A_Q3C, MAE0621A_PHY_ID_MASK },
	{ PHY_ID_MAE0621A_Q2C_BOOT, MAE0621A_PHY_ID_MASK },
	{ }
};
MODULE_DEVICE_TABLE(mdio, maxio_tbl);

MODULE_DESCRIPTION("Maxio MAE0621A PHY driver");
MODULE_AUTHOR("MD1000 RK3566 bringup");
MODULE_LICENSE("GPL");
