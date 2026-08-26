// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2018-2019  Realtek Corporation
 */

#include <linux/iopoll.h>

#include "main.h"
#include "efuse.h"
#include "reg.h"
#include "rtw8822c.h"
#include "debug.h"

#define RTW_EFUSE_BANK_WIFI		0x0

static void switch_efuse_bank(struct rtw_dev *rtwdev)
{
	rtw_write32_mask(rtwdev, REG_LDO_EFUSE_CTRL, BIT_MASK_EFUSE_BANK_SEL,
			 RTW_EFUSE_BANK_WIFI);
}

#define invalid_efuse_header(hdr1, hdr2) \
	((hdr1) == 0xff || (((hdr1) & 0x1f) == 0xf && (hdr2) == 0xff))
#define invalid_efuse_content(word_en, i) \
	(((word_en) & BIT(i)) != 0x0)
#define get_efuse_blk_idx_2_byte(hdr1, hdr2) \
	((((hdr2) & 0xf0) >> 1) | (((hdr1) >> 5) & 0x07))
#define get_efuse_blk_idx_1_byte(hdr1) \
	(((hdr1) & 0xf0) >> 4)
#define block_idx_to_logical_idx(blk_idx, i) \
	(((blk_idx) << 3) + ((i) << 1))

/* efuse header format
 *
 * | 7        5   4    0 | 7        4   3          0 | 15  8  7   0 |
 *   block[2:0]   0 1111   block[6:3]   word_en[3:0]   byte0  byte1
 * | header 1 (optional) |          header 2         |    word N    |
 *
 * word_en: 4 bits each word. 0 -> write; 1 -> not write
 * N: 1~4, depends on word_en
 */
static int rtw_dump_logical_efuse_map(struct rtw_dev *rtwdev, u8 *phy_map,
				      u8 *log_map)
{
	u32 physical_size = rtwdev->efuse.physical_size;
	u32 protect_size = rtwdev->efuse.protect_size;
	u32 logical_size = rtwdev->efuse.logical_size;
	u32 phy_idx, log_idx;
	u8 hdr1, hdr2;
	u8 blk_idx;
	u8 word_en;
	int i;

	for (phy_idx = 0; phy_idx < physical_size - protect_size;) {
		hdr1 = phy_map[phy_idx];
		hdr2 = phy_map[phy_idx + 1];
		if (invalid_efuse_header(hdr1, hdr2))
			break;

		if ((hdr1 & 0x1f) == 0xf) {
			/* 2-byte header format */
			blk_idx = get_efuse_blk_idx_2_byte(hdr1, hdr2);
			word_en = hdr2 & 0xf;
			phy_idx += 2;
		} else {
			/* 1-byte header format */
			blk_idx = get_efuse_blk_idx_1_byte(hdr1);
			word_en = hdr1 & 0xf;
			phy_idx += 1;
		}

		for (i = 0; i < 4; i++) {
			if (invalid_efuse_content(word_en, i))
				continue;

			log_idx = block_idx_to_logical_idx(blk_idx, i);
			if (phy_idx + 1 > physical_size - protect_size ||
			    log_idx + 1 > logical_size)
				return -EINVAL;

			log_map[log_idx] = phy_map[phy_idx];
			log_map[log_idx + 1] = phy_map[phy_idx + 1];
			phy_idx += 2;
		}
	}
	return 0;
}

static int rtw_dump_physical_efuse_map(struct rtw_dev *rtwdev, u8 *map)
{
	const struct rtw_chip_info *chip = rtwdev->chip;
	u32 size = rtwdev->efuse.physical_size;
	u32 efuse_ctl;
	u32 addr;
	u32 cnt;

	rtw_chip_efuse_grant_on(rtwdev);

	switch_efuse_bank(rtwdev);

	/* ==== probe-kit v4: characterise the efuse engine ======
	 * Symptom: writing REG_EFUSE_CTRL loses all bits except a
	 * static BIT29; DONE never raises, even at addr 0.
	 * Matrix: bus aliveness, ctrl background stability and a
	 * time-profile of one start attempt, LDO25 off AND on.   */

#define PRK_CANARY_REG 0x3c /* free sys register, not used here */

	/* 1) is the SDIO register file alive after grant/bank? */
	rtw_write32(rtwdev, PRK_CANARY_REG, 0x1234a55a);
	rtw_info(rtwdev,
		 "prk1: canary write/read = %s (val=0x%08x)\n",
		 rtw_read32(rtwdev, PRK_CANARY_REG) == 0x1234a55a ?
		 "ALIVE" : "DEAD-BUS", rtw_read32(rtwdev, PRK_CANARY_REG));
	rtw_write32(rtwdev, PRK_CANARY_REG, 0);

	/* 2) background stability of EFUSE_CTRL with no writes */
	{
		u32 b1 = rtw_read32(rtwdev, REG_EFUSE_CTRL);
		u32 b2 = rtw_read32(rtwdev, REG_EFUSE_CTRL);
		u32 sfe = rtw_read32(rtwdev, REG_SYS_FUNC_EN);
		u32 ldo = rtw_read8(rtwdev, REG_ANAPARLDO_POW_MAC);
		rtw_info(rtwdev,
			 "prk2: bg=0x%08x/0x%08x SYS_FUNC_EN=0x%x LDO25reg=0x%02x\n",
			 b1, b2, sfe, ldo);
	}

	for (int prk_ldo = 0; prk_ldo < 2; prk_ldo++) {
		int profile_bits;
		chip->ops->cfg_ldo25(rtwdev, prk_ldo);
		/* build "addr0 + keep other ctl bits" then start */
		efuse_ctl = rtw_read32(rtwdev, REG_EFUSE_CTRL);
		efuse_ctl &= ~(BIT_MASK_EF_DATA | BITS_EF_ADDR);
		rtw_write32(rtwdev, REG_EFUSE_CTRL,
			    efuse_ctl & ~BIT_EF_FLAG);

		/* snapshot the value immediately and every ~20ms x5 */
		profile_bits = 0;
		for (u32 t = 0; t < 6; t++) {
			if (t)
				mdelay(20);
			efuse_ctl = rtw_read32(rtwdev, REG_EFUSE_CTRL);
			rtw_info(rtwdev,
				 "prk3[ldo%d,t+%02ums]: ctl=0x%08x DONE=%d\n",
				 prk_ldo, t * 20, efuse_ctl,
				 !!(efuse_ctl & BIT_EF_FLAG));
			profile_bits |= (int)(efuse_ctl != 0);
		}
		(void)profile_bits;

		if (efuse_ctl & BIT_EF_FLAG) {
			rtw_info(rtwdev,
				 "prk4: engine ALIVE with ldo=%d! resuming dump\n",
				 prk_ldo);
			break;
		}
	}
	/* fall through to mainline loop; if engine woke via alt-LDO
	 * the first iterations will now succeed naturally */

#undef PRK_CANARY_REG

	/* standard restore for the loop path */
	chip->ops->cfg_ldo25(rtwdev, false);

	efuse_ctl = rtw_read32(rtwdev, REG_EFUSE_CTRL);

	for (addr = 0; addr < size; addr++) {
		efuse_ctl &= ~(BIT_MASK_EF_DATA | BITS_EF_ADDR);
		efuse_ctl |= (addr & BIT_MASK_EF_ADDR) << BIT_SHIFT_EF_ADDR;
		rtw_write32(rtwdev, REG_EFUSE_CTRL, efuse_ctl & (~BIT_EF_FLAG));

		cnt = 1000000;
		do {
			udelay(1);
			efuse_ctl = rtw_read32(rtwdev, REG_EFUSE_CTRL);
			if (--cnt == 0)
				return -EBUSY;
		} while (!(efuse_ctl & BIT_EF_FLAG));

		*(map + addr) = (u8)(efuse_ctl & BIT_MASK_EF_DATA);
	}

	rtw_chip_efuse_grant_off(rtwdev);

	return 0;
}

int rtw_read8_physical_efuse(struct rtw_dev *rtwdev, u16 addr, u8 *data)
{
	u32 efuse_ctl;
	int ret;

	rtw_write32_mask(rtwdev, REG_EFUSE_CTRL, 0x3ff00, addr);
	rtw_write32_clr(rtwdev, REG_EFUSE_CTRL, BIT_EF_FLAG);

	ret = read_poll_timeout(rtw_read32, efuse_ctl, efuse_ctl & BIT_EF_FLAG,
				1000, 100000, false, rtwdev, REG_EFUSE_CTRL);
	if (ret) {
		*data = EFUSE_READ_FAIL;
		return ret;
	}

	*data = rtw_read8(rtwdev, REG_EFUSE_CTRL);

	return 0;
}
EXPORT_SYMBOL(rtw_read8_physical_efuse);

int rtw_parse_efuse_map(struct rtw_dev *rtwdev)
{
	const struct rtw_chip_info *chip = rtwdev->chip;
	struct rtw_efuse *efuse = &rtwdev->efuse;
	u32 phy_size = efuse->physical_size;
	u32 log_size = efuse->logical_size;
	u8 *phy_map = NULL;
	u8 *log_map = NULL;
	int ret = 0;

	phy_map = kmalloc(phy_size, GFP_KERNEL);
	log_map = kmalloc(log_size, GFP_KERNEL);
	if (!phy_map || !log_map) {
		ret = -ENOMEM;
		goto out_free;
	}

	ret = rtw_dump_physical_efuse_map(rtwdev, phy_map);
	if (ret) {
		rtw_err(rtwdev, "failed to dump efuse physical map\n");
		goto out_free;
	}

	memset(log_map, 0xff, log_size);
	ret = rtw_dump_logical_efuse_map(rtwdev, phy_map, log_map);
	if (ret) {
		rtw_err(rtwdev, "failed to dump efuse logical map\n");
		goto out_free;
	}

	ret = chip->ops->read_efuse(rtwdev, log_map);
	if (ret) {
		rtw_err(rtwdev, "failed to read efuse map\n");
		goto out_free;
	}

out_free:
	kfree(log_map);
	kfree(phy_map);

	return ret;
}
