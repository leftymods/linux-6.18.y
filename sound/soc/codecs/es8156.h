/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * es8156.h -- Everest ES8156 ALSA SoC Audio Driver
 *
 * Copyright (C) Everest Semiconductor Co.,Ltd.
 * Ported to Mainline Linux 6.x by Reverse Engineering
 */

#ifndef _ES8156_H
#define _ES8156_H

#define ES8156_RESET_REG00              0x00
#define ES8156_MAINCLK_REG01            0x01
#define ES8156_CLK_SYS1_REG02           0x02
#define ES8156_CLK_SYS2_REG03           0x03
#define ES8156_CLK_SYS3_REG04           0x04
#define ES8156_CLK_SYS4_REG05           0x05
#define ES8156_CLK_SYS5_REG06           0x06
#define ES8156_NF_PROG1_REG07           0x07
#define ES8156_NF_PROG2_REG08           0x08
#define ES8156_MISC_CONTROL1_REG09      0x09
#define ES8156_MISC_CONTROL2_REG0A      0x0a
#define ES8156_TIME_CONTROL1_REG0B      0x0b
#define ES8156_TIME_CONTROL2_REG0C      0x0c
#define ES8156_SYSTEM_CONTROL1_REG0D    0x0d
#define ES8156_SDP_INTERFACE1_REG11     0x11
#define ES8156_SDP_INTERFACE2_REG12     0x12
#define ES8156_DAC_MUTE_REG13           0x13
#define ES8156_VOLUME_CONTROL_REG14     0x14
#define ES8156_VOLUME_CONTROL_REG15     0x15
#define ES8156_ALC1_REG15               0x15
#define ES8156_ALC2_REG16               0x16
#define ES8156_ALC3_REG17               0x17
#define ES8156_EQ_CONTROL1_REG18        0x18
#define ES8156_MISC_CONTROL3_REG19      0x19
#define ES8156_MISC_CONTROL4_REG1A      0x1a
#define ES8156_ANALOG_SYS1_REG20        0x20
#define ES8156_ANALOG_SYS2_REG21        0x21
#define ES8156_ANALOG_SYS3_REG22        0x22
#define ES8156_ANALOG_SYS4_REG23        0x23
#define ES8156_ANALOG_SYS5_REG24        0x24
#define ES8156_ANALOG_SYS6_REG25        0x25
#define ES8156_CHIP_STATUS_REGFC        0xfc
#define ES8156_CHIP_ID_REGFD            0xfd
#define ES8156_CHIP_ID_REGFE            0xfe
#define ES8156_CHIP_VERSION_REGFF       0xff

/* Bit fields */
#define ES8156_RESET_CSM_ON             0x00
#define ES8156_RESET_RST_MASK           0x1F

/* SDP format bits (Reg 0x11) */
#define ES8156_SDP_FMT_MASK             0x03
#define ES8156_SDP_FMT_I2S              0x00
#define ES8156_SDP_FMT_LJ               0x01
#define ES8156_SDP_FMT_DSP              0x03

#define ES8156_SDP_WL_MASK              0x70
#define ES8156_SDP_WL_16                0x00
#define ES8156_SDP_WL_24                0x10
#define ES8156_SDP_WL_20                0x20
#define ES8156_SDP_WL_32                0x30

/* Mute control (Reg 0x13 / 0x14) */
#define ES8156_AUTOMUTE_MASK            0x04
#define ES8156_UNMUTE                   0x00
#define ES8156_MUTE                     0x04

int es8156_headset_detect(int enable);

#endif /* _ES8156_H */
