#ifndef __AIC_BSP_EXPORT_H
#define __AIC_BSP_EXPORT_H

enum aicbsp_subsys {
	AIC_BLUETOOTH,
	AIC_WIFI,
};

enum aicbsp_pwr_state {
	AIC_PWR_OFF,
	AIC_PWR_ON,
};

enum skb_buff_id {
	AIC_RESV_MEM_TXDATA,
};

struct skb_buff_pool {
	uint32_t id;
	uint32_t size;
	const char *name;
	uint8_t used;
	struct sk_buff *skb;
};

struct aicbsp_feature_t {
	int      hwinfo;
	uint32_t sdio_clock;
	uint8_t  sdio_phase;
	bool     fwlog_en;
	uint8_t  irqf;
};

#if defined(CONFIG_DPD) || defined(CONFIG_LOFT_CALIB)
typedef struct {
    uint32_t bit_mask[3];
    uint32_t reserved;
    uint32_t dpd_high[96];
    uint32_t dpd_11b[96];
    uint32_t dpd_low[96];
    uint32_t idac_11b[48];
    uint32_t idac_high[48];
    uint32_t idac_low[48];
    uint32_t loft_res[18];
    uint32_t rx_iqim_res[16];
} rf_misc_ram_t;

typedef struct {
    uint32_t bit_mask[4];
    uint32_t dpd_high[96];
    uint32_t loft_res[18];
} rf_misc_ram_lite_t;

#define MEMBER_SIZE(type, member)   sizeof(((type *)0)->member)
#define DPD_RESULT_SIZE_8800DC      sizeof(rf_misc_ram_lite_t)
#endif

#ifdef CONFIG_DPD
extern rf_misc_ram_lite_t aicwf_sdio_dpd_res;
#endif

#ifdef CONFIG_LOFT_CALIB
extern rf_misc_ram_lite_t aicwf_sdio_loft_res_local;
#endif

int aicwf_sdio_aicbsp_set_subsys(int, int);
int aicwf_sdio_aicbsp_get_feature(struct aicbsp_feature_t *feature, char *fw_path);
struct sk_buff *aicwf_sdio_aicbsp_resv_mem_alloc_skb(unsigned int length, uint32_t id);
void aicwf_sdio_aicbsp_resv_mem_kfree_skb(struct sk_buff *skb, uint32_t id);

/* true once the combo BT firmware has been loaded over SDIO */
bool aicbsp_is_bt_fw_ready(void);

struct sdio_func;
struct aic_sdio_dev;
int aicwf_sdio_readb(struct aic_sdio_dev *sdiodev, uint regaddr, u8 *val);
struct sk_buff *aicwf_sdio_readframes(struct aic_sdio_dev *sdiodev, u8 msg);
void aicwf_sdio_get_fw_path(char* fw_path);
int aicwf_sdio_get_testmode(void);
struct sdio_func *aicwf_sdio_get_sdio_func(void);
void aicwf_sdio_set_irq_handler(void *fn);
bool aicwf_sdio_aicbsp_get_load_fw_in_fdrv(void);
int aicwf_sdio_get_adap_test(void);
bool aicwf_sdio_get_fdrv_no_reg_sdio(void);

#endif
