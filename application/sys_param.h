#ifndef __SYS_PARAM_H__
#define __SYS_PARAM_H__
#include "main.h"

#define SYS_MAX_CH_NUM				8
#define SYS_CH_NUM					4
#define SYS_PARAM_FLASH_PAGE_ADDR	0x0800F400	//page 61

typedef enum {
	CH_DISABLE, CH_ENABLE
} ch_status;

typedef enum {
	VOTL_0_5V, VOLT_0_10V, CURR_4_20MA
} ch_type;

typedef enum {
	PWR_VOLT_12V, PWR_VOLT_24V
} ch_pwr_volt;

typedef enum {
	INT_OFFSET_CALI = 0X01,
	INT_GAIN_CALI = 0X02,
	SYS_OFFSET_CALI = 0X04,
	SYS_GAIN_CALI = 0X08
} ad717x_cali_mode;

typedef struct {
	ch_status status;
	ch_type type;
	ch_pwr_volt pwr_volt;
	uint8_t init_delay;
} ch_prop;

typedef struct {
	uint8_t mode;
	uint32_t offset;
	uint32_t gain;
} ch_cali;

typedef struct {
	uint8_t update_flag;
	uint8_t ch_num;
	ch_prop prop[SYS_MAX_CH_NUM];
	ch_cali cali[SYS_MAX_CH_NUM];
	uint8_t (*saveParamToFlash)(void);
} sys_param_t;

sys_param_t* Sys_ParamInit(void);
uint8_t Sys_SaveParamToFlash(void);

#endif
