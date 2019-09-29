#ifndef __IOT_OPERATE_H__
#define __IOT_OPERATE_H__
#include "iotobject.h"
#include "ch_manage.h"
#include "sys_param.h"
#include "ad717x.h"

#define IOT_OPERATE_INVALID		-1
#define IOT_OPERATE_SUCCESS		0
#define IOT_OPERATE_FAIL 		1

#define CHL_ENABLE_ID			3
#define CHL_ERROR_ID			4
#define BATTERY_LEVEL_ID		5
#define CHL_TYPE_ID(ch_no)		(6+(ch_no)*4)
#define CHL_VOLT_TYPE_ID(ch_no) (7+(ch_no)*4)
#define CHL_DELAY_ID(ch_no) 	(8+(ch_no)*4)
#define CHL_VALUE_ID(ch_no) 	(9+(ch_no)*4)
#define CHL_CALI_ID(ch_no)		(38+(ch_no))
#define DEV_WR_PROT_ID			46

typedef struct {
	uint8_t err_flag;
	uint8_t gas_gauge_flag;
	uint8_t err_msg;
	uint8_t gas_gauge;
	uint8_t *init_delay_flag;

	sys_param_t *sys_param;
	ch_supply_t *ch_supply;
	ch_circuit_t *ch_circuit;
	iot_object_t *sensor;
	ad717x_dev *ad_dev;

	uint8_t (*operate)(void);
} IoT_dev_t;

IoT_dev_t * IoT_Init(sys_param_t *param, ch_supply_t *ch_supply,
		ch_circuit_t *ch_circuit, iot_object_t *sensor, ad717x_dev *ad_dev);

#endif
