#ifndef __CH_MANAGE_H__
#define __CH_MANAGE_H__
#include "tca6408a.h"
#include "sys_param.h"

#define CH_SET_SUCCESS			0
#define CH_SET_FAIL 			1

typedef enum{
	CH_PWR_0V,
	CH_PWR_12V,
	CH_PWR_24V
}ch_pwr_mode;

typedef enum{
	CH_PWR_VOLT,
	CH_PWR_CURR
}ch_circuit_mode;

typedef struct{
	uint8_t ch_no;
	ch_pwr_mode mode;
	TCA6408_dev *tca_dev;
	sys_param_t *param;
	uint8_t (*tcaWriteOutput)(TCA6408_dev *dev);
}ch_supply_t;

typedef struct{
	uint8_t ch_no;
	ch_circuit_mode mode;
	TCA6408_dev *tca_dev;
	sys_param_t *param;
	uint8_t (*tcaWriteOutput)(TCA6408_dev *dev);
}ch_circuit_t;

typedef enum{
	SENSOR_OFFLINE,
	SENSOR_ONLINE
}ch_sensor_status;

typedef struct{
	uint32_t det_interval;
	ch_sensor_status status[4];

	TCA6408_dev *tca_dev;
	int32_t (*dectect)(void);
}ch_sensor_t;

ch_sensor_t* Ch_SensorDetInit(TCA6408_dev *dev);
int32_t Ch_SensorDectect(void);

ch_supply_t* Ch_SupplyInit(TCA6408_dev *dev, sys_param_t *sys_param);
uint8_t Ch_SetSupply(ch_supply_t *ch_desc);

ch_circuit_t* Ch_CircuitInit(TCA6408_dev *dev, sys_param_t *sys_param);
uint8_t Ch_CircuitSwitch(ch_circuit_t *ch_circuit);

#endif
