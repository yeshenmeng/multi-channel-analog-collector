#include "main.h"
#include "ch_manage.h"
#include <string.h>

#define NULL_POINTER	(void *)0

ch_supply_t ch_supply;

/**
 * @brief 通道供电控制初始化
 *        根据系统参数配置初始化通道供电方式
 * @param dev: 指向TCA6408_dev结构的指针
 * @param sys_param: 系统参数
 * @retval 返回通道供电控制句柄
 */
ch_supply_t* Ch_SupplyInit(TCA6408_dev *dev, sys_param_t *sys_param) {
	ch_supply.tca_dev = dev;
	ch_supply.param = sys_param;
	ch_supply.tcaWriteOutput = TCA6408A_WriteOutput;

	for (int i = 0; i < ch_supply.param->ch_num; i++) {
		ch_supply.ch_no = i;
		ch_supply.mode = CH_PWR_0V;

		if (ch_supply.param->prop[i].status == CH_ENABLE) {
			if (ch_supply.param->prop[i].pwr_volt == PWR_VOLT_12V) {
				ch_supply.mode = CH_PWR_12V;
			} else if (ch_supply.param->prop[i].pwr_volt == PWR_VOLT_24V) {
				ch_supply.mode = CH_PWR_24V;
			}
		}

		if (Ch_SetSupply(&ch_supply) != CH_SET_SUCCESS)
			return NULL_POINTER ;
	}

	return &ch_supply;
}

/**
 * @brief  设置通道供电方式
 * @param  ch_supply: 指向ch_supply_t结构的指针
 * @retval 0：成功  其它：失败
 */
uint8_t Ch_SetSupply(ch_supply_t *ch_supply) {
	uint8_t ret = CH_SET_SUCCESS;

	if (ch_supply->ch_no == 0) {
		if (ch_supply->mode == CH_PWR_0V) {
			ch_supply->tca_dev->regs.Output.bit.P0 = 0;
			ch_supply->tca_dev->regs.Output.bit.P1 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_12V) {
			ch_supply->tca_dev->regs.Output.bit.P0 = 0;
			ch_supply->tca_dev->regs.Output.bit.P1 = 1;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_24V) {
			ch_supply->tca_dev->regs.Output.bit.P0 = 1;
			ch_supply->tca_dev->regs.Output.bit.P1 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		}
	} else if (ch_supply->ch_no == 1) {
		if (ch_supply->mode == CH_PWR_0V) {
			ch_supply->tca_dev->regs.Output.bit.P2 = 0;
			ch_supply->tca_dev->regs.Output.bit.P3 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_12V) {
			ch_supply->tca_dev->regs.Output.bit.P2 = 0;
			ch_supply->tca_dev->regs.Output.bit.P3 = 1;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_24V) {
			ch_supply->tca_dev->regs.Output.bit.P2 = 1;
			ch_supply->tca_dev->regs.Output.bit.P3 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		}
	} else if (ch_supply->ch_no == 2) {
		if (ch_supply->mode == CH_PWR_0V) {
			ch_supply->tca_dev->regs.Output.bit.P4 = 0;
			ch_supply->tca_dev->regs.Output.bit.P5 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_12V) {
			ch_supply->tca_dev->regs.Output.bit.P4 = 0;
			ch_supply->tca_dev->regs.Output.bit.P5 = 1;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_24V) {
			ch_supply->tca_dev->regs.Output.bit.P4 = 1;
			ch_supply->tca_dev->regs.Output.bit.P5 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		}
	} else if (ch_supply->ch_no == 3) {
		if (ch_supply->mode == CH_PWR_0V) {
			ch_supply->tca_dev->regs.Output.bit.P6 = 0;
			ch_supply->tca_dev->regs.Output.bit.P7 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_12V) {
			ch_supply->tca_dev->regs.Output.bit.P6 = 0;
			ch_supply->tca_dev->regs.Output.bit.P7 = 1;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		} else if (ch_supply->mode == CH_PWR_24V) {
			ch_supply->tca_dev->regs.Output.bit.P6 = 1;
			ch_supply->tca_dev->regs.Output.bit.P7 = 0;
			ret = ch_supply->tcaWriteOutput(ch_supply->tca_dev);
		}
	}

	return ret;
}

ch_circuit_t ch_circuit;

/**
 * @brief 通道电路初始化
 * 		      根据系统参数配置初始化通道电路方式（电压/电流）
 * @param dev: 指向TCA6408_dev结构的指针
 * @param sys_param: 系统参数
 * @retval 返回通道电路切换控制句柄
 */
ch_circuit_t* Ch_CircuitInit(TCA6408_dev *dev, sys_param_t *sys_param) {
	ch_circuit.tca_dev = dev;
	ch_circuit.param = sys_param;
	ch_circuit.tcaWriteOutput = TCA6408A_WriteOutput;

	for (int i = 0; i < ch_supply.param->ch_num; i++) {
		ch_circuit.ch_no = i;
		ch_circuit.mode = CH_PWR_VOLT;

		if (ch_circuit.param->prop[i].status == CH_ENABLE) {
			if (ch_circuit.param->prop[i].type == CURR_4_20MA) {
				ch_circuit.mode = CH_PWR_CURR;
			}
		}

		if (Ch_CircuitSwitch(&ch_circuit) != CH_SET_SUCCESS)
			return NULL_POINTER ;
	}

	return &ch_circuit;
}

/**
 * @brief  通道电路切换
 * @param  ch_circuit: 指向ch_circuit_t结构的指针
 * @retval 0：成功  其它：失败
 */
uint8_t Ch_CircuitSwitch(ch_circuit_t *ch_circuit) {
	uint8_t ret = CH_SET_SUCCESS;

	if (ch_circuit->ch_no == 0) {
		if (ch_circuit->mode == CH_PWR_VOLT) {
			ch_circuit->tca_dev->regs.Output.bit.P0 = 0;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		} else if (ch_circuit->mode == CH_PWR_CURR) {
			ch_circuit->tca_dev->regs.Output.bit.P0 = 1;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		}
	} else if (ch_circuit->ch_no == 1) {
		if (ch_circuit->mode == CH_PWR_VOLT) {
			ch_circuit->tca_dev->regs.Output.bit.P1 = 0;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		} else if (ch_circuit->mode == CH_PWR_CURR) {
			ch_circuit->tca_dev->regs.Output.bit.P1 = 1;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		}
	} else if (ch_circuit->ch_no == 2) {
		if (ch_circuit->mode == CH_PWR_VOLT) {
			ch_circuit->tca_dev->regs.Output.bit.P2 = 0;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		} else if (ch_circuit->mode == CH_PWR_CURR) {
			ch_circuit->tca_dev->regs.Output.bit.P2 = 1;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		}
	} else if (ch_circuit->ch_no == 3) {
		if (ch_circuit->mode == CH_PWR_VOLT) {
			ch_circuit->tca_dev->regs.Output.bit.P3 = 0;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		} else if (ch_circuit->mode == CH_PWR_CURR) {
			ch_circuit->tca_dev->regs.Output.bit.P3 = 1;
			ret = ch_circuit->tcaWriteOutput(ch_circuit->tca_dev);
		}
	}

	return ret;
}

ch_sensor_t ch_sensor;

/**
 * @brief 通道传感器检测功能初始化
 * @param dev: 指向TCA6408_dev结构的指针
 * @retval 返回通道传感器检测句柄
 */
ch_sensor_t* Ch_SensorDetInit(TCA6408_dev *dev) {
	ch_sensor.det_interval = 200;
	memset(ch_sensor.status, 0X00, sizeof(ch_sensor.status));

	ch_sensor.tca_dev = dev;
	ch_sensor.dectect = Ch_SensorDectect;

#ifdef CIRCUIT_TEST
	TCA6408A_ReadInput(ch_sensor.tca_dev);
#endif

	return &ch_sensor;
}

/**
 * @brief 通道传感器检测
 * 		      通道传感器状态保存在传感器检测句柄中
 * @param None
 * @retval 0：成功  其它：失败
 */
int32_t Ch_SensorDectect(void) {
	int32_t ret = 0;

	if (ch_sensor.det_interval != 0) {
		static uint32_t delay_start = 0;

		if (delay_start == 0) {
			delay_start = HAL_GetTick();
		}

		if (HAL_GetTick() - delay_start > ch_sensor.det_interval) {
			delay_start = 0;
			ret = TCA6408A_ReadInput(ch_sensor.tca_dev);
		}
	} else {
		ret = TCA6408A_ReadInput(ch_sensor.tca_dev);
	}

	ch_sensor.status[0] =
			ch_sensor.tca_dev->regs.Input.bit.P4 == RESET ?
					SENSOR_ONLINE : SENSOR_OFFLINE;
	ch_sensor.status[1] =
			ch_sensor.tca_dev->regs.Input.bit.P5 == RESET ?
					SENSOR_ONLINE : SENSOR_OFFLINE;
	ch_sensor.status[2] =
			ch_sensor.tca_dev->regs.Input.bit.P6 == RESET ?
					SENSOR_ONLINE : SENSOR_OFFLINE;
	ch_sensor.status[3] =
			ch_sensor.tca_dev->regs.Input.bit.P7 == RESET ?
					SENSOR_ONLINE : SENSOR_OFFLINE;

#ifdef CIRCUIT_TEST
	static uint8_t Test_12v_Index = 0;
	static uint8_t Test_24v_Index = 0;
	static uint8_t Test_Relay_Index = 0;
	static uint8_t Test_12v_Status = 0;
	static uint8_t Test_24v_Status = 0;
	static uint8_t Test_Relay_Status = 0;

	if(ch_sensor.status[0] == SENSOR_ONLINE && Test_12v_Status == 0)
	{
		ch_supply.ch_no = ((Test_12v_Index++)<4) ? (Test_12v_Index-1) : ((Test_12v_Index=1)==0);
		ch_supply.mode = CH_PWR_12V;
		Ch_SetSupply(&ch_supply);
		Test_12v_Status = 1;
	}
	else if(ch_sensor.status[0] == SENSOR_OFFLINE && Test_12v_Status == 1)
	{
		ch_supply.mode = CH_PWR_0V;
		Ch_SetSupply(&ch_supply);
		Test_12v_Status = 0;
	}

	if(ch_sensor.status[1] == SENSOR_ONLINE && Test_24v_Status == 0)
	{
		ch_supply.ch_no = ((Test_24v_Index++)<4) ? (Test_24v_Index-1) : ((Test_24v_Index=1)==0);
		ch_supply.mode = CH_PWR_24V;
		Ch_SetSupply(&ch_supply);
		Test_24v_Status = 1;
	}
	else if(ch_sensor.status[1] == SENSOR_OFFLINE && Test_24v_Status == 1)
	{
		ch_supply.mode = CH_PWR_0V;
		Ch_SetSupply(&ch_supply);
		Test_24v_Status = 0;
	}

	if(ch_sensor.status[2] == SENSOR_ONLINE && Test_Relay_Status == 0)
	{
		ch_circuit.ch_no = ((Test_Relay_Index++)<4) ? (Test_Relay_Index-1) : ((Test_Relay_Index=1)==0);

		if(ch_circuit.ch_no == 0)
			ch_circuit.mode = (ch_circuit.mode==CH_PWR_VOLT) ? (CH_PWR_CURR) : (CH_PWR_VOLT);

		Ch_CircuitSwitch(&ch_circuit);
		Test_Relay_Status = 1;
	}
	else if(ch_sensor.status[2] == SENSOR_OFFLINE && Test_Relay_Status == 1)
	{
		Test_Relay_Status = 0;
	}
#endif

	return ret;
}

