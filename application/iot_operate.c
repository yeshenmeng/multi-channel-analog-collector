#include "iot_operate.h"
#include "stdlib.h"
#include "main.h"


IoT_dev_t IoT_dev;

uint8_t IoT_SetChStatus(uint8_t ch_no, ch_status status)
{
	if(ch_no >= IoT_dev.sys_param->ch_num)
	{
		return IOT_OPERATE_INVALID;
	}

	if(IoT_dev.sys_param->prop[ch_no].status != status)
	{
		IoT_dev.sys_param->update_flag = 1;
	}

	IoT_dev.sys_param->prop[ch_no].status = status;
	IoT_dev.ad_dev->update_flag = 1;

	if(status == CH_DISABLE)
	{
		IoT_dev.ch_supply->ch_no = ch_no;
		IoT_dev.ch_supply->mode = CH_PWR_0V;

		return Ch_SetSupply(IoT_dev.ch_supply);
	}

	return IOT_OPERATE_SUCCESS;
}

uint8_t IoT_SetChType(uint8_t ch_no, ch_type type)
{
	if(ch_no >= IoT_dev.sys_param->ch_num)
	{
		return IOT_OPERATE_INVALID;
	}

	if(IoT_dev.sys_param->prop[ch_no].type != type)
	{
		IoT_dev.sys_param->update_flag = 1;
	}

	IoT_dev.sys_param->prop[ch_no].type = type;
	IoT_dev.ad_dev->update_flag = 1;

	IoT_dev.ch_circuit->ch_no = ch_no;
	if(type == VOTL_0_5V || type == VOLT_0_10V)
	{
		IoT_dev.ch_circuit->mode = CH_PWR_VOLT;
	}
	else if(type == CURR_4_20MA)
	{
		IoT_dev.ch_circuit->mode = CH_PWR_CURR;
	}

	return Ch_CircuitSwitch(IoT_dev.ch_circuit);
}

uint8_t IoT_SetChVoltage(uint8_t ch_no, ch_pwr_volt pwr_volt)
{
	if(ch_no >= IoT_dev.sys_param->ch_num)
	{
		return IOT_OPERATE_INVALID;
	}

	if(IoT_dev.sys_param->prop[ch_no].pwr_volt != pwr_volt)
	{
		IoT_dev.sys_param->update_flag = 1;
	}

	IoT_dev.sys_param->prop[ch_no].pwr_volt = pwr_volt;
	IoT_dev.ch_supply->ch_no = ch_no;

	if(pwr_volt == PWR_VOLT_12V)
	{
		IoT_dev.ch_supply->mode = CH_PWR_12V;
	}
	else if(pwr_volt == PWR_VOLT_24V)
	{
		IoT_dev.ch_supply->mode = CH_PWR_24V;
	}

	if(IoT_dev.sys_param->prop[ch_no].status == CH_ENABLE)
	{
		return Ch_SetSupply(IoT_dev.ch_supply);
	}

	return IOT_OPERATE_FAIL;
}

uint8_t IoT_SetChDelay(uint8_t ch_no, uint8_t delay)
{
	if(ch_no >= IoT_dev.sys_param->ch_num)
	{
		return IOT_OPERATE_INVALID;
	}

	if(IoT_dev.sys_param->prop[ch_no].init_delay != delay)
	{
		IoT_dev.sys_param->update_flag = 1;
	}

	IoT_dev.sys_param->prop[ch_no].init_delay = delay;
	IoT_dev.init_delay_flag[ch_no] = 1;

	return IOT_OPERATE_SUCCESS;
}

uint8_t IoT_SetDevWrProt(uint32_t value)
{
	if(value == 0X12345678)
	{
		write_protect_flag = 1;
	}
	else if(value == 0X87654321)
	{
		write_protect_flag = 0;
	}

	return IOT_OPERATE_SUCCESS;
}

void IoT_WriteValue(uint8_t ch_no, float value)
{
	uint8_t buf[4];
	float *pData = (float *)buf;

	*pData = value;
	IoT_dev.sensor->writePropFromBuf(CHL_VALUE_ID(ch_no), buf);
}

void IoT_WriteErrorInfo(uint8_t err_msg)
{
	IoT_dev.sensor->writePropFromBuf(CHL_ERROR_ID, (uint8_t *)&err_msg);
}

void IoT_WriteBatteryLevel(uint8_t batter_power)
{
	IoT_dev.sensor->writePropFromBuf(BATTERY_LEVEL_ID, (uint8_t *)&batter_power);
}

void IoT_WriteEnable(uint8_t enable)
{
	IoT_dev.sensor->writePropFromBuf(CHL_ENABLE_ID, (uint8_t *)&enable);
}

void IoT_WriteType(uint8_t ch_no, uint8_t type)
{
	IoT_dev.sensor->writePropFromBuf(CHL_TYPE_ID(ch_no), (uint8_t *)&type);
}

void IoT_WriteSupply(uint8_t ch_no, uint8_t type)
{
	IoT_dev.sensor->writePropFromBuf(CHL_VOLT_TYPE_ID(ch_no), (uint8_t *)&type);
}

void IoT_WriteDelay(uint8_t ch_no, uint8_t delay)
{
	IoT_dev.sensor->writePropFromBuf(CHL_DELAY_ID(ch_no), (uint8_t *)&delay);
}

void IoT_WriteCali(uint8_t ch_no, uint8_t mode)
{
	IoT_dev.sensor->writePropFromBuf(CHL_CALI_ID(ch_no), (uint8_t *)&mode);
}

uint8_t IoT_Operate(void)
{
	uint8_t ret = IOT_OPERATE_SUCCESS;

	if(IoT_dev.sensor->isPropChanged(DEV_WR_PROT_ID))
	{
		uint32_t devWrProt = 0;
		IoT_dev.sensor->readPropToBuf(DEV_WR_PROT_ID, (uint8_t *)&devWrProt);
		IoT_dev.sensor->resetPropChangeFlag(DEV_WR_PROT_ID);
		IoT_SetDevWrProt(devWrProt);
	}

	if(IoT_dev.sensor->isPropChanged(CHL_ENABLE_ID))
	{
		uint8_t ch_enable;
		IoT_dev.sensor->readPropToBuf(CHL_ENABLE_ID, (uint8_t *)&ch_enable);
		IoT_dev.sensor->resetPropChangeFlag(CHL_ENABLE_ID);

		if(write_protect_flag == 1)
		{
			for(int i = 0; i < IoT_dev.sys_param->ch_num; i++)
			{
				ret = IoT_SetChStatus(i, (ch_status)((ch_enable>>i)&0X01));
			}
		}
	}

	for(int i = 0; i < IoT_dev.sys_param->ch_num; i++)
	{
		if(IoT_dev.sensor->isPropChanged(CHL_TYPE_ID(i)))
		{
			uint8_t type;
			IoT_dev.sensor->readPropToBuf(CHL_TYPE_ID(i), (uint8_t *)&type);
			IoT_dev.sensor->resetPropChangeFlag(CHL_TYPE_ID(i));

			if(write_protect_flag == 1)
			{
				ret = IoT_SetChType(i, (ch_type)type);
			}
		}

		if(IoT_dev.sensor->isPropChanged(CHL_VOLT_TYPE_ID(i)))
		{
			uint8_t pwr_volt;
			IoT_dev.sensor->readPropToBuf(CHL_VOLT_TYPE_ID(i), (uint8_t *)&pwr_volt);
			IoT_dev.sensor->resetPropChangeFlag(CHL_VOLT_TYPE_ID(i));

			if(write_protect_flag == 1)
			{
				ret = IoT_SetChVoltage(i, (ch_pwr_volt)pwr_volt);
			}
		}

		if(IoT_dev.sensor->isPropChanged(CHL_DELAY_ID(i)))
		{
			uint8_t delay;
			IoT_dev.sensor->readPropToBuf(CHL_DELAY_ID(i), (uint8_t *)&delay);
			IoT_dev.sensor->resetPropChangeFlag(CHL_DELAY_ID(i));

			if(write_protect_flag == 1)
			{
				ret = IoT_SetChDelay(i, delay);
			}
		}

		if(IoT_dev.sensor->isPropChanged(CHL_CALI_ID(i)))
		{
			uint8_t mode;
			IoT_dev.sensor->readPropToBuf(CHL_CALI_ID(i), (uint8_t *)&mode);
			IoT_dev.sensor->resetPropChangeFlag(CHL_CALI_ID(i));

			if(write_protect_flag == 1)
			{
				ret = AD717X_Cali(i, (ad717x_cali_mode)mode);
			}
		}

		if(IoT_dev.ad_dev->data[i].update == 1)
		{
			IoT_dev.ad_dev->data[i].update = 0;

			if(IoT_dev.init_delay_flag[i] == 0)
			{
#ifdef DEBUG
				IoT_WriteValue(i, IoT_dev.ad_dev->data[i].value);
#else
				IoT_WriteValue(i, IoT_dev.ad_dev->data[i].percent);
#endif
			}
		}
	}

	if(IoT_dev.err_flag == 1)
	{
		IoT_dev.err_flag = 0;
		IoT_WriteErrorInfo(IoT_dev.err_msg);
	}

	if(IoT_dev.gas_gauge_flag == 1)
	{
		IoT_dev.gas_gauge_flag = 0;
		IoT_WriteBatteryLevel(IoT_dev.gas_gauge);
	}

	if(IoT_dev.ad_dev->update_flag == 1)
	{
		IoT_dev.ad_dev->update_flag = 0;
		ret = AD717X_ChangeAdcDev(IoT_dev.sys_param);
	}

	for(int i = 0; i < IoT_dev.sys_param->ch_num; i++)
	{
		if(IoT_dev.init_delay_flag[i] == 1)
		{
			static uint32_t delay_start[4] = {0};

			if(delay_start[i] == 0)
			{
				delay_start[i] = HAL_GetTick();
			}

			if((HAL_GetTick() - delay_start[i]) > IoT_dev.sys_param->prop[i].init_delay * 1000)
			{
				delay_start[i] = 0;
				IoT_dev.init_delay_flag[i] = 0;
			}
		}
	}

	return ret;
}

IoT_dev_t * IoT_Init(sys_param_t *param,
					 ch_supply_t *ch_supply,
					 ch_circuit_t *ch_circuit,
					 iot_object_t *sensor,
					 ad717x_dev *ad_dev)
{
	IoT_dev.err_flag = 0;
	IoT_dev.gas_gauge_flag = 0;
	IoT_dev.err_msg = 0;
	IoT_dev.gas_gauge = 100;

	IoT_dev.sys_param = param;
	IoT_dev.ch_supply = ch_supply;
	IoT_dev.ch_circuit = ch_circuit;
	IoT_dev.sensor = sensor;
	IoT_dev.ad_dev = ad_dev;
	IoT_dev.operate = IoT_Operate;

	IoT_dev.init_delay_flag = (uint8_t *)malloc(param->ch_num);

	uint8_t ch_enable = 0;
	for(int32_t i = 0; i < param->ch_num; i++)
	{
		IoT_dev.init_delay_flag[i] = 0;
		if(param->prop[i].init_delay != 0)
		{
			IoT_dev.init_delay_flag[i] = 1;
		}

		if(param->prop[i].status == CH_ENABLE)
		{
			ch_enable |= (1 << i);
		}

		IoT_WriteType(i, param->prop[i].type);
		IoT_dev.sensor->resetPropChangeFlag(CHL_TYPE_ID(i));

		IoT_WriteSupply(i, param->prop[i].pwr_volt);
		IoT_dev.sensor->resetPropChangeFlag(CHL_VOLT_TYPE_ID(i));

		IoT_WriteDelay(i, param->prop[i].init_delay);
		IoT_dev.sensor->resetPropChangeFlag(CHL_DELAY_ID(i));

		IoT_WriteCali(i, param->cali[i].mode);
		IoT_dev.sensor->resetPropChangeFlag(CHL_CALI_ID(i));
	}

	IoT_WriteEnable(ch_enable);
	IoT_dev.sensor->resetPropChangeFlag(CHL_ENABLE_ID);

	IoT_WriteErrorInfo(IoT_dev.err_msg);
	IoT_dev.sensor->resetPropChangeFlag(CHL_ERROR_ID);

	IoT_WriteBatteryLevel(IoT_dev.gas_gauge);
	IoT_dev.sensor->resetPropChangeFlag(BATTERY_LEVEL_ID);

	return &IoT_dev;
}



