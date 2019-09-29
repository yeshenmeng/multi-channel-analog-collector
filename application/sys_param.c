#include "main.h"
#include "sys_param.h"
#include "flash.h"

sys_param_t sys_param;

sys_param_t* Sys_ParamInit(void)
{
	flash_read(SYS_PARAM_FLASH_PAGE_ADDR, (uint8_t *)&sys_param, sizeof(sys_param));

	if(sys_param.update_flag == 0XFF)
	{
		sys_param.ch_num = SYS_CH_NUM;
		sys_param.ch_num = 4;

		for(int i = 0; i < sys_param.ch_num; i++)
		{
			sys_param.prop[i].status = CH_ENABLE;
			sys_param.prop[i].type = CURR_4_20MA;
			sys_param.prop[i].pwr_volt = PWR_VOLT_24V;
			sys_param.prop[i].init_delay = 3;

			sys_param.cali[i].mode = 0;
			sys_param.cali[i].offset = 0XFFFFFFFF;
			sys_param.cali[i].gain = 0XFFFFFFFF;
		}
	}

	sys_param.update_flag = 0;
	sys_param.saveParamToFlash = Sys_SaveParamToFlash;

#ifdef CIRCUIT_TEST
	sys_param.ch_num = SYS_CH_NUM;
	sys_param.ch_num = 4;

	for(int i = 0; i < sys_param.ch_num; i++)
	{
		sys_param.prop[i].status = CH_ENABLE;
//		sys_param.prop[i].status = CH_DISABLE;
		sys_param.prop[i].type = VOLT_0_10V;
		sys_param.prop[i].pwr_volt = PWR_VOLT_24V;
		sys_param.prop[i].init_delay = 3;

		sys_param.cali[i].mode = 0;
		sys_param.cali[i].offset = 0XFFFFFFFF;
		sys_param.cali[i].gain = 0XFFFFFFFF;
	}
#endif

	return &sys_param;
}

uint8_t Sys_SaveParamToFlash(void)
{
#ifdef CIRCUIT_TEST
	return 0;
#endif

	if(sys_param.update_flag == 1)
	{
		sys_param.update_flag = 0;
		return flash_write(SYS_PARAM_FLASH_PAGE_ADDR,
						   (uint32_t *)&sys_param,
						   sizeof(sys_param)%4==0?sizeof(sys_param)/4:(sizeof(sys_param)/4+1));
	}

	return 0;
}


