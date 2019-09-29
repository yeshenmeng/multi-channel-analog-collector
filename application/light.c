#include "light.h"

light_ctrl_t light_ctrl;

light_ctrl_t* Light_Init(void)
{
	light_ctrl.green_status = LIGHT_ON;
	light_ctrl.green_flash_freq = 0;

	light_ctrl.red_status = LIGHT_ON;
	light_ctrl.red_flash_freq = 0;

	light_ctrl.sync_status = LIGHT_SYNC;
	light_ctrl.lightControl = Light_Control;

	Light_Control();

	return &light_ctrl;
}

void Light_GreenSet(uint32_t flash_freq, light_status status,
					light_sync_status sync_status)
{
	if (flash_freq != 0) {
		uint32_t tick;
		static uint32_t freq_cnt = 0;

		tick = HAL_GetTick() / flash_freq;

		if (tick > freq_cnt) {
			if (tick % 2 == 0) {
				LIGHT_GREEN_ON;

				if (sync_status == LIGHT_SYNC) {
					LIGHT_RED_ON;
				} else if (sync_status == LIGHT_ALTERNATE) {
					LIGHT_RED_OFF;
				}
			} else {
				LIGHT_GREEN_OFF;

				if (sync_status == LIGHT_SYNC) {
					LIGHT_RED_OFF;
				} else if (sync_status == LIGHT_ALTERNATE) {
					LIGHT_RED_ON;
				}
			}
		}

		freq_cnt = tick;
	} else if (status == LIGHT_OFF) {
		LIGHT_GREEN_OFF;

		if (sync_status == LIGHT_SYNC) {
			LIGHT_RED_OFF;
		} else if (sync_status == LIGHT_ALTERNATE) {
			LIGHT_RED_ON;
		}
	} else if (status == LIGHT_ON) {
		LIGHT_GREEN_ON;

		if (sync_status == LIGHT_SYNC) {
			LIGHT_RED_ON;
		} else if (sync_status == LIGHT_ALTERNATE) {
			LIGHT_RED_OFF;
		}
	}
}

void Light_RedSet(uint32_t flash_freq, light_status status,
				  light_sync_status sync_status) {
	if (sync_status != LIGHT_ASYNC)
		return;

	if (flash_freq != 0) {
		uint32_t tick;
		static uint32_t freq_cnt = 0;

		tick = HAL_GetTick() / flash_freq;

		if (tick > freq_cnt) {
			if (tick % 2 == 0) {
				LIGHT_RED_ON;
			} else {
				LIGHT_RED_OFF;
			}
		}

		freq_cnt = tick;
	} else if (status == LIGHT_OFF) {
		LIGHT_RED_OFF;
	} else if (status == LIGHT_ON) {
		LIGHT_RED_ON;
	}
}

void Light_Control(void)
{
	Light_GreenSet(light_ctrl.green_flash_freq, light_ctrl.green_status,
				   light_ctrl.sync_status);

	Light_RedSet(light_ctrl.red_flash_freq, light_ctrl.red_status,
				 light_ctrl.sync_status);

	if(comm_indicate_flag != 0)
	{
		if(comm_indicate_flag == 1)
		{
			light_ctrl.green_flash_freq = 50;
		}
		else if(comm_indicate_flag == 2)
		{
			light_ctrl.green_flash_freq = 200;
		}

		static uint32_t tick_start = 0;
		if(tick_start == 0)
		{
			tick_start = HAL_GetTick();
		}

		if(HAL_GetTick() - tick_start > 1000)
		{
			tick_start = 0;
			comm_indicate_flag = 0;
			light_ctrl.green_flash_freq = 1000;
		}
	}
}

