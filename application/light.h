#ifndef __LIGHT_H__
#define __LIGHT_H__
#include "main.h"

#define LIGHT_GREEN_ON		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define LIGHT_GREEN_OFF		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define LIGHT_RED_ON		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)
#define LIGHT_RED_OFF		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)

typedef enum {
	LIGHT_OFF, LIGHT_ON
} light_status;

typedef enum {
	LIGHT_ASYNC,		//红绿灯异步状态
	LIGHT_SYNC,			//红绿灯同步状态
	LIGHT_ALTERNATE		//红绿灯交替状态
} light_sync_status;

typedef struct {
	uint32_t green_flash_freq;
	uint32_t red_flash_freq;
	light_status green_status;
	light_status red_status;

	//红绿灯状态，如果设置为同步或者交替状态则以绿灯设置的状态为准
	light_sync_status sync_status;

	void (*lightControl)(void);
} light_ctrl_t;

light_ctrl_t* Light_Init(void);
void Light_Control(void);

#endif
