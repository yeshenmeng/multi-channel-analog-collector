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
	LIGHT_ASYNC,		//���̵��첽״̬
	LIGHT_SYNC,			//���̵�ͬ��״̬
	LIGHT_ALTERNATE		//���̵ƽ���״̬
} light_sync_status;

typedef struct {
	uint32_t green_flash_freq;
	uint32_t red_flash_freq;
	light_status green_status;
	light_status red_status;

	//���̵�״̬���������Ϊͬ�����߽���״̬�����̵����õ�״̬Ϊ׼
	light_sync_status sync_status;

	void (*lightControl)(void);
} light_ctrl_t;

light_ctrl_t* Light_Init(void);
void Light_Control(void);

#endif
