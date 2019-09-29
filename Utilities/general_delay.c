#include "general_delay.h"

void delay_nus(uint32_t time)
{
	uint8_t i=0;
	while(time--)
	{
		i = 6;
		while(i--);
	}
}

void delay_nms(uint32_t time)
{
	while(time--)
	{
		delay_nus(1000);
	}
}
