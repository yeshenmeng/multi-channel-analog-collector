/********************************************************************************
  * 文件名： flash.c 
  * 描述：   flash读写操作
  * 库版本： V3.5.0
  * 日期：   2015.8.10
  * 作者：   Steven
  * 更新：
  * 	1. 修改为HAL库的写法 （徐辉 2019.2.14)
  * 
  *
********************************************************************************/  

/* Includes ------------------------------------------------------------------*/
#include "flash.h"

static FLASH_EraseInitTypeDef _eraseConfig;

/*******************************************************************************
* Function Name  : flash_write
* Description    : flash的写函数
* Input          : u32 pageStartAddr: 待写flash页的起始地址
* 				   u32 *p_data: 本页待写入的连续数据
* 				   u32 size: 写入数据的个数
* Return         : 0: 正确执行,
* 				   1: 擦除错误
* 				   2: write error
* Attention		 : 输入数据一定是u32 的指针，即数据一定是按照4字节对齐写入的。所以：size也是u32的个数（字节数的4分之一）
*******************************************************************************/
uint8_t flash_write(uint32_t pageStartAddr, uint32_t *pData, uint32_t size)
{
	uint32_t endAddr = pageStartAddr + 4*size;
	uint32_t eraseErr = 0;

	_eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
	_eraseConfig.PageAddress = pageStartAddr;
	_eraseConfig.NbPages = ((endAddr - pageStartAddr) >> TH_FLASH_PAGE_SIZE_2_POWER) + 1;

	HAL_FLASH_Unlock();          //解锁函数

	if(HAL_FLASHEx_Erase(&_eraseConfig, &eraseErr) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return 1;
	}

	uint32_t i;
	for(i = 0; i < size; i++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageStartAddr, pData[i]) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return 2;
		}
		else
			pageStartAddr += 4;
	}

	HAL_FLASH_Lock();
	return 0;
}

void flash_read(uint32_t startAddr,uint8_t *pData,uint8_t size)
{
	uint8_t i;
	for(i = 0; i<size; i++)
	{
		pData[i] = *((uint8_t*) startAddr);
		startAddr++;
	}
}

/*******************************************************************************
* Function Name  : flash_read
* Description    : flash的读函数
* Input          : u32 startAddr,u32 *p_data,u32 size
* Attention		 : 输入数据一定是u32 的指针，即数据一定是按照4字节对齐写入的。所以：size也是u32的个数（字节数的4分之一）
*******************************************************************************/
/*
void flash_read(uint32_t startAddr,uint32_t *pData,uint32_t size)
{
	uint32_t i;
	for(i = 0; i<size; i++)
	{
		pData[i] = *((uint32_t*) startAddr);
		startAddr += 4;
	}
}
*/


/********************** (C) COPYRIGHT 2019 TOHOLED *****END OF FILE*********************/
