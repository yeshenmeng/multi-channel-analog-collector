/********************************************************************************
  * �ļ����� flash.c 
  * ������   flash��д����
  * ��汾�� V3.5.0
  * ���ڣ�   2015.8.10
  * ���ߣ�   Steven
  * ���£�
  * 	1. �޸�ΪHAL���д�� ����� 2019.2.14)
  * 
  *
********************************************************************************/  

/* Includes ------------------------------------------------------------------*/
#include "flash.h"

static FLASH_EraseInitTypeDef _eraseConfig;

/*******************************************************************************
* Function Name  : flash_write
* Description    : flash��д����
* Input          : u32 pageStartAddr: ��дflashҳ����ʼ��ַ
* 				   u32 *p_data: ��ҳ��д�����������
* 				   u32 size: д�����ݵĸ���
* Return         : 0: ��ȷִ��,
* 				   1: ��������
* 				   2: write error
* Attention		 : ��������һ����u32 ��ָ�룬������һ���ǰ���4�ֽڶ���д��ġ����ԣ�sizeҲ��u32�ĸ������ֽ�����4��֮һ��
*******************************************************************************/
uint8_t flash_write(uint32_t pageStartAddr, uint32_t *pData, uint32_t size)
{
	uint32_t endAddr = pageStartAddr + 4*size;
	uint32_t eraseErr = 0;

	_eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
	_eraseConfig.PageAddress = pageStartAddr;
	_eraseConfig.NbPages = ((endAddr - pageStartAddr) >> TH_FLASH_PAGE_SIZE_2_POWER) + 1;

	HAL_FLASH_Unlock();          //��������

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
* Description    : flash�Ķ�����
* Input          : u32 startAddr,u32 *p_data,u32 size
* Attention		 : ��������һ����u32 ��ָ�룬������һ���ǰ���4�ֽڶ���д��ġ����ԣ�sizeҲ��u32�ĸ������ֽ�����4��֮һ��
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
