#include <stdlib.h>
#include "data_sort.h"

int Compare_Small2Big_32(const void *data1, const void *data2)
{
	return *(int*)data1 - *(int*)data2;
}

int Compare_Big2Small_32(const void *data1, const void *data2)
{
	return *(int*)data2 - *(int*)data1;
}

int Compare_Small2Big_16(const void *data1, const void *data2)
{
	return *(int16_t*)data1 - *(int16_t*)data2;
}

int Compare_Big2Small_16(const void *data1, const void *data2)
{
	return *(int16_t*)data2 - *(int16_t*)data1;
}

int Compare_Small2Big_8(const void *data1, const void *data2)
{
	return *(int8_t*)data1 - *(int8_t*)data2;
}

int Compare_Big2Small_8(const void *data1, const void *data2)
{
	return *(int8_t*)data2 - *(int8_t*)data1;
}

/**
 * @brief  ��������
 *         ֧��8λ��16λ��32λ������������
 * @param  pData: ����
 * @param  size: ���ݳ���
 * @param  width: ���ݵ�λ
 * @param  dir: 0:��С����  1:�Ӵ�С
 * @retval None
 */
void Data_Sort(void *pData, int size, int width, uint8_t dir)
{
	if(width == 4)
	{
		if(!dir)
			qsort(pData, size, width, Compare_Small2Big_32);
		else
			qsort(pData, size, width, Compare_Big2Small_32);
	}
	else if(width == 2)
	{
		if(!dir)
			qsort(pData, size, width, Compare_Small2Big_16);
		else
			qsort(pData, size, width, Compare_Big2Small_16);
	}
	else if(width == 1)
	{
		if(!dir)
			qsort(pData, size, width, Compare_Small2Big_8);
		else
			qsort(pData, size, width, Compare_Big2Small_8);
	}
}






