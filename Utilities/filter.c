#include <stdlib.h>
#include "filter.h"

/* @param��convert��������ת��
 * exp��	float UInt32_Convert(void* value)
 {
 return *(uint32_t *)(value);
 }
 */
/**
 * @brief  ���ݾ�ֵ�˲�
 * @param  pData: ����
 * @param  size: ���ݴ�С
 * @param  width: ���ݵ�λ
 * @param  convert: ��������ת���ӿ�
 * 			                       ��������ֵ�������Ǹ����ͣ�float Float_Convert(void* value) {return *(float *)(value);}
 * @retval ƽ��ֵ
 */
float Filter_Mean(void *pData, int size, int width, float convert(void* value)) {
	float sum = 0;
	unsigned char *pChar = pData;

	for (int i = 0; i < size; i++) {
		sum += convert(pChar);
		pChar += width;
	}

	return sum / size;
}

