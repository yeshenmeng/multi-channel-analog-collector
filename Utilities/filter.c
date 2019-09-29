#include <stdlib.h>
#include "filter.h"

/* @param：convert数据类型转换
 * exp：	float UInt32_Convert(void* value)
 {
 return *(uint32_t *)(value);
 }
 */
/**
 * @brief  数据均值滤波
 * @param  pData: 数据
 * @param  size: 数据大小
 * @param  width: 数据单位
 * @param  convert: 数据类型转换接口
 * 			                       例如待求均值的数据是浮点型：float Float_Convert(void* value) {return *(float *)(value);}
 * @retval 平均值
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

