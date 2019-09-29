#include "convert.h"
#include <ctype.h>
#include <stdio.h> 
#include <string.h> 

/**
 * @brief  十六进制字符串转换为字节流
 *         exp:Hchar[]="AD12E3C5"-->Dchar[]={0XAD,0X12,0XE3,0XC5}
 * @param  source: 源十六进制字符串
 * @param  dest: 目标字节流数组
 * @param  srcLen: 字符串长度 strlen(source)
 * @retval None
 */
void Convert_HexStringToByte(const char* source, unsigned char* dest, int srcLen)
{
    unsigned char highByte;
	unsigned char lowByte;

    for (short i = 0; i < srcLen; i += 2)
    {
        highByte = toupper(source[i]);
        lowByte  = toupper(source[i + 1]);

        if (highByte > 0x39)
            highByte -= 0x37;
        else
            highByte -= 0x30;

        if (lowByte > 0x39)
            lowByte -= 0x37;
        else
            lowByte -= 0x30;

        dest[i/2] = (highByte << 4) | lowByte;
    }
}

/**
 * @brief  字节流数组转换为十六进制字符串
 *         exp:Dchar[]={0XAD,0X12,0XE3,0XC5}-->Hchar[]="AD12E3C5"
 * @param  source: 源字节流数组
 * @param  dest: 目标十六进制字符串
 * @param  srcLen: 字节流数组 sizeof(source)
 * @retval None
 */
void Convert_ByteToHexString(const char *src,  char *dest, int srcLen)
{
    char tmp[3];

    for(int i = 0; i < srcLen; i++)
    {
        sprintf(tmp, "%02X", (unsigned char)src[i]);
        memcpy(&dest[i*2], tmp, 2);
    }

    return ;
}



