#include "convert.h"
#include <ctype.h>
#include <stdio.h> 
#include <string.h> 

/**
 * @brief  ʮ�������ַ���ת��Ϊ�ֽ���
 *         exp:Hchar[]="AD12E3C5"-->Dchar[]={0XAD,0X12,0XE3,0XC5}
 * @param  source: Դʮ�������ַ���
 * @param  dest: Ŀ���ֽ�������
 * @param  srcLen: �ַ������� strlen(source)
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
 * @brief  �ֽ�������ת��Ϊʮ�������ַ���
 *         exp:Dchar[]={0XAD,0X12,0XE3,0XC5}-->Hchar[]="AD12E3C5"
 * @param  source: Դ�ֽ�������
 * @param  dest: Ŀ��ʮ�������ַ���
 * @param  srcLen: �ֽ������� sizeof(source)
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



