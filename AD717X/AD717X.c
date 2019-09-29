/***************************************************************************//**
 *   @file    AD717X.c
 *   @brief   AD717X implementation file.
 *   @devices AD7172-2, AD7172-4, AD7173-8, AD7175-2, AD7175-8, AD7176-2
 *            AD7177-2
 *   @author  acozma (andrei.cozma@analog.com)
 *            dnechita (dan.nechita@analog.com)
 *
 ********************************************************************************
 * Copyright 2015(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "platform_drivers.h"
#include "ad717x.h"
#include "ad4111_regs.h"
#include "data_sort.h"
#include "filter.h"
#include "general_delay.h"

/* Error codes */
#define NULL_POINTER	(void *)0
#define INVALID_VAL 	-1 /* Invalid argument */
#define COMM_ERR    	-2 /* Communication error on receive */
#define TIMEOUT     	-3 /* A timeout has occured */

static ad717x_dev ad_dev;

/***************************************************************************//**
 * @brief  Searches through the list of registers of the driver instance and
 *         retrieves a pointer to the register that matches the given address.
 *
 * @param device - The handler of the instance of the driver.
 * @param reg_address - The address to be used to find the register.
 *
 * @return A pointer to the register if found or 0.
 *******************************************************************************/
ad717x_st_reg *AD717X_GetReg(ad717x_dev *device, uint8_t reg_address) {
	uint8_t i;
	ad717x_st_reg *reg = 0;

	if (!device || !device->regs)
		return 0;

	for (i = 0; i < device->num_regs; i++) {
		if (device->regs[i].addr == reg_address) {
			reg = &device->regs[i];
			break;
		}
	}

	return reg;
}

/***************************************************************************//**
 * @brief Sets the value of the specified register.
 *
 * @param device - The handler of the instance of the driver.
 * @param reg_address - The address to be used to find the register.
 * @param value - The value to be set for the register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X_SetReg(ad717x_dev *device, uint8_t reg_address, int32_t value) {
	ad717x_st_reg *reg = 0;

	reg = AD717X_GetReg(device, reg_address);
	if (!reg)
		return INVALID_VAL;

	reg->value = value;

	return 0;
}

ad717x_rdy_status AD717X_CheckRDY(void) {
	return (ad717x_rdy_status) HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
}

/***************************************************************************//**
 * @brief Reads the value of the specified register.
 *
 * @param device - The handler of the instance of the driver.
 * @addr - The address of the register to be read. The value will be stored
 *         inside the register structure that holds info about this register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X_ReadRegister(ad717x_dev *device, uint8_t addr) {
	int32_t ret = 0;
	uint8_t wrBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rdBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t check8 = 0;
	uint8_t msgBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	ad717x_st_reg *pReg;

	if (!device)
		return INVALID_VAL;

	pReg = AD717X_GetReg(device, addr);
	if (!pReg)
		return INVALID_VAL;

	/* Build the Command word */
	wrBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD
			| AD717X_COMM_REG_RA(pReg->addr);

	/* Read data from the device */
	ret = spi_write_and_read(device->spi_desc, wrBuf, rdBuf,
			((device->useCRC != AD717X_DISABLE) ? pReg->size + 1 : pReg->size)
					+ 1);

	if (ret < 0)
		return ret;

	/* Check the CRC */
	if (device->useCRC == AD717X_USE_CRC) {
		msgBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD
				| AD717X_COMM_REG_RA(pReg->addr);
		for (int i = 1; i < pReg->size + 2; ++i) {
			msgBuf[i] = rdBuf[i];
		}
		check8 = AD717X_ComputeCRC8(msgBuf, pReg->size + 2);
	}
	if (device->useCRC == AD717X_USE_XOR) {
		msgBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD
				| AD717X_COMM_REG_RA(pReg->addr);
		for (int i = 1; i < pReg->size + 2; ++i) {
			msgBuf[i] = rdBuf[i];
		}
		check8 = AD717X_ComputeXOR8(msgBuf, pReg->size + 2);
	}

	if (check8 != 0) {
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/* Build the result */
	pReg->value = 0;
	for (int i = 1; i < pReg->size + 1; i++) {
		pReg->value <<= 8;
		pReg->value += rdBuf[i];
	}

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register.
 *
 * @param device - The handler of the instance of the driver.
 * @param addr   - The address of the register to be written with the value stored
 *               inside the register structure that holds info about this
 *               register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X_WriteRegister(ad717x_dev *device, uint8_t addr) {
	int32_t ret = 0;
	int32_t regValue = 0;
	uint8_t wrBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rdBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i = 0;
	uint8_t crc8 = 0;
	ad717x_st_reg *preg;

	if (!device)
		return INVALID_VAL;

	preg = AD717X_GetReg(device, addr);
	if (!preg)
		return INVALID_VAL;

	/* Build the Command word */
	wrBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_WR
			| AD717X_COMM_REG_RA(preg->addr);

	/* Fill the write buffer */
	regValue = preg->value;
	for (i = 0; i < preg->size; i++) {
		wrBuf[preg->size - i] = regValue & 0xFF;
		regValue >>= 8;
	}

	/* Compute the CRC */
	if (device->useCRC != AD717X_DISABLE) {
		crc8 = AD717X_ComputeCRC8(wrBuf, preg->size + 1);
		wrBuf[preg->size + 1] = crc8;
	}

	/* Write data to the device */
	ret = spi_write_and_read(device->spi_desc, wrBuf, rdBuf,
			(device->useCRC != AD717X_DISABLE) ?
					preg->size + 2 : preg->size + 1);

	return ret;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @param device - The handler of the instance of the driver.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
//desc：description
int32_t AD717X_Reset(ad717x_dev *device) {
	int32_t ret = 0;
	uint8_t wrBuf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t rdBuf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	if (!device)
		return INVALID_VAL;

	ret = spi_write_and_read(device->spi_desc, wrBuf, rdBuf, 8);

	delay_nus(500);

	return ret;
}

/***************************************************************************//**
 * @brief Waits until a new conversion result is available.
 *
 * @param device  - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns if no new data is available.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X_WaitForReady(ad717x_dev *device, uint32_t timeout) {
	ad717x_st_reg *statusReg;
	int32_t ret;
	int8_t ready = 0;

	if (!device || !device->regs)
		return INVALID_VAL;

	statusReg = AD717X_GetReg(device, AD717X_STATUS_REG);
	if (!statusReg)
		return INVALID_VAL;

	while (!ready && --timeout) {
		/* Read the value of the Status Register */
		ret = AD717X_ReadRegister(device, AD717X_STATUS_REG);
		if (ret < 0)
			return ret;

		/* Check the RDY bit in the Status Register */
		ready = (statusReg->value & AD717X_STATUS_REG_RDY) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Reads the conversion result from the device.
 *
 * @param device - The handler of the instance of the driver.
 * @param pData  - Pointer to store the read data.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X_ReadData(ad717x_dev *device, uint32_t* pData) {
	ad717x_st_reg *dataReg;
	int32_t ret;

	if (!device || !device->regs)
		return INVALID_VAL;

	dataReg = AD717X_GetReg(device, AD717X_DATA_REG);
	if (!dataReg)
		return INVALID_VAL;

	/* Update the data register length with respect to device and options */
	ret = AD717X_ComputeDataregSize(device);

	/* Read the value of the Data Register */
	ret |= AD717X_ReadRegister(device, AD717X_DATA_REG);

	/* Get the read result */
	*pData = dataReg->value;

	return ret;
}

float Convert_ToUint32(void* value) {
	return *(uint32_t *) (value);
}

/**
 * @brief  数据采样处理
 *         从ADC中读取数据，将数据分组到每个通道，数据经过滤波之后保存，同时设置数据更新标志
 * @param  None
 * @retval 0：成功
 */
int32_t AD717X_DataSample(void) {
	uint8_t ch;
	int32_t ret;
	uint32_t value;

	if (AD717X_CheckRDY() == AD_READY) {
		ret = AD717X_ReadData(&ad_dev, &value);
		if (ret < 0)
			return INVALID_VAL;

		ch = value & 0X0F;
		ad_dev.data[ch].data_buf[ad_dev.data[ch].count++] = value >> 8;
		if (ad_dev.data[ch].count >= AD717X_DATA_BUF_SIZE) {
			ad_dev.data[ch].full = 1;
			ad_dev.data[ch].count = 0;
		}

		if (ad_dev.data[ch].full) {
			Data_Sort(ad_dev.data[ch].data_buf, AD717X_DATA_BUF_SIZE,
					sizeof(int32_t), 0);
			ad_dev.data[ch].value = (float) ad_dev.dataFilter(
					&ad_dev.data[ch].data_buf[ad_dev.discard],
					AD717X_DATA_BUF_SIZE - ad_dev.discard * 2, sizeof(int),
					Convert_ToUint32);
		} else {
			ad_dev.data[ch].value = (float) ad_dev.dataFilter(&ad_dev.data[ch],
					ad_dev.data[ch].count, sizeof(int), Convert_ToUint32);
		}

		if (ad_dev.param->prop[ch].type == VOTL_0_5V
				|| ad_dev.param->prop[ch].type == VOLT_0_10V) {
			float refVolt;

			if (ad_dev.param->cali[ch].mode & SYS_GAIN_CALI) {
				refVolt =
						(ad_dev.param->prop[ch].type == VOTL_0_5V) ? 5 : 9.997;
				ad_dev.data[ch].value = ad_dev.data[ch].value * refVolt
						/ 16777216;

				if (ad_dev.data[ch].value >= 9.996) {
					ad_dev.data[ch].value = 10;
				}
			} else {
				refVolt = (ad_dev.param->prop[ch].type == VOTL_0_5V) ? 5 : 10;
				ad_dev.data[ch].value = ad_dev.data[ch].value * 2.5 / 1677721.6;
			}

			ad_dev.data[ch].percent = ad_dev.data[ch].value * 100 / refVolt;
		} else if (ad_dev.param->prop[ch].type == CURR_4_20MA) {
			ad_dev.data[ch].value = ad_dev.data[ch].value * 2.5 / 16777216 / 50;
			ad_dev.data[ch].percent = ad_dev.data[ch].value * 100 / 0.02;
		}

		ad_dev.data[ch].update = 1;
	}

	return 0;
}

/***************************************************************************//**
 * @brief Computes data register read size to account for bit number and status
 * 		 read.
 *
 * @param device - The handler of the instance of the driver.
 *
 * @return 0in case of success or negative code in case of failure.
 *******************************************************************************/
int32_t AD717X_ComputeDataregSize(ad717x_dev *device) {
	ad717x_st_reg *reg_ptr;
	ad717x_st_reg *datareg_ptr;
	uint16_t case_var;

	/* Get interface mode register pointer */
	reg_ptr = AD717X_GetReg(device, AD717X_IFMODE_REG);
	/* Get data register pointer */
	datareg_ptr = AD717X_GetReg(device, AD717X_DATA_REG);
	case_var = reg_ptr->value & (AD717X_IFMODE_REG_DATA_STAT |
	AD717X_IFMODE_REG_DATA_WL16);

	/* Compute data register size */
	datareg_ptr->size = 3;
	if ((case_var & AD717X_IFMODE_REG_DATA_WL16) == AD717X_IFMODE_REG_DATA_WL16)
		datareg_ptr->size--;
	if ((case_var & AD717X_IFMODE_REG_DATA_STAT) == AD717X_IFMODE_REG_DATA_STAT)
		datareg_ptr->size++;

	/* Get ID register pointer */
	reg_ptr = AD717X_GetReg(device, AD717X_ID_REG);

	/* If the part is 32/24 bit wide add a byte to the read */
	if ((reg_ptr->value & AD717X_ID_REG_MASK) == AD7177_2_ID_REG_VALUE)
		datareg_ptr->size++;

	return 0;
}

/***************************************************************************//**
 * @brief Computes the CRC checksum for a data buffer.
 *
 * @param pBuf    - Data buffer
 * @param bufSize - Data buffer size in bytes
 *
 * @return Returns the computed CRC checksum.
 *******************************************************************************/
uint8_t AD717X_ComputeCRC8(uint8_t * pBuf, uint8_t bufSize) {
	uint8_t i = 0;
	uint8_t crc = 0;

	while (bufSize) {
		for (i = 0x80; i != 0; i >>= 1) {
			if (((crc & 0x80) != 0) != ((*pBuf & i) != 0)) { /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD717X_CRC8_POLYNOMIAL_REPRESENTATION;
			} else {
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}

/***************************************************************************//**
 * @brief Computes the XOR checksum for a data buffer.
 *
 * @param pBuf    - Data buffer
 * @param bufSize - Data buffer size in bytes
 *
 * @return Returns the computed XOR checksum.
 *******************************************************************************/
uint8_t AD717X_ComputeXOR8(uint8_t * pBuf, uint8_t bufSize) {
	uint8_t xor = 0;

	while (bufSize) {
		xor ^= *pBuf;
		pBuf++;
		bufSize--;
	}
	return xor;
}

/***************************************************************************//**
 * @brief Updates the CRC settings.
 *
 * @param device - The handler of the instance of the driver.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X_UpdateCRCSetting(ad717x_dev *device) {
	ad717x_st_reg *ifReg;

	if (!device || !device->regs)
		return INVALID_VAL;

	ifReg = AD717X_GetReg(device, AD717X_IFMODE_REG);
	if (!ifReg)
		return INVALID_VAL;

	/* Get CRC State. */
	if (AD717X_IFMODE_REG_CRC_STAT(ifReg->value)) {
		device->useCRC = AD717X_USE_CRC;
	} else if (AD717X_IFMODE_REG_XOR_STAT(ifReg->value)) {
		device->useCRC = AD717X_USE_XOR;
	} else {
		device->useCRC = AD717X_DISABLE;
	}

	return 0;
}

/**
 * @brief  创建ADC设备
 * 		         设置初始化ADC所必需的参数
 * @param  sys_param: 系统参数
 *               	      根据系统参数创建ADC设备
 * @retval 返回ADC设备句柄
 */
ad717x_dev* AD717X_CreatAdcDev(sys_param_t *sys_param) {
	ad_dev.discard = 2;
	ad_dev.update_flag = 0;
	ad_dev.regs = ad4111_regs;
	ad_dev.num_regs = sizeof(ad4111_regs) / sizeof(ad717x_st_reg);
	ad_dev.param = sys_param;
	ad_dev.adcInit = AD717X_Init;
	ad_dev.dataFilter = Filter_Mean;
	ad_dev.dataSample = AD717X_DataSample;

	memset(ad_dev.data, 0X00, sizeof(ad_dev.data));

	ad4111_setup_cfg setup_cfg;
	setup_cfg.setup_reg = 0;
	setup_cfg.CODE_BI_UNIPOLAR = AD4111_STEUP_REG_UNIPOLAR;
	setup_cfg.REF_P_BUF = AD4111_STEUP_REG_REF_P_BUF_ENABLE;
	setup_cfg.REF_N_BUF = AD4111_STEUP_REG_REF_N_BUF_ENABLE;
	setup_cfg.INPUT_BUF = AD4111_STEUP_REG_INPUT_BUF_ENABLE;
	setup_cfg.REF_SEL = AD4111_STEUP_REG_REF_SEL_INT;
//	setup_cfg.REF_SEL = AD4111_STEUP_REG_REF_SEL_EXT;

	ad4111_filter_cfg filter_cfg;
	filter_cfg.filter_reg = 0;
//	filter_cfg.ENHFILTEN = AD4111_FILTER_REG_50_60HZ_ENABLE;
	filter_cfg.ENHFILT = AD4111_FILTER_REG_16SPS_92DB_60MS;
	filter_cfg.ORDER = AD4111_FILTER_REG_ORDER_SINC5_SINC1;
	filter_cfg.ODR = AD4111_FILTER_REG_ODR_100SPS;

	//通道配置
	ad4111_ch_cfg ch_cfg;
	for (uint8_t i = 0; i < ad_dev.param->ch_num; i++) {
		ch_cfg.ch_reg = 0;
		if (ad_dev.param->prop[i].status == CH_ENABLE) {
			if (ad_dev.param->prop[i].type == VOTL_0_5V
					|| ad_dev.param->prop[i].type == VOLT_0_10V) {
				ch_cfg.INPUT = unique_volt_ch[i];
				setup_cfg.INPUT_BUF = AD4111_STEUP_REG_INPUT_BUF_ENABLE;
			} else if (ad_dev.param->prop[i].type == CURR_4_20MA) {
				ch_cfg.INPUT = unique_curr_ch[i];
				setup_cfg.INPUT_BUF = AD4111_STEUP_REG_INPUT_BUF_DISABLE;
			}

			ch_cfg.CH_EN = AD4111_CH_REG_CH_ENABLE;
			ch_cfg.SETUP_SEL = AD4111_CH_REG_SETUP_SEL_0 + i;

			if (AD717X_SetReg(&ad_dev, AD717X_SETUPCON0_REG + i,
					setup_cfg.setup_reg) != 0)
				return NULL_POINTER ;
			if (AD717X_SetReg(&ad_dev, AD717X_FILTCON0_REG + i,
					filter_cfg.filter_reg) != 0)
				return NULL_POINTER ;
		}

		if (AD717X_SetReg(&ad_dev, AD717X_CHMAP0_REG + i, ch_cfg.ch_reg) != 0)
			return NULL_POINTER ;
	}

	ad4111_gpio_cfg gpio_cfg;
	gpio_cfg.gpio_reg = 0;
	gpio_cfg.OP_EN0_1 = AD4111_GPIO_REG_GPIO_0_1_ENABLE;
	gpio_cfg.GP_DATA1 = AD4111_GPIO_REG_GPIO_1_LOW;

	if (AD717X_SetReg(&ad_dev, AD717X_GPIOCON_REG, gpio_cfg.gpio_reg) != 0)
		return NULL_POINTER ;

	ad4111_mode_cfg mode_cfg;
	mode_cfg.mode_reg = 0;
	mode_cfg.DELAY = AD4111_MODE_REG_SWITCH_DELAY_1600US;
	mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_CONTINUE;
	mode_cfg.CLOCK_SEL = AD4111_MODE_REG_CLOCK_SEL_INT_OSC;
	mode_cfg.REF_INT_EN = AD4111_MODE_REG_REF_INT_ENABLE;
//	mode_cfg.REF_INT_EN = AD4111_MODE_REG_REF_INT_DISABLE;

	if (AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg) != 0)
		return NULL_POINTER ;

	ad4111_if_cfg if_cfg;
	if_cfg.if_reg = 0;
	if_cfg.CONT_READ = AD4111_IF_REG_CONT_READ_ENABLE;
	if_cfg.DATA_STAT = AD4111_IF_REG_DATA_STAT_ENABLE;
//	if_cfg.CRC_EN = AD4111_IF_REG_CHECKSUM_CRC_ENABLE;
	if_cfg.WL16 = AD4111_IF_REG_WORD_LENGTH_24BIT;

	if (AD717X_SetReg(&ad_dev, AD717X_IFMODE_REG, if_cfg.if_reg) != 0)
		return NULL_POINTER ;

	//写系统默认的电压增益寄存器值
	if (AD717X_SetReg(&ad_dev, AD717X_GAIN0_REG, 0X55567C) != 0)
		return NULL_POINTER ;

	return &ad_dev;
}

/**
 * @brief  ADC初始化
 * 		         接口模式寄存器中的连续读取位一旦被置位，那么ADC就只能识别读数据命名与复位命名
 * 		         应当先设置好ADC模式寄存器，再写接口寄存器
 * @param  None
 * @retval 0：成功  其它：失败
 */
int32_t AD717X_Init(void) {
	int32_t ret;

	/* Initialize the SPI communication. */
	ret = spi_init(&ad_dev.spi_desc);
	if (ret < 0)
		return ret;

	/*  Reset the device interface.*/
	ret = AD717X_Reset(&ad_dev);
	if (ret < 0)
		return ret;

	for (uint8_t i = 0; i < ad_dev.param->ch_num; i++)
	{
		if (ad_dev.param->prop[i].status == CH_ENABLE) {
			if (ad_dev.param->prop[i].type != CURR_4_20MA) {
				if (ad_dev.param->cali[i].offset != 0XFFFFFFFF) {
					ad717x_st_reg *reg = AD717X_GetReg(&ad_dev,
							AD717X_OFFSET0_REG + i);
					if (!reg)
						return INVALID_VAL;

					reg->value = ad_dev.param->cali[i].offset;

					ret = AD717X_WriteRegister(&ad_dev, AD717X_OFFSET0_REG + i);
					if (ret < 0)
						return ret;
				}

				if (ad_dev.param->cali[i].gain != 0XFFFFFFFF) {
					ad717x_st_reg *reg = AD717X_GetReg(&ad_dev,
							AD717X_GAIN0_REG + i);
					if (!reg)
						return INVALID_VAL;

					reg->value = ad_dev.param->cali[i].gain;

					ret = AD717X_WriteRegister(&ad_dev, AD717X_GAIN0_REG + i);
					if (ret < 0)
						return ret;
				} else {
					ad717x_st_reg *reg = AD717X_GetReg(&ad_dev,
							AD717X_GAIN0_REG + i);
					if (!reg)
						return INVALID_VAL;

					reg->value = 0X55567C;

					ret = AD717X_WriteRegister(&ad_dev, AD717X_GAIN0_REG + i);
					if (ret < 0)
						return ret;
				}
			}

			ret = AD717X_WriteRegister(&ad_dev, AD717X_SETUPCON0_REG + i);
			if (ret < 0)
				return ret;

			ret = AD717X_WriteRegister(&ad_dev, AD717X_FILTCON0_REG + i);
			if (ret < 0)
				return ret;
		}

		ret = AD717X_WriteRegister(&ad_dev, AD717X_CHMAP0_REG + i);
		if (ret < 0)
			return ret;
	}

	ret = AD717X_WriteRegister(&ad_dev, AD717X_GPIOCON_REG);
	if (ret < 0)
		return ret;

	/* Initialize ADC mode register. */
	ad4111_mode_cfg mode_cfg;
	mode_cfg.mode_reg = 0;
	mode_cfg.DELAY = AD4111_MODE_REG_SWITCH_DELAY_1600US;
	mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_STANDBY;
	mode_cfg.CLOCK_SEL = AD4111_MODE_REG_CLOCK_SEL_INT_OSC;
	mode_cfg.REF_INT_EN = AD4111_MODE_REG_REF_INT_ENABLE;
//	mode_cfg.REF_INT_EN = AD4111_MODE_REG_REF_INT_DISABLE;
	AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
	ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
	if (ret < 0)
		return ret;

#ifdef DEBUG
	extern void UART_485TxEn(void);
	extern void UART_485RxEn(void);
	UART_485TxEn();
	printf("storage cali message!\n");
	for(int i = 0; i < ad_dev.param->ch_num; i++)
	{
		printf("ch%d offset reg:0x%08X  ", i, (unsigned int)ad_dev.param->cali[i].offset);
		printf("gain reg:0x%08X\n", (unsigned int)ad_dev.param->cali[i].gain);
	}
	UART_485RxEn();

	UART_485TxEn();
	printf("actual cali message!\n");
	for(int i = 0; i < ad_dev.param->ch_num; i++)
	{
		ad717x_st_reg *reg;
		AD717X_ReadRegister(&ad_dev, AD717X_OFFSET0_REG+i);
		reg = AD717X_GetReg(&ad_dev, AD717X_OFFSET0_REG+i);

		printf("ch%d offset reg:0x%08X  ", i, (unsigned int)reg->value);

		AD717X_ReadRegister(&ad_dev, AD717X_GAIN0_REG+i);
		reg = AD717X_GetReg(&ad_dev, AD717X_GAIN0_REG+i);

		printf("gain reg:0x%08X\n", (unsigned int)reg->value);
	}
	UART_485RxEn();
#endif

	mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_CONTINUE;
	AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
	ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
	if (ret < 0)
		return ret;

	/* Initialize Interface mode register. */
	ret = AD717X_WriteRegister(&ad_dev, AD717X_IFMODE_REG);
	if (ret < 0)
		return ret;

	/* Get CRC State */
	ret = AD717X_UpdateCRCSetting(&ad_dev);
	if (ret < 0)
		return ret;

	return ret;
}

/**
 * @brief  参数改变后重新初始化ADC
 * @param  sys_param: 系统参数
 *               	  ADC根据系统参数配置来初始化
 * @retval 0：成功  其它：失败
 */
int32_t AD717X_ChangeAdcDev(sys_param_t *sys_param) {
	ad4111_setup_cfg setup_cfg;
	setup_cfg.setup_reg = 0;
	setup_cfg.CODE_BI_UNIPOLAR = AD4111_STEUP_REG_UNIPOLAR;
	setup_cfg.REF_P_BUF = AD4111_STEUP_REG_REF_P_BUF_ENABLE;
	setup_cfg.REF_N_BUF = AD4111_STEUP_REG_REF_N_BUF_ENABLE;
	setup_cfg.INPUT_BUF = AD4111_STEUP_REG_INPUT_BUF_ENABLE;
	setup_cfg.REF_SEL = AD4111_STEUP_REG_REF_SEL_INT;
//	setup_cfg.REF_SEL = AD4111_STEUP_REG_REF_SEL_EXT;

	ad4111_filter_cfg filter_cfg;
	filter_cfg.filter_reg = 0;
//	filter_cfg.ENHFILTEN = AD4111_FILTER_REG_50_60HZ_ENABLE;
	filter_cfg.ENHFILT = AD4111_FILTER_REG_16SPS_92DB_60MS;
	filter_cfg.ORDER = AD4111_FILTER_REG_ORDER_SINC5_SINC1;
	filter_cfg.ODR = AD4111_FILTER_REG_ODR_100SPS;

	//通道配置
	ad4111_ch_cfg ch_cfg;
	for (uint8_t i = 0; i < ad_dev.param->ch_num; i++) {
		ch_cfg.ch_reg = 0;
		if (ad_dev.param->prop[i].status == CH_ENABLE) {
			if (ad_dev.param->prop[i].type == VOTL_0_5V
					|| ad_dev.param->prop[i].type == VOLT_0_10V) {
				ch_cfg.INPUT = unique_volt_ch[i];
				setup_cfg.INPUT_BUF = AD4111_STEUP_REG_INPUT_BUF_ENABLE;
			} else if (ad_dev.param->prop[i].type == CURR_4_20MA) {
				ch_cfg.INPUT = unique_curr_ch[i];
				setup_cfg.INPUT_BUF = AD4111_STEUP_REG_INPUT_BUF_DISABLE;
			}

			ch_cfg.CH_EN = AD4111_CH_REG_CH_ENABLE;
			ch_cfg.SETUP_SEL = AD4111_CH_REG_SETUP_SEL_0 + i;

			if (AD717X_SetReg(&ad_dev, AD717X_SETUPCON0_REG + i,
					setup_cfg.setup_reg) != 0)
				return INVALID_VAL;
			if (AD717X_SetReg(&ad_dev, AD717X_FILTCON0_REG + i,
					filter_cfg.filter_reg) != 0)
				return INVALID_VAL;
		}

		if (AD717X_SetReg(&ad_dev, AD717X_CHMAP0_REG + i, ch_cfg.ch_reg) != 0)
			return INVALID_VAL;
	}

	return AD717X_Init();
}

/**
 * @brief 校准通道
 * @param ch_no: 校准通道号
 * @param cali_mode: 校准模式
 *                   可以是INT_OFFSET_CALI、INT_GAIN_CALI、SYS_OFFSET_CALI、SYS_GAIN_CALI的组合
 * @retval 0：成功  其它：失败
 */
int32_t AD717X_Cali(uint8_t ch_no, ad717x_cali_mode cali_mode) {
	int32_t ret;

	if (ch_no >= ad_dev.param->ch_num)
		return INVALID_VAL;

	/*  Reset the device interface.*/
	ret = AD717X_Reset(&ad_dev);
	if (ret < 0)
		return ret;

	if (ad_dev.param->cali[ch_no].offset != 0XFFFFFFFF) {
		ad717x_st_reg *reg = AD717X_GetReg(&ad_dev, AD717X_OFFSET0_REG + ch_no);
		if (!reg)
			return INVALID_VAL;

		reg->value = ad_dev.param->cali[ch_no].offset;

		ret = AD717X_WriteRegister(&ad_dev, AD717X_OFFSET0_REG + ch_no);
		if (ret < 0)
			return ret;
	}

	if (ad_dev.param->cali[ch_no].gain != 0XFFFFFFFF) {
		ad717x_st_reg *reg = AD717X_GetReg(&ad_dev, AD717X_GAIN0_REG + ch_no);
		if (!reg)
			return INVALID_VAL;

		reg->value = ad_dev.param->cali[ch_no].gain;

		ret = AD717X_WriteRegister(&ad_dev, AD717X_GAIN0_REG + ch_no);
		if (ret < 0)
			return ret;
	} else {
		ad717x_st_reg *reg = AD717X_GetReg(&ad_dev, AD717X_GAIN0_REG + ch_no);
		if (!reg)
			return INVALID_VAL;

		reg->value = 0X55567C;

		ret = AD717X_WriteRegister(&ad_dev, AD717X_GAIN0_REG + ch_no);
		if (ret < 0)
			return ret;
	}

	ret = AD717X_WriteRegister(&ad_dev, AD717X_SETUPCON0_REG + ch_no);
	if (ret < 0)
		return ret;

	ret = AD717X_WriteRegister(&ad_dev, AD717X_FILTCON0_REG + ch_no);
	if (ret < 0)
		return ret;

	//ADC复位后默认使能通道0，如果不是对0通道校准则需要关闭0通道
	if (ch_no != 0) {
		ad4111_ch_cfg ch_cfg;
		ch_cfg.ch_reg = 0;
		if (AD717X_SetReg(&ad_dev, AD717X_CHMAP0_REG + 0, ch_cfg.ch_reg) != 0)
			return INVALID_VAL;
		ret = AD717X_WriteRegister(&ad_dev, AD717X_CHMAP0_REG + 0);
		if (ret < 0)
			return ret;
	}

	ret = AD717X_WriteRegister(&ad_dev, AD717X_CHMAP0_REG + ch_no);
	if (ret < 0)
		return ret;

	ret = AD717X_WriteRegister(&ad_dev, AD717X_GPIOCON_REG);
	if (ret < 0)
		return ret;

	/* Initialize ADC mode register. */
	ad4111_mode_cfg mode_cfg;
	mode_cfg.mode_reg = 0;
	mode_cfg.DELAY = AD4111_MODE_REG_SWITCH_DELAY_1600US;
	mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_STANDBY;
	mode_cfg.CLOCK_SEL = AD4111_MODE_REG_CLOCK_SEL_INT_OSC;
	mode_cfg.REF_INT_EN = AD4111_MODE_REG_REF_INT_ENABLE;
//	mode_cfg.REF_INT_EN = AD4111_MODE_REG_REF_INT_DISABLE;
	AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
	ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
	if (ret < 0)
		return ret;

	if (cali_mode == INT_OFFSET_CALI) {
		//内部零点平校准
		mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_INT_OFFSET_CALI;
		AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
		ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
		if (ret < 0)
			return ret;
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
		ad_dev.param->cali[ch_no].mode |= INT_OFFSET_CALI;
		ad_dev.param->cali[ch_no].mode &= ~SYS_OFFSET_CALI;
	}

	if (cali_mode == INT_GAIN_CALI) {
		//内部满量程校准
		mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_INT_GAIN_CALI;
		AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
		ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
		if (ret < 0)
			return ret;
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
		ad_dev.param->cali[ch_no].mode |= INT_GAIN_CALI;
		ad_dev.param->cali[ch_no].mode &= ~SYS_GAIN_CALI;
	}

	if (cali_mode == SYS_OFFSET_CALI) {
		//系统零点平校准
		mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_SYS_OFFSET_CALI;
		AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
		ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
		if (ret < 0)
			return ret;
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
		ad_dev.param->cali[ch_no].mode |= SYS_OFFSET_CALI;
		ad_dev.param->cali[ch_no].mode &= ~INT_OFFSET_CALI;
	}

	if (cali_mode == SYS_GAIN_CALI) {
		//系统满量程校准
		mode_cfg.OPERATE_MODE = AD4111_MODE_REG_OPERATE_MODE_SYS_GAIN_CALI;
		AD717X_SetReg(&ad_dev, AD717X_ADCMODE_REG, mode_cfg.mode_reg);
		ret = AD717X_WriteRegister(&ad_dev, AD717X_ADCMODE_REG);
		if (ret < 0)
			return ret;
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
		ad_dev.param->cali[ch_no].mode |= SYS_GAIN_CALI;
		ad_dev.param->cali[ch_no].mode &= ~INT_GAIN_CALI;
	}

	//获取校准后offset寄存器值
	ret = AD717X_ReadRegister(&ad_dev, AD717X_OFFSET0_REG + ch_no);
	if (ret < 0)
		return ret;

	ad717x_st_reg *reg = AD717X_GetReg(&ad_dev, AD717X_OFFSET0_REG + ch_no);
	if (!reg)
		return INVALID_VAL;

	ad_dev.param->cali[ch_no].offset = reg->value;

	//获取校准后gain寄存器值
	ret = AD717X_ReadRegister(&ad_dev, AD717X_GAIN0_REG + ch_no);
	if (ret < 0)
		return ret;

	reg = AD717X_GetReg(&ad_dev, AD717X_GAIN0_REG + ch_no);
	if (!reg)
		return INVALID_VAL;

	ad_dev.param->cali[ch_no].gain = reg->value;
	ad_dev.param->update_flag = 1;
	ad_dev.update_flag = 1;

#ifdef DEBUG
	extern void UART_485TxEn(void);
	extern void UART_485RxEn(void);
	UART_485TxEn();
	printf("after cali reg message!\n");
	for(int i = 0; i < ad_dev.param->ch_num; i++)
	{
		ret = AD717X_ReadRegister(&ad_dev, AD717X_OFFSET0_REG+i);
		if(ret < 0)
		return ret;
		reg = AD717X_GetReg(&ad_dev, AD717X_OFFSET0_REG+i);
		if (!reg)
		return INVALID_VAL;

		printf("ch%d offset reg:0x%08X  ", i, (unsigned int)reg->value);

		ret = AD717X_ReadRegister(&ad_dev, AD717X_GAIN0_REG+i);
		if(ret < 0)
		return ret;
		reg = AD717X_GetReg(&ad_dev, AD717X_GAIN0_REG+i);
		if (!reg)
		return INVALID_VAL;

		printf("gain reg:0x%08X\n", (unsigned int)reg->value);
	}
	UART_485RxEn();
#endif

	return ret;
}

/***************************************************************************//**
 * @brief Free the resources allocated by AD717X_Init().
 * @param dev - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
 *******************************************************************************/
int32_t AD717X_remove(ad717x_dev *dev) {
	int32_t ret;

	ret = spi_remove(dev->spi_desc);
	free(dev);

	return ret;
}

