/***************************************************************************//**
*   @file   ad4111_regs.h
*   @brief  ad4111 Registers Definitions.
*   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
* Copyright 2018(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
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
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef AD4111_REGS_H_
#define AD4111_REGS_H_

ad717x_st_reg ad4111_regs[] = {
	{ AD717X_STATUS_REG, 0x00, 1 },
	{
		AD717X_ADCMODE_REG,
		AD717X_ADCMODE_REG_MODE(0),
		2
	},
	{ AD717X_IFMODE_REG, 0x0000, 2 },
	{ AD717X_REGCHECK_REG, 0x0000, 3 },
	{ AD717X_DATA_REG, 0x0000, 3 },
	{
		AD717X_GPIOCON_REG,
		AD717X_GPIOCON_REG_SYNC_EN,
		2
	},
	{ AD717X_ID_REG, 0x0000, 2 },
	{ AD717X_CHMAP0_REG, 0x0000, 2 },
	{ AD717X_CHMAP1_REG, 0x0000, 2 },
	{ AD717X_CHMAP2_REG, 0x0000, 2 },
	{ AD717X_CHMAP3_REG, 0x0000, 2 },
	{ AD717X_CHMAP4_REG, 0x0000, 2 },
	{ AD717X_CHMAP5_REG, 0x0000, 2 },
	{ AD717X_CHMAP6_REG, 0x0000, 2 },
	{ AD717X_CHMAP7_REG, 0x0000, 2 },
	{ AD717X_CHMAP8_REG, 0x0000, 2 },
	{ AD717X_CHMAP9_REG, 0x0000, 2 },
	{ AD717X_CHMAP10_REG, 0x0000, 2 },
	{ AD717X_CHMAP11_REG, 0x0000, 2 },
	{ AD717X_CHMAP12_REG, 0x0000, 2 },
	{ AD717X_CHMAP13_REG, 0x0000, 2 },
	{ AD717X_CHMAP14_REG, 0x0000, 2 },
	{ AD717X_CHMAP15_REG, 0x0000, 2 },
	{ AD717X_SETUPCON0_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON1_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON2_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON3_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON4_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON5_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON6_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{ AD717X_SETUPCON7_REG, 0x0000 | AD717X_SETUP_CONF_REG_REF_SEL(2), 2 },
	{
		AD717X_FILTCON0_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON1_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON2_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON3_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON4_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON5_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON6_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{
		AD717X_FILTCON7_REG, AD717X_FILT_CONF_REG_ENHFILT(2), 2
	},
	{AD717X_OFFSET0_REG, 0, 3 },
	{AD717X_OFFSET1_REG, 0, 3 },
	{AD717X_OFFSET2_REG, 0, 3 },
	{AD717X_OFFSET3_REG, 0, 3 },
	{AD717X_OFFSET4_REG, 0, 3 },
	{AD717X_OFFSET5_REG, 0, 3 },
	{AD717X_OFFSET6_REG, 0, 3 },
	{AD717X_OFFSET7_REG, 0, 3 },
	{AD717X_GAIN0_REG, 0, 3 },
	{AD717X_GAIN1_REG, 0, 3 },
	{AD717X_GAIN2_REG, 0, 3 },
	{AD717X_GAIN3_REG, 0, 3 },
	{AD717X_GAIN4_REG, 0, 3 },
	{AD717X_GAIN5_REG, 0, 3 },
	{AD717X_GAIN6_REG, 0, 3 },
	{AD717X_GAIN7_REG, 0, 3 },
};

#define AD4111_STEUP_REG_UNIPOLAR				0X00 //数据单极性
#define AD4111_STEUP_REG_BIPOLAR				0X01 //数据双极性
#define AD4111_STEUP_REG_REF_P_BUF_DISABLE		0X00
#define AD4111_STEUP_REG_REF_P_BUF_ENABLE		0X01 //ADC正输入缓存
#define AD4111_STEUP_REG_REF_N_BUF_DISABLE		0X00
#define AD4111_STEUP_REG_REF_N_BUF_ENABLE		0X01 //ADC负输入缓存
#define AD4111_STEUP_REG_INPUT_BUF_DISABLE		0X00
#define AD4111_STEUP_REG_INPUT_BUF_ENABLE		0X03 //ADC输入缓存（电压通道打开、电流通道关闭）
#define AD4111_STEUP_REG_REF_SEL_EXT			0X00 //ADC转换外部参考源
#define AD4111_STEUP_REG_REF_SEL_INT			0X02 //ADC转换内部参考源
#define AD4111_STEUP_REG_REF_SEL_AVDD_AVSS		0X03 //ADC转换AVDD_AVSS参考源

typedef union
{
	struct
	{
		uint16_t RESERVE0:4;
		uint16_t REF_SEL:2;
		uint16_t RESERVE1:2;
		uint16_t INPUT_BUF:2;
		uint16_t REF_N_BUF:1;
		uint16_t REF_P_BUF:1;
		uint16_t CODE_BI_UNIPOLAR:1;
	};uint16_t setup_reg;
}ad4111_setup_cfg;

#define AD4111_FILTER_REG_SINC3_MAP_DISABLE		0X00
#define AD4111_FILTER_REG_SINC3_MAP_ENABLE		0X01 //SINC3滤波器(单通道使用)
#define AD4111_FILTER_REG_50_60HZ_DISABLE		0X00
#define AD4111_FILTER_REG_50_60HZ_ENABLE		0X01 //后置增强滤波器(需要选择SINC5_SINC1滤波器)
#define AD4111_FILTER_REG_27SPS_47DB_36MS		0X02
#define AD4111_FILTER_REG_25SPS_62DB_40MS		0X03
#define AD4111_FILTER_REG_20SPS_86DB_50MS		0X05
#define AD4111_FILTER_REG_16SPS_92DB_60MS		0X06 //增强滤波器输出速率、噪音抑制与数据建立时间
#define AD4111_FILTER_REG_ORDER_SINC5_SINC1		0X00 //选择SINC5_SINC1滤波器
#define AD4111_FILTER_REG_ORDER_SINC3			0X01 //选择SINC3滤波器
#define AD4111_FILTER_REG_ODR_31250SPS			0X00 //数据输出速率
#define AD4111_FILTER_REG_ODR_15625SPS			0X06
#define AD4111_FILTER_REG_ODR_10417SPS			0X07
#define AD4111_FILTER_REG_ODR_5208SPS			0X08
#define AD4111_FILTER_REG_ODR_2597SPS			0X09
#define AD4111_FILTER_REG_ODR_1007SPS			0X0A
#define AD4111_FILTER_REG_ODR_503SPS			0X0B
#define AD4111_FILTER_REG_ODR_381SPS			0X0C
#define AD4111_FILTER_REG_ODR_200SPS			0X0D
#define AD4111_FILTER_REG_ODR_100SPS			0X0E
#define AD4111_FILTER_REG_ODR_59SPS				0X0F
#define AD4111_FILTER_REG_ODR_49SPS				0X10
#define AD4111_FILTER_REG_ODR_20SPS				0X11
#define AD4111_FILTER_REG_ODR_16SPS				0X12
#define AD4111_FILTER_REG_ODR_10SPS				0X13
#define AD4111_FILTER_REG_ODR_5SPS				0X14
#define AD4111_FILTER_REG_ODR_2SPS				0X15
#define AD4111_FILTER_REG_ODR_1SPS				0X16

typedef union
{
	struct
	{
		uint16_t ODR:5;	//output data rate
		uint16_t ORDER:2;	//order of filter
		uint16_t RESERVE0:1;
		uint16_t ENHFILT:3;	//enhanced filter
		uint16_t ENHFILTEN:1;	//enhanced filter enable
		uint16_t RESERVE1:3;
		uint16_t SINC3_MAP:1;
	};uint16_t filter_reg;
}ad4111_filter_cfg;

#define AD4111_CH_REG_CH_DISABLE	0X00
#define AD4111_CH_REG_CH_ENABLE		0X01
#define AD4111_CH_REG_SETUP_SEL_0	0X00
#define AD4111_CH_REG_SETUP_SEL_1	0X01
#define AD4111_CH_REG_SETUP_SEL_2	0X02
#define AD4111_CH_REG_SETUP_SEL_3	0X03
#define AD4111_CH_REG_SETUP_SEL_4	0X04
#define AD4111_CH_REG_SETUP_SEL_5	0X05
#define AD4111_CH_REG_SETUP_SEL_6	0X06
#define AD4111_CH_REG_SETUP_SEL_7	0X07
#define AD4111_CH_REG_INPUT_V0_V1	0X0001
#define AD4111_CH_REG_INPUT_V0_VC	0X0010
#define AD4111_CH_REG_INPUT_V1_V0	0X0020
#define AD4111_CH_REG_INPUT_V1_VC	0X0030
#define AD4111_CH_REG_INPUT_V2_V3	0X0043
#define AD4111_CH_REG_INPUT_V2_VC	0X0050
#define AD4111_CH_REG_INPUT_V3_V2	0X0062
#define AD4111_CH_REG_INPUT_V3_VC	0X0070
#define AD4111_CH_REG_INPUT_V4_V5	0X0085
#define AD4111_CH_REG_INPUT_V4_VC	0X0090
#define AD4111_CH_REG_INPUT_V5_V4	0X00A4
#define AD4111_CH_REG_INPUT_V5_VC	0X00B0
#define AD4111_CH_REG_INPUT_V6_V7	0X00C7
#define AD4111_CH_REG_INPUT_V6_VC	0X00D0
#define AD4111_CH_REG_INPUT_V7_V6	0X00E6
#define AD4111_CH_REG_INPUT_V7_VC	0X00F0
#define AD4111_CH_REG_INPUT_I3		0X018B
#define AD4111_CH_REG_INPUT_I2		0X01AA
#define AD4111_CH_REG_INPUT_I1		0X01C9
#define AD4111_CH_REG_INPUT_I0		0X01E8
#define AD4111_CH_REG_INPUT_TEMP	0X0232
#define AD4111_CH_REG_INPUT_REF		0X02B6

//单极性电压通道
const uint16_t unique_volt_ch[] = {
	AD4111_CH_REG_INPUT_V0_VC,
	AD4111_CH_REG_INPUT_V1_VC,
	AD4111_CH_REG_INPUT_V2_VC,
	AD4111_CH_REG_INPUT_V3_VC,
	AD4111_CH_REG_INPUT_V4_VC,
	AD4111_CH_REG_INPUT_V5_VC,
	AD4111_CH_REG_INPUT_V6_VC,
	AD4111_CH_REG_INPUT_V7_VC
};

//单极性电流通道
const uint16_t unique_curr_ch[] = {
	AD4111_CH_REG_INPUT_I0,
	AD4111_CH_REG_INPUT_I1,
	AD4111_CH_REG_INPUT_I2,
	AD4111_CH_REG_INPUT_I3
};

typedef union
{
	struct
	{
		uint16_t INPUT:10;
		uint16_t RESERVE:2;
		uint16_t SETUP_SEL:3;
		uint16_t CH_EN:1;
	};uint16_t ch_reg;
}ad4111_ch_cfg;

#define AD4111_MODE_REG_REF_INT_DISABLE					0X00
#define AD4111_MODE_REG_REF_INT_ENABLE					0X01
#define AD4111_MODE_REG_SING_CYC_DISABLE				0X00
#define AD4111_MODE_REG_SING_CYC_ENABLE					0X01
#define AD4111_MODE_REG_SWITCH_DELAY_0US				0X00
#define AD4111_MODE_REG_SWITCH_DELAY_32US				0X01
#define AD4111_MODE_REG_SWITCH_DELAY_128US				0X02
#define AD4111_MODE_REG_SWITCH_DELAY_320US				0X03
#define AD4111_MODE_REG_SWITCH_DELAY_800US				0X04
#define AD4111_MODE_REG_SWITCH_DELAY_1600US				0X05
#define AD4111_MODE_REG_SWITCH_DELAY_4000US				0X06
#define AD4111_MODE_REG_SWITCH_DELAY_8000US				0X07
#define AD4111_MODE_REG_OPERATE_MODE_CONTINUE			0X00
#define AD4111_MODE_REG_OPERATE_MODE_SINGLE				0X01
#define AD4111_MODE_REG_OPERATE_MODE_STANDBY			0X02
#define AD4111_MODE_REG_OPERATE_MODE_POWER_DOWN			0X03
#define AD4111_MODE_REG_OPERATE_MODE_INT_OFFSET_CALI	0X04
#define AD4111_MODE_REG_OPERATE_MODE_INT_GAIN_CALI		0X05
#define AD4111_MODE_REG_OPERATE_MODE_SYS_OFFSET_CALI	0X06
#define AD4111_MODE_REG_OPERATE_MODE_SYS_GAIN_CALI		0X07
#define AD4111_MODE_REG_CLOCK_SEL_INT_OSC				0X00
#define AD4111_MODE_REG_CLOCK_SEL_INT_OSC_2_XTAL2		0X01
#define AD4111_MODE_REG_CLOCK_SEL_EXT_2_XTAL2			0X02
#define AD4111_MODE_REG_CLOCK_SEL_EXT_XTAL1_XTAL2		0X03

typedef union
{
	struct
	{
		uint16_t RESERVE0:2;
		uint16_t CLOCK_SEL:2;
		uint16_t OPERATE_MODE:3;
		uint16_t RESERVE1:1;
		uint16_t DELAY:3;
		uint16_t RESERVE2:2;
		uint16_t SING_CYC:1;
		uint16_t RESERVE3:1;
		uint16_t REF_INT_EN:1;
	};uint16_t mode_reg;
}ad4111_mode_cfg;

#define AD4111_IF_REG_ATL_SYNC_DISABLE			0X00
#define AD4111_IF_REG_ATL_SYNC_ENABLE			0X01
#define AD4111_IF_REG_IO_STRENGTH_DISABLE		0X00
#define AD4111_IF_REG_IO_STRENGTH_ENABLE		0X01
#define AD4111_IF_REG_DOUT_RESET_DISABLE		0X00
#define AD4111_IF_REG_DOUT_RESET_ENABLE			0X01
#define AD4111_IF_REG_CONT_READ_DISABLE			0X00
#define AD4111_IF_REG_CONT_READ_ENABLE			0X01
#define AD4111_IF_REG_DATA_STAT_DISABLE			0X00
#define AD4111_IF_REG_DATA_STAT_ENABLE			0X01
#define AD4111_IF_REG_REG_CHECK_DISABLE			0X00
#define AD4111_IF_REG_REG_CHECK_ENABLE			0X01
#define AD4111_IF_REG_CHECKSUM_DISABLE			0X00
#define AD4111_IF_REG_CHECKSUM_XOR_ENABLE		0X01
#define AD4111_IF_REG_CHECKSUM_CRC_ENABLE		0X02
#define AD4111_IF_REG_WORD_LENGTH_24BIT			0X00
#define AD4111_IF_REG_WORD_LENGTH_16BIT			0X01

typedef union
{
	struct
	{
		uint16_t WL16:1;
		uint16_t RESERVE0:1;
		uint16_t CRC_EN:2;
		uint16_t RESERVE1:1;
		uint16_t REG_CHECK:1;
		uint16_t DATA_STAT:1;
		uint16_t CONT_READ:1;
		uint16_t DOUT_RESET:1;
		uint16_t RESERVE2:2;
		uint16_t IO_STRENGTH:1;
		uint16_t ALT_SYNC:1;
		uint16_t RESERVE3:3;
	};uint16_t if_reg;
}ad4111_if_cfg;

#define AD4111_GPIO_REG_GPIO_0_1_DISABLE			0X00
#define AD4111_GPIO_REG_GPIO_0_1_ENABLE				0X01
#define AD4111_GPIO_REG_OPEN_WIRE_DISABLE			0X00
#define AD4111_GPIO_REG_OPEN_WIRE_ENABLE			0X01
#define AD4111_GPIO_REG_SYNC_DISABLE				0X00
#define AD4111_GPIO_REG_SYNC_ENABLE					0X01
#define AD4111_GPIO_REG_ERR_DISABLE					0X00
#define AD4111_GPIO_REG_ERR_INPUT					0X01
#define AD4111_GPIO_REG_ERR_OPRN_DRAIN_OUTPUT		0X02
#define AD4111_GPIO_REG_ERR_GENERAL_OUTPUT			0X03
#define AD4111_GPIO_REG_GPIO_1_LOW					0X00
#define AD4111_GPIO_REG_GPIO_1_HIGH					0X01
#define AD4111_GPIO_REG_GPIO_0_LOW					0X00
#define AD4111_GPIO_REG_GPIO_0_HIGH					0X01

typedef union
{
	struct
	{
		uint16_t RESERVE0:6;
		uint16_t GP_DATA0:1;
		uint16_t GP_DATA1:1;
		uint16_t ERR_DAT:1; //only read
		uint16_t ERR_EN:2;
		uint16_t SYNC_EN:1;
		uint16_t OW_EN:1;	//open wire
		uint16_t OP_EN0_1:1; //output GPIO0 GPIO1
		uint16_t RESERVE1:2;
	};uint16_t gpio_reg;
}ad4111_gpio_cfg;

#endif /* AD4111_CFG_H_ */
