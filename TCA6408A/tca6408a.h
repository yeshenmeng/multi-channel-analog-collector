#ifndef __TCA6408A_H__
#define __TCA6408A_H__
#include "i2c.h"


#define I2C_OPERATION_SUCCESS		0
#define I2C_OPERATION_FAIL 			1

/************************** I2C Address ***************************************/
#define TCA6408A1_ADDRESS		(0x21 << 1) // I2C Address 0100 00 + ADDR + R/W
#define TCA6408A2_ADDRESS		(0x20 << 1)	// I2C Address 0100 00 + ADDR + R/W

/************************** I2C Registers *************************************/
#define TCA6408A_INPUT_REG 		0x00		// Input status register
#define TCA6408A_OUTPUT_REG		0x01		// Output register to change state of output BIT set to 1, output set HIGH
#define TCA6408A_POLARITY_REG 	0x02		// Polarity inversion register. BIT '1' inverts input polarity of register 0x00
#define TCA6408A_CONFIG_REG		0x03		// Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output

/****************************************************************************************
 * 	The following structs and unions are used to configure a memory map of the TCA6408A *
 * 	on the MSP430																		*
 * 																						*
 * 	They allow the user to change the entire byte or alter the registers bit by bit		*
 ***************************************************************************************/

/************************** TCA6408A Bit struct *********************************/
struct TCA6408A_sbit{
	unsigned char P0:1;
	unsigned char P1:1;
	unsigned char P2:1;
	unsigned char P3:1;
	unsigned char P4:1;
	unsigned char P5:1;
	unsigned char P6:1;
	unsigned char P7:1;
};

union TCA6408A_uInput{
	unsigned char 				all;
	struct TCA6408A_sbit	 	bit;
};

union TCA6408A_uOutput{
	unsigned char 				all;
	struct TCA6408A_sbit		bit;
};

union TCA6408A_uPolarityInversion{
	unsigned char				all;
	struct TCA6408A_sbit 		bit;
};

union TCA6408A_uConfig{
	unsigned char				all;
	struct TCA6408A_sbit		bit;
};

typedef struct {
	union TCA6408A_uInput 				Input;
	union TCA6408A_uOutput				Output;
	union TCA6408A_uPolarityInversion	PolarityInversion;
	union TCA6408A_uConfig				Config;
} TCA6408ARegs;

#define TCA6408_GPIO_PIN_0	0X01
#define TCA6408_GPIO_PIN_1	0X02
#define TCA6408_GPIO_PIN_2	0X04
#define TCA6408_GPIO_PIN_3	0X08
#define TCA6408_GPIO_PIN_4	0X10
#define TCA6408_GPIO_PIN_5	0X20
#define TCA6408_GPIO_PIN_6	0X40
#define TCA6408_GPIO_PIN_7	0X80

typedef enum{
	TCA6408_GPIO_OUTPUT,
	TCA6408_GPIO_INTPUT
}TCA6408_GPIO_MODE;

typedef struct{
	uint8_t address;
	uint8_t pin;
	TCA6408_GPIO_MODE mode;
	TCA6408ARegs regs;
	I2C_HandleTypeDef *i2c_desc;
}TCA6408_dev;

typedef struct{
	TCA6408_dev dev1;
	TCA6408_dev dev2;
}TCA6408_Typedef;

TCA6408_Typedef* TCA6408_GPIOInit(void);
uint8_t TCA6408A_WriteConfig(TCA6408_dev *dev);
uint8_t TCA6408A_WriteOutput(TCA6408_dev *dev);
uint8_t TCA6408A_WritePolarity(TCA6408_dev *dev);
uint8_t TCA6408A_ReadInput(TCA6408_dev *dev);
uint8_t TCA6408A_ReadReg(TCA6408_dev *dev, uint8_t regAddress, uint8_t *pData, uint16_t size);

#endif
