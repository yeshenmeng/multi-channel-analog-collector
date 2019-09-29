#include "tca6408a.h"

TCA6408_Typedef TCA6408;

/**
 * @brief  初始化TCA设备并返回操作句柄
 * @param  None
 * @retval HAL status
 */
TCA6408_Typedef* TCA6408_GPIOInit(void) {
	TCA6408.dev1.regs.Config.all = 0x00;
	TCA6408.dev1.regs.Output.all = 0x00;
	TCA6408.dev1.regs.PolarityInversion.all = 0x00;

	TCA6408.dev1.i2c_desc = &hi2c1;
	TCA6408.dev1.address = TCA6408A1_ADDRESS;
	TCA6408.dev1.pin = TCA6408_GPIO_PIN_0 | TCA6408_GPIO_PIN_1 |
					   TCA6408_GPIO_PIN_2 | TCA6408_GPIO_PIN_3 |
					   TCA6408_GPIO_PIN_4 | TCA6408_GPIO_PIN_5 |
					   TCA6408_GPIO_PIN_6 | TCA6408_GPIO_PIN_7;
	TCA6408.dev1.mode = TCA6408_GPIO_OUTPUT;
	TCA6408A_WriteConfig(&TCA6408.dev1);

	TCA6408.dev2.regs.Config.all = 0x00;
	TCA6408.dev2.regs.Output.all = 0x00;
	TCA6408.dev2.regs.PolarityInversion.all = 0x00;

	TCA6408.dev2.i2c_desc = &hi2c1;
	TCA6408.dev2.address = TCA6408A2_ADDRESS;
	TCA6408.dev2.pin = TCA6408_GPIO_PIN_0 | TCA6408_GPIO_PIN_1 |
					   TCA6408_GPIO_PIN_2 | TCA6408_GPIO_PIN_3;
	TCA6408.dev2.mode = TCA6408_GPIO_OUTPUT;
	TCA6408A_WriteConfig(&TCA6408.dev2);

	TCA6408.dev2.pin = TCA6408_GPIO_PIN_4 | TCA6408_GPIO_PIN_5 |
					   TCA6408_GPIO_PIN_6 | TCA6408_GPIO_PIN_7;
	TCA6408.dev2.mode = TCA6408_GPIO_INTPUT;
	TCA6408A_WriteConfig(&TCA6408.dev2);

	TCA6408A_WriteOutput(&TCA6408.dev1);
	TCA6408A_WriteOutput(&TCA6408.dev2);
	TCA6408A_WritePolarity(&TCA6408.dev1);
	TCA6408A_WritePolarity(&TCA6408.dev2);

	return &TCA6408;
}

// ****************************************************************************
//! @fn          void TCA6408AWriteConfig(TCA6408ARegs* Regs)
//! @brief		 Writes config value from MSP430 memory map to TCA6408A
//! @return		 Returns I2C success or failure
//!
// ****************************************************************************
uint8_t TCA6408A_WriteConfig(TCA6408_dev *dev) {
	uint8_t ret = I2C_OPERATION_SUCCESS;

	if (dev->mode == TCA6408_GPIO_OUTPUT) {
		for (int i = 0; i < 8; i++) {
			if (dev->pin & (1 << i))
				dev->regs.Config.all &= ~(1 << i);
		}
	} else if (dev->mode == TCA6408_GPIO_INTPUT) {
		for (int i = 0; i < 8; i++) {
			if (dev->pin & (1 << i))
				dev->regs.Config.all |= (1 << i);
		}
	}

	uint8_t buf[2];
	buf[0] = TCA6408A_CONFIG_REG;
	buf[1] = dev->regs.Config.all;
	ret = HAL_I2C_Master_Transmit(dev->i2c_desc, dev->address, buf, sizeof(buf),
			(~0));
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	return ret;
}

// ****************************************************************************
//! @fn          void TCA6408AWriteOutput(TCA6408ARegs* Regs)
//! @brief		 Writes output value from MSP430 memory map to TCA6408A
//! @return		 Returns I2C success or failure
//!
// ****************************************************************************
uint8_t TCA6408A_WriteOutput(TCA6408_dev *dev) {
	uint8_t ret = I2C_OPERATION_SUCCESS;

	uint8_t buf[2];
	buf[0] = TCA6408A_OUTPUT_REG;
	buf[1] = dev->regs.Output.all;
	ret = HAL_I2C_Master_Transmit(dev->i2c_desc, dev->address, buf, sizeof(buf),
			(~0));
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	return ret;
}

// ****************************************************************************
//! @fn          void TCA6408AWritePolarity(TCA6408ARegs* Regs)
//! @brief		 Writes polarity inversion value from MSP430 memory map to TCA6408A
//! @return		 Returns I2C success or failure
//!
// ****************************************************************************
uint8_t TCA6408A_WritePolarity(TCA6408_dev *dev) {
	uint8_t ret = I2C_OPERATION_SUCCESS;

	uint8_t buf[2];
	buf[0] = TCA6408A_POLARITY_REG;
	buf[1] = dev->regs.PolarityInversion.all;
	ret = HAL_I2C_Master_Transmit(dev->i2c_desc, dev->address, buf, sizeof(buf),
			(~0));
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	return ret;
}

// ****************************************************************************
//! @fn          void TCA6408AReadInputReg(TCA6408ARegs* Regs)
//! @brief		 Stores input register values to MSP430 Memory map
//! @return		 Returns I2C success or failure
//!
// ****************************************************************************
uint8_t TCA6408A_ReadInput(TCA6408_dev *dev) {
	uint8_t ret = I2C_OPERATION_SUCCESS;
	uint8_t buf[1] = { TCA6408A_INPUT_REG };

	ret = HAL_I2C_Master_Transmit(dev->i2c_desc, dev->address, buf, sizeof(buf),
			(~0));
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	ret = HAL_I2C_Master_Receive(dev->i2c_desc, dev->address,
			&dev->regs.Input.all, 1, 10);
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	return ret;
}

// ****************************************************************************
//! @fn          void TCA6408AReadInputReg(TCA6408ARegs* Regs)
//! @brief		 Stores input register values to MSP430 Memory map
//! @return		 Returns I2C success or failure
//!
// ****************************************************************************
uint8_t TCA6408A_ReadReg(TCA6408_dev *dev, uint8_t regAddress, uint8_t *pData,
		uint16_t size) {
	uint8_t ret = I2C_OPERATION_SUCCESS;
	uint8_t buf[1];

	buf[0] = regAddress;
	ret = HAL_I2C_Master_Transmit(dev->i2c_desc, dev->address, buf, sizeof(buf),
			(~0));
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	ret = HAL_I2C_Master_Receive(dev->i2c_desc, dev->address, pData, size, 10);
	if (ret != I2C_OPERATION_SUCCESS)
		ret = I2C_OPERATION_FAIL;

	return ret;
}

