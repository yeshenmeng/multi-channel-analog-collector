/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether 
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iotobject.h"
#include "modbus_comm_services.h"
#include "ad717x.h"
#include "tca6408a.h"
#include "ch_manage.h"
#include "sys_param.h"
#include "iot_operate.h"
#include "light.h"
#include "general_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef DEBUG
void IoT_SetLongAddr(void);
#endif

void UART_485TxEn(void);
void UART_485RxEn(void);
void IoT_SetProp(iot_object_t *sensor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
modbus_comm_services_t * comm_service;
uint8_t write_protect_flag = 0; //���Ե�д������־����������ַ���ԣ���0��ʾд�����򿪣�1��ʾд�����ر�
uint8_t comm_indicate_flag = 0; //ͨ��ָʾ��־��1��ʾ�������ݽ���ʧ�ܣ�2��ʾ�������ݽ����ɹ�
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	delay_nms(20);
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	//Config this smart sensor
#ifdef DEBUG
	IoT_SetLongAddr();
#endif

	/* �豸ָʾ�Ƴ�ʼ��  */
	light_ctrl_t *light_ctrl;
	light_ctrl = Light_Init();

	iot_object_t *mySensor = createSensorHandler();
	IoT_SetProp(mySensor);
	//��ʼ���������Ե���ʼidλ�ã��Լ�load flash�еĵ��籣������ֵ
	mySensor->init();

	/* ��������ͨ�ŷ��� */
	comm_service = createModbusCommServiceHandler(UART_485TxEn, UART_485RxEn);
	comm_service->setSensorHandler(mySensor);

	/* ��ʼ��ϵͳ���� */
	sys_param_t *sys_param;
	sys_param = Sys_ParamInit();

	/* ��ʼ��TCA�豸����ͨ���������л����� */
	TCA6408_Typedef* tca6408;
	tca6408 = TCA6408_GPIOInit();

	/* ͨ�������ʼ���������л���������12V����24V */
	ch_supply_t *ch_supply;
	ch_supply = Ch_SupplyInit(&tca6408->dev1, sys_param);

	/* ͨ����·��ʼ���������л�ͨ��������ѹ */
	ch_circuit_t *ch_circuit;
	ch_circuit = Ch_CircuitInit(&tca6408->dev2, sys_param);

	/* ͨ����������⹦�ܳ�ʼ�� */
	ch_sensor_t *ch_sensor;
	ch_sensor = Ch_SensorDetInit(&tca6408->dev2);

	/* ����ADC�豸����ʼ��ADC */
	ad717x_dev *ad_dev;
	ad_dev = AD717X_CreatAdcDev(sys_param);
	ad_dev->sensor = mySensor;
	ad_dev->adcInit();

	/* ������Э�����Գ�ʼ��  */
	IoT_dev_t *iot_dev;
	iot_dev = IoT_Init(sys_param, ch_supply, ch_circuit, mySensor, ad_dev);

	/* �Լ�����źŵ���˸ */
	light_ctrl->green_flash_freq = 1000;

	/* ʹ�ܴ��ڽ��չ���  */
	HAL_UART_Receive_IT(&huart1, comm_service->_rx_temp_buf, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//�鿴�Ƿ��յ�����һ֡�������ͨѶ
		comm_service->parseMasterMsg(&huart1);

		/* ADCͨ������ѭ����������������ÿ��ͨ���Ĳ�������  */
		ad_dev->dataSample();

		/* ����������Э������  */
		iot_dev->operate();

		/* check whether permant props have been modified,
		   if so, save them to the flash */
		mySensor->saveProp2Flash();

		/* ���豸�������͸ı��洢������FLASH */
		sys_param->saveParamToFlash();

		/* �豸ָʾ�ƿ���  */
		light_ctrl->lightControl();

		/* ��������⴦��  */
		ch_sensor->dectect();
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Initializes the CPU, AHB and APB busses clocks 
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks 
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
#ifdef DEBUG
#include "flash.h"
void IoT_SetLongAddr(void)
{
	uint8_t long_addr[] = {0X77, 0X01, 0X20, 0X19, 0X04, 0X19, 0X00, 0X01};

	flash_write(LONG_ADDR_FLASH_PAGE_ADDR, (uint32_t *)long_addr, sizeof(long_addr));
}
#endif

/**
 * @brief  ����ͨ�Żص�����
 * @param  UartHandle: ָ��UART_HandleTypeDef�ṹ�ľ��
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	comm_service->uartRxCpltCallBack(&huart1, &htim1);
}

/**
 * @brief  ��ʱ���ص�����
 *         2ms ��ʱ���������ж�һ֡�Ƿ������19200bits/s����£�1byte�����Ҫ0.5ms��4���ֽھ���2ms
 *         ��������3ms��û�������ݹ�������ô��Ϊ��һ֡������
 * @param  htim: ָ��TIM_HandleTypeDef�ṹ�ľ��
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//��timer1�ж�
	if (htim->Instance == htim1.Instance) {
		comm_service->setFrameFinishFlag();
		HAL_TIM_Base_Stop_IT(&htim1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_8) {
		;
	}
}

/**
 * @brief  ʹ��485���ͽӿ�
 * @param  None
 * @retval None
 */
void UART_485TxEn(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

/**
 * @brief  ʹ��485���սӿ�
 * @param  None
 * @retval None
 */
void UART_485RxEn(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
 * @brief  ��ʼ��������Э�����Գ���
 * @param  sensor: ָ��iot_object_t�ṹ�ľ��
 * @retval None
 */
void IoT_SetProp(iot_object_t *sensor) {
	//init addr, long addr, short addr are fixed. No need to set
	sensor->setPropCount(47);
	sensor->setPropLen(CHL_ENABLE_ID, 1);
	sensor->setPropLen(CHL_ERROR_ID, 1);
	sensor->setPropLen(BATTERY_LEVEL_ID, 1);

	for (int32_t i = 0; i < 8; i++) {
		sensor->setPropLen(CHL_TYPE_ID(i), 1);
		sensor->setPropLen(CHL_VOLT_TYPE_ID(i), 1);
		sensor->setPropLen(CHL_DELAY_ID(i), 1);
		sensor->setPropLen(CHL_VALUE_ID(i), 4);
		sensor->setPropLen(CHL_CALI_ID(i), 1);
	}

	sensor->setPropLen(DEV_WR_PROT_ID, 4);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
