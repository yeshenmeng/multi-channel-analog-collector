/*
 * modbus_slave.h
 * ʵ�ֻ���485���ߵ���modbus˽��ͨѶЭ��
 *
 *  Created on: 2019��2��1��
 *      Author: xuhui
 */

#ifndef RS485_SENSOR_NETWORK_MODBUS_COMM_SERVICES_H_
#define RS485_SENSOR_NETWORK_MODBUS_COMM_SERVICES_H_

#include "main.h"
#include "iotobject.h"

#define MAX_MODBUS_COMM_BUF_SIZE	512

typedef struct modbus_comm_services {
	//�����������������������1��������ɹ������򷵻�0
	uint8_t (*parseMasterMsg)(UART_HandleTypeDef *uartHandler);

	//������յ������ݣ��������֡��־��������
	void (*uartRxCpltCallBack)(UART_HandleTypeDef *uartHandler,
			TIM_HandleTypeDef * timerHandler);

	//��Frame Finish Flag��1����ʾһ֡������Ӧ�����ⲿ��ʱ���ж��е���
	void (*setFrameFinishFlag)();

	//���ô������sensor handler
	void (*setSensorHandler)(iot_object_t *obj);

	void (*uart_485TxEn)(void);
	void (*uart_485RxEn)(void);

	uint8_t _rx_buf[MAX_MODBUS_COMM_BUF_SIZE];			//receive buffer
	uint8_t _tx_buf[MAX_MODBUS_COMM_BUF_SIZE];			//transmit buffer
	uint8_t _frame_finish_flag;		//��־һ֡���ս���
	uint8_t _rx_count;			//��ǰ����֡���ֽڸ���
	uint8_t _tx_count;			//��ǰ����֡���ֽڸ���
	uint8_t _rx_temp_buf[1];	//�����ֽڵ���ʱ����buf

	iot_object_t * _sensor;		//�������sensorʵ��

} modbus_comm_services_t;

modbus_comm_services_t * createModbusCommServiceHandler(void (*uart_485TxEn)(),
		void (*uart_485RxEn)());

#endif /* RS485_SENSOR_NETWORK_MODBUS_COMM_SERVICES_H_ */
