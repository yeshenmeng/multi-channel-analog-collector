/*
 * modbus_slave.h
 * 实现基于485总线的类modbus私有通讯协议
 *
 *  Created on: 2019年2月1日
 *      Author: xuhui
 */

#ifndef RS485_SENSOR_NETWORK_MODBUS_COMM_SERVICES_H_
#define RS485_SENSOR_NETWORK_MODBUS_COMM_SERVICES_H_

#include "main.h"
#include "iotobject.h"

#define MAX_MODBUS_COMM_BUF_SIZE	512

typedef struct modbus_comm_services {
	//解析主机发过来的命令，返回1如果解析成功，否则返回0
	uint8_t (*parseMasterMsg)(UART_HandleTypeDef *uartHandler);

	//处理接收到的数据，清理接收帧标志，并计数
	void (*uartRxCpltCallBack)(UART_HandleTypeDef *uartHandler,
			TIM_HandleTypeDef * timerHandler);

	//将Frame Finish Flag至1，表示一帧结束，应当在外部定时器中断中调用
	void (*setFrameFinishFlag)();

	//设置待处理的sensor handler
	void (*setSensorHandler)(iot_object_t *obj);

	void (*uart_485TxEn)(void);
	void (*uart_485RxEn)(void);

	uint8_t _rx_buf[MAX_MODBUS_COMM_BUF_SIZE];			//receive buffer
	uint8_t _tx_buf[MAX_MODBUS_COMM_BUF_SIZE];			//transmit buffer
	uint8_t _frame_finish_flag;		//标志一帧接收结束
	uint8_t _rx_count;			//当前接收帧的字节个数
	uint8_t _tx_count;			//当前发送帧的字节个数
	uint8_t _rx_temp_buf[1];	//单个字节的临时接收buf

	iot_object_t * _sensor;		//待处理的sensor实体

} modbus_comm_services_t;

modbus_comm_services_t * createModbusCommServiceHandler(void (*uart_485TxEn)(),
		void (*uart_485RxEn)());

#endif /* RS485_SENSOR_NETWORK_MODBUS_COMM_SERVICES_H_ */
