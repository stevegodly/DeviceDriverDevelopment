/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Oct 6, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"

#define I2C_READY	0
#define I2C_BUSY_IN_TX	1
#define I2C_BUSY_IN_RX	2

#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2

typedef struct{
	uint32_t clkSpeed;
	uint8_t  deviceAddress;
	uint8_t  ackControl;
	uint8_t fmDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_t *I2C;
	I2C_Config_t I2C_Config;
	uint8_t *TxBuffer;
	uint8_t *RxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t devAddr;
	uint8_t Sr;
}I2C_Handle_t;

void I2C_ManageAcking(I2C_t *I2C, uint8_t x);
void I2C_PeripheralControl(I2C_t *I2C, uint8_t x);
void I2C_PeriClockControl(I2C_t *I2C, uint8_t x);
void sendData(I2C_t *I2C,uint8_t *txBuffer,uint32_t flag,uint32_t len);
uint8_t I2C_FlagStatus(I2C_t *I2C , uint32_t flagName);


void I2C_Init(I2C_Handle_t *I2C_Handle);
void I2C_DeInit(I2C_t *I2C);

void I2C_MasterSendData(I2C_Handle_t *I2C_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr,uint8_t x);
void I2C_MasterRecieveData(I2C_Handle_t *I2C_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr,uint8_t x);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *I2C_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr,uint8_t x);
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *I2C_Handle,uint8_t *rxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t x);

void I2C_ApplicationEventCallback(I2C_Handle_t *I2C_Handle,uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_H_ */
