/*
 * stm32f407xx_spi.h
 *
 *  Created on: Sep 26, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include <stdint.h>
#include "stm32f407xx.h"

#define SPI_TXE_FLAG    ( 1 << 1)
#define SPI_RXNE_FLAG   ( 1 << 0)
#define SPI_BUSY_FLAG   ( 1 << 7)

#define SPI_READY 		0
#define SPI_BUSY_IN_RX 	1
#define SPI_BUSY_IN_TX 	2

#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4

typedef struct{
	uint8_t  SPI_Mode;
	uint8_t  SPI_Bus;
	uint8_t  SPI_ClkSpeed;
	uint8_t	 SPI_DFF;
	uint8_t  SPI_CPOL;
	uint8_t  SPI_CPHA;
	uint8_t  SPI_SSM;
	uint8_t  *TxAddr;
	uint8_t  *RxAddr;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t  TxState;
	uint8_t  RxState;
}SPI_Config_t;

typedef struct{
	SPI_t *SPI;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;


void SPI_PeriClockControl(SPI_t *SPI, uint8_t x);


void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_DeInit(SPI_t *SPI);


void SPI_SendData(SPI_t *SPI,uint8_t *txBuffer, uint32_t len);
void SPI_ReceiveData(SPI_t *SPI, uint8_t *rxBuffer, uint32_t len);

uint8_t SPI_SendDataInt(SPI_Handle_t *SPI_Handle,uint8_t *txBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataInt(SPI_Handle_t *SPI_Handle, uint8_t *RxBuffer, uint32_t Len);
void SPI_SendLength(SPI_t *SPI,uint16_t len);

void SPI_IRQHandling(SPI_Handle_t *SPI_Handle);


void SPI_PeripheralControl(SPI_t *SPI, uint8_t x);
void SPI_SSIConfig(SPI_t *SPI, uint8_t x);
void SPI_SSOEConfig(SPI_t *SPI, uint8_t x);
uint8_t SPI_GetFlagStatus(SPI_t *SPI , uint32_t FlagName);

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif
