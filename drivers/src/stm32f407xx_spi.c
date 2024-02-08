/*
 * stm32f407xx_spi.c
 *
 *  Created on: Sep 26, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_SPI_C_
#define INC_STM32F407XX_SPI_C_

#include <stddef.h>
#include "stm32f407xx_spi.h"


void SPI_PeriClockControl(SPI_t *SPI, uint8_t x){
	if(x==1){
		if(SPI == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (SPI == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (SPI == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
}

void SPI_Init(SPI_Handle_t *SPIHandle){
	uint16_t temp=0;

	SPI_PeriClockControl(SPIHandle->SPI,1);

	temp |= SPIHandle->SPI_Config.SPI_Mode << 2;

	if(SPIHandle->SPI_Config.SPI_Bus == 0)
	{
		temp &= ~( 1 << 15);
	}else if (SPIHandle->SPI_Config.SPI_Bus == 1)
	{
		temp |= ( 1 << 15);
	}else if (SPIHandle->SPI_Config.SPI_Bus == 2)
	{
		temp &= ~( 1 << 15);

		temp |= ( 1 << 10);
	}

	temp |= SPIHandle->SPI_Config.SPI_ClkSpeed << 3;

	temp |= SPIHandle->SPI_Config.SPI_DFF << 11;

	temp |= SPIHandle->SPI_Config.SPI_CPOL << 1;

	temp |= SPIHandle->SPI_Config.SPI_CPHA << 0;

	temp |= SPIHandle->SPI_Config.SPI_SSM << 9;

	SPIHandle->SPI->CR1 = temp;

}

void SPI_SendLength(SPI_t *SPI,uint16_t len){
	while((SPI->SR & (1<<1))==0 );
	SPI->DR=len;
}

void SPI_SendData(SPI_t *SPI,uint8_t *txBuffer, uint32_t len){
	while(len!=0){
		while((SPI->SR & (1<<1))==0 );
		if(SPI->CR1 & (1<<11)){
			SPI->DR=*((uint16_t*)txBuffer);
			len-=2;
			(uint16_t*)txBuffer++;
		}
		else{
			SPI->DR=*txBuffer;
			len-=1;
			txBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_t *SPI, uint8_t *rxBuffer, uint32_t len)
{
	while(len!=0){
		while((SPI->SR & (1))==0 );
		if(SPI->CR1 & (1<<11)){
			*((uint16_t*)rxBuffer)=SPI->DR;
			len-=2;
			(uint16_t*)rxBuffer++;
		}
		else{
			*rxBuffer=SPI->DR;
			len-=1;
			rxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_t *SPI, uint8_t x)
{
	if(x == 1){
		SPI->CR1 |=  (1 << 6);
	}else{
		SPI->CR1 &=  ~(1 << 6);
	}
}

void SPI_SSIConfig(SPI_t *SPI, uint8_t x)
{
	if(x == 1)
	{
		SPI->CR1 |=  (1 << 8);
	}else
	{
		SPI->CR1 &=  ~(1 << 8);
	}
}

void SPI_SSOEConfig(SPI_t *SPI, uint8_t x)
{
	if(x == 1)
	{
		SPI->CR2 |=  (1 << 2);
	}else
	{
		SPI->CR2 &=  ~(1 << 2);
	}
}

uint8_t SPI_GetFlagStatus(SPI_t *SPI , uint32_t FlagName)
{
	if(SPI->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



uint8_t SPI_SendDataInt(SPI_Handle_t *SPI_Handle,uint8_t *txBuffer, uint32_t len){
	uint8_t state=SPI_Handle->SPI_Config.TxState;

	if(SPI_Handle->SPI_Config.TxState!=SPI_BUSY_IN_TX){
		SPI_Handle->SPI_Config.TxAddr=txBuffer;
		SPI_Handle->SPI_Config.TxLen=len;
		SPI_Handle->SPI_Config.TxState=	SPI_BUSY_IN_TX;
		SPI_Handle->SPI->CR2|=(1<<7);
	}
	return state;
}

uint8_t SPI_ReceiveDataInt(SPI_Handle_t *SPI_Handle, uint8_t *RxBuffer, uint32_t len)
{
	uint8_t state = SPI_Handle->SPI_Config.RxState;

	if(state != SPI_BUSY_IN_RX){
		SPI_Handle->SPI_Config.RxAddr = RxBuffer;
		SPI_Handle->SPI_Config.RxLen = len;
		SPI_Handle->SPI_Config.RxState = SPI_BUSY_IN_RX;
		SPI_Handle->SPI->CR2 |= ( 1 << 6 );
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *SPI_Handle){
	if((SPI_Handle->SPI->CR2 & 1<<7) && (SPI_Handle->SPI->SR & 1<<1)){
		SPI_SendData(SPI_Handle->SPI,SPI_Handle->SPI_Config.TxAddr,(SPI_Handle->SPI_Config.TxLen));
	}
	else if((SPI_Handle->SPI->CR2 & 1<<6) && (SPI_Handle->SPI->SR & 1<<0)){
		SPI_ReceiveData(SPI_Handle->SPI,SPI_Handle->SPI_Config.RxAddr,(SPI_Handle->SPI_Config.RxLen));
	}
	if(!SPI_Handle->SPI_Config.TxLen)
	{
		SPI_CloseTransmisson(SPI_Handle);
		SPI_ApplicationEventCallback(SPI_Handle,SPI_EVENT_TX_CMPLT);
	}
	if(! SPI_Handle->SPI_Config.RxLen)
	{
		SPI_CloseReception(SPI_Handle);
		SPI_ApplicationEventCallback(SPI_Handle,SPI_EVENT_RX_CMPLT);
	}
}



void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->SPI->CR2 &= ~( 1 << 7);
	pSPIHandle->SPI_Config.TxAddr = NULL;
	pSPIHandle->SPI_Config.TxLen = 0;
	pSPIHandle->SPI_Config.TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->SPI->CR2 &= ~( 1 << 6);
	pSPIHandle->SPI_Config.RxAddr = NULL;
	pSPIHandle->SPI_Config.RxLen = 0;
	pSPIHandle->SPI_Config.RxState = SPI_READY;

}

#endif
