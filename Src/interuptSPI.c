/*
 * interuptSPI.c
 *
 *  Created on: Oct 3, 2023
 *      Author: user
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"



SPI_Handle_t SPI_Handle;

#define MAX_LEN 500
char RcvBuff[MAX_LEN];

volatile char ReadByte;
volatile uint8_t rcvStop = 0;
/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void SPI2_Inits(void)
{
	SPI_Handle.SPI= SPI2;
	SPI_Handle.SPI_Config.SPI_Bus = 0;
	SPI_Handle.SPI_Config.SPI_Mode = 1;
	SPI_Handle.SPI_Config.SPI_ClkSpeed = 4;
	SPI_Handle.SPI_Config.SPI_DFF = 0;
	SPI_Handle.SPI_Config.SPI_CPOL = 0;
	SPI_Handle.SPI_Config.SPI_CPHA = 0;
	SPI_Handle.SPI_Config.SPI_SSM = 0;

	SPI_Init(&SPI_Handle);
}

void SPI2_PinsInit(void)
{
	GPIO_Handle_t SPI2_Pin;
	memset(&SPI2_Pin,0,sizeof(SPI2_Pin));

	SPI2_Pin.GPIO=GPIOB;
	SPI2_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_Pin.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2_Pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPI2_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	SPI2_Pin.GPIO_PinConfig.GPIO_PinNumber = 10;
	GPIO_Init(&SPI2_Pin);

	//MOSI
	SPI2_Pin.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPI2_Pin);

	//MISO
	SPI2_Pin.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPI2_Pin);

	//NSS
	SPI2_Pin.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&SPI2_Pin);
}

void Slave_GPIOInterupt(void)
{
	GPIO_Handle_t GPIOBtn;
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	GPIOBtn.GPIO = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	IRQPriorityConfig(IRQ_NO_EXTI0,15);
	IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

}

int main(void)
{
	uint8_t dummy = 0xff;
	Slave_GPIOInterupt();
	SPI2_PinsInit();
	SPI2_Inits();

	SPI_SSOEConfig(SPI2,ENABLE);
	IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	while(1){
		rcvStop = 0;

		while(!dataAvailable);

		IRQInterruptConfig(IRQ_NO_EXTI0,DISABLE);

		SPI_PeripheralControl(SPI2,ENABLE);


		while(!rcvStop)
		{
			while(SPI_SendDataInt(&SPI_Handle,&dummy,1) == SPI_BUSY_IN_TX);
			while(SPI_ReceiveDataInt(&SPI_Handle,(uint8_t*)(&ReadByte),1) == SPI_BUSY_IN_RX );
		}


		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);
	}
	return 1;

}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI_Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || ( i == MAX_LEN)){
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}

}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
	dataAvailable = 1;
}
