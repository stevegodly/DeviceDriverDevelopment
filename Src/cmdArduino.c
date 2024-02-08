/*
 * cmdArduino.c
 *
 *  Created on: Oct 2, 2023
 *      Author: user
 */


#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

#define COMMAND_LED_CTRL      		0x50

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void buttonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	GPIOBtn.GPIO = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIOBtn);
}

void SPI2Init(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.SPI = SPI2;
	SPI2handle.SPI_Config.SPI_Bus = 0;
	SPI2handle.SPI_Config.SPI_Mode = 1;
	SPI2handle.SPI_Config.SPI_ClkSpeed = 4;
	SPI2handle.SPI_Config.SPI_DFF = 0;
	SPI2handle.SPI_Config.SPI_CPOL = 0;
	SPI2handle.SPI_Config.SPI_CPHA = 0;
	SPI2handle.SPI_Config.SPI_SSM = 0;

	SPI_Init(&SPI2handle);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.GPIO = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 10;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&SPIPins);
}

int main(void)
{
	uint8_t command=COMMAND_LED_CTRL,ackbyte, args[2],dummyWrite=0xff,dummyRead;
	uint32_t len=1;

 	buttonInit();

	SPI2_GPIOInits();

	SPI2Init();

	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		while( ! GPIO_ReadFromInputPin(GPIOA,0) );
		delay();
		SPI_PeripheralControl(SPI2,1);

		SPI_SendData(SPI2,&command,&len);
		len=1;

		SPI_RecieveData(SPI2,&dummyRead,&len);
		len=1;

		SPI_SendData(SPI2,&dummyWrite,&len);
		len=1;

		SPI_ReceiveData(SPI2,&ackbyte,&len);
		len=2;


		if( SPI_VerifyResponse(ackbyte)){
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2,args,&len);
			len=2;
			SPI_ReceiveData(SPI2,args,&len);
			printf("COMMAND_LED_CTRL Executed\n");
		}
	}
}
