#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"


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

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&SPIPins);
}



int main(void)
{
	uint16_t dataLen;
	char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";


	buttonInit();


	SPI2_GPIOInits();


	SPI2Init();


	SPI_SSOEConfig(SPI2,ENABLE);

	while(1){

		while( ! GPIO_ReadFromInputPin(GPIOA,0) );

		delay();

		SPI_PeripheralControl(SPI2,1);

		dataLen = strlen(user_data);
		SPI_SendLength(SPI2,dataLen);

		SPI_SendData(SPI2,(uint8_t*)user_data,(uint32_t)dataLen);


		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2,0);
	}

	return 1;

}
