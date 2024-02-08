/*
 * I2C_Interupt.c
 *
 *  Created on: Oct 11, 2023
 *      Author: user
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"


I2C_Handle_t I2C_Handle;

uint8_t txBuffer[32];
uint8_t cmd=0x51;

volatile char ReadByte;
volatile uint8_t rcvStop = 0;
/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void I2C2_Inits(void)
{
	I2C_Handle.I2C= I2C1;
	I2C_Handle.I2C_Config.clkSpeed = 1000000U;
	I2C_Handle.I2C_Config.deviceAddress = 0x61;
	I2C_Handle.I2C_Config.ackControl = 1;
	I2C_Handle.I2C_Config.fmDutyCycle = 0;

	I2C_Init(&I2C_Handle);
}

void I2C2_PinsInit(void)
{
	GPIO_Handle_t I2C2_Pin;
	memset(&I2C2_Pin,0,sizeof(I2C2_Pin));

	I2C2_Pin.GPIO=GPIOB;
	I2C2_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C2_Pin.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	I2C2_Pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	I2C2_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	I2C2_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	I2C2_Pin.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&I2C2_Pin);

	I2C2_Pin.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&I2C2_Pin);
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
	GPIO_PinInit();
	I2C_gpioPinInit();
	I2C1_Inits();

	I2C_PeriClockControl(I2C1, 1);

	I2C_ManageAcking(I2C, 1);

	IRQInterruptConfig(IRQNumber, 1);

	while(1){
		while(!dataAvailable);
		IRQInterruptConfig(IRQ_NO_EXTI0,DISABLE);
		I2C_PeripheralControl(I2C1,1);

		I2C_MasterSendDataIT(I2C_Handle, &cmd, 1, 0x68, 1);
		I2C_MasterRecieveDataIT(I2C_Handle, &len, 1,0x68, 1);

		cmd=0x52;
		I2C_MasterSendDataIT(I2C_Handle, &cmd,1, 0x68, 1);
		I2C_MasterRecieveDataIT(I2C_Handle, txBuffer, len, 0x68, 0);

		IRQInterruptConfig(IRQ_NO_EXTI0,0);
	}
}

void I2C1_IRQHandler(void)
{
	I2C_IRQHandling(&I2C_Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *I2C_Handle,uint8_t AppEv)
{
     if(AppEv == I2C_EV_TX_CMPLT)
     {
    	 printf("Tx is completed\n");
     }else if (AppEv == I2C_EV_RX_CMPLT)
     {
    	 printf("Rx is completed\n");
    	 rxComplt = SET;
     }
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
	dataAvailable = 1;
}
