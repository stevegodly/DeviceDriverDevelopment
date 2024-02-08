/*
 * i2cTesting.c
 *
 *  Created on: Oct 7, 2023
 *      Author: user
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"

char txBuffer[]="hey";

I2C_Handle_t I2C_Handle;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void GPIO_PinInit(void){
	GPIO_Handle_t gpioPin;
	gpioPin.GPIO = GPIOA;
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = 0;
	gpioPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpioPin);
}

void I2C_gpioPinInit(void){
	GPIO_Handle_t i2cPin;
	i2cPin.GPIO = GPIOB;
	i2cPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2cPin.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2cPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2cPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	i2cPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	i2cPin.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&i2cPin);

	i2cPin.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&i2cPin);
}

void I2C1_Inits(void){
	I2C_Handle.I2C=I2C1;
	I2C_Handle.I2C_Config.ackControl=1;
	I2C_Handle.I2C_Config.clkSpeed=1000000U;
	I2C_Handle.I2C_Config.deviceAddress=0x61;
	I2C_Handle.I2C_Config.fmDutyCycle=0;

	I2C_Init(&I2C_Handle);
}

int main(void)
{
	uint8_t x=0;
	GPIO_PinInit();
	I2C_gpioPinInit();
	I2C1_Inits();

	I2C_PeripheralControl(I2C1,1);



	while(1){
		if(GPIO_ReadFromInputPin(GPIOA,0)){
			delay();
			I2C_MasterSendData(&I2C_Handle,(uint8_t*)txBuffer, strlen(txBuffer), 0x68, x);
		}
	}
}
