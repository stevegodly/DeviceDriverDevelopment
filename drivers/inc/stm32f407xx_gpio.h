/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Sep 22, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"


#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_t *GPIO;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


void GPIO_PeriClockControl(GPIO_t *pGPIO, uint8_t x);
void GPIO_Init(GPIO_Handle_t *gpioHandle);
void GPIO_DeInit(GPIO_t *pGPIO);


uint8_t GPIO_ReadFromInputPin(GPIO_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_t *pGPIOx, uint8_t PinNumber);



void IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif
