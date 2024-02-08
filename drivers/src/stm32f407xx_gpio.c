/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Sep 22, 2023
 *      Author: user
 */

#ifndef SRC_STM32F407XX_GPIO_C_
#define SRC_STM32F407XX_GPIO_C_


#include "stm32f407xx_gpio.h"

void GPIO_PeriClockControl(GPIO_t *GPIO, uint8_t x){
	if(x==ENABLE){
		    if(GPIO == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (GPIO == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (GPIO == GPIOC)
			{
					GPIOC_PCLK_EN();
			}else if (GPIO == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if (GPIO == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if (GPIO == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if (GPIO == GPIOG)
			{
				GPIOG_PCLK_EN();
			}else if (GPIO == GPIOH)
			{
				GPIOH_PCLK_EN();
			}else if (GPIO == GPIOI)				{
				GPIOI_PCLK_EN();
			}
	}
}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *gpioHandle){
	uint32_t temp=0;
	int temp1,temp2;
	GPIO_PeriClockControl(gpioHandle->GPIO, ENABLE);
	if(gpioHandle->GPIO_PinConfig.GPIO_PinMode<=3){
		temp=gpioHandle->GPIO_PinConfig.GPIO_PinMode<<(2*gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		gpioHandle->GPIO->MODER &=~(0x3<<(2*gpioHandle->GPIO_PinConfig.GPIO_PinNumber));
		gpioHandle->GPIO->MODER |=temp;
	}
	else{
		if(gpioHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
		{
			EXTI->FTSR |= ( 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~( 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(gpioHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT){
			EXTI->RTSR |= ( 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~( 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(gpioHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RFT){
			EXTI->RTSR |= ( 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		temp1 = gpioHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		temp2 = gpioHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
	    int port = GPIO_BASEADDR_TO_CODE(gpioHandle->GPIO);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = port << (temp2*4);

		EXTI->IMR |= 1 << gpioHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp=gpioHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	gpioHandle->GPIO->OSPEEDR &=~(0x3<<(2*gpioHandle->GPIO_PinConfig.GPIO_PinNumber));
	gpioHandle->GPIO->OSPEEDR |=temp;

	temp=gpioHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	gpioHandle->GPIO->PUPDR &=~(0x3<<(2*gpioHandle->GPIO_PinConfig.GPIO_PinNumber));
	gpioHandle->GPIO->PUPDR |=temp;

	temp=gpioHandle->GPIO_PinConfig.GPIO_PinOPType <<(gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	gpioHandle->GPIO->OTYPER &=~(0x1<<gpioHandle->GPIO_PinConfig.GPIO_PinNumber);
	gpioHandle->GPIO->OTYPER |=temp;

	if(gpioHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{

		temp1 = gpioHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = gpioHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		gpioHandle->GPIO->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );
		gpioHandle->GPIO->AFR[temp1] |= (gpioHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}
}
void GPIO_DeInit(GPIO_t *GPIO){
	return;
}

uint8_t GPIO_ReadFromInputPin(GPIO_t *GPIO, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((GPIO->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_t *GPIO)
{
	uint16_t value;

	value = (uint16_t)GPIO->IDR;

	return value;
}


void GPIO_WriteToOutputPin(GPIO_t *GPIO, uint8_t PinNumber, uint8_t val){
	if(val==1){
		GPIO->ODR|=(1<<PinNumber);
	}
	else GPIO->ODR&=~(1<<PinNumber);
}

void GPIO_WriteToOutputPort(GPIO_t *GPIO, uint16_t Value){
	GPIO->ODR=Value;
}

void GPIO_ToggleOutputPin(GPIO_t *GPIO, uint8_t pinNo){
	GPIO->ODR^=(1<<pinNo);
}

void IRQInterruptConfig(uint8_t IRQNumber, uint8_t x){
	if(x == 1){
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64 ){
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else{
		if(IRQNumber <= 31){
			*( NVIC_ICER0 ) |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 ){
			*( NVIC_ICER1 ) |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 ){
			*( NVIC_ICER2 ) |= ( 1 << (IRQNumber % 64) );
		}
	}
}

void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + 4 ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & ( 1 << PinNumber)){
		EXTI->PR |= ( 1 << PinNumber);
	}
}

#endif
